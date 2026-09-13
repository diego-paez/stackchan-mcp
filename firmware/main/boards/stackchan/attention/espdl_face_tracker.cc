#include "espdl_face_tracker.h"

#ifdef ESP_PLATFORM

#include <esp_log.h>
#include <esp_timer.h>

#include <list>
#include <vector>

#include "linux/videodev2.h"

#include "human_face_detect.hpp"

#define TAG "face_tracker"

namespace stackchan {
namespace attention {

namespace {

// V4L2 fourcc -> the pixel type esp-dl understands.
//
// esp_video negotiates its format with the sensor at runtime and prefers
// YUV422P, then RGB565, then RGB24 — so what arrives is a property of the
// camera module, not a constant of this firmware. esp-dl's img_t speaks
// RGB888, RGB565 and GRAY. Anything else is refused rather than reinterpreted:
// handing YUYV bytes to a model expecting RGB565 does not fail, it produces
// confident detections of nothing, which is far worse than no detections.
bool MapPixelFormat(uint32_t fourcc, dl::image::pix_type_t* out) {
    switch (fourcc) {
        case V4L2_PIX_FMT_RGB565:
            *out = dl::image::DL_IMAGE_PIX_TYPE_RGB565;
            return true;
        case V4L2_PIX_FMT_RGB24:
            *out = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
            return true;
        case V4L2_PIX_FMT_GREY:
            *out = dl::image::DL_IMAGE_PIX_TYPE_GRAY;
            return true;
        default:
            return false;
    }
}

const char* FourccName(uint32_t f) {
    switch (f) {
        case V4L2_PIX_FMT_RGB565:  return "RGB565";
        case V4L2_PIX_FMT_RGB24:   return "RGB24";
        case V4L2_PIX_FMT_GREY:    return "GREY";
        case V4L2_PIX_FMT_YUV422P: return "YUV422P";
        case V4L2_PIX_FMT_YUV420:  return "YUV420";
        case V4L2_PIX_FMT_JPEG:    return "JPEG";
        default:                   return "unknown";
    }
}

}  // namespace

EspDlFaceTracker::EspDlFaceTracker(Camera* camera, const FaceTrackerConfig& cfg)
    : camera_(camera), cfg_(cfg) {
    lock_ = xSemaphoreCreateMutex();
}

EspDlFaceTracker::~EspDlFaceTracker() {
    running_ = false;
    if (task_) {
        // Give the loop one period to notice and return on its own; deleting a
        // task mid-inference would leak the model's working buffers.
        vTaskDelay(pdMS_TO_TICKS(cfg_.detect_period_ms + 100));
        task_ = nullptr;
    }
    if (lock_) vSemaphoreDelete(lock_);
}

bool EspDlFaceTracker::begin() {
    if (camera_ == nullptr) {
        ESP_LOGE(TAG, "no camera; face tracking disabled");
        return false;
    }
    // Fail before loading several megabytes of model if the camera will not
    // share pixels — a board that does not override PeekFrame can never feed
    // this, and finding that out now produces a clear message instead of a
    // tracker that runs forever and sees nobody.
    CameraFrame probe;
    if (!camera_->PeekFrame(&probe)) {
        ESP_LOGW(TAG, "camera lends no frames yet; will retry in the task");
    }

    detector_.reset(new HumanFaceDetect());
    if (!detector_) {
        ESP_LOGE(TAG, "could not create HumanFaceDetect");
        return false;
    }

    running_ = true;
    if (xTaskCreatePinnedToCore(&EspDlFaceTracker::TaskEntry, "face_detect",
                                cfg_.stack_words, this, cfg_.priority,
                                &task_, cfg_.core) != pdPASS) {
        ESP_LOGE(TAG, "could not start the detection task");
        running_ = false;
        detector_.reset();
        return false;
    }
    ESP_LOGI(TAG, "face tracking started: every %ums, min score %.2f",
             (unsigned)cfg_.detect_period_ms, cfg_.min_score);
    return true;
}

void EspDlFaceTracker::TaskEntry(void* self) {
    static_cast<EspDlFaceTracker*>(self)->Run();
    vTaskDelete(nullptr);
}

void EspDlFaceTracker::Run() {
    while (running_) {
        const uint32_t started = esp_timer_get_time() / 1000;
        DetectOnce(started);
        const uint32_t spent = (esp_timer_get_time() / 1000) - started;

        // Pace by period, not by delay: if inference took longer than the
        // period, yield briefly rather than never yielding at all. A task at
        // priority 3 that never blocks will starve everything below it.
        const uint32_t wait = (spent >= cfg_.detect_period_ms)
                                  ? 10
                                  : (cfg_.detect_period_ms - spent);
        vTaskDelay(pdMS_TO_TICKS(wait));
    }
}

bool EspDlFaceTracker::DetectOnce(uint32_t now_ms) {
    CameraFrame frame;
    if (!camera_->PeekFrame(&frame) || frame.data == nullptr) {
        if (xSemaphoreTake(lock_, portMAX_DELAY) == pdTRUE) {
            ++stats_.no_frame;
            target_.visible = false;
            xSemaphoreGive(lock_);
        }
        return false;
    }

    dl::image::pix_type_t pix;
    if (!MapPixelFormat(frame.fourcc, &pix)) {
        if (!warned_format_) {
            warned_format_ = true;
            ESP_LOGE(TAG,
                     "camera gives %s (0x%08x); esp-dl reads only RGB565, "
                     "RGB24 and GREY, so face tracking is off. Either force "
                     "RGB565 when the sensor format is negotiated in "
                     "EspVideo, or convert the frame with "
                     "esp_imgfx_color_convert, which this firmware already "
                     "links for the rotate path. Reinterpreting these bytes "
                     "is not an option: it yields confident detections of "
                     "faces that are not there.",
                     FourccName(frame.fourcc), (unsigned)frame.fourcc);
        }
        if (xSemaphoreTake(lock_, portMAX_DELAY) == pdTRUE) {
            ++stats_.bad_format;
            target_.visible = false;
            xSemaphoreGive(lock_);
        }
        return false;
    }

    dl::image::img_t img;
    img.data = const_cast<uint8_t*>(frame.data);
    img.width = frame.width;
    img.height = frame.height;
    img.pix_type = pix;

    const int64_t t0 = esp_timer_get_time();
    std::list<dl::detect::result_t>& results = detector_->run(img);
    const uint32_t latency = (esp_timer_get_time() - t0) / 1000;

    // Translate into this project's own plain type before choosing, so the
    // selection rule stays in face_geometry.h where it is tested on a host.
    std::vector<DetectedBox> boxes;
    boxes.reserve(results.size());
    for (const auto& r : results) {
        if (r.score < cfg_.min_score) continue;
        if (r.box.size() < 4) continue;
        boxes.push_back(DetectedBox{r.box[0], r.box[1], r.box[2], r.box[3], r.score});
    }

    const DetectedBox* primary =
        PickPrimaryFace(boxes.begin(), boxes.end(), frame.width, frame.height);

    FaceTarget next;
    if (primary != nullptr) {
        next = BoxToTarget(*primary, frame.width, frame.height, now_ms);
    }

    if (xSemaphoreTake(lock_, portMAX_DELAY) == pdTRUE) {
        ++stats_.frames;
        if (next.visible) ++stats_.detections;
        stats_.last_latency_ms = latency;
        target_ = next;
        xSemaphoreGive(lock_);
    }
    return next.visible;
}

FaceTarget EspDlFaceTracker::getTarget() const {
    FaceTarget copy;
    if (xSemaphoreTake(lock_, portMAX_DELAY) == pdTRUE) {
        copy = target_;
        xSemaphoreGive(lock_);
    }
    return copy;
}

EspDlFaceTracker::Stats EspDlFaceTracker::stats() const {
    Stats copy;
    if (xSemaphoreTake(lock_, portMAX_DELAY) == pdTRUE) {
        copy = stats_;
        xSemaphoreGive(lock_);
    }
    return copy;
}

}  // namespace attention
}  // namespace stackchan

#endif  // ESP_PLATFORM
