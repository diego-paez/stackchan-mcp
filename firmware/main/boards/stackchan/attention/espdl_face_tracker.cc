#include "espdl_face_tracker.h"

#if defined(ESP_PLATFORM) && defined(CONFIG_STACKCHAN_FACE_DETECT)

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <cstring>

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
            // esp-dl 3.3 split this by endianness. V4L2_PIX_FMT_RGB565 is the
            // little-endian one; V4L2_PIX_FMT_RGB565X is big-endian, and
            // esp_video.cc already converts that to RGB565 before it reaches
            // here. Picking BE would not fail — it would swap red and blue on
            // every frame and quietly cost detection accuracy.
            *out = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;
            return true;
        case V4L2_PIX_FMT_RGB24:
            *out = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
            return true;
        case V4L2_PIX_FMT_GREY:
            *out = dl::image::DL_IMAGE_PIX_TYPE_GRAY;
            return true;
        case V4L2_PIX_FMT_YUYV:
            // What this board actually produces, measured rather than
            // assumed: the GC0308 on the DVP bus reports 0x56595559, which is
            // v4l2_fourcc('Y','U','Y','V') — packed 4:2:2, two pixels per four
            // bytes. esp-dl 3.3 has a real YUYV pixel type with real
            // conversions behind it (DL_IMAGE_PIX_CVT_YUYV2RGB888), so the
            // preprocessor converts properly instead of the model reading
            // luma as red.
            //
            // This is the shipping configuration — config.json pins the
            // sensor with CONFIG_CAMERA_GC0308_DVP_YUV422_320X240_20FPS — so
            // without this case face detection sees nobody, forever, and
            // looks exactly like an empty room. An earlier version of this
            // function handled V4L2_PIX_FMT_YUV422P instead, on the
            // assumption that the driver would label its YUYV output with the
            // 422P fourcc. It does not, and it was the wrong case to write
            // anyway: 422P is *planar*, three separate planes, and pointing a
            // packed-YUYV reader at it is precisely the reinterpretation the
            // note above refuses to make. It is therefore absent below, and
            // refused, rather than quietly accepted.
            *out = dl::image::DL_IMAGE_PIX_TYPE_YUYV;
            return true;
        default:
            return false;
    }
}

// Packed YUYV 4:2:2 -> RGB888, BT.601 full range.
//
// Four bytes carry two pixels as Y0 U Y1 V: two luma samples sharing one
// chroma pair. Integer coefficients scaled by 256, which is exact enough for
// a detector and avoids a float multiply per channel per pixel on a core that
// also has to run the model.
void Yuyv422ToRgb888(const uint8_t* src, size_t src_len, uint8_t* dst) {
    const size_t pairs = src_len / 4;
    for (size_t i = 0; i < pairs; ++i) {
        const int y0 = src[0], u = src[1] - 128, y1 = src[2], v = src[3] - 128;
        const int r_off = (359 * v) >> 8;
        const int g_off = (88 * u + 183 * v) >> 8;
        const int b_off = (454 * u) >> 8;
        for (int k = 0; k < 2; ++k) {
            const int y = (k == 0) ? y0 : y1;
            int r = y + r_off, g = y - g_off, b = y + b_off;
            dst[0] = (uint8_t)(r < 0 ? 0 : (r > 255 ? 255 : r));
            dst[1] = (uint8_t)(g < 0 ? 0 : (g > 255 ? 255 : g));
            dst[2] = (uint8_t)(b < 0 ? 0 : (b > 255 ? 255 : b));
            dst += 3;
        }
        src += 4;
    }
}

const char* FourccName(uint32_t f) {
    switch (f) {
        case V4L2_PIX_FMT_RGB565:  return "RGB565";
        case V4L2_PIX_FMT_RGB24:   return "RGB24";
        case V4L2_PIX_FMT_GREY:    return "GREY";
        case V4L2_PIX_FMT_YUYV:    return "YUYV";
        case V4L2_PIX_FMT_YUV422P: return "YUV422P (planar; not readable here)";
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
    if (scratch_) { heap_caps_free(scratch_); scratch_ = nullptr; scratch_size_ = 0; }
    if (lock_) vSemaphoreDelete(lock_);
}

bool EspDlFaceTracker::begin() {
    if (camera_ == nullptr) {
        ESP_LOGE(TAG, "no camera; face tracking disabled");
        return false;
    }
    // NO CAMERA CALL HERE, deliberately.
    //
    // This used to probe with BeginFrame() so a board that cannot lend pixels
    // said so immediately. It cost a bring-up: begin() runs from the board
    // constructor, BeginFrame() captures, and capture on this sensor waits on
    // a V4L2 buffer — of which the DVP path requests exactly one. Boot
    // stopped there, before Wi-Fi, with a device that was alive (timers still
    // fired) and unreachable.
    //
    // The rule it taught: nothing on the boot thread may wait on a frame. The
    // first real capture happens on the detection task below, where blocking
    // costs a detection instead of the whole robot, and where the "no frame"
    // counter reports it.

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
    ESP_LOGI(TAG, "face tracking ready (idle): every %ums when active, "
                  "min score %.2f",
             (unsigned)cfg_.detect_period_ms, cfg_.min_score);
    return true;
}

void EspDlFaceTracker::TaskEntry(void* self) {
    static_cast<EspDlFaceTracker*>(self)->Run();
    vTaskDelete(nullptr);
}

void EspDlFaceTracker::Run() {
    while (running_) {
        if (!active_) {
            // Idle. Nothing captured, nothing converted, nothing inferred —
            // the microphone gets the memory bandwidth back. See setActive().
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
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

namespace {
// Holds the camera's frame still for as long as the detector is reading it,
// and gives it back on every path out — including the two early returns
// below, which is what an explicit EndFrame() at the end of the function
// would have missed.
class BorrowedFrame {
public:
    explicit BorrowedFrame(Camera* camera) : camera_(camera) {
        held_ = camera_ != nullptr && camera_->BeginFrame(&frame_);
    }
    ~BorrowedFrame() { if (held_) camera_->EndFrame(); }
    BorrowedFrame(const BorrowedFrame&) = delete;
    BorrowedFrame& operator=(const BorrowedFrame&) = delete;

    bool ok() const { return held_ && frame_.data != nullptr; }
    const CameraFrame& get() const { return frame_; }

    // Hand the camera back early, once the bytes have been copied out. The
    // destructor then does nothing, so every path out stays correct.
    void release() {
        if (held_) { camera_->EndFrame(); held_ = false; }
    }

private:
    Camera* camera_;
    CameraFrame frame_;
    bool held_ = false;
};
}  // namespace

bool EspDlFaceTracker::DetectOnce(uint32_t now_ms) {
    // Ask the camera for a frame rather than waiting for one to appear.
    // PeekFrame on its own lends whatever the photo path last captured, and
    // on this board the photo path runs only when someone asks for a photo —
    // so a detector built on PeekFrame alone counts "no frame" forever and
    // looks like a dead sensor.
    BorrowedFrame borrowed(camera_);
    if (!borrowed.ok()) {
        if (xSemaphoreTake(lock_, portMAX_DELAY) == pdTRUE) {
            ++stats_.no_frame;
            target_.visible = false;
            xSemaphoreGive(lock_);
        }
        return false;
    }
    const CameraFrame& frame = borrowed.get();

    dl::image::pix_type_t pix;
    if (!MapPixelFormat(frame.fourcc, &pix)) {
        if (!warned_format_) {
            warned_format_ = true;
            ESP_LOGE(TAG,
                     "camera gives %s (0x%08x); esp-dl reads RGB565, RGB24, "
                     "GREY and YUYV, so face tracking is off. Either pick one "
                     "of those when the sensor format is negotiated in "
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

    // Convert before inferring, and give the camera straight back.
    //
    // Inference on this sensor's frames takes the better part of a second.
    // Holding the camera's lock for all of it starves every other user —
    // take_photo timed out outright — for a saving of one memcpy. 150 KB out
    // of PSRAM costs a few milliseconds against ~870, so the copy is free in
    // the only terms that matter and the photo path stops competing with the
    // detector for the sensor.
    // RGB888 out, whatever came in. esp-dl advertises a YUYV pixel type and
    // the conversions behind it, and pointing the detector straight at the
    // sensor's YUYV buffer does run — it produced five faces at 0.99 on an
    // empty wooden ceiling, and the same five at every head angle, which is
    // the signature of a model reading something other than the picture.
    // Converting here costs one pass over 150 KB and removes that whole
    // question: the model gets the bytes this file computed, in the format
    // it was trained on.
    const bool yuyv = (pix == dl::image::DL_IMAGE_PIX_TYPE_YUYV);
    const size_t needed = yuyv ? (size_t)frame.width * frame.height * 3 : frame.len;
    if (scratch_size_ < needed) {
        if (scratch_) heap_caps_free(scratch_);
        // 16-byte aligned, and that is not a micro-optimisation.
        //
        // esp-dl's preprocessor reaches for the ESP32-S3's vector unit, whose
        // loads require 16-byte alignment. Hand it a byte-aligned PSRAM
        // buffer — which is what plain heap_caps_malloc returns — and the
        // model still runs, still returns boxes, and the boxes are nonsense:
        // two completely different detectors each reported their top-k cap at
        // a score of 1.000 on an empty ceiling, at an identical ~850 ms that
        // no amount of model difference could explain.
        scratch_ = (uint8_t*)heap_caps_aligned_alloc(
            16, (needed + 15) & ~(size_t)15, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        scratch_size_ = scratch_ ? needed : 0;
    }
    if (scratch_ == nullptr) {
        if (xSemaphoreTake(lock_, portMAX_DELAY) == pdTRUE) {
            ++stats_.no_frame;
            target_.visible = false;
            xSemaphoreGive(lock_);
        }
        return false;
    }
    if (yuyv) {
        Yuyv422ToRgb888(frame.data, frame.len, scratch_);
        pix = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
    } else {
        memcpy(scratch_, frame.data, needed);
    }
    const int fw = frame.width, fh = frame.height;
    const uint32_t ffourcc = frame.fourcc;
    borrowed.release();

    dl::image::img_t img;
    img.data = scratch_;
    img.width = fw;
    img.height = fh;
    img.pix_type = pix;

    // Sample the buffer the model is about to read, on a coarse grid so the
    // cost is nothing. A number that does not move when the head does means
    // the detector is not seeing the camera, whatever its scores say.
    uint32_t luma_sum = 0; uint32_t luma_n = 0;
    {
        const int step = (pix == dl::image::DL_IMAGE_PIX_TYPE_RGB888) ? 3 : 2;
        for (size_t o = 0; o + 2 < needed; o += step * 37) {
            luma_sum += scratch_[o]; ++luma_n;
        }
    }
    const uint8_t mean_luma = luma_n ? (uint8_t)(luma_sum / luma_n) : 0;

    const int64_t t0 = esp_timer_get_time();
    std::list<dl::detect::result_t>& results = detector_->run(img);
    const uint32_t latency = (esp_timer_get_time() - t0) / 1000;

    // Translate into this project's own plain type before choosing, so the
    // selection rule stays in face_geometry.h where it is tested on a host.
    std::vector<DetectedBox> boxes;
    boxes.reserve(results.size());
    float best = 0.0f;
    for (const auto& r : results) {
        if (r.score > best) best = r.score;
        if (r.score < cfg_.min_score) continue;
        if (r.box.size() < 4) continue;
        boxes.push_back(DetectedBox{r.box[0], r.box[1], r.box[2], r.box[3], r.score});
    }

    const DetectedBox* primary =
        PickPrimaryFace(boxes.begin(), boxes.end(), fw, fh);

    FaceTarget next;
    if (primary != nullptr) {
        next = BoxToTarget(*primary, fw, fh, now_ms);
    }

    if (xSemaphoreTake(lock_, portMAX_DELAY) == pdTRUE) {
        ++stats_.frames;
        if (next.visible) ++stats_.detections;
        stats_.last_latency_ms = latency;
        stats_.width = (uint16_t)fw;
        stats_.height = (uint16_t)fh;
        stats_.bytes = (uint32_t)needed;
        stats_.fourcc = ffourcc;
        stats_.raw_boxes = (uint16_t)results.size();
        stats_.kept_boxes = (uint16_t)boxes.size();
        stats_.best_score = best;
        stats_.mean_luma = mean_luma;
        if (primary != nullptr) {
            stats_.box_x1 = (int16_t)primary->left;  stats_.box_y1 = (int16_t)primary->top;
            stats_.box_x2 = (int16_t)primary->right; stats_.box_y2 = (int16_t)primary->bottom;
        }
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

#endif  // ESP_PLATFORM && CONFIG_STACKCHAN_FACE_DETECT
