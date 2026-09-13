// espdl_face_tracker.h — the real VisionTracker, backed by esp-dl.
//
// Implements the interface the attention system has been written against all
// along, so nothing downstream changes: the head controller does not know
// whether targets come from a neural network or from ScriptedVisionTracker.
//
// WHY A TASK
// Detection is not free. espressif/human_face_detect publishes its own timings
// for the ESP32-S3:
//
//     msr  preprocess 10.3 ms + model 30.6 ms + post 0.3 ms  = 41.3 ms
//     mnp  preprocess  1.2 ms + model  5.2 ms + post 0.1 ms  =  6.4 ms per candidate
//
// So one face costs roughly 48 ms — about 20 Hz at best, and that is the whole
// core doing nothing else. The servo loop runs at 40 Hz and the attention
// filter at 20 Hz; neither can afford to wait behind that. Detection therefore
// runs on its own task at a bounded rate and publishes its latest result,
// which is why VisionTracker::update() here does nothing: by the time the head
// asks, the answer already exists.
//
// This is also why the interface always had getTarget() separate from
// update(). It was written for exactly this.
#pragma once

#ifdef ESP_PLATFORM

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <cstdint>
#include <memory>

#include "attention_types.h"
#include "boards/common/camera.h"
#include "face_geometry.h"
#include "vision_tracker.h"

class HumanFaceDetect;   // espressif/human_face_detect

namespace stackchan {
namespace attention {

struct FaceTrackerConfig {
    // How often to look. The spec asks for 3-10 Hz and the model costs ~48 ms,
    // so 5 Hz leaves the core roughly three quarters of its time for
    // everything else. Raising this buys tracking responsiveness and spends
    // CPU the audio path also wants.
    uint32_t detect_period_ms = 200;

    // Detections below this are dropped before they reach the attention
    // controller, which has its own, separate minimum. Two gates because they
    // answer different questions: this one is "is that a face at all", the
    // other is "is it worth moving the head for".
    float min_score = 0.5f;

    // Task sizing. Detection allocates its working buffers from PSRAM, but the
    // stack still carries the call graph of a multi-stage model.
    uint32_t stack_words = 8192;
    UBaseType_t priority = 3;     // below audio, above idle
    BaseType_t core = 1;          // keep core 0 for the radio and audio
};

class EspDlFaceTracker : public VisionTracker {
public:
    EspDlFaceTracker(Camera* camera, const FaceTrackerConfig& cfg);
    ~EspDlFaceTracker() override;

    // Loads the model and starts the detection task. Returns false if the
    // model will not load or the camera cannot lend frames — in which case
    // this object stays alive and simply never reports a face, which the
    // attention system already handles as "nobody is there".
    bool begin();

    // Nothing: the task has already produced the answer. Kept because the
    // interface requires it and because a future tracker might need it.
    void update(uint32_t now_ms) override {}

    FaceTarget getTarget() const override;
    const char* name() const override { return "esp-dl human_face_detect"; }

    // Diagnostics. Cheap counters, because the failure that matters most here
    // is "it silently never sees anyone" and that looks identical to an empty
    // room unless something is counting.
    struct Stats {
        uint32_t frames = 0;         // frames actually run through the model
        uint32_t detections = 0;     // frames where at least one face passed
        uint32_t no_frame = 0;       // camera had nothing to lend
        uint32_t bad_format = 0;     // pixel format the model cannot read
        uint32_t last_latency_ms = 0;
    };
    Stats stats() const;

private:
    static void TaskEntry(void* self);
    void Run();
    bool DetectOnce(uint32_t now_ms);

    Camera* camera_;
    FaceTrackerConfig cfg_;
    std::unique_ptr<HumanFaceDetect> detector_;

    mutable SemaphoreHandle_t lock_ = nullptr;
    FaceTarget target_;            // guarded by lock_
    Stats stats_;                  // guarded by lock_

    TaskHandle_t task_ = nullptr;
    volatile bool running_ = false;
    bool warned_format_ = false;
};

}  // namespace attention
}  // namespace stackchan

#endif  // ESP_PLATFORM
