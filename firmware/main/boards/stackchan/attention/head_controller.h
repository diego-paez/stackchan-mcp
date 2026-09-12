// head_controller.h — the whole chain, wired once, with one exit.
//
//   VisionTracker -> AttentionController -.
//   BehaviorManager ---------------------> MotionMixer -> MotionController
//                                                              |
//                                              ServoSafetyController
//                                                              |
//                                                         ServoSink -> hardware
//
// External code talks to this class and to nothing below it. There is no
// method here that takes a raw angle outside a debug build, and even that one
// goes through the same safety controller.
//
// update() is non-blocking and expects to be called often; it runs each stage
// on its own timer rather than sleeping. Nothing here calls delay().
#pragma once

#include <cstdint>

#include "attention_controller.h"
#include "behavior_manager.h"
#include "motion_controller.h"
#include "motion_mixer.h"
#include "servo_safety_controller.h"
#include "servo_sink.h"
#include "vision_tracker.h"

namespace stackchan {
namespace attention {

struct ScheduleConfig {
    uint32_t vision_period_ms = 200;     //  5 Hz
    uint32_t attention_period_ms = 40;   // 25 Hz
    uint32_t behavior_period_ms = 50;    // 20 Hz
    uint32_t servo_period_ms = 25;       // 40 Hz
};

// Progress of the pre-flight movement test. Tracking stays disabled until
// this reports kPassed, so the first thing a servo ever does under this
// controller is a bounded, slow, known movement.
enum class SelfTestState : uint8_t {
    kNotStarted,
    kRunning,
    kPassed,
    kFailed,
};

const char* ToString(SelfTestState s);

struct Diagnostics {
    FaceTarget raw_target;
    float filtered_x = 0.0f;
    float filtered_y = 0.0f;
    HeadPose requested;
    ServoCommand issued;
    SafetyVerdict verdict = SafetyVerdict::kOk;
    Behavior behavior = Behavior::IDLE;
    TrackingState tracking = TrackingState::kNoTarget;
    MixSource source = MixSource::kNeutral;
    uint32_t lost_for_ms = 0;
    bool emergency_stopped = false;
    uint32_t safety_rejected = 0;
    uint32_t safety_clamped = 0;
};

class HeadController {
public:
    HeadController(VisionTracker& vision, ServoSink& sink, const ServoLimits& limits,
                   const NeutralPose& neutral, const HardwareBounds& bounds,
                   const AttentionConfig& attention_cfg, const MotionConfig& motion_cfg,
                   const ScheduleConfig& schedule);

    // Move to neutral slowly, then run the self-test. Tracking is not enabled
    // until both have finished.
    void begin(uint32_t now_ms);

    // Call as often as convenient. Internally rate-limited per stage.
    void update(uint32_t now_ms);

    // --- the external API. Semantic only. ---
    void setBehavior(Behavior b, uint32_t now_ms) { behavior_.setBehavior(b, now_ms); }
    Behavior behavior() const { return behavior_.current(); }
    void setTrackingEnabled(bool on);
    void emergencyStop();
    void clearEmergencyStop(uint32_t now_ms);
    bool emergencyStopped() const { return safety_.emergencyStopped(); }

    SelfTestState selfTestState() const { return self_test_; }
    Diagnostics diagnostics() const { return diag_; }

    // Diagnostics printing is opt-in and rate-limited: at 40 Hz an
    // unconditional log line is a denial of service on the serial port.
    void setDiagnosticsEnabled(bool on) { diag_enabled_ = on; }
    void setDiagnosticsPeriodMs(uint32_t ms) { diag_period_ms_ = ms; }

#ifdef STACKCHAN_ATTENTION_DEBUG_MANUAL
    // Development only, and deliberately routed through the same filter as
    // everything else. It is not compiled into a normal build, and it is not
    // reachable from the MCP layer.
    void debugRequestPose(float yaw_deg, float pitch_deg, uint32_t now_ms);
#endif

private:
    void runSelfTest(uint32_t now_ms);
    void driveTo(const HeadPose& target, float dt_sec, uint32_t now_ms);
    void emit(uint32_t now_ms);

    VisionTracker& vision_;
    ServoSink& sink_;

    ServoSafetyController safety_;
    AttentionController attention_;
    BehaviorManager behavior_;
    MotionController motion_;
    MotionMixer mixer_;
    NeutralPose neutral_;
    ScheduleConfig schedule_;

    uint32_t last_vision_ms_ = 0;
    uint32_t last_attention_ms_ = 0;
    uint32_t last_behavior_ms_ = 0;
    uint32_t last_servo_ms_ = 0;
    uint32_t last_diag_ms_ = 0;

    SelfTestState self_test_ = SelfTestState::kNotStarted;
    int self_test_step_ = 0;
    uint32_t self_test_step_ms_ = 0;

    bool started_ = false;
    bool diag_enabled_ = true;
    uint32_t diag_period_ms_ = 500;
    Diagnostics diag_;
};

}  // namespace attention
}  // namespace stackchan
