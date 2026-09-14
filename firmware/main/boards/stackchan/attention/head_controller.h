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
#include <utility>

#include "attention_controller.h"
#include "behavior_manager.h"
#include "face_mimic.h"
#include "greeting_routine.h"
#include "idle_scan.h"
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
    // 8 Hz, not 40.
    //
    // Each command becomes a StackChanBoard::WriteHeadAngles() with a
    // duration, and the board's motion driver interpolates toward it at
    // MOTION_TICK_MS. Re-commanding every 25 ms restarted that interpolation
    // before it could finish, and because the board tracks position in whole
    // degrees every restart discarded the unfinished fraction: measured on
    // hardware, the head crawled at 0.55 deg/s while the filter was allowing
    // 25, with `limited:velocity` on every diagnostic line.
    //
    // Commanding at a period the board can actually complete hands the
    // smoothing back to the driver that was written for it.
    uint32_t servo_period_ms = 120;
    uint32_t mimic_period_ms = 100;      // 10 Hz — the face is not a servo
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
    AvatarFace face = AvatarFace::kIdle;
    Affect affect = Affect::kUnknown;
    ScanState scan = ScanState::kOff;
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

    using ExpressionFn = GreetingRoutine::ExpressionFn;

    // --- looking for somebody ---------------------------------------------
    // With nobody in front of it the head sweeps a few stations, pausing long
    // enough at each for the detector to get clean frames. A voice interrupts
    // the sweep and centres the head — there is no direction of arrival to
    // turn toward, so centre is the honest guess. A face hands the head to
    // the tracker and this stops having an opinion. See idle_scan.h.
    void setIdleScanEnabled(bool on, uint32_t now_ms) { scan_.setEnabled(on, now_ms); }
    bool idleScanEnabled() const { return scan_.enabled(); }
    ScanState scanState() const { return scan_.state(); }
    ScanStats scanStats() const { return scan_.stats(); }
    void setIdleScanConfig(const IdleScanConfig& cfg) { scan_.setConfig(cfg); }
    void setScanDwellMs(uint32_t ms) { scan_.setDwellMs(ms); }
    void setScanAmplitudeDeg(float deg) { scan_.setYawAmplitudeDeg(deg); }
    void setScanPitchLiftDeg(float deg) { scan_.setPitchLiftDeg(deg); }
    IdleScanConfig idleScanConfig() const { return scan_.config(); }

    // The board already knows this from Application::IsVoiceDetected().
    void observeVoice(bool speaking, uint32_t now_ms) { scan_.observeVoice(speaking, now_ms); }

    // --- the face ---------------------------------------------------------
    // One sink, shared by the greeting and the mimic, for the same reason
    // MotionMixer exists: two components with independent opinions about one
    // output is how a screen ends up flickering between them. While the
    // greeting plays it owns the face outright and the mimic is suppressed.
    void setExpressionSink(ExpressionFn fn);

    // Report what the person sounds or looks like. Labels are the pipeline's
    // ("happy", "sad", "anger", ...); anything unrecognised is counted and
    // ignored rather than guessed at. Safe to call at any rate, including
    // once per utterance, which is what the transcribe endpoint affords.
    void observeAffect(const char* label, float confidence, uint32_t now_ms);

    void setMimicEnabled(bool on) { mimic_.setEnabled(on); }
    bool mimicEnabled() const { return mimic_.enabled(); }
    AvatarFace mimicFace() const { return mimic_.face(); }
    MimicStats mimicStats() const { return mimic_.stats(); }
    void setMimicConfig(const MimicConfig& cfg) { mimic_.setConfig(cfg); }
    void setMimicPolicy(const MimicPolicy& p) { mimic_.setPolicy(p); }

    // --- greeting ---------------------------------------------------------
    // Runs once, after the self-test passes and before tracking starts. The
    // sinks are optional; set them before begin() or the first beats go
    // nowhere. Disabling it hands straight from the self-test to tracking.
    void setGreetingEnabled(bool on) { greeting_enabled_ = on; }
    // Kept as the name the board integration already uses; the greeting and
    // the mimic have shared one sink since the mimic existed.
    void setGreetingExpressionSink(ExpressionFn fn) { setExpressionSink(std::move(fn)); }
    void setGreetingSpeechSink(GreetingRoutine::SpeechFn fn) {
        greeting_.setSpeechSink(std::move(fn));
    }
    bool greeting() const { return greeting_.running(); }
    bool greetingFinished() const { return greeting_.finished(); }

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
    void beginTracking(uint32_t now_ms);
    void driveTo(const HeadPose& target, float dt_sec, uint32_t now_ms);
    void emit(uint32_t now_ms);
    void updateMimic(uint32_t now_ms);

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

    // The head is driven to neutral BEFORE the self-test starts, which is
    // what the README has always described. It went unnoticed while neutral
    // happened to equal the board's boot pitch of 45; with neutral at 12 for
    // a low-mounted robot the first "±5° self-test beat" was really a 33°
    // journey, and the amplitude it asserts meant nothing.
    bool settling_to_neutral_ = false;
    uint32_t settle_started_ms_ = 0;

    SelfTestState self_test_ = SelfTestState::kNotStarted;
    int self_test_step_ = 0;
    uint32_t self_test_step_ms_ = 0;

    GreetingRoutine greeting_;
    bool greeting_enabled_ = true;

    IdleScan scan_;
    FaceMimic mimic_;
    ExpressionFn expression_;
    uint32_t last_mimic_ms_ = 0;
    uint32_t now_ms_ = 0;

    bool started_ = false;
    bool diag_enabled_ = true;
    uint32_t diag_period_ms_ = 500;
    Diagnostics diag_;
};

}  // namespace attention
}  // namespace stackchan
