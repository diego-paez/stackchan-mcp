#include "head_controller.h"

#include <cmath>

#ifdef ESP_PLATFORM
#include "esp_log.h"
#define ATT_LOGI(fmt, ...) ESP_LOGI("attention", fmt, ##__VA_ARGS__)
#define ATT_LOGW(fmt, ...) ESP_LOGW("attention", fmt, ##__VA_ARGS__)
#else
#include <cstdio>
#define ATT_LOGI(fmt, ...) std::printf("[attention] " fmt "\n", ##__VA_ARGS__)
#define ATT_LOGW(fmt, ...) std::printf("[attention:W] " fmt "\n", ##__VA_ARGS__)
#endif

namespace stackchan {
namespace attention {

// Close enough to neutral to call it arrived, and the longest the approach is
// allowed to take before the self-test runs anyway.
static constexpr float kSettleToleranceDeg = 2.0f;
static constexpr uint32_t kSettleTimeoutMs = 8000;

const char* ToString(SelfTestState s) {
    switch (s) {
        case SelfTestState::kNotStarted: return "not-started";
        case SelfTestState::kRunning:    return "running";
        case SelfTestState::kPassed:     return "passed";
        case SelfTestState::kFailed:     return "failed";
    }
    return "?";
}

namespace {
// The self-test movements, in order, as relative offsets from neutral. These
// amplitudes are the ones specified for the first physical test and must not
// be raised: the point is to prove the chain moves and stops, not to explore
// the travel envelope.
struct TestStep { float dyaw; float dpitch; };
constexpr TestStep kSelfTest[] = {
    {  0.0f,  0.0f },   // settle at neutral
    { 10.0f,  0.0f },
    {  0.0f,  0.0f },
    {-10.0f,  0.0f },
    {  0.0f,  0.0f },
    {  0.0f,  5.0f },
    {  0.0f,  0.0f },
    {  0.0f, -5.0f },
    {  0.0f,  0.0f },
};
constexpr int kSelfTestCount = sizeof(kSelfTest) / sizeof(kSelfTest[0]);
constexpr uint32_t kSelfTestStepMs = 1200;   // slow, on purpose
}  // namespace

HeadController::HeadController(VisionTracker& vision, ServoSink& sink,
                               const ServoLimits& limits, const NeutralPose& neutral,
                               const HardwareBounds& bounds,
                               const AttentionConfig& attention_cfg,
                               const MotionConfig& motion_cfg,
                               const ScheduleConfig& schedule)
    : vision_(vision),
      sink_(sink),
      safety_(limits, neutral, bounds),
      attention_(attention_cfg, neutral),
      behavior_(neutral),
      motion_(motion_cfg, neutral),
      mixer_(neutral),
      neutral_(neutral),
      schedule_(schedule),
      greeting_(neutral),
      scan_(IdleScanConfig{}, neutral) {}

void HeadController::begin(uint32_t now_ms) {
    // Seed from the real pose. Without this the first command interpolates
    // from an assumed position, which is exactly the "boot snap" the board's
    // own notes describe.
    const HeadPose actual = sink_.readPose();
    motion_.syncTo(actual);
    attention_.reset();
    attention_.setEnabled(false);          // nothing tracks until the test passes
    behavior_.setBehavior(Behavior::LOOK_CENTER, now_ms);

    self_test_ = SelfTestState::kNotStarted;
    settling_to_neutral_ = true;
    settle_started_ms_ = now_ms;
    self_test_step_ = 0;
    self_test_step_ms_ = now_ms;
    started_ = true;
    last_vision_ms_ = last_attention_ms_ = last_behavior_ms_ = last_servo_ms_ = now_ms;
    last_mimic_ms_ = now_ms;
    now_ms_ = now_ms;

    ATT_LOGI("begin: pose yaw=%.1f pitch=%.1f, moving to neutral first",
             actual.yaw_deg, actual.pitch_deg);
}

void HeadController::setExpressionSink(ExpressionFn fn) {
    expression_ = fn;
    // The greeting keeps its own copy: it fires expressions from inside its
    // own timeline and must not have to reach back through this class.
    greeting_.setExpressionSink(std::move(fn));
}

void HeadController::observeAffect(const char* label, float confidence, uint32_t now_ms) {
    mimic_.observe(label, confidence, now_ms);
}

// The face is suppressed, not disabled, whenever something else owns the
// screen or the robot has been stopped. Suppressed means the policy keeps
// running and the answer stays current, so nothing snaps when it lifts.
void HeadController::updateMimic(uint32_t now_ms) {
    const bool someone_else_owns_the_face =
        self_test_ != SelfTestState::kPassed || greeting_.running();
    mimic_.setSuppressed(someone_else_owns_the_face);
    mimic_.setHalted(safety_.emergencyStopped(), now_ms);

    if (mimic_.update(now_ms) && expression_) expression_(mimic_.faceName());

    diag_.face = mimic_.face();
    diag_.affect = mimic_.affect();
}

void HeadController::setTrackingEnabled(bool on) {
    behavior_.setTrackingEnabled(on);
    attention_.setEnabled(on && self_test_ == SelfTestState::kPassed);
}

void HeadController::emergencyStop() {
    safety_.emergencyStop();
    attention_.setEnabled(false);
    attention_.reset();
    greeting_.cancel();
    // Same rule as the greeting: a stopped robot does not keep emoting. The
    // face returns to idle and stays there until the stop is cleared.
    mimic_.setHalted(true, now_ms_);
    ATT_LOGW("EMERGENCY STOP — holding position, all behaviour cancelled");
}

void HeadController::clearEmergencyStop(uint32_t now_ms) {
    safety_.clearEmergencyStop();
    motion_.syncTo(sink_.readPose());
    behavior_.setBehavior(Behavior::LOOK_CENTER, now_ms);
    ATT_LOGI("emergency stop cleared");
}

void HeadController::runSelfTest(uint32_t now_ms) {
    if (self_test_step_ >= kSelfTestCount) {
        self_test_ = SelfTestState::kPassed;
        ATT_LOGI("self-test passed: %u clamped, %u rejected",
                 (unsigned)safety_.clampedCount(), (unsigned)safety_.rejectedCount());
        if (greeting_enabled_) {
            // Expressive movement only now that the safety layer has been
            // shown to work on this unit. Tracking stays off until the
            // greeting is done, so the two never fight over the head.
            behavior_.setBehavior(Behavior::GREET, now_ms);
            greeting_.start(now_ms);
            ATT_LOGI("greeting: %d beats, %u ms scripted",
                     greeting_.stepCount(), (unsigned)greeting_.totalDurationMs());
        } else {
            beginTracking(now_ms);
        }
        return;
    }
    if ((now_ms - self_test_step_ms_) >= kSelfTestStepMs) {
        ++self_test_step_;
        self_test_step_ms_ = now_ms;
    }
}

void HeadController::beginTracking(uint32_t now_ms) {
    scan_.begin(now_ms);
    // Behaviour first, then read its settings. The other order asks the
    // *outgoing* behaviour whether tracking should be on — and the behaviour
    // on the way out is always one that had it off (LOOK_CENTER while the
    // self-test ran, GREET while the robot introduced itself). The attention
    // controller was therefore left disabled and the head never followed a
    // face, silently: everything else worked, the behaviour read
    // ATTEND_FACE, and the only symptom was a robot that ignored you.
    behavior_.setBehavior(Behavior::ATTEND_FACE, now_ms);
    attention_.setEnabled(behavior_.settings().tracking_enabled);
    ATT_LOGI("tracking enabled");
}

void HeadController::driveTo(const HeadPose& target, float dt_sec, uint32_t now_ms) {
    motion_.update(target, dt_sec);
    ServoCommand want = motion_.getCommand();

    if (want.valid) safety_.noteValidCommand(now_ms);
    if (safety_.watchdogExpired(now_ms)) want.valid = false;

    ServoCommand current;
    const HeadPose actual = sink_.readPose();
    current.valid = true;
    current.yaw_deg = actual.yaw_deg;
    current.pitch_deg = actual.pitch_deg;

    const SafetyResult r = safety_.filter(want, current, dt_sec);

    diag_.requested.yaw_deg = want.yaw_deg;
    diag_.requested.pitch_deg = want.pitch_deg;
    diag_.issued = r.command;
    diag_.verdict = r.verdict;

    if (r.command.valid && sink_.ready()) {
        sink_.write(r.command);
    }
}

void HeadController::update(uint32_t now_ms) {
    if (!started_) return;
    now_ms_ = now_ms;

    if ((now_ms - last_mimic_ms_) >= schedule_.mimic_period_ms) {
        last_mimic_ms_ = now_ms;
        updateMimic(now_ms);
    }

    if ((now_ms - last_behavior_ms_) >= schedule_.behavior_period_ms) {
        last_behavior_ms_ = now_ms;
        behavior_.update(now_ms);

        if (settling_to_neutral_) {
            // Arrived, or gave up waiting: a head that cannot reach neutral
            // still needs its self-test, because the self-test is how that
            // gets discovered.
            const HeadPose at = sink_.readPose();
            const float dy = at.yaw_deg - neutral_.yaw_deg;
            const float dp = at.pitch_deg - neutral_.pitch_deg;
            const float err = (dy < 0 ? -dy : dy) + (dp < 0 ? -dp : dp);
            if (err <= kSettleToleranceDeg ||
                (now_ms - settle_started_ms_) >= kSettleTimeoutMs) {
                settling_to_neutral_ = false;
                self_test_ = SelfTestState::kRunning;
                self_test_step_ = 0;
                self_test_step_ms_ = now_ms;
                ATT_LOGI("at neutral (err %.1f deg), self-test starting", err);
            }
        }

        if (self_test_ == SelfTestState::kRunning) runSelfTest(now_ms);
        if (greeting_.running()) {
            greeting_.update(now_ms);
            if (greeting_.finished()) beginTracking(now_ms);
        }
    }

    if ((now_ms - last_vision_ms_) >= schedule_.vision_period_ms) {
        last_vision_ms_ = now_ms;
        vision_.update(now_ms);
    }

    if ((now_ms - last_attention_ms_) >= schedule_.attention_period_ms) {
        const float dt = (now_ms - last_attention_ms_) / 1000.0f;
        last_attention_ms_ = now_ms;
        FaceTarget t = vision_.getTarget();
        diag_.raw_target = t;   // report what the detector said, always
        if (!follow_faces_) {
            // Seen but not acted on. The detector is reported through
            // diagnostics either way, so its behaviour stays observable
            // while the head declines to chase it. See setFollowFaces().
            t = FaceTarget{};
        }
        attention_.update(t, now_ms, dt);
        scan_.observeFace(t.visible, now_ms);
        scan_.update(now_ms);
        diag_.scan = scan_.state();
        diag_.filtered_x = attention_.filteredX();
        diag_.filtered_y = attention_.filteredY();
        diag_.tracking = attention_.state();
        diag_.lost_for_ms = attention_.lostForMs(now_ms);
    }

    if ((now_ms - last_servo_ms_) >= schedule_.servo_period_ms) {
        const float dt = (now_ms - last_servo_ms_) / 1000.0f;
        last_servo_ms_ = now_ms;

        HeadPose target;
        if (self_test_ == SelfTestState::kRunning) {
            const int i = (self_test_step_ < kSelfTestCount) ? self_test_step_
                                                             : kSelfTestCount - 1;
            target.yaw_deg = neutral_.yaw_deg + kSelfTest[i].dyaw;
            target.pitch_deg = neutral_.pitch_deg + kSelfTest[i].dpitch;
            diag_.source = MixSource::kBehaviorPose;
        } else if (greeting_.running()) {
            target = greeting_.pose();
            diag_.source = MixSource::kBehaviorPose;
        } else {
            const BehaviorSettings s = behavior_.settings();
            const bool has_target = attention_.state() != TrackingState::kNoTarget;
            const MixResult m = mixer_.mix(s, attention_.getTarget(), has_target);
            target = m.pose;
            diag_.source = m.source;

            // The sweep is the lowest-priority opinion about the head. It
            // speaks only when tracking is enabled, nothing is being tracked,
            // and no behaviour has offered a pose of its own — so LOOK_CENTER,
            // THINK and SLEEP all still win, and a face always wins.
            if (s.tracking_enabled && !s.has_fixed_pose && !has_target &&
                scan_.wantsControl()) {
                target = scan_.pose();
                diag_.source = MixSource::kBehaviorPose;
            }
        }

        diag_.behavior = behavior_.current();
        diag_.emergency_stopped = safety_.emergencyStopped();
        driveTo(target, dt, now_ms);
    }

    emit(now_ms);
}

void HeadController::emit(uint32_t now_ms) {
    if (!diag_enabled_) return;
    if ((now_ms - last_diag_ms_) < diag_period_ms_) return;
    last_diag_ms_ = now_ms;

    diag_.safety_rejected = safety_.rejectedCount();
    diag_.safety_clamped = safety_.clampedCount();

    ATT_LOGI("b=%s trk=%s src=%s face=%d raw(%.2f,%.2f) f(%.2f,%.2f) "
             "req(%.1f,%.1f) out(%.1f,%.1f) %s lost=%ums estop=%d rej=%u clamp=%u",
             ToString(diag_.behavior), ToString(diag_.tracking), ToString(diag_.source),
             diag_.raw_target.visible ? 1 : 0,
             diag_.raw_target.x, diag_.raw_target.y,
             diag_.filtered_x, diag_.filtered_y,
             diag_.requested.yaw_deg, diag_.requested.pitch_deg,
             diag_.issued.yaw_deg, diag_.issued.pitch_deg,
             ToString(diag_.verdict), (unsigned)diag_.lost_for_ms,
             diag_.emergency_stopped ? 1 : 0,
             (unsigned)diag_.safety_rejected, (unsigned)diag_.safety_clamped);
}

#ifdef STACKCHAN_ATTENTION_DEBUG_MANUAL
void HeadController::debugRequestPose(float yaw_deg, float pitch_deg, uint32_t now_ms) {
    HeadPose p;
    p.yaw_deg = yaw_deg;
    p.pitch_deg = pitch_deg;
    behavior_.setBehavior(Behavior::IDLE, now_ms);
    attention_.setEnabled(false);
    driveTo(p, 0.025f, now_ms);
}
#endif

}  // namespace attention
}  // namespace stackchan
