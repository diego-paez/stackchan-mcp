// Host tests for the attention/servo-safety chain.
//
// These exist to prove one claim: that no command reaching the servo sink has
// bypassed the safety controller, and that the deliberately hostile inputs in
// the prototype specification are refused rather than quietly turned into
// something plausible.
//
// Nothing here touches ESP-IDF, a camera or a servo.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "attention/attention_controller.h"
#include "attention/behavior_manager.h"
#include "attention/face_geometry.h"
#include "attention/head_controller.h"
#include "attention/motion_controller.h"
#include "attention/motion_mixer.h"
#include "attention/servo_safety_controller.h"
#include "attention/servo_sink.h"
#include "attention/vision_tracker.h"

using namespace stackchan::attention;

namespace {

ServoLimits Limits() { return ServoLimits{}; }
NeutralPose Neutral() { return NeutralPose{}; }
HardwareBounds Bounds() { return HardwareBounds{}; }

ServoCommand At(float yaw, float pitch, float speed = 60.0f) {
    ServoCommand c;
    c.valid = true;
    c.yaw_deg = yaw;
    c.pitch_deg = pitch;
    c.speed_dps = speed;
    return c;
}

}  // namespace

// --------------------------------------------------------------------------
// The hostile inputs named in the specification.
// --------------------------------------------------------------------------
TEST(ServoSafety, RejectsAbsurdYaw) {
    ServoSafetyController s(Limits(), Neutral(), Bounds());
    const ServoCommand cur = At(0.0f, 45.0f);

    for (float yaw : {1000.0f, -1000.0f}) {
        const SafetyResult r = s.filter(At(yaw, 45.0f), cur, 0.025f);
        EXPECT_EQ(r.verdict, SafetyVerdict::kRejectedAbsurd) << "yaw " << yaw;
        // Held at the current pose, not clamped to the edge of travel.
        EXPECT_FLOAT_EQ(r.command.yaw_deg, cur.yaw_deg);
        EXPECT_FLOAT_EQ(r.command.pitch_deg, cur.pitch_deg);
    }
    EXPECT_EQ(s.rejectedCount(), 2u);
}

TEST(ServoSafety, RejectsNaNAndInfinity) {
    ServoSafetyController s(Limits(), Neutral(), Bounds());
    const ServoCommand cur = At(5.0f, 40.0f);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    for (float bad : {nan, inf, -inf}) {
        const SafetyResult r = s.filter(At(0.0f, bad), cur, 0.025f);
        EXPECT_EQ(r.verdict, SafetyVerdict::kRejectedNotFinite);
        EXPECT_FLOAT_EQ(r.command.pitch_deg, cur.pitch_deg);
        EXPECT_TRUE(std::isfinite(r.command.pitch_deg));
        EXPECT_TRUE(std::isfinite(r.command.yaw_deg));
    }
    // A NaN dt must not authorise anything either.
    const SafetyResult r = s.filter(At(10.0f, 45.0f), cur, nan);
    EXPECT_EQ(r.verdict, SafetyVerdict::kRejectedNotFinite);
}

TEST(ServoSafety, LimitsLargeInstantaneousJump) {
    ServoLimits lim = Limits();
    ServoSafetyController s(lim, Neutral(), Bounds());
    const ServoCommand cur = At(0.0f, 45.0f);

    // Inside the envelope, but the whole way across it in one update.
    const SafetyResult r = s.filter(At(lim.max_yaw_deg, 45.0f), cur, 0.025f);

    EXPECT_TRUE(r.modified);
    EXPECT_LE(std::fabs(r.command.yaw_deg - cur.yaw_deg), lim.max_step_deg + 1e-4f);
    EXPECT_LE(std::fabs(r.command.yaw_deg - cur.yaw_deg),
              lim.max_velocity_deg_per_sec * 0.025f + 1e-4f);
}

TEST(ServoSafety, ClampsOutsideTravelEnvelope) {
    ServoLimits lim = Limits();
    ServoSafetyController s(lim, Neutral(), Bounds());
    // Start at the edge so the rate limiter is not what stops it.
    const ServoCommand cur = At(lim.max_yaw_deg, lim.max_pitch_deg);

    const SafetyResult r = s.filter(At(lim.max_yaw_deg + 25.0f,
                                       lim.max_pitch_deg + 25.0f), cur, 0.025f);
    EXPECT_LE(r.command.yaw_deg, lim.max_yaw_deg);
    EXPECT_LE(r.command.pitch_deg, lim.max_pitch_deg);
    EXPECT_GT(s.clampedCount(), 0u);
}

TEST(ServoSafety, NeverLeavesTheHardwarePitchRangeEvenIfMisconfigured) {
    // Someone widens the config past what the hardware tolerates. The
    // hardware bounds are the floor under that mistake.
    ServoLimits lim = Limits();
    lim.min_pitch_deg = -50.0f;
    lim.max_pitch_deg = 200.0f;
    lim.max_step_deg = 500.0f;
    lim.max_velocity_deg_per_sec = 10000.0f;

    ServoSafetyController s(lim, Neutral(), Bounds());
    const SafetyResult hi = s.filter(At(0.0f, 150.0f), At(0.0f, 45.0f), 0.025f);
    const SafetyResult lo = s.filter(At(0.0f, -40.0f), At(0.0f, 45.0f), 0.025f);

    EXPECT_LE(hi.command.pitch_deg, Bounds().hard_max_pitch_deg);
    EXPECT_GE(lo.command.pitch_deg, Bounds().hard_min_pitch_deg);
}

TEST(ServoSafety, AbsurdDtCannotLicenseAHugeStep) {
    ServoLimits lim = Limits();
    ServoSafetyController s(lim, Neutral(), Bounds());
    // A stalled scheduler reports 30 seconds since the last tick.
    const SafetyResult r = s.filter(At(lim.max_yaw_deg, 45.0f), At(0.0f, 45.0f), 30.0f);
    EXPECT_LE(std::fabs(r.command.yaw_deg), lim.max_step_deg + 1e-4f);
}

TEST(ServoSafety, EmergencyStopHoldsAndRefuses) {
    ServoSafetyController s(Limits(), Neutral(), Bounds());
    const ServoCommand cur = At(12.0f, 50.0f);

    s.emergencyStop();
    const SafetyResult r = s.filter(At(-20.0f, 30.0f), cur, 0.025f);

    EXPECT_EQ(r.verdict, SafetyVerdict::kHeldEmergencyStop);
    EXPECT_FLOAT_EQ(r.command.yaw_deg, cur.yaw_deg);
    EXPECT_FLOAT_EQ(r.command.pitch_deg, cur.pitch_deg);
    EXPECT_TRUE(s.emergencyStopped());

    s.clearEmergencyStop();
    EXPECT_NE(s.filter(At(12.5f, 50.0f), cur, 0.025f).verdict,
              SafetyVerdict::kHeldEmergencyStop);
}

TEST(ServoSafety, WatchdogHoldsWhenCommandsStop) {
    ServoSafetyController s(Limits(), Neutral(), Bounds());
    s.setWatchdogTimeout(500);
    s.noteValidCommand(1000);

    EXPECT_FALSE(s.watchdogExpired(1400));
    EXPECT_TRUE(s.watchdogExpired(1600));

    // An invalid command holds position rather than driving anywhere.
    ServoCommand none;
    none.valid = false;
    const ServoCommand cur = At(7.0f, 47.0f);
    const SafetyResult r = s.filter(none, cur, 0.025f);
    EXPECT_EQ(r.verdict, SafetyVerdict::kHeldWatchdog);
    EXPECT_FLOAT_EQ(r.command.yaw_deg, 7.0f);
}

TEST(ServoSafety, IsSafeAgreesWithFilter) {
    ServoSafetyController s(Limits(), Neutral(), Bounds());
    EXPECT_TRUE(s.isSafe(At(10.0f, 50.0f)));
    EXPECT_FALSE(s.isSafe(At(1000.0f, 50.0f)));
    EXPECT_FALSE(s.isSafe(At(0.0f, std::numeric_limits<float>::quiet_NaN())));
    ServoCommand invalid;
    EXPECT_FALSE(s.isSafe(invalid));
}

// --------------------------------------------------------------------------
// Tracking behaviour
// --------------------------------------------------------------------------
TEST(Attention, DeadZoneProducesNoMotion) {
    AttentionConfig cfg;
    AttentionController a(cfg, Neutral());

    FaceTarget t;
    t.visible = true;
    t.confidence = 0.9f;
    t.x = cfg.deadzone_x * 0.5f;   // inside the dead zone
    t.y = 0.0f;

    const HeadPose before = a.getTarget();
    for (uint32_t ms = 0; ms < 1000; ms += 40) a.update(t, ms, 0.04f);
    const HeadPose after = a.getTarget();

    EXPECT_NEAR(before.yaw_deg, after.yaw_deg, 1e-3f);
    EXPECT_EQ(a.state(), TrackingState::kTracking);
}

TEST(Attention, FaceOffCentreMovesTheTargetTowardIt) {
    AttentionConfig cfg;
    cfg.invert_yaw = false;
    AttentionController a(cfg, Neutral());

    FaceTarget t;
    t.visible = true;
    t.confidence = 0.9f;
    t.x = 0.6f;                      // well outside the dead zone
    t.y = 0.0f;

    for (uint32_t ms = 0; ms < 500; ms += 40) a.update(t, ms, 0.04f);
    EXPECT_GT(a.getTarget().yaw_deg, 0.5f);
}

TEST(Attention, LowConfidenceIsIgnored) {
    AttentionConfig cfg;
    cfg.min_confidence = 0.5f;
    AttentionController a(cfg, Neutral());

    FaceTarget t;
    t.visible = true;
    t.confidence = 0.2f;             // below the bar
    t.x = 0.9f;

    for (uint32_t ms = 0; ms < 400; ms += 40) a.update(t, ms, 0.04f);
    EXPECT_EQ(a.state(), TrackingState::kNoTarget);
    EXPECT_NEAR(a.getTarget().yaw_deg, Neutral().yaw_deg, 1e-3f);
}

TEST(Attention, BrieflyLostFaceHoldsThenRelaxes) {
    AttentionConfig cfg;
    AttentionController a(cfg, Neutral());

    FaceTarget seen;
    seen.visible = true;
    seen.confidence = 0.9f;
    seen.x = 0.5f;
    uint32_t ms = 0;
    for (; ms < 600; ms += 40) a.update(seen, ms, 0.04f);
    const float held_at = a.getTarget().yaw_deg;
    EXPECT_GT(held_at, 0.5f);

    FaceTarget gone;
    gone.visible = false;

    a.update(gone, ms + 100, 0.04f);                 // 100 ms lost
    EXPECT_EQ(a.state(), TrackingState::kHolding);
    EXPECT_NEAR(a.getTarget().yaw_deg, held_at, 1e-3f);

    a.update(gone, ms + 900, 0.04f);                 // 900 ms lost
    EXPECT_EQ(a.state(), TrackingState::kRelaxing);
    EXPECT_LT(a.getTarget().yaw_deg, held_at);       // drifting back
}

TEST(Attention, NoiseDoesNotProduceContinuousOscillation) {
    // A face sitting at the centre with detector jitter must not keep the
    // head moving: this is the oscillation the specification calls out.
    AttentionConfig cfg;
    AttentionController a(cfg, Neutral());

    FaceTarget t;
    t.visible = true;
    t.confidence = 0.9f;

    float jitter = cfg.deadzone_x * 0.6f;
    for (uint32_t ms = 0; ms < 3000; ms += 40) {
        t.x = (ms / 40) % 2 ? jitter : -jitter;   // flip-flop inside the zone
        t.y = 0.0f;
        a.update(t, ms, 0.04f);
    }
    EXPECT_NEAR(a.getTarget().yaw_deg, Neutral().yaw_deg, 0.5f);
}

// --------------------------------------------------------------------------
// Behaviour
// --------------------------------------------------------------------------
TEST(Behaviors, ThinkIsTemporaryAndReturnsToAttend) {
    BehaviorManager b(Neutral());
    b.setBehavior(Behavior::ATTEND_FACE, 0);
    EXPECT_TRUE(b.settings().tracking_enabled);

    b.setThinkHoldMs(1000);
    b.setBehavior(Behavior::THINK, 100);
    EXPECT_FALSE(b.settings().tracking_enabled);
    EXPECT_TRUE(b.settings().has_fixed_pose);

    b.update(500);
    EXPECT_EQ(b.current(), Behavior::THINK);
    b.update(1300);
    EXPECT_EQ(b.current(), Behavior::ATTEND_FACE);
    EXPECT_TRUE(b.settings().tracking_enabled);
}

TEST(Behaviors, SleepAndLookCenterDisableTracking) {
    BehaviorManager b(Neutral());
    for (Behavior x : {Behavior::SLEEP, Behavior::LOOK_CENTER, Behavior::IDLE}) {
        b.setBehavior(x, 0);
        EXPECT_FALSE(b.settings().tracking_enabled) << ToString(x);
    }
}

TEST(Mixer, FixedBehaviorPoseBeatsTracking) {
    MotionMixer m(Neutral());
    BehaviorSettings s;
    s.tracking_enabled = true;
    s.has_fixed_pose = true;
    s.fixed_pose.yaw_deg = 8.0f;
    s.fixed_pose.pitch_deg = 51.0f;

    HeadPose attention;
    attention.yaw_deg = -25.0f;

    const MixResult r = m.mix(s, attention, true);
    EXPECT_EQ(r.source, MixSource::kBehaviorPose);
    EXPECT_FLOAT_EQ(r.pose.yaw_deg, 8.0f);
}

// --------------------------------------------------------------------------
// The whole chain
// --------------------------------------------------------------------------
TEST(HeadController, NothingReachesTheServoOutsideTheEnvelope) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    ScheduleConfig sched;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, sched);
    hc.setDiagnosticsEnabled(false);
    hc.begin(0);

    const ServoLimits lim = Limits();
    uint32_t ms = 0;
    // Drive a face hard to one corner for 20 seconds of simulated time.
    for (; ms < 20000; ms += 10) {
        vision.see(0.95f, -0.95f, ms);
        hc.update(ms);
        if (sink.last.valid) {
            EXPECT_GE(sink.last.yaw_deg, lim.min_yaw_deg - 1e-3f);
            EXPECT_LE(sink.last.yaw_deg, lim.max_yaw_deg + 1e-3f);
            EXPECT_GE(sink.last.pitch_deg, lim.min_pitch_deg - 1e-3f);
            EXPECT_LE(sink.last.pitch_deg, lim.max_pitch_deg + 1e-3f);
            EXPECT_LE(sink.last.speed_dps, lim.max_speed_dps + 1e-3f);
        }
    }
    EXPECT_GT(sink.writes, 100);
    EXPECT_EQ(hc.selfTestState(), SelfTestState::kPassed);
}

TEST(HeadController, ConsecutiveCommandsNeverJump) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);
    hc.begin(0);

    const ServoLimits lim = Limits();
    float prev_yaw = Neutral().yaw_deg, prev_pitch = Neutral().pitch_deg;
    bool first = true;

    for (uint32_t ms = 0; ms < 25000; ms += 10) {
        // Teleport the face between opposite corners every half second — the
        // worst case a detector can hand us.
        const bool left = ((ms / 500) % 2) == 0;
        vision.see(left ? -0.95f : 0.95f, left ? 0.9f : -0.9f, ms);
        hc.update(ms);
        if (!sink.last.valid) continue;
        if (!first) {
            EXPECT_LE(std::fabs(sink.last.yaw_deg - prev_yaw), lim.max_step_deg + 1e-3f);
            EXPECT_LE(std::fabs(sink.last.pitch_deg - prev_pitch), lim.max_step_deg + 1e-3f);
        }
        prev_yaw = sink.last.yaw_deg;
        prev_pitch = sink.last.pitch_deg;
        first = false;
    }
}

TEST(HeadController, EmergencyStopFreezesTheOutput) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);
    hc.begin(0);

    uint32_t ms = 0;
    for (; ms < 15000; ms += 10) { vision.see(0.8f, 0.0f, ms); hc.update(ms); }

    hc.emergencyStop();
    const float frozen_yaw = sink.last.yaw_deg;
    const float frozen_pitch = sink.last.pitch_deg;

    for (; ms < 18000; ms += 10) { vision.see(-0.9f, 0.9f, ms); hc.update(ms); }

    EXPECT_TRUE(hc.emergencyStopped());
    EXPECT_NEAR(sink.last.yaw_deg, frozen_yaw, 1e-3f);
    EXPECT_NEAR(sink.last.pitch_deg, frozen_pitch, 1e-3f);
}

TEST(HeadController, TrackingIsDisabledUntilTheSelfTestPasses) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);
    hc.begin(0);

    EXPECT_EQ(hc.selfTestState(), SelfTestState::kRunning);
    // During the test the face is ignored: the source is never the tracker.
    for (uint32_t ms = 0; ms < 3000; ms += 10) {
        vision.see(0.9f, 0.0f, ms);
        hc.update(ms);
        EXPECT_NE(hc.diagnostics().source, MixSource::kAttention);
    }
}

TEST(HeadController, SelfTestStaysWithinItsOwnSmallAmplitudes) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);
    hc.begin(0);

    for (uint32_t ms = 0; ms < 12000 && hc.selfTestState() == SelfTestState::kRunning;
         ms += 10) {
        hc.update(ms);
        if (!sink.last.valid) continue;
        EXPECT_LE(std::fabs(sink.last.yaw_deg - Neutral().yaw_deg), 10.0f + 1e-3f);
        EXPECT_LE(std::fabs(sink.last.pitch_deg - Neutral().pitch_deg), 5.0f + 1e-3f);
    }
    EXPECT_EQ(hc.selfTestState(), SelfTestState::kPassed);
}

// ---------------------------------------------------------------------------
// Greeting routine
//
// The greeting is the first expressive movement the robot makes and the first
// thing a stranger sees, so it gets the same treatment as everything else that
// can move the head: prove it cannot escape the envelope, cannot start before
// the safety layer has been shown to work, and cannot fight the face tracker.
// ---------------------------------------------------------------------------

TEST(Greeting, DoesNotStartUntilTheSelfTestPasses) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);
    hc.begin(0);

    // Through the whole self-test — 9 beats at 1200 ms, so ~10.8 s — nothing
    // of the greeting has happened.
    for (uint32_t ms = 0; ms < 12000; ms += 10) {
        hc.update(ms);
        if (hc.selfTestState() != SelfTestState::kPassed) {
            EXPECT_FALSE(hc.greeting());
            EXPECT_NE(hc.behavior(), Behavior::GREET);
        }
    }
    EXPECT_EQ(hc.selfTestState(), SelfTestState::kPassed);
}

TEST(Greeting, SpeaksItsLineOnceAndShowsExpressions) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);

    std::vector<std::string> said;
    std::vector<std::string> shown;
    hc.setGreetingSpeechSink([&](const char* s) { said.push_back(s); });
    hc.setGreetingExpressionSink([&](const char* e) { shown.push_back(e); });

    hc.begin(0);
    for (uint32_t ms = 0; ms < 30000; ms += 10) hc.update(ms);

    ASSERT_EQ(said.size(), 1u) << "the line must be spoken exactly once";
    EXPECT_EQ(said[0], "Greetings, I am Stacky");

    ASSERT_FALSE(shown.empty());
    EXPECT_EQ(shown.front(), "neutral");
    EXPECT_EQ(shown.back(), "neutral") << "must settle back to a neutral face";
    EXPECT_NE(std::find(shown.begin(), shown.end(), "happy"), shown.end());
}

TEST(Greeting, HandsOverToFaceTrackingWhenDone) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);
    hc.begin(0);

    bool saw_greeting = false;
    for (uint32_t ms = 0; ms < 30000; ms += 10) {
        hc.update(ms);
        if (hc.greeting()) {
            saw_greeting = true;
            // While greeting, the tracker must not also be steering.
            EXPECT_EQ(hc.behavior(), Behavior::GREET);
        }
    }
    EXPECT_TRUE(saw_greeting);
    EXPECT_TRUE(hc.greetingFinished());
    EXPECT_EQ(hc.behavior(), Behavior::ATTEND_FACE)
        << "the robot must end up tracking faces, not stuck in the greeting";
}

TEST(Greeting, NeverLeavesTheSafeEnvelope) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);
    hc.begin(0);

    const ServoLimits lim = Limits();
    float prev_yaw = Neutral().yaw_deg, prev_pitch = Neutral().pitch_deg;
    bool first = true;

    for (uint32_t ms = 0; ms < 30000; ms += 10) {
        hc.update(ms);
        if (!sink.last.valid) continue;
        EXPECT_GE(sink.last.yaw_deg, lim.min_yaw_deg - 1e-3f);
        EXPECT_LE(sink.last.yaw_deg, lim.max_yaw_deg + 1e-3f);
        EXPECT_GE(sink.last.pitch_deg, lim.min_pitch_deg - 1e-3f);
        EXPECT_LE(sink.last.pitch_deg, lim.max_pitch_deg + 1e-3f);
        if (!first) {
            EXPECT_LE(std::fabs(sink.last.yaw_deg - prev_yaw), lim.max_step_deg + 1e-3f);
            EXPECT_LE(std::fabs(sink.last.pitch_deg - prev_pitch), lim.max_step_deg + 1e-3f);
        }
        prev_yaw = sink.last.yaw_deg;
        prev_pitch = sink.last.pitch_deg;
        first = false;
    }
}

TEST(Greeting, EmergencyStopSilencesItMidSentence) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);

    std::vector<std::string> said;
    hc.setGreetingSpeechSink([&](const char* s) { said.push_back(s); });
    hc.begin(0);

    // Run until the greeting is under way, then pull the cord.
    uint32_t ms = 0;
    for (; ms < 30000 && !hc.greeting(); ms += 10) hc.update(ms);
    ASSERT_TRUE(hc.greeting());

    hc.emergencyStop();
    const size_t said_at_stop = said.size();
    for (; ms < 40000; ms += 10) hc.update(ms);

    EXPECT_FALSE(hc.greeting()) << "a stopped robot must not keep performing";
    EXPECT_EQ(said.size(), said_at_stop) << "and must not speak after the stop";
    EXPECT_TRUE(hc.emergencyStopped());
}

TEST(Greeting, CanBeDisabledAndThenTrackingStartsImmediately) {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);
    hc.setGreetingEnabled(false);
    hc.begin(0);

    for (uint32_t ms = 0; ms < 20000; ms += 10) {
        hc.update(ms);
        EXPECT_FALSE(hc.greeting());
    }
    EXPECT_EQ(hc.selfTestState(), SelfTestState::kPassed);
    EXPECT_EQ(hc.behavior(), Behavior::ATTEND_FACE);
}

TEST(HeadController, ActuallyFollowsAFaceOnceStartupIsDone) {
    // The regression this exists for: everything reported healthy — self-test
    // passed, behaviour ATTEND_FACE — while the attention controller sat
    // disabled, because tracking was enabled by asking the behaviour that was
    // on its way out. The head simply never moved toward anyone.
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc(vision, sink, Limits(), Neutral(), Bounds(),
                      AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    hc.setDiagnosticsEnabled(false);
    hc.begin(0);

    uint32_t ms = 0;
    for (; ms < 30000 && !hc.greetingFinished(); ms += 10) hc.update(ms);
    ASSERT_EQ(hc.behavior(), Behavior::ATTEND_FACE);

    const float yaw_before = sink.last.yaw_deg;
    for (uint32_t end = ms + 4000; ms < end; ms += 10) {
        vision.see(-0.6f, 0.0f, ms);   // a face well off to one side
        hc.update(ms);
    }

    EXPECT_NE(hc.diagnostics().tracking, TrackingState::kNoTarget)
        << "a visible face must register as a target";
    EXPECT_GT(std::fabs(sink.last.yaw_deg - yaw_before), 3.0f)
        << "the head must actually turn toward it";
}

// ---------------------------------------------------------------------------
// Face geometry — box in pixels to FaceTarget in -1..+1
//
// The failure mode this guards is not a crash. A sign error here yields a head
// that turns smoothly, confidently, and the wrong way.
// ---------------------------------------------------------------------------

TEST(FaceGeometry, CentredFaceIsAtTheOrigin) {
    // 160x120, a 40x40 face dead centre.
    DetectedBox b{60, 40, 100, 80, 0.9f};
    const FaceTarget t = BoxToTarget(b, 160, 120, 1000);
    EXPECT_TRUE(t.visible);
    EXPECT_NEAR(t.x, 0.0f, 1e-5f);
    EXPECT_NEAR(t.y, 0.0f, 1e-5f);
    EXPECT_NEAR(t.confidence, 0.9f, 1e-5f);
    EXPECT_EQ(t.last_seen_ms, 1000u);
}

TEST(FaceGeometry, RightOfFrameIsPositiveXAndBottomIsPositiveY) {
    // Image coordinates grow rightward and DOWNWARD. AttentionController
    // inverts pitch itself; flipping y here would double-invert it.
    DetectedBox right{120, 40, 160, 80, 0.8f};
    EXPECT_GT(BoxToTarget(right, 160, 120, 0).x, 0.4f);

    DetectedBox low{60, 80, 100, 120, 0.8f};
    EXPECT_GT(BoxToTarget(low, 160, 120, 0).y, 0.4f)
        << "a face low in the frame must give POSITIVE y";

    DetectedBox high{60, 0, 100, 40, 0.8f};
    EXPECT_LT(BoxToTarget(high, 160, 120, 0).y, -0.4f)
        << "a face high in the frame must give NEGATIVE y";
}

TEST(FaceGeometry, SizeIsTheLargerEdgeAsAFraction) {
    DetectedBox b{0, 0, 80, 30, 0.7f};       // 80 wide of 160, 30 tall of 120
    const FaceTarget t = BoxToTarget(b, 160, 120, 0);
    EXPECT_NEAR(t.size, 0.5f, 1e-5f);        // max(80/160, 30/120) = 0.5
}

TEST(FaceGeometry, NeverLeavesTheUnitSquare) {
    // A face half out of shot: the box runs past the frame edge.
    DetectedBox b{140, 100, 260, 220, 0.95f};
    const FaceTarget t = BoxToTarget(b, 160, 120, 0);
    EXPECT_LE(t.x, 1.0f); EXPECT_GE(t.x, -1.0f);
    EXPECT_LE(t.y, 1.0f); EXPECT_GE(t.y, -1.0f);
    EXPECT_LE(t.size, 1.0f); EXPECT_GE(t.size, 0.0f);
    EXPECT_LE(t.confidence, 1.0f);
}

TEST(FaceGeometry, RejectsDegenerateBoxes) {
    EXPECT_FALSE(BoxToTarget(DetectedBox{50, 50, 50, 80, 0.9f}, 160, 120, 0).visible)
        << "zero width";
    EXPECT_FALSE(BoxToTarget(DetectedBox{100, 50, 40, 80, 0.9f}, 160, 120, 0).visible)
        << "corners the wrong way round";
    EXPECT_FALSE(BoxToTarget(DetectedBox{10, 10, 50, 50, 0.9f}, 0, 0, 0).visible)
        << "a frame with no size";
    EXPECT_FALSE(BoxToTarget(DetectedBox{200, 10, 260, 50, 0.9f}, 160, 120, 0).visible)
        << "entirely off the right edge";
}

TEST(FaceGeometry, PicksTheNearestFaceNotTheMostConfidentOne) {
    // A group of children: the one closest to the robot is almost always the
    // one talking to it, and box area is the only depth cue a single camera
    // gives. A confident detection of a distant face must not win.
    std::vector<DetectedBox> faces = {
        {10, 10,  30,  30, 0.99f},   // small, very confident — someone behind
        {60, 30, 130, 100, 0.71f},   // large, less confident — the child in front
    };
    const DetectedBox* p = PickPrimaryFace(faces.begin(), faces.end(), 160, 120);
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->left, 60) << "the larger box must win";

    // Equal area: the score breaks the tie.
    std::vector<DetectedBox> tie = {
        {0, 0, 40, 40, 0.60f},
        {80, 0, 120, 40, 0.90f},
    };
    const DetectedBox* q = PickPrimaryFace(tie.begin(), tie.end(), 160, 120);
    ASSERT_NE(q, nullptr);
    EXPECT_FLOAT_EQ(q->score, 0.90f);
}

TEST(FaceGeometry, EmptyOrAllUnusableGivesNothing) {
    std::vector<DetectedBox> none;
    EXPECT_EQ(PickPrimaryFace(none.begin(), none.end(), 160, 120), nullptr);

    std::vector<DetectedBox> junk = {{50, 50, 50, 50, 0.9f}, {90, 10, 20, 40, 0.9f}};
    EXPECT_EQ(PickPrimaryFace(junk.begin(), junk.end(), 160, 120), nullptr);
}

TEST(FaceGeometry, FeedsTheAttentionControllerCoherently) {
    // End to end through the real controller: a face low and to the right of
    // frame must move the target right and, because the controller inverts
    // pitch, DOWN in gaze — i.e. toward the child.
    AttentionConfig cfg;
    AttentionController att(cfg, Neutral());
    att.setEnabled(true);
    att.reset();

    DetectedBox b{120, 90, 158, 118, 0.9f};      // right, low
    const FaceTarget t = BoxToTarget(b, 160, 120, 100);
    ASSERT_TRUE(t.visible);

    for (uint32_t ms = 100; ms < 1600; ms += 50) {
        FaceTarget f = t; f.last_seen_ms = ms;
        att.update(f, ms, 0.05f);
    }
    EXPECT_GT(att.getTarget().yaw_deg, Neutral().yaw_deg + 1.0f)
        << "a face to the right must turn the head right";
    EXPECT_LT(att.getTarget().pitch_deg, Neutral().pitch_deg - 0.5f)
        << "a face low in frame must lower the gaze";
}
