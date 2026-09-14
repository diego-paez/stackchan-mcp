// Host tests for the idle search cycle.
//
// The claims worth proving are about priority and about not moving: that a
// face always outranks the sweep, that the sweep never fights a behaviour
// that has its own opinion, that a dropped detection does not make the head
// lurch away from somebody who is still there, and that the whole thing stays
// inside the envelope the safety layer would have clamped anyway.
#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include "attention/head_controller.h"
#include "attention/idle_scan.h"
#include "attention/servo_sink.h"
#include "attention/vision_tracker.h"

using namespace stackchan::attention;

namespace {

ServoLimits Limits() { return ServoLimits{}; }
NeutralPose Neutral() { return NeutralPose{}; }
HardwareBounds Bounds() { return HardwareBounds{}; }

IdleScan Fresh(uint32_t at = 0) {
    IdleScan s(IdleScanConfig{}, Neutral());
    s.begin(at);
    return s;
}

}  // namespace

// ---------------------------------------------------------------------------
// The cycle itself
// ---------------------------------------------------------------------------

TEST(IdleScanCycle, StartsScanningAndVisitsEveryStation) {
    IdleScan s = Fresh();
    std::set<int> yaws;
    for (uint32_t t = 0; t < 40000; t += 50) {
        s.update(t);
        yaws.insert(static_cast<int>(std::lround(s.pose().yaw_deg)));
    }
    EXPECT_EQ(s.state(), ScanState::kScanning);
    EXPECT_GE(static_cast<int>(yaws.size()), 4)
        << "the sweep should visit several distinct places, not oscillate";
    EXPECT_GT(s.stats().sweeps, 0u);
}

TEST(IdleScanCycle, TheSweepStaysWellInsideTheYawEnvelope) {
    // If the sweep ever needs the safety clamp, the amplitude is wrong.
    IdleScan s = Fresh();
    const ServoLimits lim = Limits();
    for (uint32_t t = 0; t < 60000; t += 50) {
        s.update(t);
        EXPECT_GT(s.pose().yaw_deg, lim.min_yaw_deg)
            << "sweep reached the yaw limit at t=" << t;
        EXPECT_LT(s.pose().yaw_deg, lim.max_yaw_deg)
            << "sweep reached the yaw limit at t=" << t;
        EXPECT_GE(s.pose().pitch_deg, lim.min_pitch_deg);
        EXPECT_LE(s.pose().pitch_deg, lim.max_pitch_deg);
    }
}

TEST(IdleScanCycle, ItPausesLongEnoughForTheDetectorToSeeAnything) {
    // A 5 Hz detector needs the scene to hold still. A sweep that never stops
    // is a sweep that never finds a face, which is the whole failure mode.
    IdleScanConfig cfg;
    IdleScan s(cfg, Neutral());
    s.begin(0);

    float last = s.pose().yaw_deg;
    uint32_t last_change = 0;
    uint32_t shortest = 0xFFFFFFFFu;
    for (uint32_t t = 0; t < 40000; t += 10) {
        s.update(t);
        if (s.pose().yaw_deg != last) {
            if (last_change > 0) shortest = std::min(shortest, t - last_change);
            last_change = t;
            last = s.pose().yaw_deg;
        }
    }
    EXPECT_GE(shortest, cfg.dwell_ms)
        << "the head moved on before the detector could get a clean look";
    // 1400 ms at 5 Hz is seven frames.
    EXPECT_GE(cfg.dwell_ms * 5u / 1000u, 5u);
}

TEST(IdleScanCycle, AVoiceStopsTheSweepAndCentresTheHead) {
    // There is no direction of arrival on this hardware, so a voice cannot say
    // where to look — only that looking is worthwhile. Centre is the guess.
    IdleScan s = Fresh();
    uint32_t t = 0;
    for (; t < 5000; t += 50) s.update(t);   // get it away from centre
    ASSERT_EQ(s.state(), ScanState::kScanning);

    s.observeVoice(true, t);
    s.update(t);
    EXPECT_EQ(s.state(), ScanState::kListening);
    EXPECT_NEAR(s.pose().yaw_deg, Neutral().yaw_deg, 0.01f)
        << "listening must centre, not guess a bearing it cannot know";

    // And it holds still while somebody is talking.
    const float held = s.pose().yaw_deg;
    for (uint32_t u = t; u < t + 2000; u += 50) {
        s.observeVoice(true, u);
        s.update(u);
        EXPECT_NEAR(s.pose().yaw_deg, held, 0.01f);
    }
}

TEST(IdleScanCycle, TheSweepResumesOnceTheRoomGoesQuiet) {
    IdleScanConfig cfg;
    IdleScan s(cfg, Neutral());
    s.begin(0);
    s.observeVoice(true, 1000);
    s.update(1000);
    ASSERT_EQ(s.state(), ScanState::kListening);

    for (uint32_t t = 1000; t < 1000 + cfg.listen_hold_ms + 500; t += 50) s.update(t);
    EXPECT_EQ(s.state(), ScanState::kScanning);
    EXPECT_EQ(s.stats().listens, 1u);
}

TEST(IdleScanCycle, AFaceOutranksBothScanningAndListening) {
    IdleScan s = Fresh();
    uint32_t t = 0;
    for (; t < 3000; t += 50) s.update(t);

    s.observeVoice(true, t);
    s.update(t);
    ASSERT_EQ(s.state(), ScanState::kListening);

    s.observeFace(true, t);
    s.update(t);
    EXPECT_EQ(s.state(), ScanState::kEngaged);
    EXPECT_FALSE(s.wantsControl())
        << "while a face is tracked the sweep must have no opinion at all";
}

TEST(IdleScanCycle, ABlinkedDetectionDoesNotRestartTheSearch) {
    // The failure this prevents: one dropped frame and the head lurches away
    // from a child who never moved.
    IdleScanConfig cfg;
    IdleScan s(cfg, Neutral());
    s.begin(0);
    uint32_t t = 0;
    s.observeFace(true, t);
    s.update(t);
    ASSERT_EQ(s.state(), ScanState::kEngaged);

    // Detector drops out for well under reacquire_ms, then comes back.
    for (uint32_t u = t; u < t + cfg.reacquire_ms - 200; u += 50) {
        s.observeFace(false, u);
        s.update(u);
        EXPECT_EQ(s.state(), ScanState::kEngaged) << "gave up at " << (u - t) << " ms";
    }
    s.observeFace(true, t + cfg.reacquire_ms - 150);
    s.update(t + cfg.reacquire_ms - 150);
    EXPECT_EQ(s.state(), ScanState::kEngaged);
    EXPECT_EQ(s.stats().losses, 0u);
}

TEST(IdleScanCycle, AFaceThatIsReallyGoneDoesResumeTheSearch) {
    IdleScanConfig cfg;
    IdleScan s(cfg, Neutral());
    s.begin(0);
    s.observeFace(true, 0);
    s.update(0);
    ASSERT_EQ(s.state(), ScanState::kEngaged);

    for (uint32_t t = 0; t < cfg.reacquire_ms + 500; t += 50) {
        s.observeFace(false, t);
        s.update(t);
    }
    EXPECT_EQ(s.state(), ScanState::kScanning);
    EXPECT_EQ(s.stats().losses, 1u);
    EXPECT_TRUE(s.wantsControl());
}

TEST(IdleScanCycle, DisabledMeansNoOpinion) {
    IdleScan s = Fresh();
    s.setEnabled(false, 1000);
    for (uint32_t t = 1000; t < 20000; t += 50) s.update(t);
    EXPECT_EQ(s.state(), ScanState::kOff);
    EXPECT_FALSE(s.wantsControl());
}

TEST(IdleScanCycle, SurvivesTheMillisecondWrap) {
    IdleScanConfig cfg;
    IdleScan s(cfg, Neutral());
    uint32_t t = 0xFFFFF000u;
    s.begin(t);
    uint32_t changes = 0;
    float last = s.pose().yaw_deg;
    for (int i = 0; i < 2000; ++i) {
        s.update(t);
        if (s.pose().yaw_deg != last) { ++changes; last = s.pose().yaw_deg; }
        t += 50;   // wraps through zero
    }
    EXPECT_GT(changes, 3u) << "the sweep stalled across the 49-day wrap";
}

// ---------------------------------------------------------------------------
// Wired into the head controller
// ---------------------------------------------------------------------------

namespace {

struct Rig {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc{vision,   sink,              Limits(), Neutral(),
                      Bounds(), AttentionConfig{}, MotionConfig{},
                      ScheduleConfig{}};
    Rig() { hc.setDiagnosticsEnabled(false); }

    uint32_t boot() {
        uint32_t ms = 0;
        hc.begin(0);
        for (; ms < 30000; ms += 10) hc.update(ms);
        return ms;
    }
};

}  // namespace

TEST(HeadControllerScan, TheHeadLooksAroundWhenTheRoomIsEmpty) {
    Rig r;
    uint32_t ms = r.boot();
    ASSERT_EQ(r.hc.behavior(), Behavior::ATTEND_FACE);

    std::set<int> commanded;
    for (uint32_t t = ms; t < ms + 30000; t += 10) {
        r.hc.update(t);
        if (r.sink.last.valid) commanded.insert(static_cast<int>(std::lround(r.sink.last.yaw_deg)));
    }
    EXPECT_GE(static_cast<int>(commanded.size()), 8)
        << "an empty room should produce a head that looks around, not one that stares";
    EXPECT_GT(r.hc.scanStats().stations, 0u);
}

TEST(HeadControllerScan, NothingScansUntilTheSelfTestAndGreetingAreDone) {
    Rig r;
    r.hc.begin(0);
    for (uint32_t ms = 0; ms < 30000; ms += 10) {
        r.hc.update(ms);
        if (r.hc.selfTestState() != SelfTestState::kPassed || r.hc.greeting()) {
            EXPECT_NE(r.hc.scanState(), ScanState::kScanning)
                << "the sweep started before the safety layer was proven";
        }
    }
    EXPECT_EQ(r.hc.scanState(), ScanState::kScanning);
}

TEST(HeadControllerScan, AFaceStopsTheSweepAndTheTrackerTakesOver) {
    Rig r;
    uint32_t ms = r.boot();
    for (uint32_t t = ms; t < ms + 6000; t += 10) r.hc.update(t);
    ASSERT_EQ(r.hc.scanState(), ScanState::kScanning);
    ms += 6000;

    // Somebody appears, off to one side.
    for (uint32_t t = ms; t < ms + 4000; t += 10) {
        r.vision.see(0.5f, 0.0f, t);
        r.hc.update(t);
    }
    EXPECT_EQ(r.hc.scanState(), ScanState::kEngaged);
    // And the head is pointing at them, not at a scan station.
    EXPECT_GT(r.sink.last.yaw_deg, 2.0f);
}

TEST(HeadControllerScan, LookCenterAndSleepStillBeatTheSweep) {
    // The sweep is the lowest-priority opinion about the head. A behaviour
    // that offers a pose must win, or "hold still" stops meaning anything.
    Rig r;
    uint32_t ms = r.boot();
    r.hc.setBehavior(Behavior::LOOK_CENTER, ms);
    // The sweep has had 30 s to wander, so allow the bounded approach to
    // bring the head back before asserting it stays put.
    for (uint32_t t = ms; t < ms + 2000; t += 10) r.hc.update(t);
    ms += 2000;
    for (uint32_t t = ms; t < ms + 20000; t += 10) {
        r.hc.update(t);
        EXPECT_NEAR(r.sink.last.yaw_deg, Neutral().yaw_deg, 2.0f)
            << "the sweep moved the head while LOOK_CENTER was active";
    }
}

TEST(HeadControllerScan, EverythingTheSweepCommandsIsInsideTheEnvelope) {
    Rig r;
    uint32_t ms = r.boot();
    const ServoLimits lim = Limits();
    for (uint32_t t = ms; t < ms + 60000; t += 10) {
        if ((t / 7000) % 2 == 0) r.hc.observeVoice(true, t);   // chatter
        r.hc.update(t);
        if (!r.sink.last.valid) continue;
        EXPECT_GE(r.sink.last.yaw_deg, lim.min_yaw_deg);
        EXPECT_LE(r.sink.last.yaw_deg, lim.max_yaw_deg);
        EXPECT_GE(r.sink.last.pitch_deg, lim.min_pitch_deg);
        EXPECT_LE(r.sink.last.pitch_deg, lim.max_pitch_deg);
    }
    EXPECT_GT(r.hc.scanStats().listens, 0u) << "the voice path was never exercised";
}
