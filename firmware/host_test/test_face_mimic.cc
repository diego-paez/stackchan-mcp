// Host tests for the face-mimic layer.
//
// What these can prove: that the response table is applied exactly as
// written, that the face cannot flicker, that it cannot get stuck wearing an
// expression nobody caused, that a failed emotion model cannot move it at
// all, and that the greeting and an emergency stop take precedence.
//
// What they cannot prove: that the table is the right table. Whether a robot
// should answer a frightened child with a sad face is a pedagogical question
// and no assertion here settles it. See face_mimic.h.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "attention/face_mimic.h"
#include "attention/head_controller.h"
#include "attention/servo_sink.h"
#include "attention/vision_tracker.h"

using namespace stackchan::attention;

namespace {

ServoLimits Limits() { return ServoLimits{}; }
NeutralPose Neutral() { return NeutralPose{}; }
HardwareBounds Bounds() { return HardwareBounds{}; }

// A mimic with the defaults, and a clock the test drives by hand.
FaceMimic Default() { return FaceMimic(MimicConfig{}, MimicPolicy{}); }

// Feed one observation and let the policy settle, without letting min_hold_ms
// silently swallow the change the test is looking for.
void Feed(FaceMimic& m, const char* label, float conf, uint32_t& t,
          int times = 2, uint32_t step_ms = 400) {
    for (int i = 0; i < times; ++i) {
        m.observe(label, conf, t);
        m.update(t);
        t += step_ms;
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// The vocabulary
//
// Labels arrive as strings, over a network, from whichever model is loaded
// this week. Matching them by name is the whole defence against a model being
// swapped and the robot's feelings being silently renumbered.
// ---------------------------------------------------------------------------

TEST(MimicVocabulary, KnowsTheSevenLabelsThePipelineEmits) {
    // chatbot/models/emotion/base.py: EMOTION_LABELS.
    EXPECT_EQ(AffectFromLabel("anger"), Affect::kAnger);
    EXPECT_EQ(AffectFromLabel("disgust"), Affect::kDisgust);
    EXPECT_EQ(AffectFromLabel("fear"), Affect::kFear);
    EXPECT_EQ(AffectFromLabel("happy"), Affect::kHappy);
    EXPECT_EQ(AffectFromLabel("neutral"), Affect::kNeutral);
    EXPECT_EQ(AffectFromLabel("sad"), Affect::kSad);
    EXPECT_EQ(AffectFromLabel("surprise"), Affect::kSurprise);
}

TEST(MimicVocabulary, ToleratesTheSpellingsTheModelsActuallyUse) {
    EXPECT_EQ(AffectFromLabel("angry"), Affect::kAnger);
    EXPECT_EQ(AffectFromLabel("joy"), Affect::kHappy);
    EXPECT_EQ(AffectFromLabel("sadness"), Affect::kSad);
    EXPECT_EQ(AffectFromLabel("surprised"), Affect::kSurprise);
    EXPECT_EQ(AffectFromLabel("fearful"), Affect::kFear);
    EXPECT_EQ(AffectFromLabel("HAPPY"), Affect::kHappy);
    EXPECT_EQ(AffectFromLabel("Neutral"), Affect::kNeutral);
}

TEST(MimicVocabulary, AnUnrecognisedLabelIsUnknownAndNotNeutral) {
    // The distinction matters: "I did not understand that" must not put a
    // resting face on a robot that is being shouted at.
    EXPECT_EQ(AffectFromLabel("excited"), Affect::kUnknown);
    EXPECT_EQ(AffectFromLabel(""), Affect::kUnknown);
    EXPECT_EQ(AffectFromLabel(nullptr), Affect::kUnknown);
    EXPECT_NE(AffectFromLabel("excited"), Affect::kNeutral);
}

TEST(MimicVocabulary, EveryFaceNameRoundTripsThroughTheAvatarsOwnTable) {
    // These six strings are the ones StackChanBoard::FaceNameToIndex() knows.
    // A seventh would render nothing at all.
    const char* names[] = {"idle", "happy", "thinking", "sad", "surprised",
                           "embarrassed"};
    for (const char* n : names) {
        AvatarFace f{};
        ASSERT_TRUE(FaceFromName(n, &f)) << n;
        EXPECT_STREQ(ToString(f), n);
    }
    EXPECT_FALSE(FaceFromName("neutral", nullptr))
        << "the avatar has no face called neutral; its resting face is idle";
    EXPECT_FALSE(FaceFromName("angry", nullptr));
    EXPECT_FALSE(FaceFromName(nullptr, nullptr));
}

// ---------------------------------------------------------------------------
// The response table
// ---------------------------------------------------------------------------

TEST(MimicPolicyTable, MirrorsWhatIsSafeToMirror) {
    const MimicPolicy p;
    EXPECT_EQ(ResponseTo(Affect::kHappy, p), AvatarFace::kHappy);
    EXPECT_EQ(ResponseTo(Affect::kSurprise, p), AvatarFace::kSurprised);
    EXPECT_EQ(ResponseTo(Affect::kSad, p), AvatarFace::kSad);
    EXPECT_EQ(ResponseTo(Affect::kNeutral, p), AvatarFace::kIdle);
}

TEST(MimicPolicyTable, AnswersRatherThanEscalates) {
    const MimicPolicy p;
    // Anger is met with attention, not with more anger. The avatar has no
    // angry face at all, which is the point: there is nothing to escalate to.
    EXPECT_EQ(ResponseTo(Affect::kAnger, p), AvatarFace::kThinking);
    // Fear is met with concern, not with alarm.
    EXPECT_EQ(ResponseTo(Affect::kFear, p), AvatarFace::kSad);
    EXPECT_EQ(ResponseTo(Affect::kDisgust, p), AvatarFace::kEmbarrassed);
}

TEST(MimicPolicyTable, UnknownRestsRatherThanGuesses) {
    EXPECT_EQ(ResponseTo(Affect::kUnknown, MimicPolicy{}), AvatarFace::kIdle);
}

TEST(MimicPolicyTable, IsATableAndCanBeChangedInOnePlace) {
    MimicPolicy p;
    p.on_anger = AvatarFace::kSad;
    EXPECT_EQ(ResponseTo(Affect::kAnger, p), AvatarFace::kSad);
    EXPECT_EQ(ResponseTo(Affect::kHappy, p), AvatarFace::kHappy)
        << "changing one row must not disturb the others";
}

// ---------------------------------------------------------------------------
// Not moving — which, as with the head, is most of the job
// ---------------------------------------------------------------------------

TEST(FaceMimicStillness, AFailedEmotionModelCannotMoveTheFace) {
    // The pipeline returns a uniform distribution when a model fails
    // (get_default_emotion_scores). 1/7 = 0.143 on every label, forever.
    FaceMimic m = Default();
    uint32_t t = 0;
    for (int i = 0; i < 200; ++i) {
        m.observe("sad", 1.0f / 7.0f, t);
        EXPECT_FALSE(m.update(t));
        t += 100;
    }
    EXPECT_EQ(m.face(), AvatarFace::kIdle);
    EXPECT_EQ(m.stats().changes, 0u);
    EXPECT_EQ(m.stats().weak, 200u);
    EXPECT_EQ(m.stats().accepted, 0u);
}

TEST(FaceMimicStillness, NaNConfidenceIsRejectedRatherThanRankedHigh) {
    FaceMimic m = Default();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (uint32_t t = 0; t < 3000; t += 100) {
        m.observe("happy", nan, t);
        EXPECT_FALSE(m.update(t));
    }
    EXPECT_EQ(m.face(), AvatarFace::kIdle);
    EXPECT_EQ(m.stats().weak, 30u);
}

TEST(FaceMimicStillness, OneMiddlingObservationIsNotEnoughOnItsOwn) {
    // 0.52 is above "this is not evidence" and below "this stands alone":
    // the band the confirmation rules exist for.
    FaceMimic m = Default();
    m.observe("happy", 0.52f, 0);
    EXPECT_FALSE(m.update(0));
    EXPECT_EQ(m.face(), AvatarFace::kIdle)
        << "a single middling reading must not move the face immediately";
    EXPECT_EQ(m.stats().accepted, 1u);
}

TEST(FaceMimicStillness, AnUnknownLabelDoesNotKnockTheFaceBackToIdle) {
    FaceMimic m = Default();
    uint32_t t = 0;
    Feed(m, "happy", 0.7f, t);
    ASSERT_EQ(m.face(), AvatarFace::kHappy);

    // A label from a model we do not know is no evidence at all — it must
    // neither change the face nor refresh the staleness clock.
    for (int i = 0; i < 5; ++i) {
        m.observe("excited", 0.99f, t);
        EXPECT_FALSE(m.update(t));
        t += 100;
    }
    EXPECT_EQ(m.face(), AvatarFace::kHappy);
    EXPECT_EQ(m.stats().unknown, 5u);
}

TEST(FaceMimicStillness, AlternatingStrongReadingsCannotMakeItFlicker) {
    // The pathological input: a model that swings between two confident
    // answers ten times a second. min_hold_ms is the floor that survives it.
    MimicConfig cfg;
    cfg.min_hold_ms = 1200;
    FaceMimic m(cfg, MimicPolicy{});

    for (uint32_t t = 0; t < 12000; t += 100) {
        m.observe((t / 100) % 2 == 0 ? "happy" : "sad", 0.95f, t);
        m.update(t);
    }
    // 12 s of maximal provocation at a 1.2 s floor: ten changes is the
    // ceiling, and the assertion is on the bound, not on the exact count.
    EXPECT_LE(m.stats().changes, 10u) << "the floor on change rate was not held";
    EXPECT_GT(m.stats().held, 0u) << "nothing was ever actually suppressed";
}

// ---------------------------------------------------------------------------
// Moving, when it should
// ---------------------------------------------------------------------------

TEST(FaceMimicResponse, TwoAgreeingReadingsChangeTheFace) {
    FaceMimic m = Default();
    uint32_t t = 0;
    m.observe("happy", 0.52f, t);
    EXPECT_FALSE(m.update(t));
    t += 400;
    m.observe("happy", 0.52f, t);
    EXPECT_TRUE(m.update(t)) << "a confirmed reading must reach the display";
    EXPECT_EQ(m.face(), AvatarFace::kHappy);
    EXPECT_EQ(m.affect(), Affect::kHappy);
    EXPECT_STREQ(m.faceName(), "happy");
}

TEST(FaceMimicResponse, AConfidentReadingLandsOnTheFirstOne) {
    // Surprise that takes two turns to arrive is not surprise. 0.78 is what
    // a clear utterance actually scores; the bar has to be reachable by one.
    FaceMimic m = Default();
    m.observe("surprise", 0.78f, 0);
    EXPECT_TRUE(m.update(0));
    EXPECT_EQ(m.face(), AvatarFace::kSurprised);
}

TEST(FaceMimicResponse, AMiddlingReadingNothingContradictsLandsOnDwell) {
    // One utterance, scored 0.52, and then silence. A child who says one sad
    // thing and stops talking is still sad, and the face has to arrive.
    FaceMimic m = Default();
    uint32_t t = 0;
    m.observe("sad", 0.52f, t);
    EXPECT_FALSE(m.update(t));

    bool arrived = false;
    for (; t < 3000; t += 100) {
        if (m.update(t)) arrived = true;
    }
    EXPECT_TRUE(arrived);
    EXPECT_EQ(m.face(), AvatarFace::kSad);
    EXPECT_EQ(m.stats().accepted, 1u) << "it took one reading, as it should";
}

TEST(FaceMimicResponse, ContradictedReadingsNeverConfirmByDwell) {
    // Dwell must not become a way for noise to get through: a reading that
    // keeps being contradicted resets the clock and confirms by no route.
    MimicConfig cfg;
    cfg.instant_confidence = 0.95f;   // shut the confidence route
    FaceMimic m(cfg, MimicPolicy{});
    for (uint32_t t = 0; t < 20000; t += 800) {
        m.observe(t % 1600 == 0 ? "happy" : "sad", 0.6f, t);
        EXPECT_FALSE(m.update(t));
    }
    EXPECT_EQ(m.face(), AvatarFace::kIdle);
    EXPECT_EQ(m.stats().changes, 0u);
    EXPECT_GT(m.stats().accepted, 10u) << "the readings were being accepted";
}

TEST(FaceMimicResponse, TwoAffectsThatShareAFaceProduceNoSecondChange) {
    // fear and sad both answer with sad. The reading changes; the screen
    // does not, and must not be told again.
    FaceMimic m = Default();
    uint32_t t = 0;
    Feed(m, "fear", 0.7f, t);
    ASSERT_EQ(m.face(), AvatarFace::kSad);
    const uint32_t changes = m.stats().changes;

    Feed(m, "sad", 0.7f, t);
    EXPECT_EQ(m.affect(), Affect::kSad) << "the reading itself did change";
    EXPECT_EQ(m.face(), AvatarFace::kSad);
    EXPECT_EQ(m.stats().changes, changes) << "the display was told twice";
}

TEST(FaceMimicResponse, ReturnsToIdleWhenNobodyHasSaidAnythingInAWhile) {
    MimicConfig cfg;
    cfg.stale_ms = 6000;
    FaceMimic m(cfg, MimicPolicy{});
    uint32_t t = 0;
    Feed(m, "happy", 0.8f, t);
    ASSERT_EQ(m.face(), AvatarFace::kHappy);

    bool decayed = false;
    const uint32_t started = t;
    for (; t < started + 10000; t += 100) {
        if (m.update(t)) decayed = true;
    }
    EXPECT_TRUE(decayed);
    EXPECT_EQ(m.face(), AvatarFace::kIdle)
        << "a robot still wearing the last thing it was told is stuck, not expressive";
    EXPECT_EQ(m.stats().decays, 1u);
}

TEST(FaceMimicResponse, StalenessSurvivesTheMillisecondWrap) {
    // uint32 milliseconds wrap every 49 days. A robot in a classroom will
    // reach that, and the failure would be a face frozen forever.
    FaceMimic m = Default();
    uint32_t t = 0xFFFFFF00u;
    Feed(m, "happy", 0.8f, t);   // straddles the wrap
    ASSERT_EQ(m.face(), AvatarFace::kHappy);

    for (int i = 0; i < 100; ++i) {
        m.update(t);
        t += 100;                 // wraps through zero
    }
    EXPECT_EQ(m.face(), AvatarFace::kIdle);
}

TEST(FaceMimicResponse, ResetGoesToIdleAndForgetsTheReading) {
    FaceMimic m = Default();
    uint32_t t = 0;
    Feed(m, "happy", 0.9f, t);
    ASSERT_EQ(m.face(), AvatarFace::kHappy);

    m.reset(t);
    EXPECT_EQ(m.face(), AvatarFace::kIdle);
    EXPECT_EQ(m.affect(), Affect::kUnknown);
    EXPECT_TRUE(m.update(t)) << "the display must be told it went to idle";
}

TEST(FaceMimicResponse, DisabledMeansResting) {
    MimicConfig cfg;
    cfg.enabled = false;
    FaceMimic m(cfg, MimicPolicy{});
    uint32_t t = 0;
    for (int i = 0; i < 10; ++i) {
        m.observe("happy", 0.99f, t);
        m.update(t);
        t += 400;
    }
    EXPECT_EQ(m.face(), AvatarFace::kIdle);
    EXPECT_EQ(m.stats().changes, 0u);
}

TEST(FaceMimicResponse, SuppressedKeepsThinkingButSaysNothing) {
    FaceMimic m = Default();
    m.setSuppressed(true);
    uint32_t t = 0;
    Feed(m, "happy", 0.9f, t);
    EXPECT_EQ(m.face(), AvatarFace::kHappy) << "the policy kept running";

    // Nothing was reported while something else owned the screen...
    m.setSuppressed(false);
    EXPECT_TRUE(m.update(t)) << "...and the answer is delivered once when it is free";
    EXPECT_FALSE(m.update(t + 10)) << "but only once";
}

// ---------------------------------------------------------------------------
// Wired into the head controller
//
// The face and the head share a startup order and an emergency stop, and the
// rule is the same one the greeting follows: expressive behaviour waits until
// the safety layer has been shown to work, and stops when it says stop.
// ---------------------------------------------------------------------------

namespace {

// Runs a controller from boot with an expression recorder attached.
struct Rig {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController hc{vision,      sink,          Limits(), Neutral(),
                      Bounds(),    AttentionConfig{}, MotionConfig{},
                      ScheduleConfig{}};
    std::vector<std::string> shown;

    Rig() {
        hc.setDiagnosticsEnabled(false);
        hc.setExpressionSink([this](const char* e) { shown.push_back(e); });
    }

    // Boot takes ~10.8 s of self-test plus ~5 s of greeting.
    uint32_t runUntilTracking(uint32_t step = 10) {
        uint32_t ms = 0;
        hc.begin(0);
        for (; ms < 45000; ms += step) hc.update(ms);
        return ms;
    }
};

}  // namespace

TEST(HeadControllerMimic, EveryExpressionTheRobotShowsIsOneTheAvatarCanRender) {
    // The test that would have caught "neutral": walk a whole boot and check
    // every string that leaves for the display against the avatar's own table.
    Rig r;
    uint32_t ms = r.runUntilTracking();
    r.hc.observeAffect("happy", 0.9f, ms);
    for (uint32_t t = ms; t < ms + 3000; t += 10) r.hc.update(t);

    ASSERT_FALSE(r.shown.empty());
    for (const std::string& e : r.shown) {
        EXPECT_TRUE(FaceFromName(e.c_str(), nullptr))
            << '"' << e << "\" is not a face the avatar can show";
    }
}

TEST(HeadControllerMimic, TheGreetingOwnsTheFaceWhileItPlays) {
    // Suppression is about the screen, not about the policy: the mimic keeps
    // reading the room while the greeting plays, so that it is up to date the
    // moment it is free. What must not happen is a second writer to the
    // display. Anger answers with "thinking", which the greeting script never
    // shows, so its appearance mid-greeting would be unambiguous.
    Rig r;
    r.hc.begin(0);
    bool observed_during_greeting = false;
    for (uint32_t ms = 0; ms < 45000; ms += 10) {
        r.hc.update(ms);
        if (r.hc.greeting()) {
            r.hc.observeAffect("anger", 0.99f, ms);   // shout at it mid-bow
            observed_during_greeting = true;
            for (const std::string& e : r.shown) {
                EXPECT_NE(e, "thinking") << "the mimic wrote to the display "
                                            "while the greeting owned it";
            }
        }
    }
    EXPECT_TRUE(observed_during_greeting);
    EXPECT_TRUE(r.hc.greetingFinished());
    // And it was listening all along, so it is current when it takes over.
    EXPECT_GT(r.hc.mimicStats().accepted, 0u);
}

TEST(HeadControllerMimic, NothingIsExpressedBeforeTheSelfTestPasses) {
    // Expressive behaviour waits for the safety layer to be shown to work on
    // this particular unit — the same rule the greeting follows, applied to
    // the screen as well as to the servos.
    Rig r;
    r.hc.begin(0);
    for (uint32_t ms = 0; ms < 22000; ms += 10) {
        r.hc.observeAffect("happy", 0.99f, ms);
        r.hc.update(ms);
        if (r.hc.selfTestState() != SelfTestState::kPassed) {
            EXPECT_TRUE(r.shown.empty())
                << "expression reached the display before the self-test passed";
        }
    }
    EXPECT_EQ(r.hc.selfTestState(), SelfTestState::kPassed);
}

TEST(HeadControllerMimic, AnObservationReachesTheDisplayOnceTrackingHasStarted) {
    Rig r;
    uint32_t ms = r.runUntilTracking();
    ASSERT_EQ(r.hc.behavior(), Behavior::ATTEND_FACE);
    const size_t before = r.shown.size();

    for (uint32_t t = ms; t < ms + 2000; t += 10) {
        if ((t - ms) % 300 == 0) r.hc.observeAffect("happy", 0.7f, t);
        r.hc.update(t);
    }
    ASSERT_GT(r.shown.size(), before);
    EXPECT_EQ(r.shown.back(), "happy");
    EXPECT_EQ(r.hc.mimicFace(), AvatarFace::kHappy);
}

TEST(HeadControllerMimic, EmergencyStopReturnsTheFaceToIdleAndKeepsItThere) {
    Rig r;
    uint32_t ms = r.runUntilTracking();
    for (uint32_t t = ms; t < ms + 2000; t += 10) {
        if ((t - ms) % 300 == 0) r.hc.observeAffect("happy", 0.9f, t);
        r.hc.update(t);
    }
    ms += 2000;
    ASSERT_EQ(r.hc.mimicFace(), AvatarFace::kHappy);

    r.hc.emergencyStop();
    const size_t after_stop = r.shown.size();
    for (uint32_t t = ms; t < ms + 5000; t += 10) {
        r.hc.observeAffect("happy", 0.99f, t);   // keep provoking it
        r.hc.update(t);
    }
    EXPECT_EQ(r.hc.mimicFace(), AvatarFace::kIdle)
        << "a stopped robot must not keep emoting";
    for (size_t i = after_stop; i < r.shown.size(); ++i) {
        EXPECT_EQ(r.shown[i], "idle") << "expression fired after an emergency stop";
    }
}

TEST(HeadControllerMimic, TheFaceDoesNotDisturbTheHead) {
    // The two are independent on purpose: an expression must never become a
    // servo command. Provoke the face continuously while a child sits still
    // in the dead zone and assert the head stays put.
    Rig r;
    uint32_t ms = r.runUntilTracking();
    r.vision.see(0.0f, 0.0f, ms);
    // The approach is deliberately unhurried (MotionConfig::approach_per_sec
    // is 0.8), so give it long enough to actually arrive before asserting it
    // then stays put; otherwise this measures convergence, not stability.
    for (uint32_t t = ms; t < ms + 5000; t += 10) {
        r.vision.see(0.0f, 0.0f, t);
        r.hc.update(t);
    }
    ms += 4000;

    const ServoCommand settled = r.sink.last;
    ASSERT_TRUE(settled.valid);

    for (uint32_t t = ms + 1000; t < ms + 12000; t += 10) {
        r.vision.see(0.0f, 0.0f, t);
        if (t % 200 == 0) {
            r.hc.observeAffect((t / 200) % 2 ? "happy" : "surprise", 0.99f, t);
        }
        r.hc.update(t);
        EXPECT_NEAR(r.sink.last.yaw_deg, settled.yaw_deg, 0.5f);
        EXPECT_NEAR(r.sink.last.pitch_deg, settled.pitch_deg, 0.5f);
    }
    EXPECT_GT(r.hc.mimicStats().changes, 0u) << "the face never moved, so this proved nothing";
}
