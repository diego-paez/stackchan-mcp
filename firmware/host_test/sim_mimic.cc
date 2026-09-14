// sim_mimic.cc — watch the robot's face answer a child's, without a child.
//
// A simulation, not a test: nothing is asserted. The response table in
// face_mimic.h is a pedagogical judgement, and the tests can only prove it is
// applied consistently. Whether it reads as empathy rather than as mockery is
// something you have to look at, so this prints the timeline of a short
// conversation — what was heard, what the robot concluded, and what its face
// actually did about it.
//
// Build:  cmake --build <dir> --target sim_mimic && ./sim_mimic
#include <cstdio>
#include <string>
#include <vector>

#include "attention/face_mimic.h"
#include "attention/head_controller.h"
#include "attention/servo_sink.h"
#include "attention/vision_tracker.h"

using namespace stackchan::attention;

namespace {

void rule(const char* title) {
    std::printf("\n\033[1m%s\033[0m\n", title);
    std::printf("──────────────────────────────────────────────────────────────────────\n");
}

// One thing the child said, and what the emotion models made of it. The
// confidences are the interesting part: a real utterance rarely comes back
// at 0.9, and a failed model comes back at 1/7 on everything.
struct Utterance {
    uint32_t at_ms;
    const char* said;
    const char* label;
    float confidence;
};

const char* FaceGlyph(AvatarFace f) {
    switch (f) {
        case AvatarFace::kIdle: return "( - _ - )";
        case AvatarFace::kHappy: return "( ^ _ ^ )";
        case AvatarFace::kThinking: return "( o _ - )";
        case AvatarFace::kSad: return "( ; _ ; )";
        case AvatarFace::kSurprised: return "( O _ O )";
        case AvatarFace::kEmbarrassed: return "( > _ < )";
    }
    return "( ? _ ? )";
}

}  // namespace

int main() {
    std::printf("\n\033[1mstacky's face, answering a child's\033[0m\n");
    std::printf("The head is following her the whole time; this is only the screen.\n");

    rule("the response table");
    const MimicPolicy policy;
    const Affect all[] = {Affect::kHappy,   Affect::kSurprise, Affect::kSad,
                          Affect::kAnger,   Affect::kFear,     Affect::kDisgust,
                          Affect::kNeutral};
    for (Affect a : all) {
        const AvatarFace f = ResponseTo(a, policy);
        const bool mirror = std::string(ToString(a)) == ToString(f) ||
                            (a == Affect::kSurprise && f == AvatarFace::kSurprised);
        std::printf("  she is %-9s  →  %-12s %s   %s\n", ToString(a), ToString(f),
                    FaceGlyph(f), mirror ? "mirror" : "answer");
    }

    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController head(vision, sink, ServoLimits{}, NeutralPose{}, HardwareBounds{},
                        AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    head.setDiagnosticsEnabled(false);

    std::vector<std::string> shown;
    head.setExpressionSink([&](const char* e) { shown.push_back(e); });
    head.begin(0);

    // Boot: self-test then greeting. Nothing the child does reaches the face
    // until both are done, which is roughly the first 16 seconds.
    uint32_t ms = 0;
    for (; ms < 17000; ms += 10) head.update(ms);
    const size_t after_boot = shown.size();

    rule("boot");
    std::printf("  self-test %s, greeting played %zu expressions, ending on \"%s\"\n",
                head.selfTestState() == SelfTestState::kPassed ? "passed" : "FAILED",
                after_boot, shown.empty() ? "-" : shown.back().c_str());
    std::printf("  the face is now the mimic's to write\n");

    // A short conversation. She arrives cheerful, is startled, gets upset,
    // is comforted, and at the end the emotion model falls over and returns
    // a uniform distribution — which must move nothing.
    const Utterance script[] = {
        {18000, "Hello Stacky!",                  "happy",    0.78f},
        {19500, "I made you a drawing",           "happy",    0.71f},
        {23000, "oh! what was that noise",        "surprise", 0.88f},
        {26000, "my tower fell over",             "sad",      0.66f},
        {27500, "it took me ALL morning",         "anger",    0.74f},
        {29000, "and nobody even helped me",      "anger",    0.69f},
        {34000, "...",                            "neutral",  0.55f},
        {38000, "ok. can we build it again",      "neutral",  0.61f},
        {41000, "yes! this bit goes here",        "happy",    0.83f},
        {45000, "(model failed)",                 "sad",      0.1428f},
        {46000, "(model failed)",                 "happy",    0.1428f},
    };

    rule("the conversation");
    std::printf("  %7s  %-28s %-9s %5s   %-12s %s\n",
                "time", "she says", "heard as", "conf", "stacky", "");

    size_t next = 0;
    size_t reported = shown.size();
    for (; ms < 56000; ms += 10) {
        if (next < sizeof(script) / sizeof(script[0]) && ms >= script[next].at_ms) {
            const Utterance& u = script[next++];
            head.observeAffect(u.label, u.confidence, ms);
            head.update(ms);
            const bool changed = shown.size() > reported;
            std::printf("  %6.2fs  %-28s %-9s  %.2f   %-12s %s%s\n", ms / 1000.0f,
                        u.said, u.label, u.confidence,
                        ToString(head.mimicFace()), FaceGlyph(head.mimicFace()),
                        changed ? "  \033[2m← screen written\033[0m" : "");
            reported = shown.size();
            continue;
        }
        head.update(ms);
        if (shown.size() > reported) {
            reported = shown.size();
            std::printf("  %6.2fs  %-28s %-9s  %4s   %-12s %s  \033[2m← screen written\033[0m\n",
                        ms / 1000.0f, "(silence)", "-", "-",
                        ToString(head.mimicFace()), FaceGlyph(head.mimicFace()));
        }
    }

    const MimicStats st = head.mimicStats();
    rule("what the face did");
    std::printf("  observations offered   %u\n", (unsigned)st.observations);
    std::printf("  accepted as evidence   %u\n", (unsigned)st.accepted);
    std::printf("  too weak to count      %u   (a failed model is all of these)\n",
                (unsigned)st.weak);
    std::printf("  label not in vocabulary %u\n", (unsigned)st.unknown);
    std::printf("  times the face changed %u\n", (unsigned)st.changes);
    std::printf("  changes held back      %u   (by the %u ms floor)\n",
                (unsigned)st.held, (unsigned)MimicConfig{}.min_hold_ms);
    std::printf("  returns to idle        %u   (nobody said anything for %u ms)\n",
                (unsigned)st.decays, (unsigned)MimicConfig{}.stale_ms);
    std::printf("\n  Note the two rows the table does not mirror: she is angry twice\n");
    std::printf("  and the robot looks thoughtful, not angry back.\n\n");
    return 0;
}
