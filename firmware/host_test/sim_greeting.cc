// sim_greeting.cc — watch the robot wake up, introduce itself, and start
// following a child's face, without a robot.
//
// This is a simulation, not a test: nothing is asserted. It exists because a
// timeline of what the head *does* is the only way to judge whether a
// greeting reads as a greeting, and the tests can only tell you it stayed
// inside its limits while doing it.
//
// Build:  cmake --build <dir> --target sim_greeting && ./sim_greeting
#include <cstdio>
#include <cmath>
#include <string>

#include "attention/head_controller.h"
#include "attention/servo_sink.h"
#include "attention/vision_tracker.h"

using namespace stackchan::attention;

namespace {

ServoLimits Limits() { return ServoLimits{}; }
NeutralPose Neutral() { return NeutralPose{}; }
HardwareBounds Bounds() { return HardwareBounds{}; }

void rule(const char* title) {
    std::printf("\n\033[1m%s\033[0m\n", title);
    std::printf("────────────────────────────────────────────────────────────────────\n");
}

// A head position drawn as a bar, so a sweep is visible as movement rather
// than as a column of numbers that all look alike.
std::string gauge(float value, float lo, float hi, int width = 21) {
    if (hi <= lo) return std::string(width, '-');
    float t = (value - lo) / (hi - lo);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const int pos = static_cast<int>(t * (width - 1) + 0.5f);
    std::string s(width, '.');   // ASCII: a char literal must be one byte
    s[width / 2] = '|';          // neutral
    s[pos] = '#';
    return s;
}

}  // namespace

int main() {
    ScriptedVisionTracker vision;
    RecordingServoSink sink;
    HeadController head(vision, sink, Limits(), Neutral(), Bounds(),
                        AttentionConfig{}, MotionConfig{}, ScheduleConfig{});
    head.setDiagnosticsEnabled(false);

    std::string spoken;
    std::string face = "(none)";
    head.setGreetingSpeechSink([&](const char* s) { spoken = s; });
    head.setGreetingExpressionSink([&](const char* e) { face = e; });

    const ServoLimits lim = Limits();

    std::printf("\n\033[1mStack-chan · body simulation\033[0m\n");
    std::printf("neutral yaw %.0f° pitch %.0f°   ·   envelope yaw %.0f..%.0f°, pitch %.0f..%.0f°\n",
                Neutral().yaw_deg, Neutral().pitch_deg,
                lim.min_yaw_deg, lim.max_yaw_deg, lim.min_pitch_deg, lim.max_pitch_deg);

    rule("1 · boot — safety self-test (tracking disabled)");
    head.begin(0);

    uint32_t ms = 0;
    int last_reported = -1;
    auto report = [&](const char* stage) {
        std::printf("  %6.2fs  %-11s  yaw %+6.1f° %s  pitch %5.1f°  face %-10s\n",
                    ms / 1000.0f, stage, sink.last.yaw_deg,
                    gauge(sink.last.yaw_deg, lim.min_yaw_deg, lim.max_yaw_deg).c_str(),
                    sink.last.pitch_deg, face.c_str());
    };

    // --- self-test ---------------------------------------------------------
    for (; ms < 12000 && head.selfTestState() != SelfTestState::kPassed; ms += 10) {
        head.update(ms);
        const int sec = static_cast<int>(ms / 1200);
        if (sec != last_reported && sink.last.valid) {
            last_reported = sec;
            report("self-test");
        }
    }
    std::printf("  → self-test %s\n", ToString(head.selfTestState()));

    // --- greeting ----------------------------------------------------------
    rule("2 · greeting — \"Greetings, I am Stacky\" (tracking still disabled)");
    int beat = -1;
    for (; ms < 25000 && !head.greetingFinished(); ms += 10) {
        head.update(ms);
        // Report on each change of expression or on the spoken line: those are
        // the beats a person would actually notice.
        static std::string prev_face, prev_spoken;
        if (face != prev_face || spoken != prev_spoken) {
            prev_face = face;
            if (spoken != prev_spoken) {
                prev_spoken = spoken;
                std::printf("  %6.2fs  %-11s  \033[1m\"%s\"\033[0m\n", ms / 1000.0f, "SPEAKS", spoken.c_str());
            }
            ++beat;
            report("greeting");
        }
    }
    std::printf("  → greeting finished, behaviour is now %s\n", ToString(head.behavior()));

    // --- tracking ----------------------------------------------------------
    //
    // The camera is bolted to the head, so where a face lands in the image
    // depends on where the head is pointing — that is what closes the loop and
    // it has to be modelled or the simulation lies. Feeding a fixed image
    // position regardless of head angle makes the controller look broken: the
    // error never falls, the target winds to the limit, and the head appears
    // to stick there. It is the simulation that is wrong in that case, not the
    // controller.
    //
    // The child is therefore placed at a real-world bearing and the image
    // position is derived each tick.
    rule("3 · a child sits down in front of it and moves about");

    // Half-angle the camera sees. Nothing in the firmware declares an FOV, so
    // this is an assumption: ~60° horizontal is typical of the CoreS3's
    // sensor. It sets how many degrees of head movement one unit of image
    // error is worth, so a wrong value changes the convergence rate here and
    // nothing about the robot.
    constexpr float kHalfFovX = 30.0f;
    constexpr float kHalfFovY = 22.0f;

    struct Move { const char* what; float bearing_deg; float elev_deg; uint32_t hold_ms; bool visible; };
    const Move moves[] = {
        { "sits down, centre",     2.0f,   4.0f, 2500, true  },
        { "leans to her left",   -18.0f,   2.0f, 3500, true  },
        { "sits back, centre",     0.0f,   0.0f, 2500, true  },
        { "leans right",          22.0f,  -4.0f, 3500, true  },
        { "stands up (taller)",    6.0f,  16.0f, 3000, true  },
        { "walks out of frame",    0.0f,   0.0f, 3500, false },
        { "comes back",          -10.0f,   6.0f, 3500, true  },
    };

    for (const Move& m : moves) {
        std::printf("  \033[2m%6.2fs  child: %s", ms / 1000.0f, m.what);
        if (m.visible) std::printf("  (bearing %+.0f°, elevation %+.0f°)", m.bearing_deg, m.elev_deg);
        std::printf("\033[0m\n");

        const uint32_t until = ms + m.hold_ms;
        uint32_t next_print = ms;
        for (; ms < until; ms += 10) {
            if (m.visible) {
                // Where the head is now decides where the face falls in frame.
                const HeadPose at = sink.readPose();
                // +y is DOWN in the image, which is why the elevation term is
                // negated: a child above where the head is aimed appears in the
                // upper part of the frame, and up is negative y. Getting this
                // backwards turns the pitch loop into positive feedback — the
                // first run of this simulation drove pitch to its lower limit
                // and pinned it there, which looked exactly like a controller
                // bug and was a bug in this file.
                float ix = (m.bearing_deg - at.yaw_deg) / kHalfFovX;
                float iy = -(m.elev_deg - (at.pitch_deg - Neutral().pitch_deg)) / kHalfFovY;
                const bool in_frame = std::fabs(ix) <= 1.0f && std::fabs(iy) <= 1.0f;
                if (in_frame) {
                    vision.see(ix, iy, ms);
                } else {
                    vision.lose();   // turned so far the child left the frame
                }
            } else {
                vision.lose();
            }
            head.update(ms);

            if (ms >= next_print && sink.last.valid) {
                next_print = ms + 700;
                const Diagnostics d = head.diagnostics();
                const float err = m.visible ? (m.bearing_deg - sink.last.yaw_deg) : 0.0f;
                std::printf("  %6.2fs  %-11s  yaw %+6.1f° %s  pitch %5.1f°",
                            ms / 1000.0f, ToString(d.tracking), sink.last.yaw_deg,
                            gauge(sink.last.yaw_deg, lim.min_yaw_deg, lim.max_yaw_deg).c_str(),
                            sink.last.pitch_deg);
                if (m.visible) std::printf("  off by %+5.1f°", err);
                std::printf("\n");
            }
        }
    }

    const Diagnostics d = head.diagnostics();
    rule("summary");
    std::printf("  servo writes        %d\n", sink.writes);
    std::printf("  rejected by safety  %u\n", (unsigned)d.safety_rejected);
    std::printf("  clamped by safety   %u\n", (unsigned)d.safety_clamped);
    std::printf("  emergency stopped   %s\n", d.emergency_stopped ? "yes" : "no");
    std::printf("  final behaviour     %s\n", ToString(head.behavior()));
    std::printf("  the line spoken     \"%s\"\n\n", spoken.c_str());
    return 0;
}
