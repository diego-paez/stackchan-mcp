// greeting_routine.h — the robot introduces itself, once, on waking.
//
// A scripted timeline of poses, expressions and one spoken line. It is the
// same shape as the self-test and deliberately so: a table of keyframes with
// a hold time, stepped by the same scheduler, producing a target pose that
// goes through the ordinary motion and safety path. The greeting writes no
// angle to any servo. Like a behaviour, it only offers a pose.
//
// Order matters and is fixed: neutral, then the self-test, then this, then
// face tracking. The greeting is expressive movement, and expressive movement
// does not happen until the safety layer has been shown to work on this
// particular unit. If the self-test fails the greeting never runs.
//
// Expression and speech leave through std::function sinks rather than a
// direct call into the display or the audio path, for the same reason
// board_servo_sink.h exists: this directory stays free of ESP-IDF so the whole
// timeline can be stepped and asserted on a host, with no hardware and no
// waiting 5 seconds per test.
#pragma once

#include <cstdint>
#include <functional>

#include "attention_types.h"
#include "servo_limits.h"

namespace stackchan {
namespace attention {

// One beat of the routine. Offsets are relative to neutral, never absolute:
// the neutral pose is a property of the assembled robot and the script should
// not have to know it.
struct GreetingKeyframe {
    float dyaw_deg;          // + and - are symmetric here; the script sweeps both ways
    float look_up_deg;       // + is "raise the gaze", whatever that means in servo terms
    const char* expression;  // nullptr leaves the previous expression alone
    const char* say;         // nullptr says nothing on this beat
    uint32_t hold_ms;
};

// Which servo direction counts as "up".
//
// AttentionController documents image y as growing downward while pitch grows
// upward, and defaults invert_pitch to true on that basis. The script is
// written in gaze terms — "look up", "nod down" — and the assumption is
// applied here, once. If the head turns out to nod when it should look up,
// this is the single line to change, not eight rows of a table.
constexpr bool kPitchUpIsPositive = true;

class GreetingRoutine {
public:
    using ExpressionFn = std::function<void(const char*)>;
    using SpeechFn = std::function<void(const char*)>;

    explicit GreetingRoutine(const NeutralPose& neutral);

    // Both optional. A routine with no sinks still moves — useful on a unit
    // with no speaker, and it is what the host tests run.
    void setExpressionSink(ExpressionFn fn) { expression_ = std::move(fn); }
    void setSpeechSink(SpeechFn fn) { speech_ = std::move(fn); }

    void start(uint32_t now_ms);

    // Advances the timeline and fires the sinks on the beat they belong to.
    // Safe to call more often than the timeline changes.
    void update(uint32_t now_ms);

    // Abandon the routine wherever it is. The head is left to the ordinary
    // path, which returns it to neutral. Used by emergency stop.
    void cancel();

    bool running() const { return running_; }
    bool finished() const { return finished_; }
    int step() const { return step_; }
    int stepCount() const;

    // The pose for the current beat, absolute, ready for the motion
    // controller. Neutral before the routine starts and after it ends.
    HeadPose pose() const;

    // Total scripted duration, for a caller that wants to log it or bound a
    // watchdog. Does not include the time the head takes to reach each pose,
    // which the velocity limit decides.
    uint32_t totalDurationMs() const;

private:
    void enter(int index);

    NeutralPose neutral_;
    ExpressionFn expression_;
    SpeechFn speech_;

    bool running_ = false;
    bool finished_ = false;
    int step_ = 0;
    uint32_t step_entered_ms_ = 0;
};

}  // namespace attention
}  // namespace stackchan
