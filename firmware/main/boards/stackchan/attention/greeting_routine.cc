#include "greeting_routine.h"

namespace stackchan {
namespace attention {

namespace {

// "Greetings, I am Stacky."
//
// Eight beats, about five seconds. The shape is: wake, notice you, look around
// once, face you and speak, nod, settle. It reads as a greeting because the
// line lands while the head is centred and facing forward — the sweep before
// it is what makes the centring feel deliberate rather than accidental.
//
// Amplitudes are small on purpose. Yaw stays inside ±14° of a ±30° envelope
// and gaze inside ±8° of a ±20° one, so every beat is comfortably interior and
// the safety layer is never the thing deciding where the head stops. A
// greeting that reaches its limits looks like a machine straining; one that
// stays well inside looks like a creature choosing to move.
//
// Hold times are the *dwell* at each pose, not the travel. The motion
// controller's velocity limit decides how long the head takes to arrive, so
// the routine runs slightly longer than the sum below and does so smoothly.
constexpr GreetingKeyframe kScript[] = {
    // dyaw  up    expression     say                        hold
    {   0.0f, -6.0f, "neutral",   nullptr,                   500 },  // head dips: waking
    {   0.0f,  8.0f, "surprised", nullptr,                   450 },  // lifts: notices you
    { -14.0f,  4.0f, "happy",     nullptr,                   500 },  // looks off to one side
    {  14.0f,  4.0f, "happy",     nullptr,                   600 },  // sweeps across
    {   0.0f,  3.0f, "happy",     "Greetings, I am Stacky",  1800 }, // centres, and speaks
    {   0.0f, -5.0f, "happy",     nullptr,                   350 },  // nod down
    {   0.0f,  3.0f, "happy",     nullptr,                   350 },  // and up: the bow closes
    {   0.0f,  0.0f, "neutral",   nullptr,                   500 },  // settle, ready to track
};
constexpr int kScriptCount = sizeof(kScript) / sizeof(kScript[0]);

float pitchFor(const NeutralPose& neutral, float look_up_deg) {
    return neutral.pitch_deg + (kPitchUpIsPositive ? look_up_deg : -look_up_deg);
}

}  // namespace

GreetingRoutine::GreetingRoutine(const NeutralPose& neutral) : neutral_(neutral) {}

int GreetingRoutine::stepCount() const { return kScriptCount; }

uint32_t GreetingRoutine::totalDurationMs() const {
    uint32_t total = 0;
    for (int i = 0; i < kScriptCount; ++i) total += kScript[i].hold_ms;
    return total;
}

void GreetingRoutine::start(uint32_t now_ms) {
    running_ = true;
    finished_ = false;
    step_ = 0;
    step_entered_ms_ = now_ms;
    enter(0);
}

void GreetingRoutine::cancel() {
    // No sinks fire on the way out. A cancelled greeting is usually an
    // emergency stop, and the last thing that should happen then is the robot
    // cheerfully announcing itself.
    running_ = false;
    finished_ = false;
    step_ = 0;
}

void GreetingRoutine::enter(int index) {
    if (index < 0 || index >= kScriptCount) return;
    const GreetingKeyframe& k = kScript[index];
    if (k.expression && expression_) expression_(k.expression);
    if (k.say && speech_) speech_(k.say);
}

void GreetingRoutine::update(uint32_t now_ms) {
    if (!running_) return;

    // Guard against a clock that went backwards across a wrap or a resync.
    // Treating that as "no time has passed" holds the current beat, which is
    // the harmless direction: the alternative skips the rest of the routine.
    if (now_ms < step_entered_ms_) {
        step_entered_ms_ = now_ms;
        return;
    }

    if ((now_ms - step_entered_ms_) < kScript[step_].hold_ms) return;

    ++step_;
    step_entered_ms_ = now_ms;

    if (step_ >= kScriptCount) {
        step_ = kScriptCount;   // past the end; pose() reads neutral
        running_ = false;
        finished_ = true;
        return;
    }
    enter(step_);
}

HeadPose GreetingRoutine::pose() const {
    HeadPose p;
    p.yaw_deg = neutral_.yaw_deg;
    p.pitch_deg = neutral_.pitch_deg;
    if (!running_ || step_ < 0 || step_ >= kScriptCount) return p;

    const GreetingKeyframe& k = kScript[step_];
    p.yaw_deg = neutral_.yaw_deg + k.dyaw_deg;
    p.pitch_deg = pitchFor(neutral_, k.look_up_deg);
    return p;
}

}  // namespace attention
}  // namespace stackchan
