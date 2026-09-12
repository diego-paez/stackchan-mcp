#include "servo_safety_controller.h"

#include <cmath>

namespace stackchan {
namespace attention {

namespace {

// Beyond this, the caller is not slightly wrong, it is broken. Clamping
// +1000 degrees to +30 would hide that; refusing it does not.
constexpr float kAbsurdDeg = 360.0f;

// dt outside this range means the scheduler misbehaved. Treat it as one
// nominal tick rather than believing it: a reported dt of 30 seconds would
// otherwise authorise a 30 * max_velocity move in a single step.
constexpr float kMinDtSec = 0.001f;
constexpr float kMaxDtSec = 0.200f;

inline bool finite(float v) { return std::isfinite(v); }

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

const char* ToString(SafetyVerdict v) {
    switch (v) {
        case SafetyVerdict::kOk:                 return "ok";
        case SafetyVerdict::kRejectedNotFinite:  return "rejected:not-finite";
        case SafetyVerdict::kRejectedAbsurd:     return "rejected:absurd";
        case SafetyVerdict::kClampedTravel:      return "clamped:travel";
        case SafetyVerdict::kLimitedStep:        return "limited:step";
        case SafetyVerdict::kLimitedVelocity:    return "limited:velocity";
        case SafetyVerdict::kHeldWatchdog:       return "held:watchdog";
        case SafetyVerdict::kHeldEmergencyStop:  return "held:e-stop";
    }
    return "?";
}

const char* ToString(Behavior b) {
    switch (b) {
        case Behavior::IDLE:        return "IDLE";
        case Behavior::ATTEND_FACE: return "ATTEND_FACE";
        case Behavior::LOOK_CENTER: return "LOOK_CENTER";
        case Behavior::THINK:       return "THINK";
        case Behavior::SLEEP:       return "SLEEP";
    }
    return "?";
}

ServoSafetyController::ServoSafetyController(const ServoLimits& limits,
                                             const NeutralPose& neutral,
                                             const HardwareBounds& bounds)
    : limits_(limits), neutral_(neutral), bounds_(bounds) {}

bool ServoSafetyController::isSafe(const ServoCommand& c) const {
    if (!c.valid) return false;
    if (!finite(c.yaw_deg) || !finite(c.pitch_deg) || !finite(c.speed_dps)) return false;
    if (std::fabs(c.yaw_deg) > kAbsurdDeg || std::fabs(c.pitch_deg) > kAbsurdDeg) return false;
    if (c.yaw_deg < limits_.min_yaw_deg || c.yaw_deg > limits_.max_yaw_deg) return false;
    if (c.pitch_deg < limits_.min_pitch_deg || c.pitch_deg > limits_.max_pitch_deg) return false;
    if (c.speed_dps < 0.0f || c.speed_dps > limits_.max_speed_dps) return false;
    return true;
}

bool ServoSafetyController::watchdogExpired(uint32_t now_ms) const {
    if (!has_valid_) return false;          // nothing has started yet
    if (watchdog_timeout_ms_ == 0) return false;
    return (now_ms - last_valid_ms_) > watchdog_timeout_ms_;
}

SafetyResult ServoSafetyController::filter(const ServoCommand& requested,
                                           const ServoCommand& current,
                                           float dt_sec) {
    SafetyResult out;

    // Hold position. Not "go to neutral" — an emergency stop that moves the
    // head is not a stop.
    const auto hold = [&](SafetyVerdict why) {
        out.command = current;
        out.command.valid = true;
        out.command.speed_dps = limits_.min_speed_dps;
        out.verdict = why;
        out.modified = true;
        return out;
    };

    if (emergency_stopped_) {
        return hold(SafetyVerdict::kHeldEmergencyStop);
    }

    if (!requested.valid) {
        return hold(SafetyVerdict::kHeldWatchdog);
    }

    // 1 — reject rather than sanitise. A non-finite angle is a broken caller.
    if (!finite(requested.yaw_deg) || !finite(requested.pitch_deg) ||
        !finite(requested.speed_dps) || !finite(dt_sec)) {
        ++rejected_;
        return hold(SafetyVerdict::kRejectedNotFinite);
    }
    if (std::fabs(requested.yaw_deg) > kAbsurdDeg ||
        std::fabs(requested.pitch_deg) > kAbsurdDeg) {
        ++rejected_;
        return hold(SafetyVerdict::kRejectedAbsurd);
    }

    // A dt the scheduler cannot have meant.
    const float dt = clampf(dt_sec, kMinDtSec, kMaxDtSec);

    SafetyVerdict verdict = SafetyVerdict::kOk;
    float yaw = requested.yaw_deg;
    float pitch = requested.pitch_deg;

    // 2 — travel envelope.
    const float yaw_c = clampf(yaw, limits_.min_yaw_deg, limits_.max_yaw_deg);
    const float pitch_c = clampf(pitch, limits_.min_pitch_deg, limits_.max_pitch_deg);
    if (yaw_c != yaw || pitch_c != pitch) {
        verdict = SafetyVerdict::kClampedTravel;
        ++clamped_;
    }
    yaw = yaw_c;
    pitch = pitch_c;

    // 3 — velocity, then 4 — per-update step. Velocity first so that the step
    // limiter is the tighter of the two on a long tick, and the step limiter
    // is what actually bounds a scheduler stall.
    const float max_by_velocity = limits_.max_velocity_deg_per_sec * dt;
    const float budget = (max_by_velocity < limits_.max_step_deg) ? max_by_velocity
                                                                  : limits_.max_step_deg;

    const float dyaw = yaw - current.yaw_deg;
    const float dpitch = pitch - current.pitch_deg;

    if (std::fabs(dyaw) > budget) {
        yaw = current.yaw_deg + (dyaw > 0 ? budget : -budget);
        verdict = (budget == max_by_velocity) ? SafetyVerdict::kLimitedVelocity
                                              : SafetyVerdict::kLimitedStep;
        ++clamped_;
    }
    if (std::fabs(dpitch) > budget) {
        pitch = current.pitch_deg + (dpitch > 0 ? budget : -budget);
        verdict = (budget == max_by_velocity) ? SafetyVerdict::kLimitedVelocity
                                              : SafetyVerdict::kLimitedStep;
        ++clamped_;
    }

    // Belt and braces: after rate limiting, re-clamp. If `current` was itself
    // outside the envelope — a head pushed by hand while powered off — rate
    // limiting alone would walk toward the target from an illegal position
    // and could stay illegal for several ticks.
    yaw = clampf(yaw, limits_.min_yaw_deg, limits_.max_yaw_deg);
    pitch = clampf(pitch, limits_.min_pitch_deg, limits_.max_pitch_deg);

    // And never outside what the hardware itself tolerates, whatever the
    // configuration says.
    yaw = clampf(yaw, bounds_.hard_min_yaw_deg, bounds_.hard_max_yaw_deg);
    pitch = clampf(pitch, bounds_.hard_min_pitch_deg, bounds_.hard_max_pitch_deg);

    float speed = clampf(requested.speed_dps, limits_.min_speed_dps, limits_.max_speed_dps);

    out.command.valid = true;
    out.command.yaw_deg = yaw;
    out.command.pitch_deg = pitch;
    out.command.speed_dps = speed;
    out.verdict = verdict;
    out.modified = (verdict != SafetyVerdict::kOk) ||
                   (speed != requested.speed_dps);
    return out;
}

void ServoSafetyController::emergencyStop() { emergency_stopped_ = true; }
void ServoSafetyController::clearEmergencyStop() { emergency_stopped_ = false; }

}  // namespace attention
}  // namespace stackchan
