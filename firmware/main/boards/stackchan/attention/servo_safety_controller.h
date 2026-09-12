// servo_safety_controller.h — the last thing between software and a servo.
//
// Every command reaching the hardware passes through filter(). There is no
// second path. If you find yourself wanting one, that is the bug.
//
// What it guarantees, in order:
//   1. a non-finite or absurd request is rejected outright, never clamped
//      into something plausible — NaN means the caller is broken, and
//      silently turning it into 0 hides that;
//   2. every accepted angle is inside the configured envelope;
//   3. no single update moves further than max_step_deg;
//   4. no sustained motion exceeds max_velocity_deg_per_sec;
//   5. a caller that stops producing commands stops the head, rather than
//      leaving it driving toward a stale target;
//   6. emergency stop holds position and refuses everything until cleared.
//
// It is deliberately free of ESP-IDF: it is the part most worth unit testing,
// and it is tested on the host in host_test/test_servo_safety.cc.
#pragma once

#include <cstdint>

#include "attention_types.h"
#include "servo_limits.h"

namespace stackchan {
namespace attention {

// Why a command was altered or refused. Logged, and asserted on in tests —
// "it was clamped" is only useful if you can see which rule fired.
enum class SafetyVerdict : uint8_t {
    kOk,
    kRejectedNotFinite,     // NaN or infinity
    kRejectedAbsurd,        // beyond any plausible angle, e.g. +1000 deg
    kClampedTravel,         // outside the configured envelope
    kLimitedStep,           // single-update jump too large
    kLimitedVelocity,       // sustained rate too high
    kHeldWatchdog,          // no valid command for too long
    kHeldEmergencyStop,
};

const char* ToString(SafetyVerdict v);

struct SafetyResult {
    ServoCommand command;         // what may actually be sent
    SafetyVerdict verdict = SafetyVerdict::kOk;
    bool modified = false;        // differs from what was requested
};

class ServoSafetyController {
public:
    ServoSafetyController(const ServoLimits& limits, const NeutralPose& neutral,
                          const HardwareBounds& bounds);

    // Filter one request. `current` is where the head actually is; `dt_sec`
    // is the time since the last call and is itself sanity-checked, because a
    // scheduler hiccup that reports a huge dt would otherwise license a huge
    // step.
    SafetyResult filter(const ServoCommand& requested, const ServoCommand& current,
                        float dt_sec);

    // Would this command be accepted unchanged? Does not mutate state.
    bool isSafe(const ServoCommand& command) const;

    // Cancel everything and hold. Not torque-off: on the SCS0009 a head that
    // is holding is safer than a head that is free to fall under its own
    // weight, and the board's own auto-release path already handles torque.
    void emergencyStop();
    void clearEmergencyStop();
    bool emergencyStopped() const { return emergency_stopped_; }

    // Watchdog. Call noteValidCommand() whenever a controller produces one;
    // if none arrives within the timeout, filter() holds position.
    void setWatchdogTimeout(uint32_t ms) { watchdog_timeout_ms_ = ms; }
    void noteValidCommand(uint32_t now_ms) { last_valid_ms_ = now_ms; has_valid_ = true; }
    bool watchdogExpired(uint32_t now_ms) const;

    const ServoLimits& limits() const { return limits_; }
    const NeutralPose& neutral() const { return neutral_; }

    // Counters, for diagnostics and for proving in a test that the layer is
    // actually intercepting rather than passing everything through.
    uint32_t rejectedCount() const { return rejected_; }
    uint32_t clampedCount() const { return clamped_; }

private:
    ServoLimits limits_;
    NeutralPose neutral_;
    HardwareBounds bounds_;

    bool emergency_stopped_ = false;
    bool has_valid_ = false;
    uint32_t last_valid_ms_ = 0;
    uint32_t watchdog_timeout_ms_ = 500;

    uint32_t rejected_ = 0;
    uint32_t clamped_ = 0;
};

}  // namespace attention
}  // namespace stackchan
