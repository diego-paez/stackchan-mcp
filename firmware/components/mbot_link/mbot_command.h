// SPDX-License-Identifier: MIT
//
// Host-side validation of Stacky protocol v1 `sk` commands.
//
// Every command the Stack-chan sends to the mBot passes through
// CheckCommand() first. It rejects anything it does not know, clamps every
// number to mbot_limits.h, and rejects mini-programs that exceed
// MBOT_MAX_PROG_STEPS or MBOT_MAX_PROG_S. The mBot runtime clamps again on its
// side; the two layers are independent on purpose.
//
// Pure C++ (no ESP-IDF), unit-tested on the host.

#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace mbot {

struct CommandCheck {
    bool ok = false;
    // Canonical command text to put after the id, e.g. "fwd 40 1.5".
    std::string normalized;
    // Short machine-readable reason when !ok (e.g. "unknown_cmd:jump").
    std::string error;
    // Human-readable notes about values that were clamped (empty if none).
    std::string clamped;
    std::string cmd;          // first token after normalisation ("fwd", "prog", ...)
    bool is_stop = false;
    bool is_motion = false;   // drives wheels or servos (fwd/back/left/right/arm/grip/home/prog)
    double seconds = 0.0;     // expected duration (moves + waits + servo glide estimate)
    int steps = 0;            // prog only
};

// Validate a full command (without id), e.g. "fwd 40 1", "straight 20 30",
// "turn -90", "odom 500", "sync 123456", "arm up",
// "prog fwd 40 1; led 0 255 0; wait 0.5; arm up".
CommandCheck CheckCommand(const std::string& text);

// Expected (= timeout) duration of a v1.1 straight/turn, as the runtime
// computes it: est * 1.5 + 1 s, capped at MBOT_MAX_MOVE_S.
double ClosedLoopSeconds(const std::string& cmd, double amount, double speed_pct);

// Worst-case servo glide time used in the program-length estimate.
double ServoGlideSeconds(const std::string& cmd, int target_deg);

// ---- v1.1 event parsing (mBot -> host) ----

// `done <id> <reason> dist=<cm> yaw=<deg> t=<mbot_ms>` or
// `done <id> prog t=<ms>`; `args` is everything after "done ".
// dist / yaw are empty when the mBot reports `none` or omits them (v1).
struct DoneEvent {
    int id = 0;
    std::string reason;           // target, time, timeout, obstacle, cmd, button,
                                  // watchdog, replaced, prog ("" for a v1 `done <id>`)
    std::optional<double> dist_cm;
    std::optional<double> yaw_deg;
    std::optional<int64_t> mbot_ms;
};
bool ParseDoneArgs(const std::string& args, DoneEvent* out);

// `odom t=<ms> l=<cm> r=<cm> yaw=<deg>`; `args` is everything after "odom ".
struct OdomSample {
    std::optional<int64_t> mbot_ms;
    std::optional<double> left_cm;
    std::optional<double> right_cm;
    std::optional<double> yaw_deg;
};
bool ParseOdomArgs(const std::string& args, OdomSample* out);

}  // namespace mbot
