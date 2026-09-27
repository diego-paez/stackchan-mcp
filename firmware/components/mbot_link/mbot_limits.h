// SPDX-License-Identifier: MIT
//
// Safety limits for the Stack-chan -> mBot2 (CyberPi) BLE link.
//
// Single source of truth: the "Safety limits" table in
// mblock-stacky-bridge/docs/protocol.md (Stacky protocol v1). The mBot-side
// runtime (mbot/micropython/stacky_runtime.py) clamps to the same numbers;
// this header is the Stack-chan-side copy, and the two layers enforce the
// limits independently. If you change a value here, change the table and the
// mBot runtime too — never loosen one side alone.
//
// Pure constants, no ESP-IDF dependency, so the host unit tests can include it.

#pragma once

// Wheels: 100 % speed == MBOT_MAX_RPM (about 20 cm/s with 6.5 cm wheels).
constexpr int MBOT_MAX_RPM = 60;
// A single fwd/back/left/right never lasts longer than this.
constexpr double MBOT_MAX_MOVE_S = 5.0;
// A whole mini-program (sum of its moves and waits, see mbot_command.cc).
constexpr double MBOT_MAX_PROG_S = 30.0;
constexpr int MBOT_MAX_PROG_STEPS = 20;

// Arm servo on port S4: 40 = lowest, 120 = highest.
constexpr int MBOT_ARM_MIN = 40;
constexpr int MBOT_ARM_MAX = 120;
constexpr int MBOT_ARM_HOME = 90;
// Gripper servo on port S3: 45 = closed, 120 = open.
constexpr int MBOT_GRIP_MIN = 45;
constexpr int MBOT_GRIP_MAX = 120;
constexpr int MBOT_GRIP_HOME = 90;
#define MBOT_ARM_PORT "S4"
#define MBOT_GRIP_PORT "S3"

// Servos glide (3 degree steps every 33 ms); they never jump.
constexpr int MBOT_SERVO_DEG_PER_S = 90;
// Forward motion auto-stops below this distance (mBot side, ultrasonic).
constexpr int MBOT_OBSTACLE_CM = 10;
// While anything moves, the mBot stops if no sk_hb / sk arrives for this long.
constexpr double MBOT_WATCHDOG_S = 3.0;
// Host sends sk_hb this often while connected.
constexpr double MBOT_HB_PERIOD_S = 1.0;

// v1.1: longest single closed-loop `straight`, largest single `turn`.
constexpr int MBOT_MAX_STEP_CM = 30;
constexpr int MBOT_MAX_TURN_DEG = 180;

// --- Protocol constants from the "Delivery rules" section (not limits) ---

// Shortest move the mBot accepts (it clamps to [0.1, MAX_MOVE_S]).
constexpr double MBOT_MIN_MOVE_S = 0.1;
// Default speed / duration used by the mBot when an argument is missing.
constexpr int MBOT_DEFAULT_SPEED_PCT = 40;
constexpr double MBOT_DEFAULT_MOVE_S = 1.0;
// Wait for sk_ack this long, then retry once with the same id.
constexpr int MBOT_ACK_TIMEOUT_MS = 1500;
// Command ids are 1..MBOT_MAX_CMD_ID.
constexpr int MBOT_MAX_CMD_ID = 9999;
// v1.1 straight/turn: speed % range and default (runtime parse_step).
constexpr int MBOT_STEP_SPEED_MIN_PCT = 5;
constexpr int MBOT_STEP_DEFAULT_SPEED_PCT = 30;
// v1.1 odom stream period: 0 = off, otherwise clamped to this range.
constexpr int MBOT_ODOM_MIN_MS = 100;
constexpr int MBOT_ODOM_MAX_MS = 5000;
// Runtime calibration used only for the host-side time estimate of
// straight/turn (stacky_runtime.py WHEEL_CM_PER_DEG / TRACK_CM).
constexpr double MBOT_WHEEL_CM_PER_DEG = 6.5 * 3.14159 / 360.0;
constexpr double MBOT_TRACK_CM = 11.5;
