// servo_limits.h — every number that can hurt the hardware, in one place.
//
// PROVENANCE. These are not invented. They are taken from the constants the
// stackchan board already enforces (stackchan.cc), and the comments there
// record how each was established on real hardware. Repeating them here
// rather than inventing a second set is the whole point; where this file is
// stricter than the board, it is stricter on purpose and says so.
//
//   SAFE_PITCH_MIN 0 / SAFE_PITCH_MAX 88   hard clamp, validated by on-device
//                                          sweep (Issue #98). pitch 89 made an
//                                          audible sub-stall; 0 is one degree
//                                          off the mechanical stop (PR #81).
//   RECOMMENDED 5..85                      M5Stack's documented range:
//                                          "Operating at extreme angles may
//                                          cause servo stall and permanent
//                                          damage."
//   BOOT_INIT_PITCH_DEG 45                 neutral, centre of 5..85.
//   MAX_SPEED_DPS 240                      SCS0009 datasheet working speed.
//   MIN_SMOOTH_SPEED_DPS 72                measured smoothness floor.
//   BOOT_INIT_TARGET_DEG_PER_SEC 15        the speed a boot move is allowed.
//
// A note on why this layer exists at all when the board already clamps pitch:
// the board clamps PITCH at the servo-write boundary. YAW is only saturated
// by YawDegToPos() at the raw position level, which is a different and much
// weaker guarantee. Face tracking drives yaw hardest, so yaw gets a real
// limit here.
#pragma once

namespace stackchan {
namespace attention {

struct ServoLimits {
    // Absolute travel. Tracking is given a much smaller envelope than the
    // hardware can reach: the first prototype has no reason to approach a
    // mechanical stop, and a bug that saturates a limit should saturate a
    // conservative one.
    float min_yaw_deg = -30.0f;
    float max_yaw_deg = 30.0f;

    // 45 +/- 20, i.e. 25..65. Inside M5Stack's recommended 5..85 with room to
    // spare. Never widen these toward the hard clamp without a reason written
    // down next to the change.
    float min_pitch_deg = 25.0f;
    float max_pitch_deg = 65.0f;

    // Rate limits. max_velocity is the sustained cap; max_step bounds a single
    // update so a stalled scheduler cannot turn one late tick into a lurch.
    float max_velocity_deg_per_sec = 60.0f;
    float max_step_deg = 3.0f;

    // Below this the SCS0009 stutters rather than moves (MIN_SMOOTH_SPEED_DPS
    // in stackchan.cc is 72 for smooth motion; 15 is the documented floor at
    // which stepping is still safe). A command slower than this is issued at
    // this speed instead, because a stuttering servo is worse than a slightly
    // fast one.
    float min_speed_dps = 15.0f;

    // Never exceeded regardless of what any controller asks for.
    float max_speed_dps = 120.0f;
};

// The pose the head is driven to at boot, on emergency stop, and whenever a
// controller stops producing valid commands.
struct NeutralPose {
    float yaw_deg = 0.0f;
    float pitch_deg = 45.0f;
};

// The hardware's own absolute bounds, mirrored so the safety controller can
// assert it never asks for something the board would have to clamp. If these
// two disagree, the board wins and this file is wrong.
struct HardwareBounds {
    float hard_min_pitch_deg = 0.0f;
    float hard_max_pitch_deg = 88.0f;
    float hard_min_yaw_deg = -90.0f;
    float hard_max_yaw_deg = 90.0f;
};

}  // namespace attention
}  // namespace stackchan
