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

    // The pitch envelope follows where the robot actually sits. This unit is
    // on a LOW base, so 45 -- the boot pose, nominally "level" -- points the
    // camera at the ceiling: photographs across the whole old envelope showed
    // wall and ceiling at every angle and never reached horizontal.
    //
    // 5..55 instead. The floor is M5Stack's documented recommended minimum,
    // not the 0 hard clamp: level is close to the bottom of travel for a
    // low-mounted robot, and a servo parked continuously at its end stop is
    // exactly the "extreme angle" the datasheet warns about. The ceiling gives
    // 43 degrees of look-up from neutral, which is what a robot on the floor
    // needs to find a standing adult.
    float min_pitch_deg = 5.0f;
    float max_pitch_deg = 55.0f;

    // Rate limits. max_velocity is the sustained cap; max_step bounds a single
    // update so a stalled scheduler cannot turn one late tick into a lurch.
    // Slow on purpose. At 60 deg/s the sweep read as constant motion and was
    // simply annoying to sit next to; a robot looking around a room is not in
    // a hurry. This is the sustained cap, not the speed anything asks for.
    float max_velocity_deg_per_sec = 25.0f;
    // Sized against the 120 ms command period: 25 deg/s allows 3 deg in that
    // time, so the velocity cap is what shapes normal motion and this only
    // bites when the scheduler stalls, which is its job.
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
// Where the head rests. Pitch is NOT the board's BOOT_INIT_PITCH_DEG of 45:
// that is the middle of the servo's travel, not the direction the camera
// looks, and on a low base it aims at the ceiling. 12 is a few degrees above
// the recommended floor, which for this mounting is roughly level.
struct NeutralPose {
    float yaw_deg = 0.0f;
    float pitch_deg = 12.0f;
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
