// motion_controller.h — turn a desired pose into a bounded command.
//
// The attention controller says where to look. This decides how to get
// there: it moves the commanded pose toward the target at a limited rate, so
// the safety controller downstream normally has nothing to do. The safety
// layer is the net, not the plan.
#pragma once

#include "attention_types.h"
#include "servo_limits.h"

namespace stackchan {
namespace attention {

struct MotionConfig {
    // Fraction of the remaining error consumed per second. 2.0 means roughly
    // 86% of the way there in one second. Deliberately unhurried.
    // Lower is gentler. 2.0 covered 86% of the distance in a second, which
    // looks like a head snapping to each new station; 0.8 is an unhurried
    // turn that reads as looking rather than twitching.
    float approach_per_sec = 0.8f;

    // Cruise speed handed to the servo layer. Kept between the smooth floor
    // and something well short of the datasheet maximum.
    // 30 deg/s is the gateway's own "low" preset, chosen for slow expressive
    // motion. It is below stackchan.cc's MIN_SMOOTH_SPEED_DPS, so the SCS0009
    // will look faintly textured -- which is the right trade for a robot that
    // should not appear restless.
    float speed_dps = 30.0f;

    // Below this the head is considered to have arrived, and stops being
    // commanded — a servo told to make a 0.2 degree correction forever will
    // buzz.
    float arrive_deg = 0.4f;
};

class MotionController {
public:
    MotionController(const MotionConfig& cfg, const NeutralPose& neutral);

    void update(const HeadPose& target, float dt_sec);
    ServoCommand getCommand() const { return command_; }

    // Seed the commanded pose from where the head actually is, at boot or
    // after any period where something else was driving.
    void syncTo(const HeadPose& actual);

    bool arrived() const { return arrived_; }

private:
    MotionConfig cfg_;
    HeadPose pose_;
    ServoCommand command_;
    bool arrived_ = true;
};

}  // namespace attention
}  // namespace stackchan
