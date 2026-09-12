// behavior_manager.h — what the robot is trying to do, not where its head is.
//
// Behaviours set high-level intent. None of them writes an angle: the only
// thing a behaviour may do is enable or disable tracking and offer a pose for
// the mixer to use when tracking is off. That is the rule that keeps a single
// path to the hardware.
#pragma once

#include <cstdint>

#include "attention_types.h"
#include "servo_limits.h"

namespace stackchan {
namespace attention {

struct BehaviorSettings {
    bool tracking_enabled = true;
    bool has_fixed_pose = false;
    HeadPose fixed_pose;
    float speed_scale = 1.0f;   // SLEEP moves slower, for instance
};

class BehaviorManager {
public:
    explicit BehaviorManager(const NeutralPose& neutral);

    void setBehavior(Behavior b, uint32_t now_ms);
    Behavior current() const { return current_; }
    BehaviorSettings settings() const { return settings_; }

    // THINK is temporary: after think_hold_ms it returns to whatever was
    // running before, which for an interactive robot is ATTEND_FACE.
    void update(uint32_t now_ms);

    void setThinkHoldMs(uint32_t ms) { think_hold_ms_ = ms; }
    void setTrackingEnabled(bool on);

private:
    void apply(Behavior b);

    NeutralPose neutral_;
    Behavior current_ = Behavior::ATTEND_FACE;
    Behavior previous_ = Behavior::ATTEND_FACE;
    BehaviorSettings settings_;
    uint32_t entered_ms_ = 0;
    uint32_t think_hold_ms_ = 2500;
    bool tracking_override_off_ = false;
};

}  // namespace attention
}  // namespace stackchan
