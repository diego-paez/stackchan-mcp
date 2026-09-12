#include "behavior_manager.h"

namespace stackchan {
namespace attention {

BehaviorManager::BehaviorManager(const NeutralPose& neutral) : neutral_(neutral) {
    apply(current_);
}

void BehaviorManager::setTrackingEnabled(bool on) {
    tracking_override_off_ = !on;
    apply(current_);
}

void BehaviorManager::setBehavior(Behavior b, uint32_t now_ms) {
    if (b != current_) {
        // Remember only durable behaviours, so THINK -> THINK cannot make
        // THINK its own fallback and strand the robot there.
        if (current_ != Behavior::THINK) previous_ = current_;
        current_ = b;
        entered_ms_ = now_ms;
        apply(b);
    }
}

void BehaviorManager::update(uint32_t now_ms) {
    if (current_ == Behavior::THINK && think_hold_ms_ > 0 &&
        (now_ms - entered_ms_) > think_hold_ms_) {
        const Behavior back = (previous_ == Behavior::THINK) ? Behavior::ATTEND_FACE
                                                             : previous_;
        setBehavior(back, now_ms);
    }
}

void BehaviorManager::apply(Behavior b) {
    BehaviorSettings s;
    switch (b) {
        case Behavior::ATTEND_FACE:
            s.tracking_enabled = true;
            s.has_fixed_pose = false;
            break;
        case Behavior::LOOK_CENTER:
            s.tracking_enabled = false;
            s.has_fixed_pose = true;
            s.fixed_pose.yaw_deg = neutral_.yaw_deg;
            s.fixed_pose.pitch_deg = neutral_.pitch_deg;
            break;
        case Behavior::THINK:
            // A small, safe offset — head tilted slightly up and to one side.
            // Chosen well inside the travel envelope so it can never be the
            // thing that reaches a limit.
            s.tracking_enabled = false;
            s.has_fixed_pose = true;
            s.fixed_pose.yaw_deg = neutral_.yaw_deg + 8.0f;
            s.fixed_pose.pitch_deg = neutral_.pitch_deg + 6.0f;
            break;
        case Behavior::SLEEP:
            s.tracking_enabled = false;
            s.has_fixed_pose = true;
            s.fixed_pose.yaw_deg = neutral_.yaw_deg;
            s.fixed_pose.pitch_deg = neutral_.pitch_deg;
            s.speed_scale = 0.4f;
            break;
        case Behavior::IDLE:
        default:
            s.tracking_enabled = false;
            s.has_fixed_pose = false;
            break;
    }
    if (tracking_override_off_) s.tracking_enabled = false;
    settings_ = s;
}

}  // namespace attention
}  // namespace stackchan
