#include "motion_mixer.h"

namespace stackchan {
namespace attention {

const char* ToString(MixSource s) {
    switch (s) {
        case MixSource::kNeutral:       return "neutral";
        case MixSource::kBehaviorPose:  return "behavior";
        case MixSource::kAttention:     return "attention";
    }
    return "?";
}

MixResult MotionMixer::mix(const BehaviorSettings& settings,
                           const HeadPose& attention_pose,
                           bool attention_has_target) const {
    MixResult r;
    r.speed_scale = settings.speed_scale;

    // A behaviour that pins the head wins outright. LOOK_CENTER and SLEEP
    // mean "stop looking at things", and a tracker that kept nudging would
    // make them a suggestion rather than a state.
    if (settings.has_fixed_pose) {
        r.pose = settings.fixed_pose;
        r.source = MixSource::kBehaviorPose;
        return r;
    }

    if (settings.tracking_enabled && attention_has_target) {
        r.pose = attention_pose;
        r.source = MixSource::kAttention;
        return r;
    }

    r.pose.yaw_deg = neutral_.yaw_deg;
    r.pose.pitch_deg = neutral_.pitch_deg;
    r.source = MixSource::kNeutral;
    return r;
}

}  // namespace attention
}  // namespace stackchan
