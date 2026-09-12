// motion_mixer.h — decide which of the competing opinions the head follows.
//
// Exactly one pose reaches the motion controller each tick. The mixer is
// where that choice is made and, importantly, the only place it is made:
// behaviours do not write angles and the attention controller does not know
// about behaviours.
#pragma once

#include "attention_types.h"
#include "behavior_manager.h"
#include "servo_limits.h"

namespace stackchan {
namespace attention {

enum class MixSource : uint8_t {
    kNeutral,
    kBehaviorPose,
    kAttention,
};

const char* ToString(MixSource s);

struct MixResult {
    HeadPose pose;
    MixSource source = MixSource::kNeutral;
    float speed_scale = 1.0f;
};

class MotionMixer {
public:
    explicit MotionMixer(const NeutralPose& neutral) : neutral_(neutral) {}

    MixResult mix(const BehaviorSettings& settings, const HeadPose& attention_pose,
                  bool attention_has_target) const;

private:
    NeutralPose neutral_;
};

}  // namespace attention
}  // namespace stackchan
