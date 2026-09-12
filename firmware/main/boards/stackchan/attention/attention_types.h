// attention_types.h — the values that pass between the attention components.
//
// Deliberately plain structs with no behaviour and no allocation: they cross
// task boundaries at up to 50 Hz, and they have to be copyable into a host
// unit test with no ESP-IDF present.
//
// ANGLE CONVENTION
// Absolute degrees, in the same frame the board's WriteHeadAngles() already
// uses — NOT offsets from neutral. Yaw 0 is straight ahead; pitch 45 is level
// (BOOT_INIT_PITCH_DEG in stackchan.cc). Converting between relative and
// absolute in more than one place is how sign errors reach a servo, so this
// layer never does it.
#pragma once

#include <cstdint>

namespace stackchan {
namespace attention {

// One detected face, in normalised image coordinates.
//
// x and y are -1..+1 with the origin at the image centre, so the controller
// never needs to know the sensor resolution. size is the larger box edge as a
// fraction of the image, which is the only depth cue available here.
struct FaceTarget {
    bool visible = false;
    float x = 0.0f;
    float y = 0.0f;
    float size = 0.0f;
    float confidence = 0.0f;
    uint32_t last_seen_ms = 0;
};

// Where the head should be pointing. Absolute degrees.
struct HeadPose {
    float yaw_deg = 0.0f;
    float pitch_deg = 45.0f;
};

// What the servo layer is being asked to do. Absolute degrees plus the speed
// the mover is allowed to use getting there.
//
// `valid` is not decoration: the safety controller uses it to distinguish
// "no opinion this tick" from "go to 0,0", and those are very different
// instructions to give a servo.
struct ServoCommand {
    bool valid = false;
    float yaw_deg = 0.0f;
    float pitch_deg = 45.0f;
    float speed_dps = 0.0f;
};

// The behaviours this prototype supports. Deliberately few.
enum class Behavior : uint8_t {
    IDLE,
    ATTEND_FACE,
    LOOK_CENTER,
    THINK,
    SLEEP,
};

const char* ToString(Behavior b);

}  // namespace attention
}  // namespace stackchan
