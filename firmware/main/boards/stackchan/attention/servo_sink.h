// servo_sink.h — the one door to the hardware.
//
// The controllers never touch a servo. They hand a filtered command to a
// ServoSink, and on the real robot exactly one implementation exists: an
// adapter that calls the board's existing WriteHeadAngles(). That function
// already owns the motion mutex, the torque state, the boot sequence and the
// pitch hard clamp, and reimplementing any of it here would create the second
// path this design is meant to prevent.
//
// Injecting it as an interface is also what lets the host tests assert on
// what would have been sent without a servo present.
#pragma once

#include "attention_types.h"

namespace stackchan {
namespace attention {

class ServoSink {
public:
    virtual ~ServoSink() = default;

    // Send an already-filtered command. Implementations must treat this as
    // the final word on angle; they may translate units and choose a duration
    // from speed, but must not re-plan the motion.
    virtual void write(const ServoCommand& safe_command) = 0;

    // Where the head actually is. Used to seed the motion controller at boot
    // and to give the safety controller a truthful `current`.
    virtual HeadPose readPose() = 0;

    // True once the servo bus is up. The SCS0009 needs ~200 ms after power
    // enable before it answers, and commanding it before that is how the
    // "boot snap" was produced.
    virtual bool ready() const = 0;
};

// Records what it was asked to do and never moves anything.
class RecordingServoSink : public ServoSink {
public:
    void write(const ServoCommand& c) override {
        last = c;
        ++writes;
        if (c.valid) {
            pose_.yaw_deg = c.yaw_deg;
            pose_.pitch_deg = c.pitch_deg;
        }
    }
    HeadPose readPose() override { return pose_; }
    bool ready() const override { return ready_; }

    void setPose(const HeadPose& p) { pose_ = p; }
    void setReady(bool r) { ready_ = r; }

    ServoCommand last;
    int writes = 0;

private:
    HeadPose pose_;
    bool ready_ = true;
};

}  // namespace attention
}  // namespace stackchan
