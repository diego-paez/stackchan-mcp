// vision_tracker.h — where a face comes from.
//
// An interface, not an implementation, for two reasons. The obvious one is
// that everything downstream can then be tested on a laptop with no camera.
// The other is that the detector is the least settled part of this design:
// the board's existing Camera abstraction exposes Capture() and Explain() and
// deliberately hands no pixels to anyone, so local detection needs either a
// frame accessor added to Esp32Camera or a different source entirely. Putting
// an interface here means that decision does not block the control chain.
//
// Implementations must be cheap to poll and must not block: update() is
// called from the same loop as the servo scheduler.
#pragma once

#include <cstdint>

#include "attention_types.h"

namespace stackchan {
namespace attention {

class VisionTracker {
public:
    virtual ~VisionTracker() = default;

    // Poll the detector. Called at the vision rate (3-10 Hz), not the servo
    // rate; implementations that cannot produce a new result simply keep the
    // previous one, with its original last_seen_ms.
    virtual void update(uint32_t now_ms) = 0;

    // The current best target. visible==false is a valid, meaningful answer.
    virtual FaceTarget getTarget() const = 0;

    virtual bool ready() const { return true; }
    virtual const char* name() const { return "vision"; }
};

// A tracker that returns whatever it is told to. The whole control chain is
// exercised against this in the host tests, which is how the loss policy and
// the dead zone are tested without a room, a camera, and a person.
class ScriptedVisionTracker : public VisionTracker {
public:
    void update(uint32_t) override {}
    FaceTarget getTarget() const override { return target_; }
    const char* name() const override { return "scripted"; }

    void setTarget(const FaceTarget& t) { target_ = t; }
    void see(float x, float y, uint32_t now_ms, float confidence = 0.9f) {
        target_.visible = true;
        target_.x = x;
        target_.y = y;
        target_.size = 0.2f;
        target_.confidence = confidence;
        target_.last_seen_ms = now_ms;
    }
    void lose() { target_.visible = false; }

private:
    FaceTarget target_;
};

}  // namespace attention
}  // namespace stackchan
