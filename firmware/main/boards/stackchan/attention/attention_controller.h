// attention_controller.h — from "there is a face at x,y" to "look there".
//
// Three jobs, and they are all about not moving:
//   * a dead zone, so a face near the centre produces no motion at all;
//   * a low-pass filter, so detector jitter does not become head jitter;
//   * a loss policy, so a face that blinks out of detection for a frame does
//     not send the head hunting.
//
// The detector runs at 3-10 Hz and the servos at 30-50 Hz, so this must
// produce a sensible target between detections rather than only on them.
#pragma once

#include <cstdint>

#include "attention_types.h"
#include "servo_limits.h"

namespace stackchan {
namespace attention {

struct AttentionConfig {
    // Fraction of the half-image inside which a face is "centred enough".
    // 5-10% is the suggested starting range; below about 4% detector noise
    // alone will keep the head busy.
    float deadzone_x = 0.08f;
    float deadzone_y = 0.10f;

    // Degrees of head movement per unit of normalised image error. Low on
    // purpose: the first test is for stability, not speed.
    float gain_yaw_deg = 18.0f;
    float gain_pitch_deg = 10.0f;

    // Exponential smoothing on the face position, 0..1 per update. Smaller is
    // smoother and laggier.
    float position_alpha = 0.35f;

    // Loss policy, milliseconds since the face was last seen.
    uint32_t hold_ms = 500;      // below this, keep pointing where it was
    uint32_t relax_ms = 1500;    // between: drift back toward neutral
    bool search_enabled = false; // beyond: a small search. Off for test one.
    float search_amplitude_deg = 6.0f;
    float search_period_sec = 8.0f;

    // Ignore detections this weak outright.
    float min_confidence = 0.5f;

    // Camera mounting. If the image is mirrored relative to the head, a
    // positive image error means "turn the other way" — getting this wrong
    // produces a head that runs away from the face and hits a limit, so it is
    // a named setting rather than a minus sign buried in the maths.
    bool invert_yaw = false;
    bool invert_pitch = true;   // image y grows downward; pitch grows upward
};

enum class TrackingState : uint8_t {
    kNoTarget,
    kTracking,
    kHolding,    // recently lost, still pointing at the last position
    kRelaxing,   // lost for a while, drifting back to neutral
    kSearching,
};

const char* ToString(TrackingState s);

class AttentionController {
public:
    AttentionController(const AttentionConfig& cfg, const NeutralPose& neutral);

    // Feed the newest detection (visible==false is a perfectly good input,
    // and is what a lost face looks like). Call at the attention rate, not
    // only when a detection arrives.
    void update(const FaceTarget& target, uint32_t now_ms, float dt_sec);

    HeadPose getTarget() const { return target_; }
    TrackingState state() const { return state_; }
    uint32_t lostForMs(uint32_t now_ms) const;

    // Filtered face position, for diagnostics.
    float filteredX() const { return fx_; }
    float filteredY() const { return fy_; }

    void reset();
    void setEnabled(bool on) { enabled_ = on; }
    bool enabled() const { return enabled_; }

private:
    AttentionConfig cfg_;
    NeutralPose neutral_;

    HeadPose target_;
    TrackingState state_ = TrackingState::kNoTarget;
    bool enabled_ = true;

    bool have_face_ = false;
    float fx_ = 0.0f, fy_ = 0.0f;      // filtered, normalised
    uint32_t last_seen_ms_ = 0;
    float search_phase_ = 0.0f;
};

}  // namespace attention
}  // namespace stackchan
