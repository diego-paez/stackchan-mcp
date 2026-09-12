#include "attention_controller.h"

#include <cmath>

namespace stackchan {
namespace attention {

const char* ToString(TrackingState s) {
    switch (s) {
        case TrackingState::kNoTarget:  return "no-target";
        case TrackingState::kTracking:  return "tracking";
        case TrackingState::kHolding:   return "holding";
        case TrackingState::kRelaxing:  return "relaxing";
        case TrackingState::kSearching: return "searching";
    }
    return "?";
}

namespace {
inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
// Error outside the dead zone, measured from the edge of the zone rather than
// from the centre. Measuring from the centre makes the head jump by the
// dead-zone width the instant the zone is crossed.
inline float beyond_deadzone(float v, float dz) {
    if (v > dz) return v - dz;
    if (v < -dz) return v + dz;
    return 0.0f;
}
}  // namespace

AttentionController::AttentionController(const AttentionConfig& cfg,
                                         const NeutralPose& neutral)
    : cfg_(cfg), neutral_(neutral) {
    target_.yaw_deg = neutral.yaw_deg;
    target_.pitch_deg = neutral.pitch_deg;
}

void AttentionController::reset() {
    have_face_ = false;
    fx_ = fy_ = 0.0f;
    state_ = TrackingState::kNoTarget;
    target_.yaw_deg = neutral_.yaw_deg;
    target_.pitch_deg = neutral_.pitch_deg;
    search_phase_ = 0.0f;
}

uint32_t AttentionController::lostForMs(uint32_t now_ms) const {
    if (!have_face_) return 0;
    return now_ms - last_seen_ms_;
}

void AttentionController::update(const FaceTarget& t, uint32_t now_ms, float dt_sec) {
    if (!enabled_) {
        state_ = TrackingState::kNoTarget;
        return;
    }
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0f) dt_sec = 0.02f;

    const bool usable = t.visible && std::isfinite(t.x) && std::isfinite(t.y) &&
                        t.confidence >= cfg_.min_confidence;

    if (usable) {
        const float x = clampf(t.x, -1.0f, 1.0f);
        const float y = clampf(t.y, -1.0f, 1.0f);
        if (!have_face_) {
            // First sight: adopt the position rather than filtering up to it
            // from zero, which would otherwise sweep the head across.
            fx_ = x;
            fy_ = y;
            have_face_ = true;
        } else {
            const float a = clampf(cfg_.position_alpha, 0.01f, 1.0f);
            fx_ += a * (x - fx_);
            fy_ += a * (y - fy_);
        }
        last_seen_ms_ = now_ms;
        state_ = TrackingState::kTracking;

        const float ex = beyond_deadzone(fx_, cfg_.deadzone_x);
        const float ey = beyond_deadzone(fy_, cfg_.deadzone_y);

        // Proportional, applied to the *target* rather than the current pose,
        // so the motion controller alone decides how fast to approach it.
        const float dyaw = (cfg_.invert_yaw ? -ex : ex) * cfg_.gain_yaw_deg;
        const float dpitch = (cfg_.invert_pitch ? -ey : ey) * cfg_.gain_pitch_deg;

        target_.yaw_deg += dyaw * dt_sec * 10.0f;
        target_.pitch_deg += dpitch * dt_sec * 10.0f;
        return;
    }

    // Not visible.
    if (!have_face_) {
        state_ = TrackingState::kNoTarget;
        return;
    }

    const uint32_t lost = now_ms - last_seen_ms_;
    if (lost < cfg_.hold_ms) {
        state_ = TrackingState::kHolding;   // keep the target exactly where it was
        return;
    }
    if (lost < cfg_.relax_ms) {
        state_ = TrackingState::kRelaxing;
        // Drift back toward neutral over the relax window, gently.
        const float k = clampf(dt_sec / 1.5f, 0.0f, 1.0f);
        target_.yaw_deg += k * (neutral_.yaw_deg - target_.yaw_deg);
        target_.pitch_deg += k * (neutral_.pitch_deg - target_.pitch_deg);
        return;
    }

    if (!cfg_.search_enabled) {
        state_ = TrackingState::kRelaxing;
        const float k = clampf(dt_sec / 1.5f, 0.0f, 1.0f);
        target_.yaw_deg += k * (neutral_.yaw_deg - target_.yaw_deg);
        target_.pitch_deg += k * (neutral_.pitch_deg - target_.pitch_deg);
        if (std::fabs(target_.yaw_deg - neutral_.yaw_deg) < 0.5f &&
            std::fabs(target_.pitch_deg - neutral_.pitch_deg) < 0.5f) {
            have_face_ = false;
            state_ = TrackingState::kNoTarget;
        }
        return;
    }

    // A very small search: a slow sweep about neutral, nothing wide.
    state_ = TrackingState::kSearching;
    const float period = (cfg_.search_period_sec > 0.1f) ? cfg_.search_period_sec : 8.0f;
    search_phase_ += dt_sec * 6.2831853f / period;
    target_.yaw_deg = neutral_.yaw_deg +
                      cfg_.search_amplitude_deg * std::sin(search_phase_);
    target_.pitch_deg = neutral_.pitch_deg;
}

}  // namespace attention
}  // namespace stackchan
