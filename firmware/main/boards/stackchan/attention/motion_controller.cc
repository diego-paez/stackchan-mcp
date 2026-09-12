#include "motion_controller.h"

#include <cmath>

namespace stackchan {
namespace attention {

MotionController::MotionController(const MotionConfig& cfg, const NeutralPose& neutral)
    : cfg_(cfg) {
    pose_.yaw_deg = neutral.yaw_deg;
    pose_.pitch_deg = neutral.pitch_deg;
    command_.valid = false;
    command_.yaw_deg = pose_.yaw_deg;
    command_.pitch_deg = pose_.pitch_deg;
    command_.speed_dps = cfg_.speed_dps;
}

void MotionController::syncTo(const HeadPose& actual) {
    if (std::isfinite(actual.yaw_deg)) pose_.yaw_deg = actual.yaw_deg;
    if (std::isfinite(actual.pitch_deg)) pose_.pitch_deg = actual.pitch_deg;
    command_.yaw_deg = pose_.yaw_deg;
    command_.pitch_deg = pose_.pitch_deg;
    arrived_ = true;
}

void MotionController::update(const HeadPose& target, float dt_sec) {
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0f) dt_sec = 0.02f;
    if (dt_sec > 0.2f) dt_sec = 0.2f;

    // A non-finite target is the caller's bug. Do not propagate it; hold, and
    // let the safety layer's counters stay clean so they mean something.
    if (!std::isfinite(target.yaw_deg) || !std::isfinite(target.pitch_deg)) {
        command_.valid = false;
        return;
    }

    // Exponential approach. k is bounded to 1 so a long dt cannot overshoot
    // past the target and oscillate.
    float k = cfg_.approach_per_sec * dt_sec;
    if (k > 1.0f) k = 1.0f;

    pose_.yaw_deg += k * (target.yaw_deg - pose_.yaw_deg);
    pose_.pitch_deg += k * (target.pitch_deg - pose_.pitch_deg);

    const float err = std::fabs(target.yaw_deg - pose_.yaw_deg) +
                      std::fabs(target.pitch_deg - pose_.pitch_deg);
    arrived_ = (err < cfg_.arrive_deg);

    command_.valid = true;
    command_.yaw_deg = pose_.yaw_deg;
    command_.pitch_deg = pose_.pitch_deg;
    command_.speed_dps = cfg_.speed_dps;
}

}  // namespace attention
}  // namespace stackchan
