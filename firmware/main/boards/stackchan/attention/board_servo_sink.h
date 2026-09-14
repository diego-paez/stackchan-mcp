// board_servo_sink.h — the adapter, and the only thing that knows a servo exists.
//
// WriteHeadAngles() is a private member of StackChanBoard and already owns the
// motion mutex, the torque state, the boot sequence and the pitch hard clamp.
// Rather than reach into the board or duplicate any of that, this takes two
// callables and lets stackchan.cc supply lambdas that call its own methods.
// Nothing in attention/ links against ESP-IDF as a result, which is what keeps
// the host tests possible.
//
// Wiring, inside StackChanBoard:
//
//     board_sink_ = std::make_unique<BoardServoSink>(
//         [this](int yaw, int pitch, uint32_t ms) {
//             WriteHeadAngles(yaw, pitch, ms, /*prefer_linear=*/true);
//         },
//         [this]() {
//             HeadPose p;
//             p.yaw_deg   = static_cast<float>(yaw_motion_.current_deg);
//             p.pitch_deg = static_cast<float>(pitch_motion_.current_deg);
//             return p;
//         },
//         [this]() { return servo_ok_; });
#pragma once

#include <cstdint>
#include <functional>

#include "attention_types.h"
#include "servo_sink.h"

namespace stackchan {
namespace attention {

class BoardServoSink : public ServoSink {
public:
    using WriteFn = std::function<void(int yaw_deg, int pitch_deg, uint32_t duration_ms)>;
    using ReadFn = std::function<HeadPose()>;
    using ReadyFn = std::function<bool()>;

    BoardServoSink(WriteFn write, ReadFn read, ReadyFn ready)
        : write_(std::move(write)), read_(std::move(read)), ready_(std::move(ready)) {}

    void write(const ServoCommand& c) override {
        if (!c.valid || !ready_()) return;

        // The board takes integer degrees. Rounding, not truncation: -0.6
        // truncates to 0 and biases every small negative correction toward
        // centre, which reads as a head that cannot look left.
        const int yaw = static_cast<int>(c.yaw_deg < 0 ? c.yaw_deg - 0.5f : c.yaw_deg + 0.5f);
        const int pitch = static_cast<int>(c.pitch_deg < 0 ? c.pitch_deg - 0.5f
                                                           : c.pitch_deg + 0.5f);

        // Duration from distance and the approved speed, so the board's own
        // interpolation moves no faster than the safety controller allowed.
        const HeadPose now = read_();
        const float dy = (yaw - now.yaw_deg);
        const float dp = (pitch - now.pitch_deg);
        const float dist = (dy < 0 ? -dy : dy) + (dp < 0 ? -dp : dp);
        const float speed = (c.speed_dps > 1.0f) ? c.speed_dps : 1.0f;

        uint32_t ms = static_cast<uint32_t>((dist / speed) * 1000.0f);
        if (ms < kMinDurationMs) ms = kMinDurationMs;
        if (ms > kMaxDurationMs) ms = kMaxDurationMs;

        write_(yaw, pitch, ms);
    }

    HeadPose readPose() override { return read_(); }
    bool ready() const override { return ready_ ? ready_() : false; }

private:
    // A duration of zero asks the servo bus for an instantaneous move, which
    // is the one thing this whole layer exists to prevent. The ceiling keeps a
    // tiny correction at a low speed from scheduling a move that outlives
    // several updates and fights the next one.
    // Matched to ScheduleConfig::servo_period_ms: a move the board cannot
    // finish before the next command arrives is a move that gets restarted,
    // and restarts lose the unfinished fraction to integer rounding.
    static constexpr uint32_t kMinDurationMs = 110;
    static constexpr uint32_t kMaxDurationMs = 600;

    WriteFn write_;
    ReadFn read_;
    ReadyFn ready_;
};

}  // namespace attention
}  // namespace stackchan
