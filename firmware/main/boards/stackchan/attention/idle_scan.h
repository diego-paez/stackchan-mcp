// idle_scan.h — what the robot does when nobody is in front of it.
//
// The original Stack-chan idles by looking around, reacting to sound, and
// settling on a face when it finds one. This is that cycle, built the way
// everything else in this directory is built: no ESP-IDF, no allocation, and
// it never writes an angle — it offers a pose and the mixer decides, exactly
// as GreetingRoutine does.
//
//     ┌─────────── no face, quiet ───────────┐
//     │                                      │
//     ▼                                      │
//   SCANNING ──── voice heard ────► LISTENING│
//     │                                │     │
//     │ face found                     │ face found
//     ▼                                ▼     │
//   ENGAGED ──── face lost for reacquire_ms ─┘
//
// ENGAGED is the one state this class does not drive: it hands the head to
// AttentionController and says so through wantsControl(). Two components
// steering one head is the failure the mixer exists to prevent.
//
// THERE IS NO SOUND DIRECTION, AND THAT SHAPES THE DESIGN
// A Stack-chan with a mic array turns toward a voice. This firmware's AFE
// publishes a voice-activity boolean and nothing else — no bearing, no
// direction of arrival (Application::IsVoiceDetected()). So a voice cannot
// tell the head WHERE to look, only that looking is worthwhile. LISTENING
// therefore stops the sweep and brings the head to centre, which is both the
// honest best guess at where a speaker is and the thing that makes the next
// few detector frames stable. Inventing a bearing from an amplitude would be
// the same class of mistake as reading YUYV bytes as RGB: confident, and
// wrong.
#pragma once

#include <cstdint>

#include "attention_types.h"
#include "servo_limits.h"

namespace stackchan {
namespace attention {

enum class ScanState : uint8_t {
    kOff,        // disabled, or something else owns the head
    kScanning,   // sweeping stations, looking for somebody
    kListening,  // heard a voice, holding still and attending
    kEngaged,    // a face is being tracked; the tracker has the head
};

const char* ToString(ScanState s);

struct IdleScanConfig {
    // Well inside the ±30° yaw envelope, so the safety layer is never the
    // thing deciding where a sweep stops.
    float yaw_amplitude_deg = 22.0f;

    // Zero, because this robot is on a LOW base and neutral is already about
    // level. The lift existed for a desk robot looking up at people standing
    // over it; applied here it aimed the camera further into the ceiling,
    // which is the opposite of helpful. Raise it only for a high mounting.
    float pitch_lift_deg = 0.0f;

    // How long to hold each station.
    //
    // 1400 ms was picked as "enough frames for a 5 Hz detector" and it is, but
    // on hardware it reads as a head that never stops moving, and sitting next
    // to it is irritating rather than companionable. 4500 ms is twenty-two
    // frames, and a full seven-station sweep now takes about half a minute
    // instead of ten seconds. Looking around a room is not a metronome.
    uint32_t dwell_ms = 4500;

    // A voice keeps the head centred and still for this long after the last
    // time anybody spoke.
    uint32_t listen_hold_ms = 4000;

    // A face must be gone this long before the sweep resumes. Without it a
    // single dropped detection restarts the search and the head lurches away
    // from somebody who is still standing there.
    uint32_t reacquire_ms = 1200;

    bool enabled = true;
};

struct ScanStats {
    uint32_t stations = 0;     // station changes
    uint32_t sweeps = 0;       // full passes through the table
    uint32_t listens = 0;      // times a voice interrupted the sweep
    uint32_t engagements = 0;  // times a face was acquired
    uint32_t losses = 0;       // times a face was lost back to scanning
};

class IdleScan {
public:
    IdleScan() = default;
    IdleScan(const IdleScanConfig& cfg, const NeutralPose& neutral)
        : cfg_(cfg), neutral_(neutral) {}

    // Feed the same two facts the board already has.
    void observeFace(bool visible, uint32_t now_ms);
    void observeVoice(bool speaking, uint32_t now_ms);

    void begin(uint32_t now_ms);
    void update(uint32_t now_ms);

    // False while a face is being tracked: the tracker owns the head then,
    // and this class must not also have an opinion about where it points.
    bool wantsControl() const {
        return started_ && cfg_.enabled && state_ != ScanState::kEngaged &&
               state_ != ScanState::kOff;
    }

    HeadPose pose() const { return pose_; }
    ScanState state() const { return state_; }
    ScanStats stats() const { return stats_; }

    void setEnabled(bool on, uint32_t now_ms);
    bool enabled() const { return cfg_.enabled; }
    void setConfig(const IdleScanConfig& cfg) { cfg_ = cfg; }
    IdleScanConfig config() const { return cfg_; }

    // Live tuning. The right dwell and amplitude depend on the room and on
    // how the robot is mounted, neither of which is knowable from a header,
    // so they are adjustable without a reflash. Applies from the next station.
    void setDwellMs(uint32_t ms) { if (ms > 0) cfg_.dwell_ms = ms; }
    void setYawAmplitudeDeg(float deg) { if (deg > 0.0f) cfg_.yaw_amplitude_deg = deg; }
    void setPitchLiftDeg(float deg) { cfg_.pitch_lift_deg = deg; }

    // How many stations one full sweep visits. Exposed so a test can assert
    // on a whole pass without hard-coding the table's length.
    static int stationCount();

private:
    void enter(ScanState s, uint32_t now_ms);
    void advanceStation(uint32_t now_ms);
    void applyStation();

    IdleScanConfig cfg_;
    NeutralPose neutral_;

    ScanState state_ = ScanState::kOff;
    HeadPose pose_;
    int station_ = 0;
    uint32_t station_since_ms_ = 0;
    uint32_t last_voice_ms_ = 0;
    uint32_t face_lost_since_ms_ = 0;
    bool have_voice_ = false;
    bool face_visible_ = false;
    bool face_ever_seen_ = false;
    // Nothing sweeps until begin(), which HeadController calls only once the
    // self-test has passed and the greeting has finished. Without this,
    // update() woke the sweep up mid-self-test.
    bool started_ = false;
    ScanStats stats_;
};

}  // namespace attention
}  // namespace stackchan
