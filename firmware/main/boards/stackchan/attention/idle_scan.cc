#include "idle_scan.h"

namespace stackchan {
namespace attention {
namespace {

// Where to look, as a fraction of yaw_amplitude_deg, and how much of the
// pitch lift to apply at each stop.
//
// The pattern returns to centre between every excursion, and that is the whole
// design rather than a detail. People stand in front of a desk robot far more
// often than beside it, so centre is where a face is most likely to appear and
// it should be sampled twice as often as either edge. It also looks like
// looking around rather than like a metronome, because the head changes
// direction at irregular intervals.
struct Station {
    float yaw_fraction;
    float pitch_fraction;
};

constexpr Station kStations[] = {
    { 0.00f, 1.00f},   // centre, gaze lifted: the most likely place
    {-1.00f, 0.70f},   // full left
    { 0.00f, 1.00f},   // back through centre
    { 0.55f, 1.00f},   // near right
    { 1.00f, 0.70f},   // full right
    { 0.00f, 1.00f},   // centre again
    {-0.55f, 1.00f},   // near left
};

constexpr int kStationCount = sizeof(kStations) / sizeof(kStations[0]);

}  // namespace

const char* ToString(ScanState s) {
    switch (s) {
        case ScanState::kOff: return "off";
        case ScanState::kScanning: return "scanning";
        case ScanState::kListening: return "listening";
        case ScanState::kEngaged: return "engaged";
    }
    return "off";
}

int IdleScan::stationCount() { return kStationCount; }

void IdleScan::begin(uint32_t now_ms) {
    started_ = true;
    station_ = 0;
    face_visible_ = false;
    face_ever_seen_ = false;
    have_voice_ = false;
    applyStation();
    enter(cfg_.enabled ? ScanState::kScanning : ScanState::kOff, now_ms);
}

void IdleScan::setEnabled(bool on, uint32_t now_ms) {
    if (on == cfg_.enabled) return;
    cfg_.enabled = on;
    if (!on) {
        enter(ScanState::kOff, now_ms);
    } else {
        enter(face_visible_ ? ScanState::kEngaged : ScanState::kScanning, now_ms);
    }
}

void IdleScan::observeFace(bool visible, uint32_t now_ms) {
    if (visible) {
        face_ever_seen_ = true;
    } else if (face_visible_) {
        // First tick of a loss — start the reacquire clock rather than
        // reacting immediately.
        face_lost_since_ms_ = now_ms;
    }
    face_visible_ = visible;
}

void IdleScan::observeVoice(bool speaking, uint32_t now_ms) {
    if (!speaking) return;
    have_voice_ = true;
    last_voice_ms_ = now_ms;
}

void IdleScan::enter(ScanState s, uint32_t now_ms) {
    if (s == state_) return;
    switch (s) {
        case ScanState::kListening: ++stats_.listens; break;
        case ScanState::kEngaged: ++stats_.engagements; break;
        case ScanState::kScanning:
            if (state_ == ScanState::kEngaged) ++stats_.losses;
            break;
        default: break;
    }
    state_ = s;
    station_since_ms_ = now_ms;

    if (s == ScanState::kListening) {
        // No bearing to turn toward, so centre and hold. See the header.
        pose_.yaw_deg = neutral_.yaw_deg;
        pose_.pitch_deg = neutral_.pitch_deg + cfg_.pitch_lift_deg;
    } else if (s == ScanState::kScanning) {
        applyStation();
    }
}

void IdleScan::applyStation() {
    const Station& st = kStations[station_];
    pose_.yaw_deg = neutral_.yaw_deg + st.yaw_fraction * cfg_.yaw_amplitude_deg;
    pose_.pitch_deg = neutral_.pitch_deg + st.pitch_fraction * cfg_.pitch_lift_deg;
}

void IdleScan::advanceStation(uint32_t now_ms) {
    station_ = station_ + 1;
    if (station_ >= kStationCount) {
        station_ = 0;
        ++stats_.sweeps;
    }
    ++stats_.stations;
    station_since_ms_ = now_ms;
    applyStation();
}

void IdleScan::update(uint32_t now_ms) {
    if (!started_) return;
    if (!cfg_.enabled) {
        if (state_ != ScanState::kOff) enter(ScanState::kOff, now_ms);
        return;
    }
    if (state_ == ScanState::kOff) enter(ScanState::kScanning, now_ms);

    // A face outranks everything. Unsigned subtraction throughout, so the
    // 49-day millisecond wrap is a non-event.
    if (face_visible_) {
        enter(ScanState::kEngaged, now_ms);
        return;
    }

    if (state_ == ScanState::kEngaged) {
        // Do not lurch away from somebody the detector merely blinked on.
        if (!face_ever_seen_ || (now_ms - face_lost_since_ms_) >= cfg_.reacquire_ms) {
            enter(ScanState::kScanning, now_ms);
        }
        return;
    }

    const bool voice_is_fresh =
        have_voice_ && (now_ms - last_voice_ms_) < cfg_.listen_hold_ms;

    if (voice_is_fresh) {
        enter(ScanState::kListening, now_ms);
        return;
    }

    if (state_ == ScanState::kListening) {
        // The voice has gone stale; pick the sweep back up where it was.
        have_voice_ = false;
        enter(ScanState::kScanning, now_ms);
        return;
    }

    if ((now_ms - station_since_ms_) >= cfg_.dwell_ms) {
        advanceStation(now_ms);
    }
}

}  // namespace attention
}  // namespace stackchan
