// SPDX-License-Identifier: MIT
//
// MbotLink: NimBLE central that drives one Makeblock mBot2 (CyberPi running
// the Stacky runtime) over Stacky protocol v1.1 (mblock-stacky-bridge
// docs/protocol.md).
//
//   * scans (only while not connected) for a name starting "Makeblock_LE",
//     optionally pinned to the MAC in NVS namespace "mbot", key "mac";
//   * connects, discovers service FFE1 / write FFE3 / notify FFE2, enables
//     notifications and asks for a 512-byte MTU;
//   * reconnects with exponential backoff (1 s .. 30 s);
//   * sends `sk` commands one at a time and waits for `sk_ack` (1.5 s, one
//     retry with the same id); `stop` bypasses the queue and cancels the
//     command that is waiting for its ack so it is never retried;
//   * sends `sk_hb <counter>` every MBOT_HB_PERIOD_S while connected and while
//     the host link is alive (see NoteHostAlive), so the mBot's own watchdog
//     stops it when the LLM side goes away;
//   * every command is validated/clamped by CheckCommand() before sending;
//   * v1.1: caches the latest `odom` sample and the last kDoneHistory `done`
//     events (with actual dist / yaw), and measures the send->ack RTT.
//
// Thread model: public methods may be called from any task. NimBLE
// callbacks run in the NimBLE host task; the heartbeat / reconnect loop runs
// in its own small task. The event callback is invoked from the NimBLE host
// task and must return quickly (post work elsewhere).

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "f3_codec.h"
#include "mbot_command.h"
#include "mbot_limits.h"

// NimBLE C types (global namespace), so this header does not pull in NimBLE.
struct ble_gap_event;
struct ble_gap_disc_desc;
struct ble_gatt_error;
struct ble_gatt_svc;
struct ble_gatt_chr;
struct ble_gatt_dsc;
struct ble_gatt_attr;

class MbotLink {
public:
    enum class State { kOff, kIdle, kScanning, kConnecting, kDiscovering, kReady };

    struct Result {
        bool ok = false;
        int id = 0;
        // "ok", "err" (mBot refused), "timeout", "not_connected", "invalid",
        // "cancelled" (a stop overtook it), "disconnected", "send_failed".
        std::string status;
        // mBot detail text ("dup", "locked", "3_steps 4s", ...) or the
        // validation error.
        std::string detail;
        // Normalised command that was sent (after clamping).
        std::string sent;
        // Notes about clamped values, empty if nothing was clamped.
        std::string clamped;
        double expected_seconds = 0.0;
        // Send -> ack round trip of the answered attempt, -1 if none.
        int rtt_ms = -1;
    };

    // v1.1 caches. at_us is Stack-chan's esp_timer time of arrival.
    struct Done {
        bool valid = false;
        int64_t at_us = 0;
        mbot::DoneEvent event;
        std::string raw;  // args after "done "
    };
    struct Odometry {
        bool valid = false;
        int64_t at_us = 0;
        mbot::OdomSample sample;
    };

    struct Sensors {
        bool valid = false;
        int64_t at_us = 0;
        std::string raw;  // "dist=12 batt=90 arm=90 grip=90 locked=0"
    };

    // name: event name ("ready", "done", "obstacle", "stopped", "locked",
    // "unlocked", "button", "sensors", "hb"), plus the link events
    // "connected" / "disconnected" generated locally. args: the rest.
    using EventCallback = std::function<void(const std::string& name, const std::string& args)>;

    static MbotLink& GetInstance();

    // Initialise NimBLE and start scanning. Safe to call once; later calls
    // are no-ops. If a start gate is set, NimBLE is brought up only once the
    // gate returns true (or after kStartGateMaxWaitMs), so the BLE controller's
    // internal-RAM allocation does not compete with activation / TLS at boot.
    void Start();
    void SetStartGate(std::function<bool()> gate) { start_gate_ = std::move(gate); }

    // Validate, clamp, send one command and wait for its ack. Never call
    // with "stop" semantics expected to queue — use Stop() (SendCommand
    // forwards "stop" to Stop() anyway).
    Result SendCommand(const std::string& cmd, int ack_timeout_ms = MBOT_ACK_TIMEOUT_MS);

    // Send stop immediately, even while another command waits for its ack.
    Result Stop(const char* reason = "cmd");

    // Send `read` and wait (up to wait_ms after the ack) for the sensors event.
    Result ReadSensors(Sensors* out, int wait_ms = 800);

    void SetEventCallback(EventCallback cb);

    // Heartbeat gating. When required, sk_hb is sent only if NoteHostAlive()
    // was called within host_alive_window_ms. The board calls NoteHostAlive
    // while the gateway WebSocket is up, so losing the LLM link stops the
    // heartbeat and the mBot watchdog halts any motion within WATCHDOG_S.
    void RequireHostAlive(bool required, int host_alive_window_ms = 4000);
    void NoteHostAlive();

    State GetState() const { return state_.load(); }
    static const char* StateName(State s);
    bool IsConnected() const { return state_.load() == State::kReady; }
    std::string PeerAddress() const;
    std::string PeerName() const;
    std::string TargetMac() const;
    std::string RuntimeVersion() const;
    int BatteryPercent() const { return battery_.load(); }
    bool Locked() const { return locked_.load(); }
    int Mtu() const { return mtu_.load(); }
    Sensors LastSensors() const;
    // Most recent `done` event, and the one for a given command id (the last
    // kDoneHistory are kept).
    Done LastDone() const;
    bool FindDone(int id, Done* out) const;
    Odometry LastOdometry() const;
    // Last measured send->ack round trip (any command), -1 before the first.
    int LastRttMs() const { return last_rtt_ms_.load(); }
    static constexpr int kDoneHistory = 8;
    int64_t LastRxAgeMs() const;

private:
    MbotLink() = default;

    // --- NimBLE glue (static trampolines take `this` via arg) ---
    static void HostTask(void* param);
    static void OnSync();
    static void OnReset(int reason);
    static int GapEvent(struct ble_gap_event* event, void* arg);
    static int OnMtu(uint16_t conn, const struct ble_gatt_error* error, uint16_t mtu, void* arg);
    static int OnService(uint16_t conn, const struct ble_gatt_error* error,
                         const struct ble_gatt_svc* service, void* arg);
    static int OnCharacteristic(uint16_t conn, const struct ble_gatt_error* error,
                                const struct ble_gatt_chr* chr, void* arg);
    static int OnDescriptor(uint16_t conn, const struct ble_gatt_error* error, uint16_t chr_val_handle,
                            const struct ble_gatt_dsc* dsc, void* arg);
    static int OnSubscribed(uint16_t conn, const struct ble_gatt_error* error,
                            struct ble_gatt_attr* attr, void* arg);

    static void ManagerTask(void* param);
    void ManagerLoop();
    bool InitNimble();
    void StartScan();
    void ScheduleReconnect(const char* why);
    void FailDiscovery(const char* why);
    bool AdvMatches(const struct ble_gap_disc_desc& disc, std::string* name_out);
    void HandleNotify(const uint8_t* data, size_t len);
    void HandleAck(const std::string& value);
    void HandleEvent(const std::string& value);
    void EmitEvent(const std::string& name, const std::string& args);
    void OnLinkDown(const char* why);

    bool WriteFrame(const std::vector<uint8_t>& frame);
    bool SendTopic(const char* topic, const std::string& value);
    int NextId();
    // Send "<id> <text>" and wait for the ack with one same-id retry.
    Result Transact(int id, const std::string& text, int ack_timeout_ms, bool is_stop,
                    uint32_t generation);

    std::atomic<State> state_{State::kOff};
    std::atomic<bool> started_{false};
    std::function<bool()> start_gate_;
    std::atomic<bool> synced_{false};
    uint8_t own_addr_type_ = 0;

    // Connection (written in the host task, read elsewhere under mutex_).
    mutable std::mutex mutex_;
    uint16_t conn_handle_ = 0xFFFF;
    uint16_t svc_start_ = 0, svc_end_ = 0;
    uint16_t write_handle_ = 0, notify_handle_ = 0, notify_end_ = 0, cccd_handle_ = 0;
    uint16_t last_chr_def_after_notify_ = 0;
    std::string peer_addr_, peer_name_, target_mac_, runtime_version_;
    std::atomic<int> mtu_{23};
    std::atomic<int> battery_{-1};
    std::atomic<bool> locked_{false};
    std::atomic<int64_t> last_rx_us_{0};
    Sensors sensors_;
    Done done_history_[kDoneHistory];  // guarded by mutex_, ring
    int done_next_ = 0;                // guarded by mutex_
    Done last_done_;                   // guarded by mutex_
    Odometry odom_;                    // guarded by mutex_
    std::atomic<int> last_rtt_ms_{-1};

    mbot::F3Decoder decoder_;  // host task only

    // Reconnect backoff (manager task + host task, atomics).
    std::atomic<int64_t> next_scan_at_us_{0};
    std::atomic<int> backoff_ms_{1000};

    // Command/ack bookkeeping. cmd_mutex_ serialises ordinary commands;
    // stop does not take it.
    std::mutex cmd_mutex_;
    std::mutex stop_mutex_;
    // Keeps a frame's chunks together when the heartbeat task and a
    // command write at the same time (only matters at a small MTU).
    std::mutex write_mutex_;
    std::condition_variable ack_cv_;
    // Bumped by every Stop(); a command whose generation changed while it
    // waited is reported "cancelled" and never retried.
    uint32_t stop_generation_ = 0;  // guarded by mutex_
    struct Pending {
        int id = 0;
        bool active = false;
        bool done = false;
        bool cancelled = false;
        std::string reply;  // "ok ..." / "err ..."
    };
    Pending pending_cmd_;   // guarded by mutex_
    Pending pending_stop_;  // guarded by mutex_
    std::atomic<int> next_id_{0};

    // Heartbeat.
    std::atomic<bool> require_host_alive_{false};
    std::atomic<int> host_alive_window_ms_{4000};
    std::atomic<int64_t> host_alive_us_{0};
    uint32_t hb_counter_ = 0;
    bool hb_gate_was_open_ = true;

    std::mutex cb_mutex_;
    EventCallback event_cb_;
};
