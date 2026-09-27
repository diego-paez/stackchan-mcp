// SPDX-License-Identifier: MIT
#include "mbot_link.h"

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs.h>

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

const char* TAG = "MbotLink";

constexpr char kNamePrefix[] = "Makeblock_LE";
constexpr uint16_t kServiceUuid = 0xFFE1;
constexpr uint16_t kWriteUuid = 0xFFE3;
constexpr uint16_t kNotifyUuid = 0xFFE2;
constexpr uint16_t kCccdUuid = 0x2902;

// BLE_UUID16_DECLARE is a C compound literal (not valid C++), so keep
// named UUID objects instead.
const ble_uuid16_t kServiceUuid16 = BLE_UUID16_INIT(kServiceUuid);
const ble_uuid16_t kWriteUuid16 = BLE_UUID16_INIT(kWriteUuid);
const ble_uuid16_t kNotifyUuid16 = BLE_UUID16_INIT(kNotifyUuid);
const ble_uuid16_t kCccdUuid16 = BLE_UUID16_INIT(kCccdUuid);

constexpr int kScanDurationMs = 10000;
// Low duty cycle (30 ms of every 100 ms) so Wi-Fi keeps most of the radio.
constexpr uint16_t kScanItvl = 0x00A0;    // 100 ms in 0.625 ms units
constexpr uint16_t kScanWindow = 0x0030;  // 30 ms
constexpr int kConnectTimeoutMs = 5000;
constexpr int kBackoffMinMs = 1000;
constexpr int kBackoffMaxMs = 30000;
constexpr int kFirstScanDelayMs = 4000;  // let Wi-Fi / audio come up first
constexpr int kStartGateMaxWaitMs = 90000;  // start BLE anyway after this (offline use)
constexpr int kStartGateSettleMs = 3000;    // after the gate opens, let TLS/WS settle
constexpr int kManagerTickMs = 100;

int64_t NowUs() { return esp_timer_get_time(); }

std::string AddrToString(const ble_addr_t& a) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", a.val[5], a.val[4], a.val[3], a.val[2],
             a.val[1], a.val[0]);
    return buf;
}

std::string Upper(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

}  // namespace

MbotLink& MbotLink::GetInstance() {
    static MbotLink instance;
    return instance;
}

const char* MbotLink::StateName(State s) {
    switch (s) {
        case State::kOff: return "off";
        case State::kIdle: return "idle";
        case State::kScanning: return "scanning";
        case State::kConnecting: return "connecting";
        case State::kDiscovering: return "discovering";
        case State::kReady: return "connected";
    }
    return "unknown";
}

// ------------------------------------------------------------------ startup

void MbotLink::Start() {
    if (started_.exchange(true)) {
        return;
    }
    // Optional pin to one robot: NVS namespace "mbot", key "mac" ("AA:BB:..").
    nvs_handle_t h;
    if (nvs_open("mbot", NVS_READONLY, &h) == ESP_OK) {
        char mac[24] = {0};
        size_t len = sizeof(mac);
        if (nvs_get_str(h, "mac", mac, &len) == ESP_OK) {
            std::lock_guard<std::mutex> lock(mutex_);
            target_mac_ = Upper(mac);
        }
        nvs_close(h);
    }
    // Random first id: after a Stack-chan reboot the mBot still remembers the
    // last id it saw and would answer "ok dup" (without acting) if we reused it.
    next_id_.store(static_cast<int>(esp_random() % MBOT_MAX_CMD_ID) + 1);

    // NimBLE is initialised from the manager task after a short delay so the
    // BLE controller's internal-RAM allocation happens after Wi-Fi and audio
    // have taken theirs.
    if (xTaskCreate(ManagerTask, "mbot_link", 4096, this, 3, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "failed to create manager task; mBot link disabled");
    }
}

bool MbotLink::InitNimble() {
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s; mBot link disabled", esp_err_to_name(err));
        return false;
    }
    ble_hs_cfg.sync_cb = &MbotLink::OnSync;
    ble_hs_cfg.reset_cb = &MbotLink::OnReset;
    ble_att_set_preferred_mtu(512);
    nimble_port_freertos_init(&MbotLink::HostTask);
    ESP_LOGI(TAG, "NimBLE central started (target=%s)",
             target_mac_.empty() ? "any Makeblock_LE*" : target_mac_.c_str());
    return true;
}

void MbotLink::HostTask(void*) {
    nimble_port_run();  // returns only after nimble_port_stop()
    nimble_port_freertos_deinit();
}

void MbotLink::OnSync() {
    auto& self = GetInstance();
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) {
        rc = ble_hs_id_infer_auto(0, &self.own_addr_type_);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "no usable BLE address (rc=%d)", rc);
        return;
    }
    self.synced_.store(true);
    ESP_LOGI(TAG, "BLE host synced");
}

void MbotLink::OnReset(int reason) {
    auto& self = GetInstance();
    ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
    self.synced_.store(false);
    self.OnLinkDown("host reset");
    self.ScheduleReconnect("host reset");
}

// ------------------------------------------------------------------ manager

void MbotLink::ManagerTask(void* param) {
    static_cast<MbotLink*>(param)->ManagerLoop();
}

void MbotLink::ManagerLoop() {
    vTaskDelay(pdMS_TO_TICKS(kFirstScanDelayMs));
    if (start_gate_) {
        int waited = 0;
        while (!start_gate_() && waited < kStartGateMaxWaitMs) {
            vTaskDelay(pdMS_TO_TICKS(500));
            waited += 500;
        }
        ESP_LOGI(TAG, "start gate %s after %d ms", waited < kStartGateMaxWaitMs ? "opened" : "timed out",
                 waited + kFirstScanDelayMs);
        vTaskDelay(pdMS_TO_TICKS(kStartGateSettleMs));
    }
    ESP_LOGI(TAG, "internal RAM before BLE init: free=%u largest=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    if (!InitNimble()) {
        state_.store(State::kOff);
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "internal RAM after BLE init: free=%u largest=%u min_ever=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    state_.store(State::kIdle);
    next_scan_at_us_.store(0);

    const int64_t hb_period_us = static_cast<int64_t>(MBOT_HB_PERIOD_S * 1e6);
    int64_t last_hb_us = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(kManagerTickMs));
        const int64_t now = NowUs();
        const State st = state_.load();

        if (st == State::kIdle && synced_.load() && now >= next_scan_at_us_.load()) {
            StartScan();
            continue;
        }

        if (st == State::kReady && now - last_hb_us >= hb_period_us) {
            bool gate = true;
            if (require_host_alive_.load()) {
                gate = now - host_alive_us_.load() < static_cast<int64_t>(host_alive_window_ms_.load()) * 1000;
            }
            if (gate != hb_gate_was_open_) {
                if (gate) {
                    ESP_LOGI(TAG, "host link alive again; heartbeat resumed");
                } else {
                    ESP_LOGW(TAG, "host (gateway) link lost; heartbeat paused, the mBot watchdog "
                                  "stops any motion within %.0f s", MBOT_WATCHDOG_S);
                }
                hb_gate_was_open_ = gate;
            }
            if (gate) {
                SendTopic("sk_hb", std::to_string(++hb_counter_));
            }
            last_hb_us = now;
        }
    }
}

void MbotLink::ScheduleReconnect(const char* why) {
    int backoff = backoff_ms_.load();
    next_scan_at_us_.store(NowUs() + static_cast<int64_t>(backoff) * 1000);
    backoff_ms_.store(std::min(backoff * 2, kBackoffMaxMs));
    state_.store(State::kIdle);
    ESP_LOGI(TAG, "next scan in %d ms (%s)", backoff, why);
}

void MbotLink::StartScan() {
    struct ble_gap_disc_params params;
    memset(&params, 0, sizeof(params));
    params.itvl = kScanItvl;
    params.window = kScanWindow;
    params.filter_duplicates = 1;
    params.passive = 0;  // active: the name may only be in the scan response
    state_.store(State::kScanning);
    int rc = ble_gap_disc(own_addr_type_, kScanDurationMs, &params, &MbotLink::GapEvent, this);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_disc failed rc=%d", rc);
        ScheduleReconnect("scan start failed");
        return;
    }
    ESP_LOGI(TAG, "scanning for %s...", kNamePrefix);
}

bool MbotLink::AdvMatches(const struct ble_gap_disc_desc& disc, std::string* name_out) {
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, disc.data, disc.length_data) != 0) {
        return false;
    }
    if (fields.name == nullptr || fields.name_len < sizeof(kNamePrefix) - 1) {
        return false;
    }
    std::string name(reinterpret_cast<const char*>(fields.name), fields.name_len);
    if (name.compare(0, sizeof(kNamePrefix) - 1, kNamePrefix) != 0) {
        return false;
    }
    std::string target;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        target = target_mac_;
    }
    if (!target.empty() && AddrToString(disc.addr) != target) {
        ESP_LOGD(TAG, "ignoring %s (%s): not the pinned mbot.mac", name.c_str(),
                 AddrToString(disc.addr).c_str());
        return false;
    }
    *name_out = name;
    return true;
}

// ------------------------------------------------------------------ GAP

int MbotLink::GapEvent(struct ble_gap_event* event, void* arg) {
    auto* self = static_cast<MbotLink*>(arg);
    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            if (self->state_.load() != State::kScanning) {
                return 0;
            }
            std::string name;
            if (!self->AdvMatches(event->disc, &name)) {
                return 0;
            }
            ble_gap_disc_cancel();
            self->state_.store(State::kConnecting);
            {
                std::lock_guard<std::mutex> lock(self->mutex_);
                self->peer_name_ = name;
                self->peer_addr_ = AddrToString(event->disc.addr);
            }
            ESP_LOGI(TAG, "found %s (%s, rssi %d); connecting", name.c_str(),
                     AddrToString(event->disc.addr).c_str(), event->disc.rssi);
            int rc = ble_gap_connect(self->own_addr_type_, &event->disc.addr, kConnectTimeoutMs, nullptr,
                                     &MbotLink::GapEvent, self);
            if (rc != 0) {
                ESP_LOGW(TAG, "ble_gap_connect failed rc=%d", rc);
                self->ScheduleReconnect("connect failed");
            }
            return 0;
        }
        case BLE_GAP_EVENT_DISC_COMPLETE:
            if (self->state_.load() == State::kScanning) {
                self->ScheduleReconnect("no mBot found");
            }
            return 0;
        case BLE_GAP_EVENT_CONNECT: {
            if (event->connect.status != 0) {
                ESP_LOGW(TAG, "connection failed status=%d", event->connect.status);
                self->ScheduleReconnect("connect failed");
                return 0;
            }
            {
                std::lock_guard<std::mutex> lock(self->mutex_);
                self->conn_handle_ = event->connect.conn_handle;
                self->svc_start_ = self->svc_end_ = 0;
                self->write_handle_ = self->notify_handle_ = self->notify_end_ = self->cccd_handle_ = 0;
                self->last_chr_def_after_notify_ = 0;
            }
            self->decoder_.Reset();
            self->mtu_.store(23);
            self->state_.store(State::kDiscovering);
            ESP_LOGI(TAG, "BLE connected (handle %d); discovering", event->connect.conn_handle);
            int rc = ble_gattc_exchange_mtu(event->connect.conn_handle, &MbotLink::OnMtu, self);
            if (rc != 0) {
                // Carry on with the default MTU; frames are chunked if needed.
                OnMtu(event->connect.conn_handle, nullptr, 23, self);
            }
            return 0;
        }
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGW(TAG, "mBot BLE link lost (reason 0x%x). Nothing to send: the mBot watchdog "
                          "stops any motion within %.0f s", event->disconnect.reason, MBOT_WATCHDOG_S);
            {
                std::lock_guard<std::mutex> lock(self->mutex_);
                self->conn_handle_ = 0xFFFF;
            }
            self->OnLinkDown("disconnected");
            self->ScheduleReconnect("disconnected");
            return 0;
        case BLE_GAP_EVENT_MTU:
            self->mtu_.store(event->mtu.value);
            ESP_LOGI(TAG, "MTU %d", event->mtu.value);
            return 0;
        case BLE_GAP_EVENT_NOTIFY_RX: {
            uint16_t notify_handle;
            {
                std::lock_guard<std::mutex> lock(self->mutex_);
                notify_handle = self->notify_handle_;
            }
            if (event->notify_rx.attr_handle != notify_handle || event->notify_rx.om == nullptr) {
                return 0;
            }
            uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
            uint8_t buf[600];
            if (len > sizeof(buf)) {
                len = sizeof(buf);
            }
            if (os_mbuf_copydata(event->notify_rx.om, 0, len, buf) == 0) {
                self->HandleNotify(buf, len);
            }
            return 0;
        }
        default:
            return 0;
    }
}

void MbotLink::FailDiscovery(const char* why) {
    ESP_LOGE(TAG, "GATT setup failed: %s; disconnecting", why);
    uint16_t conn;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        conn = conn_handle_;
    }
    if (conn != 0xFFFF) {
        ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    } else {
        ScheduleReconnect(why);
    }
}

int MbotLink::OnMtu(uint16_t conn, const struct ble_gatt_error* error, uint16_t mtu, void* arg) {
    auto* self = static_cast<MbotLink*>(arg);
    if (error == nullptr || error->status == 0) {
        self->mtu_.store(mtu);
        ESP_LOGI(TAG, "MTU exchanged: %d", mtu);
    } else {
        ESP_LOGW(TAG, "MTU exchange failed (status %d); using %d", error->status, self->mtu_.load());
    }
    int rc = ble_gattc_disc_svc_by_uuid(conn, &kServiceUuid16.u, &MbotLink::OnService, self);
    if (rc != 0) {
        self->FailDiscovery("service discovery start");
    }
    return 0;
}

int MbotLink::OnService(uint16_t conn, const struct ble_gatt_error* error, const struct ble_gatt_svc* service,
                        void* arg) {
    auto* self = static_cast<MbotLink*>(arg);
    if (error->status == 0 && service != nullptr) {
        std::lock_guard<std::mutex> lock(self->mutex_);
        self->svc_start_ = service->start_handle;
        self->svc_end_ = service->end_handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        self->FailDiscovery("service discovery");
        return 0;
    }
    uint16_t start, end;
    {
        std::lock_guard<std::mutex> lock(self->mutex_);
        start = self->svc_start_;
        end = self->svc_end_;
    }
    if (start == 0) {
        self->FailDiscovery("service FFE1 not found");
        return 0;
    }
    if (ble_gattc_disc_all_chrs(conn, start, end, &MbotLink::OnCharacteristic, self) != 0) {
        self->FailDiscovery("characteristic discovery start");
    }
    return 0;
}

int MbotLink::OnCharacteristic(uint16_t conn, const struct ble_gatt_error* error, const struct ble_gatt_chr* chr,
                               void* arg) {
    auto* self = static_cast<MbotLink*>(arg);
    if (error->status == 0 && chr != nullptr) {
        std::lock_guard<std::mutex> lock(self->mutex_);
        if (self->notify_handle_ != 0 && self->last_chr_def_after_notify_ == 0 &&
            chr->def_handle > self->notify_handle_) {
            self->last_chr_def_after_notify_ = chr->def_handle;
        }
        if (ble_uuid_cmp(&chr->uuid.u, &kWriteUuid16.u) == 0) {
            self->write_handle_ = chr->val_handle;
        } else if (ble_uuid_cmp(&chr->uuid.u, &kNotifyUuid16.u) == 0) {
            self->notify_handle_ = chr->val_handle;
        }
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        self->FailDiscovery("characteristic discovery");
        return 0;
    }
    uint16_t notify, end;
    {
        std::lock_guard<std::mutex> lock(self->mutex_);
        if (self->write_handle_ == 0 || self->notify_handle_ == 0) {
            notify = 0;
            end = 0;
        } else {
            notify = self->notify_handle_;
            end = self->last_chr_def_after_notify_ != 0 ? self->last_chr_def_after_notify_ - 1
                                                        : self->svc_end_;
            self->notify_end_ = end;
        }
    }
    if (notify == 0) {
        self->FailDiscovery("FFE3/FFE2 characteristics not found");
        return 0;
    }
    if (end <= notify) {
        // No room for descriptors reported: assume CCCD right after the value.
        OnDescriptor(conn, nullptr, notify, nullptr, self);
        return 0;
    }
    if (ble_gattc_disc_all_dscs(conn, notify, end, &MbotLink::OnDescriptor, self) != 0) {
        self->FailDiscovery("descriptor discovery start");
    }
    return 0;
}

int MbotLink::OnDescriptor(uint16_t conn, const struct ble_gatt_error* error, uint16_t chr_val_handle,
                           const struct ble_gatt_dsc* dsc, void* arg) {
    auto* self = static_cast<MbotLink*>(arg);
    if (error != nullptr && error->status == 0 && dsc != nullptr) {
        if (ble_uuid_cmp(&dsc->uuid.u, &kCccdUuid16.u) == 0) {
            std::lock_guard<std::mutex> lock(self->mutex_);
            self->cccd_handle_ = dsc->handle;
        }
        return 0;
    }
    if (error != nullptr && error->status != BLE_HS_EDONE) {
        self->FailDiscovery("descriptor discovery");
        return 0;
    }
    uint16_t cccd;
    {
        std::lock_guard<std::mutex> lock(self->mutex_);
        if (self->cccd_handle_ == 0) {
            self->cccd_handle_ = self->notify_handle_ + 1;
            ESP_LOGW(TAG, "no CCCD reported; trying handle %d", self->cccd_handle_);
        }
        cccd = self->cccd_handle_;
    }
    static const uint8_t kEnableNotify[2] = {0x01, 0x00};
    if (ble_gattc_write_flat(conn, cccd, kEnableNotify, sizeof(kEnableNotify), &MbotLink::OnSubscribed, self) != 0) {
        self->FailDiscovery("subscribe start");
    }
    return 0;
}

int MbotLink::OnSubscribed(uint16_t conn, const struct ble_gatt_error* error, struct ble_gatt_attr*, void* arg) {
    auto* self = static_cast<MbotLink*>(arg);
    if (error->status != 0) {
        self->FailDiscovery("enable notifications");
        return 0;
    }
    self->backoff_ms_.store(kBackoffMinMs);
    self->last_rx_us_.store(NowUs());
    self->state_.store(State::kReady);
    std::string name, addr;
    {
        std::lock_guard<std::mutex> lock(self->mutex_);
        name = self->peer_name_;
        addr = self->peer_addr_;
    }
    ESP_LOGI(TAG, "mBot ready: %s (%s), MTU %d", name.c_str(), addr.c_str(), self->mtu_.load());
    self->EmitEvent("connected", name);
    return 0;
}

// ------------------------------------------------------------------ receive

void MbotLink::HandleNotify(const uint8_t* data, size_t len) {
    std::vector<mbot::F3Frame> frames;
    decoder_.Feed(data, len, frames);
    for (const auto& f : frames) {
        if (f.type != mbot::kF3TypeUploadMsg) {
            ESP_LOGD(TAG, "ignoring F3 frame type 0x%02x", f.type);
            continue;
        }
        std::string topic, value;
        if (!mbot::ParseUploadMessage(f.text, &topic, &value)) {
            ESP_LOGW(TAG, "unparseable message: %.80s", f.text.c_str());
            continue;
        }
        last_rx_us_.store(NowUs());
        if (topic == "sk_ack") {
            HandleAck(value);
        } else if (topic == "sk_evt") {
            HandleEvent(value);
        } else {
            ESP_LOGD(TAG, "message %s=%s", topic.c_str(), value.c_str());
        }
    }
}

void MbotLink::HandleAck(const std::string& value) {
    // "<id> ok [detail]" | "<id> err <reason>"
    const char* s = value.c_str();
    char* end = nullptr;
    long id = strtol(s, &end, 10);
    if (end == s) {
        ESP_LOGW(TAG, "bad ack: %s", value.c_str());
        return;
    }
    std::string reply(end);
    size_t b = reply.find_first_not_of(' ');
    size_t e = reply.find_last_not_of(' ');
    reply = b == std::string::npos ? "" : reply.substr(b, e - b + 1);
    ESP_LOGI(TAG, "ack %ld %s", id, reply.c_str());
    std::lock_guard<std::mutex> lock(mutex_);
    for (Pending* p : {&pending_cmd_, &pending_stop_}) {
        if (p->active && !p->done && p->id == id) {
            p->done = true;
            p->reply = reply;
        }
    }
    ack_cv_.notify_all();
}

void MbotLink::HandleEvent(const std::string& value) {
    std::string name = value;
    std::string args;
    size_t sp = value.find(' ');
    if (sp != std::string::npos) {
        name = value.substr(0, sp);
        args = value.substr(sp + 1);
        size_t e = args.find_last_not_of(' ');
        args = e == std::string::npos ? "" : args.substr(0, e + 1);
    }
    if (name == "hb") {
        battery_.store(atoi(args.c_str()));
    } else if (name == "odom") {
        ESP_LOGD(TAG, "odom %s", args.c_str());  // stream rate: keep quiet
    } else {
        ESP_LOGI(TAG, "event %s %s", name.c_str(), args.c_str());
    }
    if (name == "sensors") {
        std::lock_guard<std::mutex> lock(mutex_);
        sensors_.valid = true;
        sensors_.at_us = NowUs();
        sensors_.raw = args;
        ack_cv_.notify_all();
        // batt=<n> also refreshes the battery cache
        size_t p = args.find("batt=");
        if (p != std::string::npos) battery_.store(atoi(args.c_str() + p + 5));
        size_t l = args.find("locked=");
        if (l != std::string::npos) locked_.store(args.compare(l + 7, 1, "1") == 0);
    } else if (name == "done") {
        Done d;
        if (mbot::ParseDoneArgs(args, &d.event)) {
            d.valid = true;
            d.at_us = NowUs();
            d.raw = args;
            std::lock_guard<std::mutex> lock(mutex_);
            done_history_[done_next_] = d;
            done_next_ = (done_next_ + 1) % kDoneHistory;
            last_done_ = d;
        } else {
            ESP_LOGW(TAG, "unparseable done: %s", args.c_str());
        }
    } else if (name == "odom") {
        Odometry o;
        if (mbot::ParseOdomArgs(args, &o.sample)) {
            o.valid = true;
            o.at_us = NowUs();
            std::lock_guard<std::mutex> lock(mutex_);
            odom_ = o;
        }
    } else if (name == "ready") {
        std::lock_guard<std::mutex> lock(mutex_);
        runtime_version_ = args;
        locked_.store(false);
    } else if (name == "locked") {
        locked_.store(true);
    } else if (name == "unlocked") {
        locked_.store(false);
    }
    EmitEvent(name, args);
}

void MbotLink::EmitEvent(const std::string& name, const std::string& args) {
    EventCallback cb;
    {
        std::lock_guard<std::mutex> lock(cb_mutex_);
        cb = event_cb_;
    }
    if (cb) {
        cb(name, args);
    }
}

void MbotLink::OnLinkDown(const char* why) {
    bool was_ready = state_.load() == State::kReady;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (Pending* p : {&pending_cmd_, &pending_stop_}) {
            if (p->active && !p->done) {
                p->done = true;
                p->reply = "__disconnected";
            }
        }
        ack_cv_.notify_all();
    }
    state_.store(State::kIdle);
    if (was_ready) {
        EmitEvent("disconnected", why);
    }
}

// ------------------------------------------------------------------ send

bool MbotLink::WriteFrame(const std::vector<uint8_t>& frame) {
    uint16_t conn, handle;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        conn = conn_handle_;
        handle = write_handle_;
    }
    if (conn == 0xFFFF || handle == 0) {
        return false;
    }
    std::lock_guard<std::mutex> write_lock(write_mutex_);
    const size_t chunk = std::max(20, mtu_.load() - 3);
    for (size_t off = 0; off < frame.size(); off += chunk) {
        const size_t n = std::min(chunk, frame.size() - off);
        int rc = 0;
        for (int attempt = 0; attempt < 20; ++attempt) {
            rc = ble_gattc_write_no_rsp_flat(conn, handle, frame.data() + off, n);
            if (rc != BLE_HS_ENOMEM && rc != BLE_HS_EBUSY) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (rc != 0) {
            ESP_LOGW(TAG, "write failed rc=%d", rc);
            return false;
        }
    }
    return true;
}

bool MbotLink::SendTopic(const char* topic, const std::string& value) {
    return WriteFrame(mbot::EncodeMessage(topic, value));
}

int MbotLink::NextId() {
    int id = next_id_.fetch_add(1);
    if (id > MBOT_MAX_CMD_ID || id < 1) {
        next_id_.store(2);
        id = 1;
    }
    return id;
}

MbotLink::Result MbotLink::Transact(int id, const std::string& text, int ack_timeout_ms, bool is_stop,
                                    uint32_t generation) {
    Result r;
    r.id = id;
    r.sent = text;
    Pending& slot = is_stop ? pending_stop_ : pending_cmd_;
    const std::string payload = std::to_string(id) + " " + text;

    for (int attempt = 0; attempt < 2; ++attempt) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!is_stop && stop_generation_ != generation) {
                r.status = "cancelled";
                r.detail = "a stop was sent";
                return r;
            }
            slot.id = id;
            slot.active = true;
            slot.done = false;
            slot.reply.clear();
        }
        if (attempt > 0) {
            ESP_LOGW(TAG, "no ack for %d, retrying once with the same id", id);
        }
        const int64_t sent_us = NowUs();
        if (!SendTopic("sk", payload)) {
            std::lock_guard<std::mutex> lock(mutex_);
            slot.active = false;
            r.status = IsConnected() ? "send_failed" : "not_connected";
            return r;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        ack_cv_.wait_for(lock, std::chrono::milliseconds(ack_timeout_ms), [&] { return slot.done; });
        if (slot.done) {
            slot.active = false;
            const std::string reply = slot.reply;
            if (reply.compare(0, 2, "__") != 0) {
                r.rtt_ms = static_cast<int>((NowUs() - sent_us) / 1000);
                last_rtt_ms_.store(r.rtt_ms);
            }
            if (reply == "__cancelled") {
                r.status = "cancelled";
                r.detail = "a stop was sent";
            } else if (reply == "__disconnected") {
                r.status = "disconnected";
            } else if (reply.compare(0, 2, "ok") == 0) {
                r.ok = true;
                r.status = "ok";
                r.detail = reply.size() > 3 ? reply.substr(3) : "";
            } else {
                r.status = "err";
                r.detail = reply.compare(0, 3, "err") == 0 ? (reply.size() > 4 ? reply.substr(4) : "") : reply;
            }
            return r;
        }
        slot.active = false;
        lock.unlock();
        if (!IsConnected()) {
            r.status = "disconnected";
            return r;
        }
    }
    r.status = "timeout";
    return r;
}

MbotLink::Result MbotLink::SendCommand(const std::string& cmd, int ack_timeout_ms) {
    mbot::CommandCheck check = mbot::CheckCommand(cmd);
    if (check.ok && check.is_stop) {
        return Stop("cmd");
    }
    Result r;
    if (!check.ok) {
        r.status = "invalid";
        r.detail = check.error;
        return r;
    }
    r.sent = check.normalized;
    r.clamped = check.clamped;
    r.expected_seconds = check.seconds;
    if (!IsConnected()) {
        r.status = "not_connected";
        return r;
    }

    std::lock_guard<std::mutex> serial(cmd_mutex_);  // one sk command in flight
    uint32_t generation;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        generation = stop_generation_;
    }
    Result t = Transact(NextId(), check.normalized, ack_timeout_ms, false, generation);
    t.clamped = check.clamped;
    t.expected_seconds = check.seconds;

    // Safety: a motion command that failed in an unknown way may or may not
    // have started. Make sure it did not.
    if (check.is_motion && !t.ok &&
        (t.status == "err" || t.status == "timeout" || t.status == "send_failed")) {
        ESP_LOGW(TAG, "motion '%s' failed (%s %s); sending stop", check.normalized.c_str(), t.status.c_str(),
                 t.detail.c_str());
        Stop("error");
    }
    return t;
}

MbotLink::Result MbotLink::Stop(const char* reason) {
    Result r;
    r.sent = "stop";
    if (!IsConnected()) {
        ESP_LOGW(TAG, "stop (%s) requested but mBot not connected; its watchdog / buttons are in charge",
                 reason);
        r.status = "not_connected";
        return r;
    }
    std::lock_guard<std::mutex> serial(stop_mutex_);
    uint32_t generation;
    {
        // Cancel whatever command is waiting for its ack: it must not be
        // retried after the stop (that would restart the motion).
        std::lock_guard<std::mutex> lock(mutex_);
        ++stop_generation_;
        generation = stop_generation_;
        if (pending_cmd_.active && !pending_cmd_.done) {
            pending_cmd_.done = true;
            pending_cmd_.reply = "__cancelled";
        }
        ack_cv_.notify_all();
    }
    ESP_LOGI(TAG, "stop (%s)", reason);
    return Transact(NextId(), "stop", MBOT_ACK_TIMEOUT_MS, true, generation);
}

MbotLink::Result MbotLink::ReadSensors(Sensors* out, int wait_ms) {
    const int64_t t0 = NowUs();
    Result r = SendCommand("read");
    if (r.ok) {
        std::unique_lock<std::mutex> lock(mutex_);
        ack_cv_.wait_for(lock, std::chrono::milliseconds(wait_ms),
                         [&] { return sensors_.valid && sensors_.at_us >= t0; });
        if (out != nullptr) {
            *out = sensors_;
        }
        if (!(sensors_.valid && sensors_.at_us >= t0)) {
            r.ok = false;
            r.status = "timeout";
            r.detail = "no sensors event";
        }
    }
    return r;
}

// ------------------------------------------------------------------ misc

void MbotLink::SetEventCallback(EventCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    event_cb_ = std::move(cb);
}

void MbotLink::RequireHostAlive(bool required, int host_alive_window_ms) {
    host_alive_window_ms_.store(host_alive_window_ms);
    require_host_alive_.store(required);
}

void MbotLink::NoteHostAlive() {
    host_alive_us_.store(NowUs());
}

std::string MbotLink::PeerAddress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return peer_addr_;
}

std::string MbotLink::PeerName() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return peer_name_;
}

std::string MbotLink::TargetMac() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_mac_;
}

std::string MbotLink::RuntimeVersion() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return runtime_version_;
}

MbotLink::Sensors MbotLink::LastSensors() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sensors_;
}

int64_t MbotLink::LastRxAgeMs() const {
    int64_t t = last_rx_us_.load();
    return t == 0 ? -1 : (NowUs() - t) / 1000;
}

MbotLink::Done MbotLink::LastDone() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_done_;
}

bool MbotLink::FindDone(int id, Done* out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 1; i <= kDoneHistory; ++i) {  // newest first
        const Done& d = done_history_[(done_next_ - i + kDoneHistory) % kDoneHistory];
        if (d.valid && d.event.id == id) {
            if (out != nullptr) *out = d;
            return true;
        }
    }
    return false;
}

MbotLink::Odometry MbotLink::LastOdometry() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return odom_;
}
