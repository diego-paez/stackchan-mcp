// SPDX-License-Identifier: MIT
//
// See mbot_teleop.h. One task, one UDP socket:
//   - drain every waiting packet and keep only the newest (latest wins),
//   - renew the mBot `drive` lease at 10 Hz while the joystick is enabled,
//   - stop the mBot when packets stop for 300 ms (dead-man) or on E-stop,
//   - answer HELLO and every 4th CMD_VEL with a STATUS for the joystick screen.
// The mBot runtime also stops by itself when a lease runs out, so a stuck
// Stack-chan cannot leave the wheels turning.

#include "mbot_teleop.h"

#include <sdkconfig.h>

#if CONFIG_STACKCHAN_MBOT_LINK && CONFIG_STACKCHAN_MBOT_TELEOP

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/idf_additions.h>
#include <freertos/task.h>
#include <lwip/sockets.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "application.h"
#include "mbot_limits.h"
#include "mbot_link.h"
#include "teleop_packet.h"

namespace {

constexpr char TAG[] = "MbotTeleop";

constexpr int64_t kDeadmanUs = 300 * 1000;         // no packet this long -> stop
constexpr int64_t kOwnerTimeoutUs = 1000 * 1000;   // another joystick may take over after this
constexpr int64_t kDriveEveryUs = 100 * 1000;      // lease renewals to the mBot (10 Hz)
constexpr int kDriveLeaseMs = 300;
constexpr int64_t kServoEveryUs = 150 * 1000;
constexpr int kServoStepDeg = 8;                   // ~50 deg/s while a button is held
constexpr int64_t kSensorsEveryUs = 1000 * 1000;
constexpr int kStatusEvery = 4;                    // STATUS for every 4th CMD_VEL
constexpr int kTaskStack = 4096;

std::atomic<bool> g_gateway_up{false};

int64_t NowUs() { return esp_timer_get_time(); }

int Clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// "dist=12 batt=90 arm=90 grip=90 ..." -> value of `key`, or `fallback`.
int RawInt(const std::string& raw, const char* key, int fallback) {
    std::string k = std::string(key) + "=";
    size_t pos = raw.find(k);
    if (pos == std::string::npos || (pos > 0 && raw[pos - 1] != ' ')) return fallback;
    const char* s = raw.c_str() + pos + k.size();
    char* end = nullptr;
    long v = std::strtol(s, &end, 10);
    return end == s ? fallback : static_cast<int>(v);
}

bool WifiHasIp() {
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip = {};
    return netif != nullptr && esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0;
}

struct Session {
    bool has_owner = false;
    sockaddr_in owner = {};
    uint32_t last_seq = 0;
    int64_t last_rx_us = 0;
    teleop::CmdVel cmd;
    uint16_t prev_buttons = 0;
    int cmd_count = 0;

    bool active = false;          // wheels are being driven by the joystick
    int64_t last_drive_us = 0;
    int64_t last_servo_us = 0;
    int64_t last_sensors_us = 0;
    int arm = MBOT_ARM_HOME;
    int grip = MBOT_GRIP_HOME;
    uint32_t status_seq = 0;
};

bool SameSender(const sockaddr_in& a, const sockaddr_in& b) {
    return a.sin_addr.s_addr == b.sin_addr.s_addr && a.sin_port == b.sin_port;
}

teleop::Status BuildStatus(bool active) {
    auto& link = MbotLink::GetInstance();
    teleop::Status st;
    st.flags = (link.IsConnected() ? teleop::kMbotConnected : 0) | (link.Locked() ? teleop::kMbotLocked : 0) |
               (active ? teleop::kTeleopActive : 0) | (g_gateway_up.load() ? teleop::kGatewayConnected : 0);
    const int batt = link.BatteryPercent();
    st.battery_pct = batt < 0 ? 0xFF : static_cast<uint8_t>(Clampi(batt, 0, 100));
    MbotLink::Sensors s = link.LastSensors();
    if (s.valid) {
        const int dist = RawInt(s.raw, "dist", -1);
        st.dist_cm = dist < 0 ? 0xFFFF : static_cast<uint16_t>(Clampi(dist, 0, 0xFFFE));
        st.arm_deg = static_cast<uint8_t>(Clampi(RawInt(s.raw, "arm", 0), 0, 255));
        st.grip_deg = static_cast<uint8_t>(Clampi(RawInt(s.raw, "grip", 0), 0, 255));
    }
    const int rtt = link.LastRttMs();
    st.rtt_ms = rtt < 0 ? 0xFFFF : static_cast<uint16_t>(Clampi(rtt, 0, 0xFFFE));
    return st;
}

void SendStatus(int sock, const sockaddr_in& to, uint8_t type, Session* ss) {
    uint8_t buf[teleop::kStatusSize];
    size_t n = teleop::EncodeStatus(type, ++ss->status_seq, CONFIG_STACKCHAN_MBOT_TELEOP_KEY,
                                    BuildStatus(ss->active), buf, sizeof(buf));
    sendto(sock, buf, n, 0, reinterpret_cast<const sockaddr*>(&to), sizeof(to));
}

void Notify(const char* what) {
    Application::GetInstance().SendStackChanEvent("mbot", "teleop", 0, what);
}

// Handle one datagram. Returns true if it was a valid packet from the owner.
bool HandlePacket(int sock, const uint8_t* data, size_t len, const sockaddr_in& from, Session* ss) {
    teleop::Header hdr;
    if (!teleop::ParseHeader(data, len, &hdr) || hdr.key != CONFIG_STACKCHAN_MBOT_TELEOP_KEY) return false;
    const int64_t now = NowUs();
    if (hdr.type == teleop::kHello) {
        SendStatus(sock, from, teleop::kHelloAck, ss);
        return false;
    }
    teleop::CmdVel cmd;
    if (!teleop::ParseCmdVel(data, len, &hdr, &cmd)) return false;

    const bool owner_alive = ss->has_owner && now - ss->last_rx_us < kOwnerTimeoutUs;
    if (owner_alive && !SameSender(ss->owner, from)) return false;   // someone else is driving
    if (owner_alive && !teleop::SeqNewer(hdr.seq, ss->last_seq)) return false;  // late / duplicate
    if (!owner_alive) {
        ss->has_owner = true;
        ss->owner = from;
        ESP_LOGI(TAG, "joystick %s:%u", inet_ntoa(from.sin_addr), ntohs(from.sin_port));
    }
    ss->last_seq = hdr.seq;
    ss->last_rx_us = now;
    ss->cmd = cmd;
    if (++ss->cmd_count % kStatusEvery == 0) SendStatus(sock, from, teleop::kStatus, ss);
    MbotLink::GetInstance().NoteHostAlive();   // keeps the mBot heartbeat going without the Mac
    return true;
}

void Step(Session* ss) {
    auto& link = MbotLink::GetInstance();
    const int64_t now = NowUs();
    const bool fresh = ss->has_owner && now - ss->last_rx_us < kDeadmanUs;
    const uint16_t buttons = fresh ? ss->cmd.buttons : 0;
    const uint16_t pressed = buttons & ~ss->prev_buttons;
    ss->prev_buttons = buttons;

    if (pressed & teleop::kEstop) {
        link.Stop("teleop_estop");
        ESP_LOGW(TAG, "E-stop from joystick");
        Notify("estop");
    }
    const bool enabled = fresh && (buttons & teleop::kEnable) && !(buttons & teleop::kEstop);

    if (!enabled) {
        if (ss->active) {
            ss->active = false;
            link.Stop(fresh ? "teleop_off" : "teleop_deadman");
            ESP_LOGI(TAG, "teleop %s", fresh ? "off" : "dead-man stop");
            Notify(fresh ? "off" : "deadman");
        }
        return;
    }
    if (!link.IsConnected()) return;

    if (!ss->active) {
        ss->active = true;
        MbotLink::Sensors s = link.LastSensors();
        ss->arm = s.valid ? Clampi(RawInt(s.raw, "arm", MBOT_ARM_HOME), MBOT_ARM_MIN, MBOT_ARM_MAX) : MBOT_ARM_HOME;
        ss->grip = s.valid ? Clampi(RawInt(s.raw, "grip", MBOT_GRIP_HOME), MBOT_GRIP_MIN, MBOT_GRIP_MAX)
                           : MBOT_GRIP_HOME;
        ss->last_drive_us = 0;
        ESP_LOGI(TAG, "teleop on");
        Notify("on");
    }

    if (now - ss->last_drive_us >= kDriveEveryUs) {
        ss->last_drive_us = now;
        const int lease = Clampi(ss->cmd.lease_ms ? ss->cmd.lease_ms : kDriveLeaseMs, 50, kDriveLeaseMs);
        char text[48];
        snprintf(text, sizeof(text), "drive %.1f %.0f %d", teleop::LinearCmPerS(ss->cmd),
                 teleop::AngularDegPerS(ss->cmd), lease);
        link.SendCommand(text, 400);   // validated and clamped by CheckCommand
    }

    if (pressed & teleop::kHome) {
        ss->arm = MBOT_ARM_HOME;
        ss->grip = MBOT_GRIP_HOME;
        link.SendCommand("home", 400);
    } else if ((buttons & (teleop::kArmUp | teleop::kArmDown | teleop::kGripOpen | teleop::kGripClose)) &&
               now - ss->last_servo_us >= kServoEveryUs) {
        ss->last_servo_us = now;
        char text[24];
        if (buttons & (teleop::kArmUp | teleop::kArmDown)) {
            const int arm = Clampi(ss->arm + ((buttons & teleop::kArmUp) ? kServoStepDeg : -kServoStepDeg),
                                   MBOT_ARM_MIN, MBOT_ARM_MAX);
            if (arm != ss->arm) {
                ss->arm = arm;
                snprintf(text, sizeof(text), "arm %d", arm);
                link.SendCommand(text, 400);
            }
        }
        if (buttons & (teleop::kGripOpen | teleop::kGripClose)) {
            const int grip = Clampi(ss->grip + ((buttons & teleop::kGripOpen) ? kServoStepDeg : -kServoStepDeg),
                                    MBOT_GRIP_MIN, MBOT_GRIP_MAX);
            if (grip != ss->grip) {
                ss->grip = grip;
                snprintf(text, sizeof(text), "grip %d", grip);
                link.SendCommand(text, 400);
            }
        }
    }

    if (now - ss->last_sensors_us >= kSensorsEveryUs) {   // distance and battery for the screen
        ss->last_sensors_us = now;
        MbotLink::Sensors s;
        link.ReadSensors(&s, 300);
    }
}

void TeleopTask(void*) {
    while (!WifiHasIp()) vTaskDelay(pdMS_TO_TICKS(1000));

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(CONFIG_STACKCHAN_MBOT_TELEOP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (sock < 0 || bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "UDP socket/bind failed on port %d", CONFIG_STACKCHAN_MBOT_TELEOP_PORT);
        if (sock >= 0) close(sock);
        vTaskDelete(nullptr);
        return;
    }
    timeval tv = {0, 50 * 1000};   // 50 ms: the loop also runs the dead-man check
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ESP_LOGI(TAG, "listening on UDP %d; internal RAM free=%u", CONFIG_STACKCHAN_MBOT_TELEOP_PORT,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    Session ss;
    uint8_t buf[64];
    while (true) {
        sockaddr_in from = {};
        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &from_len);
        while (n > 0) {   // latest wins: drain everything that is waiting
            HandlePacket(sock, buf, static_cast<size_t>(n), from, &ss);
            from_len = sizeof(from);
            n = recvfrom(sock, buf, sizeof(buf), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&from), &from_len);
        }
        Step(&ss);
    }
}

}  // namespace

void StartMbotTeleop() {
    static bool started = false;
    if (started) return;
    started = true;
    const unsigned before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    // Stack in PSRAM: internal RAM is what BLE and Wi-Fi are short of.
    if (xTaskCreatePinnedToCoreWithCaps(TeleopTask, "mbot_teleop", kTaskStack, nullptr, 4, nullptr, tskNO_AFFINITY,
                                        MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return;
    }
    ESP_LOGI(TAG, "task started; internal RAM %u -> %u", before,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

void MbotTeleopSetGatewayUp(bool up) { g_gateway_up.store(up); }

#else

void StartMbotTeleop() {}
void MbotTeleopSetGatewayUp(bool) {}

#endif
