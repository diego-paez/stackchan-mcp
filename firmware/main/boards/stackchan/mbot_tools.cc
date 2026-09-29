// SPDX-License-Identifier: MIT
//
// self.mbot.* MCP tools for the stackchan board. See mbot_tools.h.
//
// Safety model (a child uses this robot):
//   * every argument is range-checked by the MCP schema, then validated and
//     clamped again by mbot::CheckCommand() against mbot_limits.h, and the
//     mBot runtime clamps a third time on its side;
//   * tools return as soon as the mBot acknowledges (never wait for the
//     motion to finish), so a stop is never stuck behind a move;
//   * a motion whose ack is an error or never arrives is followed by stop;
//   * the heartbeat only runs while the gateway WebSocket is up, so losing
//     the LLM side lets the mBot watchdog stop everything within 3 s.

#include "mbot_tools.h"

#include <sdkconfig.h>

#if CONFIG_STACKCHAN_MBOT_LINK

#include <cJSON.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <cstdlib>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "application.h"
#include "mbot_limits.h"
#include "mbot_link.h"
#include "mbot_teleop.h"
#include "mcp_server.h"

namespace {

const char* TAG = "MbotTools";

// Local reflexes: mBot event -> Stack-chan face, no LLM round trip.
// Keep this table tiny; set CONFIG_STACKCHAN_MBOT_REFLEXES=n to disable.
struct Reflex {
    const char* event;
    const char* face;
};
constexpr Reflex kReflexes[] = {
    {"obstacle", "surprised"},
};

// Events not forwarded to the gateway: hb is periodic noise (battery is in
// self.mbot.status), sensors is answered by self.mbot.read_sensors, and odom
// streams at up to 10 Hz (cached; read it with self.mbot.odometry).
bool ForwardEvent(const std::string& name) {
    return name != "hb" && name != "sensors" && name != "odom";
}

void AddOptNumber(cJSON* obj, const char* key, const std::optional<double>& v) {
    if (v) {
        cJSON_AddNumberToObject(obj, key, *v);
    } else {
        cJSON_AddNullToObject(obj, key);
    }
}

void AddOptInt(cJSON* obj, const char* key, const std::optional<int64_t>& v) {
    if (v) {
        cJSON_AddNumberToObject(obj, key, static_cast<double>(*v));
    } else {
        cJSON_AddNullToObject(obj, key);
    }
}

double AgeMs(int64_t at_us) {
    return static_cast<double>((esp_timer_get_time() - at_us) / 1000);
}

// null when no done has arrived yet.
cJSON* DoneJson(const MbotLink::Done& d) {
    if (!d.valid) return cJSON_CreateNull();
    // Keys match the mBot's own event fields: id (string), reason, dist (cm),
    // yaw (deg), t (mbot ms); "none" -> null.
    cJSON* o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "id", std::to_string(d.event.id).c_str());
    cJSON_AddStringToObject(o, "reason", d.event.reason.c_str());
    AddOptNumber(o, "dist", d.event.dist_cm);
    AddOptNumber(o, "yaw", d.event.yaw_deg);
    AddOptInt(o, "t", d.event.mbot_ms);
    cJSON_AddNumberToObject(o, "age_ms", AgeMs(d.at_us));
    return o;
}

cJSON* OdomJson(const MbotLink::Odometry& od) {
    if (!od.valid) return cJSON_CreateNull();
    cJSON* o = cJSON_CreateObject();
    AddOptInt(o, "t", od.sample.mbot_ms);
    AddOptNumber(o, "l", od.sample.left_cm);
    AddOptNumber(o, "r", od.sample.right_cm);
    AddOptNumber(o, "yaw", od.sample.yaw_deg);
    cJSON_AddNumberToObject(o, "age_ms", AgeMs(od.at_us));
    return o;
}

cJSON* ResultJson(const MbotLink::Result& r) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", r.ok);
    cJSON_AddStringToObject(root, "status", r.status.c_str());
    // Contract used by clients (mblock-stacky-bridge GatewayRobot): "id" is the
    // command id as sent to the mBot, as a string; "detail" is always present
    // (ack detail, mBot refusal reason such as "locked", or the error).
    if (r.id != 0) cJSON_AddStringToObject(root, "id", std::to_string(r.id).c_str());
    cJSON_AddStringToObject(root, "detail",
                            (r.detail.empty() && !r.ok ? r.status : r.detail).c_str());
    if (!r.sent.empty()) cJSON_AddStringToObject(root, "sent", r.sent.c_str());
    if (!r.clamped.empty()) cJSON_AddStringToObject(root, "clamped", r.clamped.c_str());
    if (r.expected_seconds > 0) cJSON_AddNumberToObject(root, "expected_seconds", r.expected_seconds);
    if (r.rtt_ms >= 0) cJSON_AddNumberToObject(root, "rtt_ms", r.rtt_ms);
    if (r.status == "not_connected") {
        cJSON_AddStringToObject(root, "hint",
                                "The mBot is not connected over Bluetooth. Make sure it is switched on "
                                "and running the Stacky program; Stack-chan reconnects automatically.");
    } else if (r.status == "err" && r.detail == "locked") {
        cJSON_AddStringToObject(root, "hint",
                                "The mBot is locked by its B button (emergency stop). A person must "
                                "press button A on the mBot to unlock it.");
    } else if (r.ok && r.expected_seconds > 0) {
        cJSON_AddStringToObject(root, "note",
                                "Accepted. The motion runs on the mBot; an mbot 'done' event follows "
                                "when it finishes. self.mbot.stop halts it at any time.");
    }
    return root;
}

MbotLink::Result Run(const std::string& cmd) {
    auto& link = MbotLink::GetInstance();
    link.NoteHostAlive();  // a tool call proves the gateway is there
    MbotLink::Result r = link.SendCommand(cmd);
    ESP_LOGI(TAG, "'%s' -> %s %s", cmd.c_str(), r.status.c_str(), r.detail.c_str());
    return r;
}

// "up|down|home" (arm) / "open|close|home" (gripper) or an angle.
std::string ServoArg(const PropertyList& p, const char* const* presets, int npresets) {
    const std::string pos = p["position"].value<std::string>();
    const bool angle_given = p["angle"].was_provided();
    if (!pos.empty() && angle_given) {
        throw std::invalid_argument("Give either position or angle, not both");
    }
    if (angle_given) {
        return std::to_string(p["angle"].value<int>());
    }
    for (int i = 0; i < npresets; ++i) {
        if (pos == presets[i]) return pos;
    }
    std::string allowed;
    for (int i = 0; i < npresets; ++i) allowed += (i ? "|" : "") + std::string(presets[i]);
    throw std::invalid_argument("position must be one of " + allowed + " (or give angle)");
}

cJSON* SensorsJson(const MbotLink::Sensors& s) {
    cJSON* root = cJSON_CreateObject();
    std::istringstream is(s.raw);
    std::string kv;
    while (is >> kv) {
        size_t eq = kv.find('=');
        if (eq == std::string::npos) continue;
        std::string k = kv.substr(0, eq);
        std::string v = kv.substr(eq + 1);
        const char* key = k == "dist" ? "distance_cm"
                          : k == "batt" ? "battery_percent"
                          : k == "arm" ? "arm_deg"
                          : k == "grip" ? "gripper_deg"
                          : k == "l" ? "left_wheel_cm"
                          : k == "r" ? "right_wheel_cm"
                          : k == "yaw" ? "yaw_deg"
                          : k == "t" ? "mbot_ms"
                          : k.c_str();
        if (k == "dist") {
            // Also as top-level "dist" (numeric or null), as clients expect.
            char* end = nullptr;
            double d = strtod(v.c_str(), &end);
            if (v != "none" && !v.empty() && end != v.c_str() && *end == '\0') {
                cJSON_AddNumberToObject(root, "dist", d);
            } else {
                cJSON_AddNullToObject(root, "dist");
            }
        }
        if (k == "locked") {
            cJSON_AddBoolToObject(root, "locked", v == "1");
        } else if (v == "none" || v.empty()) {
            cJSON_AddNullToObject(root, key);
        } else {
            char* end = nullptr;
            double d = strtod(v.c_str(), &end);
            if (end != v.c_str() && *end == '\0') {
                cJSON_AddNumberToObject(root, key, d);
            } else {
                cJSON_AddStringToObject(root, key, v.c_str());
            }
        }
    }
    cJSON_AddStringToObject(root, "raw", s.raw.c_str());
    return root;
}

std::function<void()> g_keep_awake;

void HostAliveTick(void*) {
    // Runs in the esp_timer task; the gateway state is read on the main task.
    Application::GetInstance().Schedule([]() {
        const bool gateway_up = !Application::GetInstance().GetConnectedGatewayUrl().empty();
        MbotTeleopSetGatewayUp(gateway_up);
        if (gateway_up) {
            MbotLink::GetInstance().NoteHostAlive();
        }
        if (g_keep_awake && MbotLink::GetInstance().IsConnected()) {
            g_keep_awake();
        }
    });
}

}  // namespace

void RegisterMbotTools(McpServer& mcp_server, MbotBoardHooks hooks) {
    auto& link = MbotLink::GetInstance();
    g_keep_awake = hooks.keep_awake;

    // ---- events: mBot -> gateway, plus local reflexes ----
    link.SetEventCallback([hooks](const std::string& name, const std::string& args) {
        // NimBLE host task: only post work from here.
        if (ForwardEvent(name)) {
            Application::GetInstance().SendStackChanEvent("mbot", name.c_str(), 0, args);
        }
#if CONFIG_STACKCHAN_MBOT_REFLEXES
        if (hooks.show_reaction_face) {
            for (const auto& reflex : kReflexes) {
                if (name == reflex.event) {
                    const char* face = reflex.face;
                    auto show = hooks.show_reaction_face;
                    Application::GetInstance().Schedule([show, face]() { show(face); });
                }
            }
        }
#else
        (void)hooks;
#endif
    });

    // ---- heartbeat gate: only while the gateway WebSocket is up ----
    link.RequireHostAlive(true, 4000);
    esp_timer_handle_t timer = nullptr;
    esp_timer_create_args_t args = {};
    args.callback = &HostAliveTick;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "mbot_host_alive";
    args.skip_unhandled_events = true;
    if (esp_timer_create(&args, &timer) == ESP_OK) {
        esp_timer_start_periodic(timer, 1000 * 1000);
    } else {
        ESP_LOGW(TAG, "host-alive timer failed; heartbeat runs only after tool calls");
    }

    // ---- tools ----
    char limits[512];
    snprintf(limits, sizeof(limits),
             "Safety limits (enforced on Stack-chan and again on the mBot): speed 0-100%% of %d RPM "
             "(about 20 cm/s at 100%%); each move lasts 0.1-%.0f s; arm %d-%d deg; gripper %d-%d deg "
             "(%d closed, %d open); one step max %d cm, one turn max %d deg; programs max %d steps "
             "and %.0f s. Forward motion auto-stops below %d cm. self.mbot.stop is always available "
             "and always accepted.",
             MBOT_MAX_RPM, MBOT_MAX_MOVE_S, MBOT_ARM_MIN, MBOT_ARM_MAX, MBOT_GRIP_MIN, MBOT_GRIP_MAX,
             MBOT_GRIP_MIN, MBOT_GRIP_MAX, MBOT_MAX_STEP_CM, MBOT_MAX_TURN_DEG, MBOT_MAX_PROG_STEPS,
             MBOT_MAX_PROG_S, MBOT_OBSTACLE_CM);
    const std::string kLimits(limits);

    mcp_server.AddTool(
        "self.mbot.status",
        "Get the state of the Bluetooth link to the Makeblock mBot2 robot car: connected, "
        "robot name/address, Stacky runtime version, battery percent, whether it is locked "
        "(its B button is an emergency stop that locks it; button A unlocks), and the safety "
        "limits. " + kLimits,
        PropertyList(),
        [](const PropertyList&) -> ReturnValue {
            auto& link = MbotLink::GetInstance();
            link.NoteHostAlive();
            cJSON* root = cJSON_CreateObject();
            cJSON_AddBoolToObject(root, "connected", link.IsConnected());
            cJSON_AddStringToObject(root, "state", MbotLink::StateName(link.GetState()));
            cJSON_AddStringToObject(root, "name", link.PeerName().c_str());
            cJSON_AddStringToObject(root, "address", link.PeerAddress().c_str());
            std::string target = link.TargetMac();
            if (target.empty()) {
                cJSON_AddNullToObject(root, "pinned_mac");
            } else {
                cJSON_AddStringToObject(root, "pinned_mac", target.c_str());
            }
            cJSON_AddStringToObject(root, "runtime_version", link.RuntimeVersion().c_str());
            int batt = link.BatteryPercent();
            if (batt >= 0) {
                cJSON_AddNumberToObject(root, "battery_percent", batt);
            } else {
                cJSON_AddNullToObject(root, "battery_percent");
            }
            cJSON_AddBoolToObject(root, "locked", link.Locked());
            cJSON_AddNumberToObject(root, "mtu", link.Mtu());
            cJSON_AddNumberToObject(root, "last_rx_age_ms", static_cast<double>(link.LastRxAgeMs()));
            cJSON* lim = cJSON_CreateObject();
            cJSON_AddNumberToObject(lim, "max_rpm", MBOT_MAX_RPM);
            cJSON_AddNumberToObject(lim, "max_move_s", MBOT_MAX_MOVE_S);
            cJSON_AddNumberToObject(lim, "max_program_s", MBOT_MAX_PROG_S);
            cJSON_AddNumberToObject(lim, "max_program_steps", MBOT_MAX_PROG_STEPS);
            cJSON_AddNumberToObject(lim, "max_step_cm", MBOT_MAX_STEP_CM);
            cJSON_AddNumberToObject(lim, "max_turn_deg", MBOT_MAX_TURN_DEG);
            cJSON_AddNumberToObject(lim, "arm_min", MBOT_ARM_MIN);
            cJSON_AddNumberToObject(lim, "arm_max", MBOT_ARM_MAX);
            cJSON_AddNumberToObject(lim, "gripper_min", MBOT_GRIP_MIN);
            cJSON_AddNumberToObject(lim, "gripper_max", MBOT_GRIP_MAX);
            cJSON_AddNumberToObject(lim, "obstacle_cm", MBOT_OBSTACLE_CM);
            cJSON_AddNumberToObject(lim, "watchdog_s", MBOT_WATCHDOG_S);
            cJSON_AddItemToObject(root, "limits", lim);
            return root;
        });

    mcp_server.AddTool(
        "self.mbot.move",
        "Drive the mBot2 robot car. direction: forward | backward | left | right (left/right "
        "turn in place). speed_percent 0-100 (default 40). duration_ms 100-5000 (default 1000); "
        "one move never lasts longer than 5 s. Returns as soon as the mBot accepts the command; "
        "the motion then runs on its own and a new move replaces the current one. " + kLimits,
        PropertyList({
            Property("direction", kPropertyTypeString),
            Property("speed_percent", kPropertyTypeInteger, MBOT_DEFAULT_SPEED_PCT, 0, 100),
            Property("duration_ms", kPropertyTypeInteger, 1000, static_cast<int>(MBOT_MIN_MOVE_S * 1000),
                     static_cast<int>(MBOT_MAX_MOVE_S * 1000)),
        }),
        [](const PropertyList& p) -> ReturnValue {
            const std::string dir = p["direction"].value<std::string>();
            const char* cmd = dir == "forward" ? "fwd"
                              : dir == "backward" ? "back"
                              : dir == "left" ? "left"
                              : dir == "right" ? "right"
                              : nullptr;
            if (cmd == nullptr) {
                throw std::invalid_argument("direction must be forward, backward, left or right");
            }
            char buf[64];
            snprintf(buf, sizeof(buf), "%s %d %.2f", cmd, p["speed_percent"].value<int>(),
                     p["duration_ms"].value<int>() / 1000.0);
            return ResultJson(Run(buf));
        });

    mcp_server.AddTool(
        "self.mbot.stop",
        "Stop the mBot2 immediately: wheels stop, any running program is cancelled and queued "
        "commands are dropped. Always available and always accepted, even when the mBot is "
        "locked. Use it whenever anything looks wrong.",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue {
            auto& link = MbotLink::GetInstance();
            link.NoteHostAlive();
            return ResultJson(link.Stop("tool"));
        });

    mcp_server.AddTool(
        "self.mbot.set_led",
        "Set the colour of the mBot2's CyberPi LEDs. r, g, b: 0-255. index: \"all\" (default) or "
        "\"1\"..\"5\" for a single LED.",
        PropertyList({
            Property("r", kPropertyTypeInteger, 0, 255),
            Property("g", kPropertyTypeInteger, 0, 255),
            Property("b", kPropertyTypeInteger, 0, 255),
            Property("index", kPropertyTypeString, std::string("all")),
        }),
        [](const PropertyList& p) -> ReturnValue {
            std::string index = p["index"].value<std::string>();
            if (index != "all" && !(index.size() == 1 && index[0] >= '1' && index[0] <= '5')) {
                throw std::invalid_argument("index must be \"all\" or \"1\"..\"5\"");
            }
            char buf[48];
            snprintf(buf, sizeof(buf), "led %d %d %d %s", p["r"].value<int>(), p["g"].value<int>(),
                     p["b"].value<int>(), index.c_str());
            return ResultJson(Run(buf));
        });

    static const char* const kArmPresets[] = {"up", "down", "home"};
    mcp_server.AddTool(
        "self.mbot.arm",
        "Move the mBot2 arm (servo on port S4). Give position (up = 120 deg, down = 40 deg, "
        "home = 90 deg) or angle 40-120. The servo glides at 90 deg/s, it never jumps.",
        PropertyList({
            Property("position", kPropertyTypeString, std::string()),
            Property("angle", kPropertyTypeInteger, MBOT_ARM_HOME, MBOT_ARM_MIN, MBOT_ARM_MAX),
        }),
        [](const PropertyList& p) -> ReturnValue {
            return ResultJson(Run("arm " + ServoArg(p, kArmPresets, 3)));
        });

    static const char* const kGripPresets[] = {"open", "close", "home"};
    mcp_server.AddTool(
        "self.mbot.gripper",
        "Move the mBot2 gripper (servo on port S3). Give position (open = 120 deg, close = 45 "
        "deg, home = 90 deg) or angle 45-120 (45 closed, 120 open). Glides at 90 deg/s.",
        PropertyList({
            Property("position", kPropertyTypeString, std::string()),
            Property("angle", kPropertyTypeInteger, MBOT_GRIP_HOME, MBOT_GRIP_MIN, MBOT_GRIP_MAX),
        }),
        [](const PropertyList& p) -> ReturnValue {
            return ResultJson(Run("grip " + ServoArg(p, kGripPresets, 3)));
        });

    mcp_server.AddTool(
        "self.mbot.home",
        "Move the mBot2 arm and gripper back to their home position (90 deg each).",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue { return ResultJson(Run("home")); });

    mcp_server.AddTool(
        "self.mbot.read_sensors",
        "Read the mBot2 sensors: distance_cm from the ultrasonic sensor (null if none), "
        "battery_percent, arm_deg, gripper_deg and locked.",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue {
            auto& link = MbotLink::GetInstance();
            link.NoteHostAlive();
            MbotLink::Sensors s;
            MbotLink::Result r = link.ReadSensors(&s);
            if (!r.ok) {
                cJSON* root = ResultJson(r);
                MbotLink::Sensors last = link.LastSensors();
                if (last.valid) {
                    cJSON_AddItemToObject(root, "last_known", SensorsJson(last));
                }
                return root;
            }
            cJSON* root = SensorsJson(s);
            cJSON_AddBoolToObject(root, "ok", true);
            return root;
        });

    mcp_server.AddTool(
        "self.mbot.run_program",
        "Run a short mini-program on the mBot2. steps: steps separated by ';', e.g. "
        "\"fwd 40 1; led 0 255 0; wait 0.5; arm up\". Step forms: fwd|back|left|right <speed% 0-100> "
        "<seconds 0.1-5>; straight <cm -30..30> [speed% 5-100]; turn <deg -180..180, + = left> "
        "[speed% 5-100]; wait <seconds 0-5>; led <r> <g> <b> [all|1-5]; arm up|down|home|<40-120>; "
        "grip open|close|home|<45-120>; home. Max 20 steps and 30 s in total (moves + waits, plus "
        "about 1 s per arm/grip/home step for the servo glide); a longer program is rejected as "
        "a whole. Numbers are clamped to the limits. Returns when the mBot accepts it; a 'done' "
        "event follows at the end. self.mbot.stop aborts it at any time. " + kLimits,
        PropertyList({Property("steps", kPropertyTypeString)}),
        [](const PropertyList& p) -> ReturnValue {
            return ResultJson(Run("prog " + p["steps"].value<std::string>()));
        });

    // Bring BLE up only after activation (HTTPS version check + protocol
    // init) is done: the BT controller takes tens of KB of internal RAM,
    // which TLS also needs at that moment.
    link.SetStartGate([]() {
        switch (Application::GetInstance().GetDeviceState()) {
            case kDeviceStateUnknown:
            case kDeviceStateStarting:
            case kDeviceStateWifiConfiguring:
            case kDeviceStateActivating:
            case kDeviceStateUpgrading:
                return false;
            default:
                return true;
        }
    });
    // ---- v1.1: closed-loop steps for exploring with slow decisions ----
    static const char* const kExploreLoop =
        "For exploring, use the stop -> look -> decide -> step loop: while the mBot is still, "
        "look (photos, self.mbot.read_sensors), decide, then send ONE bounded step or turn. "
        "The mBot closes the loop on its own encoders/gyro, so slow decisions or link delay "
        "cannot change how far it goes. The tool returns as soon as the mBot accepts; the "
        "step ends with an mbot 'done' event '<id> <reason> dist=<cm> yaw=<deg> t=<ms>' that "
        "reports what it ACTUALLY did (reason target, timeout, obstacle, cmd, button, "
        "watchdog or replaced). Poll self.mbot.odometry for last_done with the same id before "
        "the next step. self.mbot.stop is always available.";

    mcp_server.AddTool(
        "self.mbot.step",
        std::string("Drive the mBot2 straight by a distance, closed-loop on its wheel encoders. "
                    "distance_cm -30..30 (negative = backwards; one step is at most 30 cm). "
                    "speed_percent 5-100 (default 30). Forward motion still auto-stops below 10 cm "
                    "from an obstacle. ") + kExploreLoop,
        PropertyList({
            Property("distance_cm", kPropertyTypeInteger, -MBOT_MAX_STEP_CM, MBOT_MAX_STEP_CM),
            Property("speed_percent", kPropertyTypeInteger, MBOT_STEP_DEFAULT_SPEED_PCT,
                     MBOT_STEP_SPEED_MIN_PCT, 100),
        }),
        [](const PropertyList& p) -> ReturnValue {
            char buf[48];
            snprintf(buf, sizeof(buf), "straight %d %d", p["distance_cm"].value<int>(),
                     p["speed_percent"].value<int>());
            return ResultJson(Run(buf));
        });

    mcp_server.AddTool(
        "self.mbot.turn",
        std::string("Turn the mBot2 in place by an angle, closed-loop on its gyro. degrees "
                    "-180..180, positive = left (counter-clockwise), negative = right. "
                    "speed_percent 5-100 (default 30). ") + kExploreLoop,
        PropertyList({
            Property("degrees", kPropertyTypeInteger, -MBOT_MAX_TURN_DEG, MBOT_MAX_TURN_DEG),
            Property("speed_percent", kPropertyTypeInteger, MBOT_STEP_DEFAULT_SPEED_PCT,
                     MBOT_STEP_SPEED_MIN_PCT, 100),
        }),
        [](const PropertyList& p) -> ReturnValue {
            char buf[48];
            snprintf(buf, sizeof(buf), "turn %d %d", p["degrees"].value<int>(),
                     p["speed_percent"].value<int>());
            return ResultJson(Run(buf));
        });

    mcp_server.AddTool(
        "self.mbot.odometry",
        "Pose information cached on Stack-chan (no motion). Returns last_done (the most recent "
        "mbot 'done': id, reason, dist (cm) and yaw (deg) actually travelled, t = mBot ms, "
        "age_ms; null values when the mBot reported none), done (the done for done_id if given "
        "and still among the last 8), odom (latest streamed t, l / r wheel cm, yaw deg, age_ms) "
        "and rtt_ms (last command round trip). stream_ms: optional, sets the mBot's odom stream period "
        "(0 = off, 100-5000 ms); leave it out to only read the cache.",
        PropertyList({
            Property("done_id", kPropertyTypeInteger, 0, 0, MBOT_MAX_CMD_ID),
            Property("stream_ms", kPropertyTypeInteger, 0, 0, MBOT_ODOM_MAX_MS),
        }),
        [](const PropertyList& p) -> ReturnValue {
            auto& link = MbotLink::GetInstance();
            link.NoteHostAlive();
            cJSON* root = cJSON_CreateObject();
            cJSON_AddBoolToObject(root, "connected", link.IsConnected());
            if (p["stream_ms"].was_provided()) {
                MbotLink::Result r = Run("odom " + std::to_string(p["stream_ms"].value<int>()));
                cJSON_AddItemToObject(root, "stream", ResultJson(r));
            }
            cJSON_AddItemToObject(root, "last_done", DoneJson(link.LastDone()));
            if (p["done_id"].was_provided()) {
                MbotLink::Done d;
                if (link.FindDone(p["done_id"].value<int>(), &d)) {
                    cJSON_AddItemToObject(root, "done", DoneJson(d));
                } else {
                    cJSON_AddNullToObject(root, "done");
                }
            }
            cJSON_AddItemToObject(root, "odom", OdomJson(link.LastOdometry()));
            int rtt = link.LastRttMs();
            if (rtt >= 0) {
                cJSON_AddNumberToObject(root, "rtt_ms", rtt);
            } else {
                cJSON_AddNullToObject(root, "rtt_ms");
            }
            return root;
        });

    mcp_server.AddTool(
        "self.mbot.sync",
        "Measure the Bluetooth round trip to the mBot2 and its clock. Returns host_ms "
        "(Stack-chan uptime when sent), mbot_ms (mBot clock when it answered), rtt_ms and "
        "offset_ms (mbot_ms minus the host time at the middle of the round trip), so mbot_ms "
        "values in done/odom events can be mapped onto Stack-chan time.",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue {
            const int64_t host_ms = esp_timer_get_time() / 1000;
            MbotLink::Result r = Run("sync " + std::to_string(host_ms));
            cJSON* root = ResultJson(r);
            cJSON_AddNumberToObject(root, "host_ms", static_cast<double>(host_ms));
            if (r.ok) {
                // detail: "<host_ms> <mbot_ms>"
                long long echoed = 0, mbot_ms = 0;
                if (sscanf(r.detail.c_str(), "%lld %lld", &echoed, &mbot_ms) == 2) {
                    cJSON_AddNumberToObject(root, "mbot_ms", static_cast<double>(mbot_ms));
                    if (r.rtt_ms >= 0) {
                        const double mid = static_cast<double>(host_ms) + r.rtt_ms / 2.0;
                        cJSON_AddNumberToObject(root, "offset_ms", static_cast<double>(mbot_ms) - mid);
                    }
                }
            }
            return root;
        });

    ESP_LOGI(TAG, "self.mbot.* tools registered; BLE link starts after activation");
    link.Start();
    StartMbotTeleop();   // waits for Wi-Fi by itself
}

#endif  // CONFIG_STACKCHAN_MBOT_LINK
