// SPDX-License-Identifier: MIT
#include "mbot_command.h"

#include "mbot_limits.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace mbot {
namespace {

// Longest command text we are willing to send. Id, JSON and F3 framing add
// about 40 bytes, so a frame stays inside one 509-byte ATT write at the
// negotiated MTU of 512. Twenty "led 255 255 255 all" steps are 400 bytes.
constexpr size_t kMaxCommandLen = 440;

std::vector<std::string> SplitWhitespace(const std::string& s) {
    std::vector<std::string> out;
    std::istringstream is(s);
    std::string tok;
    while (is >> tok) {
        out.push_back(tok);
    }
    return out;
}

std::string Trim(const std::string& s) {
    size_t b = 0;
    size_t e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

// Strict number parse: the whole token must be a finite number.
bool ParseNumber(const std::string& tok, double* out) {
    if (tok.empty()) return false;
    const char* begin = tok.c_str();
    char* end = nullptr;
    double v = std::strtod(begin, &end);
    if (end == begin || *end != '\0' || !std::isfinite(v)) return false;
    *out = v;
    return true;
}

double Clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

std::string FormatSeconds(double s) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.2f", s);
    std::string out(buf);
    while (!out.empty() && out.back() == '0') out.pop_back();
    if (!out.empty() && out.back() == '.') out.pop_back();
    return out;
}

void NoteClamp(std::string* notes, const std::string& what, double asked, double used) {
    if (std::fabs(asked - used) < 1e-9) return;
    if (!notes->empty()) *notes += "; ";
    *notes += what + " " + FormatSeconds(asked) + " -> " + FormatSeconds(used);
}

bool IsMove(const std::string& c) {
    return c == "fwd" || c == "back" || c == "left" || c == "right";
}

// Validate one step. `in_program` allows `wait` and forbids stop/read/prog.
CommandCheck CheckStep(const std::vector<std::string>& tokens, bool in_program) {
    CommandCheck r;
    if (tokens.empty()) {
        r.error = "empty";
        return r;
    }
    const std::string& c = tokens[0];
    const size_t nargs = tokens.size() - 1;
    r.cmd = c;

    if (IsMove(c)) {
        if (nargs > 2) {
            r.error = "too_many_args:" + c;
            return r;
        }
        double pct = MBOT_DEFAULT_SPEED_PCT;
        double sec = MBOT_DEFAULT_MOVE_S;
        if (nargs >= 1 && !ParseNumber(tokens[1], &pct)) {
            r.error = "bad_speed:" + tokens[1];
            return r;
        }
        if (nargs >= 2 && !ParseNumber(tokens[2], &sec)) {
            r.error = "bad_seconds:" + tokens[2];
            return r;
        }
        const double pct_c = std::round(Clamp(pct, 0, 100));
        const double sec_c = Clamp(sec, MBOT_MIN_MOVE_S, MBOT_MAX_MOVE_S);
        NoteClamp(&r.clamped, c + " speed%", pct, pct_c);
        NoteClamp(&r.clamped, c + " seconds", sec, sec_c);
        r.normalized = c + " " + std::to_string(static_cast<int>(pct_c)) + " " + FormatSeconds(sec_c);
        r.seconds = sec_c;
        r.is_motion = true;
        r.ok = true;
        return r;
    }

    if (c == "straight" || c == "turn") {
        // v1.1 closed-loop step: straight <cm> [speed%] / turn <deg> [speed%]
        if (nargs < 1 || nargs > 2) {
            r.error = "bad_args:" + c;
            return r;
        }
        const bool straight = c == "straight";
        const double lim = straight ? MBOT_MAX_STEP_CM : MBOT_MAX_TURN_DEG;
        double amount = 0;
        if (!ParseNumber(tokens[1], &amount)) {
            r.error = (straight ? "bad_distance:" : "bad_degrees:") + tokens[1];
            return r;
        }
        double pct = MBOT_STEP_DEFAULT_SPEED_PCT;
        if (nargs == 2 && !ParseNumber(tokens[2], &pct)) {
            r.error = "bad_speed:" + tokens[2];
            return r;
        }
        const double amount_c = std::round(Clamp(amount, -lim, lim) * 10.0) / 10.0;
        const double pct_c = std::round(Clamp(pct, MBOT_STEP_SPEED_MIN_PCT, 100));
        NoteClamp(&r.clamped, c + (straight ? " cm" : " degrees"), amount, amount_c);
        NoteClamp(&r.clamped, c + " speed%", pct, pct_c);
        r.normalized = c + " " + FormatSeconds(amount_c) + " " + std::to_string(static_cast<int>(pct_c));
        r.seconds = ClosedLoopSeconds(c, amount_c, pct_c);
        r.is_motion = true;
        r.ok = true;
        return r;
    }

    if (c == "drive") {
        // v1.2 teleop, ROS cmd_vel shape: drive <v_cm_s> <w_deg_s> [lease_ms]
        if (in_program) {
            r.error = "step_not_allowed:drive";
            return r;
        }
        if (nargs < 2 || nargs > 3) {
            r.error = "bad_args:drive";
            return r;
        }
        double v = 0, w = 0, lease = MBOT_MAX_DRIVE_LEASE_MS;
        if (!ParseNumber(tokens[1], &v)) {
            r.error = "bad_speed:" + tokens[1];
            return r;
        }
        if (!ParseNumber(tokens[2], &w)) {
            r.error = "bad_turn_rate:" + tokens[2];
            return r;
        }
        if (nargs == 3 && !ParseNumber(tokens[3], &lease)) {
            r.error = "bad_lease:" + tokens[3];
            return r;
        }
        const double vmax = MBOT_MAX_RPM * 6.0 * MBOT_WHEEL_CM_PER_DEG;  // cm/s at MAX_RPM
        const double v_c = std::round(Clamp(v, -vmax, vmax) * 10.0) / 10.0;
        const double w_c = std::round(Clamp(w, -MBOT_MAX_DRIVE_DEG_S, MBOT_MAX_DRIVE_DEG_S));
        const double lease_c = std::round(Clamp(lease, 50, MBOT_MAX_DRIVE_LEASE_MS));
        NoteClamp(&r.clamped, "drive cm/s", v, v_c);
        NoteClamp(&r.clamped, "drive deg/s", w, w_c);
        NoteClamp(&r.clamped, "drive lease_ms", lease, lease_c);
        r.normalized = "drive " + FormatSeconds(v_c) + " " + std::to_string(static_cast<int>(w_c)) + " " +
                       std::to_string(static_cast<int>(lease_c));
        r.seconds = lease_c / 1000.0;
        r.is_motion = true;
        r.ok = true;
        return r;
    }

    if (c == "wait") {
        if (!in_program) {
            r.error = "wait_only_in_prog";
            return r;
        }
        if (nargs != 1) {
            r.error = "bad_args:wait";
            return r;
        }
        double sec = 0;
        if (!ParseNumber(tokens[1], &sec)) {
            r.error = "bad_seconds:" + tokens[1];
            return r;
        }
        const double sec_c = Clamp(sec, 0.0, MBOT_MAX_MOVE_S);
        NoteClamp(&r.clamped, "wait seconds", sec, sec_c);
        r.normalized = "wait " + FormatSeconds(sec_c);
        r.seconds = sec_c;
        r.ok = true;
        return r;
    }

    if (c == "led") {
        if (nargs < 3 || nargs > 4) {
            r.error = "bad_args:led";
            return r;
        }
        std::string out = "led";
        for (int i = 1; i <= 3; ++i) {
            double v = 0;
            if (!ParseNumber(tokens[i], &v)) {
                r.error = "bad_color:" + tokens[i];
                return r;
            }
            const double vc = std::round(Clamp(v, 0, 255));
            NoteClamp(&r.clamped, "led color", v, vc);
            out += " " + std::to_string(static_cast<int>(vc));
        }
        if (nargs == 4) {
            if (tokens[4] == "all") {
                out += " all";
            } else {
                double idx = 0;
                if (!ParseNumber(tokens[4], &idx) || idx != std::floor(idx) || idx < 1 || idx > 5) {
                    r.error = "bad_led_index:" + tokens[4];
                    return r;
                }
                out += " " + std::to_string(static_cast<int>(idx));
            }
        } else {
            out += " all";
        }
        r.normalized = out;
        r.ok = true;
        return r;
    }

    if (c == "arm" || c == "grip") {
        if (nargs != 1) {
            r.error = "bad_args:" + c;
            return r;
        }
        const bool arm = c == "arm";
        const int lo = arm ? MBOT_ARM_MIN : MBOT_GRIP_MIN;
        const int hi = arm ? MBOT_ARM_MAX : MBOT_GRIP_MAX;
        const int home = arm ? MBOT_ARM_HOME : MBOT_GRIP_HOME;
        const std::string& a = tokens[1];
        int target = -1;
        if (a == "home") {
            target = home;
        } else if (arm && a == "up") {
            target = MBOT_ARM_MAX;
        } else if (arm && a == "down") {
            target = MBOT_ARM_MIN;
        } else if (!arm && a == "open") {
            target = MBOT_GRIP_MAX;
        } else if (!arm && a == "close") {
            target = MBOT_GRIP_MIN;
        }
        std::string arg = a;
        if (target < 0) {
            double deg = 0;
            if (!ParseNumber(a, &deg)) {
                r.error = "bad_position:" + a;
                return r;
            }
            const double dc = std::round(Clamp(deg, lo, hi));
            NoteClamp(&r.clamped, c + " angle", deg, dc);
            target = static_cast<int>(dc);
            arg = std::to_string(target);
        }
        r.normalized = c + " " + arg;
        r.seconds = ServoGlideSeconds(c, target);
        r.is_motion = true;
        r.ok = true;
        return r;
    }

    if (c == "home") {
        if (nargs != 0) {
            r.error = "bad_args:home";
            return r;
        }
        r.normalized = "home";
        r.seconds = ServoGlideSeconds("home", 0);
        r.is_motion = true;
        r.ok = true;
        return r;
    }

    if (!in_program && c == "odom") {
        // v1.1 odometry stream period: 0 = off, else 100..5000 ms.
        if (nargs != 1) {
            r.error = "bad_args:odom";
            return r;
        }
        double ms = 0;
        if (!ParseNumber(tokens[1], &ms)) {
            r.error = "bad_period:" + tokens[1];
            return r;
        }
        const double ms_c = ms <= 0 ? 0 : std::round(Clamp(ms, MBOT_ODOM_MIN_MS, MBOT_ODOM_MAX_MS));
        NoteClamp(&r.clamped, "odom period_ms", ms < 0 ? 0 : ms, ms_c);
        r.normalized = "odom " + std::to_string(static_cast<int>(ms_c));
        r.ok = true;
        return r;
    }

    if (!in_program && c == "sync") {
        // v1.1 clock sync: sync <host_ms> (non-negative integer).
        if (nargs != 1) {
            r.error = "bad_args:sync";
            return r;
        }
        const std::string& t = tokens[1];
        if (t.empty() || t.size() > 12 ||
            !std::all_of(t.begin(), t.end(), [](char ch) { return ch >= '0' && ch <= '9'; })) {
            r.error = "bad_host_ms:" + t;
            return r;
        }
        r.normalized = "sync " + t;
        r.ok = true;
        return r;
    }

    if (!in_program && (c == "stop" || c == "read")) {
        // Extra arguments are ignored; stop must always go through.
        r.normalized = c;
        r.is_stop = c == "stop";
        r.ok = true;
        return r;
    }

    r.error = (in_program ? "step_not_allowed:" : "unknown_cmd:") + c;
    return r;
}

}  // namespace

double ClosedLoopSeconds(const std::string& cmd, double amount, double speed_pct) {
    // Same as stacky_runtime.py parse_step: the mBot uses this as the step's
    // timeout, so it is also the worst-case duration.
    const double v_cm_s = MBOT_MAX_RPM * speed_pct / 100.0 * 6.0 * MBOT_WHEEL_CM_PER_DEG;  // rpm*6 = deg/s
    if (v_cm_s <= 0) return MBOT_MAX_MOVE_S;
    double est;
    if (cmd == "straight") {
        est = std::fabs(amount) / v_cm_s;
    } else {
        est = std::fabs(amount) / (2.0 * v_cm_s / MBOT_TRACK_CM * 57.2958);
    }
    return std::min(est * 1.5 + 1.0, MBOT_MAX_MOVE_S);
}

double ServoGlideSeconds(const std::string& cmd, int target_deg) {
    // Worst case: the servo starts at the far end of its range.
    auto worst = [](int lo, int hi, int target) {
        return static_cast<double>(std::max(std::abs(target - lo), std::abs(hi - target))) /
               MBOT_SERVO_DEG_PER_S;
    };
    if (cmd == "arm") return worst(MBOT_ARM_MIN, MBOT_ARM_MAX, target_deg);
    if (cmd == "grip") return worst(MBOT_GRIP_MIN, MBOT_GRIP_MAX, target_deg);
    if (cmd == "home") {
        // Both servos glide at the same time.
        return std::max(worst(MBOT_ARM_MIN, MBOT_ARM_MAX, MBOT_ARM_HOME),
                        worst(MBOT_GRIP_MIN, MBOT_GRIP_MAX, MBOT_GRIP_HOME));
    }
    return 0.0;
}

CommandCheck CheckCommand(const std::string& text) {
    std::string lower;
    lower.reserve(text.size());
    for (char ch : text) {
        const unsigned char u = static_cast<unsigned char>(ch);
        if (u >= 0x80 || (u < 0x20 && !std::isspace(u))) {
            CommandCheck r;
            r.error = "non_ascii";
            return r;
        }
        lower.push_back(static_cast<char>(std::tolower(u)));
    }
    std::string trimmed = Trim(lower);
    if (trimmed.empty()) {
        CommandCheck r;
        r.error = "empty";
        return r;
    }

    // prog: everything after the keyword, steps separated by ';'
    if (trimmed.compare(0, 4, "prog") == 0 &&
        (trimmed.size() == 4 || std::isspace(static_cast<unsigned char>(trimmed[4])))) {
        CommandCheck r;
        r.cmd = "prog";
        std::string body = trimmed.substr(4);
        std::vector<std::string> steps;
        std::string cur;
        std::istringstream is(body);
        while (std::getline(is, cur, ';')) {
            cur = Trim(cur);
            if (!cur.empty()) steps.push_back(cur);
        }
        if (steps.empty()) {
            r.error = "empty_program";
            return r;
        }
        if (static_cast<int>(steps.size()) > MBOT_MAX_PROG_STEPS) {
            r.error = "too_many_steps:" + std::to_string(steps.size()) + ">" +
                      std::to_string(MBOT_MAX_PROG_STEPS);
            return r;
        }
        std::string out = "prog ";
        double total = 0;
        for (size_t i = 0; i < steps.size(); ++i) {
            CommandCheck s = CheckStep(SplitWhitespace(steps[i]), true);
            if (!s.ok) {
                r.error = "step" + std::to_string(i + 1) + ":" + s.error;
                return r;
            }
            if (!s.clamped.empty()) {
                if (!r.clamped.empty()) r.clamped += "; ";
                r.clamped += "step" + std::to_string(i + 1) + ": " + s.clamped;
            }
            total += s.seconds;
            if (i > 0) out += ";";
            out += s.normalized;
        }
        if (total > MBOT_MAX_PROG_S + 1e-9) {
            r.error = "too_long:" + FormatSeconds(total) + "s>" + FormatSeconds(MBOT_MAX_PROG_S) + "s";
            return r;
        }
        if (out.size() > kMaxCommandLen) {
            r.error = "program_text_too_long";
            return r;
        }
        r.normalized = out;
        r.seconds = total;
        r.steps = static_cast<int>(steps.size());
        r.is_motion = true;
        r.ok = true;
        return r;
    }

    CommandCheck r = CheckStep(SplitWhitespace(trimmed), false);
    if (r.ok && r.normalized.size() > kMaxCommandLen) {
        r.ok = false;
        r.error = "command_too_long";
    }
    return r;
}

namespace {

// "key=value" -> value, for tokens after the positional ones.
bool FindKey(const std::vector<std::string>& toks, size_t from, const char* key, std::string* out) {
    const std::string prefix = std::string(key) + "=";
    for (size_t i = from; i < toks.size(); ++i) {
        if (toks[i].compare(0, prefix.size(), prefix) == 0) {
            *out = toks[i].substr(prefix.size());
            return true;
        }
    }
    return false;
}

std::optional<double> KeyNumber(const std::vector<std::string>& toks, size_t from, const char* key) {
    std::string v;
    double d = 0;
    if (FindKey(toks, from, key, &v) && v != "none" && ParseNumber(v, &d)) return d;
    return std::nullopt;
}

std::optional<int64_t> KeyInt(const std::vector<std::string>& toks, size_t from, const char* key) {
    auto d = KeyNumber(toks, from, key);
    if (!d) return std::nullopt;
    return static_cast<int64_t>(*d);
}

}  // namespace

bool ParseDoneArgs(const std::string& args, DoneEvent* out) {
    auto toks = SplitWhitespace(args);
    if (toks.empty()) return false;
    double id = 0;
    if (!ParseNumber(toks[0], &id) || id < 1 || id != std::floor(id)) return false;
    DoneEvent d;
    d.id = static_cast<int>(id);
    size_t kv_from = 1;
    if (toks.size() > 1 && toks[1].find('=') == std::string::npos) {
        d.reason = toks[1];
        kv_from = 2;
    }
    d.dist_cm = KeyNumber(toks, kv_from, "dist");
    d.yaw_deg = KeyNumber(toks, kv_from, "yaw");
    d.mbot_ms = KeyInt(toks, kv_from, "t");
    *out = d;
    return true;
}

bool ParseOdomArgs(const std::string& args, OdomSample* out) {
    auto toks = SplitWhitespace(args);
    if (toks.empty()) return false;
    OdomSample o;
    o.mbot_ms = KeyInt(toks, 0, "t");
    o.left_cm = KeyNumber(toks, 0, "l");
    o.right_cm = KeyNumber(toks, 0, "r");
    o.yaw_deg = KeyNumber(toks, 0, "yaw");
    *out = o;
    return true;
}

}  // namespace mbot
