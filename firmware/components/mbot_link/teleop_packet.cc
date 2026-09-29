// SPDX-License-Identifier: MIT
#include "teleop_packet.h"

namespace teleop {
namespace {

void Put16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void Put32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}

uint16_t Get16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

uint32_t Get32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

size_t PutHeader(uint8_t type, uint32_t seq, uint32_t key, uint8_t* buf) {
    buf[0] = 'S';
    buf[1] = 'T';
    buf[2] = kVersion;
    buf[3] = type;
    Put32(buf + 4, seq);
    Put32(buf + 8, key);
    return kHeaderSize;
}

}  // namespace

bool ParseHeader(const uint8_t* data, size_t len, Header* out) {
    if (data == nullptr || len < kHeaderSize) return false;
    if (data[0] != 'S' || data[1] != 'T' || data[2] != kVersion) return false;
    out->type = data[3];
    out->seq = Get32(data + 4);
    out->key = Get32(data + 8);
    return true;
}

bool ParseCmdVel(const uint8_t* data, size_t len, Header* hdr, CmdVel* out) {
    if (!ParseHeader(data, len, hdr) || hdr->type != kCmdVel || len < kCmdVelSize) return false;
    out->linear_x_mm_s = static_cast<int16_t>(Get16(data + 12));
    out->angular_z_mrad_s = static_cast<int16_t>(Get16(data + 14));
    out->buttons = Get16(data + 16);
    out->lease_ms = Get16(data + 18);
    return true;
}

bool ParseStatus(const uint8_t* data, size_t len, Header* hdr, Status* out) {
    if (!ParseHeader(data, len, hdr) || (hdr->type != kStatus && hdr->type != kHelloAck) ||
        len < kStatusSize) {
        return false;
    }
    out->flags = data[12];
    out->battery_pct = data[13];
    out->dist_cm = Get16(data + 14);
    out->arm_deg = data[16];
    out->grip_deg = data[17];
    out->rtt_ms = Get16(data + 18);
    return true;
}

size_t EncodeHello(uint32_t seq, uint32_t key, uint8_t* buf, size_t cap) {
    if (cap < kHeaderSize) return 0;
    return PutHeader(kHello, seq, key, buf);
}

size_t EncodeCmdVel(uint32_t seq, uint32_t key, const CmdVel& v, uint8_t* buf, size_t cap) {
    if (cap < kCmdVelSize) return 0;
    PutHeader(kCmdVel, seq, key, buf);
    Put16(buf + 12, static_cast<uint16_t>(v.linear_x_mm_s));
    Put16(buf + 14, static_cast<uint16_t>(v.angular_z_mrad_s));
    Put16(buf + 16, v.buttons);
    Put16(buf + 18, v.lease_ms);
    return kCmdVelSize;
}

size_t EncodeStatus(uint8_t type, uint32_t seq, uint32_t key, const Status& s, uint8_t* buf, size_t cap) {
    if (cap < kStatusSize) return 0;
    PutHeader(type, seq, key, buf);
    buf[12] = s.flags;
    buf[13] = s.battery_pct;
    Put16(buf + 14, s.dist_cm);
    buf[16] = s.arm_deg;
    buf[17] = s.grip_deg;
    Put16(buf + 18, s.rtt_ms);
    return kStatusSize;
}

}  // namespace teleop
