// SPDX-License-Identifier: MIT
//
// Stacky teleop v1: the UDP packets between a Wi-Fi joystick (M5StickC Plus +
// JoyC) and Stack-chan. Shaped after ROS 2 so a ROS bridge is a thin
// translation: CMD_VEL carries geometry_msgs/Twist (linear.x, angular.z) plus
// joystick buttons; STATUS is what the joystick shows on its screen.
// Spec: mblock-stacky-bridge/docs/teleop.md
//
// All fields little-endian. Every packet starts with a 12-byte header:
//   0  'S' 'T'   magic
//   2  u8        version (1)
//   3  u8        type (PacketType)
//   4  u32       seq     (per sender, increasing; older packets are dropped)
//   8  u32       key     (pairing key; must match the receiver's)
// CMD_VEL (20 bytes): i16 linear_x_mm_s, i16 angular_z_mrad_s (+ = left/CCW),
//                     u16 buttons (Button bits), u16 lease_ms
// STATUS / HELLO_ACK (20 bytes): u8 flags (StatusFlag bits), u8 battery_pct
//                     (0xFF unknown), u16 dist_cm (0xFFFF none), u8 arm_deg,
//                     u8 grip_deg, u16 rtt_ms (0xFFFF unknown)
// HELLO (12 bytes): header only; asks the receiver for a HELLO_ACK.
//
// Pure C++ (no ESP-IDF), unit-tested on the host.

#pragma once

#include <cstddef>
#include <cstdint>

namespace teleop {

constexpr uint8_t kVersion = 1;
constexpr size_t kHeaderSize = 12;
constexpr size_t kCmdVelSize = 20;
constexpr size_t kStatusSize = 20;
constexpr uint16_t kDefaultPort = 8790;

enum PacketType : uint8_t {
    kCmdVel = 1,
    kStatus = 2,
    kHello = 3,
    kHelloAck = 4,
};

enum Button : uint16_t {
    kEnable = 1 << 0,     // dead-man: wheels move only while set
    kEstop = 1 << 1,      // stop now
    kArmUp = 1 << 2,
    kArmDown = 1 << 3,
    kGripOpen = 1 << 4,
    kGripClose = 1 << 5,
    kHome = 1 << 6,
};

enum StatusFlag : uint8_t {
    kMbotConnected = 1 << 0,
    kMbotLocked = 1 << 1,
    kTeleopActive = 1 << 2,   // this joystick is driving right now
    kGatewayConnected = 1 << 3,
};

struct Header {
    uint8_t type = 0;
    uint32_t seq = 0;
    uint32_t key = 0;
};

struct CmdVel {
    int16_t linear_x_mm_s = 0;
    int16_t angular_z_mrad_s = 0;
    uint16_t buttons = 0;
    uint16_t lease_ms = 0;
};

struct Status {
    uint8_t flags = 0;
    uint8_t battery_pct = 0xFF;
    uint16_t dist_cm = 0xFFFF;
    uint8_t arm_deg = 0;
    uint8_t grip_deg = 0;
    uint16_t rtt_ms = 0xFFFF;
};

// Parse the header; false if the magic, version or length is wrong.
bool ParseHeader(const uint8_t* data, size_t len, Header* out);
bool ParseCmdVel(const uint8_t* data, size_t len, Header* hdr, CmdVel* out);
bool ParseStatus(const uint8_t* data, size_t len, Header* hdr, Status* out);

// Encoders return the number of bytes written (0 if `cap` is too small).
size_t EncodeHello(uint32_t seq, uint32_t key, uint8_t* buf, size_t cap);
size_t EncodeCmdVel(uint32_t seq, uint32_t key, const CmdVel& v, uint8_t* buf, size_t cap);
size_t EncodeStatus(uint8_t type, uint32_t seq, uint32_t key, const Status& s, uint8_t* buf, size_t cap);

// Twist units -> Stacky `drive` units: cm/s and deg/s.
inline double LinearCmPerS(const CmdVel& v) { return v.linear_x_mm_s / 10.0; }
inline double AngularDegPerS(const CmdVel& v) { return v.angular_z_mrad_s / 1000.0 * 57.29578; }

// Sequence check that survives the u32 wrap: true if `seq` is newer than `last`.
inline bool SeqNewer(uint32_t seq, uint32_t last) { return static_cast<int32_t>(seq - last) > 0; }

}  // namespace teleop
