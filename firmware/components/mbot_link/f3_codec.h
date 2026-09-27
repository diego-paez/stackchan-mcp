// SPDX-License-Identifier: MIT
//
// Makeblock CyberPi "F3" framing, upload-mode message only (type 0x03).
// C++ port of mblock-stacky-bridge hub/stacky_hub/transports/f3.py
// (encode_message / Decoder), which is byte-verified against a real CyberPi.
//
//   F3 | (F3+lenL+lenH)&FF | lenL lenH | 03 | {"message":"<topic>","value":"<text>"} | 00 | sum(payload)&FF | F4
//   payload = 03 + JSON + 00      len = len(payload)
//
// The CyberPi sends JSON with spaces ({"message": "pong", "value": "1"}), so
// ParseUploadMessage uses cJSON and accepts both spaced and compact forms.
//
// Pure C++ plus cJSON: no ESP-IDF headers, so it is unit-tested on the host
// (firmware/host_test/test_mbot_link.cc).

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mbot {

constexpr uint8_t kF3Header = 0xF3;
constexpr uint8_t kF3Footer = 0xF4;
constexpr uint8_t kF3TypeUploadMsg = 0x03;

// Frames longer than this are dropped by the decoder instead of buffered.
// Real traffic is well under 600 bytes (CyberPi MTU is 515).
constexpr size_t kF3MaxFrameLen = 2048;

// Build one upload-mode message frame. The JSON body is compact
// ({"message":"sk","value":"12 fwd 40 1"}), identical to f3.py's
// json.dumps(..., separators=(",", ":")).
std::vector<uint8_t> EncodeMessage(const std::string& topic, const std::string& value);

struct F3Frame {
    uint8_t type = 0;
    std::vector<uint8_t> raw;  // whole frame, F3 ... F4
    std::string text;          // type 0x03: JSON text with trailing NULs removed
};

// Streaming decoder. Same resync rule as f3.py / Makeblock's parser: whenever
// the last four bytes look like a header (F3, hchk, lenL, lenH with a valid
// header checksum) the buffer restarts there; a frame is complete after
// len + 6 bytes and is kept only if it ends with F4.
class F3Decoder {
public:
    // Appends complete frames to `out`.
    void Feed(const uint8_t* data, size_t len, std::vector<F3Frame>& out);
    void Reset();

private:
    std::vector<uint8_t> buf_;
    long expected_len_ = -1;  // -1: no header seen yet
};

// Extract topic ("message") and value from an upload-message JSON body.
// Accepts spaced and compact JSON. A non-string value is converted to text
// (numbers without a trailing ".0" when integral). Returns false on bad JSON
// or when "message" is missing.
bool ParseUploadMessage(const std::string& json_text, std::string* topic, std::string* value);

}  // namespace mbot
