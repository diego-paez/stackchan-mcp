// SPDX-License-Identifier: MIT
#include "f3_codec.h"

#include <cJSON.h>

#include <cmath>
#include <cstdio>

namespace mbot {

std::vector<uint8_t> EncodeMessage(const std::string& topic, const std::string& value) {
    std::string body;
    cJSON* root = cJSON_CreateObject();
    if (root != nullptr) {
        cJSON_AddStringToObject(root, "message", topic.c_str());
        cJSON_AddStringToObject(root, "value", value.c_str());
        char* printed = cJSON_PrintUnformatted(root);
        if (printed != nullptr) {
            body = printed;
            cJSON_free(printed);
        }
        cJSON_Delete(root);
    }

    // payload = 03 + JSON + 00
    std::vector<uint8_t> payload;
    payload.reserve(body.size() + 2);
    payload.push_back(kF3TypeUploadMsg);
    payload.insert(payload.end(), body.begin(), body.end());
    payload.push_back(0x00);

    const size_t n = payload.size();
    const uint8_t lo = static_cast<uint8_t>(n & 0xFF);
    const uint8_t hi = static_cast<uint8_t>((n >> 8) & 0xFF);
    uint32_t sum = 0;
    for (uint8_t b : payload) {
        sum += b;
    }

    std::vector<uint8_t> frame;
    frame.reserve(n + 6);
    frame.push_back(kF3Header);
    frame.push_back(static_cast<uint8_t>((kF3Header + lo + hi) & 0xFF));
    frame.push_back(lo);
    frame.push_back(hi);
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(static_cast<uint8_t>(sum & 0xFF));
    frame.push_back(kF3Footer);
    return frame;
}

void F3Decoder::Reset() {
    buf_.clear();
    expected_len_ = -1;
}

void F3Decoder::Feed(const uint8_t* data, size_t len, std::vector<F3Frame>& out) {
    for (size_t i = 0; i < len; ++i) {
        buf_.push_back(data[i]);
        const size_t n = buf_.size();
        if (n > 3 && buf_[n - 4] == kF3Header &&
            ((buf_[n - 1] + buf_[n - 2] + buf_[n - 4]) & 0xFF) == buf_[n - 3]) {
            // Header candidate: restart the buffer at it (f3.py resync rule).
            uint8_t hdr[4] = {buf_[n - 4], buf_[n - 3], buf_[n - 2], buf_[n - 1]};
            buf_.assign(hdr, hdr + 4);
            expected_len_ = static_cast<long>(hdr[2]) | (static_cast<long>(hdr[3]) << 8);
            if (static_cast<size_t>(expected_len_) + 6 > kF3MaxFrameLen) {
                // Implausible length: drop it rather than buffer 64 KB.
                // Keep the 4 bytes so a later header can still resync.
                expected_len_ = -1;
            }
        } else if (expected_len_ >= 0 && n == static_cast<size_t>(expected_len_) + 6) {
            if (buf_.back() == kF3Footer) {
                F3Frame frame;
                frame.raw = buf_;
                frame.type = buf_[4];
                if (frame.type == kF3TypeUploadMsg) {
                    // raw[5:-2], trailing NULs stripped
                    size_t end = n - 2;
                    while (end > 5 && buf_[end - 1] == 0x00) {
                        --end;
                    }
                    frame.text.assign(buf_.begin() + 5, buf_.begin() + end);
                }
                out.push_back(std::move(frame));
            }
            buf_.clear();
            expected_len_ = -1;
        } else if (expected_len_ < 0 && n > 64) {
            // No header in sight (f3.py grows forever here). Only the last
            // three bytes can still become part of a header.
            buf_.erase(buf_.begin(), buf_.end() - 3);
        }
    }
}

bool ParseUploadMessage(const std::string& json_text, std::string* topic, std::string* value) {
    cJSON* root = cJSON_ParseWithLength(json_text.c_str(), json_text.size());
    if (root == nullptr) {
        return false;
    }
    bool ok = false;
    const cJSON* message = cJSON_GetObjectItemCaseSensitive(root, "message");
    const cJSON* val = cJSON_GetObjectItemCaseSensitive(root, "value");
    if (cJSON_IsString(message) && message->valuestring != nullptr) {
        ok = true;
        if (topic != nullptr) {
            *topic = message->valuestring;
        }
        if (value != nullptr) {
            value->clear();
            if (cJSON_IsString(val) && val->valuestring != nullptr) {
                *value = val->valuestring;
            } else if (cJSON_IsNumber(val)) {
                char tmp[32];
                double d = val->valuedouble;
                if (std::isfinite(d) && d == std::floor(d) && std::fabs(d) < 1e15) {
                    snprintf(tmp, sizeof(tmp), "%lld", static_cast<long long>(d));
                } else {
                    snprintf(tmp, sizeof(tmp), "%g", d);
                }
                *value = tmp;
            } else if (cJSON_IsBool(val)) {
                *value = cJSON_IsTrue(val) ? "true" : "false";
            }
        }
    }
    cJSON_Delete(root);
    return ok;
}

}  // namespace mbot
