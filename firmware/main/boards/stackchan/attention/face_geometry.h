// face_geometry.h — turning a detector's box into something the head can aim at.
//
// Separated from the detector for the same reason everything else in this
// directory is separated from the hardware: this is the part that can be
// wrong in a way that matters, and it is pure arithmetic, so it can be tested
// on a laptop instead of discovered on a robot.
//
// The one job: a bounding box in image pixels, from a frame of known size,
// becomes a FaceTarget in the normalised -1..+1 coordinates AttentionController
// expects. Getting a sign or a divisor wrong here does not fail loudly — it
// produces a head that turns smoothly and confidently in the wrong direction.
#pragma once

#include <algorithm>
#include <cstdint>

#include "attention_types.h"

namespace stackchan {
namespace attention {

// A detection as esp-dl reports it: corners in pixels, plus a score.
// Deliberately not dl::detect::result_t — that type drags in the whole
// library, and this header has to compile on a host with no ESP-IDF.
struct DetectedBox {
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
    float score = 0.0f;

    int width() const { return right - left; }
    int height() const { return bottom - top; }
    long area() const {
        const long w = width() > 0 ? width() : 0;
        const long h = height() > 0 ? height() : 0;
        return w * h;
    }
};

// Is this box usable at all? A detector can return a zero-width box, a box
// with its corners the wrong way round, or one that runs off the frame.
inline bool IsUsableBox(const DetectedBox& b, int img_w, int img_h) {
    if (img_w <= 0 || img_h <= 0) return false;
    if (b.width() <= 0 || b.height() <= 0) return false;
    if (b.right <= 0 || b.bottom <= 0) return false;
    if (b.left >= img_w || b.top >= img_h) return false;
    return true;
}

// Pick the face to follow when several are in frame.
//
// Largest box, not highest score. With a group of children the nearest one is
// almost always the one addressing the robot, and box area is the only depth
// cue available from a single camera. Score decides ties, because two boxes of
// equal area are usually the same face reported twice.
//
// Returns nullptr when the list is empty or nothing is usable.
template <typename It>
const DetectedBox* PickPrimaryFace(It begin, It end, int img_w, int img_h) {
    const DetectedBox* best = nullptr;
    for (It it = begin; it != end; ++it) {
        const DetectedBox& b = *it;
        if (!IsUsableBox(b, img_w, img_h)) continue;
        if (best == nullptr) { best = &b; continue; }
        if (b.area() > best->area()) { best = &b; continue; }
        if (b.area() == best->area() && b.score > best->score) best = &b;
    }
    return best;
}

// Box in pixels -> FaceTarget in -1..+1.
//
// x is +1 at the right edge of the image, y is +1 at the BOTTOM: image
// coordinates grow downward and AttentionController is written expecting that
// (it inverts the pitch term itself). Flipping y here to make it "up" would
// silently double-invert and drive the head the wrong way in elevation.
//
// size is the larger box edge as a fraction of the corresponding image edge —
// the same convention FaceTarget documents, and the only depth cue there is.
inline FaceTarget BoxToTarget(const DetectedBox& b, int img_w, int img_h, uint32_t now_ms) {
    FaceTarget t;
    if (!IsUsableBox(b, img_w, img_h)) return t;   // visible stays false

    const float cx = (static_cast<float>(b.left) + static_cast<float>(b.right)) * 0.5f;
    const float cy = (static_cast<float>(b.top) + static_cast<float>(b.bottom)) * 0.5f;

    // Centre of the image is 0; edges are ±1.
    t.x = (cx / static_cast<float>(img_w)) * 2.0f - 1.0f;
    t.y = (cy / static_cast<float>(img_h)) * 2.0f - 1.0f;

    // A box may legitimately extend past the frame edge when a face is half
    // out of shot; its centre can then land outside ±1. Clamp, because
    // everything downstream treats ±1 as the edge of what the camera can see.
    t.x = std::max(-1.0f, std::min(1.0f, t.x));
    t.y = std::max(-1.0f, std::min(1.0f, t.y));

    const float fw = static_cast<float>(b.width()) / static_cast<float>(img_w);
    const float fh = static_cast<float>(b.height()) / static_cast<float>(img_h);
    t.size = std::max(0.0f, std::min(1.0f, std::max(fw, fh)));

    t.confidence = std::max(0.0f, std::min(1.0f, b.score));
    t.last_seen_ms = now_ms;
    t.visible = true;
    return t;
}

}  // namespace attention
}  // namespace stackchan
