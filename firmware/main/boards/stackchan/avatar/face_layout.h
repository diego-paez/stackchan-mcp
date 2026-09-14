// face_layout.h — where the eyes and the mouth go, on a face that fills the screen.
//
// The avatar used to be a 160x120 bitmap upscaled 2x into the middle of a
// 320x240 LCD, with the features drawn small inside it. This computes the
// face instead of storing it: one flat background colour over the whole
// display, and eyes, brows and a mouth sized as fractions of the canvas, so
// they are as large as the screen allows and stay proportionate on any panel.
//
// Three things fall out of computing rather than storing. Expressions become
// parameters instead of six images; blinking and lip-sync become continuous
// values rather than frames; and nothing has to live in PSRAM. The 90-image
// matrix an AvatarSet can hold is 3.4 MB of RGB565 for what is, geometrically,
// a dozen numbers.
//
// Pure arithmetic, no LVGL, no ESP-IDF — the same reason face_geometry.h is
// pure. A face that is subtly wrong is much easier to see in an assertion
// than on a desk.
//
// COORDINATES
// Screen pixels, origin top-left, +y DOWN, matching LVGL. Sizes are radii and
// half-widths, not diameters, because that is what the drawing calls take.
#pragma once

#include <algorithm>
#include <cstdint>

namespace stackchan {
namespace avatar {

// The six the rest of the firmware already speaks — FaceNameToIndex() in
// stackchan.cc, and AvatarFace in attention/face_mimic.h.
enum class Expression : uint8_t {
    kIdle = 0,
    kHappy,
    kThinking,
    kSad,
    kSurprised,
    kEmbarrassed,
};

inline const char* ToString(Expression e) {
    switch (e) {
        case Expression::kIdle: return "idle";
        case Expression::kHappy: return "happy";
        case Expression::kThinking: return "thinking";
        case Expression::kSad: return "sad";
        case Expression::kSurprised: return "surprised";
        case Expression::kEmbarrassed: return "embarrassed";
    }
    return "idle";
}

struct Canvas {
    int width = 320;
    int height = 240;
};

struct Eye {
    int cx = 0, cy = 0;   // centre
    int rx = 0, ry = 0;   // radii; ry collapses to a line on a blink
};

struct Brow {
    bool visible = false;
    int cx = 0, cy = 0;
    int half_w = 0;
    int thickness = 0;
    // Positive tilts the INNER end (toward the nose) up, which is the shape
    // of worry. Negative tilts it down, which is the shape of a scowl.
    int tilt_deg = 0;
};

struct Mouth {
    int cx = 0, cy = 0;
    int half_w = 0;
    int open = 0;          // vertical opening in pixels; 0 is a closed line
    int thickness = 0;
    // Positive lifts the corners (a smile), negative drops them (a frown).
    int corner_lift = 0;
};

struct Blush {
    bool visible = false;
    int cx = 0, cy = 0;
    int rx = 0, ry = 0;
};

// The mouth is drawn as one ellipse with a background-coloured rectangle over
// the half that should not be there: a smile is the bottom of an ellipse, a
// frown is the top. Keeping that decision here rather than in the LVGL layer
// means the host simulation draws exactly what the panel will.
struct MouthMask {
    bool visible = false;
    int x = 0, y = 0, w = 0, h = 0;
};

struct FaceLayout {
    Eye left_eye, right_eye;
    Brow left_brow, right_brow;
    Mouth mouth;
    Blush left_blush, right_blush;
};

// Every proportion in one place, as fractions of the canvas, so the face can
// be retuned without touching the arithmetic and scales to any panel.
//
// The numbers are chosen to fill the screen: on 320x240 the eyes are 74 px
// across and sit 147 px apart, and the drawn face spans roughly 80% of the
// width and 70% of the height. That is the whole point of the change — the
// old bitmap face read as a small picture of a face on a large black screen.
struct FaceStyle {
    float eye_cy = 0.375f;        // eye centre, as a fraction of height
    float eye_dx = 0.26f;         // eye centre offset from the midline
    float eye_rx = 0.135f;        // horizontal radius, fraction of width
    float eye_ry = 0.175f;        // vertical radius, fraction of height

    float brow_gap = 0.075f;      // brow above the eye, fraction of height
    float brow_half_w = 0.125f;
    float brow_thickness = 0.030f;

    float mouth_cy = 0.775f;
    float mouth_half_w = 0.21f;
    float mouth_thickness = 0.032f;
    float mouth_open_max = 0.17f; // fraction of height at full open

    float blush_cy = 0.60f;
    float blush_dx = 0.36f;
    float blush_rx = 0.060f;
    float blush_ry = 0.035f;

    // A closed eye is still a drawn line, not nothing.
    int closed_eye_ry = 3;
};

namespace detail {

inline int Round(float v) { return static_cast<int>(v < 0 ? v - 0.5f : v + 0.5f); }
inline float Clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

// Keep a drawn feature fully inside the canvas. A face that runs off the
// panel does not look expressive, it looks broken, and on a round or offset
// display it is the first thing to go wrong.
inline void ClampSpan(int* centre, int radius, int limit) {
    if (radius * 2 >= limit) {
        *centre = limit / 2;
        return;
    }
    if (*centre - radius < 0) *centre = radius;
    if (*centre + radius > limit) *centre = limit - radius;
}

}  // namespace detail

// blink: 0 is wide open, 1 is fully closed.
// mouth_open: 0 is a closed line, 1 is the widest the style allows.
inline FaceLayout ComputeFace(const Canvas& c, Expression e, float blink = 0.0f,
                              float mouth_open = 0.0f, const FaceStyle& s = FaceStyle{}) {
    using detail::Round;
    using detail::Clamp01;
    using detail::ClampSpan;

    FaceLayout f;
    const int w = std::max(1, c.width);
    const int h = std::max(1, c.height);
    const float fw = static_cast<float>(w);
    const float fh = static_cast<float>(h);
    blink = Clamp01(blink);
    mouth_open = Clamp01(mouth_open);

    // --- per-expression modifiers -----------------------------------------
    // Solid eyes with no pupils, so expression has to come from eye height,
    // brow angle and the mouth. That is also how the M5Stack avatar reads at
    // a glance from across a room.
    float eye_ry_scale = 1.0f;
    float eye_rx_scale = 1.0f;
    int brow_tilt = 0;
    float brow_lift = 0.0f;       // extra fraction of height above the eye
    int corner_lift_px = 0;
    float mouth_w_scale = 1.0f;
    float mouth_floor = 0.0f;     // opening even when mouth_open is 0
    bool blush = false;

    switch (e) {
        case Expression::kIdle:
            break;
        case Expression::kHappy:
            // Eyes squeeze upward in a real smile; the mouth does the rest.
            eye_ry_scale = 0.70f;
            brow_lift = 0.010f;
            corner_lift_px = Round(0.055f * fh);
            mouth_w_scale = 1.10f;
            break;
        case Expression::kThinking:
            eye_ry_scale = 0.80f;
            eye_rx_scale = 0.95f;
            brow_tilt = -12;      // drawn down at the inner end: concentration
            brow_lift = 0.018f;
            mouth_w_scale = 0.65f;
            break;
        case Expression::kSad:
            eye_ry_scale = 0.88f;
            brow_tilt = 16;       // inner ends up: the shape of worry
            corner_lift_px = -Round(0.050f * fh);
            mouth_w_scale = 0.90f;
            break;
        case Expression::kSurprised:
            eye_ry_scale = 1.20f;
            eye_rx_scale = 1.06f;
            brow_lift = 0.028f;
            mouth_w_scale = 0.70f;
            mouth_floor = 0.55f;  // an open, round mouth even at rest
            break;
        case Expression::kEmbarrassed:
            eye_ry_scale = 0.55f;
            brow_tilt = 10;
            mouth_w_scale = 0.75f;
            corner_lift_px = -Round(0.018f * fh);
            blush = true;
            break;
    }

    // --- eyes --------------------------------------------------------------
    const int eye_cy = Round(s.eye_cy * fh);
    const int eye_dx = Round(s.eye_dx * fw);
    const int rx = std::max(1, Round(s.eye_rx * fw * eye_rx_scale));
    const int open_ry = std::max(1, Round(s.eye_ry * fh * eye_ry_scale));

    // A blink interpolates the height only: the eye must not drift up the
    // face as it closes, which is what scaling the whole shape would do.
    const int ry = std::max(s.closed_eye_ry,
                            Round(open_ry * (1.0f - blink) + s.closed_eye_ry * blink));

    f.left_eye.cx = w / 2 - eye_dx;
    f.right_eye.cx = w / 2 + eye_dx;
    f.left_eye.cy = f.right_eye.cy = eye_cy;
    f.left_eye.rx = f.right_eye.rx = rx;
    f.left_eye.ry = f.right_eye.ry = ry;

    ClampSpan(&f.left_eye.cx, rx, w);
    ClampSpan(&f.right_eye.cx, rx, w);
    ClampSpan(&f.left_eye.cy, ry, h);
    f.right_eye.cy = f.left_eye.cy;

    // --- brows -------------------------------------------------------------
    const int brow_thickness = std::max(1, Round(s.brow_thickness * fh));
    const int brow_half_w = std::max(1, Round(s.brow_half_w * fw));
    // Anchored to the eye's RESTING height, not its current one. Hanging the
    // brow off a squinted eye drags it down the face, and the face stops
    // filling the screen exactly when the expression is strongest.
    const int rest_ry = std::max(1, Round(s.eye_ry * fh));
    int brow_cy = f.left_eye.cy - rest_ry - Round((s.brow_gap + brow_lift) * fh);

    f.left_brow.visible = f.right_brow.visible = true;
    f.left_brow.cx = f.left_eye.cx;
    f.right_brow.cx = f.right_eye.cx;
    f.left_brow.cy = f.right_brow.cy = brow_cy;
    f.left_brow.half_w = f.right_brow.half_w = brow_half_w;
    f.left_brow.thickness = f.right_brow.thickness = brow_thickness;
    // Mirrored: "inner end up" is a different screen rotation on each side.
    f.left_brow.tilt_deg = brow_tilt;
    f.right_brow.tilt_deg = -brow_tilt;

    ClampSpan(&f.left_brow.cy, brow_thickness, h);
    f.right_brow.cy = f.left_brow.cy;
    ClampSpan(&f.left_brow.cx, brow_half_w, w);
    ClampSpan(&f.right_brow.cx, brow_half_w, w);

    // --- mouth -------------------------------------------------------------
    const float open_fraction = std::max(mouth_floor, mouth_open);
    f.mouth.cx = w / 2;
    f.mouth.cy = Round(s.mouth_cy * fh);
    f.mouth.half_w = std::max(1, Round(s.mouth_half_w * fw * mouth_w_scale));
    f.mouth.thickness = std::max(1, Round(s.mouth_thickness * fh));
    f.mouth.open = Round(s.mouth_open_max * fh * open_fraction);
    f.mouth.corner_lift = corner_lift_px;

    // The mouth's drawn extent is its opening plus its stroke plus whatever
    // the corners are doing, and all of it has to fit.
    const int mouth_half_h = f.mouth.open / 2 + f.mouth.thickness +
                             (f.mouth.corner_lift < 0 ? -f.mouth.corner_lift
                                                      : f.mouth.corner_lift);
    ClampSpan(&f.mouth.cx, f.mouth.half_w, w);
    ClampSpan(&f.mouth.cy, mouth_half_h, h);

    // --- blush -------------------------------------------------------------
    if (blush) {
        const int brx = std::max(1, Round(s.blush_rx * fw));
        const int bry = std::max(1, Round(s.blush_ry * fh));
        const int bdx = Round(s.blush_dx * fw);
        f.left_blush.visible = f.right_blush.visible = true;
        f.left_blush.cx = w / 2 - bdx;
        f.right_blush.cx = w / 2 + bdx;
        f.left_blush.cy = f.right_blush.cy = Round(s.blush_cy * fh);
        f.left_blush.rx = f.right_blush.rx = brx;
        f.left_blush.ry = f.right_blush.ry = bry;
        ClampSpan(&f.left_blush.cx, brx, w);
        ClampSpan(&f.right_blush.cx, brx, w);
        ClampSpan(&f.left_blush.cy, bry, h);
        f.right_blush.cy = f.left_blush.cy;
    }
    return f;
}

// The mouth's full drawn ellipse: half-height covers the opening, the stroke
// and whatever the corners are doing.
inline int MouthHalfHeight(const Mouth& m) {
    const int curve = m.corner_lift < 0 ? -m.corner_lift : m.corner_lift;
    return (m.open / 2) + m.thickness + curve;
}

inline MouthMask ComputeMouthMask(const Mouth& m) {
    MouthMask mask;
    const int curve = m.corner_lift < 0 ? -m.corner_lift : m.corner_lift;
    if (curve <= 0 || m.open >= m.thickness) return mask;   // nothing to hide

    const int half_h = MouthHalfHeight(m);
    // Keep a crescent this tall at the smiling (or frowning) edge.
    const int keep = m.thickness * 2 + m.open;
    const int mask_h = half_h * 2 - keep;
    if (mask_h <= 0) return mask;

    // Overhang on all sides. A rectangle covers [y, y+h), the ellipse reaches
    // cy+half_h inclusive, and without the margin exactly one row of the half
    // that should be hidden survives — a stray mark under every frown. It is
    // one pixel, it is on the outside edge where nothing else is drawn, and it
    // is the kind of thing only a picture shows you.
    constexpr int kOverhang = 2;
    mask.visible = true;
    mask.w = m.half_w * 2 + kOverhang * 2;
    mask.h = mask_h + kOverhang;
    mask.x = m.cx - m.half_w - kOverhang;
    mask.y = (m.corner_lift > 0)
                 ? (m.cy - half_h - kOverhang)   // smile: hide the top
                 : (m.cy + half_h - mask_h);     // frown: hide the bottom
    return mask;
}

}  // namespace avatar
}  // namespace stackchan
