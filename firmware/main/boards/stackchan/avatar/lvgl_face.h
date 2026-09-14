// lvgl_face.h — drawing the computed face with LVGL primitives.
//
// The thin half of the split: face_layout.h decides where everything goes and
// is tested on a laptop; this turns those numbers into widgets and is the
// part that needs a screen to judge. Nothing here does arithmetic about the
// face beyond converting centre-and-radius into LVGL's top-left-and-size.
//
// Everything is a rectangle or an ellipse, deliberately. LVGL will draw arcs
// and polygons, but a shape built from primitives whose geometry is already
// asserted on the host is a shape that cannot be subtly wrong in a way only a
// desk visit would reveal. The mouth curve is the one place that needs a
// trick: a smile is the bottom half of an ellipse, so the mouth is an ellipse
// with a background-coloured rectangle laid over the half that should not be
// there. A frown is the same trick upside down.
#pragma once

#ifdef ESP_PLATFORM

#include <lvgl.h>

#include "face_layout.h"

namespace stackchan {
namespace avatar {

struct FacePalette {
    // A flat ground over the entire panel: the point of the redesign is that
    // there is no picture-of-a-face-on-a-screen, only a face.
    uint32_t background = 0x101820;   // near-black with a little blue in it
    uint32_t feature = 0xF2F4F8;      // eyes, brows, mouth
    uint32_t blush = 0xE2716B;
};

class LvglFace {
public:
    LvglFace() = default;
    ~LvglFace();

    LvglFace(const LvglFace&) = delete;
    LvglFace& operator=(const LvglFace&) = delete;

    // Builds the widgets on `screen` and paints the first frame. The caller
    // must hold the LVGL lock. Returns false if any widget could not be
    // created, in which case nothing is left half-built.
    bool begin(lv_obj_t* screen, const FacePalette& palette = FacePalette{});
    bool ready() const { return root_ != nullptr; }

    // All of these are cheap and idempotent; each redraws only if something
    // actually changed. The caller must hold the LVGL lock.
    void setExpression(Expression e);
    void setBlink(float blink);        // 0 open .. 1 closed
    void setMouthOpen(float open);     // 0 closed .. 1 wide
    void setPalette(const FacePalette& p);

    Expression expression() const { return expression_; }

    void show();
    void hide();
    bool visible() const { return visible_; }

    // Destroys the widgets. Safe to call twice; safe to call on a face that
    // was never begun.
    void destroy();

private:
    void layout();
    lv_obj_t* makeShape(bool circle);

    lv_obj_t* root_ = nullptr;       // full-screen background
    lv_obj_t* left_eye_ = nullptr;
    lv_obj_t* right_eye_ = nullptr;
    lv_obj_t* left_brow_ = nullptr;
    lv_obj_t* right_brow_ = nullptr;
    lv_obj_t* mouth_ = nullptr;
    lv_obj_t* mouth_mask_ = nullptr;  // hides the half of the mouth ellipse
    lv_obj_t* left_blush_ = nullptr;  // that should not be drawn
    lv_obj_t* right_blush_ = nullptr;

    FacePalette palette_;
    Canvas canvas_;
    Expression expression_ = Expression::kIdle;
    float blink_ = 0.0f;
    float mouth_open_ = 0.0f;
    bool visible_ = true;
};

}  // namespace avatar
}  // namespace stackchan

#endif  // ESP_PLATFORM
