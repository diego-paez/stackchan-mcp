#include "lvgl_face.h"

#ifdef ESP_PLATFORM

#include <esp_log.h>

#define TAG "LvglFace"

namespace stackchan {
namespace avatar {
namespace {

// LVGL positions by top-left corner and size; the layout speaks in centres
// and radii, which is what the drawing maths wants. One place to convert.
void PlaceEllipse(lv_obj_t* obj, int cx, int cy, int rx, int ry) {
    if (obj == nullptr) return;
    const int w = rx * 2;
    const int h = ry * 2;
    lv_obj_set_size(obj, w, h);
    lv_obj_set_pos(obj, cx - rx, cy - ry);
}

}  // namespace

LvglFace::~LvglFace() { destroy(); }

lv_obj_t* LvglFace::makeShape(bool circle) {
    lv_obj_t* o = lv_obj_create(root_);
    if (o == nullptr) return nullptr;
    lv_obj_remove_style_all(o);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(o, lv_color_hex(palette_.feature), 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_set_style_radius(o, circle ? LV_RADIUS_CIRCLE : 0, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
    return o;
}

bool LvglFace::begin(lv_obj_t* screen, const FacePalette& palette) {
    if (screen == nullptr) return false;
    destroy();
    palette_ = palette;

    canvas_.width = lv_obj_get_width(screen);
    canvas_.height = lv_obj_get_height(screen);
    if (canvas_.width <= 0 || canvas_.height <= 0) {
        // Before the first layout pass the screen can still report zero.
        // Falling back to the panel constants is better than drawing nothing.
        canvas_.width = LV_HOR_RES;
        canvas_.height = LV_VER_RES;
    }

    root_ = lv_obj_create(screen);
    if (root_ == nullptr) {
        ESP_LOGE(TAG, "could not create the face root");
        return false;
    }
    lv_obj_remove_style_all(root_);
    lv_obj_set_size(root_, canvas_.width, canvas_.height);
    lv_obj_set_pos(root_, 0, 0);
    lv_obj_set_style_bg_opa(root_, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(root_, lv_color_hex(palette_.background), 0);
    lv_obj_set_style_border_width(root_, 0, 0);
    lv_obj_set_style_pad_all(root_, 0, 0);
    lv_obj_set_style_radius(root_, 0, 0);
    lv_obj_clear_flag(root_, LV_OBJ_FLAG_SCROLLABLE);

    left_brow_ = makeShape(true);
    right_brow_ = makeShape(true);
    left_eye_ = makeShape(true);
    right_eye_ = makeShape(true);
    left_blush_ = makeShape(true);
    right_blush_ = makeShape(true);
    mouth_ = makeShape(true);
    // The mask is painted in the BACKGROUND colour and sits on top of the
    // mouth, so the half of the ellipse it covers simply is not there. It is
    // created last so it stacks above the mouth.
    mouth_mask_ = makeShape(false);

    if (left_eye_ == nullptr || right_eye_ == nullptr || left_brow_ == nullptr ||
        right_brow_ == nullptr || mouth_ == nullptr || mouth_mask_ == nullptr ||
        left_blush_ == nullptr || right_blush_ == nullptr) {
        ESP_LOGE(TAG, "could not create the face widgets");
        destroy();
        return false;
    }

    lv_obj_set_style_bg_color(left_blush_, lv_color_hex(palette_.blush), 0);
    lv_obj_set_style_bg_color(right_blush_, lv_color_hex(palette_.blush), 0);
    lv_obj_set_style_bg_color(mouth_mask_, lv_color_hex(palette_.background), 0);

    layout();
    ESP_LOGI(TAG, "drawn face ready: %dx%d, eyes fill the panel",
             canvas_.width, canvas_.height);
    return true;
}

void LvglFace::destroy() {
    if (root_ != nullptr) {
        lv_obj_del(root_);   // takes the children with it
        root_ = nullptr;
    }
    left_eye_ = right_eye_ = left_brow_ = right_brow_ = nullptr;
    mouth_ = mouth_mask_ = left_blush_ = right_blush_ = nullptr;
}

void LvglFace::setPalette(const FacePalette& p) {
    palette_ = p;
    if (root_ == nullptr) return;
    lv_obj_set_style_bg_color(root_, lv_color_hex(palette_.background), 0);
    for (lv_obj_t* o : {left_eye_, right_eye_, left_brow_, right_brow_, mouth_}) {
        if (o != nullptr) lv_obj_set_style_bg_color(o, lv_color_hex(palette_.feature), 0);
    }
    if (left_blush_) lv_obj_set_style_bg_color(left_blush_, lv_color_hex(palette_.blush), 0);
    if (right_blush_) lv_obj_set_style_bg_color(right_blush_, lv_color_hex(palette_.blush), 0);
    if (mouth_mask_) lv_obj_set_style_bg_color(mouth_mask_, lv_color_hex(palette_.background), 0);
}

void LvglFace::setExpression(Expression e) {
    if (e == expression_) return;
    expression_ = e;
    layout();
}

void LvglFace::setBlink(float blink) {
    if (blink < 0.0f) blink = 0.0f;
    if (blink > 1.0f) blink = 1.0f;
    if (blink == blink_) return;
    blink_ = blink;
    layout();
}

void LvglFace::setMouthOpen(float open) {
    if (open < 0.0f) open = 0.0f;
    if (open > 1.0f) open = 1.0f;
    if (open == mouth_open_) return;
    mouth_open_ = open;
    layout();
}

void LvglFace::show() {
    visible_ = true;
    if (root_ != nullptr) {
        lv_obj_clear_flag(root_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(root_);
    }
}

void LvglFace::hide() {
    visible_ = false;
    if (root_ != nullptr) lv_obj_add_flag(root_, LV_OBJ_FLAG_HIDDEN);
}

void LvglFace::layout() {
    if (root_ == nullptr) return;
    const FaceLayout f = ComputeFace(canvas_, expression_, blink_, mouth_open_);

    PlaceEllipse(left_eye_, f.left_eye.cx, f.left_eye.cy, f.left_eye.rx, f.left_eye.ry);
    PlaceEllipse(right_eye_, f.right_eye.cx, f.right_eye.cy, f.right_eye.rx, f.right_eye.ry);

    PlaceEllipse(left_brow_, f.left_brow.cx, f.left_brow.cy, f.left_brow.half_w,
                 f.left_brow.thickness);
    PlaceEllipse(right_brow_, f.right_brow.cx, f.right_brow.cy, f.right_brow.half_w,
                 f.right_brow.thickness);
    // LVGL takes rotation in tenths of a degree, and rotates about the
    // widget's centre once the pivot is set there.
    lv_obj_set_style_transform_pivot_x(left_brow_, f.left_brow.half_w, 0);
    lv_obj_set_style_transform_pivot_y(left_brow_, f.left_brow.thickness, 0);
    lv_obj_set_style_transform_pivot_x(right_brow_, f.right_brow.half_w, 0);
    lv_obj_set_style_transform_pivot_y(right_brow_, f.right_brow.thickness, 0);
    lv_obj_set_style_transform_rotation(left_brow_, f.left_brow.tilt_deg * 10, 0);
    lv_obj_set_style_transform_rotation(right_brow_, f.right_brow.tilt_deg * 10, 0);

    // --- the mouth ---------------------------------------------------------
    // One ellipse, and a mask over the half that should not show. A smile is
    // the bottom of an ellipse; a frown is the top; a neutral mouth is a bar
    // thin enough that the distinction does not arise.
    const int half_h = MouthHalfHeight(f.mouth);
    PlaceEllipse(mouth_, f.mouth.cx, f.mouth.cy, f.mouth.half_w, half_h);

    const MouthMask mask = ComputeMouthMask(f.mouth);
    if (mask.visible) {
        lv_obj_set_size(mouth_mask_, mask.w, mask.h);
        lv_obj_set_pos(mouth_mask_, mask.x, mask.y);
        lv_obj_clear_flag(mouth_mask_, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(mouth_mask_, LV_OBJ_FLAG_HIDDEN);
    }

    // --- blush -------------------------------------------------------------
    if (f.left_blush.visible) {
        PlaceEllipse(left_blush_, f.left_blush.cx, f.left_blush.cy, f.left_blush.rx,
                     f.left_blush.ry);
        PlaceEllipse(right_blush_, f.right_blush.cx, f.right_blush.cy, f.right_blush.rx,
                     f.right_blush.ry);
        lv_obj_clear_flag(left_blush_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(right_blush_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_background(left_blush_);
        lv_obj_move_background(right_blush_);
    } else {
        lv_obj_add_flag(left_blush_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(right_blush_, LV_OBJ_FLAG_HIDDEN);
    }
}

}  // namespace avatar
}  // namespace stackchan

#endif  // ESP_PLATFORM
