// sim_face.cc — look at the face without a robot.
//
// Not a test: it renders the same geometry the panel will draw, with the same
// ellipses and the same mouth mask, and writes a PNG contact sheet. The host
// tests prove the face fits the screen and that the expressions differ; only
// a picture tells you whether it reads as a face.
//
// Build:  cmake --build <dir> --target sim_face && ./sim_face out.png
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

#include "avatar/face_layout.h"

using namespace stackchan::avatar;

namespace {

constexpr int kTileW = 320;
constexpr int kTileH = 240;
constexpr int kGap = 8;

struct Image {
    int w, h;
    std::vector<uint8_t> px;   // RGB
    Image(int w_, int h_) : w(w_), h(h_), px(static_cast<size_t>(w_) * h_ * 3, 0x18) {}
    void set(int x, int y, uint32_t rgb) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        uint8_t* p = &px[(static_cast<size_t>(y) * w + x) * 3];
        p[0] = (rgb >> 16) & 0xFF;
        p[1] = (rgb >> 8) & 0xFF;
        p[2] = rgb & 0xFF;
    }
};

void FillRect(Image& im, int x0, int y0, int w, int h, uint32_t rgb) {
    for (int y = y0; y < y0 + h; ++y)
        for (int x = x0; x < x0 + w; ++x) im.set(x, y, rgb);
}

void FillEllipse(Image& im, int cx, int cy, int rx, int ry, uint32_t rgb) {
    if (rx <= 0 || ry <= 0) return;
    for (int y = cy - ry; y <= cy + ry; ++y) {
        for (int x = cx - rx; x <= cx + rx; ++x) {
            const float dx = static_cast<float>(x - cx) / rx;
            const float dy = static_cast<float>(y - cy) / ry;
            if (dx * dx + dy * dy <= 1.0f) im.set(x, y, rgb);
        }
    }
}

// Brows are rotated about their own centre, the way LVGL's transform does it.
void FillRotatedEllipse(Image& im, int cx, int cy, int rx, int ry, int deg, uint32_t rgb) {
    if (rx <= 0 || ry <= 0) return;
    const float rad = deg * 3.14159265f / 180.0f;
    const float cs = std::cos(rad), sn = std::sin(rad);
    const int reach = rx + ry + 2;
    for (int y = cy - reach; y <= cy + reach; ++y) {
        for (int x = cx - reach; x <= cx + reach; ++x) {
            const float ox = static_cast<float>(x - cx), oy = static_cast<float>(y - cy);
            const float u = (ox * cs + oy * sn) / rx;
            const float v = (-ox * sn + oy * cs) / ry;
            if (u * u + v * v <= 1.0f) im.set(x, y, rgb);
        }
    }
}

void DrawFace(Image& im, int ox, int oy, Expression e, float blink, float mouth) {
    const uint32_t bg = 0x101820, fg = 0xF2F4F8, blush_c = 0xE2716B;
    const Canvas c{kTileW, kTileH};
    const FaceLayout f = ComputeFace(c, e, blink, mouth);

    FillRect(im, ox, oy, kTileW, kTileH, bg);

    if (f.left_blush.visible) {
        FillEllipse(im, ox + f.left_blush.cx, oy + f.left_blush.cy, f.left_blush.rx,
                    f.left_blush.ry, blush_c);
        FillEllipse(im, ox + f.right_blush.cx, oy + f.right_blush.cy, f.right_blush.rx,
                    f.right_blush.ry, blush_c);
    }

    FillRotatedEllipse(im, ox + f.left_brow.cx, oy + f.left_brow.cy, f.left_brow.half_w,
                       f.left_brow.thickness, f.left_brow.tilt_deg, fg);
    FillRotatedEllipse(im, ox + f.right_brow.cx, oy + f.right_brow.cy, f.right_brow.half_w,
                       f.right_brow.thickness, f.right_brow.tilt_deg, fg);

    FillEllipse(im, ox + f.left_eye.cx, oy + f.left_eye.cy, f.left_eye.rx, f.left_eye.ry, fg);
    FillEllipse(im, ox + f.right_eye.cx, oy + f.right_eye.cy, f.right_eye.rx, f.right_eye.ry, fg);

    FillEllipse(im, ox + f.mouth.cx, oy + f.mouth.cy, f.mouth.half_w,
                MouthHalfHeight(f.mouth), fg);
    const MouthMask mask = ComputeMouthMask(f.mouth);
    if (mask.visible) FillRect(im, ox + mask.x, oy + mask.y, mask.w, mask.h, bg);
}

void WritePpm(const Image& im, const char* path) {
    FILE* f = std::fopen(path, "wb");
    if (f == nullptr) { std::perror("open"); return; }
    std::fprintf(f, "P6\n%d %d\n255\n", im.w, im.h);
    std::fwrite(im.px.data(), 1, im.px.size(), f);
    std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
    const char* out = (argc > 1) ? argv[1] : "face.ppm";

    struct Cell { Expression e; float blink; float mouth; const char* label; };
    const Cell cells[] = {
        {Expression::kIdle, 0.0f, 0.0f, "idle"},
        {Expression::kHappy, 0.0f, 0.0f, "happy"},
        {Expression::kThinking, 0.0f, 0.0f, "thinking"},
        {Expression::kSad, 0.0f, 0.0f, "sad"},
        {Expression::kSurprised, 0.0f, 0.0f, "surprised"},
        {Expression::kEmbarrassed, 0.0f, 0.0f, "embarrassed"},
        {Expression::kIdle, 1.0f, 0.0f, "idle, blinking"},
        {Expression::kHappy, 0.5f, 0.45f, "happy, mid-blink, speaking"},
        {Expression::kIdle, 0.0f, 1.0f, "idle, mouth wide"},
    };
    const int cols = 3;
    const int rows = 3;
    Image sheet(cols * kTileW + (cols + 1) * kGap, rows * kTileH + (rows + 1) * kGap);

    for (int i = 0; i < 9; ++i) {
        const int cx = i % cols, cy = i / cols;
        DrawFace(sheet, kGap + cx * (kTileW + kGap), kGap + cy * (kTileH + kGap),
                 cells[i].e, cells[i].blink, cells[i].mouth);
        std::printf("  %-28s eyes %3dx%-3d\n", cells[i].label,
                    ComputeFace(Canvas{kTileW, kTileH}, cells[i].e, cells[i].blink,
                                cells[i].mouth).left_eye.rx * 2,
                    ComputeFace(Canvas{kTileW, kTileH}, cells[i].e, cells[i].blink,
                                cells[i].mouth).left_eye.ry * 2);
    }
    WritePpm(sheet, out);
    std::printf("\n  wrote %s (%dx%d)\n", out, sheet.w, sheet.h);
    return 0;
}
