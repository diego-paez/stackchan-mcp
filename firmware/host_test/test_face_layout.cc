// Host tests for the drawn face.
//
// The requirement these encode is "it should take almost all the screen":
// the old avatar was a 160x120 bitmap with small features, upscaled into the
// middle of a 320x240 panel. FaceFillsTheScreen is the assertion that stops
// that regressing quietly.
//
// The rest is the usual: nothing drawn off the panel, nothing inverted,
// nothing overlapping, and the same proportions on a different display.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "avatar/face_layout.h"

using namespace stackchan::avatar;

namespace {

const Expression kAll[] = {Expression::kIdle,      Expression::kHappy,
                           Expression::kThinking,  Expression::kSad,
                           Expression::kSurprised, Expression::kEmbarrassed};

Canvas Screen() { return Canvas{320, 240}; }

struct Box { int x0, y0, x1, y1; };

void Grow(Box* b, int cx, int cy, int rx, int ry) {
    if (cx - rx < b->x0) b->x0 = cx - rx;
    if (cy - ry < b->y0) b->y0 = cy - ry;
    if (cx + rx > b->x1) b->x1 = cx + rx;
    if (cy + ry > b->y1) b->y1 = cy + ry;
}

// Bounding box of everything actually drawn.
Box Extent(const FaceLayout& f) {
    Box b{1 << 20, 1 << 20, -(1 << 20), -(1 << 20)};
    Grow(&b, f.left_eye.cx, f.left_eye.cy, f.left_eye.rx, f.left_eye.ry);
    Grow(&b, f.right_eye.cx, f.right_eye.cy, f.right_eye.rx, f.right_eye.ry);
    if (f.left_brow.visible) {
        Grow(&b, f.left_brow.cx, f.left_brow.cy, f.left_brow.half_w, f.left_brow.thickness);
        Grow(&b, f.right_brow.cx, f.right_brow.cy, f.right_brow.half_w, f.right_brow.thickness);
    }
    Grow(&b, f.mouth.cx, f.mouth.cy, f.mouth.half_w,
         f.mouth.open / 2 + f.mouth.thickness);
    return b;
}

}  // namespace

// ---------------------------------------------------------------------------
// The point of the whole change
// ---------------------------------------------------------------------------

TEST(FaceLayout, FaceFillsTheScreen) {
    const Canvas c = Screen();
    for (Expression e : kAll) {
        const FaceLayout f = ComputeFace(c, e);
        const Box b = Extent(f);
        const float wf = static_cast<float>(b.x1 - b.x0) / c.width;
        const float hf = static_cast<float>(b.y1 - b.y0) / c.height;
        EXPECT_GT(wf, 0.75f) << ToString(e) << " spans only " << wf << " of the width";
        EXPECT_GT(hf, 0.68f) << ToString(e) << " spans only " << hf << " of the height";
    }
}

TEST(FaceLayout, TheEyesAreBigEnoughToReadAcrossARoom) {
    // The old face had features a few pixels across after upscaling. An eye
    // should be a substantial object, not a dot.
    const Canvas c = Screen();
    const FaceLayout f = ComputeFace(c, Expression::kIdle);
    EXPECT_GE(f.left_eye.rx * 2, c.width / 5)
        << "eye is " << f.left_eye.rx * 2 << " px wide on a " << c.width << " px screen";
    EXPECT_GE(f.left_eye.ry * 2, c.height / 4);
    EXPECT_GE(f.mouth.half_w * 2, c.width / 4);
}

// ---------------------------------------------------------------------------
// Nothing off the panel, in any state
// ---------------------------------------------------------------------------

TEST(FaceLayout, NothingIsEverDrawnOffTheScreen) {
    for (const Canvas c : {Canvas{320, 240}, Canvas{240, 240}, Canvas{128, 128},
                           Canvas{480, 320}}) {
        for (Expression e : kAll) {
            for (int b = 0; b <= 10; ++b) {
                for (int m = 0; m <= 10; ++m) {
                    const FaceLayout f = ComputeFace(c, e, b / 10.0f, m / 10.0f);
                    const Box box = Extent(f);
                    EXPECT_GE(box.x0, 0) << ToString(e) << " on " << c.width << "x" << c.height;
                    EXPECT_GE(box.y0, 0) << ToString(e) << " on " << c.width << "x" << c.height;
                    EXPECT_LE(box.x1, c.width) << ToString(e);
                    EXPECT_LE(box.y1, c.height) << ToString(e);
                }
            }
        }
    }
}

TEST(FaceLayout, NoFeatureEverHasANegativeOrZeroSize) {
    for (const Canvas c : {Canvas{320, 240}, Canvas{16, 16}, Canvas{1, 1}, Canvas{0, 0}}) {
        for (Expression e : kAll) {
            const FaceLayout f = ComputeFace(c, e, 1.0f, 1.0f);
            EXPECT_GT(f.left_eye.rx, 0);
            EXPECT_GT(f.left_eye.ry, 0);
            EXPECT_GT(f.right_eye.rx, 0);
            EXPECT_GT(f.right_eye.ry, 0);
            EXPECT_GT(f.mouth.half_w, 0);
            EXPECT_GT(f.mouth.thickness, 0);
            EXPECT_GE(f.mouth.open, 0);
        }
    }
}

TEST(FaceLayout, TheEyesAreSymmetricAboutTheMidline) {
    const Canvas c = Screen();
    for (Expression e : kAll) {
        const FaceLayout f = ComputeFace(c, e);
        EXPECT_EQ(f.left_eye.cy, f.right_eye.cy) << ToString(e);
        EXPECT_EQ(f.left_eye.rx, f.right_eye.rx);
        EXPECT_EQ(f.left_eye.ry, f.right_eye.ry);
        EXPECT_EQ(c.width - f.right_eye.cx, f.left_eye.cx)
            << ToString(e) << ": the eyes are not mirrored";
        // Brows mirror in tilt as well as position, or one eyebrow ends up
        // shrugging while the other frowns.
        EXPECT_EQ(f.left_brow.tilt_deg, -f.right_brow.tilt_deg) << ToString(e);
    }
}

TEST(FaceLayout, TheEyesNeverCollideWithEachOtherOrTheMouth) {
    const Canvas c = Screen();
    for (Expression e : kAll) {
        for (int m = 0; m <= 10; ++m) {
            const FaceLayout f = ComputeFace(c, e, 0.0f, m / 10.0f);
            EXPECT_LT(f.left_eye.cx + f.left_eye.rx, f.right_eye.cx - f.right_eye.rx)
                << ToString(e) << ": the eyes overlap";
            const int eye_bottom = f.left_eye.cy + f.left_eye.ry;
            const int mouth_top = f.mouth.cy - f.mouth.open / 2 - f.mouth.thickness;
            EXPECT_LT(eye_bottom, mouth_top)
                << ToString(e) << " at mouth=" << m << ": the mouth reaches the eyes";
        }
    }
}

// ---------------------------------------------------------------------------
// Blinking and speaking
// ---------------------------------------------------------------------------

TEST(FaceLayout, BlinkingClosesTheEyesWithoutMovingThem) {
    const Canvas c = Screen();
    const FaceLayout open = ComputeFace(c, Expression::kIdle, 0.0f);
    const FaceLayout shut = ComputeFace(c, Expression::kIdle, 1.0f);

    EXPECT_LT(shut.left_eye.ry, open.left_eye.ry / 3)
        << "a closed eye should be a line, not a slightly shorter eye";
    EXPECT_GT(shut.left_eye.ry, 0) << "a closed eye is still drawn";
    EXPECT_EQ(shut.left_eye.cy, open.left_eye.cy)
        << "the eye drifted up the face as it closed";
    EXPECT_EQ(shut.left_eye.rx, open.left_eye.rx)
        << "an eyelid closes vertically; the width should not change";
    EXPECT_EQ(shut.left_brow.cy, open.left_brow.cy)
        << "the brows should not move when the eyes blink";
}

TEST(FaceLayout, BlinkingIsMonotonic) {
    const Canvas c = Screen();
    int prev = 1 << 20;
    for (int b = 0; b <= 10; ++b) {
        const FaceLayout f = ComputeFace(c, Expression::kIdle, b / 10.0f);
        EXPECT_LE(f.left_eye.ry, prev) << "eye height grew partway through a blink";
        prev = f.left_eye.ry;
    }
}

TEST(FaceLayout, TheMouthOpensForSpeech) {
    const Canvas c = Screen();
    const FaceLayout shut = ComputeFace(c, Expression::kIdle, 0.0f, 0.0f);
    const FaceLayout wide = ComputeFace(c, Expression::kIdle, 0.0f, 1.0f);
    EXPECT_EQ(shut.mouth.open, 0) << "a closed mouth is a line";
    EXPECT_GT(wide.mouth.open, c.height / 10) << "a full open mouth should be obvious";
    EXPECT_EQ(shut.mouth.cx, wide.mouth.cx) << "the mouth should open, not slide";
}

// ---------------------------------------------------------------------------
// The expressions actually differ
// ---------------------------------------------------------------------------

TEST(FaceLayout, HappyAndSadAreOppositeAtTheCorners) {
    const Canvas c = Screen();
    const FaceLayout happy = ComputeFace(c, Expression::kHappy);
    const FaceLayout sad = ComputeFace(c, Expression::kSad);
    EXPECT_GT(happy.mouth.corner_lift, 0) << "happy should smile";
    EXPECT_LT(sad.mouth.corner_lift, 0) << "sad should frown";
}

TEST(FaceLayout, WorryTiltsTheBrowsInwardAndUp) {
    const Canvas c = Screen();
    EXPECT_GT(ComputeFace(c, Expression::kSad).left_brow.tilt_deg, 0);
    // Concentration is the other way round.
    EXPECT_LT(ComputeFace(c, Expression::kThinking).left_brow.tilt_deg, 0);
    EXPECT_EQ(ComputeFace(c, Expression::kIdle).left_brow.tilt_deg, 0);
}

TEST(FaceLayout, SurpriseHasTheWidestEyesAndAnOpenMouth) {
    const Canvas c = Screen();
    const FaceLayout surprised = ComputeFace(c, Expression::kSurprised);
    for (Expression e : kAll) {
        if (e == Expression::kSurprised) continue;
        EXPECT_GE(surprised.left_eye.ry, ComputeFace(c, e).left_eye.ry)
            << "surprise should not have smaller eyes than " << ToString(e);
    }
    EXPECT_GT(surprised.mouth.open, 0) << "surprise should hold its mouth open at rest";
}

TEST(FaceLayout, EmbarrassedIsTheOnlyOneThatBlushes) {
    const Canvas c = Screen();
    for (Expression e : kAll) {
        const FaceLayout f = ComputeFace(c, e);
        const bool want = (e == Expression::kEmbarrassed);
        EXPECT_EQ(f.left_blush.visible, want) << ToString(e);
        EXPECT_EQ(f.right_blush.visible, f.left_blush.visible);
    }
}

TEST(FaceLayout, EveryExpressionLooksDifferentFromEveryOther) {
    // Six names that render identically would be six lies.
    const Canvas c = Screen();
    std::vector<std::string> seen;
    for (Expression e : kAll) {
        const FaceLayout f = ComputeFace(c, e);
        std::string key = std::to_string(f.left_eye.rx) + "," + std::to_string(f.left_eye.ry) +
                          "," + std::to_string(f.left_brow.tilt_deg) + "," +
                          std::to_string(f.left_brow.cy) + "," +
                          std::to_string(f.mouth.half_w) + "," +
                          std::to_string(f.mouth.open) + "," +
                          std::to_string(f.mouth.corner_lift) + "," +
                          std::to_string(f.left_blush.visible);
        for (const std::string& s : seen) {
            EXPECT_NE(s, key) << ToString(e) << " renders identically to another expression";
        }
        seen.push_back(key);
    }
}

// ---------------------------------------------------------------------------
// Other panels
// ---------------------------------------------------------------------------

TEST(FaceLayout, ProportionsHoldOnADifferentPanel) {
    const FaceLayout small = ComputeFace(Canvas{320, 240}, Expression::kIdle);
    const FaceLayout big = ComputeFace(Canvas{640, 480}, Expression::kIdle);
    EXPECT_NEAR(big.left_eye.rx, small.left_eye.rx * 2, 2);
    EXPECT_NEAR(big.left_eye.ry, small.left_eye.ry * 2, 2);
    EXPECT_NEAR(big.mouth.cy, small.mouth.cy * 2, 2);
    EXPECT_NEAR(big.left_eye.cx, small.left_eye.cx * 2, 2);
}

TEST(FaceLayout, ASquarePanelStillGetsAWholeFace) {
    const Canvas c{240, 240};
    const FaceLayout f = ComputeFace(c, Expression::kSurprised, 0.0f, 1.0f);
    const Box b = Extent(f);
    EXPECT_GE(b.x0, 0);
    EXPECT_GE(b.y0, 0);
    EXPECT_LE(b.x1, c.width);
    EXPECT_LE(b.y1, c.height);
    EXPECT_GT(static_cast<float>(b.y1 - b.y0) / c.height, 0.55f);
}

// ---------------------------------------------------------------------------
// The mouth mask
//
// Found by rendering the face, not by reading it: the mask covered [y, y+h)
// while the ellipse reached cy+half_h inclusive, so one row of the hidden
// half survived as a stray mark under every frown.
// ---------------------------------------------------------------------------

TEST(FaceLayout, TheMouthMaskFullyCoversTheHalfItHides) {
    const Canvas c = Screen();
    for (Expression e : kAll) {
        const FaceLayout f = ComputeFace(c, e);
        const MouthMask m = ComputeMouthMask(f.mouth);
        if (!m.visible) continue;

        const int half_h = MouthHalfHeight(f.mouth);
        const int ell_top = f.mouth.cy - half_h;
        const int ell_bottom = f.mouth.cy + half_h;
        const int ell_left = f.mouth.cx - f.mouth.half_w;
        const int ell_right = f.mouth.cx + f.mouth.half_w;

        EXPECT_LE(m.x, ell_left) << ToString(e) << ": mask leaves a left rim";
        EXPECT_GE(m.x + m.w, ell_right + 1) << ToString(e) << ": mask leaves a right rim";
        if (f.mouth.corner_lift > 0) {
            EXPECT_LE(m.y, ell_top) << ToString(e) << ": smile mask misses the top";
        } else {
            EXPECT_GE(m.y + m.h, ell_bottom + 1)
                << ToString(e) << ": frown mask misses the bottom row";
        }
    }
}

TEST(FaceLayout, TheMaskNeverSwallowsTheWholeMouth) {
    const Canvas c = Screen();
    for (Expression e : kAll) {
        for (int mo = 0; mo <= 10; ++mo) {
            const FaceLayout f = ComputeFace(c, e, 0.0f, mo / 10.0f);
            const MouthMask m = ComputeMouthMask(f.mouth);
            if (!m.visible) continue;
            const int half_h = MouthHalfHeight(f.mouth);
            const int visible_band = half_h * 2 - m.h;
            EXPECT_GT(visible_band, 0)
                << ToString(e) << " at mouth=" << mo << ": the mouth is entirely masked";
        }
    }
}

TEST(FaceLayout, AnOpenMouthIsNotMaskedAtAll) {
    // Speech wins over the smile curve: a wide-open mouth is a shape, not a
    // crescent, and masking half of it would make the robot look toothless.
    const Canvas c = Screen();
    const FaceLayout f = ComputeFace(c, Expression::kHappy, 0.0f, 1.0f);
    EXPECT_FALSE(ComputeMouthMask(f.mouth).visible);
}
