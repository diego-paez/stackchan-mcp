// SPDX-License-Identifier: MIT
//
// Host tests for the mBot2 link: the F3 codec (byte-compared with the
// reference Python codec, mblock-stacky-bridge hub/stacky_hub/transports/f3.py)
// and the host-side command validation that enforces mbot_limits.h.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "f3_codec.h"
#include "mbot_command.h"
#include "mbot_limits.h"
#include "teleop_packet.h"

namespace {

std::vector<uint8_t> FromHex(const std::string& hex) {
    std::vector<uint8_t> out;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
        out.push_back(static_cast<uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    }
    return out;
}

// encode_message("sk", "12 fwd 40 1") from f3.py
const char* kEncodedFwd =
    "f31b2800037b226d657373616765223a22736b222c2276616c7565223a223132206677642034302031227d0024f4";
// A frame as the CyberPi sends it: JSON with spaces.
const char* kSpacedAck =
    "f31d2a00037b226d657373616765223a2022736b5f61636b222c202276616c7565223a20223132206f6b20227d00f6f4";

}  // namespace

// ---------------------------------------------------------------- codec

TEST(F3Codec, EncodeMatchesPythonReference) {
    EXPECT_EQ(mbot::EncodeMessage("sk", "12 fwd 40 1"), FromHex(kEncodedFwd));
}

TEST(F3Codec, DecodesSpacedJsonFromCyberPi) {
    mbot::F3Decoder dec;
    std::vector<mbot::F3Frame> frames;
    auto raw = FromHex(kSpacedAck);
    dec.Feed(raw.data(), raw.size(), frames);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].type, mbot::kF3TypeUploadMsg);
    std::string topic, value;
    ASSERT_TRUE(mbot::ParseUploadMessage(frames[0].text, &topic, &value));
    EXPECT_EQ(topic, "sk_ack");
    EXPECT_EQ(value, "12 ok ");
}

TEST(F3Codec, RoundTripCompactJson) {
    mbot::F3Decoder dec;
    std::vector<mbot::F3Frame> frames;
    auto raw = mbot::EncodeMessage("sk_evt", "obstacle 7");
    dec.Feed(raw.data(), raw.size(), frames);
    ASSERT_EQ(frames.size(), 1u);
    std::string topic, value;
    ASSERT_TRUE(mbot::ParseUploadMessage(frames[0].text, &topic, &value));
    EXPECT_EQ(topic, "sk_evt");
    EXPECT_EQ(value, "obstacle 7");
}

TEST(F3Codec, ResyncsAfterGarbageAndSplitsAcrossFeeds) {
    // Same sequence as the f3.py check: garbage + frame + partial, then rest + frame.
    auto fr = FromHex(kSpacedAck);
    std::vector<uint8_t> first = {0x00, 0x11};
    first.insert(first.end(), fr.begin(), fr.end());
    first.insert(first.end(), fr.begin(), fr.begin() + 5);
    std::vector<uint8_t> second(fr.begin() + 5, fr.end());
    second.insert(second.end(), fr.begin(), fr.end());

    mbot::F3Decoder dec;
    std::vector<mbot::F3Frame> frames;
    dec.Feed(first.data(), first.size(), frames);
    EXPECT_EQ(frames.size(), 1u);
    dec.Feed(second.data(), second.size(), frames);
    EXPECT_EQ(frames.size(), 3u);
}

TEST(F3Codec, ByteByByteFeed) {
    auto fr = FromHex(kSpacedAck);
    mbot::F3Decoder dec;
    std::vector<mbot::F3Frame> frames;
    for (uint8_t b : fr) dec.Feed(&b, 1, frames);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].raw, fr);
}

TEST(F3Codec, DropsFrameWithBadFooter) {
    auto fr = FromHex(kSpacedAck);
    fr.back() = 0x00;
    mbot::F3Decoder dec;
    std::vector<mbot::F3Frame> frames;
    dec.Feed(fr.data(), fr.size(), frames);
    EXPECT_TRUE(frames.empty());
    auto good = FromHex(kSpacedAck);
    dec.Feed(good.data(), good.size(), frames);
    EXPECT_EQ(frames.size(), 1u);
}

TEST(F3Codec, LongGarbageDoesNotGrowForever) {
    mbot::F3Decoder dec;
    std::vector<mbot::F3Frame> frames;
    std::vector<uint8_t> junk(10000, 0x41);
    dec.Feed(junk.data(), junk.size(), frames);
    auto fr = FromHex(kSpacedAck);
    dec.Feed(fr.data(), fr.size(), frames);
    EXPECT_EQ(frames.size(), 1u);
}

TEST(F3Codec, ParseRejectsBadJson) {
    std::string topic, value;
    EXPECT_FALSE(mbot::ParseUploadMessage("{\"message\": ", &topic, &value));
    EXPECT_FALSE(mbot::ParseUploadMessage("{\"value\": \"x\"}", &topic, &value));
    ASSERT_TRUE(mbot::ParseUploadMessage("{\"message\": \"sk_evt\", \"value\": 42}", &topic, &value));
    EXPECT_EQ(value, "42");
}

// ---------------------------------------------------------------- limits

TEST(MbotLimits, MatchProtocolTable) {
    // docs/protocol.md "Safety limits" table. Changing a value here must be a
    // deliberate edit of the table and the mBot runtime as well.
    EXPECT_EQ(MBOT_MAX_RPM, 60);
    EXPECT_DOUBLE_EQ(MBOT_MAX_MOVE_S, 5.0);
    EXPECT_DOUBLE_EQ(MBOT_MAX_PROG_S, 30.0);
    EXPECT_EQ(MBOT_MAX_PROG_STEPS, 20);
    EXPECT_EQ(MBOT_ARM_MIN, 45);   // 5 deg inside the hardware range 40-120
    EXPECT_EQ(MBOT_ARM_MAX, 115);
    EXPECT_EQ(MBOT_ARM_HOME, 90);
    EXPECT_EQ(MBOT_GRIP_MIN, 50);  // 5 deg inside the hardware range 45-120
    EXPECT_EQ(MBOT_GRIP_MAX, 115);
    EXPECT_EQ(MBOT_GRIP_HOME, 90);
    EXPECT_EQ(MBOT_SERVO_DEG_PER_S, 40);
    EXPECT_EQ(MBOT_OBSTACLE_CM, 10);
    EXPECT_DOUBLE_EQ(MBOT_WATCHDOG_S, 3.0);
    EXPECT_DOUBLE_EQ(MBOT_HB_PERIOD_S, 1.0);
    EXPECT_EQ(MBOT_MAX_STEP_CM, 30);
    EXPECT_EQ(MBOT_MAX_TURN_DEG, 180);
}

// ---------------------------------------------------------------- commands

TEST(MbotCommand, MoveIsClamped) {
    auto r = mbot::CheckCommand("fwd 250 60");
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.normalized, "fwd 100 5");
    EXPECT_DOUBLE_EQ(r.seconds, MBOT_MAX_MOVE_S);
    EXPECT_FALSE(r.clamped.empty());
    EXPECT_TRUE(r.is_motion);

    r = mbot::CheckCommand("back -5 0.01");
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.normalized, "back 0 0.1");

    r = mbot::CheckCommand("LEFT 40 1.5");
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.normalized, "left 40 1.5");

    r = mbot::CheckCommand("right");
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.normalized, "right 40 1");
}

TEST(MbotCommand, RejectsNonNumbersAndInfinity) {
    EXPECT_FALSE(mbot::CheckCommand("fwd fast 1").ok);
    EXPECT_FALSE(mbot::CheckCommand("fwd 40 inf").ok);
    EXPECT_FALSE(mbot::CheckCommand("fwd nan 1").ok);
    EXPECT_FALSE(mbot::CheckCommand("fwd 40 1 extra").ok);
}

TEST(MbotCommand, RejectsUnknownCommands) {
    auto r = mbot::CheckCommand("jump 3");
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.error, "unknown_cmd:jump");
    EXPECT_FALSE(mbot::CheckCommand("").ok);
    EXPECT_FALSE(mbot::CheckCommand("wait 1").ok);  // only inside prog
    EXPECT_FALSE(mbot::CheckCommand("hb").ok);      // heartbeat is its own topic now
    EXPECT_FALSE(mbot::CheckCommand("fwd 40 1\xc3\xa9").ok);
}

TEST(MbotCommand, ServoPresetsAndAnglesAreClamped) {
    EXPECT_EQ(mbot::CheckCommand("arm up").normalized, "arm up");
    EXPECT_EQ(mbot::CheckCommand("arm 10").normalized, "arm 45");
    EXPECT_EQ(mbot::CheckCommand("arm 200").normalized, "arm 115");
    EXPECT_EQ(mbot::CheckCommand("grip close").normalized, "grip close");
    EXPECT_EQ(mbot::CheckCommand("grip 0").normalized, "grip 50");
    EXPECT_EQ(mbot::CheckCommand("grip 999").normalized, "grip 115");
    EXPECT_FALSE(mbot::CheckCommand("arm open").ok);
    EXPECT_FALSE(mbot::CheckCommand("grip up").ok);
    EXPECT_FALSE(mbot::CheckCommand("arm").ok);
}

TEST(MbotCommand, LedIsClampedAndIndexed) {
    EXPECT_EQ(mbot::CheckCommand("led 300 -1 12").normalized, "led 255 0 12 all");
    EXPECT_EQ(mbot::CheckCommand("led 1 2 3 5").normalized, "led 1 2 3 5");
    EXPECT_FALSE(mbot::CheckCommand("led 1 2 3 6").ok);
    EXPECT_FALSE(mbot::CheckCommand("led 1 2").ok);
}

TEST(MbotCommand, StopAndReadAlwaysPass) {
    auto r = mbot::CheckCommand("stop");
    ASSERT_TRUE(r.ok);
    EXPECT_TRUE(r.is_stop);
    EXPECT_TRUE(mbot::CheckCommand("read").ok);
}

TEST(MbotCommand, ProgramIsValidatedAsAWhole) {
    auto r = mbot::CheckCommand("prog fwd 40 1; led 0 255 0; wait 0.5; arm up");
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.normalized, "prog fwd 40 1;led 0 255 0 all;wait 0.5;arm up");
    EXPECT_EQ(r.steps, 4);
    EXPECT_TRUE(r.is_motion);
}

TEST(MbotCommand, ProgramStepLimit) {
    std::string p = "prog";
    for (int i = 0; i < MBOT_MAX_PROG_STEPS; ++i) p += " led 0 0 0;";
    EXPECT_TRUE(mbot::CheckCommand(p).ok);
    p += " led 0 0 0";
    auto r = mbot::CheckCommand(p);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.error.rfind("too_many_steps", 0), 0u);
}

TEST(MbotCommand, ProgramTimeLimit) {
    // 6 x 5 s = 30 s exactly: allowed.
    std::string ok = "prog";
    for (int i = 0; i < 6; ++i) ok += " fwd 40 5;";
    EXPECT_TRUE(mbot::CheckCommand(ok).ok);
    // Clamping cannot sneak past the limit: 7 x "fwd 40 60" is 7 x 5 s.
    std::string too_long = "prog";
    for (int i = 0; i < 7; ++i) too_long += " fwd 40 60;";
    auto r = mbot::CheckCommand(too_long);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.error.rfind("too_long", 0), 0u);
    // Servo glide time counts too.
    std::string servo = "prog";
    for (int i = 0; i < 6; ++i) servo += " fwd 40 5;";
    servo += " arm up";
    EXPECT_FALSE(mbot::CheckCommand(servo).ok);
}

TEST(MbotCommand, ProgramRejectsForbiddenSteps) {
    EXPECT_FALSE(mbot::CheckCommand("prog fwd 40 1; stop").ok);
    EXPECT_FALSE(mbot::CheckCommand("prog read").ok);
    EXPECT_FALSE(mbot::CheckCommand("prog prog fwd").ok);
    EXPECT_FALSE(mbot::CheckCommand("prog fwd 40 1; dance").ok);
    EXPECT_FALSE(mbot::CheckCommand("prog").ok);
    EXPECT_FALSE(mbot::CheckCommand("prog ;;;").ok);
}

TEST(MbotCommand, WaitInProgramIsClamped) {
    auto r = mbot::CheckCommand("prog wait 99");
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.normalized, "prog wait 5");
}

// ---------------------------------------------------------------- v1.1

TEST(MbotCommandV11, StraightIsClampedWithDefaultSpeed) {
    auto r = mbot::CheckCommand("straight 20");
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.normalized, "straight 20 30");
    EXPECT_TRUE(r.is_motion);

    r = mbot::CheckCommand("straight -75 250");
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.normalized, "straight -30 100");
    EXPECT_FALSE(r.clamped.empty());

    r = mbot::CheckCommand("straight 12.34 1");
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.normalized, "straight 12.3 5");

    EXPECT_FALSE(mbot::CheckCommand("straight").ok);
    EXPECT_FALSE(mbot::CheckCommand("straight far").ok);
    EXPECT_FALSE(mbot::CheckCommand("straight 10 30 1").ok);
}

TEST(MbotCommandV11, TurnIsClamped) {
    auto r = mbot::CheckCommand("turn 90");
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.normalized, "turn 90 30");
    EXPECT_EQ(mbot::CheckCommand("turn -400 50").normalized, "turn -180 50");
    EXPECT_FALSE(mbot::CheckCommand("turn inf").ok);
}

TEST(MbotCommandV11, ClosedLoopTimeMatchesRuntime) {
    // Runtime: v = 60 * pct/100 * 6 * (6.5*pi/360) cm/s; est*1.5+1, cap 5 s.
    const double v30 = 60 * 0.30 * 6 * (6.5 * 3.14159 / 360.0);  // ~6.13 cm/s
    EXPECT_NEAR(mbot::ClosedLoopSeconds("straight", 3, 30), 3 / v30 * 1.5 + 1.0, 1e-9);
    EXPECT_DOUBLE_EQ(mbot::ClosedLoopSeconds("straight", 30, 30), MBOT_MAX_MOVE_S);
    const double w = 2 * v30 / 11.5 * 57.2958;  // deg/s
    EXPECT_NEAR(mbot::ClosedLoopSeconds("turn", 45, 30), 45 / w * 1.5 + 1.0, 1e-9);
    EXPECT_DOUBLE_EQ(mbot::CheckCommand("straight 30 30").seconds, MBOT_MAX_MOVE_S);
}

TEST(MbotCommandV11, OdomAndSync) {
    EXPECT_EQ(mbot::CheckCommand("odom 0").normalized, "odom 0");
    EXPECT_EQ(mbot::CheckCommand("odom -5").normalized, "odom 0");
    EXPECT_EQ(mbot::CheckCommand("odom 20").normalized, "odom 100");
    EXPECT_EQ(mbot::CheckCommand("odom 99999").normalized, "odom 5000");
    EXPECT_FALSE(mbot::CheckCommand("odom").ok);
    EXPECT_FALSE(mbot::CheckCommand("odom 1 2").ok);
    EXPECT_FALSE(mbot::CheckCommand("odom 500").is_motion);

    EXPECT_EQ(mbot::CheckCommand("sync 123456").normalized, "sync 123456");
    EXPECT_FALSE(mbot::CheckCommand("sync").ok);
    EXPECT_FALSE(mbot::CheckCommand("sync -1").ok);
    EXPECT_FALSE(mbot::CheckCommand("sync 1.5").ok);
}

TEST(MbotCommandV11, ProgramsAcceptStraightAndTurnButNotOdomOrSync) {
    auto r = mbot::CheckCommand("prog straight 20; turn 90; straight -10 50");
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.normalized, "prog straight 20 30;turn 90 30;straight -10 50");
    EXPECT_FALSE(mbot::CheckCommand("prog odom 500").ok);
    EXPECT_FALSE(mbot::CheckCommand("prog sync 1").ok);
    // Seven full-length steps (5 s each) exceed 30 s.
    std::string p = "prog";
    for (int i = 0; i < 7; ++i) p += " straight 30;";
    auto t = mbot::CheckCommand(p);
    EXPECT_FALSE(t.ok);
    EXPECT_EQ(t.error.rfind("too_long", 0), 0u);
}

TEST(MbotEventsV11, ParseDoneWithReasonDistYaw) {
    mbot::DoneEvent d;
    ASSERT_TRUE(mbot::ParseDoneArgs("42 target dist=29.8 yaw=-0.4 t=123456", &d));
    EXPECT_EQ(d.id, 42);
    EXPECT_EQ(d.reason, "target");
    ASSERT_TRUE(d.dist_cm.has_value());
    EXPECT_DOUBLE_EQ(*d.dist_cm, 29.8);
    ASSERT_TRUE(d.yaw_deg.has_value());
    EXPECT_DOUBLE_EQ(*d.yaw_deg, -0.4);
    ASSERT_TRUE(d.mbot_ms.has_value());
    EXPECT_EQ(*d.mbot_ms, 123456);
}

TEST(MbotEventsV11, ParseDoneNoneAndProgAndV1) {
    mbot::DoneEvent d;
    ASSERT_TRUE(mbot::ParseDoneArgs("7 timeout dist=none yaw=none t=5", &d));
    EXPECT_EQ(d.reason, "timeout");
    EXPECT_FALSE(d.dist_cm.has_value());
    EXPECT_FALSE(d.yaw_deg.has_value());

    ASSERT_TRUE(mbot::ParseDoneArgs("9 prog t=777", &d));
    EXPECT_EQ(d.reason, "prog");
    EXPECT_EQ(*d.mbot_ms, 777);

    ASSERT_TRUE(mbot::ParseDoneArgs("12", &d));  // v1 runtime
    EXPECT_EQ(d.id, 12);
    EXPECT_EQ(d.reason, "");

    EXPECT_FALSE(mbot::ParseDoneArgs("", &d));
    EXPECT_FALSE(mbot::ParseDoneArgs("abc target", &d));
}

TEST(MbotEventsV11, ParseOdom) {
    mbot::OdomSample o;
    ASSERT_TRUE(mbot::ParseOdomArgs("t=1000 l=12.5 r=12.1 yaw=3.0", &o));
    EXPECT_EQ(*o.mbot_ms, 1000);
    EXPECT_DOUBLE_EQ(*o.left_cm, 12.5);
    EXPECT_DOUBLE_EQ(*o.right_cm, 12.1);
    EXPECT_DOUBLE_EQ(*o.yaw_deg, 3.0);
    ASSERT_TRUE(mbot::ParseOdomArgs("t=5 l=none r=none yaw=none", &o));
    EXPECT_FALSE(o.left_cm.has_value());
    EXPECT_FALSE(o.yaw_deg.has_value());
}


// ---- v1.2 drive validation ----
TEST(MbotCommand, DriveNormalizedAndClamped) {
    auto r = mbot::CheckCommand("drive 10 45 300");
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.normalized, "drive 10 45 300");
    EXPECT_TRUE(r.is_motion);
    EXPECT_TRUE(r.clamped.empty());

    r = mbot::CheckCommand("drive 999 -999 99999");
    ASSERT_TRUE(r.ok) << r.error;
    // 60 rpm * 6 deg/s * 6.5*pi/360 cm/deg = 20.4 cm/s
    EXPECT_EQ(r.normalized, "drive 20.4 -" + std::to_string(MBOT_MAX_DRIVE_DEG_S) + " " +
                                std::to_string(MBOT_MAX_DRIVE_LEASE_MS));
    EXPECT_FALSE(r.clamped.empty());
    EXPECT_DOUBLE_EQ(r.seconds, MBOT_MAX_DRIVE_LEASE_MS / 1000.0);

    EXPECT_EQ(mbot::CheckCommand("drive 5 0").normalized, "drive 5 0 " + std::to_string(MBOT_MAX_DRIVE_LEASE_MS));
}

TEST(MbotCommand, DriveRejectsBadInputAndPrograms) {
    EXPECT_FALSE(mbot::CheckCommand("drive").ok);
    EXPECT_FALSE(mbot::CheckCommand("drive 10").ok);
    EXPECT_FALSE(mbot::CheckCommand("drive fast 0").ok);
    EXPECT_FALSE(mbot::CheckCommand("drive 10 0 300 9").ok);
    auto r = mbot::CheckCommand("prog led 0 0 9; drive 10 0 300");
    EXPECT_FALSE(r.ok);
    EXPECT_NE(r.error.find("drive"), std::string::npos);
}

// ---- teleop packets ----
// Golden vector, shared with docs/teleop.md, scripts/teleop_udp.py and the JoyC firmware:
// seq 7, key 0x12345678, 150 mm/s, -500 mrad/s, buttons enable|arm_up, lease 300 ms.
const char* kGoldenCmdVel = "5354010107000000785634129600" "0cfe05002c01";

TEST(Teleop, CmdVelMatchesGoldenVector) {
    teleop::CmdVel v;
    v.linear_x_mm_s = 150;
    v.angular_z_mrad_s = -500;
    v.buttons = teleop::kEnable | teleop::kArmUp;
    v.lease_ms = 300;
    uint8_t buf[32];
    size_t n = teleop::EncodeCmdVel(7, 0x12345678, v, buf, sizeof(buf));
    ASSERT_EQ(n, teleop::kCmdVelSize);
    EXPECT_EQ(std::vector<uint8_t>(buf, buf + n), FromHex(kGoldenCmdVel));

    teleop::Header h;
    teleop::CmdVel back;
    ASSERT_TRUE(teleop::ParseCmdVel(buf, n, &h, &back));
    EXPECT_EQ(h.seq, 7u);
    EXPECT_EQ(h.key, 0x12345678u);
    EXPECT_EQ(back.linear_x_mm_s, 150);
    EXPECT_EQ(back.angular_z_mrad_s, -500);
    EXPECT_EQ(back.buttons, teleop::kEnable | teleop::kArmUp);
    EXPECT_EQ(back.lease_ms, 300);
    EXPECT_DOUBLE_EQ(teleop::LinearCmPerS(back), 15.0);
    EXPECT_NEAR(teleop::AngularDegPerS(back), -28.648, 0.001);
}

TEST(Teleop, StatusRoundTripAndRejects) {
    teleop::Status s;
    s.flags = teleop::kMbotConnected | teleop::kTeleopActive;
    s.battery_pct = 80;
    s.dist_cm = 123;
    s.arm_deg = 100;
    s.grip_deg = 45;
    s.rtt_ms = 60;
    uint8_t buf[32];
    size_t n = teleop::EncodeStatus(teleop::kStatus, 9, 1, s, buf, sizeof(buf));
    ASSERT_EQ(n, teleop::kStatusSize);
    teleop::Header h;
    teleop::Status b;
    ASSERT_TRUE(teleop::ParseStatus(buf, n, &h, &b));
    EXPECT_EQ(b.flags, s.flags);
    EXPECT_EQ(b.battery_pct, 80);
    EXPECT_EQ(b.dist_cm, 123);
    EXPECT_EQ(b.arm_deg, 100);
    EXPECT_EQ(b.grip_deg, 45);
    EXPECT_EQ(b.rtt_ms, 60);

    teleop::CmdVel v;
    EXPECT_FALSE(teleop::ParseCmdVel(buf, n, &h, &v));          // wrong type
    EXPECT_FALSE(teleop::ParseStatus(buf, n - 1, &h, &b));      // short
    buf[0] = 'X';
    EXPECT_FALSE(teleop::ParseHeader(buf, n, &h));              // bad magic
    EXPECT_EQ(teleop::EncodeCmdVel(1, 1, v, buf, 19), 0u);      // too small
}

TEST(Teleop, SequenceSurvivesWrap) {
    EXPECT_TRUE(teleop::SeqNewer(2, 1));
    EXPECT_FALSE(teleop::SeqNewer(1, 1));
    EXPECT_FALSE(teleop::SeqNewer(1, 2));
    EXPECT_TRUE(teleop::SeqNewer(3, 0xFFFFFFFEu));
}
