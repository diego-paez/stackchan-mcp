# attention/ — local face tracking and head behaviour

A first prototype of autonomous head control: acquire a face, orient toward
it slowly, tolerate losing it, and never issue a servo command that has not
passed the safety filter.

Priority order, as specified: **safety > stability > smoothness > tracking speed.**

## Status

| Part | State |
|---|---|
| Control chain (attention → motion → mixer → safety → sink) | written, 22 host tests passing |
| Servo safety filter incl. hostile-input rejection | written, tested |
| Behaviour manager (IDLE / ATTEND_FACE / LOOK_CENTER / THINK / SLEEP) | written, tested |
| Servo self-test | written, tested |
| Board integration adapter | wired into `StackChanBoard`, **compiles** (ESP-IDF v5.5.2) |
| Face detection | compiles, **off by default** — `CONFIG_STACKCHAN_FACE_DETECT`, see Size |
| Face mimic (what the screen does about the face) | written, 27 host tests passing |

Nothing here has run on hardware. It does now build: `python scripts/release.py
stackchan` under `espressif/idf:v5.5.2` produces a 2.92 MB app with 26% of the
OTA partition free.

## Arming

Autonomous control is built at boot and **not started**. The head already has
three writers — the `move_head` MCP tool, the touch wobble, and the boot
sequence — and a fourth that starts unbidden on every power-up is what the
one-door rule below forbids. Nothing moves until:

    self.robot.set_attention(enabled=true)

which runs the self-test, then the greeting, then enables tracking.
`self.robot.get_attention` reports the counters; `self.robot.observe_affect`
feeds the mimic. `set_head_angles` **disarms** — a remote command wins over
autonomy, because otherwise the tracker steers back on its next tick and the
caller's move looks ignored.

## Size

Face detection costs **1.76 MB** of app image: 2.92 MB without it, 4.68 MB
with, against a 4032K OTA partition. That is why `CONFIG_STACKCHAN_FACE_DETECT`
defaults to `n`. Turning it on needs one of:

* `assets` shrunk from 8 M to ~6 M and both OTA slots grown to ~4992K;
* a single OTA slot, which costs A/B updates;
* the model weights moved into a partition of their own, which is what
  esp-dl's loader is designed for and the only option that costs nothing else.

Everything else — head control, the self-test, the greeting, the face mimic —
fits comfortably and works without a detector. The head simply has nothing to
follow, which the attention system already treats as an empty room.

## Data flow

    VisionTracker ──► AttentionController ──┐
                                            ├─► MotionMixer ─► MotionController
    BehaviorManager ────────────────────────┘                        │
                                                     ServoSafetyController
                                                                     │
                                                          ServoSink ─► hardware

There is no other path to the hardware. `ServoSink` is an interface with
exactly one real implementation (`BoardServoSink`), which forwards to the
board's existing `WriteHeadAngles()`.

## Files

    attention_types.h           FaceTarget, HeadPose, ServoCommand, Behavior
    servo_limits.h              every number that can damage the hardware
    vision_tracker.h            detector interface + a scripted fake for tests
    attention_controller.{h,cc} face position → head target; dead zone, filter, loss policy
    behavior_manager.{h,cc}     what the robot is trying to do
    motion_mixer.{h,cc}         chooses the single pose to pursue this tick
    motion_controller.{h,cc}    bounded approach toward that pose
    servo_safety_controller.{h,cc}  clamp, rate limit, reject, watchdog, e-stop
    servo_sink.h                the one door to hardware (+ a recording fake)
    board_servo_sink.h          adapter onto StackChanBoard::WriteHeadAngles
    greeting_routine.{h,cc}     the scripted introduction, played once on waking
    face_geometry.h             detector box -> FaceTarget; pure, host-tested
    face_mimic.{h,cc}           observed affect -> avatar face; policy, hysteresis
    espdl_face_tracker.{h,cc}   the real VisionTracker, backed by esp-dl
    head_controller.{h,cc}      wires it together and schedules it

## The face

The head follows a person; `face_mimic` decides what the screen does about
them. It is a separate axis from everything above — an expression is never a
servo command, and `HeadControllerMimic.TheFaceDoesNotDisturbTheHead` asserts
exactly that.

**The two vocabularies do not match and cannot.** The pipeline reports seven
affects (`chatbot/models/emotion/base.py`); the avatar has six faces
(`stackchan.cc`, `FaceNameToIndex`). So there is a table:

| she is | the robot shows | |
|---|---|---|
| happy | happy | mirror |
| surprise | surprised | mirror — shared surprise is joint attention |
| sad | sad | mirror — a sad child met with a cheerful face reads as not having been heard |
| anger | thinking | **answer.** Attentive, rather than matching the escalation |
| fear | sad | **answer.** Concern, not alarm |
| disgust | embarrassed | answer — the nearest thing the avatar has |
| neutral | idle | rest |

Mirroring indiscriminately is not empathy, it is a loop: anger reflected at an
angry child escalates, fear reflected confirms there is something to fear.
Those seven rows are a pedagogical judgement and **no host test can tell you
they are right.** The tests prove the table is applied consistently, that the
face cannot flicker, and that it cannot get stuck. Run `sim_mimic` to judge
the rest — it prints a short conversation and what the face did about it.

### Not moving, again

The same priority order applies. A reading confirms by any of three routes,
because this layer cannot know how fast its source is — a per-frame emotion
stream arrives at 5 Hz, the transcribe endpoint once per utterance:

* **by count** — `confirmations` agreeing readings in a row (400 ms at 5 Hz;
  a whole conversational turn at one per utterance, which is why it is not
  the only route)
* **by confidence** — `instant_confidence`, set at 0.65 from what the models
  actually return for a clear utterance, not from a round number
* **by dwell** — `confirm_dwell_ms` with nothing contradicting it, which is
  what lets one utterance land without lowering the confidence bar to where
  noise passes. A contradiction resets the clock, so alternating readings
  confirm by no route at all.

Then `min_hold_ms` is a floor between changes regardless, and `stale_ms`
returns the face to idle when nobody has said anything — a robot still
wearing the last thing it was told is stuck, not expressive.

A failed emotion model returns a uniform distribution, 1/7 = 0.143 on every
label. `min_confidence` is what makes that move nothing at all.

### Who owns the screen

One sink, for the same reason `MotionMixer` exists. Two states stop the mimic
and they are **not** the same thing:

* **suppressed** — the greeting owns the face while it plays, the mouth owns
  it while the robot speaks. The policy keeps running so the answer is
  current the moment it is free.
* **halted** — the robot has been emergency-stopped. Observations are dropped
  rather than accumulated and the face goes to idle. Conflating this with
  suppression was a bug: provoking a stopped robot left it wearing an
  expression the instant it was released.

### `"neutral"` was never a face

The greeting script opened and closed on the expression `"neutral"`, which
`FaceNameToIndex()` has never known — the avatar's resting face is `"idle"`.
On hardware this fails silently: the expression is logged as deferred and the
previous face stays up. Fixed, and `FaceFromName()` now publishes the six
renderable names so this is checkable;
`HeadControllerMimic.EveryExpressionTheRobotShowsIsOneTheAvatarCanRender`
walks a whole boot and checks every string that leaves for the display.

## Startup sequence

The order is fixed, and the reason is safety rather than presentation:

    begin()
      │
      ├─ read the real pose and sync to it        no "boot snap"
      ├─ move to neutral, slowly
      ├─ self-test    ±10° yaw, ±5° pitch, 1.2 s per beat, ~10.8 s
      │                tracking disabled throughout
      │
      ├─ greeting     8 beats, ~5 s               ← only if the self-test passed
      │                "Greetings, I am Stacky"
      │                tracking still disabled
      │
      └─ ATTEND_FACE  tracking enabled

Expressive movement does not happen until the safety layer has been shown to
work on this particular unit. If the self-test fails, the greeting never runs
and tracking never starts.

`GreetingRoutine` is a table of keyframes — yaw offset, gaze offset,
expression, optional line, dwell — stepped by the same scheduler as everything
else. It writes no angle: it offers a pose, exactly as a behaviour does, and
that pose goes through the motion controller and the safety filter like any
other. `Behavior::GREET` therefore sets `has_fixed_pose = false`, so there is
still only one opinion about where the head should be.

Amplitudes stay well inside the envelope — yaw within ±14° of ±30°, gaze
within ±8° — so the safety layer is never the thing deciding where the head
stops during a greeting.

Expression and speech leave through `std::function` sinks:

    head.setGreetingExpressionSink([display](const char* e) {
        display->SetEmotion(e);            // "idle", "happy", "surprised"
    });
    head.setGreetingSpeechSink([](const char* line) {
        // whatever this firmware's speech path turns out to be
    });

Both are optional. Unset, the greeting still moves — which is what the host
tests run, and what a unit with no speaker does. `setGreetingEnabled(false)`
skips it entirely and hands straight from the self-test to tracking.

An emergency stop cancels a greeting in progress and fires no further sinks:
a stopped robot must not keep performing, and must not finish its sentence.

## Building and testing

The controllers are free of ESP-IDF so they can be tested where a mistake
costs nothing:

    cd firmware
    cmake -S host_test -B build_host/host_test
    cmake --build build_host/host_test
    (cd build_host/host_test && ctest)

`build_host/`, not `build/`: ESP-IDF owns `firmware/build/` and `release.py`
refuses to run when something else has been there first.

40 ctest entries. `attention_safety_test` is the servo chain; `face_mimic_test`
is the screen. The ones that matter:

* `RejectsAbsurdYaw` — ±1000° is **refused**, not clamped to the edge of travel
* `RejectsNaNAndInfinity` — NaN/±Inf in pitch, and a NaN `dt`
* `LimitsLargeInstantaneousJump` — a full-envelope jump becomes one bounded step
* `AbsurdDtCannotLicenseAHugeStep` — a stalled scheduler reporting 30 s
* `NeverLeavesTheHardwarePitchRangeEvenIfMisconfigured` — a widened config is
  still floored by the hardware bounds
* `EmergencyStopHoldsAndRefuses`, `WatchdogHoldsWhenCommandsStop`
* `NothingReachesTheServoOutsideTheEnvelope` — 20 s of a face pinned to a
  corner, asserting every single command
* `ConsecutiveCommandsNeverJump` — a face teleporting between corners twice a
  second, asserting the step limit on every write
* `NoiseDoesNotProduceContinuousOscillation` — jitter inside the dead zone
  produces no motion

And for the face:

* `AFailedEmotionModelCannotMoveTheFace` — 200 readings at 1/7 confidence
* `NaNConfidenceIsRejectedRatherThanRankedHigh`
* `AlternatingStrongReadingsCannotMakeItFlicker` — 12 s of a model swinging
  between two confident answers ten times a second
* `ContradictedReadingsNeverConfirmByDwell` — dwell is not a way in for noise
* `StalenessSurvivesTheMillisecondWrap` — 49 days, and a face frozen forever
* `EmergencyStopReturnsTheFaceToIdleAndKeepsItThere`
* `TheFaceDoesNotDisturbTheHead` — an expression is never a servo command

Rejection is deliberate where clamping would be wrong. NaN does not mean
"somewhere near the middle"; it means the caller is broken, and turning it
into a plausible angle hides that.

## Integration point

In `StackChanBoard`, after the servo is initialised and the boot move has
finished:

```cpp
#include "attention/head_controller.h"
#include "attention/board_servo_sink.h"

sink_ = std::make_unique<attention::BoardServoSink>(
    [this](int yaw, int pitch, uint32_t ms) {
        WriteHeadAngles(yaw, pitch, ms, /*prefer_linear=*/true);
    },
    [this]{ attention::HeadPose p;
            p.yaw_deg   = static_cast<float>(yaw_motion_.current_deg);
            p.pitch_deg = static_cast<float>(pitch_motion_.current_deg);
            return p; },
    [this]{ return servo_ok_; });

head_ = std::make_unique<attention::HeadController>(
    *vision_, *sink_, attention::ServoLimits{}, attention::NeutralPose{},
    attention::HardwareBounds{}, attention::AttentionConfig{},
    attention::MotionConfig{}, attention::ScheduleConfig{});
head_->begin(now_ms);
```

then call `head_->update(now_ms)` from the existing servo task tick. It is
non-blocking and rate-limits each stage internally; it never calls `delay()`.

`begin()` seeds the motion controller from the **actual** pose before
commanding anything. Skipping that is what produces the boot snap the board's
own notes describe.

### External API

```cpp
head_->setBehavior(Behavior::ATTEND_FACE, now_ms);
head_->setTrackingEnabled(false);
head_->emergencyStop();
```

No method takes a raw angle. `debugRequestPose()` exists only under
`-DSTACKCHAN_ATTENTION_DEBUG_MANUAL` and still goes through the same filter.

## Configuration

Everything tunable is a struct field, not a literal. Defaults:

| | value | why |
|---|---|---|
| yaw travel | −30°…+30° | far inside anything mechanical |
| pitch travel | 25°…65° | neutral 45 ± 20, inside M5Stack's 5–85 |
| max velocity | 60 °/s | |
| max step | 3° per update | bounds a scheduler stall |
| speed | 80 °/s, capped 120 | `MAX_SPEED_DPS` is 240; no reason to approach it |
| dead zone | 8% x, 10% y | below ~4%, detector noise alone moves the head |
| loss: hold / relax / search | 500 / 1500 ms / off | search off for the first test |
| rates | servo 40 Hz, attention 25 Hz, behaviour 20 Hz, vision 5 Hz | |

## Assumptions

Stated explicitly, because several are unverified.

1. **Hardware is CoreS3 + two SCS0009 serial-bus servos**, per
   `boards/stackchan/config.h` (UART1, 1 Mbaud, TX 6 / RX 7, yaw id 1,
   pitch id 2). These are *not* PWM servos; nothing here generates a pulse
   width, and "raw PWM" has no meaning on this hardware.
2. **Angles are absolute, in the board's own frame.** Yaw 0 is ahead, pitch
   45 is level (`BOOT_INIT_PITCH_DEG`). This layer never converts to or from
   offsets — doing that in two places is how sign errors reach a servo.
3. **The board's limits are authoritative.** `SAFE_PITCH_MIN/MAX` = 0/88 and
   `RECOMMENDED` = 5/85 are mirrored into `HardwareBounds`. If they ever
   disagree, the board is right and `servo_limits.h` is wrong.
4. **Yaw has no hard clamp in the board.** `PitchDegToPos()` clamps pitch at
   the servo-write boundary; `YawDegToPos()` only saturates the raw position.
   Face tracking drives yaw hardest, so the yaw limit here is a real gap
   being closed, not a duplicate.
5. **`WriteHeadAngles(int, int, uint32_t, bool)` is the only sanctioned
   entry point**, and it already owns the motion mutex, torque state and boot
   sequence. The adapter supplies a duration derived from distance and the
   approved speed; it never re-plans the motion.
6. ~~**Not compiled against ESP-IDF.**~~ **Settled.** Built with
   `espressif/idf:v5.5.2` in Docker. Three things only a real compiler found:
   `attention::` does not resolve from global scope (`StackChanBoard` is not in
   namespace `stackchan`), `-Werror=reorder` on `HeadController`'s init list,
   and the RGB565 endianness split above. The host build enables none of these
   warnings, which is worth remembering before trusting it alone.
7. **Face detection compiles, and the version pairing is load-bearing.**
   `human_face_detect` 0.2.3 asks for esp-dl `^3.1.3`; the resolver honours
   that with 3.3.0, and the component then does not compile against it —
   `DL_IMAGE_CAP_RGB_SWAP` gone, `MSRPostprocessor`'s constructor arity
   changed, `MNPPostprocessor::set_resize_scale_x` gone, `MSRMNP` left
   abstract. esp-dl made breaking changes inside its own caret range. The
   manifest now pins `^0.5.0`, whose versions ask for `~3.3.0`. Loosening it
   reintroduces the breakage.

   Historical note: `espdl_face_tracker`
   targeted `espressif/human_face_detect` ^0.2.0, which pulls
   `espressif/esp-dl` ^3.0.0 and is declared in `main/idf_component.yml` for
   esp32s3 and esp32p4. The API was read from the published component rather
   than guessed: `HumanFaceDetect::run(const dl::image::img_t&)` returning
   `std::list<dl::detect::result_t>&`, boxes as `[left, top, right, bottom]`.
   None of it has been through a compiler — there is no ESP-IDF on the machine
   where it was written. Treat the adapter as a first draft; the geometry it
   depends on is tested and is the part that fails silently.

   Two facts from the component's own documentation shape the design. On the
   S3 the first stage costs ~41 ms and each candidate ~6 ms, so one face is
   ~48 ms — hence a dedicated task at 5 Hz rather than a call from the head
   loop, which runs at 40 Hz. And the models are published only for esp32s3
   and esp32p4, which is why the dependency is gated.


8. ~~**The camera's pixel format is not known in advance.**~~ **Settled.**
   The board pins the sensor to YUV422
   (`CONFIG_CAMERA_GC0308_DVP_YUV422_320X240_20FPS` in `config.json`), so it
   was not unknown — it was configured to the one family esp-dl 3.1 could not
   read, and face tracking would have reported an empty room forever.

   No sensor change was needed in the end. esp-dl 3.3 gained a real
   `DL_IMAGE_PIX_TYPE_YUYV` with genuine conversions behind it
   (`DL_IMAGE_PIX_CVT_YUYV2RGB888`), and `esp_video.cc:194` records that what
   this stack labels YUV422P is byte-wise packed YUYV. So `MapPixelFormat()`
   now declares the true format rather than reinterpreting bytes — the
   opposite of the mistake the file warns about, because the preprocessor
   converts properly instead of the model reading luma as red. The photo path
   keeps the format it was tuned for.

   One trap remains, recorded because it is invisible: esp-dl 3.3 split RGB565
   by endianness. `V4L2_PIX_FMT_RGB565` is `RGB565LE`. Choosing `BE` would not
   fail — it would swap red and blue on every frame and quietly cost accuracy.

9. **"Up" is the positive pitch direction.** `AttentionController` documents
   image y as growing downward while pitch grows upward, and defaults
   `invert_pitch` to true on that basis. The greeting script is written in
   gaze terms — "look up", "nod down" — and that assumption is applied in one
   place, `kPitchUpIsPositive` in `greeting_routine.h`. If the head nods when
   it should raise its gaze, that constant is the fix, not the eight rows of
   the table. Unverified on hardware.

10. **Servo presence is unverified.** Whether the unit this is destined for
   has the servo base attached at all has not been confirmed. If it does not,
   the self-test is the safe way to find out: it moves ±10° yaw and ±5° pitch
   slowly and reports.

## Before the first physical test

Run the host tests. Then flash with tracking disabled and watch the
self-test: it starts at neutral, requests +10° yaw, returns, −10°, returns,
+5° pitch, returns, −5°, returns — each step over 1.2 s. Tracking does not
enable until it completes.

Watch the diagnostics line. `rej=` and `clamp=` are the proof the safety
layer is intercepting rather than passing everything through; if they stay
zero while the head is tracking, that is expected — the motion controller is
supposed to stay inside the envelope so the filter has nothing to do. Force a
non-zero count with the host tests, not with the hardware.
