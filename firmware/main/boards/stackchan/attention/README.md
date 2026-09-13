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
| Board integration adapter | written, **not compiled** — see Assumptions |
| Face detection | **not implemented** — see Assumptions |

Nothing here has run on hardware.

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
    head_controller.{h,cc}      wires it together and schedules it

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
        display->SetEmotion(e);            // "neutral", "happy", "surprised"
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
    cmake -S host_test -B build/host_test
    cmake --build build/host_test --target attention_safety_test
    ./build/host_test/attention_safety_test

22 tests. The ones that matter:

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
6. **Not compiled against ESP-IDF.** No `idf.py` and no `IDF_PATH` on the
   machine this was written on, so the adapter and the `ESP_LOGx` path are
   unverified. The controllers and their tests are verified, on the host.
7. **Face detection is not implemented.** Two things block it, and both are
   design decisions rather than work items:
   * `Camera` (`boards/common/camera.h`) exposes `Capture()` and
     `Explain(question)` and deliberately hands pixels to nobody —
     `Esp32Camera::current_fb_` is private. Local inference needs a frame
     accessor added there.
   * esp-dl / `human_face_detect` is not a dependency of this firmware
     (`main/idf_component.yml`), and adding it has a PSRAM cost that has to
     be weighed against the audio buffers already resident.

   `VisionTracker` is therefore an interface, with `ScriptedVisionTracker`
   standing in. Everything downstream is finished and tested; swapping in a
   real detector is one class.
8. **"Up" is the positive pitch direction.** `AttentionController` documents
   image y as growing downward while pitch grows upward, and defaults
   `invert_pitch` to true on that basis. The greeting script is written in
   gaze terms — "look up", "nod down" — and that assumption is applied in one
   place, `kPitchUpIsPositive` in `greeting_routine.h`. If the head nods when
   it should raise its gaze, that constant is the fix, not the eight rows of
   the table. Unverified on hardware.

9. **Servo presence is unverified.** Whether the unit this is destined for
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
