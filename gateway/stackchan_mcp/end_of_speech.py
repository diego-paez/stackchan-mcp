"""Decide when the person stopped talking, from the device's own audio.

The xiaozhi protocol has three listening modes, and the wake-word path
enters ``kListeningModeAutoStop`` — which, read literally, says *the
server closes the window*. stackchan-mcp never did. So a device-driven
capture (wake word, button, LCD touch) ran until something else
happened to end it: the user pressing the head again, a disconnection,
a session timeout. In practice that meant tens of seconds of room tone
glued to the end of a two-second question, and a caller waiting on a
reply that could not start until the window closed.

This module closes it. :class:`EndOfSpeech` is fed one decoded frame at
a time and answers a single question — *is this window finished?* —
with the reason, so the caller can log why.

Why here and not in the firmware: the AFE on the device does run a VAD,
but the device is in the mode that expects the server to decide, the
threshold wants tuning against real rooms without a reflash, and every
frame is already crossing the gateway to be packed into Ogg. The
decision is made where the evidence already is.

Design notes:

**The floor moves.** A fixed dBFS threshold works in the room it was
measured in. The Stack-chan's own capture sits around 0.001 RMS of room
tone with speech peaks near 0.03 — a factor of thirty, but both ends
slide with the room, the gain and the distance to the microphone. So
the noise floor is tracked continuously and the speech threshold is a
multiple of it, with an absolute minimum so a silent room cannot drive
the threshold to zero.

**The floor is estimated from silence only.** The obvious smoothing —
move the floor toward every frame, slowly up and quickly down — has a
failure that shows up on sustained sound: hold a level for long enough
and the floor climbs until the threshold crosses *above* the very
sound holding it up, and speech starts reading as silence. A minute of
someone talking is enough. So frames judged to be speech do not update
the floor at all; it is built from the gaps, which is the only part of
the signal that is actually evidence about the room.

**The floor starts where the room is.** Estimating from gaps alone
cannot get started if the room is loud enough that nothing reads as a
gap: every frame of that hum would read as speech, forever. So the
floor is seeded from the opening half-second rather than assuming a
quiet room — from the *quietest* frame in it, not the first, because a
capture can open mid-syllable and one loud frame would seed the floor
above everything that follows and latch exactly the same way.

**One loud frame is not speech.** A chair creak clears any sensible
threshold. Onset requires consecutive frames, which costs a frame of
latency and rejects every transient shorter than that.

**The pause at the start of a window is not the pause inside a
sentence.** This is the one a single threshold gets wrong, and the
recordings show it plainly. A window opens the instant the wake word
fires, and people do not start talking then — they check first. The
capture this was tuned against opens with "Can you hear me?", then
1.38 s of nothing while the speaker waits for an answer, and only then
the real question. The gaps before that question began ran 1.08 s and
1.38 s; the pauses *inside* it topped out at 0.90 s. One threshold
cannot sit above the second and below the first.

So there are two, chosen by *when the silence started* rather than by
how much has been said: a gap that opens inside the first few seconds
gets the generous tolerance, and everything after gets the tight one.
Latching on the start of the gap matters — deciding afresh each frame
would let a lead-in pause flip to the tight rule halfway through and
close the window on a speaker who had not started yet. The generous
phase costs nothing, because by construction it only applies before the
sentence is under way.

**Three ways to end, not one.** Trailing silence is the ordinary one.
A window where nobody ever spoke ends on the lead-in cap. And a window
that never goes quiet ends on the hard cap — which exists because a
transcriber has its own limit (Whisper's is thirty seconds) and a
capture that sails past it is not slow, it is lost.
"""

from __future__ import annotations

import array
import logging
import math
import os
from typing import Optional

__all__ = [
    "EndOfSpeech",
    "OpusEndOfSpeech",
    "frame_rms",
    "settings_from_env",
]

logger = logging.getLogger(__name__)


#: Trailing silence that ends a window, once the sentence is under way.
#: The longest pause measured *inside* a real utterance on this hardware
#: was 0.90 s, so this clears it — and no more, because every
#: millisecond here is a millisecond the speaker spends waiting.
DEFAULT_SILENCE_MS = 1000

#: Trailing silence tolerated for a gap that opens inside the lead
#: window — the wake word, then the speaker deciding to begin. See the
#: module docstring; on real captures this gap reaches 1.38 s.
DEFAULT_LEAD_SILENCE_MS = 2000

#: How long the generous tolerance applies for. A gap that opens after
#: this point in the capture is judged as a pause inside a sentence.
DEFAULT_LEAD_WINDOW_MS = 4000

#: A window where nobody ever spoke ends here. Covers a wake word that
#: fired on the radio, and stops the slot being held open for nothing.
DEFAULT_LEAD_IN_MS = 6000

#: Hard cap, whatever the audio is doing. Under Whisper's 30 s limit
#: with room for the wake-word preroll the firmware prepends.
DEFAULT_MAX_MS = 25000

#: Speech is this many times the tracked noise floor.
DEFAULT_FLOOR_RATIO = 3.5

#: The floor never drops below this, so a truly silent room does not
#: turn the threshold into a hair trigger.
DEFAULT_FLOOR_MIN = 0.0008

#: Consecutive frames above the threshold before speech is believed.
DEFAULT_ONSET_FRAMES = 2

#: Proportion of the gap a quieter frame closes — about a second to
#: converge at 60 ms frames.
_FLOOR_FALL = 0.10

#: How fast a non-speech frame above the floor may raise it. Only
#: frames judged *not* to be speech move the floor at all, so this
#: tracks a room getting noisier without letting an utterance lift the
#: threshold over its own head.
_FLOOR_CREEP = 0.005

#: Frames used to seed the floor. Eight 60 ms frames is half a second:
#: long enough to catch a gap between syllables, short enough that the
#: detector is judging almost from the start.
_SEED_FRAMES = 8


def frame_rms(pcm: bytes) -> float:
    """Root-mean-square of one frame of signed 16-bit mono PCM, 0.0-1.0.

    Returns 0.0 for an empty or odd-length buffer rather than raising:
    a malformed frame is a reason to hear silence, not to tear down the
    capture.
    """
    if len(pcm) < 2:
        return 0.0
    samples = array.array("h")
    samples.frombytes(pcm[: len(pcm) - (len(pcm) % 2)])
    if not samples:
        return 0.0
    total = 0.0
    for s in samples:
        total += float(s) * float(s)
    return math.sqrt(total / len(samples)) / 32768.0


class EndOfSpeech:
    """Streaming end-of-utterance detector over fixed-size PCM frames.

    Feed every frame of a capture in order; the first call that returns
    a reason means the window is finished. The instance latches at that
    point and keeps returning the same reason, so a caller that is
    slightly late closing the window cannot fire twice.
    """

    def __init__(
        self,
        *,
        frame_ms: int = 60,
        silence_ms: int = DEFAULT_SILENCE_MS,
        lead_silence_ms: int = DEFAULT_LEAD_SILENCE_MS,
        lead_window_ms: int = DEFAULT_LEAD_WINDOW_MS,
        lead_in_ms: int = DEFAULT_LEAD_IN_MS,
        max_ms: int = DEFAULT_MAX_MS,
        floor_ratio: float = DEFAULT_FLOOR_RATIO,
        floor_min: float = DEFAULT_FLOOR_MIN,
        onset_frames: int = DEFAULT_ONSET_FRAMES,
    ) -> None:
        if frame_ms <= 0:
            raise ValueError("frame_ms must be positive")
        self.frame_ms = frame_ms
        self.silence_ms = max(0, silence_ms)
        self.lead_silence_ms = max(self.silence_ms, lead_silence_ms)
        self.lead_window_ms = max(0, lead_window_ms)
        self.lead_in_ms = max(0, lead_in_ms)
        self.max_ms = max(0, max_ms)
        self.floor_ratio = max(1.0, floor_ratio)
        self.floor_min = max(0.0, floor_min)
        self.onset_frames = max(1, onset_frames)

        self._floor = self.floor_min
        self._seed: list[float] = []
        self._above = 0
        self._speech_ms = 0
        self._spoken = False
        self._elapsed_ms = 0
        self._silence_ms = 0
        self._verdict: Optional[str] = None

    # -- state a caller may want to log ---------------------------------

    @property
    def elapsed_ms(self) -> int:
        """Audio fed so far, in milliseconds."""
        return self._elapsed_ms

    @property
    def heard_speech(self) -> bool:
        """Whether anything crossed the speech threshold in this window."""
        return self._spoken

    @property
    def speech_ms(self) -> int:
        """Speech accumulated so far, in milliseconds."""
        return self._speech_ms

    @property
    def silence_started_ms(self) -> int:
        """Where in the capture the current run of silence began."""
        return self._elapsed_ms - self._silence_ms

    @property
    def noise_floor(self) -> float:
        """The tracked floor, for tuning against a real room."""
        return self._floor

    @property
    def seeded(self) -> bool:
        """Whether enough audio has arrived to have a real floor."""
        return len(self._seed) >= _SEED_FRAMES

    @property
    def verdict(self) -> Optional[str]:
        """The latched reason, or ``None`` while the window is open."""
        return self._verdict

    # -- the decision ---------------------------------------------------

    def feed(self, pcm: bytes) -> Optional[str]:
        """Feed one frame of signed 16-bit mono PCM.

        Returns ``None`` while the window should stay open, or one of
        ``"silence"``, ``"lead_in"`` or ``"max"`` once it should close.
        """
        return self.feed_rms(frame_rms(pcm))

    def feed_rms(self, rms: float) -> Optional[str]:
        """Feed one frame's RMS directly.

        Separated from :meth:`feed` so the decision can be tested, and
        tuned against a recording, without a decoder in the loop.
        """
        if self._verdict is not None:
            return self._verdict

        self._elapsed_ms += self.frame_ms

        # Seed from the room rather than assuming it is quiet; see the
        # module docstring. Nothing is judged until the floor is real.
        if len(self._seed) < _SEED_FRAMES:
            self._seed.append(rms)
            if len(self._seed) == _SEED_FRAMES:
                self._floor = max(min(self._seed), self.floor_min)
            return None

        voiced = rms > self._floor * self.floor_ratio

        # Only silence is evidence about the room, so only silence
        # moves the floor.
        if not voiced:
            if rms < self._floor:
                self._floor += (rms - self._floor) * _FLOOR_FALL
            else:
                self._floor *= 1.0 + _FLOOR_CREEP
            if self._floor < self.floor_min:
                self._floor = self.floor_min

        if voiced:
            self._above += 1
            if self._above == self.onset_frames:
                # Credit the whole run at once: the frames that built
                # the onset were speech too, we just could not say so
                # until the run was long enough to rule out a transient.
                self._speech_ms += self.frame_ms * self.onset_frames
                self._spoken = True
                self._silence_ms = 0
            elif self._above > self.onset_frames:
                self._speech_ms += self.frame_ms
                self._silence_ms = 0
        else:
            self._above = 0
            if self._spoken:
                self._silence_ms += self.frame_ms

        # Which tolerance applies is fixed by where this run of silence
        # *started*, not by where it has got to — see the module
        # docstring on latching.
        in_lead = self.silence_started_ms < self.lead_window_ms
        tolerated = self.lead_silence_ms if in_lead else self.silence_ms

        if self._spoken and self._silence_ms >= tolerated:
            self._verdict = "silence"
        elif not self._spoken and self.lead_in_ms and self._elapsed_ms >= self.lead_in_ms:
            self._verdict = "lead_in"
        elif self.max_ms and self._elapsed_ms >= self.max_ms:
            self._verdict = "max"

        return self._verdict


class OpusEndOfSpeech:
    """:class:`EndOfSpeech` fed straight from the device's Opus frames.

    Wraps the decoder so the WebSocket read loop can hand over each
    binary frame exactly as it arrived. One 60 ms frame decodes in tens
    of microseconds, so this sits on the read loop without pacing it.

    If ``opuslib`` is missing the watcher reports itself
    :attr:`available` ``False`` and every frame is a no-op: a gateway
    without the codec extra keeps capturing exactly as it did before,
    just without an automatic stop. Losing the auto-stop is a
    degradation; refusing to record is a breakage.
    """

    def __init__(
        self,
        *,
        sample_rate: int = 16000,
        frame_ms: int = 60,
        **detector_kwargs: object,
    ) -> None:
        self.detector = EndOfSpeech(frame_ms=frame_ms, **detector_kwargs)  # type: ignore[arg-type]
        self._decoder = None
        self._error = ""
        try:
            from .stt.audio_utils import StreamingOpusDecoder

            self._decoder = StreamingOpusDecoder(
                sample_rate=sample_rate,
                frame_duration_ms=frame_ms,
            )
        except Exception as exc:  # opuslib absent, or no system libopus
            self._error = str(exc)

    @property
    def available(self) -> bool:
        """Whether frames can actually be decoded and judged."""
        return self._decoder is not None

    @property
    def error(self) -> str:
        """Why the watcher is unavailable, for a one-time log line."""
        return self._error

    def feed(self, frame: bytes) -> Optional[str]:
        """Decode and judge one Opus frame; see :meth:`EndOfSpeech.feed`.

        A frame that fails to decode is treated as silence rather than
        raising — one corrupt packet should not strand the window open,
        and it should not close it either.
        """
        if self._decoder is None:
            return None
        try:
            pcm = self._decoder.decode_frame(frame)
        except Exception:
            return self.detector.feed_rms(0.0)
        return self.detector.feed(pcm)


def settings_from_env() -> tuple[bool, dict[str, int]]:
    """Read tuning from the environment.

    Every knob is optional. A room with a noisy fan, or a speaker who
    pauses more than most, is a configuration problem rather than a code
    problem, and reflashing nothing and restarting one process is the
    cheapest way to find the right number.

    Values that will not parse as integers are ignored with a warning
    rather than taken as zero — a typo that silently set the silence
    window to nothing would close every capture on the first frame.
    """
    enabled = os.getenv("STACKCHAN_VAD_AUTOSTOP", "1").strip().lower() not in (
        "0", "false", "no", "off",
    )
    opts: dict[str, int] = {}
    for env, key in (
        ("STACKCHAN_VAD_SILENCE_MS", "silence_ms"),
        ("STACKCHAN_VAD_LEAD_SILENCE_MS", "lead_silence_ms"),
        ("STACKCHAN_VAD_LEAD_WINDOW_MS", "lead_window_ms"),
        ("STACKCHAN_VAD_LEAD_IN_MS", "lead_in_ms"),
        ("STACKCHAN_VAD_MAX_MS", "max_ms"),
    ):
        raw = os.getenv(env)
        if raw is None or not raw.strip():
            continue
        try:
            opts[key] = int(raw)
        except ValueError:
            logger.warning("%s=%r is not an integer; ignoring", env, raw)
    return enabled, opts
