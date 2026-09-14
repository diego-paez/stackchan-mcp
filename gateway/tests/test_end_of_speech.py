"""Tests for end-of-utterance detection on device-driven captures.

The numbers here are not invented. They come from a Stack-chan capture
that ran 32.22 s for a nine-second question — the failure this module
exists to prevent — measured at 60 ms resolution:

    speech   0.66-3.00 s   "Can you hear me?"
    gap      3.00-4.38 s   (1.38 s — the speaker waiting for an answer)
    speech   4.38-8.82 s   "So could you tell me a story...?"
    silence  8.82-32.22 s  (room tone, until something unrelated ended it)

and the pauses *inside* that second sentence, which the detector must
sit through: 0.90, 0.06, 0.12, 0.48, 0.36, 0.54 s.
"""

from __future__ import annotations

import math
import struct

import pytest

from stackchan_mcp.end_of_speech import (
    DEFAULT_SILENCE_MS,
    EndOfSpeech,
    OpusEndOfSpeech,
    frame_rms,
)
from stackchan_mcp.end_of_speech import _SEED_FRAMES as SEED_FRAMES

#: Levels measured on the real capture: room tone, and speech.
QUIET = 0.001
LOUD = 0.012


def feed(detector: EndOfSpeech, pattern: str, *, seed: bool = True) -> str | None:
    """Feed a frame pattern; ``#`` is speech, ``.`` is silence.

    By default the pattern is preceded by enough room tone for the
    detector to seed its noise floor, which is what a real capture
    always supplies: the microphone is running before the speaker
    starts. Pass ``seed=False`` to test what happens when it is not.

    Returns the verdict, or ``None`` if the window stayed open.
    """
    verdict = None
    if seed:
        for _ in range(SEED_FRAMES):
            detector.feed_rms(QUIET)
    for ch in pattern:
        verdict = detector.feed_rms(LOUD if ch == "#" else QUIET)
        if verdict:
            break
    return verdict


def frames(seconds: float, amplitude: float, frame_ms: int = 60) -> list[bytes]:
    """Signed 16-bit mono PCM frames of a tone at a given amplitude."""
    n = int(16000 * frame_ms / 1000)
    out = []
    phase = 0.0
    for _ in range(int(seconds * 1000 / frame_ms)):
        samples = []
        for _ in range(n):
            samples.append(int(amplitude * 32767 * math.sin(phase)))
            phase += 2 * math.pi * 220 / 16000
        out.append(struct.pack(f"<{n}h", *samples))
    return out


# --- frame_rms ---------------------------------------------------------------


def test_frame_rms_of_silence_is_zero():
    assert frame_rms(struct.pack("<480h", *([0] * 480))) == 0.0


def test_frame_rms_of_full_scale_is_one():
    full = struct.pack("<480h", *([32767] * 480))
    assert frame_rms(full) == pytest.approx(1.0, abs=0.001)


def test_frame_rms_tolerates_a_truncated_frame():
    """A half-delivered frame is silence, not an exception."""
    assert frame_rms(b"\x00") == 0.0
    assert frame_rms(b"") == 0.0


def test_frame_rms_ignores_a_trailing_odd_byte():
    good = struct.pack("<4h", 1000, 1000, 1000, 1000)
    assert frame_rms(good + b"\x7f") == pytest.approx(frame_rms(good))


# --- the ordinary case -------------------------------------------------------


def test_closes_after_the_configured_silence():
    d = EndOfSpeech(silence_ms=1000, lead_window_ms=0)
    # 1 s of speech, then silence. 1000 ms of silence is 17 frames at 60 ms.
    assert feed(d, "#" * 17 + "." * 16) is None
    assert d.feed_rms(QUIET) == "silence"


def test_stays_open_while_someone_is_talking():
    d = EndOfSpeech(silence_ms=1000, max_ms=0)
    assert feed(d, "#" * 200) is None


def test_stays_open_through_a_pause_inside_a_sentence():
    """The longest measured intra-sentence pause was 0.90 s."""
    d = EndOfSpeech(lead_window_ms=0)
    assert feed(d, "#" * 10 + "." * 15 + "#" * 10 + "." * 5) is None


def test_the_default_clears_the_longest_measured_pause():
    """0.90 s inside a sentence must not end the window."""
    assert DEFAULT_SILENCE_MS > 900


def test_verdict_latches_once_decided():
    d = EndOfSpeech(silence_ms=120, lead_window_ms=0)
    feed(d, "#" * 5 + "." * 5)
    assert d.verdict == "silence"
    # Loud audio afterwards must not reopen a window already closed.
    assert d.feed_rms(LOUD) == "silence"
    assert d.verdict == "silence"


# --- the lead-in, which is the whole reason for two thresholds ---------------


def test_the_long_pause_before_a_sentence_does_not_close_the_window():
    """The 1.38 s gap from the real capture, at its real position."""
    d = EndOfSpeech(silence_ms=1000, lead_silence_ms=2000, lead_window_ms=4000)
    # ~2.3 s of speech, then the 1.38 s gap (23 frames), then more speech.
    assert feed(d, "#" * 39 + "." * 23 + "#" * 10) is None


def test_the_same_pause_later_in_the_capture_does_close_it():
    """Past the lead window, 1.38 s of quiet is the end of the turn."""
    d = EndOfSpeech(silence_ms=1000, lead_silence_ms=2000, lead_window_ms=4000)
    assert feed(d, "#" * 80 + "." * 23) == "silence"


def test_lead_tolerance_is_latched_on_the_gap_that_started_it():
    """A gap opening inside the lead window keeps the generous rule.

    Deciding afresh each frame would flip this gap to the tight rule as
    the clock crossed the lead window and cut off a speaker who had not
    started yet.
    """
    d = EndOfSpeech(silence_ms=300, lead_silence_ms=2000, lead_window_ms=1200)
    # Speech ends at 600 ms, inside the lead window; the gap then runs
    # well past it. The tight 300 ms rule must not apply.
    assert feed(d, "#" * 10 + "." * 20) is None


def test_lead_silence_is_never_shorter_than_the_tight_one():
    d = EndOfSpeech(silence_ms=1500, lead_silence_ms=200)
    assert d.lead_silence_ms == 1500


# --- windows that must not stay open forever ---------------------------------


def test_a_window_nobody_spoke_in_closes_on_the_lead_in_cap():
    d = EndOfSpeech(lead_in_ms=1200, max_ms=0)
    assert feed(d, "." * 100) == "lead_in"
    assert not d.heard_speech


def test_a_window_that_never_goes_quiet_closes_on_the_hard_cap():
    d = EndOfSpeech(max_ms=1200, silence_ms=100000)
    assert feed(d, "#" * 200) == "max"


def test_the_hard_cap_keeps_a_capture_inside_whisper_s_window():
    """Whisper's encoder takes 30 s; past that a capture is lost."""
    assert EndOfSpeech().max_ms < 30000


# --- the threshold itself ----------------------------------------------------


def test_a_single_loud_frame_is_not_speech():
    """A chair creak clears any sensible threshold; two frames do not."""
    d = EndOfSpeech(onset_frames=2, lead_in_ms=1200, max_ms=0)
    assert feed(d, ("#" + "." * 9) * 10) == "lead_in"
    assert not d.heard_speech


def test_the_floor_follows_the_quiet_level_of_the_room():
    """Speech is a multiple of the floor, so a noisier room raises the bar."""
    quiet = EndOfSpeech(floor_min=0.0001)
    for _ in range(200):
        quiet.feed_rms(0.0005)
    noisy = EndOfSpeech(floor_min=0.0001)
    for _ in range(200):
        noisy.feed_rms(0.004)
    assert noisy.noise_floor > quiet.noise_floor * 3


def test_sustained_sound_cannot_lift_the_floor_over_its_own_head():
    """The failure that a naive rise-toward-the-frame smoothing has.

    Hold one level long enough and the floor climbs until the threshold
    crosses above it, at which point the sound holding the floor up
    reads as silence and the window closes mid-sentence.
    """
    d = EndOfSpeech(silence_ms=1000, max_ms=0)
    for _ in range(SEED_FRAMES):
        d.feed_rms(QUIET)
    for _ in range(1000):  # a full minute of unbroken speech
        assert d.feed_rms(LOUD) is None
    assert d.noise_floor * d.floor_ratio < LOUD


def test_the_floor_is_seeded_from_the_quietest_opening_frame():
    """A capture that opens mid-syllable must not seed on that syllable.

    Seeding on the first frame alone would put the floor above
    everything that followed, and nothing would ever read as speech
    again.
    """
    d = EndOfSpeech(silence_ms=1000, lead_window_ms=0, max_ms=0)
    d.feed_rms(LOUD)  # opens mid-word
    for _ in range(SEED_FRAMES - 1):
        d.feed_rms(QUIET)
    assert d.seeded
    assert d.noise_floor * d.floor_ratio < LOUD
    assert feed(d, "#" * 10, seed=False) is None
    assert d.heard_speech


def test_a_room_that_hums_seeds_a_higher_floor():
    """Otherwise every frame of the hum reads as speech, forever."""
    hum = EndOfSpeech(floor_min=0.0008)
    for _ in range(SEED_FRAMES):
        hum.feed_rms(0.004)
    assert hum.noise_floor == pytest.approx(0.004)
    # The hum itself is now below the threshold, so it is not speech.
    assert hum.feed_rms(0.004) is None
    assert not hum.heard_speech


def test_nothing_is_judged_before_the_floor_is_real():
    d = EndOfSpeech(silence_ms=60, lead_window_ms=0, lead_in_ms=60, max_ms=60)
    for _ in range(SEED_FRAMES - 1):
        assert d.feed_rms(QUIET) is None
    assert not d.seeded


def test_the_floor_never_falls_to_zero():
    """A silent room must not turn the threshold into a hair trigger."""
    d = EndOfSpeech(floor_min=0.0008)
    for _ in range(500):
        d.feed_rms(0.0)
    assert d.noise_floor == pytest.approx(0.0008)


def test_speech_far_below_the_floor_ratio_reads_as_silence():
    d = EndOfSpeech(floor_min=0.001, floor_ratio=3.5, lead_in_ms=1200, max_ms=0)
    assert feed(d, "x" * 100 and "".join("." for _ in range(100))) == "lead_in"


def test_rejects_a_nonsense_frame_size():
    with pytest.raises(ValueError):
        EndOfSpeech(frame_ms=0)


# --- the PCM entry point -----------------------------------------------------


def test_feed_accepts_pcm_frames():
    d = EndOfSpeech(silence_ms=600, lead_window_ms=0, frame_ms=60)
    verdict = None
    # Room tone first, as any real capture has, so the floor is seeded
    # on the room and not on the speech.
    for frame in frames(0.6, 0.0) + frames(1.0, 0.2) + frames(1.0, 0.0):
        verdict = d.feed(frame)
        if verdict:
            break
    assert verdict == "silence"
    assert d.heard_speech


def test_elapsed_tracks_the_audio_fed():
    d = EndOfSpeech(frame_ms=60, max_ms=0, silence_ms=100000)
    for _ in range(10):
        d.feed_rms(QUIET)
    assert d.elapsed_ms == 600


# --- the Opus wrapper --------------------------------------------------------


def test_opus_watcher_without_the_codec_is_inert_not_broken(monkeypatch):
    """No codec means no auto-stop — never a refusal to record."""
    import stackchan_mcp.end_of_speech as mod

    watcher = mod.OpusEndOfSpeech.__new__(mod.OpusEndOfSpeech)
    watcher.detector = EndOfSpeech()
    watcher._decoder = None
    watcher._error = "opuslib is not installed"

    assert watcher.available is False
    assert watcher.feed(b"\xff" * 40) is None
    assert watcher.error


def test_opus_watcher_treats_an_undecodable_frame_as_silence():
    class _Boom:
        def decode_frame(self, frame):
            raise RuntimeError("corrupt packet")

    watcher = OpusEndOfSpeech.__new__(OpusEndOfSpeech)
    watcher.detector = EndOfSpeech(lead_in_ms=1200, max_ms=0)
    watcher._decoder = _Boom()
    watcher._error = ""

    verdict = None
    for _ in range(100):
        verdict = watcher.feed(b"\x00" * 40)
        if verdict:
            break
    # Silence, so the lead-in cap closes it — the window never strands.
    assert verdict == "lead_in"
