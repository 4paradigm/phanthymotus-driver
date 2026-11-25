#!/usr/bin/env python3
"""Pure, ROS-free logic for the Bumi music beat performance card.

Everything that can silently produce a wrong answer lives here and is testable
without rclpy, the vendor SDK, or a robot:

  * WAV / raw-PCM16 decoding into the 16 kHz mono stream the speaker card owns;
  * beat detection from a PCM energy envelope;
  * the spec-driven choreography (which action on which beat, cooldowns, turn
    pulses) and the flat status payload reported to the dashboard.

The device-side module only reads a file, publishes AudioChunk frames, and
sends the commands this module schedules.  It never changes volume or calls any
``MediaController.set_*`` -- that 500 ms red line belongs to the speaker card.
"""

from __future__ import annotations

import math
import os
import struct
import wave
from dataclasses import dataclass
from typing import Iterable, Iterator

# The audio contract is the one SpeakerPlugin subscribes to: mono, little
# endian PCM16 at 16 kHz.  Raw .pcm files are interpreted with exactly those
# parameters; a WAV is rejected rather than silently resampled into noise.
SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2
FRAME_SAMPLES = 320                 # 20 ms at 16 kHz, matches the mic cadence
BEAT_WINDOW_S = 0.010               # 10 ms energy frames for beat detection
MIN_BEAT_GAP_S = 0.25               # <= 240 BPM; below this is one beat, not two
DEFAULT_FALLBACK_BPM = 120.0
MAX_DURATION_S = 120.0
MAX_FILE_BYTES = 64 * 1024 * 1024

# Action names and their vendor ControlCmd names mirror LocoPlugin's
# _PRESET_ACTIONS.  Only a subset is offered: these are the actions that read
# as a "performance" while the music keeps playing.
ACTION_COMMANDS = {
    "wave": "SWING",
    "cheer": "CHEER",
    "dance_1": "DANCE",
    "dance_2": "DANCE1",
    "dance_3": "DANCE2",
}
DEFAULT_ACTION_CYCLE = ("wave", "cheer", "dance_1", "dance_2")

# Anything under the driver's own work dir is reachable from /file/upload.
# Paths from the canvas are absolute container paths; accepting arbitrary
# paths would let a tool argument read the container's filesystem.
DEFAULT_ALLOWED_DIRS = ("/work/uploads", "/work")
ALLOWED_SUFFIXES = (".wav", ".pcm")


@dataclass(frozen=True)
class DecodedAudio:
    """A validated mono PCM16 stream, ready to be split into AudioChunks."""

    pcm: bytes
    sample_rate: int
    duration_s: float
    source: str                     # "wav" or "pcm"

    @property
    def frames(self) -> int:
        return len(self.pcm) // (FRAME_SAMPLES * SAMPLE_WIDTH)


@dataclass(frozen=True)
class Beat:
    time_s: float
    strength: float


@dataclass(frozen=True)
class PerformanceEvent:
    """One scheduled edge, consumed by the device thread at its beat time."""

    time_s: float
    kind: str                       # "semantic_action" or "turn_pulse"
    action: str | None = None
    command: str | None = None
    vyaw: float = 0.0
    duration_ms: int = 0


class BeatSpecError(ValueError):
    """A bad file or bad configuration; always surfaced to the caller as-is."""


def validate_audio_path(
    path: object,
    allowed_dirs: Iterable[str] = DEFAULT_ALLOWED_DIRS,
) -> str:
    """Confine a canvas-supplied upload path to the driver's own directories.

    Uploads land under ``/work/uploads`` (see ``common/file_intake.py``).
    Accepting ``../../etc/passwd`` or a symlink-friendly arbitrary path would
    turn a file-picker argument into a filesystem read primitive.
    """
    if not isinstance(path, str) or not path.strip():
        raise BeatSpecError("audio_file is required")
    candidate = os.path.abspath(os.path.normpath(path.strip()))
    suffix = os.path.splitext(candidate)[1].lower()
    if suffix not in ALLOWED_SUFFIXES:
        raise BeatSpecError(
            f"audio_file must end with one of {', '.join(ALLOWED_SUFFIXES)}"
        )
    # A link inside the upload dir can still point at /etc.  Reject links
    # outright and check containment against the *resolved* target, including
    # any linked parent directory, before the file is ever opened.
    if os.path.islink(candidate):
        raise BeatSpecError("audio_file must not be a symbolic link")
    real_candidate = os.path.realpath(candidate)
    for raw_root in allowed_dirs:
        root = os.path.realpath(os.path.abspath(os.path.normpath(raw_root)))
        try:
            if os.path.commonpath((root, real_candidate)) == root:
                return real_candidate
        except ValueError:
            continue
    raise BeatSpecError(
        "audio_file must be an uploaded file under "
        + ", ".join(DEFAULT_ALLOWED_DIRS)
    )


def _wav_to_mono_pcm16(wav: wave.Wave_read) -> bytes:
    if wav.getsampwidth() != SAMPLE_WIDTH:
        raise BeatSpecError("WAV must be 16-bit PCM")
    rate = wav.getframerate()
    if rate != SAMPLE_RATE:
        raise BeatSpecError(
            f"WAV sample rate must be {SAMPLE_RATE} Hz, got {rate} Hz; "
            "resample before uploading"
        )
    channels = wav.getnchannels()
    if channels not in (1, 2):
        raise BeatSpecError("WAV must be mono or stereo")
    payload = wav.readframes(wav.getnframes())
    if channels == 1:
        return payload
    samples = struct.unpack(f"<{len(payload) // SAMPLE_WIDTH}h", payload)
    mono = [(left + right) // 2
            for left, right in zip(samples[0::2], samples[1::2])]
    return struct.pack(f"<{len(mono)}h", *mono)


def decode_audio(
    path: str,
    *,
    max_duration_s: float = MAX_DURATION_S,
    max_bytes: int = MAX_FILE_BYTES,
    allowed_dirs: Iterable[str] = DEFAULT_ALLOWED_DIRS,
) -> DecodedAudio:
    """Decode a validated WAV or raw PCM16 file without touching ROS."""
    candidate = validate_audio_path(path, allowed_dirs)
    # Re-check after validation: a link swapped in between the two calls must
    # not turn a confined upload into a read of an arbitrary host file.
    if os.path.islink(candidate) or not os.path.isfile(candidate):
        raise BeatSpecError("audio_file does not exist or is a symbolic link")
    size = os.path.getsize(candidate)
    if not 0 < size <= max_bytes:
        raise BeatSpecError(f"audio_file must be 1..{max_bytes} bytes")

    if candidate.lower().endswith(".wav"):
        try:
            with wave.open(candidate, "rb") as wav:
                pcm = _wav_to_mono_pcm16(wav)
        except (wave.Error, EOFError, struct.error) as exc:
            raise BeatSpecError(f"cannot decode WAV: {exc}") from exc
        source = "wav"
    else:
        with open(candidate, "rb") as handle:
            pcm = handle.read()
        source = "pcm"

    if len(pcm) < SAMPLE_WIDTH or len(pcm) % SAMPLE_WIDTH:
        raise BeatSpecError("audio stream must contain whole 16-bit samples")
    duration = len(pcm) / (SAMPLE_RATE * SAMPLE_WIDTH)
    if duration > max_duration_s:
        raise BeatSpecError(
            f"audio is {duration:.1f}s, longer than the {max_duration_s:.0f}s limit"
        )
    return DecodedAudio(pcm=pcm, sample_rate=SAMPLE_RATE,
                        duration_s=duration, source=source)


def _frame_rms(pcm: bytes, samples_per_frame: int) -> list[float]:
    values = struct.unpack(f"<{len(pcm) // SAMPLE_WIDTH}h", pcm)
    frame_count = len(values) // samples_per_frame
    result = []
    for index in range(frame_count):
        frame = values[index * samples_per_frame:(index + 1) * samples_per_frame]
        result.append(math.sqrt(sum(v * v for v in frame) / samples_per_frame))
    return result


def _moving_average(values: list[float], window: int) -> list[float]:
    if not values:
        return []
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)
    half = window // 2
    result = []
    for index in range(len(values)):
        lo = max(0, index - half)
        hi = min(len(values), index + half + 1)
        result.append((prefix[hi] - prefix[lo]) / (hi - lo))
    return result


def detect_beats(
    audio: DecodedAudio,
    *,
    fallback_bpm: float = DEFAULT_FALLBACK_BPM,
) -> tuple[list[Beat], str]:
    """Return (beats, source) from a 10 ms RMS envelope.

    A real peak table is preferred.  A silent or rhythm-free clip yields too
    few peaks to choreograph, so a uniform metronome table is used instead and
    the ``source`` says so -- the card must never claim it detected beats it
    did not detect.
    """
    if audio.sample_rate != SAMPLE_RATE:
        raise BeatSpecError("beat detection requires a 16 kHz stream")
    if not math.isfinite(fallback_bpm) or not 40.0 <= fallback_bpm <= 240.0:
        raise BeatSpecError("fallback_bpm must be in [40, 240]")
    samples_per_frame = int(round(BEAT_WINDOW_S * audio.sample_rate))
    rms = _frame_rms(audio.pcm, samples_per_frame)
    if not rms:
        return [], "empty"

    window = max(3, int(round(0.30 / BEAT_WINDOW_S)) | 1)
    local_mean = _moving_average(rms, window)
    floor = 80.0
    beats: list[Beat] = []
    last_time = -MIN_BEAT_GAP_S
    for index, value in enumerate(rms):
        time_s = index * BEAT_WINDOW_S
        threshold = max(floor, 1.35 * local_mean[index])
        if value < threshold:
            continue
        # An onset (rising edge), not the peak of a long burst: a 40 ms click
        # then reads as one beat at its start instead of four at its tail.
        previous = rms[index - 1] if index > 0 else 0.0
        if previous >= threshold:
            continue
        if time_s - last_time < MIN_BEAT_GAP_S:
            continue
        beats.append(Beat(time_s=round(time_s, 3), strength=round(value, 2)))
        last_time = time_s

    if len(beats) >= 4:
        return beats, "detected"

    interval = 60.0 / fallback_bpm
    beats = [
        Beat(time_s=round(index * interval, 3), strength=0.0)
        for index in range(int(audio.duration_s / interval))
    ]
    return beats, "fallback_bpm"


def _parse_cycle(raw: object) -> tuple[str, ...]:
    if isinstance(raw, str):
        names = tuple(part.strip() for part in raw.split(",") if part.strip())
    elif isinstance(raw, (list, tuple)):
        names = tuple(str(part).strip() for part in raw if str(part).strip())
    else:
        raise BeatSpecError("action_cycle must be a comma-separated string")
    if not names or len(names) > 16:
        raise BeatSpecError("action_cycle must contain 1..16 actions")
    for name in names:
        if name not in ACTION_COMMANDS:
            raise BeatSpecError(
                f"action_cycle contains unsupported action {name!r}; "
                f"allowed: {', '.join(ACTION_COMMANDS)}"
            )
    return names


def normalise_config(config: dict | None) -> dict:
    """Validate and fill defaults for every runtime-tunable choreography knob."""
    raw = dict(config or {})

    def integer(name: str, default: int, low: int, high: int) -> int:
        value = raw.get(name, default)
        if isinstance(value, bool):
            raise BeatSpecError(f"{name} must be an integer")
        try:
            result = int(value)
        except (TypeError, ValueError) as exc:
            raise BeatSpecError(f"{name} must be an integer") from exc
        if not low <= result <= high:
            raise BeatSpecError(f"{name} must be in [{low}, {high}]")
        return result

    def number(name: str, default: float, low: float, high: float) -> float:
        value = raw.get(name, default)
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise BeatSpecError(f"{name} must be a number") from exc
        if not math.isfinite(result) or not low <= result <= high:
            raise BeatSpecError(f"{name} must be in [{low}, {high}]")
        return result

    cycle = _parse_cycle(raw.get("action_cycle", ",".join(DEFAULT_ACTION_CYCLE)))
    return {
        "action_cycle": cycle,
        "action_every_beats": integer("action_every_beats", 4, 1, 32),
        "turn_every_beats": integer("turn_every_beats", 8, 1, 32),
        "action_cooldown_s": number("action_cooldown_s", 3.0, 0.5, 30.0),
        "turn_cooldown_s": number("turn_cooldown_s", 4.0, 0.5, 30.0),
        # A normalized yaw command well below the ~0.4 locomotion stiction
        # band would be a no-op; 0.5 is a deliberate small but real pulse.
        "turn_vyaw": number("turn_vyaw", 0.5, -1.0, 1.0),
        "turn_frames": integer("turn_frames", 3, 1, 10),
        "frame_ms": integer("frame_ms", FRAME_SAMPLES * 1000 // SAMPLE_RATE, 10, 100),
        "fallback_bpm": number("fallback_bpm", DEFAULT_FALLBACK_BPM, 40.0, 240.0),
        "max_duration_s": number("max_duration_s", MAX_DURATION_S, 1.0, MAX_DURATION_S),
    }


def build_choreography(beats: list[Beat], config: dict) -> list[PerformanceEvent]:
    """Map beat indices to spec-owned actions and turn pulses.

    The action order and the beat-to-action relationship are intentionally a
    pure function of the spec: changing ``action_cycle`` or ``*_every_beats``
    changes the performance without any device-side edits.  Cooldowns prevent
    two edges landing on adjacent beats, which the vendor driver would read
    as a single held command.
    """
    events: list[PerformanceEvent] = []
    last_action_s = -config["action_cooldown_s"]
    last_turn_s = -config["turn_cooldown_s"]
    for index, beat in enumerate(beats):
        beat_number = index + 1
        if (beat_number % config["action_every_beats"] == 0
                and beat.time_s - last_action_s >= config["action_cooldown_s"]):
            action = config["action_cycle"][
                (beat_number // config["action_every_beats"] - 1) % len(config["action_cycle"])
            ]
            events.append(PerformanceEvent(
                time_s=beat.time_s, kind="semantic_action",
                action=action, command=ACTION_COMMANDS[action],
            ))
            last_action_s = beat.time_s
        if (config["turn_vyaw"] != 0.0
                and beat_number % config["turn_every_beats"] == 0
                and beat.time_s - last_turn_s >= config["turn_cooldown_s"]):
            frame_ms = config["frame_ms"]
            events.append(PerformanceEvent(
                time_s=beat.time_s, kind="turn_pulse",
                vyaw=config["turn_vyaw"],
                duration_ms=config["turn_frames"] * frame_ms,
            ))
            last_turn_s = beat.time_s
    events.sort(key=lambda event: (event.time_s, event.kind))
    return events


def iter_audio_frames(pcm: bytes, frame_ms: int = 20) -> Iterator[bytes]:
    """Split a PCM stream into fixed-duration AudioChunk payloads.

    The last frame is padded with silence so its duration stays exact; a
    short final chunk would make the speaker's playback clock drift.
    """
    samples = frame_ms * SAMPLE_RATE // 1000
    if samples <= 0:
        raise BeatSpecError("frame_ms is too small")
    total = len(pcm) // SAMPLE_WIDTH
    for start in range(0, total, samples):
        chunk = pcm[start * SAMPLE_WIDTH:(start + samples) * SAMPLE_WIDTH]
        if len(chunk) < samples * SAMPLE_WIDTH:
            chunk += b"\x00" * (samples * SAMPLE_WIDTH - len(chunk))
        yield chunk


def protection_result(action: str, workmode: int) -> dict:
    """The honest, non-mutating answer when workmode == 26 blocks all commands."""
    return {
        "state": "error",
        "command_sent": False,
        "action": action,
        "current_workmode": workmode,
        "current_workmode_name": "protection",
        "protection": True,
        "error": "Robot is in protection mode (workmode=26); no beat action was sent.",
        "recovery": "Stop the performance and restart the robot as described in the Bumi product guide.",
    }


def status_payload(
    *, state: str, audio_file: str | None, beat_count: int, beat_source: str,
    event_count: int, events_fired: int, action_count: int, turn_count: int,
    frames_published: int, duration_s: float, elapsed_s: float,
    last_action: str | None, last_action_drift_ms: float | None,
    workmode: int | None, protection: bool, last_error: str | None,
) -> dict:
    """Flat scalars only: nested objects render as raw JSON on the dashboard."""
    return {
        "state": state,
        "audio_file": audio_file,
        "beat_count": beat_count,
        "beat_source": beat_source,
        "event_count": event_count,
        "events_fired": events_fired,
        "action_count": action_count,
        "turn_count": turn_count,
        "frames_published": frames_published,
        "duration_s": round(duration_s, 3),
        "elapsed_s": round(elapsed_s, 3),
        "last_action": last_action,
        "last_action_drift_ms": (
            None if last_action_drift_ms is None else round(last_action_drift_ms, 1)
        ),
        "workmode": workmode,
        "protection": protection,
        "last_error": last_error,
    }
