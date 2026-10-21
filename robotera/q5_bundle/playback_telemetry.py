"""Playback task identity, framing, and thread-safe telemetry reduction."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import struct
import threading
import time
from typing import BinaryIO, Callable, Dict, List, Optional, Tuple
from urllib.parse import unquote


_PACKET_MAGIC = b"Q5PT"
_PACKET_PREFIX = struct.Struct("!4sII")
_MAX_HEADER_BYTES = 64 * 1024
_MAX_PCM_BYTES = 16 * 1024 * 1024
_TERMINAL_STATES = {"COMPLETED", "INTERRUPTED", "FAILED"}


@dataclass(frozen=True)
class AudioChunkMetadata:
    base_format: str
    speech_id: Optional[str] = None
    sequence: Optional[int] = None
    eof: bool = False
    tracked: bool = False
    valid: bool = True
    error: Optional[str] = None


def parse_audio_format(value: str) -> AudioChunkMetadata:
    """Parse the backward-compatible AudioChunk format parameter extension."""
    if not isinstance(value, str) or not value:
        return AudioChunkMetadata("", valid=False, error="audio format is empty")
    parts = value.split(";")
    base_format = parts[0].strip()
    if len(parts) == 1:
        return AudioChunkMetadata(base_format=base_format)

    params: Dict[str, str] = {}
    for part in parts[1:]:
        if "=" not in part:
            return AudioChunkMetadata(base_format, valid=False,
                                      error=f"invalid audio metadata parameter: {part}")
        key, raw_value = part.split("=", 1)
        if key in params or key not in {"speech_id", "sequence", "eof"}:
            return AudioChunkMetadata(base_format, valid=False,
                                      error=f"unsupported or duplicate audio metadata: {key}")
        params[key] = raw_value

    if set(params) != {"speech_id", "sequence", "eof"}:
        return AudioChunkMetadata(base_format, valid=False,
                                  error="tracked audio requires speech_id, sequence, and eof")
    speech_id = unquote(params["speech_id"])
    if not speech_id or len(speech_id) > 128 or any(ord(char) < 32 for char in speech_id):
        return AudioChunkMetadata(base_format, valid=False, error="invalid speech_id")
    try:
        sequence = int(params["sequence"])
    except ValueError:
        sequence = -1
    if sequence < 0 or str(sequence) != params["sequence"]:
        return AudioChunkMetadata(base_format, valid=False, error="invalid sequence")
    if params["eof"] not in {"0", "1"}:
        return AudioChunkMetadata(base_format, valid=False, error="invalid eof")
    return AudioChunkMetadata(base_format, speech_id, sequence, params["eof"] == "1",
                              tracked=True)


def encode_packet(metadata: AudioChunkMetadata, pcm: bytes) -> bytes:
    """Encode one metadata/PCM record for the remote player's stdin."""
    if not metadata.valid:
        raise ValueError(metadata.error or "invalid audio metadata")
    payload = bytes(pcm)
    header = json.dumps({
        "format": metadata.base_format,
        "speech_id": metadata.speech_id,
        "sequence": metadata.sequence,
        "eof": metadata.eof,
        "tracked": metadata.tracked,
    }, separators=(",", ":"), ensure_ascii=True).encode("ascii")
    if len(header) > _MAX_HEADER_BYTES or len(payload) > _MAX_PCM_BYTES:
        raise ValueError("playback packet is too large")
    return _PACKET_PREFIX.pack(_PACKET_MAGIC, len(header), len(payload)) + header + payload


def _read_exact(stream: BinaryIO, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            raise EOFError("truncated playback packet")
        chunks.extend(chunk)
    return bytes(chunks)


def decode_packet(stream: BinaryIO) -> Tuple[AudioChunkMetadata, bytes]:
    """Decode one framed record. Used by fake backends and the remote protocol."""
    prefix = _read_exact(stream, _PACKET_PREFIX.size)
    magic, header_size, pcm_size = _PACKET_PREFIX.unpack(prefix)
    if magic != _PACKET_MAGIC:
        raise ValueError("invalid playback packet magic")
    if header_size > _MAX_HEADER_BYTES or pcm_size > _MAX_PCM_BYTES:
        raise ValueError("playback packet is too large")
    try:
        header = json.loads(_read_exact(stream, header_size).decode("ascii"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid playback packet header") from exc
    metadata = AudioChunkMetadata(
        base_format=str(header.get("format", "")),
        speech_id=header.get("speech_id"),
        sequence=header.get("sequence"),
        eof=header.get("eof") is True,
        tracked=header.get("tracked") is True,
    )
    if metadata.tracked and (not metadata.speech_id or not isinstance(metadata.sequence, int)):
        raise ValueError("invalid tracked playback packet header")
    return metadata, _read_exact(stream, pcm_size)


@dataclass
class _Speech:
    speech_id: str
    state: str = "SPEAKER_RECEIVED"
    next_sequence: int = 0
    eof_received: bool = False
    total_input_samples: int = 0
    submitted_frames: int = 0
    consumed_frames: int = 0
    total_output_frames: Optional[int] = None
    generation: Optional[int] = None
    device_sample_rate_hz: Optional[int] = None
    underrun_count: int = 0
    speaker_received_at_ns: Optional[int] = None
    playback_started_at_ns: Optional[int] = None
    playback_finished_at_ns: Optional[int] = None
    error_code: Optional[str] = None
    error_message: Optional[str] = None
    completion_level: Optional[str] = None
    device_progress_trusted: bool = False
    trusted_consumed_frames: int = 0
    duplicate_chunk_count: int = 0


class PlaybackTracker:
    """Reduce bridge and remote ALSA facts into monotonic per-speech state."""

    def __init__(self, clock_ns: Callable[[], int] = time.monotonic_ns,
                 wall_clock_ns: Callable[[], int] = time.time_ns,
                 max_events: int = 2048, max_speeches: int = 10_000):
        self._clock_ns = clock_ns
        self._wall_clock_ns = wall_clock_ns
        self._max_events = max(1, int(max_events))
        self._max_speeches = max(1, int(max_speeches))
        self._condition = threading.Condition(threading.RLock())
        self._speeches: Dict[str, _Speech] = {}
        self._rejections: Dict[str, _Speech] = {}
        self._events: List[dict] = []
        self._generation: Optional[int] = None

    def _reject_new_speech(self, speech_id: str, code: str, message: str) -> bool:
        speech = _Speech(speech_id, state="FAILED", error_code=code,
                         error_message=message)
        self._rejections[speech_id] = speech
        while len(self._rejections) > self._max_speeches:
            del self._rejections[next(iter(self._rejections))]
        self._emit(speech, "failed", error_code=code, details_message=message)
        self._condition.notify_all()
        return False

    def _emit(self, speech: _Speech, event: str, source_timestamp_ns: Optional[int] = None,
              source_clock_id: str = "driver_host_monotonic",
              local_received_monotonic_ns: Optional[int] = None, **details) -> None:
        item = {
            "speech_id": speech.speech_id,
            "event": event,
            "state": speech.state,
            "source_clock_id": source_clock_id,
            "source_timestamp_ns": source_timestamp_ns,
            "local_received_monotonic_ns": (
                int(self._clock_ns()) if local_received_monotonic_ns is None
                else int(local_received_monotonic_ns)),
            "local_received_wall_ns": int(self._wall_clock_ns()),
            **details,
        }
        self._events.append(item)
        if len(self._events) > self._max_events:
            del self._events[:-self._max_events]

    def receive_chunk(self, metadata: AudioChunkMetadata, input_samples: int) -> bool:
        if not metadata.valid or not metadata.tracked or metadata.speech_id is None:
            return False
        with self._condition:
            if metadata.speech_id in self._rejections:
                return False
            speech = self._speeches.get(metadata.speech_id)
            if speech is None:
                if metadata.sequence != 0:
                    return self._reject_new_speech(
                        metadata.speech_id, "CHUNK_SEQUENCE_ERROR",
                        "first sequence is not zero")
                while len(self._speeches) >= self._max_speeches:
                    terminal_id = next((speech_id for speech_id, item in self._speeches.items()
                                        if item.state in _TERMINAL_STATES), None)
                    if terminal_id is None:
                        break
                    del self._speeches[terminal_id]
                if len(self._speeches) >= self._max_speeches:
                    return self._reject_new_speech(
                        metadata.speech_id, "PLAYBACK_CAPACITY",
                        "playback tracker capacity is exhausted by active tasks")
                now = int(self._clock_ns())
                speech = _Speech(metadata.speech_id, speaker_received_at_ns=now)
                self._speeches[metadata.speech_id] = speech
                self._emit(speech, "speaker_received", source_timestamp_ns=now,
                           local_received_monotonic_ns=now)
            if speech.state in _TERMINAL_STATES or speech.eof_received:
                speech.error_code = "CHUNK_AFTER_TERMINAL"
                return False
            if metadata.sequence != speech.next_sequence:
                if metadata.sequence < speech.next_sequence:
                    speech.duplicate_chunk_count += 1
                    return False
                speech.state = "FAILED"
                speech.error_code = "CHUNK_SEQUENCE_ERROR"
                speech.error_message = (
                    f"expected sequence {speech.next_sequence}, got {metadata.sequence}")
                self._emit(speech, "failed", error_code=speech.error_code,
                           details_message=speech.error_message)
                self._condition.notify_all()
                return False
            speech.next_sequence += 1
            speech.total_input_samples += max(0, int(input_samples))
            speech.eof_received = metadata.eof
            return True

    def record_submitted(self, speech_id: str, submitted_frames: int,
                         total_output_frames: Optional[int] = None, generation: int = 0,
                         remote_timestamp_ns: int = 0,
                         device_sample_rate_hz: Optional[int] = None) -> bool:
        with self._condition:
            speech = self._speeches.get(speech_id)
            if speech is None or speech.state in _TERMINAL_STATES:
                return False
            if self._generation is not None and generation != self._generation:
                return False
            self._generation = generation
            if speech.generation is not None and generation != speech.generation:
                return False
            speech.generation = generation
            if device_sample_rate_hz is not None:
                speech.device_sample_rate_hz = int(device_sample_rate_hz)
            speech.submitted_frames = max(speech.submitted_frames, int(submitted_frames))
            if total_output_frames is not None and speech.eof_received:
                speech.total_output_frames = max(0, int(total_output_frames))
            if speech.total_output_frames == 0:
                speech.state = "FAILED"
                speech.error_code = "EMPTY_PCM"
                speech.error_message = "speech produced no output frames"
                self._emit(speech, "failed", remote_timestamp_ns,
                           "q5_audio_host_monotonic", error_code=speech.error_code,
                           details_message=speech.error_message)
                self._condition.notify_all()
                return True
            speech.state = "PCM_SUBMITTED"
            self._emit(speech, "pcm_submitted", remote_timestamp_ns,
                       "q5_audio_host_monotonic",
                       submitted_frames=speech.submitted_frames,
                       total_output_frames=speech.total_output_frames,
                       timestamp_quality="PCM_SUBMITTED")
            return True

    def record_underrun(self, speech_id: str, generation: int,
                        remote_timestamp_ns: int) -> bool:
        with self._condition:
            speech = self._speeches.get(speech_id)
            if speech is None or speech.state in _TERMINAL_STATES:
                return False
            if self._generation is not None and generation != self._generation:
                return False
            if speech.generation is not None and generation != speech.generation:
                return False
            self._generation = generation
            speech.generation = generation
            speech.underrun_count += 1
            speech.state = "FAILED"
            speech.error_code = "ALSA_UNDERRUN"
            speech.error_message = "ALSA underrun discarded buffered playback frames"
            self._emit(speech, "underrun", remote_timestamp_ns,
                       "q5_audio_host_monotonic", error_code="ALSA_UNDERRUN",
                       underrun_count=speech.underrun_count,
                       timestamp_quality="DEVICE_PROGRESS_OBSERVED")
            self._condition.notify_all()
            return True

    def record_device_progress(self, speech_id: str, consumed_frames: int,
                               generation: int, remote_timestamp_ns: int,
                               evidence: Optional[str] = "snd_pcm_delay") -> bool:
        with self._condition:
            speech = self._speeches.get(speech_id)
            if speech is None or speech.state in _TERMINAL_STATES:
                return False
            if generation != self._generation or generation != speech.generation:
                return False
            observed_consumed = min(
                max(0, int(consumed_frames)), speech.submitted_frames)
            speech.consumed_frames = max(
                speech.consumed_frames, observed_consumed)
            if evidence in {"snd_pcm_delay", "snd_pcm_drain"}:
                speech.device_progress_trusted = True
                speech.trusted_consumed_frames = max(
                    speech.trusted_consumed_frames, observed_consumed)
            if speech.consumed_frames > 0 and speech.playback_started_at_ns is None:
                speech.playback_started_at_ns = int(remote_timestamp_ns)
            if (speech.eof_received
                    and speech.total_output_frames is not None
                    and speech.trusted_consumed_frames >= speech.total_output_frames):
                if (speech.device_progress_trusted
                        and speech.underrun_count == 0
                        and speech.error_code is None):
                    speech.state = "COMPLETED"
                    speech.playback_finished_at_ns = int(remote_timestamp_ns)
                    speech.completion_level = "alsa_device_consumption"
                    event = "device_playback_completed"
                else:
                    speech.state = "PLAYING"
                    event = "device_progress"
            else:
                speech.state = "PLAYING"
                event = "device_progress"
            self._emit(speech, event, remote_timestamp_ns, "q5_audio_host_monotonic",
                       consumed_frames=speech.consumed_frames,
                       total_output_frames=speech.total_output_frames,
                       timestamp_quality=("DEVICE_PROGRESS_OBSERVED"
                                          if speech.device_progress_trusted
                                          else "UNTRUSTED_DEVICE_PROGRESS"),
                       evidence=evidence)
            self._condition.notify_all()
            return True

    def fail(self, speech_id: str, code: str, message: str,
             state: str = "FAILED") -> bool:
        if state not in {"FAILED", "INTERRUPTED"}:
            raise ValueError("terminal failure state must be FAILED or INTERRUPTED")
        with self._condition:
            speech = self._speeches.get(speech_id)
            if speech is None or speech.state in _TERMINAL_STATES:
                return False
            speech.state = state
            speech.error_code = code
            speech.error_message = message
            self._emit(speech, state.lower(), error_code=code,
                       details_message=message)
            self._condition.notify_all()
            return True

    def fail_or_reject_events(self, speech_id: str, code: str,
                              message: str) -> List[dict]:
        """Fail a tracked speech even if its first PCM chunk never arrived.

        Return only newly emitted terminal events for external publication.
        """
        if not speech_id:
            return []
        with self._condition:
            if speech_id in self._rejections:
                return []
            speech = self._speeches.get(speech_id)
            if speech is None:
                self._reject_new_speech(speech_id, code, message)
            elif speech.state in _TERMINAL_STATES:
                return []
            else:
                self.fail(speech_id, code, message)
            return [dict(self._events[-1])]

    def fail_active_events(self, code: str, message: str,
                           state: str = "FAILED") -> List[dict]:
        """Atomically terminate active speeches and return only new events."""
        with self._condition:
            active = [item.speech_id for item in self._speeches.values()
                      if item.state not in _TERMINAL_STATES]
            emitted = []
            for speech_id in active:
                if self.fail(speech_id, code, message, state):
                    emitted.append(dict(self._events[-1]))
            return emitted

    def fail_active(self, code: str, message: str,
                    state: str = "FAILED") -> int:
        return len(self.fail_active_events(code, message, state))

    def backend_reset_events(self, new_generation: int,
                             reason: str) -> List[dict]:
        with self._condition:
            self._generation = int(new_generation)
            return self.fail_active_events("BACKEND_RESET", reason)

    def backend_reset(self, new_generation: int, reason: str) -> None:
        self.backend_reset_events(new_generation, reason)

    def get_status(self, speech_id: str) -> dict:
        with self._condition:
            speech = self._speeches.get(speech_id)
            if speech is None:
                speech = self._rejections.get(speech_id)
            if speech is None:
                return {"speech_id": speech_id, "state": "UNKNOWN", "progress": None}
            result = asdict(speech)
            total = speech.total_output_frames
            result["progress"] = (
                min(1.0, speech.consumed_frames / total) if total else None)
            result["duration_audio_seconds"] = (
                round(total / speech.device_sample_rate_hz, 6)
                if total is not None and speech.device_sample_rate_hz else None)
            result["timestamp_quality"] = (
                "DEVICE_PROGRESS_OBSERVED" if speech.device_progress_trusted
                else "UNTRUSTED_DEVICE_PROGRESS" if speech.consumed_frames
                else "PCM_SUBMITTED" if speech.submitted_frames
                else "QUEUE_OBSERVED")
            result["source_clock_id"] = (
                "q5_audio_host_monotonic" if speech.submitted_frames
                else "driver_host_monotonic")
            return result

    def get_recent_events(self, speech_id: Optional[str] = None,
                          limit: int = 100) -> List[dict]:
        with self._condition:
            events = self._events
            if speech_id is not None:
                events = [item for item in events if item["speech_id"] == speech_id]
            return [dict(item) for item in events[-max(0, int(limit)):]]

    def wait_for_completion(self, speech_id: str, timeout_seconds: float) -> dict:
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        with self._condition:
            while True:
                speech = self._speeches.get(speech_id)
                if speech is None:
                    speech = self._rejections.get(speech_id)
                if speech is None:
                    return {"speech_id": speech_id, "result": "unknown"}
                if speech.state in _TERMINAL_STATES:
                    return {
                        "speech_id": speech_id,
                        "result": speech.state.lower(),
                        "completion_level": speech.completion_level,
                        "playback_finished_at_ns": speech.playback_finished_at_ns,
                        "clock_id": ("q5_audio_host_monotonic"
                                     if speech.playback_finished_at_ns is not None else None),
                        "timestamp_quality": ("DEVICE_PROGRESS_OBSERVED"
                                              if speech.state == "COMPLETED" else None),
                        "error_code": speech.error_code,
                        "error_message": speech.error_message,
                    }
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return {"speech_id": speech_id, "result": "timeout"}
                self._condition.wait(remaining)

    @staticmethod
    def capabilities() -> dict:
        return {
            "tracked_format": "audio/pcm-16k;speech_id=<id>;sequence=<n>;eof=<0|1>",
            "legacy_playback": True,
            "per_speech_tracking_requires_metadata": True,
            "submission_observed": True,
            "device_progress": "snd_pcm_delay",
            "device_completion": "alsa_device_consumption",
            "acoustic_completion": False,
            "remote_clock_id": "q5_audio_host_monotonic",
            "local_clock_id": "driver_host_monotonic",
        }
