"""Bounded, ROS-independent Pose frame and round recording."""

from __future__ import annotations

from collections import OrderedDict, deque
from copy import deepcopy
from dataclasses import dataclass
import json
import math
import threading
import time
from typing import Callable, Deque, Dict, List, Optional, Tuple


@dataclass(frozen=True)
class PoseFrame:
    raw_json: str
    source_timestamp: Optional[float]
    bridge_received_monotonic_ns: int
    driver_received_monotonic_ns: int
    persons: Tuple[dict, ...]

    def as_dict(self) -> dict:
        return {
            "raw_json": self.raw_json,
            "source_timestamp": self.source_timestamp,
            "bridge_received_monotonic_ns": self.bridge_received_monotonic_ns,
            "driver_received_monotonic_ns": self.driver_received_monotonic_ns,
            "persons": deepcopy(list(self.persons)),
        }


@dataclass(frozen=True)
class RoundSnapshot:
    round_id: str
    outcome: str
    started_monotonic_ns: int
    ended_monotonic_ns: int
    frames: Tuple[PoseFrame, ...]
    complete: bool
    playback_result: Optional[dict] = None
    completion_observed_monotonic_ns: Optional[int] = None

    def as_dict(self) -> dict:
        return {
            "round_id": self.round_id,
            "outcome": self.outcome,
            "started_monotonic_ns": self.started_monotonic_ns,
            "ended_monotonic_ns": self.ended_monotonic_ns,
            "complete": self.complete,
            "playback_result": deepcopy(self.playback_result),
            "completion_observed_monotonic_ns": (
                self.completion_observed_monotonic_ns),
            "frames": [frame.as_dict() for frame in self.frames],
        }


class PoseRecorder:
    """Thread-safe bounded recorder keyed by Bridge monotonic receive time."""

    _OUTCOMES = {
        "COMPLETED", "PLAYBACK_FAILED", "TIMEOUT", "MANUAL_TERMINATED",
        "SUBMIT_FAILED", "UNTRUSTED_COMPLETION",
    }

    def __init__(self, max_frames: int = 600, max_rounds: int = 20,
                 max_frames_per_round: int = 600,
                 clock_ns: Callable[[], int] = time.monotonic_ns):
        self._max_frames = max(1, int(max_frames))
        self._max_rounds = max(1, int(max_rounds))
        self._max_frames_per_round = max(1, int(max_frames_per_round))
        self._clock_ns = clock_ns
        self._lock = threading.RLock()
        self._frames: Deque[PoseFrame] = deque(maxlen=self._max_frames)
        self._rounds: "OrderedDict[str, RoundSnapshot]" = OrderedDict()
        self._active_round = None
        self._counters: Dict[str, int] = {
            "recorder_cache_evictions": 0,
            "frozen_round_evictions": 0,
            "snapshot_frame_truncations": 0,
            "invalid_json": 0,
            "invalid_schema": 0,
            "invalid_source_timestamps": 0,
            "invalid_persons": 0,
            "monotonic_anomalies": 0,
        }

    @staticmethod
    def _source_timestamp(value) -> Optional[float]:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        timestamp = float(value)
        return timestamp if math.isfinite(timestamp) else None

    @staticmethod
    def _valid_person_id(value) -> bool:
        return (not isinstance(value, bool)
                and isinstance(value, (str, int, float))
                and (not isinstance(value, float) or math.isfinite(value))
                and str(value) != "")

    def ingest(self, raw_json: str, bridge_received_monotonic_ns: int,
               driver_received_monotonic_ns: Optional[int] = None) -> bool:
        try:
            payload = json.loads(raw_json)
        except (TypeError, ValueError, json.JSONDecodeError):
            with self._lock:
                self._counters["invalid_json"] += 1
            return False
        if not isinstance(payload, dict) or not isinstance(payload.get("persons"), list):
            with self._lock:
                self._counters["invalid_schema"] += 1
            return False

        source_timestamp = self._source_timestamp(payload.get("timestamp"))
        bridge_ns = int(bridge_received_monotonic_ns)
        driver_ns = int(self._clock_ns() if driver_received_monotonic_ns is None
                        else driver_received_monotonic_ns)
        persons: List[dict] = []
        invalid_persons = 0
        for person in payload["persons"]:
            if (not isinstance(person, dict)
                    or not self._valid_person_id(person.get("id"))):
                invalid_persons += 1
                continue
            persons.append(deepcopy(person))

        frame = PoseFrame(str(raw_json), source_timestamp, bridge_ns, driver_ns,
                          tuple(persons))
        with self._lock:
            if source_timestamp is None:
                self._counters["invalid_source_timestamps"] += 1
            self._counters["invalid_persons"] += invalid_persons
            if driver_ns < bridge_ns:
                self._counters["monotonic_anomalies"] += 1
            if len(self._frames) >= self._max_frames:
                self._counters["recorder_cache_evictions"] += 1
                if self._active_round is not None:
                    self._active_round["evicted_during_round"] = True
            self._frames.append(frame)
        return True

    def recent_frames(self, limit: Optional[int] = None) -> List[dict]:
        with self._lock:
            frames = list(self._frames)
            if limit is not None:
                frames = frames[-max(0, int(limit)):]
            return [frame.as_dict() for frame in frames]

    def start_round(self, round_id: str,
                    started_monotonic_ns: Optional[int] = None) -> dict:
        round_id = str(round_id)
        if not round_id:
            raise ValueError("round_id is required")
        started_ns = int(self._clock_ns() if started_monotonic_ns is None
                         else started_monotonic_ns)
        with self._lock:
            if self._active_round is not None:
                raise RuntimeError(
                    f"round {self._active_round['round_id']} is already active")
            if round_id in self._rounds:
                raise ValueError(f"round {round_id} already exists")
            self._active_round = {
                "round_id": round_id,
                "started_monotonic_ns": started_ns,
                "evicted_during_round": False,
            }
            return {"round_id": round_id, "state": "RECORDING",
                    "started_monotonic_ns": started_ns}

    @staticmethod
    def _trusted_completion(playback_result: Optional[dict]) -> bool:
        return bool(
            isinstance(playback_result, dict)
            and playback_result.get("result") == "completed"
            and playback_result.get("completion_level") == "alsa_device_consumption"
            and playback_result.get("timestamp_quality") == "DEVICE_PROGRESS_OBSERVED"
            and playback_result.get("playback_finished_at_ns") is not None)

    def finish_round(self, round_id: str, outcome: str,
                     ended_monotonic_ns: Optional[int] = None,
                     playback_result: Optional[dict] = None,
                     completion_observed_monotonic_ns: Optional[int] = None) -> dict:
        outcome = str(outcome)
        if outcome not in self._OUTCOMES:
            raise ValueError(f"unsupported round outcome: {outcome}")
        ended_ns = int(self._clock_ns() if ended_monotonic_ns is None
                       else ended_monotonic_ns)
        if outcome == "COMPLETED":
            if not self._trusted_completion(playback_result):
                raise ValueError("COMPLETED requires trusted playback evidence")
            if completion_observed_monotonic_ns is None:
                raise ValueError("COMPLETED requires local completion observation")
            observed_ns = int(completion_observed_monotonic_ns)
            if ended_ns < observed_ns + 3_000_000_000:
                raise ValueError("COMPLETED requires the full three-second window")

        with self._lock:
            active = self._active_round
            if active is None or active["round_id"] != str(round_id):
                raise ValueError(f"round {round_id} is not active")
            started_ns = active["started_monotonic_ns"]
            if ended_ns < started_ns:
                raise ValueError("round end precedes round start")
            frames = [
                frame for frame in self._frames
                if started_ns <= frame.bridge_received_monotonic_ns <= ended_ns
            ]
            complete = not active["evicted_during_round"]
            if len(frames) > self._max_frames_per_round:
                frames = frames[-self._max_frames_per_round:]
                self._counters["snapshot_frame_truncations"] += 1
                complete = False
            snapshot = RoundSnapshot(
                str(round_id), outcome, started_ns, ended_ns, tuple(frames),
                complete, deepcopy(playback_result),
                (int(completion_observed_monotonic_ns)
                 if completion_observed_monotonic_ns is not None else None))
            if len(self._rounds) >= self._max_rounds:
                self._rounds.popitem(last=False)
                self._counters["frozen_round_evictions"] += 1
            self._rounds[str(round_id)] = snapshot
            self._active_round = None
            return snapshot.as_dict()

    def export_round(self, round_id: str) -> dict:
        with self._lock:
            snapshot = self._rounds.get(str(round_id))
            if snapshot is None:
                raise KeyError(str(round_id))
            return snapshot.as_dict()

    def status(self, bridge_transport_drops: int = 0) -> dict:
        with self._lock:
            return {
                "active_round": (self._active_round["round_id"]
                                 if self._active_round is not None else None),
                "buffered_frames": len(self._frames),
                "max_frames": self._max_frames,
                "frozen_rounds": len(self._rounds),
                "max_rounds": self._max_rounds,
                "max_frames_per_round": self._max_frames_per_round,
                "bridge_transport_drops": max(0, int(bridge_transport_drops)),
                **self._counters,
            }


class PoseRoundController:
    """Orchestrate a round while Pose ingestion runs on an independent path."""

    def __init__(self, recorder: PoseRecorder, submit_tts, wait_for_playback,
                 clock_ns: Callable[[], int] = time.monotonic_ns,
                 wait_post_window=time.sleep):
        self._recorder = recorder
        self._submit_tts = submit_tts
        self._wait_for_playback = wait_for_playback
        self._clock_ns = clock_ns
        self._wait_post_window = wait_post_window

    def run_round(self, round_id: str, instruction: str,
                  timeout_seconds: float) -> dict:
        self._recorder.start_round(round_id, int(self._clock_ns()))
        try:
            speech_id = self._submit_tts(instruction)
        except Exception as exc:
            return self._finish_error(
                round_id, "SUBMIT_FAILED", "TTS_SUBMIT_FAILED", exc)
        try:
            playback_result = self._wait_for_playback(
                speech_id, float(timeout_seconds))
        except TimeoutError as exc:
            return self._finish_error(
                round_id, "TIMEOUT", "PLAYBACK_TIMEOUT", exc)
        except Exception as exc:
            return self._finish_error(
                round_id, "PLAYBACK_FAILED", "PLAYBACK_WAIT_FAILED", exc)

        if not self._recorder._trusted_completion(playback_result):
            result = playback_result.get("result") if isinstance(
                playback_result, dict) else None
            outcome = {
                "failed": "PLAYBACK_FAILED",
                "interrupted": "MANUAL_TERMINATED",
                "timeout": "TIMEOUT",
            }.get(result, "UNTRUSTED_COMPLETION")
            return self._recorder.finish_round(
                round_id, outcome, int(self._clock_ns()), playback_result)
        observed_ns = int(self._clock_ns())
        self._wait_post_window(3.0)
        return self._recorder.finish_round(
            round_id, "COMPLETED", int(self._clock_ns()), playback_result,
            observed_ns)

    def _finish_error(self, round_id: str, outcome: str, error_code: str,
                      error: Exception) -> dict:
        return self._recorder.finish_round(
            round_id, outcome, int(self._clock_ns()), {
                "result": "timeout" if outcome == "TIMEOUT" else "failed",
                "error_code": error_code,
                "error_message": str(error),
            })
