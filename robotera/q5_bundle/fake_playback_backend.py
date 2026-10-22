"""Deterministic ALSA-like backend for playback telemetry tests."""

from __future__ import annotations

import time
from typing import Iterable, Optional

from playback_telemetry import AudioChunkMetadata, PlaybackTracker


class FakePlaybackBackend:
    def __init__(self, tracker: PlaybackTracker, input_rate: int = 16000,
                 output_rate: int = 44100, output_channels: int = 2,
                 max_records: int = 2048, max_write_stalls: int = 250,
                 max_progress_failures: int = 50):
        self.tracker = tracker
        self.input_rate = int(input_rate)
        self.output_rate = int(output_rate)
        self.output_channels = int(output_channels)
        self.generation = 1
        self.submitted_global = 0
        self.consumed_global = 0
        self.write_attempts = 0
        self.eagain_count = 0
        self.max_records = max(1, int(max_records))
        self.max_write_stalls = max(1, int(max_write_stalls))
        self.max_progress_failures = max(1, int(max_progress_failures))
        self._progress_failures = 0
        self._progress_observed = False
        self._resample_remainder = 0
        self._ranges = []
        self._by_id = {}

    @property
    def record_count(self) -> int:
        return len(self._by_id)

    def has_record(self, speech_id: str) -> bool:
        return speech_id in self._by_id

    def _prune_terminal(self) -> None:
        terminal_ids = {
            speech_id for speech_id in self._by_id
            if self.tracker.get_status(speech_id)["state"] in {
                "COMPLETED", "FAILED", "INTERRUPTED"
            }
        }
        if not terminal_ids:
            return
        self._ranges = [record for record in self._ranges
                        if record["speech_id"] not in terminal_ids]
        for speech_id in terminal_ids:
            self._by_id.pop(speech_id, None)

    def _fail_buffered(self, code: str, message: str,
                       current_speech_id: Optional[str] = None) -> None:
        for record in self._ranges:
            speech_id = record["speech_id"]
            status = self.tracker.get_status(speech_id)
            if status["state"] in {"COMPLETED", "FAILED", "INTERRUPTED"}:
                continue
            has_pending_frames = any(end > self.consumed_global
                                     for _start, end in record["segments"])
            if (speech_id == current_speech_id or has_pending_frames
                    or not self._progress_observed):
                if code == "ALSA_UNDERRUN":
                    self.tracker.record_underrun(
                        speech_id, self.generation, time.monotonic_ns())
                else:
                    self.tracker.fail(speech_id, code, message)

    def _output_frames(self, input_frames: int) -> int:
        numerator = max(0, int(input_frames)) * self.output_rate + self._resample_remainder
        frames, self._resample_remainder = divmod(numerator, self.input_rate)
        return frames

    def push(self, metadata: AudioChunkMetadata, input_frames: int,
             write_results: Optional[Iterable[object]] = None) -> int:
        if not self.tracker.receive_chunk(metadata, input_frames):
            return 0
        speech_id = metadata.speech_id
        record = self._by_id.get(speech_id)
        if record is None:
            self._prune_terminal()
            if len(self._by_id) >= self.max_records:
                self.tracker.fail(
                    speech_id, "PLAYBACK_CAPACITY",
                    "playback record capacity is exhausted by active tasks")
                return 0
            record = {"speech_id": speech_id, "start": self.submitted_global,
                      "submitted": 0, "total": None, "generation": self.generation,
                      "segments": []}
            self._by_id[speech_id] = record
            self._ranges.append(record)
        elif self._ranges[-1] is not record:
            raise ValueError("fake backend does not interleave speech frame ranges")
        output_frames = self._output_frames(input_frames)

        plan = iter(write_results or ())
        accepted = 0
        consecutive_stalls = 0
        while accepted < output_frames:
            self.write_attempts += 1
            try:
                result = next(plan)
            except StopIteration:
                result = output_frames - accepted
            if result == "EAGAIN":
                self.eagain_count += 1
                consecutive_stalls += 1
                if consecutive_stalls < self.max_write_stalls:
                    continue
                self._fail_buffered(
                    "ALSA_WRITE_STALLED", "ALSA writes made no bounded progress",
                    speech_id)
                return accepted
            if result == "EPIPE":
                self._fail_buffered(
                    "ALSA_UNDERRUN", "ALSA underrun discarded buffered frames",
                    speech_id)
                self.consumed_global = self.submitted_global
                return accepted
            count = min(output_frames - accepted, max(0, int(result)))
            if count == 0:
                consecutive_stalls += 1
                if consecutive_stalls < self.max_write_stalls:
                    continue
                self._fail_buffered(
                    "ALSA_WRITE_STALLED", "ALSA writes made no bounded progress",
                    speech_id)
                return accepted
            consecutive_stalls = 0
            accepted += count
            record["submitted"] += count
            start = self.submitted_global
            self.submitted_global += count
            record["segments"].append((start, self.submitted_global))
            self.tracker.record_submitted(
                speech_id, submitted_frames=record["submitted"],
                generation=self.generation, remote_timestamp_ns=time.monotonic_ns(),
                device_sample_rate_hz=self.output_rate)

        if metadata.eof:
            record["total"] = record["submitted"]
        self.tracker.record_submitted(
            speech_id,
            submitted_frames=record["submitted"],
            total_output_frames=record["total"],
            generation=self.generation,
            remote_timestamp_ns=time.monotonic_ns(),
            device_sample_rate_hz=self.output_rate,
        )
        return accepted

    def consume(self, frames: int) -> int:
        previous = self.consumed_global
        self.consumed_global = min(
            self.submitted_global,
            self.consumed_global + max(0, int(frames)),
        )
        self._progress_failures = 0
        self._progress_observed = True
        for record in self._ranges:
            if record["generation"] != self.generation:
                continue
            consumed = min(
                record["submitted"],
                max(0, self.consumed_global - record["start"]),
            )
            if consumed or record["total"] == 0:
                self.tracker.record_device_progress(
                    record["speech_id"], consumed, self.generation, time.monotonic_ns(),
                    evidence="snd_pcm_delay")
        return self.consumed_global - previous

    def progress_query_failed(self) -> None:
        self._progress_failures += 1
        if self._progress_failures >= self.max_progress_failures:
            self.tracker.fail_active(
                "ALSA_DELAY_FAILED", "snd_pcm_delay repeatedly failed")

    def drain(self, success: bool) -> None:
        if not success:
            self.tracker.fail_active("ALSA_DRAIN_FAILED", "snd_pcm_drain failed")
            return
        self.consumed_global = self.submitted_global
        self._progress_observed = True
        for record in self._ranges:
            if self.tracker.get_status(record["speech_id"])["state"] in {
                    "COMPLETED", "FAILED", "INTERRUPTED"}:
                continue
            self.tracker.record_device_progress(
                record["speech_id"], record["submitted"], self.generation,
                time.monotonic_ns(), evidence="snd_pcm_drain")

    def restart(self) -> None:
        self.generation += 1
        self.tracker.backend_reset(self.generation, "fake ALSA device restarted")
        self.submitted_global = 0
        self.consumed_global = 0
        self._resample_remainder = 0
        self._progress_failures = 0
        self._progress_observed = False
        self._ranges = []
        self._by_id = {}
