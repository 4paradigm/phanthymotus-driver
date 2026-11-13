#!/usr/bin/env python3
"""Bounded, hardware-free stress test for Q5 playback telemetry."""

from __future__ import annotations

import argparse
import json
import random
import time
import tracemalloc

from fake_playback_backend import FakePlaybackBackend
from playback_telemetry import PlaybackTracker, parse_audio_format


def _metadata(speech_id: str, sequence: int, eof: bool):
    return parse_audio_format(
        f"audio/pcm-16k;speech_id={speech_id};sequence={sequence};eof={int(eof)}"
    )


def run_soak(speeches: int = 10_000, chunks_per_speech: int = 10,
             seed: int = 20261020) -> dict:
    speeches = max(1, int(speeches))
    chunks_per_speech = max(1, int(chunks_per_speech))
    randomizer = random.Random(seed)
    tracker = PlaybackTracker(max_events=4096)
    backend = FakePlaybackBackend(tracker)
    stats = {
        "speeches": speeches,
        "chunks": 0,
        "queries": 0,
        "waits": 0,
        "interruptions": 0,
        "restarts": 0,
        "underruns_injected": 0,
        "eagain_injected": 0,
        "duplicates_rejected": 0,
        "failures": 0,
    }
    started = time.monotonic()
    tracemalloc.start()
    for speech_index in range(speeches):
        if speech_index % 100 == 0:
            backend.restart()
            stats["restarts"] += 1
        speech_id = f"soak_{speech_index:05d}"
        total_output = 0
        speech_underrun = False
        for sequence in range(chunks_per_speech):
            meta = _metadata(speech_id, sequence, sequence == chunks_per_speech - 1)
            plan = None
            selector = randomizer.randrange(1000)
            if selector < 10 and speech_index % 10 != 0:
                plan = [100, "EPIPE", 341]
                stats["underruns_injected"] += 1
                speech_underrun = True
            elif selector < 20:
                plan = ["EAGAIN", 441]
                stats["eagain_injected"] += 1
            total_output += backend.push(meta, input_frames=160, write_results=plan)
            stats["chunks"] += 1
            if sequence == 0 and speech_index % 50 == 0:
                if tracker.receive_chunk(meta, input_samples=160) is False:
                    stats["duplicates_rejected"] += 1
                else:
                    stats["failures"] += 1

        if speech_index % 10 == 0:
            if tracker.fail(speech_id, "CANCELLED", "soak interruption", "INTERRUPTED"):
                stats["interruptions"] += 1
            expected_state = "INTERRUPTED"
        elif speech_underrun:
            expected_state = "FAILED"
        else:
            expected_state = "COMPLETED"
        backend.consume(total_output)

        status = tracker.get_status(speech_id)
        stats["queries"] += 1
        if status["state"] != expected_state:
            stats["failures"] += 1
        if speech_index % 10 == 5:
            waited = tracker.wait_for_completion(speech_id, timeout_seconds=0)
            stats["waits"] += 1
            if waited["result"] != expected_state.lower():
                stats["failures"] += 1

    current_bytes, peak_bytes = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    stats.update({
        "duration_seconds": round(time.monotonic() - started, 6),
        "current_memory_bytes": current_bytes,
        "peak_memory_bytes": peak_bytes,
        "seed": seed,
    })
    return stats


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speeches", type=int, default=10_000)
    parser.add_argument("--chunks-per-speech", type=int, default=10)
    parser.add_argument("--seed", type=int, default=20261020)
    args = parser.parse_args()
    result = run_soak(args.speeches, args.chunks_per_speech, args.seed)
    print(json.dumps(result, sort_keys=True))
    return 1 if result["failures"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
