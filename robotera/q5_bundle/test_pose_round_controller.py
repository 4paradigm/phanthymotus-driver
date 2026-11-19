from __future__ import annotations

import json
import threading

import pytest

from pose_recorder import PoseRecorder, PoseRoundController


class FakeClock:
    def __init__(self, now_ns=0):
        self._now_ns = now_ns
        self._lock = threading.Lock()

    def __call__(self):
        with self._lock:
            return self._now_ns

    def set(self, value):
        with self._lock:
            self._now_ns = value


def pose(person_id, source_timestamp):
    return json.dumps({
        "timestamp": source_timestamp,
        "persons": [{"id": person_id, "posture": "standing"}],
    })


TRUSTED_COMPLETION = {
    "speech_id": "speech-1",
    "result": "completed",
    "completion_level": "alsa_device_consumption",
    "playback_finished_at_ns": 999_000_000_000,
    "timestamp_quality": "DEVICE_PROGRESS_OBSERVED",
    "error_code": None,
    "error_message": None,
}


def test_pose_continues_during_blocked_tts_and_three_second_post_window():
    clock = FakeClock(0)
    recorder = PoseRecorder(max_frames=20, clock_ns=clock)
    playback_waiting = threading.Event()
    release_playback = threading.Event()
    post_waiting = threading.Event()
    release_post = threading.Event()
    result = {}

    def submit_tts(instruction):
        assert instruction == "raise your hand"
        assert recorder.status()["active_round"] == "round-1"
        return "speech-1"

    def wait_for_playback(speech_id, timeout_seconds):
        assert speech_id == "speech-1"
        assert timeout_seconds == 8
        playback_waiting.set()
        assert release_playback.wait(1)
        return dict(TRUSTED_COMPLETION)

    def wait_post_window(seconds):
        assert seconds == 3.0
        post_waiting.set()
        assert release_post.wait(1)

    controller = PoseRoundController(
        recorder, submit_tts, wait_for_playback,
        clock_ns=clock, wait_post_window=wait_post_window)

    thread = threading.Thread(target=lambda: result.update(
        controller.run_round("round-1", "raise your hand", 8)))
    thread.start()
    assert playback_waiting.wait(1)

    assert recorder.ingest(pose("speaking-1", -5000), 100, 110)
    assert recorder.ingest(pose("speaking-2", 99_000_000), 200, 210)
    assert recorder.status()["buffered_frames"] == 2

    clock.set(1_000_000_000)
    release_playback.set()
    assert post_waiting.wait(1)
    assert recorder.ingest(pose("post-1", 1), 1_500_000_000, 1_500_000_010)
    assert recorder.ingest(pose("post-boundary", 2), 4_000_000_000, 4_000_000_010)

    clock.set(4_000_000_000)
    release_post.set()
    thread.join(1)
    assert not thread.is_alive()

    assert result["outcome"] == "COMPLETED"
    assert result["completion_observed_monotonic_ns"] == 1_000_000_000
    assert result["ended_monotonic_ns"] == 4_000_000_000
    assert [frame["persons"][0]["id"] for frame in result["frames"]] == [
        "speaking-1", "speaking-2", "post-1", "post-boundary"]


@pytest.mark.parametrize(("playback_result", "expected"), [
    ({"result": "failed", "error_code": "ALSA_UNDERRUN"}, "PLAYBACK_FAILED"),
    ({"result": "interrupted", "error_code": "CANCELLED"}, "MANUAL_TERMINATED"),
    ({"result": "timeout"}, "TIMEOUT"),
    ({"result": "completed", "completion_level": "alsa_device_consumption",
      "timestamp_quality": "DEVICE_PROGRESS_OBSERVED",
      "playback_finished_at_ns": None}, "UNTRUSTED_COMPLETION"),
    ({"result": "unknown"}, "UNTRUSTED_COMPLETION"),
])
def test_non_completed_playback_results_freeze_distinct_error_outcomes(
        playback_result, expected):
    clock = FakeClock(100)
    recorder = PoseRecorder(clock_ns=clock)
    post_waits = []
    controller = PoseRoundController(
        recorder, lambda _text: "speech-1",
        lambda _speech_id, _timeout: dict(playback_result),
        clock_ns=clock, wait_post_window=post_waits.append)

    result = controller.run_round("round-error", "instruction", 5)

    assert result["outcome"] == expected
    assert result["ended_monotonic_ns"] == 100
    assert post_waits == []
    assert recorder.status()["active_round"] is None


def test_tts_submit_exception_freezes_submit_failed_without_fake_completion():
    clock = FakeClock(200)
    recorder = PoseRecorder(clock_ns=clock)

    def fail_submit(_instruction):
        raise RuntimeError("mock submit failed")

    controller = PoseRoundController(
        recorder, fail_submit, lambda *_args: TRUSTED_COMPLETION,
        clock_ns=clock, wait_post_window=lambda _seconds: None)

    result = controller.run_round("submit-error", "instruction", 5)

    assert result["outcome"] == "SUBMIT_FAILED"
    assert result["playback_result"]["error_message"] == "mock submit failed"


def test_wait_timeout_exception_freezes_timeout():
    clock = FakeClock(300)
    recorder = PoseRecorder(clock_ns=clock)

    def timeout(*_args):
        raise TimeoutError("mock wait expired")

    controller = PoseRoundController(
        recorder, lambda _text: "speech-1", timeout,
        clock_ns=clock, wait_post_window=lambda _seconds: None)

    assert controller.run_round("wait-timeout", "instruction", 5)["outcome"] == "TIMEOUT"


def test_ten_continuous_rounds_remain_isolated_and_exportable():
    clock = FakeClock(1_000)
    recorder = PoseRecorder(max_frames=100, max_rounds=10,
                            max_frames_per_round=10, clock_ns=clock)
    active = {"round_id": None}

    def submit_tts(_instruction):
        round_id = recorder.status()["active_round"]
        active["round_id"] = round_id
        now = clock()
        recorder.ingest(pose(f"{round_id}-speaking", -now), now + 1, now + 2)
        clock.set(now + 10)
        return f"speech-{round_id}"

    def wait_for_playback(_speech_id, _timeout):
        return dict(TRUSTED_COMPLETION)

    def post_window(seconds):
        assert seconds == 3.0
        observed = clock()
        end = observed + 3_000_000_000
        recorder.ingest(pose(f"{active['round_id']}-post", 9),
                        end, end)
        clock.set(end)

    controller = PoseRoundController(
        recorder, submit_tts, wait_for_playback,
        clock_ns=clock, wait_post_window=post_window)

    for index in range(10):
        round_id = f"round-{index}"
        result = controller.run_round(round_id, f"instruction-{index}", 5)
        assert result["outcome"] == "COMPLETED"
        clock.set(clock() + 100)

    assert recorder.status()["frozen_rounds"] == 10
    for index in range(10):
        round_id = f"round-{index}"
        assert [frame["persons"][0]["id"]
                for frame in recorder.export_round(round_id)["frames"]] == [
            f"{round_id}-speaking", f"{round_id}-post"]
