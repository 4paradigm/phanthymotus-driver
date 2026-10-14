"""Recording trust boundary and file failure tests, with no hardware."""
import math
from pathlib import Path
import queue
import threading
from unittest.mock import patch

import pytest

from agilex.piper.trajectory import Recorder, read_recording, list_recordings, validate_frame


def frame(t=0):
    return {"type": "frame", "t_ns": t, "target_q": [0.]*12, "target_width_mm": [20., 20.],
            "feedback_q": [0.]*12, "feedback_width_mm": [20., 20.], "feedback_time": [1.]*4,
            "send_ok": [True, True], "send_offset_ns": [1000, 2000]}


def recording(directory):
    recorder = Recorder(directory, {"calibration": "test"})
    for i in range(3):
        assert recorder.append(frame(1_000_000_000 + i*50_000_000))
    result = recorder.stop()
    assert result["complete"], result
    return recorder


def test_round_trip_has_relative_time_real_feedback_and_atomic_name(tmp_path):
    rec = recording(tmp_path)
    result = read_recording(tmp_path, rec.filename, rec.binding)
    assert result["frames"][0]["t_ns"] == 0
    assert result["frames"][-1]["t_ns"] == 100_000_000
    assert result["frames"][0]["feedback_q"] == [0.]*12
    assert not rec.partial.exists()
    assert (tmp_path/rec.filename).stat().st_mode & 0o777 == 0o600
    assert list_recordings(tmp_path) == [{"filename": rec.filename, "bytes": (tmp_path/rec.filename).stat().st_size, "complete": True}]


@pytest.mark.parametrize("name", ["../outside.jsonl", "/tmp/test.jsonl", "bad.jsonl.partial", "a/b.jsonl", "a.jsonl\x00", ""])
def test_file_names_cannot_escape_recording_directory(tmp_path, name):
    with pytest.raises(ValueError):
        read_recording(tmp_path, name, {})


def test_replay_rejects_symlink_and_wrong_binding(tmp_path):
    rec = recording(tmp_path)
    (tmp_path/"link.jsonl").symlink_to(tmp_path/rec.filename)
    with pytest.raises(OSError):
        read_recording(tmp_path, "link.jsonl", rec.binding)
    with pytest.raises(ValueError, match="binding"):
        read_recording(tmp_path, rec.filename, {"calibration": "different"})


@pytest.mark.parametrize("mutation", ["truncate", "checksum", "extra", "duplicate", "count"])
def test_complete_marker_and_entire_file_are_verified(tmp_path, mutation):
    rec = recording(tmp_path)
    path = tmp_path/rec.filename
    raw = path.read_bytes()
    if mutation == "truncate":
        raw = raw.rsplit(b"\n", 2)[0] + b"\n"
    elif mutation == "checksum":
        raw = raw.replace(b'"feedback_q":[0.0', b'"feedback_q":[0.1', 1)
    elif mutation == "extra":
        raw += b'{}\n'
    elif mutation == "duplicate":
        raw = raw.replace(b'"t_ns":0', b'"t_ns":0,"t_ns":0', 1)
    else:
        raw = raw.replace(b'"frames":3', b'"frames":4')
    path.write_bytes(raw)
    with pytest.raises(ValueError):
        read_recording(tmp_path, rec.filename, rec.binding)


@pytest.mark.parametrize("key,value", [("target_q", [float("nan")]*12), ("feedback_q", [0.]*11),
    ("target_width_mm", [True, 0]), ("send_ok", [True, False]), ("send_ok", [1, 1]),
    ("send_offset_ns", [0, 60_000_000]), ("t_ns", True), ("t_ns", -1)])
def test_bad_frames_never_become_replayable(key, value):
    value_frame = frame()
    value_frame[key] = value
    with pytest.raises(ValueError):
        validate_frame(value_frame)


@pytest.mark.parametrize("mutation", ["backwards", "gap", "too_fast", "joint_speed", "gripper_speed"])
def test_timestamp_and_speed_limits(mutation):
    previous, current = frame(), frame(50_000_000)
    if mutation == "backwards": current["t_ns"] = 0
    if mutation == "gap": current["t_ns"] = 400_000_000
    if mutation == "too_fast": current["t_ns"] = 1_000_000
    if mutation == "joint_speed": current["target_q"][0] = math.radians(.5)
    if mutation == "gripper_speed": current["target_width_mm"][0] += 1
    with pytest.raises(ValueError):
        validate_frame(current, previous)


def test_recording_failure_keeps_partial_and_prevents_replay(tmp_path):
    recorder = Recorder(tmp_path, {})
    recorder.append(frame(1_000_000_000))
    recorder.append(frame(1_050_000_000))
    recorder.fail("CAN partial write")
    result = recorder.stop()
    assert result["error"] == "CAN partial write" and not result["complete"]
    assert recorder.partial.exists() and not (tmp_path/recorder.filename).exists()


def test_disk_open_failure_is_visible(tmp_path):
    with patch.object(Path, "open", side_effect=OSError("disk full")):
        with pytest.raises(RuntimeError, match="disk full"):
            Recorder(tmp_path, {})


def test_queue_overflow_does_not_block_motion_producer(tmp_path):
    recorder = Recorder(tmp_path, {})
    with patch.object(recorder.queue, "put_nowait", side_effect=queue.Full):
        assert not recorder.append(frame())
    result = recorder.stop()
    assert result["error"] == "recording_queue_full"
    assert not result["complete"]


def test_limit_and_single_frame_remain_partial(tmp_path):
    recorder = Recorder(tmp_path, {})
    recorder.append(frame())
    assert recorder.stop()["error"] == "recording_needs_two_frames"
    recorder = Recorder(tmp_path, {})
    with patch("agilex.piper.trajectory.MAX_FRAMES", 1):
        recorder.append(frame())
        recorder.append(frame(50_000_000))
        assert recorder.stop()["error"] == "recording_limit_reached"


def test_simultaneous_stop_and_append_never_write_after_complete(tmp_path):
    recorder = Recorder(tmp_path, {})
    for i in range(5): recorder.append(frame(i*50_000_000))
    thread = threading.Thread(target=recorder.stop)
    thread.start()
    thread.join(2)
    assert not thread.is_alive()
    assert not recorder.append(frame(250_000_000))
    result = read_recording(tmp_path, recorder.filename, {})
    assert len(result["frames"]) == 5
