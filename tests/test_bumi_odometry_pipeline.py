"""Quality gates must prevent stale predictions being counted as displacement."""
import json
import csv
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiments.bumi_odometry.preflight import inspect_device
from experiments.bumi_odometry.quality import TrackingQuality, TrialCSV
from experiments.bumi_odometry.configuration import load_calibration, pipeline_settings, validate_mode


def test_waits_for_both_sources_and_uses_ros_time_for_replay_pause():
    q = TrackingQuality()
    assert q.state(0) == "initializing"
    assert q.update("visual", 100, 100, "a", True) == "initializing"
    assert q.update("imu", 101, 101, "a", True) == "initializing"
    assert q.update("filtered", 102, 102, "a", True) == "tracking"
    assert q.state(102) == "tracking"  # Paused /clock doesn't age the data.
    assert q.state(500_000_103) == "lost"
    assert q.update("visual", 500_000_104, 500_000_104, "a", True) == "lost"


@pytest.mark.parametrize("kind,stamp,received,segment,valid", [
    ("visual", 100, 102, "a", True),  # duplicate source time
    ("imu", 99, 102, "a", True),
    ("visual", 102, 102, "b", True),
    ("visual", 102, 102, "a", False),
    ("imu", 102, 600_000_103, "a", True),  # fresh receipt, stale source
    ("imu", 600_000_103, 102, "a", True),  # wrong clock domain
])
def test_faults_latch_until_new_trial(kind, stamp, received, segment, valid):
    q = TrackingQuality()
    q.update("visual", 100, 100, "a", True)
    q.update("imu", 100, 100, "a", True)
    q.update("filtered", 100, 100, "a", True)
    assert q.update(kind, stamp, received, segment, valid) == "lost"
    assert q.update("visual", 200, 200, "a", True) == "lost"


def test_clock_backwards_and_missing_second_stream_cannot_track():
    q = TrackingQuality()
    q.update("visual", 1_000_000_000, 1_000_000_000, "a", True)
    assert q.state(2_000_000_000) == "lost"
    q = TrackingQuality()
    q.update("visual", 100, 100, "a", True)
    q.update("imu", 100, 100, "a", True)
    q.update("filtered", 100, 100, "a", True)
    assert q.state(99) == "lost"


def sdk(streams):
    # RealSense enumeration/profile reads are external hardware calls.
    class Profile:
        def __init__(self, stream): self.stream = stream
        def stream_type(self): return self.stream
        def fps(self): return 200 if self.stream in ("gyro", "accel") else 30
        def format(self): return "format"
        def stream_index(self): return 0
    class Device:
        def get_info(self, key): return "D435i" if key == "name" else "example"
        def query_sensors(self):
            return [SimpleNamespace(get_stream_profiles=lambda: [Profile(s) for s in streams])]
    def forbidden(*args, **kwargs): raise AssertionError("enumeration must not start streaming")
    return SimpleNamespace(context=lambda: SimpleNamespace(query_devices=lambda: [Device()]),
                           camera_info=SimpleNamespace(name="name", serial_number="serial", firmware_version="fw"),
                           stream=SimpleNamespace(color="color", depth="depth", gyro="gyro", accel="accel"),
                           pipeline=forbidden)


def test_preflight_rejects_rgb_depth_only_without_opening_camera():
    result = inspect_device(sdk(["color", "depth"]))
    assert result["ready"] is False
    assert result["devices"][0]["missing_streams"] == ["gyro", "accel"]


def test_preflight_accepts_motion_capabilities_but_not_claims_streaming_verified():
    result = inspect_device(sdk(["color", "depth", "gyro", "accel"]))
    assert result["ready"] is True
    assert result["streaming_verified"] is False


def calibration():
    return dict(fixed_mount=True, translation_m=[0.1, 0, 0.3], quaternion_xyzw=[0, 0, 0, 1],
                initial_base_rpy_rad=[0, 0, 0], calibration_id="measured-001")


@pytest.mark.parametrize("change", [dict(fixed_mount=False), dict(quaternion_xyzw=[0, 0, 0, 0]),
                                   dict(translation_m=[float("nan"), 0, 0]), dict(calibration_id="")])
def test_missing_or_invalid_mount_calibration_prevents_pipeline_start(tmp_path, change):
    data = dict(calibration(), **change)
    p = tmp_path / "calibration.json"
    p.write_text(json.dumps(data))
    with pytest.raises(ValueError): load_calibration(p)


def test_pipeline_keeps_camera_off_and_tf_separate_and_does_not_integrate_acceleration(tmp_path):
    p = tmp_path / "calibration.json"
    p.write_text(json.dumps(calibration()))
    config = pipeline_settings(load_calibration(p))
    assert config["camera"]["camera_name"] == "bumi_odom_camera"
    assert config["visual"]["frame_id"] == "bumi_odom_base_link"
    assert config["visual"]["publish_tf"] is False
    assert config["visual"]["wait_imu_to_init"] is False
    assert config["ekf"]["two_d_mode"] is False
    assert config["ekf"]["imu0_config"] == [False]*9 + [True]*3 + [False]*3
    assert config["ekf"]["odom0_config"] == [True]*6 + [False]*9
    assert config["ekf"]["world_frame"] == "bumi_odom_odom"


def test_replay_cannot_open_camera_or_republish_recorded_mount():
    with pytest.raises(ValueError): validate_mode(True, True, False)
    with pytest.raises(ValueError): validate_mode(False, True, True)
    validate_mode(False, True, False)
    validate_mode(False, False, True)


def test_fault_is_persisted_without_another_filtered_pose(tmp_path):
    path = tmp_path / "trial.csv"
    with path.open("w", newline="") as stream:
        output = TrialCSV(stream, "trial")
        output.pose(1_000_000_000, [1, 0, 0, 0, 0, 0, 1], "tracking")
        output.fault(1_100_000_000, 900_000_000)
        # Read before closing: late diagnostics must be flushed immediately.
        rows = list(csv.DictReader(path.open()))
        assert rows[-1]["status"] == "lost"
        assert rows[-1]["fault_stamp_ns"] == "900000000"


def test_ekf_dropout_latches_loss_while_other_inputs_continue(tmp_path):
    q = TrackingQuality()
    for kind in ("visual", "imu", "filtered"):
        q.update(kind, 1_000_000_000, 1_000_000_000, "trial", True)
    assert q.state(1_000_000_000) == "tracking"
    path = tmp_path / "trial.csv"
    with path.open("w", newline="") as stream:
        output = TrialCSV(stream, "trial")
        output.pose(1_000_000_000, [1, 0, 0, 0, 0, 0, 1], "tracking")
        for now in (1_200_000_000, 1_400_000_000, 1_600_000_000):
            q.update("visual", now, now, "trial", True)
            q.update("imu", now, now, "trial", True)
        assert q.state(1_600_000_000) == "lost"
        output.fault(1_600_000_000, 1_000_000_000)
        assert list(csv.DictReader(path.open()))[-1]["status"] == "lost"
        q.update("filtered", 1_600_000_000, 1_600_000_000, "trial", True)
        assert q.state(1_600_000_000) == "lost"
