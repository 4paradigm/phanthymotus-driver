"""Offline Go1 swing trajectory and world-frame contract checks."""
import contextlib
import io
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "unitree" / "go1"))

from sensors import SwingTrajectoryPlugin, _swing_vector, _swing_world_point  # noqa: E402


class Client:
    def snapshot(self):
        return {"fresh": False}


def sample(t, force=100, foot=None, position=None, quaternion=None):
    return {
        "fresh": True, "mode": 2, "control_level": "HIGHLEVEL",
        "foot_force": [force, 100, 100, 100],
        "foot_pos": [foot or {"x": 0.2, "y": -0.1, "z": -0.25}] +
                    [{"x": 0.2, "y": 0.1, "z": -0.25}] * 3,
        "position": position if position is not None else [1.0, 2.0, 0.5],
        "imu": {"quaternion_wxyz": quaternion or [1, 0, 0, 0]},
    }


def test_world_rotation_and_translation():
    q = [math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)]
    assert _swing_world_point([1, 0, 0], [10, 20, 3], q) == [10, 21, 3]
    assert _swing_world_point([1, 0, 0], None, q) is None
    assert _swing_world_point([1, 0, 0], [0, 0, 0], [0, 0, 0, 0]) is None


def test_swing_lifecycle_both_frames_and_duplicate_rejection():
    plugin = SwingTrajectoryPlugin({"phase_confirm_samples": 2}, "test", None, Client())
    t = time.monotonic()
    for force in (100, 100, 0, 0):
        plugin.process_sample(sample(t, force=force), now=t)
        t += 0.05
    state = plugin._build()["feet"]["FR"]
    assert state["phase"] == "swing" and len(state["active"]) == 1
    assert state["active"][0]["body_xyz_m"] == [0.2, -0.1, -0.25]
    assert state["active"][0]["world_xyz_m"] == [1.2, 1.9, 0.25]
    plugin.process_sample(sample(t - 0.05, force=0), now=t)
    assert len(plugin._build()["feet"]["FR"]["active"]) == 1
    plugin.process_sample(sample(t, force=0, foot={"x": 0.3, "y": -0.1, "z": -0.2}), now=t)
    t += 0.05
    assert len(plugin._build()["feet"]["FR"]["active"]) == 2
    for force in (100, 100):
        plugin.process_sample(sample(t, force=force), now=t)
        t += 0.05
    state = plugin._build()["feet"]["FR"]
    assert state["phase"] == "stance" and not state["active"]
    assert len(state["last_completed"]) == 3  # first contact-confirm frame is retained


def test_missing_world_pose_keeps_body_and_unchanged_gap_does_not_join_swings():
    plugin = SwingTrajectoryPlugin({"phase_confirm_samples": 1, "max_sample_age_s": 0.5}, "test", None, Client())
    t = time.monotonic()
    plugin.process_sample(sample(t, force=0, quaternion=[0, 0, 0, 0]), now=t)
    data = plugin._build()
    assert data["status"] == "world_pose_unavailable"
    assert data["feet"]["FR"]["active"][0]["world_xyz_m"] is None
    assert data["feet"]["FR"]["active"][0]["body_xyz_m"] is not None
    plugin.process_sample(sample(t + 0.1, force=0, quaternion=[0, 0, 0, 0]),
                          now=t + 1.0)  # identical snapshot
    assert len(plugin._feet["FR"]["active"]) == 1
    plugin.process_sample(sample(t + 1.0, force=0, foot={"x": 0.3, "y": -0.1, "z": -0.2}),
                          now=t + 1.0)  # observed change after gap resets segment
    assert len(plugin._feet["FR"]["active"]) == 1
    plugin.process_sample({"fresh": False}, now=t + 1.1)
    assert plugin._last_status == "source_unavailable"


def test_invalid_available_pose_and_foot_values_are_rejected_by_card():
    assert _swing_vector([1, float('nan'), 3]) is None
    assert _swing_world_point([1, 0, 0], [0, float('inf'), 0], [1, 0, 0, 0]) is None
    assert _swing_world_point([1, 0, 0], [0, 0, 0], [0, 0, 0, 0]) is None


def test_damp_mode_does_not_create_false_swing_from_low_foot_force():
    plugin = SwingTrajectoryPlugin({"phase_confirm_samples": 2}, "test", None, Client())
    t = time.monotonic()
    snap = sample(t, force=3)
    snap["mode"] = 7
    plugin.process_sample(snap, now=t)
    plugin.process_sample(snap, now=t + 0.05)
    data = plugin._build()
    assert data["status"] == "not_walking" and data["fresh"] is False
    assert all(not foot["active"] for foot in data["feet"].values())


def test_existing_sdk_receive_timestamp_tracks_identical_new_packets():
    plugin = SwingTrajectoryPlugin({"phase_confirm_samples": 2}, "test", None, Client())
    t = time.monotonic()
    for index, force in enumerate((100, 100, 0, 0, 0)):
        snap = sample(t, force=force)
        snap["received_monotonic_s"] = t + 0.05 * index
        plugin.process_sample(snap, now=t + 0.05 * index)
    data = plugin._build()
    assert data["sample_time_basis"] == "sdk_receive_monotonic"
    assert data["source_freshness_confirmed"] is True
    assert len(data["feet"]["FR"]["active"]) == 2
    assert data["feet"]["FR"]["active"][1]["t_from_liftoff_s"] == 0.05
    plugin.process_sample(snap, now=t + 0.25)  # same source timestamp
    assert len(plugin._feet["FR"]["active"]) == 2
    stale = sample(t, force=0)
    stale["received_monotonic_s"] = t + 0.21
    plugin.process_sample(stale, now=t + 1.0)
    assert plugin._last_status == "source_stale"
    assert len(plugin._feet["FR"]["active"]) == 2


def test_wiring_and_no_hardware_output():
    root = Path(__file__).resolve().parents[1] / "unitree" / "go1"
    plugin = SwingTrajectoryPlugin({}, "test", None, Client())
    assert plugin.get_tool()["name"] == "swing_trajectory"
    assert plugin.dispatch("read", {})["data"]["fresh"] is False
    assert plugin.dispatch("read", {})["data"]["control_level"] == "HIGHLEVEL"
    assert plugin.dispatch("read", {})["data"]["source_freshness_confirmed"] is False
    for file, token in (("main.py", "make_swing_trajectory"),
                        ("config.yaml", "swing_trajectory:"),
                        ("driver.yaml", "name: swing_trajectory"),
                        ("Dockerfile", "COPY sensors.py"),
                        ("sensors.py", "def make_swing_trajectory")):
        assert token in (root / file).read_text()


def test_lifecycle_idempotent_and_restartable():
    plugin = SwingTrajectoryPlugin({"sample_hz": 50}, "test", None, Client())
    plugin.start()
    first = plugin._thread
    plugin.start()
    assert plugin._thread is first
    plugin.stop()
    assert plugin._thread is None
    plugin.start()
    assert plugin._thread is not first
    plugin.stop()


def test_persistent_sample_error_does_not_flood_logs():
    class FailingClient:
        def snapshot(self):
            raise ValueError("bad sample")

    class BoundedStop:
        calls = 0

        def is_set(self):
            self.calls += 1
            return self.calls > 105

        def wait(self, _interval):
            pass

    plugin = SwingTrajectoryPlugin({}, "test", None, FailingClient())
    plugin._stop = BoundedStop()
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        plugin._loop()
    lines = output.getvalue().splitlines()
    assert len(lines) == 2
    assert "(1)" in lines[0] and "(100)" in lines[1]
