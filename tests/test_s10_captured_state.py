"""Parse the previously captured real messages; this remains an offline test."""
import json
from pathlib import Path

from deep_robotics.s10_pro.device import build_plugins


class CapturedBridge:
    def __init__(self):
        self.samples = json.loads((Path(__file__).parent / "fixtures/s10_pro_state.json").read_text())["samples"]

    def snapshot(self, key):
        topic = {"battery": "/BATTERY_DATA", "motion_info": "/MOTION_INFO", "joints": "/JOINTS_DATA_10HZ",
                 "imu": "/IMU", "odometry": "/ODOM"}.get(key)
        return {"data": self.samples.get(topic), "received_at_ms": 1, "age_s": 0.1,
                "fresh": topic in self.samples, "sequence": 1, "binary": None}

    def status(self):
        return {"reader_running": False, "last_error": None}


def test_captured_sixteen_joints_and_battery_keep_unknown_units_and_raw_values():
    cards = {c.name: c for c in build_plugins({}, "offline_fixture", CapturedBridge(), lambda *args: None)}
    state, _ = cards["state"]._payload()
    battery = state["data"]["battery"]["data"][0]
    assert battery["battery_level"] == 59
    assert battery["voltage"] == 7501 and battery["current"] == -49
    assert state["units"]["battery"]["voltage"] is None
    assert not state["capabilities"]["motion"]["supported"]
    joints, _ = cards["joints"]._payload()
    assert joints["joint_count"] == 16
    assert joints["joint_labels"] == [str(n) for n in range(16)]
    assert joints["data"]["data"]["joints_data"][0]["position"] == 0.06772799044847488
    # A cached data fixture must not make an unstarted card claim connected.
    assert not cards["joints"].status()["connected"]


def test_captured_empty_frames_and_different_timestamp_scales_are_preserved():
    cards = {c.name: c for c in build_plugins({}, "offline_fixture", CapturedBridge(), lambda *args: None)}
    imu, _ = cards["imu"]._payload()
    odometry, _ = cards["odometry"]._payload()
    assert imu["data"]["header"]["frame_id"] == ""
    assert odometry["data"]["child_frame_id"] == ""
    assert odometry["data"]["header"]["frame_id"] == "map"
    state, _ = cards["state"]._payload()
    assert state["source"]["battery"]["stamp"]["sec"] == 1501
    assert imu["source"]["imu"]["stamp"]["sec"] == 1792647441
