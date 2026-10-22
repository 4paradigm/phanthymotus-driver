"""Offline contracts for the five S10 Pro cards; no ROS or robot access."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
import threading
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deep_robotics/s10_pro"))
spec = importlib.util.spec_from_file_location("s10_cards_contract", ROOT / "deep_robotics/s10_pro/device.py")
cards = importlib.util.module_from_spec(spec)
assert spec.loader
spec.loader.exec_module(cards)


class FakeBridge:
    def __init__(self):
        self.lock = threading.Lock()
        self.active = {}
        self.acquires = []
        self.releases = []
        self.values = {}
        self.acquire_entered = threading.Event()
        self.allow_acquire = None
        self.acquire_error = None

    def acquire(self, card_name, sources, options):
        self.acquire_entered.set()
        if self.allow_acquire is not None:
            self.allow_acquire.wait(2)
        if self.acquire_error:
            raise RuntimeError(self.acquire_error)
        with self.lock:
            self.acquires.append((card_name, sources, options))
            self.active[card_name] = list(sources)

    def release(self, card_name):
        with self.lock:
            self.releases.append(card_name)
            self.active.pop(card_name, None)

    def snapshot(self, source):
        with self.lock:
            return copy.deepcopy(self.values.get(source, {"data": None, "received_at_ms": None,
                "age_s": None, "fresh": False, "error": None, "sequence": 0, "binary": None}))

    def status(self):
        with self.lock:
            return {"reader_running": bool(self.active), "last_error": None}

    def put(self, source, data, *, age=0.01, sequence=1, fresh=True, binary=None):
        with self.lock:
            self.values[source] = {"data": copy.deepcopy(data), "received_at_ms": 123456789,
                "age_s": age, "fresh": fresh, "error": None, "sequence": sequence, "binary": binary}


class Publications:
    def __init__(self):
        self.lock = threading.Lock()
        self.frames = []

    def __call__(self, topic, fmt, payload):
        # A canvas JSON payload must be ready for strict JSON serialization.
        if fmt == "data/json":
            json.dumps(payload, allow_nan=False)
        with self.lock:
            self.frames.append((topic, fmt, copy.deepcopy(payload)))

    def matching(self, suffix, fmt=None):
        with self.lock:
            return [p for topic, kind, p in self.frames if topic.endswith(suffix) and (fmt is None or kind == fmt)]


def eventually(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition did not become true")


class S10CardTests(unittest.TestCase):
    def setUp(self):
        self.bridge = FakeBridge()
        self.output = Publications()
        self.plugins = cards.build_plugins({}, "test_robot", self.bridge, self.output)
        self.by_name = {plugin.name: plugin for plugin in self.plugins}

    def tearDown(self):
        for plugin in self.plugins:
            plugin.stop()

    def start(self, name, **options):
        return self.by_name[name].dispatch("start", {"refresh_hz": 30, **options})

    def test_exactly_five_idle_read_only_sensors_have_fixed_ports_and_lifecycle_schema(self):
        self.assertEqual(["state", "joints", "imu", "odometry", "lidar"], list(self.by_name))
        self.assertEqual([], self.bridge.acquires)
        for name, plugin in self.by_name.items():
            definition = plugin.get_tool()
            self.assertEqual("sensor", definition["type"])
            self.assertFalse(definition["multiInstance"])
            self.assertIn("S10 Pro", definition["title"])
            self.assertEqual(["info", "start", "stop", "status", "config"],
                             definition["inputSchema"]["properties"]["action"]["enum"])
            self.assertEqual([], definition["topic_in"])
            info = plugin.dispatch("info", {})
            self.assertEqual("idle", info["state"])
            self.assertFalse(info["connected"])
            self.assertEqual(definition["topic_out"], info["topic_out"])
            self.assertEqual(f"/test_robot/s10_pro/{name}", info["topic_out"][0]["topic"])
        self.assertEqual("data/json", self.by_name["joints"].get_tool()["topic_out"][0]["format"])
        self.assertEqual("data/json", self.by_name["odometry"].get_tool()["topic_out"][0]["format"])
        self.assertEqual(["sensor/pointcloud", "data/json"],
                         [port["format"] for port in self.by_name["lidar"].get_tool()["topic_out"]])
        self.assertFalse(self.by_name["state"].status()["capabilities"]["motion"]["supported"])

    def test_start_is_idempotent_stop_has_no_later_publication_and_releases_only_this_card(self):
        self.bridge.put("imu", {"header": {"frame_id": "", "stamp": {"sec": 4, "nanosec": 9}},
                                "angular_velocity": {"x": 0.1}})
        self.assertEqual("running", self.start("imu")["state"])
        self.assertEqual("running", self.start("imu")["state"])
        eventually(lambda: bool(self.output.matching("/imu")))
        self.assertEqual(1, len(self.bridge.acquires))
        stopped = self.by_name["imu"].dispatch("stop", {})
        count = len(self.output.matching("/imu"))
        self.assertEqual("idle", stopped["state"])
        self.assertFalse(stopped["connected"])
        self.assertEqual(["imu"], self.bridge.releases)
        time.sleep(0.08)
        self.assertEqual(count, len(self.output.matching("/imu")))
        self.by_name["imu"].stop()
        self.assertEqual(["imu"], self.bridge.releases)
        self.assertIsNone(self.by_name["imu"].dispatch("stand", {}))

    def test_stopping_one_card_keeps_other_card_reader_lease_and_publications(self):
        self.bridge.put("imu", {"header": {"frame_id": ""}})
        self.bridge.put("battery", {"data": [{"battery_level": 59}]})
        self.bridge.put("motion_info", {"data": {"motion_state": {"state": 0}}})
        self.start("state")
        self.start("imu")
        eventually(lambda: bool(self.output.matching("/state")) and bool(self.output.matching("/imu")))
        self.by_name["state"].stop()
        count = len(self.output.matching("/imu"))
        eventually(lambda: len(self.output.matching("/imu")) > count)
        self.assertEqual({"imu": ["imu"]}, self.bridge.active)
        self.assertTrue(self.by_name["imu"].status()["connected"])

    def test_stop_cancels_pending_start_without_waiting_and_late_acquire_is_released(self):
        self.bridge.allow_acquire = threading.Event()
        result = []
        thread = threading.Thread(target=lambda: result.append(self.start("imu")))
        thread.start()
        self.assertTrue(self.bridge.acquire_entered.wait(1))
        begin = time.monotonic()
        self.by_name["imu"].stop()
        self.assertLess(time.monotonic() - begin, 0.3)
        self.bridge.allow_acquire.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual("idle", result[0]["state"])
        self.assertEqual({}, self.bridge.active)
        self.assertEqual(["imu"], self.bridge.releases)
        self.assertEqual([], self.output.matching("/imu"))

    def test_acquire_failure_is_reported_and_can_retry(self):
        self.bridge.acquire_error = "reader unavailable"
        result = self.start("imu")
        self.assertEqual("error", result["state"])
        self.assertIn("reader unavailable", result["last_error"])
        self.assertFalse(result["connected"])
        self.bridge.acquire_error = None
        self.assertEqual("running", self.start("imu")["state"])

    def test_no_sample_and_expired_sample_never_claim_connected(self):
        self.start("imu")
        eventually(lambda: bool(self.output.matching("/imu")))
        missing = self.output.matching("/imu")[-1]
        self.assertEqual("no_data", missing["status"])
        self.assertIsNone(missing["data"])
        self.assertFalse(missing["connected"])
        self.assertFalse(missing["source"]["imu"]["received"])
        raw = {"header": {"frame_id": "", "stamp": {"sec": 2, "nanosec": 3}}}
        self.bridge.put("imu", raw, age=9)
        eventually(lambda: any(p["stale"] for p in self.output.matching("/imu")))
        expired = self.output.matching("/imu")[-1]
        self.assertFalse(expired["fresh"])
        self.assertFalse(expired["connected"])
        self.assertEqual(9, expired["source"]["imu"]["age_s"])
        self.assertEqual(raw["header"]["stamp"], expired["source"]["imu"]["stamp"])

    def test_sixteen_joint_records_names_and_unknown_units_remain_raw(self):
        records = [{"name": list(str(i).encode("ascii")) + [0] * (4 - len(str(i))),
                    "position": i / 10, "velocity": 0, "torque": 0, "status_word": 3}
                   for i in range(16)]
        raw = {"header": {"stamp": {"sec": 11, "nanosec": 12}}, "data": {"joints_data": records}}
        self.bridge.put("joints", raw)
        self.start("joints")
        eventually(lambda: bool(self.output.matching("/joints")))
        payload = self.output.matching("/joints")[-1]
        self.assertEqual(raw, payload["data"])
        self.assertEqual(16, payload["joint_count"])
        self.assertEqual([str(i) for i in range(16)], payload["joint_labels"])
        self.assertTrue(all(value is None for value in payload["units"].values()))
        self.assertEqual("unknown", payload["status_word_meaning"])

    def test_state_keeps_battery_raw_and_reports_partial_source_freshness(self):
        battery = {"data": [{"battery_level": 59, "voltage": 7501, "current": -49}]}
        self.bridge.put("battery", battery)
        self.start("state")
        eventually(lambda: bool(self.output.matching("/state")))
        payload = self.output.matching("/state")[-1]
        self.assertEqual(battery, payload["data"]["battery"])
        self.assertIsNone(payload["data"]["motion_info"])
        self.assertEqual("partial", payload["status"])
        self.assertTrue(payload["connected"])
        self.assertFalse(payload["fresh"])
        self.assertIsNone(payload["units"]["battery"]["battery_level"])
        self.assertEqual("requires_vendor_contract", payload["capabilities"]["motion"]["status"])

    def test_standard_imu_and_odometry_preserve_empty_frame_and_ros_structure(self):
        imu = {"header": {"frame_id": "", "stamp": {"sec": 4, "nanosec": 2}},
               "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}
        odom = {"header": {"frame_id": "map"}, "child_frame_id": "", "pose": {"pose": {"position": {"x": 2}}}}
        self.bridge.put("imu", imu)
        self.bridge.put("odometry", odom)
        self.start("imu")
        self.start("odometry")
        eventually(lambda: bool(self.output.matching("/imu")) and bool(self.output.matching("/odometry")))
        self.assertEqual(imu, self.output.matching("/imu")[-1]["data"])
        payload = self.output.matching("/odometry")[-1]
        self.assertEqual(odom, payload["data"])
        self.assertNotIn("schema", payload)
        self.assertNotIn("motus.odom", json.dumps(payload))

    def test_lidar_sends_binary_once_per_fresh_sequence_but_metadata_keeps_age(self):
        self.bridge.put("lidar", {"header": {"frame_id": "lidar_link"}, "point_count": 3}, binary=b"frame-one")
        self.start("lidar", max_points=100)
        eventually(lambda: bool(self.output.matching("/lidar", "sensor/pointcloud")))
        eventually(lambda: len(self.output.matching("/lidar_info")) >= 2)
        self.assertEqual([b"frame-one"], self.output.matching("/lidar", "sensor/pointcloud"))
        self.bridge.put("lidar", {"point_count": 3}, sequence=2, age=8, binary=b"stale-frame")
        eventually(lambda: any(p["stale"] for p in self.output.matching("/lidar_info")))
        self.assertEqual([b"frame-one"], self.output.matching("/lidar", "sensor/pointcloud"))
        self.bridge.put("lidar", {"point_count": 2}, sequence=3, binary=b"new-frame")
        eventually(lambda: len(self.output.matching("/lidar", "sensor/pointcloud")) == 2)
        self.assertEqual([b"frame-one", b"new-frame"], self.output.matching("/lidar", "sensor/pointcloud"))
        self.assertEqual(100, self.bridge.acquires[0][2]["max_points"])
        self.by_name["lidar"].stop()
        count = len(self.output.matching("/lidar_info"))
        time.sleep(0.08)
        self.assertEqual(count, len(self.output.matching("/lidar_info")))

    def test_config_defaults_validation_and_disabled_start(self):
        expected_hz = {"state": 1, "joints": 5, "imu": 5, "odometry": 5, "lidar": 2}
        for name, plugin in self.by_name.items():
            self.assertEqual(expected_hz[name], plugin.status()["settings"]["refresh_hz"])
            self.assertEqual(5, plugin.status()["settings"]["stale_after_s"])
        self.assertEqual(6000, self.by_name["lidar"].status()["settings"]["max_points"])
        disabled = cards.build_plugins({"plugins": {"imu": {"enabled": False}}}, "host", self.bridge, self.output)[2]
        self.assertEqual("disabled", disabled.start()["state"])
        self.assertEqual([], self.bridge.acquires)
        for invalid in (float("nan"), float("inf"), 0, -1, True):
            with self.assertRaises(ValueError):
                self.by_name["imu"].start({"refresh_hz": invalid})
        with self.assertRaises(ValueError):
            self.by_name["lidar"].start({"max_points": 3.5})

    def test_nonfinite_ros_values_are_explicit_null_in_strict_json(self):
        self.bridge.put("imu", {"header": {"frame_id": ""}, "orientation": {"x": float("nan")}})
        self.start("imu")
        eventually(lambda: bool(self.output.matching("/imu")))
        self.assertIsNone(self.output.matching("/imu")[-1]["data"]["orientation"]["x"])

    def test_config_replay_updates_info_defaults_and_next_start_without_starting_reader(self):
        card = self.by_name["lidar"]
        ports = card.get_tool()["topic_out"]
        response = card.dispatch("config", {"refresh_hz": 3, "stale_after_s": 8, "max_points": 1000,
                                             "_tool_name": "lidar"})
        self.assertEqual("idle", response["state"])
        self.assertEqual([], self.bridge.acquires)
        self.assertEqual(1000, card.dispatch("info", {})["settings"]["max_points"])
        self.assertEqual(3, card.get_tool()["configSchema"]["properties"]["refresh_hz"]["default"])
        self.assertEqual(ports, response["topic_out"])
        card.start()
        self.assertEqual({"enabled": True, "refresh_hz": 3.0, "stale_after_s": 8.0, "max_points": 1000},
                         self.bridge.acquires[-1][2])

    def test_live_config_updates_reader_options_and_disabled_config_closes_only_this_card(self):
        self.start("imu")
        self.start("lidar")
        response = self.by_name["lidar"].dispatch("config", {"config": {"max_points": 500, "refresh_hz": 4}})
        self.assertEqual("running", response["state"])
        self.assertEqual("lidar", self.bridge.acquires[-1][0])
        self.assertEqual(500, self.bridge.acquires[-1][2]["max_points"])
        disabled = self.by_name["lidar"].dispatch("config", {"enabled": False})
        self.assertEqual("idle", disabled["state"])
        self.assertFalse(disabled["enabled"])
        self.assertEqual({"imu": ["imu"]}, self.bridge.active)
        self.assertEqual("disabled", self.by_name["lidar"].start()["state"])

    def test_config_validation_is_atomic_and_start_accepts_nested_config(self):
        card = self.by_name["lidar"]
        before = card.status()["settings"]
        with self.assertRaises(ValueError):
            card.dispatch("config", {"max_points": 50001, "refresh_hz": 4})
        self.assertEqual(before, card.status()["settings"])
        started = card.dispatch("start", {"config": {"max_points": 400, "stale_after_s": 2}})
        self.assertEqual(400, started["settings"]["max_points"])
        self.assertEqual(2, started["settings"]["stale_after_s"])

    def test_config_updates_do_not_revive_a_concurrently_stopped_card(self):
        card = self.by_name["lidar"]
        self.start("lidar")
        self.bridge.acquire_entered.clear()
        self.bridge.allow_acquire = threading.Event()
        result = []
        thread = threading.Thread(target=lambda: result.append(card.dispatch("config", {"max_points": 1000})))
        thread.start()
        self.assertTrue(self.bridge.acquire_entered.wait(1))
        card.stop()
        self.bridge.allow_acquire.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual("idle", result[0]["state"])
        self.assertEqual({}, self.bridge.active)

    def test_custom_source_topic_is_reported_and_missing_joint_array_is_safe(self):
        configured = cards.build_plugins({"topics": {"imu": "/custom/IMU"}}, "", self.bridge, self.output)
        self.assertEqual("/s10_pro/imu", configured[2].topic)
        self.assertEqual("/custom/IMU", configured[2].status()["source"]["imu"]["topic"])
        self.bridge.put("joints", {"data": None})
        self.start("joints")
        eventually(lambda: bool(self.output.matching("/joints")))
        self.assertEqual(0, self.output.matching("/joints")[-1]["joint_count"])

    def test_stop_cleans_lease_even_if_start_is_just_returning_while_lease_lock_is_busy(self):
        card = self.by_name["imu"]
        self.start("imu")
        # Emulate the return-path interval after acquisition succeeds but
        # before start's lease operation has left its critical section.
        card._lease_lock.acquire()
        try:
            stopped = card.stop()
            self.assertEqual("idle", stopped["state"])
        finally:
            card._lease_lock.release()
        eventually(lambda: not self.bridge.active)

    def test_lidar_rejects_out_of_order_binary_sequences(self):
        self.bridge.put("lidar", {"point_count": 1}, sequence=5, binary=b"latest")
        self.start("lidar")
        eventually(lambda: bool(self.output.matching("/lidar", "sensor/pointcloud")))
        self.bridge.put("lidar", {"point_count": 1}, sequence=4, binary=b"out-of-order")
        eventually(lambda: len(self.output.matching("/lidar_info")) >= 3)
        self.assertEqual([b"latest"], self.output.matching("/lidar", "sensor/pointcloud"))

    def test_older_config_cannot_overwrite_reader_options_after_newer_config_overtakes_it(self):
        class GateLock:
            def __init__(self):
                self.lock = threading.Lock()
                self.older_waiting = threading.Event()
                self.allow_older = threading.Event()

            def __enter__(self):
                if threading.current_thread().name == "older-config":
                    self.older_waiting.set()
                    if not self.allow_older.wait(1):
                        raise AssertionError("newer config never completed")
                self.lock.acquire()
                return self

            def __exit__(self, *_):
                self.lock.release()

            def acquire(self, *args, **kwargs):
                return self.lock.acquire(*args, **kwargs)

            def release(self):
                self.lock.release()

        card = self.by_name["lidar"]
        self.start("lidar")
        gate = GateLock()
        card._lease_lock = gate
        results = []
        older = threading.Thread(name="older-config", target=lambda: results.append(
            card.dispatch("config", {"max_points": 1000})))
        older.start()
        self.assertTrue(gate.older_waiting.wait(1))
        try:
            newer = card.dispatch("config", {"max_points": 2000})
            self.assertEqual(2000, newer["settings"]["max_points"])
        finally:
            gate.allow_older.set()
        older.join(1)
        self.assertFalse(older.is_alive())
        self.assertEqual(2000, results[0]["settings"]["max_points"])
        self.assertEqual(2000, self.bridge.acquires[-1][2]["max_points"])

    def test_declarative_topic_and_message_type_overrides_match_reader_and_payload_metadata(self):
        configured = cards.build_plugins({
            "topics": {"imu": {"topic": "/IMU_DATA", "type": "drdds/msg/ImuData"},
                       "odometry": "/custom/ODOM"},
            "message_types": {"odometry": "custom_msgs/msg/Odometry"},
        }, "host", self.bridge, self.output)
        imu, odometry = configured[2], configured[3]
        self.bridge.put("imu", {"header": {"frame_id": "", "stamp": {"sec": 1}}, "data": {"roll": 0.1}})
        self.bridge.put("odometry", {"header": {"frame_id": "map"}, "position": [1, 2, 3]})
        try:
            imu.start()
            odometry.start()
            eventually(lambda: bool(self.output.matching("/imu")) and bool(self.output.matching("/odometry")))
            imu_data = self.output.matching("/imu")[-1]
            self.assertEqual("/IMU_DATA", imu_data["source"]["imu"]["topic"])
            self.assertEqual("drdds/msg/ImuData", imu_data["source"]["imu"]["message_type"])
            self.assertEqual("drdds/msg/ImuData", imu_data["message_type"])
            self.assertEqual("", imu_data["data"]["header"]["frame_id"])
            odom_data = self.output.matching("/odometry")[-1]
            self.assertEqual("/custom/ODOM", odom_data["source"]["odometry"]["topic"])
            self.assertEqual("custom_msgs/msg/Odometry", odom_data["message_type"])
            self.assertEqual("raw_ros_message", odom_data["frame_contract"])
            self.assertEqual("drdds/msg/ImuData", imu.status()["source"]["imu"]["message_type"])
        finally:
            imu.stop()
            odometry.stop()

    def test_overtaken_disable_config_does_not_stop_a_newer_enabled_configuration(self):
        card = self.by_name["imu"]
        self.start("imu")
        real_stop = card.stop
        disable_waiting, allow_disable = threading.Event(), threading.Event()

        def gated_stop(**kwargs):
            if threading.current_thread().name == "older-disable":
                disable_waiting.set()
                if not allow_disable.wait(1):
                    raise AssertionError("newer enabled config never completed")
            return real_stop(**kwargs)

        card.stop = gated_stop
        results = []
        older = threading.Thread(name="older-disable", target=lambda: results.append(
            card.dispatch("config", {"enabled": False})))
        older.start()
        self.assertTrue(disable_waiting.wait(1))
        try:
            newer = card.dispatch("config", {"enabled": True})
            self.assertEqual("running", newer["state"])
        finally:
            allow_disable.set()
        older.join(1)
        self.assertFalse(older.is_alive())
        self.assertEqual("running", results[0]["state"])
        self.assertTrue(card.status()["settings"]["enabled"])
        self.assertEqual({"imu": ["imu"]}, self.bridge.active)


if __name__ == "__main__":
    unittest.main()
