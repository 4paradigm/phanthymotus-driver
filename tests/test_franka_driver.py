"""Franka canvas contracts and multi-device lifecycle, without ROS or hardware."""

from __future__ import annotations

import copy
import importlib.util
import math
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

from common.vendor_runtime import DriverBundle


DRIVER = Path(__file__).resolve().parents[1] / "franka" / "fr3"


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, DRIVER / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Other drivers also have a bare `hardware` import. Do not pollute their tests.
hardware = load_module("franka_test_hardware", "hardware.py")
with mock.patch.dict(sys.modules, {"hardware": hardware}):
    device = load_module("franka_test_device", "device.py")


class FakeSession:
    """Only the read-only session interface exists; no motion API is available."""

    def __init__(self, address, **options):
        self.address = address
        self.closed = 0
        self._stop = threading.Event()
        self.close_deadline = 0.5
        self.value = {
            "robot_ip": address, "state": "connecting", "connected": False,
            "data_age_ms": None, "error": None,
        }

    def snapshot(self):
        return copy.deepcopy(self.value)

    @property
    def stopping(self):
        return self._stop.is_set()

    @property
    def stopped(self):
        return self.closed > 0

    def ready(self, q=None):
        self.value.update(
            state="connected", connected=True, data_age_ms=12, error=None,
            q=q if q is not None else [0, math.pi / 2, -math.pi / 2, 0, 0, 0, 0],
            robot_mode=1, timestamp_ms=1234567890,
        )

    def close(self):
        self._stop.set()
        self.closed += 1


class FakePublisher:
    def __init__(self, topic, hz, callback):
        self.topic = topic
        self.hz = hz
        self.callback = callback
        self.frames = []
        self.closed = 0

    def publish(self, payload):
        self.frames.append(copy.deepcopy(payload))

    def close(self):
        self.closed += 1


class FrankaCardTests(unittest.TestCase):
    def setUp(self):
        self.sessions = []
        self.publishers = []

        def session_factory(address, **options):
            session = FakeSession(address, **options)
            self.sessions.append(session)
            return session

        self.session_patch = mock.patch.object(hardware, "RobotSession", session_factory)
        self.session_patch.start()
        self.addCleanup(self.session_patch.stop)
        self.manager = hardware.SessionManager()

        def publisher_factory(topic, hz, callback):
            publisher = FakePublisher(topic, hz, callback)
            self.publishers.append(publisher)
            return publisher

        self.plugin = device.FrankaPlugin(
            {}, namespace="test_franka", manager=self.manager,
            publisher_factory=publisher_factory,
        )
        self.addCleanup(self.plugin.stop)
        self.bundle = DriverBundle([self.plugin])

    def call(self, name="joint_states", action="info", instance_id="card-a", **args):
        return self.bundle.dispatch(name, {
            "action": action, "instance_id": instance_id, **args,
        })

    def start_card(self, name="joint_states", instance_id="card-a", ip="172.16.0.3", **args):
        return self.call(name, "start", instance_id, robot_ip=ip, **args)

    def test_only_joint_sensor_and_automatic_model_resource_are_registered(self):
        self.assertEqual(self.plugin.PREFIX, "joint_states")
        definitions = {item["name"]: item for item in self.bundle.get_all_tools()}
        self.assertEqual(set(definitions), {"joint_states", "model"})
        spec = definitions["joint_states"]
        self.assertEqual(spec["type"], "sensor")
        self.assertTrue(spec["multiInstance"])
        self.assertEqual(spec["configSchema"]["required"], ["robot_ip"])
        ip = spec["configSchema"]["properties"]["robot_ip"]
        self.assertEqual(ip["scope"], "instance")
        self.assertTrue(ip["x-sensitive"])
        self.assertNotIn("topic", spec["topic_out"][0])
        self.assertEqual(spec["topic_out"][0]["format"], device.JOINT_FORMAT)
        self.assertEqual(spec["inputSchema"]["properties"], {})
        self.assertEqual(definitions["model"]["type"], "resource")
        self.assertNotIn("configSchema", definitions["model"])

    def test_removed_connection_card_cannot_be_dispatched(self):
        for action in ("config", "start", "info", "stop"):
            with self.subTest(action=action):
                self.assertIsNone(self.call("connection", action, robot_ip="172.16.0.3"))
                self.assertIsNone(self.plugin.dispatch(action, {
                    "_tool_name": "connection", "instance_id": "card-a", "robot_ip": "172.16.0.3",
                }))
        self.assertEqual(self.sessions, [])
        self.assertEqual(self.publishers, [])

    def test_driver_start_and_unconfigured_info_do_not_open_hardware(self):
        self.bundle.start_all()
        info = self.call()
        self.assertEqual(info["state"], "idle")
        self.assertEqual(info["robot_ip"], "")
        self.assertEqual(info["topic_out"], [{
            "topic": "/test_franka/card_a/state/joints", "format": device.JOINT_FORMAT,
        }])
        self.assertEqual(self.sessions, [])
        self.assertEqual(self.publishers, [])

    def test_idle_config_infers_topic_without_hardware_and_start_reuses_config(self):
        info = self.call(action="config", robot_ip="172.16.0.3", publish_hz=15)
        self.assertEqual(info["state"], "idle")
        self.assertEqual(self.sessions, [])
        started = self.call(action="start")
        self.assertEqual(started["topic_out"], info["topic_out"])
        self.assertEqual(started["robot_ip"], "172.16.0.3")
        self.assertEqual(self.publishers[0].hz, 15)

    def test_shared_defaults_and_instance_overrides_keep_devices_separate(self):
        self.bundle.dispatch("joint_states", {
            "action": "config", "robot_ip": "172.16.0.3", "publish_hz": 8,
        })
        self.assertEqual(self.sessions, [])
        self.call(action="config", instance_id="card-b", robot_ip="172.16.0.4")
        self.call(action="start")
        self.call(action="start", instance_id="card-b")
        self.assertEqual([s.address for s in self.sessions], ["172.16.0.3", "172.16.0.4"])
        self.assertEqual([p.hz for p in self.publishers], [8, 8])
        self.assertEqual([p.topic for p in self.publishers], [
            "/test_franka/card_a/state/joints", "/test_franka/card_b/state/joints",
        ])

    def test_start_requires_ip_and_does_not_choose_another_connected_device(self):
        self.start_card()
        with self.assertRaisesRegex(ValueError, "robot_ip is required"):
            self.call(action="start", instance_id="card-b")
        self.assertEqual(len(self.sessions), 1)
        self.assertEqual(len(self.publishers), 1)

    def test_joint_start_reports_loading_until_first_valid_sample(self):
        info = self.start_card()
        self.assertEqual(info["state"], "loading")
        self.assertFalse(info["connected"])
        self.assertTrue(info["message"])
        self.sessions[0].ready()
        info = self.call()
        self.assertEqual(info["state"], "running")
        self.assertTrue(info["connected"])
        self.assertEqual(info["data_age_ms"], 12)

    def test_joint_info_reports_offline_error_without_publishing_a_pose(self):
        info = self.start_card()
        self.assertEqual(info["state"], "loading")
        self.assertFalse(info["connected"])
        self.sessions[0].value.update(state="disconnected", error="FCI unavailable")
        self.publishers[0].callback()
        self.assertEqual(self.publishers[0].frames, [])
        info = self.call()
        self.assertEqual(info["error"], "FCI unavailable")
        self.assertEqual(info["message"], "FCI unavailable")
        self.assertEqual(info["readiness"], "disconnected")
        self.assertFalse(info["connected"])
        self.assertTrue(info["read_only"])
        self.assertEqual(info["state"], "loading")

    def test_same_ip_joint_cards_share_session_until_last_owner_stops(self):
        self.start_card()
        self.start_card(instance_id="card-b")
        self.start_card(instance_id="card-c", ip=" ::ffff:172.16.0.3 ")
        self.assertEqual(len(self.sessions), 1)
        self.assertEqual(len(self.publishers), 3)
        self.assertEqual(len({p.topic for p in self.publishers}), 3)
        self.call(action="stop")
        self.assertEqual(self.sessions[0].closed, 0)
        self.call(action="stop", instance_id="card-b")
        self.assertEqual(self.sessions[0].closed, 0)
        self.call(action="stop", instance_id="card-c")
        self.assertEqual(self.sessions[0].closed, 1)
        self.assertTrue(all(p.closed == 1 for p in self.publishers))

    def test_different_ip_failure_does_not_poison_other_card(self):
        self.start_card()
        self.start_card(instance_id="card-b", ip="172.16.0.4")
        self.sessions[0].value.update(state="disconnected", error="No route")
        self.sessions[1].ready([0.1] * 7)
        for publisher in self.publishers:
            publisher.callback()
        self.assertEqual(self.publishers[0].frames, [])
        self.assertEqual(len(self.publishers[1].frames), 1)
        self.assertEqual(self.call(instance_id="card-b")["state"], "running")
        self.assertEqual(self.call()["state"], "loading")

    def test_duplicate_start_and_stop_are_idempotent(self):
        self.start_card()
        self.call(action="start")
        self.call(action="config", robot_ip="172.16.0.3", publish_hz=10)
        self.assertEqual(len(self.sessions), 1)
        self.assertEqual(len(self.publishers), 1)
        self.call(action="stop")
        self.call(action="stop")
        self.assertEqual(self.sessions[0].closed, 1)
        self.assertEqual(self.publishers[0].closed, 1)

    def test_changing_live_target_or_frequency_requires_stop(self):
        self.start_card()
        for changed in ({"robot_ip": "172.16.0.4"}, {"publish_hz": 20}, {"robot_ip": ""}):
            with self.subTest(changed=changed), self.assertRaisesRegex(ValueError, "Stop this card"):
                self.call(action="config", **changed)
        self.assertEqual(self.call()["robot_ip"], "172.16.0.3")
        self.assertEqual(len(self.sessions), 1)
        self.call(action="stop")
        self.call(action="config", robot_ip="172.16.0.4", publish_hz=20)
        self.call(action="start")
        self.assertEqual(self.sessions[-1].address, "172.16.0.4")
        self.assertEqual(self.publishers[-1].hz, 20)

    def test_invalid_ip_is_rejected_before_any_connection(self):
        for ip in ("franka.local", "172.16.0.3:80", "999.16.0.3", "http://172.16.0.3", "fe80::1%eno1"):
            with self.subTest(ip=ip), self.assertRaisesRegex(ValueError, "robot_ip"):
                self.call(action="config", robot_ip=ip)
        self.assertEqual(self.sessions, [])

    def test_publish_frequency_must_be_finite_and_bounded(self):
        for hz in (0, -1, 31, math.nan, math.inf, -math.inf, True, None, "abc", 10 ** 1000):
            with self.subTest(hz=hz), self.assertRaisesRegex(ValueError, "publish_hz"):
                self.call(action="config", robot_ip="172.16.0.3", publish_hz=hz)
        self.assertEqual(self.sessions, [])

    def test_invalid_config_does_not_overwrite_a_previously_valid_target(self):
        self.call(action="config", robot_ip="172.16.0.3", publish_hz=15)
        with self.assertRaisesRegex(ValueError, "publish_hz"):
            self.call(action="config", robot_ip="172.16.0.4", publish_hz=math.nan)
        self.call(action="start")
        self.assertEqual(self.sessions[0].address, "172.16.0.3")
        self.assertEqual(self.publishers[0].hz, 15)

    def test_instance_ids_are_validated_and_topic_collisions_rejected(self):
        for instance_id in ("", "../card", "123card", "card/a", 1):
            with self.subTest(instance_id=instance_id), self.assertRaisesRegex(ValueError, "instance_id"):
                self.call(instance_id=instance_id)
        first = self.call(instance_id="card-a")
        with self.assertRaisesRegex(ValueError, "collides"):
            self.call(instance_id="card_a")
        second = self.call(instance_id="card-b")
        self.assertNotEqual(first["topic_out"], second["topic_out"])

    def test_false_or_null_instance_id_cannot_change_shared_defaults(self):
        for instance_id in (False, 0, None):
            with self.subTest(instance_id=instance_id), self.assertRaisesRegex(ValueError, "instance_id"):
                self.call(action="config", instance_id=instance_id, robot_ip="172.16.0.3")
        with self.assertRaisesRegex(ValueError, "robot_ip is required"):
            self.call(action="start")

    def test_publisher_failure_releases_only_its_own_session_owner(self):
        self.start_card(instance_id="card-b")
        self.sessions[0].ready()
        original = self.plugin._publisher_factory
        self.plugin._publisher_factory = mock.Mock(side_effect=RuntimeError("DDS unavailable"))
        with self.assertRaisesRegex(RuntimeError, "DDS unavailable"):
            self.start_card()
        self.assertEqual(self.sessions[0].closed, 0)
        self.assertEqual(self.call()["state"], "idle")
        self.assertEqual(self.call(instance_id="card-b")["state"], "running")
        self.plugin._publisher_factory = original
        self.call(action="start")
        self.assertEqual(len(self.sessions), 1)

    def test_publisher_failure_closes_new_session_and_can_retry(self):
        original = self.plugin._publisher_factory
        self.plugin._publisher_factory = mock.Mock(side_effect=RuntimeError("DDS unavailable"))
        with self.assertRaisesRegex(RuntimeError, "DDS unavailable"):
            self.start_card()
        self.assertEqual(self.sessions[0].closed, 1)
        self.plugin._publisher_factory = original
        self.call(action="start")
        self.assertEqual(len(self.sessions), 2)

    def test_stopped_callback_is_inert_and_restart_uses_saved_config(self):
        self.start_card()
        self.sessions[0].ready()
        old_publisher = self.publishers[0]
        self.call(action="stop")
        old_publisher.callback()
        self.assertEqual(old_publisher.frames, [])
        self.call(action="start")
        self.assertEqual(self.sessions[-1].address, "172.16.0.3")
        self.sessions[-1].ready()
        old_publisher.callback()
        self.assertEqual(self.publishers[-1].frames, [], "Old timer must not publish after restart")
        self.publishers[-1].callback()
        self.assertEqual(len(self.publishers[-1].frames), 1)

    def test_stale_pose_is_not_republished_but_info_remains_available(self):
        self.start_card()
        self.sessions[0].ready()
        self.publishers[0].callback()
        self.sessions[0].value.update(connected=False, state="stale", data_age_ms=2500,
                                      error="Robot state stream is stale")
        self.publishers[0].callback()
        self.assertEqual(len(self.publishers[0].frames), 1)
        info = self.call()
        self.assertEqual(info["readiness"], "stale")
        self.assertEqual(info["error"], "Robot state stream is stale")
        self.assertEqual(info["data_age_ms"], 2500)
        self.assertFalse(info["connected"])
        self.sessions[0].ready()
        self.publishers[0].callback()
        self.assertEqual(len(self.publishers[0].frames), 2)

    def test_invalid_joint_samples_are_not_published(self):
        self.start_card()
        for q in ([0] * 6, [0] * 8, [math.nan] * 7, [math.inf] * 7):
            with self.subTest(q=q):
                self.sessions[0].ready(q)
                self.publishers[0].callback()
        self.assertEqual(self.publishers[0].frames, [])

    def test_publish_failure_is_contained_visible_and_retried_without_affecting_other_card(self):
        self.start_card()
        self.start_card(instance_id="card-b", ip="172.16.0.4")
        for session in self.sessions:
            session.ready()
        failing = self.publishers[0]
        normal_publish = failing.publish
        failing.publish = mock.Mock(side_effect=RuntimeError("DDS unavailable"))
        with self.assertLogs(device.log, level="WARNING") as logs:
            for _ in range(3):
                failing.callback()
                self.publishers[1].callback()
        self.assertEqual(len(logs.output), 1, "A persistent fault must not log at frame rate")
        self.assertEqual(len(self.publishers[1].frames), 3)
        info = self.call()
        self.assertEqual(info["state"], "error")
        self.assertTrue(info["connected"], "The error belongs to DDS, not the robot connection")
        self.assertIn("DDS unavailable", info["publication_error"])
        self.assertEqual(info["error"], info["publication_error"])
        self.assertEqual(self.call(instance_id="card-b")["state"], "running")
        failing.publish = normal_publish
        failing.callback()
        self.assertEqual(len(failing.frames), 1)
        self.assertEqual(self.call()["state"], "running")
        self.assertIsNone(self.call()["publication_error"])

    def test_stop_clears_a_previous_publication_error(self):
        self.start_card()
        self.sessions[0].ready()
        self.publishers[0].publish = mock.Mock(side_effect=RuntimeError("DDS unavailable"))
        with self.assertLogs(device.log, level="WARNING"):
            self.publishers[0].callback()
        self.call(action="stop")
        self.assertIsNone(self.call()["publication_error"])
        self.call(action="start")
        self.assertEqual(self.call()["state"], "loading")
        self.assertIsNone(self.call()["error"])

    def test_skeleton_uses_actual_radians_sample_timestamp_and_matching_urdf_names(self):
        self.start_card()
        self.sessions[0].ready()
        self.publishers[0].callback()
        frame = self.publishers[0].frames[-1]
        self.assertEqual(frame["timestamp_ms"], 1234567890)
        self.assertEqual(frame["position_unit"], "rad")
        self.assertEqual(frame["joint_count"], 7)
        self.assertEqual(frame["format"], "sensor/skeleton")
        self.assertAlmostEqual(frame["joints"][1]["q"], math.pi / 2)
        self.assertAlmostEqual(frame["joints"][1]["degree"], 90)
        self.assertEqual([j["idx"] for j in frame["joints"]], list(range(7)))
        urdf = ET.fromstring(self.bundle.dispatch("model", {})["urdf"])
        movable = [j.attrib["name"] for j in urdf.findall("joint") if j.attrib["type"] != "fixed"]
        self.assertEqual(movable, [j["name"] for j in frame["joints"]])
        self.assertEqual(len(movable), 7)

    def test_model_dispatch_is_plain_static_resource_without_hardware(self):
        for arguments in ({}, {"action": "start"}, {"action": "info", "instance_id": "model-card"}):
            result = self.bundle.dispatch("model", arguments)
            self.assertEqual(set(result), {"urdf"})
            self.assertEqual(ET.fromstring(result["urdf"]).tag, "robot")
        self.assertEqual(self.sessions, [])

    def test_urdf_is_one_connected_chain_with_valid_joint_references(self):
        robot = ET.fromstring(self.bundle.dispatch("model", {})["urdf"])
        links = {link.attrib["name"] for link in robot.findall("link")}
        joints = robot.findall("joint")
        children = [joint.find("child").attrib["link"] for joint in joints]
        self.assertEqual(len(children), len(set(children)))
        roots = links - set(children)
        self.assertEqual(len(roots), 1)
        reached = set(roots)
        for _ in joints:
            for joint in joints:
                parent = joint.find("parent").attrib["link"]
                child = joint.find("child").attrib["link"]
                self.assertIn(parent, links)
                self.assertIn(child, links)
                if parent in reached:
                    reached.add(child)
        self.assertEqual(reached, links)

    def test_stop_releases_all_active_cards_once_and_prevents_restart(self):
        self.start_card()
        self.start_card(instance_id="card-c")
        self.start_card(instance_id="card-b", ip="172.16.0.4")
        self.plugin.stop()
        self.plugin.stop()
        self.assertEqual([s.closed for s in self.sessions], [1, 1])
        self.assertEqual([p.closed for p in self.publishers], [1, 1, 1])
        with self.assertRaisesRegex(RuntimeError, "shutting down"):
            self.call(action="start")
        with self.assertRaisesRegex(RuntimeError, "shutting down"):
            self.bundle.dispatch("joint_states", {"action": "config", "robot_ip": "172.16.0.5"})

    def test_stop_cancels_inflight_start_before_creating_a_publisher(self):
        entered, resume = threading.Event(), threading.Event()
        acquired = self.manager.acquire
        results, errors = [], []

        def slow_acquire(*args):
            entered.set()
            if not resume.wait(2):
                raise TimeoutError("Test did not release the pending start")
            return acquired(*args)

        def invoke(action):
            try:
                results.append(self.call(action=action, robot_ip="172.16.0.3"))
            except Exception as exc:
                errors.append(exc)

        with mock.patch.object(self.manager, "acquire", side_effect=slow_acquire):
            starting = threading.Thread(target=lambda: invoke("start"))
            stopping = threading.Thread(target=lambda: invoke("stop"))
            starting.start()
            try:
                self.assertTrue(entered.wait(1))
                stopping.start()
                deadline = time.monotonic() + 1
                while self.plugin._instances["card-a"].start_epoch == 0:
                    if time.monotonic() >= deadline:
                        self.fail("Stop did not signal cancellation of initialization")
                    resume.wait(0.005)
            finally:
                resume.set()
                starting.join(2)
                if stopping.ident is not None:
                    stopping.join(2)
        self.assertEqual(errors, [])
        self.assertFalse(starting.is_alive())
        self.assertFalse(stopping.is_alive())
        self.assertEqual([r["state"] for r in results], ["idle", "idle"])
        self.assertEqual(self.publishers, [])
        self.assertEqual(self.sessions[0].closed, 1)
        self.call(action="start")
        self.assertEqual(len(self.publishers), 1, "A later explicit start must remain possible")

    def test_shutdown_cleans_other_cards_and_manager_when_publisher_close_fails(self):
        self.start_card()
        self.start_card(instance_id="card-c")
        self.start_card(instance_id="card-b", ip="172.16.0.4")
        self.publishers[0].close = mock.Mock(side_effect=RuntimeError("DDS teardown failed"))
        with mock.patch.object(self.manager, "close", wraps=self.manager.close) as close:
            with self.assertRaisesRegex(RuntimeError, "DDS teardown failed"):
                self.plugin.stop()
            close.assert_called_once_with()
        self.assertEqual([s.closed for s in self.sessions], [1, 1])
        self.assertEqual([p.closed for p in self.publishers[1:]], [1, 1])
        self.assertTrue(all(instance.publisher is None and instance.session is None
                            for instance in self.plugin._instances.values()))

    def test_card_stop_releases_device_even_when_publisher_close_fails(self):
        self.start_card()
        self.publishers[0].close = mock.Mock(side_effect=RuntimeError("DDS teardown failed"))
        with self.assertRaisesRegex(RuntimeError, "DDS teardown failed"):
            self.call(action="stop")
        self.assertEqual(self.sessions[0].closed, 1)
        self.assertEqual(self.call()["state"], "idle")
        self.call(action="start")
        self.assertEqual(len(self.sessions), 2)

    def test_slow_close_of_one_device_does_not_block_other_device_info(self):
        self.start_card()
        self.start_card(instance_id="card-b", ip="172.16.0.4")
        entered, release = threading.Event(), threading.Event()
        self.sessions[0].close = lambda: (entered.set(), release.wait(3))
        stopping = threading.Thread(target=lambda: self.call(action="stop"))
        stopping.start()
        self.assertTrue(entered.wait(1))
        answered = threading.Event()
        query = threading.Thread(target=lambda: (self.call(instance_id="card-b"), answered.set()))
        try:
            query.start()
            self.assertTrue(answered.wait(1), "Device B info blocked behind device A cleanup")
        finally:
            release.set()
            stopping.join(2)
            query.join(2)
        self.assertFalse(stopping.is_alive())


class RosPublisherCleanupTests(unittest.TestCase):
    def setUp(self):
        self.node = mock.Mock()
        self.ros2 = mock.Mock()
        self.node_class = mock.Mock(return_value=self.node)
        node_module = types.ModuleType("rclpy.node")
        node_module.Node = self.node_class
        qos_module = types.ModuleType("rclpy.qos")
        qos_module.QoSProfile = mock.Mock()
        self.qos_class = qos_module.QoSProfile
        qos_module.DurabilityPolicy = types.SimpleNamespace(VOLATILE="volatile")
        qos_module.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
        qos_module.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT="best_effort")
        msg_module = types.ModuleType("std_msgs.msg")
        msg_module.String = type("String", (), {})
        modules = {
            "rclpy": types.ModuleType("rclpy"), "rclpy.node": node_module,
            "rclpy.qos": qos_module, "std_msgs": types.ModuleType("std_msgs"),
            "std_msgs.msg": msg_module,
        }
        patch = mock.patch.dict(sys.modules, modules)
        patch.start()
        self.addCleanup(patch.stop)

    def make_publisher(self):
        return device.RosPublisher(self.ros2, "/franka/card_a/state/joints", 10, mock.Mock())

    def test_create_publisher_failure_destroys_created_node(self):
        self.node.create_publisher.side_effect = RuntimeError("cannot create publisher")
        with self.assertRaisesRegex(RuntimeError, "cannot create publisher"):
            self.make_publisher()
        self.node.destroy_node.assert_called_once_with()
        self.ros2.executor_core.add_node.assert_not_called()

    def test_qos_failure_does_not_leak_a_ros_node(self):
        self.qos_class.side_effect = RuntimeError("cannot construct QoS")
        with self.assertRaisesRegex(RuntimeError, "cannot construct QoS"):
            self.make_publisher()
        self.node_class.assert_not_called()

    def test_create_timer_failure_destroys_node_and_its_publisher(self):
        self.node.create_timer.side_effect = RuntimeError("cannot create timer")
        with self.assertRaisesRegex(RuntimeError, "cannot create timer"):
            self.make_publisher()
        self.node.destroy_node.assert_called_once_with()
        self.ros2.executor_core.add_node.assert_not_called()

    def test_executor_attach_failure_destroys_created_node(self):
        self.ros2.executor_core.add_node.side_effect = RuntimeError("cannot add node")
        with self.assertRaisesRegex(RuntimeError, "cannot add node"):
            self.make_publisher()
        self.node.destroy_node.assert_called_once_with()

    def test_executor_remove_failure_still_destroys_node(self):
        publisher = self.make_publisher()
        self.ros2.executor_core.remove_node.side_effect = RuntimeError("cannot remove node")
        with self.assertRaisesRegex(RuntimeError, "cannot remove node"):
            publisher.close()
        self.node.destroy_node.assert_called_once_with()
        publisher.close()
        self.node.destroy_node.assert_called_once_with()

    def test_normal_close_removes_and_destroys_node_once(self):
        publisher = self.make_publisher()
        publisher.close()
        publisher.close()
        self.ros2.executor_core.remove_node.assert_called_once_with(self.node)
        self.node.destroy_node.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
