"""Franka camera selection and lifecycle, without ROS, USB or camera SDKs."""

from __future__ import annotations

import copy
import sys
import threading
import types
import unittest
from unittest import mock

from common.camera_info import parse
from common.vendor_runtime import DriverBundle
from franka.fr3.camera import ExtCameraPlugin
from franka.fr3 import camera_backend as shared_camera


def camera_device(serial):
    return {
        "path": f"realsense://{serial}",
        "name": "Intel RealSense D435",
        "serial_number": serial,
        "usb_type": "3.2",
        "realsense": True,
        "channels": ["rgb", "depth", "infrared"],
    }


class FakeSession:
    """Emulate per-camera ownership; images are covered by backend tests."""

    instances = []
    offline = set()

    def __init__(self, namespace, serial_number):
        self.namespace = namespace
        self.serial_number = serial_number
        self.routes = {}
        self.start_calls = []
        self.stop_calls = []
        self.instances.append(self)

    def start(self, instance_id, channel):
        self.routes[instance_id] = channel
        self.start_calls.append((instance_id, channel))
        return self.info(instance_id, channel)

    def stop(self, instance_id):
        self.stop_calls.append(instance_id)
        channel = self.routes.pop(instance_id, "rgb")
        return self.info(instance_id, channel)

    def info(self, instance_id, channel):
        active = instance_id in self.routes
        fresh = active and self.serial_number not in self.offline
        return {
            "state": "running" if fresh else "starting" if active else "idle",
            "fresh": fresh,
            "serial_number": self.serial_number,
            "channel": channel,
            "width": 1280 if channel == "rgb" else 640,
            "height": 720 if channel == "rgb" else 480,
            "fps": 15,
            "error": None,
        }


class OtherCard:
    def __init__(self, name):
        self.name = name
        self.calls = []

    def get_tools(self):
        return [{"name": self.name, "type": "sensor"}]

    def dispatch(self, action, args):
        self.calls.append((action, copy.deepcopy(args)))
        return {"state": "idle", "name": self.name}


class FrankaExtCameraTests(unittest.TestCase):
    def setUp(self):
        self.devices = [camera_device("1234"), camera_device("5678")]
        self.enumeration = mock.patch.object(
            shared_camera, "_enumerate_ext_cameras", side_effect=lambda: list(self.devices)
        )
        self.discover = self.enumeration.start()
        self.addCleanup(self.enumeration.stop)
        FakeSession.instances = []
        FakeSession.offline = set()
        self.backend = mock.patch.dict(sys.modules, {
            "franka.fr3.realsense": types.SimpleNamespace(
                RealSenseSession=FakeSession
            ),
        })
        self.backend.start()
        self.addCleanup(self.backend.stop)
        self.plugin = ExtCameraPlugin({}, "franka", None)
        self.addCleanup(self.plugin.stop)
        self.bundle = DriverBundle([self.plugin])

    def call(self, action="info", instance_id="camera-a", **args):
        return self.bundle.dispatch("ext_camera", {
            "action": action, "instance_id": instance_id, **args,
        })

    def start(self, instance_id="camera-a", serial="1234", channel="rgb"):
        return self.call("start", instance_id,
                         device_path=f"realsense://{serial}", channel=channel)

    def session(self, serial="1234"):
        return next(session for session in FakeSession.instances
                    if session.serial_number == serial)

    def test_schema_exposes_independent_camera_and_channel_choices(self):
        tool = self.plugin.get_tools()[0]
        self.assertEqual(tool["name"], "ext_camera")
        self.assertEqual(tool["type"], "sensor")
        self.assertTrue(tool["multiInstance"])
        schema = tool["configSchema"]
        self.assertIn("device_path", schema["required"])
        props = schema["properties"]
        for field in ("device_path", "channel"):
            self.assertEqual(props[field]["scope"], "instance")
        self.assertTrue(props["device_path"]["x-sensitive"])
        self.assertEqual(props["device_path"]["title"], "相机")
        self.assertEqual(props["channel"]["title"], "通道")
        self.assertEqual(props["channel"]["enum"], ["rgb", "depth", "infrared"])
        self.assertEqual(props["channel"]["default"], "rgb")
        self.assertEqual({option["const"] for option in props["device_path"]["oneOf"]},
                         {"realsense://1234", "realsense://5678"})
        self.assertFalse(FakeSession.instances)

    def test_config_saves_selection_without_starting_a_camera(self):
        result = self.call("config", device_path="realsense://5678", channel="depth")
        self.assertEqual(result["state"], "idle")
        self.assertEqual(result["device_path"], "realsense://5678")
        self.assertEqual(result["channel"], "depth")
        self.assertEqual(result["topic_out"], [{
            "topic": "/franka/ext_camera/camera_a/depth", "format": "image/depth-zlib",
        }])
        self.assertFalse(FakeSession.instances)
        self.call("start")
        self.assertEqual(self.session("5678").routes, {"camera-a": "depth"})

    def test_two_cameras_bind_to_their_own_serials(self):
        self.start("camera-a", "5678", "infrared")
        self.start("camera-b", "1234", "rgb")
        self.assertEqual(self.session("5678").routes, {"camera-a": "infrared"})
        self.assertEqual(self.session("1234").routes, {"camera-b": "rgb"})
        self.assertEqual(len(FakeSession.instances), 2)
        self.devices.reverse()
        self.call("start", "camera-a")
        self.assertEqual(self.call(instance_id="camera-a")["device_path"],
                         "realsense://5678")

    def test_stopping_one_card_leaves_other_routes_and_cameras_running(self):
        self.start("rgb-a")
        self.start("depth-a", channel="depth")
        self.start("rgb-b", serial="5678")
        result = self.call("stop", "rgb-a")
        self.assertEqual(result["state"], "idle")
        self.assertEqual(self.session().routes, {"depth-a": "depth"})
        self.assertEqual(self.session("5678").routes, {"rgb-b": "rgb"})
        self.assertEqual(self.call(instance_id="depth-a")["state"], "running")

    def test_running_channel_change_replaces_only_this_route(self):
        self.start()
        self.start("other")
        result = self.call("config", channel="depth")
        self.assertEqual(result["channel"], "depth")
        self.assertEqual(result["state"], "running")
        self.assertEqual(self.session().routes, {"camera-a": "depth", "other": "rgb"})
        self.assertEqual(self.session().stop_calls, [])

    def test_rapid_switch_keeps_latest_selection_when_it_matches_current(self):
        self.start()
        result = self._hold_discovery_during("config", {"channel": "depth"},
                                             lambda: self.call("config", channel="rgb"))
        self.assertEqual(result["channel"], "rgb")
        self.assertEqual(self.session().routes, {"camera-a": "rgb"})
        self.assertFalse(self.session().stop_calls)

    def test_running_camera_change_moves_only_this_card(self):
        self.start()
        self.start("other")
        result = self.call("config", device_path="realsense://5678", channel="infrared")
        self.assertEqual(result["state"], "running")
        self.assertEqual(self.session().routes, {"other": "rgb"})
        self.assertEqual(self.session("5678").routes, {"camera-a": "infrared"})

    def test_disconnected_selection_does_not_fall_back_to_another_camera(self):
        self.call("config", device_path="realsense://1234")
        self.devices[:] = [camera_device("5678")]
        FakeSession.offline.add("1234")
        result = self.call("start")
        self.assertEqual(result["serial_number"], "1234")
        self.assertFalse(result["fresh"])
        self.assertEqual([session.serial_number for session in FakeSession.instances], ["1234"])

    def test_ids_that_collapse_to_same_dds_topic_are_rejected(self):
        self.start("camera-a")
        with self.assertRaises(ValueError):
            self.start("camera_a", serial="5678")
        self.assertEqual(len(FakeSession.instances), 1)

    def test_info_declares_camera_geometry_without_invented_calibration(self):
        self.call("config", device_path="realsense://1234", channel="depth")
        for active in (False, True):
            with self.subTest(active=active):
                if active:
                    self.call("start")
                info = self.call()
                self.assertEqual(len(info["camera_info"]), 1)
                descriptor = info["camera_info"][0]
                parsed = parse(descriptor)
                self.assertEqual(parsed.topic, info["topic_out"][0]["topic"])
                self.assertEqual(parsed.format, "image/depth-zlib")
                self.assertEqual(parsed.source, "unknown")
                self.assertIsNone(parsed.K)
                self.assertIsNone(parsed.D)
                self.assertIsNone(parsed.half_fov_rad)
                self.assertIsNone(parsed.half_fov_v_rad)

    def test_missing_sdk_leaves_camera_listing_and_other_cards_available(self):
        with mock.patch.dict(sys.modules, {"pyrealsense2": None}), mock.patch.object(
            shared_camera, "_enumerate_ext_cameras", wraps=self.real_enumeration
        ):
            plugin = ExtCameraPlugin({}, "franka", None)
            self.addCleanup(plugin.stop)
            other = OtherCard("master_states")
            bundle = DriverBundle([plugin, other])
            tools = bundle.get_all_tools()
            self.assertEqual({tool["name"] for tool in tools}, {"ext_camera", "master_states"})
            self.assertEqual(bundle.dispatch("master_states", {"action": "info"})["state"], "idle")
        self.assertFalse(FakeSession.instances)

    def _hold_discovery_during(self, action, arguments, concurrent_change):
        entered, release = threading.Event(), threading.Event()
        outcomes, failures = [], []

        def enumerate_blocked():
            if threading.current_thread().name == "stale-camera-request":
                entered.set()
                if not release.wait(2):
                    raise RuntimeError("test enumeration was not released")
            return list(self.devices)

        def pending_request():
            try:
                outcomes.append(self.plugin.dispatch(action, {
                    "instance_id": "camera-a", **arguments,
                }))
            except Exception as exc:
                failures.append(exc)

        self.discover.side_effect = enumerate_blocked
        pending = threading.Thread(target=pending_request, name="stale-camera-request")
        pending.start()
        try:
            self.assertTrue(entered.wait(1), "pending request did not reach discovery")
            concurrent_change()
        finally:
            release.set()
            pending.join(2)
            self.discover.side_effect = lambda: list(self.devices)
        self.assertFalse(pending.is_alive())
        self.assertFalse(failures, failures)
        self.assertEqual(len(outcomes), 1)
        return outcomes[0]

    def test_stop_cancels_start_blocked_in_device_enumeration(self):
        self.call("config", device_path="realsense://1234")
        entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
        outcomes, failures = [], []

        def enumerate_blocked():
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test enumeration was not released")
            return list(self.devices)

        def start_card():
            try:
                outcomes.append(self.plugin.dispatch("start", {"instance_id": "camera-a"}))
            except Exception as exc:
                failures.append(exc)

        def stop_card():
            try:
                self.plugin.dispatch("stop", {"instance_id": "camera-a"})
            except Exception as exc:
                failures.append(exc)
            finally:
                stopped.set()

        self.discover.side_effect = enumerate_blocked
        starter = threading.Thread(target=start_card)
        stopper = threading.Thread(target=stop_card)
        starter.start()
        try:
            self.assertTrue(entered.wait(1), "start did not reach discovery")
            stopper.start()
            self.assertTrue(stopped.wait(0.5), "stop was blocked behind discovery")
        finally:
            release.set()
            starter.join(2)
            if stopper.ident is not None:
                stopper.join(2)
            self.discover.side_effect = lambda: list(self.devices)
        self.assertFalse(starter.is_alive())
        self.assertFalse(stopper.is_alive())
        self.assertFalse(failures, failures)
        self.assertEqual(self.call()["state"], "idle")
        self.assertTrue(all(not session.routes for session in FakeSession.instances))


# Retain the real lazy enumerator before each test replaces it with fake USB data.
FrankaExtCameraTests.real_enumeration = staticmethod(shared_camera._enumerate_ext_cameras)


if __name__ == "__main__":
    unittest.main()

class CameraFormatDeclarationTests(unittest.TestCase):
    def test_declares_one_dynamic_port_with_exact_instance_format(self):
        with mock.patch.object(shared_camera, '_enumerate_ext_cameras', return_value=[]):
            plugin = ExtCameraPlugin({}, 'franka', None)
            definition = plugin.get_tools()[0]
            self.assertEqual(len(definition['topic_out']), 1)
            self.assertNotIn('format', definition['topic_out'][0])
            for channel in ('rgb', 'depth', 'infrared'):
                result = plugin.dispatch('config', {
                    'instance_id': 'format_' + channel,
                    'device_path': 'realsense://263022075182', 'channel': channel})
                self.assertEqual(len(result['topic_out']), 1)
                self.assertEqual(result['topic_out'][0]['format'],
                                 shared_camera.TOPIC_FORMATS[channel])
            plugin.stop()
