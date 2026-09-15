from __future__ import annotations

import importlib.util
import sys
import time
import types
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "pndbotics" / "adam"


def load_device():
    sys.path.insert(0, str(DRIVER))
    try:
        sys.modules.pop("device", None)
        spec = importlib.util.spec_from_file_location("adam_device", DRIVER / "device.py")
        module = importlib.util.module_from_spec(spec)
        assert spec.loader
        numpy = types.ModuleType("numpy")
        numpy.float64 = float
        with mock.patch.dict(sys.modules, {"numpy": numpy}):
            spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(DRIVER))


adam = load_device()


class FakeSubscriber:
    def Close(self):
        pass


class AdamHandStatePluginTests(unittest.TestCase):
    def _plugin(self, payload):
        cache = mock.Mock()
        cache.snapshot.return_value = payload
        cache.status.return_value = {"reader_available": True}
        node = types.SimpleNamespace(_topic="/adam/state/hand", set_active=mock.Mock())
        with mock.patch.object(adam, "_HandStatePublisherNode", return_value=node):
            return adam.HandStatePlugin({}, "adam", mock.Mock(), cache)

    def test_info_includes_topic_for_fresh_sample(self):
        result = self._plugin({"position": [0] * 12, "fresh": True}).dispatch("info", {})
        self.assertEqual([{"topic": "/adam/state/hand", "format": "data/json"}], result["topic_out"])
        self.assertEqual([0] * 12, result["position"])

    def test_info_includes_topic_when_waiting_for_sample(self):
        result = self._plugin(None).dispatch("info", {})
        self.assertEqual("waiting", result["state"])
        self.assertEqual([{"topic": "/adam/state/hand", "format": "data/json"}], result["topic_out"])


class AdamDriverContractTests(unittest.TestCase):
    def test_health_payload_reports_transport_and_control_topology(self):
        state = type("State", (), {"mode_pr": 0, "motor_state": [object()] * 31, "tick": 123})()
        payload = adam._state_health_payload(state, time.monotonic())
        self.assertEqual("online", payload["status"])
        self.assertEqual("PR 串联关节控制", payload["control_topology"])
        self.assertEqual(31, payload["body_joint_count"])
        self.assertEqual(123, payload["tick"])

    def test_hand_cache_status_distinguishes_waiting_and_stale(self):
        cache = adam.HandStateCache(FakeSubscriber())
        self.assertEqual("waiting", adam._hand_status_payload(cache, 1.0)["state"])
        with cache._lock:
            cache._latest_position = [100] * 12
            cache._received_at_ms = int(time.time() * 1000) - 2000
            cache._received_monotonic = time.monotonic() - 2
        stale = cache.snapshot(timeout_sec=1.0)
        self.assertFalse(stale["fresh"])
        cache.close()

    def test_hand_state_payload_exposes_left_and_right_channels(self):
        payload = adam._hand_state_payload(list(range(12)), int(time.time() * 1000), fresh=True)
        self.assertEqual([0, 1, 2, 3, 4, 5], payload["left"]["position"])
        self.assertEqual([6, 7, 8, 9, 10, 11], payload["right"]["position"])
        self.assertEqual(1000, payload["position_max"])

    def test_skeleton_payload_contains_only_urdf_joints(self):
        state = type("State", (), {"motor_state": [type("Motor", (), {"q": 0.1})() for _ in range(31)]})()
        payload = adam._skeleton_payload(state, adam.ADAM_PRO_JOINTS)
        self.assertEqual(31, len(payload["joints"]))
        self.assertEqual(list(adam.ADAM_PRO_JOINTS), [joint["name"] for joint in payload["joints"]])
        self.assertTrue(all(joint["unit"] == "rad" for joint in payload["joints"]))

    def test_arm_gesture_uses_base_arm_control(self):
        arm = mock.Mock()
        arm.dispatch.return_value = {"state": "active", "joints_set": 7}
        gesture = adam.ArmGesturePlugin(arm)
        result = gesture.dispatch("raise_hand", {"side": "left"})
        self.assertEqual([
            mock.call("set_joints", {"joints": adam.ARM_RAISE_POSE["left"]}),
            mock.call("enable", {}),
        ], arm.dispatch.call_args_list)
        self.assertEqual("raise_hand", result["gesture"])
        self.assertEqual("left", result["side"])
        self.assertNotIn("gesture", adam.ArmPlugin.get_tool(object.__new__(adam.ArmPlugin))["inputSchema"]["properties"]["action"]["enum"])
        self.assertEqual("error", gesture.dispatch("raise_hand", {"side": "both"})["state"])

    def test_hand_gesture_shares_control_but_is_not_on_base_tool(self):
        hand = object.__new__(adam.HandPlugin)
        hand._open_positions = [1000] * 12
        hand._close_positions = [0] * 12
        hand._thumb_close_positions = [100, 900, 200, 800]
        hand._thumb_close_min_flex_position = 100
        hand._max_val = 1000
        hand._base_positions = lambda: [500] * 12
        hand._activate = mock.Mock(return_value={"state": "active"})
        gesture = adam.HandGesturePlugin(hand)
        self.assertNotIn("point", hand.get_tool()["inputSchema"]["properties"]["action"]["enum"])
        result = gesture.dispatch("point", {"side": "right"})
        self.assertEqual("active", result["state"])
        self.assertEqual([500] * 6, hand._activate.call_args.args[0][:6])
        self.assertEqual("error", gesture.dispatch("point", {"side": "both"})["state"])

    def test_bundle_registers_gesture_tools_separately(self):
        config = {
            "plugins": {
                "state": {"enabled": False},
                "estop": {"enabled": False},
                "loco": {"enabled": False},
                "camera": {"enabled": False},
                "vision_capture": {"enabled": False},
                "arm": {"enabled": True},
                "hand": {"enabled": True},
                "hand_state": {"enabled": False},
                "model": {"enabled": False},
            },
        }
        arm = types.SimpleNamespace(
            get_tool=lambda: {"name": "arm"}, dispatch=mock.Mock(),
        )
        hand = types.SimpleNamespace(
            get_tool=lambda: {"name": "hand"}, dispatch=mock.Mock(),
        )
        with mock.patch.object(adam, "HAS_ROS2", True), \
                mock.patch.object(adam, "ArmPlugin", return_value=arm), \
                mock.patch.object(adam, "HandPlugin", return_value=hand):
            bundle = adam.AdamDeviceBundle(
                config, "adam", mock.Mock(), mock.Mock(), ros2_enabled=True,
            )
        tools = {tool["name"] for tool in bundle.get_all_tools()}
        self.assertEqual({"arm", "arm_gesture", "hand", "hand_gesture"}, tools)

    def test_hand_gestures_only_change_the_selected_hand(self):
        plugin = object.__new__(adam.HandPlugin)
        plugin._open_positions = [1000] * 12
        plugin._close_positions = [0] * 12
        plugin._thumb_close_positions = [100, 900, 200, 800]
        plugin._thumb_close_min_flex_position = 100
        plugin._base_positions = lambda: [500] * 12
        self.assertEqual([0, 0, 0, 0, 1000, 1000], plugin._gesture_target("thumbs_up", "left")[:6])
        self.assertEqual([0, 0, 0, 1000, 200, 800], plugin._gesture_target("point", "right")[6:])
        self.assertEqual([0, 0, 1000, 1000, 100, 900], plugin._gesture_target("victory", "left")[:6])
        self.assertEqual([1000, 0, 0, 1000, 200, 800], plugin._gesture_target("rock", "right")[6:])

    def test_state_plugin_tool_contracts(self):
        node = types.SimpleNamespace(
            _topic_skeleton="/adam/state/joints", _topic_motor_state="/adam/state/motors",
            _topic_robot_state="/adam/state/robot", _topic_imu="/adam/state/imu",
            _topic_battery="/adam/state/battery", _topic_health="/adam/state/health",
            _topic_hand="/adam/state/hands",
        )
        plugin = object.__new__(adam.StatePlugin)
        plugin._node = node
        plugin._running = False
        tools = {tool["name"]: tool for tool in plugin.get_tools()}
        self.assertEqual([{"topic": "/adam/state/motors", "format": "data/json"}], tools["motor_state"]["topic_out"])
        self.assertEqual([{"topic": "/adam/state/robot", "format": "data/json"}], tools["robot_state"]["topic_out"])
        self.assertIn("health", tools)
        self.assertIn("hands", tools)

    def test_battery_payload_retains_bms_values_and_lowstate_source(self):
        battery = types.SimpleNamespace(voltage=48.2, current=3.4, power=163.9, wh_accumulated=120.0, status="normal")
        result = adam._battery_payload(battery, 123)
        self.assertEqual(123, result["timestamp_ms"])
        self.assertEqual("rt/lowstate", result["source_topic"])
        self.assertEqual(48.2, result["voltage"])
        self.assertEqual("normal", result["status"])


if __name__ == "__main__":
    unittest.main()
