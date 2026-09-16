"""No-hardware contract tests for the AS2W driver cards.

Run with: python3 -m unittest unitree/as2w/test_driver.py
"""
import importlib.util
import math
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch
from pathlib import Path


ROOT = Path(__file__).parent


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _install_device_stubs():
    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.String = type("String", (), {})
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std_msgs
    for name in ("unitree_sdk2py", "unitree_sdk2py.core", "unitree_sdk2py.idl",
                 "unitree_sdk2py.idl.unitree_go", "unitree_sdk2py.idl.unitree_go.msg"):
        sys.modules.setdefault(name, types.ModuleType(name))
    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = type("ChannelSubscriber", (), {})
    sys.modules["unitree_sdk2py.core.channel"] = channel
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_go.msg.dds_")
    dds.LowState_ = type("LowState_", (), {})
    dds.SportModeState_ = type("SportModeState_", (), {})
    sys.modules["unitree_sdk2py.idl.unitree_go.msg.dds_"] = dds


class _Proxy:
    def __init__(self):
        self.moves = []
        self.stops = 0
        self.calls = []

    def Move(self, *args):
        self.moves.append(args)
        return 0

    def StopMove(self):
        self.stops += 1
        return 0

    def Euler(self, *args):
        self.calls.append(("Euler", args))
        return 0

    def SpeedLevel(self, *args):
        self.calls.append(("SpeedLevel", args))
        return 0

    def BodyHeight(self, *args):
        self.calls.append(("BodyHeight", args))
        return 0

    def BodyPosition(self, *args):
        self.calls.append(("BodyPosition", args))
        return 0

    def SwitchGait(self, *args):
        self.calls.append(("SwitchGait", args))
        return 0


class TestDriverContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _install_device_stubs()
        cls.device = _load("as2w_device_under_test", ROOT / "device.py")
        cls.motion = _load("as2w_motion_under_test", ROOT / "motion_tools.py")
        cls.spatial = _load("as2w_spatial_under_test", ROOT / "controlled_spatial.py")
        yaml_stub = types.ModuleType("yaml")
        yaml_stub.safe_load = lambda *_: {}
        rclpy_stub = types.ModuleType("rclpy")
        executors_stub = types.ModuleType("rclpy.executors")
        executors_stub.MultiThreadedExecutor = object
        rclpy_stub.executors = executors_stub
        channel_stub = types.ModuleType("unitree_sdk2py.core.channel")
        channel_stub.ChannelFactoryInitialize = lambda *_: None
        rpc_stub = types.ModuleType("rpc_proxy")
        rpc_stub.RpcProxy = object
        with patch.dict(sys.modules, {
            "yaml": yaml_stub,
            "rclpy": rclpy_stub,
            "rclpy.executors": executors_stub,
            "unitree_sdk2py.core.channel": channel_stub,
            "rpc_proxy": rpc_stub,
        }):
            cls.main = _load("as2w_main_under_test", ROOT / "main.py")

    def test_card_stop_cancels_continuous_move(self):
        proxy = _Proxy()
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        result = plugin.dispatch("move", {"vx": 0.1, "vy": 0, "vyaw": 0, "duration": -1})
        self.assertEqual("running", result["status"])
        stopped = plugin.dispatch("stop", {})
        self.assertEqual("idle", stopped["state"])
        self.assertGreaterEqual(proxy.stops, 1)
        self.assertIsNone(plugin._stop)

    def test_loco_preempts_external_background_motion(self):
        proxy = _Proxy()
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        preemptions = []
        plugin.set_external_motion_stop(lambda: preemptions.append(True))

        result = plugin.dispatch("move", {"vx": 0.1, "vy": 0, "vyaw": 0})

        self.assertEqual(0, result["ret"])
        self.assertEqual([True], preemptions)

    def test_timed_move_returns_action_id_and_notifies_acp(self):
        proxy = _Proxy()
        notifications = []
        executor = self.motion.MotionExecutor(
            proxy, notifier=lambda *args: notifications.append(args),
        )
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        plugin.set_motion_executor(executor)

        started_at = time.monotonic()
        result = plugin.dispatch("timed_move", {
            "vx": 0.1, "vy": 0, "vyaw": 0, "duration": 0.03,
        })

        self.assertLess(time.monotonic() - started_at, 0.2)
        self.assertEqual("running", result["state"])
        self.assertTrue(result["action_id"].startswith("as2w_loco_"))
        deadline = time.monotonic() + 1
        while not notifications and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(1, len(notifications))
        self.assertEqual("completed", notifications[0][1])
        self.assertGreaterEqual(proxy.stops, 2)

    def test_timed_move_cancellation_notifies_acp_once(self):
        proxy = _Proxy()
        notifications = []
        executor = self.motion.MotionExecutor(
            proxy, notifier=lambda *args: notifications.append(args),
        )
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        plugin.set_motion_executor(executor)

        result = plugin.dispatch("timed_move", {
            "vx": 0.1, "vy": 0, "vyaw": 0, "duration": 1.0,
        })
        deadline = time.monotonic() + 1
        while not proxy.moves and time.monotonic() < deadline:
            time.sleep(0.005)
        stopped = plugin.dispatch("stop_move", {})

        self.assertEqual(0, stopped["ret"])
        self.assertEqual(1, len(notifications))
        self.assertEqual(result["action_id"], notifications[0][0])
        self.assertEqual("cancelled", notifications[0][1])
        self.assertEqual("cancelled", notifications[0][2]["state"])
        time.sleep(0.02)
        self.assertEqual(1, len(notifications))

    def test_loco_schema_declares_timed_move_completion(self):
        plugin = self.device.LocoPlugin({}, "test", None, _Proxy())
        completion = plugin.get_tool()["inputSchema"]["x-completion"]
        self.assertEqual(["timed_move"], completion["actions"])
        self.assertEqual(40, completion["timeout"])

    def test_loco_move_sync_and_continuous_modes_do_not_claim_completion(self):
        proxy = _Proxy()
        plugin = self.device.LocoPlugin({}, "test", None, proxy)

        immediate = plugin.dispatch("move", {"vx": 0.1})
        continuous = plugin.dispatch("move", {"vx": 0.1, "duration": -1})
        stopped = plugin.dispatch("move", {"duration": 0})
        finite = plugin.dispatch("move", {"vx": 0.1, "duration": 1})
        missing = plugin.dispatch("timed_move", {"vx": 0.1})

        self.assertNotIn("action_id", immediate)
        self.assertNotIn("action_id", continuous)
        self.assertNotIn("action_id", stopped)
        self.assertEqual("INVALID_ARGUMENT", finite["code"])
        self.assertEqual("INVALID_ARGUMENT", missing["code"])
        plugin.dispatch("stop_move", {})

    def test_loco_rejects_nonfinite_and_out_of_range_controls(self):
        proxy = _Proxy()
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        cases = (
            ("move", {"vx": math.inf}),
            ("euler", {"roll": 0.21}),
            ("euler", {"pitch": math.nan}),
            ("body_height", {"height": 0.31}),
            ("body_position", {"x": 0.21}),
            ("switch_gait", {"level": 1}),
            ("speed_level", {"level": 2}),
        )

        for action, args in cases:
            with self.subTest(action=action, args=args):
                result = plugin.dispatch(action, args)
                self.assertEqual("INVALID_ARGUMENT", result["code"])
        self.assertEqual([], proxy.moves)
        self.assertEqual([], proxy.calls)

    def test_special_actions_are_schema_marked_and_confirmed(self):
        proxy = _Proxy()
        plugin = self.device.SpecialActionPlugin({}, "test", None, proxy)
        schema = plugin.get_tool()["inputSchema"]
        self.assertTrue(schema["x-is-dangerous"])
        self.assertIn("confirm", schema["x-action-params"]["front_flip"]["params"])
        self.assertIn("error", plugin.dispatch("front_flip", {}))

    def test_navigation_declares_completion(self):
        plugin = self.spatial.ControlledSpatialPlugin.__new__(self.spatial.ControlledSpatialPlugin)
        schema = plugin.get_tool()["inputSchema"]
        self.assertIn("navigate_to", schema["x-completion"]["actions"])
        self.assertEqual(180, schema["x-completion"]["timeout"])

    def test_navigation_returns_action_id_without_waiting_for_arrival(self):
        plugin = self.spatial.ControlledSpatialPlugin.__new__(self.spatial.ControlledSpatialPlugin)
        plugin._client = types.SimpleNamespace(call=lambda *_: {"code": 0, "response": "{}"})
        plugin._nav_done = self.spatial.threading.Event()
        plugin._nav_result = None
        plugin._nav_action_id = None
        plugin._nav_lock = self.spatial.threading.Lock()
        with patch.object(self.spatial.threading, "Thread") as thread:
            result = plugin.dispatch("navigate_to", {"x": 1, "y": 2})
        self.assertEqual("navigating", result["status"])
        self.assertTrue(result["action_id"].startswith("as2w_nav_"))
        thread.assert_called_once()

    def test_navigation_rejects_malformed_numeric_arguments(self):
        plugin = self.spatial.ControlledSpatialPlugin.__new__(self.spatial.ControlledSpatialPlugin)
        calls = []
        plugin._client = types.SimpleNamespace(call=lambda *args: calls.append(args))
        cases = (
            ("navigate_to", {"x": "not-a-number"}),
            ("navigate_to", {"x": math.inf}),
            ("navigate_to", {"speed": math.nan}),
            ("navigate_to", {"speed": 0.19}),
            ("navigate_to", {"speed": 1.51}),
            ("navigate_to", {"mode": "1"}),
            ("navigate_to", {"mode": 2}),
            ("init_pose", {"address": "/tmp/map.pcd", "q_w": math.inf}),
        )

        for action, args in cases:
            with self.subTest(action=action, args=args):
                result = plugin.dispatch(action, args)
                self.assertEqual("INVALID_ARGUMENT", result["code"])
        self.assertEqual([], calls)

    def test_bundle_boundary_returns_invalid_argument_for_bad_navigation(self):
        plugin = self.spatial.ControlledSpatialPlugin.__new__(self.spatial.ControlledSpatialPlugin)
        plugin._client = types.SimpleNamespace(call=lambda *_: self.fail("RPC must not be called"))
        bundle = self.main.Bundle.__new__(self.main.Bundle)
        bundle.plugins = [plugin]

        result = bundle.call("controlled_spatial", {
            "action": "navigate_to", "x": "not-a-number",
        })

        self.assertEqual("INVALID_ARGUMENT", result["code"])

    def test_model_resource_is_textual_urdf(self):
        urdf = (ROOT / "resource" / "as2w.urdf").read_text()
        self.assertIn('<robot name="AS2W">', urdf)
        self.assertNotIn("meshes/", urdf)

    def test_sdk_crc_library_is_selected_and_verified_during_image_build(self):
        self.assertEqual([], list((ROOT / "unitree_sdk2py").rglob("crc_*.so")))
        dockerfile = (ROOT / "Dockerfile").read_text()
        self.assertNotIn("raw.githubusercontent.com", dockerfile)
        for expected in (
            "ARG TARGETARCH",
            "cdn.jsdelivr.net/gh/unitreerobotics/unitree_sdk2_python@",
            "crc_amd64.so",
            "crc_aarch64.so",
            "65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5",
            "b136a91a7e99c5cf914f465e131f63c897a17107939184a6f85d1a57cf315ade",
            "a4db103653db78540d141ff4dc55216b83e5298280e639b2ad54a84ac2fae9a5",
            "sha256sum -c -",
        ):
            self.assertIn(expected, dockerfile)

    def test_state_sensor_info_includes_topic(self):
        plugin = self.device.StatePlugin.__new__(self.device.StatePlugin)
        plugin._namespace = "test"
        for name in ("imu", "joints", "joint_state", "battery", "loco_state"):
            result = plugin.dispatch(name, {})
            self.assertEqual("running", result["state"])
            self.assertTrue(result["topic_out"][0]["topic"].startswith("/test/"))

    def test_lowstate_extra_motor_slots_are_ignored(self):
        node = self.device._StateNode.__new__(self.device._StateNode)
        published = []
        node.imu = node.joints = node.joint_state = node.battery = node.remote_controller = types.SimpleNamespace(
            publish=lambda message: published.append(message.data))
        node._last_remote_time = 0.0
        node._last_remote = None
        node._remote_lock = self.device.threading.Lock()
        motors = [types.SimpleNamespace(q=float(i), dq=0, tau_est=0, temperature=0) for i in range(20)]
        imu = types.SimpleNamespace(quaternion=[], gyroscope=[], accelerometer=[], rpy=[])
        node._on_low(types.SimpleNamespace(imu_state=imu, motor_state=motors, bms_state=None))
        self.assertEqual(4, len(published))
        self.assertEqual(16, len(__import__("json").loads(published[1])["joint_states"]))


if __name__ == "__main__":
    unittest.main()
