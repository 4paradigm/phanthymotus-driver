"""No-hardware contract tests for the As2W driver cards.

Run with: python3 -m unittest unitree/as2w/test_driver.py
"""
import importlib.util
import sys
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
    std_msgs.UInt8MultiArray = type("UInt8MultiArray", (), {})
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std_msgs
    qos = types.ModuleType("rclpy.qos")
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.QoSProfile = lambda **kwargs: kwargs
    sys.modules["rclpy.qos"] = qos
    for name in ("unitree_sdk2py", "unitree_sdk2py.core", "unitree_sdk2py.idl",
                 "unitree_sdk2py.idl.unitree_go", "unitree_sdk2py.idl.unitree_go.msg",
                 "unitree_sdk2py.idl.sensor_msgs", "unitree_sdk2py.idl.sensor_msgs.msg",
                 "unitree_sdk2py.idl.unitree_hg", "unitree_sdk2py.idl.unitree_hg.msg"):
        sys.modules.setdefault(name, types.ModuleType(name))
    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = type("ChannelSubscriber", (), {})
    sys.modules["unitree_sdk2py.core.channel"] = channel
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_go.msg.dds_")
    dds.SportModeState_ = type("SportModeState_", (), {})
    sys.modules["unitree_sdk2py.idl.unitree_go.msg.dds_"] = dds
    sensor_dds = types.ModuleType("unitree_sdk2py.idl.sensor_msgs.msg.dds_")
    sensor_dds.PointCloud2_ = type("PointCloud2_", (), {})
    sys.modules["unitree_sdk2py.idl.sensor_msgs.msg.dds_"] = sensor_dds
    hg_dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    hg_dds.LowState_ = type("LowState_", (), {})
    hg_dds.BmsState_ = type("BmsState_", (), {})
    sys.modules["unitree_sdk2py.idl.unitree_hg.msg.dds_"] = hg_dds


class _Proxy:
    def __init__(self):
        self.moves = []
        self.stops = 0

    def Move(self, *args):
        self.moves.append(args)
        return 0

    def StopMove(self):
        self.stops += 1
        return 0


class TestDriverContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _install_device_stubs()
        cls.device = _load("as2w_device_under_test", ROOT / "device.py")
        cls.spatial = _load("as2w_spatial_under_test", ROOT / "controlled_spatial.py")

    def test_card_stop_cancels_continuous_move(self):
        proxy = _Proxy()
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        result = plugin.dispatch("move", {"vx": 0.1, "vy": 0, "vyaw": 0, "duration": -1})
        self.assertEqual("running", result["status"])
        stopped = plugin.dispatch("stop", {})
        self.assertEqual("idle", stopped["state"])
        self.assertGreaterEqual(proxy.stops, 1)
        self.assertIsNone(plugin._stop)

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

    def test_navigation_buffers_task_result_arriving_during_rpc(self):
        plugin = self.spatial.ControlledSpatialPlugin.__new__(self.spatial.ControlledSpatialPlugin)
        plugin._nav_done = self.spatial.threading.Event()
        plugin._nav_result = None
        plugin._nav_action_id = None
        plugin._nav_lock = self.spatial.threading.Lock()
        def call(action, data):
            plugin._on_slam_key_info(types.SimpleNamespace(data='{"type":"task_result","errorCode":0,"data":{"is_arrived":true}}'))
            return {"code": 0, "response": "{}"}
        plugin._client = types.SimpleNamespace(call=call)
        with patch.object(self.spatial.threading, "Thread") as thread:
            result = plugin.dispatch("navigate_to", {"x": 1, "y": 2})
        self.assertTrue(result["action_id"].startswith("as2w_nav_"))
        self.assertTrue(plugin._nav_done.is_set())
        self.assertTrue(plugin._nav_result["data"]["is_arrived"])
        thread.assert_called_once()

    def test_model_resource_is_textual_urdf(self):
        urdf = (ROOT / "resource" / "as2w.urdf").read_text()
        self.assertIn('<robot name="As2W">', urdf)
        self.assertNotIn("meshes/", urdf)
        for name in ("FL_foot", "FR_foot", "RL_foot", "RR_foot"):
            self.assertIn(f'<joint name="{name}" type="continuous">', urdf)

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
        node.imu = node.joints = node.joint_state = node.battery = types.SimpleNamespace(
            publish=lambda message: published.append(message.data))
        motors = [types.SimpleNamespace(q=float(i), dq=0, tau_est=0, temperature=0) for i in range(20)]
        imu = types.SimpleNamespace(quaternion=[], gyroscope=[], accelerometer=[], rpy=[])
        node._on_low(types.SimpleNamespace(imu_state=imu, motor_state=motors, bms_state=None))
        self.assertEqual(3, len(published))
        joint_state = __import__("json").loads(published[1])
        self.assertEqual(16, len([key for key in joint_state if key.endswith("_q")]))
        self.assertIn("FR_hip_q", joint_state)

    def test_joints_payload_keeps_skeleton_contract(self):
        node = self.device._StateNode.__new__(self.device._StateNode)
        published = []
        node.imu = node.joint_state = node.battery = types.SimpleNamespace(publish=lambda message: None)
        node.joints = types.SimpleNamespace(publish=lambda message: published.append(message.data))
        motors = [types.SimpleNamespace(q=float(i), dq=0, tau_est=0, temperature=[0, 0]) for i in range(16)]
        imu = types.SimpleNamespace(quaternion=[1, 0, 0, 0], gyroscope=[], accelerometer=[], rpy=[])
        node._on_low(types.SimpleNamespace(imu_state=imu, motor_state=motors))
        payload = __import__("json").loads(published[0])
        self.assertEqual({"joints", "imu_quat"}, set(payload))
        self.assertEqual(16, len(payload["joints"]))
        self.assertEqual([1, 0, 0, 0], payload["imu_quat"])
        self.assertEqual({"idx", "name", "q", "dq", "tau", "temperature"},
                         set(payload["joints"][0]))

    def test_battery_current_is_explicitly_exposed_in_ma_and_a(self):
        node = self.device._StateNode.__new__(self.device._StateNode)
        published = []
        node.battery = types.SimpleNamespace(publish=lambda message: published.append(message.data))
        node._on_bms(types.SimpleNamespace(soc=87, current=325, cycle=4, temperature=[]))
        payload = __import__("json").loads(published[0])
        self.assertEqual(325, payload["current_ma"])
        self.assertNotIn("current", payload)
        self.assertNotIn("current_a", payload)

    def test_loco_state_does_not_duplicate_imu(self):
        node = self.device._StateNode.__new__(self.device._StateNode)
        published = []
        node.loco = types.SimpleNamespace(publish=lambda message: published.append(message.data))
        node._on_sport(types.SimpleNamespace(mode=2, velocity=[1, 2, 3], position=[4, 5, 6], body_height=0.2,
                                              imu_state=types.SimpleNamespace(rpy=[7, 8, 9])))
        self.assertNotIn("imu_rpy_0", __import__("json").loads(published[0]))

    def test_loco_uses_presets_and_acp_completion(self):
        plugin = self.device.LocoPlugin({}, "test", None, _Proxy())
        schema = plugin.get_tool()["inputSchema"]
        self.assertEqual(["slow", "normal", "fast"], schema["properties"]["speed_preset"]["enum"])
        self.assertIn("stand_up", schema["x-completion"]["actions"])
        self.assertNotIn("switch_gait", schema["properties"]["action"]["enum"])

    def test_state_stop_then_start_recreates_shared_node(self):
        plugin = self.device.StatePlugin.__new__(self.device.StatePlugin)
        old_state = types.SimpleNamespace(close=lambda: setattr(plugin, "closed", True))
        plugin._namespace = "test"
        plugin._executor = object()
        plugin._state = old_state
        plugin.closed = False
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
        self.assertTrue(plugin.closed)
        self.assertIsNone(plugin._state)
        replacement = object()
        with patch.object(self.device, "_StateNode", return_value=replacement) as node:
            self.assertEqual("running", plugin.dispatch("start", {})["state"])
        node.assert_called_once_with("test", plugin._executor)
        self.assertIs(replacement, plugin._state)

    def test_lidar_stop_then_start_recreates_node(self):
        lidar = _load("as2w_lidar_lifecycle_test", ROOT / "lidar.py")
        plugin = lidar.LidarPlugin.__new__(lidar.LidarPlugin)
        plugin.topic = "/test/lidar/cloud"
        plugin._executor = object()
        plugin._config = {"source_topics": ["rt/test"]}
        plugin.node = types.SimpleNamespace(close=lambda: setattr(plugin, "closed", True))
        plugin.closed = False
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
        self.assertTrue(plugin.closed)
        self.assertIsNone(plugin.node)
        replacement = object()
        with patch.object(lidar, "_LidarNode", return_value=replacement) as node:
            self.assertEqual("running", plugin.dispatch("start", {})["state"])
        node.assert_called_once_with("/test/lidar/cloud", plugin._executor, ["rt/test"])
        self.assertIs(replacement, plugin.node)

    def test_mcp_supports_sse_and_never_falls_back_to_wifi(self):
        source = (ROOT / "main.py").read_text()
        self.assertIn('parsed.path != "/mcp/sse"', source)
        self.assertIn('"/mcp/messages"', source)
        self.assertIn("must never silently bind to the office Wi-Fi", source)

    def test_lidar_uses_direct_sensor_topics_not_conditional_slam_clouds(self):
        source = (ROOT / "lidar.py").read_text()
        self.assertIn('"rt/utlidar/cloud_deskewed"', source)
        self.assertIn('"rt/utlidar/cloud"', source)
        self.assertNotIn('"rt/unitree/slam_mapping/points"', source)

    def test_lidar_normalizes_pointcloud_fields_for_renderer(self):
        import struct
        node = self.device  # keep the test module's SDK stubs loaded
        del node
        lidar = _load("as2w_lidar_under_test", ROOT / "lidar.py")
        raw = b"\x00\x00\x00\x00" + struct.pack("<fff", 1.0, 2.0, 3.0) + b"\x00\x00\x00\x00"
        normalized = lidar._LidarNode._to_xyz(raw, 20, 1,
                                               {"x": 4, "y": 8, "z": 12}, False)
        x, y, z = struct.unpack("<fff", normalized)
        self.assertAlmostEqual(1.308, x, places=2)
        self.assertAlmostEqual(2.879, y, places=2)
        self.assertAlmostEqual(-2.0, z, places=3)

    def test_lidar_normalizes_big_endian_xyz(self):
        import struct
        lidar = _load("as2w_lidar_endian_test", ROOT / "lidar.py")
        raw = struct.pack(">fff", 1.0, -2.0, 3.0)
        normalized = lidar._LidarNode._to_xyz(raw, 12, 1,
                                               {"x": 0, "y": 4, "z": 8}, True)
        x, y, z = struct.unpack("<fff", normalized)
        self.assertAlmostEqual(1.308, x, places=2)
        self.assertAlmostEqual(2.879, y, places=2)
        self.assertAlmostEqual(2.0, z, places=3)

    def test_lidar_applies_as2w_jt128_mount_rotation(self):
        import struct
        lidar = _load("as2w_lidar_mount_test", ROOT / "lidar.py")
        raw = struct.pack("<fff", 1.0, 0.0, 0.0)
        normalized = lidar._LidarNode._to_xyz(raw, 12, 1,
                                               {"x": 0, "y": 4, "z": 8}, False)
        x, y, z = struct.unpack("<fff", normalized)
        self.assertAlmostEqual(0.9945, x, places=3)
        self.assertAlmostEqual(-0.1045, y, places=3)
        self.assertAlmostEqual(0.0, z, places=3)


if __name__ == "__main__":
    unittest.main()
