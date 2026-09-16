"""No-hardware contract tests for the AS2W driver cards.

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
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std_msgs
    for name in ("unitree_sdk2py", "unitree_sdk2py.core", "unitree_sdk2py.idl",
                 "unitree_sdk2py.idl.unitree_go", "unitree_sdk2py.idl.unitree_go.msg",
                 "unitree_sdk2py.idl.unitree_hg", "unitree_sdk2py.idl.unitree_hg.msg"):
        sys.modules.setdefault(name, types.ModuleType(name))
    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = type("ChannelSubscriber", (), {})
    sys.modules["unitree_sdk2py.core.channel"] = channel
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_go.msg.dds_")
    dds.SportModeState_ = type("SportModeState_", (), {})
    sys.modules["unitree_sdk2py.idl.unitree_go.msg.dds_"] = dds
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

    def test_model_resource_is_textual_urdf(self):
        urdf = (ROOT / "resource" / "as2w.urdf").read_text()
        self.assertIn('<robot name="AS2W">', urdf)
        self.assertNotIn("meshes/", urdf)

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
        self.assertEqual(16, len(__import__("json").loads(published[1])["joint_states"]))

    def test_loco_uses_presets_and_acp_completion(self):
        plugin = self.device.LocoPlugin({}, "test", None, _Proxy())
        schema = plugin.get_tool()["inputSchema"]
        self.assertEqual(["slow", "normal", "fast"], schema["properties"]["speed_preset"]["enum"])
        self.assertIn("stand_up", schema["x-completion"]["actions"])
        self.assertNotIn("switch_gait", schema["properties"]["action"]["enum"])

    def test_mcp_supports_sse_and_never_falls_back_to_wifi(self):
        source = (ROOT / "main.py").read_text()
        self.assertIn('parsed.path != "/mcp/sse"', source)
        self.assertIn('"/mcp/messages"', source)
        self.assertIn("must never silently bind to the office Wi-Fi", source)


if __name__ == "__main__":
    unittest.main()
