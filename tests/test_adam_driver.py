import importlib.util
import sys
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
        with mock.patch.dict(sys.modules, {"numpy": mock.Mock()}):
            spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(DRIVER))


class AdamHandStatePluginTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = load_device()

    def _plugin(self, payload):
        cache = mock.Mock()
        cache.snapshot.return_value = payload
        cache.status.return_value = {"reader_available": True}
        node = types.SimpleNamespace(_topic="/adam/state/hand", set_active=mock.Mock())
        with mock.patch.object(self.device, "_HandStatePublisherNode", return_value=node):
            plugin = self.device.HandStatePlugin({}, "adam", mock.Mock(), cache)
        return plugin

    def test_info_includes_topic_for_fresh_sample(self):
        plugin = self._plugin({"position": [0] * 12, "fresh": True})

        result = plugin.dispatch("info", {})

        self.assertEqual([{"topic": "/adam/state/hand", "format": "data/json"}], result["topic_out"])
        self.assertEqual([0] * 12, result["position"])

    def test_info_includes_topic_when_waiting_for_sample(self):
        plugin = self._plugin(None)

        result = plugin.dispatch("info", {})

        self.assertEqual("waiting", result["state"])
        self.assertEqual([{"topic": "/adam/state/hand", "format": "data/json"}], result["topic_out"])


class AdamStatePluginTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = load_device()

    def _plugin(self):
        node = types.SimpleNamespace(
            _topic_skeleton="/adam/state/joints",
            _topic_motor_state="/adam/state/motors",
            _topic_robot_state="/adam/state/robot",
            _topic_imu="/adam/state/imu",
            _topic_battery="/adam/state/battery",
            set_active=mock.Mock(),
        )
        with mock.patch.object(self.device, "_StatePublisherNode", return_value=node):
            return self.device.StatePlugin({}, "adam", mock.Mock(), "pro")

    def test_motor_and_robot_state_tool_contracts(self):
        plugin = self._plugin()
        tools = {tool["name"]: tool for tool in plugin.get_tools()}

        self.assertEqual(
            [{"topic": "/adam/state/motors", "format": "data/json"}],
            tools["motor_state"]["topic_out"],
        )
        self.assertEqual(
            [{"topic": "/adam/state/robot", "format": "data/json"}],
            tools["robot_state"]["topic_out"],
        )
        self.assertEqual(
            [{"topic": "/adam/state/motors", "format": "data/json"}],
            plugin.dispatch("info", {"_tool_name": "motor_state"})["topic_out"],
        )
        self.assertEqual(
            [{"topic": "/adam/state/robot", "format": "data/json"}],
            plugin.dispatch("info", {"_tool_name": "robot_state"})["topic_out"],
        )


if __name__ == "__main__":
    unittest.main()
