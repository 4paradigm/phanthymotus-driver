"""Contract tests for the Go1 motor_health sensor card.

The card deliberately stays read-only and converts the existing joint
temperature snapshot into data/json so it can connect to Agent Core.
"""

import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
GO1_DIR = ROOT / "unitree/go1"
sys.path.insert(0, str(GO1_DIR))

import sensors  # noqa: E402


def joint_snapshot(temperatures, *, fresh=True):
    return {
        "fresh": fresh,
        "control_level": "HIGHLEVEL",
        "joints": [
            {"i": index, "temp": temperature}
            for index, temperature in enumerate(temperatures)
        ],
    }


class MotorHealthTests(unittest.TestCase):
    def test_ok_payload_is_machine_readable_and_complete(self):
        payload = sensors._build_motor_health(
            joint_snapshot([40 + index for index in range(12)]), 60.0, 70.0)

        self.assertTrue(payload["available"])
        self.assertTrue(payload["telemetry_valid"])
        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["joint_count"], 12)
        self.assertEqual(payload["max_temperature_c"], 51.0)
        self.assertEqual(payload["hottest_joint"], "RL_calf_joint")
        self.assertEqual(payload["warning_joints"], [])
        self.assertEqual(payload["critical_joints"], [])
        self.assertEqual(payload["recommendation"], "continue_monitoring")

    def test_warning_and_critical_joints_are_named(self):
        payload = sensors._build_motor_health(
            joint_snapshot([45, 61, 50, 71, 48, 49, 50, 51, 52, 53, 54, 55]),
            60.0,
            70.0,
        )

        self.assertEqual(payload["status"], "critical")
        self.assertEqual(payload["hottest_joint"], "FL_hip_joint")
        self.assertEqual(
            [item["name"] for item in payload["warning_joints"]], ["FR_thigh_joint"])
        self.assertEqual(
            [item["name"] for item in payload["critical_joints"]], ["FL_hip_joint"])
        self.assertEqual(payload["recommendation"], "stop_and_cool_down")

    def test_missing_zero_filled_or_stale_data_never_passes(self):
        cases = [
            ({"fresh": True}, "joint_data_missing"),
            (joint_snapshot([0] * 12), "zero_filled_joint_temperatures"),
            (joint_snapshot([45] * 12, fresh=False), "stale_snapshot"),
        ]
        for snapshot, reason in cases:
            with self.subTest(reason=reason):
                payload = sensors._build_motor_health(snapshot, 60.0, 70.0)
                self.assertFalse(payload["telemetry_valid"])
                self.assertEqual(payload["status"], "unavailable")
                self.assertEqual(payload["reason"], reason)

    def test_partial_joint_data_warns_instead_of_passing(self):
        payload = sensors._build_motor_health(
            joint_snapshot([45] * 11 + [0]), 60.0, 70.0)

        self.assertFalse(payload["telemetry_valid"])
        self.assertEqual(payload["status"], "warning")
        self.assertEqual(payload["invalid_joints"], ["RL_calf_joint"])
        self.assertEqual(payload["recommendation"], "check_missing_telemetry")

    def test_plugin_contract_is_data_json_and_dispatches_read_only(self):
        class Client:
            def snapshot(self):
                return joint_snapshot([45] * 12)

        plugin = sensors.MotorHealthPlugin({}, "go1", None, Client())
        tool = plugin.get_tool()

        self.assertEqual(tool["name"], "motor_health")
        self.assertEqual(tool["type"], "sensor")
        self.assertEqual(tool["inputSchema"], {"type": "object", "properties": {}})
        self.assertEqual(plugin.dispatch("read", {})["data"]["status"], "ok")
        self.assertIsNone(plugin.dispatch("unknown", {}))
        plugin._node = object()
        self.assertEqual(
            plugin.get_tool()["topic_out"],
            [{"topic": "/go1/state/motor_health", "format": "data/json"}],
        )

    def test_invalid_configuration_fails_fast(self):
        cases = [
            {"publish_hz": 0},
            {"publish_hz": 21},
            {"warning_temperature_c": 70, "critical_temperature_c": 70},
            {"warning_temperature_c": 80, "critical_temperature_c": 70},
        ]
        for config in cases:
            with self.subTest(config=config), self.assertRaises(ValueError):
                sensors.MotorHealthPlugin(config, "go1", None, object())

    def test_manifest_and_config_enable_the_card(self):
        manifest = (GO1_DIR / "driver.yaml").read_text()
        config = (GO1_DIR / "config.yaml").read_text()
        main = (GO1_DIR / "main.py").read_text()

        self.assertIn("- { name: motor_health,       type: sensor }", manifest)
        self.assertIn("  motor_health:", config)
        self.assertIn("sensors.make_motor_health", main)

    def test_bundle_loads_motor_health_factory(self):
        yaml_stub = types.ModuleType("yaml")
        yaml_stub.safe_load = lambda value: value
        spec = importlib.util.spec_from_file_location("go1_motor_health_main_test", GO1_DIR / "main.py")
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {"yaml": yaml_stub}):
            spec.loader.exec_module(module)

        class Client:
            def snapshot(self):
                return joint_snapshot([45] * 12)

        bundle = module.Go1Bundle(
            {"plugins": {"motor_health": {"enabled": True}}}, "go1", None, Client())
        tools = bundle.get_all_tools()

        self.assertEqual([tool["name"] for tool in tools], ["motor_health"])
        self.assertEqual(bundle.dispatch("motor_health", {})["data"]["status"], "ok")


if __name__ == "__main__":
    unittest.main()
