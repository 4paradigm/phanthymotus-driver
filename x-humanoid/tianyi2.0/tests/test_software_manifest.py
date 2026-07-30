import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


TIANYI_DIR = Path(__file__).resolve().parents[1]
DEVICE_PATH = TIANYI_DIR / "device.py"


def _install_ros_stubs():
    """Provide the imports needed to load device.py without a ROS install."""

    class _QoSProfile:
        def __init__(self, **kwargs):
            self.settings = kwargs

    class _Policy:
        BEST_EFFORT = "best_effort"
        RELIABLE = "reliable"
        KEEP_LAST = "keep_last"
        VOLATILE = "volatile"

    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_qos_module = types.ModuleType("rclpy.qos")
    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")

    rclpy_node_module.Node = type("Node", (), {})
    rclpy_qos_module.QoSProfile = _QoSProfile
    rclpy_qos_module.ReliabilityPolicy = _Policy
    rclpy_qos_module.HistoryPolicy = _Policy
    rclpy_qos_module.DurabilityPolicy = _Policy
    std_msgs_msg_module.String = type("String", (), {})
    std_msgs_msg_module.Bool = type("Bool", (), {})

    rclpy_module.node = rclpy_node_module
    rclpy_module.qos = rclpy_qos_module
    std_msgs_module.msg = std_msgs_msg_module

    sys.modules.setdefault("rclpy", rclpy_module)
    sys.modules.setdefault("rclpy.node", rclpy_node_module)
    sys.modules.setdefault("rclpy.qos", rclpy_qos_module)
    sys.modules.setdefault("std_msgs", std_msgs_module)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg_module)


def _load_device_module():
    _install_ros_stubs()
    spec = importlib.util.spec_from_file_location(
        "tianyi_device_software_manifest_test", DEVICE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


device = _load_device_module()


class SoftwareManifestTests(unittest.TestCase):
    def _plugin(self, path):
        return device.SystemPlugin(
            {"software_manifest_path": str(path)}, "test_robot", None)

    def test_tool_schema_is_a_pathless_read_only_resource(self):
        plugin = self._plugin("/tmp/version_info.json")

        self.assertEqual(
            [
                {
                    "name": "software_manifest",
                    "type": "resource",
                    "multiInstance": False,
                    "readOnly": True,
                    "description": mock.ANY,
                    "inputSchema": {
                        "type": "object",
                        "properties": {},
                        "additionalProperties": False,
                    },
                }
            ],
            plugin.get_tools(),
        )

    def test_default_path_targets_the_host_file_through_proc(self):
        plugin = device.SystemPlugin({}, "test_robot", None)

        self.assertEqual(
            Path("/proc/1/root/home/ubuntu/ros2ws/version_info.json"),
            plugin._software_manifest_path,
        )

    def test_returns_the_complete_manifest_without_field_filtering(self):
        expected = {
            "product": "TG2.0-Pro",
            "version": "v2.4.1",
            "commit": "integration/abc123",
            "release_time": "2026-07-30 10:00:00",
            "x86": [{"module": "benti", "status": "enable", "custom": 1}],
            "orin": [{"module": "lyre", "status": "enable", "custom": 2}],
            "firmware": [{"module": "arm", "version": "3.2.0", "custom": 3}],
            "future_section": {"must": ["remain", "unchanged"]},
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "version_info.json"
            path.write_text(json.dumps(expected), encoding="utf-8")
            plugin = self._plugin(path)

            result = plugin.dispatch("software_manifest", {})

        self.assertEqual(str(path), result["source"])
        self.assertEqual(expected, result["manifest"])
        self.assertNotIn("error", result)

    def test_mcp_arguments_cannot_override_the_configured_path(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            configured = Path(temp_dir) / "configured.json"
            ignored = Path(temp_dir) / "ignored.json"
            configured.write_text('{"selected": "configured"}', encoding="utf-8")
            ignored.write_text('{"selected": "mcp_arg"}', encoding="utf-8")
            plugin = self._plugin(configured)

            result = plugin.dispatch(
                "software_manifest", {"path": str(ignored)})

        self.assertEqual({"selected": "configured"}, result["manifest"])

    def test_missing_file_has_a_stable_error_code(self):
        plugin = self._plugin("/definitely/missing/version_info.json")

        result = plugin.dispatch("software_manifest", {})

        self.assertEqual("manifest_not_found", result["code"])

    def test_permission_error_has_a_stable_error_code(self):
        plugin = self._plugin("/private/version_info.json")

        with mock.patch.object(
                Path, "open", side_effect=PermissionError("denied")):
            result = plugin.dispatch("software_manifest", {})

        self.assertEqual(
            "manifest_permission_denied", result["code"])

    def test_oversized_file_is_rejected_after_a_bounded_read(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "version_info.json"
            path.write_bytes(
                b"{" + b" " * device._SOFTWARE_MANIFEST_MAX_BYTES + b"}")
            plugin = self._plugin(path)

            result = plugin.dispatch("software_manifest", {})

        self.assertEqual("manifest_too_large", result["code"])

    def test_invalid_json_has_a_stable_error_code(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "version_info.json"
            path.write_text('{"broken":', encoding="utf-8")
            plugin = self._plugin(path)

            result = plugin.dispatch("software_manifest", {})

        self.assertEqual("manifest_invalid_json", result["code"])

    def test_nonstandard_json_constants_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "version_info.json"
            path.write_text('{"temperature": NaN}', encoding="utf-8")
            plugin = self._plugin(path)

            result = plugin.dispatch("software_manifest", {})

        self.assertEqual("manifest_invalid_json", result["code"])

    def test_json_root_must_be_an_object(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "version_info.json"
            path.write_text('["not", "an", "object"]', encoding="utf-8")
            plugin = self._plugin(path)

            result = plugin.dispatch("software_manifest", {})

        self.assertEqual("manifest_not_object", result["code"])

    def test_unexpected_read_error_has_a_stable_error_code(self):
        plugin = self._plugin("/tmp/version_info.json")

        with mock.patch.object(
                Path, "open", side_effect=OSError("device error")):
            result = plugin.dispatch("software_manifest", {})

        self.assertEqual("manifest_read_failed", result["code"])

    def test_lifecycle_is_idempotent_and_info_reports_state(self):
        plugin = self._plugin("/tmp/version_info.json")

        self.assertEqual(
            {"state": "idle", "tools": ["software_manifest"]},
            plugin.dispatch("info", {}),
        )
        self.assertEqual(
            {"state": "running", "tools": ["software_manifest"]},
            plugin.dispatch("start", {}),
        )
        self.assertEqual(
            {"state": "running", "tools": ["software_manifest"]},
            plugin.dispatch("start", {}),
        )
        self.assertEqual(
            {"state": "idle", "tools": ["software_manifest"]},
            plugin.dispatch("stop", {}),
        )


if __name__ == "__main__":
    unittest.main()
