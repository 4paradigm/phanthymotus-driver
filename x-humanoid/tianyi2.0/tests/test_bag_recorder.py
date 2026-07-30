"""Unit tests for the Tianyi 2.0 host-side bag recorder card."""

from __future__ import annotations

import importlib.util
import os
import signal
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


TIANYI_DIR = Path(__file__).resolve().parents[1]
DEVICE_PATH = TIANYI_DIR / "device.py"


def _install_ros_stubs():
    class _QoSProfile:
        def __init__(self, **kwargs):
            self.settings = kwargs

    class _Policy:
        BEST_EFFORT = "best_effort"
        RELIABLE = "reliable"
        KEEP_LAST = "keep_last"
        VOLATILE = "volatile"
        TRANSIENT_LOCAL = "transient_local"

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
    sys.modules.update({
        "rclpy": rclpy_module,
        "rclpy.node": rclpy_node_module,
        "rclpy.qos": rclpy_qos_module,
        "std_msgs": std_msgs_module,
        "std_msgs.msg": std_msgs_msg_module,
    })


def _load_device_module():
    _install_ros_stubs()
    spec = importlib.util.spec_from_file_location(
        "tianyi2_device_bag_test", DEVICE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


device = _load_device_module()


class _FakeProcess:
    def __init__(self, pid=4242, wait_results=None):
        self.pid = pid
        self.returncode = None
        self._wait_results = list(wait_results or [0])

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        result = self._wait_results.pop(0)
        if isinstance(result, BaseException):
            raise result
        self.returncode = result
        return result


class BagRecorderTests(unittest.TestCase):
    def setUp(self):
        self._temp_dir = tempfile.TemporaryDirectory()
        self.host_root = Path(self._temp_dir.name) / "host"
        self.bag_dir = self.host_root / "home" / "ubuntu" / "bags"
        self.setup = (
            self.host_root
            / "home"
            / "ubuntu"
            / "ros2ws"
            / "install"
            / "setup.bash"
        )
        self.bag_dir.mkdir(parents=True)
        self.setup.parent.mkdir(parents=True)
        self.setup.write_text("# test setup\n", encoding="utf-8")
        self.plugin = device.SystemPlugin(
            {
                "host_root": str(self.host_root),
                "bag_dir": "/home/ubuntu/bags",
                "setup": "/home/ubuntu/ros2ws/install/setup.bash",
                "stop_timeout_s": 0.01,
                "terminate_timeout_s": 0.01,
                "kill_timeout_s": 0.01,
            },
            "test_robot",
            None,
        )
        self.plugin.start()

    def tearDown(self):
        self._temp_dir.cleanup()

    def test_tool_exposes_only_bounded_recording_actions(self):
        tool = next(
            item for item in self.plugin.get_tools()
            if item["name"] == "bag_recorder"
        )
        schema = tool["inputSchema"]

        self.assertEqual(
            [
                "start_recording",
                "stop_recording",
                "status",
                "list_sessions",
            ],
            schema["properties"]["action"]["enum"],
        )
        self.assertEqual({"action"}, set(schema["properties"]))
        self.assertFalse(schema["additionalProperties"])

    def test_start_uses_fixed_host_command_and_rejects_duplicate(self):
        process = _FakeProcess()

        with mock.patch.object(
                device.subprocess, "Popen", return_value=process) as popen:
            started = self.plugin.dispatch("start_recording", {})
            duplicate = self.plugin.dispatch("start_recording", {})

        self.assertTrue(started["ok"])
        self.assertEqual("recording", started["state"])
        self.assertEqual(process.pid, started["pid"])
        self.assertFalse(duplicate["ok"])
        self.assertEqual("already_recording", duplicate["code"])
        popen.assert_called_once()
        self.assertEqual(
            [
                "nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p",
                "--", "bash", "-lc",
                'source "$1" && exec ros2 launch utils record_trigger.py',
                "bash", "/home/ubuntu/ros2ws/install/setup.bash",
            ],
            popen.call_args.args[0],
        )
        self.assertEqual(
            {
                "stdin": subprocess.DEVNULL,
                "stdout": subprocess.DEVNULL,
                "stderr": subprocess.DEVNULL,
                "start_new_session": True,
                "close_fds": True,
            },
            popen.call_args.kwargs,
        )

    def test_missing_setup_is_reported_without_spawning(self):
        self.setup.unlink()

        with mock.patch.object(device.subprocess, "Popen") as popen:
            result = self.plugin.dispatch("start_recording", {})

        self.assertEqual("recorder_setup_not_found", result["code"])
        popen.assert_not_called()

    def test_stop_sends_sigint_to_the_managed_process_group(self):
        process = _FakeProcess()
        with mock.patch.object(
                device.subprocess, "Popen", return_value=process):
            self.plugin.dispatch("start_recording", {})

        with (
            mock.patch.object(
                device.os, "getpgid", return_value=process.pid),
            mock.patch.object(device.os, "killpg") as killpg,
        ):
            stopped = self.plugin.dispatch("stop_recording", {})

        self.assertTrue(stopped["ok"])
        self.assertEqual("idle", stopped["state"])
        self.assertEqual("sigint", stopped["stop_stage"])
        killpg.assert_called_once_with(process.pid, signal.SIGINT)
        self.assertEqual(
            "idle", self.plugin.dispatch("status", {})["state"])

    def test_stop_escalates_to_sigterm_then_sigkill(self):
        process = _FakeProcess(wait_results=[
            subprocess.TimeoutExpired("record_trigger", 0.01),
            subprocess.TimeoutExpired("record_trigger", 0.01),
            -signal.SIGKILL,
        ])
        with mock.patch.object(
                device.subprocess, "Popen", return_value=process):
            self.plugin.dispatch("start_recording", {})

        with (
            mock.patch.object(
                device.os, "getpgid", return_value=process.pid),
            mock.patch.object(device.os, "killpg") as killpg,
        ):
            stopped = self.plugin.dispatch("stop_recording", {})

        self.assertEqual("sigkill", stopped["stop_stage"])
        self.assertEqual(
            [signal.SIGINT, signal.SIGTERM, signal.SIGKILL],
            [call.args[1] for call in killpg.call_args_list],
        )

    def test_list_sessions_is_filtered_read_only_and_newest_first(self):
        old_session = self.bag_dir / "session-old"
        new_session = self.bag_dir / "session-new"
        direct_bag = self.bag_dir / "capture.mcap"
        old_session.mkdir()
        new_session.mkdir()
        direct_bag.write_bytes(b"bag")
        (self.bag_dir / "notes.txt").write_text(
            "not a bag", encoding="utf-8")
        (self.bag_dir / "session-link").symlink_to(
            new_session, target_is_directory=True)
        os.utime(old_session, (10, 10))
        os.utime(new_session, (30, 30))
        os.utime(direct_bag, (20, 20))

        result = self.plugin.dispatch("list_sessions", {})

        self.assertTrue(result["ok"])
        self.assertEqual(
            ["session-new", "capture.mcap", "session-old"],
            [item["name"] for item in result["sessions"]],
        )
        self.assertTrue(all(
            item["path"].startswith("/home/ubuntu/bags/")
            for item in result["sessions"]
        ))

    def test_lifecycle_stop_terminates_active_recording(self):
        process = _FakeProcess()
        with mock.patch.object(
                device.subprocess, "Popen", return_value=process):
            self.plugin.dispatch("start_recording", {})

        with (
            mock.patch.object(
                device.os, "getpgid", return_value=process.pid),
            mock.patch.object(device.os, "killpg"),
        ):
            self.plugin.stop()

        info = self.plugin.dispatch("info", {})
        self.assertEqual("idle", info["plugin_state"])
        self.assertEqual("idle", info["state"])

    def test_invalid_host_paths_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "absolute host path"):
            device.SystemPlugin(
                {
                    "host_root": str(self.host_root),
                    "bag_dir": "../bags",
                },
                "test_robot",
                None,
            )


if __name__ == "__main__":
    unittest.main()
