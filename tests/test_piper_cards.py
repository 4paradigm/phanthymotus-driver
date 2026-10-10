"""No hardware required: MCP contracts, exclusive ownership and fail-closed gates."""
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import patch
import urllib.request
from http.server import ThreadingHTTPServer

from agilex.piper.arm import ArmState
from agilex.piper.device import PiperPlugin
from common.vendor_runtime import DriverBundle, make_handler


class FakeDriver:
    def __init__(self, can_name):
        self.cancelled = threading.Event()
        self.state = ArmState((1., 0., -1., 0., 30., 0.), (True,) * 6,
                              "CAN_CTRL(0x1)", "NORMAL(0x0)", "DISABLED(0x0)", 0)
        self.calls = []

    def connect(self):
        self.calls.append("connect")
        return self

    def get_state(self):
        return self.state

    def close(self):
        self.calls.append("close")

    def cancel_motion(self):
        self.cancelled.set()

    def move_j1_to(self, target, speed, take_control):
        self.calls.append(("move", target, speed, take_control))
        return self.state


class PiperCardsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        with patch.dict("os.environ", {"PIPER_MOTION_ENABLED": "0"}):
            self.plugin = PiperPlugin({"data_dir": self.tmp.name}, driver_factory=FakeDriver)
        self.bundle = DriverBundle([self.plugin])

    def call(self, action, **kwargs):
        return self.bundle.dispatch("piper_arm", {"action": action, **kwargs})

    def test_lifecycle_never_connects_or_enables(self):
        self.plugin.start()
        self.assertEqual(self.call("start"), {"state": "ready"})
        self.assertIsNone(self.plugin.driver)
        self.assertFalse(self.call("stop")["motor_torque_released"])
        self.assertIsNone(self.plugin.driver)

    def test_motion_and_boolean_validation_before_connect(self):
        for action, args in [("move_j1", {"target_deg": 5, "workspace_clear": True, "take_control": True}),
                             ("prepare", {"execute": "false"}),
                             ("disable", {"arm_supported": "true"}),
                             ("save_default", {})]:
            with self.subTest(action=action), self.assertRaises((RuntimeError, ValueError)):
                self.call(action, **args)
            self.assertIsNone(self.plugin.driver)

    def test_resource_cannot_dispatch_a_motion_action(self):
        result = self.bundle.dispatch("piper_status", {"action": "move_j1", "target_deg": 9})
        self.assertEqual(result["error_code"], 0)
        self.assertEqual(self.plugin.driver.calls, ["connect"])

    def test_camera_card_cannot_dispatch_arm_actions(self):
        with self.assertRaises(ValueError):
            self.bundle.dispatch("piper_photo", {"action": "disable", "arm_supported": True})
        self.assertIsNone(self.plugin.driver)

    def test_busy_rejected_and_stop_survives_start_race(self):
        with self.plugin.operation():
            self.plugin.connected()
            with self.assertRaises(RuntimeError):
                self.bundle.dispatch("piper_status", {})
            self.assertEqual(self.call("stop")["state"], "stopping")
            with self.assertRaises(RuntimeError):
                self.call("start")
            self.assertTrue(self.plugin.driver.cancelled.is_set())
        with self.assertRaises(RuntimeError):
            self.bundle.dispatch("piper_status", {})
        self.call("start")
        self.assertEqual(self.bundle.dispatch("piper_status", {})["error_code"], 0)

    def test_default_is_local_pose_and_preview_does_not_move(self):
        pose = self.call("save_default", confirm_save=True)
        self.assertFalse(pose["factory_zero"])
        self.plugin.driver.state = replace(self.plugin.driver.state, joints_deg=(6., 0., -1., 0., 30., 0.))
        preview = self.call("return_default")
        self.assertEqual(preview["j1_delta_deg"], -5)
        self.assertEqual(self.plugin.driver.calls, ["connect"])
        self.plugin.motion_enabled = True
        self.call("return_default", execute=True, workspace_clear=True, take_control=True)
        self.assertEqual(self.plugin.driver.calls[-1], ("move", 1., 5, True))

    def test_default_return_rejects_other_joint_drift_and_large_steps(self):
        self.call("save_default", confirm_save=True)
        for joints in [(1., 0., -1., 0., 35., 0.), (20., 0., -1., 0., 30., 0.)]:
            self.plugin.driver.state = replace(self.plugin.driver.state, joints_deg=joints)
            with self.assertRaises(RuntimeError):
                self.call("return_default")
        self.assertEqual(self.plugin.driver.calls, ["connect"])

    def test_capture_releases_stalled_child_on_stop(self):
        plugin = self.plugin
        class Child:
            returncode = None
            killed = False
            def communicate(self, timeout=None):
                if timeout is None:
                    return "", ""
                plugin.request_stop()
                raise subprocess.TimeoutExpired("capture", timeout)
            def poll(self):
                return self.returncode
            def kill(self):
                self.killed = True
                self.returncode = -9
        child = Child()
        with patch("agilex.piper.device.subprocess.Popen", return_value=child):
            with self.assertRaisesRegex(RuntimeError, "cancelled"):
                self.bundle.dispatch("piper_photo", {"action": "capture"})
        self.assertTrue(child.killed)
        self.assertFalse(list(Path(self.tmp.name).glob("*.jpg")))
        self.assertIsNone(plugin.driver)

    def test_capture_returns_topic_and_metadata_without_serial_or_motion(self):
        class Child:
            returncode = 0
            def __init__(self, cmd, **kwargs):
                self.output = cmd[cmd.index("--output") + 1]
                Path(self.output).write_bytes(b"test-jpeg")
            def communicate(self, timeout=None):
                return json.dumps({"device": "private-serial", "output": self.output,
                                   "width": 640, "height": 480}), ""
            def poll(self):
                return 0
        with patch("agilex.piper.device.subprocess.Popen", Child):
            result = self.bundle.dispatch("piper_photo", {"action": "capture"})
        self.assertNotIn("device", result)
        self.assertFalse(result["forward_view_confirmed"])
        self.assertFalse(result["arm_moved"])
        self.assertEqual(result["topic_out"][0]["format"], "image/jpeg")
        self.assertTrue(Path(result["output"]).with_suffix(".json").exists())
        self.assertIsNone(self.plugin.driver)

    def test_http_mcp_handshake_list_and_plain_dict_result(self):
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(lambda: self.bundle, "piper", "agilex-piper-driver"))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        def rpc(method, params=None):
            payload = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}).encode()
            req = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp", data=payload,
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=2) as response:
                return json.load(response)
        try:
            self.assertIn("serverInfo", rpc("initialize")["result"])
            cards = rpc("tools/list")["result"]["tools"]
            self.assertEqual({c["name"]: c["type"] for c in cards}, {
                "piper_status": "resource", "piper_arm": "actuator", "piper_photo": "actuator"})
            response = rpc("tools/call", {"name": "piper_status", "arguments": {}})
            result = json.loads(response["result"]["content"][0]["text"])
            self.assertIsInstance(result, dict)
            self.assertEqual(result["error_code"], 0)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
