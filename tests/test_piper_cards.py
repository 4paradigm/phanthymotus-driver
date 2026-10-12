"""No hardware required: MCP contracts, exclusive ownership and fail-closed gates."""
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
from agilex.piper.runtime import PiperRuntime as PiperPlugin
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


class PiperCardsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        with patch.dict("os.environ", {"PIPER_MOTION_ENABLED": "0"}):
            self.plugin = PiperPlugin({"data_dir": self.tmp.name, "async_actions": False}, driver_factory=FakeDriver)
        self.addCleanup(self.plugin.stop)
        self.bundle = DriverBundle([self.plugin])

    def call(self, action, **kwargs):
        return self.bundle.dispatch("piper_arm", {"action": action, **kwargs})

    def test_lifecycle_never_connects_or_enables(self):
        self.plugin.start()
        self.assertEqual(self.call("start"), {"state": "ready"})
        self.assertIsNone(self.plugin.driver)
        self.assertFalse(self.call("stop")["motor_torque_released"])
        self.assertIsNone(self.plugin.driver)

    def test_status_has_canvas_read_action_and_safe_lifecycle(self):
        schema = next(t for t in self.plugin.get_tools() if t["name"] == "piper_status")["inputSchema"]
        self.assertIn("read", schema["properties"]["action"]["enum"])
        for action in ("start", "info", "stop"):
            self.assertIn(self.bundle.dispatch("piper_status", {"action": action})["state"], ("ready", "idle"))
            self.assertIsNone(self.plugin.driver)
        self.assertEqual(self.bundle.dispatch("piper_status", {"action": "read"})["error_code"], 0)
        self.assertEqual(self.plugin.driver.calls, ["connect"])

    def test_motion_and_boolean_validation_before_connect(self):
        for action, args in [("move_joint", {"joint": 1, "target_deg": 5, "execute": True, "workspace_clear": True, "take_control": True}),
                             ("prepare", {"execute": "false"}),
                             ("disable", {"arm_supported": "true"})]:
            with self.subTest(action=action), self.assertRaises((RuntimeError, ValueError)):
                self.call(action, **args)
            self.assertIsNone(self.plugin.driver)

    def test_resource_cannot_dispatch_a_motion_action(self):
        result = self.bundle.dispatch("piper_status", {"action": "move_joint", "joint": 1, "target_deg": 9})
        self.assertEqual(result["error_code"], 0)
        self.assertEqual(self.plugin.driver.calls, ["connect"])

    def test_camera_card_cannot_dispatch_arm_actions(self):
        with self.assertRaises(ValueError):
            self.bundle.dispatch("piper_photo", {"action": "disable", "arm_supported": True})
        self.assertIsNone(self.plugin.driver)

    def test_busy_rejected_and_stop_survives_start_race(self):
        with self.plugin.operation():
            self.plugin.connected()
            self.assertEqual(self.bundle.dispatch("piper_status", {})["error_code"], 0)
            self.assertTrue(self.call("stop")["operation_stopping"])
            with self.assertRaises(RuntimeError):
                self.call("start")
            self.assertTrue(self.plugin.driver.cancelled.is_set())
        self.assertEqual(self.bundle.dispatch("piper_status", {})["error_code"], 0)
        self.call("start")
        self.assertEqual(self.bundle.dispatch("piper_status", {})["error_code"], 0)

    def test_removed_arm_actions_reject_before_connect_even_with_motion_permission(self):
        self.plugin.motion_enabled = True
        pose = Path(self.tmp.name) / "default_pose.json"
        original = '{"joints_deg": [1, 0, -1, 0, 30, 0]}\n'
        pose.write_text(original)
        for async_actions in (False, True):
            self.plugin.config["async_actions"] = async_actions
            for action in ("return_default", "move_j1", "return_pose", "save_default", "move_joints"):
                for execute in (False, True):
                    with self.subTest(action=action, execute=execute, asynchronous=async_actions):
                        with self.assertRaisesRegex(ValueError, "Unknown arm action"):
                            self.call(action, execute=execute, workspace_clear=True, take_control=True)
                        self.assertIsNone(self.plugin.driver)
                        self.assertFalse(self.plugin.gate.locked())
                        self.assertFalse(self.plugin.last_actions)
        self.assertEqual(pose.read_text(), original)
        self.assertFalse(self.plugin.previews.entries)

    def test_arm_schema_exposes_only_retained_actions_and_parameters(self):
        schemas = {t["name"]: t["inputSchema"] for t in self.plugin.get_tools()}
        self.assertEqual(set(schemas), {"piper_status", "piper_mode", "piper_arm", "piper_gripper", "piper_photo"})
        schema = schemas["piper_arm"]
        self.assertEqual(set(schema["properties"]["action"]["enum"]),
                         {"start", "stop", "info", "prepare", "disable", "move_joint", "hold", "allow_motion", "block_motion"})
        self.assertEqual(set(schema["x-action-params"]),
                         {"start", "stop", "info", "prepare", "disable", "move_joint", "hold"})
        self.assertEqual(schema["x-completion"]["actions"], ["prepare", "move_joint"])
        self.assertFalse({"confirm_save", *(f"j{i}_deg" for i in range(1, 7))} & set(schema["properties"]))

    def test_capture_releases_stalled_child_on_stop(self):
        plugin = self.plugin
        plugin.camera_enabled = True
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
        with patch("agilex.piper.runtime.subprocess.Popen", return_value=child):
            with self.assertRaisesRegex(RuntimeError, "cancelled"):
                self.bundle.dispatch("piper_photo", {"action": "capture", "capture_confirmed": True})
        self.assertTrue(child.killed)
        self.assertFalse(list(Path(self.tmp.name).glob("*.jpg")))
        self.assertIsNone(plugin.driver)

    def test_capture_returns_topic_and_metadata_without_serial_or_motion(self):
        self.plugin.camera_enabled = True
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
        with patch("agilex.piper.runtime.subprocess.Popen", Child):
            result = self.bundle.dispatch("piper_photo", {"action": "capture", "capture_confirmed": True})
        self.assertNotIn("device", result)
        self.assertTrue(result["capture_confirmed"])
        self.assertTrue(result["camera_accessed"])
        self.assertNotIn("forward_view_confirmed", result)
        self.assertFalse(result["arm_moved"])
        self.assertEqual(result["topic_out"][0]["format"], "image/jpeg")
        self.assertTrue(Path(result["output"]).with_suffix(".json").exists())
        self.assertIsNone(self.plugin.driver)

    def test_capture_requires_host_permission_and_current_shot_confirmation(self):
        with patch("agilex.piper.runtime.subprocess.Popen") as camera:
            with self.assertRaisesRegex(RuntimeError, "默认关闭"):
                self.bundle.dispatch("piper_photo", {"action": "capture", "capture_confirmed": True})
            self.plugin.camera_enabled = True
            for value in (None, False, "true", 1):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    self.bundle.dispatch("piper_photo", {"action": "capture", "capture_confirmed": value})
            with self.assertRaisesRegex(ValueError, "capture_confirmed"):
                self.bundle.dispatch("piper_photo", {"action": "capture", "forward_view_confirmed": True})
            with self.assertRaisesRegex(ValueError, "forward_view_confirmed"):
                self.bundle.dispatch("piper_photo", {"action": "capture", "capture_confirmed": True,
                                                     "forward_view_confirmed": "true"})
        camera.assert_not_called()
        self.assertIsNone(self.plugin.driver)

    def test_capture_card_exposes_current_shot_confirmation(self):
        schema = next(t["inputSchema"] for t in self.plugin.get_tools() if t["name"] == "piper_photo")
        self.assertEqual(schema["x-action-params"]["capture"]["params"], ["capture_confirmed"])
        self.assertFalse(schema["properties"]["capture_confirmed"]["default"])

    def test_import_photo_saves_decodable_jpeg_without_camera_or_can(self):
        import cv2
        import numpy as np
        root = Path(self.tmp.name) / "imports"
        root.mkdir()
        cv2.imwrite(str(root / "provided.png"), np.full((24, 32, 3), 120, dtype=np.uint8))
        with patch("agilex.piper.runtime.subprocess.Popen") as camera:
            result = self.bundle.dispatch("piper_photo", {"action": "import_photo", "filename": "provided.png"})
        camera.assert_not_called()
        self.assertIsNone(self.plugin.driver)
        self.assertEqual(result["source"], "user_provided")
        self.assertFalse(result["camera_accessed"])
        self.assertIsNone(result["captured_at"])
        self.assertEqual(cv2.imread(result["output"]).shape, (24, 32, 3))
        self.assertEqual(json.loads(Path(result["output"]).with_suffix(".json").read_text()), result)

    def test_import_rejects_paths_symlinks_corruption_and_cleans_publication_failure(self):
        import cv2
        import numpy as np
        root = Path(self.tmp.name) / "imports"
        root.mkdir()
        (root / "bad.jpg").write_bytes(b"not an image")
        (root / "link.jpg").symlink_to(root / "bad.jpg")
        for filename in ("../bad.jpg", "/tmp/image.jpg", "a\\b.jpg", "link.jpg", "bad.jpg", "missing.jpg", "a.txt", None):
            with self.subTest(filename=filename), self.assertRaises(ValueError):
                self.bundle.dispatch("piper_photo", {"action": "import_photo", "filename": filename})
        cv2.imwrite(str(root / "valid.jpg"), np.zeros((4, 4, 3), dtype=np.uint8))
        with patch.object(self.plugin, "publish_photo", side_effect=RuntimeError("DDS failed")):
            with self.assertRaisesRegex(RuntimeError, "DDS failed"):
                self.bundle.dispatch("piper_photo", {"action": "import_photo", "filename": "valid.jpg"})
        self.assertFalse(list(Path(self.tmp.name).glob("rgb_*")))
        self.assertFalse(self.plugin.gate.locked())
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
                "piper_status": "resource", "piper_mode": "actuator", "piper_arm": "actuator",
                "piper_gripper": "actuator", "piper_photo": "actuator"})
            for action in ("return_default", "move_j1", "return_pose", "save_default", "move_joints"):
                with self.subTest(removed_action=action):
                    response = rpc("tools/call", {"name": "piper_arm", "arguments": {"action": action, "execute": True}})
                    self.assertEqual(response["error"]["code"], -32602)
                    self.assertIn("Unknown arm action", response["error"]["message"])
                    self.assertIsNone(self.plugin.driver)
            response = rpc("tools/call", {"name": "piper_status", "arguments": {}})
            result = json.loads(response["result"]["content"][0]["text"])
            self.assertIsInstance(result, dict)
            self.assertEqual(result["error_code"], 0)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
