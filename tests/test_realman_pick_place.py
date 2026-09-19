from pathlib import Path
import subprocess
import sys
import unittest

from test_realman_rm75_driver import DRIVER, ROOT, load_device
from common.vendor_runtime import DriverBundle

sys.path.insert(0, str(DRIVER))


class PickPlaceConfigTests(unittest.TestCase):
    def setUp(self):
        device = load_device()
        self.bundle = DriverBundle(device.build_plugins({}, "rm75", None))

    def configure(self, **values):
        return self.bundle.dispatch("pick_place", {"action": "config", **values})

    def test_registered_config_defaults(self):
        card = next(tool for tool in self.bundle.get_all_tools() if tool["name"] == "pick_place")
        self.assertEqual(card["type"], "actuator")
        self.assertNotIn("topic_in", card)
        expected = {
            "speed_percent": 50, "observation_joints_deg": "-90,0,0,90,0,90,0",
            "x_compensation_mm": 30, "y_compensation_mm": -75,
            "pick_descent_mm": 91, "pick_grip_force": 10, "place_descent_mm": 87,
        }
        self.assertEqual({key: prop["default"] for key, prop in card["configSchema"]["properties"].items()}, expected)
        self.assertEqual(self.configure(), {"ok": True, **expected})

    def test_observe_has_no_parameters_or_asynchronous_completion_contract(self):
        card = next(tool for tool in self.bundle.get_all_tools() if tool["name"] == "pick_place")
        schema = card["inputSchema"]
        self.assertEqual(set(schema["properties"]["action"]["enum"]), {"observe", "transfer_to", "transfer_by", "cancel"})
        self.assertEqual(set(schema["properties"]), {"action", "x1", "y1", "x2", "y2", "dx_mm", "dy_mm"})
        self.assertEqual(schema["x-action-params"]["observe"]["params"], [])
        self.assertEqual(schema["x-action-params"]["transfer_to"]["params"], ["x1", "y1", "x2", "y2"])
        self.assertEqual(schema["x-action-params"]["transfer_by"]["params"], ["x1", "y1", "dx_mm", "dy_mm"])
        for name in ("x1", "y1", "x2", "y2"):
            prop = schema["properties"][name]
            self.assertEqual((prop["type"], prop["minimum"], prop["maximum"]), ("number", -1, 1))
            self.assertIn("归一化", prop["description"])
            self.assertIn("正方向向右" if name.startswith("x") else "正方向向下", prop["description"])
        self.assertIn("position[0]", schema["properties"]["x1"]["description"])
        self.assertIn("position[1]", schema["properties"]["y1"]["description"])
        for name in ("dx_mm", "dy_mm"):
            self.assertEqual(schema["properties"][name]["type"], "number")
            self.assertNotIn("minimum", schema["properties"][name])
            self.assertIn("mm", schema["properties"][name]["description"])
        self.assertIn("X 正方向向右（正值），负方向向左（负值）", schema["properties"]["dx_mm"]["description"])
        self.assertIn("Y 正方向向照片下方（正值），负方向向上方（负值）", schema["properties"]["dy_mm"]["description"])
        self.assertEqual(schema["required"], ["action"])
        self.assertNotIn("x-completion", schema)
        self.assertTrue(schema["x-is-dangerous"])
        self.assertEqual(schema["x-resource"], "arm")
        self.assertEqual(schema["x-hooks"]["on_interrupt_motion"], {"action": "cancel"})
        self.assertEqual(schema["x-hooks"]["on_interrupt_all"], {"action": "cancel"})

    def test_each_mcp_action_exposes_observation_lifecycle(self):
        card = next(tool for tool in self.bundle.get_all_tools() if tool["name"] == "pick_place")
        actions = card["inputSchema"]["x-action-params"]
        self.assertIn(card["topic_out"][0]["topic"] + "/objects", actions["observe"]["description"])
        for name in ("transfer_to", "transfer_by"):
            description = actions[name]["description"]
            for requirement in ("observe", "captured_at", "position[0]", "position[1]",
                                "每张照片仅供一次搬运", "observation_required=true", "不自动重试"):
                self.assertIn(requirement, description)

    def test_partial_updates_and_invalid_updates_are_atomic(self):
        configured = self.configure(speed_percent=25, x_compensation_mm=-1.5)
        self.assertTrue(configured["ok"])
        self.assertEqual(configured["pick_grip_force"], 10)
        for invalid in (
            {"speed_percent": 0}, {"speed_percent": 101}, {"speed_percent": 1.5},
            {"speed_percent": True}, {"pick_grip_force": -1}, {"pick_grip_force": 101},
            {"pick_grip_force": 2.5}, {"pick_descent_mm": 0}, {"place_descent_mm": -1},
            {"x_compensation_mm": float("nan")}, {"y_compensation_mm": float("inf")},
            {"pick_descent_mm": "91"}, {"place_descent_mm": None},
            {"observation_joints_deg": [0] * 7}, {"observation_joints_deg": "0,0"},
            {"observation_joints_deg": "0,0,0,0,0,0,nan"},
            {"observation_joints_deg": "0,131,0,0,0,0,0"},
            {"observation_joints_deg": "0,0,0,0,0,0,no"}, {"unknown": 1},
        ):
            with self.subTest(invalid=invalid):
                result = self.configure(**{"y_compensation_mm": 42, **invalid})
                self.assertEqual(result["code"], "INVALID_CONFIG")
                self.assertEqual(self.configure(), configured)

    def test_boundaries_and_placeholder_do_not_call_hardware(self):
        self.assertTrue(self.configure(speed_percent=100, pick_grip_force=0,
                                      pick_descent_mm=0.5, place_descent_mm=0.5,
                                      observation_joints_deg="178,130,178,135,178,128,360")["ok"])
        self.assertTrue(self.configure(speed_percent=1, pick_grip_force=100,
                                      observation_joints_deg="-178,-130,-178,-135,-178,-128,-360")["ok"])
        for action in ("observe", "transfer_to", "transfer_by"):
            self.assertEqual(self.bundle.dispatch("pick_place", {"action": action})["state"], "error")


class IndependenceTests(unittest.TestCase):
    def test_observe_runs_with_other_card_modules_unavailable(self):
        script = '''
import sys, unittest
sys.path[:0] = ["tests", "realman/rm75_6f_v"]
for name in ("device", "camera", "realsense", "servo", "vision_capture"):
    sys.modules[name] = None
from test_realman_pick_place import ObserveTests
result = unittest.TextTestRunner().run(unittest.TestSuite([
    ObserveTests("test_call_returns_after_one_move_and_one_photo"),
    ObserveTests("test_activation_and_info_do_not_capture_or_move"),
    __import__("test_realman_pick_place_transfer").TransferTests("test_complete_transfer_uses_configured_absolute_targets"),
    __import__("test_realman_pick_place_transfer").TransferTests("test_transfer_by_uses_pick_point_and_configured_millimetres"),
]))
sys.exit(not result.wasSuccessful())
'''
        result = subprocess.run([sys.executable, "-c", script], cwd=ROOT,
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_other_cards_configuration_does_not_configure_pick_place(self):
        from pick_place import PickPlacePlugin
        from pick_place.camera import SnapshotCameras
        from hardware import RM75SDKClient
        config = {"ext_camera": {"enabled": False, "serial_number": "unrelated"},
                  "vision_capture": {"enabled": False, "output_dir": "/unrelated"},
                  "safety": {"max_speed_percent": 1}}
        card = PickPlacePlugin(RM75SDKClient({}).exclusive_client(), config)
        self.assertIsInstance(card._cameras, SnapshotCameras)
        self.assertEqual(card._output_dir, Path("/opt/phanthy-motus/data/pick_place/realman"))
        self.assertEqual(card.dispatch("config", {})["speed_percent"], 50)


class ObserveTests(unittest.TestCase):
    def setUp(self):
        import copy
        import tempfile
        import threading
        import types
        from unittest import mock
        from pick_place import PickPlacePlugin
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.client = types.SimpleNamespace(connected=True, motion_enabled=True,
                                           motion_lock=threading.Lock())
        self.joints = [0.0] * 7
        self.pose = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
        self.state = {"joint_err_code": [0]*7, "joint_en_flag": [1]*7, "err": {"err": [0]}}
        self.frame = {"name": "base", "pose": [0]*6}
        self.commands = []
        self.client.command = self.command
        self.client.call = self.call
        self.client.call_dict = lambda method: {"trajectory_type": 0, "data": self.joints[:]}
        self.client.status = lambda: {"endpoint": "test-arm:8080"}
        self.camera = mock.Mock()
        self.camera.info.return_value = {"state": "running", "fresh": True}
        self.camera.snapshot.side_effect = self.snapshot
        self.pool = mock.Mock()
        self.pool.select.return_value = self.camera
        self.plugin = PickPlacePlugin(self.client, {"pick_place": {"output_dir": self.temp.name}},
                                      cameras=self.pool)
        self.plugin._ensure_publisher = mock.Mock()
        self.plugin._publish_photo = mock.Mock()
        self.copy = copy.deepcopy

    def command(self, method, *args):
        self.commands.append((method, args))
        if method == "rm_movej":
            self.joints = list(args[0])

    def call(self, method):
        if method == "rm_get_arm_all_state":
            return self.copy(self.state)
        if method == "rm_get_current_arm_state":
            return {"joint": self.joints[:], "pose": self.pose[:], "err": {"err": [0]}}
        if method == "rm_get_joint_degree":
            return self.joints[:]
        if method == "rm_get_joint_drive_min_pos":
            return [-178, -130, -178, -135, -178, -128, -360]
        if method == "rm_get_joint_drive_max_pos":
            return [178, 130, 178, 135, 178, 128, 360]
        if method in ("rm_get_current_work_frame", "rm_get_current_tool_frame"):
            return self.copy(self.frame)
        raise AssertionError(method)

    def snapshot(self, after, cancel, check):
        import time
        check()
        return {"jpeg": b"\xff\xd8test\xff\xd9", "depth_zlib": b"test-depth",
                "captured_at": max(time.time(), after + 0.001), "depth_captured_at": after + 0.001,
                "serial_number": "D435-test", "width": 640, "height": 480,
                "intrinsics": {"fx": 500}, "depth_scale_m": 0.001}

    def observe(self):
        return self.plugin.dispatch("observe", {})

    def test_call_returns_after_one_move_and_one_photo(self):
        from pathlib import Path
        import json
        result = self.observe()
        self.assertEqual(result["state"], "completed", result)
        self.assertFalse(result["observation_required"])
        self.assertFalse(self.plugin.dispatch("info", {})["observation_required"])
        self.assertIsNone(self.plugin._active)
        self.assertNotIn("action_id", result)
        self.assertNotIn("request_id", result)
        self.assertEqual(self.commands, [("rm_movej", ([-90., 0., 0., 90., 0., 90., 0.], 50, 0, 0, 0))])
        self.camera.snapshot.assert_called_once()
        self.plugin._publish_photo.assert_called_once()
        path = Path(result["result"]["file_path"])
        self.assertTrue(path.exists())
        metadata = json.loads(path.with_name("metadata.json").read_text())
        self.assertEqual(metadata["depth_aligned_to"], "color")
        self.assertEqual(metadata["arm_endpoint"], "test-arm:8080")
        self.assertEqual(metadata["joint_degree"], [-90., 0., 0., 90., 0., 90., 0.])
        self.assertFalse(self.client.motion_lock.locked())
        self.camera.stop.assert_called_once_with()

    def test_each_call_takes_a_new_photo(self):
        first, second = self.observe(), self.observe()
        self.assertEqual(first["state"], "completed")
        self.assertEqual(second["state"], "completed")
        self.assertNotEqual(first["result"]["observation_id"], second["result"]["observation_id"])
        self.assertEqual(sum(name == "rm_movej" for name, _ in self.commands), 2)
        self.assertEqual(self.camera.snapshot.call_count, 2)
        self.assertEqual(self.plugin._publish_photo.call_count, 2)

    def test_observe_uses_configured_speed_and_joints(self):
        self.assertTrue(self.plugin.dispatch("config", {
            "speed_percent": 23, "observation_joints_deg": "-80,1,2,85,3,80,4"})["ok"])
        result = self.observe()
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(self.commands, [("rm_movej", ([-80., 1., 2., 85., 3., 80., 4.], 23, 0, 0, 0))])

    def test_mcp_call_returns_completed_photo_without_extra_parameters(self):
        from http.server import ThreadingHTTPServer
        import json
        import threading
        import time
        import urllib.request
        from common.vendor_runtime import make_handler

        def delayed_snapshot(after, cancel, check):
            # A synchronous call longer than the ACP guide's three-second
            # threshold still returns the terminal result in this HTTP response.
            time.sleep(3.1)
            return self.snapshot(after, cancel, check)

        self.camera.snapshot.side_effect = delayed_snapshot
        bundle = DriverBundle([self.plugin])
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(lambda: bundle, "test", "test"))
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            body = {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                    "params": {"name": "pick_place", "arguments": {"action": "observe"}}}
            request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp",
                                             data=json.dumps(body).encode(),
                                             headers={"Content-Type": "application/json"})
            started = time.monotonic()
            with urllib.request.urlopen(request, timeout=10) as response:
                rpc = json.load(response)
            self.assertGreaterEqual(time.monotonic() - started, 3)
            result = json.loads(rpc["result"]["content"][0]["text"])
            self.assertEqual(result["state"], "completed", result)
            self.assertNotIn("action_id", result)
            self.assertTrue(Path(result["result"]["file_path"]).exists())
            self.assertIsNone(self.plugin._active)
            self.camera.stop.assert_called_once()
            self.plugin._publish_photo.assert_called_once()
        finally:
            server.shutdown()
            worker.join(timeout=2)
            server.server_close()

    def test_activation_and_info_do_not_capture_or_move(self):
        self.plugin.start()
        self.plugin.dispatch("info", {})
        self.plugin.stop()
        self.assertEqual(self.commands, [])
        self.pool.select.assert_not_called()
        self.plugin._publish_photo.assert_not_called()

    def test_disconnected_readonly_and_busy_errors_are_distinct(self):
        self.client.connected = False
        self.assertIn("not connected", self.observe()["message"])
        self.client.connected = True
        self.client.motion_enabled = False
        self.assertIn("read-only mode", self.observe()["message"])
        self.client.motion_enabled = True
        self.client.motion_lock.acquire()
        try:
            self.assertEqual(self.observe()["state"], "error")
        finally:
            self.client.motion_lock.release()
        self.assertFalse(self.commands)
        self.pool.select.assert_not_called()

    def test_joint_fault_blocks_before_motion(self):
        self.state["joint_err_code"][6] = 0xF000
        result = self.observe()
        self.assertEqual(result["state"], "error")
        self.assertFalse(self.commands)
        self.plugin._publish_photo.assert_not_called()

    def test_camera_failure_blocks_before_motion(self):
        self.camera.info.return_value = {"state": "error", "fresh": False, "error": "disconnected"}
        self.assertEqual(self.observe()["state"], "error")
        self.assertFalse(self.commands)
        self.plugin._publish_photo.assert_not_called()

    def test_capture_failure_stops_without_photo_or_return_motion(self):
        self.camera.snapshot.side_effect = RuntimeError("camera timeout")
        self.assertEqual(self.observe()["state"], "error")
        self.assertEqual([name for name, _ in self.commands], ["rm_movej", "rm_set_arm_slow_stop"])
        self.plugin._publish_photo.assert_not_called()
        self.assertIsNone(self.plugin._observation)
        self.assertTrue(self.plugin.dispatch("info", {})["observation_required"])

    def test_synchronous_timeout_stops_motion_and_returns_error(self):
        import time
        from unittest import mock

        clock = time.monotonic
        elapsed = [0]
        original = self.command
        def command(method, *args):
            original(method, *args)
            if method == "rm_movej":
                elapsed[0] = 46

        self.client.command = command
        with mock.patch("pick_place.time.monotonic", side_effect=lambda: clock() + elapsed[0]):
            result = self.observe()
        self.assertEqual(result["state"], "error")
        self.assertIn("timed out", result["result"]["message"])
        self.assertEqual([name for name, _ in self.commands], ["rm_movej", "rm_set_arm_slow_stop"])
        self.assertFalse(self.client.motion_lock.locked())
        self.camera.snapshot.assert_not_called()
        self.plugin._publish_photo.assert_not_called()

    def test_cancel_and_config_during_motion(self):
        from concurrent.futures import ThreadPoolExecutor
        import threading
        entered = threading.Event()
        def snapshot(after, cancel, check):
            entered.set()
            cancel.wait(2)
            check()
        self.camera.snapshot.side_effect = snapshot
        with ThreadPoolExecutor(max_workers=1) as executor:
            pending = executor.submit(self.observe)
            self.assertTrue(entered.wait(3))
            self.assertFalse(pending.done())
            self.assertEqual(self.observe()["state"], "error")
            self.assertEqual(self.plugin.dispatch("config", {"speed_percent": 10})["code"], "ACTION_IN_PROGRESS")
            self.plugin.dispatch("cancel", {})
            self.assertEqual(pending.result(timeout=3)["state"], "cancelled")
        self.plugin._publish_photo.assert_not_called()
        self.assertEqual(sum(name == "rm_movej" for name, _ in self.commands), 1)
        self.assertFalse(self.client.motion_lock.locked())

    def test_frame_change_during_capture_rejects_photo(self):
        def changed(after, cancel, check):
            photo = self.snapshot(after, cancel, check)
            self.frame["name"] = "changed"
            return photo
        self.camera.snapshot.side_effect = changed
        self.assertEqual(self.observe()["state"], "error")
        self.plugin._publish_photo.assert_not_called()

    def test_stop_waits_for_the_current_call_to_cancel_and_release_resources(self):
        from concurrent.futures import ThreadPoolExecutor
        import threading

        entered = threading.Event()
        def snapshot(after, cancel, check):
            entered.set()
            cancel.wait(3)
            check()

        self.camera.snapshot.side_effect = snapshot
        with ThreadPoolExecutor(max_workers=1) as executor:
            pending = executor.submit(self.observe)
            self.assertTrue(entered.wait(3))
            self.assertEqual(self.plugin.stop()["state"], "idle")
            self.assertEqual(pending.result(timeout=1)["state"], "cancelled")
        self.assertFalse(self.client.motion_lock.locked())
        self.camera.stop.assert_called_once()
        self.plugin._publish_photo.assert_not_called()

    def test_read_feedback_rejects_idle_disagreement(self):
        import threading
        from pick_place.motion import ObservationMotion
        self.client.call_dict = lambda method: {"trajectory_type": 0, "data": [1]*7}
        with self.assertRaisesRegex(RuntimeError, "disagree"):
            ObservationMotion(self.client, threading.Event()).read()

    def test_settle_requires_idle_and_fresh_samples(self):
        import threading
        from pick_place.motion import ObservationMotion
        self.client.call_dict = lambda method: {"trajectory_type": 1, "data": self.joints[:]}
        with self.assertRaisesRegex(RuntimeError, "timeout"):
            ObservationMotion(self.client, threading.Event()).settled(self.joints, timeout=0.2)

    def test_all_motion_cards_share_driver_connection_and_upstream_motion_gate(self):
        device = load_device()
        plugins = device.build_plugins({}, "test", None)
        client = plugins[0]
        self.assertIs(plugins[1]._motion_lock, client.motion_lock)
        self.assertIs(client.motion_gate, client.motion_lock)
        self.assertIsNot(plugins[2]._gripper_lock, client.motion_lock)
        self.assertFalse(hasattr(plugins[3], "_motion_lock"))
        self.assertIs(plugins[1].client._client, client)
        self.assertIs(plugins[2].client._client, client)
        self.assertIs(plugins[3].client._client, client)
        self.assertIs(plugins[4].client._client, client)
        self.assertIs(plugins[4]._motion_lock, client.motion_gate)
        self.assertIs(plugins[5].client._client, client)


    def test_stop_failure_keeps_device_ownership(self):
        original = self.command
        def command(method, *args):
            original(method, *args)
            if method == "rm_set_arm_slow_stop":
                raise RuntimeError("stop failed")
        self.client.command = command
        self.camera.snapshot.side_effect = RuntimeError("camera failed")
        self.assertEqual(self.observe()["state"], "error")
        self.assertTrue(self.client.motion_lock.locked())
        self.assertTrue(self.plugin.dispatch("info", {})["motion_blocked"])
        self.assertEqual(self.observe()["state"], "error")

    def test_fault_during_motion_prevents_capture(self):
        original = self.command
        def command(method, *args):
            original(method, *args)
            if method == "rm_movej":
                self.state["joint_err_code"][6] = 0xF000
        self.client.command = command
        self.assertEqual(self.observe()["state"], "error")
        self.camera.snapshot.assert_not_called()
        self.plugin._publish_photo.assert_not_called()
        self.assertTrue(self.client.motion_lock.locked())

    def test_output_is_one_jpeg_message_with_capture_identity(self):
        import sys
        import types
        from unittest import mock
        from pick_place import PickPlacePlugin
        def message():
            return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(), frame_id=""))
        self.plugin._publisher = mock.Mock()
        photo = {"captured_at": 100.25, "jpeg": b"test-photo"}
        with mock.patch.dict(sys.modules, {"sensor_msgs.msg": types.SimpleNamespace(CompressedImage=message)}):
            PickPlacePlugin._publish_photo(self.plugin, photo, "photo-id")
        self.plugin._publisher.publish.assert_called_once()
        msg = self.plugin._publisher.publish.call_args.args[0]
        self.assertEqual(msg.data, b"test-photo")
        self.assertEqual(msg.format, "jpeg")
        self.assertEqual(msg.header.frame_id, "photo-id")
        self.assertEqual(msg.header.stamp.sec, 100)
        self.assertEqual(msg.header.stamp.nanosec, 250000000)


if __name__ == "__main__":
    unittest.main()
