from pathlib import Path
import subprocess
import sys
import unittest

from test_realman_rm75_driver import DRIVER, ROOT, load_device
from common.vendor_runtime import DriverBundle

sys.path.insert(0, str(DRIVER))


def wait_for_completion(plugin, response):
    """Wait only in offline tests; production completion is delivered through ACP."""
    import threading
    import time
    if response["state"] != "running":
        return response
    deadline = time.perf_counter() + 6
    while time.perf_counter() < deadline:
        terminal = plugin.dispatch("info", {})["last_result"]
        if (terminal and terminal["action_id"] == response["action_id"]
                and terminal["callback"] != "pending"):
            return terminal
        threading.Event().wait(.005)
    raise AssertionError(f"No completion for {response!r}")


class PickPlaceConfigTests(unittest.TestCase):
    def setUp(self):
        device = load_device()
        self.bundle = DriverBundle(device.build_plugins({}, "rm75", None))

    def configure(self, **values):
        return self.bundle.dispatch("vision_pick_and_drop", {"action": "config", **values})

    def test_driver_speed_limits_apply_to_schema_defaults_and_atomic_updates(self):
        from pick_place import PickPlacePlugin
        for safety, maximum, default in (
            ({}, 10, 5),
            ({"max_speed_percent": 3}, 3, 3),
            ({"max_speed_percent": 7, "default_speed_percent": 2}, 7, 2),
            ({"max_speed_percent": 7, "default_speed_percent": 9}, 7, 7),
            ({"max_speed_percent": 100, "default_speed_percent": 50}, 10, 10),
        ):
            with self.subTest(safety=safety):
                card = PickPlacePlugin(object(), {"safety": safety})
                speed = card.get_tools()[0]["configSchema"]["properties"]["speed_percent"]
                self.assertEqual((speed["maximum"], speed["default"]), (maximum, default))
                self.assertEqual(card.dispatch("config", {})["speed_percent"], default)
                accepted = card.dispatch("config", {"speed_percent": maximum})
                self.assertTrue(accepted["ok"])
                for value in (maximum + 1, 50, 100):
                    rejected = card.dispatch("config", {"speed_percent": value, "pick_grip_force": 20})
                    self.assertEqual(rejected["code"], "INVALID_CONFIG")
                    self.assertEqual(card.dispatch("config", {}), accepted)
        # Instance-specific limits must not change the next card's schema.
        self.assertEqual(PickPlacePlugin(object(), {}).get_tools()[0]["configSchema"]
                         ["properties"]["speed_percent"]["maximum"], 10)

    def test_registered_config_defaults(self):
        card = next(tool for tool in self.bundle.get_all_tools() if tool["name"] == "vision_pick_and_drop")
        self.assertEqual(card["type"], "actuator")
        self.assertEqual([p["format"] for p in card["topic_in"]], ["image/depth-zlib", "image/jpeg", "data/json"])
        self.assertNotIn("topic_out", card)
        expected = {
            "speed_percent": 5, "observation_joints_deg": "-90,0,0,90,0,90,0",
            "x_compensation_mm": 30, "y_compensation_mm": -75,
            "pick_descent_mm": 91, "pick_grip_force": 15, "place_descent_mm": 60, "observe_after_transfer": False,
        }
        self.assertEqual({key: prop["default"] for key, prop in card["configSchema"]["properties"].items()}, expected)
        self.assertEqual(self.configure(), {"ok": True, **expected})

    def test_motion_actions_declare_confirmation_and_acp_completion(self):
        card = next(tool for tool in self.bundle.get_all_tools() if tool["name"] == "vision_pick_and_drop")
        schema = card["inputSchema"]
        self.assertEqual(set(schema["properties"]["action"]["enum"]), {"observe", "grab_to", "grab_by", "cancel"})
        self.assertEqual(set(schema["properties"]), {"action", "start_point_x", "start_point_y", "target_point_x", "target_point_y", "delta_x", "delta_y", "rotation_deg", "confirm_motion"})
        self.assertEqual(schema["x-action-params"]["observe"]["params"], ["confirm_motion"])
        self.assertEqual(schema["x-action-params"]["grab_to"]["params"], ["start_point_x", "start_point_y", "target_point_x", "target_point_y", "rotation_deg", "confirm_motion"])
        self.assertEqual(schema["x-action-params"]["grab_by"]["params"], ["start_point_x", "start_point_y", "delta_x", "delta_y", "rotation_deg", "confirm_motion"])
        for action in ("grab_to", "grab_by"):
            params = schema["x-action-params"][action]["params"]
            self.assertEqual([name for name in schema["properties"] if name in params], params)
            self.assertEqual(params[-2:], ["rotation_deg", "confirm_motion"])
        for name in ("start_point_x", "start_point_y", "target_point_x", "target_point_y"):
            prop = schema["properties"][name]
            self.assertEqual((prop["type"], prop["minimum"], prop["maximum"]), ("number", -1, 1))
            self.assertIn("归一化", prop["description"])
            self.assertIn("正方向向右" if name.endswith("_x") else "正方向向下", prop["description"])
        self.assertIn("position[0]", schema["properties"]["start_point_x"]["description"])
        self.assertIn("position[1]", schema["properties"]["start_point_y"]["description"])
        for name in ("delta_x", "delta_y"):
            self.assertEqual(schema["properties"][name]["type"], "number")
            self.assertNotIn("minimum", schema["properties"][name])
            self.assertIn("mm", schema["properties"][name]["description"])
        self.assertIn("X 正方向向右（正值），负方向向左（负值）", schema["properties"]["delta_x"]["description"])
        self.assertIn("Y 正方向向照片下方（正值），负方向向上方（负值）", schema["properties"]["delta_y"]["description"])
        self.assertEqual(schema["required"], ["action"])
        self.assertEqual(schema["x-completion"], {"actions": ["observe", "grab_to", "grab_by"], "timeout": 210})
        self.assertEqual(schema["properties"]["confirm_motion"]["type"], "boolean")
        self.assertIs(schema["properties"]["confirm_motion"]["const"], True)
        self.assertEqual(schema["allOf"][0]["then"]["required"], ["confirm_motion"])
        self.assertEqual(schema["allOf"][0]["if"]["properties"]["action"]["enum"],
                         ["observe", "grab_to", "grab_by"])
        self.assertEqual(schema["x-action-params"]["cancel"]["params"], [])
        self.assertTrue(schema["x-is-dangerous"])
        self.assertEqual(schema["x-resource"], "arm")
        self.assertEqual(schema["x-hooks"]["on_interrupt_motion"], {"action": "cancel"})
        self.assertEqual(schema["x-hooks"]["on_interrupt_all"], {"action": "cancel"})

    def test_rotation_is_optional_and_explicitly_requested_in_mcp_guidance(self):
        card = next(tool for tool in self.bundle.get_all_tools() if tool["name"] == "vision_pick_and_drop")
        schema = card["inputSchema"]
        self.assertEqual(schema["properties"]["rotation_deg"]["default"], 0)
        self.assertEqual(schema["properties"]["rotation_deg"]["minimum"], -180)
        self.assertEqual(schema["properties"]["rotation_deg"]["maximum"], 180)
        self.assertNotIn("rotation_deg", schema["required"])
        for name in ("grab_to", "grab_by"):
            desc = schema["x-action-params"][name]["description"]
            for text in ("常规搬运省略", "用户明确要求", "俯视顺时针", "逆时针", "rotation_deg"):
                self.assertIn(text, desc)
        self.assertEqual([p["desc"] for p in card["topic_in"]], ["深度图像", "RGB 图像", "VOP 物品列表"])
        self.assertNotIn("pick_place", [t["name"] for t in self.bundle.get_all_tools()])

    def test_each_mcp_action_exposes_observation_lifecycle(self):
        card = next(tool for tool in self.bundle.get_all_tools() if tool["name"] == "vision_pick_and_drop")
        actions = card["inputSchema"]["x-action-params"]
        self.assertIn("result.objects", actions["observe"]["description"])
        for name in ("grab_to", "grab_by"):
            description = actions[name]["description"]
            for requirement in ("observe", "captured_at", "position[0]", "position[1]",
                                "每张照片仅供一次搬运", "observation_required=true", "不自动重试", "observe_after_transfer", "observation.skipped=true",
                                "holding_object_possible", "release_completed", "recovery_required=true"):
                self.assertIn(requirement, description)

    def test_partial_updates_and_invalid_updates_are_atomic(self):
        configured = self.configure(speed_percent=7, x_compensation_mm=-1.5)
        self.assertTrue(configured["ok"])
        self.assertEqual(configured["pick_grip_force"], 15)
        for invalid in (
            {"speed_percent": 0}, {"speed_percent": 11}, {"speed_percent": 1.5},
            {"speed_percent": True}, {"pick_grip_force": -1}, {"pick_grip_force": 101},
            {"pick_grip_force": 2.5}, {"pick_descent_mm": 0}, {"place_descent_mm": -1},
            {"x_compensation_mm": float("nan")}, {"y_compensation_mm": float("inf")},
            {"pick_descent_mm": "91"}, {"place_descent_mm": None},
            {"observe_after_transfer": 1}, {"observe_after_transfer": 0},
            {"observe_after_transfer": "true"}, {"observe_after_transfer": None},
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
        self.assertTrue(self.configure(speed_percent=10, pick_grip_force=0,
                                      pick_descent_mm=0.5, place_descent_mm=0.5,
                                      observation_joints_deg="178,130,178,135,178,128,360")["ok"])
        self.assertTrue(self.configure(speed_percent=1, pick_grip_force=100,
                                      observation_joints_deg="-178,-130,-178,-135,-178,-128,-360")["ok"])
        for action in ("observe", "grab_to", "grab_by"):
            self.assertEqual(self.bundle.dispatch("vision_pick_and_drop", {"action": action})["state"], "error")


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
    __import__("test_realman_pick_place_transfer").TransferTests("test_grab_by_uses_pick_point_and_configured_millimetres"),
]))
sys.exit(not result.wasSuccessful())
'''
        result = subprocess.run([sys.executable, "-c", script], cwd=ROOT,
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_other_cards_configuration_does_not_configure_pick_place(self):
        from pick_place import PickPlacePlugin
        from pick_place.inputs import ObservationInputs
        from hardware import RM75SDKClient
        config = {"ext_camera": {"enabled": False, "serial_number": "unrelated"},
                  "vision_capture": {"enabled": False, "output_dir": "/unrelated"}}
        card = PickPlacePlugin(RM75SDKClient({}).exclusive_client(), config)
        self.assertIsInstance(card._inputs, ObservationInputs)
        self.assertIsNone(card._observation)
        self.assertEqual(card.dispatch("config", {})["speed_percent"], 5)


class ObserveTests(unittest.TestCase):
    def setUp(self):
        import copy
        import threading
        import types
        from unittest import mock
        from pick_place import PickPlacePlugin
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
        self.camera.identity.return_value = {"topics": ["/test/rgb", "/test/depth", "/test/rgb/objects"], "serial_number": "D435-test", "session_id": "session-test"}
        self.camera.topics.return_value = [{"topic": t, "format": f} for t, f in zip(self.camera.identity()["topics"], ("image/jpeg", "image/depth-zlib", "data/json"))]
        self.plugin = PickPlacePlugin(self.client, {}, inputs=self.camera)
        self.copy = copy.deepcopy
        self.completion_factory = self.enterContext(mock.patch("pick_place.Completion"))
        self.completion_factory.return_value.send.return_value = ("accepted", None)

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
                "intrinsics": {"fx": 500}, "depth_scale_m": 0.001,
                "objects": [{"name": "banana", "position": [.1, .2], "confidence": .9}],
                "objects_timestamp": after + .01, "input_identity": self.camera.identity()}

    def observe(self):
        return wait_for_completion(self.plugin, self.plugin.dispatch("observe", {"confirm_motion": True}))

    def test_stop_does_not_wait_for_subscription_start_or_allow_late_restart(self):
        import threading
        entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
        results = []
        def start(args, cancel):
            entered.set()
            if not release.wait(3):
                raise RuntimeError("test setup release timed out")
            self.assertTrue(cancel.is_set())
        self.camera.start.side_effect = start
        worker = threading.Thread(target=lambda: results.append(self.plugin.start({"input_topics": []})))
        stopper = threading.Thread(target=lambda: (self.plugin.stop(), stopped.set()))
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.plugin.start()["state"], "error")
            self.assertEqual(self.plugin.dispatch("observe", {"confirm_motion": True})["state"], "error")
            stopper.start()
            self.assertTrue(stopped.wait(1), "stop must not wait for ROS setup")
            self.camera.stop.assert_called_once()
            self.assertEqual(self.plugin.start()["state"], "error")
        finally:
            release.set()
            worker.join(2)
            if stopper.ident is not None:
                stopper.join(2)
        self.assertEqual(results, [{"state": "idle"}])
        self.assertIsNone(self.plugin._starting)
        self.assertFalse(self.plugin._stopping)
        self.camera.start.side_effect = None
        self.assertEqual(self.plugin.start()["state"], "ready")
        self.assertEqual(self.commands, [])

    def test_call_returns_after_one_move_and_one_photo(self):
        import json
        from unittest import mock
        with mock.patch("builtins.open", side_effect=AssertionError("Observation must stay in memory")), \
                mock.patch.object(Path, "open", side_effect=AssertionError("Observation must stay in memory")), \
                mock.patch.object(Path, "mkdir", side_effect=AssertionError("Observation needs no output directory")):
            result = self.observe()
        self.assertEqual(result["state"], "completed", result)
        self.assertFalse(result["observation_required"])
        self.assertFalse(self.plugin.dispatch("info", {})["observation_required"])
        self.assertIsNone(self.plugin._active)
        self.assertTrue(result["action_id"].startswith("vision_pick_and_drop_observe_"))
        self.assertNotIn("request_id", result)
        self.assertEqual(self.commands, [("rm_movej", ([-90., 0., 0., 90., 0., 90., 0.], 5, 0, 0, 0))])
        self.camera.snapshot.assert_called_once()
        self.assertEqual(result["result"]["objects"], [{"name": "banana", "position": [.1, .2], "confidence": .9}])
        snapshot = self.plugin._observation
        self.assertEqual(snapshot["jpeg"], b"\xff\xd8test\xff\xd9")
        self.assertEqual(snapshot["depth_zlib"], b"test-depth")
        metadata = snapshot["metadata"]
        public = self.plugin.dispatch("info", {})
        self.assertEqual(public["observation"], result["result"])
        for field in ("file_path", "metadata_path", "jpeg", "depth_zlib", "metadata"):
            self.assertNotIn(field, public["observation"])
            self.assertNotIn(field, public["last_result"]["result"])
        json.dumps(public)
        json.dumps(self.completion_factory.return_value.send.call_args.args[-1])
        self.assertEqual(metadata["depth_aligned_to"], "color")
        self.assertEqual(metadata["arm_endpoint"], "test-arm:8080")
        self.assertEqual(metadata["joint_degree"], [-90., 0., 0., 90., 0., 90., 0.])
        self.assertFalse(self.client.motion_lock.locked())
        self.camera.stop.assert_not_called()

    def test_snapshot_owns_input_context_and_public_results_cannot_mutate_it(self):
        photo = self.snapshot(1234, None, lambda: None)
        self.camera.snapshot.side_effect = lambda *args: photo
        terminal = self.observe()
        self.assertEqual(terminal["state"], "completed", terminal)
        photo["intrinsics"]["fx"] = 1
        photo["objects"][0]["position"][0] = -.9
        public = self.plugin.dispatch("info", {})
        public["observation"]["pose"][0] = 99
        public["observation"]["objects"][0]["position"][0] = -.8
        terminal["result"]["objects"][0]["position"][0] = -.7
        self.assertEqual(self.plugin._observation["metadata"]["intrinsics"]["fx"], 500)
        self.assertEqual(self.plugin._observation["metadata"]["pose"][0], .1)
        self.assertEqual(self.plugin._observation["result"]["objects"][0]["position"], [.1, .2])

    def test_cancellation_before_snapshot_commit_leaves_no_usable_observation(self):
        from unittest import mock
        prepare = self.plugin._make_observation
        def cancelled(*args):
            observation = prepare(*args)
            self.plugin.dispatch("cancel", {})
            return observation
        with mock.patch.object(self.plugin, "_make_observation", side_effect=cancelled):
            result = self.observe()
        self.assertEqual(result["state"], "cancelled", result)
        self.assertTrue(result["observation_required"])
        self.assertIsNone(self.plugin._observation)

    def test_camera_session_change_invalidates_saved_observation_without_motion(self):
        self.assertEqual(self.observe()["state"], "completed")
        count = len(self.commands)
        self.camera.identity.return_value = {**self.camera.identity(), "session_id": "new-session"}
        self.assertTrue(self.plugin.dispatch("info", {})["observation_required"])
        result = self.plugin.dispatch("grab_by", {
            "confirm_motion": True, "start_point_x": 0, "start_point_y": 0, "delta_x": 10, "delta_y": 0})
        self.assertEqual(result["code"], "OBSERVATION_REQUIRED")
        self.assertEqual(len(self.commands), count)

    def test_each_call_takes_a_new_photo(self):
        first, second = self.observe(), self.observe()
        self.assertEqual(first["state"], "completed")
        self.assertEqual(second["state"], "completed")
        self.assertNotEqual(first["result"]["observation_id"], second["result"]["observation_id"])
        self.assertEqual(sum(name == "rm_movej" for name, _ in self.commands), 2)
        self.assertEqual(self.camera.snapshot.call_count, 2)
        self.assertEqual(self.plugin.dispatch("info", {})["observation"], second["result"])
        self.assertEqual(first["result"]["objects"], second["result"]["objects"])

    def test_observe_uses_configured_speed_and_joints(self):
        self.assertTrue(self.plugin.dispatch("config", {
            "speed_percent": 7, "observation_joints_deg": "-80,1,2,85,3,80,4"})["ok"])
        result = self.observe()
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(self.commands, [("rm_movej", ([-80., 1., 2., 85., 3., 80., 4.], 7, 0, 0, 0))])

    def test_observe_obeys_lower_driver_speed_limit(self):
        from pick_place import PickPlacePlugin
        self.plugin = PickPlacePlugin(self.client, {"safety": {"max_speed_percent": 3}}, inputs=self.camera)
        self.assertEqual(self.plugin.dispatch("config", {"speed_percent": 4})["code"], "INVALID_CONFIG")
        self.assertEqual(self.commands, [])
        result = self.observe()
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(self.commands, [("rm_movej", ([-90., 0., 0., 90., 0., 90., 0.], 3, 0, 0, 0))])

    def test_activation_and_info_do_not_capture_or_move(self):
        self.plugin.start()
        self.plugin.dispatch("info", {})
        self.plugin.stop()
        self.assertEqual(self.commands, [])
        self.camera.snapshot.assert_not_called()
        self.assertIsNone(self.plugin._observation)

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
        self.camera.snapshot.assert_not_called()

    def test_joint_fault_blocks_before_motion(self):
        self.state["joint_err_code"][6] = 0xF000
        result = self.observe()
        self.assertEqual(result["state"], "error")
        self.assertFalse(self.commands)
        self.assertIsNone(self.plugin._observation)

    def test_camera_failure_blocks_before_motion(self):
        self.camera.info.return_value = {"state": "error", "fresh": False, "error": "disconnected"}
        self.assertEqual(self.observe()["state"], "error")
        self.assertFalse(self.commands)
        self.assertIsNone(self.plugin._observation)

    def test_capture_failure_stops_without_photo_or_return_motion(self):
        self.camera.snapshot.side_effect = RuntimeError("camera timeout")
        self.assertEqual(self.observe()["state"], "error")
        self.assertEqual([name for name, _ in self.commands], ["rm_movej", "rm_set_arm_slow_stop"])
        self.assertIsNone(self.plugin._observation)
        self.assertIsNone(self.plugin._observation)
        self.assertTrue(self.plugin.dispatch("info", {})["observation_required"])

    def test_worker_timeout_stops_motion_and_reports_error(self):
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
        self.assertIsNone(self.plugin._observation)

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
        self.assertIsNone(self.plugin._observation)
        self.assertEqual(sum(name == "rm_movej" for name, _ in self.commands), 1)
        self.assertFalse(self.client.motion_lock.locked())

    def test_frame_change_during_capture_rejects_photo(self):
        def changed(after, cancel, check):
            photo = self.snapshot(after, cancel, check)
            self.frame["name"] = "changed"
            return photo
        self.camera.snapshot.side_effect = changed
        self.assertEqual(self.observe()["state"], "error")
        self.assertIsNone(self.plugin._observation)

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
        self.assertIsNone(self.plugin._observation)

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
        self.assertIsNone(self.plugin._observation)
        self.assertTrue(self.client.motion_lock.locked())



if __name__ == "__main__":
    unittest.main()
