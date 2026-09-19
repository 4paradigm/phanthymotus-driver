"""Hardware ownership is enforced below card implementations."""

from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "realman/rm75_6f_v"))
from hardware import RM75SDKClient
from common.vendor_runtime import DriverBundle


class SDKImportTests(unittest.TestCase):
    def test_device_loads_in_fresh_process_without_changing_search_path(self):
        script = '''
import importlib.util, sys
from pathlib import Path
root = Path(sys.argv[1])
driver = root / "realman/rm75_6f_v"
sys.path.insert(0, str(root))
if sys.argv[2] == "runtime":
    sys.path.insert(0, str(driver))
    import hardware
search_path = list(sys.path)
spec = importlib.util.spec_from_file_location("realman_rm75_device", driver / "device.py")
device = importlib.util.module_from_spec(spec)
spec.loader.exec_module(device)
import hardware
assert sys.path == search_path
assert device.RM75SDKClient is hardware.RM75SDKClient
assert device.JOINT_LIMITS_DEG is hardware.JOINT_LIMITS_DEG
sys.modules["device"] = device
spec = importlib.util.spec_from_file_location("realman_rm75_servo", driver / "servo.py")
servo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(servo)
assert servo.build_descriptor()["joint_names"] == device.JOINT_NAMES
# Later normal driver imports must reuse the same SDK class and module globals.
sys.path.insert(0, str(driver))
import device as normal_device
import pick_place
assert normal_device.RM75SDKClient is hardware.RM75SDKClient
assert pick_place.JOINT_LIMITS_DEG is hardware.JOINT_LIMITS_DEG
'''
        root = Path(__file__).resolve().parents[1]
        for mode in ("file", "runtime"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                result = subprocess.run([sys.executable, "-I", "-c", script, str(root), mode],
                                        cwd=directory, capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_hardware_dependency_failure_is_not_hidden_by_import_fallback(self):
        script = '''
import importlib.util, sys
from pathlib import Path
root = Path(sys.argv[1])
sys.path[:0] = [sys.argv[2], str(root)]
spec = importlib.util.spec_from_file_location("realman_rm75_device", root / "realman/rm75_6f_v/device.py")
device = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(device)
except ModuleNotFoundError as exc:
    assert exc.name == "sdk_dependency_missing", exc
else:
    raise AssertionError("The missing dependency was hidden")
'''
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "hardware.py").write_text(
                'raise ModuleNotFoundError("SDK dependency missing", name="sdk_dependency_missing")\n')
            root = Path(__file__).resolve().parents[1]
            result = subprocess.run([sys.executable, "-I", "-c", script, str(root), directory],
                                    cwd=directory, capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class SDKOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.client = RM75SDKClient({})
        self.robot = mock.Mock()
        self.robot.rm_movej.return_value = 0
        self.robot.rm_set_arm_slow_stop.return_value = 0
        self.robot.rm_set_gripper_position.return_value = 0
        self.robot.rm_get_joint_degree.return_value = (0, [0] * 7)
        self.client._robot = self.robot
        self.client._handle = types.SimpleNamespace(id=1)
        self.owner = self.client.exclusive_client()

    def test_register_reads_forward_arguments_and_validate_sdk_result(self):
        self.robot.rm_get_rm_plus_reg.return_value = (0, [10, 100])
        self.assertEqual(self.owner.call("rm_get_rm_plus_reg", 1220, 2), [10, 100])
        self.robot.rm_get_rm_plus_reg.assert_called_once_with(1220, 2)
        self.robot.rm_get_rm_plus_reg.return_value = (1, [])
        with self.assertRaisesRegex(RuntimeError, "SDK code 1"):
            self.owner.call("rm_get_rm_plus_reg", 1220, 1)

    def test_uncooperative_commands_cannot_interrupt_exclusive_action(self):
        self.assertTrue(self.owner.motion_lock.acquire(blocking=False))
        try:
            for method in ("rm_movej", "rm_set_arm_slow_stop", "rm_set_gripper_position"):
                with self.subTest(method=method):
                    with self.assertRaisesRegex(RuntimeError, "reserved"):
                        self.client.command(method)
                    getattr(self.robot, method).assert_not_called()
            self.assertEqual(self.client.call("rm_get_joint_degree"), [0] * 7)
            self.owner.command("rm_movej", [0] * 7, 50, 0, 0, 0)
            self.robot.rm_movej.assert_called_once_with([0] * 7, 50, 0, 0, 0)
            with self.assertRaisesRegex(RuntimeError, "disconnect"):
                self.client.stop()
            self.assertTrue(self.client.connected)
        finally:
            self.owner.motion_lock.release()
        self.client.command("rm_set_arm_slow_stop")
        self.robot.rm_set_arm_slow_stop.assert_called_once_with()

    def test_scoped_commands_require_ownership_and_cannot_release_another_owner(self):
        with self.assertRaisesRegex(RuntimeError, "ownership"):
            self.owner.command("rm_movej")
        self.assertTrue(self.owner.acquire())
        other = self.client.exclusive_client()
        self.assertFalse(other.acquire())
        with self.assertRaisesRegex(RuntimeError, "another owner"):
            other.release()
        self.assertFalse(self.owner.acquire())
        self.owner.release()
        with self.assertRaisesRegex(RuntimeError, "ownership"):
            self.owner.command("rm_movej")
        self.robot.rm_movej.assert_not_called()

    def test_existing_action_ownership_blocks_pick_place(self):
        self.client.motion_lock.acquire()
        try:
            self.assertFalse(self.owner.acquire())
        finally:
            self.client.motion_lock.release()
        self.assertTrue(self.owner.acquire())
        self.owner.release()

    def test_active_joint_action_blocks_pick_place_and_can_still_stop(self):
        from test_realman_rm75_driver import load_device

        device = load_device()
        self.client.motion_enabled = True
        self.robot.rm_get_arm_all_state.return_value = (0, {
            "joint_err_code": [0] * 7, "joint_en_flag": [1] * 7,
            "err": {"err_len": 0, "err": []},
        })
        self.robot.rm_get_joint_drive_min_pos.return_value = (0, [v[0] for v in device.JOINT_LIMITS_DEG])
        self.robot.rm_get_joint_drive_max_pos.return_value = (0, [v[1] for v in device.JOINT_LIMITS_DEG])
        arm = device.RM75Plugin(self.client.shared_client(), {
            "safety": {"start_grace_seconds": 60, "poll_interval_seconds": 0.001},
        })
        completed = threading.Event()
        arm._acp_callback = mock.Mock(side_effect=lambda *args: completed.set())
        started = arm.dispatch("set", {
            "_tool_name": "joint_control", "joint1_deg": 10, "confirm_motion": True,
        })
        try:
            self.assertFalse(self.owner.acquire())
            self.assertEqual("running", started["state"])
        finally:
            stopped = arm.dispatch("stopmotion", {"_tool_name": "joint_control"})
        self.assertEqual(started["action_id"], stopped["action_id"])
        self.assertTrue(completed.wait(2))
        self.robot.rm_set_arm_slow_stop.assert_called_once_with()
        arm._acp_callback.assert_called_once_with(
            started["action_id"], "cancelled", {"reason": "stopmotion"})
        self.assertTrue(self.owner.acquire())
        self.owner.release()

    def test_active_servo_stream_blocks_pick_place_until_stop(self):
        from servo import RM75ServoPlugin

        self.client.motion_enabled = True
        servo = RM75ServoPlugin(self.client.shared_client(), {}, ros2=mock.Mock())
        servo._subscribe = mock.Mock()
        self.assertEqual("running", servo.dispatch("start", {"input_topic": "/control/test"})["state"])
        try:
            self.assertFalse(self.owner.acquire())
        finally:
            servo.dispatch("stop", {})
        self.assertTrue(self.owner.acquire())
        self.owner.release()

    def test_pick_place_ownership_blocks_other_registered_motion_cards(self):
        from test_realman_rm75_driver import load_device
        from servo import RM75ServoPlugin

        device = load_device()
        self.client.motion_enabled = True
        shared = self.client.shared_client()
        arm = device.RM75Plugin(shared, {})
        gripper = device.GripperPlugin(shared, {})
        cartesian = device.CartesianPlugin(shared, {"cartesian": {"enabled": True}}, arm_plugin=arm)
        servo = RM75ServoPlugin(shared, {}, ros2=mock.Mock())
        self.assertTrue(self.owner.acquire())
        try:
            for card, action, args in (
                (arm, "set", {"_tool_name": "joint_control", "joint1_deg": 10, "confirm_motion": True}),
                (gripper, "set_position", {"position": 500, "confirm_motion": True}),
                (cartesian, "movel", {"speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True}),
            ):
                with self.subTest(card=type(card).__name__):
                    with self.assertRaisesRegex(RuntimeError, "another .*active"):
                        card.dispatch(action, args)
            result = servo.dispatch("start", {"input_topic": "/control/test"})
            self.assertEqual("error", result["state"])
            self.assertIn("another arm operation", result["message"])
            self.assertEqual([], self.robot.mock_calls)
        finally:
            self.owner.release()

    def test_upstream_sdk_write_paths_respect_exclusive_ownership(self):
        self.assertTrue(self.owner.acquire())
        try:
            with self.assertRaisesRegex(RuntimeError, "reserved"):
                self.client.command_trajectory("rm_movel", [0] * 6, 5, 0, 0, 0)
            with self.assertRaisesRegex(RuntimeError, "reserved"):
                self.client.upload_recording("unused.txt", 5, 1, run=True)
            with self.assertRaisesRegex(RuntimeError, "reserved"):
                self.client.command_interrupt("rm_set_arm_slow_stop")
            self.assertEqual("idle", self.client.trajectory_wait_state())
            self.assertEqual([], self.robot.mock_calls)
        finally:
            self.owner.release()

    def test_shared_client_preserves_connection_across_card_lifecycle_calls(self):
        from test_realman_rm75_driver import load_device

        shared = self.client.shared_client()
        card = load_device().RM75Plugin(shared, {})
        card.start()
        self.assertTrue(self.owner.acquire())
        try:
            card.stop()
            self.assertTrue(self.client.connected)
            self.robot.rm_create_robot_arm.assert_not_called()
            self.robot.rm_delete_robot_arm.assert_not_called()
            with self.assertRaisesRegex(RuntimeError, "reserved"):
                shared.command("rm_set_gripper_position", 500, True, 30)
            self.robot.rm_set_gripper_position.assert_not_called()
        finally:
            self.owner.release()
        self.client.stop()
        self.robot.rm_delete_robot_arm.assert_called_once_with()

    def test_sdk_lifecycle_without_any_other_card(self):
        from pick_place import PickPlacePlugin

        client = RM75SDKClient({"arm_ip": "test-arm"})
        client.enabled = True
        robot = mock.Mock()
        robot.rm_create_robot_arm.return_value = types.SimpleNamespace(id=1)
        sdk = types.SimpleNamespace(RoboticArm=mock.Mock(return_value=robot),
                                    rm_event_callback_ptr=lambda callback: callback,
                                    rm_thread_mode_e=types.SimpleNamespace(RM_TRIPLE_MODE_E=3))
        card = PickPlacePlugin(client.exclusive_client(), {})
        bundle = DriverBundle([client, card])
        with tempfile.NamedTemporaryFile() as library, \
             mock.patch("hardware.SDK_LIBRARY_PATH", Path(library.name)), \
             mock.patch.dict(sys.modules, {"Robotic_Arm.rm_robot_interface": sdk}):
            bundle.start_all()
            self.assertTrue(client.connected)
            self.assertEqual([tool["name"] for tool in bundle.get_all_tools()], ["pick_place"])
            # Stopping the card does not disconnect the Driver's device resource.
            card.stop()
            self.assertTrue(client.connected)
            robot.rm_delete_robot_arm.assert_not_called()
            bundle.stop_all()
            self.assertFalse(client.connected)
        robot.rm_create_robot_arm.assert_called_once_with("test-arm", 8080)
        robot.rm_delete_robot_arm.assert_called_once_with()
        robot.rm_movej.assert_not_called()
