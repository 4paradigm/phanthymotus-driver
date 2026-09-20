"""Normalized-position transfers against a simulated arm and gripper; no hardware access."""

import ast
import ctypes
import math
import unittest
from unittest import mock
import zlib
from uuid import uuid4

import numpy as np

import test_realman_pick_place as fixtures
from pick_place.motion import ObservationMotion, rotation


# Use the bundled SDK's actual wire-to-Python shape without loading its ARM library.
sdk_source = fixtures.DRIVER / "vendor/Robotic_Arm/rm_ctypes_wrap.py"
sdk_node = next(node for node in ast.parse(sdk_source.read_text()).body
                if isinstance(node, ast.ClassDef) and node.name == "rm_plus_state_info_t")
sdk_types = {name: getattr(ctypes, name) for name in ("Structure", "c_int", "c_uint32")}
exec(compile(ast.Module(body=[sdk_node], type_ignores=[]), str(sdk_source), "exec"), sdk_types)
SDKGripperState = sdk_types["rm_plus_state_info_t"]


class TransferTests(unittest.TestCase):
    def setUp(self):
        fixtures.ObserveTests.setUp(self)
        self.plugin.dispatch("config", {"observe_after_transfer": True})
        self.pose[3] = math.pi
        self.grip_position, self.grip_force = 0, 100
        self.register_reads = []
        self.after_command = lambda method, args: None
        self.now = 1000.0
        self.on_wait = lambda: None
        # Advance the real settling/deadline logic without sleeping or bypassing checks.
        self.enterContext(mock.patch("pick_place.time.monotonic", side_effect=lambda: self.now))
        original = ObservationMotion.__init__
        def initialize(motion, client, cancel, deadline=None):
            original(motion, client, cancel, deadline)
            cancel.wait = self.wait
        self.enterContext(mock.patch.object(ObservationMotion, "__init__", initialize))
        self.depth = np.full((3, 4), 400, dtype="<u2")
        self.depth[2, 3] = 500
        self.make_photo()

    def wait(self, seconds):
        self.now += seconds
        self.on_wait()

    def snapshot(self, after, cancel, check):
        check()
        return {**self.copy(self.photo_data), "captured_at": after + .01,
                "objects": [{"name": "banana", "position": [.1, .2], "confidence": .9}],
                "objects_timestamp": after + .02, "input_identity": self.camera.identity()}

    def make_photo(self):
        photo = {"jpeg": b"photo", "depth_zlib": zlib.compress(self.depth.tobytes()),
                 "captured_at": 1234, "width": self.depth.shape[1], "height": self.depth.shape[0], "depth_scale_m": 0.001,
                 "intrinsics": {"fx": 10, "fy": 20, "ppx": 1, "ppy": 1,
                                "model": "distortion.none", "coeffs": [0]*5}}
        self.photo_data = photo
        self.photo_pose = self.pose[:]
        self.plugin._observation = self.plugin._make_observation(photo, {
            "observation_id": "photo-" + uuid4().hex,
            "config": dict(self.plugin._config)}, {"joints": self.joints, "pose": self.pose},
            {"work": self.copy(self.frame), "tool": self.copy(self.frame)})
        self.photo = self.copy(self.plugin._observation["result"])

    def edit_metadata(self, edit):
        edit(self.plugin._observation["metadata"])

    def call(self, method, *args):
        if method == "rm_get_rm_plus_base_info":
            return {"manu": "ZX", "type": 1, "dof": 1, "force": 1}
        if method == "rm_get_rm_plus_state_info":
            state = SDKGripperState()
            state.pos[0], state.dof_state[0] = self.grip_position, 2
            return state.to_dict()
        if method == "rm_get_rm_plus_reg":
            self.register_reads.append(args)
            return [self.grip_force] * args[1]
        return fixtures.ObserveTests.call(self, method)

    def command(self, method, *args):
        self.commands.append((method, self.copy(args)))
        if method == "rm_movel":
            self.pose = list(args[0])
        elif method == "rm_movej":
            self.joints = list(args[0])
            self.pose = self.photo_pose[:]
        elif method == "rm_set_rm_plus_reg":
            self.grip_force = args[2][0]
        elif method == "rm_set_hand_follow_pos":
            # Closing onto an object must still permit lifting.
            self.grip_position = args[0][0] or 300
        self.after_command(method, args)

    def transfer(self, **updates):
        return fixtures.wait_for_completion(self.plugin, self.plugin.dispatch("grab_to", {
            "confirm_motion": True, "start_point_x": -.5, "start_point_y": -1/3, "target_point_x": .5, "target_point_y": 1/3, **updates}))

    def grab_by(self, **updates):
        return fixtures.wait_for_completion(self.plugin, self.plugin.dispatch("grab_by", {
            "confirm_motion": True, "start_point_x": -.5, "start_point_y": -1/3, "delta_x": -30, "delta_y": 0, **updates}))

    def moves(self):
        return [args[0] for method, args in self.commands if method == "rm_movel"]

    def test_both_grabs_rotation_and_return_obey_lower_driver_speed_limit(self):
        from pick_place import PickPlacePlugin
        self.plugin = PickPlacePlugin(self.client, {"safety": {"max_speed_percent": 3}}, inputs=self.camera)
        self.assertTrue(self.plugin.dispatch("config", {"observe_after_transfer": True})["ok"])
        self.assertEqual(self.plugin.dispatch("config", {"speed_percent": 4})["code"], "INVALID_CONFIG")
        for action in (self.transfer, self.grab_by):
            with self.subTest(action=action.__name__):
                self.commands.clear()
                self.make_photo()
                result = action(rotation_deg=30)
                self.assertEqual(result["state"], "completed", result)
                self.assertTrue(result["result"]["rotation_completed"])
                self.assertTrue(result["result"]["observation"]["ok"])
                moves = [(name, args) for name, args in self.commands if name in ("rm_movej", "rm_movel")]
                self.assertEqual([name for name, _ in moves], ["rm_movel"] * 7 + ["rm_movej"])
                self.assertTrue(all(args[1] == 3 for _, args in moves))

    def test_grab_uses_memory_snapshot_without_reading_or_writing_files(self):
        from pathlib import Path
        for action in (self.transfer, self.grab_by):
            with self.subTest(action=action.__name__):
                self.make_photo()
                with mock.patch("builtins.open", side_effect=AssertionError("Unexpected file access")), \
                        mock.patch.object(Path, "open", side_effect=AssertionError("Unexpected file access")), \
                        mock.patch.object(Path, "mkdir", side_effect=AssertionError("Unexpected output directory")):
                    result = action()
                self.assertEqual(result["state"], "completed", result)
                self.assertTrue(result["result"]["observation"]["ok"])
                self.assertEqual(self.plugin._observation["result"], result["result"]["observation"])

    def test_optional_rotation_preserves_pick_place_positions_and_returns_to_observation(self):
        for action in (self.transfer, self.grab_by):
            for angle in (-180, -90, -30.5, 30.5, 90, 180):
                with self.subTest(action=action.__name__, angle=angle):
                    self.commands.clear()
                    self.make_photo()
                    result = action(rotation_deg=angle)
                    self.assertEqual(result["state"], "completed", result)
                    moves = self.moves()
                    self.assertEqual(len(moves), 7)
                    np.testing.assert_allclose(moves[3][:3], moves[2][:3])
                    expected = rotation(moves[2]) @ rotation([0, 0, 0, 0, 0, math.radians(angle)])
                    for pose in moves[3:]:
                        np.testing.assert_allclose(rotation(pose), expected, atol=1e-12)
                    # Positive Rz around a downward gripper sends its +X toward base -Y.
                    np.testing.assert_allclose(rotation(moves[3])[:, 0],
                        [math.cos(math.radians(angle)), -math.sin(math.radians(angle)), 0], atol=1e-12)
                    self.assertEqual(result["result"]["rotation_deg"], angle)
                    self.assertTrue(result["result"]["rotation_completed"])
                    self.assertTrue(result["result"]["release_completed"])
                    self.assertTrue(result["result"]["return_completed"])
                    self.assertEqual(self.pose, self.photo_pose)
                    self.assertTrue(all(args[1] == 5 for name, args in self.commands if name == "rm_movel"))

    def test_rotation_is_relative_to_gripper_with_rotated_work_frame(self):
        for work_angles, angle in (([.2, -.1, .3], 35), ([0, math.pi / 2, 0], 180)):
            with self.subTest(work_angles=work_angles):
                self.commands.clear()
                self.frame["pose"] = [.2, -.1, .05, *work_angles]
                work_rotation = rotation(self.frame["pose"])
                matrix = work_rotation.T @ rotation([0, 0, 0, math.pi, 0, 0])
                self.pose[3:] = [math.atan2(matrix[2, 1], matrix[2, 2]),
                                 math.asin(-matrix[2, 0]), math.atan2(matrix[1, 0], matrix[0, 0])]
                self.make_photo()
                result = self.grab_by(rotation_deg=angle)
                self.assertEqual(result["state"], "completed", result)
                moves = self.moves()
                expected = rotation([0, 0, 0, math.pi, 0, 0]) @ rotation([0, 0, 0, 0, 0, math.radians(angle)])
                np.testing.assert_allclose(work_rotation @ rotation(moves[3]), expected, atol=1e-12)
                np.testing.assert_allclose(moves[3][:3], moves[2][:3])

    def test_in_place_rotation_and_default_no_rotation(self):
        result = self.grab_by(delta_x=0, delta_y=0, rotation_deg=45)
        self.assertEqual(result["state"], "completed", result)
        np.testing.assert_allclose(result["result"]["pick_base_xy_mm"], result["result"]["place_base_xy_mm"])
        self.commands.clear()
        result = self.grab_by(rotation_deg=0)
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(len(self.moves()), 6)
        self.assertFalse(result["result"]["rotation_completed"])

    def test_rotation_validation_rejects_bad_angles_before_consuming_photo(self):
        for action in (self.transfer, self.grab_by):
            for angle in (None, True, "45", float("nan"), float("inf"), -180.01, 180.01):
                with self.subTest(action=action.__name__, angle=angle):
                    result = action(rotation_deg=angle)
                    self.assertEqual(result["state"], "error", result)
                    self.assertIn("rotation_deg", result["message"])
                    self.assertEqual(self.commands, [])
                    self.assertIsNotNone(self.plugin._observation)

    def test_rotation_waits_for_orientation_before_moving_to_place(self):
        pending = []
        def moving(method, args):
            if method == "rm_movel" and len(self.moves()) == 4:
                pending.append(list(args[0]))
                self.pose[5] = -math.pi / 4
        def finish():
            if pending:
                self.pose = pending.pop()
        self.after_command, self.on_wait = moving, finish
        result = self.transfer(rotation_deg=90)
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(len(self.moves()), 7)
        self.assertTrue(result["result"]["rotation_completed"])

    def test_rotation_fault_direction_and_cancel_stop_with_held_object_status(self):
        for failure in ("wrong_direction", "position", "fault", "cancel"):
            with self.subTest(failure=failure):
                self.commands.clear()
                self.pose = self.photo_pose[:]
                self.state["joint_err_code"] = [0] * 7
                self.make_photo()
                def fail(method, args):
                    if method != "rm_movel" or len(self.moves()) != 4:
                        return
                    if failure == "wrong_direction":
                        self.pose[5] = math.pi / 4
                    elif failure == "position":
                        self.pose[0] += .003
                    elif failure == "fault":
                        self.state["joint_err_code"][6] = 61440
                    else:
                        self.plugin.dispatch("cancel", {})
                self.after_command = fail
                result = self.transfer(rotation_deg=90)
                self.assertIn(result["state"], ("error", "cancelled"), result)
                self.assertEqual(len(self.moves()), 4)
                self.assertEqual(result["result"]["stage"], "rotate_gripper")
                self.assertFalse(result["result"]["rotation_completed"])
                self.assertFalse(result["result"]["release_completed"])
                self.assertTrue(result["result"]["holding_object_possible"])
                self.assertTrue(result["result"]["recovery_required"])
                self.assertTrue(result["observation_required"])
                self.assertFalse(any(name == "rm_movej" for name, _ in self.commands))
                # Reset the offline fault fixture between independent executions.
                self.plugin._motion_blocked = False
                if self.client.motion_lock.locked():
                    self.client.motion_lock.release()

    def test_complete_transfer_uses_configured_absolute_targets(self):
        self.assertTrue(self.plugin.dispatch("config", {"speed_percent": 7,
            "x_compensation_mm": 40, "y_compensation_mm": -60,
            "pick_grip_force": 35, "pick_descent_mm": 80, "place_descent_mm": 70})["ok"])
        self.make_photo()
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        expected = [[.14, .14, .3], [.14, .14, .22], [.14, .14, .3],
                    [.04, .165, .3], [.04, .165, .23], [.04, .165, .3]]
        np.testing.assert_allclose(np.array(self.moves())[:, :3], expected)
        for pose in self.moves():
            self.assertEqual(pose[3:], [math.pi, 0, 0])
        for method, args in self.commands:
            if method == "rm_movel":
                self.assertEqual(args[1:], (7, 0, 0, 0))
        self.assertEqual([args for method, args in self.commands if method == "rm_set_rm_plus_reg"],
                         [(1220, 1, [100]), (1220, 1, [35]), (1220, 1, [100])])
        self.assertEqual(self.register_reads, [(1220, 2), (1220, 1)] * 9)
        self.assertEqual([args[0][0] for method, args in self.commands if method == "rm_set_hand_follow_pos"],
                         [1000, 0, 1000, 0])
        self.assertFalse(result["result"]["grasp_checked"])
        self.assertEqual(result["result"]["pick_pixel"], [1, 1])
        self.assertEqual(result["result"]["place_pixel"], [3, 2])
        self.assertIsNone(self.plugin._active)
        observation = result["result"]["observation"]
        self.assertEqual(self.plugin._observation["result"], observation)
        self.assertEqual(result["result"]["final_pose"], self.photo_pose)
        self.assertFalse(result["observation_required"])
        self.assertTrue(result["result"]["transfer_completed"])
        self.assertFalse(self.client.motion_lock.locked())
        self.camera.snapshot.assert_called_once()
        self.assertEqual(self.commands[-1], ("rm_movej", ([-90., 0., 0., 90., 0., 90., 0.], 7, 0, 0, 0)))
        self.assertNotEqual(observation["observation_id"], self.photo["observation_id"])
        self.assertEqual(self.plugin._observation["jpeg"], b"photo")
        metadata = self.plugin._observation["metadata"]
        self.assertEqual(metadata["config"], self.plugin._config)
        self.assertEqual(metadata["pose"], self.photo_pose)

    def test_default_descent_is_directly_91_and_60_mm(self):
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        np.testing.assert_allclose(np.array(self.moves())[:, 2], [.3, .209, .3, .3, .24, .3])
        self.assertTrue(all(args[1] == 5 for name, args in self.commands if name == "rm_movel"))
        self.assertIn(("rm_set_rm_plus_reg", (1220, 1, [15])), self.commands)

    def test_both_transfers_consume_old_photo_and_replace_it_after_observing(self):
        for transfer in (self.transfer, self.grab_by):
            with self.subTest(action=transfer.__name__):
                self.make_photo()
                photo = self.copy(self.photo)
                def check_consumed(method, args):
                    self.assertIsNone(self.plugin._observation)
                self.after_command = check_consumed
                result = transfer()
                self.assertEqual(result["state"], "completed", result)
                self.assertFalse(result["observation_required"])
                info = self.plugin.dispatch("info", {})
                self.assertFalse(info["observation_required"])
                self.assertEqual(info["observation"], result["result"]["observation"])
                self.assertNotEqual(info["observation"]["observation_id"], photo["observation_id"])
                self.assertEqual(result["result"]["observation_id"], photo["observation_id"])
                self.assertEqual(info["last_result"], result)
                self.assertFalse(self.client.motion_lock.locked())

    def test_failed_first_command_consumes_photo_before_sdk_returns(self):
        for transfer in (self.transfer, self.grab_by):
            with self.subTest(action=transfer.__name__):
                self.make_photo()
                def fail(method, args):
                    if method == "rm_movel":
                        self.assertIsNone(self.plugin._observation)
                        self.assertTrue(self.plugin.dispatch("info", {})["observation_required"])
                        raise RuntimeError("SDK command failed")
                self.after_command = fail
                result = transfer()
                self.assertEqual(result["state"], "error", result)
                self.assertTrue(result["observation_required"])
                self.assertFalse(result["result"]["transfer_completed"])
                self.camera.snapshot.assert_not_called()
                self.assertFalse(any(name == "rm_movej" for name, _ in self.commands))
                count = len(self.commands)
                for repeated in (self.transfer, self.grab_by):
                    self.assertEqual(repeated()["code"], "OBSERVATION_REQUIRED")
                self.assertEqual(len(self.commands), count)
                self.assertFalse(self.client.motion_lock.locked())

    def test_grab_by_uses_pick_point_and_configured_millimetres(self):
        self.assertTrue(self.plugin.dispatch("config", {"speed_percent": 7,
            "x_compensation_mm": 40, "y_compensation_mm": -60,
            "pick_grip_force": 35, "pick_descent_mm": 80, "place_descent_mm": 70})["ok"])
        self.make_photo()
        result = self.grab_by(start_point_x=.5, start_point_y=1/3, delta_x=-30.5, delta_y=20.25)
        self.assertEqual(result["state"], "completed", result)
        expected = [[.04, .165, .3], [.04, .165, .22], [.04, .165, .3],
                    [.0705, .18525, .3], [.0705, .18525, .23], [.0705, .18525, .3]]
        np.testing.assert_allclose(np.array(self.moves())[:, :3], expected)
        self.assertTrue(all(args[1] == 7 for name, args in self.commands if name == "rm_movel"))
        self.assertEqual([args for name, args in self.commands if name == "rm_set_rm_plus_reg"],
                         [(1220, 1, [100]), (1220, 1, [35]), (1220, 1, [100])])
        self.assertEqual([args[0][0] for name, args in self.commands if name == "rm_set_hand_follow_pos"],
                         [1000, 0, 1000, 0])
        self.assertEqual(result["result"]["delta_x"], -30.5)
        self.assertEqual(result["result"]["delta_y"], 20.25)
        self.assertEqual(result["result"]["direction_reference"], "observation_image")
        self.assertEqual(result["result"]["pick_pixel"], [3, 2])
        self.assertNotIn("place_pixel", result["result"])
        self.assertEqual(self.plugin._observation["result"], result["result"]["observation"])
        self.assertFalse(self.client.motion_lock.locked())
        self.camera.snapshot.assert_called_once()
        self.assertEqual(self.commands[-1][0], "rm_movej")

    def test_grab_by_signed_image_directions(self):
        for dx, dy, base_delta in ((30, 0, [-30, 0]), (-30, 0, [30, 0]),
                                   (0, 25, [0, 25]), (0, -25, [0, -25]),
                                   (30, -25, [-30, -25])):
            with self.subTest(dx=dx, dy=dy):
                self.make_photo()
                result = self.grab_by(delta_x=dx, delta_y=dy)
                self.assertEqual(result["state"], "completed", result)
                data = result["result"]
                np.testing.assert_allclose(np.subtract(data["place_base_xy_mm"], data["pick_base_xy_mm"]), base_delta)

    def test_grab_by_needs_only_pick_depth_not_destination_pixels(self):
        self.depth[:] = 0
        self.depth[1, 1] = 400
        self.make_photo()
        result = self.grab_by()
        self.assertEqual(result["state"], "completed", result)
        np.testing.assert_allclose(result["result"]["place_base_xy_mm"], [160, 125])

    def test_grab_by_rejects_invalid_displacement_and_pick_before_commands(self):
        for args in ({"delta_x": None}, {"delta_y": None}, {"delta_x": True}, {"delta_y": "30"},
                     {"delta_x": float("nan")}, {"delta_y": float("inf")},
                     {"delta_x": 0, "delta_y": 0}, {"start_point_x": 1.0001}, {"start_point_y": -1.0001},
                     {"start_point_x": 691}, {"start_point_y": "0"}, {"start_point_x": True}, {"start_point_y": float("nan")}):
            with self.subTest(args=args):
                self.assertEqual(self.grab_by(**args)["state"], "error")
                self.assertEqual(self.commands, [])
                self.assertIsNotNone(self.plugin._observation)
        self.assertEqual(self.plugin.dispatch("grab_by", {"confirm_motion": True, "start_point_x": 0, "start_point_y": 0, "delta_x": 30})["state"], "error")
        self.depth[1, 1] = 0
        self.make_photo()
        self.assertIn("no valid depth", self.grab_by()["result"]["message"])
        self.assertEqual(self.commands, [])

    def test_grab_by_keeps_base_direction_with_rotated_work_frame(self):
        self.frame["pose"] = [.2, -.1, .05, .2, -.1, .3]
        work_rotation = rotation(self.frame["pose"])
        orientation = work_rotation.T @ rotation([0, 0, 0, math.pi, 0, 0])
        self.pose[3:] = [math.atan2(orientation[2, 1], orientation[2, 2]),
                         math.asin(-orientation[2, 0]), math.atan2(orientation[1, 0], orientation[0, 0])]
        self.make_photo()
        result = self.grab_by(delta_x=-30, delta_y=20)
        self.assertEqual(result["state"], "completed", result)
        bases = np.array([work_rotation @ pose[:3] + self.frame["pose"][:3] for pose in self.moves()])
        np.testing.assert_allclose(bases[3] - bases[0], [.03, .02, 0], atol=1e-12)
        np.testing.assert_allclose(bases[:, 2] - bases[0, 2], [0, -.091, 0, 0, -.060, 0], atol=1e-12)

    def test_grab_by_uses_shared_cancellation_and_consumes_photo(self):
        def cancel(method, args):
            if method == "rm_movel":
                self.assertEqual(self.plugin.dispatch("config", {"speed_percent": 1})["code"], "ACTION_IN_PROGRESS")
                self.assertEqual(self.transfer()["state"], "error")
                self.plugin.dispatch("cancel", {})
        self.after_command = cancel
        result = self.grab_by()
        self.assertEqual(result["state"], "cancelled", result)
        self.assertEqual([name for name, _ in self.commands], ["rm_movel", "rm_set_arm_slow_stop"])
        self.assertFalse(self.client.motion_lock.locked())
        self.assertIsNone(self.plugin._observation)
        self.assertTrue(result["observation_required"])
        for transfer in (self.transfer, self.grab_by):
            self.assertEqual(transfer()["code"], "OBSERVATION_REQUIRED")

    def test_observe_then_transfer_refreshes_photo_without_other_card_implementations(self):
        def snapshot(after, cancel, check):
            check()
            return {**self.photo_data, "captured_at": after + .01}
        self.camera.snapshot.side_effect = snapshot
        observed = fixtures.ObserveTests.observe(self)
        self.assertEqual(observed["state"], "completed", observed)
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(result["result"]["observation_id"], observed["result"]["observation_id"])
        self.assertEqual(sum(name == "rm_movej" for name, _ in self.commands), 2)
        self.assertEqual(len(self.moves()), 6)
        self.assertEqual(self.camera.snapshot.call_count, 2)
        self.camera.stop.assert_not_called()
        self.assertNotIn("topic_out", observed["result"])

    def test_follow_up_observation_allows_next_transfer_without_extra_observe(self):
        first = self.transfer()
        self.assertEqual(first["state"], "completed", first)
        latest = first["result"]["observation"]
        x, y = latest["objects"][0]["position"]
        second = self.grab_by(start_point_x=x, start_point_y=y, delta_x=10, delta_y=0)
        self.assertEqual(second["state"], "completed", second)
        self.assertEqual(second["result"]["observation_id"], latest["observation_id"])
        self.assertNotEqual(second["result"]["observation"]["observation_id"], latest["observation_id"])
        self.assertFalse(second["observation_required"])
        self.assertEqual(self.camera.snapshot.call_count, 2)
        self.assertEqual(sum(name == "rm_movej" for name, _ in self.commands), 2)
        self.assertEqual(len(self.moves()), 12)

    def test_follow_up_capture_failure_preserves_completed_transfer_and_requires_observe(self):
        self.camera.snapshot.side_effect = RuntimeError("VOP observation timed out")
        result = self.transfer()
        self.assertEqual(result["state"], "error", result)
        self.assertTrue(result["result"]["transfer_completed"])
        self.assertFalse(result["result"]["ok"])
        self.assertFalse(result["result"]["observation"]["ok"])
        self.assertIn("VOP observation timed out", result["result"]["observation"]["message"])
        self.assertEqual(result["result"]["stage"], "observing")
        self.assertEqual(result["result"]["pick_pixel"], [1, 1])
        self.assertNotIn("final_pose", result["result"])
        self.assertTrue(result["observation_required"])
        self.assertIsNone(self.plugin._observation)
        self.assertEqual([name for name, _ in self.commands][-2:], ["rm_movej", "rm_set_arm_slow_stop"])
        self.assertEqual(len(self.moves()), 6)
        self.assertEqual(self.grab_by()["code"], "OBSERVATION_REQUIRED")

    def test_return_uses_configured_observation_joints_and_cancel_does_not_capture(self):
        target = [-80., 1., 2., 85., 3., 80., 4.]
        self.plugin.dispatch("config", {"observation_joints_deg": "-80,1,2,85,3,80,4", "speed_percent": 7})
        self.make_photo()
        def cancel_on_return(method, args):
            if method == "rm_movej":
                self.assertEqual(args, (target, 7, 0, 0, 0))
                self.plugin.dispatch("cancel", {})
        self.after_command = cancel_on_return
        result = self.grab_by()
        self.assertEqual(result["state"], "cancelled", result)
        self.assertTrue(result["result"]["transfer_completed"])
        self.assertTrue(result["observation_required"])
        self.camera.snapshot.assert_not_called()
        self.assertEqual(sum(name == "rm_set_arm_slow_stop" for name, _ in self.commands), 1)
        self.assertFalse(self.client.motion_lock.locked())

    def test_cancel_during_follow_up_capture_discards_new_photo(self):
        def snapshot(after, cancel, check):
            self.plugin.dispatch("cancel", {})
            return self.snapshot(after, cancel, check)
        self.camera.snapshot.side_effect = snapshot
        result = self.transfer()
        self.assertEqual(result["state"], "cancelled", result)
        self.assertTrue(result["result"]["transfer_completed"])
        self.assertTrue(result["observation_required"])
        self.assertIsNone(self.plugin._observation)
        self.assertEqual(sum(name == "rm_set_arm_slow_stop" for name, _ in self.commands), 1)

    def test_follow_up_observation_has_its_own_bounded_time_budget(self):
        def delay_return(method, args):
            if method == "rm_movej":
                self.now += 38
        self.after_command = delay_return
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        self.assertGreater(self.now - 1000, 45)
        self.make_photo()
        def expire_return(method, args):
            if method == "rm_movej":
                self.now += 46
        self.after_command = expire_return
        failed = self.grab_by()
        self.assertEqual(failed["state"], "error", failed)
        self.assertTrue(failed["result"]["transfer_completed"])
        self.assertTrue(failed["observation_required"])
        self.assertIn("timed out", failed["result"]["message"])

    def test_busy_and_config_changes_are_rejected_until_follow_up_observation_finishes(self):
        def snapshot(after, cancel, check):
            self.assertTrue(self.client.motion_lock.locked())
            self.assertEqual(self.grab_by()["state"], "error")
            self.assertEqual(self.plugin.dispatch("config", {"speed_percent": 1})["code"], "ACTION_IN_PROGRESS")
            return self.snapshot(after, cancel, check)
        self.camera.snapshot.side_effect = snapshot
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        self.assertFalse(self.client.motion_lock.locked())

    def test_rotated_work_frame_preserves_base_horizontal_and_vertical_axes(self):
        self.frame["pose"] = [.2, -.1, .05, .2, -.1, .3]
        work_rotation = rotation(self.frame["pose"])
        # R_work^T * R_down expressed as an equivalent XYZ Euler pose.
        orientation = work_rotation.T @ rotation([0, 0, 0, math.pi, 0, 0])
        self.pose[3:] = [math.atan2(orientation[2, 1], orientation[2, 2]),
                         math.asin(-orientation[2, 0]), math.atan2(orientation[1, 0], orientation[0, 0])]
        origin = work_rotation @ self.pose[:3] + self.frame["pose"][:3]
        self.make_photo()
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        bases = np.array([work_rotation @ pose[:3] + self.frame["pose"][:3] for pose in self.moves()])
        np.testing.assert_allclose(bases[:, 2], origin[2] + np.array([0, -.091, 0, 0, -.060, 0]))
        np.testing.assert_allclose(bases[:3, :2], [origin[:2] + [.03, -.075]] * 3)
        np.testing.assert_allclose(bases[3:, :2], [origin[:2] + [-.07, -.05]] * 3)

    def test_both_positions_are_validated_before_any_command(self):
        for args in ({"start_point_x": None}, {"target_point_x": None}, {"start_point_y": -1.0001}, {"start_point_x": True},
                     {"target_point_y": 1.0001}, {"start_point_x": float("inf")}, {"target_point_x": float("nan")},
                     {"target_point_x": 700}, {"target_point_y": 350}, {"start_point_y": "0"}):
            with self.subTest(args=args):
                self.assertEqual(self.transfer(**args)["state"], "error")
                self.assertEqual(self.commands, [])
                self.assertIsNotNone(self.plugin._observation)
        self.depth[2, 3] = 0
        self.make_photo()
        result = self.transfer()
        self.assertIn("no valid depth", result["result"]["message"])
        self.assertEqual(self.commands, [])
        self.depth[2, 3], self.depth[1, 1] = 500, 0
        self.make_photo()
        self.assertEqual(self.transfer()["state"], "error")
        self.assertEqual(self.commands, [])

    def test_vop_position_uses_each_observations_width_and_height(self):
        for width, height, pixel in ((1280, 720, [691, 332]), (640, 480, [346, 221]),
                                     (256, 144, [138, 66])):
            with self.subTest(width=width, height=height):
                self.depth = np.zeros((height, width), dtype="<u2")
                self.depth[pixel[1], pixel[0]] = 400
                self.make_photo()
                self.edit_metadata(lambda data: data["intrinsics"].update(
                    fx=width, fy=height, ppx=width/2, ppy=height/2))
                result = self.grab_by(start_point_x=.08, start_point_y=-.079, delta_x=30, delta_y=0)
                self.assertEqual(result["state"], "completed", result)
                data = result["result"]
                self.assertEqual(data["pick_pixel"], pixel)
                np.testing.assert_allclose(np.subtract(data["place_base_xy_mm"], data["pick_base_xy_mm"]), [-30, 0])

    def test_normalized_center_and_edges_resolve_to_valid_pixels(self):
        for position, pixel in (((0, 0), [2, 2]), ((-1, -1), [0, 0]), ((1, 1), [3, 2]),
                                ((1, -1), [3, 0]), ((-1, 1), [0, 2]), ((.999, .999), [3, 2])):
            with self.subTest(position=position):
                self.make_photo()
                result = self.transfer(start_point_x=position[0], start_point_y=position[1], target_point_x=position[0], target_point_y=position[1])
                self.assertEqual(result["state"], "completed", result)
                self.assertEqual(result["result"]["pick_pixel"], pixel)
                self.assertEqual(result["result"]["place_pixel"], pixel)

    def test_normalized_xy_signs_preserve_installed_base_directions(self):
        for target_point_x, target_point_y, pixel, delta in ((.5, -1/3, [3, 1], [-80, 0]),
                                    (-1, -1/3, [0, 1], [40, 0]),
                                    (-.5, 1/3, [1, 2], [0, 20]),
                                    (-.5, -1, [1, 0], [0, -20])):
            with self.subTest(target_point_x=target_point_x, target_point_y=target_point_y):
                self.make_photo()
                result = self.transfer(target_point_x=target_point_x, target_point_y=target_point_y)
                self.assertEqual(result["state"], "completed", result)
                data = result["result"]
                self.assertEqual(data["pick_pixel"], [1, 1])
                self.assertEqual(data["place_pixel"], pixel)
                np.testing.assert_allclose(np.subtract(data["place_base_xy_mm"], data["pick_base_xy_mm"]), delta, atol=1e-10)

    def test_missing_arguments_and_missing_observation_do_not_move(self):
        self.assertEqual(self.plugin.dispatch("grab_to", {"confirm_motion": True})["state"], "error")
        self.plugin._observation = None
        self.assertIn("Run observe", self.transfer()["message"])
        self.assertEqual(self.commands, [])

    def test_configuration_change_requires_new_observation(self):
        self.plugin.dispatch("config", {"speed_percent": 7})
        self.assertIn("Run observe", self.transfer()["message"])
        self.assertEqual(self.commands, [])

    def test_mismatched_photo_identity_and_context_reject_before_motion(self):
        for edit in (lambda data: data.update(arm_endpoint="another-arm"),
                     lambda data: data.update(observation_id="another-photo"),
                     lambda data: data["config"].update(pick_descent_mm=90),
                     lambda data: data.update(depth_aligned_to="depth")):
            with self.subTest(edit=edit):
                self.make_photo()
                self.edit_metadata(edit)
                self.assertEqual(self.transfer()["state"], "error")
                self.assertEqual(self.commands, [])

    def test_changed_frame_pose_or_joint_fault_reject_before_motion(self):
        for change in (lambda: self.frame.update(name="another-frame"),
                       lambda: self.pose.__setitem__(2, .31),
                       lambda: self.pose.__setitem__(3, math.pi - .04),
                       lambda: self.state["joint_err_code"].__setitem__(6, 0xF000)):
            with self.subTest(change=change):
                frame, pose, state = self.copy(self.frame), self.pose[:], self.copy(self.state)
                change()
                self.assertEqual(self.transfer()["state"], "error")
                self.assertEqual(self.commands, [])
                self.frame, self.pose, self.state = frame, pose, state

    def test_horizontal_height_drift_stops_before_opening(self):
        def drift(method, args):
            if method == "rm_movel":
                self.pose[2] += .002
        self.after_command = drift
        result = self.transfer()
        self.assertEqual(result["state"], "error")
        self.assertIn("changed height", result["result"]["message"])
        self.assertEqual([name for name, _ in self.commands], ["rm_movel", "rm_set_arm_slow_stop"])

    def test_vertical_xy_drift_stops_without_close_or_return(self):
        def drift(method, args):
            if method == "rm_movel" and len(self.moves()) == 2:
                self.pose[0] += .003
        self.after_command = drift
        result = self.transfer()
        self.assertEqual(result["state"], "error")
        self.assertIn("fixed XY", result["result"]["message"])
        self.assertEqual(len(self.moves()), 2)
        self.assertEqual([args[0][0] for method, args in self.commands if method == "rm_set_hand_follow_pos"], [1000])
        self.assertEqual(self.commands[-1][0], "rm_set_arm_slow_stop")

    def test_force_mismatch_writes_once_and_never_opens_or_descends(self):
        original = self.call
        def call(method, *args):
            return [99] * args[1] if method == "rm_get_rm_plus_reg" else original(method, *args)
        self.client.call = call
        result = self.transfer()
        self.assertEqual(result["state"], "error")
        self.assertIn("force setting was not confirmed", result["result"]["message"])
        self.assertEqual([name for name, _ in self.commands], ["rm_movel", "rm_set_rm_plus_reg", "rm_set_arm_slow_stop"])

    def test_opening_must_arrive_before_descent(self):
        def stuck(method, args):
            if method == "rm_set_hand_follow_pos":
                self.grip_position = 950
        self.after_command = stuck
        result = self.transfer()
        self.assertEqual(result["state"], "error")
        self.assertIn("opening did not reach", result["result"]["message"])
        self.assertEqual(len(self.moves()), 1)
        self.assertEqual(self.commands[-1][0], "rm_set_arm_slow_stop")

    def test_cancel_after_close_prevents_lifting_and_placing(self):
        def cancel(method, args):
            if method == "rm_set_hand_follow_pos" and args[0][0] == 0:
                self.plugin.dispatch("cancel", {})
        self.after_command = cancel
        result = self.transfer()
        self.assertEqual(result["state"], "cancelled", result)
        self.assertEqual(len(self.moves()), 2)
        self.assertEqual(self.commands[-1][0], "rm_set_arm_slow_stop")
        self.assertEqual(sum(name == "rm_set_arm_slow_stop" for name, _ in self.commands), 1)
        self.assertFalse(self.client.motion_lock.locked())

    def test_busy_and_configuration_update_are_rejected_during_transfer(self):
        checked = []
        def check(method, args):
            if method == "rm_movel" and not checked:
                checked.append(True)
                self.assertEqual(self.transfer()["state"], "error")
                self.assertEqual(self.plugin.dispatch("observe", {"confirm_motion": True})["state"], "error")
                self.assertEqual(self.plugin.dispatch("config", {"speed_percent": 1})["code"], "ACTION_IN_PROGRESS")
        self.after_command = check
        self.assertEqual(self.transfer()["state"], "completed")
        self.assertEqual(checked, [True])

    def test_sdk_error_and_unverified_stop_keep_device_reserved(self):
        def fail(method, args):
            raise RuntimeError("SDK failure")
        self.after_command = fail
        result = self.transfer()
        self.assertEqual(result["state"], "error")
        self.assertEqual(len(self.moves()), 1)
        self.assertIn("stop_error", result["result"])
        self.assertTrue(self.client.motion_lock.locked())
        self.assertTrue(self.plugin.dispatch("info", {})["motion_blocked"])

    def test_action_timeout_stops_before_further_commands(self):
        def timeout(method, args):
            if method == "rm_movel":
                self.now += 121
        self.after_command = timeout
        result = self.transfer()
        self.assertEqual(result["state"], "error", result)
        self.assertIn("timed out", result["result"]["message"])
        self.assertEqual([name for name, _ in self.commands], ["rm_movel", "rm_set_arm_slow_stop"])
        self.assertFalse(self.client.motion_lock.locked())

    def test_gripper_fault_during_closing_aborts_without_lift(self):
        original = self.call
        def call(method, *args):
            result = original(method, *args)
            if method == "rm_get_rm_plus_state_info" and self.grip_position == 300:
                result["dof_err"] = [1]
            return result
        self.client.call = call
        result = self.transfer()
        self.assertEqual(result["state"], "error")
        self.assertEqual(len(self.moves()), 2)
        self.assertIn("faulted", result["result"]["message"])
        self.assertEqual(self.commands[-1][0], "rm_set_arm_slow_stop")

    def test_missing_or_faulted_gripper_system_state_rejects_before_motion(self):
        original = self.call
        for system_state in (None, 1):
            with self.subTest(system_state=system_state):
                def call(method, *args):
                    result = original(method, *args)
                    if method == "rm_get_rm_plus_state_info":
                        if system_state is None:
                            result.pop("sys_state")
                        else:
                            result["sys_state"] = system_state
                    return result
                self.client.call = call
                self.assertEqual(self.transfer()["state"], "error")
                self.assertEqual(self.commands, [])

    def test_within_tolerance_feedback_does_not_accumulate_into_targets(self):
        def drift(method, args):
            if method == "rm_movel":
                self.pose[0] += .0004
                self.pose[2] += .0004
        self.after_command = drift
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        expected_z = [.3, .209, .3, .3, .24, .3]
        np.testing.assert_allclose(np.array(self.moves())[:, 2], expected_z)
        np.testing.assert_allclose(np.array(self.moves())[:3, 0], [.13] * 3)
        np.testing.assert_allclose(np.array(self.moves())[3:, 0], [.03] * 3)

    def test_small_capture_pose_change_waits_and_refreshes_reference(self):
        calls = []
        def snapshot(after, cancel, check):
            if not calls:
                self.pose[0] += .0005
                self.assertIs(check(), False)
                self.assertIs(check(), True)
            calls.append(True)
            return self.snapshot(after, cancel, check)
        self.camera.snapshot.side_effect = snapshot
        result = fixtures.wait_for_completion(self.plugin, self.plugin.dispatch("observe", {"confirm_motion": True}))
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(result["result"]["pose"], self.pose)
        self.assertEqual(sum(m == "rm_movej" for m, _ in self.commands), 1)

    def test_short_capture_input_gap_requests_a_fresh_window(self):
        def snapshot(after, cancel, check):
            self.camera.info.return_value = {"state": "starting", "fresh": False}
            self.assertIs(check(), False)
            self.camera.info.return_value = {"state": "running", "fresh": True}
            self.assertIs(check(), True)
            return self.snapshot(after, cancel, check)
        self.camera.snapshot.side_effect = snapshot
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        self.assertTrue(result["result"]["observation"]["ok"])
        self.assertEqual(sum(m == "rm_movej" for m, _ in self.commands), 1)

    def test_disabled_follow_up_returns_to_observation_without_camera(self):
        self.plugin.dispatch("config", {"observe_after_transfer": False})
        for transfer in (self.transfer, self.grab_by):
            with self.subTest(action=transfer.__name__):
                self.make_photo()
                self.commands.clear()
                result = transfer()
                self.assertEqual(result["state"], "completed", result)
                data = result["result"]
                self.assertTrue(data["transfer_completed"])
                self.assertFalse(data["observe_after_transfer"])
                self.assertEqual(data["observation"], {"skipped": True, "reason": "disabled"})
                self.assertTrue(result["observation_required"])
                self.assertIsNone(self.plugin._observation)
                self.assertEqual(data["final_pose"], self.photo_pose)
                self.assertTrue(data["return_completed"])
                self.assertEqual(len(self.moves()), 6)
                self.assertEqual(sum(m == "rm_movej" for m, _ in self.commands), 1)
                self.assertTrue(data["release_completed"])
                self.assertFalse(data["holding_object_possible"])
                self.assertFalse(data["recovery_required"])
                self.assertEqual(self.transfer()["code"], "OBSERVATION_REQUIRED")
        self.camera.snapshot.assert_not_called()

    def test_return_does_not_depend_on_camera_inputs_in_either_mode(self):
        for capture in (False, True):
            with self.subTest(capture=capture):
                self.plugin.dispatch("config", {"observe_after_transfer": capture})
                self.make_photo()
                self.commands.clear()
                self.camera.info.return_value = {"state": "error", "fresh": False, "error": "camera unavailable"}
                result = self.transfer()
                self.assertEqual(result["state"], "error" if capture else "completed", result)
                self.assertTrue(result["result"]["transfer_completed"])
                self.assertTrue(result["result"]["return_completed"])
                self.assertEqual(sum(m == "rm_movej" for m, _ in self.commands), 1)
                self.assertTrue(result["observation_required"])

    def test_disabled_capture_return_failure_and_cancel_are_not_success(self):
        self.plugin.dispatch("config", {"observe_after_transfer": False})
        for cancel in (False, True):
            with self.subTest(cancel=cancel):
                self.make_photo()
                self.commands.clear()
                def fail(method, args):
                    if method == "rm_movej":
                        if cancel:
                            self.plugin.dispatch("cancel", {})
                        else:
                            raise RuntimeError("return failed")
                self.after_command = fail
                result = self.grab_by()
                self.assertEqual(result["state"], "cancelled" if cancel else "error", result)
                self.assertTrue(result["result"]["transfer_completed"])
                self.assertFalse(result["result"]["return_completed"])
                self.assertEqual(result["result"]["stage"], "returning")
                self.assertNotIn("final_pose", result["result"])
                self.camera.snapshot.assert_not_called()

    def test_minor_disturbance_while_holding_recovers_and_finishes_placing(self):
        for disturbed_stage in ("pick_close", "pick_lift", "move_to_place", "place_open"):
            with self.subTest(stage=disturbed_stage):
                self.make_photo()
                self.commands.clear()
                disturbed, restored = [], []
                def disturb(method, args):
                    if self.plugin._active["stage"] == disturbed_stage and not disturbed:
                        disturbed.append((self.pose[:], self.now + .4, len(self.commands)))
                        self.pose[2 if disturbed_stage in ("pick_lift", "move_to_place") else 0] += .0015
                def recover():
                    if disturbed and not restored:
                        pose, deadline, count = disturbed[0]
                        self.assertEqual(len(self.commands), count)  # No command before recovery.
                        if self.now >= deadline:
                            self.pose = pose[:]
                            restored.append(True)
                self.after_command, self.on_wait = disturb, recover
                result = self.transfer()
                self.assertEqual(result["state"], "completed", result)
                self.assertEqual(restored, [True])
                self.assertEqual(len(self.moves()), 6)
                self.assertEqual([a[0][0] for m, a in self.commands if m == "rm_set_hand_follow_pos"],
                                 [1000, 0, 1000, 0])
                self.assertFalse(any(m == "rm_set_arm_slow_stop" for m, _ in self.commands))
                self.assertTrue(result["result"]["release_completed"])
                self.assertFalse(result["result"]["holding_object_possible"])
                self.assertGreater(result["result"]["feedback_recoveries"], 0)

    def test_transient_inconsistent_sdk_samples_after_grasp_do_not_repeat_commands(self):
        remaining = [0]
        def disturb(method, args):
            if method == "rm_set_hand_follow_pos" and args[0][0] == 0 and len(self.moves()) == 2:
                remaining[0] = 2
        def trajectory(method):
            planned = self.joints[:]
            if remaining[0]:
                planned[0] += .3
                remaining[0] -= 1
            return {"trajectory_type": 0, "data": planned}
        self.after_command, self.client.call_dict = disturb, trajectory
        result = self.grab_by()
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(len(self.moves()), 6)
        self.assertEqual([a[0][0] for m, a in self.commands if m == "rm_set_hand_follow_pos"], [1000, 0, 1000, 0])
        self.assertGreater(result["result"]["feedback_recoveries"], 0)
        self.assertTrue(result["result"]["release_completed"])

    def test_single_slow_arm_or_gripper_read_while_holding_recovers(self):
        original = self.call
        for slow_method in ("rm_get_joint_degree", "rm_get_rm_plus_state_info"):
            with self.subTest(method=slow_method):
                self.make_photo()
                self.commands.clear()
                delayed = []
                def call(method, *args):
                    result = original(method, *args)
                    if method == slow_method and len(self.moves()) == 2 and self.grip_position == 300 and not delayed:
                        delayed.append(True)
                        self.now += 1.1
                    return result
                self.client.call = call
                result = self.transfer()
                self.assertEqual(result["state"], "completed", result)
                self.assertEqual(delayed, [True])
                self.assertEqual(len(self.moves()), 6)
                self.assertTrue(result["result"]["release_completed"])
                self.assertGreater(result["result"]["feedback_recoveries"], 0)

    def test_persistent_disturbance_after_grasp_reports_holding_and_stops(self):
        def disturb(method, args):
            if method == "rm_set_hand_follow_pos" and args[0][0] == 0:
                self.pose[0] += .0015
        self.after_command = disturb
        result = self.transfer()
        self.assertEqual(result["state"], "error", result)
        data = result["result"]
        self.assertIn("Feedback did not recover", data["message"])
        self.assertTrue(data["holding_object_possible"])
        self.assertTrue(data["recovery_required"])
        self.assertFalse(data["release_completed"])
        self.assertFalse(data["transfer_completed"])
        self.assertEqual(data["stage"], "pick_close")
        self.assertEqual(data["last_completed_stage"], "pick_descend")
        self.assertEqual(len(self.moves()), 2)
        self.assertEqual([a[0][0] for m, a in self.commands if m == "rm_set_hand_follow_pos"], [1000, 0])
        self.camera.snapshot.assert_not_called()

    def test_cancel_during_feedback_recovery_never_releases_or_continues(self):
        disturbed = []
        def disturb(method, args):
            if method == "rm_set_hand_follow_pos" and args[0][0] == 0:
                self.pose[0] += .0015
                disturbed.append(True)
        def cancel():
            if disturbed:
                disturbed.clear()
                self.plugin.dispatch("cancel", {})
        self.after_command, self.on_wait = disturb, cancel
        result = self.transfer()
        self.assertEqual(result["state"], "cancelled", result)
        self.assertTrue(result["result"]["holding_object_possible"])
        self.assertTrue(result["result"]["recovery_required"])
        self.assertEqual(len(self.moves()), 2)
        self.assertEqual(self.commands[-1][0], "rm_set_arm_slow_stop")

    def test_real_gripper_fault_in_slow_feedback_is_not_retried(self):
        original, faults = self.call, []
        def call(method, *args):
            result = original(method, *args)
            if method == "rm_get_rm_plus_state_info" and self.grip_position == 300:
                faults.append(True)
                result["dof_err"] = [1]
                self.now += 1.1
            return result
        self.client.call = call
        result = self.transfer()
        self.assertEqual(result["state"], "error", result)
        self.assertEqual(faults, [True])
        self.assertTrue(result["result"]["holding_object_possible"])
        self.assertEqual(len(self.moves()), 2)

    def test_failure_after_release_does_not_claim_object_is_still_held(self):
        def fail(method, args):
            if method == "rm_movel" and len(self.moves()) == 6:
                raise RuntimeError("SDK lift failed")
        self.after_command = fail
        result = self.transfer()
        self.assertEqual(result["state"], "error", result)
        self.assertEqual(result["result"]["stage"], "place_lift")
        self.assertTrue(result["result"]["release_completed"])
        self.assertFalse(result["result"]["holding_object_possible"])
        self.assertFalse(result["result"]["transfer_completed"])
        self.camera.snapshot.assert_not_called()

    def test_transfer_has_time_for_all_steps_after_slow_initial_move(self):
        def delay(method, args):
            if method == "rm_movel" and len(self.moves()) == 1:
                self.now += 40
        self.after_command = delay
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        self.assertGreater(self.now - 1000, 45)
        self.assertTrue(result["result"]["release_completed"])


if __name__ == "__main__":
    unittest.main()
