"""Normalized-position transfers against a simulated arm and gripper; no hardware access."""

import ast
import ctypes
import json
import math
from pathlib import Path
import unittest
from unittest import mock
import zlib

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

    def snapshot(self, *args):
        return fixtures.ObserveTests.snapshot(self, *args)

    def make_photo(self):
        photo = {"jpeg": b"photo", "depth_zlib": zlib.compress(self.depth.tobytes()),
                 "captured_at": 1234, "width": self.depth.shape[1], "height": self.depth.shape[0], "depth_scale_m": 0.001,
                 "intrinsics": {"fx": 10, "fy": 20, "ppx": 1, "ppy": 1,
                                "model": "distortion.none", "coeffs": [0]*5}}
        self.photo_data = photo
        self.photo = self.plugin._save_photo(photo, {
            "observation_id": "photo-" + str(len(list(Path(self.temp.name).iterdir()))),
            "config": dict(self.plugin._config)}, {"joints": self.joints, "pose": self.pose},
            {"work": self.copy(self.frame), "tool": self.copy(self.frame)})
        self.plugin._observation = dict(self.photo)

    def edit_metadata(self, edit):
        path = Path(self.photo["metadata_path"])
        data = json.loads(path.read_text())
        edit(data)
        path.write_text(json.dumps(data))

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
        elif method == "rm_set_rm_plus_reg":
            self.grip_force = args[2][0]
        elif method == "rm_set_hand_follow_pos":
            # Closing onto an object must still permit lifting.
            self.grip_position = args[0][0] or 300
        self.after_command(method, args)

    def transfer(self, **updates):
        return fixtures.wait_for_completion(self.plugin, self.plugin.dispatch("transfer_to", {
            "confirm_motion": True, "x1": -.5, "y1": -1/3, "x2": .5, "y2": 1/3, **updates}))

    def transfer_by(self, **updates):
        return fixtures.wait_for_completion(self.plugin, self.plugin.dispatch("transfer_by", {
            "confirm_motion": True, "x1": -.5, "y1": -1/3, "dx_mm": -30, "dy_mm": 0, **updates}))

    def moves(self):
        return [args[0] for method, args in self.commands if method == "rm_movel"]

    def test_complete_transfer_uses_configured_absolute_targets(self):
        self.assertTrue(self.plugin.dispatch("config", {"speed_percent": 37,
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
                self.assertEqual(args[1:], (37, 0, 0, 0))
        self.assertEqual([args for method, args in self.commands if method == "rm_set_rm_plus_reg"],
                         [(1220, 1, [100]), (1220, 1, [35]), (1220, 1, [100])])
        self.assertEqual(self.register_reads, [(1220, 2), (1220, 1)] * 9)
        self.assertEqual([args[0][0] for method, args in self.commands if method == "rm_set_hand_follow_pos"],
                         [1000, 0, 1000, 0])
        self.assertFalse(result["result"]["grasp_checked"])
        self.assertEqual(result["result"]["pick_pixel"], [1, 1])
        self.assertEqual(result["result"]["place_pixel"], [3, 2])
        self.assertIsNone(self.plugin._active)
        self.assertIsNone(self.plugin._observation)
        self.assertFalse(self.client.motion_lock.locked())
        self.pool.select.assert_not_called()
        self.plugin._publish_photo.assert_not_called()
        count = len(self.commands)
        self.assertIn("Run observe", self.transfer()["message"])
        self.assertEqual(len(self.commands), count)

    def test_default_descent_is_directly_91_and_60_mm(self):
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        np.testing.assert_allclose(np.array(self.moves())[:, 2], [.3, .209, .3, .3, .24, .3])
        self.assertTrue(all(args[1] == 50 for name, args in self.commands if name == "rm_movel"))
        self.assertIn(("rm_set_rm_plus_reg", (1220, 1, [15])), self.commands)

    def test_one_photo_cannot_be_reused_by_either_transfer_action(self):
        for first in (self.transfer, self.transfer_by):
            with self.subTest(action=first.__name__):
                self.make_photo()
                photo = self.copy(self.photo)
                result = first()
                self.assertEqual(result["state"], "completed", result)
                self.assertTrue(result["observation_required"])
                info = self.plugin.dispatch("info", {})
                self.assertTrue(info["observation_required"])
                self.assertIsNone(info["observation"])
                self.assertEqual(info["last_result"], result)
                # Saved images and returned observation metadata cannot restore validity.
                self.assertTrue(Path(photo["file_path"]).exists())
                count = len(self.commands)
                for repeated in (self.transfer, self.transfer_by):
                    rejected = repeated(**photo)
                    self.assertEqual(rejected["code"], "OBSERVATION_REQUIRED")
                    self.assertTrue(rejected["observation_required"])
                    self.assertEqual(len(self.commands), count)
                self.assertFalse(self.client.motion_lock.locked())

    def test_failed_first_command_consumes_photo_before_sdk_returns(self):
        for transfer in (self.transfer, self.transfer_by):
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
                count = len(self.commands)
                for repeated in (self.transfer, self.transfer_by):
                    self.assertEqual(repeated()["code"], "OBSERVATION_REQUIRED")
                self.assertEqual(len(self.commands), count)
                self.assertFalse(self.client.motion_lock.locked())

    def test_transfer_by_uses_pick_point_and_configured_millimetres(self):
        self.assertTrue(self.plugin.dispatch("config", {"speed_percent": 37,
            "x_compensation_mm": 40, "y_compensation_mm": -60,
            "pick_grip_force": 35, "pick_descent_mm": 80, "place_descent_mm": 70})["ok"])
        self.make_photo()
        result = self.transfer_by(x1=.5, y1=1/3, dx_mm=-30.5, dy_mm=20.25)
        self.assertEqual(result["state"], "completed", result)
        expected = [[.04, .165, .3], [.04, .165, .22], [.04, .165, .3],
                    [.0705, .18525, .3], [.0705, .18525, .23], [.0705, .18525, .3]]
        np.testing.assert_allclose(np.array(self.moves())[:, :3], expected)
        self.assertTrue(all(args[1] == 37 for name, args in self.commands if name == "rm_movel"))
        self.assertEqual([args for name, args in self.commands if name == "rm_set_rm_plus_reg"],
                         [(1220, 1, [100]), (1220, 1, [35]), (1220, 1, [100])])
        self.assertEqual([args[0][0] for name, args in self.commands if name == "rm_set_hand_follow_pos"],
                         [1000, 0, 1000, 0])
        self.assertEqual(result["result"]["dx_mm"], -30.5)
        self.assertEqual(result["result"]["dy_mm"], 20.25)
        self.assertEqual(result["result"]["direction_reference"], "observation_image")
        self.assertEqual(result["result"]["pick_pixel"], [3, 2])
        self.assertNotIn("place_pixel", result["result"])
        self.assertIsNone(self.plugin._observation)
        self.assertFalse(self.client.motion_lock.locked())
        self.pool.select.assert_not_called()
        self.plugin._publish_photo.assert_not_called()

    def test_transfer_by_signed_image_directions(self):
        for dx, dy, base_delta in ((30, 0, [-30, 0]), (-30, 0, [30, 0]),
                                   (0, 25, [0, 25]), (0, -25, [0, -25]),
                                   (30, -25, [-30, -25])):
            with self.subTest(dx=dx, dy=dy):
                self.make_photo()
                result = self.transfer_by(dx_mm=dx, dy_mm=dy)
                self.assertEqual(result["state"], "completed", result)
                data = result["result"]
                np.testing.assert_allclose(np.subtract(data["place_base_xy_mm"], data["pick_base_xy_mm"]), base_delta)

    def test_transfer_by_needs_only_pick_depth_not_destination_pixels(self):
        self.depth[:] = 0
        self.depth[1, 1] = 400
        self.make_photo()
        result = self.transfer_by()
        self.assertEqual(result["state"], "completed", result)
        np.testing.assert_allclose(result["result"]["place_base_xy_mm"], [160, 125])

    def test_transfer_by_rejects_invalid_displacement_and_pick_before_commands(self):
        for args in ({"dx_mm": None}, {"dy_mm": None}, {"dx_mm": True}, {"dy_mm": "30"},
                     {"dx_mm": float("nan")}, {"dy_mm": float("inf")},
                     {"dx_mm": 0, "dy_mm": 0}, {"x1": 1.0001}, {"y1": -1.0001},
                     {"x1": 691}, {"y1": "0"}, {"x1": True}, {"y1": float("nan")}):
            with self.subTest(args=args):
                self.assertEqual(self.transfer_by(**args)["state"], "error")
                self.assertEqual(self.commands, [])
                self.assertIsNotNone(self.plugin._observation)
        self.assertEqual(self.plugin.dispatch("transfer_by", {"confirm_motion": True, "x1": 0, "y1": 0, "dx_mm": 30})["state"], "error")
        self.depth[1, 1] = 0
        self.make_photo()
        self.assertIn("no valid depth", self.transfer_by()["result"]["message"])
        self.assertEqual(self.commands, [])

    def test_transfer_by_keeps_base_direction_with_rotated_work_frame(self):
        self.frame["pose"] = [.2, -.1, .05, .2, -.1, .3]
        work_rotation = rotation(self.frame["pose"])
        orientation = work_rotation.T @ rotation([0, 0, 0, math.pi, 0, 0])
        self.pose[3:] = [math.atan2(orientation[2, 1], orientation[2, 2]),
                         math.asin(-orientation[2, 0]), math.atan2(orientation[1, 0], orientation[0, 0])]
        self.make_photo()
        result = self.transfer_by(dx_mm=-30, dy_mm=20)
        self.assertEqual(result["state"], "completed", result)
        bases = np.array([work_rotation @ pose[:3] + self.frame["pose"][:3] for pose in self.moves()])
        np.testing.assert_allclose(bases[3] - bases[0], [.03, .02, 0], atol=1e-12)
        np.testing.assert_allclose(bases[:, 2] - bases[0, 2], [0, -.091, 0, 0, -.060, 0], atol=1e-12)

    def test_transfer_by_uses_shared_cancellation_and_consumes_photo(self):
        def cancel(method, args):
            if method == "rm_movel":
                self.assertEqual(self.plugin.dispatch("config", {"speed_percent": 1})["code"], "ACTION_IN_PROGRESS")
                self.assertEqual(self.transfer()["state"], "error")
                self.plugin.dispatch("cancel", {})
        self.after_command = cancel
        result = self.transfer_by()
        self.assertEqual(result["state"], "cancelled", result)
        self.assertEqual([name for name, _ in self.commands], ["rm_movel", "rm_set_arm_slow_stop"])
        self.assertFalse(self.client.motion_lock.locked())
        self.assertIsNone(self.plugin._observation)
        self.assertTrue(result["observation_required"])
        for transfer in (self.transfer, self.transfer_by):
            self.assertEqual(transfer()["code"], "OBSERVATION_REQUIRED")

    def test_observe_then_transfer_uses_one_photo_and_no_other_cards(self):
        def snapshot(after, cancel, check):
            check()
            return {**self.photo_data, "captured_at": after + .01}
        self.camera.snapshot.side_effect = snapshot
        observed = fixtures.ObserveTests.observe(self)
        self.assertEqual(observed["state"], "completed", observed)
        result = self.transfer()
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(result["result"]["observation_id"], observed["result"]["observation_id"])
        self.assertEqual(sum(name == "rm_movej" for name, _ in self.commands), 1)
        self.assertEqual(len(self.moves()), 6)
        self.camera.snapshot.assert_called_once()
        self.camera.stop.assert_called_once()
        self.plugin._publish_photo.assert_called_once()

    def test_new_observe_allows_next_transfer_with_new_photo_only(self):
        self.assertEqual(self.transfer()["state"], "completed")
        self.assertEqual(self.transfer_by()["code"], "OBSERVATION_REQUIRED")
        previous_id = self.photo["observation_id"]
        def snapshot(after, cancel, check):
            check()
            return {**self.photo_data, "captured_at": after + .01}
        self.camera.snapshot.side_effect = snapshot
        observed = fixtures.ObserveTests.observe(self)
        self.assertEqual(observed["state"], "completed", observed)
        self.assertFalse(observed["observation_required"])
        self.assertNotEqual(observed["result"]["observation_id"], previous_id)
        result = self.transfer_by()
        self.assertEqual(result["state"], "completed", result)
        self.assertEqual(result["result"]["observation_id"], observed["result"]["observation_id"])
        self.assertTrue(result["observation_required"])
        self.camera.snapshot.assert_called_once()
        self.plugin._publish_photo.assert_called_once()

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
        for args in ({"x1": None}, {"x2": None}, {"y1": -1.0001}, {"x1": True},
                     {"y2": 1.0001}, {"x1": float("inf")}, {"x2": float("nan")},
                     {"x2": 700}, {"y2": 350}, {"y1": "0"}):
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
                result = self.transfer_by(x1=.08, y1=-.079, dx_mm=30, dy_mm=0)
                self.assertEqual(result["state"], "completed", result)
                data = result["result"]
                self.assertEqual(data["pick_pixel"], pixel)
                np.testing.assert_allclose(np.subtract(data["place_base_xy_mm"], data["pick_base_xy_mm"]), [-30, 0])

    def test_normalized_center_and_edges_resolve_to_valid_pixels(self):
        for position, pixel in (((0, 0), [2, 2]), ((-1, -1), [0, 0]), ((1, 1), [3, 2]),
                                ((1, -1), [3, 0]), ((-1, 1), [0, 2]), ((.999, .999), [3, 2])):
            with self.subTest(position=position):
                self.make_photo()
                result = self.transfer(x1=position[0], y1=position[1], x2=position[0], y2=position[1])
                self.assertEqual(result["state"], "completed", result)
                self.assertEqual(result["result"]["pick_pixel"], pixel)
                self.assertEqual(result["result"]["place_pixel"], pixel)

    def test_normalized_xy_signs_preserve_installed_base_directions(self):
        for x2, y2, pixel, delta in ((.5, -1/3, [3, 1], [-80, 0]),
                                    (-1, -1/3, [0, 1], [40, 0]),
                                    (-.5, 1/3, [1, 2], [0, 20]),
                                    (-.5, -1, [1, 0], [0, -20])):
            with self.subTest(x2=x2, y2=y2):
                self.make_photo()
                result = self.transfer(x2=x2, y2=y2)
                self.assertEqual(result["state"], "completed", result)
                data = result["result"]
                self.assertEqual(data["pick_pixel"], [1, 1])
                self.assertEqual(data["place_pixel"], pixel)
                np.testing.assert_allclose(np.subtract(data["place_base_xy_mm"], data["pick_base_xy_mm"]), delta, atol=1e-10)

    def test_missing_arguments_and_missing_observation_do_not_move(self):
        self.assertEqual(self.plugin.dispatch("transfer_to", {"confirm_motion": True})["state"], "error")
        self.plugin._observation = None
        self.assertIn("Run observe", self.transfer()["message"])
        self.assertEqual(self.commands, [])

    def test_configuration_change_requires_new_observation(self):
        self.plugin.dispatch("config", {"speed_percent": 40})
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
                self.now += 46
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


if __name__ == "__main__":
    unittest.main()
