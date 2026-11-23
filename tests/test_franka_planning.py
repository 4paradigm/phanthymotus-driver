"""Validate the grasp targets passed to MoveIt from a saved observation."""
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'franka/fr3')]
from franka.fr3.pick_place.geometry import locate
from franka.fr3.pick_place.planning import grasp_targets


class GraspTargetTests(unittest.TestCase):
    def setUp(self):
        fixture = json.loads((ROOT / 'tests/fixtures/franka/spherical_grasp_preview.json').read_text())
        self.observation = fixture['observation']
        self.observation['depth_zlib'] = bytes.fromhex(self.observation.pop('depth_zlib_hex'))
        self.profile = fixture['profile']
        surface = locate(self.observation, [0, 0], self.profile['handeye'])
        self.assertEqual(surface['depth_mm'], 536.)
        self.scene = dict(diameter_mm=65, centre_base_mm=surface['base_xyz_mm'],
                          table_normal_base=self.profile['table_normal_base'])

    def test_approach_and_grasp_keep_orientation_and_nominal_distance(self):
        grasp = grasp_targets(self.observation, self.scene, self.profile)[0]
        approach, target, lift = np.asarray(grasp['poses'])
        np.testing.assert_allclose(approach[:3, :3], target[:3, :3])
        np.testing.assert_allclose(target[:3, 3] - approach[:3, 3], .15 * target[:3, 2])
        np.testing.assert_allclose(lift[:3, 3] - target[:3, 3],
                                   .01 * np.asarray(self.scene['table_normal_base']), atol=1e-5)

    def test_invalid_width_and_observation_joint_margin_rejected(self):
        with self.assertRaisesRegex(ValueError, 'diameter'):
            grasp_targets(self.observation, {**self.scene, 'diameter_mm': 90}, self.profile)
        self.observation['metadata']['joint_degree'][0] = 360
        with self.assertRaisesRegex(ValueError, 'joint'):
            grasp_targets(self.observation, self.scene, self.profile)


    def test_shallower_target_preserves_approach_and_rechecks_section(self):
        initial = grasp_targets(self.observation, self.scene, self.profile)[0]
        centre = np.asarray(self.scene['centre_base_mm'])
        grasp = np.asarray(initial['poses'][1])
        self.scene['visible_surface_base_mm'] = [
            (centre + y * grasp[:3, 1]).tolist() for y in (-10, 10)
        ]
        pick = grasp_targets(self.observation, self.scene, self.profile,
                             extra_retreat_mm=1, only_yaw=initial['yaw_deg'])[0]
        np.testing.assert_allclose(pick['poses'][0], initial['poses'][0])
        np.testing.assert_allclose(np.asarray(pick['poses'][1])[:3, 3] - grasp[:3, 3],
                                   -.001 * grasp[:3, 2])
        self.assertAlmostEqual(pick['grasp_width_mm'], 20)
        with self.assertRaisesRegex(ValueError, 'No usable'):
            grasp_targets(self.observation, self.scene, self.profile,
                          extra_retreat_mm=12, only_yaw=initial['yaw_deg'])


    def test_shorten_descent_keeps_timed_prefix_and_stops_with_clearance(self):
        import time
        from types import SimpleNamespace
        from unittest.mock import Mock

        from scipy.interpolate import BPoly
        from franka.fr3.pick_place.hand_collision import TableClearanceError
        from franka.fr3.pick_place.remote_execution import RemoteExecution

        executor = RemoteExecution.__new__(RemoteExecution)
        executor.check = Mock()
        executor.active = {}
        executor.options = dict(speed_deg_s=5)
        executor.profile = dict(descent_min_clearance_mm=1, descent_clearance_allowance_mm=5)
        executor.scene = dict(table_point_base_mm=[0, 0, 0], table_normal_base=[0, 0, 1])
        executor.visible_surface = None
        executor.hand_points = np.zeros((1, 3))
        orientation = np.diag([1., -1., -1.])

        def pose(q):
            result = np.eye(4)
            result[:3, :3] = orientation
            result[2, 3] = q[0]
            return result

        executor.model = SimpleNamespace(pose=pose, low=np.full(7, -1.), high=np.full(7, 1.),
                                         clearance=lambda q: 1., table_clearance=lambda *args: 1.)
        executor.q = [.023] + [0.] * 6
        plan = dict(code=1, fraction=1., joint_names=[f'fr3_joint{i}' for i in range(1, 8)],
                    points=[dict(time_ns=t*10**9, positions=[z] + [0.] * 6,
                                 velocities=[v] + [0.] * 6, accelerations=[0.] * 7)
                            for t, z, v in [(0, .023, 0.), (1, .015, -.01), (2, .0055, 0.)]])
        original_target = pose(plan['points'][-1]['positions'])
        with self.assertRaises(TableClearanceError):
            executor.prepare_trajectory('descend', [original_target], plan, True, False, time.monotonic())
        self.assertLess(executor.descent['segments'][-1][0], 2.)
        segment = executor.descent['segments'][0]
        target = original_target.copy()
        target[2, 3] += .002
        _, shorter = executor.shorten_descent(target)
        self.assertEqual(shorter['segments'][0], segment)
        self.assertAlmostEqual(shorter['points'][-1]['positions'][0], .0075)
        self.assertGreaterEqual(shorter['descent_gap_bound_mm'], 1.)
        first = BPoly(np.asarray(segment[1])[:, None, :], [0., segment[0]])
        end, controls = shorter['segments'][-1]
        tail = BPoly(np.asarray(controls)[:, None, :], [segment[0], end])
        for degree in (0, 1, 2):
            np.testing.assert_allclose(first.derivative(degree)(segment[0]),
                                       tail.derivative(degree)(segment[0]), atol=1e-9)
        for degree in (1, 2):
            np.testing.assert_allclose(tail.derivative(degree)(end), 0., atol=1e-9)


    def test_vertical_offset_raises_all_grasp_and_placement_targets(self):
        from franka.fr3.pick_place.geometry import transform
        from franka.fr3.pick_place.transfer import image_table_axes

        original = grasp_targets(self.observation, self.scene, self.profile)[0]
        corrected = grasp_targets(self.observation, self.scene, {**self.profile, 'grasp_offset_mm': [0, -2, 3]})[0]
        ee = np.asarray(self.observation['metadata']['O_T_EE']).reshape(4, 4, order='F')
        camera = ee @ transform(self.profile['handeye']['EE_T_camera'], 'EE_T_camera')
        normal = np.asarray(self.scene['table_normal_base'])
        horizontal = image_table_axes(camera, normal)[:, 1] * -.002
        delta = np.asarray(corrected['poses'])[:, :3, 3] - np.asarray(original['poses'])[:, :3, 3]
        np.testing.assert_allclose(delta, np.tile(horizontal + normal * .003, (3, 1)))
        np.testing.assert_allclose(corrected['placement_reference'],
                                   np.asarray(original['poses'][1])[:3, 3] + normal * .003)

        from franka.fr3.pick_place.transfer import moveit_targets
        scene = dict(self.scene, table_point_base_mm=self.scene['centre_base_mm'])
        options = dict(displacement_mm=[20, 0], rotation_deg=0)
        old_targets, new_targets = (
            moveit_targets(self.observation, pick, scene, self.profile['handeye'], options)
            for pick in (original, corrected)
        )
        for name in ('approach', 'descent', 'lift', 'carry', 'place_lift'):
            before = np.asarray(old_targets[name]).reshape(-1, 4, 4)
            after = np.asarray(new_targets[name]).reshape(-1, 4, 4)
            np.testing.assert_allclose((after[:, :3, 3] - before[:, :3, 3]) @ normal, .003)
