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


    def test_trim_descent_keeps_prefix_and_stops_short(self):
        from types import SimpleNamespace
        from franka.fr3.pick_place.planning import rigid
        from franka.fr3.pick_place.remote_execution import RemoteExecution

        executor = SimpleNamespace(check=lambda: None,
                                   model=SimpleNamespace(pose=lambda q: rigid(np.eye(3), [0, 0, q[0]])))
        points = [dict(time_ns=t, positions=[z] + [0.] * 6, velocities=[0.] * 7,
                       accelerations=[0.] * 7) for t, z in [(0, 0.), (10**9, .01), (2*10**9, .02)]]
        plan = dict(joint_names=[f'fr3_joint{i}' for i in range(1, 8)], points=points)
        trimmed = RemoteExecution.trim_descent(executor, plan, rigid(np.eye(3), [0, 0, .019]))
        self.assertEqual(trimmed['points'][:-1], points[:-1])
        self.assertAlmostEqual(trimmed['points'][-1]['positions'][0], .019)
        self.assertEqual(trimmed['points'][-1]['velocities'], [0.] * 7)
        self.assertEqual(plan['points'][-1]['positions'][0], .02)
