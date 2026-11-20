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
