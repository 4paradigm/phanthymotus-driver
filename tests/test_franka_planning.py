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


def reference_segment_distance(a, b, c, d):
    u, v = b-a, d-c
    matrix = np.column_stack((u, -v))
    s, t = np.linalg.lstsq(matrix, c-a, rcond=None)[0]
    distances = []
    if 0 <= s <= 1 and 0 <= t <= 1:
        distances.append(np.linalg.norm(a-c+s*u-t*v))
    for p, start, direction in ((a,c,v), (b,c,v), (c,a,u), (d,a,u)):
        f = np.clip((p-start)@direction/max(direction@direction, 1e-15), 0, 1)
        distances.append(np.linalg.norm(p-start-f*direction))
    return min(distances)


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

    def test_batched_link_clearance_matches_scalar_at_sampled_joint_poses(self):
        from franka.fr3.pick_place.kinematics import Kinematics, _FIRST, _SECOND, _RADII

        q = np.radians(self.observation['metadata']['joint_degree'])
        model = Kinematics(q, np.eye(4))
        for offset in np.linspace(-.1, .1, 11):
            sample = q + offset
            measured = model.clearance(sample)
            a, b = model._capsule_starts, model._capsule_ends
            reference = min(reference_segment_distance(a[i], b[i], a[j], b[j]) - _RADII[i] - _RADII[j]
                            for i, j in zip(_FIRST, _SECOND))
            self.assertAlmostEqual(measured, reference, places=12)
