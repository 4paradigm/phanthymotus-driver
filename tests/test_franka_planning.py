"""Offline card-entry regression using recorded frames and synthetic sphere depth."""
import json
from pathlib import Path
import sys
import unittest
from unittest import mock
import zlib

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'franka/fr3')]
from franka.fr3.pick_place import PickPlacePlugin
from franka.fr3.pick_place import planning


class PlanningTests(unittest.TestCase):
    def setUp(self):
        fixture = json.loads((ROOT/'tests/fixtures/franka/spherical_grasp_preview.json').read_text())
        self.plugin = PickPlacePlugin(
            {'vision_pick_and_drop': {'planning': fixture['profile']}},
            None, None, None, callback=lambda *args: None)
        observation = fixture['observation']
        observation['depth_zlib'] = bytes.fromhex(observation.pop('depth_zlib_hex'))
        self.observation = observation
        self.args = dict(start_point_x=0, start_point_y=0, diameter_mm=65)

    def preview(self, args):
        try:
            return planning.preview(self.plugin.observation,[args['start_point_x'],args['start_point_y']],
                                    args['diameter_mm'],self.plugin.planning_profile)
        except (ValueError,KeyError,TypeError):
            return dict(ok=False,motion_sent=False)

    def test_planner_generates_complete_serializable_path_without_hardware(self):
        self.plugin.observation = self.observation
        self.plugin.planning_profile['shallow_retreat_mm'] = 20
        result = self.preview(self.args)
        self.assertTrue(result['ok'], result)
        self.assertFalse(result['motion_sent'])
        axis = -np.asarray(self.plugin.planning_profile['table_normal_base'])
        self.assertLessEqual(result['max_descent_tilt_deg'], 3)
        np.testing.assert_allclose(
            np.asarray(result['object_centre_base_mm']) - result['grasp_centre_base_mm'],
            20 * axis, atol=1e-8)
        self.assertEqual(result['phase_end_indices'], [100, 200, 300])
        path = np.array(result['joint_path_rad'])
        self.assertEqual(path.shape, (301, 7))
        self.assertGreater(result['arm_capsule_clearance_mm'], 20)
        self.assertGreaterEqual(result['checked_samples'], 301)
        json.dumps(result, allow_nan=False)

    def test_missing_observation_bad_depth_and_invalid_diameter_are_rejected(self):
        self.assertFalse(self.preview(self.args)['ok'])
        self.plugin.observation = self.observation
        for diameter in (0, 90, float('nan')):
            self.assertFalse(self.preview({**self.args, 'diameter_mm': diameter})['ok'])
        self.observation['depth_zlib'] = zlib.compress(bytes(128))
        self.assertFalse(self.preview(self.args)['ok'])

    def test_left_depth_correction_is_part_of_the_solved_grasp_pose(self):
        from franka.fr3.pick_place.kinematics import Kinematics
        self.plugin.observation=self.observation
        profile=self.plugin.planning_profile
        profile.update(grasp_depth_offset_mm=25,z_compensation_mm=0,shallow_retreat_mm=20)
        result=self.preview(self.args)
        self.assertTrue(result['ok'],result)
        self.assertAlmostEqual(result['sdk_to_pad_offset_mm'],-5)
        down=-np.asarray(profile['table_normal_base'])
        np.testing.assert_allclose(np.asarray(result['sdk_grasp_base_mm'])-
                                   result['grasp_centre_base_mm'],5*down,atol=1e-8)
        path=np.asarray(result['joint_path_rad'])
        actual=np.asarray(self.observation['metadata']['O_T_EE']).reshape(4,4,order='F')
        model=Kinematics(path[0],actual)
        np.testing.assert_allclose(model.pose(path[200])[:3,3]*1000,
                                   result['sdk_grasp_base_mm'],atol=.1)
    def test_bad_table_or_camera_identity_is_rejected(self):
        self.plugin.observation = self.observation
        self.plugin.planning_profile['table_normal_base'] = [0, 0, 0]
        self.assertFalse(self.preview(self.args)['ok'])
        self.plugin.planning_profile['handeye']['camera_serial'] = 'other'
        self.assertFalse(self.preview(self.args)['ok'])

    def test_colliding_paths_are_rejected_before_motion(self):
        self.plugin.observation = self.observation
        # Force every capsule pair to overlap, including otherwise solvable poses.
        with mock.patch.object(planning, 'segment_distance', return_value=0):
            result = self.preview(self.args)
        self.assertFalse(result['ok'])
        self.assertFalse(result['motion_sent'])

    def test_parallel_and_degenerate_capsules(self):
        self.assertAlmostEqual(planning.segment_distance(
            np.array([0.,0,0]), np.array([1.,0,0]),
            np.array([0.,1,0]), np.array([1.,1,0])), 1)
        self.assertAlmostEqual(planning.segment_distance(
            np.zeros(3), np.zeros(3), np.ones(3), np.ones(3)), np.sqrt(3))


if __name__ == '__main__':
    unittest.main()
