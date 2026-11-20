"""Transfer interface, calibrated directions and complete-path preflight."""
import json
from pathlib import Path
import sys
import unittest
from unittest import mock
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'franka/fr3')]
from franka.fr3.pick_place import transfer


class TransferTests(unittest.TestCase):
    def setUp(self):
        f=json.loads((ROOT/'tests/fixtures/franka/transfer.json').read_text())
        self.observation,self.handeye,self.pick=f['observation'],f['handeye'],f['pick']
        self.args=dict(start_point_x=0,start_point_y=0,delta_x=30,delta_y=0)

    def test_realman_fields_units_and_invalid_inputs(self):
        self.assertEqual(transfer.parameters('grab_by',self.args)['rotation_deg'],0)
        for extra in ({'delta_x':float('nan')},{'delta_y':None},{'rotation_deg':181},
                      {'start_point_x':True},{'delta_x':0}):
            with self.assertRaises(ValueError):transfer.parameters('grab_by',{**self.args,**extra})
        with self.assertRaises(ValueError):transfer.parameters('grab_to',self.args)

    def test_image_right_is_actual_30mm_on_table_not_base_x_or_pixels(self):
        options=transfer.parameters('grab_by',self.args)
        d=transfer.destination(self.observation,self.pick['scene'],self.handeye,options)
        self.assertAlmostEqual(np.linalg.norm(d)*1000,30)
        self.assertAlmostEqual(d@self.pick['scene']['table_normal_base'],0)
        self.assertLess(d[0],0);self.assertGreater(d[1],0)

    def test_grab_to_projects_selected_ray_to_same_table(self):
        n=np.array(self.pick['scene']['table_normal_base'])
        table=np.array(self.pick['scene']['table_point_base_mm'])/1000
        surface=table+np.array([.03,-.02,.01])
        options=transfer.parameters('grab_to',dict(start_point_x=0,start_point_y=0,target_point_x=.2,target_point_y=.1))
        with mock.patch.object(transfer,'locate',return_value=dict(base_xyz_mm=(surface*1000).tolist())):
            delta=transfer.destination(self.observation,self.pick['scene'],self.handeye,options)
        self.assertAlmostEqual(delta@n,0,places=7)

    def test_moveit_placement_keeps_vertical_axis_with_rotation(self):
        normal = np.asarray(self.pick['scene']['table_normal_base'])
        axis = np.cross([0., 0., 1.], normal)
        axis /= np.linalg.norm(axis)
        orientation = np.column_stack((axis, np.cross(-normal, axis), -normal))
        grasp = transfer.rigid(orientation, np.asarray(self.pick['sdk_grasp_base_mm']) / 1000)
        pick = dict(poses=[grasp, grasp, grasp], sdk_to_pad_offset_mm=self.pick['sdk_to_pad_offset_mm'])
        options = transfer.parameters('grab_by', {**self.args, 'rotation_deg': 45})
        targets = transfer.moveit_targets(self.observation, pick, self.pick['scene'], self.handeye, options)
        place = targets['carry'][-1]
        np.testing.assert_allclose(place[:3, :3], transfer.rotation(-normal, np.pi / 4) @ orientation)
        np.testing.assert_allclose(targets['place_lift'][:3, 3] - place[:3, 3], .15 * normal)
