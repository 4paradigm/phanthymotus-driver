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
        q0,overhead,lift=f['joints']
        self.pick['joint_path_rad']=np.vstack((np.linspace(q0,overhead,101),np.linspace(overhead,lift,200))).tolist()
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

    def test_optional_rotation_preserves_vertical_placement(self):
        options=transfer.parameters('grab_by',{**self.args,'rotation_deg':45})
        result=transfer.plan_transfer(self.observation,self.pick,self.handeye,options)
        qs=np.asarray(self.pick['joint_path_rad'])
        model=transfer.Kinematics(qs[0],np.asarray(self.observation['metadata']['O_T_EE']).reshape(4,4,order='F'))
        start=model.pose(qs[-1]);finish=model.pose(result['paths'][0][-1])
        expected=transfer.rotation(-np.asarray(self.pick['scene']['table_normal_base']),np.pi/4)@start[:3,:3]
        np.testing.assert_allclose(finish[:3,:3],expected,atol=.0002)
        self.assertEqual(result['result']['rotation_deg'],45)

    def test_complete_transfer_release_retreat_and_return_paths(self):
        result=transfer.plan_transfer(self.observation,self.pick,self.handeye,transfer.parameters('grab_by',self.args))
        self.assertEqual(result['stages'],['move_to_place','place_lift','return_to_observation'])
        self.assertEqual(result['result']['delta_x'],30)
        self.assertGreater(result['arm_capsule_clearance_mm'],20)
        np.testing.assert_allclose(result['paths'][-1][-1],self.pick['joint_path_rad'][0])
        for a,b in zip(result['paths'],result['paths'][1:]):np.testing.assert_allclose(a[-1],b[0])
        self.assertTrue(all(len(p)<=21 for p in result['paths']))
        with mock.patch.object(transfer.Kinematics,'clearance',return_value=0):
            with self.assertRaisesRegex(ValueError,'self-collision'):
                transfer.plan_transfer(self.observation,self.pick,self.handeye,transfer.parameters('grab_by',self.args))

if __name__=='__main__':unittest.main()
