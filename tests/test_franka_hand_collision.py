"""Physical finger geometry must not inherit empirical grasp offsets."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'franka/fr3')]
from franka.fr3.pick_place.hand_collision import frame, vertices, check_table
from franka.fr3.pick_place.planning import rigid, rotation

class HandCollisionTests(unittest.TestCase):
    def test_sdk_frame_and_compensation_are_independent(self):
        measured=rigid(rotation((0,0,1),-math.pi/4),[0,0,.1034])
        profile={'tool_frame':{'F_T_EE':measured.tolist()},'depth_compensation_mm':25}
        actual=frame(profile,measured.flatten(order='F').tolist())
        np.testing.assert_allclose(actual[:3,3],[0,0,-.1034],atol=1e-9)
        profile['depth_compensation_mm']=0
        np.testing.assert_allclose(actual,frame(profile,measured.flatten(order='F').tolist()))
        changed=measured.copy();changed[2,3]+=.02
        with self.assertRaisesRegex(ValueError,'tool frame changed'):
            frame(profile,changed.flatten(order='F').tolist())
        with self.assertRaisesRegex(ValueError,'Measured SDK'):
            frame(profile)

    def test_finger_touches_table_while_housing_is_clear(self):
        points=vertices(frame({}))
        ee=rigid(rotation((1,0,0),math.pi),[0,0,.0095])
        # Housing ends 37.44 mm behind EE; the fingertips extend 9.5 mm past it.
        self.assertGreater(ee[2,3]+.03744,.04)
        with self.assertRaisesRegex(ValueError,'Hand/finger table clearance'):
            check_table(ee,points,np.zeros(3),np.array([0,0,1]),'descent')
        ee[2,3]=.02
        self.assertGreater(check_table(ee,points,np.zeros(3),np.array([0,0,1]),'lift'),.005)

    def test_both_open_fingers_are_included(self):
        points=vertices(frame({}))
        self.assertGreater(points[:,1].max(),.06)
        self.assertLess(points[:,1].min(),-.06)
        self.assertAlmostEqual(points[:,2].max(),.0095,places=6)

    def test_fixed_bench_plane_catches_bad_depth_plane(self):
        from unittest.mock import Mock
        from franka.fr3.pick_place.hand_collision import check_path_table
        model=Mock()
        model.pose.return_value=np.eye(4)
        model.table_clearance.return_value=.1
        points=np.array([[0.,0.,.02]])
        profile={'table_plane':{'point_base_mm':[0,0,30], 'normal_base':[0,0,1]}}
        with self.assertRaisesRegex(ValueError,'Hand/finger table clearance'):
            check_path_table(model,np.zeros(7),points,np.zeros(3),np.array([0,0,1]),'return',profile)

    def test_arm_table_check_is_independent_of_finger_clearance(self):
        from unittest.mock import Mock
        from franka.fr3.pick_place.hand_collision import check_path_table
        model=Mock();model.pose.return_value=np.eye(4);model.table_clearance.return_value=-.001
        with self.assertRaisesRegex(ValueError,'Arm/table clearance'):
            check_path_table(model,np.zeros(7),np.array([[0.,0.,.1]]),
                             np.zeros(3),np.array([0,0,1]),'return',{})

    def test_recorded_left_return_lifts_before_shoulder_turn(self):
        import json
        from franka.fr3.pick_place.hand_collision import observation_path
        from franka.fr3.pick_place.kinematics import Kinematics
        f=json.loads((ROOT/'tests/fixtures/franka/left_return.json').read_text())
        profile={'tool_frame':{'F_T_EE':np.array(f['F_T_EE']).reshape(4,4,order='F').tolist()},
                 'table_plane':f['table_plane']}
        path=observation_path(f,f['target'],profile)
        self.assertEqual(path.shape,(21,7))
        np.testing.assert_allclose(path[0],f['q'])
        np.testing.assert_allclose(path[-1],np.radians(f['target']))
        model=Kinematics(f['q'],np.array(f['O_T_EE']).reshape(4,4,order='F'))
        normal=np.array(f['table_plane']['normal_base'])
        lift=model.pose(path[10])[:3,3]-model.pose(path[0])[:3,3]
        self.assertGreater(lift@normal,.099)
        self.assertLess(np.linalg.norm(lift-normal*(lift@normal)),.001)
