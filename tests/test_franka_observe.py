"""Observation contracts, isolation and terminal feedback without robot motion."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'franka/fr3'))
from franka.fr3.pick_place import PickPlacePlugin, joint_target, pose_vector
from franka.fr3.pick_place.inputs import resolve_topics, detection, ObservationInputs
from franka.fr3.pick_place.alignment import decode_depth
import numpy as np
import zlib

ANGLES = '-36.09,53.79,-17.09,-137.78,-89.19,244.19,33.34'


class Inputs:
    topics = ['/test/rgb', '/test/depth', '/test/rgb/objects']
    def select_serial(self, serial): self.serial = serial
    def prepare(self): pass
    def binding(self): return {'serial_number':'test'}
    def info(self): return {'state':'running', 'fresh':True}
    def snapshot(self, after, cancel, check):
        check()
        return dict(jpeg=b'jpeg', depth_zlib=b'depth', captured_at=after+.7,
                    width=1280,height=720, objects=[dict(name='orange',position=[.1,.2],confidence=.8)],
                    objects_timestamp=after+.8,input_identity={},synchronization={},calibration={})


class Session:
    def __init__(self): self.moves=[]
    def snapshot(self):
        return dict(connected=True,data_age_ms=10,robot_mode=1,
                    q=[math.radians(x) for x in joint_target(ANGLES)],dq=[0]*7,
                    O_T_EE=np.eye(4).flatten(order='F').tolist())
    def move(self,*args,**kwargs): self.moves.append(args);raise AssertionError('Unexpected motion')


class Manager:
    def __init__(self): self.session=Session();self.released=False
    def acquire(self,*args):return self.session
    def release(self,*args):self.released=True


class ObserveTests(unittest.TestCase):
    def plugin(self):
        self.manager=Manager();self.completions=[]
        return PickPlacePlugin({'motion_arm_ips':{'right':'172.16.0.3'},
            'vision_pick_and_drop':{'camera_serial':'test','observation_joints_deg':ANGLES}},
            self.manager,None,None,inputs=Inputs(),callback=lambda *a:self.completions.append(a))

    def dual_plugin(self):
        return PickPlacePlugin({'motion_arm_ips': {'left':'172.16.0.2','right':'172.16.0.3'},
            'vision_pick_and_drop': {'camera_serial':'right-camera','observation_joints_deg':ANGLES,
                'planning': {'handeye': {'robot_ip':'172.16.0.3','camera_serial':'right-camera'}},
                'arm_profiles': {'left': {'camera_serial':'left-camera',
                    'observation_joints_deg':'-129.3,-47.8,134.6,-131.4,-122.5,144.0,52.1',
                    'planning': {'grasp_depth_offset_mm':25, 'handeye': {'robot_ip':'172.16.0.2','camera_serial':'left-camera'}}}}}},
            Manager(),None,None,inputs=Inputs(),callback=lambda *a:None)

    def test_arm_switch_selects_profile_and_invalidates_the_previous_photograph(self):
        p=self.dual_plugin();p.observation={'old':'right'}
        args={**p.config,'arm':'172.16.0.2（左手）','force_n':30}
        result=p.dispatch('config',args)
        self.assertTrue(result['ok'])
        self.assertEqual((p.address,p.serial,p.inputs.serial),('172.16.0.2','left-camera','left-camera'))
        self.assertEqual(p.planning_profile['handeye']['robot_ip'],'172.16.0.2')
        self.assertTrue(p.config['observation_joints_deg'].startswith('-129.3'))
        self.assertEqual(p.config['z_compensation_mm'],0)
        self.assertIsNone(p.observation)
        # Replayed forms must not copy the foreign observation pose back into this arm.
        p.dispatch('config',args)
        self.assertTrue(p.config['observation_joints_deg'].startswith('-129.3'))
        p.dispatch('config',dict(p.config,arm='172.16.0.3（右手）'))
        self.assertEqual(p.config['observation_joints_deg'],ANGLES)
        self.assertEqual((p.address,p.serial),('172.16.0.3','right-camera'))
        self.assertEqual(p.config['z_compensation_mm'],0)
        for value in (float('nan'),51,True):
            self.assertEqual(p.dispatch('config',{'z_compensation_mm':value})['code'],'INVALID_CONFIG')

    def test_arm_specific_observation_fields_and_legacy_offset(self):
        p = self.dual_plugin()
        schema = p.get_tools()[0]['configSchema']['properties']
        self.assertEqual(schema['left_observation_joints_deg']['x-show-when'],
                         {'arm': '172.16.0.2（左手）'})
        result = p.dispatch('config', {
            'arm': '172.16.0.2（左手）', 'left_observation_joints_deg': ANGLES,
            'depth_compensation_mm': 25, 'x_compensation_mm': 3})
        self.assertTrue(result['ok'])
        self.assertEqual(result['observation_joints_deg'], ANGLES)
        self.assertEqual(result['z_compensation_mm'], 0)
        self.assertEqual(result['x_compensation_mm'], 3)

    def test_arm_switch_rejects_active_motion_and_mismatched_calibration(self):
        p=self.dual_plugin();p.active={'action_id':'moving'}
        self.assertEqual(p.dispatch('config',{'arm':'172.16.0.2（左手）'})['code'],'ACTION_IN_PROGRESS')
        self.assertEqual(p.address,'172.16.0.3')
        p.active=None
        p.profiles['left']['planning']['handeye']['robot_ip']='172.16.0.3'
        self.assertEqual(p.dispatch('config',{'arm':'172.16.0.2（左手）'})['code'],'INVALID_CONFIG')
        self.assertEqual(p.address,'172.16.0.3')


if __name__ == '__main__': unittest.main()
