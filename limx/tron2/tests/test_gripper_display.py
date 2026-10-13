"""Gripper outlines follow real wrist FK without changing motors or feedback."""
import hashlib
import json
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from skeleton import model, JOINT_NAMES

PREFIX='display_hand_gripper_'
PINNED_KINEMATICS='5aa3938eb8346d7da7b174def0e7f9ff4c631bdce526d3ff58a8b6176ecc8552'


def multiply(a,b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotate(r,p):
    return [sum(r[i][k]*p[k] for k in range(3)) for i in range(3)]


def rotation(axis,angle):
    x,y,z=axis;c=math.cos(angle);s=math.sin(angle);d=1-c
    return [[c+x*x*d,x*y*d-z*s,x*z*d+y*s],
            [y*x*d+z*s,c+y*y*d,y*z*d-x*s],
            [z*x*d-y*s,z*y*d+x*s,c+z*z*d]]


def positions(root,q):
    identity=[[1,0,0],[0,1,0],[0,0,1]]
    poses={'base_Link':(identity,[0,0,0])}; pending=list(root.findall('joint'))
    while pending:
        progress=False
        for joint in list(pending):
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent not in poses:continue
            r,p=poses[parent];origin=joint.find('origin')
            xyz=[float(v) for v in origin.get('xyz','0 0 0').split()]
            angles=[float(v) for v in origin.get('rpy','0 0 0').split()]
            offset=rotate(r,xyz);p2=[p[i]+offset[i] for i in range(3)]
            local=multiply(multiply(rotation([1,0,0],angles[0]),rotation([0,1,0],angles[1])),rotation([0,0,1],angles[2]))
            if joint.get('type')=='revolute':
                axis=[float(v) for v in joint.find('axis').get('xyz').split()]
                local=multiply(local,rotation(axis,q.get(joint.get('name'),0)))
            poses[child]=(multiply(r,local),p2);pending.remove(joint);progress=True
        if not progress:raise ValueError('Unconnected model graph')
    return {name:p for name,(r,p) in poses.items()}


class GripperDisplayTests(unittest.TestCase):
    def setUp(self):self.root=ET.fromstring(model()['urdf'])

    def test_original_pinned_kinematics_and_sixteen_motors_are_unchanged(self):
        original={'links':sorted(x.get('name') for x in self.root.findall('link') if not x.get('name').startswith(PREFIX)),
                  'joints':sorted([{'attrs':j.attrib,'children':[{c.tag:c.attrib} for c in j]}
                                   for j in self.root.findall('joint') if not j.get('name').startswith(PREFIX)],
                                   key=lambda j:j['attrs']['name'])}
        self.assertEqual(hashlib.sha256(json.dumps(original,sort_keys=True).encode()).hexdigest(),PINNED_KINEMATICS)
        moving={j.get('name') for j in self.root.findall('joint') if j.get('type')!='fixed'}
        self.assertEqual(moving,set(JOINT_NAMES));self.assertEqual(len(moving),16)
        self.assertFalse(self.root.findall('.//mesh'))

    def test_outline_nodes_are_fixed_and_do_not_add_fake_actuators(self):
        nodes=[j for j in self.root.findall('joint') if j.get('name').startswith(PREFIX)]
        self.assertEqual(len(nodes),8)
        self.assertTrue(all(j.get('type')=='fixed' and j.find('axis') is None for j in nodes))
        for side in ('L','R'):
            children=[j for j in nodes if j.find('parent').get('link')==f'grasper_{side}_Link']
            self.assertEqual(len(children),2)
            for base in children:
                downstream=[j for j in nodes if j.find('parent').get('link')==base.find('child').get('link')]
                self.assertEqual(len(downstream),1)
                # Existing Motus shows fixed connectors and a sphere at leaf links.
                self.assertGreater(math.sqrt(sum(float(v)**2 for v in downstream[0].find('origin').get('xyz').split())),.005)

    def test_two_fingers_follow_only_their_own_arm_with_constant_display_gap(self):
        before=positions(self.root,{})
        after=positions(self.root,{'wrist_roll_R_Joint':.4,'elbow_R_Joint':-.2})
        for finger in ('a','b'):
            left=f'{PREFIX}L_{finger}_tip_Link';right=f'{PREFIX}R_{finger}_tip_Link'
            self.assertEqual(before[left],after[left])
            self.assertGreater(math.dist(before[right],after[right]),.01)
        for poses in (before,after):
            for side in ('L','R'):
                self.assertAlmostEqual(math.dist(poses[f'{PREFIX}{side}_a_tip_Link'],poses[f'{PREFIX}{side}_b_tip_Link']),.05)
                for f in ('a','b'):
                    self.assertAlmostEqual(math.dist(poses[f'{PREFIX}{side}_{f}_tip_Link'],poses[f'grasper_{side}_Link']),math.hypot(.025,.065))

    def test_head_motion_does_not_move_gripper_outlines(self):
        before=positions(self.root,{})
        after=positions(self.root,{'head_yaw_Joint':.5,'head_pitch_Joint':.25})
        self.assertNotEqual(before['head_pitch_Link'],after['head_pitch_Link'])
        for side in ('L','R'):
            for f in ('a','b'):
                name=f'{PREFIX}{side}_{f}_tip_Link';self.assertEqual(before[name],after[name])

if __name__=='__main__':unittest.main()
