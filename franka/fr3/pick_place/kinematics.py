"""FR3 kinematics and sampled arm clearance in the captured SDK EE frame."""
import itertools
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from hardware import JOINT_LOWER, JOINT_UPPER
from .arm_collision import CAPSULES, EXCLUDED_PAIRS


class Kinematics:
    def __init__(self, q, actual):
        from .planning import rigid, rotation
        self.rigid, self.rotation = rigid, rotation
        self.low, self.high = np.asarray(JOINT_LOWER)+.05, np.asarray(JOINT_UPPER)-.05
        self.origins = []
        for joint in ET.parse(Path(__file__).parents[1]/'resource/fr3.urdf').findall('joint'):
            o = joint.find('origin'); x, y, z = map(float, o.get('rpy').split())
            self.origins.append(rigid(rotation((0,0,1),z)@rotation((0,1,0),y)@rotation((1,0,0),x),
                                      list(map(float,o.get('xyz').split()))))
        self.offset = np.linalg.solve(self.frames(q)[-1], actual)

    def frames(self, q):
        chain = [np.eye(4)]
        for origin, angle in zip(self.origins, [*q,0]):
            chain.append(chain[-1]@origin@self.rigid(self.rotation((0,0,1),angle), [0]*3))
        return chain

    def pose(self, q):
        return self.frames(q)[-1]@self.offset

    def vector(self, q):
        t = self.pose(q)
        return np.r_[t[:3,3], .25*t[:3,:3].ravel()]

    def clearance(self, q):
        from .planning import segment_distance
        chain, caps = self.frames(q), []
        for link, start, end, radius in CAPSULES:
            t = chain[link]
            caps.append((link,t[:3,:3]@start+t[:3,3],t[:3,:3]@end+t[:3,3],radius))
        return min(segment_distance(a,b,c,d)-r-s
                   for (i,a,b,r),(j,c,d,s) in itertools.combinations(caps,2)
                   if i != j and tuple(sorted((i,j))) not in EXCLUDED_PAIRS)

    def table_clearance(self, q, table, normal):
        chain = self.frames(q)
        return min(min((chain[i][:3,:3] @ np.asarray(a)+chain[i][:3,3]-table) @ normal,
                       (chain[i][:3,:3] @ np.asarray(b)+chain[i][:3,3]-table) @ normal)-radius
                   for i, a, b, radius in CAPSULES)

    def solve(self, q, goal):
        q = np.array(q, dtype=float, copy=True)
        target = np.r_[goal[:3,3], .25*goal[:3,:3].ravel()]
        for _ in range(180):
            f = self.vector(q); error = target-f
            if np.linalg.norm(error) < .00003:
                return q
            jac = np.column_stack([(self.vector(q+d)-f)/1e-5 for d in np.eye(7)*1e-5])
            step = np.linalg.solve(jac.T@jac+1e-5*np.eye(7),jac.T@error)
            q = np.clip(q+np.clip(step,-.02,.02),self.low,self.high)
        raise ValueError('No complete transfer trajectory within joint limits')
