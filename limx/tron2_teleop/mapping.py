"""Relative controller mapping adopted from G1 PR #330 (Apache-2.0)."""
import copy
import math


def mul(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def inverse(q):
    return [-q[0], -q[1], -q[2], q[3]]


def rotate(q, p):
    return mul(mul(q, [*p, 0]), inverse(q))[:3]


class RelativeMapping:
    def __init__(self, frame, poses, scale):
        self.origin, self.poses, self.scale = copy.deepcopy(frame), copy.deepcopy(poses), scale
        forward = rotate(frame['head_reference']['orientation_xyzw'], [1, 0, 0])
        if math.hypot(*forward[:2]) < .1:
            raise ValueError('head_yaw_unavailable')
        yaw = math.atan2(forward[1], forward[0])
        self.alignment = [0, 0, math.sin(-yaw/2), math.cos(-yaw/2)]

    def targets(self, frame):
        values = []
        for side, robot in zip(('left', 'right'), self.poses):
            current, start = frame[side], self.origin[side]
            delta = rotate(self.alignment, [a-b for a, b in zip(current['position'], start['position'])])
            rotation = mul(current['orientation_xyzw'], inverse(start['orientation_xyzw']))
            aligned = mul(mul(self.alignment, rotation), inverse(self.alignment))
            values.extend(a+self.scale*b for a, b in zip(robot[:3], delta))
            values.extend(mul(aligned, robot[3:]))
        return values
