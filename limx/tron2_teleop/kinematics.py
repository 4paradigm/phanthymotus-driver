"""Seven-joint DLS IK, adapted from PiPER PR #395 (Apache-2.0)."""
import hashlib
import time
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np

from limx.tron2_teleop.model import ARM_NAMES, MODEL, MODEL_SHA256, limits


def vector(value, size, name):
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != size:
        raise ValueError("invalid_" + name)
    if any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, float, np.number))
           or not math.isfinite(v) for v in value):
        raise ValueError("invalid_" + name)
    return np.asarray(value, dtype=float)


def rotation(axis, angle):
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


def transform(pose):
    if not isinstance(pose, dict):
        raise ValueError("invalid_pose")
    p = vector(pose.get("position"), 3, "position")
    q = vector(pose.get("orientation"), 4, "orientation")
    if abs(float(q @ q) - 1) > .002:
        raise ValueError("invalid_quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = p
    return out


def quaternion(matrix):
    # Eigenvector form remains well-conditioned at 180 degrees.
    r = matrix[:3, :3]
    k = np.array([[r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
                  [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
                  [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
                  [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)]])
    _, eigenvectors = np.linalg.eigh(k)
    q = eigenvectors[:, -1]
    return q if q[3] >= 0 else -q


def pose_values(matrix):
    return matrix[:3, 3].tolist() + quaternion(matrix).tolist()


def rotation_error(target, actual):
    q = quaternion(target @ actual.T)
    length = np.linalg.norm(q[:3])
    return 2*q[:3] if length < 1e-10 else q[:3] * (2*math.atan2(length, q[3])/length)


class Chain:
    def __init__(self, side, tree):
        index = 0 if side == 'left' else 7
        self.origins, self.axes = [], []
        lower, upper = limits()
        self.lower, self.upper = np.array(lower[index:index+7]), np.array(upper[index:index+7])
        parent = 'base_Link'
        for name in ARM_NAMES[index:index+7]:
            joint = tree.find(f"joint[@name='{name}']")
            if joint.attrib['type'] != 'revolute' or joint.find('parent').attrib['link'] != parent:
                raise ValueError('invalid_tron2_chain')
            parent = joint.find('child').attrib['link']
            origin = joint.find('origin')
            rpy = [float(v) for v in origin.attrib['rpy'].split()]
            t = np.eye(4)
            t[:3, :3] = rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0])
            t[:3, 3] = [float(v) for v in origin.attrib['xyz'].split()]
            axis = np.array([float(v) for v in joint.find('axis').attrib['xyz'].split()])
            self.origins.append(t)
            self.axes.append(axis / np.linalg.norm(axis))

    def fk(self, q, jacobian=False):
        q = vector(q, 7, 'joint_positions')
        t, origins, axes = np.eye(4), [], []
        for origin, axis, angle in zip(self.origins, self.axes, q):
            t = t @ origin
            origins.append(t[:3, 3].copy())
            axes.append(t[:3, :3] @ axis)
            r = np.eye(4)
            r[:3, :3] = rotation(axis, angle)
            t = t @ r
        if not jacobian:
            return t
        j = np.array([np.r_[np.cross(a, t[:3, 3]-o), a] for a, o in zip(axes, origins)]).T
        return t, j

    def solve(self, target, seed, reference, deadline):
        q = vector(seed, 7, 'joint_positions').copy()
        for _ in range(100):
            if time.monotonic() >= deadline:
                raise ValueError('command_expired')
            pose, j = self.fk(q, jacobian=True)
            error = np.r_[target[:3, 3]-pose[:3, 3], rotation_error(target[:3, :3], pose[:3, :3])]
            if np.linalg.norm(error[:3]) <= .002 and np.linalg.norm(error[3:]) <= .02:
                return q
            error[3:] *= .2
            j[3:] *= .2
            pinv = j.T @ np.linalg.solve(j @ j.T + .0001*np.eye(6), np.eye(6))
            # Bias the redundant joint toward measured posture, not an arbitrary elbow branch.
            step = pinv @ error + (np.eye(7)-pinv @ j) @ (.05*(reference-q))
            q = np.clip(q + np.clip(step, -.1, .1), self.lower, self.upper)
        raise ValueError('tron2_ik_no_solution')


class Tron2IK:
    def __init__(self, path):
        raw = Path(path).read_bytes()
        if hashlib.sha256(raw).hexdigest() != MODEL_SHA256:
            raise ValueError('tron2_model_mismatch')
        tree = ET.fromstring(raw)
        self.chains = [Chain(side, tree) for side in ('left', 'right')]
        self.indices = np.arange(14)
        lower, upper = limits()
        self.velocity = self.velocity_limit = .35
        self.model = SimpleNamespace(lowerPositionLimit=np.array(lower[:14]),
            upperPositionLimit=np.array(upper[:14]), velocityLimit=np.full(14, self.velocity))
        self.profile = {'model_sha256': MODEL_SHA256, 'end_frames': ['wrist_roll_L_Link', 'wrist_roll_R_Link']}
        self.profile_sha256 = MODEL_SHA256
        self.hands_enabled = self.collision_checks_enabled = False
        self.last_ms, self.last_envelope = 0., {}

    def self_test(self, q):
        q = vector(q, 14, 'joint_positions')
        if np.any(q < self.model.lowerPositionLimit) or np.any(q > self.model.upperPositionLimit):
            raise ValueError('feedback_joint_limit')

    def palms(self, q):
        self.self_test(q)
        return [chain.fk(q[i*7:i*7+7]) for i, chain in enumerate(self.chains)]

    def solve(self, targets, q, commanded, deadline_monotonic):
        started = time.monotonic()
        self.self_test(q)
        seed = q if commanded is None else commanded
        self.self_test(seed)
        result = []
        for i, (chain, target) in enumerate(zip(self.chains, targets)):
            result.extend(chain.solve(target, seed[i*7:i*7+7], np.array(q[i*7:i*7+7]), deadline_monotonic))
        self.last_ms = (time.monotonic()-started)*1000
        self.last_envelope = {'collision_checked': False}
        return result
