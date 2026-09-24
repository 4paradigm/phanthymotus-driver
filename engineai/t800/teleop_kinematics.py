"""Bounded, NumPy-only arm IK for the checked-in T800 25-joint model.

Five joints cannot satisfy an arbitrary six-dimensional pose. Position is the
primary task; orientation uses the position Jacobian's null space. This is a
local workspace projection, not a global nearest-point or collision planner.
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from control import T800_JOINT_NAMES, T800_JOINT_POSITION_LIMITS


ARM_INDICES = tuple(range(13, 23))
MODEL = Path(__file__).with_name("resource") / "serial_t800.urdf"


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross


def quaternion_matrix(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def rotation_error(target, actual):
    # Bounded spatial orientation error; position remains authoritative even
    # at the 180-degree ambiguity of this secondary task.
    return np.cross(actual.T, target.T).sum(axis=0) * .5


class ArmModel:
    def __init__(self, side, path=MODEL):
        root = ET.parse(path).getroot()
        joints = {j.find("child").get("link"): j for j in root.findall("joint")}
        expected = T800_JOINT_NAMES[13:18] if side == "left" else T800_JOINT_NAMES[18:23]
        link = "LINK_WRIST_END_L" if side == "left" else "LINK_WRIST_END_R"
        chain = []
        # Stop at the parent of the first shoulder joint: torso-local targets
        # never command waist or legs to compensate for an unreachable hand.
        while True:
            joint = joints[link]
            chain.append(joint)
            if joint.get("name") == expected[0]:
                break
            link = joint.find("parent").get("link")
        self.chain = []
        names, limits = [], []
        for joint in reversed(chain):
            origin = joint.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            r, p, y = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
            transform = np.eye(4)
            transform[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
            transform[:3, 3] = xyz
            axis = None
            if joint.get("type") != "fixed":
                if joint.get("type") != "revolute":
                    raise ValueError("unsupported_t800_joint")
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                axis /= np.linalg.norm(axis)
                limit = joint.find("limit")
                limits.append([float(limit.get("lower")), float(limit.get("upper"))])
                names.append(joint.get("name"))
            self.chain.append((transform, axis))
        if tuple(names) != tuple(expected):
            raise ValueError("t800_arm_model_mismatch")
        indices = range(13, 18) if side == "left" else range(18, 23)
        if not np.allclose(limits, [T800_JOINT_POSITION_LIMITS[i] for i in indices]):
            raise ValueError("t800_arm_limits_mismatch")
        self.lower = np.asarray(limits)[:, 0] + .02
        self.upper = np.asarray(limits)[:, 1] - .02

    def forward(self, q, *, jacobian=True):
        transform = np.eye(4)
        origins, axes = [], []
        index = 0
        for fixed, axis in self.chain:
            transform = transform @ fixed
            if axis is not None:
                if jacobian:
                    origins.append(transform[:3, 3].copy())
                    axes.append(transform[:3, :3] @ axis)
                transform[:3, :3] = transform[:3, :3] @ rotation(axis, q[index])
                index += 1
        position = transform[:3, 3]
        if not jacobian:
            return position.copy(), transform[:3, :3].copy(), None, None
        linear = np.cross(np.asarray(axes), position-np.asarray(origins)).T
        return position.copy(), transform[:3, :3].copy(), linear, np.asarray(axes).T

    def solve(self, position, orientation, seed):
        """Return a finite in-limit local projection, with explicit residual."""
        q = np.clip(np.asarray(seed, dtype=float), self.lower, self.upper)
        target = np.asarray(position, dtype=float)
        desired_rotation = np.asarray(orientation, dtype=float)
        if target.shape != (3,) or desired_rotation.shape != (3, 3) or not (
            np.isfinite(target).all() and np.isfinite(desired_rotation).all()
        ):
            raise ValueError("invalid_ik_target")
        # Fixed iteration/line-search budgets keep the numerical worker bounded.
        # Reject steps that worsen position; orientation gets at most 1 mm slack.
        for _ in range(8):
            p, r, j, angular = self.forward(q)
            error = target - p
            distance = float(np.linalg.norm(error))
            orientation_error = rotation_error(desired_rotation, r)
            if distance < .001 and np.linalg.norm(orientation_error) < .02:
                break
            inverse = j.T @ np.linalg.solve(j @ j.T + .015**2 * np.eye(3), np.eye(3))
            primary = inverse @ error
            null = np.eye(5) - np.linalg.pinv(j, rcond=.02) @ j
            secondary = .25 * null @ angular.T @ orientation_error
            step = primary + secondary
            step *= min(1., .12 / max(float(np.max(np.abs(step))), 1e-12))
            accepted = False
            for fraction in (1., .5, .25, .125):
                candidate = np.clip(q + fraction * step, self.lower, self.upper)
                cp, cr, _, _ = self.forward(candidate, jacobian=False)
                residual = float(np.linalg.norm(target - cp))
                better_position = residual < distance - 1e-7
                better_orientation = (distance < .004 and residual <= .004
                    and np.linalg.norm(rotation_error(desired_rotation, cr))
                    < np.linalg.norm(rotation_error(desired_rotation, r)) - 1e-6)
                if better_position or better_orientation:
                    q, accepted = candidate, True
                    break
            if not accepted:
                break
        residual = float(np.linalg.norm(target-self.forward(q, jacobian=False)[0]))
        return q, residual


class DualArmModel:
    def __init__(self):
        self.arms = [ArmModel("left"), ArmModel("right")]
        self.lower = np.concatenate([arm.lower for arm in self.arms])
        self.upper = np.concatenate([arm.upper for arm in self.arms])

    def poses(self, q):
        return [arm.forward(q[5*i:5*i+5], jacobian=False)[:2] for i, arm in enumerate(self.arms)]

    def solve(self, targets, seed, reference=None):
        answers = [arm.solve(*targets[i], seed[5*i:5*i+5]) for i, arm in enumerate(self.arms)]
        # A fully extended elbow can get trapped on the other side of a local
        # singularity/limit. The measured calibration pose is a second seed for
        # recovery, never a new mapping. Execution still applies its rate limits.
        if reference is not None:
            for i, arm in enumerate(self.arms):
                if answers[i][1] > .015:
                    alternative = arm.solve(*targets[i], reference[5*i:5*i+5])
                    if alternative[1] + .005 < answers[i][1]:
                        answers[i] = alternative
        return np.concatenate([v[0] for v in answers]), [v[1] for v in answers]


class RelativeMapping:
    def __init__(self, frame, poses, scale, reference=None):
        forward = quaternion_matrix(frame["head_reference"]["orientation_xyzw"])[:, 0]
        if np.linalg.norm(forward[:2]) < .2:
            raise ValueError("look_forward_to_calibrate")
        self.alignment = rotation([0, 0, 1], -math.atan2(forward[1], forward[0]))
        self.origins = [np.asarray(frame[s]["position"]) for s in ("left", "right")]
        self.rotations = [quaternion_matrix(frame[s]["orientation_xyzw"]) for s in ("left", "right")]
        self.poses, self.scale = poses, scale
        self.reference = None if reference is None else reference.copy()

    def targets(self, frame):
        result = []
        for i, side in enumerate(("left", "right")):
            pose = frame[side]
            delta = self.alignment @ (np.asarray(pose["position"]) - self.origins[i])
            turn = quaternion_matrix(pose["orientation_xyzw"]) @ self.rotations[i].T
            result.append((self.poses[i][0] + self.scale * delta,
                self.alignment @ turn @ self.alignment.T @ self.poses[i][1]))
        return result


def smooth_reference(q, velocity, target, dt, lower, upper):
    """Critically damped reference with velocity/acceleration bounds.

Bounds apply to command references, not measured physical motor dynamics.
Safety holds may deliberately zero velocity immediately.
"""
    acceleration = np.clip(64. * (target-q) - 16. * velocity, -2., 2.)
    next_velocity = np.clip(velocity + acceleration * dt, -.5, .5)
    position = np.clip(q + next_velocity * dt, lower, upper)
    return position, (position-q) / dt
