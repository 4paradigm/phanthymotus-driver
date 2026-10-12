"""URDF serial-chain FK and bounded damped least-squares IK using existing NumPy."""
import hashlib
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np

SIDES = ("left", "right")
MODEL = Path(__file__).with_name("models") / "piper.urdf"
MODEL_SHA256 = "22c15f95e50003b01713bae158638f7d19cdb71b1bd1095289865b835067002b"


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


class Calibration:
    def __init__(self, path):
        raw = Path(path).read_bytes()
        if len(raw) > 65536:
            raise ValueError("calibration_too_large")
        self.profile = json.loads(raw)
        p = self.profile
        if (not isinstance(p, dict) or p.get("schema") != "motus.piper.calibration/1"
                or p.get("hardware_verified") is not True):
            raise ValueError("verified_piper_calibration_required")
        if p.get("model_sha256") != MODEL_SHA256 or hashlib.sha256(MODEL.read_bytes()).hexdigest() != MODEL_SHA256:
            raise ValueError("piper_model_mismatch")
        if not isinstance(p.get("calibration_id"), str) or not p["calibration_id"]:
            raise ValueError("calibration_id_required")
        if not isinstance(p.get("arms"), dict) or set(p["arms"]) != set(SIDES):
            raise ValueError("left_and_right_calibration_required")
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.tcp, self.controller, self.tracking, self.bases, self.workspaces = {}, {}, {}, {}, {}
        for side in SIDES:
            arm = p["arms"][side]
            if not isinstance(arm.get("can_interface"), str) or not arm["can_interface"]:
                raise ValueError("can_binding_required")
            self.tcp[side] = transform(arm.get("flange_to_tcp"))
            self.controller[side] = transform(arm.get("controller_to_tcp"))
            self.tracking[side] = transform(arm.get("tracking_to_base"))[:3, :3]
            self.bases[side] = transform(arm.get("base_to_world"))
            bounds = arm.get("workspace_m")
            if not isinstance(bounds, list) or len(bounds) != 2:
                raise ValueError("workspace_required")
            low, high = (vector(v, 3, "workspace") for v in bounds)
            if np.any(low >= high):
                raise ValueError("invalid_workspace")
            self.workspaces[side] = low, high
        if p["arms"]["left"]["can_interface"] == p["arms"]["right"]["can_interface"]:
            raise ValueError("duplicate_can_binding")

    def check_workspace(self, side, pose):
        low, high = self.workspaces[side]
        if np.any(pose[:3, 3] < low) or np.any(pose[:3, 3] > high):
            raise ValueError(side + "_tcp_outside_workspace")


class Chain:
    def __init__(self):
        self.origins, self.axes, self.lower, self.upper = [], [], [], []
        tree = ET.fromstring(MODEL.read_bytes())
        parent = "base_link"
        for i in range(1, 7):
            joint = tree.find(f"joint[@name='joint{i}']")
            if joint is None or joint.attrib["type"] != "revolute" or joint.find("parent").attrib["link"] != parent:
                raise ValueError("invalid_piper_serial_chain")
            parent = joint.find("child").attrib["link"]
            origin = joint.find("origin")
            rpy = [float(v) for v in origin.attrib["rpy"].split()]
            t = np.eye(4)
            t[:3, :3] = rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0])
            t[:3, 3] = [float(v) for v in origin.attrib["xyz"].split()]
            axis = np.array([float(v) for v in joint.find("axis").attrib["xyz"].split()])
            self.origins.append(t)
            self.axes.append(axis / np.linalg.norm(axis))
            self.lower.append(float(joint.find("limit").attrib["lower"]))
            self.upper.append(float(joint.find("limit").attrib["upper"]))
        self.lower, self.upper = np.array(self.lower), np.array(self.upper)

    def fk(self, q, tcp, jacobian=False):
        q = vector(q, 6, "joint_positions")
        t = np.eye(4)
        origins, axes = [], []
        for origin, axis, angle in zip(self.origins, self.axes, q):
            t = t @ origin
            origins.append(t[:3, 3].copy())
            axes.append(t[:3, :3] @ axis)
            r = np.eye(4)
            r[:3, :3] = rotation(axis, angle)
            t = t @ r
        t = t @ tcp
        if not jacobian:
            return t
        j = np.array([np.r_[np.cross(a, t[:3, 3] - o), a] for a, o in zip(axes, origins)]).T
        return t, j

    def solve(self, target, seed, tcp, deadline):
        q = vector(seed, 6, "joint_positions").copy()
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("feedback_outside_model_limits")
        for _ in range(80):
            if time.monotonic() >= deadline:
                raise ValueError("command_expired")
            pose, j = self.fk(q, tcp, jacobian=True)
            error = np.r_[target[:3, 3] - pose[:3, 3], rotation_error(target[:3, :3], pose[:3, :3])]
            if np.linalg.norm(error[:3]) <= .002 and np.linalg.norm(error[3:]) <= .02:
                return q
            # Metres and radians need comparable weighting in the least-squares objective.
            error[3:] *= .2
            j[3:] *= .2
            step = j.T @ np.linalg.solve(j @ j.T + .0001*np.eye(6), error)
            q = np.clip(q + np.clip(step, -.15, .15), self.lower, self.upper)
        raise ValueError("piper_ik_no_solution")


class PiperIK:
    """Solver interface consumed by common.motion.worker.NumericalWorker."""
    def __init__(self, path):
        self.calibration = Calibration(path)
        self.profile, self.profile_sha256 = self.calibration.profile, self.calibration.sha256
        self.chain = Chain()
        self.indices = np.arange(12)
        self.velocity = self.velocity_limit = math.radians(1)
        self.model = SimpleNamespace(lowerPositionLimit=np.tile(self.chain.lower, 2),
                                     upperPositionLimit=np.tile(self.chain.upper, 2),
                                     velocityLimit=np.full(12, self.velocity))
        self.hands_enabled = False
        self.collision_checks_enabled = False
        self.last_ms = 0.
        self.last_envelope = {}

    def self_test(self, q):
        q = vector(q, 12, "joint_positions")
        if np.any(q < self.model.lowerPositionLimit) or np.any(q > self.model.upperPositionLimit):
            raise ValueError("feedback_outside_model_limits")
        for side, tip in zip(SIDES, self.palms(q)):
            self.calibration.check_workspace(side, tip)

    def palms(self, q):
        return [self.chain.fk(q[i*6:(i+1)*6], self.calibration.tcp[side]) for i, side in enumerate(SIDES)]

    def solve(self, targets, q, commanded, deadline_monotonic):
        started = time.monotonic()
        q = vector(q, 12, "joint_positions")
        out = []
        for i, (side, target) in enumerate(zip(SIDES, targets)):
            self.calibration.check_workspace(side, target)
            out.extend(self.chain.solve(target, q[i*6:(i+1)*6], self.calibration.tcp[side], deadline_monotonic))
        self.last_ms = (time.monotonic() - started)*1000
        self.last_envelope = {"collision_checked": False}
        return out


class RelativeMapping:
    def __init__(self, calibration, q, frame, widths):
        self.calibration = calibration
        chain = Chain()
        self.tips = {side: chain.fk(q[i*6:(i+1)*6], calibration.tcp[side]) for i, side in enumerate(SIDES)}
        self.controllers = {side: self.pose(frame[side], side) for side in SIDES}
        self.triggers = {side: frame[side]["trigger"] for side in SIDES}
        self.widths = dict(widths)

    def pose(self, value, side):
        return transform({"position": value["position"], "orientation": value["orientation_xyzw"]}) @ self.calibration.controller[side]

    def targets(self, frame, ranges):
        targets, widths = [], {}
        for side in SIDES:
            current, initial = self.pose(frame[side], side), self.controllers[side]
            rotate = self.calibration.tracking[side]
            target = self.tips[side].copy()
            target[:3, 3] += rotate @ (current[:3, 3] - initial[:3, 3])
            target[:3, :3] = rotate @ current[:3, :3] @ initial[:3, :3].T @ rotate.T @ target[:3, :3]
            self.calibration.check_workspace(side, target)
            targets.extend(pose_values(target))
            low, high = ranges[side]
            widths[side] = float(np.clip(self.widths[side] - (frame[side]["trigger"]-self.triggers[side])*(high-low), low, high))
        return targets, widths
