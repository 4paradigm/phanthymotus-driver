"""Fresh SDK feedback and settling checks for pick_place movements."""

import math
import time

import numpy as np

from hardware import JOINT_LIMITS_DEG


def vector(value, size, label):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise RuntimeError(f"Invalid {label}")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise RuntimeError(f"Non-finite {label}")
    return result


def rotation(pose):
    rx, ry, rz = pose[3:]
    cx, cy, cz = math.cos(rx), math.cos(ry), math.cos(rz)
    sx, sy, sz = math.sin(rx), math.sin(ry), math.sin(rz)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                     [-sy, cy*sx, cy*cx]])


def pose_close(first, second, distance=0.00015, angle=0.1):
    displacement = math.dist(first[:3], second[:3])
    cosine = (np.trace(rotation(first).T @ rotation(second)) - 1) / 2
    degrees = math.degrees(math.acos(float(np.clip(cosine, -1, 1))))
    return displacement <= distance and degrees <= angle


class ObservationMotion:
    def __init__(self, client, cancel, deadline=None):
        self.client = client
        self.cancel = cancel
        self.deadline = deadline

    def check_cancel(self):
        if self.cancel.is_set():
            raise RuntimeError("Action cancelled")
        if self.deadline is not None and time.monotonic() >= self.deadline:
            raise RuntimeError("Action timed out")

    def read(self):
        self.check_cancel()
        started = time.monotonic()
        state = self.client.call("rm_get_arm_all_state")
        errors = state.get("joint_err_code", [])
        enabled = state.get("joint_en_flag", [])
        if len(errors) != 7 or any(int(value) for value in errors):
            raise RuntimeError(f"Joint faults: {errors}")
        if len(enabled) != 7 or not all(int(value) == 1 for value in enabled):
            raise RuntimeError("All seven joints must be enabled")
        arm = self.client.call("rm_get_current_arm_state")
        for source in (state, arm):
            errors = source.get("err", {}).get("err")
            if not isinstance(errors, list) or any(int(value) for value in errors):
                raise RuntimeError(f"Invalid or faulted arm feedback: {errors}")
        joints = vector(self.client.call("rm_get_joint_degree"), 7, "joint feedback")
        pose = vector(arm.get("pose"), 6, "pose feedback")
        arm_joints = vector(arm.get("joint"), 7, "arm joint feedback")
        trajectory = self.client.call_dict("rm_get_arm_current_trajectory")
        idle = trajectory.get("trajectory_type") == 0
        if idle:
            planned = vector(trajectory.get("data"), 7, "idle joint feedback")
            if any(abs(a-b) > 0.2 for a, b in zip(joints, planned)):
                raise RuntimeError("Idle controller and joint feedback disagree")
            if any(abs(a-b) > 0.2 for a, b in zip(joints, arm_joints)):
                raise RuntimeError("Arm and joint feedback disagree")
        if time.monotonic() - started > 1.0:
            raise RuntimeError("SDK feedback is stale")
        self.check_cancel()
        return {"joints": joints, "pose": pose, "idle": idle}

    def frames(self):
        frames = {name: self.client.call(method) for name, method in (
            ("work", "rm_get_current_work_frame"), ("tool", "rm_get_current_tool_frame"))}
        for name, frame in frames.items():
            vector(frame.get("pose"), 6, name + " frame")
        self.check_cancel()
        return frames

    def validate_target(self, target):
        lower = vector(self.client.call("rm_get_joint_drive_min_pos"), 7, "lower joint limits")
        upper = vector(self.client.call("rm_get_joint_drive_max_pos"), 7, "upper joint limits")
        for index, (value, limits) in enumerate(zip(target, JOINT_LIMITS_DEG)):
            low, high = max(lower[index], limits[0]), min(upper[index], limits[1])
            if not low <= value <= high:
                raise ValueError(f"Observation J{index + 1} must be within [{low}, {high}]")

    def settled(self, target=None, timeout=4.0, check=None, guard=None, reached=None):
        deadline = time.monotonic() + timeout
        window = []
        best_error, progress_at = math.inf, time.monotonic()
        while time.monotonic() < deadline:
            feedback = self.read()
            if guard is not None:
                guard(feedback)
            if check is not None:
                check()
            now = time.monotonic()
            error = max(abs(a-b) for a, b in zip(feedback["joints"], target)) if target else 0
            if target and best_error - error >= 0.05:
                best_error, progress_at = error, now
            elif target and error > 0.2 and now - progress_at > 10:
                raise RuntimeError("Observation motion stalled")
            if feedback["idle"] and error <= 0.2 and (reached is None or reached(feedback)):
                if window and (not pose_close(window[0][1]["pose"], feedback["pose"])
                               or any(abs(a-b) > 0.05 for a, b in zip(window[0][1]["joints"], feedback["joints"]))):
                    window.clear()
                window.append((now, feedback))
                if len(window) >= 4 and now - window[0][0] >= 0.3:
                    return feedback
            else:
                window.clear()
            self.cancel.wait(0.1)
        raise RuntimeError("Arm motion did not settle before timeout")
