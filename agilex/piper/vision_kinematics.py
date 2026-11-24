"""PiPER joint feedback to link6 pose using the installed URDF chain."""

import xml.etree.ElementTree as ET

import cv2
import numpy as np


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


def vector(value):
    result = np.asarray(value.split(), dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("Invalid PiPER URDF vector")
    return result


def origin(joint):
    result = np.eye(4)
    element = joint.find("origin")
    if element is not None:
        roll, pitch, yaw = vector(element.get("rpy", "0 0 0"))
        result[:3, :3] = (cv2.Rodrigues(np.array([0, 0, yaw]))[0]
                           @ cv2.Rodrigues(np.array([0, pitch, 0]))[0]
                           @ cv2.Rodrigues(np.array([roll, 0, 0]))[0])
        result[:3, 3] = vector(element.get("xyz", "0 0 0"))
    return result


class PiperKinematics:
    def __init__(self, arm_urdf):
        arm = ET.fromstring(arm_urdf)
        self.joints = []
        parent = "base_link"
        for index, name in enumerate(JOINT_NAMES, start=1):
            joint = arm.find(f"joint[@name='{name}']")
            child = f"link{index}"
            if (joint is None or joint.get("type") != "revolute"
                    or joint.find("parent").get("link") != parent
                    or joint.find("child").get("link") != child):
                raise ValueError(f"Invalid PiPER URDF chain at {name}")
            axis = vector(joint.find("axis").get("xyz"))
            norm = np.linalg.norm(axis)
            if norm == 0:
                raise ValueError(f"Invalid PiPER URDF axis at {name}")
            self.joints.append((origin(joint), axis / norm))
            parent = child

    def base_from_link6_mm(self, joints_deg):
        angles = np.deg2rad(joints_deg)
        if angles.shape != (6,) or not np.isfinite(angles).all():
            raise ValueError("Expected six finite PiPER joint angles")
        pose = np.eye(4)
        for (fixed, axis), angle in zip(self.joints, angles):
            motion = np.eye(4)
            motion[:3, :3] = cv2.Rodrigues(axis * angle)[0]
            pose = pose @ fixed @ motion
        pose[:3, 3] *= 1000
        return pose
