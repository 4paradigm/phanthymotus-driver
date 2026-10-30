"""Approximate two-hand preview for the dashboard's sensor/skeleton renderer.

The vendor protocol supplies ten 0..255 positions, not joint angles. Only the
five flexion motors have measured open/closed endpoints. This model is for a
live visual preview and is never used to calculate motor commands.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from modbus_client import _bytes

MODEL_DIR = Path(__file__).with_name("resource")
FLEX_MOTORS = {
    "thumb_cmc_pitch": 0,
    "index_mcp_pitch": 2,
    "middle_mcp_pitch": 3,
    "ring_mcp_pitch": 4,
    "pinky_mcp_pitch": 5,
}
UNCALIBRATED_THUMB_MIMICS = {"thumb_mcp", "thumb_ip"}


class GloveSkeleton:
    def __init__(self, poses_by_side, thumb_endpoints=None):
        self.thumb_endpoints = thumb_endpoints
        self.poses = {}
        self.joints = {}
        sources = {}
        for side in ("left", "right"):
            open_pose, fist_pose = poses_by_side[side]
            self.poses[side] = (_bytes(open_pose), _bytes(fist_pose))
            if thumb_endpoints:
                for motor, opened, closed in zip((0, 1, 9), *thumb_endpoints[side]):
                    self.poses[side][0][motor] = opened
                    self.poses[side][1][motor] = closed
            source = ET.parse(MODEL_DIR / f"l20lite_{side}_skeleton.urdf").getroot()
            sources[side] = source
            joints = []
            for element in source.findall("joint"):
                limit = element.find("limit")
                mimic = element.find("mimic")
                joints.append(
                    (
                        element.attrib["name"],
                        float(limit.attrib["lower"]) if limit is not None else 0.0,
                        float(limit.attrib["upper"]) if limit is not None else 0.0,
                        mimic.attrib["joint"] if mimic is not None else None,
                        (
                            float(mimic.attrib.get("multiplier", "1"))
                            if mimic is not None
                            else 1.0
                        ),
                        (
                            float(mimic.attrib.get("offset", "0"))
                            if mimic is not None
                            else 0.0
                        ),
                    )
                )
            if len(joints) != 20:
                raise ValueError(f"expected 20 joints in {side} L20 Lite preview URDF")
            for joint, motor in FLEX_MOTORS.items():
                if self.poses[side][0][motor] == self.poses[side][1][motor]:
                    raise ValueError(f"{side} {joint} needs distinct visual endpoints")
            self.joints[side] = joints
        self.urdf = self._dual_urdf(sources)

    @staticmethod
    def _dual_urdf(sources):
        """One model resource shows both hands, since Core fetches one per driver."""
        target = ET.Element("robot", {"name": "l20lite_dual_preview"})
        ET.SubElement(target, "link", {"name": "dual_hand_base"})
        for side, x in (("left", -0.45), ("right", 0.45)):
            for element in sources[side]:
                copy = ET.fromstring(ET.tostring(element))
                copy.attrib["name"] = f"{side}_{copy.attrib['name']}"
                if copy.tag == "joint":
                    for tag in ("parent", "child"):
                        node = copy.find(tag)
                        node.attrib["link"] = f"{side}_{node.attrib['link']}"
                    mimic = copy.find("mimic")
                    if mimic is not None:
                        mimic.attrib["joint"] = f"{side}_{mimic.attrib['joint']}"
                target.append(copy)
            anchor = ET.SubElement(
                target, "joint", {"name": f"{side}_anchor", "type": "fixed"}
            )
            ET.SubElement(anchor, "origin", {"xyz": f"{x} 0 0", "rpy": "0 0 0"})
            ET.SubElement(anchor, "parent", {"link": "dual_hand_base"})
            ET.SubElement(anchor, "child", {"link": f"{side}_hand_base_link"})
        return ET.tostring(target, encoding="unicode")

    def sample(self, side: str, raw):
        raw = _bytes(raw)
        open_pose, fist_pose = self.poses[side]
        angles = {}
        result = []
        for index, (name, lower, upper, parent, multiplier, offset) in enumerate(
            self.joints[side]
        ):
            motor = FLEX_MOTORS.get(name)
            if self.thumb_endpoints and name in ("thumb_cmc_yaw", "thumb_cmc_roll"):
                motor = 1 if name == "thumb_cmc_yaw" else 9
            if motor is not None:
                fraction = (raw[motor] - open_pose[motor]) / (
                    fist_pose[motor] - open_pose[motor]
                )
                angle = lower + max(0.0, min(1.0, fraction)) * (upper - lower)
            elif name in UNCALIBRATED_THUMB_MIMICS and not self.thumb_endpoints:
                angle = lower
            elif parent:
                angle = angles[parent] * multiplier + offset
            else:
                angle = lower
            angle = max(lower, min(upper, angle))
            angles[name] = angle
            result.append(
                {
                    "idx": index + (0 if side == "left" else 20),
                    "name": f"{side}_{name}",
                    "q": angle,
                    "degree": math.degrees(angle),
                }
            )
        return result
