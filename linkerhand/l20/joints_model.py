"""L20-10 joint feedback: calibrated angles and the skeleton model.

One driver serves one model per request. A panel asks for the model with its
own topic, so the card's hand can be drawn alone; without a topic the old
dual-hand URDF is kept as a fallback for callers that cannot send one.

Hand joints are named `{side}_hand_...` (links keep `{side}_...`): Core's
renderer picks hand styling from the joint name containing "hand", so this is
what makes every finger use the thin bone and small joint sphere.

Raw 0..255 positions are measured. Angles come from the vendor's per-joint
calibration tables for the L20-10 (files named g20, resource/g20_*_G20.json),
which also assign every URDF joint to a motor. The vendor confirmed these files
for the L20-10; every output carries ANGLE_SOURCE so consumers can tell.
"""

from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

from can_client import CHINESE_JOINT_NAMES_16, JOINT_NAMES_16

MODEL_DIR = Path(__file__).with_name("resource")
MODEL_FILES = {"left": "linkerhand_g20_left.urdf", "right": "linkerhand_g20_right.urdf"}
CALIBRATION_FILES = {"left": "g20_left_G20.json", "right": "g20_right_G20.json"}
# Fallback URDF (no topic): both hands side by side, as before.
ANCHOR_X = {"left": -0.45, "right": 0.45}
# Both vendor hands have the palm towards +x, so a plain shift shows them facing
# the same way. Turning each hand -90 deg about z puts both palms towards Core's
# camera (which looks from -y) with the thumbs pointing outward, as when looking
# at one's own palms.
ANCHOR_YAW = {"left": -math.pi / 2, "right": -math.pi / 2}
# Core's camera is framed for full-size robots; a bare hand needs enlarging.
DISPLAY_SCALE = 4.0
ANGLE_SOURCE = "vendor_g20_calibration_table_adjusted"
# Tables used with the opposite sign. The vendor left-hand file gives the four
# finger mcp_roll tables the same sign as the right hand while both URDFs use
# the same axis, so on the left the V sign closed the fingers instead of
# spreading them (real hand, 2026-11-13).
NEGATED_TABLES = {
    "left": frozenset(f"{finger}_mcp_roll" for finger in ("index", "middle", "ring", "pinky")),
    "right": frozenset(),
}
# Fingertip in each distal link frame (metres). Core draws bones only up to joint
# origins, so without a tip link each finger would end at its last knuckle. The
# tip continues the direction of the previous bone (so a straight finger is drawn
# straight) up to the end of the vendor STL mesh along that direction; the mesh's
# own far point lies towards the back of the hand and made tips look bent outward.
FINGERTIPS = {
    "thumb": ("thumb_distal", (-0.0044, 0.0, 0.0315)),
    **{finger: (f"{finger}_distal", (0.0019, 0.0, 0.0214)) for finger in ("index", "middle", "ring", "pinky")},
}

# Index of each active joint in the driver's 20-item vector (11..14 are reserved).
VECTOR_INDEX = {
    "thumb_pitch": 0, "thumb_yaw": 5, "thumb_roll": 10, "thumb_tip": 15,
    "index_pitch": 1, "index_yaw": 6, "index_tip": 16,
    "middle_pitch": 2, "middle_yaw": 7, "middle_tip": 17,
    "ring_pitch": 3, "ring_yaw": 8, "ring_tip": 18,
    "pinky_pitch": 4, "pinky_yaw": 9, "pinky_tip": 19,
}
RESERVED_VECTOR_INDEXES = (11, 12, 13, 14)

# Driver joint -> URDF joint, as given by motor_index in the calibration files
# (checked on load). The driver's thumb "yaw" motor turns the URDF cmc_roll joint
# and its "roll" motor the cmc_yaw joint.
URDF_JOINT = {
    "thumb_pitch": "thumb_cmc_pitch", "thumb_yaw": "thumb_cmc_roll",
    "thumb_roll": "thumb_cmc_yaw", "thumb_tip": "thumb_mcp",
    **{f"{finger}_{part}": f"{finger}_{joint}"
       for finger in ("index", "middle", "ring", "pinky")
       for part, joint in (("pitch", "mcp_pitch"), ("yaw", "mcp_roll"), ("tip", "pip"))},
}

JOINT_LABELS = dict(zip(JOINT_NAMES_16, CHINESE_JOINT_NAMES_16))
SIDE_LABELS = {"left": "左手", "right": "右手"}


def _vendor_root(side: str) -> ET.Element:
    return ET.parse(MODEL_DIR / MODEL_FILES[side]).getroot()


@lru_cache(maxsize=None)
def side_joints(side: str) -> tuple[dict, ...]:
    """Revolute joints of one vendor hand in document order, with limits and mimic."""
    joints = []
    for element in _vendor_root(side).findall("joint"):
        if element.get("type") != "revolute":
            continue
        limit = element.find("limit")
        mimic = element.find("mimic")
        joints.append({
            "name": element.get("name"),
            "lower": float(limit.get("lower")),
            "upper": float(limit.get("upper")),
            "mimic": None if mimic is None else (
                mimic.get("joint"), float(mimic.get("multiplier", "1")), float(mimic.get("offset", "0"))),
        })
    return tuple(joints)


@lru_cache(maxsize=None)
def calibration(side: str) -> dict[str, dict]:
    """Per URDF joint: driving motor, passive flag and the 256-entry raw -> radian table.

    Passive distal joints have their own table on the motor of their parent.
    Tables listed in NEGATED_TABLES are returned with the opposite sign.
    Raises if the file does not cover exactly the URDF's revolute joints or
    disagrees with URDF_JOINT.
    """
    data = json.loads((MODEL_DIR / CALIBRATION_FILES[side]).read_text(encoding="utf-8"))
    joints = {
        name: {"motor_index": int(entry["motor_index"]), "passive": bool(entry.get("passive", False)),
               "angle_rad": tuple((-1.0 if name in NEGATED_TABLES[side] else 1.0) * float(v)
                                  for v in entry["angle_rad"])}
        for name, entry in data["joints"].items()
    }
    if data.get("side") != side or set(joints) != {j["name"] for j in side_joints(side)}:
        raise ValueError(f"{CALIBRATION_FILES[side]} does not match {MODEL_FILES[side]}")
    active = {entry["motor_index"]: name for name, entry in joints.items() if not entry["passive"]}
    if active != {VECTOR_INDEX[name]: joint for name, joint in URDF_JOINT.items()}:
        raise ValueError(f"{CALIBRATION_FILES[side]} motor assignment differs from URDF_JOINT")
    for name, entry in joints.items():
        if len(entry["angle_rad"]) != 256 or entry["motor_index"] not in active:
            raise ValueError(f"{CALIBRATION_FILES[side]}: bad table for {name}")
    return joints


def _table_angle(entry: dict, raw: int) -> float:
    return entry["angle_rad"][min(255, max(0, raw))]


def raw_to_angle(side: str, name: str, raw: int) -> float:
    """Calibrated URDF angle of one active joint, in radians."""
    return _table_angle(calibration(side)[URDF_JOINT[name]], raw)


def skeleton_joints(side: str, raw: dict[str, int]) -> list[dict]:
    """sensor/skeleton entries for one hand from raw positions, keyed by dual-model joint names.

    Values are limited to the model range (flagged when that changes the value).
    Passive distal joints use their own calibration table on the parent's motor
    and are flagged as derived, not measured.
    """
    driver = {VECTOR_INDEX[name]: name for name in raw}
    tables = calibration(side)
    entries = []
    offset = 0 if side == "left" else len(side_joints("left"))
    for index, joint in enumerate(side_joints(side)):
        name = joint["name"]
        table = tables[name]
        driver_name = driver.get(table["motor_index"])
        if driver_name is None:
            continue
        value = _table_angle(table, raw[driver_name])
        if table["passive"]:
            source, label = "derived_calibration", f"{name}（由{JOINT_LABELS[driver_name]}推算）"
        else:
            source, label = "measured", JOINT_LABELS[driver_name]
        q = min(joint["upper"], max(joint["lower"], value))
        entries.append({
            "idx": offset + index, "name": f"{side}_hand_{name}", "q": q, "q_deg": math.degrees(q),
            "group": SIDE_LABELS[side], "label": label, "source": source,
            "clamped_to_model": q != value,
        })
    return entries


def _floats(text: str | None) -> list[float]:
    return [float(v) for v in (text or "0 0 0").split()]


def _fmt(values) -> str:
    return " ".join(repr(float(v)) for v in values)


def _single_axis_origins(xyz: list[float], rpy: list[float]) -> list[tuple[list[float], list[float]]]:
    """Split URDF fixed-axis rpy (R = Rz*Ry*Rx) into single-axis steps.

    Core applies rpy as an intrinsic XYZ Euler rotation (Rx*Ry*Rz), which only
    agrees with URDF when at most one component is non-zero.
    """
    roll, pitch, yaw = rpy
    steps = [rot for rot in ([0.0, 0.0, yaw], [0.0, pitch, 0.0], [roll, 0.0, 0.0]) if any(rot)]
    if len(steps) <= 1:
        return [(xyz, rpy)]
    return [(xyz if i == 0 else [0.0, 0.0, 0.0], rot) for i, rot in enumerate(steps)]


def _side_elements(side: str) -> list[ET.Element]:
    source = _vendor_root(side)
    prefix = f"{side}_"          # links
    joint_prefix = f"{side}_hand_"  # joints: Core reads "hand" for hand styling
    elements: list[ET.Element] = []
    for link in source.findall("link"):
        # Kinematics only: the renderer draws joint connectors, not meshes.
        elements.append(ET.Element("link", {"name": prefix + link.get("name")}))
    for joint in source.findall("joint"):
        name = joint.get("name")
        origin = joint.find("origin")
        xyz = [v * DISPLAY_SCALE for v in _floats(origin.get("xyz") if origin is not None else None)]
        rpy = _floats(origin.get("rpy") if origin is not None else None)
        parent = prefix + joint.find("parent").get("link")
        steps = _single_axis_origins(xyz, rpy)
        for step, (step_xyz, step_rpy) in enumerate(steps[:-1]):
            helper = f"{prefix}{name}_rpy{step}"
            elements.append(ET.Element("link", {"name": helper}))
            fixed = ET.Element("joint", {"name": f"{joint_prefix}{name}_rpy{step}_fixed", "type": "fixed"})
            ET.SubElement(fixed, "origin", {"xyz": _fmt(step_xyz), "rpy": _fmt(step_rpy)})
            ET.SubElement(fixed, "parent", {"link": parent})
            ET.SubElement(fixed, "child", {"link": helper})
            elements.append(fixed)
            parent = helper
        step_xyz, step_rpy = steps[-1]
        copy = ET.Element("joint", {"name": joint_prefix + name, "type": joint.get("type")})
        ET.SubElement(copy, "origin", {"xyz": _fmt(step_xyz), "rpy": _fmt(step_rpy)})
        ET.SubElement(copy, "parent", {"link": parent})
        ET.SubElement(copy, "child", {"link": prefix + joint.find("child").get("link")})
        for tag in ("axis", "limit"):
            if joint.find(tag) is not None:
                ET.SubElement(copy, tag, dict(joint.find(tag).attrib))
        mimic = joint.find("mimic")
        if mimic is not None:
            ET.SubElement(copy, "mimic", {**mimic.attrib, "joint": joint_prefix + mimic.get("joint")})
        elements.append(copy)
    for finger, (distal, xyz) in FINGERTIPS.items():
        tip = f"{prefix}{finger}_tip"
        elements.append(ET.Element("link", {"name": tip}))
        fixed = ET.Element("joint", {"name": f"{joint_prefix}{finger}_tip_fixed", "type": "fixed"})
        ET.SubElement(fixed, "origin", {"xyz": _fmt(v * DISPLAY_SCALE for v in xyz), "rpy": "0 0 0"})
        ET.SubElement(fixed, "parent", {"link": prefix + distal})
        ET.SubElement(fixed, "child", {"link": tip})
        elements.append(fixed)
    return elements


def _root_link(source: ET.Element) -> str:
    children = {j.find("child").get("link") for j in source.findall("joint")}
    roots = [l.get("name") for l in source.findall("link") if l.get("name") not in children]
    if len(roots) != 1:
        raise ValueError(f"expected one root link, got {roots}")
    return roots[0]


@lru_cache(maxsize=None)
def dual_urdf() -> str:
    robot = ET.Element("robot", {"name": "linkerhand_l20_dual_skeleton"})
    ET.SubElement(robot, "link", {"name": "dual_hand_base"})
    for side in ("left", "right"):
        robot.extend(_side_elements(side))
        anchor = ET.SubElement(robot, "joint", {"name": f"{side}_anchor", "type": "fixed"})
        ET.SubElement(anchor, "origin", {"xyz": _fmt([ANCHOR_X[side], 0, 0]), "rpy": _fmt([0, 0, ANCHOR_YAW[side]])})
        ET.SubElement(anchor, "parent", {"link": "dual_hand_base"})
        ET.SubElement(anchor, "child", {"link": f"{side}_{_root_link(_vendor_root(side))}"})
    return ET.tostring(robot, encoding="unicode")


# A skeleton panel's topic carries the hand side it belongs to, e.g.
# /nvidia_desktop/hand/left/joints_ab12cd34_skeleton.
_SIDE_IN_TOPIC = re.compile(r"/hand/(left|right)(?:/|$)")


def side_from_topic(topic: str | None) -> str | None:
    """Hand side named by a joints topic, or None when the topic carries none."""
    match = _SIDE_IN_TOPIC.search(topic or "")
    return match.group(1) if match else None


@lru_cache(maxsize=None)
def single_urdf(side: str) -> str:
    """One hand only, centered where Core's fixed camera looks.

    Joint names keep the `{side}_` prefix, so skeleton frames (which use the
    same prefixed names) still address them.
    """
    robot = ET.Element("robot", {"name": f"linkerhand_l20_{side}_skeleton"})
    ET.SubElement(robot, "link", {"name": f"{side}_base"})
    robot.extend(_side_elements(side))
    anchor = ET.SubElement(robot, "joint", {"name": f"{side}_anchor", "type": "fixed"})
    ET.SubElement(anchor, "origin", {"xyz": _fmt([0.0, 0.0, 0.0]), "rpy": _fmt([0.0, 0.0, ANCHOR_YAW[side]])})
    ET.SubElement(anchor, "parent", {"link": f"{side}_base"})
    ET.SubElement(anchor, "child", {"link": f"{side}_{_root_link(_vendor_root(side))}"})
    return ET.tostring(robot, encoding="unicode")


def model(topic: str | None = None) -> dict:
    """Single-hand URDF for the topic's side; dual-hand fallback without one."""
    side = side_from_topic(topic)
    return {"urdf": single_urdf(side) if side else dual_urdf()}
