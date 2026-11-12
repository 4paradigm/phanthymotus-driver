"""Official V4 CR3 chain for the controller-identified CR3A (type 113)."""

from pathlib import Path
from functools import lru_cache
import copy
import math
import xml.etree.ElementTree as ET


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
MODEL_COMMIT = "ec201c40d5db63185a3593ec9c39f80fe25aa527"
# Center planes of the straight portions in the official J2/J3 CAD meshes.
# The actual rods are offset from the joint origins: drawing only origin-to-
# origin translations loses their out-and-back connectors. See resource/README.
LINK2_CENTER_Z = 0.1288
LINK3_CENTER_Z = 0.0123


@lru_cache(maxsize=1)
def skeleton_urdf():
    """Show the official offsets as rigid segments without adding movable axes.

    The platform draws each fixed/revolute origin as a straight segment and
    applies multi-axis RPY using THREE's XYZ Euler order. Factor the original
    URDF Rz*Ry*Rx into single-axis fixed frames so both interpretations agree.
    Route the two long rods through their actual offset center planes, adding
    two rigid corners per rod. Frame rotations cancel at each route's end:
    every original physical link transform and six-axis joint contract remains.
    """
    source = ET.parse(Path(__file__).with_name("resource") / "cr3_robot.urdf").getroot()
    root = ET.Element("robot", name="cr3a_rigid_skeleton")
    root.append(ET.Comment("Display-only rigid decomposition of the official CR3 V4 model; six actuated axes."))
    for link in source.findall("link"):
        ET.SubElement(root, "link", name=link.get("name"))

    def fixed(parent, name, xyz, rpy=(0, 0, 0)):
        link = name + "_link"
        ET.SubElement(root, "link", name=link)
        joint = ET.SubElement(root, "joint", name=name, type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=link)
        ET.SubElement(joint, "origin", xyz=" ".join(map(str, xyz)), rpy=" ".join(map(str, rpy)))
        return link

    def rigid_rod(parent, prefix, xyz, center_z, turns):
        x, y, z = xyz
        if y != 0:
            raise ValueError("unexpected lateral rod offset; review CAD centerline")
        points = ((0, 0, center_z), (x, 0, center_z), (x, 0, z))
        previous = (0, 0, 0)
        theta = 0.0
        for label, point, turn in zip(("corner1", "corner2", "end"), points, turns):
            dx, dy, dz = (point[i] - previous[i] for i in range(3))
            # Express the next segment in the preceding rigid frame.
            local = (math.cos(theta) * dx - math.sin(theta) * dz,
                     dy, math.sin(theta) * dx + math.cos(theta) * dz)
            parent = fixed(parent, prefix + "_" + label, local, (0, turn, 0))
            theta += turn
            previous = point
        return parent

    for original in source.findall("joint"):
        if original.get("type") == "fixed":
            root.append(copy.deepcopy(original))
            continue
        name = original.get("name")
        parent = original.find("parent").get("link")
        origin = original.find("origin")
        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        roll, pitch, yaw = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        if name == "joint3":
            parent = rigid_rod(parent, "upper_arm_rigid", xyz, LINK2_CENTER_Z,
                               (math.pi / 2, math.pi / 2, -math.pi))
        elif name == "joint4":
            parent = rigid_rod(parent, "forearm_rigid", xyz, LINK3_CENTER_Z,
                               (math.pi / 2, -math.pi / 2, 0))
        elif any(xyz):
            parent = fixed(parent, name + "_offset", xyz)
        for label, rotation in (("yaw", (0, 0, yaw)), ("pitch", (0, pitch, 0)), ("roll", (roll, 0, 0))):
            if any(rotation):
                parent = fixed(parent, name + "_" + label, (0, 0, 0), rotation)
        joint = copy.deepcopy(original)
        joint.find("parent").set("link", parent)
        joint.find("origin").set("xyz", "0 0 0")
        joint.find("origin").set("rpy", "0 0 0")
        root.append(joint)
    return ET.tostring(root, encoding="unicode")


def resolve_model(snapshot, configured_name=""):
    code = snapshot.get("robot_type_code")
    configured = str(configured_name).strip().upper()
    if code == 113:
        if configured and configured != "CR3A":
            raise ValueError("configured model disagrees with feedback type 113 (CR3A)")
        return {"model_name": "CR3A", "model_source": "feedback_type_113"}
    if code is not None:
        raise ValueError(f"no verified skeleton model bundled for robot type {code}")
    if configured == "CR3A":
        return {"model_name": "CR3A", "model_source": "operator_config"}
    raise ValueError("CR3A identification requires feedback or explicit model_name=CR3A")


def model_resource(snapshot, configured_name=""):
    try:
        identity = resolve_model(snapshot, configured_name)
    except ValueError as exc:
        # The platform falls back to a G1 humanoid when the model call errors
        # or omits URDF. An empty, explicitly unavailable scene is preferable to
        # presenting a different robot as the connected arm.
        return {
            "available": False, "model_error": str(exc), "joint_names": [],
            "urdf": '<robot name="dobot_model_unavailable"><link name="base_link"/></robot>',
        }
    return {
        **identity, "available": True,
        "urdf": skeleton_urdf(),
        "joint_names": list(JOINT_NAMES),
        "model_commit": MODEL_COMMIT,
        "model_repository": "https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4",
        "purpose": "visualization_only",
        "actuated_joint_count": 6,
    }
