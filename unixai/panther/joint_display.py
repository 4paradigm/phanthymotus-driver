"""Render feedback with the bundled articulated model, independently of control limits."""

import xml.etree.ElementTree as ET

from contracts import json_float


def _fixed_segment(root, parent, child, xyz):
    ET.SubElement(root, "link", name=child)
    joint = ET.SubElement(root, "joint", name=f"{child}_joint", type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=xyz)
    return child


def _add_base_outline(root):
    """Canvas draws fixed-joint offsets as line segments but ignores URDF meshes."""
    # SDK footprint: 650 × 510 mm. Approximate the bumper/body height in the preview.
    lower = _fixed_segment(root, "base_link", "display_base_lower_0", "-0.325 -0.255 0")
    edges = ("0.65 0 0", "0 0.51 0", "-0.65 0 0", "0 -0.51 0")
    upper = None
    for i, edge in enumerate(edges):
        corner = _fixed_segment(root, lower, f"display_base_upper_{i}", "0 0 0.2")
        if upper is None:
            upper = corner
        lower = _fixed_segment(root, lower, f"display_base_lower_{i + 1}", edge)
    for i, edge in enumerate(edges):
        upper = _fixed_segment(root, upper, f"display_base_top_edge_{i}", edge)


def _add_fixed_column(root, joints):
    """Align the fixed rail with the head in the neutral Canvas pose."""
    lift = joints["body_joint"]
    x, y, _ = lift.find("origin").get("xyz").split()
    # Canvas does not apply the lift's prismatic feedback to the drawing.
    # Use the displayed head top (RGB-D mount), not the lift travel limit.
    top = sum(float(joints[name].find("origin").get("xyz").split()[2])
              for name in ("body_joint", "joint_head0", "joint_head1", "joint_rgbd"))
    parent = lift.find("parent").get("link")
    for child, xyz in (("display_column_base", f"{x} {y} 0"),
                       ("display_column_top", f"0 0 {top:g}")):
        parent = _fixed_segment(root, parent, child, xyz)


def render_model(urdf):
    root = ET.fromstring(urdf)
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    # The vendor model labels the axes in the opposite order to the measured
    # controller channels. Correct just the model sent to Canvas, not ROS limits.
    for name, axis in (("joint_head0", "0 0 1"), ("joint_head1", "0 1 0")):
        joint = joints[name]
        joint.find("axis").set("xyz", axis)
    _add_fixed_column(root, joints)
    _add_base_outline(root)
    mimics = {}
    for name, joint in joints.items():
        mimic = joint.find("mimic")
        if mimic is not None:
            mimics[name] = (mimic.get("joint"), float(mimic.get("multiplier", "1")),
                            float(mimic.get("offset", "0")))
    return ET.tostring(root, encoding="unicode"), mimics


def joint_row(msg, index):
    if index >= len(msg.position) or (q := json_float(msg.position[index])) is None:
        return None
    row = {"q": q}
    for field, array in (("dq", msg.velocity), ("tau", msg.effort)):
        if index < len(array):
            row[field] = json_float(array[index])
    return row


def dedicated_row(msg, model_name, *, gripper=False):
    """Match by name; single-channel controller topics may have an empty/vendor name."""
    if model_name in msg.name:
        index = msg.name.index(model_name)
    elif len(msg.position) == 1:
        index = 0
    else:
        return None
    row = joint_row(msg, index)
    if gripper and row is not None and not 0.0 <= row["q"] <= 1.0:
        return None
    return row


def display_gripper_row(row):
    """The display linkage is open at q=1; the hardware reports open at 0."""
    display = {**row, "q": 1.0 - row["q"]}
    if row.get("dq") is not None:
        display["dq"] = -row["dq"]
    return display
