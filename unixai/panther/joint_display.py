"""Render feedback with the bundled articulated model, independently of control limits."""

import xml.etree.ElementTree as ET

from contracts import json_float


def _add_base_visuals(root, lift, joints):
    """Replace the wheel and lidar skeleton branches with a simple chassis."""
    base_name = lift.find("parent").get("link")
    base = root.find(f"link[@name='{base_name}']")
    # The bundled model's only other branches from the base are wheels and lidars.
    def prune_children(parent):
        for joint in root.findall("joint"):
            if joint is lift or joint.find("parent").get("link") != parent:
                continue
            child = joint.find("child").get("link")
            prune_children(child)
            root.remove(joint)
            root.remove(root.find(f"link[@name='{child}']"))

    prune_children(base_name)

    for visual in base.findall("visual"):
        base.remove(visual)
    # Keep the front edge (+X) of the original footprint. Halve the rear
    # overhang measured from the lift column, without moving the column.
    x, y, _ = lift.find("origin").get("xyz").split()
    front_x, original_rear_x = 0.325, -0.325
    rear_x = (float(x) + original_rear_x) / 2
    center_x = (front_x + rear_x) / 2
    height = 0.2
    box = ET.SubElement(base, "visual", name="display_base")
    ET.SubElement(box, "origin", xyz=f"{center_x:.9g} 0 {height / 2:g}")
    ET.SubElement(ET.SubElement(box, "geometry"), "box", size=f"{front_x - rear_x:.9g} 0.51 {height:g}")

    # Match the head top at the observed upper position (0.62 m). The rail
    # belongs to the base, while the torso moves on body_joint independently.
    top = 0.62 + sum(float(joints[name].find("origin").get("xyz").split()[2])
                     for name in ("body_joint", "joint_head0", "joint_head1", "joint_rgbd"))
    rail = ET.SubElement(base, "visual", name="display_column")
    ET.SubElement(rail, "origin", xyz=f"{x} {y} {(height + top) / 2:.9g}")
    ET.SubElement(ET.SubElement(rail, "geometry"), "cylinder", radius="0.008", length=f"{top - height:.9g}")


def render_model(urdf):
    root = ET.fromstring(urdf)
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    # The vendor model labels the axes in the opposite order to the measured
    # controller channels. Correct just the model sent to Canvas, not ROS limits.
    for name, axis in (("joint_head0", "0 0 1"), ("joint_head1", "0 1 0")):
        joint = joints[name]
        joint.find("axis").set("xyz", axis)
    # The mechanical mounts have lateral offsets that show up as a kink in
    # the stick figure. Keep both neck pivots and the RGB-D tip on the centerline.
    # The paired eye mounts already have mirrored offsets.
    for name in ("joint_head0", "joint_head1", "joint_rgbd"):
        origin = joints[name].find("origin")
        x, _, z = origin.get("xyz").split()
        origin.set("xyz", f"{x} 0 {z}")
    _add_base_visuals(root, joints["body_joint"], joints)
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
