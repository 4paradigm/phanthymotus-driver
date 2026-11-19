"""Render feedback with the bundled articulated model, independently of control limits."""

import xml.etree.ElementTree as ET

from contracts import json_float


def render_model(urdf):
    root = ET.fromstring(urdf)
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    # The vendor model labels the axes in the opposite order to the measured
    # controller channels. Correct just the model sent to Canvas, not ROS limits.
    for name, axis in (("joint_head0", "0 0 1"), ("joint_head1", "0 1 0")):
        joint = joints[name]
        joint.find("axis").set("xyz", axis)
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
