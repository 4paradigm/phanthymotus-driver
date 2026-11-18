"""Convert metric optical-camera poses to TransformStamped-shaped JSON and extend Bumi's URDF."""
import copy
import math
import re
import xml.etree.ElementTree as ET

CAMERA_ID = "noetix/bumi/camera_color"
MOUNT = "bumi_camera_mount"
OPTICAL = "bumi_camera_color_optical"
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)
_NAME = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]*\Z")


def frame_name(value):
    if not isinstance(value, str) or len(value) > 220 or not all(_NAME.fullmatch(p) for p in value.split("/")):
        raise ValueError("Expected a relative TF frame name with nonempty components")
    return value


def component(value):
    if isinstance(value, bool) or not isinstance(value, (int, str)) or not _NAME.fullmatch(str(value)):
        raise ValueError("Each observation needs a stable object ID without slashes")
    return str(value)


def vector(value, keys):
    if isinstance(value, dict):
        value = [value.get(k) for k in keys]
    if (not isinstance(value, (list, tuple)) or len(value) != len(keys)
            or any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in value)):
        raise ValueError("Expected finite numeric " + "/".join(keys))
    return tuple(float(v) for v in value)


def quaternion(value):
    q = vector(value, "xyzw")
    norm = math.hypot(*q)
    if norm < 1e-10 or not math.isfinite(norm):
        raise ValueError("Rotation quaternion must have nonzero finite norm")
    return tuple(v / norm for v in q)


def rodrigues(value):
    v = vector(value, "xyz")
    angle = math.hypot(*v)
    if not math.isfinite(angle):
        raise ValueError("Rotation vector magnitude is out of range")
    scale = math.sin(angle / 2) / angle if angle else 0.5
    return tuple(a * scale for a in v) + (math.cos(angle / 2),)


def stamp(value):
    if not isinstance(value, dict):
        raise ValueError("An acquisition stamp {sec, nanosec} is required")
    sec, ns = value.get("sec"), value.get("nanosec")
    if type(sec) is not int or type(ns) is not int or not 0 <= ns < 10**9 or not 0 <= sec < 2**31:
        raise ValueError("Invalid ROS acquisition stamp")
    return {"sec": sec, "nanosec": ns}


def stamped(parent, child, when, translation, rotation):
    return {"header": {"frame_id": frame_name(parent), "stamp": stamp(when)},
            "child_frame_id": frame_name(child),
            "transform": {"translation": dict(zip("xyz", vector(translation, "xyz"))),
                          "rotation": dict(zip("xyzw", quaternion(rotation)))}}


def vision_transforms(payload, namespace, source="vision", bumi_optical=OPTICAL):
    """ArUco, ROS PoseStamped + object_id, or an array of identified metric camera poses."""
    if not isinstance(payload, dict):
        raise ValueError("Expected a JSON object containing metric camera poses")
    if payload.get("fresh") is False or payload.get("pose_status", "calibrated") != "calibrated":
        return {"transforms": []}
    header = payload.get("header", payload)
    if not isinstance(header, dict):
        raise ValueError("Invalid pose header")
    when = stamp(header.get("stamp"))
    parent = header.get("frame_id")
    if payload.get("camera_id") == CAMERA_ID:
        parent = f"{namespace}/{frame_name(bumi_optical)}"
    parent = frame_name(parent)
    if payload.get("coordinate_frame", "camera_optical") != "camera_optical":
        raise ValueError("Expected camera optical coordinates (right, down, forward)")
    if "markers" in payload:
        records, prefix = payload["markers"], "aruco_" + component(payload.get("dictionary"))
    elif "objects" in payload:
        records, prefix = payload["objects"], component(source)
    elif "pose" in payload:
        records, prefix = [payload], component(source)
    else:
        raise ValueError("Expected markers, metric objects, or an identified ROS pose; pixels and range alone are insufficient")
    if not isinstance(records, list) or len(records) > 512:
        raise ValueError("Expected at most 512 observed poses")
    transforms = []
    children = set()
    for row in records:
        if not isinstance(row, dict):
            raise ValueError("Invalid pose record")
        ident = component(row.get("id", row.get("object_id", row.get("child_frame_id"))))
        child = f"{parent}/objects/{prefix}_{ident}"
        if child in children:
            raise ValueError("Duplicate object identity in a camera frame")
        children.add(child)
        pose = row.get("pose", row)
        if not isinstance(pose, dict):
            raise ValueError("Invalid object pose")
        pos = pose.get("translation_m", pose.get("position_m", pose.get("position")))
        if "rotation_rvec_rad" in pose:
            rot = rodrigues(pose["rotation_rvec_rad"])
        else:
            rot = pose.get("orientation_xyzw", pose.get("orientation"))
        transforms.append(stamped(parent, child, when, pos, rot))
    return {"transforms": transforms}


def validate_edges(batch, optical_parents, robot_frames, namespace=""):
    if not isinstance(batch, dict) or not isinstance(batch.get("transforms"), list) or len(batch["transforms"]) > 512:
        raise ValueError("Expected TFMessage JSON with at most 512 transforms")
    prefix = namespace + "/" if namespace else ""
    result, seen = [], set()
    for edge in batch["transforms"]:
        if not isinstance(edge, dict) or not isinstance(edge.get("header"), dict) or not isinstance(edge.get("transform"), dict):
            raise ValueError("Invalid TransformStamped")
        parent = frame_name(edge["header"].get("frame_id"))
        child = frame_name(edge.get("child_frame_id"))
        if prefix:
            if parent not in optical_parents and prefix + parent in optical_parents:
                parent = frame_name(prefix + parent)
            if not child.startswith(prefix):
                child = frame_name(prefix + child)
        if parent not in optical_parents or child in robot_frames or child in seen or child == parent:
            raise ValueError("Unknown camera parent, duplicate child, or collision with the robot tree")
        seen.add(child)
        pose = edge["transform"]
        result.append(stamped(parent, child, edge["header"].get("stamp"), pose.get("translation"), pose.get("rotation")))
    return result


def add_fixed(root, parent, child, xyz, rpy):
    if not any(link.get("name") == parent for link in root.findall("link")):
        raise ValueError("Unknown mount parent: " + parent)
    if any(link.get("name") == child for link in root.findall("link")):
        raise ValueError("Duplicate fixed frame: " + child)
    ET.SubElement(root, "link", name=child)
    joint = ET.SubElement(root, "joint", name=child + "_fixed", type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=" ".join(map(str, vector(xyz, "xyz"))), rpy=" ".join(map(str, vector(rpy, "rpy"))))


def robot_description(base, mount, hand_frames=()):
    root = copy.deepcopy(base)
    if mount is not None:
        add_fixed(root, frame_name(mount["parent"]), MOUNT, mount.get("xyz_m"), mount.get("rpy_rad"))
        add_fixed(root, MOUNT, OPTICAL, (0, 0, 0), OPTICAL_RPY)
    for hand in hand_frames:
        add_fixed(root, frame_name(hand["parent"]), component(hand["name"]), hand.get("xyz_m"), hand.get("rpy_rad"))
    links = {frame_name(link.get("name")) for link in root.findall("link")}
    joints = {j.get("name") for j in root.findall("joint") if j.get("type") in ("revolute", "continuous", "prismatic") and j.find("mimic") is None}
    return ET.tostring(root, encoding="unicode"), links, joints


def joint_positions(payload, known_joints):
    if not isinstance(payload, dict) or not isinstance(payload.get("joints"), list):
        raise ValueError("Expected the Bumi joints card payload")
    names, positions = [], []
    for item in payload["joints"]:
        if not isinstance(item, dict) or item.get("name") not in known_joints or item["name"] in names:
            raise ValueError("Unknown or duplicate URDF joint")
        names.append(item["name"])
        positions.append(vector([item.get("q")], "q")[0])
    if set(names) != known_joints:
        raise ValueError("Incomplete joint sample")
    return names, positions
