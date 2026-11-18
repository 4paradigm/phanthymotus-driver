import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))
from calibrate_camera_mount import fit_mount
from vision_tf import CAMERA_ID, OPTICAL, joint_positions, robot_description, validate_edges, vision_transforms

STAMP = {"sec": 32, "nanosec": 810}
CAM = "lab/" + OPTICAL


def aruco():
    return {"camera_id": CAMERA_ID, "frame_id": "", "stamp": STAMP, "coordinate_frame": "camera_optical",
            "dictionary": "DICT_4X4_50", "markers": [
                {"id": 7, "translation_m": [.1, .2, .6], "rotation_rvec_rad": [0, 0, math.pi]},
                {"id": 23, "translation_m": [0, 0, .4], "rotation_rvec_rad": [0, 0, 0]}]}


def test_aruco_axes_timestamp_orientation_and_identity():
    result = vision_transforms(aruco(), "lab")
    edges = validate_edges(result, {CAM}, {CAM, "lab/base_link"})
    assert [e["child_frame_id"] for e in edges] == [CAM+"/objects/aruco_DICT_4X4_50_"+i for i in ("7", "23")]
    assert edges[0]["header"] == {"frame_id": CAM, "stamp": STAMP}
    assert list(edges[0]["transform"]["rotation"].values()) == pytest.approx([0, 0, 1, 0])
    assert edges[0]["transform"]["translation"] == {"x": .1, "y": .2, "z": .6}
    assert vision_transforms(dict(aruco(), markers=[]), "lab")["transforms"] == []


def test_ros_and_generic_metric_sources_preserve_camera():
    pose = {"position": {"x": 1, "y": 2, "z": 3}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 2}}
    header = {"frame_id": "other/camera_optical", "stamp": STAMP}
    a = vision_transforms({"header": header, "object_id": "box", "pose": pose}, "lab")
    b = vision_transforms({"header": header, "objects": [{"id": "box", "position_m": [1,2,3],
                                                           "orientation_xyzw": [0,0,0,2]}]}, "lab")
    assert a == b
    assert a["transforms"][0]["header"] == header
    assert a["transforms"][0]["transform"]["rotation"]["w"] == 1


@pytest.mark.parametrize("bad", [
    {"stamp": None}, {"stamp": {"sec": 3, "nanosec": -1}}, {"coordinate_frame": "base_link"},
    {"markers": [{"id": 1, "translation_m": [4, 4, 2], "rotation_rvec_rad": [float("nan"), 0, 0]}]},
    {"markers": [{"id": 3, "box": [0,0,10,20]}]}, {"dictionary": "../../bad"},
    {"markers": aruco()["markers"]*2}])
def test_invalid_geometry_rejected_atomically(bad):
    with pytest.raises(ValueError):
        vision_transforms(dict(aruco(), **bad), "lab")


def test_alien_camera_and_protected_robot_frames():
    edges = vision_transforms(dict(aruco(), camera_id="someone/else", frame_id="foreign/optical"), "lab")
    with pytest.raises(ValueError):
        validate_edges(edges, {CAM}, {CAM})
    edge = vision_transforms(aruco(), "lab")["transforms"][0]
    for child in (CAM, "lab/base_link"):
        with pytest.raises(ValueError):
            validate_edges({"transforms": [dict(edge, child_frame_id=child)]}, {CAM}, {CAM, "lab/base_link"})


def test_urdf_and_all_named_joints():
    root = ET.parse(DRIVER / "resource/bumi_model.urdf").getroot()
    before = ET.tostring(root)
    raw, links, names = robot_description(root, None)
    assert len(names) == 21 and OPTICAL not in links
    xyz = [.03, -.02, .2]
    xml, links, joints = robot_description(root, {"parent":"waist_yaw_link","xyz_m":xyz,"rpy_rad":[0,0,0]},
                                           [{"parent":"l_elbow_pitch_link", "name":"l_hand", "xyz_m":[0,0,-.1], "rpy_rad":[0,0,0]}])
    assert ET.tostring(root) == before and {OPTICAL, "l_hand"} <= links and names == joints
    assert ET.fromstring(xml).find("joint[@name='bumi_camera_mount_fixed']/origin").get("xyz") == "0.03 -0.02 0.2"
    readings = [{"name": n, "q": i*.05} for i,n in enumerate(sorted(names))]
    assert joint_positions({"joints":readings}, names) == ([r["name"] for r in readings], [r["q"] for r in readings])
    with pytest.raises(ValueError):
        joint_positions({"joints":readings[:-1]}, names)
    with pytest.raises(ValueError):
        robot_description(root, {"parent":"wrong","xyz_m":xyz,"rpy_rad":[0,0,0]})


def test_fit_measured_optical_center_and_mount_tilt():
    points = np.array([[0,0,.4], [.15,0,.5], [0,.2,.6], [-.1,.1,.35], [.2,-.1,.7]])
    basis = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    yaw = .35
    zrot = np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    trans = np.array([.04,.02,.19])
    pairs = [{"optical_m":a.tolist(),"link_m":b.tolist()} for a,b in zip(points, points@(zrot@basis).T+trans)]
    result = fit_mount(pairs,"waist_yaw_link",.002)
    assert result["camera_mount"]["xyz_m"] == pytest.approx(trans)
    assert result["camera_mount"]["rpy_rad"] == pytest.approx([0,0,yaw])
    assert result["max_m"] < 1e-8
    bad = json.loads(json.dumps(pairs))
    bad[-1]["link_m"][0] += .08
    with pytest.raises(ValueError, match="residual"):
        fit_mount(bad,"waist_yaw_link",.002)
    with pytest.raises(ValueError, match="collinear"):
        fit_mount([pairs[0]]*4,"waist_yaw_link",.002)
