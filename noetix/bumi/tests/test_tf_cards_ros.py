"""Local ROS 2 integration: actual state publisher, DDS streams and a tf2 listener. No hardware."""
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("ament_index_python.packages")
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.qos import qos_profile_sensor_data

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))
from tf_cards import TfTreePlugin, VisionTfPlugin
from vision_tf import CAMERA_ID, OPTICAL


def mat(xyz, angles):
    t = np.eye(4)
    r,p,y = angles
    cr,cp,cy = np.cos((r,p,y))
    sr,sp,sy = np.sin((r,p,y))
    t[:3,:3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    t[:3,3] = xyz
    return t


def test_real_tf2_hand_to_marker(monkeypatch):
    try:
        get_package_prefix("robot_state_publisher")
    except PackageNotFoundError:
        pytest.skip("robot_state_publisher not installed")
    # Scope discovery and the child publisher to this loopback test graph.
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", str(125 + os.getpid() % 80))
    ns = "bumi_tf_test"
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = rclpy.create_node("tf_card_test")
    executor.add_node(node)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    cfg = {"camera_mount":{"parent":"waist_yaw_link","xyz_m":[.04,.02,.20],"rpy_rad":[0,0,0]},
           "hand_frames":[{"parent":"l_elbow_pitch_link","name":"l_hand","xyz_m":[0,0,-.10],"rpy_rad":[0,0,0]}]}
    cv = VisionTfPlugin({}, ns, executor)
    tree = TfTreePlugin(cfg, ns, executor)
    bare = TfTreePlugin({}, ns, executor)
    assert cv.get_tool()["name"] == VisionTfPlugin.PREFIX == "vision_tf"
    assert tree.get_tool()["name"] == TfTreePlugin.PREFIX == "tf_tree"
    wire = {"instance_id":"left","input_topic":"/weekly/aruco"}
    camera_pub = node.create_publisher(String,wire["input_topic"],qos_profile_sensor_data)
    joints_pub = node.create_publisher(String,"/"+ns+"/state/joints",qos_profile_sensor_data)
    urdf = ET.parse(DRIVER/"resource/bumi_model.urdf")
    names = [j.get("name") for j in urdf.findall("joint")]
    joints = {name:0.0 for name in names}
    outcomes = []
    try:
        rhs = cv.dispatch("info",wire)["topic_out"][0]["topic"]
        assert rhs == cv.dispatch("start",wire)["topic_out"][0]["topic"]
        with pytest.raises(ValueError,match="measured"):
            bare.dispatch("start",{"input_topic":rhs})
        assert tree.dispatch("start",{"input_topic":rhs})["topic_out"] == []
        target = ns + "/" + OPTICAL + "/objects/aruco_DICT_4X4_50_7"
        for angle in (0.0, .5):
            joints["l_elbow_pitch_joint"] = angle
            # Separate FK from the actual bundled URDF for each arm link at q=0.
            arm = np.eye(4)
            for key in ("l_arm_pitch_joint","l_arm_roll_joint","l_arm_yaw_joint","l_elbow_pitch_joint"):
                j = urdf.find("joint[@name='%s']" % key)
                o = j.find("origin")
                arm = arm @ mat(list(map(float,o.get("xyz").split())),list(map(float,o.get("rpy").split())))
                if key == "l_elbow_pitch_joint":
                    assert j.find("axis").get("xyz") == "0 1 0"
                    arm = arm @ mat([0,0,0],[0,angle,0])
            arm = arm @ mat([0,0,-.1],[0,0,0])
            optical = mat([.04,.02,.2],[-math.pi/2,0,-math.pi/2])
            offset = mat([.1,.03,.45],[0,0,0])
            expected = np.linalg.inv(arm) @ optical @ offset
            end = time.monotonic()+12
            got = None
            while time.monotonic() < end:
                stamp = node.get_clock().now().to_msg()
                when = {"sec":stamp.sec,"nanosec":stamp.nanosec}
                joints_pub.publish(String(data=json.dumps({"stamp":when,"joints":[{"name":k,"q":v} for k,v in joints.items()]})))
                obs = {"camera_id":CAMERA_ID,"frame_id":"","dictionary":"DICT_4X4_50","stamp":when,
                       "markers":[{"id":7,"translation_m":[.1,.03,.45],"rotation_rvec_rad":[0,0,0]},
                                  {"id":23,"translation_m":[0,0,.6],"rotation_rvec_rad":[0,0,.2]}]}
                if angle:
                    obs.update(camera_id="third_party_rgb", frame_id=OPTICAL)
                camera_pub.publish(String(data=json.dumps(obs)))
                for _ in range(12):
                    executor.spin_once(timeout_sec=.015)
                try:
                    tr = buffer.lookup_transform(ns+"/l_hand", target, Time()).transform
                    buffer.lookup_transform(ns+"/base_link",target[:-1]+"23",Time())
                except TransformException:
                    continue
                got = np.array([tr.translation.x,tr.translation.y,tr.translation.z])
                if np.allclose(got,expected[:3,3],atol=1e-6):
                    break
            assert got is not None and np.allclose(got,expected[:3,3],atol=1e-6), (got,expected[:3,3])
            q = tr.rotation
            cross = np.array([[0,-q.z,q.y],[q.z,0,-q.x],[-q.y,q.x,0]])
            np.testing.assert_allclose(np.eye(3)+2*q.w*cross+2*cross@cross,expected[:3,:3],atol=1e-6)
            outcomes.append(got.tolist())
        assert np.linalg.norm(np.subtract(*outcomes)) > .03
        print("tf2 hand-to-marker metres for elbow 0, 0.5 rad:", json.dumps(outcomes))
    finally:
        bare.stop()
        tree.stop()
        cv.stop()
        listener.unregister()
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
