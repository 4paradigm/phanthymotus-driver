"""Verify preview deployment with ROS2 topics and the real MCP HTTP transport."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
from std_msgs.msg import String

from agilex.piper.vision_rgbd import ObservationSource


ROOT = Path(__file__).resolve().parents[1]
TOOL = "piper_vision_pick_and_drop"


def rpc(port, method, **params):
    req = urllib.request.Request(f"http://127.0.0.1:{port}/mcp", data=json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": method, "params": params
    }).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=2) as response:
        return json.load(response)


def call(port, action, **args):
    response = rpc(port, "tools/call", name=TOOL, arguments={"action": action, **args})
    if "error" in response:
        raise RuntimeError(response["error"]["message"])
    return json.loads(response["result"]["content"][0]["text"])


def test_wrist_observation_requires_close_stationary_joint_samples():
    angles = np.zeros(6)
    moved = np.array([.2, 0, 0, 0, 0, 0])
    assert ObservationSource.stationary_angles([(10, angles), (50, angles)], 20, 40) == pytest.approx(angles)
    assert ObservationSource.stationary_angles([(10, angles)], 20, 40) is None
    assert ObservationSource.stationary_angles([(10, angles), (200_000_000, angles)], 20, 40) is None
    assert ObservationSource.stationary_angles([(10, angles), (30, moved), (50, angles)], 20, 40) is None


@pytest.mark.parametrize("wrist", [False, True], ids=["fixed", "wrist"])
def test_preview_deployment_receives_rgbd_and_vop(tmp_path, monkeypatch, wrist):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    config = tmp_path / "preview.yaml"
    vision = {"enabled": True, "camera_topic_base": "/test_camera"}
    if wrist:
        model_path = ROOT / "agilex/piper/resource/upstream/piper_description.urdf"
        vision.update(joint_state_topic="/test_joints", calibration={
            "id": "test-wrist", "arm_can_interface": "can0", "optical_frame_id": "test_camera_optical",
            "reference_frame": "link6", "rotation": np.eye(3).tolist(), "translation_mm": [0, 0, 0],
            "arm_urdf_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        })
    config.write_text(yaml.safe_dump({"mcp_port": port, "ros": {"robot_domain_id": 207, "core_domain_id": 208},
                                      "vision_pick": vision, "data_dir": str(tmp_path)}))
    registration = []

    class Receiver(BaseHTTPRequestHandler):
        def do_POST(self):
            registration.append((self.path, json.loads(self.rfile.read(int(self.headers["Content-Length"])))))
            self.send_response(200)
            self.end_headers()

    core_http = ThreadingHTTPServer(("127.0.0.1", 0), Receiver)
    http_thread = threading.Thread(target=core_http.serve_forever, daemon=True)
    http_thread.start()
    camera, core = Context(), Context()
    rclpy.init(context=camera, domain_id=207)
    rclpy.init(context=core, domain_id=208)
    station = rclpy.create_node("synthetic_station", context=camera)
    detector = rclpy.create_node("synthetic_vop", context=core)
    detector_executor = SingleThreadedExecutor(context=core)
    detector_executor.add_node(detector)
    qos = qos_profile_sensor_data
    rgb_out = station.create_publisher(CompressedImage, "/test_camera/color/image_raw/compressed", qos)
    depth_out = station.create_publisher(Image, "/test_camera/aligned_depth_to_color/image_raw", qos)
    info_out = station.create_publisher(CameraInfo, "/test_camera/aligned_depth_to_color/camera_info", qos)
    joints_out = station.create_publisher(JointState, "/test_joints", qos)

    def publish_joints(stamp):
        state = JointState(name=[f"joint{i}" for i in range(1, 7)], position=[0.] * 6)
        state.header.stamp = stamp
        joints_out.publish(state)

    preview_rgb = "/isolated_piper_test/piper/vision_pick/rgb"
    objects_out = detector.create_publisher(String, preview_rgb + "/objects", qos)
    detector.create_subscription(CompressedImage, preview_rgb, lambda _: objects_out.publish(String(data=json.dumps(
        {"timestamp": time.time(), "latency_ms": 0, "objects": [{"name": "orange", "position": [0, 0]}]}
    ))), qos)

    output = tmp_path / "preview.log"
    env = {**os.environ, "ROS_NAMESPACE": "isolated_piper_test", "CONFIG_PATH": str(config),
           "AGENT_CORE_URL": f"http://127.0.0.1:{core_http.server_port}"}
    with output.open("wb") as log:
        child = subprocess.Popen([sys.executable, "-m", "agilex.piper.main"], cwd=ROOT,
                                 env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            _, jpg = cv2.imencode(".jpg", np.zeros((12, 16, 3), dtype=np.uint8))
            depth = np.full((12, 16), 600, dtype="<u2").tobytes()
            deadline = time.monotonic() + 18
            while time.monotonic() < deadline and child.poll() is None:
                now = station.get_clock().now()
                if wrist:
                    publish_joints((now - Duration(nanoseconds=20_000_000)).to_msg())
                rgb = CompressedImage(format="jpeg", data=jpg.tobytes())
                image = Image(width=16, height=12, encoding="16UC1", step=32, data=depth)
                info = CameraInfo(width=16, height=12, k=[100., 0., 8., 0., 100., 6., 0., 0., 1.])
                for msg in (rgb, image, info):
                    msg.header.stamp, msg.header.frame_id = now.to_msg(), "test_camera_optical"
                rgb_out.publish(rgb)
                depth_out.publish(image)
                info_out.publish(info)
                if wrist:
                    time.sleep(.02)
                    publish_joints(station.get_clock().now().to_msg())
                detector_executor.spin_once(timeout_sec=.1)
                try:
                    observation = call(port, "observe")
                    if observation["objects"] and registration:
                        break
                except (urllib.error.URLError, RuntimeError):
                    time.sleep(.1)
            else:
                pytest.fail(output.read_text(encoding="utf-8"))
            assert registration[0][0] == "/api/mcp"
            assert registration[0][1]["id"] == "agilex-piper-driver"
            assert registration[0][1]["url"] == f"http://localhost:{port}/mcp"
            assert observation["objects"] == [{"name": "orange", "position": [0, 0]}]
            result = call(port, "plan_pick", observation_id=observation["observation_id"], pick_x=0, pick_y=0)
            assert result["camera_xyz_mm"] == pytest.approx([0, 0, 600], abs=.001)
            if wrist:
                assert result["base_xyz_mm"] == pytest.approx([653.85, 0, 265.47], abs=.2)
            else:
                assert result["base_xyz_mm"] is None
            tool = next(tool for tool in rpc(port, "tools/list")["result"]["tools"] if tool["name"] == TOOL)
            assert "grab_to" not in tool["inputSchema"]["properties"]["action"]["enum"]
        finally:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait(timeout=10)
            detector_executor.shutdown()
            station.destroy_node()
            detector.destroy_node()
            camera.try_shutdown()
            core.try_shutdown()
            core_http.shutdown()
            core_http.server_close()
