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
from types import SimpleNamespace
import urllib.error
import urllib.request

import cv2
import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from agilex.piper.device import build_plugins
from common.vendor_runtime import DriverBundle
from test_piper_intent import IntentDriver
from test_piper_unified import fast_execute

ROOT = Path(__file__).resolve().parents[1]
TOOL = "visio_pick_and_drop"
TRANSFER = {
    "workspace_mm": [[0, 800], [-100, 100], [10, 320]], "pick_tcp_offset_mm": [0, 0, -5],
    "drop_tcp_xyz_mm": [250, 60, 30], "tool_rpy_deg": [180, 0, 90], "transit_z_mm": 300,
    "gripper_open_close_mm": [60, 5], "contact_width_mm": [15, 30],
    "min_contact_effort_nm": .2, "speed_percent": 1,
}


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


def test_canvas_card_registers_and_plans_with_wrist_rgbd(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    config = tmp_path / "preview.yaml"
    model_path = ROOT / "agilex/piper/resource/upstream/piper_description.urdf"
    vision = {
        "enabled": True, "camera_topic_base": "/test_camera",
        "calibration": {
            "id": "test-wrist", "arm_can_interface": "can0", "optical_frame_id": "test_camera_optical",
            "reference_frame": "link6", "rotation": np.eye(3).tolist(), "translation_mm": [0, 0, 0],
            "arm_urdf_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        },
    }
    config.write_text(yaml.safe_dump({
        "mcp_port": port, "ros": {"robot_domain_id": 207, "core_domain_id": 208},
        "vision_pick": vision, "data_dir": str(tmp_path),
        "intent_control": {"tcp_verified": False},
    }))
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
    preview_rgb = "/isolated_piper_test/piper/vision_pick/rgb"
    objects_out = detector.create_publisher(String, preview_rgb + "/objects", qos)
    detector.create_subscription(CompressedImage, preview_rgb, lambda _: objects_out.publish(String(data=json.dumps(
        {"timestamp": time.time(), "latency_ms": 0, "objects": [{"name": "orange", "position": [0, 0]}]}
    ))), qos)

    output = tmp_path / "preview.log"
    env = {**os.environ, "PIPER_MOTION_ENABLED": "0",
           "ROS_NAMESPACE": "isolated_piper_test", "CONFIG_PATH": str(config),
           "AGENT_CORE_URL": f"http://127.0.0.1:{core_http.server_port}"}
    launch = """
import runpy
import time
from agilex.piper.runtime import PiperRuntime

def read_recorded_joints(self):
    last = time.time_ns()
    return [0.0] * 6, last - 2_000_000, last

PiperRuntime.read_cached_joint_feedback = read_recorded_joints
runpy.run_module("agilex.piper.main", run_name="__main__")
"""
    with output.open("wb") as log:
        child = subprocess.Popen([sys.executable, "-c", launch], cwd=ROOT,
                                 env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            _, jpg = cv2.imencode(".jpg", np.zeros((12, 16, 3), dtype=np.uint8))
            depth = np.full((12, 16), 600, dtype="<u2").tobytes()
            deadline = time.monotonic() + 18
            while time.monotonic() < deadline and child.poll() is None:
                now = station.get_clock().now()
                rgb = CompressedImage(format="jpeg", data=jpg.tobytes())
                image = Image(width=16, height=12, encoding="16UC1", step=32, data=depth)
                info = CameraInfo(width=16, height=12, k=[100., 0., 8., 0., 100., 6., 0., 0., 1.])
                for msg in (rgb, image, info):
                    msg.header.stamp, msg.header.frame_id = now.to_msg(), "test_camera_optical"
                rgb_out.publish(rgb)
                depth_out.publish(image)
                info_out.publish(info)
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
            assert call(port, "info")["source"]["joint_feedback_source"] == "can"
            result = call(port, "plan_pick", observation_id=observation["observation_id"], pick_x=0, pick_y=0)
            assert result["camera_xyz_mm"] == pytest.approx([0, 0, 600], abs=.001)
            assert result["base_xyz_mm"] == pytest.approx([653.85, 0, 265.47], abs=.2)
            assert result["executed"] is False
            assert "plan_id" not in result and "calibrated" in result["transfer_setup_error"]
            tool = next(tool for tool in rpc(port, "tools/list")["result"]["tools"] if tool["name"] == TOOL)
            assert tool["title"] == TOOL and tool["type"] == "actuator"
            assert "grab_to" not in tool["inputSchema"]["properties"]["action"]["enum"]
            with pytest.raises(RuntimeError, match="not calibrated"):
                call(port, "grab_to", plan_id="unavailable", confirm_motion=True)
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


def test_canvas_transfer_checks_end_pose_and_completes_with_gripper_contact(tmp_path, monkeypatch):
    config = {
        "data_dir": str(tmp_path),
        "intent_control": {"tcp_verified": True, "tcp_offset_mm_deg": [0] * 6},
        "vision_pick": {"enabled": True, "transfer": TRANSFER, "calibration": {
            "id": "fixed", "arm_can_interface": "can0", "optical_frame_id": "optical",
            "reference_frame": "base_link", "rotation": [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
            "translation_mm": [220, 0, 550],
        }},
    }
    plugins = build_plugins(config, "test", None)
    bundle = DriverBundle(plugins)
    runtime, card = plugins[0], plugins[-1]
    runtime.factory = IntentDriver
    runtime.motion_enabled = True
    monkeypatch.setattr(runtime, "acp_complete", lambda *_: None)
    monkeypatch.setattr("agilex.piper.vision_transfer.execute_target", fast_execute)
    sdk = runtime.connected().arm
    sdk.pose = (200000, 0, 300000, 180000, 0, 90000)
    sdk.gripper_enabled = True
    send_gripper = sdk.GripperCtrl

    def grip_with_object(width, effort, enabled, zero):
        send_gripper(width, effort, enabled, zero)
        sdk.width = max(sdk.width, 20.)

    monkeypatch.setattr(sdk, "GripperCtrl", grip_with_object)
    info = SimpleNamespace(width=8, height=8, header=SimpleNamespace(frame_id="optical"),
                           k=[200., 0., 4., 0., 200., 4., 0., 0., 1.], d=[], distortion_model="plumb_bob")
    monkeypatch.setattr(card.source, "snapshot", lambda: {
        "info": info, "depth": np.full((8, 8), .5), "rgb_stamp_ns": time.time_ns(),
        "depth_stamp_ns": time.time_ns(), "objects": [],
    })

    def submit():
        observed = bundle.dispatch(TOOL, {"action": "observe"})
        plan = bundle.dispatch(TOOL, {"action": "plan_pick", "observation_id": observed["observation_id"],
                                      "pick_x": 0, "pick_y": 0})
        response = bundle.dispatch(TOOL, {"action": "grab_to", "plan_id": plan["plan_id"], "confirm_motion": True})
        assert response["state"] == "running"
        runtime.completions.wait(6)
        return bundle.dispatch(TOOL, {"action": "info"})["last_transfer"]

    try:
        sdk.pose_frozen = True
        assert "End-effector feedback is stale" in submit()["error"]
        assert not sdk.calls
        sdk.pose_frozen = False
        result = submit()
        assert result["state"] == "completed"
        assert result["grasp_feedback"]["width_mm"] == 20.
        assert result["release_confirmed"] is True
        assert [entry[1][:3] for entry in sdk.calls if entry[0] == "end_pose"] == [
            (220000, 0, 300000), (220000, 0, 45000), (220000, 0, 300000),
            (250000, 60000, 300000), (250000, 60000, 30000), (250000, 60000, 300000),
        ]
        with runtime.operation(TOOL, "grab_to"):
            stopped = bundle.dispatch("arm_control", {"action": "stop_motion"})
            assert stopped["operation_stopping"] and runtime.cancel.is_set()
    finally:
        bundle.stop_all()
