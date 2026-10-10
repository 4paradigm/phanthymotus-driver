"""PiPER cards: guarded J1 actions, on-demand feedback and D435 RGB snapshots.

No connection, enable, motion or camera capture happens at construction/start.
Each physical operation owns one gate shared by the arm and mounted camera.
"""

from contextlib import contextmanager
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from agilex.piper.arm import PiperDriver


def number(args, key, default, low, high, integer=False):
    value = args.get(key, default)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high
            or (integer and not isinstance(value, int))):
        raise ValueError(f"{key} must be {'an integer' if integer else 'a number'} in [{low}, {high}]")
    return value


def confirm(args, key):
    if args.get(key) is not True:
        raise ValueError(f"{key}=true requires the operator's current physical confirmation")


class PiperPlugin:
    def __init__(self, config, namespace="piper", ros2=None, driver_factory=PiperDriver):
        self.config = config
        self.factory = driver_factory
        self.driver = None
        self.ros2 = ros2
        self.topic = f"/{namespace}/piper/rgb"
        self.data_dir = Path(config.get("data_dir", "/opt/phanthy-motus/data/piper"))
        self.gate = threading.Lock()
        self.lifecycle = threading.Lock()
        self.cancel = threading.Event()
        self.closed = False
        self.node = None
        self.publisher = None
        self.last_image = None
        self.motion_enabled = os.environ.get("PIPER_MOTION_ENABLED", "0") == "1"

    def get_tools(self):
        actions = {
            "start": ([], "Ready only; never enables motors"),
            "stop": ([], "Cancel current operation; best-effort hold, never release torque"),
            "info": ([], "Driver configuration and busy state"),
            "prepare": (["execute", "workspace_clear", "allow_limit_adjustment", "speed_pct"],
                        "Preview first; explicit execution enables at measured pose"),
            "move_j1": (["target_deg", "speed_pct", "workspace_clear", "take_control"],
                        "Absolute J1 target, at most 12 degrees from current angle; then hold"),
            "disable": (["arm_supported"], "Release all six motors only with physical support"),
            "save_default": (["confirm_save"], "Save current feedback as local parking pose; no motion"),
            "return_default": (["execute", "speed_pct", "workspace_clear", "take_control"],
                               "Preview/return J1 only if J2-J6 already match saved pose within 1 degree"),
        }
        flags = {key: {"type": "boolean", "default": False} for key in (
            "execute", "workspace_clear", "allow_limit_adjustment", "take_control", "arm_supported", "confirm_save")}
        schema = action_schema(actions, {**flags,
            "target_deg": {"type": "number", "minimum": -140, "maximum": 140},
            "speed_pct": {"type": "integer", "minimum": 1, "maximum": 10, "default": 5},
        })
        schema["additionalProperties"] = False
        schema["x-resource"] = "arm"
        schema["x-is-dangerous"] = True
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"}}
        camera_schema = action_schema({
            "start": ([], "Ready; no capture or arm motion"),
            "stop": ([], "Cancel in-flight operation"),
            "info": ([], "Camera output topic"),
            "capture": (["forward_view_confirmed"],
                        "Capture current RGB view without moving the arm; confirm forward view for this shot only"),
        }, {"forward_view_confirmed": {"type": "boolean", "default": False,
             "description": "Operator has verified current arm/camera orientation, not a persistent calibration"}})
        camera_schema["additionalProperties"] = False
        # Camera is mounted on the wrist: prevent capture during an arm move.
        camera_schema["x-resource"] = ["arm", "camera"]
        return [
            tool("piper_status", "resource", "Fresh PiPER feedback on demand; no motion commands"),
            tool("piper_arm", "actuator", "Bounded PiPER single-arm control; motion disabled by default", schema),
            tool("piper_photo", "actuator", "D435 RGB snapshot at current pose; no automatic head raising",
                 camera_schema, topic_out=[{"topic": self.topic, "format": "image/jpeg"}]),
        ]

    def start(self):
        with self.lifecycle:
            if self.gate.locked():
                raise RuntimeError("An operation is still stopping; retry start after it completes")
            self.closed = False

    def request_stop(self):
        with self.lifecycle:
            self.closed = True
            self.cancel.set()
            if self.driver is not None:
                self.driver.cancel_motion()
        return {"state": "stopping" if self.gate.locked() else "idle",
                "motor_torque_released": False,
                "note": "Best-effort hold only; not an emergency stop. CAN loss can prevent holding."}

    def stop(self):
        self.request_stop()
        # Do not close the bus until the action's error/hold path is finished.
        if self.gate.acquire(timeout=2):
            try:
                if self.driver is not None:
                    self.driver.close()
                    self.driver = None
                if self.node is not None:
                    self.ros2.executor_core.remove_node(self.node)
                    self.node.destroy_node()
                    self.node = self.publisher = self.last_image = None
            finally:
                self.gate.release()

    @contextmanager
    def operation(self):
        with self.lifecycle:
            if self.closed:
                raise RuntimeError("Driver stopped; call start before another operation")
            if not self.gate.acquire(blocking=False):
                raise RuntimeError("Arm/camera busy; concurrent operations are refused")
            self.cancel.clear()
            if self.driver is not None:
                self.driver.cancelled.clear()
        try:
            yield
        finally:
            self.gate.release()

    def connected(self):
        if self.driver is None:
            candidate = self.factory(self.config.get("can_interface", "can0"))
            candidate.connect()
            with self.lifecycle:
                if self.cancel.is_set():
                    candidate.close()
                    raise RuntimeError("Connection cancelled")
                self.driver = candidate
        return self.driver

    def motion_allowed(self, args):
        if not self.motion_enabled:
            raise RuntimeError("Set PIPER_MOTION_ENABLED=1 on the host after commissioning")
        confirm(args, "workspace_clear")

    def dispatch(self, action, args):
        name = args.get("_tool_name", "piper_arm")
        if name not in {"piper_status", "piper_arm", "piper_photo"}:
            return None
        if name == "piper_status":
            with self.operation():
                return asdict(self.connected().get_state())
        if action == "stop":
            return self.request_stop()
        if action == "start":
            self.start()
            return {"state": "ready"}
        if action == "info":
            return {"state": "idle" if self.closed else "ready", "busy": self.gate.locked(),
                    "motion_enabled": self.motion_enabled, "max_j1_step_deg": 12,
                    "topic_out": [{"topic": self.topic, "format": "image/jpeg"}] if name == "piper_photo" else []}
        if name == "piper_photo":
            if action != "capture":
                raise ValueError(f"Unknown photo action: {action}")
            with self.operation():
                return self.capture(args)
        if action not in {"prepare", "move_j1", "disable", "save_default", "return_default"}:
            raise ValueError(f"Unknown arm action: {action}")
        # Reject strings like 'false' before any hardware access.
        for key in ("execute", "workspace_clear", "allow_limit_adjustment", "take_control", "arm_supported", "confirm_save"):
            if key in args and type(args[key]) is not bool:
                raise ValueError(f"{key} must be a boolean")
        speed = number(args, "speed_pct", 5, 1, 10, integer=True)
        execute = args.get("execute", False)
        if action == "move_j1" or (action in {"prepare", "return_default"} and execute):
            self.motion_allowed(args)
        if action in {"move_j1", "return_default"} and (action == "move_j1" or execute):
            confirm(args, "take_control")
        if action == "disable":
            confirm(args, "arm_supported")
        if action == "save_default":
            confirm(args, "confirm_save")
        with self.operation():
            driver = self.connected()
            if action == "prepare":
                return driver.prepare_control(execute, args.get("workspace_clear", False),
                                              args.get("allow_limit_adjustment", False), speed)
            if action == "disable":
                return asdict(driver.disable(supported=True))
            if action == "move_j1":
                target = number(args, "target_deg", None, -140, 140)
                return asdict(driver.move_j1_to(target, speed, take_control=True))
            if action == "save_default":
                state = driver.get_state()
                pose = {"joints_deg": state.joints_deg, "saved_at": datetime.now(timezone.utc).isoformat(),
                        "kind": "operator_parking_pose", "factory_zero": False}
                self.data_dir.mkdir(parents=True, exist_ok=True)
                temp = self.data_dir / "default_pose.json.tmp"
                temp.write_text(json.dumps(pose, indent=2) + "\n", encoding="utf-8")
                temp.replace(self.data_dir / "default_pose.json")
                return pose
            pose = json.loads((self.data_dir / "default_pose.json").read_text(encoding="utf-8"))
            target = pose["joints_deg"]
            if (len(target) != 6 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                       or not math.isfinite(v) for v in target)):
                raise ValueError("Invalid saved pose")
            state = driver.get_state()
            if any(abs(a-b) > 1 for a, b in zip(state.joints_deg[1:], target[1:])):
                raise RuntimeError("J2-J6 differ from default; this driver cannot plan a six-joint return")
            delta = target[0] - state.joints_deg[0]
            if abs(delta) > 12 or not -140 <= target[0] <= 140:
                raise RuntimeError("Default J1 outside the bounded return range")
            if not execute:
                return {"executed": False, "j1_target_deg": target[0], "j1_delta_deg": delta,
                        "other_joints": "Already within 1 degree; held at current angles"}
            if abs(delta) < 0.1:
                return {"executed": False, "already_at_default": True, "state": asdict(state)}
            return {"executed": True, "state": asdict(driver.move_j1_to(target[0], speed, True))}

    def capture(self, args):
        forward = args.get("forward_view_confirmed", False)
        if type(forward) is not bool:
            raise ValueError("forward_view_confirmed must be a boolean")
        cfg = self.config.get("camera", {})
        warmup = number(cfg, "warmup_seconds", 3, 0.1, 10)
        contrast = number(cfg, "min_contrast", 8, 0, 255)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        output = self.data_dir / f"rgb_{uuid4().hex}.jpg"
        cmd = [sys.executable, str(Path(__file__).with_name("camera_capture.py")),
               "--output", str(output), "--warmup", str(warmup), "--min-contrast", str(contrast)]
        if cfg.get("device"):
            cmd.extend(["--device", str(cfg["device"])])
        # Isolate V4L2's blocking read. Stop/timeout terminate the child and
        # release its device handles even when the USB camera has stalled.
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        deadline = time.monotonic() + warmup + 8
        try:
            while True:
                if self.cancel.is_set():
                    raise RuntimeError("Capture cancelled")
                if time.monotonic() > deadline:
                    raise RuntimeError("Camera capture timed out")
                try:
                    stdout, stderr = proc.communicate(timeout=0.1)
                    break
                except subprocess.TimeoutExpired:
                    continue
            if proc.returncode:
                raise RuntimeError(f"Camera capture failed: {stderr[-1000:]}")
            info = json.loads(stdout)
            info.pop("device", None)  # Do not publish the USB serial in card output.
            info.update(captured_at=datetime.now(timezone.utc).isoformat(),
                        forward_view_confirmed=forward, arm_moved=False)
            if self.cancel.is_set():
                raise RuntimeError("Capture cancelled")
            self.publish_photo(output)
            output.with_suffix(".json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
            return {**info, "topic_out": [{"topic": self.topic, "format": "image/jpeg"}]}
        except BaseException:
            output.unlink(missing_ok=True)
            raise
        finally:
            if proc.poll() is None:
                proc.kill()
            proc.communicate()

    def publish_photo(self, output):
        if self.ros2 is None:
            return  # Offline contract tests; production runtime always supplies ROS.
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage

        if self.node is None:
            self.node = Node("piper_photo", context=self.ros2.ctx_core)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.node.create_publisher(CompressedImage, self.topic, qos)
            # Replay only the most recent still image for late dashboard subscribers.
            self.node.create_timer(1.0, self.republish_photo)
            self.ros2.executor_core.add_node(self.node)
        msg = CompressedImage()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "piper_camera_color_optical_frame"
        msg.format = "jpeg"
        msg.data = output.read_bytes()
        self.last_image = msg
        self.publisher.publish(msg)

    def republish_photo(self):
        if not self.closed and self.last_image is not None:
            self.publisher.publish(self.last_image)


def build_plugins(config, namespace, ros2):
    return [PiperPlugin(config, namespace, ros2)]
