#!/usr/bin/env python3
"""RealMan RM75-6F-V MCP Driver using the official Python API2 SDK."""

from __future__ import annotations

import json
import math
import os
import threading
import time
from uuid import uuid4
from pathlib import Path

from common.vendor_runtime import action_schema, jsonable, tool


JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
SDK_LIBRARY_PATH = Path("/work/Robotic_Arm/libs/linux_arm/libapi_c.so")
JOINT_LIMITS_DEG = [(-178.0, 178.0), (-130.0, 130.0), (-178.0, 178.0),
                    (-135.0, 135.0), (-178.0, 178.0), (-128.0, 128.0),
                    (-360.0, 360.0)]
JOINT_MAX_SPEED_DEG_S = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]


def _sdk_result(name, result):
    if not isinstance(result, tuple) or not result:
        raise RuntimeError(f"{name} returned an invalid SDK result: {result!r}")
    code = int(result[0])
    if code != 0:
        raise RuntimeError(f"{name} failed with RealMan SDK code {code}")
    if len(result) == 2:
        return jsonable(result[1])
    return jsonable(result[1:])


class RM75SDKClient:
    """Own one SDK handle and serialize all access to the vendor library."""

    def __init__(self, config):
        self.ip = os.environ.get("RM_ARM_IP", str(config.get("arm_ip", "")).strip())
        self.port = int(os.environ.get("RM_TCP_PORT", config.get("tcp_port", 8080)))
        self.enabled = os.environ.get("RM_DRIVER_ENABLED", "0") == "1"
        self.motion_enabled = os.environ.get("RM_MOTION_ENABLED", "0") == "1"
        self._lock = threading.RLock()
        self._robot = None
        self._handle = None

    @property
    def connected(self):
        return self._handle is not None and int(getattr(self._handle, "id", -1)) >= 0

    def start(self):
        if not self.enabled:
            print("[rm75] SDK connection disabled; set RM_DRIVER_ENABLED=1 and RM_ARM_IP after safety checks", flush=True)
            return
        if not self.ip:
            raise ValueError("RM_ARM_IP is required when RM_DRIVER_ENABLED=1")
        if not SDK_LIBRARY_PATH.is_file():
            raise FileNotFoundError(
                "RealMan API2 ARM64 library is missing; mount RM_API2_LIB_DIR "
                "to /work/Robotic_Arm/libs/linux_arm"
            )
        from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

        with self._lock:
            self._robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            self._handle = self._robot.rm_create_robot_arm(self.ip, self.port)
            if not self.connected:
                bad_id = getattr(self._handle, "id", None)
                self._handle = None
                self._robot = None
                raise ConnectionError(f"RealMan SDK could not connect to {self.ip}:{self.port}; handle={bad_id}")
            print(f"[rm75] SDK connected to {self.ip}:{self.port} handle={self._handle.id}", flush=True)

    def stop(self):
        with self._lock:
            robot, self._robot = self._robot, None
            self._handle = None
            if robot is not None:
                robot.rm_delete_robot_arm()

    def status(self):
        return {
            "state": "connected" if self.connected else "disabled" if not self.enabled else "disconnected",
            "endpoint": f"{self.ip}:{self.port}" if self.ip else None,
            "read_only": not self.motion_enabled,
            "motion_enabled": self.motion_enabled,
        }

    def call(self, method):
        with self._lock:
            if not self.connected or self._robot is None:
                raise ConnectionError("RM75 SDK is not connected")
            return _sdk_result(method, getattr(self._robot, method)())

    def call_dict(self, method):
        with self._lock:
            if not self.connected or self._robot is None:
                raise ConnectionError("RM75 SDK is not connected")
            result = getattr(self._robot, method)()
            if not isinstance(result, dict) or "return_code" not in result:
                raise RuntimeError(f"{method} returned an invalid SDK result: {result!r}")
            code = int(result["return_code"])
            if code != 0:
                raise RuntimeError(f"{method} failed with RealMan SDK code {code}")
            return jsonable(result)

    def joint_states(self):
        degrees = self.call("rm_get_joint_degree")
        if not isinstance(degrees, list) or len(degrees) != 7:
            raise RuntimeError(f"rm_get_joint_degree returned {len(degrees) if isinstance(degrees, list) else 'invalid'} joints")
        radians = [math.radians(float(value)) for value in degrees]
        return {"name": JOINT_NAMES, "position": radians, "position_unit": "rad", "raw_degree": degrees}

    def command(self, method, *args):
        with self._lock:
            if not self.connected or self._robot is None:
                raise ConnectionError("RM75 SDK is not connected")
            code = int(getattr(self._robot, method)(*args))
            if code != 0:
                raise RuntimeError(f"{method} failed with RealMan SDK code {code}")
            return code


class RM75Plugin:
    PREFIX = "joint_control"

    METHODS = {
        "robot_info": "rm_get_robot_info",
        "software_info": "rm_get_arm_software_info",
        "arm_all_state": "rm_get_arm_all_state",
        "controller_state": "rm_get_controller_state",
    }

    def __init__(self, client, config, namespace="rm75", ros2=None):
        self.client = client
        self._ros2 = ros2
        self._skeleton_topic = f"/{namespace.strip('/') or 'rm75'}/state/joints"
        ros_config = config.get("ros", {})
        self._skeleton_publish_hz = float(ros_config.get("skeleton_publish_hz", 10.0))
        if not math.isfinite(self._skeleton_publish_hz) or self._skeleton_publish_hz <= 0:
            raise ValueError("ros.skeleton_publish_hz must be a positive finite number")
        self._skeleton_node = None
        self._skeleton_pub = None
        self._skeleton_message_type = None
        self._last_skeleton_error = None
        self._skeleton_retry_at = 0.0
        safety = config.get("safety", {})
        self.max_speed_percent = min(int(safety.get("max_speed_percent", 10)), 10)
        self.default_speed_percent = min(int(safety.get("default_speed_percent", 5)), self.max_speed_percent)
        self.target_tolerance_deg = float(safety.get("target_tolerance_deg", 0.5))
        self.poll_interval_seconds = float(safety.get("poll_interval_seconds", 0.2))
        self.start_grace_seconds = float(safety.get("start_grace_seconds", 2.0))
        self.stall_timeout_seconds = float(safety.get("stall_timeout_seconds", 10.0))
        self.progress_threshold_deg = float(safety.get("progress_threshold_deg", 0.05))
        self.max_motion_seconds = float(safety.get("max_motion_seconds", 300.0))
        self._motion_lock = threading.Lock()
        self._action_lock = threading.Lock()
        self._active_action_id = None
        self._cancelled = set()
        self._last_completion = None

    def _skeleton_topic_out(self):
        return [{"topic": self._skeleton_topic, "format": "sensor/skeleton"}]

    def get_tools(self):
        definitions = [
            tool("connection", "sensor", "RM75 SDK connection status; never initiates motion"),
            tool(
                "joint_states",
                "sensor",
                f"Read and publish seven RM75 joint angles in radians at {self._skeleton_publish_hz:g} Hz",
                topic_out=self._skeleton_topic_out(),
            ),
            tool("model", "resource", "RM75-6F-V simplified URDF for skeleton rendering"),
        ]
        definitions.extend(tool(name, "sensor", f"Read-only RealMan API2 call: {method}") for name, method in self.METHODS.items())
        joint_properties = {
            f"joint{i}_deg": {
                "type": "number", "minimum": low, "maximum": high,
                "description": f"[{low:g}°, {high:g}°]",
            }
            for i, (low, high) in enumerate(JOINT_LIMITS_DEG, 1)
        }
        joint_properties.update({
            "speed_percent": {"type": "integer", "minimum": 1, "maximum": self.max_speed_percent,
                              "default": self.default_speed_percent},
            "confirm_motion": {"type": "boolean", "description": "Must be true for every movement request"},
        })
        schema = action_schema(
            {
                "set": ([*(f"joint{i}_deg" for i in range(1, 8)), "speed_percent", "confirm_motion"],
                        "Send absolute joint targets in degrees; omitted joints keep their current positions"),
                "stopmotion": ([], "Request a controlled trajectory stop"),
                "info": ([], "Read motion safety and active-action status"),
            },
            joint_properties,
        )
        schema["x-completion"] = {"actions": ["set"], "timeout": 305}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stopmotion"}}
        schema["x-is-dangerous"] = True
        definitions.append(tool("joint_control", "actuator", "Bounded RM75 joint motion using official API2 movej", schema))
        return definitions

    def start(self):
        self.client.start()
        if self.client.connected and self._ros2 is not None:
            self._start_skeleton_publisher()

    def stop(self):
        with self._action_lock:
            action_id = self._active_action_id
            if action_id:
                self._cancelled.add(action_id)
        if action_id and self.client.connected:
            try:
                self.client.command("rm_set_arm_slow_stop")
            except Exception as exc:
                print(f"[rm75] shutdown stop failed: {exc}", flush=True)
        self._stop_skeleton_publisher()
        self.client.stop()

    def _start_skeleton_publisher(self):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        node = Node("rm75_skeleton", context=self._ros2.ctx_core)
        self._skeleton_message_type = String
        self._skeleton_pub = node.create_publisher(String, self._skeleton_topic, qos)
        node.create_timer(1.0 / self._skeleton_publish_hz, self._publish_skeleton)
        self._ros2.executor_core.add_node(node)
        self._skeleton_node = node

    def _stop_skeleton_publisher(self):
        node, self._skeleton_node = self._skeleton_node, None
        self._skeleton_pub = None
        self._skeleton_message_type = None
        if node is None:
            return
        try:
            self._ros2.executor_core.remove_node(node)
        finally:
            node.destroy_node()

    def _skeleton_payload(self):
        state = self.client.joint_states()
        return {
            "timestamp_ms": int(time.time() * 1000),
            "format": "sensor/skeleton",
            "position_unit": "rad",
            "joint_count": len(JOINT_NAMES),
            "joints": [
                {"idx": index, "name": name, "q": float(position)}
                for index, (name, position) in enumerate(zip(JOINT_NAMES, state["position"]))
            ],
        }

    def _publish_skeleton(self):
        publisher = self._skeleton_pub
        message_type = self._skeleton_message_type
        if publisher is None or message_type is None:
            return
        # A failed controller is sampled at most once every two seconds.
        # Visualization must not queue behind motion/stop SDK operations.
        if time.monotonic() < self._skeleton_retry_at:
            return
        if not self.client.connected:
            self._skeleton_retry_at = time.monotonic() + 2.0
            return
        if not self.client._lock.acquire(blocking=False):
            return
        try:
            message = message_type()
            message.data = json.dumps(self._skeleton_payload(), ensure_ascii=False)
            publisher.publish(message)
            self._last_skeleton_error = None
            self._skeleton_retry_at = 0.0
        except Exception as exc:
            self._skeleton_retry_at = time.monotonic() + 2.0
            # Report the outage transition once, even if each error text differs.
            if self._last_skeleton_error is None:
                error = str(exc).encode("unicode_escape").decode("ascii")[:200]
                print(f"[rm75] skeleton publish failed: {error}", flush=True)
            self._last_skeleton_error = str(exc)
        finally:
            self.client._lock.release()

    def _motion_status(self):
        with self._action_lock:
            active_action_id = self._active_action_id
            last_completion = dict(self._last_completion) if self._last_completion else None
        return {
            **self.client.status(),
            "active_action_id": active_action_id,
            "last_completion": last_completion,
            "limits_deg": JOINT_LIMITS_DEG,
            "max_speed_percent": self.max_speed_percent,
            "watchdog": {
                "start_grace_seconds": self.start_grace_seconds,
                "stall_timeout_seconds": self.stall_timeout_seconds,
                "progress_threshold_deg": self.progress_threshold_deg,
                "max_motion_seconds": self.max_motion_seconds,
            },
        }

    def _preflight(self):
        state = self.client.call("rm_get_arm_all_state")
        joint_errors = [int(value) for value in state.get("joint_err_code", [])]
        arm_errors = state.get("err", {})
        if len(joint_errors) != 7 or any(joint_errors):
            raise RuntimeError(f"joint error preflight failed: {joint_errors}")
        arm_error_codes = [int(value) for value in arm_errors.get("err", []) if int(value) != 0]
        if arm_error_codes:
            raise RuntimeError(f"arm error preflight failed: {arm_errors}")
        enabled = [int(value) for value in state.get("joint_en_flag", [])]
        if len(enabled) != 7 or not all(enabled):
            raise RuntimeError(f"all seven joints must be enabled before motion: {enabled}")
        return state

    def _prepare_target(self, args):
        if not self.client.motion_enabled:
            raise PermissionError("motion is locked; set RM_MOTION_ENABLED=1 only for supervised hardware testing")
        if args.get("confirm_motion") is not True:
            raise ValueError("confirm_motion must be true")
        joint_fields = [f"joint{i}_deg" for i in range(1, 8)]
        current = [float(value) for value in self.client.call("rm_get_joint_degree")]
        if len(current) != 7 or not all(math.isfinite(value) for value in current):
            raise RuntimeError(f"invalid current joint state: {current!r}")
        requested = {
            index: args.get(field, current[index])
            for index, field in enumerate(joint_fields)
        }
        controller_min = [float(value) for value in self.client.call("rm_get_joint_drive_min_pos")]
        controller_max = [float(value) for value in self.client.call("rm_get_joint_drive_max_pos")]
        if len(controller_min) != 7 or len(controller_max) != 7:
            raise RuntimeError("controller did not return seven joint limits")
        target = [0.0] * 7
        for index, raw in requested.items():
            value = float(raw)
            if not math.isfinite(value):
                raise ValueError(f"joint{index + 1}_deg must be finite")
            official_low, official_high = JOINT_LIMITS_DEG[index]
            low = max(official_low, controller_min[index])
            high = min(official_high, controller_max[index])
            if not math.isfinite(low) or not math.isfinite(high) or low > high:
                raise RuntimeError(f"invalid controller limits for joint{index + 1}: [{low}, {high}]")
            if not low <= value <= high:
                raise ValueError(f"joint{index + 1}_deg must be within [{low}, {high}]")
            target[index] = value
        speed = int(args.get("speed_percent", self.default_speed_percent))
        if not 1 <= speed <= self.max_speed_percent:
            raise ValueError(f"speed_percent must be within [1, {self.max_speed_percent}]")
        return current, target, speed

    def _motion_deadline_seconds(self, start, target, speed_percent):
        estimates = [
            abs(expected - actual) / (maximum * speed_percent / 100.0)
            for actual, expected, maximum in zip(start, target, JOINT_MAX_SPEED_DEG_S)
        ]
        return min(self.max_motion_seconds, max(30.0, max(estimates, default=0.0) * 3.0 + 10.0))

    def _acp_callback(self, action_id: str, status: str, result: dict):
        """POST action completion to Agent Core."""
        import urllib.request as _urllib
        import ssl as _ssl
        import json
        import os as _os

        agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
        ctx = _ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = _ssl.CERT_NONE
        # Keep full motion evidence in info; the completion event stays small.
        summary = {}
        if status == "completed":
            summary = {"reason": "target_reached"}
        elif "reason" in result:
            summary = {"reason": str(result["reason"])[:240]}
        body = {"action_id": action_id, "status": status, "result": summary,
                "tool": self.PREFIX, "ts": time.time()}
        record = {"action_id": action_id, "status": status, "result": dict(result),
                  "callback": "sending"}
        with self._action_lock:
            self._last_completion = record
        try:
            req = _urllib.Request(
                f"{agent_core_url.rstrip('/')}/api/acp/complete",
                data=json.dumps(body).encode(),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with _urllib.urlopen(req, timeout=5, context=ctx) as response:
                acknowledgement = json.loads(response.read())
            if (not isinstance(acknowledgement, dict)
                    or acknowledgement.get("ok") is not True
                    or acknowledgement.get("action_id") != action_id):
                raise RuntimeError("Agent Core did not acknowledge this action_id")
            with self._action_lock:
                record["callback"] = "accepted"
            # Acceptance proves receipt, not that Core matched a pending action.
            print(f"[rm75 ACP] {action_id} {status}: accepted", flush=True)
        except Exception as exc:
            with self._action_lock:
                record["callback"] = "failed"
                record["callback_error"] = str(exc)
            print(f"[rm75 ACP] {action_id} {status}: callback failed: {exc}", flush=True)

    def _monitor_motion(self, action_id, start, target, max_duration):
        started = time.monotonic()
        deadline = started + max_duration
        last_progress = started + self.start_grace_seconds
        best_error = max(abs(actual - expected) for actual, expected in zip(start, target))
        status, result = "error", {"reason": "unknown"}
        try:
            while time.monotonic() < deadline:
                with self._action_lock:
                    cancelled = action_id in self._cancelled
                if cancelled:
                    status, result = "cancelled", {"reason": "stopmotion"}
                    break
                self._preflight()
                current = [float(value) for value in self.client.call("rm_get_joint_degree")]
                error = max(abs(actual - expected) for actual, expected in zip(current, target))
                now = time.monotonic()
                if error <= self.target_tolerance_deg:
                    status = "completed"
                    result = {"target_degree": target, "actual_degree": current,
                              "max_error_deg": error, "elapsed_seconds": now - started}
                    break
                if best_error - error >= self.progress_threshold_deg:
                    best_error = error
                    last_progress = now
                elif now >= started + self.start_grace_seconds and now - last_progress >= self.stall_timeout_seconds:
                    self.client.command("rm_set_arm_slow_stop")
                    result = {
                        "reason": "motion_stalled",
                        "stall_seconds": self.stall_timeout_seconds,
                        "target_degree": target,
                        "actual_degree": current,
                        "max_error_deg": error,
                        "elapsed_seconds": now - started,
                    }
                    break
                time.sleep(self.poll_interval_seconds)
            else:
                self.client.command("rm_set_arm_slow_stop")
                result = {"reason": "motion_deadline_exceeded",
                          "max_motion_seconds": max_duration,
                          "elapsed_seconds": time.monotonic() - started}
        except Exception as exc:
            try:
                self.client.command("rm_set_arm_slow_stop")
            except Exception:
                pass
            result = {"reason": str(exc)}
        finally:
            with self._action_lock:
                if action_id in self._cancelled:
                    status, result = "cancelled", {"reason": "stopmotion"}
                self._cancelled.discard(action_id)
                if self._active_action_id == action_id:
                    self._active_action_id = None
            self._motion_lock.release()
            self._acp_callback(action_id, status, result)

    def _start_motion(self, args):
        if not self.client.motion_enabled:
            raise PermissionError("motion is locked; set RM_MOTION_ENABLED=1 only for supervised hardware testing")
        if args.get("confirm_motion") is not True:
            raise ValueError("confirm_motion must be true")
        if not self._motion_lock.acquire(blocking=False):
            raise RuntimeError(f"another motion is active: {self._active_action_id}")
        try:
            self._preflight()
            current, target, speed = self._prepare_target(args)
            max_duration = self._motion_deadline_seconds(current, target, speed)
            action_id = f"rm75_movej_{uuid4().hex[:10]}"
            # Reserve the ID and submit under the same lock used by stopmotion.
            # An interrupt must see either no submitted move or its actual ID.
            with self._action_lock:
                self._active_action_id = action_id
                try:
                    self.client.command("rm_movej", target, speed, 0, 0, 0)
                except Exception:
                    self._active_action_id = None
                    raise
            threading.Thread(
                target=self._monitor_motion,
                args=(action_id, current, target, max_duration),
                daemon=True,
            ).start()
            print(f"[rm75 ACP] {action_id}: started", flush=True)
            return {"state": "running", "action_id": action_id}
        except Exception:
            self._motion_lock.release()
            raise

    def _stop_motion(self):
        # Keep the action-state lock across the SDK stop request. The monitor
        # cannot select a terminal state between cancellation and slow-stop.
        with self._action_lock:
            action_id = self._active_action_id
            if action_id:
                self._cancelled.add(action_id)
            self.client.command("rm_set_arm_slow_stop")
        return {"state": "stop_requested", "action_id": action_id}

    def dispatch(self, action, args):
        name = args.get("_tool_name")
        if action == "start":
            return {"state": "ready" if name in ("joint_control", "model") else "running"}
        if action == "stop":
            if name == "joint_control":
                self._stop_motion()
            return {"state": "idle"}
        if action == "info":
            topic_out = self._skeleton_topic_out() if name == "joint_states" else []
            return {**self._motion_status(), "topic_out": topic_out}
        if name == "connection":
            return self.client.status()
        if name == "joint_states":
            return self.client.joint_states()
        if name == "model":
            path = Path(__file__).with_name("resource") / "rm75_6f_v.urdf"
            return {"urdf": path.read_text(encoding="utf-8")}
        if name in self.METHODS:
            if name == "controller_state":
                return self.client.call_dict(self.METHODS[name])
            return self.client.call(self.METHODS[name])
        if name == "joint_control":
            if action == "set":
                return self._start_motion(args)
            if action == "stopmotion":
                return self._stop_motion()
            if action == "info":
                return self._motion_status()
        return None


def build_plugins(config, namespace, ros2):
    client = RM75SDKClient(config)
    plugins = [RM75Plugin(client, config, namespace=namespace, ros2=ros2)]
    if config.get("gripper_bridge", {}).get("enabled", True):
        from gripper_bridge import GripperBridgePlugin

        plugins.append(GripperBridgePlugin(client, config, ros2))
    camera_config = config.get("ext_camera", {})
    if camera_config.get("enabled", False):
        from camera import ExtCameraPlugin

        plugins.append(ExtCameraPlugin(camera_config, namespace, ros2.executor_core))
    return plugins
