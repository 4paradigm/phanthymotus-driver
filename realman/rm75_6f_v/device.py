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


GRIPPER_POSITION_MIN = 1   # SDK 契约：手爪开口位置 1~1000
GRIPPER_POSITION_MAX = 1000
GRIPPER_COMPLETION_TIMEOUT = 30  # SDK 阻塞模式下等待夹爪到位的秒数上限（ACP 完成窗口取 +10）
GRIPPER_STOP_WAIT_MARGIN = 5     # stop 等待在途命令到达安全终态的额外余量


def _gripper_position(value) -> int:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("position must be a number") from exc
    if not math.isfinite(numeric) or not GRIPPER_POSITION_MIN <= numeric <= GRIPPER_POSITION_MAX:
        raise ValueError(f"position must be within {GRIPPER_POSITION_MIN}~{GRIPPER_POSITION_MAX}")
    return int(round(numeric))


class GripperPlugin:
    """RealMan 二指夹爪位置控制：复用 RM75SDKClient 的 SDK 连接调用 SDK 夹爪 API。

    与 ext_camera 同模式，作为 RM75-6F-V 驱动的内置卡片；不另起容器、
    不另开 TCP 8080 连接（控制箱单客户端）。运动守卫与 joint_control 一致，
    到位结果通过 ACP 回调异步上报（x-completion 契约）。
    """

    PREFIX = "gripper"

    def __init__(self, client, config, namespace="rm75", ros2=None):
        self.client = client
        self._gripper_lock = threading.Lock()
        self._action_lock = threading.Lock()
        self._active_action_id = None
        self._interrupted = set()
        self._worker_thread = None
        self._last_completion = None

    def get_tools(self):
        schema = action_schema(
            {
                "set_position": (["position", "confirm_motion"], "设置二指夹爪目标位置"),
                "info": ([], "读取夹爪与 SDK 连接状态"),
            },
            {
                "position": {
                    "type": "integer",
                    "minimum": GRIPPER_POSITION_MIN,
                    "maximum": GRIPPER_POSITION_MAX,
                    "description": f"夹爪驱动器目标位置，{GRIPPER_POSITION_MIN}~{GRIPPER_POSITION_MAX}，对应 0~120 mm 行程",
                },
                "confirm_motion": {"type": "boolean", "description": "Must be true for every movement request"},
            },
        )
        schema["x-completion"] = {"actions": ["set_position"], "timeout": GRIPPER_COMPLETION_TIMEOUT + 10}
        schema["x-is-dangerous"] = True
        return [
            tool(
                "gripper",
                "actuator",
                f"RealMan 二指夹爪位置控制。位置范围 {GRIPPER_POSITION_MIN}~{GRIPPER_POSITION_MAX}，对应夹爪行程 0~120 mm。",
                schema,
            )
        ]

    def start(self):
        pass

    def stop(self):
        # SDK 没有夹爪中途停止 API：请求停止时等待在途命令到达安全终态
        # （夹爪走完目标位），再允许共享 SDK 连接被上层销毁。
        self._mark_interrupted()
        self._wait_for_worker()

    def _mark_interrupted(self):
        with self._action_lock:
            action_id = self._active_action_id
            if action_id:
                self._interrupted.add(action_id)

    def _wait_for_worker(self):
        thread = self._worker_thread
        if thread is not None and thread.is_alive():
            thread.join(GRIPPER_COMPLETION_TIMEOUT + GRIPPER_STOP_WAIT_MARGIN)

    def dispatch(self, action, args):
        if action == "info":
            with self._action_lock:
                active = self._active_action_id
            return {
                "state": "connected" if self.client.connected else "disconnected",
                "active_action_id": active,
            }
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self._mark_interrupted()
            self._wait_for_worker()
            return {"state": "idle"}
        if action != "set_position":
            return None
        if not self.client.motion_enabled:
            raise PermissionError("motion is locked; set RM_MOTION_ENABLED=1 only for supervised hardware testing")
        if args.get("confirm_motion") is not True:
            raise ValueError("confirm_motion must be true")
        position = _gripper_position(args.get("position"))
        if not self._gripper_lock.acquire(blocking=False):
            raise RuntimeError(f"another gripper motion is active: {self._active_action_id}")
        action_id = f"rm75_gripper_{uuid4().hex[:10]}"
        with self._action_lock:
            self._active_action_id = action_id
            self._interrupted.discard(action_id)
        self._worker_thread = threading.Thread(
            target=self._gripper_worker,
            args=(action_id, position),
            daemon=True,
        )
        self._worker_thread.start()
        print(f"[rm75 ACP] {action_id}: started", flush=True)
        return {"state": "running", "action_id": action_id}

    def _gripper_worker(self, action_id, position):
        try:
            # 阻塞模式：SDK 等待夹爪到位（上限 GRIPPER_COMPLETION_TIMEOUT 秒）后返回状态码。
            # SDK 没有夹爪中途停止 API，收到停止请求后夹爪仍会走完目标位 —— 这是唯一
            # 确定的安全终态，因此如实上报 completed/target_reached，并附 interrupted 标记。
            self.client.command("rm_set_gripper_position", position, True, GRIPPER_COMPLETION_TIMEOUT)
            interrupted = action_id in self._interrupted
            status, result = "completed", {
                "reason": "target_reached", "position": position, "interrupted": interrupted,
            }
        except Exception as exc:
            status, result = "failed", {"reason": str(exc), "position": position}
        finally:
            with self._action_lock:
                if self._active_action_id == action_id:
                    self._active_action_id = None
                self._interrupted.discard(action_id)
            self._gripper_lock.release()
            self._acp_callback(action_id, status, result)

    def _acp_callback(self, action_id, status, result):
        """记录完成事件并上报 Agent Core（网络部分见共享的 _acp_complete）。"""
        with self._action_lock:
            self._last_completion = {"action_id": action_id, "status": status, "result": dict(result)}
        _acp_complete(action_id, status, result, self.PREFIX)


def _acp_complete(action_id, status, result, tool_name):
    """POST action completion to Agent Core（与 RM75Plugin 同协议）。

    TLS 校验关闭是有意为之：本驱动的部署契约不带 CA 证书
    （见 deploy/service.yml 与镜像契约测试对 AGENT_CORE_CA_CERT 的断言），
    与 RM75Plugin._acp_callback 及 common/vendor_runtime.start_registration
    的既有实现保持一致。
    """
    import json
    import os as _os
    import ssl as _ssl
    import urllib.request as _urllib

    agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    ctx = _ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = _ssl.CERT_NONE
    summary = {}
    if status == "completed":
        summary = {"reason": "target_reached"}
    elif "reason" in result:
        summary = {"reason": str(result["reason"])[:240]}
    body = {"action_id": action_id, "status": status, "result": summary,
            "tool": tool_name, "ts": time.time()}
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
        print(f"[rm75 ACP] {action_id} {status}: accepted", flush=True)
    except Exception as exc:
        print(f"[rm75 ACP] {action_id} {status}: callback failed: {exc}", flush=True)


class CartesianPlugin:
    """笛卡尔空间运动卡片：movel 直线 / move_offset 偏移 / movep 多路径点轨迹。

    位姿单位面向画布：位置毫米、姿态度（SDK 内部为米/弧度，转换封装在插件内）。
    与 joint_control 共享运动锁（同一时刻只允许一个运动流），安全守卫、
    stall 检测与 ACP 异步完成与 joint_control 保持一致。
    """

    PREFIX = "cartesian_control"

    def __init__(self, client, config, namespace="rm75", ros2=None, arm_plugin=None):
        self.client = client
        self._arm = arm_plugin  # 共享运动锁与 preflight
        self._motion_lock = arm_plugin._motion_lock if arm_plugin is not None else threading.Lock()
        self._action_lock = threading.Lock()
        self._active_action_id = None
        self._cancelled = set()
        self._last_completion = None
        safety = config.get("safety", {})
        self.max_speed_percent = min(int(safety.get("max_speed_percent", 10)), 10)
        self.default_speed_percent = min(int(safety.get("default_speed_percent", 5)), self.max_speed_percent)
        self.position_tolerance_mm = float(safety.get("position_tolerance_mm", 5.0))
        self.euler_tolerance_deg = float(safety.get("euler_tolerance_deg", 2.0))
        self.poll_interval_seconds = float(safety.get("poll_interval_seconds", 0.2))
        self.start_grace_seconds = float(safety.get("start_grace_seconds", 2.0))
        self.stall_timeout_seconds = float(safety.get("stall_timeout_seconds", 10.0))
        self.progress_threshold_mm = float(safety.get("progress_threshold_mm", 1.0))
        self.max_motion_seconds = float(safety.get("max_motion_seconds", 300.0))

    def get_tools(self):
        position_props = {
            field: {"type": "number", "description": desc}
            for field, desc in (
                ("x_mm", "目标位置 X，毫米（基座坐标系）"),
                ("y_mm", "目标位置 Y，毫米（基座坐标系）"),
                ("z_mm", "目标位置 Z，毫米（基座坐标系）"),
                ("rx_deg", "目标姿态 Roll，度"),
                ("ry_deg", "目标姿态 Pitch，度"),
                ("rz_deg", "目标姿态 Yaw，度"),
            )
        }
        offset_props = {
            field: {"type": "number", "description": desc}
            for field, desc in (
                ("dx_mm", "位置偏移 X，毫米"),
                ("dy_mm", "位置偏移 Y，毫米"),
                ("dz_mm", "位置偏移 Z，毫米"),
                ("drx_deg", "姿态偏移 Roll，度"),
                ("dry_deg", "姿态偏移 Pitch，度"),
                ("drz_deg", "姿态偏移 Yaw，度"),
            )
        }
        properties = {
            **position_props,
            **offset_props,
            "frame_type": {"type": "string", "enum": ["tool", "work"], "default": "tool",
                           "description": "偏移参考坐标系：tool 工具系 / work 工作系"},
            "waypoints": {
                "type": "array",
                "minItems": 1,
                "maxItems": 20,
                "items": {"type": "array", "minItems": 6, "maxItems": 6, "items": {"type": "number"}},
                "description": "路径点序列，每个点 [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]",
            },
            "speed_percent": {"type": "integer", "minimum": 1, "maximum": self.max_speed_percent,
                              "default": self.default_speed_percent},
            "confirm_motion": {"type": "boolean", "description": "Must be true for every movement request"},
        }
        schema = action_schema(
            {
                "movel": (["x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg", "speed_percent", "confirm_motion"],
                          "笛卡尔直线运动到绝对位姿（位置毫米、姿态度，相对基座坐标系）"),
                "move_offset": (["dx_mm", "dy_mm", "dz_mm", "drx_deg", "dry_deg", "drz_deg", "frame_type", "speed_percent", "confirm_motion"],
                                "沿工具/工作坐标系做直线偏移（相对当前位姿）"),
                "movep": (["waypoints", "speed_percent", "confirm_motion"],
                          "依次经过多个路径点的轨迹运动"),
                "stopmotion": ([], "请求受控减速停止"),
                "info": ([], "读取运动状态与安全配置"),
            },
            properties,
        )
        schema["x-completion"] = {"actions": ["movel", "move_offset", "movep"], "timeout": 305}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stopmotion"}}
        schema["x-is-dangerous"] = True
        return [
            tool(
                "cartesian_control",
                "actuator",
                "笛卡尔空间运动：直线(movel)、偏移(move_offset)、多路径点轨迹(movep)。位置毫米、姿态度。",
                schema,
            )
        ]

    def start(self):
        pass

    def stop(self):
        with self._action_lock:
            action_id = self._active_action_id
            if action_id:
                self._cancelled.add(action_id)
        if action_id and self.client.connected:
            try:
                self.client.command("rm_set_arm_slow_stop")
            except Exception as exc:
                print(f"[rm75] cartesian shutdown stop failed: {exc}", flush=True)

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self._stop_motion()
            return {"state": "idle"}
        if action == "info":
            return self._motion_status()
        if action == "stopmotion":
            return self._stop_motion()
        if action in ("movel", "move_offset", "movep"):
            return self._start_cartesian(action, args)
        return None

    def _motion_status(self):
        with self._action_lock:
            active_action_id = self._active_action_id
            last = self._last_completion
        return {
            "state": "moving" if active_action_id else "ready",
            "active_action_id": active_action_id,
            "last_completion": jsonable(last),
            "motion_enabled": self.client.motion_enabled,
            "read_only": not self.client.motion_enabled,
            "position_tolerance_mm": self.position_tolerance_mm,
            "euler_tolerance_deg": self.euler_tolerance_deg,
            "max_speed_percent": self.max_speed_percent,
        }

    def _start_cartesian(self, motion_type, args):
        if not self.client.motion_enabled:
            raise PermissionError("motion is locked; set RM_MOTION_ENABLED=1 only for supervised hardware testing")
        if args.get("confirm_motion") is not True:
            raise ValueError("confirm_motion must be true")
        speed_percent = int(args.get("speed_percent", self.default_speed_percent))
        if not 1 <= speed_percent <= self.max_speed_percent:
            raise ValueError(f"speed_percent must be within 1~{self.max_speed_percent}")
        if not self._motion_lock.acquire(blocking=False):
            raise RuntimeError(f"another motion is active: {self._active_action_id}")
        action_id = f"rm75_cart_{uuid4().hex[:10]}"
        with self._action_lock:
            self._active_action_id = action_id
            self._cancelled.discard(action_id)
        try:
            if self._arm is not None:
                self._arm._preflight()
            target = self._plan_target(motion_type, args, speed_percent)
            max_duration = self._motion_deadline_seconds(target, speed_percent)
            threading.Thread(
                target=self._monitor_cartesian,
                args=(action_id, target, max_duration),
                daemon=True,
            ).start()
            print(f"[rm75 ACP] {action_id}: started ({motion_type})", flush=True)
            return {"state": "running", "action_id": action_id}
        except Exception:
            with self._action_lock:
                if self._active_action_id == action_id:
                    self._active_action_id = None
            self._motion_lock.release()
            raise

    def _plan_target(self, motion_type, args, speed_percent):
        if motion_type == "movel":
            pose_mm_deg = self._pose_from_fields(args, ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg"))
            self.client.command("rm_movel", self._to_sdk_pose(pose_mm_deg), speed_percent, 0, 0, 0)
            return pose_mm_deg
        if motion_type == "move_offset":
            offset_mm_deg = self._pose_from_fields(args, ("dx_mm", "dy_mm", "dz_mm", "drx_deg", "dry_deg", "drz_deg"))
            frame = args.get("frame_type", "tool")
            if frame not in ("tool", "work"):
                raise ValueError("frame_type must be 'tool' or 'work'")
            frame_type = 1 if frame == "tool" else 0
            current = self._current_pose_mm_deg()
            target = [a + b for a, b in zip(current, offset_mm_deg)]
            self.client.command("rm_movel_offset", self._to_sdk_pose(offset_mm_deg), speed_percent, 0, 0, frame_type, 0)
            return target
        if motion_type == "movep":
            waypoints = args.get("waypoints")
            if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 20:
                raise ValueError("waypoints must be a list of 1~20 poses")
            poses = [self._waypoint_pose(item, index) for index, item in enumerate(waypoints)]
            # 前 N-1 个点 connect=1（与下一条轨迹联合规划），末点 connect=0 立即执行
            for pose in poses[:-1]:
                self.client.command("rm_movel", self._to_sdk_pose(pose), speed_percent, 0, 1, 0)
            self.client.command("rm_movel", self._to_sdk_pose(poses[-1]), speed_percent, 0, 0, 0)
            return poses[-1]
        raise ValueError(f"unknown motion type: {motion_type}")

    def _pose_from_fields(self, args, fields):
        pose = []
        for field in fields:
            value = args.get(field)
            try:
                numeric = float(value)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{field} must be a number") from exc
            if not math.isfinite(numeric):
                raise ValueError(f"{field} must be finite")
            pose.append(numeric)
        return pose

    def _waypoint_pose(self, item, index):
        if not isinstance(item, (list, tuple)) or len(item) != 6:
            raise ValueError(f"waypoint {index} must have exactly 6 numbers")
        try:
            numeric = [float(value) for value in item]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"waypoint {index} must contain numbers") from exc
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError(f"waypoint {index} must be finite")
        return numeric

    @staticmethod
    def _to_sdk_pose(pose_mm_deg):
        x, y, z, rx, ry, rz = pose_mm_deg
        return [x / 1000.0, y / 1000.0, z / 1000.0,
                math.radians(rx), math.radians(ry), math.radians(rz)]

    def _current_pose_mm_deg(self):
        state = self.client.call("rm_get_current_arm_state")
        pose = [float(value) for value in state.get("pose", [])]
        if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
            raise RuntimeError(f"invalid arm pose: {pose!r}")
        x, y, z, rx, ry, rz = pose
        return [x * 1000.0, y * 1000.0, z * 1000.0,
                math.degrees(rx), math.degrees(ry), math.degrees(rz)]

    @staticmethod
    def _pose_error(current, target):
        position_error = max(abs(a - b) for a, b in zip(current[:3], target[:3]))
        euler_error = max(abs((a - b + 180.0) % 360.0 - 180.0) for a, b in zip(current[3:], target[3:]))
        return position_error, euler_error

    def _motion_deadline_seconds(self, target, speed_percent):
        current = self._current_pose_mm_deg()
        distance_mm = math.sqrt(sum((a - b) ** 2 for a, b in zip(current[:3], target[:3])))
        # RM75 最大直线速度按 600 mm/s 粗估，速度百分比按比例折算，留 3 倍余量；
        # stall 检测是真正的安全网，此估算只用于给 ACP 完成窗口一个上界。
        speed_mm_s = 600.0 * speed_percent / 100.0
        return min(self.max_motion_seconds, max(30.0, distance_mm / speed_mm_s * 3.0 + 10.0))

    def _monitor_cartesian(self, action_id, target, max_duration):
        started = time.monotonic()
        deadline = started + max_duration
        last_progress = started + self.start_grace_seconds
        best_position_error = None
        status, result = "error", {"reason": "unknown"}
        try:
            while time.monotonic() < deadline:
                with self._action_lock:
                    cancelled = action_id in self._cancelled
                if cancelled:
                    status, result = "cancelled", {"reason": "stopmotion"}
                    break
                if self._arm is not None:
                    self._arm._preflight()
                current = self._current_pose_mm_deg()
                position_error, euler_error = self._pose_error(current, target)
                now = time.monotonic()
                if position_error <= self.position_tolerance_mm and euler_error <= self.euler_tolerance_deg:
                    status = "completed"
                    result = {"target_pose_mm_deg": target, "actual_pose_mm_deg": current,
                              "position_error_mm": position_error, "euler_error_deg": euler_error,
                              "elapsed_seconds": now - started}
                    break
                if (best_position_error is None
                        or best_position_error - position_error >= self.progress_threshold_mm):
                    best_position_error = position_error
                    last_progress = now
                elif (now >= started + self.start_grace_seconds
                        and now - last_progress >= self.stall_timeout_seconds):
                    self.client.command("rm_set_arm_slow_stop")
                    result = {"reason": "motion_stalled",
                              "stall_seconds": self.stall_timeout_seconds,
                              "target_pose_mm_deg": target, "actual_pose_mm_deg": current,
                              "position_error_mm": position_error, "euler_error_deg": euler_error,
                              "elapsed_seconds": now - started}
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
                self._last_completion = {"action_id": action_id, "status": status, "result": dict(result)}
            self._motion_lock.release()
            self._acp_callback(action_id, status, result)

    def _stop_motion(self):
        with self._action_lock:
            action_id = self._active_action_id
            if action_id:
                self._cancelled.add(action_id)
            self.client.command("rm_set_arm_slow_stop")
        return {"state": "stop_requested", "action_id": action_id}

    def _acp_callback(self, action_id, status, result):
        _acp_complete(action_id, status, result, self.PREFIX)


def build_plugins(config, namespace, ros2):
    client = RM75SDKClient(config)
    arm = RM75Plugin(client, config, namespace=namespace, ros2=ros2)
    return [
        arm,
        GripperPlugin(client, config, namespace=namespace, ros2=ros2),
        CartesianPlugin(client, config, arm_plugin=arm, namespace=namespace, ros2=ros2),
    ]
