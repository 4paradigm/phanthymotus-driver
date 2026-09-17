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
        self._trajectory_event = threading.Event()
        self._trajectory_lock = threading.Lock()
        self._trajectory_waiting = False
        self._trajectory_result = None
        # ctypes 回调必须由 Python 对象持有，否则可能被 GC 后导致 C SDK 回调失效。
        self._event_callback = None

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
        from Robotic_Arm.rm_robot_interface import RoboticArm, rm_event_callback_ptr, rm_thread_mode_e

        with self._lock:
            self._robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            self._handle = self._robot.rm_create_robot_arm(self.ip, self.port)
            if not self.connected:
                bad_id = getattr(self._handle, "id", None)
                self._handle = None
                self._robot = None
                raise ConnectionError(f"RealMan SDK could not connect to {self.ip}:{self.port}; handle={bad_id}")
            self._event_callback = rm_event_callback_ptr(self._on_arm_event)
            self._robot.rm_get_arm_event_call_back(self._event_callback)
            print(f"[rm75] SDK connected to {self.ip}:{self.port} handle={self._handle.id}", flush=True)

    def stop(self):
        self.cancel_trajectory_wait()
        with self._lock:
            robot, self._robot = self._robot, None
            self._handle = None
            self._event_callback = None
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

    def _on_arm_event(self, data):
        """接收三线程 API2 的规划轨迹到位事件。"""
        try:
            if int(data.event_type) != 1 or int(data.device) != 0:
                return
            handle_id = int(getattr(data, "handle_id", -1))
            if self.connected and handle_id != int(self._handle.id):
                return
            with self._trajectory_lock:
                if not self._trajectory_waiting:
                    return
                self._trajectory_result = bool(data.trajectory_state)
                self._trajectory_event.set()
            outcome = "completed" if bool(data.trajectory_state) else "failed"
            print(f"[rm75] controller trajectory event: {outcome}", flush=True)
        except Exception as exc:
            print(f"[rm75] controller trajectory event ignored: {exc}", flush=True)

    def command_trajectory(self, method, *args):
        """非阻塞下发轨迹，并在下发前准备官方到位事件。"""
        with self._lock:
            if not self.connected or self._robot is None:
                raise ConnectionError("RM75 SDK is not connected")
            with self._trajectory_lock:
                if self._trajectory_waiting:
                    raise RuntimeError("another controller trajectory wait is active")
                self._trajectory_result = None
                self._trajectory_event.clear()
                self._trajectory_waiting = True
            try:
                code = int(getattr(self._robot, method)(*args))
                if code != 0:
                    raise RuntimeError(f"{method} failed with RealMan SDK code {code}")
                return code
            except Exception:
                self._discard_trajectory_wait()
                raise

    def poll_trajectory(self, timeout_seconds=0.0):
        """读取官方到位事件；尚未收到时保留等待状态并返回 None。"""
        self._trajectory_event.wait(timeout=max(0.0, float(timeout_seconds)))
        with self._trajectory_lock:
            # 回调可能恰好在 Event.wait 超时与取得锁之间到达；以锁内结果为准，
            # 避免把已经收到的成功事件误判为超时。
            result = self._trajectory_result
            if result is not None:
                self._trajectory_waiting = False
                self._trajectory_result = None
                self._trajectory_event.clear()
        return result

    def wait_trajectory(self, timeout_seconds):
        """等待官方 current trajectory state 回调；None 表示超时。"""
        result = self.poll_trajectory(timeout_seconds)
        if result is None:
            self.discard_trajectory_wait()
        return result

    def discard_trajectory_wait(self):
        with self._trajectory_lock:
            self._trajectory_waiting = False
            self._trajectory_result = None
            self._trajectory_event.clear()

    # 兼容类内旧调用名称。
    _discard_trajectory_wait = discard_trajectory_wait

    def cancel_trajectory_wait(self):
        """解除 Python 侧事件等待；不会代替控制器慢停命令。"""
        with self._trajectory_lock:
            if not self._trajectory_waiting:
                return False
            self._trajectory_result = False
            self._trajectory_event.set()
            return True

    def command_interrupt(self, method, *args):
        """运动命令为非阻塞模式，停止命令可安全使用同一串行 SDK 入口。"""
        return self.command(method, *args)


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
        self._motion_state = {"active_action_id": None}
        # 提交/慢停串行化锁：与 CartesianPlugin 共享，保证任一卡片的 stopmotion
        # 都排在另一卡片正在进行的 SDK 运动下发之后。
        self._submission_lock = threading.Lock()
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
            active_action_id = self._motion_state["active_action_id"]
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
        record = {"action_id": action_id, "status": status, "result": dict(result),
                  "callback": "sending"}
        with self._action_lock:
            self._last_completion = record
        callback, error = _acp_complete(action_id, status, result, self.PREFIX)
        with self._action_lock:
            record["callback"] = callback
            if error is not None:
                record["callback_error"] = error

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
                if self._motion_state["active_action_id"] == action_id:
                    self._motion_state["active_action_id"] = None
            self._motion_lock.release()
            self._acp_callback(action_id, status, result)

    def _start_motion(self, args):
        if not self.client.motion_enabled:
            raise PermissionError("motion is locked; set RM_MOTION_ENABLED=1 only for supervised hardware testing")
        if args.get("confirm_motion") is not True:
            raise ValueError("confirm_motion must be true")
        if not self._motion_lock.acquire(blocking=False):
            raise RuntimeError(f"another motion is active: {self._motion_state['active_action_id']}")
        try:
            self._preflight()
            current, target, speed = self._prepare_target(args)
            max_duration = self._motion_deadline_seconds(current, target, speed)
            action_id = f"rm75_movej_{uuid4().hex[:10]}"
            # Reserve the ID and submit under the same lock used by stopmotion.
            # An interrupt must see either no submitted move or its actual ID.
            # rm_movej 在共享的 _submission_lock 内下发：任何卡片的 stopmotion
            # 都必须等它完成后才能发慢停。
            with self._action_lock:
                self._active_action_id = action_id
                self._motion_state["active_action_id"] = action_id
                try:
                    with self._submission_lock:
                        self.client.command("rm_movej", target, speed, 0, 0, 0)
                except Exception:
                    self._active_action_id = None
                    self._motion_state["active_action_id"] = None
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
        # 运动锁和动作 ID 在两张卡之间共享；任一 stop 卡都必须能停止实际持有者。
        # SDK 慢停调用无超时上限，不得在 _action_lock 内执行；且必须排在
        # 共享 _submission_lock 里正在进行的运动下发之后。
        with self._action_lock:
            action_id = self._motion_state["active_action_id"] or self._active_action_id
            if action_id:
                self._cancelled.add(action_id)
        if self.client.connected:
            with self._submission_lock:
                # 失败向上传播：stop 失败不得谎报 idle（见生命周期测试契约）
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
CARTESIAN_STOP_JOIN_MARGIN = 10  # 笛卡尔 stop 等待监控线程收尾的额外余量（秒）


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

    TLS 证书校验保持开启：部署通过 AGENT_CORE_CA_CERT 挂载 Agent Core 的 CA
    （见 deploy/service.yml），缺失时不发送 —— 不回退到关闭校验。
    """
    import json
    import os as _os
    import ssl as _ssl
    import urllib.request as _urllib

    agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    ca_cert = _os.environ.get("AGENT_CORE_CA_CERT")
    if not ca_cert:
        error = "AGENT_CORE_CA_CERT is required"
        print(f"[rm75 ACP] {action_id} {status}: callback failed: {error}", flush=True)
        return "failed", error
    summary = {}
    if status == "completed":
        summary = {"reason": "target_reached"}
    elif "reason" in result:
        summary = {"reason": str(result["reason"])[:240]}
    body = {"action_id": action_id, "status": status, "result": summary,
            "tool": tool_name, "ts": time.time()}
    try:
        ctx = _ssl.create_default_context(cafile=ca_cert)
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
        return "accepted", None
    except Exception as exc:
        print(f"[rm75 ACP] {action_id} {status}: callback failed: {exc}", flush=True)
        return "failed", str(exc)


class CartesianPlugin:
    """笛卡尔空间运动卡片：工具坐标系相对偏移。

    位姿单位面向画布：位置毫米、姿态度（SDK 内部为米/弧度，转换封装在插件内）。
    与 joint_control 共享运动锁（同一时刻只允许一个运动流），安全守卫、
    stall 检测与 ACP 异步完成与 joint_control 保持一致。
    """

    PREFIX = "cartesian_control"
    # Agent Core 短暂重启时，不能让已完成的运动永久停留在画布“执行中”。
    ACP_RETRY_DELAY_SECONDS = 2.0
    ACP_RETRY_ATTEMPTS = 30

    def __init__(self, client, config, namespace="rm75", ros2=None, arm_plugin=None):
        self.client = client
        self._arm = arm_plugin  # 共享运动锁、动作状态与 preflight
        self._motion_lock = arm_plugin._motion_lock if arm_plugin is not None else threading.Lock()
        self._motion_state = arm_plugin._motion_state if arm_plugin is not None else {"active_action_id": None}
        self._action_lock = threading.Lock()
        # 与 RM75Plugin 共享：提交/慢停串行化对两张卡片全局生效
        self._submission_lock = arm_plugin._submission_lock if arm_plugin is not None else threading.Lock()
        self._active_action_id = None
        self._cancelled = arm_plugin._cancelled if arm_plugin is not None else set()
        self._monitor_thread = None
        self._last_completion = None
        self._terminal_action_ids = set()
        safety = config.get("safety", {})
        self.max_speed_percent = min(int(safety.get("max_speed_percent", 10)), 10)
        self.default_speed_percent = min(int(safety.get("default_speed_percent", 5)), self.max_speed_percent)
        self.position_tolerance_mm = float(safety.get("position_tolerance_mm", 5.0))
        self.euler_tolerance_deg = float(safety.get("euler_tolerance_deg", 2.0))
        self.poll_interval_seconds = float(safety.get("poll_interval_seconds", 0.2))
        self.start_grace_seconds = float(safety.get("start_grace_seconds", 2.0))
        self.stall_timeout_seconds = float(safety.get("stall_timeout_seconds", 10.0))
        self.progress_threshold_mm = float(safety.get("progress_threshold_mm", 1.0))
        self.euler_progress_threshold_deg = float(safety.get("euler_progress_threshold_deg", 0.5))
        self.max_motion_seconds = float(safety.get("max_motion_seconds", 300.0))
        cartesian = config.get("cartesian", {})
        self.cartesian_enabled = cartesian.get("enabled", False) is True
        # 水平工作半径（基座轴线到 TCP 的水平距离，RM75-6F 官方标称 638.5mm，取整 640 留 1.5mm 余量）
        self.max_radius_mm = float(cartesian.get("max_radius_mm", 640.0))
        # 肩关节距基座平面的高度，取自官方 RM75 系列 MDH 参数 d1=240.5mm
        # （develop.realman-robotics.com RM75 本体参数，取代旧展示用 URDF 的 340mm）。
        self.shoulder_height_mm = float(cartesian.get("shoulder_height_mm", 240.5))
        # 肩部到法兰的最大直线臂展：MDH d3+d5+d7 = 256+210+184 = 650mm（RM75-6F-V）。
        self.max_reach_mm = float(cartesian.get("max_reach_mm", 650.0))
        self.max_euler_abs_deg = float(cartesian.get("max_euler_abs_deg", 360.0))
        # 夹爪 TCP 相对法兰的伸出量。校验不再用它从姿态反推法兰位置——
        # 真机竖直位姿上报欧拉角 (0,0,0) 而夹爪实际竖直伸出（TCP z≈1112 =
        # 240.5+650+222.5），说明上报欧拉角不编码物理工具轴方向。改为把
        # 工具长度作为包络余量：TCP 距肩部 ≤ 臂展 + 工具长度。
        # 仅当控制器已把工具坐标系设为夹爪 TCP 时才应配置非零值；否则保持 0（法兰即 TCP）。
        self.tool_length_mm = float(cartesian.get("tool_length_mm", 0.0))
        # 四代控制器支持原生工具系偏移。由控制器从当前关节构型规划，
        # 能避免驱动把相对偏移转换成绝对位姿后在奇异点附近丢失构型信息。
        # 三代控制器返回 SDK -7 时自动退回 rm_movel 兼容路径。
        self.native_tool_offset_enabled = cartesian.get("native_tool_offset_enabled", True) is not False
        self.stop_finalize_seconds = float(cartesian.get("stop_finalize_seconds", 2.0))

    def get_tools(self):
        offset_props = {
            field: {"type": "number", "description": desc}
            for field, desc in (
                ("dx_mm", "位置偏移 X，毫米；留空表示不沿 X 偏移"),
                ("dy_mm", "位置偏移 Y，毫米；留空表示不沿 Y 偏移"),
                ("dz_mm", "位置偏移 Z，毫米；留空表示不沿 Z 偏移"),
                ("drx_deg", "姿态偏移 Roll，度；留空表示不旋转"),
                ("dry_deg", "姿态偏移 Pitch，度；留空表示不旋转"),
                ("drz_deg", "姿态偏移 Yaw，度；留空表示不旋转"),
            )
        }
        properties = {
            **offset_props,
            "frame_type": {"type": "string", "enum": ["tool"], "default": "tool",
                           "description": "偏移参考坐标系：目前仅支持 tool 工具系（工作坐标系偏移需控制器激活坐标系位姿，暂不开放）"},
            "speed_percent": {"type": "integer", "minimum": 1, "maximum": self.max_speed_percent,
                              "default": self.default_speed_percent},
            "cartesian_enabled": {
                "type": "boolean",
                "default": False,
                "description": "本次是否允许笛卡尔运动；必须显式设为 true",
            },
            "confirm_motion": {"type": "boolean", "description": "Must be true for every movement request"},
        }
        schema = action_schema(
            {
                "move_offset": (["dx_mm", "dy_mm", "dz_mm", "drx_deg", "dry_deg", "drz_deg", "frame_type", "speed_percent", "cartesian_enabled", "confirm_motion"],
                                "沿工具坐标系做直线偏移（相对当前位姿；未填写的轴不偏移）"),
                "stopmotion": ([], "请求受控减速停止"),
                "info": ([], "读取运动状态与安全配置"),
            },
            properties,
        )
        schema["x-completion"] = {"actions": ["move_offset"], "timeout": 305}
        schema["x-hooks"] = {
            "on_interrupt_motion": {"action": "stopmotion"},
            "on_interrupt_all": {"action": "stopmotion"},
        }
        schema["x-is-dangerous"] = True
        return [
            tool(
                "cartesian_control",
                "actuator",
                "笛卡尔空间相对运动：沿工具系偏移执行 move_offset。位置毫米、姿态度。",
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
            self._request_slow_stop()
        # 监控线程在下一轮询看到 cancelled 后立即收尾；join 保证共享 SDK 连接
        # 被上层（RM75Plugin.stop）销毁前，ACP 终态已上报完成。
        thread = self._monitor_thread
        if thread is not None and thread.is_alive():
            thread.join(self.poll_interval_seconds * 5 + CARTESIAN_STOP_JOIN_MARGIN)

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
        # movel/movep 已从卡片定义中移除；仅保留此处兼容已保存的旧流程，
        # 新建流程只能选择 move_offset。
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
            "motion_enabled": self.client.motion_enabled and self.cartesian_enabled,
            "read_only": not (self.client.motion_enabled and self.cartesian_enabled),
            "cartesian_enabled": self.cartesian_enabled,
            "position_tolerance_mm": self.position_tolerance_mm,
            "euler_tolerance_deg": self.euler_tolerance_deg,
            "max_speed_percent": self.max_speed_percent,
            "max_radius_mm": self.max_radius_mm,
            "shoulder_height_mm": self.shoulder_height_mm,
            "max_reach_mm": self.max_reach_mm,
            "max_euler_abs_deg": self.max_euler_abs_deg,
            "tool_length_mm": self.tool_length_mm,
            "native_tool_offset_enabled": self.native_tool_offset_enabled,
        }

    def _start_cartesian(self, motion_type, args):
        if not self.cartesian_enabled:
            raise PermissionError(
                "cartesian motion is disabled by deployment configuration; "
                "set cartesian.enabled=true only after supervised workspace validation"
            )
        if args.get("cartesian_enabled") is not True:
            raise PermissionError(
                "cartesian motion is disabled for this request; "
                "set cartesian_enabled=true on the card"
            )
        if not self.client.motion_enabled:
            raise PermissionError("motion is locked; set RM_MOTION_ENABLED=1 only for supervised hardware testing")
        if args.get("confirm_motion") is not True:
            raise ValueError("confirm_motion must be true")
        speed_percent = int(args.get("speed_percent", self.default_speed_percent))
        if not 1 <= speed_percent <= self.max_speed_percent:
            raise ValueError(f"speed_percent must be within 1~{self.max_speed_percent}")
        if not self._motion_lock.acquire(blocking=False):
            active = self._motion_state["active_action_id"] or "joint/cartesian motion"
            raise RuntimeError(f"another motion is active: {active}")
        action_id = f"rm75_cart_{uuid4().hex[:10]}"
        # 状态锁不能覆盖无超时上限的 SDK 调用，否则 info 也会被控制器故障拖死。
        # submission_lock 只负责保证 stopmotion 排在运动下发之后。
        controller_completion = self._uses_controller_completion(motion_type)
        submitted = False
        try:
            with self._action_lock:
                self._active_action_id = action_id
                self._motion_state["active_action_id"] = action_id
                self._cancelled.discard(action_id)
            with self._submission_lock:
                try:
                    if self._arm is not None:
                        self._arm._preflight()
                    current = self._current_pose_mm_deg()
                    target = self._plan_target(motion_type, args, current)
                    max_duration = self._motion_deadline_seconds(current, target, speed_percent)
                    if not controller_completion:
                        submitted = True
                        self._submit(motion_type, args, target, speed_percent)
                except Exception:
                    with self._action_lock:
                        if self._active_action_id == action_id:
                            self._active_action_id = None
                        if self._motion_state["active_action_id"] == action_id:
                            self._motion_state["active_action_id"] = None
                    if submitted:
                        # 旧 movep 可能已提交部分路径；统一请求慢停，不留无看护的运动。
                        try:
                            self.client.command("rm_set_arm_slow_stop")
                        except Exception:
                            pass
                    raise
        except Exception:
            self._motion_lock.release()
            raise
        if controller_completion:
            # 三线程 API2 使用 block=0 下发，再等待官方 current trajectory state
            # 回调。不能把 block=1 放进后台线程：若 C SDK 不返回，旧线程会继续
            # 占用同一控制器句柄，使下一次运动和 stopmotion 一起卡住。
            target_fn = self._execute_controller_cartesian
            target_args = (
                action_id,
                motion_type,
                args,
                current,
                target,
                speed_percent,
                self._controller_wait_deadline_seconds(current, target, speed_percent),
            )
        else:
            target_fn = self._monitor_cartesian
            target_args = (action_id, current, target, max_duration)
        self._monitor_thread = threading.Thread(target=target_fn, args=target_args, daemon=True)
        self._monitor_thread.start()
        print(f"[rm75 ACP] {action_id}: started ({motion_type})", flush=True)
        return {"state": "running", "action_id": action_id}

    def _uses_controller_completion(self, motion_type):
        return (
            motion_type == "move_offset"
            and self.native_tool_offset_enabled
            and callable(getattr(self.client, "command_trajectory", None))
            and callable(getattr(self.client, "poll_trajectory", None))
        )

    def _execute_controller_cartesian(
            self, action_id, motion_type, args, start_pose, target, speed_percent, timeout_seconds):
        try:
            with self._action_lock:
                cancelled = action_id in self._cancelled
            if cancelled:
                self._finish_cartesian(action_id, "cancelled", {"reason": "stopmotion"})
                return
            self._submit(motion_type, args, target, speed_percent, wait_for_completion=True)
        except Exception as exc:
            self._finish_cartesian(
                action_id, "error", {"reason": str(exc), "target_pose_mm_deg": target}
            )
            return
        # 官方事件是主完成信号；同时轮询实际位姿与当前规划类型。部分控制器固件
        # 不会上报 rm_movel_offset 到位事件，但 TCP 已经到位，不能因此一直占用动作锁。
        self._monitor_cartesian(
            action_id, start_pose, target, timeout_seconds, controller_event=True
        )

    def _controller_wait_deadline_seconds(self, current, target, speed_percent):
        distance_mm = math.sqrt(sum((a - b) ** 2 for a, b in zip(current[:3], target[:3])))
        speed_mm_s = 600.0 * speed_percent / 100.0
        return min(self.max_motion_seconds, max(15.0, distance_mm / speed_mm_s * 4.0 + 10.0))

    def _plan_target(self, motion_type, args, current):
        """纯计算：校验参数并返回监控用的基系绝对目标位姿（毫米/度），不做任何 SDK 调用。"""
        if motion_type == "movel":
            target = self._pose_from_fields(args, ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg"))
        elif motion_type == "move_offset":
            offset_mm_deg = self._pose_from_fields(
                args, ("dx_mm", "dy_mm", "dz_mm", "drx_deg", "dry_deg", "drz_deg"), empty_value=0.0
            )
            frame = args.get("frame_type", "tool")
            if frame != "tool":
                raise ValueError("frame_type must be 'tool'（工作坐标系偏移暂不支持）")
            # 工具系偏移 ≠ 基系直接相加：平移需按当前工具姿态旋转、姿态右乘组合
            target = self._compose_tool_offset(current, offset_mm_deg)
        elif motion_type == "movep":
            waypoints = args.get("waypoints")
            if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 20:
                raise ValueError("waypoints must be a list of 1~20 poses")
            poses = [self._waypoint_pose(item, index) for index, item in enumerate(waypoints)]
            for pose in poses:
                self._validate_workspace(pose)
            return poses[-1]
        else:
            raise ValueError(f"unknown motion type: {motion_type}")
        self._validate_workspace(target)
        return target

    def _validate_workspace(self, pose_mm_deg):
        x, y, z, rx, ry, rz = pose_mm_deg
        # 直接校验 TCP 本身，不从姿态反推法兰位置：控制器上报的欧拉角
        # 不编码物理工具轴方向（真机竖直位姿上报 (0,0,0) 而夹爪竖直伸出），
        # 按姿态回退 tool_length 会把法兰算到错误方向，造成可达点被误拒。
        # 因此把 tool_length 作为包络余量：夹爪沿任意方向伸出都不会被误杀。
        horizontal_radius = math.sqrt(x * x + y * y)
        if horizontal_radius > self.max_radius_mm + self.tool_length_mm:
            raise ValueError(
                f"pose horizontal radius {horizontal_radius:.0f} mm exceeds cartesian.max_radius_mm "
                f"{self.max_radius_mm:g} + tool_length_mm {self.tool_length_mm:g}")
        # 竖直方向用肩关节几何模型校验：以肩部为球心、臂展+工具长度为半径。
        # 这是仿人构型的自然约束，覆盖「竖直臂展大、水平半径小」的真实工作空间。
        # 真实可达性由控制器逆解兜底；本校验只负责在明显不可达时提前给出清晰报错。
        shoulder_radius = math.sqrt(x * x + y * y + (z - self.shoulder_height_mm) ** 2)
        if shoulder_radius > self.max_reach_mm + self.tool_length_mm:
            raise ValueError(
                f"pose distance {shoulder_radius:.0f} mm from shoulder exceeds cartesian.max_reach_mm "
                f"{self.max_reach_mm:g} + tool_length_mm {self.tool_length_mm:g}")
        for value, label in ((rx, "rx"), (ry, "ry"), (rz, "rz")):
            if abs(value) > self.max_euler_abs_deg:
                raise ValueError(
                    f"{label} {value:.0f} deg exceeds cartesian.max_euler_abs_deg {self.max_euler_abs_deg:g}")

    def _submit(self, motion_type, args, target, speed_percent, wait_for_completion=False):
        """下发 SDK 运动命令；原生相对运动通过控制器事件等待终态。"""
        try:
            command = self.client.command_trajectory if wait_for_completion else self.client.command
            # command_trajectory 已准备到位回调，实际 SDK 调用必须保持非阻塞。
            block = 0
            if motion_type == "movel":
                pose = self._pose_from_fields(args, ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg"))
                command("rm_movel", self._to_sdk_pose(pose), speed_percent, 0, 0, block)
            elif motion_type == "move_offset":
                offset = self._pose_from_fields(
                    args, ("dx_mm", "dy_mm", "dz_mm", "drx_deg", "dry_deg", "drz_deg"), empty_value=0.0
                )
                if self.native_tool_offset_enabled:
                    try:
                        # frame_type=1 表示工具坐标系。控制器据当前关节构型直接规划，
                        # 不再把驱动计算的绝对目标当作实际运动命令。
                        command(
                            "rm_movel_offset", self._to_sdk_pose(offset), speed_percent, 0, 0, 1, block
                        )
                    except RuntimeError as exc:
                        # API2 约定 -7 表示三代控制器不支持 rm_movel_offset；该错误未
                        # 下发运动，可安全退回绝对位姿兼容路径。
                        if "code -7" not in str(exc):
                            raise
                        command("rm_movel", self._to_sdk_pose(target), speed_percent, 0, 0, block)
                else:
                    command("rm_movel", self._to_sdk_pose(target), speed_percent, 0, 0, block)
            elif motion_type == "movep":
                poses = [self._waypoint_pose(item, index) for index, item in enumerate(args["waypoints"])]
                for pose in poses[:-1]:
                    self.client.command("rm_movel", self._to_sdk_pose(pose), speed_percent, 0, 1, 0)
                self.client.command("rm_movel", self._to_sdk_pose(poses[-1]), speed_percent, 0, 0, 0)
            else:
                raise ValueError(f"unknown motion type: {motion_type}")
        except RuntimeError as exc:
            if "code -4" in str(exc):
                raise RuntimeError(
                    "控制器到位设备校验失败（SDK -4）：请在 RealMan Studio/控制器中将当前到位设备设为笛卡尔设备；"
                    "同时确认没有其他客户端占用运动通道"
                ) from exc
            raise

    def _pose_from_fields(self, args, fields, empty_value=None):
        pose = []
        for field in fields:
            value = args.get(field)
            if empty_value is not None and (value is None or (isinstance(value, str) and not value.strip())):
                pose.append(float(empty_value))
                continue
            try:
                numeric = float(value)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{field} must be a number") from exc
            if not math.isfinite(numeric):
                raise ValueError(f"{field} must be finite")
            pose.append(numeric)
        return pose

    @staticmethod
    def _waypoint_pose(item, index):
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

    @staticmethod
    def _euler_to_matrix(rx_deg, ry_deg, rz_deg):
        """ZYX 欧拉角（度）→ 旋转矩阵，R = Rz·Ry·Rx。"""
        rx, ry, rz = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)
        return [
            [cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz],
            [cy * sz, cx * cz + sx * sy * sz, -cz * sx + cx * sy * sz],
            [-sy, cy * sx, cx * cy],
        ]

    @staticmethod
    def _matrix_to_euler(matrix):
        sy = max(-1.0, min(1.0, -matrix[2][0]))
        ry = math.degrees(math.asin(sy))
        if abs(sy) < 0.999999:
            rx = math.degrees(math.atan2(matrix[2][1], matrix[2][2]))
            rz = math.degrees(math.atan2(matrix[1][0], matrix[0][0]))
        else:
            rz = 0.0
            rx = math.degrees(math.atan2(-matrix[0][1], matrix[1][1]))
        return [rx, ry, rz]

    @staticmethod
    def _mat_mul(left, right):
        return [[sum(left[i][k] * right[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    @staticmethod
    def _mat_vec(matrix, vector):
        return [sum(matrix[i][k] * vector[k] for k in range(3)) for i in range(3)]

    @classmethod
    def _compose_tool_offset(cls, current_mm_deg, offset_mm_deg):
        """工具系偏移 → 基系绝对目标：平移按当前工具姿态旋转，姿态右乘组合。"""
        cx, cy, cz, crx, cry, crz = current_mm_deg
        dx, dy, dz, drx, dry, drz = offset_mm_deg
        r_cur = cls._euler_to_matrix(crx, cry, crz)
        tx, ty, tz = cls._mat_vec(r_cur, (dx, dy, dz))
        r_off = cls._euler_to_matrix(drx, dry, drz)
        nx, ny, nz = cls._matrix_to_euler(cls._mat_mul(r_cur, r_off))
        return [cx + tx, cy + ty, cz + tz, nx, ny, nz]

    def _motion_deadline_seconds(self, current, target, speed_percent):
        distance_mm = math.sqrt(sum((a - b) ** 2 for a, b in zip(current[:3], target[:3])))
        # RM75 最大直线速度按 600 mm/s 粗估，速度百分比按比例折算，留 3 倍余量；
        # stall 检测是真正的安全网，此估算只用于给 ACP 完成窗口一个上界。
        speed_mm_s = 600.0 * speed_percent / 100.0
        return min(self.max_motion_seconds, max(30.0, distance_mm / speed_mm_s * 3.0 + 10.0))

    def _monitor_cartesian(
            self, action_id, start_pose, target, max_duration, controller_event=False):
        started = time.monotonic()
        deadline = started + max_duration
        last_progress = started + self.start_grace_seconds
        best_position_error = None
        motion_observed = False
        status, result = "error", {"reason": "unknown"}
        try:
            while time.monotonic() < deadline:
                with self._action_lock:
                    cancelled = action_id in self._cancelled
                if cancelled:
                    status, result = "cancelled", {"reason": "stopmotion"}
                    break
                if controller_event:
                    trajectory_state = self.client.poll_trajectory(0.0)
                    if trajectory_state is True:
                        status = "completed"
                        result = {
                            "reason": "controller_target_reached",
                            "target_pose_mm_deg": target,
                            "elapsed_seconds": time.monotonic() - started,
                        }
                        break
                    if trajectory_state is False:
                        result = {
                            "reason": "controller reported trajectory planning or execution failure",
                            "target_pose_mm_deg": target,
                            "elapsed_seconds": time.monotonic() - started,
                        }
                        break
                if self._arm is not None:
                    self._arm._preflight()
                current = self._current_pose_mm_deg()
                position_error, euler_error = self._pose_error(current, target)
                now = time.monotonic()
                if position_error <= self.position_tolerance_mm and euler_error <= self.euler_tolerance_deg:
                    status = "completed"
                    result = {"reason": "pose_target_reached",
                              "target_pose_mm_deg": target, "actual_pose_mm_deg": current,
                              "position_error_mm": position_error, "euler_error_deg": euler_error,
                              "elapsed_seconds": now - started}
                    break
                start_position_error, start_euler_error = self._pose_error(current, start_pose)
                if (start_position_error >= self.progress_threshold_mm
                        or start_euler_error >= self.euler_progress_threshold_deg):
                    motion_observed = True
                # 原生 rm_movel_offset 由控制器根据关节构型规划。若控制器的 TCP
                # 欧拉角表达与驱动组合结果略有不同，不能因差几毫米一直等待。
                trajectory_type = self._controller_trajectory_type()
                if motion_observed and trajectory_type == 0:
                    status = "completed"
                    result = {
                        "reason": "controller_trajectory_finished",
                        "target_pose_mm_deg": target,
                        "actual_pose_mm_deg": current,
                        "position_error_mm": position_error,
                        "euler_error_deg": euler_error,
                        "elapsed_seconds": now - started,
                    }
                    break
                # 控制器可能在 SDK 命令返回成功后才发现该位姿无逆解。此时不会
                # 产生轨迹、TCP 也不会移动；启动宽限期过后应立刻结束为 error，
                # 不能继续占用动作锁直到 stall 超时。
                if (not motion_observed and trajectory_type == 0
                        and now >= started + self.start_grace_seconds):
                    result = {
                        "reason": "controller_rejected_trajectory",
                        "target_pose_mm_deg": target,
                        "actual_pose_mm_deg": current,
                        "position_error_mm": position_error,
                        "euler_error_deg": euler_error,
                        "elapsed_seconds": now - started,
                    }
                    break
                # 进度检测必须同时看位置与姿态：纯旋转运动位置误差恒为 0，
                # 只看位置会把正常旋转误判为 stall 而中途慢停。
                progress = False
                if best_position_error is None:
                    best_position_error = position_error
                    best_euler_error = euler_error
                    progress = True
                else:
                    if best_position_error - position_error >= self.progress_threshold_mm:
                        best_position_error = position_error
                        progress = True
                    if best_euler_error - euler_error >= self.euler_progress_threshold_deg:
                        best_euler_error = euler_error
                        progress = True
                if progress:
                    last_progress = now
                elif (now >= started + self.start_grace_seconds
                        and now - last_progress >= self.stall_timeout_seconds):
                    self._request_slow_stop()
                    result = {"reason": "motion_stalled",
                              "stall_seconds": self.stall_timeout_seconds,
                              "target_pose_mm_deg": target, "actual_pose_mm_deg": current,
                              "position_error_mm": position_error, "euler_error_deg": euler_error,
                              "elapsed_seconds": now - started}
                    break
                time.sleep(self.poll_interval_seconds)
            else:
                self._request_slow_stop()
                result = {"reason": "motion_deadline_exceeded",
                          "max_motion_seconds": max_duration,
                          "elapsed_seconds": time.monotonic() - started}
        except Exception as exc:
            self._request_slow_stop()
            result = {"reason": str(exc)}
        finally:
            if controller_event:
                discard_wait = getattr(self.client, "discard_trajectory_wait", None)
                if callable(discard_wait):
                    discard_wait()
            self._finish_cartesian(action_id, status, result)

    def _finish_cartesian(self, action_id, status, result):
        release_motion_lock = False
        with self._action_lock:
            if action_id in self._terminal_action_ids:
                return False
            if (self._active_action_id != action_id
                    and self._motion_state["active_action_id"] != action_id):
                return False
            self._terminal_action_ids.add(action_id)
            if action_id in self._cancelled:
                status, result = "cancelled", {"reason": "stopmotion"}
            self._cancelled.discard(action_id)
            if self._active_action_id == action_id:
                self._active_action_id = None
            if self._motion_state["active_action_id"] == action_id:
                self._motion_state["active_action_id"] = None
            self._last_completion = {"action_id": action_id, "status": status, "result": dict(result)}
            release_motion_lock = self._motion_lock.locked()
        if release_motion_lock:
            self._motion_lock.release()
        self._acp_callback(action_id, status, result)
        return True

    def _controller_trajectory_type(self):
        """返回 API2 当前规划类型；不支持或查询失败时保留位姿监控。"""
        try:
            state = self.client.call_dict("rm_get_arm_current_trajectory")
            return int(state.get("trajectory_type"))
        except (AttributeError, TypeError, ValueError, RuntimeError):
            return None

    def _request_slow_stop(self):
        """在独立线程请求控制器慢停，避免停止接口拖住 MCP 响应。"""
        if not self.client.connected:
            return None

        def worker():
            # 运动使用 block=0；停止走同一个串行 SDK 入口，避免并发访问句柄。
            interrupt = getattr(self.client, "command_interrupt", None)
            if callable(interrupt):
                try:
                    interrupt("rm_set_arm_slow_stop")
                except Exception as exc:
                    print(f"[rm75] cartesian slow-stop failed: {exc}", flush=True)
                return
            # 兼容不提供独立停止通道的旧客户端。
            with self._submission_lock:
                try:
                    self.client.command("rm_set_arm_slow_stop")
                except Exception as exc:
                    print(f"[rm75] cartesian slow-stop failed: {exc}", flush=True)

        thread = threading.Thread(target=worker, daemon=True, name="rm75-cartesian-slow-stop")
        thread.start()
        return thread

    def _stop_motion(self):
        # 运动锁和动作 ID 在两张卡之间共享；任一 stop 卡都必须能停止实际持有者。
        # SDK 慢停调用无超时上限，不得在 _action_lock 内执行。
        with self._action_lock:
            action_id = self._motion_state["active_action_id"] or self._active_action_id
            owns_action = self._active_action_id == action_id
            if action_id:
                self._cancelled.add(action_id)
        if action_id and self.client.connected:
            self._request_slow_stop()
        if action_id and owns_action:
            self._finish_after_stop(action_id, "cancelled", {"reason": "stopmotion"})
        return {"state": "stop_requested", "action_id": action_id}

    def _finish_after_stop(self, action_id, status, result):
        """控制器未返回停止事件时，解除事件等待并保证 ACP 有终态。"""
        def worker():
            time.sleep(self.stop_finalize_seconds)
            cancel_wait = getattr(self.client, "cancel_trajectory_wait", None)
            if callable(cancel_wait):
                cancel_wait()
            monitor = self._monitor_thread
            if monitor is not None and monitor is not threading.current_thread():
                monitor.join(self.poll_interval_seconds * 5 + CARTESIAN_STOP_JOIN_MARGIN)
            self._finish_cartesian(action_id, status, result)

        threading.Thread(
            target=worker,
            daemon=True,
            name="rm75-cartesian-stop-finalize",
        ).start()

    def _acp_callback(self, action_id, status, result):
        outcome, error = _acp_complete(action_id, status, result, self.PREFIX)
        retrying = outcome != "accepted" and self.ACP_RETRY_ATTEMPTS > 0
        with self._action_lock:
            if self._last_completion and self._last_completion.get("action_id") == action_id:
                self._last_completion["callback"] = "retrying" if retrying else outcome
                if error is not None:
                    self._last_completion["callback_error"] = error
        if retrying:
            threading.Thread(
                target=self._retry_acp_callback,
                args=(action_id, status, dict(result)),
                daemon=True,
                name="rm75-cartesian-acp-retry",
            ).start()

    def _retry_acp_callback(self, action_id, status, result):
        for _ in range(self.ACP_RETRY_ATTEMPTS):
            time.sleep(self.ACP_RETRY_DELAY_SECONDS)
            outcome, error = _acp_complete(action_id, status, result, self.PREFIX)
            with self._action_lock:
                if not self._last_completion or self._last_completion.get("action_id") != action_id:
                    return
                self._last_completion["callback"] = outcome
                if error is None:
                    self._last_completion.pop("callback_error", None)
                else:
                    self._last_completion["callback_error"] = error
            if outcome == "accepted":
                return


def build_plugins(config, namespace, ros2):
    client = RM75SDKClient(config)
    arm = RM75Plugin(client, config, namespace=namespace, ros2=ros2)
    plugins = [
        arm,
        GripperPlugin(client, config, namespace=namespace, ros2=ros2),
        CartesianPlugin(client, config, arm_plugin=arm, namespace=namespace, ros2=ros2),
    ]
    external_camera = None
    camera_config = config.get("ext_camera", {})
    if camera_config.get("enabled", False):
        from camera import ExtCameraPlugin

        external_camera = ExtCameraPlugin(camera_config, namespace, ros2.executor_core)
        plugins.append(external_camera)
    capture_config = config.get("vision_capture", {})
    if capture_config.get("enabled", False):
        from vision_capture import VisionCapturePlugin

        plugins.append(VisionCapturePlugin(
            capture_config, namespace, ros2.executor_core, external_camera))
    return plugins
