#!/usr/bin/env python3
"""RealMan RM75-6F-V MCP Driver using the official Python API2 SDK."""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from uuid import uuid4
from pathlib import Path

from common.vendor_runtime import action_schema, jsonable, tool


try:
    import hardware
except ModuleNotFoundError as exc:
    if exc.name != "hardware":
        raise
    # File-based loaders expose the repository root, not the driver directory.
    from realman.rm75_6f_v import hardware
    # Reuse this module when sibling cards import the normal runtime name.
    sys.modules["hardware"] = hardware

from hardware import (
    JOINT_NAMES, JOINT_LIMITS_DEG, JOINT_MAX_SPEED_DEG_S, RM75SDKClient,
)


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
        self._motion_lock = client.motion_gate
        self._motion_state = {"active_action_id": None}
        # 提交/慢停串行化锁：与 CartesianPlugin 共享，保证任一卡片的 stopmotion
        # 都排在另一卡片正在进行的 SDK 运动下发之后。
        self._submission_lock = threading.Lock()
        self._action_lock = threading.Lock()
        self._active_action_id = None
        self._cancelled = set()
        self._last_completion = None
        self._cartesian_pose_invalidator = None
        self._cartesian_stop_finalizer = None

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
            tool("model", "resource", "RM75-6F-V URDF for skeleton rendering"),
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
        schema["x-resource"] = "arm"
        schema["x-is-dangerous"] = True
        definitions.append(tool("joint_control", "actuator", "Bounded RM75 joint motion using official API2 movej", schema))
        return definitions

    def start(self):
        try:
            self.client.start()
        finally:
            # Keep the state publisher available after a transient startup
            # connection failure.  A later tool request can reconnect the
            # shared client, after which this existing publisher resumes.
            if self._ros2 is not None:
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
            "angle_unit": "deg",
            "joint_count": len(JOINT_NAMES),
            "joints": [
                {
                    "idx": index,
                    "name": name,
                    "q": float(position),
                    "degree": float(state["raw_degree"][index]),
                }
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
            skeleton = self._skeleton_payload()
            message.data = json.dumps(skeleton, ensure_ascii=False)
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
            # 关节运动会改变 TCP 位姿，笛卡尔偏移不能继续沿用此前确认的目标。
            if callable(self._cartesian_pose_invalidator):
                self._cartesian_pose_invalidator()
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
        # joint_control and abs_move share the action ID and motion lock, but
        # each card owns its own ACP completion state.  Once the physical stop
        # has been ordered, let abs_move finalize an action that it owns.
        if action_id and callable(self._cartesian_stop_finalizer):
            self._cartesian_stop_finalizer(action_id)
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
            result = self.client.call(self.METHODS[name])
            return result
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
GRIPPER_POLL_INTERVAL_SECONDS = 0.15
GRIPPER_POSITION_TOLERANCE = 5
GRIPPER_PROGRESS_THRESHOLD = 5
GRIPPER_DEFAULT_STALL_TIMEOUT_SECONDS = 3.0
GRIPPER_FORCE_MIN = 50
GRIPPER_FORCE_MAX = 1000
GRIPPER_DEFAULT_FORCE = 200
GRIPPER_MOVING_MODES = frozenset((4, 5))
GRIPPER_TERMINAL_MODES = frozenset((1, 2, 3, 6))
CARTESIAN_STOP_JOIN_MARGIN = 10  # 笛卡尔 stop 等待监控线程收尾的额外余量（秒）


def _gripper_position(value) -> int:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("position must be a number") from exc
    if not math.isfinite(numeric) or not GRIPPER_POSITION_MIN <= numeric <= GRIPPER_POSITION_MAX:
        raise ValueError(f"position must be within {GRIPPER_POSITION_MIN}~{GRIPPER_POSITION_MAX}")
    return int(round(numeric))


def _gripper_stall_timeout(value) -> float:
    if value is None:
        return GRIPPER_DEFAULT_STALL_TIMEOUT_SECONDS
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("stall_timeout_seconds must be a number") from exc
    if not math.isfinite(numeric) or not 0.5 <= numeric <= GRIPPER_COMPLETION_TIMEOUT:
        raise ValueError(
            f"stall_timeout_seconds must be within 0.5~{GRIPPER_COMPLETION_TIMEOUT}"
        )
    return numeric


def _gripper_force(value) -> int:
    if value is None:
        return GRIPPER_DEFAULT_FORCE
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("force must be a number") from exc
    if not math.isfinite(numeric) or not GRIPPER_FORCE_MIN <= numeric <= GRIPPER_FORCE_MAX:
        raise ValueError(
            f"force must be within {GRIPPER_FORCE_MIN}~{GRIPPER_FORCE_MAX}"
        )
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
        self._last_state = None
        self._last_target_position = GRIPPER_POSITION_MAX

    def get_tools(self):
        schema = action_schema(
            {
                "set_position": (
                    ["position", "on_stall", "stall_timeout_seconds", "confirm_motion"],
                    "设置二指夹爪目标位置，并指定受阻时保持当前位置或返回起点",
                ),
                "info": ([], "读取夹爪与 SDK 连接状态"),
            },
            {
                "position": {
                    "type": "integer",
                    "minimum": GRIPPER_POSITION_MIN,
                    "maximum": GRIPPER_POSITION_MAX,
                    "description": f"夹爪驱动器目标位置，{GRIPPER_POSITION_MIN}~{GRIPPER_POSITION_MAX}，对应 0~120 mm 行程",
                },
                "on_stall": {
                    "type": "string",
                    "enum": ["keep", "rollback"],
                    "default": "keep",
                    "description": "夹爪连续无进展时：keep 保持当前实际位置；rollback 返回运动前实际位置",
                },
                "stall_timeout_seconds": {
                    "type": "number",
                    "minimum": 0.5,
                    "maximum": GRIPPER_COMPLETION_TIMEOUT,
                    "default": GRIPPER_DEFAULT_STALL_TIMEOUT_SECONDS,
                    "description": "连续无明显位置进展多久后判定受阻（秒）",
                },
                "force": {
                    "type": "integer",
                    "minimum": GRIPPER_FORCE_MIN,
                    "maximum": GRIPPER_FORCE_MAX,
                    "default": GRIPPER_DEFAULT_FORCE,
                    "description": "力控夹持阈值 (50~1000)，在受阻停止或力控夹取时生效",
                },
                "confirm_motion": {"type": "boolean", "description": "Must be true for every movement request"},
            },
        )
        # 最坏情况下，原始动作和安全恢复各自需要一个完整确认窗口。
        schema["x-completion"] = {
            "actions": ["set_position"],
            "timeout": GRIPPER_COMPLETION_TIMEOUT * 2 + 10,
        }
        schema["x-resource"] = "arm"
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
        # SDK 没有夹爪中途停止 API：请求停止时通知 worker，且只有确认
        # worker 已结束后才允许共享 SDK 连接被上层销毁。
        self._mark_interrupted()
        if not self._wait_for_worker():
            raise RuntimeError("gripper worker is still active; refusing teardown")

    def _mark_interrupted(self):
        with self._action_lock:
            action_id = self._active_action_id
            if action_id:
                self._interrupted.add(action_id)

    def _wait_for_worker(self):
        thread = self._worker_thread
        if thread is not None and thread.is_alive():
            thread.join(GRIPPER_COMPLETION_TIMEOUT * 2 + GRIPPER_STOP_WAIT_MARGIN)
        return thread is None or not thread.is_alive()

    def dispatch(self, action, args):
        if action == "info":
            with self._action_lock:
                active = self._active_action_id
                last_completion = dict(self._last_completion) if self._last_completion else None
                gripper_state = dict(self._last_state) if self._last_state else None
            # 空闲时主动刷新；运动期间由 worker 持续更新缓存，避免 info 与一个
            # 卡住的 SDK 命令争用底层连接锁。
            if active is None and self.client.connected:
                try:
                    gripper_state = self._read_gripper_state()
                except Exception as exc:
                    return {
                        "state": "error",
                        "active_action_id": active,
                        "last_completion": last_completion,
                        "gripper_state": gripper_state,
                        "state_error": str(exc),
                    }
            return {
                "state": "connected" if self.client.connected else "disconnected",
                "active_action_id": active,
                "last_completion": last_completion,
                "gripper_state": gripper_state,
            }
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self._mark_interrupted()
            if not self._wait_for_worker():
                with self._action_lock:
                    active = self._active_action_id
                return {
                    "state": "busy",
                    "active_action_id": active,
                    "reason": "gripper worker did not stop before the safety deadline",
                }
            return {"state": "idle"}
        if action != "set_position":
            return None
        if not self.client.motion_enabled:
            raise PermissionError("motion is locked; set RM_MOTION_ENABLED=1 only for supervised hardware testing")
        if args.get("confirm_motion") is not True:
            raise ValueError("confirm_motion must be true")
        position = _gripper_position(args.get("position"))
        on_stall = args.get("on_stall", "keep")
        if on_stall not in ("keep", "rollback"):
            raise ValueError("on_stall must be keep or rollback")
        stall_timeout = _gripper_stall_timeout(args.get("stall_timeout_seconds"))
        force = _gripper_force(args.get("force"))
        if not self._gripper_lock.acquire(blocking=False):
            raise RuntimeError(f"another gripper motion is active: {self._active_action_id}")
        if not self.client.motion_gate.acquire(blocking=False):
            self._gripper_lock.release()
            raise RuntimeError("another arm operation is active")
        action_id = f"rm75_gripper_{uuid4().hex[:10]}"
        with self._action_lock:
            self._active_action_id = action_id
            self._interrupted.discard(action_id)
        self._worker_thread = threading.Thread(
            target=self._gripper_worker,
            args=(action_id, position, on_stall, stall_timeout, force),
            daemon=True,
        )
        self._worker_thread.start()
        print(f"[rm75 ACP] {action_id}: started", flush=True)
        return {"state": "running", "action_id": action_id}

    def _read_gripper_state(self):
        # 1. 优先尝试原生因时夹爪接口 rm_get_gripper_state
        state = None
        try:
            state = self.client.call("rm_get_gripper_state")
        except Exception:
            state = None

        if isinstance(state, dict) and self._gripper_feedback_available(state):
            try:
                actual = float(state["actpos"])
                current_force = float(state.get("current_force", 0))
                mode = int(state.get("mode", 0))
                error = int(state.get("error", 0))
                if math.isfinite(actual) and math.isfinite(current_force):
                    normalized = {
                        **state,
                        "actpos": int(round(actual)),
                        "current_force": current_force,
                        "mode": mode,
                        "error": error,
                    }
                    with self._action_lock:
                        self._last_state = dict(normalized)
                    return normalized
            except (KeyError, TypeError, ValueError):
                pass

        # 2. 若原生接口不可用或返回全零占位（如真机安装的是 RM_ARM+ 生态夹爪/知行夹爪）
        # 则尝试末端生态协议 rm_get_rm_plus_state_info
        try:
            plus_state = self.client.call("rm_get_rm_plus_state_info")
            if isinstance(plus_state, dict) and isinstance(plus_state.get("pos"), list) and plus_state["pos"]:
                actual = float(plus_state["pos"][0])
                if math.isfinite(actual):
                    dof_state = plus_state.get("dof_state", [0])[0] if plus_state.get("dof_state") else 0
                    speed = abs(float(plus_state.get("speed", [0])[0])) if plus_state.get("speed") else 0.0
                    force = float(plus_state.get("force", [0])[0]) if plus_state.get("force") else 0.0
                    dof_err = int(plus_state.get("dof_err", [0])[0]) if plus_state.get("dof_err") else 0
                    # dof_state: 2 为到位/锁定状态，无速度时视为 terminal mode 3；否则为运动中模式 4
                    mode = 3 if (dof_state == 2 or speed <= 1) else 4
                    normalized = {
                        "enable_state": 1,
                        "status": 1,
                        "actpos": int(round(actual)),
                        "current_force": force,
                        "mode": mode,
                        "error": dof_err,
                        "temperature": 0,
                    }
                    with self._action_lock:
                        self._last_state = dict(normalized)
                    return normalized
        except Exception:
            pass

        # 3. 若均无法解析出有效遥测，按原逻辑处理 state（有效 dict 或抛出异常）
        if not isinstance(state, dict):
            raise RuntimeError(f"invalid gripper state: {state!r}")
        try:
            actual = float(state["actpos"])
            current_force = float(state.get("current_force", 0))
            mode = int(state.get("mode", 0))
            error = int(state.get("error", 0))
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"invalid gripper state: {state!r}") from exc
        if not math.isfinite(actual) or not math.isfinite(current_force):
            raise RuntimeError(f"invalid gripper state: {state!r}")
        normalized = {
            **state,
            "actpos": int(round(actual)),
            "current_force": current_force,
            "mode": mode,
            "error": error,
        }
        with self._action_lock:
            self._last_state = dict(normalized)
        return normalized

    @staticmethod
    def _gripper_is_terminal(state):
        return state["mode"] in GRIPPER_TERMINAL_MODES or bool(state["error"] & 1)

    @staticmethod
    def _gripper_feedback_available(state):
        """Return false for the all-zero placeholder returned by some firmware.

        A successful rm_get_gripper_state call can still contain an entirely
        zeroed structure when live gripper telemetry is unavailable.  Treating
        actpos=0 from that placeholder as a measured position can command an
        unexpected close, so it must never enter measured-position recovery.
        """
        return any((
            state.get("enable_state", 0),
            state.get("status", 0),
            state.get("error", 0),
            state.get("mode", 0),
            state.get("current_force", 0),
            state.get("temperature", 0),
            state.get("actpos", 0),
        ))

    def _is_interrupted(self, action_id):
        with self._action_lock:
            return action_id in self._interrupted

    def _wait_for_gripper(
            self, action_id, target, stall_timeout, deadline, *, allow_interrupt=True):
        state = self._read_gripper_state()
        best_error = abs(state["actpos"] - target)
        last_progress = time.monotonic()
        while True:
            now = time.monotonic()
            error = abs(state["actpos"] - target)
            interrupted = self._is_interrupted(action_id)
            if allow_interrupt and interrupted:
                return "interrupted", state
            if error <= GRIPPER_POSITION_TOLERANCE:
                return "reached", state
            if state["error"] & ~1:
                raise RuntimeError(f"gripper error 0x{state['error']:x}")
            if best_error - error >= GRIPPER_PROGRESS_THRESHOLD:
                best_error = error
                last_progress = now
            elif (state["mode"] not in GRIPPER_MOVING_MODES
                    and now - last_progress >= stall_timeout):
                return "stalled", state
            if now >= deadline:
                return "timed_out", state
            time.sleep(GRIPPER_POLL_INTERVAL_SECONDS)
            state = self._read_gripper_state()

    @staticmethod
    def _safe_gripper_position(position):
        return max(GRIPPER_POSITION_MIN, min(GRIPPER_POSITION_MAX, int(round(position))))

    def _gripper_worker(self, action_id, position, on_stall, stall_timeout, force=GRIPPER_DEFAULT_FORCE):
        started = time.monotonic()
        start_position = None
        command_started = False
        try:
            initial = self._read_gripper_state()
            if self._gripper_feedback_available(initial):
                start_position = initial["actpos"]
            else:
                start_position = self._last_target_position or GRIPPER_POSITION_MAX
            # stop 可能在线程启动后、首条 SDK 命令前到达。此时没有在途动作，
            # 直接取消即可；绝不能先发送用户目标再尝试恢复。
            if self._is_interrupted(action_id):
                status, result = "cancelled", {
                    "reason": "interrupted_before_start",
                    "requested_position": position,
                    "start_position": start_position,
                    "final_position": start_position,
                    "current_force": initial["current_force"],
                    "mode": initial["mode"],
                    "error": initial["error"],
                    "stall_policy": on_stall,
                    "rollback_succeeded": None,
                    "elapsed_seconds": time.monotonic() - started,
                }
            elif not self._gripper_feedback_available(initial):
                # 部分控制器/固件会让查询成功但返回全零结构。此时无法获取
                # 实时位移做轮询，使用带 stall_timeout 的阻塞模式下发命令。
                command_started = True
                timeout_int = max(1, int(math.ceil(stall_timeout)))
                stalled = False
                try:
                    self.client.command(
                        "rm_set_gripper_position",
                        position,
                        True,
                        timeout_int,
                    )
                except Exception as exc:
                    err_msg = str(exc)
                    if "code -4" in err_msg or "code 1" in err_msg:
                        stalled = True
                    else:
                        raise exc

                if not stalled:
                    self._last_target_position = position
                    status, result = "completed", {
                        "reason": "sdk_command_completed_without_feedback",
                        "requested_position": position,
                        "start_position": start_position,
                        "final_position": position,
                        "feedback_available": False,
                        "current_force": initial["current_force"],
                        "mode": initial["mode"],
                        "error": initial["error"],
                        "stall_policy": on_stall,
                        "rollback_succeeded": None,
                        "elapsed_seconds": time.monotonic() - started,
                    }
                elif on_stall == "keep":
                    # 方案 B：受阻时下发力控夹取 rm_set_gripper_pick。
                    # 由于夹爪已压在障碍物上，受力达到 force 阈值后，
                    # 机械臂控制箱底层自动停止驱动并锁定，实现保持在极限位置。
                    try:
                        self.client.command("rm_set_gripper_pick", 100, force, False, 1)
                    except Exception as pick_err:
                        print(f"[rm75 gripper] force stop attempt: {pick_err}", flush=True)
                    status, result = "completed", {
                        "reason": "motion_stalled_kept",
                        "requested_position": position,
                        "start_position": start_position,
                        "blocked_position": None,
                        "final_position": None,
                        "feedback_available": False,
                        "current_force": initial["current_force"],
                        "mode": initial["mode"],
                        "error": initial["error"],
                        "stall_policy": "keep",
                        "rollback_succeeded": None,
                        "elapsed_seconds": time.monotonic() - started,
                    }
                else:  # on_stall == "rollback"
                    recovery_target = start_position
                    rollback_ok = True
                    try:
                        self.client.command(
                            "rm_set_gripper_position",
                            recovery_target,
                            True,
                            GRIPPER_COMPLETION_TIMEOUT,
                        )
                        self._last_target_position = recovery_target
                    except Exception:
                        rollback_ok = False
                    status = "completed" if rollback_ok else "error"
                    result = {
                        "reason": "motion_stalled_rolled_back" if rollback_ok else "motion_stalled_rollback_failed",
                        "requested_position": position,
                        "start_position": start_position,
                        "blocked_position": None,
                        "final_position": recovery_target if rollback_ok else None,
                        "feedback_available": False,
                        "current_force": initial["current_force"],
                        "mode": initial["mode"],
                        "error": initial["error"],
                        "stall_policy": "rollback",
                        "rollback_succeeded": rollback_ok,
                        "elapsed_seconds": time.monotonic() - started,
                    }
            else:
                deadline = started + GRIPPER_COMPLETION_TIMEOUT
                # 非阻塞下发后轮询真实位置，才能区分到位、受阻与超时。
                command_started = True
                self.client.command("rm_set_gripper_position", position, False, 1)
                outcome, terminal = self._wait_for_gripper(
                    action_id, position, stall_timeout, deadline
                )
                common = {
                    "requested_position": position,
                    "start_position": start_position,
                    "current_force": terminal["current_force"],
                    "mode": terminal["mode"],
                    "error": terminal["error"],
                    "stall_policy": on_stall,
                    "elapsed_seconds": time.monotonic() - started,
                }
                if outcome == "reached":
                    status, result = "completed", {
                        **common,
                        "reason": "target_reached",
                        "final_position": terminal["actpos"],
                        "rollback_succeeded": None,
                    }
                elif on_stall == "keep":
                    blocked_position = terminal["actpos"]
                    final = terminal
                    # 若控制器明确报告仍在运动（mode 4/5），继续等待其稳定
                    if final["mode"] in GRIPPER_MOVING_MODES:
                        _, final = self._wait_for_gripper(
                            action_id,
                            position,
                            stall_timeout,
                            time.monotonic() + GRIPPER_COMPLETION_TIMEOUT,
                            allow_interrupt=False,
                        )
                    if outcome == "interrupted":
                        reason = "interrupted_kept"
                        status = "cancelled"
                    elif outcome == "timed_out":
                        reason = "motion_timed_out_kept"
                        status = "error"
                    else:
                        reason = "motion_stalled_kept"
                        status = "completed"
                    result = {
                        **common,
                        "reason": reason,
                        "blocked_position": blocked_position,
                        "final_position": final["actpos"],
                        "current_force": final["current_force"],
                        "mode": final["mode"],
                        "error": final["error"],
                        "rollback_succeeded": None,
                        "elapsed_seconds": time.monotonic() - started,
                    }
                else:
                    blocked_position = terminal["actpos"]
                    recovery_target = self._safe_gripper_position(start_position)
                    # The interrupt selected this recovery; do not let the recovery
                    # wait immediately observe the old interrupt marker again.
                    with self._action_lock:
                        self._interrupted.discard(action_id)
                    self.client.command(
                        "rm_set_gripper_position", recovery_target, False, 1
                    )
                    # 原始窗口可能已经耗尽；rollback 使用独立恢复窗口。
                    recovery, final = self._wait_for_gripper(
                        action_id,
                        recovery_target,
                        stall_timeout,
                        time.monotonic() + GRIPPER_COMPLETION_TIMEOUT,
                        allow_interrupt=False,
                    )
                    rollback_succeeded = recovery == "reached"
                    status = (
                        "completed"
                        if rollback_succeeded and outcome != "timed_out"
                        else "error"
                    )
                    if outcome == "interrupted":
                        reason = "interrupted_rolled_back"
                    elif outcome == "timed_out":
                        reason = "motion_timed_out_rolled_back"
                    else:
                        reason = "motion_stalled_rolled_back"
                    if not rollback_succeeded:
                        reason = f"{reason}_failed"
                    result = {
                        **common,
                        "reason": reason,
                        "blocked_position": blocked_position,
                        "final_position": final["actpos"],
                        "current_force": final["current_force"],
                        "mode": final["mode"],
                        "error": final["error"],
                        "rollback_succeeded": rollback_succeeded,
                        "elapsed_seconds": time.monotonic() - started,
                    }
        except Exception as exc:
            status, result = "error", {
                "reason": str(exc),
                "requested_position": position,
                "start_position": start_position,
                "stall_policy": on_stall,
                "elapsed_seconds": time.monotonic() - started,
            }
        finally:
            # 无论何种结果，worker 结束时必须清理 action_id 并释放锁，允许下一次调用
            with self._action_lock:
                if self._active_action_id == action_id:
                    self._active_action_id = None
                self._interrupted.discard(action_id)
            try:
                self._gripper_lock.release()
            except RuntimeError:
                pass
            try:
                self.client.motion_gate.release()
            except RuntimeError:
                pass
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
    if tool_name == "gripper":
        # Keep the completion useful to Canvas while limiting it to bounded,
        # JSON-safe gripper telemetry rather than forwarding arbitrary SDK data.
        for key in (
                "reason", "requested_position", "start_position",
                "blocked_position", "final_position", "stall_policy",
                "rollback_succeeded", "current_force", "mode", "error",
                "elapsed_seconds"):
            if key in result:
                summary[key] = result[key]
        if "reason" in summary:
            summary["reason"] = str(summary["reason"])[:240]
    elif status == "completed":
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
    """绝对位姿运动卡片：从当前 TCP 运动到基坐标系目标位姿。

    位姿单位面向画布：位置毫米、姿态度（SDK 内部为米/弧度，转换封装在插件内）。
    与 joint_control 共享运动锁（同一时刻只允许一个运动流），安全守卫、
    stall 检测与 ACP 异步完成与 joint_control 保持一致。
    """

    PREFIX = "abs_move"
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
        # move_a_to_b records the measured source pose and tracks its controller
        # stage.  _action_lock prevents an event, stop request and watchdog from
        # all finishing the same action.
        self._a_to_b_actions = {}
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
        configured_enabled = cartesian.get("enabled", False) is True
        environment_enabled = os.environ.get("RM_CARTESIAN_ENABLED")
        if environment_enabled is None:
            self.cartesian_enabled = configured_enabled
        elif environment_enabled in ("0", "1"):
            self.cartesian_enabled = environment_enabled == "1"
        else:
            raise ValueError("RM_CARTESIAN_ENABLED must be 0 or 1")
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
        self._native_tool_offset_supported = None if self.native_tool_offset_enabled else False
        # 最近一次由控制器确认完成的 TCP 目标。连续 offset 从这个位姿组合并
        # 校验，避免每次动作都调用部分真机会卡住的 current-arm-state 查询。
        self._confirmed_pose_lock = threading.Lock()
        self._confirmed_pose_mm_deg = None
        self._confirmed_pose_revision = 0
        if arm_plugin is not None:
            arm_plugin._cartesian_pose_invalidator = self._invalidate_confirmed_pose
            arm_plugin._cartesian_stop_finalizer = self._finalize_shared_stop
        self.stop_finalize_seconds = float(cartesian.get("stop_finalize_seconds", 2.0))

    def get_tools(self):
        point_props = {
            f"b_{axis}": {
                "type": "number",
                "description": f"目标 B 点 TCP {label}（基坐标系；留空沿用当前 A 点值）",
            }
            for axis, label in (
                ("x_mm", "X，毫米"),
                ("y_mm", "Y，毫米"),
                ("z_mm", "Z，毫米"),
                ("rx_deg", "Roll，度"),
                ("ry_deg", "Pitch，度"),
                ("rz_deg", "Yaw，度"),
            )
        }
        properties = {
            **point_props,
            "motion_mode": {
                "type": "string",
                "enum": ["joint", "linear"],
                "default": "joint",
                "description": "到 B 的路径：joint 使用 rm_movej_p 关节空间规划；linear 使用 rm_movel TCP 直线",
            },
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
                "move": ([
                    "b_x_mm", "b_y_mm", "b_z_mm", "b_rx_deg", "b_ry_deg", "b_rz_deg",
                    "motion_mode", "speed_percent", "cartesian_enabled", "confirm_motion",
                ], "自动读取当前 TCP 为 A 点，再以关节空间或 TCP 直线运动到基坐标系 B 点"),
                "stopmotion": ([], "请求受控减速停止"),
                "info": ([], "读取运动状态与安全配置"),
            },
            properties,
        )
        schema["x-completion"] = {"actions": ["move"], "timeout": 305}
        schema["x-hooks"] = {
            "on_interrupt_motion": {"action": "stopmotion"},
            "on_interrupt_all": {"action": "stopmotion"},
        }
        schema["x-resource"] = "arm"
        schema["x-is-dangerous"] = True
        return [
            tool(
                "abs_move",
                "actuator",
                "做绝对位置下的移动",
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
        if action == "move":
            return self._start_a_to_b(args)
        if action == "move_offset":
            raise ValueError("move_offset is no longer supported")
        # movel/movep 已从卡片定义中移除；仅保留此处兼容已保存的旧流程。
        # move_offset 已完全移除，手工 MCP 调用也不会再下发相对运动。
        if action in ("movel", "movep"):
            return self._start_cartesian(action, args)
        return None

    def _motion_speed(self, args):
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
        return speed_percent

    def _motion_status(self):
        with self._action_lock:
            active_action_id = self._active_action_id
            last = self._last_completion
            a_to_b_state = self._a_to_b_actions.get(active_action_id)
        trajectory_wait_state = getattr(self.client, "trajectory_wait_state", None)
        return {
            "state": "moving" if active_action_id else "ready",
            "active_action_id": active_action_id,
            "active_stage": a_to_b_state.get("stage") if a_to_b_state else None,
            "active_motion_mode": a_to_b_state.get("motion_mode") if a_to_b_state else None,
            "controller_trajectory_state": (
                trajectory_wait_state() if callable(trajectory_wait_state) else None
            ),
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
        }

    def _start_cartesian(self, motion_type, args):
        speed_percent = self._motion_speed(args)
        controller_completion = self._uses_controller_completion(motion_type)
        if controller_completion:
            offset = self._native_tool_offset(args)
            # Native controller-completion motion must pass the same arm/joint
            # checks as the compatibility path.  Run this before reserving an
            # action or submitting rm_movel_offset so a failed preflight cannot
            # leave a running card or an occupied motion lock.
            if self._arm is not None:
                self._arm._preflight()
            ensure_trajectory_ready = getattr(self.client, "ensure_trajectory_ready", None)
            if callable(ensure_trajectory_ready):
                ensure_trajectory_ready()
            # 原生 rm_movel_offset 仍需在驱动侧校验最终 TCP：只校验偏移长度会
            # 允许一个合法起点越过工作空间边界。首次读取当前 TCP，之后使用
            # 控制器确认完成的目标缓存；查询发生在占用动作锁之前。
            target, pose_revision = self._validated_offset_target(offset)
            controller_timeout = self._controller_offset_wait_deadline_seconds(
                offset, speed_percent
            )
        if not self._motion_lock.acquire(blocking=False):
            active = self._motion_state["active_action_id"] or "joint/cartesian motion"
            raise RuntimeError(f"another motion is active: {active}")
        if controller_completion:
            with self._confirmed_pose_lock:
                pose_changed = pose_revision != self._confirmed_pose_revision
            if pose_changed:
                self._motion_lock.release()
                raise RuntimeError(
                    "Cartesian reference pose changed during workspace validation; retry"
                )
        action_id = f"rm75_cart_{uuid4().hex[:10]}"
        # 状态锁不能覆盖无超时上限的 SDK 调用，否则 info 也会被控制器故障拖死。
        # submission_lock 只负责保证 stopmotion 排在运动下发之后。
        submitted = False
        try:
            with self._action_lock:
                self._active_action_id = action_id
                self._motion_state["active_action_id"] = action_id
                self._cancelled.discard(action_id)
            if not controller_completion:
                with self._submission_lock:
                    try:
                        if self._arm is not None:
                            self._arm._preflight()
                        current = self._current_pose_mm_deg()
                        target = self._plan_target(motion_type, args, current)
                        max_duration = self._motion_deadline_seconds(current, target, speed_percent)
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
                offset,
                target,
                speed_percent,
                controller_timeout,
            )
        else:
            target_fn = self._monitor_cartesian
            target_args = (action_id, current, target, max_duration)
        self._monitor_thread = threading.Thread(target=target_fn, args=target_args, daemon=True)
        self._monitor_thread.start()
        if controller_completion:
            threading.Thread(
                target=self._controller_action_watchdog,
                args=(action_id, offset, controller_timeout),
                daemon=True,
                name=f"rm75-cartesian-watchdog-{action_id}",
            ).start()
        print(f"[rm75 ACP] {action_id}: started ({motion_type})", flush=True)
        return {"state": "running", "action_id": action_id}

    def _start_a_to_b(self, args):
        """Read the current TCP as A, then move from A to the requested B."""
        speed_percent = self._motion_speed(args)
        motion_mode = args.get("motion_mode", "joint")
        if motion_mode not in ("joint", "linear"):
            raise ValueError("motion_mode must be 'joint' or 'linear'")
        point_fields = (
            "b_x_mm", "b_y_mm", "b_z_mm", "b_rx_deg", "b_ry_deg", "b_rz_deg"
        )
        supplied_fields = [
            field for field in point_fields
            if args.get(field) is not None
            and not (isinstance(args.get(field), str) and not args.get(field).strip())
        ]
        if not supplied_fields:
            raise ValueError("move requires at least one B pose field")
        if not callable(getattr(self.client, "command_trajectory", None)):
            raise RuntimeError("RM75 SDK client does not support controller trajectory completion")

        # 每次调用都读取实际 TCP，并将它作为本次动作的 A 点。B 的真实可达性、
        # 奇异点和控制器碰撞保护由 rm_movej_p/rm_movel 的规划结果兜底。
        if self._arm is not None:
            self._arm._preflight()
        ensure_trajectory_ready = getattr(self.client, "ensure_trajectory_ready", None)
        if callable(ensure_trajectory_ready):
            ensure_trajectory_ready()
        point_a = self._current_pose_mm_deg()
        point_b = list(point_a)
        for index, field in enumerate(point_fields):
            if field in supplied_fields:
                point_b[index] = self._pose_from_fields(args, (field,))[0]
        self._validate_workspace(point_b)

        if not self._motion_lock.acquire(blocking=False):
            active = self._motion_state["active_action_id"] or "joint/cartesian motion"
            raise RuntimeError(f"another motion is active: {active}")

        action_id = f"rm75_cart_{uuid4().hex[:10]}"
        timeout_to_b = (
            self.max_motion_seconds
            if motion_mode == "joint"
            else self._motion_deadline_seconds(point_a, point_b, speed_percent)
        )
        with self._action_lock:
            self._active_action_id = action_id
            self._motion_state["active_action_id"] = action_id
            self._cancelled.discard(action_id)
            self._a_to_b_actions[action_id] = {
                "stage": "to_b",
                "point_a": point_a,
                "point_b": point_b,
                "motion_mode": motion_mode,
                "speed_percent": speed_percent,
                "timeout_seconds": timeout_to_b,
            }
        self._monitor_thread = threading.Thread(
            target=self._submit_a_to_b_segment,
            args=(action_id, "to_b"),
            daemon=True,
            name=f"rm75-a-to-b-to-b-{action_id}",
        )
        self._monitor_thread.start()
        threading.Thread(
            target=self._a_to_b_watchdog,
            args=(action_id, "to_b"),
            daemon=True,
            name=f"rm75-a-to-b-watchdog-to-b-{action_id}",
        ).start()
        print(f"[rm75 ACP] {action_id}: started (move)", flush=True)
        return {"state": "running", "action_id": action_id}

    def _a_to_b_result(self, state, reason, **extra):
        return {
            "reason": reason,
            "point_a_pose_mm_deg": list(state["point_a"]),
            "target_pose_mm_deg": list(state["point_b"]),
            "motion_mode": state["motion_mode"],
            **extra,
        }

    def _submit_a_to_b_segment(self, action_id, segment):
        with self._action_lock:
            state = self._a_to_b_actions.get(action_id)
            if (state is None or state["stage"] != segment
                    or action_id in self._cancelled
                    or action_id in self._terminal_action_ids):
                return
            state_snapshot = dict(state)
        target = state_snapshot["point_b"]
        target_args = dict(zip(
            ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg"),
            target,
        ))

        def on_controller_event(trajectory_state):
            threading.Thread(
                target=self._handle_a_to_b_event,
                args=(action_id, segment, bool(trajectory_state)),
                daemon=True,
                name=f"rm75-a-to-b-event-{segment}-{action_id}",
            ).start()

        try:
            # The final cancellation check and SDK submission are one critical
            # section shared with stopmotion.  If stop marks the action first,
            # submission exits without moving.  If submission wins, stop waits
            # here and sends rm_set_arm_slow_stop before it returns.
            with self._submission_lock:
                with self._action_lock:
                    state = self._a_to_b_actions.get(action_id)
                    if (state is None or state["stage"] != segment
                            or action_id in self._cancelled
                            or action_id in self._terminal_action_ids):
                        return
                    state_snapshot = dict(state)
                self._submit(
                    "movej_p" if state_snapshot["motion_mode"] == "joint" else "movel",
                    target_args,
                    target,
                    state_snapshot["speed_percent"],
                    wait_for_completion=True,
                    completion_callback=on_controller_event,
                )
        except Exception as exc:
            with self._action_lock:
                state = self._a_to_b_actions.get(action_id)
                if state is None or state["stage"] != segment:
                    return
                state["stage"] = "failed"
                state_snapshot = dict(state)
            self._request_slow_stop()
            self._finish_cartesian(
                action_id,
                "error",
                self._a_to_b_result(
                    state_snapshot,
                    str(exc),
                    failed_segment=segment,
                ),
            )

    def _handle_a_to_b_event(self, action_id, segment, trajectory_state):
        cancelled = False
        with self._action_lock:
            state = self._a_to_b_actions.get(action_id)
            if (state is None or state["stage"] != segment
                    or action_id in self._terminal_action_ids):
                return
            if action_id in self._cancelled:
                cancelled = True
                state_snapshot = dict(state)
            elif not trajectory_state:
                state["stage"] = "failed"
                state_snapshot = dict(state)
            else:
                state["stage"] = "completed"
                state_snapshot = dict(state)

        if cancelled:
            self._request_slow_stop()
            return

        if not trajectory_state:
            self._finish_cartesian(
                action_id,
                "error",
                self._a_to_b_result(
                    state_snapshot,
                    "controller reported trajectory planning, reachability or collision failure",
                    failed_segment=segment,
                ),
            )
            return
        self._finish_cartesian(
            action_id,
            "completed",
            self._a_to_b_result(
                state_snapshot,
                "controller_reached_point_b",
            ),
        )

    def _a_to_b_watchdog(self, action_id, segment):
        with self._action_lock:
            state = self._a_to_b_actions.get(action_id)
            if state is None or state["stage"] != segment:
                return
            timeout_seconds = state["timeout_seconds"]
            target = list(state["point_b"])

        started = time.monotonic()
        deadline = started + timeout_seconds
        # Match the existing Cartesian monitor: allow the controller a short
        # startup grace period, then require bounded position or orientation
        # progress.  The controller event remains the authoritative success
        # signal, but a jam cannot keep the arm energized until the 300 s
        # action deadline.
        last_progress = started + self.start_grace_seconds
        best_position_error = None
        best_euler_error = None
        state_snapshot = None
        failure_reason = None
        failure_extra = {}

        while time.monotonic() < deadline:
            with self._action_lock:
                state = self._a_to_b_actions.get(action_id)
                if (state is None or state["stage"] != segment
                        or action_id in self._cancelled
                        or action_id in self._terminal_action_ids):
                    return

            try:
                if self._arm is not None:
                    self._arm._preflight()
                current = self._current_pose_mm_deg()
                position_error, euler_error = self._pose_error(current, target)
            except Exception as exc:
                failure_reason = f"progress monitoring failed: {exc}"
                failure_extra = {"failed_segment": segment}
                claim_stage = "monitor_failed"
            else:
                now = time.monotonic()
                if (position_error <= self.position_tolerance_mm
                        and euler_error <= self.euler_tolerance_deg):
                    # At the requested pose there is no stall.  Continue to
                    # wait for the controller's terminal event.
                    last_progress = now
                else:
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
                        failure_reason = "motion_stalled"
                        failure_extra = {
                            "stall_seconds": self.stall_timeout_seconds,
                            "actual_pose_mm_deg": current,
                            "position_error_mm": position_error,
                            "euler_error_deg": euler_error,
                            "elapsed_seconds": now - started,
                            "failed_segment": segment,
                        }
                        claim_stage = "stalled"

            if failure_reason is not None:
                with self._action_lock:
                    state = self._a_to_b_actions.get(action_id)
                    if (state is None or state["stage"] != segment
                            or action_id in self._cancelled
                            or action_id in self._terminal_action_ids):
                        return
                    # Claim the action before cancelling the global trajectory
                    # wait so a simultaneous controller event is ignored.
                    state["stage"] = claim_stage
                    state_snapshot = dict(state)
                break
            time.sleep(min(self.poll_interval_seconds, max(0.0, deadline - time.monotonic())))
        else:
            with self._action_lock:
                state = self._a_to_b_actions.get(action_id)
                if (state is None or state["stage"] != segment
                        or action_id in self._cancelled
                        or action_id in self._terminal_action_ids):
                    return
                state["stage"] = "timed_out"
                state_snapshot = dict(state)
            failure_reason = "controller completion event timed out"
            failure_extra = {
                "timeout_seconds": timeout_seconds,
                "failed_segment": segment,
            }

        cancel_wait = getattr(self.client, "cancel_trajectory_wait", None)
        if callable(cancel_wait):
            cancel_wait()
        self._request_slow_stop()
        time.sleep(self.stop_finalize_seconds)
        self._finish_cartesian(
            action_id,
            "error",
            self._a_to_b_result(
                state_snapshot,
                failure_reason,
                **failure_extra,
            ),
        )

    def _uses_controller_completion(self, motion_type):
        return (
            motion_type == "move_offset"
            and callable(getattr(self.client, "command_trajectory", None))
            and callable(getattr(self.client, "wait_trajectory", None))
        )

    def _execute_controller_cartesian(
            self, action_id, motion_type, args, offset, target, speed_percent,
            timeout_seconds):
        status, result = "error", {"reason": "unknown", "requested_offset_mm_deg": offset}
        native_submission = {"used": False}

        def result_payload(reason, **extra):
            return {
                "reason": reason,
                "requested_offset_mm_deg": offset,
                "target_pose_mm_deg": target,
                **extra,
            }

        def on_controller_event(trajectory_state):
            if trajectory_state:
                if native_submission["used"]:
                    self._native_tool_offset_supported = True
                event_status = "completed"
                event_result = result_payload("controller_target_reached")
            else:
                self._invalidate_confirmed_pose()
                event_status = "error"
                event_result = result_payload(
                    "controller reported trajectory planning or execution failure"
                )
            threading.Thread(
                target=self._finish_cartesian,
                args=(action_id, event_status, event_result),
                daemon=True,
                name=f"rm75-cartesian-event-{action_id}",
            ).start()

        try:
            with self._action_lock:
                cancelled = action_id in self._cancelled
            if cancelled:
                status, result = "cancelled", {"reason": "stopmotion"}
            else:
                submitted = False
                if self._native_tool_offset_supported is not False:
                    try:
                        native_submission["used"] = True
                        self._submit(
                            motion_type,
                            args,
                            offset,
                            speed_percent,
                            wait_for_completion=True,
                            completion_callback=on_controller_event,
                        )
                        submitted = True
                    except RuntimeError as exc:
                        native_submission["used"] = False
                        if "SDK -7" not in str(exc):
                            raise
                        self._native_tool_offset_supported = False
                        print(
                            "[rm75] native rm_movel_offset unsupported; using rm_movel compatibility mode",
                            flush=True,
                        )
                if not submitted:
                    if not self._controller_action_is_active(action_id):
                        return
                    self.client.command_trajectory(
                        "rm_movel",
                        self._to_sdk_pose(target),
                        speed_percent,
                        0,
                        0,
                        0,
                        completion_callback=on_controller_event,
                    )
                # 到位事件可能在 SDK 包装调用返回前已经直接完成了动作。此时不能
                # 再读取全局事件槽，否则可能误消费随后动作登记的新事件。
                with self._action_lock:
                    already_finished = action_id in self._terminal_action_ids
                if already_finished:
                    return
                # 这里只等待 SDK 三线程模式的官方到位事件，不能再调用
                # rm_get_current_arm_state/rm_get_arm_current_trajectory。真机上这些
                # 状态查询偶尔会卡在 C SDK 内，即使到位事件已经收到也无法收尾。
                trajectory_state = self.client.wait_trajectory(timeout_seconds)
                with self._action_lock:
                    cancelled = action_id in self._cancelled
                if cancelled:
                    status, result = "cancelled", {"reason": "stopmotion"}
                elif trajectory_state is True:
                    status = "completed"
                    result = result_payload("controller_target_reached")
                elif trajectory_state is False:
                    self._invalidate_confirmed_pose()
                    result = result_payload(
                        "controller reported trajectory planning or execution failure"
                    )
                else:
                    self._invalidate_confirmed_pose()
                    self._request_slow_stop()
                    time.sleep(self.stop_finalize_seconds)
                    result = result_payload(
                        "controller completion event timed out",
                        timeout_seconds=timeout_seconds,
                    )
        except Exception as exc:
            self._invalidate_confirmed_pose()
            result = result_payload(str(exc))
        finally:
            self._finish_cartesian(action_id, status, result)

    def _controller_action_is_active(self, action_id):
        with self._action_lock:
            return (
                action_id not in self._terminal_action_ids
                and action_id not in self._cancelled
                and (self._active_action_id == action_id
                     or self._motion_state["active_action_id"] == action_id)
            )

    def _validated_offset_target(self, offset):
        """Compose and validate the absolute TCP target before any offset submit."""
        with self._confirmed_pose_lock:
            revision = self._confirmed_pose_revision
            current = (
                list(self._confirmed_pose_mm_deg)
                if self._confirmed_pose_mm_deg is not None
                else None
            )
        if current is None:
            current = self._current_pose_mm_deg()
        target = self._compose_tool_offset(current, offset)
        self._validate_workspace(target)
        return target, revision

    def _set_confirmed_pose(self, pose):
        with self._confirmed_pose_lock:
            self._confirmed_pose_mm_deg = list(pose)
            self._confirmed_pose_revision += 1

    def _invalidate_confirmed_pose(self):
        with self._confirmed_pose_lock:
            self._confirmed_pose_mm_deg = None
            self._confirmed_pose_revision += 1

    def _controller_offset_wait_deadline_seconds(self, offset, speed_percent):
        distance_mm = math.sqrt(sum(value * value for value in offset[:3]))
        speed_mm_s = 600.0 * speed_percent / 100.0
        return min(self.max_motion_seconds, max(15.0, distance_mm / speed_mm_s * 4.0 + 10.0))

    def _controller_action_watchdog(self, action_id, offset, timeout_seconds):
        """SDK 调用和控制器事件同时失联时，也必须给 ACP 一个有界终态。"""
        time.sleep(timeout_seconds)
        with self._action_lock:
            active = (
                action_id not in self._terminal_action_ids
                and (self._active_action_id == action_id
                     or self._motion_state["active_action_id"] == action_id)
            )
        if not active:
            return
        self._invalidate_confirmed_pose()
        cancel_wait = getattr(self.client, "cancel_trajectory_wait", None)
        if callable(cancel_wait):
            cancel_wait()
        self._request_slow_stop()
        time.sleep(self.stop_finalize_seconds)
        self._finish_cartesian(
            action_id,
            "error",
            {
                "reason": "controller completion event timed out",
                "timeout_seconds": timeout_seconds,
                "requested_offset_mm_deg": offset,
            },
        )

    def _native_tool_offset(self, args):
        offset = self._pose_from_fields(
            args,
            ("dx_mm", "dy_mm", "dz_mm", "drx_deg", "dry_deg", "drz_deg"),
            empty_value=0.0,
        )
        if args.get("frame_type", "tool") != "tool":
            raise ValueError("frame_type must be 'tool'（工作坐标系偏移暂不支持）")
        translation_mm = math.sqrt(sum(value * value for value in offset[:3]))
        max_translation_mm = self.max_reach_mm + self.tool_length_mm
        if translation_mm > max_translation_mm:
            raise ValueError(
                f"tool offset distance {translation_mm:.0f} mm exceeds "
                f"cartesian.max_reach_mm {self.max_reach_mm:g} + "
                f"tool_length_mm {self.tool_length_mm:g}"
            )
        for value, label in zip(offset[3:], ("drx", "dry", "drz")):
            if abs(value) > self.max_euler_abs_deg:
                raise ValueError(
                    f"{label} {value:.0f} deg exceeds cartesian.max_euler_abs_deg "
                    f"{self.max_euler_abs_deg:g}"
                )
        return offset

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

    def _submit(
            self, motion_type, args, target, speed_percent, wait_for_completion=False,
            completion_callback=None):
        """下发 SDK 运动命令；原生相对运动通过控制器事件等待终态。"""
        try:
            if wait_for_completion:
                def command(method, *command_args):
                    return self.client.command_trajectory(
                        method, *command_args, completion_callback=completion_callback
                    )
            else:
                command = self.client.command
            # command_trajectory 已准备到位回调，实际 SDK 调用必须保持非阻塞。
            block = 0
            if motion_type in ("movel", "movej_p"):
                pose = self._pose_from_fields(args, ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg"))
                method = "rm_movej_p" if motion_type == "movej_p" else "rm_movel"
                command(method, self._to_sdk_pose(pose), speed_percent, 0, 0, block)
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
                        if wait_for_completion:
                            raise RuntimeError(
                                "controller does not support native rm_movel_offset (SDK -7)"
                            ) from exc
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
                expected_device = "关节设备" if motion_type == "movej_p" else "笛卡尔设备"
                raise RuntimeError(
                    f"控制器到位设备校验失败（SDK -4）：请在 RealMan Studio/控制器中将当前到位设备设为{expected_device}；"
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

    def _monitor_cartesian(self, action_id, start_pose, target, max_duration):
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
            self._a_to_b_actions.pop(action_id, None)
            self._last_completion = {"action_id": action_id, "status": status, "result": dict(result)}
            release_motion_lock = self._motion_lock.locked()
        confirmed_target = result.get("target_pose_mm_deg") if status == "completed" else None
        if isinstance(confirmed_target, list) and len(confirmed_target) == 6:
            self._set_confirmed_pose(confirmed_target)
        else:
            self._invalidate_confirmed_pose()
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
            # Serialize watchdog/error stops with every motion submission.
            with self._submission_lock:
                try:
                    self._send_slow_stop()
                except Exception as exc:
                    print(f"[rm75] cartesian slow-stop failed: {exc}", flush=True)

        thread = threading.Thread(target=worker, daemon=True, name="rm75-cartesian-slow-stop")
        thread.start()
        return thread

    def _send_slow_stop(self):
        """Send slow-stop while the caller owns _submission_lock."""
        interrupt = getattr(self.client, "command_interrupt", None)
        if callable(interrupt):
            return interrupt("rm_set_arm_slow_stop")
        return self.client.command("rm_set_arm_slow_stop")

    def _stop_motion(self):
        # 运动锁和动作 ID 在两张卡之间共享；任一 stop 卡都必须能停止实际持有者。
        # SDK 慢停调用无超时上限，不得在 _action_lock 内执行。
        with self._action_lock:
            action_id = self._motion_state["active_action_id"] or self._active_action_id
            owns_action = self._active_action_id == action_id
            if action_id:
                self._cancelled.add(action_id)
        if action_id and owns_action:
            self._invalidate_confirmed_pose()
        try:
            if action_id and self.client.connected:
                # Synchronous ordering barrier: stopmotion cannot return while
                # an already-claimed submission can still start afterwards.
                with self._submission_lock:
                    self._send_slow_stop()
        finally:
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
                # 官方事件等待被 cancel 后应立即退出；只给很短的收尾窗口，
                # 绝不能因状态查询/C SDK 异常再次让 stopmotion 长时间占锁。
                monitor.join(min(0.5, self.poll_interval_seconds + CARTESIAN_STOP_JOIN_MARGIN))
            self._finish_cartesian(action_id, status, result)

        threading.Thread(
            target=worker,
            daemon=True,
            name="rm75-cartesian-stop-finalize",
        ).start()

    def _finalize_shared_stop(self, action_id):
        """Finish an abs_move action stopped through joint_control."""
        with self._action_lock:
            owns_action = self._active_action_id == action_id
        if not owns_action:
            return False
        self._invalidate_confirmed_pose()
        self._finish_after_stop(action_id, "cancelled", {"reason": "stopmotion"})
        return True

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
    from pick_place import PickPlacePlugin
    from servo import RM75ServoPlugin

    client = RM75SDKClient(config)
    shared = client.shared_client()
    arm = RM75Plugin(shared, config, namespace=namespace, ros2=ros2)
    plugins = [
        client,
        arm,
        GripperPlugin(shared, config, namespace=namespace, ros2=ros2),
        RM75ServoPlugin(shared, config, namespace=namespace, ros2=ros2),
        CartesianPlugin(shared, config, arm_plugin=arm, namespace=namespace, ros2=ros2),
        PickPlacePlugin(client.exclusive_client(), config, namespace, ros2),
    ]
    camera_config = config.get("ext_camera", {})
    ext_camera_plugin = None
    if camera_config.get("enabled", False):
        from camera import ExtCameraPlugin

        ext_camera_plugin = ExtCameraPlugin(
            camera_config, namespace, ros2.executor_core
        )
        plugins.append(ext_camera_plugin)
    vision_config = config.get("vision_capture", {})
    # Vision capture is an explicit capability; an enabled RGB camera alone
    # must not implicitly add an undeclared recording card.
    if vision_config.get("enabled", False):
        from vision_capture import VisionCapturePlugin

        plugins.append(
            VisionCapturePlugin(
                vision_config,
                namespace,
                ros2.executor_core,
                external_camera=ext_camera_plugin,
            )
        )
    return plugins
