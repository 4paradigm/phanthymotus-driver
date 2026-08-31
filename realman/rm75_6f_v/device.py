#!/usr/bin/env python3
"""Upper-level adapter for RealMan RM75-6F-V through the official ROS2 driver."""

from __future__ import annotations

import json
import math
import re
import threading
import time
from pathlib import Path
from uuid import uuid4

from common.vendor_runtime import action_schema, jsonable, tool


ARM_STATUS = {
    0: "idle",
    1: "move_l",
    2: "move_j",
    3: "move_c",
    4: "move_s",
    5: "joint_pass_through",
    6: "pose_pass_through",
    7: "force_pose_pass_through",
    8: "current_pass_through",
    9: "emergency_stop",
    10: "slow_stop",
    11: "pause",
    12: "current_drag",
    13: "force_drag",
    14: "teach",
}


def _require(args, key):
    if key not in args:
        raise ValueError(f"missing required argument: {key}")
    return args[key]


def _finite_float(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _finite_int(value, name):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer")
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value) or not value.is_integer():
            raise ValueError(f"{name} must be an integer")
        return int(value)
    raise ValueError(f"{name} must be an integer")


def _bounded_int(value, name, lower, upper):
    result = _finite_int(value, name)
    if result < lower or result > upper:
        raise ValueError(f"{name} must be in [{lower}, {upper}]")
    return result


def _bounded_float(value, name, lower, upper):
    result = _finite_float(value, name)
    if result < lower or result > upper:
        raise ValueError(f"{name} must be in [{lower}, {upper}]")
    return result


def _with_completion(schema, actions, timeout=30):
    schema = dict(schema)
    schema["x-completion"] = {"actions": actions, "timeout": timeout}
    return schema


class RM75Nodes:
    def __init__(self, config, namespace, ros2):
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Empty, String
        from rm_ros_interfaces import msg as rm_msg

        self.config = config
        self.namespace = namespace
        self.dof = int(config.get("arm_dof", 7))
        self.safety = config.get("safety", {})
        self.topics = config.get("topics", {})
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.values = {}
        self.sequences = {}
        self.streams = {}
        self.robot = Node("rm75_adapter_robot", context=ros2.ctx_robot)
        self.core = Node("rm75_adapter_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self.robot)
        ros2.executor_core.add_node(self.core)

        self._String = String
        self._Movej = self._message_type(rm_msg, "Movej")
        self._Stop = self._message_type(rm_msg, "Stop")
        self._Jointerrclear = self._message_type(rm_msg, "Jointerrclear")
        self.arm_state_request_pub = self.robot.create_publisher(Empty, self.topics["arm_state_cmd"], 10)
        self.movej_pub = self.robot.create_publisher(self._Movej, self.topics["movej_cmd"], 10)
        self.stop_pub = self.robot.create_publisher(self._Stop, self.topics["move_stop_cmd"], 10)
        self.clear_joint_error_pub = self.robot.create_publisher(self._Jointerrclear, self.topics["joint_error_clear_cmd"], 10)

        self._bridge("joint_states", JointState, self.topics["joint_states"], "sensor/skeleton", self._joint_state_payload)
        self._bridge("arm_state", self._message_type(rm_msg, "Armstate"), self.topics["arm_state"], "data/json")
        self._bridge("arm_original_state", self._message_type(rm_msg, "Armoriginalstate"), self.topics["arm_original_state"], "data/json")
        self._bridge("arm_current_status", self._message_type(rm_msg, "Armcurrentstatus"), self.topics["arm_current_status"], "data/json")
        self._bridge("joint_error", self._message_type(rm_msg, "Jointerrorcode"), self.topics["joint_error"], "data/json")
        self._bridge("rm_error", self._message_type(rm_msg, "Rmerr"), self.topics["rm_error"], "data/json")
        self._optional_bridge(rm_msg, "movej_result", self.topics.get("movej_result"), ["Movejresult", "MovejResult", "Planstate", "PlanState"])
        self._optional_bridge(rm_msg, "move_stop_result", self.topics.get("move_stop_result"), ["Moveresult", "MoveResult", "Stopresult", "StopResult"])
        self._optional_bridge(rm_msg, "joint_error_clear_result", self.topics.get("joint_error_clear_result"), ["Jointerrclearresult", "JointerrclearResult"])

    @staticmethod
    def _message_type(module, name):
        if hasattr(module, name):
            return getattr(module, name)
        available = [item for item in dir(module) if item.lower() == name.lower()]
        if available:
            return getattr(module, available[0])
        raise ImportError(f"rm_ros_interfaces.msg.{name} is unavailable; check the sourced ros2_rm_robot workspace")

    def _optional_bridge(self, module, key, topic_name, candidates):
        if not topic_name:
            return
        for candidate in candidates:
            try:
                msg_type = self._message_type(module, candidate)
                self._bridge(key, msg_type, topic_name, "data/json")
                return
            except ImportError:
                continue
        print(f"[rm75] skip optional result topic {topic_name}: no matching message type for {candidates}", flush=True)

    def _bridge(self, key, msg_type, robot_topic, fmt, transform=None):
        core_topic = f"/{self.namespace}/realman/rm75/{key}"
        publisher = self.core.create_publisher(self._String, core_topic, 20)
        self.robot.create_subscription(
            msg_type,
            robot_topic,
            self._callback(key, publisher, transform=transform),
            20,
        )
        self.streams[key] = {"robot_topic": robot_topic, "topic": core_topic, "format": fmt}

    def _callback(self, key, publisher, transform=None):
        def callback(msg):
            value = transform(msg) if transform else jsonable(msg)
            out = self._String()
            out.data = json.dumps(value, ensure_ascii=False)
            publisher.publish(out)
            with self.lock:
                self.values[key] = {"timestamp": time.time(), "data": value}
                self.sequences[key] = self.sequences.get(key, 0) + 1
                self.condition.notify_all()
        return callback

    @staticmethod
    def _joint_state_payload(msg):
        names = list(msg.name)
        positions = list(msg.position)
        velocities = list(msg.velocity)
        efforts = list(msg.effort)
        joints = []
        for idx, name in enumerate(names):
            joints.append({
                "idx": idx,
                "name": name,
                "q": positions[idx] if idx < len(positions) else 0.0,
                "dq": velocities[idx] if idx < len(velocities) else 0.0,
                "tau": efforts[idx] if idx < len(efforts) else 0.0,
            })
        return {"joints": joints, "timestamp": time.time()}

    def snapshot(self):
        with self.lock:
            values = dict(self.values)
        status = values.get("arm_current_status", {}).get("data", {}).get("arm_current_status")
        if status is not None:
            values["arm_current_status_name"] = ARM_STATUS.get(int(status), "unknown")
        values["configured_arm"] = {
            "model": "RM75-6F-V",
            "dof": self.dof,
            "arm_ip": self.config.get("arm_ip"),
            "tcp_port": self.config.get("tcp_port"),
        }
        values["health"] = self.health_summary(values)
        return values

    def health_summary(self, values=None):
        values = values if values is not None else self.snapshot()
        now = time.time()
        max_age = float(self.safety.get("max_state_age_seconds", 2.0))
        stale = []
        for key in ("joint_states", "joint_error", "rm_error"):
            item = values.get(key)
            if not item:
                stale.append({"key": key, "reason": "missing"})
                continue
            age = now - float(item.get("timestamp", 0))
            if age > max_age:
                stale.append({"key": key, "reason": "stale", "age_seconds": age})
        errors = self._active_errors(values)
        return {
            "can_move": not stale and not errors,
            "stale_inputs": stale,
            "active_errors": errors,
        }

    def request_arm_state(self):
        from std_msgs.msg import Empty

        self.arm_state_request_pub.publish(Empty())
        return {"state": "requested", "topic": self.topics["arm_state_cmd"]}

    def publish_movej(self, joints, speed=20, block=False, trajectory_connect=0, wait_result=False, timeout=None, blend_radius=None):
        joints, speed, trajectory_connect, timeout, blend_radius = self._normalize_movej_args(
            joints, speed, trajectory_connect, timeout, blend_radius
        )
        preflight = self.preflight_movej(joints)
        if preflight:
            return preflight
        return self._publish_movej_checked(joints, speed, block, trajectory_connect, wait_result, timeout, blend_radius)

    def _normalize_movej_args(self, joints, speed=20, trajectory_connect=0, timeout=None, blend_radius=None):
        if not isinstance(joints, (list, tuple)):
            raise ValueError("joints must be an array of finite numbers")
        if len(joints) != self.dof:
            raise ValueError(f"movej requires exactly {self.dof} joint angles in radians")
        joints = [_finite_float(value, f"joints[{idx}]") for idx, value in enumerate(joints)]
        speed = _finite_int(speed, "speed")
        if speed < 1 or speed > 100:
            raise ValueError("speed must be in [1, 100]")
        trajectory_connect = _finite_int(trajectory_connect, "trajectory_connect")
        if trajectory_connect not in (0, 1):
            raise ValueError("trajectory_connect must be 0 or 1")
        timeout = self._motion_timeout(timeout)
        if blend_radius is None:
            blend_radius = self.safety.get("movej_blend_radius", 0)
        blend_radius = _finite_float(blend_radius, "blend_radius")
        if blend_radius < 0:
            raise ValueError("blend_radius must be non-negative")
        return joints, speed, trajectory_connect, timeout, blend_radius

    def _publish_movej_checked(self, joints, speed, block, trajectory_connect, wait_result, timeout, blend_radius):
        baseline = self._sequence("movej_result")
        msg = self._Movej()
        msg.joint = [float(value) for value in joints]
        fields = set(getattr(msg, "get_fields_and_field_types", lambda: {})())
        if "speed" in fields:
            msg.speed = int(speed)
        elif "v" in fields:
            msg.v = int(speed)
        else:
            raise AttributeError("Movej message has neither speed nor v field")
        if "r" in fields:
            msg.r = float(blend_radius)
        if "block" in fields:
            msg.block = bool(block)
        if "trajectory_connect" in fields:
            msg.trajectory_connect = int(trajectory_connect)
        if "dof" in fields:
            msg.dof = self.dof
        self.movej_pub.publish(msg)
        result = {
            "state": "published",
            "topic": self.topics["movej_cmd"],
            "result_topic": self.topics.get("movej_result"),
            "joint": msg.joint,
            "speed": speed,
            "movej_fields": sorted(fields),
            "block": bool(block),
            "trajectory_connect": int(trajectory_connect),
            "dof": self.dof,
        }
        if "v" in fields:
            result["v"] = int(speed)
        if "r" in fields:
            result["r"] = float(blend_radius)
            result["blend_radius"] = float(blend_radius)
        if wait_result:
            update = self.wait_for_update(
                "movej_result",
                baseline,
                timeout,
            )
            result["result"] = self._classify_motion_result(update)
        else:
            result["completion"] = self.wait_for_motion_complete(joints, baseline, timeout)
        return result

    def start_movej_action(self, joints, *, speed=20, block=False, trajectory_connect=0, wait_result=False, timeout=None, blend_radius=None, tool_name="joint_control"):
        joints, speed, trajectory_connect, timeout, blend_radius = self._normalize_movej_args(
            joints, speed, trajectory_connect, timeout, blend_radius
        )
        preflight = self.preflight_movej(joints)
        if preflight:
            return preflight
        action_id = f"rm75_movej_{uuid4().hex[:12]}"
        threading.Thread(
            target=self._motion_worker,
            args=(action_id, tool_name, self._publish_movej_checked),
            kwargs={
                "joints": joints,
                "speed": speed,
                "block": block,
                "trajectory_connect": trajectory_connect,
                "wait_result": wait_result,
                "timeout": timeout,
                "blend_radius": blend_radius,
            },
            daemon=True,
            name=action_id,
        ).start()
        return {"state": "moving", "action_id": action_id, "timeout": timeout}

    def current_joint_positions(self):
        latest = self._latest("joint_states")
        if latest is None:
            raise ValueError("No /joint_states data has been received yet")
        joints = latest.get("data", {}).get("joints", [])
        positions_by_name = {}
        for item in joints:
            name = str(item.get("name", "")).strip().lower()
            match = re.fullmatch(r"joint([1-9][0-9]*)", name)
            if not match:
                continue
            joint_num = int(match.group(1))
            if not 1 <= joint_num <= self.dof:
                continue
            if name in positions_by_name:
                raise ValueError(f"/joint_states contains duplicate {name}")
            positions_by_name[name] = _finite_float(item.get("q", 0.0), f"current {name}")
        missing = [f"joint{idx}" for idx in range(1, self.dof + 1) if f"joint{idx}" not in positions_by_name]
        if missing:
            raise ValueError(f"/joint_states is missing required joint names: {missing}")
        positions = [positions_by_name[f"joint{idx}"] for idx in range(1, self.dof + 1)]
        return positions

    def _motion_worker(self, action_id, tool_name, func, **kwargs):
        try:
            result = func(**kwargs)
            status = self._completion_status(result)
        except Exception as exc:
            result = {"state": "error", "error": str(exc)}
            status = "error"
        self._acp_notify(action_id, status, result, tool_name)

    @staticmethod
    def _completion_status(result):
        if result.get("state") in ("rejected", "error", "timeout"):
            return "error"
        for key in ("completion", "result"):
            item = result.get(key)
            if isinstance(item, dict) and item.get("state") in ("rejected", "error", "timeout"):
                return "error"
        return "completed"

    @staticmethod
    def _acp_notify(action_id, status, result, tool_name):
        import os as _os
        import ssl as _ssl
        import urllib.request as _urllib

        agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
        ctx = _ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = _ssl.CERT_NONE
        payload = json.dumps({
            "action_id": action_id,
            "status": status,
            "result": jsonable(result),
            "tool": tool_name,
            "ts": time.time(),
        }, ensure_ascii=False).encode()
        try:
            request = _urllib.Request(
                f"{agent_core_url}/api/acp/complete",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with _urllib.urlopen(request, timeout=5, context=ctx):
                pass
        except Exception as exc:
            print(f"[ACP] callback failed for {action_id}: {exc}", flush=True)

    def publish_stop(self, block=False):
        baseline = self._sequence("move_stop_result")
        msg = self._Stop()
        if hasattr(msg, "block"):
            msg.block = bool(block)
        self.stop_pub.publish(msg)
        result = {"state": "stopped", "topic": self.topics["move_stop_cmd"], "result_topic": self.topics.get("move_stop_result")}
        if "move_stop_result" in self.streams:
            update = self.wait_for_update("move_stop_result", baseline, 1.0)
            if update.get("state") != "timeout":
                result["result"] = update
        return result

    def clear_joint_error(self, joint_num):
        joint_num = _finite_int(joint_num, "joint_num")
        if joint_num < 1 or joint_num > self.dof:
            raise ValueError(f"joint_num must be in [1, {self.dof}]")
        msg = self._Jointerrclear()
        msg.joint_num = joint_num
        baseline = self._sequence("joint_error_clear_result")
        self.clear_joint_error_pub.publish(msg)
        result = {"state": "published", "topic": self.topics["joint_error_clear_cmd"], "joint_num": joint_num}
        if "joint_error_clear_result" in self.streams:
            update = self.wait_for_update("joint_error_clear_result", baseline, 1.0)
            if update.get("state") != "timeout":
                result["result"] = update
        return result

    def _sequence(self, key):
        with self.lock:
            return self.sequences.get(key, 0)

    def wait_for_update(self, key, baseline, timeout):
        timeout = _finite_float(timeout, "timeout")
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.sequences.get(key, 0) <= baseline:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return {"state": "timeout", "topic": self.topics.get(key)}
                self.condition.wait(remaining)
            return {"state": "received", **self.values.get(key, {})}

    def wait_for_motion_complete(self, target_joints, baseline, timeout):
        result = self.wait_for_update("movej_result", baseline, 0.05)
        if result.get("state") != "timeout":
            classified = self._classify_motion_result(result)
            return {"source": "movej_result", **classified}

        tolerance = _finite_float(self.safety.get("joint_target_tolerance_rad", 0.01), "joint_target_tolerance_rad")
        if tolerance <= 0:
            raise ValueError("joint_target_tolerance_rad must be positive")
        deadline = time.monotonic() + timeout
        last_error = None
        while True:
            try:
                current = self.current_joint_positions()
                max_error = max(abs(current[idx] - target_joints[idx]) for idx in range(self.dof))
                last_error = max_error
                if max_error <= tolerance:
                    return {"state": "completed", "source": "joint_states", "max_error_rad": max_error}
            except ValueError as exc:
                last_error = str(exc)

            values = dict(self.values)
            errors = self._active_errors(values)
            if errors:
                return {"state": "error", "source": "error_topics", "errors": errors}

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return {"state": "timeout", "timeout_seconds": timeout, "last_error_rad": last_error}
            time.sleep(min(0.05, remaining))

    def _classify_motion_result(self, update):
        if update.get("state") == "timeout":
            return update
        data = update.get("data", update)
        failure = self._motion_result_failure(data)
        if failure:
            return {"state": "error", "result": update, **failure}
        return {"state": "completed", "result": update}

    def _motion_result_failure(self, value):
        if isinstance(value, dict):
            success = value.get("success")
            if isinstance(success, bool) and not success:
                return {"error": "movej_result reports success=false"}

            for key in ("error", "message", "msg", "reason"):
                text = value.get(key)
                if isinstance(text, str) and self._looks_like_failure(text):
                    return {"error": text, "field": key}

            for key in ("ret", "code", "err", "errno", "err_code", "error_code", "result", "status"):
                if key not in value:
                    continue
                item = value[key]
                if isinstance(item, bool):
                    continue
                if isinstance(item, str):
                    if self._looks_like_failure(item):
                        return {"error": item, "field": key}
                    try:
                        item = float(item)
                    except ValueError:
                        continue
                if isinstance(item, (int, float)):
                    if not math.isfinite(float(item)) or abs(float(item)) > 1e-9:
                        return {"error": f"movej_result {key}={item}", "field": key, "code": item}

            for item in value.values():
                failure = self._motion_result_failure(item)
                if failure:
                    return failure
        elif isinstance(value, list):
            for item in value:
                failure = self._motion_result_failure(item)
                if failure:
                    return failure
        elif isinstance(value, str) and self._looks_like_failure(value):
            return {"error": value}
        return None

    @staticmethod
    def _looks_like_failure(text):
        lowered = text.strip().lower()
        return any(token in lowered for token in ("fail", "error", "err", "abort", "reject", "timeout", "失败", "错误", "异常"))

    def _motion_timeout(self, timeout):
        if timeout is None:
            timeout = self.safety.get("max_result_wait_seconds", 10.0)
        timeout = _finite_float(timeout, "timeout")
        if timeout <= 0 or timeout > 30:
            raise ValueError("timeout must be in (0, 30]")
        return timeout

    def preflight_movej(self, joints):
        now = time.time()
        max_age = float(self.safety.get("max_state_age_seconds", 2.0))
        if self.safety.get("require_fresh_joint_state", True):
            latest = self._latest("joint_states")
            if latest is None:
                return self._reject("joint_state_missing", "No /joint_states data has been received yet")
            age = now - latest["timestamp"]
            if age > max_age:
                return self._reject("joint_state_stale", f"/joint_states is stale ({age:.2f}s)")
        if self.safety.get("require_no_errors", True):
            telemetry = self._error_telemetry_status(now, max_age)
            if telemetry:
                return telemetry
        limits = self.safety.get("joint_limits_rad") or []
        for idx, value in enumerate(joints):
            value = _finite_float(value, f"joints[{idx}]")
            if idx >= len(limits):
                continue
            lower, upper = limits[idx]
            lower = _finite_float(lower, f"joint_limits_rad[{idx}][0]")
            upper = _finite_float(upper, f"joint_limits_rad[{idx}][1]")
            if value < lower or value > upper:
                return self._reject(
                    "joint_limit",
                    f"joint{idx + 1} target {value:.4f} rad is outside [{lower:.4f}, {upper:.4f}]",
                )
        if self.safety.get("require_no_errors", True):
            with self.lock:
                values = dict(self.values)
            errors = self._active_errors(values)
            if errors:
                return self._reject("active_robot_error", "RM75 reports active errors; clear or diagnose before moving", errors=errors)
        return None

    def _error_telemetry_status(self, now, max_age):
        stale = []
        with self.lock:
            values = dict(self.values)
        for key in ("joint_error", "rm_error"):
            item = values.get(key)
            if not item:
                stale.append({"key": key, "reason": "missing"})
                continue
            age = now - float(item.get("timestamp", 0))
            if age > max_age:
                stale.append({"key": key, "reason": "stale", "age_seconds": age})
        if stale:
            return self._reject(
                "error_telemetry_unavailable",
                "RM75 error telemetry is missing or stale; refusing to move",
                stale_inputs=stale,
            )
        return None

    def _latest(self, key):
        with self.lock:
            return self.values.get(key)

    @staticmethod
    def _reject(code, message, **details):
        return {"state": "rejected", "code": code, "error": message, **details}

    def _active_errors(self, values):
        errors = []
        for key in ("joint_error", "rm_error"):
            item = values.get(key)
            if not item:
                continue
            data = item.get("data", {})
            if key == "rm_error" and self._rm_error_is_empty(data):
                continue
            numbers = self._error_numbers(data)
            nonzero = [number for number in numbers if abs(number) > 1e-9]
            if nonzero:
                errors.append({"key": key, "data": data})
        status = values.get("arm_current_status", {}).get("data", {}).get("arm_current_status")
        if status is not None and int(status) == 9:
            errors.append({"key": "arm_current_status", "status": int(status), "status_name": "emergency_stop"})
        return errors

    @classmethod
    def _numbers(cls, value):
        if isinstance(value, bool):
            return [int(value)]
        if isinstance(value, (int, float)):
            return [float(value)]
        if isinstance(value, dict):
            numbers = []
            for item in value.values():
                numbers.extend(cls._numbers(item))
            return numbers
        if isinstance(value, list):
            numbers = []
            for item in value:
                numbers.extend(cls._numbers(item))
            return numbers
        return []

    @classmethod
    def _error_numbers(cls, value):
        if isinstance(value, dict):
            if "joint_error" in value and isinstance(value["joint_error"], list):
                return cls._numbers(value["joint_error"])
            if "err" in value and isinstance(value["err"], list):
                return cls._numbers(value["err"])
            numbers = []
            for key, item in value.items():
                if key in ("header", "stamp", "timestamp", "seq", "frame_id", "dof", "err_len"):
                    continue
                numbers.extend(cls._error_numbers(item))
            return numbers
        return cls._numbers(value)

    @staticmethod
    def _rm_error_is_empty(data):
        if not isinstance(data, dict):
            return False
        err_len = data.get("err_len")
        return err_len == 0 or err_len == "0"

    def close(self):
        self.robot.destroy_node()
        self.core.destroy_node()


class RM75StatePlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tools(self):
        definitions = [
            tool("state", "sensor", "RM75-6F-V 聚合状态快照（request/response，无 topic_out）"),
            tool("refresh_state", "actuator", "请求睿尔曼 rm_driver 发布当前机械臂状态"),
            tool("model", "resource", "RM75-6F-V 简化 URDF 模型，用于卡片系统骨架渲染"),
        ]
        definitions.extend(
            tool(key, "sensor", f"RM75-6F-V {key} 数据流", topic_out=[{"topic": item["topic"], "format": item["format"]}])
            for key, item in self.nodes.streams.items()
        )
        return definitions

    def start(self):
        pass

    def stop(self):
        self.nodes.close()

    def dispatch(self, action, args):
        name = args.get("_tool_name")
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            if name in self.nodes.streams:
                item = self.nodes.streams[name]
                return {"state": "running", "topic_out": [{"topic": item["topic"], "format": item["format"]}]}
            if name == "state":
                return {"state": "ready", "mode": "snapshot", "topic_out": []}
            return {"state": "ready"}
        if name == "state":
            return self.nodes.snapshot()
        if name == "refresh_state":
            return self.nodes.request_arm_state()
        if name == "model":
            urdf_path = Path(__file__).with_name("resource") / "rm75_6f_v.urdf"
            return {"urdf": urdf_path.read_text(encoding="utf-8")}
        if name in self.nodes.streams:
            return {"state": "running", **self.nodes.streams[name]}
        return None


class RM75JointControlPlugin:
    ACTIONS = {
        "set": (["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "speed", "block", "wait_result", "timeout"], "输入 joint1..joint7 的完整目标关节角，单位 rad"),
        "stopmotion": (["block"], "立即停止机械臂运动"),
    }

    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        properties = {
            f"joint{idx}": {
                "type": "number",
                "description": f"joint{idx} 目标关节角，单位 rad",
            }
            for idx in range(1, self.nodes.dof + 1)
        }
        properties.update({
            "speed": {"type": "integer", "minimum": 1, "maximum": 30, "default": 5, "description": "七关节测试速度百分比，默认 5，最高限制 30"},
            "block": {"type": "boolean", "default": False, "description": "是否使用 rm_driver 阻塞模式"},
            "wait_result": {"type": "boolean", "default": False, "description": "优先等待 movej_result；默认按 /joint_states/状态同步等待完成"},
            "timeout": {"type": "number", "minimum": 0.1, "maximum": 30, "default": 10, "description": "等待结果超时时间，单位秒"},
        })
        return tool(
            "joint_control",
            "actuator",
            "RM75-6F-V 七关节控制卡片：输入 joint1..joint7 后执行 set",
            _with_completion(action_schema(self.ACTIONS, properties), ["set"], 30),
        )

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready", "joints": [f"joint{idx}" for idx in range(1, self.nodes.dof + 1)]}
        if action == "set":
            return self.nodes.start_movej_action(
                self._joint_targets(args),
                speed=_bounded_int(args.get("speed", 5), "speed", 1, 30),
                block=args.get("block", False),
                trajectory_connect=0,
                wait_result=args.get("wait_result", False),
                timeout=args.get("timeout"),
                tool_name="joint_control",
            )
        if action in ("stopmotion", "stop"):
            return self.nodes.publish_stop(block=args.get("block", False))
        return None

    def _joint_targets(self, args):
        return [_finite_float(_require(args, f"joint{idx}"), f"joint{idx}") for idx in range(1, self.nodes.dof + 1)]


def build_plugins(config, namespace, ros2):
    nodes = RM75Nodes(config, namespace, ros2)
    return [
        RM75StatePlugin(nodes),
        RM75JointControlPlugin(nodes),
    ]
