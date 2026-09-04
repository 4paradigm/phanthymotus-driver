#!/usr/bin/env python3
"""RealMan RM75-6F-V MCP Driver using the official Python API2 SDK."""

from __future__ import annotations

import math
import os
import threading
import time
from uuid import uuid4
from pathlib import Path

from common.vendor_runtime import action_schema, jsonable, tool


JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
JOINT_LIMITS_DEG = [(-178.0, 178.0), (-130.0, 130.0), (-178.0, 178.0),
                    (-135.0, 135.0), (-178.0, 178.0), (-128.0, 128.0),
                    (-360.0, 360.0)]


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
    PREFIX = "realman_state"

    METHODS = {
        "robot_info": "rm_get_robot_info",
        "software_info": "rm_get_arm_software_info",
        "arm_all_state": "rm_get_arm_all_state",
        "controller_state": "rm_get_controller_state",
    }

    def __init__(self, client, config):
        self.client = client
        safety = config.get("safety", {})
        self.max_step_deg = min(float(safety.get("max_step_deg", 5.0)), 5.0)
        self.max_speed_percent = min(int(safety.get("max_speed_percent", 10)), 10)
        self.default_speed_percent = min(int(safety.get("default_speed_percent", 5)), self.max_speed_percent)
        self.target_tolerance_deg = float(safety.get("target_tolerance_deg", 0.5))
        self.default_timeout_seconds = float(safety.get("motion_timeout_seconds", 30.0))
        self.poll_interval_seconds = float(safety.get("poll_interval_seconds", 0.2))
        self._motion_lock = threading.Lock()
        self._active_action_id = None
        self._cancelled = set()

    def get_tools(self):
        definitions = [
            tool("connection", "sensor", "RM75 SDK connection status; never initiates motion"),
            tool("joint_states", "sensor", "Read seven RM75 joint angles; output positions are radians"),
            tool("model", "resource", "RM75-6F-V simplified URDF for skeleton rendering"),
        ]
        definitions.extend(tool(name, "sensor", f"Read-only RealMan API2 call: {method}") for name, method in self.METHODS.items())
        joint_properties = {
            f"joint{i}_deg": {
                "type": "number", "minimum": low, "maximum": high,
                "description": f"Required absolute J{i} target in degrees",
            }
            for i, (low, high) in enumerate(JOINT_LIMITS_DEG, 1)
        }
        joint_properties.update({
            "speed_percent": {"type": "integer", "minimum": 1, "maximum": self.max_speed_percent,
                              "default": self.default_speed_percent},
            "timeout_seconds": {"type": "number", "minimum": 2, "maximum": 60,
                                "default": self.default_timeout_seconds},
            "confirm_motion": {"type": "boolean", "description": "Must be true for every movement request"},
        })
        schema = action_schema(
            {
                "set": ([*(f"joint{i}_deg" for i in range(1, 8)), "speed_percent", "timeout_seconds", "confirm_motion"],
                        "Send one complete seven-joint target; all joints are planned together"),
                "stopmotion": ([], "Request a controlled trajectory stop"),
                "info": ([], "Read motion safety and active-action status"),
            },
            joint_properties,
        )
        schema["x-completion"] = {"actions": ["set"], "timeout": 65}
        definitions.append(tool("joint_control", "actuator", "Bounded RM75 joint motion using official API2 movej", schema))
        return definitions

    def start(self):
        self.client.start()

    def stop(self):
        if self._active_action_id and self.client.connected:
            try:
                self.client.command("rm_set_arm_slow_stop")
            except Exception as exc:
                print(f"[rm75] shutdown stop failed: {exc}", flush=True)
        self.client.stop()

    def _motion_status(self):
        return {
            **self.client.status(),
            "active_action_id": self._active_action_id,
            "limits_deg": JOINT_LIMITS_DEG,
            "max_step_deg": self.max_step_deg,
            "max_speed_percent": self.max_speed_percent,
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
        missing = [field for field in joint_fields if field not in args]
        if missing:
            raise ValueError(f"set requires all seven joint targets; missing: {', '.join(missing)}")
        requested = {index: args[field] for index, field in enumerate(joint_fields)}
        current = [float(value) for value in self.client.call("rm_get_joint_degree")]
        if len(current) != 7 or not all(math.isfinite(value) for value in current):
            raise RuntimeError(f"invalid current joint state: {current!r}")
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
            if abs(value - current[index]) > self.max_step_deg:
                raise ValueError(
                    f"joint{index + 1} step {abs(value - current[index]):.3f}deg exceeds {self.max_step_deg}deg limit"
                )
            target[index] = value
        speed = int(args.get("speed_percent", self.default_speed_percent))
        if not 1 <= speed <= self.max_speed_percent:
            raise ValueError(f"speed_percent must be within [1, {self.max_speed_percent}]")
        timeout = float(args.get("timeout_seconds", self.default_timeout_seconds))
        if not math.isfinite(timeout) or not 2 <= timeout <= 60:
            raise ValueError("timeout_seconds must be finite and within [2, 60]")
        return current, target, speed, timeout

    def _acp_callback(self, action_id, status, result):
        import json
        import ssl
        import urllib.request
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                              "tool": "joint_control", "ts": time.time()}).encode()
        try:
            request = urllib.request.Request(f"{url}/api/acp/complete", data=payload,
                                             headers={"Content-Type": "application/json"}, method="POST")
            urllib.request.urlopen(request, timeout=5, context=context).close()
        except Exception as exc:
            print(f"[rm75] ACP callback failed for {action_id}: {exc}", flush=True)

    def _monitor_motion(self, action_id, target, timeout):
        deadline = time.monotonic() + timeout
        status, result = "error", {"reason": "unknown"}
        try:
            while time.monotonic() < deadline:
                if action_id in self._cancelled:
                    status, result = "cancelled", {"reason": "stopmotion"}
                    break
                self._preflight()
                current = [float(value) for value in self.client.call("rm_get_joint_degree")]
                error = max(abs(actual - expected) for actual, expected in zip(current, target))
                if error <= self.target_tolerance_deg:
                    status = "completed"
                    result = {"target_degree": target, "actual_degree": current, "max_error_deg": error}
                    break
                time.sleep(self.poll_interval_seconds)
            else:
                self.client.command("rm_set_arm_slow_stop")
                result = {"reason": "timeout", "timeout_seconds": timeout}
        except Exception as exc:
            try:
                self.client.command("rm_set_arm_slow_stop")
            except Exception:
                pass
            result = {"reason": str(exc)}
        finally:
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
            current, target, speed, timeout = self._prepare_target(args)
            action_id = f"rm75_movej_{uuid4().hex[:10]}"
            self.client.command("rm_movej", target, speed, 0, 0, 0)
            self._active_action_id = action_id
            threading.Thread(target=self._monitor_motion, args=(action_id, target, timeout), daemon=True).start()
            return {"status": "executing", "action_id": action_id, "start_degree": current,
                    "target_degree": target, "speed_percent": speed, "timeout_seconds": timeout}
        except Exception:
            self._motion_lock.release()
            raise

    def _stop_motion(self):
        action_id = self._active_action_id
        self.client.command("rm_set_arm_slow_stop")
        if action_id:
            self._cancelled.add(action_id)
        return {"state": "stop_requested", "action_id": action_id}

    def dispatch(self, action, args):
        name = args.get("_tool_name")
        if action == "start":
            return self.client.status()
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {**self._motion_status(), "topic_out": []}
        if name == "connection":
            return self.client.status()
        if name == "joint_states":
            return self.client.joint_states()
        if name == "model":
            path = Path(__file__).with_name("resource") / "rm75_6f_v.urdf"
            return {"urdf": path.read_text(encoding="utf-8")}
        if name in self.METHODS:
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
    del namespace, ros2
    return [RM75Plugin(RM75SDKClient(config), config)]
