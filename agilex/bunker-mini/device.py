"""MCP tools and ROS topic adapters for AgileX Bunker Mini v2."""

from __future__ import annotations

import json
import math
import os
import ssl
import threading
import time
import urllib.request
from uuid import uuid4

from can_runtime import BunkerCAN, find_gs_usb_interface
from common.odom import build_interface, build_sample
from common.vendor_runtime import tool
from dds_bridge import BunkerDDSBridge
import protocol

MOVE_MIN_DURATION_S = 0.05
MOVE_MAX_DURATION_S = 10.0
MOVE_COMPLETION_TIMEOUT_S = 20


def _validate_move_duration(value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) \
            or not math.isfinite(value) \
            or not MOVE_MIN_DURATION_S <= value <= MOVE_MAX_DURATION_S:
        raise ValueError(
            f"duration_s must be a finite number between "
            f"{MOVE_MIN_DURATION_S:g} and {MOVE_MAX_DURATION_S:g} seconds"
        )
    return float(value)


def _acp_notify(action_id: str, status: str, result: dict) -> None:
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    body = json.dumps({
        "action_id": action_id,
        "status": status,
        "result": result,
        "tool": "base_drive",
        "ts": time.time(),
    }).encode()
    request = urllib.request.Request(
        f"{url}/api/acp/complete", data=body,
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=5, context=context):
            pass
    except Exception as exc:
        print(f"[ACP] completion callback failed ({type(exc).__name__})", flush=True)


class BunkerMiniDevice:
    def __init__(self, config: dict, namespace: str, ros2):
        self.config = config
        self._ros2 = ros2
        self.bridge = BunkerDDSBridge(ros2, namespace)
        self.can = BunkerCAN(config, config.get("can_interface", ""), self._on_odom)
        self._state_thread = None
        self._stop = threading.Event()
        self._enabled_sensors = set()
        self._last_battery_publish = 0.0
        self._motion_enabled = os.environ.get("BUNKER_MOTION_ENABLED", "1") == "1"
        self._direct_until = 0.0
        self._motion_lock = threading.Lock()
        self._active_move_id = None
        self._active_move_stop = None

    def start(self):
        interface = (os.environ.get("BUNKER_CAN_INTERFACE")
                     or self.config.get("can_interface") or find_gs_usb_interface())
        self.can.interface = interface
        self.can.start()
        self._stop.clear()
        self._state_thread = threading.Thread(target=self._publish_loop, daemon=True,
                                              name="bunker-state-publish")
        self._state_thread.start()

    def stop(self):
        self._stop.set()
        if self._state_thread:
            self._state_thread.join(timeout=2)
        self.can.stop()
        self.bridge.close()

    def _publish_loop(self):
        while not self._stop.wait(0.1):
            snapshot = self.can.snapshot()
            if "motor_state" in self._enabled_sensors:
                self.bridge.publish_state(self._motor_state(snapshot))
            if "battery" in self._enabled_sensors and \
                    time.monotonic() - self._last_battery_publish >= 1.0:
                battery = self._battery_state(snapshot)
                if battery is not None:
                    self.bridge.publish_battery(battery)
                    self._last_battery_publish = time.monotonic()
            if "remote_controller" in self._enabled_sensors:
                remote = self._remote_controller_state(snapshot)
                if remote is not None:
                    self.bridge.publish_remote_controller(remote)
            rows, _ = self.can.odometry_window()
            if rows and "odom" in self._enabled_sensors:
                stamp_ms, twist = rows[-1]
                interface = build_interface(provides=["vx"], rate_hz=10,
                                            pose_drift="none")
                sample = build_sample(stamp_ms=stamp_ms,
                    twist=twist, frame="body", pose=None)
                wheel = snapshot.get("wheel_odometry_raw") or {}
                motion_state = snapshot.get("motion_state") or {}
                self.bridge.publish_odom(
                    {"odom_interface": interface, **sample},
                    {"vx": twist[0],
                     "steering_angle_deg": motion_state.get("steering_angle_deg"),
                     "left_wheel_mm": wheel.get("left_wheel_mm"),
                     "right_wheel_mm": wheel.get("right_wheel_mm")})

    @staticmethod
    def _motor_state(snapshot: dict) -> dict:
        system = snapshot.get("system_state") or {}
        motion = snapshot.get("motion_state") or {}
        system_fresh = snapshot.get("system_state_age_ms") is not None and \
            snapshot["system_state_age_ms"] <= 500
        motion_fresh = snapshot.get("motion_state_age_ms") is not None and \
            snapshot["motion_state_age_ms"] <= 500
        vehicle_states = {0: "normal", 1: "estop", 2: "exception"}
        control_modes = {0: "standby", 1: "can", 3: "rc"}
        error_flags = system.get("error_flags") if system_fresh else None
        out = {
            "connected": snapshot.get("connected", False),
            "vehicle_state": vehicle_states.get(system.get("vehicle_state"), "unknown") if system_fresh else "unknown",
            "control_mode": control_modes.get(system.get("control_mode"), "unknown") if system_fresh else "unknown",
            "error_flags": system.get("error_flags") if system_fresh else None,
            "system_counter": system.get("counter") if system_fresh else None,
            "battery_undervoltage_fault": None if error_flags is None else bool(error_flags & 0x01),
            "battery_undervoltage_warning": None if error_flags is None else bool(error_flags & 0x02),
            "remote_lost": None if error_flags is None else bool(error_flags & 0x04),
            "motor_2_communication_fault": None if error_flags is None else bool(error_flags & 0x10),
            "motor_3_communication_fault": None if error_flags is None else bool(error_flags & 0x20),
            "emergency_stop": None if error_flags is None else bool(error_flags & 0x80),
            "linear_velocity_m_s": motion.get("linear_velocity_m_s") if motion_fresh else None,
            "steering_angle_deg": motion.get("steering_angle_deg") if motion_fresh else None,
            "system_state_age_ms": snapshot.get("system_state_age_ms"),
            "motion_state_age_ms": snapshot.get("motion_state_age_ms"),
        }
        for motor_id in (2, 3):
            motor = snapshot.get("motor_state", {}).get(motor_id, {})
            fresh = snapshot.get("motor_state_age_ms", {}).get(motor_id) is not None and \
                snapshot["motor_state_age_ms"][motor_id] <= 500
            fields = ("speed_rpm", "current_a", "position_pulses", "driver_voltage_v",
                      "driver_temperature_c", "motor_temperature_c", "status_flags")
            for field in fields:
                value = motor.get(field)
                out[f"motor_{motor_id}_{field}"] = value if fresh else None
            flags = motor.get("status_flags", 0) if fresh else 0
            for name, mask in (("voltage_low", 0x01), ("motor_overtemperature", 0x02),
                               ("overcurrent", 0x04), ("driver_overtemperature", 0x08),
                               ("sensor_fault", 0x10), ("driver_error", 0x20),
                               ("motor_disabled", 0x40)):
                out[f"motor_{motor_id}_{name}"] = bool(flags & mask) if fresh else None
            out[f"motor_{motor_id}_state_age_ms"] = snapshot.get("motor_state_age_ms", {}).get(motor_id)
        return out

    @staticmethod
    def _battery_state(snapshot: dict) -> dict | None:
        battery = snapshot.get("battery_state")
        age = snapshot.get("battery_state_age_ms")
        if battery is None or age is None or age > 1500:
            system = snapshot.get("system_state") or {}
            system_age = snapshot.get("system_state_age_ms")
            if system_age is None or system_age > 500:
                return None
            return {"soc_percent": None, "soh_percent": None,
                    "voltage_v": system.get("battery_voltage_v"),
                    "current_a": None, "temperature_c": None,
                    "feedback_age_ms": system_age}
        alarms = snapshot.get("battery_alarm") or {}
        alarm_fresh = snapshot.get("battery_alarm_age_ms") is not None and \
            snapshot["battery_alarm_age_ms"] <= 1500
        alarm1 = alarms.get("alarm_status_1", 0)
        alarm2 = alarms.get("alarm_status_2", 0)
        warning1 = alarms.get("warning_status_1", 0)
        warning2 = alarms.get("warning_status_2", 0)
        out = {**battery, "feedback_age_ms": age,
               "alarm_feedback_age_ms": snapshot.get("battery_alarm_age_ms")}
        for prefix, status1, status2 in (("alarm", alarm1, alarm2),
                                          ("warning", warning1, warning2)):
            out[f"{prefix}_overvoltage"] = bool(status1 & 0x02) if alarm_fresh else None
            out[f"{prefix}_undervoltage"] = bool(status1 & 0x04) if alarm_fresh else None
            out[f"{prefix}_overtemperature"] = bool(status1 & 0x08) if alarm_fresh else None
            out[f"{prefix}_undertemperature"] = bool(status1 & 0x10) if alarm_fresh else None
            out[f"{prefix}_discharge_overcurrent"] = bool(status1 & 0x80) if alarm_fresh else None
            out[f"{prefix}_charge_overcurrent"] = bool(status2 & 0x01) if alarm_fresh else None
        return out

    @staticmethod
    def _remote_controller_state(snapshot: dict) -> dict | None:
        remote = snapshot.get("remote_controller")
        age = snapshot.get("remote_controller_age_ms")
        if remote is None or age is None or age > 500:
            return None
        return {**remote, "feedback_age_ms": age}

    def _on_odom(self, _value):
        return

    def _run_move(self, action_id: str, stop_event: threading.Event,
                  linear: float, angular: float, duration: float) -> None:
        status = "error"
        result = {}
        target_started = False
        try:
            # Serialize target acquisition with stop so a cancellation that
            # arrives before this worker starts cannot be followed by motion.
            with self._motion_lock:
                if stop_event.is_set():
                    status = "cancelled"
                else:
                    self.can.set_target(linear, angular, lease_s=duration)
                    target_started = True
            if status != "cancelled":
                deadline = time.monotonic() + duration
                while not stop_event.is_set():
                    snapshot = self.can.snapshot()
                    if snapshot.get("last_tx_error"):
                        raise RuntimeError("CAN command transmission failed")
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    stop_event.wait(min(0.02, remaining))
                status = "cancelled" if stop_event.is_set() else "completed"
            if target_started:
                result = self.can.stop_motion()
        except Exception as exc:
            status = "cancelled" if stop_event.is_set() else "error"
            result = {"error": type(exc).__name__}
            if target_started:
                try:
                    self.can.stop_motion()
                except Exception:
                    pass
        finally:
            with self._motion_lock:
                if self._active_move_id == action_id:
                    self._active_move_id = None
                    self._active_move_stop = None
                    self._direct_until = 0.0
        _acp_notify(action_id, status, result)

    def get_tools(self):
        max_linear = min(0.2, float(self.config.get("max_linear_m_s", 0.2)))
        max_yaw_deg_s = math.degrees(min(
            0.4, float(self.config.get("max_angular_rad_s", 0.4))))
        schema = {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["move"]},
                "vx": {"type": "number", "minimum": -max_linear, "maximum": max_linear,
                       "description": "Forward velocity in m/s"},
                "vyaw": {"type": "number", "minimum": -max_yaw_deg_s,
                         "maximum": max_yaw_deg_s,
                         "description": "Yaw velocity in degrees/s"},
                "duration_s": {"type": "number", "minimum": MOVE_MIN_DURATION_S,
                               "maximum": MOVE_MAX_DURATION_S,
                               "description": "Move duration in seconds"},
            },
            "required": ["action"],
            "x-action-params": {
                "move": {"params": ["vx", "vyaw", "duration_s"], "description": "Move for a bounded duration; vx is m/s and vyaw is degrees/s"},
            },
            "x-completion": {
                "actions": ["move"],
                "timeout": MOVE_COMPLETION_TIMEOUT_S,
            },
            "x-resource": "base",
        }
        odom_topics = [{"topic": self.bridge._odom_topic, "format": "data/json"}]
        odom_topics.extend({"topic": topic, "format": "data/json"}
                           for topic in self.bridge._odom_field_topics.values())
        return [
            tool("motor_state", "sensor", "Bunker Mini chassis feedback, motor telemetry and faults",
                 topic_out=[{"topic": self.bridge._state_topic, "format": "data/json"}]),
            tool("battery", "sensor", "Bunker Mini BMS state, voltage, current, temperature and alarms",
                 topic_out=[{"topic": self.bridge._battery_topic, "format": "data/json"}]),
            tool("remote_controller", "sensor", "Bunker Mini remote switches and stick positions",
                 topic_out=[{"topic": self.bridge._remote_controller_topic, "format": "data/json"}]),
            tool("odom", "sensor", "Measured Bunker Mini odometry",
                 topic_out=odom_topics),
            tool("base_drive", "actuator", "Guarded Bunker Mini base movement", schema),
        ]

    def dispatch(self, action, args):
        tool_name = args.get("_tool_name", "base_drive")
        if tool_name in ("motor_state", "battery", "remote_controller", "odom"):
            if action == "start":
                self._enabled_sensors.add(tool_name)
                return {"state": "running"}
            if action == "stop":
                self._enabled_sensors.discard(tool_name)
                return {"state": "idle"}
            if action == "info":
                result = {"state": "running" if tool_name in self._enabled_sensors else "idle"}
                if tool_name == "motor_state":
                    result["data"] = self._motor_state(self.can.snapshot())
                elif tool_name == "battery":
                    result["data"] = self._battery_state(self.can.snapshot())
                elif tool_name == "remote_controller":
                    result["data"] = self._remote_controller_state(self.can.snapshot())
                else:
                    result["odom_interface"] = build_interface(
                        provides=["vx"], rate_hz=10, pose_drift="none")
                return result
        if tool_name == "base_drive" and action == "start":
            return {"state": "ready"}
        if tool_name == "base_drive" and action == "stop":
            with self._motion_lock:
                if self._active_move_stop is not None:
                    self._active_move_stop.set()
                self._direct_until = 0.0
                result = self.can.stop_motion()
                return {**result, "state": "idle"}
        if action == "info" and tool_name == "base_drive":
            return {"state": self.can.snapshot(),
                    "odom_interface": build_interface(provides=["vx"],
                        rate_hz=10, pose_drift="none")}
        if tool_name == "base_drive" and action == "move":
            if not self._motion_enabled:
                raise RuntimeError("movement is disabled by BUNKER_MOTION_ENABLED=0")
            duration = _validate_move_duration(args.get("duration_s"))
            max_linear = min(0.2, float(self.config.get("max_linear_m_s", 0.2)))
            linear = BunkerCAN._limit(args.get("vx"), max_linear, "linear")
            vyaw = args.get("vyaw")
            if isinstance(vyaw, bool) or not isinstance(vyaw, (int, float)) or not math.isfinite(vyaw):
                raise ValueError("vyaw must be a finite number in degrees/s")
            max_yaw_rad_s = min(
                0.4, float(self.config.get("max_angular_rad_s", 0.4)))
            angular = BunkerCAN._limit(
                math.radians(float(vyaw)), max_yaw_rad_s, "angular")
            self.can.request_control_mode(protocol.CONTROL_CAN)
            # Recheck live chassis state and configured bounds immediately
            # before the target is registered, after mode negotiation.
            linear, angular = self.can.validate_target(linear, angular)
            action_id = f"bunker_move_{uuid4().hex[:12]}"
            stop_event = threading.Event()
            with self._motion_lock:
                if self._active_move_id is not None:
                    return {"error": "a timed base move is already running"}
                self.can.stop_motion()
                self._direct_until = time.monotonic() + duration
                self._active_move_id = action_id
                self._active_move_stop = stop_event
            worker = threading.Thread(
                target=self._run_move,
                args=(action_id, stop_event, linear, angular, duration),
                daemon=True, name="bunker-timed-move")
            worker.start()
            return {"state": "running", "action_id": action_id,
                    "duration_s": duration, "vyaw_deg_s": float(vyaw)}
        raise ValueError(f"unknown base_drive action: {action}")
