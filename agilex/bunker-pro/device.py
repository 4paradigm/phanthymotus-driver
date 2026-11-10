"""MCP tools and ROS topic adapters for AgileX Bunker Pro v2."""

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
from common.control.sink import ControlSink
from common.vendor_runtime import tool
from dds_bridge import BunkerDDSBridge
import protocol

MOVE_MIN_DURATION_S = 0.05
MOVE_MAX_DURATION_S = 10.0
MOVE_COMPLETION_TIMEOUT_S = 20
REJECTION_LOG_INTERVAL_S = 5.0


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
        "tool": "bunker_base",
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


class BunkerProDevice:
    def __init__(self, config: dict, namespace: str, ros2):
        from rclpy.node import Node
        from std_msgs.msg import String

        self.config = config
        self._ros2 = ros2
        self.bridge = BunkerDDSBridge(ros2, namespace)
        self.can = BunkerCAN(config, config.get("can_interface", ""), self._on_odom)
        self._node = Node("bunker_pro_can_adapter", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._String = String
        self._command_sub = self._node.create_subscription(
            String, self.bridge._command_topic, self._on_command, 10)
        self._state_thread = None
        self._stop = threading.Event()
        self._enabled_sensors = set()
        self._motion_enabled = os.environ.get("BUNKER_MOTION_ENABLED", "1") == "1"
        self._direct_until = 0.0
        self._motion_lock = threading.Lock()
        self._active_move_id = None
        self._active_move_stop = None
        self._rejection_log_lock = threading.Lock()
        self._rejection_counts = {}
        self._last_rejection_log_at = 0.0
        self._sink = ControlSink(self._control_descriptor(), self._apply_stream,
                                 on_watchdog=self._watchdog_stop,
                                 on_abort=self._watchdog_stop)
        self._node.create_timer(0.02, self._sink.tick)

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
        self._ros2.executor_robot.remove_node(self._node)
        self._node.destroy_node()
        self.bridge.close()

    def _publish_loop(self):
        while not self._stop.wait(0.1):
            if "bunker_state" in self._enabled_sensors:
                self.bridge.publish_state(self.can.snapshot())
            rows, motion = self.can.odometry_window()
            if rows and "bunker_odom" in self._enabled_sensors:
                stamp_ms = int(time.time() * 1000)
                interface = build_interface(provides=["vx", "wz"], rate_hz=10,
                                            pose_drift="none")
                sample = build_sample(stamp_ms=stamp_ms,
                    twist=rows[-1], frame="body", pose=None,
                    vendor={"motion_state": motion,
                            "wheel_odometry_raw": self.can.snapshot()["wheel_odometry_raw"]})
                self.bridge.publish_odom({"odom_interface": interface, **sample})

    def _control_descriptor(self):
        max_linear = min(0.2, float(self.config.get("max_linear_m_s", 0.2)))
        max_angular = min(0.4, float(self.config.get("max_angular_rad_s", 0.4)))
        return {
            "control_interface": "motus.control/1", "mode": "twist", "dof": 6,
            "joint_names": ["vx", "vy", "vz", "wx", "wy", "wz"],
            "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
            "limits": {
                "lower": [-max_linear, 0, 0, 0, 0, -max_angular],
                "upper": [max_linear, 0, 0, 0, 0, max_angular],
                "max_delta_per_step": [0.05, 1e-6, 1e-6, 1e-6, 1e-6, 0.1],
            },
            "frame": "base_link", "rate": {"max_hz": 50, "expected_hz": 20,
                "watchdog_ms": 350, "max_obs_age_ms": 1000},
            "force_torque": None,
        }

    def _apply_stream(self, values, _gripper):
        if not self._motion_enabled:
            raise RuntimeError("movement is disabled")
        if time.monotonic() < self._direct_until:
            raise RuntimeError("direct base action owns the motion resource")
        if abs(values[1]) > 1e-9 or abs(values[2]) > 1e-9 or any(abs(v) > 1e-9 for v in values[3:5]):
            raise ValueError("Bunker Pro supports only vx and wz")
        self.can.set_target(values[0], values[5], lease_s=0.35)

    def _watchdog_stop(self):
        if time.monotonic() >= self._direct_until:
            self.can.stop_motion()

    def _on_odom(self, _value):
        return

    def _on_command(self, message):
        try:
            payload = json.loads(message.data)
            if not self._motion_enabled:
                self._log_stream_rejection("motion_disabled")
                return
            values = payload.get("values")
            if not isinstance(values, list) or len(values) != 6:
                self._log_stream_rejection("invalid_twist_shape")
                return
            if any(abs(float(values[index])) > 1e-9 for index in (1, 2, 3, 4)):
                self._log_stream_rejection("unsupported_twist_axis")
                return
            with self._motion_lock:
                if time.monotonic() < self._direct_until:
                    return
                result = self._sink.submit(payload)
            if not result.applied and result.verdict.value != "dropped":
                self._log_stream_rejection("control_sink_rejected")
            elif result.applied:
                self._sink.tick()
        except json.JSONDecodeError:
            self._log_stream_rejection("invalid_json")
        except (TypeError, ValueError):
            self._log_stream_rejection("invalid_command")
        except Exception:
            self._log_stream_rejection("control_callback_error")

    def _log_stream_rejection(self, category: str) -> None:
        now = time.monotonic()
        with self._rejection_log_lock:
            self._rejection_counts[category] = self._rejection_counts.get(category, 0) + 1
            if self._last_rejection_log_at and \
                    now - self._last_rejection_log_at < REJECTION_LOG_INTERVAL_S:
                return
            summary = ", ".join(
                f"{name}={count}" for name, count in sorted(self._rejection_counts.items()))
            self._rejection_counts.clear()
            self._last_rejection_log_at = now
        print(f"[bunker] rejected streamed control messages ({summary})", flush=True)

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
        schema = {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["info", "enable_can", "move", "stop"]},
                "vx": {"type": "number", "description": "Forward velocity in m/s"},
                "wz": {"type": "number", "description": "Yaw velocity in rad/s"},
                "duration_s": {"type": "number", "description": "Move duration, 0.05 to 10 seconds"},
            },
            "required": ["action"],
            "x-action-params": {
                "info": {"params": [], "description": "Read CAN and chassis state"},
                "enable_can": {"params": [], "description": "Explicitly request CAN control mode"},
                "move": {"params": ["vx", "wz", "duration_s"], "description": "Move for a bounded duration"},
                "stop": {"params": [], "description": "Stop base motion"},
            },
            "x-completion": {
                "actions": ["move"],
                "timeout": MOVE_COMPLETION_TIMEOUT_S,
            },
            "x-resource": "base",
        }
        return [
            tool("bunker_state", "sensor", "Bunker Pro chassis and CAN status",
                 topic_out=[{"topic": self.bridge._state_topic, "format": "data/json"}]),
            tool("bunker_odom", "sensor", "Measured Bunker Pro body velocity and raw wheel counts",
                 topic_out=[{"topic": self.bridge._odom_topic, "format": "data/json"}]),
            tool("bunker_base", "actuator", "Guarded Bunker Pro base control",
                 schema,
                 topic_in=[{"format": "control/velocity"}])
        ]

    def dispatch(self, action, args):
        tool_name = args.get("_tool_name", "bunker_base")
        if tool_name in ("bunker_state", "bunker_odom"):
            if action == "start":
                self._enabled_sensors.add(tool_name)
                return {"state": "running"}
            if action == "stop":
                self._enabled_sensors.discard(tool_name)
                return {"state": "idle"}
            if action == "info":
                result = {"state": self.can.snapshot()}
                if tool_name == "bunker_odom":
                    result["odom_interface"] = build_interface(
                        provides=["vx", "wz"], rate_hz=10, pose_drift="none")
                return result
        if tool_name == "bunker_base" and action == "start":
            input_topic = str(args.get("input_topic") or "").strip()
            if not input_topic:
                return {"error": "Missing input_topic"}
            self.bridge.bind_control_topic(input_topic)
            return {"state": "ready", "input_topic": input_topic}
        if tool_name == "bunker_base" and action == "stop":
            self.bridge.bind_control_topic(None)
            with self._motion_lock:
                if self._active_move_stop is not None:
                    self._active_move_stop.set()
                self._direct_until = 0.0
                result = self.can.stop_motion()
                return {**result, "state": "idle"}
        if action == "info":
            return {"state": self.can.snapshot(),
                    "control_interface": self._control_descriptor(),
                    "odom_interface": build_interface(provides=["vx", "wz"],
                        rate_hz=10, pose_drift="none")}
        if action == "enable_can":
            return self.can.request_control_mode(protocol.CONTROL_CAN)
        if action == "stop":
            result = self.can.stop_motion()
            return {**result, "state": "idle"}
        if action == "move":
            if not self._motion_enabled:
                raise RuntimeError("movement is disabled by BUNKER_MOTION_ENABLED=0")
            if args.get("_tool_name") != "bunker_base":
                raise ValueError("invalid tool")
            duration = _validate_move_duration(args.get("duration_s"))
            linear, angular = self.can.validate_target(args.get("vx"), args.get("wz"))
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
                    "duration_s": duration}
        raise ValueError(f"unknown bunker_base action: {action}")
