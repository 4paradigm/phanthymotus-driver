"""MCP tools and ROS topic adapters for AgileX Bunker Pro v2."""

from __future__ import annotations

import json
import os
import threading
import time

from can_runtime import BunkerCAN, find_gs_usb_interface
from common.odom import build_interface, build_sample
from common.control.sink import ControlSink
from common.vendor_runtime import tool
from dds_bridge import BunkerDDSBridge
import protocol


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
        self._motion_enabled = os.environ.get("BUNKER_MOTION_ENABLED", "0") == "1"
        self._direct_until = 0.0
        self._motion_lock = threading.Lock()
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
                raise RuntimeError("movement is disabled; set BUNKER_MOTION_ENABLED=1 after commissioning")
            values = payload.get("values")
            if not isinstance(values, list) or len(values) != 6:
                raise ValueError("control/velocity requires a six-axis twist values list")
            if any(abs(float(values[index])) > 1e-9 for index in (1, 2, 3, 4)):
                raise ValueError("Bunker Pro supports only vx and wz")
            with self._motion_lock:
                if time.monotonic() < self._direct_until:
                    return
                result = self._sink.submit(payload)
            if not result.applied and result.verdict.value != "dropped":
                print(f"[bunker] rejected control command: {result.reason}", flush=True)
            elif result.applied:
                self._sink.tick()
        except Exception as exc:
            print(f"[bunker] rejected streamed velocity command: {exc}", flush=True)

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
            return {"state": "running", "input_topic": input_topic}
        if tool_name == "bunker_base" and action == "stop":
            self.bridge.bind_control_topic(None)
            with self._motion_lock:
                self._direct_until = 0.0
                return self.can.stop_motion()
        if action == "info":
            return {"state": self.can.snapshot(),
                    "control_interface": self._control_descriptor(),
                    "odom_interface": build_interface(provides=["vx", "wz"],
                        rate_hz=10, pose_drift="none")}
        if action == "enable_can":
            return self.can.request_control_mode(protocol.CONTROL_CAN)
        if action == "stop":
            return self.can.stop_motion()
        if action == "move":
            if not self._motion_enabled:
                raise RuntimeError("movement is disabled; set BUNKER_MOTION_ENABLED=1 after commissioning")
            if args.get("_tool_name") != "bunker_base":
                raise ValueError("invalid tool")
            duration = float(args.get("duration_s", 0))
            with self._motion_lock:
                self.can.stop_motion()
                self._direct_until = time.monotonic() + duration
            try:
                return self.can.move_for(float(args["vx"]), float(args["wz"]), duration)
            finally:
                with self._motion_lock:
                    self._direct_until = 0.0
        raise ValueError(f"unknown bunker_base action: {action}")
