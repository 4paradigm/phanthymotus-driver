"""ROS/MCP adapter for T800 teleoperation, reusing JointOverrideCommand."""
from __future__ import annotations

import json
import threading
import time

from common.teleop_contract import COMMAND_TOPIC, STATE_TOPIC, MAX_WIRE_BYTES
from teleop_control import TeleopControl
from teleop_kinematics import ARM_INDICES


class TeleopControlPlugin:
    def __init__(self, config, namespace, ros2, state):
        self.config, self.ros2, self.state = config, ros2, state
        self.control = TeleopControl(config["plugins"]["teleop_control"], self._feedback, self._publish)
        self._planner_lock = threading.Lock()
        self._planner = {}
        self._planner_received = None
        self._closed = threading.Event()
        self._threads = []
        self._robot_node = self._core_node = self._publisher = self._monitor = None
        self._monitor_error = None

    def get_tool(self):
        return {"name": "teleop_control", "type": "actuator", "multiInstance": False,
            "description": "T800 开发版双臂遥操：连接 PICO teleop_device，先切换下肢平衡并确认规划器空闲，再启动项目。松开双握把建立基准，双握跟随，松握保持，再握不重标定。每臂 5 关节，位置优先、姿态近似；状态在监控面板显示。停止释放关节覆盖，由 Native SDK 接管。首次联调默认为 Shadow。",
            "inputSchema": {"type": "object", "required": ["action"],
                "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"]},
                    "input_topic": {"type": "string"}},
                "x-resource": ["arm_l", "arm_r"],
                "x-action-params": {"info": {"params": []},
                    "start": {"params": ["input_topic"]}, "stop": {"params": []}},
                "x-hooks": {"on_interrupt_motion": {"action": "stop"}}},
            "topic_in": [{"port_id": "command", "format": "data/teleop-cmd", "topic": COMMAND_TOPIC}],
            "topic_out": [{"port_id": "state", "format": "data/teleop-state", "topic": STATE_TOPIC}]}

    def start(self):
        # Bundle startup creates resources only. Canvas start is dispatch(start).
        if self._threads:
            return
        if self.config["ros"]["core_domain_id"] != 42:
            raise ValueError("teleop_requires_core_domain_42")
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from std_msgs.msg import String
        from interface_protocol.msg import JointOverrideCommand, JointMotionPlanState

        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self._message_type, self._string_type = JointOverrideCommand, String
        self._robot_node = Node("t800_teleop_execution", context=self.ros2.ctx_robot)
        self._core_node = Node("t800_teleop_input", context=self.ros2.ctx_core)
        self._publisher = self._robot_node.create_publisher(
            JointOverrideCommand, self.config["topics"]["joint_override"], reliable)
        self._robot_node.create_subscription(JointMotionPlanState,
            self.config["topics"]["joint_plan_state"], self._on_planner, best_effort)
        self._core_node.create_subscription(String, COMMAND_TOPIC, self._on_input, reliable)
        self._monitor = self._core_node.create_publisher(String, STATE_TOPIC, best_effort)
        self._core_node.create_timer(.2, self._publish_monitor)
        self.ros2.executor_robot.add_node(self._robot_node)
        self.ros2.executor_core.add_node(self._core_node)
        self._closed.clear()
        for name, callback, period in (("watchdog", self.control.tick, .01),
                                        ("ik", self.control.solve_once, .02)):
            thread = threading.Thread(target=self._run, args=(callback, period),
                daemon=True, name="t800-teleop-" + name)
            self._threads.append(thread)
            thread.start()

    def _run(self, callback, period):
        while not self._closed.is_set():
            started = time.monotonic()
            try:
                callback()
            except Exception as exc:
                with self.control.lock:
                    self.control._fault("control_loop_failed: " + str(exc))
            self._closed.wait(max(.001, period-(time.monotonic()-started)))

    def _on_planner(self, message):
        with self._planner_lock:
            self._planner = {"status": int(message.status), "request_id": int(message.request_id)}
            self._planner_received = time.monotonic()

    def _feedback(self):
        with self._planner_lock:
            planner = {**self._planner, "age_sec": None if self._planner_received is None
                else time.monotonic()-self._planner_received}
        return {"joints": self.state.dispatch("joints", {}),
                "motion": self.state.dispatch("motion_state", {}), "planner": planner}

    @staticmethod
    def _unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate_json_field")
            result[key] = value
        return result

    def _on_input(self, message):
        try:
            if len(message.data.encode("utf-8")) > MAX_WIRE_BYTES:
                raise ValueError("input_too_large")
            value = json.loads(message.data, object_pairs_hook=self._unique)
            self.control.receive(value)
        except (ValueError, TypeError, RecursionError) as exc:
            with self.control.lock:
                self.control.input_error = str(exc)
                self.control.rejected += 1

    def _publish(self, position, velocity, weight):
        message = self._message_type()
        message.header.stamp = self._robot_node.get_clock().now().to_msg()
        message.weight = weight
        message.joint_indices = list(ARM_INDICES)
        message.position, message.velocity = position.tolist(), velocity.tolist()
        message.feed_forward_torque = [0.] * 10
        message.torque = [0.] * 10
        # Official T800 arm gains from the existing motion_recorder preset.
        message.stiffness = [30., 30., 15., 30., 15., 40., 40., 20., 40., 20.] if weight else [0.] * 10
        message.damping = [1.] * 10 if weight else [0.] * 10
        self._publisher.publish(message)

    def _publish_monitor(self, *, force=False):
        if self._monitor is None or (not force and not self.control.motion_active()):
            return
        try:
            message = self._string_type()
            message.data = json.dumps(self.control.feedback(), allow_nan=False)
            self._monitor.publish(message)
            self._monitor_error = None
        except Exception as exc:
            self._monitor_error = str(exc)

    def dispatch(self, action, args):
        try:
            if action in ("info", "status"):
                return {**self.control.info(), "monitor_error": self._monitor_error}
            if action == "start":
                return self.control.begin(args)
            if action == "stop":
                return self.halt()
            raise ValueError("unknown_teleop_action")
        except (ValueError, RuntimeError) as exc:
            return {"state": "error", "error": str(exc)}

    def motion_active(self):
        return self.control.motion_active()

    def halt(self):
        result = self.control.halt()
        self._publish_monitor(force=True)
        return result

    def stop(self):
        result = self.halt()
        if result["override_release_pending"]:
            # Keep the watchdog alive to retry. Never advertise a clean stop or
            # destroy the only publisher while release publication is failing.
            raise RuntimeError("teleop override release is still pending")
        self._closed.set()
        for thread in self._threads:
            thread.join(timeout=.5)
        if any(thread.is_alive() for thread in self._threads):
            raise RuntimeError("teleop worker is still stopping")
        self._threads = []
        for node, executor in ((self._core_node, self.ros2.executor_core),
                               (self._robot_node, self.ros2.executor_robot)):
            if node is not None:
                executor.remove_node(node)
                node.destroy_node()
        self._core_node = self._robot_node = None
