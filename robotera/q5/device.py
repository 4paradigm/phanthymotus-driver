#!/usr/bin/env python3
"""RobotEra Q5 plugins backed by the vendor ROS2 interfaces."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

from common.vendor_runtime import action_schema, jsonable, tool


def _wait_future(future, timeout: float = 10.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    if not future.done():
        raise TimeoutError(f"ROS2 request timed out after {timeout:.1f}s")
    error = future.exception()
    if error:
        raise error
    return future.result()


class Q5NodeSet:
    def __init__(self, config: dict, namespace: str, ros2):
        from geometry_msgs.msg import TwistStamped
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from xbot_common_interfaces.action import SimpleActions
        from xbot_common_interfaces.msg import HybridJointCommand, ServoPose
        from xbot_common_interfaces.srv import DynamicLaunch, StringMessage

        self.config = config
        self.robot = Node("q5_driver_robot", context=ros2.ctx_robot)
        self.core = Node("q5_driver_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self.robot)
        ros2.executor_core.add_node(self.core)
        topics = config["topics"]
        self.joint_state = None
        self.joint_lock = threading.Lock()
        self.state_pub = self.core.create_publisher(String, f"/{namespace}/q5/state", 10)
        self.robot.create_subscription(JointState, topics["joint_state"], self._on_joint_state, 10)
        self.body_pub = self.robot.create_publisher(HybridJointCommand, topics["body_command"], 1)
        self.hand_pub = self.robot.create_publisher(HybridJointCommand, topics["hand_command"], 10)
        self.base_pub = self.robot.create_publisher(TwistStamped, topics["base_command"], 10)
        self.servo_pub = self.robot.create_publisher(ServoPose, topics["servo_pose"], 10)
        self.dynamic_launch = self.robot.create_client(DynamicLaunch, "/dynamic_launch")
        self.ready = self.robot.create_client(Trigger, "/ready_service")
        self.activate = self.robot.create_client(Trigger, "/activate_service")
        self.deactivate = self.robot.create_client(Trigger, "/deactivate_service")
        self.teleop = self.robot.create_client(StringMessage, "/teleoperation/service")
        self.simple_action = ActionClient(self.robot, SimpleActions, "/simple_actions")
        self.last_mpc_state: int | None = None
        self.closed = False

    def _on_joint_state(self, msg):
        from std_msgs.msg import String

        with self.joint_lock:
            self.joint_state = msg
        payload = {
            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
            "joints": [
                {
                    "name": name,
                    "position": msg.position[index] if index < len(msg.position) else None,
                    "velocity": msg.velocity[index] if index < len(msg.velocity) else None,
                    "effort": msg.effort[index] if index < len(msg.effort) else None,
                }
                for index, name in enumerate(msg.name)
            ],
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(out)

    def joint_snapshot(self) -> list[dict]:
        with self.joint_lock:
            msg = self.joint_state
            if msg is None:
                return []
            return [
                {
                    "name": name,
                    "position": msg.position[index] if index < len(msg.position) else None,
                    "velocity": msg.velocity[index] if index < len(msg.velocity) else None,
                    "effort": msg.effort[index] if index < len(msg.effort) else None,
                }
                for index, name in enumerate(msg.name)
            ]

    def call(self, client, request, timeout: float = 10.0):
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            raise RuntimeError(f"ROS2 service unavailable: {client.srv_name}")
        return _wait_future(client.call_async(request), timeout)

    def simple_pose(self, name: str, duration: float):
        from xbot_common_interfaces.action import SimpleActions

        if not self.simple_action.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("ROS2 action unavailable: /simple_actions")
        goal = SimpleActions.Goal()
        goal.action_name = name
        goal.time_cost = float(duration)
        handle = _wait_future(self.simple_action.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError(f"Q5 action rejected: {name}")
        wrapped = _wait_future(handle.get_result_async(), duration + 10.0)
        result = wrapped.result
        return {"action": name, "result": int(result.result), "message": result.message}

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.robot.destroy_node()
        self.core.destroy_node()


class Q5StatePlugin:
    CAPABILITIES = [
        "joint_state", "body_trajectory", "dual_arm_mpc", "xhand", "wheel_base",
        "lidar_mapping", "topological_navigation", "teleoperation", "choreography",
    ]

    def __init__(self, nodes: Q5NodeSet, namespace: str):
        self.nodes = nodes
        self.topic = f"/{namespace}/q5/state"

    def get_tools(self):
        return [
            tool("joints", "sensor", "Q5 joint position, velocity and effort", topic_out=[{"topic": self.topic, "format": "data/json"}]),
            tool("capabilities", "sensor", "Q5 driver capability discovery"),
            tool("ros_graph", "sensor", "Live Q5 ROS2 topic/service/action discovery"),
        ]

    def start(self):
        pass

    def stop(self): self.nodes.close()

    def dispatch(self, action, args):
        name = args.get("_tool_name")
        if action == "info":
            return {"state": "ready", "topic_out": [{"topic": self.topic, "format": "data/json"}]} if name == "joints" else {"state": "ready"}
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if name == "joints":
            return {"state": "connected" if self.nodes.joint_state is not None else "waiting", "joints": self.nodes.joint_snapshot(), "topic": self.topic}
        if name == "capabilities":
            return {"model": "RobotEra Q5", "capabilities": self.CAPABILITIES, "source": "roboterax/xbot_sdk_api + era_nav_msgs"}
        if name == "ros_graph":
            return {
                "topics": self.nodes.robot.get_topic_names_and_types(),
                "services": self.nodes.robot.get_service_names_and_types(),
                "nodes": self.nodes.robot.get_node_names_and_namespaces(),
            }
        return None


class Q5LifecyclePlugin:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_tool(self):
        return tool("lifecycle", "actuator", "Start, initialize and switch Q5 joint/algorithm control", action_schema(
            {
                "start_joint_service": (["app_name", "sync_control", "launch_mode"], "Start Q5 joint controller"),
                "initialize": (["timeout"], "Initialize all Q5 joints"),
                "activate": ([], "Activate algorithm control"),
                "deactivate": ([], "Deactivate algorithm control"),
                "zero": (["duration"], "Move Q5 to zero pose"),
                "lift_up": (["duration"], "Move Q5 to lift-up pose"),
            },
            {
                "app_name": {"type": "string"}, "sync_control": {"type": "boolean"},
                "launch_mode": {"type": "string", "enum": ["pos", "pd", "no_hand_pd"]},
                "timeout": {"type": "number"}, "duration": {"type": "number"},
            },
        ))

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        from std_srvs.srv import Trigger
        from xbot_common_interfaces.srv import DynamicLaunch

        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action == "start_joint_service":
            req = DynamicLaunch.Request()
            req.app_name = str(args.get("app_name", "phanthy_motus"))
            req.sync_control = bool(args.get("sync_control", False))
            req.launch_mode = str(args.get("launch_mode", "pos"))
            return jsonable(self.nodes.call(self.nodes.dynamic_launch, req, 10.0))
        if action in ("initialize", "activate", "deactivate"):
            client = {"initialize": self.nodes.ready, "activate": self.nodes.activate, "deactivate": self.nodes.deactivate}[action]
            return jsonable(self.nodes.call(client, Trigger.Request(), float(args.get("timeout", 25.0))))
        if action in ("zero", "lift_up"):
            return self.nodes.simple_pose(action, float(args.get("duration", 4.0)))
        return None


class Q5BodyPlugin:
    def __init__(self, nodes):
        self.nodes = nodes
        self._cancel = threading.Event()
        self._thread = None
        self._status = {"state": "idle"}

    def get_tool(self):
        return tool("body", "actuator", "Q5 named-joint trajectory control", action_schema(
            {
                "move_joints": (["targets", "duration", "rate_hz", "kp", "kd"], "Interpolate named Q5 joints to target radians"),
                "hold": (["duration"], "Hold all joints at the latest measured pose"),
                "cancel": ([], "Cancel the active body trajectory"),
                "status": ([], "Get active trajectory status"),
            },
            {
                "targets": {"type": "object", "additionalProperties": {"type": "number"}},
                "duration": {"type": "number"}, "rate_hz": {"type": "number"},
                "kp": {"type": "number"}, "kd": {"type": "number"},
            },
        ))

    def start(self): pass
    def stop(self): self._cancel.set()

    def _run(self, targets, duration, rate_hz, kp, kd):
        from xbot_common_interfaces.msg import HybridJointCommand

        joints = self.nodes.joint_snapshot()
        if not joints:
            raise RuntimeError("No /joint_states received")
        names = [j["name"] for j in joints]
        start = [float(j["position"] or 0.0) for j in joints]
        end = [float(targets.get(name, start[index])) for index, name in enumerate(names)]
        self._cancel.clear()
        started = time.monotonic()
        period = 1.0 / max(1.0, rate_hz)
        while not self._cancel.is_set():
            ratio = min(1.0, (time.monotonic() - started) / max(0.01, duration))
            msg = HybridJointCommand()
            msg.header.stamp = self.nodes.robot.get_clock().now().to_msg()
            msg.joint_name = names
            msg.position = [a + (b - a) * ratio for a, b in zip(start, end)]
            msg.velocity = [0.0] * len(names)
            msg.feedforward = [0.0] * len(names)
            msg.kp = [kp] * len(names)
            msg.kd = [kd] * len(names)
            self.nodes.body_pub.publish(msg)
            self._status = {"state": "running", "progress": ratio, "targets": targets}
            if ratio >= 1.0:
                break
            time.sleep(period)

    def run_blocking(self, targets, duration, rate_hz=100.0, kp=100.0, kd=1.0):
        self._run(targets, duration, rate_hz, kp, kd)

    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": self._cancel.set(); return {"state": "idle"}
        if action == "cancel": self._cancel.set(); return {"state": "cancelled"}
        if action == "status": return dict(self._status)
        if action in ("move_joints", "hold"):
            if self._thread and self._thread.is_alive():
                raise RuntimeError("A Q5 body trajectory is already running")
            targets = args.get("targets", {}) if action == "move_joints" else {j["name"]: j["position"] for j in self.nodes.joint_snapshot()}
            def guarded():
                try:
                    self._run(targets, float(args.get("duration", 3.0)), float(args.get("rate_hz", 100.0)), float(args.get("kp", 100.0)), float(args.get("kd", 1.0)))
                    self._status = {"state": "cancelled" if self._cancel.is_set() else "completed", "targets": targets}
                except Exception as exc:
                    self._status = {"state": "failed", "error": str(exc)}
            self._thread = threading.Thread(target=guarded, daemon=True)
            self._thread.start()
            return {"state": "running", "targets": targets}
        return None


class Q5BasePlugin:
    def __init__(self, nodes, config):
        self.nodes = nodes
        self.max_linear = float(config["control"].get("max_linear_mps", 1.0))
        self.max_angular = float(config["control"].get("max_angular_rad_s", 2.0))
        self._cancel = threading.Event()

    def get_tool(self):
        return tool("base", "actuator", "Q5 wheel-base velocity control", action_schema(
            {"move": (["linear_x", "angular_z", "duration"], "Drive Q5 with linear/angular velocity"), "stop": ([], "Stop the Q5 wheel base")},
            {"linear_x": {"type": "number"}, "angular_z": {"type": "number"}, "duration": {"type": "number"}},
        ))

    def start(self): pass
    def stop(self): self._publish(0.0, 0.0)

    def _publish(self, linear, angular):
        from geometry_msgs.msg import TwistStamped
        msg = TwistStamped()
        msg.header.stamp = self.nodes.robot.get_clock().now().to_msg()
        msg.twist.linear.x = max(-self.max_linear, min(self.max_linear, linear))
        msg.twist.angular.z = max(-self.max_angular, min(self.max_angular, angular))
        self.nodes.base_pub.publish(msg)

    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": self._cancel.set(); self._publish(0.0, 0.0); return {"state": "stopped"}
        if action == "move":
            linear, angular = float(args.get("linear_x", 0.0)), float(args.get("angular_z", 0.0))
            duration = max(0.0, float(args.get("duration", 0.5)))
            self._cancel.clear()
            def run():
                end = time.monotonic() + duration
                while time.monotonic() < end and not self._cancel.is_set():
                    self._publish(linear, angular); time.sleep(0.05)
                self._publish(0.0, 0.0)
            threading.Thread(target=run, daemon=True).start()
            return {"state": "moving", "linear_x": linear, "angular_z": angular, "duration": duration}
        return None


class Q5HandPlugin:
    LITE = [
        "left_hand_thumb_bend_joint", "left_hand_thumb_rota_joint1", "left_hand_index_joint1", "left_hand_mid_joint1", "left_hand_ring_joint1", "left_hand_pinky_joint1",
        "right_hand_thumb_bend_joint", "right_hand_thumb_rota_joint1", "right_hand_index_joint1", "right_hand_mid_joint1", "right_hand_ring_joint1", "right_hand_pinky_joint1",
    ]

    def __init__(self, nodes): self.nodes = nodes
    def get_tool(self):
        return tool("hand", "actuator", "Q5 XHand/XHand Lite control", action_schema(
            {"open": (["side"], "Open XHand Lite fingers"), "grasp": (["side", "strength"], "Execute a gentle grasp"), "set_joints": (["joint_names", "positions", "kp", "kd"], "Control named hand joints")},
            {"side": {"type": "string", "enum": ["left", "right", "both"]}, "strength": {"type": "number"}, "joint_names": {"type": "array", "items": {"type": "string"}}, "positions": {"type": "array", "items": {"type": "number"}}, "kp": {"type": "number"}, "kd": {"type": "number"}},
        ))
    def start(self): pass
    def stop(self): pass

    def _send(self, names, positions, kp=100.0, kd=0.0):
        from xbot_common_interfaces.msg import HybridJointCommand
        if len(names) != len(positions): raise ValueError("joint_names and positions must have the same length")
        msg = HybridJointCommand(); msg.header.stamp = self.nodes.robot.get_clock().now().to_msg()
        msg.joint_name = list(names); msg.position = list(map(float, positions)); msg.velocity = [0.0] * len(names)
        msg.feedforward = [350.0] * len(names); msg.kp = [float(kp)] * len(names); msg.kd = [float(kd)] * len(names)
        self.nodes.hand_pub.publish(msg)
        return {"state": "published", "joints": names, "positions": positions}

    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action == "set_joints": return self._send(args.get("joint_names", []), args.get("positions", []), args.get("kp", 100.0), args.get("kd", 0.0))
        if action in ("open", "grasp"):
            side = args.get("side", "both")
            names = [name for name in self.LITE if side == "both" or name.startswith(side)]
            value = 0.0 if action == "open" else max(0.0, min(1.0, float(args.get("strength", 0.7))))
            return self._send(names, [value] * len(names))
        return None


class Q5MpcPlugin:
    def __init__(self, nodes): self.nodes = nodes
    def get_tool(self):
        pose = {"type": "array", "items": {"type": "number"}, "minItems": 7, "maxItems": 7, "description": "[x,y,z,qx,qy,qz,qw]"}
        return tool("mpc", "actuator", "Q5 dual-arm/head MPC servo control", action_schema(
            {"start_mpc": ([], "Start Q5 MPC"), "stop_mpc": ([], "Stop Q5 MPC"), "query": ([], "Query Q5 MPC"), "servo_pose": (["left_pose", "right_pose", "head_pose", "frame_id", "duration", "rate_hz"], "Publish dual-arm/head targets")},
            {"left_pose": pose, "right_pose": pose, "head_pose": pose, "frame_id": {"type": "string"}, "duration": {"type": "number"}, "rate_hz": {"type": "number"}},
        ))
    def start(self): pass
    def stop(self): pass

    def _teleop(self, command):
        from xbot_common_interfaces.srv import StringMessage
        req = StringMessage.Request(); req.data = json.dumps({"type": "mpc", "message": json.dumps({"command": command})})
        result = self.nodes.call(self.nodes.teleop, req, 10.0)
        if command == "query":
            try: self.nodes.last_mpc_state = int(json.loads(result.message).get("status"))
            except Exception: self.nodes.last_mpc_state = 1 if result.result else 0
        return jsonable(result)

    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action in ("start_mpc", "stop_mpc", "query"): return self._teleop(action.replace("_mpc", ""))
        if action == "servo_pose":
            from geometry_msgs.msg import PoseStamped
            from xbot_common_interfaces.msg import ServoPose
            msg = ServoPose(); frame = str(args.get("frame_id", "base_link"))
            def fill(values):
                pose = PoseStamped(); pose.header.frame_id = frame
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, values[:3])
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, values[3:])
                return pose
            if args.get("left_pose"): msg.left_pose = fill(args["left_pose"])
            if args.get("right_pose"): msg.right_pose = fill(args["right_pose"])
            if args.get("head_pose"): msg.head_pose = fill(args["head_pose"])
            duration, rate = float(args.get("duration", 2.0)), float(args.get("rate_hz", 50.0))
            def run():
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    stamp = self.nodes.robot.get_clock().now().to_msg()
                    for pose in (msg.left_pose, msg.right_pose, msg.head_pose): pose.header.stamp = stamp
                    self.nodes.servo_pub.publish(msg); time.sleep(1.0 / max(1.0, rate))
            threading.Thread(target=run, daemon=True).start()
            return {"state": "publishing", "duration": duration, "rate_hz": rate}
        return None


class Q5NavigationPlugin:
    def __init__(self, nodes):
        from era_nav_msgs.action import Navigate
        from era_nav_msgs.srv import CreateMap, InitPose, LoadMap, NavMapOp, QueryMap
        from rclpy.action import ActionClient
        from std_srvs.srv import Trigger
        self.nodes = nodes
        robot = nodes.robot
        self.start_map = robot.create_client(Trigger, "/slam/start_map")
        self.cancel_map = robot.create_client(Trigger, "/slam/cancel_map")
        self.create_map = robot.create_client(CreateMap, "/slam/create_map")
        self.load_map = robot.create_client(LoadMap, "/slam/load_map")
        self.init_pose = robot.create_client(InitPose, "/slam/init_pos")
        self.query_map = robot.create_client(QueryMap, "/slam/query_map")
        self.nav_map = robot.create_client(NavMapOp, "/era_nav/nav_map_op")
        self.nav = ActionClient(robot, Navigate, "/era_nav/nav_act")
        self.goal = None; self.counter = 1; self.feedback = None

    def get_tools(self):
        return [
            tool("mapping", "actuator", "Q5 LiDAR and topological map management", action_schema(
                {"start_lidar": ([], "Start LiDAR mapping"), "cancel_lidar": ([], "Cancel LiDAR mapping"), "create_lidar": (["map_name", "resolution", "data_path"], "Optimize and save LiDAR map"), "load_lidar": (["map_name"], "Load LiDAR map"), "query_lidar": ([], "Query loaded LiDAR map"), "init_pose": (["x", "y", "z", "yaw"], "Initialize LiDAR localization"), "nav_map": (["operation", "map_name", "force_reload"], "Load/save/read/clear navigation map")},
                {"map_name": {"type": "string"}, "resolution": {"type": "number"}, "data_path": {"type": "string"}, "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}, "yaw": {"type": "number"}, "operation": {"type": "string", "enum": ["LoadMap", "SaveMap", "ReadMap", "ClearMap"]}, "force_reload": {"type": "boolean"}},
            )),
            tool("navigation", "actuator", "Q5 station/pose navigation", action_schema(
                {"to_node": (["node_id", "name", "max_speed"], "Navigate to a topological station"), "to_pose": (["x", "y", "z", "yaw", "frame", "max_speed"], "Navigate to a global pose"), "cancel": ([], "Cancel Q5 navigation"), "status": ([], "Get Q5 navigation status")},
                {"node_id": {"type": "integer"}, "name": {"type": "string"}, "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}, "yaw": {"type": "number"}, "frame": {"type": "string"}, "max_speed": {"type": "number"}},
            )),
        ]
    def start(self): pass
    def stop(self): pass

    def _pose(self, x, y, z, yaw):
        from geometry_msgs.msg import Pose
        pose = Pose(); pose.position.x = float(x); pose.position.y = float(y); pose.position.z = float(z)
        pose.orientation.z = math.sin(float(yaw) / 2.0); pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def dispatch(self, action, args):
        from era_nav_msgs.action import Navigate
        from era_nav_msgs.srv import CreateMap, InitPose, LoadMap, NavMapOp, QueryMap
        from std_srvs.srv import Trigger
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        target = args.get("_tool_name")
        if target == "mapping":
            if action in ("start_lidar", "cancel_lidar"):
                return jsonable(self.nodes.call(self.start_map if action == "start_lidar" else self.cancel_map, Trigger.Request()))
            if action == "create_lidar":
                req = CreateMap.Request(); req.map_name = str(args["map_name"]); req.resolution = float(args.get("resolution", 0.05)); req.data_abs_path = str(args.get("data_path", ""))
                return jsonable(self.nodes.call(self.create_map, req, 120.0))
            if action == "load_lidar":
                req = LoadMap.Request(); req.map_name = str(args["map_name"]); return jsonable(self.nodes.call(self.load_map, req, 30.0))
            if action == "query_lidar": return jsonable(self.nodes.call(self.query_map, QueryMap.Request()))
            if action == "init_pose":
                req = InitPose.Request(); req.poses = [self._pose(args.get("x", 0), args.get("y", 0), args.get("z", 0), args.get("yaw", 0))]
                return jsonable(self.nodes.call(self.init_pose, req, 30.0))
            if action == "nav_map":
                req = NavMapOp.Request(); req.request_id = self.counter; self.counter += 1; req.op_type = str(args["operation"]); req.map_name = str(args.get("map_name", "")); req.force_reload = bool(args.get("force_reload", False))
                return jsonable(self.nodes.call(self.nav_map, req, 60.0))
        if target == "navigation":
            if action == "status": return {"state": "active" if self.goal else "idle", "feedback": jsonable(self.feedback)}
            if not self.nav.wait_for_server(timeout_sec=2.0): raise RuntimeError("ROS2 action unavailable: /era_nav/nav_act")
            goal = Navigate.Goal(); goal.request_id = -1 if action == "cancel" else self.counter; self.counter += 1
            goal.max_speed = float(args.get("max_speed", -1.0)); goal.max_detour_radius = -1.0; goal.parking_position_tolerance = -1.0; goal.parking_angle_tolerance = -1.0; goal.max_acceleration = -1.0; goal.max_angular_speed = -1.0; goal.max_angular_acceleration = -1.0; goal.max_centrifugal = -1.0
            if action == "to_node":
                goal.nav_type = "NavToGlobalNode"; goal.goal_node_id = int(args.get("node_id", -1)); goal.goal_attr = "name" if goal.goal_node_id < 0 else ""; goal.goal_attr_value = str(args.get("name", ""))
            elif action == "to_pose":
                goal.nav_type = "NavToGlobalPose"; goal.goal_ready = True; goal.goal = self._pose(args.get("x", 0), args.get("y", 0), args.get("z", 0), args.get("yaw", 0)); goal.frame = str(args.get("frame", "nav_map"))
            elif action == "cancel": goal.nav_type = "CancelNav"
            else: return None
            future = self.nav.send_goal_async(goal, feedback_callback=lambda value: setattr(self, "feedback", value.feedback))
            def accepted(done): self.goal = done.result() if done.exception() is None and done.result().accepted else None
            future.add_done_callback(accepted)
            return {"state": "submitted", "request_id": goal.request_id, "nav_type": goal.nav_type}
        return None


class Q5ChoreographyPlugin:
    """Arbitrary Q5 named-joint timelines plus vendor SimpleActions passthrough."""
    def __init__(self, nodes, body):
        self.nodes = nodes; self.body = body; self.cancel = threading.Event(); self.thread = None; self.status = {"state": "idle"}
    def get_tool(self):
        return tool("choreography", "actuator", "Q5 custom full-body choreography and vendor action playback", action_schema(
            {
                "custom": (["steps", "repeat"], "Play an arbitrary named-joint choreography"),
                "vendor_action": (["name", "duration"], "Play any SimpleActions program installed in Q5 firmware"),
                "stop": ([], "Stop custom choreography"), "status": ([], "Get choreography status"),
            },
            {"steps": {"type": "array", "items": {"type": "object"}}, "repeat": {"type": "integer"}, "name": {"type": "string"}, "duration": {"type": "number"}},
        ))
    def start(self): pass
    def stop(self): self.cancel.set(); self.body._cancel.set()
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": self.stop(); return {"state": "stopping"}
        if action == "status": return dict(self.status)
        if action == "vendor_action": return self.nodes.simple_pose(str(args["name"]), float(args.get("duration", 4.0)))
        if action == "custom":
            if self.thread and self.thread.is_alive(): raise RuntimeError("Q5 choreography is busy")
            steps = list(args.get("steps", [])); repeat = max(1, int(args.get("repeat", 1)))
            if not steps: raise ValueError("steps cannot be empty")
            self.cancel.clear(); self.body._cancel.clear()
            def run():
                completed = 0
                try:
                    for _ in range(repeat):
                        for step in steps:
                            if self.cancel.is_set(): self.status = {"state": "cancelled", "completed_steps": completed}; return
                            self.body.run_blocking(dict(step.get("targets", {})), float(step.get("duration", 1.0)), float(step.get("rate_hz", 100.0)), float(step.get("kp", 100.0)), float(step.get("kd", 1.0)))
                            completed += 1; self.status = {"state": "running", "completed_steps": completed, "total_steps": len(steps) * repeat}
                    self.status = {"state": "completed", "completed_steps": completed}
                except Exception as exc: self.status = {"state": "failed", "error": str(exc)}
            self.thread = threading.Thread(target=run, daemon=True, name="q5-choreography"); self.thread.start()
            return {"state": "running", "steps": len(steps), "repeat": repeat}
        return None


def build_plugins(config: dict, namespace: str, ros2):
    nodes = Q5NodeSet(config, namespace, ros2)
    plugins = config.get("plugins", {})
    result: list[Any] = []
    body = Q5BodyPlugin(nodes)
    if plugins.get("state", {}).get("enabled", True): result.append(Q5StatePlugin(nodes, namespace))
    if plugins.get("lifecycle", {}).get("enabled", True): result.append(Q5LifecyclePlugin(nodes))
    if plugins.get("body", {}).get("enabled", True): result.append(body)
    if plugins.get("base", {}).get("enabled", True): result.append(Q5BasePlugin(nodes, config))
    if plugins.get("hand", {}).get("enabled", True): result.append(Q5HandPlugin(nodes))
    if plugins.get("mpc", {}).get("enabled", True): result.append(Q5MpcPlugin(nodes))
    if plugins.get("navigation", {}).get("enabled", True): result.append(Q5NavigationPlugin(nodes))
    if plugins.get("choreography", {}).get("enabled", True): result.append(Q5ChoreographyPlugin(nodes, body))
    return result
