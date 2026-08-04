#!/usr/bin/env python3
"""S10Pro standard ROS2 controls plus a correlated supplier command bridge."""

from __future__ import annotations

import threading
import time

from common.ros2_json_bridge import JsonCommandBridge
from common.vendor_runtime import action_schema, jsonable, tool


class S10Nodes:
    def __init__(self, config, namespace, ros2):
        from geometry_msgs.msg import Twist, PoseStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import BatteryState, Image, Imu, JointState, PointCloud2

        self.bridge = JsonCommandBridge(config, namespace, ros2, "s10_pro")
        self.robot = self.bridge.robot; self.core = self.bridge.core
        topics = config.get("topics", {})
        self.cmd_vel_pub = self.robot.create_publisher(Twist, topics.get("cmd_vel", "/cmd_vel"), 10)
        self.goal_pub = self.robot.create_publisher(PoseStamped, topics.get("goal_pose", "/goal_pose"), 10)
        self.lock = threading.Lock(); self.values = {}; self.streams = {}

        for key, msg_type, default, fmt in (
            ("joint_states", JointState, "/joint_states", "data/ros2"),
            ("imu", Imu, "/imu/data", "data/imu"),
            ("odometry", Odometry, "/odom", "data/odometry"),
            ("battery", BatteryState, "/battery_state", "data/battery"),
            ("camera", Image, "/camera/color/image_raw", "video/raw"),
            ("depth", Image, "/camera/depth/image_raw", "video/depth"),
            ("lidar", PointCloud2, "/lidar/points", "pointcloud/ros2"),
        ):
            robot_topic = topics.get(key, default)
            core_topic = f"/{namespace}/s10_pro/{key}"
            publisher = self.core.create_publisher(msg_type, core_topic, 2 if key in ("camera", "depth", "lidar") else 10)
            self.robot.create_subscription(msg_type, robot_topic, self._callback(key, publisher), 2 if key in ("camera", "depth", "lidar") else 10)
            self.streams[key] = {"robot_topic": robot_topic, "topic": core_topic, "format": fmt}

    def _callback(self, key, publisher):
        def callback(msg):
            publisher.publish(msg)
            if key not in ("camera", "depth", "lidar"):
                with self.lock: self.values[key] = jsonable(msg)
            else:
                header = getattr(msg, "header", None)
                with self.lock:
                    self.values[key] = {"received": True, "frame_id": getattr(header, "frame_id", ""), "timestamp": time.time()}
        return callback

    def snapshot(self):
        with self.lock: values = dict(self.values)
        values["supplier_bridge"] = self.bridge.snapshot()
        return values


class S10StatePlugin:
    def __init__(self, nodes): self.nodes = nodes
    def get_tools(self):
        return [
            tool("state", "sensor", "S10Pro joints, IMU, odometry, battery and supplier state", topic_out=[{"topic": self.nodes.bridge.state_topic, "format": "data/json"}]),
            *[tool(key, "sensor", f"S10Pro {key} ROS2 stream", topic_out=[{"topic": item["topic"], "format": item["format"]}]) for key, item in self.nodes.streams.items()],
            tool("command_ack", "sensor", "Query S10Pro supplier acknowledgement", {"type": "object", "properties": {"id": {"type": "string"}}, "required": ["id"]}),
            tool("capabilities", "sensor", "S10Pro capability and binding discovery"),
            tool("ros_graph", "sensor", "Live S10Pro ROS2 graph"),
        ]
    def start(self): pass
    def stop(self): self.nodes.bridge.close()
    def dispatch(self, action, args):
        name = args.get("_tool_name")
        if action == "info":
            if name == "state": return {"state": "ready", "topic_out": [{"topic": self.nodes.bridge.state_topic, "format": "data/json"}]}
            if name in self.nodes.streams:
                item = self.nodes.streams[name]; return {"state": "ready", "topic_out": [{"topic": item["topic"], "format": item["format"]}]}
            return {"state": "ready"}
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if name == "state": return self.nodes.snapshot()
        if name in self.nodes.streams: return {"state": "running", **self.nodes.streams[name]}
        if name == "command_ack": return self.nodes.bridge.acknowledgement(args["id"])
        if name == "ros_graph": return self.nodes.bridge.graph()
        if name == "capabilities": return {
            "model": "Deep Robotics S10Pro", "standard_ros2_connected": True, "supplier_idl_connected": False,
            "capabilities": ["velocity", "stop", "goal_pose", "gait", "height", "posture", "stand", "sit", "lie", "self_recovery", "stairs", "slope", "dance", "custom_action", "mapping", "navigation", "patrol", "follow", "dock", "camera", "depth", "lidar", "imu", "odometry", "joints", "battery"],
        }
        return None


class S10BasePlugin:
    def __init__(self, nodes): self.nodes = nodes
    def get_tool(self):
        return tool("base", "actuator", "S10Pro standard ROS2 velocity control", action_schema(
            {"velocity": (["linear_x", "linear_y", "angular_z"], "Publish geometry_msgs/Twist"), "stop": ([], "Publish zero velocity")},
            {"linear_x": {"type": "number"}, "linear_y": {"type": "number"}, "angular_z": {"type": "number"}},
        ))
    def start(self): pass
    def stop(self): self._publish({})
    def _publish(self, args):
        from geometry_msgs.msg import Twist
        msg = Twist(); msg.linear.x = float(args.get("linear_x", 0.0)); msg.linear.y = float(args.get("linear_y", 0.0)); msg.angular.z = float(args.get("angular_z", 0.0)); self.nodes.cmd_vel_pub.publish(msg)
        return {"state": "published", "topic": self.nodes.config.get("topics", {}).get("cmd_vel", "/cmd_vel"), "linear_x": msg.linear.x, "linear_y": msg.linear.y, "angular_z": msg.angular.z}
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action in ("velocity", "stop"): return self._publish(args if action == "velocity" else {})
        return None


class S10NavigationPlugin:
    def __init__(self, nodes): self.nodes = nodes
    def get_tool(self):
        return tool("navigation", "actuator", "S10Pro standard goal pose and supplier autonomy commands", action_schema(
            {"goal": (["x", "y", "yaw", "frame"], "Publish geometry_msgs/PoseStamped goal"), "cancel": ([], "Cancel supplier navigation"),
             "map_start": (["name"], "Start mapping"), "map_save": (["name"], "Save map"), "map_load": (["name"], "Load map"),
             "patrol": (["waypoints", "repeat"], "Start waypoint patrol"), "follow": (["target"], "Start target following"), "dock": ([], "Return to dock")},
            {"x": {"type": "number"}, "y": {"type": "number"}, "yaw": {"type": "number"}, "frame": {"type": "string"}, "name": {"type": "string"},
             "waypoints": {"type": "array", "items": {"type": "object"}}, "repeat": {"type": "integer"}, "target": {"type": "string"}},
        ))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action == "goal":
            import math
            from geometry_msgs.msg import PoseStamped
            msg = PoseStamped(); msg.header.frame_id = str(args.get("frame", "map")); msg.header.stamp = self.nodes.robot.get_clock().now().to_msg()
            msg.pose.position.x = float(args["x"]); msg.pose.position.y = float(args["y"]); yaw = float(args.get("yaw", 0.0)); msg.pose.orientation.z = math.sin(yaw / 2.0); msg.pose.orientation.w = math.cos(yaw / 2.0)
            self.nodes.goal_pub.publish(msg); return {"state": "published", "topic": self.nodes.config.get("topics", {}).get("goal_pose", "/goal_pose")}
        return self.nodes.bridge.publish(f"navigation.{action}", {k: v for k, v in args.items() if not k.startswith("_")})


class S10MotionPlugin:
    ACTIONS = {
        "stand": ([], "Stand up"), "sit": ([], "Sit"), "lie": ([], "Lie down"), "recover": ([], "Self-recover after a fall"),
        "gait": (["name"], "Select gait"), "height": (["value"], "Adjust body height"), "attitude": (["roll", "pitch", "yaw"], "Adjust body attitude"),
        "action": (["name", "params"], "Play a supplier action or stunt"), "stop": ([], "Stop current action"),
    }
    def __init__(self, bridge): self.bridge = bridge
    def get_tool(self):
        return tool("motion", "actuator", "S10Pro gait, posture, balance and stunt command bridge", action_schema(self.ACTIONS, {
            "name": {"type": "string"}, "value": {"type": "number"}, "roll": {"type": "number"}, "pitch": {"type": "number"}, "yaw": {"type": "number"}, "params": {"type": "object"},
        }))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        return self.bridge.publish(f"motion.{action}", {k: v for k, v in args.items() if not k.startswith("_")})


class S10DancePlugin:
    def __init__(self, bridge): self.bridge = bridge
    def get_tool(self):
        return tool("choreography", "actuator", "S10Pro dances and custom motion sequences", action_schema(
            {"list": ([], "Request the firmware action catalog"), "play": (["name", "repeat"], "Play a firmware dance/action"), "custom": (["timeline", "repeat"], "Play a custom action timeline"), "stop": ([], "Stop choreography")},
            {"name": {"type": "string"}, "repeat": {"type": "integer"}, "timeline": {"type": "array", "items": {"type": "object"}}},
        ))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        return self.bridge.publish(f"choreography.{action}", {k: v for k, v in args.items() if not k.startswith("_")})


def build_plugins(config, namespace, ros2):
    nodes = S10Nodes(config, namespace, ros2); nodes.config = config
    return [S10StatePlugin(nodes), S10BasePlugin(nodes), S10NavigationPlugin(nodes), S10MotionPlugin(nodes.bridge), S10DancePlugin(nodes.bridge)]
