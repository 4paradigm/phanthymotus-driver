#!/usr/bin/env python3
"""山猫 M20 ROS 2/Fast DDS 与 basic_server 原生协议适配层。"""

from __future__ import annotations

import threading
import time

from basic_server import BasicServerClient
from common.vendor_runtime import action_schema, jsonable, tool


GAITS = {"basic": 0x1001, "standard_stairs": 0x1003, "agile_flat": 0x3002, "agile_stairs": 0x3003}
MOTION_STATES = {"idle": 0, "stand": 1, "soft_estop": 2, "damping": 3, "lie": 4, "rl_control": 17}


class M20Nodes:
    def __init__(self, config, namespace, ros2):
        from drdds.msg import Gait, JointsData, MotionInfo, MotionState, NavCmd, NavSat, StdMsgInt32, StdStatus
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2
        from rclpy.node import Node

        self.config = config
        self.variant = str(config.get("model_variant", "standard")).lower()
        self.is_pro = self.variant == "pro"
        self.native = BasicServerClient(config)
        self.robot = Node("lynx_m20_driver_robot", context=ros2.ctx_robot)
        self.core = Node("lynx_m20_driver_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self.robot)
        ros2.executor_core.add_node(self.core)
        topics = config.get("topics", {})
        self.motion_state_pub = self.robot.create_publisher(MotionState, topics.get("motion_state", "/MOTION_STATE"), 10)
        self.gait_pub = self.robot.create_publisher(Gait, topics.get("gait", "/GAIT"), 10)
        self.nav_cmd_pub = self.robot.create_publisher(NavCmd, topics.get("nav_cmd", "/NAV_CMD"), 10)
        self.charge_pub = self.robot.create_publisher(StdMsgInt32, topics.get("charge", "/CHARGE"), 10)
        self.lock = threading.Lock()
        self.values = {}
        self.streams = {}
        self.frame_id = 0
        self.velocity = (0.0, 0.0, 0.0)
        self.velocity_deadline = 0.0
        self.velocity_was_active = False
        self.native_velocity_command = 25
        self.native_velocity = {"X": 0.0, "Y": 0.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
        self.native_velocity_deadline = 0.0
        self.native_velocity_was_active = False
        self.rtsp_streams = {
            "camera_front": {"url": f"rtsp://{self.native.host}:8554/video1", "format": "video/h265"},
            "camera_rear": {"url": f"rtsp://{self.native.host}:8554/video2", "format": "video/h265"},
        }
        self.robot.create_timer(0.1, self._publish_active_velocity)
        self.robot.create_timer(0.05, self._send_active_native_velocity)

        streams = [
            ("motion_info", MotionInfo, "/MOTION_INFO", "data/json", 20),
            ("imu", Imu, "/IMU", "data/imu", 50),
            ("lidar", PointCloud2, "/LIDAR/POINTS", "pointcloud/ros2", 10),
            ("lidar_rear", PointCloud2, "/LIDAR/POINTS2", "pointcloud/ros2", 10),
            ("hard_estop", StdMsgInt32, "/HES_STATUS", "data/json", 10),
            ("charge_status", StdStatus, "/CHARGE_STATUS", "data/json", 10),
            ("gps", NavSat, "/GPS", "data/json", 10),
            ("joints", JointsData, "/JOINTS_DATA", "data/json", 20),
        ]
        if self.is_pro:
            streams.append(("odometry", Odometry, "/ODOM", "data/odometry", 10))
        for key, msg_type, default, fmt, depth in streams:
            robot_topic = topics.get(key, default)
            core_topic = f"/{namespace}/lynx_m20/{key}"
            pub = self.core.create_publisher(msg_type, core_topic, depth)
            self.robot.create_subscription(msg_type, robot_topic, self._callback(key, pub), depth)
            self.streams[key] = {"robot_topic": robot_topic, "topic": core_topic, "format": fmt}

    def _callback(self, key, publisher):
        def callback(msg):
            publisher.publish(msg)
            if key.startswith("lidar"):
                header = getattr(msg, "header", None)
                value = {"received": True, "frame_id": getattr(header, "frame_id", ""), "timestamp": time.time()}
            else:
                value = jsonable(msg)
            with self.lock:
                self.values[key] = value
        return callback

    def _header(self, msg):
        self.frame_id += 1
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.robot.get_clock().now().to_msg()

    def publish_motion_state(self, state: int) -> dict:
        from drdds.msg import MotionState
        msg = MotionState(); self._header(msg); msg.data.state = int(state); self.motion_state_pub.publish(msg)
        return {"state": "published", "topic": self.config["topics"]["motion_state"], "motion_state": int(state)}

    def publish_gait(self, gait: int) -> dict:
        from drdds.msg import Gait
        msg = Gait(); self._header(msg); msg.data.gait = int(gait); self.gait_pub.publish(msg)
        return {"state": "published", "topic": self.config["topics"]["gait"], "gait": int(gait)}

    def set_ros_velocity(self, x: float, y: float, yaw: float) -> dict:
        self.native_velocity_deadline = 0.0
        self.velocity = (float(x), float(y), float(yaw))
        self.velocity_deadline = time.monotonic() + float(self.config.get("velocity_command_timeout", 0.5))
        self._publish_active_velocity()
        return {"state": "streaming", "topic": self.config["topics"]["nav_cmd"], "frequency_hz": 10, "timeout_seconds": self.config.get("velocity_command_timeout", 0.5)}

    def set_native_velocity(self, command: int, values: dict) -> dict:
        self.velocity_deadline = 0.0
        self.native_velocity_command = int(command)
        self.native_velocity = dict(values)
        self.native_velocity_deadline = time.monotonic() + float(self.config.get("velocity_command_timeout", 0.5))
        result = self.native.send_velocity(self.native_velocity_command, self.native_velocity)
        self.native_velocity_was_active = True
        return {**result, "state": "streaming", "frequency_hz": 20, "timeout_seconds": self.config.get("velocity_command_timeout", 0.5)}

    def _send_active_native_velocity(self):
        active = time.monotonic() < self.native_velocity_deadline
        try:
            if active:
                self.native.send_velocity(self.native_velocity_command, self.native_velocity)
                self.native_velocity_was_active = True
            elif self.native_velocity_was_active:
                zero = {key: 0.0 for key in ("X", "Y", "Z", "Roll", "Pitch", "Yaw")}
                self.native.send_velocity(self.native_velocity_command, zero)
                self.native_velocity_was_active = False
        except OSError as exc:
            self.native.last_error = str(exc)

    def _publish_nav_cmd(self, velocity):
        from drdds.msg import NavCmd
        msg = NavCmd(); self._header(msg)
        msg.data.x_vel, msg.data.y_vel, msg.data.yaw_vel = velocity
        self.nav_cmd_pub.publish(msg)

    def _publish_active_velocity(self):
        active = time.monotonic() < self.velocity_deadline
        if active:
            self._publish_nav_cmd(self.velocity); self.velocity_was_active = True
        elif self.velocity_was_active:
            self._publish_nav_cmd((0.0, 0.0, 0.0)); self.velocity_was_active = False

    def stop_velocity(self):
        self.velocity_deadline = 0.0; self.velocity_was_active = False; self._publish_nav_cmd((0.0, 0.0, 0.0))
        self.native_velocity_deadline = 0.0
        if self.native_velocity_was_active:
            zero = {key: 0.0 for key in ("X", "Y", "Z", "Roll", "Pitch", "Yaw")}
            self.native.send_velocity(self.native_velocity_command, zero); self.native_velocity_was_active = False

    def motion_summary(self):
        with self.lock:
            info = self.values.get("motion_info", {})
        data = info.get("data", {}) if isinstance(info, dict) else {}
        return {
            "state": data.get("motion_state", {}).get("state"),
            "gait": data.get("gait_state", {}).get("gait"),
            "vel_x": data.get("vel_x"), "vel_y": data.get("vel_y"), "vel_yaw": data.get("vel_yaw"),
        }

    def snapshot(self):
        with self.lock: values = dict(self.values)
        values["basic_server"] = self.native.snapshot()
        return values

    def close(self):
        self.native.close(); self.robot.destroy_node(); self.core.destroy_node()


class M20StatePlugin:
    def __init__(self, nodes): self.nodes = nodes
    def get_tools(self):
        result = [tool("state", "sensor", "山猫 M20 ROS 2 与 basic_server 聚合状态")]
        result.extend(tool(key, "sensor", f"山猫 M20 {key} 数据流", topic_out=[{"topic": item["topic"], "format": item["format"]}]) for key, item in self.nodes.streams.items())
        result.extend(tool(key, "sensor", f"山猫 M20 {key} RTSP 视频流", topic_out=[{"topic": item["url"], "format": item["format"]}]) for key, item in self.nodes.rtsp_streams.items())
        result.extend([
            tool("native_motion_status", "sensor", "查询运控与 16 关节状态（Type=1002 Cmd=4）"),
            tool("native_device_status", "sensor", "查询双电池、温度、灯光和相机状态（Type=1002 Cmd=5）"),
            tool("native_basic_status", "sensor", "查询运动、步态、模式、充电和休眠状态（Type=1002 Cmd=6）"),
            tool("errors", "sensor", "查询设备异常列表（Type=1002 Cmd=3）"),
            tool("capabilities", "sensor", "查询按 M20/M20 Pro 区分的能力"), tool("ros_graph", "sensor", "查看 M20 实时 ROS 2 图"),
        ])
        return result
    def start(self): self.nodes.native.start()
    def stop(self): self.nodes.close()
    def dispatch(self, action, args):
        name = args.get("_tool_name")
        if action in ("info", "start", "stop"): return {"state": "ready" if action != "stop" else "idle"}
        if name == "state": return self.nodes.snapshot()
        if name in self.nodes.streams: return {"state": "running", **self.nodes.streams[name]}
        if name in self.nodes.rtsp_streams: return {"state": "ready", **self.nodes.rtsp_streams[name]}
        native_queries = {"errors": (1002, 3), "native_motion_status": (1002, 4), "native_device_status": (1002, 5), "native_basic_status": (1002, 6)}
        if name in native_queries: return self.nodes.native.request(*native_queries[name])
        if name == "ros_graph": return {"topics": self.nodes.robot.get_topic_names_and_types(), "nodes": self.nodes.robot.get_node_names_and_namespaces()}
        if name == "capabilities": return {
            "model": "DEEPRobotics Lynx M20 Pro" if self.nodes.is_pro else "DEEPRobotics Lynx M20",
            "firmware_documented": "V1.1.8", "ros_middleware": "Fast DDS", "native_protocol": "basic_server TCP/UDP",
            "standard": ["stand", "lie", "soft_estop", "four_gaits", "normalized_axis", "navigation_velocity", "usage_mode", "sleep", "front_rear_lights", "motion_state", "sixteen_joint_state", "dual_battery_state", "temperature", "errors", "front_rear_rtsp", "lidar", "imu", "hard_estop", "optional_gps", "optional_autonomous_charging"],
            "pro_only_enabled": ["odometry", "localization", "navigation"] if self.nodes.is_pro else [],
            "unsupported_by_supplied_document": ["dance", "custom_stunt", "joint_position_control"],
            "hardware_validated": False,
        }


class M20MotionPlugin:
    ACTIONS = {
        "stand": ([], "起立；原生协议自动推进状态机，ROS 2 按 1→17 推进"), "lie": ([], "静止后趴下"),
        "soft_estop": ([], "触发最高优先级软急停"), "gait": (["gait"], "切换四种官方步态"),
        "axis": (["x", "y", "yaw"], "原生 Cmd=21 归一化轴控制，需由调用方持续刷新"),
        "velocity": (["x", "y", "yaw"], "导航模式速度控制"), "stop": ([], "发送零速度"),
    }
    def __init__(self, nodes): self.nodes = nodes
    def get_tool(self):
        return tool("motion", "actuator", "山猫 M20 官方运动状态、步态和速度控制", action_schema(self.ACTIONS, {
            "transport": {"type": "string", "enum": ["native", "ros2"], "default": "native"},
            "gait": {"type": "string", "enum": list(GAITS)},
            "x": {"type": "number"}, "y": {"type": "number"}, "yaw": {"type": "number"},
        }))
    def start(self): pass
    def stop(self): self.nodes.stop_velocity()
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        transport = args.get("transport", "native")
        if action in ("stand", "lie", "soft_estop"):
            target = {"stand": 1, "lie": 4, "soft_estop": 2}[action]
            if transport == "native": return self.nodes.native.request(2, 22, {"MotionParam": target})
            result = self.nodes.publish_motion_state(target)
            if action == "stand":
                def enter_rl():
                    deadline = time.monotonic() + 10
                    while time.monotonic() < deadline:
                        if self.nodes.motion_summary()["state"] == 1:
                            self.nodes.publish_motion_state(17); return
                        time.sleep(0.1)
                threading.Thread(target=enter_rl, daemon=True).start()
                result["transition"] = [1, 17]
            return result
        if action == "gait":
            gait = GAITS[args["gait"]]
            return self.nodes.native.request(2, 23, {"GaitParam": gait}) if transport == "native" else self.nodes.publish_gait(gait)
        values = {"X": float(args.get("x", 0)), "Y": float(args.get("y", 0)), "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": float(args.get("yaw", 0))}
        if action == "axis":
            if transport != "native": raise ValueError("axis 仅对应 basic_server Cmd=21")
            if any(abs(values[key]) > 1 for key in ("X", "Y", "Yaw")): raise ValueError("axis 的 x/y/yaw 必须在 [-1, 1]")
            return self.nodes.set_native_velocity(21, values)
        if action in ("velocity", "stop"):
            if action == "stop": values.update({"X": 0.0, "Y": 0.0, "Yaw": 0.0})
            if transport == "native": return self.nodes.set_native_velocity(25, values)
            return self.nodes.set_ros_velocity(values["X"], values["Y"], values["Yaw"]) if action == "velocity" else (self.nodes.stop_velocity() or {"state": "stopped"})


class M20ChargePlugin:
    def __init__(self, nodes): self.nodes = nodes
    def get_tool(self):
        return tool("charging", "actuator", "山猫 M20 选配自主充电（机器人需已位于充电桩前）", action_schema(
            {"start": ([], "进桩充电"), "stop": ([], "退出充电"), "reset": ([], "异常时强制复位")},
            {"transport": {"type": "string", "enum": ["native", "ros2"], "default": "native"}},
        ))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        value = {"stop": 0, "start": 1, "reset": 2}[action]
        if args.get("transport", "native") == "native": return self.nodes.native.request(2, 24, {"Charge": value})
        from drdds.msg import StdMsgInt32
        msg = StdMsgInt32(); msg.value = value; self.nodes.charge_pub.publish(msg)
        return {"state": "published", "topic": self.nodes.config["topics"]["charge"], "value": value}


class M20DevicePlugin:
    def __init__(self, nodes): self.nodes = nodes
    def get_tool(self):
        return tool("device", "actuator", "山猫 M20 灯光、使用模式和休眠管理", action_schema(
            {
                "lights": (["front", "rear"], "独立设置前后照明灯"),
                "mode": (["mode"], "切换常规、导航或辅助模式"),
                "sleep": ([], "进入休眠；仅趴下且处于常规/辅助模式时允许"),
                "wake": ([], "退出休眠"),
                "auto_sleep": (["enabled"], "配置 5 到 30 分钟自动休眠"),
                "sleep_status": ([], "查询休眠状态"),
            },
            {
                "front": {"type": "boolean"}, "rear": {"type": "boolean"},
                "mode": {"type": "string", "enum": ["normal", "navigation", "assist"]},
                "enabled": {"type": "boolean"}, "minutes": {"type": "integer", "minimum": 5, "maximum": 30},
            },
        ))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "lights": return self.nodes.native.request(1101, 2, {"Front": int(args["front"]), "Back": int(args["rear"])})
        if action == "mode": return self.nodes.native.request(1101, 5, {"Mode": {"normal": 0, "navigation": 1, "assist": 2}[args["mode"]]})
        if action == "sleep": return self.nodes.native.request(1101, 6, {"Sleep": True})
        if action == "wake": return self.nodes.native.request(1101, 6, {"Sleep": False})
        if action == "sleep_status": return self.nodes.native.request(1101, 7)
        items = {"Auto": bool(args["enabled"])}
        if items["Auto"]: items.update({"Sleep": False, "Time": int(args.get("minutes", 5))})
        return self.nodes.native.request(1101, 6, items)


class M20ProNavigationPlugin:
    def __init__(self, nodes): self.nodes = nodes
    def get_tool(self):
        return tool("navigation", "actuator", "仅 M20 Pro：原生单点导航、取消、状态与定位初始化", action_schema(
            {"goal": (["x", "y", "yaw"], "下发单点导航"), "cancel": ([], "取消任务"), "status": ([], "查询任务状态"), "localize": (["x", "y", "yaw"], "初始化定位")},
            {"x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}, "yaw": {"type": "number"}, "point_type": {"type": "integer"}, "gait": {"type": "string", "enum": ["agile_flat", "agile_stairs"]}, "speed": {"type": "integer", "minimum": 0, "maximum": 2}, "reverse": {"type": "boolean"}, "avoid_obstacles": {"type": "boolean"}, "autonomous": {"type": "boolean"}},
        ))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "cancel": return self.nodes.native.request(1004, 1)
        if action == "status": return self.nodes.native.request(1007, 1)
        if action == "localize": return self.nodes.native.request(2101, 1, {"PosX": args["x"], "PosY": args["y"], "PosZ": args.get("z", 0), "Yaw": args["yaw"]})
        items = {"Value": 0, "MapID": 0, "PosX": args["x"], "PosY": args["y"], "PosZ": args.get("z", 0), "AngleYaw": args["yaw"], "PointInfo": args.get("point_type", 1), "Gait": GAITS[args.get("gait", "agile_flat")], "Speed": args.get("speed", 0), "Manner": int(bool(args.get("reverse", False))), "ObsMode": 0 if args.get("avoid_obstacles", True) else 1, "NavMode": int(bool(args.get("autonomous", True)))}
        return self.nodes.native.request(1003, 1, items)


def build_plugins(config, namespace, ros2):
    nodes = M20Nodes(config, namespace, ros2)
    plugins = [M20StatePlugin(nodes), M20MotionPlugin(nodes), M20ChargePlugin(nodes), M20DevicePlugin(nodes)]
    if nodes.is_pro: plugins.append(M20ProNavigationPlugin(nodes))
    return plugins
