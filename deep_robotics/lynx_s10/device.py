#!/usr/bin/env python3
"""云深处山猫 S10 原生 basic_server 驱动。"""

from __future__ import annotations

import threading
import time
import math
import json
import os
import ssl
import struct
import urllib.request
from uuid import uuid4
from pathlib import Path

from basic_server import BasicServerClient
from common.vendor_runtime import action_schema, tool


def _mark_dangerous(result, actions):
    schema = result["inputSchema"]
    schema["x-is-dangerous"] = True
    schema["properties"]["confirm"] = {
        "type": "boolean", "description": "Required true to confirm this physical or safety-relevant action."}
    for action in actions:
        schema["x-action-params"][action]["params"].append("confirm")
    return result


def _confirmation_error(action, args):
    if args.get("confirm") is True:
        return None
    return {"ok": False, "code": "INVALID_ARGUMENT",
            "error": f"{action} requires boolean confirm=true"}


def _post_acp_callback(action_id, status, result, tool_name):
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    context = ssl.create_default_context(cafile=os.environ.get("AGENT_CORE_CA_CERT") or None)
    if not os.environ.get("AGENT_CORE_CA_CERT") and url.startswith(("https://localhost:", "https://127.0.0.1:")):
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                          "tool": tool_name, "ts": time.time()}).encode()
    request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                     headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(request, timeout=5, context=context) as response:
        acknowledgement = json.load(response)
    if acknowledgement.get("ok") is not True or acknowledgement.get("action_id") != action_id:
        raise RuntimeError("Agent Core did not acknowledge action_id")


class S10Nodes:
    """Forward robot-domain ROS samples and native TCP state into Agent Core DDS."""
    def __init__(self, config, namespace, ros2):
        import json
        import time
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from drdds.msg import BatteryData, JointsData, MotionInfo
        from sensor_msgs.msg import Imu, PointCloud2
        from std_msgs.msg import String

        self.json = json
        self.robot = Node("lynx_s10_driver_robot", context=ros2.ctx_robot)
        self.core = Node("lynx_s10_driver_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self.robot)
        ros2.executor_core.add_node(self.core)
        prefix = f"/{namespace}/" if namespace else "/"
        base = prefix + "lynx_s10/"
        self.topics = {
            "imu": {"topic": base + "imu", "format": "data/json"},
            "joints": {"topic": base + "joints", "format": "sensor/skeleton"},
            "joint_state": {"topic": base + "joint_state", "format": "data/json"},
            "loco_state": {"topic": base + "loco_state", "format": "data/json"},
            "battery": {"topic": base + "battery", "format": "data/json"},
            "lidar": {"topic": base + "lidar", "format": "sensor/pointcloud"},
        }
        self.publishers = {
            key: self.core.create_publisher(String, spec["topic"], 10)
            for key, spec in self.topics.items() if key != "lidar"
        }
        from std_msgs.msg import UInt8MultiArray
        self.lidar_publisher = self.core.create_publisher(UInt8MultiArray, self.topics["lidar"]["topic"], 2)
        topics = config.get("topics", {})
        self.robot.create_subscription(Imu, topics.get("imu", "/IMU"), self._forward_imu, qos_profile_sensor_data)
        self.robot.create_subscription(JointsData, topics.get("joints", "/JOINTS_DATA_10HZ"), self._forward_joints, qos_profile_sensor_data)
        self.robot.create_subscription(BatteryData, topics.get("battery", "/BATTERY_DATA"), self._forward_battery, qos_profile_sensor_data)
        self.robot.create_subscription(MotionInfo, topics.get("motion_info", "/MOTION_INFO"), self._forward_loco_state, qos_profile_sensor_data)
        self.robot.create_subscription(PointCloud2, topics.get("lidar", "/LIDAR/POINTS"), self._forward_lidar, qos_profile_sensor_data)

    def _publish_json(self, key, value):
        from std_msgs.msg import String
        message = String()
        message.data = self.json.dumps(value, ensure_ascii=False, separators=(",", ":"), default=str)
        self.publishers[key].publish(message)

    def _forward_imu(self, msg):
        from common.vendor_runtime import jsonable
        self._publish_json("imu", jsonable(msg))

    def _forward_joints(self, msg):
        from common.vendor_runtime import jsonable
        values = jsonable(msg)
        data = values.get("data", {})
        joints = values.get("joints_data", data.get("joints_data", []) if isinstance(data, dict) else data)
        motor_status = {"Joint": joints}
        self._publish_json("joints", _skeleton_payload({"MotorStatus": motor_status}))
        self._publish_json("joint_state", values)

    def _forward_battery(self, msg):
        from common.vendor_runtime import jsonable
        values = jsonable(msg)
        self._publish_json("battery", {"batteries": values.get("data", [])})

    def _forward_loco_state(self, msg):
        from common.vendor_runtime import jsonable
        values = jsonable(msg)
        self._publish_json("loco_state", values.get("data", values))

    def _forward_lidar(self, msg):
        packet = self._pointcloud_packet(msg)
        if packet is None:
            return
        from std_msgs.msg import UInt8MultiArray
        output = UInt8MultiArray()
        output.data = list(packet)
        self.lidar_publisher.publish(output)

    @staticmethod
    def _pointcloud_packet(msg):
        """Encode XYZ PointCloud2 using the renderer's binary point-cloud contract."""
        fields = {field.name: field.offset for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            return None
        endian = ">" if msg.is_bigendian else "<"
        points = []
        for row in range(int(msg.height)):
            for col in range(int(msg.width)):
                offset = row * int(msg.row_step) + col * int(msg.point_step)
                try:
                    point = tuple(struct.unpack_from(endian + "f", msg.data, offset + fields[name])[0]
                                  for name in ("x", "y", "z"))
                except struct.error:
                    continue
                if all(math.isfinite(value) for value in point):
                    points.append(point)
                if len(points) >= 20000:
                    break
            if len(points) >= 20000:
                break
        return struct.pack("<II", 12, len(points)) + b"".join(
            struct.pack("<fff", *point) for point in points)

    def close(self):
        self.robot.destroy_node()
        self.core.destroy_node()


LOCO_STATES = {"stand_up": 1, "damp": 2, "stand_down": 4, "rl_control": 17}
GAITS = {"basic_flat": 0x1001, "basic_stairs": 0x1003, "navigation_flat": 0x3002, "navigation_stairs": 0x3003}
MODES = {"normal": 0, "navigation": 1, "assist": 2}
JOINT_NAMES = (
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint",
)
URDF_PATH = Path(__file__).with_name("resource") / "S10.urdf"


def _joint_value(joint, *keys):
    if not isinstance(joint, dict):
        return None
    for key in keys:
        if key in joint:
            return joint[key]
    return None


def _skeleton_payload(items):
    motor_status = items.get("MotorStatus", {})
    raw_joints = motor_status.get("Joint", []) if isinstance(motor_status, dict) else []
    if isinstance(raw_joints, dict):
        raw_joints = raw_joints.get("Joint", raw_joints.get("Data", []))
    if not isinstance(raw_joints, list):
        raw_joints = []
    joints = []
    for idx, name in enumerate(JOINT_NAMES):
        raw = raw_joints[idx] if idx < len(raw_joints) else None
        joints.append({
            "idx": idx, "name": name,
            "q": _joint_value(raw, "q", "Q", "position", "Position", "Pos", "Angle"),
            "dq": _joint_value(raw, "dq", "DQ", "velocity", "Velocity", "Speed"),
            "tau": _joint_value(raw, "tau", "Tau", "torque", "Torque", "TorqueEst"),
            "temperature": _joint_value(raw, "temperature", "Temperature", "Temp", "motion_temp", "driver_temp"),
        })
    payload = {"format": "sensor/skeleton", "available": len(raw_joints) >= len(JOINT_NAMES), "joints": joints}
    motion = items.get("MotionStatus", {})
    try:
        roll, pitch, yaw = (float(motion[key]) for key in ("Roll", "Pitch", "Yaw"))
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        payload["imu_quat"] = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                               cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
    except (KeyError, TypeError, ValueError):
        pass
    return payload


class S10ModelPlugin:
    def get_tool(self):
        return tool("model", "resource", "云深处山猫 S10 官方 URDF 运动学模型，用于 joints 骨架渲染")
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "model":
            return {"urdf": URDF_PATH.read_text(encoding="utf-8")}
        return {"state": "ready"}


class S10StatePlugin:
    def __init__(self, native, nodes=None): self.native, self.nodes = native, nodes
    def get_tools(self):
        cards = {
            "state": "山猫 S10 原生 TCP 协议连接及全部设备主动上报状态",
            "imu": "山猫 S10 IMU 加速度和角速度（由原生运控状态报告提供，10Hz）",
            "joints": "山猫 S10 16 个轮足关节角度（原生协议 10Hz）",
            "joint_state": "山猫 S10 轮足关节位置、速度、力矩和温度（原生协议 10Hz）",
            "loco_state": "山猫 S10 运动状态、步态、模式和速度（原生协议 10Hz）",
            "battery": "山猫 S10 电池电量、电压、温度和充电状态",
            "errors": "山猫 S10 原生协议聚合故障列表及故障变化",
            "navigation_status": "山猫 S10 导航任务和定位状态（Pro 支持）",
            "lidar": "山猫 S10 激光雷达点云（实时 ROS 2 转发）",
        }
        topics = self.nodes.topics if self.nodes else {}
        return [tool(name, "sensor", description, topic_out=[topics[name]] if name in topics else None) for name, description in cards.items()]

    def _topic_result(self, name, state):
        result = {"state": state}
        if self.nodes and name in self.nodes.topics:
            result["topic_out"] = [self.nodes.topics[name]]
        return result

    def start(self): self.native.start()
    def stop(self):
        if self.nodes: self.nodes.close()
        self.native.close()
    def dispatch(self, action, args):
        name = args.get("_tool_name", "state")
        if action == "start":
            self.native.start()
            return self._topic_result(name, "running")
        if action == "stop":
            # The TCP client is shared by every S10 card and is closed only
            # when the driver bundle itself shuts down.
            return self._topic_result(name, "idle")
        if action == "info": return self._topic_result(name, "ready")
        reports = self.native.snapshot()["reports"]
        keys = {"imu": "1048577:15728640", "joints": "1048577:15728640",
                "joint_state": "1048577:15728640", "loco_state": "1048577:15728640",
                "battery": "1048578:15728640",
                "errors": "1048703:15728640", "navigation_status": "1048579:15728640"}
        if name == "state": return self.native.snapshot()
        if name == "lidar" and self.nodes: return {"state": "streaming", **self.nodes.topics[name]}
        if name not in keys: return {"state": "unsupported_card", "card": name}
        report = reports.get(keys[name])
        # Some firmware revisions serialize Type/Command as decimal strings,
        # while others use numeric JSON values.  Match both representations.
        if report is None:
            expected = tuple(int(part) for part in keys[name].split(":"))
            for candidate in reports.values():
                body = candidate.get("PatrolDevice", {}) if isinstance(candidate, dict) else {}
                try:
                    if (int(str(body.get("Type")), 0), int(str(body.get("Command")), 0)) == expected:
                        report = candidate
                        break
                except (TypeError, ValueError):
                    continue
        if not report: return {"state": "waiting_for_report", "report_key": keys[name]}
        items = report.get("PatrolDevice", {}).get("Items", {})
        if name in ("imu", "joints", "joint_state", "loco_state"):
            data = items.get("MotionStatus", {})
            if name == "imu": return {"acceleration_m_s2": [data.get("AccX"), data.get("AccY"), data.get("AccZ")], "angular_velocity_rad_s": [data.get("OmegaX"), data.get("OmegaY"), data.get("OmegaZ")], "orientation_rad": [data.get("Roll"), data.get("Pitch"), data.get("Yaw")]}
            if name == "joints": return _skeleton_payload(items)
            if name == "joint_state": return items.get("MotorStatus", {})
            basic_report = reports.get("1048676:15728640", {})
            basic_items = basic_report.get("PatrolDevice", {}).get("Items", {})
            basic = basic_items.get("BasicStatus", {})
            return {"motion_state": data, "basic_state": basic}
        if name == "battery": return {"batteries": items.get("BatteryList", [])}
        if name == "errors": return {"ErrorList": items.get("ErrorList", [])}
        return items


class S10MotionPlugin:
    ACTIONS = {
        "move": (["vx", "vy", "vyaw", "duration"], "导航模式下按速度移动；duration=-1 为兼容写法，最多持续 60 秒"),
        "stop_move": ([], "立即发送零速度停止运动"),
        "stand_up": ([], "切换到站立状态"), "stand_down": ([], "切换到趴下状态"),
        "damp": ([], "切换到阻尼状态"), "gait": (["gait"], "切换步态；请先确认机器人完全停止"),
        "mode": (["mode"], "切换常规、导航或辅助控制模式"),
        "axis": (["x", "y", "z", "roll", "pitch", "yaw", "duration"], "常规/辅助模式归一化轴控制"),
        "rl_control": ([], "切换到 RL 控制状态"),
    }
    def __init__(self, native):
        self.native = native
        self._condition = threading.Condition()
        self._active = None
        self._shutdown = False
        self._thread = threading.Thread(target=self._stream_loop, daemon=True, name="s10-motion-stream")

    def _acp_callback(self, action_id, status, result):
        _post_acp_callback(action_id, status, result, "loco")

    def _finish(self, active, status, result):
        if active and active.get("action_id"):
            try:
                self._acp_callback(active["action_id"], status, result)
            except Exception as exc:
                print(f"[ACP] {active['action_id']}: {exc}", flush=True)
    def get_tool(self):
        result = tool("loco", "actuator", "山猫 S10 运动控制。move 会切换到导航模式和平地步态后执行；duration=-1 最多持续 60 秒", action_schema(self.ACTIONS, {
            "gait": {"type": "string", "enum": list(GAITS)},
            "mode": {"type": "string", "enum": list(MODES)},
            **{axis: {"type": "number", "minimum": -1, "maximum": 1} for axis in ("x", "y", "z", "roll", "pitch", "yaw")},
            "vx": {"type": "number", "minimum": -1.67, "maximum": 1.67, "description": "Navigation forward/backward velocity in m/s"},
            "vy": {"type": "number", "minimum": -0.4, "maximum": 0.4, "description": "Navigation lateral velocity in m/s"},
            "vyaw": {"type": "number", "minimum": -1.0, "maximum": 1.0, "description": "Navigation yaw rate in rad/s"},
            "duration": {"type": "number", "minimum": -1, "maximum": 60, "default": 1,
                         "description": "Seconds; -1 is a compatibility alias for 60 seconds; 0 stops immediately"},
        }))
        schema = result["inputSchema"]
        schema["x-completion"] = {"actions": ["move", "axis"], "timeout": 65}
        return result
    def start(self):
        if not self._thread.is_alive(): self._thread.start()
    def stop(self):
        self._stop_stream()
        with self._condition:
            self._shutdown = True
            self._condition.notify_all()
        if self._thread.is_alive(): self._thread.join(timeout=1)
    def _send_zero(self, kind=None):
        zero = {"X": 0.0, "Y": 0.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
        if kind in (None, "axis"): self.native.send(0x00100001, 0x00100002, zero)
        if kind in (None, "velocity"): self.native.send(0x00100001, 0x00110002, zero)
    def _stop_stream(self, send_both=False):
        with self._condition:
            active = self._active
            self._active = None
            self._condition.notify_all()
        stop_error = None
        try:
            if send_both: self._send_zero()
            elif active: self._send_zero(active["kind"])
        except Exception as exc:
            stop_error = exc
        if active:
            if stop_error:
                self._finish(active, "error", {"state": "error", "message": f"zero command failed: {stop_error}"})
            else:
                self._finish(active, "cancelled", {"state": "cancelled", "reason": "stopped_or_replaced"})
        if stop_error:
            raise stop_error
    def _stream_loop(self):
        while True:
            with self._condition:
                while not self._shutdown and self._active is None: self._condition.wait()
                if self._shutdown: return
                command = self._active
                now = time.monotonic()
                if now >= command["deadline"]:
                    self._active = None
                    try:
                        self._send_zero(command["kind"])
                    except Exception as exc:
                        self._finish(command, "error", {"state": "error", "message": f"zero command failed: {exc}"})
                    else:
                        self._finish(command, "completed", {"state": "completed", "reason": "duration_elapsed"})
                    continue
                delay = command["next_send"] - now
                if delay > 0:
                    self._condition.wait(min(delay, command["deadline"] - now))
                    continue
                try:
                    self.native.send(command["type"], command["command"], command["items"])
                    command["next_send"] = time.monotonic() + 0.1
                except Exception as exc:
                    self._active = None
                    try:
                        self._send_zero(command["kind"])
                    except Exception as stop_exc:
                        exc = RuntimeError(f"{exc}; zero command failed: {stop_exc}")
                    self._finish(command, "error", {"state": "error", "message": str(exc)})
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop":
            self._stop_stream()
            return {"state": "idle"}
        if action == "info": return {"state": "ready"}
        if action == "stop_move":
            self._stop_stream(send_both=True)
            return {"state": "stopped", "sent_zero_to": ["axis", "velocity"]}
        if action in LOCO_STATES:
            self._stop_stream()
            value = LOCO_STATES[action]
            return self.native.send(0x00100001, 0x00200002, {"MotionParam": value})
        if action == "gait":
            self._stop_stream()
            return self.native.send(0x00100001, 0x00300002, {"GaitParam": GAITS[args["gait"]]})
        if action == "mode":
            self._stop_stream()
            return self.native.send(0x00100002, 0x00500002, {"Mode": MODES[args["mode"]]})
        if action in ("axis", "move"):
            axes = ("x", "y", "z", "roll", "pitch", "yaw")
            wire_names = {"x": "X", "y": "Y", "z": "Z", "roll": "Roll", "pitch": "Pitch", "yaw": "Yaw"}
            items = {wire_names[key]: float(args.get(key if action == "axis" else {"x": "vx", "y": "vy", "yaw": "vyaw"}.get(key, key), 0)) for key in axes}
            if action == "move":
                bounds = {"X": 1.67, "Y": 0.4, "Yaw": 1.0}
                if any(abs(items[key]) > limit for key, limit in bounds.items()):
                    raise ValueError("velocity exceeds documented limits: X=1.67 m/s, Y=0.4 m/s, Yaw=1 rad/s")
                kind, message_type, command = "velocity", 0x00100001, 0x00110002
            else:
                if any(abs(value) > 1 for value in items.values()): raise ValueError("axis values must be within [-1, 1]")
                kind, message_type, command = "axis", 0x00100001, 0x00100002
            duration = float(args.get("duration", 1))
            if not -1 <= duration <= 60: raise ValueError("duration must be within [-1, 60]")
            if duration == 0:
                self._stop_stream(send_both=True)
                action_id = f"s10_{action}_{uuid4().hex}"
                self._finish({"action_id": action_id}, "completed", {"state": "stopped", "reason": "zero_duration"})
                return {"state": "stopped", "sent_zero_to": ["axis", "velocity"], "action_id": action_id}
            self._stop_stream()
            if action == "move":
                self.native.send(0x00100002, 0x00500002, {"Mode": MODES["navigation"]})
                self.native.send(0x00100001, 0x00300002, {"GaitParam": GAITS["navigation_flat"]})
            action_id = f"s10_{action}_{uuid4().hex}"
            sent = self.native.send(message_type, command, items)
            with self._condition:
                now = time.monotonic()
                self._active = {"kind": kind, "type": message_type, "command": command, "items": items,
                                "deadline": now + (60 if duration == -1 else duration), "next_send": now + 0.1,
                                "action_id": action_id}
                self._condition.notify_all()
            return {**sent, "state": "streaming", "frequency_hz": 10,
                    "duration": 60 if duration == -1 else duration, "auto_stop": True,
                    "action_id": action_id}
        raise ValueError(f"unsupported S10 loco action: {action}")


class S10NavigationPlugin:
    CONFIRM_ACTIONS = {"goal", "localize"}
    ACTIONS = {
        "goal": (["x", "y", "z", "yaw", "point_type", "gait", "speed", "reverse", "avoid_obstacles", "autonomous"], "前往地图坐标系单点目标；异步等待任务结束"),
        "cancel": ([], "取消当前导航任务"), "status": ([], "查询导航任务状态"),
        "localize": (["x", "y", "z", "yaw"], "按地图坐标初始化/重置定位"),
        "pose": ([], "查询机器人地图坐标和姿态"),
        "perception_status": ([], "查询定位与避障状态"),
    }
    def __init__(self, native):
        self.native = native
        self._goal_lock = threading.Lock()
        self._active_goal = None

    def get_tool(self):
        result = tool("navigation", "actuator", "山猫 S10 Pro 导航控制；目标导航和定位初始化需要显式 confirm=true", action_schema(self.ACTIONS, {
            "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number", "default": 0},
            "yaw": {"type": "number"}, "point_type": {"type": "integer", "enum": [0, 1, 2, 3, 4], "default": 1},
            "gait": {"type": "string", "enum": ["navigation_flat", "navigation_stairs"], "default": "navigation_flat"},
            "speed": {"type": "integer", "enum": [0, 1, 2], "default": 0}, "reverse": {"type": "boolean", "default": False},
            "avoid_obstacles": {"type": "boolean", "default": True}, "autonomous": {"type": "boolean", "default": False},
            "confirm": {"type": "boolean", "description": "Required true for goal navigation and localization changes."},
        }))
        _mark_dangerous(result, self.CONFIRM_ACTIONS)
        timeout = max(10, math.ceil(float(self.native.config.get("navigation_task_timeout", 180))) + 10)
        result["inputSchema"]["x-completion"] = {"actions": ["goal"], "timeout": timeout}
        return result

    def _acp_callback(self, action_id, status, result):
        _post_acp_callback(action_id, status, result, "navigation")

    def _finish_goal(self, action_id, status, result):
        with self._goal_lock:
            if self._active_goal != action_id:
                return False
            self._active_goal = None
        try:
            self._acp_callback(action_id, status, result)
        except Exception as exc:
            print(f"[ACP] navigation {action_id}: {exc}", flush=True)
        return True

    @staticmethod
    def _response_error(response):
        body = response.get("PatrolDevice", response) if isinstance(response, dict) else {}
        items = body.get("Items", body) if isinstance(body, dict) else {}
        code = items.get("ErrorCode") if isinstance(items, dict) else None
        if code is None:
            return None
        try:
            raw_code = str(code).strip().lower()
            value = int(raw_code, 16 if raw_code.startswith("0x") else 10)
        except (TypeError, ValueError):
            return str(code)
        return None if value == 0 else value

    def _run_goal(self, action_id, item, timeout):
        try:
            response = self.native.request(0x00100003, 0x00100002, {"Item": [item]}, response_timeout=timeout)
            error = self._response_error(response)
            if error is None:
                self._finish_goal(action_id, "completed", {"state": "completed", "response": response})
            else:
                self._finish_goal(action_id, "error", {"state": "error", "error_code": error, "response": response})
        except Exception as exc:
            self._finish_goal(action_id, "error", {"state": "error", "message": str(exc)})

    def _cancel_goal(self):
        with self._goal_lock:
            action_id = self._active_goal
        if action_id is None:
            return None
        response = self.native.request(0x00100003, 0x00100004)
        self._finish_goal(action_id, "cancelled", {"state": "cancelled", "reason": "navigation_cancelled"})
        return response

    def start(self): pass
    def stop(self):
        if self._active_goal is not None:
            self._cancel_goal()

    def dispatch(self, action, args):
        if action in self.CONFIRM_ACTIONS:
            error = _confirmation_error(action, args)
            if error:
                return error
        if action == "start": return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info": return {"state": "ready"}
        if action == "status": return self.native.request(0x00100003, 0x00100001)
        if action == "cancel":
            response = self._cancel_goal()
            return response if response is not None else self.native.request(0x00100003, 0x00100004)
        if action == "pose": return self.native.request(0x00100003, 0x00200001)
        if action == "perception_status": return self.native.request(0x00100003, 0x00110001)
        if action == "localize":
            return self.native.request(0x00100003, 0x00200002, {"PosX": args["x"], "PosY": args["y"], "PosZ": args.get("z", 0), "Yaw": args["yaw"]})
        if action == "goal":
            item = {"Value": 0, "MapID": 0, "PosX": args["x"], "PosY": args["y"], "PosZ": args.get("z", 0),
                    "AngleYaw": args["yaw"], "PointInfo": int(args.get("point_type", 1)),
                    "Gait": GAITS[args.get("gait", "navigation_flat")], "Speed": int(args.get("speed", 0)),
                    "Manner": int(bool(args.get("reverse", False))), "ObsMode": 0 if args.get("avoid_obstacles", True) else 1,
                    "NavMode": int(bool(args.get("autonomous", False))), "Terrain": 0, "Posture": 0, "Waypoints": {}}
            timeout = float(self.native.config.get("navigation_task_timeout", 180))
            action_id = f"s10_navigation_{uuid4().hex}"
            with self._goal_lock:
                if self._active_goal is not None:
                    return {"ok": False, "code": "ACTION_IN_PROGRESS", "action_id": self._active_goal,
                            "error": "navigation goal already active; cancel it before sending another goal"}
                self._active_goal = action_id
            worker = threading.Thread(target=self._run_goal, args=(action_id, item, timeout),
                                      daemon=True, name="s10-navigation-goal")
            try:
                worker.start()
            except Exception as exc:
                with self._goal_lock:
                    if self._active_goal == action_id:
                        self._active_goal = None
                return {"ok": False, "code": "WORKER_START_FAILED", "error": str(exc)}
            return {"state": "accepted", "action": "goal", "action_id": action_id,
                    "task_timeout": timeout}
        raise ValueError(f"unsupported S10 navigation action: {action}")


class S10DevicePlugin:
    CONFIRM_ACTIONS = {"sleep", "wake", "auto_sleep", "sdk_mode"}
    ACTIONS = {"lights": (["head_mode", "head_color", "head_cycle", "tail_mode", "tail_color", "tail_cycle"], "分别设置头尾灯模式、颜色和周期"),
               "sleep": ([], "进入休眠"), "wake": ([], "唤醒"), "auto_sleep": (["minutes"], "配置心跳断联后的自动休眠等待时间"),
               "sdk_mode": (["enabled", "frequency"], "配置运动 SDK 模式及关节数据频率")}
    def __init__(self, native): self.native = native
    def get_tool(self):
        result = tool("device", "actuator", "山猫 S10 灯光与休眠管理；休眠和 SDK 模式更改需要显式 confirm=true", action_schema(self.ACTIONS, {
            "head_mode": {"type": "integer", "enum": list(range(6))}, "tail_mode": {"type": "integer", "enum": list(range(6))},
            "head_color": {"type": "integer", "enum": list(range(4))}, "tail_color": {"type": "integer", "enum": list(range(4))},
            "head_cycle": {"type": "integer", "minimum": 1}, "tail_cycle": {"type": "integer", "minimum": 1},
            "minutes": {"type": "integer", "minimum": 1}, "enabled": {"type": "boolean"},
            "frequency": {"type": "integer", "minimum": 1, "maximum": 200},
            "confirm": {"type": "boolean", "description": "Required true for sleep/wake and SDK-mode changes."},
        }))
        return _mark_dangerous(result, self.CONFIRM_ACTIONS)
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action in self.CONFIRM_ACTIONS:
            error = _confirmation_error(action, args)
            if error:
                return error
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action == "info": return {"state": "ready"}
        if action == "lights":
            def led(prefix):
                return {"Type": int(args.get(f"{prefix}_mode", 0)), "Color": [int(args.get(f"{prefix}_color", 1))], "Cycle": int(args.get(f"{prefix}_cycle", 1))}
            items = {"CustomMode": True, "Led": [led("head"), led("tail")]}
            return self.native.request(0x00100005, 0x00200002, items)
        if action in ("sleep", "wake", "auto_sleep"):
            items = {"Sleep": action == "sleep", "Auto": action == "auto_sleep"}
            if action == "auto_sleep": items["Time"] = int(args["minutes"])
            return self.native.request(0x00100002, 0x00400002, items)
        if action == "sdk_mode":
            frequency = int(args.get("frequency", 100))
            if not 1 <= frequency <= 200 or 1000 % frequency:
                raise ValueError("frequency must be 1..200 Hz and divide 1000")
            return self.native.request(0x00100005, 0x00300002, {"SDKEnable": bool(args["enabled"]), "Frequency": frequency})
        raise ValueError(f"unsupported S10 device action: {action}")


class S10ChargePlugin:
    CONFIRM_ACTIONS = {"start_charging", "stop_charging"}
    def __init__(self, native): self.native = native
    def get_tool(self):
        result = tool("charging", "actuator", "山猫 S10 自主充电控制；开始自主充电需要显式 confirm=true", action_schema(
            {"start_charging": ([], "开始自主充电"), "stop_charging": ([], "停止自主充电")}, {
                "confirm": {"type": "boolean", "description": "Required true to start autonomous charging."},
            }))
        return _mark_dangerous(result, self.CONFIRM_ACTIONS)
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action in self.CONFIRM_ACTIONS:
            error = _confirmation_error(action, args)
            if error:
                return error
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action == "info": return {"state": "ready"}
        if action == "start_charging": return self.native.request(0x00100004, 0x00100002, {"Charge": 1})
        if action == "stop_charging": return self.native.request(0x00100004, 0x00100002, {"Charge": 0})
        raise ValueError(f"unsupported S10 charging action: {action}")


def build_plugins(config, namespace=None, ros2=None):
    native = BasicServerClient(config)
    nodes = S10Nodes(config, namespace or "", ros2) if ros2 is not None else None
    plugins = [S10ModelPlugin(), S10StatePlugin(native, nodes), S10MotionPlugin(native), S10DevicePlugin(native), S10ChargePlugin(native)]
    if str(config.get("model_variant", "standard")).lower() == "pro":
        plugins.append(S10NavigationPlugin(native))
    return plugins
