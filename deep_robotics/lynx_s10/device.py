#!/usr/bin/env python3
"""云深处山猫 S10 原生 basic_server 驱动。"""

from __future__ import annotations

import threading
import time
import math
from pathlib import Path

from basic_server import BasicServerClient
from common.vendor_runtime import action_schema, tool


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
            "q": _joint_value(raw, "q", "Q", "Position", "Pos", "Angle"),
            "dq": _joint_value(raw, "dq", "DQ", "Velocity", "Speed"),
            "tau": _joint_value(raw, "tau", "Tau", "Torque", "TorqueEst"),
            "temperature": _joint_value(raw, "temperature", "Temperature", "Temp"),
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
    def __init__(self, native): self.native = native
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
        }
        return [tool(name, "sensor", description) for name, description in cards.items()]
    def start(self): self.native.start()
    def stop(self): self.native.close()
    def dispatch(self, action, args):
        name = args.get("_tool_name", "state")
        if action == "start":
            self.native.start()
            return {"state": "running"}
        if action == "stop":
            if name == "state": self.native.close()
            return {"state": "idle"}
        if action == "info": return {"state": "ready"}
        reports = self.native.snapshot()["reports"]
        keys = {"imu": "1048577:15728640", "joints": "1048577:15728640",
                "joint_state": "1048577:15728640", "loco_state": "1048577:15728640",
                "battery": "1048578:15728640",
                "errors": "1048703:15728640", "navigation_status": "1048579:15728640"}
        if name == "state": return self.native.snapshot()
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
        "move": (["vx", "vy", "vyaw", "duration"], "导航模式下按速度移动；duration=-1 持续直到 stop_move"),
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
    def get_tool(self):
        result = tool("loco", "actuator", "山猫 S10 运动控制；使用 move/stop_move、stand_up/stand_down、damp，以及 S10 步态和模式控制", action_schema(self.ACTIONS, {
            "gait": {"type": "string", "enum": list(GAITS)},
            "mode": {"type": "string", "enum": list(MODES)},
            **{axis: {"type": "number", "minimum": -1, "maximum": 1} for axis in ("x", "y", "z", "roll", "pitch", "yaw")},
            "vx": {"type": "number", "minimum": -1.67, "maximum": 1.67, "description": "Navigation forward/backward velocity in m/s"},
            "vy": {"type": "number", "minimum": -0.4, "maximum": 0.4, "description": "Navigation lateral velocity in m/s"},
            "vyaw": {"type": "number", "minimum": -1.0, "maximum": 1.0, "description": "Navigation yaw rate in rad/s"},
            "duration": {"type": "number", "minimum": -1, "maximum": 60, "default": 1,
                         "description": "Seconds; -1 continues until stop_move; 0 stops immediately"},
        }))
        result["x-completion"] = {"actions": ["move", "stop_move", "stand_up", "stand_down"], "timeout": 30}
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
        if send_both: self._send_zero()
        elif active: self._send_zero(active["kind"])
    def _stream_loop(self):
        while True:
            with self._condition:
                while not self._shutdown and self._active is None: self._condition.wait()
                if self._shutdown: return
                command = self._active
                now = time.monotonic()
                if now >= command["deadline"]:
                    self._active = None
                    self._send_zero(command["kind"])
                    continue
                delay = command["next_send"] - now
                if delay > 0:
                    self._condition.wait(min(delay, command["deadline"] - now))
                    continue
                try:
                    self.native.send(command["type"], command["command"], command["items"])
                    command["next_send"] = time.monotonic() + 0.1
                except Exception:
                    self._active = None
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
            return self.native.request(0x00100001, 0x00200002, {"MotionParam": value})
        if action == "gait":
            self._stop_stream()
            return self.native.request(0x00100001, 0x00300002, {"GaitParam": GAITS[args["gait"]]})
        if action == "mode":
            self._stop_stream()
            return self.native.request(0x00100002, 0x00500002, {"Mode": MODES[args["mode"]]})
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
                return {"state": "stopped", "sent_zero_to": ["axis", "velocity"]}
            self._stop_stream()
            sent = self.native.send(message_type, command, items)
            with self._condition:
                now = time.monotonic()
                self._active = {"kind": kind, "type": message_type, "command": command, "items": items,
                                "deadline": now + duration if duration > 0 else float("inf"), "next_send": now + 0.1}
                self._condition.notify_all()
            return {**sent, "state": "streaming", "frequency_hz": 10,
                    "duration": duration, "auto_stop": duration > 0}
        raise ValueError(f"unsupported S10 loco action: {action}")


class S10NavigationPlugin:
    ACTIONS = {
        "goal": (["x", "y", "z", "yaw", "point_type", "gait", "speed", "reverse", "avoid_obstacles", "autonomous"], "前往地图坐标系单点目标；等待任务结束后返回结果"),
        "cancel": ([], "取消当前导航任务"), "status": ([], "查询导航任务状态"),
        "localize": (["x", "y", "z", "yaw"], "按地图坐标初始化/重置定位"),
        "pose": ([], "查询机器人地图坐标和姿态"),
        "perception_status": ([], "查询定位与避障状态"),
    }
    def __init__(self, native): self.native = native
    def get_tool(self):
        return tool("navigation", "actuator", "山猫 S10 Pro 单点导航、取消、定位初始化和状态查询", action_schema(self.ACTIONS, {
            "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number", "default": 0},
            "yaw": {"type": "number"}, "point_type": {"type": "integer", "enum": [0, 1, 2, 3, 4], "default": 1},
            "gait": {"type": "string", "enum": ["navigation_flat", "navigation_stairs"], "default": "navigation_flat"},
            "speed": {"type": "integer", "enum": [0, 1, 2], "default": 0}, "reverse": {"type": "boolean", "default": False},
            "avoid_obstacles": {"type": "boolean", "default": True}, "autonomous": {"type": "boolean", "default": False},
        }))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action == "info": return {"state": "ready"}
        if action == "status": return self.native.request(0x00100003, 0x00100001)
        if action == "cancel": return self.native.request(0x00100003, 0x00100004)
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
            return self.native.request(0x00100003, 0x00100002, {"Item": [item]}, response_timeout=timeout)
        raise ValueError(f"unsupported S10 navigation action: {action}")


class S10DevicePlugin:
    ACTIONS = {"lights": (["head_mode", "head_color", "head_cycle", "tail_mode", "tail_color", "tail_cycle"], "分别设置头尾灯模式、颜色和周期"),
               "sleep": ([], "进入休眠"), "wake": ([], "唤醒"), "auto_sleep": (["minutes"], "配置心跳断联后的自动休眠等待时间"),
               "sdk_mode": (["enabled", "frequency"], "配置运动 SDK 模式及关节数据频率")}
    def __init__(self, native): self.native = native
    def get_tool(self):
        return tool("device", "actuator", "山猫 S10 灯光与休眠管理", action_schema(self.ACTIONS, {
            "head_mode": {"type": "integer", "enum": list(range(6))}, "tail_mode": {"type": "integer", "enum": list(range(6))},
            "head_color": {"type": "integer", "enum": list(range(4))}, "tail_color": {"type": "integer", "enum": list(range(4))},
            "head_cycle": {"type": "integer", "minimum": 1}, "tail_cycle": {"type": "integer", "minimum": 1},
            "minutes": {"type": "integer", "minimum": 1}, "enabled": {"type": "boolean"},
            "frequency": {"type": "integer", "minimum": 1, "maximum": 200},
        }))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
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
    def __init__(self, native): self.native = native
    def get_tool(self):
        return tool("charging", "actuator", "山猫 S10 自主充电控制（机器人需已位于充电桩前）", action_schema(
            {"start_charging": ([], "开始自主充电"), "stop_charging": ([], "停止自主充电")}, {}))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action == "info": return {"state": "ready"}
        if action == "start_charging": return self.native.request(0x00100004, 0x00100002, {"Charge": 1})
        if action == "stop_charging": return self.native.request(0x00100004, 0x00100002, {"Charge": 0})
        raise ValueError(f"unsupported S10 charging action: {action}")


def build_plugins(config, namespace=None, ros2=None):
    native = BasicServerClient(config)
    plugins = [S10ModelPlugin(), S10StatePlugin(native), S10MotionPlugin(native), S10DevicePlugin(native), S10ChargePlugin(native)]
    if str(config.get("model_variant", "standard")).lower() == "pro":
        plugins.append(S10NavigationPlugin(native))
    return plugins
