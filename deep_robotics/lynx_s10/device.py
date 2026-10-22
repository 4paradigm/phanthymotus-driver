#!/usr/bin/env python3
"""云深处山猫 S10 原生 basic_server 驱动。"""

from __future__ import annotations

import threading
import time

from basic_server import BasicServerClient
from common.vendor_runtime import action_schema, tool


MOTION_STATES = {"stand": 1, "damping": 2, "lie": 4, "rl_control": 17}
GAITS = {"basic_flat": 0x1001, "basic_stairs": 0x1003, "navigation_flat": 0x3002, "navigation_stairs": 0x3003}
MODES = {"normal": 0, "navigation": 1, "assist": 2}


class S10StatePlugin:
    def __init__(self, native): self.native = native
    def get_tools(self):
        cards = {
            "state": "山猫 S10 原生 TLS 协议连接及全部设备主动上报状态",
            "motion_info": "山猫 S10 运控状态：速度、姿态、关节角度和运动状态（原生协议 10Hz）",
            "imu": "山猫 S10 IMU 加速度和角速度（由原生运控状态报告提供，10Hz）",
            "joints": "山猫 S10 16 个轮足关节角度（原生协议 10Hz）",
            "battery": "山猫 S10 电池电量、电压、温度和充电状态",
            "native_device_status": "山猫 S10 设备温度、外设使能和处理器状态",
            "native_basic_status": "山猫 S10 基础状态：运动、步态、模式、急停、电源、休眠和充电",
            "hard_estop": "山猫 S10 硬急停状态",
            "charge_status": "山猫 S10 自主充电状态及当前充电概览",
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
        keys = {"motion_info": "1048577:15728640", "imu": "1048577:15728640",
                "joints": "1048577:15728640", "battery": "1048578:15728640",
                "native_device_status": "1048578:15728640", "native_basic_status": "1048676:15728640",
                "hard_estop": "1048676:15728640", "charge_status": "1048676:15728640",
                "errors": "1048703:15728640", "navigation_status": "1048579:15728640"}
        if name == "state": return self.native.snapshot()
        report = reports.get(keys[name])
        if not report: return {"state": "waiting_for_report", "report_key": keys[name]}
        items = report.get("PatrolDevice", {}).get("Items", {})
        if name in ("motion_info", "imu", "joints"):
            data = items.get("MotionStatus", {})
            if name == "motion_info": return {"state": "available", **data}
            if name == "imu": return {"acceleration_m_s2": [data.get("AccX"), data.get("AccY"), data.get("AccZ")], "angular_velocity_rad_s": [data.get("OmegaX"), data.get("OmegaY"), data.get("OmegaZ")], "orientation_rad": [data.get("Roll"), data.get("Pitch"), data.get("Yaw")]}
            return {"joints_rad": items.get("MotorStatus", {}).get("Joint", [])}
        if name == "battery": return {"batteries": items.get("BatteryList", [])}
        if name == "native_device_status": return items
        if name in ("hard_estop", "native_basic_status", "charge_status"):
            basic = items.get("BasicStatus", {})
            if name == "hard_estop": return {"HES": basic.get("HES")}
            if name == "charge_status": return {"Charge": basic.get("Charge")}
            return basic
        if name == "errors": return {"ErrorList": items.get("ErrorList", [])}
        return items


class S10MotionPlugin:
    ACTIONS = {
        "stand": ([], "切换到站立状态"), "lie": ([], "切换到趴下状态"),
        "damping": ([], "切换到阻尼状态"), "gait": (["gait"], "切换步态；请先确认机器人完全停止"),
        "mode": (["mode"], "切换常规、导航或辅助控制模式"),
        "axis": (["x", "y", "z", "roll", "pitch", "yaw", "seconds"], "常规/辅助模式归一化轴控制"),
        "velocity": (["velocity_x", "velocity_y", "velocity_yaw", "seconds"], "导航模式真实速度控制"),
        "rl_control": ([], "切换到 RL 控制状态"), "stopmotion": ([], "发送零速度停止运动"),
    }
    def __init__(self, native):
        self.native = native
        self._condition = threading.Condition()
        self._active = None
        self._shutdown = False
        self._thread = threading.Thread(target=self._stream_loop, daemon=True, name="s10-motion-stream")
    def get_tool(self):
        return tool("motion", "actuator", "山猫 S10 官方运动、步态、模式及速度控制", action_schema(self.ACTIONS, {
            "gait": {"type": "string", "enum": list(GAITS)},
            "mode": {"type": "string", "enum": list(MODES)},
            **{axis: {"type": "number", "minimum": -1, "maximum": 1} for axis in ("x", "y", "z", "roll", "pitch", "yaw")},
            "velocity_x": {"type": "number", "minimum": -1.67, "maximum": 1.67, "description": "Navigation speed X in m/s"},
            "velocity_y": {"type": "number", "minimum": -0.4, "maximum": 0.4, "description": "Navigation speed Y in m/s"},
            "velocity_yaw": {"type": "number", "minimum": -1.0, "maximum": 1.0, "description": "Navigation yaw rate in rad/s"},
            "seconds": {"type": "number", "minimum": 0, "maximum": 60, "default": 1},
        }))
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
        if action in ("stop", "stopmotion"):
            self._stop_stream(send_both=True)
            return {"state": "stopped", "sent_zero_to": ["axis", "velocity"]}
        if action in MOTION_STATES:
            self._stop_stream()
            value = MOTION_STATES[action]
            return self.native.request(0x00100001, 0x00200002, {"MotionParam": value})
        if action == "gait":
            self._stop_stream()
            return self.native.request(0x00100001, 0x00300002, {"GaitParam": GAITS[args["gait"]]})
        if action == "mode":
            self._stop_stream()
            return self.native.request(0x00100002, 0x00500002, {"Mode": MODES[args["mode"]]})
        if action in ("axis", "velocity"):
            axes = ("x", "y", "z", "roll", "pitch", "yaw")
            wire_names = {"x": "X", "y": "Y", "z": "Z", "roll": "Roll", "pitch": "Pitch", "yaw": "Yaw"}
            items = {wire_names[key]: float(args.get(key if action == "axis" else {"x": "velocity_x", "y": "velocity_y", "yaw": "velocity_yaw"}.get(key, key), 0)) for key in axes}
            if action == "velocity":
                bounds = {"X": 1.67, "Y": 0.4, "Yaw": 1.0}
                if any(abs(items[key]) > limit for key, limit in bounds.items()):
                    raise ValueError("velocity exceeds documented limits: X=1.67 m/s, Y=0.4 m/s, Yaw=1 rad/s")
                kind, message_type, command = "velocity", 0x00100001, 0x00110002
            else:
                if any(abs(value) > 1 for value in items.values()): raise ValueError("axis values must be within [-1, 1]")
                kind, message_type, command = "axis", 0x00100001, 0x00100002
            duration = float(args.get("seconds", 1))
            if not 0 <= duration <= 60: raise ValueError("seconds must be within [0, 60]")
            self._stop_stream()
            sent = self.native.send(message_type, command, items)
            with self._condition:
                now = time.monotonic()
                self._active = {"kind": kind, "type": message_type, "command": command, "items": items,
                                "deadline": now + duration if duration else float("inf"), "next_send": now + 0.1}
                self._condition.notify_all()
            return {**sent, "state": "streaming", "frequency_hz": 10, "duration": duration, "auto_stop": duration > 0}
        raise ValueError(f"unsupported S10 motion action: {action}")


class S10NavigationPlugin:
    ACTIONS = {
        "goal": (["x", "y", "yaw"], "前往地图坐标系单点目标；等待任务结束后返回结果"),
        "cancel": ([], "取消当前导航任务"), "status": ([], "查询导航任务状态"),
        "localize": (["x", "y", "yaw"], "按地图坐标初始化/重置定位"),
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
    ACTIONS = {"lights": (["head_mode", "head_color", "tail_mode", "tail_color"], "分别设置头尾灯模式、颜色和周期"),
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
            {"start": ([], "开始自主充电"), "stop": ([], "停止自主充电")}, {}))
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action == "start": return self.native.request(0x00100004, 0x00100002, {"Charge": 1})
        if action == "stop": return self.native.request(0x00100004, 0x00100002, {"Charge": 0})
        if action in ("info",): return {"state": "ready"}
        raise ValueError(f"unsupported S10 charging action: {action}")


def build_plugins(config, namespace=None, ros2=None):
    native = BasicServerClient(config)
    plugins = [S10StatePlugin(native), S10MotionPlugin(native), S10DevicePlugin(native), S10ChargePlugin(native)]
    if str(config.get("model_variant", "standard")).lower() == "pro":
        plugins.append(S10NavigationPlugin(native))
    return plugins
