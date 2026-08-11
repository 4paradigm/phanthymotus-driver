#!/usr/bin/env python3
"""
x-humanoid/bumi/device.py — Bumi Edu Max 设备插件（传感器 + 执行器）。

设计原则：
  - 一个设备 = 一个 tool (或 multi-tool plugin)
  - sensor：启动时自动 start，通过 DDS 回调持续获取数据并桥接到 Agent Core topic
  - actuator：action 参数分发操作，通过 publish_cmd() 下发控制指令到运控板
  - resource：返回静态数据 (如 URDF)
  - 所有插件通过 BumiSDK 共享 HighController / LowController / AoLionDriver 实例

Bumi 通信架构：
  - SDK (pybind11): HighController + LowController + AoLionDriver + MediaController
  - DDS (CycloneDDS 0.11): 算力板 ←→ 运控板，500Hz 状态推送 + 控制指令下发
  - Agent Core: HTTP JSON-RPC (MCP) 暴露工具，传感器 topic 输出

插件列表：
  StatePlugin           (sensor, multi-tool) — joints/imu/battery/estop/robot_faults/model
  CameraPlugin          (sensor)             — RealSense D435i 头部 RGB
  DepthCameraPlugin      (sensor)            — RealSense D435i 深度图
  PointCloudPlugin       (sensor)            — RealSense D435i 彩色点云
  RemoteEventPlugin      (sensor)            — 手柄遥控事件 (DDS)
  MotorsPlugin           (sensor)            — 电机状态快照 (2Hz)
  JoystickDirectPlugin   (sensor)            — 手柄直连数据 (算力板 USB)
  StandPlugin            (actuator)          — 站立/使能/失能/准备模式切换
  WalkPlugin             (actuator)          — 双足行走 (x,y,z 速度向量)
  ArmGesturePlugin       (actuator)          — 挥手/握手/欢呼/擦眼泪
  DancePlugin            (actuator)          — 舞蹈模式 (dance1/2/3)
  TeachPlugin             (actuator)         — 示教录制与回放
  FallRecoveryPlugin      (actuator)         — 倒地起身/起身倒地
  RLPolicyPlugin          (actuator)         — RL 策略部署 (LowController + ONNX)
"""

from __future__ import annotations

import base64
import math
import time
import threading
from pathlib import Path

# ── SSE publish callback (set by main.py) ──────────────────────────────────────

_publish_fn = None


def set_publish_fn(fn):
    """Set the SSE publish function from main.py. Used by camera plugins to push data to Agent Core."""
    global _publish_fn
    _publish_fn = fn


def _publish(topic: str, data: dict):
    """Publish data to Agent Core via SSE. No-op if publish function is not set."""
    if _publish_fn is not None:
        _publish_fn({"topic": topic, "data": data})

# ── 关节映射: motor_id → joint_name (21 DOF) ─────────────────────────────────

_JOINTS = {
    # 左臂 (4 DOF)
    0: "arm_l1_joint",    # 左臂肩关节 pitch
    1: "arm_l2_joint",    # 左臂肩关节 roll
    2: "arm_l3_joint",    # 左臂关节 yaw
    3: "arm_l4_joint",    # 左臂肘关节 pitch
    # 左腿 (6 DOF)
    4: "leg_l1_joint",    # 左腿髋关节 pitch
    5: "leg_l2_joint",    # 左腿髋关节 roll
    6: "leg_l3_joint",    # 左大腿关节 yaw
    7: "leg_l4_joint",    # 左腿膝关节 pitch
    8: "leg_l5_joint",    # 左腿踝关节 pitch
    9: "leg_l6_joint",    # 左腿踝关节 roll
    # 右臂 (4 DOF)
    10: "arm_r1_joint",   # 右臂肩关节 pitch
    11: "arm_r2_joint",   # 右臂肩关节 roll
    12: "arm_r3_joint",   # 右臂关节 yaw
    13: "arm_r4_joint",   # 右臂肘关节 pitch
    # 右腿 (6 DOF)
    14: "leg_r1_joint",   # 右腿髋关节 pitch
    15: "leg_r2_joint",   # 右腿髋关节 roll
    16: "leg_r3_joint",   # 右大腿关节 yaw
    17: "leg_r4_joint",   # 右腿膝关节 pitch
    18: "leg_r5_joint",   # 右腿踝关节 pitch
    19: "leg_r6_joint",   # 右腿踝关节 roll
    # 腰 (1 DOF)
    20: "waist_1_joint",  # 腰关节 yaw
}

# ── 关节限位 (rad): motor_id → (min_rad, max_rad, max_torque_Nm) ─────────────

_JOINT_LIMITS = {
    0: (-3.14, 1.57, 5),      # 左臂肩 pitch
    1: (-0.14, 1.94, 5),      # 左臂肩 roll
    2: (-1.57, 1.57, 5),      # 左臂 yaw
    3: (-2.26, 0.00, 5),      # 左臂肘 pitch
    4: (-2.09, 2.09, 60),     # 左腿髋 pitch
    5: (-0.66, 1.57, 60),     # 左腿髋 roll
    6: (-2.53, 2.53, 15),     # 左大腿 yaw
    7: (0.00, 2.24, 60),      # 左膝 pitch
    8: (-0.96, 0.44, 30),     # 左踝 pitch
    9: (-0.17, 0.17, 30),     # 左踝 roll
    10: (-3.14, 1.57, 5),     # 右臂肩 pitch
    11: (-1.94, 0.14, 5),     # 右臂肩 roll
    12: (-1.57, 1.57, 5),     # 右臂 yaw
    13: (-2.26, 0.00, 5),     # 右臂肘 pitch
    14: (-2.09, 2.09, 60),    # 右腿髋 pitch
    15: (-1.57, 0.66, 60),    # 右腿髋 roll
    16: (-2.53, 2.53, 15),    # 右大腿 yaw
    17: (0.00, 2.24, 60),     # 右膝 pitch
    18: (-0.96, 0.44, 30),    # 右踝 pitch
    19: (-0.17, 0.17, 30),    # 右踝 roll
    20: (-1.57, 1.57, 27),    # 腰 yaw
}

# ── 电机错误码映射 ────────────────────────────────────────────────────────────

_MOTOR_ERROR_DESCRIPTIONS = {
    0x02: "motor_over_current",
    0x03: "motor_under_voltage",
    0x04: "encoder_error",
    0x06: "brake_voltage_over",
    0x07: "drv_driver_error",
    0x08: "over_voltage",
    0x09: "under_voltage",
    0x0A: "over_current",
    0x0B: "mos_over_temperature",
    0x0C: "coil_over_temperature",
    0x0D: "communication_lost",
    0x0E: "overload",
}

# ── workmode 映射 ────────────────────────────────────────────────────────────

_WORKMODE_NAMES = {
    0: "enabled",
    1: "ready",
    2: "walk",
    5: "dance",
    8: "swing_wave",
    9: "shake_hands",
    10: "cheer",
    11: "start_teach",
    12: "end_teach",
    14: "save_teach_1",
    23: "play_teach",
    26: "protection",
    27: "fall_to_stand",
    28: "stand_to_fall",
    29: "save_teach_2",
    30: "disabled",
    31: "dance_mode_1",
    32: "dance_mode_2",
    33: "tear_action",
}


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


# ══════════════════════════════════════════════════════════════════════════════
# StatePlugin (sensor, multi-tool)
# ══════════════════════════════════════════════════════════════════════════════

class StatePlugin:
    """关节状态 + IMU + 电池 + 急停 + 故障汇总 + URDF 模型"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False
        self._lock = threading.Lock()

        self._joint_data = {}
        self._imu_data = {}
        self._battery = {}
        self._workmode = None

        self._topic_joints = f"/{namespace}/state/joints"
        self._topic_imu = f"/{namespace}/state/imu"
        self._topic_battery = f"/{namespace}/state/battery"
        self._topic_estop = f"/{namespace}/state/estop"
        self._topic_faults = f"/{namespace}/state/faults"

        self._urdf_path = Path(__file__).parent / "resource" / "bumi_model.urdf"

    def get_tools(self) -> list:
        return [
            {
                "name": "joints",
                "type": "sensor",
                "description": "Bumi 全身关节状态 — 21 个电机的 pos/vel/tau/temperature/error",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_joints, "format": "sensor/skeleton"}],
            },
            {
                "name": "imu",
                "type": "sensor",
                "description": "Bumi IMU 姿态数据 — 四元数 + 角速度 + 线加速度 (含协方差矩阵)",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_imu, "format": "data/json"}],
            },
            {
                "name": "battery",
                "type": "sensor",
                "description": "Bumi 电池状态 — SOC(电量) / SOH(健康度) / 温度 / 告警",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_battery, "format": "data/json"}],
            },
            {
                "name": "estop",
                "type": "sensor",
                "description": "Bumi 急停/保护状态 — workmode=26 保护模式 / workmode=30 失能",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_estop, "format": "data/json"}],
            },
            {
                "name": "robot_faults",
                "type": "sensor",
                "description": "Bumi 机器人故障汇总 — 各电机 error 码聚合 + 保护模式 + 电池告警",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_faults, "format": "data/json"}],
            },
            {
                "name": "model",
                "type": "resource",
                "description": "Bumi URDF 骨架模型 — 用于 3D 可视化",
                "inputSchema": {"type": "object", "properties": {}},
            },
        ]

    def start(self):
        self._running = True
        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

    def stop(self):
        self._running = False

    def _publish_loop(self):
        """从 SDK 轮询状态数据并发布到 topic。"""
        joint_counter = 0
        while self._running:
            time.sleep(0.1)  # 10Hz
            joint_counter += 1

            ctrl = self._sdk.high
            if ctrl is None:
                continue

            try:
                # 关节状态 (10Hz)
                joint_state = ctrl.get_joint_state()
                with self._lock:
                    joints = []
                    for i, m in enumerate(joint_state):
                        name = _JOINTS.get(i, f"motor_{i}")
                        self._joint_data[i] = {
                            "pos": float(m.pos),
                            "vel": float(m.vel),
                            "tau": float(m.tau),
                            "motor_id": int(m.motor_id),
                            "error": int(m.error),
                            "temperature": int(m.temperature),
                        }
                        joints.append({
                            "idx": i,
                            "name": name,
                            "q": float(m.pos),
                            "dq": float(m.vel),
                            "tau": float(m.tau),
                            "temp": int(m.temperature),
                            "error": int(m.error),
                        })
                    self._latest_joints = {"joints": joints}
                    _publish(self._topic_joints, self._latest_joints)

                # IMU (随关节一起发布，10Hz)
                imu = ctrl.get_imu_data()
                with self._lock:
                    self._imu_data = {
                        "ori": list(imu.ori),
                        "ori_cov": list(imu.ori_cov),
                        "angular_vel": list(imu.angular_vel),
                        "angular_vel_cov": list(imu.angular_vel_cov),
                        "linear_acc": list(imu.linear_acc),
                        "linear_acc_cov": list(imu.linear_acc_cov),
                    }
                _publish(self._topic_imu, self._imu_data)

                # workmode
                self._workmode = ctrl.get_mode()

                # 电池 + estop + faults (1Hz)
                if joint_counter % 10 == 0:
                    bms = ctrl.get_robot_bms_data()
                    with self._lock:
                        self._battery = {
                            "soc": int(bms.battery_soc),
                            "soh": int(bms.battery_soh),
                            "temp": int(bms.battery_temp),
                            "alarm": int(bms.battery_alarm),
                        }
                    _publish(self._topic_battery, self._battery)

                    # estop
                    mode = self._workmode
                    _publish(self._topic_estop, {
                        "workmode": mode,
                        "mode_name": _WORKMODE_NAMES.get(mode, "unknown"),
                        "protection_active": mode == 26,
                        "disabled": mode == 30,
                    })

                    # faults
                    _publish(self._topic_faults, self._collect_faults())

            except Exception as e:
                print(f"[StatePlugin] poll error: {e}", flush=True)

    def dispatch(self, action_or_tool: str, args: dict) -> dict:
        if action_or_tool == "model":
            try:
                return {"urdf": self._urdf_path.read_text()}
            except FileNotFoundError:
                return {"error": "URDF file not found"}
        if action_or_tool == "joints":
            with self._lock:
                return getattr(self, "_latest_joints", {"joints": []})
        if action_or_tool == "imu":
            with self._lock:
                return self._imu_data or {"state": "no_data"}
        if action_or_tool == "battery":
            with self._lock:
                return self._battery or {"state": "no_data"}
        if action_or_tool == "estop":
            mode = self._workmode
            return {
                "workmode": mode,
                "mode_name": _WORKMODE_NAMES.get(mode, "unknown"),
                "protection_active": mode == 26,
                "disabled": mode == 30,
            }
        if action_or_tool == "robot_faults":
            return self._collect_faults()
        if action_or_tool == "start":
            return {"state": "running"}
        if action_or_tool == "stop":
            return {"state": "idle"}
        if action_or_tool == "info":
            tool_name = args.get("_tool_name", "joints")
            topic_map = {
                "joints": (self._topic_joints, "sensor/skeleton"),
                "imu": (self._topic_imu, "data/json"),
                "battery": (self._topic_battery, "data/json"),
                "estop": (self._topic_estop, "data/json"),
                "robot_faults": (self._topic_faults, "data/json"),
            }
            topic, fmt = topic_map.get(tool_name, (self._topic_joints, "sensor/skeleton"))
            return {"state": "running", "topic_out": [{"topic": topic, "format": fmt}]}
        return {"error": f"unknown action: {action_or_tool}"}

    def _collect_faults(self) -> dict:
        faults = []
        with self._lock:
            for idx, data in self._joint_data.items():
                err = data.get("error", 0)
                if err != 0:
                    faults.append({
                        "motor_id": idx,
                        "joint": _JOINTS.get(idx, f"motor_{idx}"),
                        "error_code": err,
                        "description": _MOTOR_ERROR_DESCRIPTIONS.get(err, "unknown_error"),
                    })
        mode = self._workmode
        return {
            "motor_faults": faults,
            "protection_mode": mode == 26,
            "workmode": mode,
            "mode_name": _WORKMODE_NAMES.get(mode, "unknown"),
            "battery_alarm": self._battery.get("alarm", 0) if self._battery else 0,
        }


# ══════════════════════════════════════════════════════════════════════════════
# CameraPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class CameraPlugin:
    """RealSense D435i 头部 RGB 相机"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._topic = f"/{namespace}/camera/head"
        self._running = False
        self._pipeline = None
        self._pub_thread = None
        self._lock = threading.Lock()
        self._latest = None

    def get_tool(self) -> dict:
        return {
            "name": "camera_head",
            "type": "sensor",
            "description": "Bumi 头部相机 (RealSense D435i RGB) — 彩色图像流",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "image/jpeg"}],
        }

    def start(self):
        self._running = True
        self._pub_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._pub_thread.start()

    def stop(self):
        self._running = False
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass

    def _capture_loop(self):
        try:
            import pyrealsense2 as rs
            import numpy as np
            import cv2

            self._pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self._pipeline.start(config)
            print("[CameraPlugin] RealSense color stream started")

            while self._running:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                img = np.asanyarray(color_frame.get_data())
                _, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 50])
                b64 = base64.b64encode(jpeg.tobytes()).decode("ascii")
                with self._lock:
                    self._latest = {
                        "image": b64,
                        "format": "image/jpeg",
                        "width": 640,
                        "height": 480,
                        "timestamp": time.time(),
                    }
                _publish(self._topic, self._latest)
                time.sleep(0.033)  # ~30Hz capture
        except ImportError as e:
            print(f"[CameraPlugin] WARNING: import failed ({e})")
        except Exception as e:
            print(f"[CameraPlugin] capture error: {e}")

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}
        if action in ("read", "get", "camera_head"):
            with self._lock:
                return dict(self._latest) if self._latest else {"state": "no_data"}
        return {"state": "running" if self._running else "idle"}


# ══════════════════════════════════════════════════════════════════════════════
# DepthCameraPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class DepthCameraPlugin:
    """RealSense D435i 深度图 (Z16)"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._topic = f"/{namespace}/camera/head/depth"
        self._max_hz = max(1.0, min(float(plugin_config.get("hz", 8)), 15.0))
        self._running = False
        self._pipeline = None
        self._lock = threading.Lock()
        self._latest = None

    def get_tool(self) -> dict:
        return {
            "name": "camera_depth",
            "type": "sensor",
            "description": "Bumi 头部深度图 (RealSense D435i Z16)",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "image/depth-z16"}],
        }

    def start(self):
        self._running = True
        self._pub_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._pub_thread.start()

    def stop(self):
        self._running = False
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass

    def _capture_loop(self):
        try:
            import pyrealsense2 as rs
            self._pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self._pipeline.start(config)
            print("[DepthCameraPlugin] RealSense depth stream started")

            last_pub = 0.0
            while self._running:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    continue
                now = time.monotonic()
                if now - last_pub < 1.0 / self._max_hz:
                    continue
                last_pub = now
                depth_data = np.asanyarray(depth_frame.get_data())
                b64 = base64.b64encode(depth_data.tobytes()).decode("ascii")
                with self._lock:
                    self._latest = {
                        "depth": b64,
                        "format": "image/depth-z16",
                        "width": 640,
                        "height": 480,
                        "timestamp": time.time(),
                    }
                _publish(self._topic, self._latest)
        except ImportError as e:
            print(f"[DepthCameraPlugin] WARNING: import failed ({e})")
        except Exception as e:
            print(f"[DepthCameraPlugin] capture error: {e}")

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "image/depth-z16"}]}
        if action in ("read", "get", "camera_depth"):
            with self._lock:
                return dict(self._latest) if self._latest else {"state": "no_data"}
        return {"state": "running" if self._running else "idle"}


# ══════════════════════════════════════════════════════════════════════════════
# PointCloudPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class PointCloudPlugin:
    """RealSense D435i 彩色点云"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._topic = f"/{namespace}/camera/head/points"
        self._floor_offset_m = float(plugin_config.get("floor_offset_m", 0.98))
        self._running = False
        self._pipeline = None
        self._lock = threading.Lock()
        self._latest = None

    def get_tool(self) -> dict:
        return {
            "name": "camera_pointcloud",
            "type": "sensor",
            "description": "Bumi 头部彩色点云 (RealSense D435i 深度对齐 RGB)",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "sensor/pointcloud"}],
        }

    def start(self):
        self._running = True
        self._pub_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._pub_thread.start()

    def stop(self):
        self._running = False
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass

    def _capture_loop(self):
        try:
            import pyrealsense2 as rs
            self._pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self._pipeline.start(config)
            print("[PointCloudPlugin] RealSense point cloud stream started")

            while self._running:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue
                # Build aligned point cloud and publish
                try:
                    import numpy as np
                    depth_data = np.asanyarray(depth_frame.get_data())
                    color_data = np.asanyarray(color_frame.get_data())
                    with self._lock:
                        self._latest = {
                            "format": "sensor/pointcloud",
                            "width": 640,
                            "height": 480,
                            "floor_offset_m": self._floor_offset_m,
                            "timestamp": time.time(),
                            "note": "point cloud computation deferred to Agent Core (depth + color available)",
                        }
                    _publish(self._topic, self._latest)
                except Exception as e:
                    print(f"[PointCloudPlugin] publish error: {e}")
                time.sleep(0.1)
        except ImportError as e:
            print(f"[PointCloudPlugin] WARNING: import failed ({e})")
        except Exception as e:
            print(f"[PointCloudPlugin] capture error: {e}")

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "sensor/pointcloud"}]}
        if action in ("read", "get", "camera_pointcloud"):
            with self._lock:
                return dict(self._latest) if self._latest else {"state": "no_data"}
        return {"state": "running" if self._running else "idle"}


# ══════════════════════════════════════════════════════════════════════════════
# RemoteEventPlugin / MotorsPlugin / JoystickDirectPlugin (sensors)
# ══════════════════════════════════════════════════════════════════════════════

class _JsonSensor:
    """传感器基类 — 从 SDK 轮询数据并输出 JSON。"""

    _format = "data/json"

    def _tool(self, name, description, topic):
        return {
            "name": name,
            "type": "sensor",
            "description": description,
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": topic, "format": self._format}],
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": self._format}]}
        return {"state": "running" if self._running else "idle"}


class RemoteEventPlugin(_JsonSensor):
    """手柄遥控事件 (DDS 获取)"""

    def __init__(self, plugin_config, namespace, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False
        self._topic = f"/{namespace}/remote/event"
        self._latest = {"axes": [0, 0], "buttons": [0] * 14}

    def get_tool(self):
        return self._tool("remote_event",
                          "Bumi 手柄遥控事件 — 摇杆 axes[2] + 14 个按钮 (DDS)",
                          self._topic)

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _poll_loop(self):
        while self._running:
            time.sleep(0.2)  # 5Hz
            ctrl = self._sdk.high
            if ctrl is None:
                continue
            try:
                joy = ctrl.from_dds_get_joydata()
                self._latest = {
                    "axes": list(joy.axes),
                    "buttons": [int(b) for b in joy.button],
                }
                _publish(self._topic, self._latest)
            except Exception as e:
                print(f"[RemoteEventPlugin] poll error: {e}", flush=True)

    def dispatch(self, action, args):
        if action in ("read", "get", "remote_event"):
            return dict(self._latest)
        return super().dispatch(action, args)


class MotorsPlugin(_JsonSensor):
    """电机状态快照 (2Hz)"""

    def __init__(self, plugin_config, namespace, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False
        self._topic = f"/{namespace}/state/motors"
        self._latest = {"motors": []}

    def get_tool(self):
        return self._tool("motors",
                          "Bumi 电机状态快照 — 21 电机 ID + pos/vel/tau/temp/error (2Hz)",
                          self._topic)

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _poll_loop(self):
        while self._running:
            time.sleep(0.5)  # 2Hz
            ctrl = self._sdk.high
            if ctrl is None:
                continue
            try:
                joint_state = ctrl.get_joint_state()
                motors = []
                for i, m in enumerate(joint_state):
                    motors.append({
                        "idx": i,
                        "name": _JOINTS.get(i, f"motor_{i}"),
                        "motor_id": int(m.motor_id),
                        "pos": float(m.pos),
                        "vel": float(m.vel),
                        "tau": float(m.tau),
                        "temp": int(m.temperature),
                        "error": int(m.error),
                    })
                self._latest = {"motors": motors}
                _publish(self._topic, self._latest)
            except Exception as e:
                print(f"[MotorsPlugin] poll error: {e}", flush=True)

    def dispatch(self, action, args):
        if action in ("read", "get", "motors"):
            return dict(self._latest)
        return super().dispatch(action, args)


class JoystickDirectPlugin(_JsonSensor):
    """手柄直连数据 (算力板 USB, AoLionDriver)"""

    def __init__(self, plugin_config, namespace, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False
        self._topic = f"/{namespace}/remote/joystick_direct"
        self._latest = {"axes": [0, 0], "buttons": [0] * 14, "source": "aolion_direct"}

    def get_tool(self):
        return self._tool("joystick_direct",
                          "Bumi 手柄直连数据 — 摇杆 + 14 按钮 (算力板 USB AoLionDriver)",
                          self._topic)

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _poll_loop(self):
        while self._running:
            time.sleep(0.2)  # 5Hz
            al = self._sdk.aolion
            if al is None:
                continue
            try:
                joy = al.getremotedata()
                self._latest = {
                    "axes": list(joy.axes),
                    "buttons": [int(b) for b in joy.button],
                    "source": "aolion_direct",
                }
                _publish(self._topic, self._latest)
            except Exception as e:
                print(f"[JoystickDirectPlugin] poll error: {e}", flush=True)

    def dispatch(self, action, args):
        if action in ("read", "get", "joystick_direct"):
            return dict(self._latest)
        return super().dispatch(action, args)


# ══════════════════════════════════════════════════════════════════════════════
# Actuator Plugins
# ══════════════════════════════════════════════════════════════════════════════

class _ActuatorBase:
    """执行器基类 — 封装 publish_cmd 调用和 workmode 检查。"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False

    def start(self):
        self._running = True

    def stop(self):
        self._running = False

    def dispatch(self, action: str, args: dict) -> dict | None:
        """Base dispatch — handles start/stop lifecycle. Subclasses should call super().dispatch() as fallback."""
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}

    def _publish_cmd(self, x=0.0, y=0.0, z=0.0, action_cmd=None, index=0):
        """调用 HighController.publish_cmd，每次至少间隔 2ms。"""
        ctrl = self._sdk.high
        if ctrl is None:
            return {"error": "HighController not initialized"}
        try:
            from highcontrol_py import ControlCmd
            if action_cmd is not None:
                ctrl.publish_cmd(x, y, z, action_cmd, index)
            else:
                ctrl.publish_cmd(x, y, z, ControlCmd.DEFAULT, index)
            time.sleep(0.002)  # 至少 2ms 延时
            return {"state": "ok"}
        except Exception as e:
            return {"error": str(e)}

    def _get_mode(self):
        ctrl = self._sdk.high
        if ctrl is None:
            return None
        try:
            return ctrl.get_mode()
        except Exception:
            return None

    def _check_mode(self, expected_modes: list) -> dict | None:
        """检查当前 workmode 是否在预期范围内，返回错误 dict 或 None。"""
        mode = self._get_mode()
        if mode is None:
            return {"error": "cannot read workmode"}
        if expected_modes and mode not in expected_modes:
            return {
                "error": f"workmode {mode} ({_WORKMODE_NAMES.get(mode, 'unknown')}) "
                         f"not in expected modes {expected_modes}",
            }
        return None


class StandPlugin(_ActuatorBase):
    """站立/使能状态机 — enable/disable/ready/get_mode"""

    def get_tool(self) -> dict:
        return {
            "name": "stand",
            "type": "actuator",
            "description": "Bumi 站立/使能状态机 — enable(使能) / disable(失能) / ready(准备)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["enable", "disable", "ready", "get_mode"],
                        "default": "get_mode",
                        "description": "enable=使能(workmode需=30), disable=失能(workmode需!=30), ready=准备模式",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "enable": {"params": [], "description": "发送START使机器人使能(workmode==30时生效)"},
                    "disable": {"params": [], "description": "发送START使机器人失能(workmode!=30时生效)"},
                    "ready": {"params": [], "description": "发送SWITCH进入准备模式"},
                    "get_mode": {"params": [], "description": "查询当前 workmode"},
                },
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        from highcontrol_py import ControlCmd
        mode = self._get_mode()
        if action == "get_mode":
            return {"workmode": mode, "mode_name": _WORKMODE_NAMES.get(mode, "unknown")}
        if action == "enable":
            if mode != 30:
                return {"error": f"enable requires workmode=30 (disabled), current={mode}"}
            return self._publish_cmd(action_cmd=ControlCmd.START)
        if action == "disable":
            if mode == 30:
                return {"error": f"disable requires workmode!=30, current=30(disabled)"}
            return self._publish_cmd(action_cmd=ControlCmd.START)
        if action == "ready":
            return self._publish_cmd(action_cmd=ControlCmd.SWITCH)
        return super().dispatch(action, args)


class WalkPlugin(_ActuatorBase):
    """双足行走 — walk(x, y, z) 速度向量"""

    def get_tool(self) -> dict:
        return {
            "name": "walk",
            "type": "actuator",
            "description": "Bumi 双足行走 — 发送 WALK 模式 + (x,y,z) 速度向量",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["walk"],
                        "default": "walk",
                    },
                    "x": {
                        "type": "number", "minimum": -1, "maximum": 1, "default": 0,
                        "description": "纵向速度 [-1,1], >0 前进",
                    },
                    "y": {
                        "type": "number", "minimum": -1, "maximum": 1, "default": 0,
                        "description": "横向速度 [-1,1], >0 右移",
                    },
                    "z": {
                        "type": "number", "minimum": -1, "maximum": 1, "default": 0,
                        "description": "转向速度 [-1,1], >0 左转",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "walk": {"params": ["x", "y", "z"],
                             "description": "发送 WALK + 速度向量，仅在 WALK 模式下生效"},
                },
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        from highcontrol_py import ControlCmd
        if action == "walk":
            x = _clamp(args.get("x", 0), -1, 1)
            y = _clamp(args.get("y", 0), -1, 1)
            z = _clamp(args.get("z", 0), -1, 1)
            result = self._publish_cmd(x, y, z, ControlCmd.WALK)
            result.update({"x": x, "y": y, "z": z})
            return result
        return super().dispatch(action, args)


class ArmGesturePlugin(_ActuatorBase):
    """预设上半身动作 — wave/shake_hands/cheer/wipe_tear"""

    _GESTURES = {
        "wave": "SWING",
        "shake_hands": "SHAKE",
        "cheer": "CHEER",
        "wipe_tear": "TEAR",
    }

    def get_tool(self) -> dict:
        return {
            "name": "arm_gesture",
            "type": "actuator",
            "description": "Bumi 预设上半身动作 — 挥手/握手/欢呼/擦眼泪",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["wave", "shake_hands", "cheer", "wipe_tear", "cancel"],
                        "default": "wave",
                        "description": "预设动作 (边沿触发, 只发一次)",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "wave": {"params": [], "description": "挥手 (SWING)"},
                    "shake_hands": {"params": [], "description": "握手 (SHAKE)"},
                    "cheer": {"params": [], "description": "欢呼 (CHEER)"},
                    "wipe_tear": {"params": [], "description": "擦眼泪 (TEAR)"},
                    "cancel": {"params": [], "description": "发送 DEFAULT 空指令"},
                },
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        from highcontrol_py import ControlCmd
        if action == "cancel":
            return self._publish_cmd(action_cmd=ControlCmd.DEFAULT)
        gesture_map = {
            "wave": ControlCmd.SWING,
            "shake_hands": ControlCmd.SHAKE,
            "cheer": ControlCmd.CHEER,
            "wipe_tear": ControlCmd.TEAR,
        }
        cmd = gesture_map.get(action)
        if cmd is None:
            return super().dispatch(action, args)
        return self._publish_cmd(action_cmd=cmd)


class DancePlugin(_ActuatorBase):
    """舞蹈模式 — dance1/dance2/dance3"""

    def get_tool(self) -> dict:
        return {
            "name": "dance",
            "type": "actuator",
            "description": "Bumi 舞蹈模式 — dance1/dance2/dance3 (边沿触发)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["dance1", "dance2", "dance3", "cancel"],
                        "default": "dance1",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "dance1": {"params": [], "description": "舞蹈1 (DANCE)"},
                    "dance2": {"params": [], "description": "舞蹈2 (DANCE1)"},
                    "dance3": {"params": [], "description": "舞蹈3 (DANCE2)"},
                    "cancel": {"params": [], "description": "发送 DEFAULT"},
                },
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        from highcontrol_py import ControlCmd
        if action == "cancel":
            return self._publish_cmd(action_cmd=ControlCmd.DEFAULT)
        dance_map = {
            "dance1": ControlCmd.DANCE,
            "dance2": ControlCmd.DANCE1,
            "dance3": ControlCmd.DANCE2,
        }
        cmd = dance_map.get(action)
        if cmd is None:
            return super().dispatch(action, args)
        return self._publish_cmd(action_cmd=cmd)


class TeachPlugin(_ActuatorBase):
    """示教录制与回放 — start_record/save/play (ENDTEACH 已弃用, SAVETEACH 自动结束录制)"""

    def get_tool(self) -> dict:
        return {
            "name": "teach",
            "type": "actuator",
            "description": "Bumi 示教录制与回放 — start_record → save(自动结束) → play",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start_record", "save", "play", "cancel"],
                        "default": "play",
                    },
                    "index": {
                        "type": "integer", "minimum": 1, "maximum": 10, "default": 1,
                        "description": "示教文件索引 (save/play 时使用)",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start_record": {"params": [], "description": "开始录制 (STARTTEACH)"},
                    "save": {"params": ["index"], "description": "结束录制并保存 (SAVETEACH), 需 sleep(2s)"},
                    "play": {"params": ["index"], "description": "播放示教 (PLAYTEACH, 边沿触发, 只发一次)"},
                    "cancel": {"params": [], "description": "发送 DEFAULT"},
                },
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        from highcontrol_py import ControlCmd
        index = int(args.get("index", 1))
        if action == "start_record":
            return self._publish_cmd(action_cmd=ControlCmd.STARTTEACH)
        if action == "save":
            result = self._publish_cmd(action_cmd=ControlCmd.SAVETEACH, index=index)
            time.sleep(2)  # 等待保存完成，防止被下条命令打断
            return result
        if action == "play":
            return self._publish_cmd(action_cmd=ControlCmd.PLAYTEACH, index=index)
        if action == "cancel":
            return self._publish_cmd(action_cmd=ControlCmd.DEFAULT)
        return super().dispatch(action, args)


class FallRecoveryPlugin(_ActuatorBase):
    """倒地起身 — fall_to_stand/stand_to_fall"""

    def get_tool(self) -> dict:
        return {
            "name": "fall_recovery",
            "type": "actuator",
            "description": "Bumi 倒地起身 — fall_to_stand(倒地→站立) / stand_to_fall(站立→倒地)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["fall_to_stand", "stand_to_fall"],
                        "default": "fall_to_stand",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "fall_to_stand": {"params": [], "description": "从倒地状态站立 (FALLTOSTAND)"},
                    "stand_to_fall": {"params": [], "description": "从站立状态倒地 (STANDTOFALL, 测试用)"},
                },
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        from highcontrol_py import ControlCmd
        if action == "fall_to_stand":
            return self._publish_cmd(action_cmd=ControlCmd.FALLTOSTAND)
        if action == "stand_to_fall":
            return self._publish_cmd(action_cmd=ControlCmd.STANDTOFALL)
        return super().dispatch(action, args)


# ══════════════════════════════════════════════════════════════════════════════
# RLPolicyPlugin (actuator, optional)
# ══════════════════════════════════════════════════════════════════════════════

class RLPolicyPlugin(_ActuatorBase):
    """RL 策略部署 — LowController + ONNX 模型 (sim2real)"""

    def __init__(self, plugin_config, namespace, sdk):
        super().__init__(plugin_config, namespace, sdk)
        self._model_path = plugin_config.get("model_path", "policy/policy.onnx")
        self._config_path = plugin_config.get("config_path", "config/bumi_ac.yaml")
        self._policy = None

    def get_tool(self) -> dict:
        return {
            "name": "rl_policy",
            "type": "actuator",
            "description": "Bumi RL 策略部署 — 加载 ONNX 模型, 500Hz 底层电机控制 (sim2real)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["load_model", "init", "load_config", "update_state",
                                 "compute_obs", "compute_action", "set_params", "send_cmd"],
                        "default": "load_model",
                    },
                    "x": {"type": "number", "description": "纵向速度 [-1,1]"},
                    "y": {"type": "number", "description": "横向速度 [-1,1]"},
                    "yaw": {"type": "number", "description": "转向速度 [-1,1]"},
                    "is_first": {"type": "boolean", "description": "是否首次设置"},
                },
                "required": ["action"],
            },
        }

    def start(self):
        self._running = True
        # LowController 需要单独初始化
        self._sdk.init_low_controller()

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "stop"):
            return super().dispatch(action, args)
        low = self._sdk.low
        if low is None:
            return {"error": "LowController not initialized"}
        # TODO: implement RL policy dispatch
        return {"error": f"action {action} not yet implemented"}


# ══════════════════════════════════════════════════════════════════════════════
# Voice Plugins (TTS / Voice Play / Chat / Voice Chat)
# ══════════════════════════════════════════════════════════════════════════════

class TtsPlugin:
    """语音合成 — 通过 MediaController 外部音频推流到扬声器实现 TTS"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False

    def get_tool(self) -> dict:
        return {
            "name": "tts",
            "type": "actuator",
            "description": "Bumi 语音合成 (TTS) — 文字转语音播放 (通过 MediaController 外部音频推流)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["speak", "interrupt"],
                        "default": "speak",
                        "description": "控制动作",
                    },
                    "text": {"type": "string", "description": "要播放的文本"},
                    "force": {"type": "boolean", "description": "是否强制播放(打断当前播放)", "default": False},
                },
                "required": ["action"],
                "x-completion": {"actions": ["speak"], "timeout": 180},
                "x-action-params": {
                    "speak": {"params": ["text", "force"], "description": "合成并播放文本"},
                    "interrupt": {"params": [], "description": "中止播放"},
                },
            },
        }

    def start(self):
        self._running = True

    def stop(self):
        self._running = False

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "speak":
            text = args.get("text", "")
            if not text:
                return {"error": "text is required"}
            # TODO: 调用本地 TTS 引擎合成 PCM, 通过 MediaController.publish_external_audio_playback_stream 推流
            return {"state": "ok", "text": text, "note": "TTS engine integration TODO"}
        if action == "interrupt":
            # TODO: 通过 MediaController.pause_audio_playback 中断
            return {"state": "interrupted"}
        return {"error": f"unknown action: {action}"}


class VoicePlayPlugin:
    """音频播放 — 播放指定音频文件或 URL"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False

    def get_tool(self) -> dict:
        return {
            "name": "voice_play",
            "type": "actuator",
            "description": "Bumi 音频播放 — 播放指定音频文件或 URL (通过 MediaController 外部音频推流)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["play", "stop"],
                        "default": "play",
                    },
                    "url": {"type": "string", "description": "音频文件路径或 URL"},
                },
                "required": ["action"],
                "x-completion": {"actions": ["play"], "timeout": 180},
                "x-action-params": {
                    "play": {"params": ["url"], "description": "播放音频文件/URL"},
                    "stop": {"params": [], "description": "停止播放"},
                },
            },
        }

    def start(self):
        self._running = True

    def stop(self):
        self._running = False

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "play":
            url = args.get("url", "")
            if not url:
                return {"error": "url is required"}
            # TODO: 读取音频文件, 转为 PCM 16kHz, 通过 MediaController.publish_external_audio_playback_stream 分片推送
            return {"state": "playing", "url": url, "note": "audio file decoding TODO"}
        if action == "stop":
            # TODO: 通过 MediaController.pause_audio_playback 停止
            return {"state": "stopped"}
        return {"error": f"unknown action: {action}"}


class ChatPlugin:
    """语音对话开关 — 通过 MediaController 唤醒/休眠控制"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False

    def get_tool(self) -> dict:
        return {
            "name": "chat",
            "type": "actuator",
            "description": "Bumi 语音对话开关 — chat_start/chat_stop 控制语音对话模式 (通过 MediaController 唤醒/休眠)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["chat_start", "chat_stop"],
                        "default": "chat_start",
                    },
                },
                "required": ["action"],
            },
        }

    def start(self):
        self._running = True

    def stop(self):
        self._running = False

    def dispatch(self, action: str, args: dict) -> dict:
        # Handle lifecycle start/stop (different from chat start/stop)
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        # Chat start/stop actions (mapped to wakeup/sleep)
        media = self._sdk.media
        if media is None:
            return {"error": "MediaController not initialized"}
        try:
            if action == "chat_start":
                media.wakeup()
                return {"state": "chat_started"}
            if action == "chat_stop":
                media.sleep()
                return {"state": "chat_stopped"}
            return {"error": f"unknown action: {action}"}
        except Exception as e:
            return {"error": str(e)}


class VoiceChatPlugin(ChatPlugin):
    """语音对话开关 — ChatPlugin 的别名 (不同 card name, 相同逻辑)"""

    def get_tool(self) -> dict:
        tool = super().get_tool()
        tool["name"] = "voice_chat"
        tool["description"] = "Bumi 语音对话开关 — chat_start/chat_stop 语音对话 (voice_chat, 同 chat)"
        return tool


# ══════════════════════════════════════════════════════════════════════════════
# AsrPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class AsrPlugin:
    """语音识别 — 从麦克风音频流提取文字 (需配合 microphone 传感器)"""

    def __init__(self, plugin_config: dict, namespace: str, sdk):
        self._ns = namespace
        self._sdk = sdk
        self._running = False
        self._topic = f"/{namespace}/asr/text"
        self._latest = {"text": "", "timestamp_ms": 0}

    def get_tool(self) -> dict:
        return {
            "name": "asr",
            "type": "sensor",
            "description": "Bumi 语音识别 — 实时语音转文字 (麦克风阵列 → ASR 引擎)",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        # TODO: 启动 ASR 引擎, 订阅 microphone 音频流, 识别结果写入 self._latest
        print("[AsrPlugin] started (ASR engine integration TODO)")

    def stop(self):
        self._running = False

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("read", "get", "asr"):
            return dict(self._latest)
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return {"state": "running" if self._running else "idle"}
