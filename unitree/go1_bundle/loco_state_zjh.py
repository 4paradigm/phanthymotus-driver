"""
loco_state_zjh.py — Go1 全传感器汇聚卡（运动状态 + IMU + 关节 + 电量 + 障碍 + 遥控）。

一张卡汇集所有可读的 HighState 数据：
  - 运动模式/步态/里程计/速度/机身高度
  - IMU: 欧拉角(roll/pitch/yaw) + 角速度
  - 12 个关节角度(FR/FL/RR/RL 各 3 个)
  - 足端力(4 脚)
  - 电池: SOC% + 状态
  - 前向障碍: 4 通道 range_obstacle
  - 遥控: 按键状态 + 模拟摇杆

数据来源：共享的只读 client 的 snapshot()（见 go1_sdk_client.py，字段来自 HighState）。
"""

from __future__ import annotations

import json
import time

try:
    from go1_sdk_client import parse_wireless_remote
except Exception:
    def parse_wireless_remote(_raw):
        return {"buttons": {}, "axes": {}}

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "loco_state_zjh"
TYPE = "sensor"
TOPIC = "/{ns}/loco/state_zjh"
FMT = "data/json"
HZ = 10.0
NODE = "go1_loco_state_zjh"
DESC = "Go1 全传感器汇聚：运动+IMU+关节+足端力+电量+障碍+遥控"


def build(snap: dict) -> dict:
    """snapshot -> 本卡对外的 dict。公共头带 timestamp/control_level/fresh，无新包不伪造。"""
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}

    try:
        # ── 运动状态 ────
        vel = snap.get("velocity") or [0.0, 0.0, 0.0]
        d["locomotion"] = {
            "mode": snap.get("mode", 0),
            "mode_name": snap.get("mode_name", "unknown"),
            "gait_type": snap.get("gait_type", 0),
            "gait_name": snap.get("gait_name", "unknown"),
            "position_m": snap.get("position"),
            "body_height_m": snap.get("body_height", 0.0),
            "foot_raise_height_m": snap.get("foot_raise_height", 0.0),
            "velocity_body_mps": {
                "forward": float(vel[0]) if len(vel) > 0 else 0.0,
                "lateral": float(vel[1]) if len(vel) > 1 else 0.0
            },
            "yaw_speed_rad_s": snap.get("yaw_speed", 0.0),
        }

        # ── IMU ────
        imu = snap.get("imu") or {}
        rpy = imu.get("rpy_rad") or [0.0, 0.0, 0.0]
        gyro = imu.get("gyroscope_rad_s") or [0.0, 0.0, 0.0]
        d["imu"] = {
            "euler_rad": {
                "roll": float(rpy[0]) if len(rpy) > 0 else 0.0,
                "pitch": float(rpy[1]) if len(rpy) > 1 else 0.0,
                "yaw": float(rpy[2]) if len(rpy) > 2 else 0.0
            },
            "gyroscope_rad_s": {
                "roll": float(gyro[0]) if len(gyro) > 0 else 0.0,
                "pitch": float(gyro[1]) if len(gyro) > 1 else 0.0,
                "yaw": float(gyro[2]) if len(gyro) > 2 else 0.0
            },
        }

        # ── 关节(12 个) ────
        # snapshot["joints"] 是 list[{"i","q",...}]（见 go1_sdk_client.parse_joints），取每个的 q 作为角度
        joints_list = snap.get("joints") or []
        joint_names = ["FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
                       "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"]
        d["joints"] = {}
        for i, name in enumerate(joint_names):
            m = joints_list[i] if i < len(joints_list) else None
            d["joints"][name] = float(m.get("q", 0.0)) if isinstance(m, dict) else 0.0

        # ── 足端力(FR/FL/RR/RL) ────
        foot_force = snap.get("foot_force") or [0.0, 0.0, 0.0, 0.0]
        d["foot_force_raw"] = {foot: float(foot_force[i]) if i < len(foot_force) else 0.0
                               for i, foot in enumerate(["FR", "FL", "RR", "RL"])}

        # ── 电池 ────
        battery = snap.get("battery") or {}
        d["battery"] = {
            "soc_percent": battery.get("soc_percent"),
            "status": battery.get("status"),
        }

        # ── 前向障碍(4 通道) ────
        range_obstacle = snap.get("range_obstacle") or [0.0, 0.0, 0.0, 0.0]
        d["range_obstacle_m"] = {f"ch{i}": float(range_obstacle[i]) if i < len(range_obstacle) else 0.0
                                 for i in range(4)}

        # ── 遥控 ────
        # snapshot["wireless_remote"] 是 40 字节原始 bytes（见 go1_sdk_client._to_bytes40），需解析成 buttons/axes
        wr_raw = snap.get("wireless_remote")
        wr = parse_wireless_remote(wr_raw) if wr_raw else {"buttons": {}, "axes": {}}
        d["wireless_remote"] = {
            "buttons": wr.get("buttons", {}),
            "axes": wr.get("axes", {}),
        }

    except Exception as e:
        print(f"[{CARD}] build() error: {e}", flush=True)

    return d


class Plugin:
    """状态卡插件：装了 rclpy 就发 topic；始终支持 MCP action=info 轮询。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 state → {self._topic} @ {HZ}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(build(self._client.snapshot()))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            if self._node:
                self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = DESC + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", CARD):
            return {"state": "running", "data": build(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
