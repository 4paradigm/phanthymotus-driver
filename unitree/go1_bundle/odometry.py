"""
odometry.py — Go1 里程计卡（当前 position/yaw + 相对起点位移；reset_origin 重置起点）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
数据来源：共享只读 client 的 snapshot()["position"] 与 ["imu"]["rpy_rad"][2]（仅 HIGHLEVEL 有 position）。
让高层规划知道「在哪 / 离起点多远」。origin 默认取首帧，可用 reset_origin 动作重置。

⚠️ 不输出累计总路程 total_distance：本 bundle 的 client 不做逐帧累计（逐帧累加会把 IMU/位置
   噪声放大成虚高路程，不可靠）。相对起点的直线位移 displacement 才是可信量。
照 loco_state.py 骨架改写，详见 CONTRIBUTING.md。
"""

from __future__ import annotations

import json
import math
import time

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

# ── 卡片元数据 ────────────────────────────────────────────────────────────────
CARD = "odometry"
TYPE = "sensor"
TOPIC = "/{ns}/state/odometry"
FMT = "data/json"
HZ = 5.0
NODE = "go1_odometry"
CONTROL_LEVEL = "HIGHLEVEL"
DESC = ("Go1 odometry — position/yaw + displacement from origin; "
        "action=read 读取, action=reset_origin 重置起点。HIGHLEVEL only.")


def _ms() -> int:
    return int(time.time() * 1000)


class Plugin:
    """状态卡插件：持起点 origin；装了 rclpy 就发 topic；支持 MCP read / reset_origin。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._origin = None            # [x, y] 起点；首帧或 reset_origin 时设
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 odometry → {self._topic} @ {HZ}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _build(self) -> dict:
        """snapshot -> 里程计 dict（读取时惰性初始化 origin）。"""
        snap = self._client.snapshot()
        d = {"timestamp_ms": _ms(),
             "control_level": snap.get("control_level", "HIGHLEVEL"),
             "fresh": bool(snap.get("fresh", False))}
        pos = snap.get("position") or [0.0, 0.0, 0.0]
        imu = snap.get("imu") or {}
        rpy = imu.get("rpy_rad") or [0.0, 0.0, 0.0]
        yaw = float(rpy[2]) if len(rpy) > 2 else 0.0
        if self._origin is None:
            self._origin = [pos[0], pos[1]]
        dx = pos[0] - self._origin[0]
        dy = pos[1] - self._origin[1]
        d.update({"position_m": pos, "yaw_rad": round(yaw, 4),
                  "origin_m": list(self._origin),
                  "displacement_m": {"dx": round(dx, 3), "dy": round(dy, 3),
                                     "distance": round(math.hypot(dx, dy), 3)}})
        return d

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(self._build())
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = DESC + (f" — → {self._topic}" if self._node else " — poll via MCP action=read")
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": desc,
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": ["read", "reset_origin"],
                                   "description": "read=读里程计（默认）；reset_origin=把当前位置设为新起点"},
                    },
                    "x-action-params": {
                        "read": {"params": [], "description": "读取当前 position/yaw 与相对起点位移。"},
                        "reset_origin": {"params": [], "description": "把当前位置设为新起点，位移归零。"},
                    },
                },
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
        if action == "reset_origin":
            snap = self._client.snapshot()
            pos = snap.get("position") or [0.0, 0.0, 0.0]
            self._origin = [pos[0], pos[1]]
            return {"ok": True, "card": CARD, "action": "reset_origin",
                    "control_level": CONTROL_LEVEL,
                    "applied": {"origin_m": list(self._origin)}, "timestamp_ms": _ms()}
        if action in ("info", "read", "get", CARD):
            return {"state": "running", "data": self._build(),
                    "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
