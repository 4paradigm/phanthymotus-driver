"""
fall_alarm.py — Go1 跌倒/侧翻告警卡（由 IMU 的 roll/pitch 幅度判定 ok/tilted/fallen）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
数据来源：共享只读 client 的 snapshot()["imu"]["rpy_rad"]。运动安全兜底。
阈值可在 config.yaml 里配：tilt_warn_rad（默认 0.6≈34°）/ fall_rad（默认 1.2≈69°）。
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
CARD = "fall_alarm"
TYPE = "sensor"
TOPIC = "/{ns}/state/fall_alarm"
FMT = "data/json"
HZ = 10.0
NODE = "go1_fall_alarm"
DESC = "Go1 fall/tilt alarm — ok/tilted/fallen from IMU roll/pitch magnitude + tilt angle"

WARN_DEFAULT = 0.6                     # rad ≈34°：明显倾斜
FALL_DEFAULT = 1.2                     # rad ≈69°：判定跌倒/翻倒


def build(snap: dict, warn: float, fall: float) -> dict:
    """snapshot + 阈值 -> 告警 dict。公共头带 timestamp/control_level/fresh。"""
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    imu = snap.get("imu") or {}
    rpy = imu.get("rpy_rad") or [0.0, 0.0, 0.0]
    roll = float(rpy[0]) if len(rpy) > 0 else 0.0
    pitch = float(rpy[1]) if len(rpy) > 1 else 0.0
    tilt = max(abs(roll), abs(pitch))
    if tilt >= fall:
        status, hint = "fallen", "已跌倒/翻倒 —— 立即停止运动，需人工扶正后再操作"
    elif tilt >= warn:
        status, hint = "tilted", "机身明显倾斜 —— 谨慎，可能即将失稳"
    else:
        status, hint = "ok", "姿态正常"
    d.update({"status": status,
              "roll_rad": round(roll, 4), "pitch_rad": round(pitch, 4),
              "roll_deg": round(math.degrees(roll), 1), "pitch_deg": round(math.degrees(pitch), 1),
              "tilt_warn_rad": warn, "fall_rad": fall, "hint": hint})
    return d


class Plugin:
    """状态卡插件：阈值取自 config；装了 rclpy 就发 topic；始终支持 MCP action=info 轮询。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        cfg = plugin_config or {}
        self._warn = float(cfg.get("tilt_warn_rad", WARN_DEFAULT))
        self._fall = float(cfg.get("fall_rad", FALL_DEFAULT))
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 fall_alarm → {self._topic} @ {HZ}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(build(self._client.snapshot(), self._warn, self._fall))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
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
            return {"state": "running", "data": build(self._client.snapshot(), self._warn, self._fall),
                    "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
