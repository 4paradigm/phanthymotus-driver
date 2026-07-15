"""
feet.py — Go1 足端状态卡（足底力 + 高层时的足端相对机身位置/速度）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
数据来源：共享只读 client 的 snapshot()（foot_force / foot_pos / foot_speed，来自 HighState）。
照 loco_state.py 骨架改写，详见 CONTRIBUTING.md。
"""

from __future__ import annotations

import json
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
CARD = "feet"                          # 卡名 = MCP 工具名 = config.yaml key = 本文件名
TYPE = "sensor"
TOPIC = "/{ns}/state/feet"
FMT = "data/json"
HZ = 10.0
NODE = "go1_feet"                      # ROS2 node 名（须全局唯一）
DESC = "Go1 feet — foot force raw[4]; foot position/speed to body (HIGHLEVEL only)"

FOOT_ORDER = ["FR", "FL", "RR", "RL"]  # 与 go1_sdk_client 一致


def build(snap: dict) -> dict:
    """snapshot -> 足端 dict。公共头带 timestamp/control_level/fresh，无新包不伪造。"""
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    d["order"] = FOOT_ORDER
    d["foot_force_raw"] = snap.get("foot_force")
    # 足端相对机身位置/速度仅高层提供
    fp = snap.get("foot_pos")
    if fp is not None:
        d["position_to_body"] = fp
        d["speed_to_body"] = snap.get("foot_speed")
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
                self._node.get_logger().info(f"go1 feet → {self._topic} @ {HZ}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(build(self._client.snapshot()))
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
            return {"state": "running", "data": build(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
