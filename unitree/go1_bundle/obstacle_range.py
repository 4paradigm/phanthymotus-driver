"""
obstacle_range.py — Go1 超声波避障状态卡（最近障碍物距离，只读）。

自包含：一张卡 = 一个文件（builder + MCP 插件 + 可选 ROS2 发布）。main.py 会根据
config.yaml 里的卡名自动 import 本模块并调用 make_plugin()。加新卡就照本文件复制一份改写，
详见 CONTRIBUTING.md。

数据来源：共享的只读 client 的 snapshot()（见 go1_sdk_client.py，字段来自 HighState.rangeObstacle）。
本卡只取 snapshot["range_obstacle"]（4 路原始距离）。

⚠️ 仅 HIGHLEVEL 可用（LowState 无 rangeObstacle）；宇树官方未公开各路方向与量纲，故原样输出、
   标注 direction_mapping/unit 为 "undocumented"，不做换算、不编造。
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

# ── 卡片元数据（改这几项即可派生一张新卡）────────────────────────────────────
CARD = "obstacle_range"                    # 卡名 = MCP 工具名 = config.yaml 里的 key = 本文件名
TYPE = "sensor"
TOPIC = "/{ns}/state/obstacle_range"       # ROS2 topic（{ns} 由 namespace 填充）
FMT = "data/json"                          # 画布渲染格式
HZ = 10.0                                  # topic 发布频率
NODE = "go1_obstacle"                      # ROS2 node 名（须全局唯一）
DESC = "Go1 nearest-obstacle raw ranges[4] (direction/unit undocumented; read-only)"


def build(snap: dict) -> dict:
    """snapshot -> 本卡对外的 dict。公共头带 timestamp/control_level/fresh，无新包不伪造。"""
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    ro = snap.get("range_obstacle")            # HighState.rangeObstacle[4]，可能为 None
    d.update({"available": ro is not None, "range_raw": ro,
              "direction_mapping": "undocumented", "unit": "undocumented"})
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
