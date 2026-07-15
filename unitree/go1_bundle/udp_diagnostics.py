"""
udp_diagnostics.py — Go1 UDP 通信健康状态卡。

自包含：一张卡 = 一个文件（builder + MCP 插件 + 可选 ROS2 发布）。main.py 会根据
config.yaml 里的卡名自动 import 本模块并调用 make_plugin()。加新卡就照本文件/battery.py
复制一份改写，详见 CONTRIBUTING.md。

数据来源：共享只读 client 的 diagnostics()（见 go1_sdk_client.py 里后台收发线程维护的
收发/错误计数）。与其它状态卡不同，本卡读 client.diagnostics() 而非 snapshot()：诊断计数
不依赖是否收到 HighState，STUB / 无真机时计数恒为 0、accessible=false。
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
CARD = "udp_diagnostics"               # 卡名 = MCP 工具名 = config.yaml 里的 key = 本文件名
TYPE = "sensor"
TOPIC = "/{ns}/state/udp_diagnostics"  # ROS2 topic（{ns} 由 namespace 填充）
FMT = "data/json"                      # 画布渲染格式
HZ = 1.0                               # topic 发布频率
NODE = "go1_udp_diagnostics"           # ROS2 node 名（须全局唯一）
DESC = "Go1 UDP link health — send/recv counts and CRC/lose/flag error counters"


def build(client) -> dict:
    """client -> 本卡对外的 dict。公共头带 timestamp/control_level/fresh，无新包不伪造。

    本卡取 client.diagnostics()（收发计数），control_level/fresh 沿用 snapshot() 的公共头
    以与其它状态卡保持一致；诊断计数本身即使 fresh=false 也有意义（能看出是否在收发）。
    """
    snap = client.snapshot()
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    diag = client.diagnostics()
    d.update(diag)                     # total/send/recv_count, send/flag/recv_crc/recv_lose_error, accessible
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
            m.data = json.dumps(build(self._client))
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
            return {"state": "running", "data": build(self._client),
                    "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
