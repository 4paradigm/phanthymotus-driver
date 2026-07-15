"""
net.py — Go1 网络健康卡（主机名 / IPv4 / Wi-Fi 信号强度）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
纯系统读（不经硬件 HighState）→ 数据始终可用、始终 fresh；联调无线掉线时排查用。
照 loco_state.py 骨架改写，详见 CONTRIBUTING.md。
"""

from __future__ import annotations

import json
import socket
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
CARD = "net"
TYPE = "sensor"
TOPIC = "/{ns}/state/net"
FMT = "data/json"
HZ = 0.5
NODE = "go1_net"
DESC = "Go1 network health — hostname/IPv4/Wi-Fi signal (link-loss triage during bring-up)"


def _primary_ipv4():
    """选默认出口网卡的 IPv4（connect 不真发包，只用于查路由）。"""
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:  # noqa: BLE001
        return None
    finally:
        if s is not None:
            try:
                s.close()
            except Exception:  # noqa: BLE001
                pass


def _wifi_stats():
    """Linux（狗）读 /proc/net/wireless；Mac/无该文件 → available:false（诚实标注）。"""
    try:
        with open("/proc/net/wireless") as f:
            lines = f.read().splitlines()
    except Exception:  # noqa: BLE001
        return {"available": False, "reason": "无 /proc/net/wireless（非 Linux 或无无线网卡）"}
    for ln in lines[2:]:
        parts = ln.split()
        if len(parts) >= 4 and parts[0].endswith(":"):
            iface = parts[0].rstrip(":")
            try:
                link = float(parts[2].rstrip("."))
                level = float(parts[3].rstrip("."))
            except (ValueError, IndexError):
                link, level = None, None
            return {"available": True, "iface": iface,
                    "link_quality": link, "signal_dbm": level}
    return {"available": False, "reason": "无活动无线网卡"}


def build(snap: dict) -> dict:
    """系统级网络快照。数据不经 HighState → fresh 恒 True（自身即最新）。"""
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": True, "available": True}
    try:
        d["hostname"] = socket.gethostname()
    except Exception:  # noqa: BLE001
        d["hostname"] = None
    d["ipv4"] = _primary_ipv4()
    d["wifi"] = _wifi_stats()
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
                self._node.get_logger().info(f"go1 net → {self._topic} @ {HZ}Hz")
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
