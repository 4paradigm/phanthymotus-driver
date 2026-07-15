"""
loco_state.py — Go1 运动状态卡（mode / gait / 里程计 / 机体速度 / 机身高度）。

自包含：一张卡 = 一个文件（builder + MCP 插件 + 可选 ROS2 发布）。main.py 会根据
config.yaml 里的卡名自动 import 本模块并调用 make_plugin()。加新卡就照本文件复制一份改写，
详见 CONTRIBUTING.md。

数据来源：共享的只读 client 的 snapshot()（见 go1_sdk_client.py，字段来自 HighState）。
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
CARD = "loco_state"                    # 卡名 = MCP 工具名 = config.yaml 里的 key = 本文件名
TYPE = "sensor"
TOPIC = "/{ns}/loco/state"             # ROS2 topic（{ns} 由 namespace 填充）
FMT = "data/json"                      # 画布渲染格式
HZ = 10.0                              # topic 发布频率
NODE = "go1_loco_state"                # ROS2 node 名（须全局唯一）
DESC = "Go1 locomotion state — mode/gait/odometry/velocity/body_height (HighState)"


def build(snap: dict) -> dict:
    """snapshot -> 本卡对外的 dict。公共头带 timestamp/control_level/fresh，无新包不伪造。"""
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    vel = snap.get("velocity") or [0.0, 0.0, 0.0]
    d.update({
        "mode": snap.get("mode", 0), "mode_name": snap.get("mode_name", "unknown"),
        "gait_type": snap.get("gait_type", 0), "gait_name": snap.get("gait_name", "unknown"),
        "foot_raise_height_m": snap.get("foot_raise_height", 0.0),
        "position_m": snap.get("position"), "body_height_m": snap.get("body_height", 0.0),
        "velocity_body_mps": {"forward": vel[0] if len(vel) > 0 else 0.0,
                              "lateral": vel[1] if len(vel) > 1 else 0.0},
        "velocity_index_2_raw": vel[2] if len(vel) > 2 else 0.0,
        "yaw_speed_rad_s": snap.get("yaw_speed", 0.0),
    })
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
