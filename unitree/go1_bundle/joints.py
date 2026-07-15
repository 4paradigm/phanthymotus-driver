"""
joints.py — Go1 12 腿关节状态卡（骨架渲染）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
数据来源：共享只读 client 的 snapshot()["joints"]（来自 HighState.motorState 前 12 个）+
snapshot()["imu"]（机身姿态）。照 imu.py 骨架改写，详见 CONTRIBUTING.md。

【骨架渲染要点，实机验证过】dashboard 的 skeleton 渲染器：
  · 按每个关节的 `name` 匹配 model 卡 URDF 里的 joint（名字须一致，用官方 <腿>_hip/thigh/calf_joint）；
  · 用 `q`（弧度，注意字段名就是 q，不是 q_rad）设定关节角；
  · 用顶层 `imu_quat` 摆正机身朝向。
需配套 model 卡（返回 go1 URDF）才能渲染成四足狗，否则退回默认骨架。
"""

from __future__ import annotations

import json
import time

from go1_sdk_client import JOINT_NAMES   # 官方 <腿>_hip/thigh/calf_joint（与 URDF 一致）

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
CARD = "joints"
TYPE = "sensor"
TOPIC = "/{ns}/state/joints"
FMT = "sensor/skeleton"
HZ = 10.0
NODE = "go1_joints"
DESC = "Go1 12 leg-joint states (q/dq/tau/temp) as skeleton; needs model card (URDF) to render as quadruped"


def build(snap: dict) -> dict:
    """snapshot -> joints dict。公共头带 timestamp/control_level/fresh，无新包不伪造。"""
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False)),
         "format": "sensor/skeleton"}
    js = snap.get("joints")
    if not js:
        d["available"] = False
        return d
    joints = []
    for j in js:
        i = int(j.get("i", 0))
        name = JOINT_NAMES[i] if 0 <= i < len(JOINT_NAMES) else f"joint_{i}"
        q = j.get("q", 0.0)
        joints.append({
            "idx": i, "name": name,
            "q": q, "q_rad": q,           # 渲染器读 `q`（弧度）；q_rad 保留为可读别名
            "dq": j.get("dq", 0.0),
            "tau": j.get("tau", 0.0),
            "ddq_rad_s2": j.get("ddq", 0.0),
            "mode": j.get("mode", 0),
            "temperature_c": j.get("temp", 0),
        })
    d["available"] = True
    d["joints"] = joints
    # 机身朝向：供渲染器摆正整体骨架
    imu = snap.get("imu") or {}
    d["imu_quat"] = imu.get("quaternion_wxyz", [1.0, 0.0, 0.0, 0.0])
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
                self._node.get_logger().info(f"go1 joints → {self._topic} @ {HZ}Hz")
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
