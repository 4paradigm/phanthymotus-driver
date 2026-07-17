"""
contact_check.py — Go1 触地/悬空自检卡（足底力判定四脚接触状态 + 被抱起检测）。

自包含：一张卡 = 一个文件（builder + MCP 插件 + 可选 ROS2 发布）。main.py 会根据
config.yaml 里的卡名自动 import 本模块并调用 make_plugin()。

与队友 feet 卡的区别（不重复造）：feet 卡只上报**原始足底力/足端位置速度**，不做任何判定；
本卡在 snapshot()["foot_force"]（HighState.footForce，顺序 FR/FL/RR/RL）之上做**接触判定层**：
  · 每脚是否触地（force ≥ 阈值）
  · 支撑姿态分类：stand(4) / tripod(3) / two_feet(2) / one_foot(1) / airborne(0)
  · 被抱起检测：四脚同时悬空并**持续** picked_up_hold_s 秒 → picked_up=true
    （持续判定避开行走腾空瞬间；正常 trot 总有对角两脚触地，不会四脚同时空）

只读、不下发、零运动风险。验收极简：手抬一只脚看该脚 contact 翻 false；抱起整只狗看 picked_up=true。

⚠ footForce 是**未标定原始整数**（无单位），站立每脚约几十~两三百、悬空接近 0。阈值 contact_force_threshold
可配，真机首次用 info 看站立读数标定一次即可。HIGHLEVEL 下 footForce 有效（区别于常为零的 motorState）。
"""

from __future__ import annotations

import json
import threading
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

try:
    from go1_sdk_client import FOOT_ORDER
except Exception:
    FOOT_ORDER = ["FR", "FL", "RR", "RL"]

# ── 卡片元数据 ──────────────────────────────────────────────────────────────
CARD = "test_contact_check_zjh"         # 卡名 = MCP 工具名 = config.yaml 里的 key = 本文件名
TYPE = "sensor"
TOPIC = "/{ns}/test/state/contact"
FMT = "data/json"
NODE = "go1_test_contact_check_zjh"
DESC = ("Go1 触地/悬空自检：足底力(FR/FL/RR/RL)判定每脚是否触地 + 支撑姿态"
        "(stand/tripod/two_feet/one_foot/airborne) + 被抱起检测(四脚持续悬空)。只读。")

# 姿态名：按触地脚数分类
_POSTURE = {4: "stand", 3: "tripod", 2: "two_feet", 1: "one_foot", 0: "airborne"}


class Plugin:
    """只读状态卡：后台线程按 publish_hz 维护 picked_up 计时；装了 rclpy 就发 topic，
    始终支持 MCP action=info/read 轮询。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._ns = namespace
        self._topic = TOPIC.format(ns=namespace)
        cfg = plugin_config or {}
        # 触地力阈值（原始 footForce 整数，真机可标定）；被抱起需四脚悬空持续这么久
        self._threshold = float(cfg.get("contact_force_threshold", 30.0))
        self._picked_up_hold_s = float(cfg.get("picked_up_hold_s", 0.6))
        self._hz = float(cfg.get("publish_hz", 10.0))

        self._lock = threading.Lock()
        self._airborne_since = None    # monotonic 时间戳：四脚全悬空的起始（None=当前有触地）
        self._last = None              # 最近一次判定结果 dict
        self._running = False
        self._thread = None

        self._node = None
        self._pub = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(f"{NODE}_{namespace}")
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.get_logger().info(f"go1 state → {self._topic} @ {self._hz}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None
                self._pub = None

    # ── 判定核心 ─────────────────────────────────────────────────────────
    def _compute(self, snap):
        """snapshot → (data 不含 picked_up, contacts, airborne_bool)。无新包不伪造。"""
        d = {"timestamp_ms": int(time.time() * 1000),
             "control_level": snap.get("control_level", "HIGHLEVEL"),
             "fresh": bool(snap.get("fresh", False)),
             "threshold": self._threshold}
        force = snap.get("foot_force")
        if force is None:
            d["available"] = False
            d["reason"] = "footForce 不可用（无新鲜反馈）"
            return d, None, None
        feet = []
        contacts = 0
        for i, name in enumerate(FOOT_ORDER):
            f = force[i] if i < len(force) else None
            contact = (f is not None and f >= self._threshold)
            if contact:
                contacts += 1
            feet.append({"foot": name, "force": f, "contact": contact})
        d["order"] = FOOT_ORDER
        d["feet"] = feet
        d["contacts"] = contacts
        d["posture"] = _POSTURE.get(contacts, "unknown")
        return d, contacts, (contacts == 0)

    def _evaluate(self, now):
        """读一帧、算判定、维护 picked_up 计时；返回完整 data（含 picked_up）。"""
        snap = self._client.snapshot()
        d, contacts, airborne = self._compute(snap)
        with self._lock:
            if not d.get("fresh") or airborne is None:
                # 无有效反馈：不推进被抱起计时，避免误判
                self._airborne_since = None
                picked_up = False
            elif airborne:
                if self._airborne_since is None:
                    self._airborne_since = now
                picked_up = (now - self._airborne_since) >= self._picked_up_hold_s
            else:
                self._airborne_since = None
                picked_up = False
            d["picked_up"] = bool(picked_up)
            self._last = d
        return d

    # ── 后台线程 ─────────────────────────────────────────────────────────
    def _loop(self):
        period = 1.0 / self._hz if self._hz > 0 else 0.1
        while self._running:
            try:
                d = self._evaluate(time.monotonic())
                if self._pub is not None:
                    m = String()
                    m.data = json.dumps(d)
                    self._pub.publish(m)
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] tick error: {e}", flush=True)
            time.sleep(period)

    def start(self):
        self._evaluate(time.monotonic())   # 先算一帧，保证 info 立即有值
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name=f"{CARD}_{self._ns}", daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        t = self._thread
        if t is not None:
            t.join(timeout=1.0)

    # ── MCP 契约 ─────────────────────────────────────────────────────────
    def get_tool(self):
        desc = DESC + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": desc,
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": ["info", "read"],
                                   "description": "info/read 查当前触地判定；不传默认 info"},
                    }},
                "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", CARD):
            d = self._evaluate(time.monotonic())
            return {"ok": True, "state": "running", "data": d,
                    "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}
        return None   # 未知 action → 契约：MCP 报 Unknown


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
