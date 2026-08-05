#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
engineai/t800/plugins/motion.py — 众擎（EngineAI）T800 运动相关能力插件（5 张卡片）。

数据流（双 context 转发，参照 tianyi2.0 双域模式）:
  domain 69 (ctx_t800, CycloneDDS)                domain 42 (ctx_core, FastDDS)
  /motion/motion_state  ──────────────────────► /{ns}/motion/state   (2Hz, data/json)
  /motion/set_motion_state     ◄── motion_switcher 发布 (RELIABLE depth1)
  /motion/body_vel_cmd         ◄── loco 100Hz 持续发布 (depth10)
  /motion/joint_motion_plan/request ◄── arm 发布 (RELIABLE depth1)
  /motion/joint_motion_plan/state   ────────► /{ns}/arm/state       (5Hz, data/json)
  /hardware/joint_state        ──► 缓存（joint_bridge 插值起点）
  /hardware/joint_command      ◄── joint_bridge 500Hz 发布 (BEST_EFFORT depth3)

插件列表:
  MotionStatePlugin    (sensor)   — 运动状态机订阅缓存 + 2Hz 转发 core 域 + get 查询
  MotionSwitcherPlugin (actuator) — 状态切换（白名单校验 + 3 秒超时确认）
  LocoPlugin           (actuator) — 机体速度控制（100Hz / 20% 渐变 / duration 自停 / 2s 防御自停）
  ArmMotionPlugin      (actuator) — 上肢运动规划（官方预设手势队列，request_id 递增）
  JointBridgePlugin    (actuator) — 关节透传 500Hz 全身 PD 控制（config 默认关闭，危险）

设计约束（README_dev / 设计文档 / 官方代码核实）:
  - 模块级只 import 标准库；rclpy / interface_protocol / std_msgs 全部延迟导入，
    无 ROS2 环境时进入 stub 模式（可纯 import 测试，参照 tianyi2.0 MotorStatePlugin.start()）
  - dispatch 返回纯 dict（绝不返回 MCP content 数组），必须处理 start/stop/info 系统动作
  - 控制类插件缓存最新 motion_state 做前置检查，不在所需模式时返回
    {"state":"error","error":"WRONG_MOTION_STATE","message":...}（中文 message）
  - 本批传感器卡 topic_out format 一律 "data/json"
"""

from __future__ import annotations

import json
import queue
import threading
import time

# ── 常量 ──────────────────────────────────────────────────────────────────────

# T800 开发版 25 自由度，语义名 J00_HIP_PITCH_L ... J24_HEAD_YAW
JOINT_COUNT = 25

# 上肢运动规划关节索引：腰(12) + 左臂(13-17) + 右臂(18-22) + 头(23-24)，共 13 关节
UPPER_BODY_JOINT_INDICES = [12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]

# 官方运动状态机全量状态（description 中列出，供 LLM/用户参考）
MOTION_STATE_NAMES = [
    "idle",                 # 空闲
    "passive",              # 阻力模式
    "pd_stand",             # PD 站立
    "rl_basic",             # 自然行走（RL）
    "lower_body_balance",   # 下肢平衡（上肢运动规划前置）
    "joint_bridge",         # 关节透传（关节命令前置）
    "pd_sitground",         # 坐着起身（PD）
    "walk_server",          # 行走服务端
    "rl_mimic_supine_to_stance",      # 模仿-仰卧起身
    "rl_mimic_prone_to_stance",       # 模仿-俯卧起身
    "rl_mimic_stance_to_supine",      # 模仿-站立躺下（仰）
    "rl_mimic_stance_to_sitdown",     # 模仿-站立坐下
    "rl_mimic_sitdown_to_stance",     # 模仿-坐着起身
]

# 中文别名 → 官方状态名
_MODE_ALIASES = {
    "空闲": "idle",
    "阻力模式": "passive",
    "PD站立": "pd_stand",
    "自然行走": "rl_basic",
    "下肢平衡": "lower_body_balance",
    "关节透传": "joint_bridge",
    "坐着起身": "pd_sitground",
    "行走服务端": "walk_server",
}

# 官方 t800/motion_plan_shake_hand.yaml 的 stiffness/damping
#（腰 [400] / 左臂 [40,40,20,40,20] / 右臂 [40,40,20,40,20] / 头 [100,100] 展平为 13 元）
_UPPER_STIFFNESS_HAND = [400.0, 40.0, 40.0, 20.0, 40.0, 20.0, 40.0, 40.0, 20.0, 40.0, 20.0, 100.0, 100.0]
_UPPER_DAMPING_HAND = [3.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 3.0, 3.0]

# 官方 t800/motion_plan_wave_hands.yaml 的 stiffness（每个任务不同）
_UPPER_STIFFNESS_WAVE_INIT = [200.0, 30.0, 30.0, 15.0, 30.0, 15.0, 40.0, 40.0, 20.0, 40.0, 20.0, 100.0, 100.0]
_UPPER_STIFFNESS_WAVE_STEP = [500.0, 30.0, 30.0, 15.0, 30.0, 15.0, 40.0, 40.0, 20.0, 40.0, 20.0, 100.0, 100.0]
_UPPER_STIFFNESS_WAVE_END = [200.0, 10.0, 10.0, 5.0, 10.0, 5.0, 10.0, 10.0, 5.0, 10.0, 5.0, 100.0, 100.0]

# 官方上肢预设动作（target_positions 与 duration 逐字取自 yaml 文件）
_ARM_EXTEND_POSITIONS = [0.0, 0.024, 0.081, -0.001, -0.069, 0.000, -0.47, 0.255, 0.161, -0.731, 0.028, 0.000, 0.000]
_ARM_WITHDRAW_POSITIONS = [0.0, 0.024, 0.081, -0.001, -0.069, 0.000, 0.028, -0.084, 0.001, -0.066, 0.000, 0.000, 0.000]
_WAVE_INIT_POSITIONS = [0.0, 0.028, 0.084, -0.001, -0.066, 0.000, 0.024, -0.081, 0.001, -0.069, 0.000, 0.0, 0.0]
_WAVE_RAISE_POSITIONS = [0.0, -1.29568, 1.17971, 0.0757227, -1.06603, -0.0989933,
                         -0.0211716, -0.322156, 0.0440607, -0.0871668, 0.0196457, 0.0, 0.0]
_WAVE_HANDS_POSITIONS = [0.0, -1.07786, 1.13928, 0.177577, -1.83356, -0.0875483,
                         -0.0211716, -0.322156, 0.0440607, -0.0871668, 0.0196457, 0.0, 0.0]

# 官方 t800/pd_joint_test.yaml 的 kp/kd（腿 6+6 / 腰 1 / 臂 5+5 / 头 2，展平为 25 元）
_JB_DEFAULT_STIFFNESS = [
    200.0, 200.0, 380.0, 450.0, 400.0, 200.0,   # 左腿 0-5
    200.0, 200.0, 380.0, 450.0, 400.0, 200.0,   # 右腿 6-11
    200.0,                                       # 腰 12
    250.0, 250.0, 250.0, 250.0, 250.0,           # 左臂 13-17
    250.0, 250.0, 250.0, 250.0, 250.0,           # 右臂 18-22
    100.0, 100.0,                                # 头 23-24
]
_JB_DEFAULT_DAMPING = [
    5.0, 5.0, 5.0, 5.0, 2.0, 2.0,
    5.0, 5.0, 5.0, 5.0, 2.0, 2.0,
    1.0,
    1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0,
]

# 规划器状态码 → 名称
_PLAN_STATUS_NAMES = {0: "DISABLED", 1: "IDLE", 2: "EXECUTING", 3: "EXITING"}

# 前置检查：loco 需要的行走类模式（body_vel_cmd 适用）
_LOCO_REQUIRED_MODES = ("rl_basic", "walk_server")


def _err(error: str, message: str) -> dict:
    """构造统一错误返回（state=error + 错误码 + 中文 message）。"""
    return {"state": "error", "error": error, "message": message}


def _load_ros2_contract():
    """延迟导入 ros2.py 契约常量；无 ROS2 环境时返回 None（stub 模式）。"""
    try:
        # 注意: 必须用绝对导入（与 sensors.py/peripherals.py 一致）——本模块可能经
        # importlib.import_module("plugins.motion") 以顶层包名加载（__package__="plugins"），
        # 相对导入会抛 "attempted relative import with no known parent package"，
        # 被 except 吞掉后整栈落入 stub 模式（生产环境运动控制失效）
        from ros2 import (QOS_T800_RELIABLE, QOS_T800_BEST_EFFORT,
                          QOS_T800_JOINT, QOS_CORE, T800_TOPICS)
        return (QOS_T800_RELIABLE, QOS_T800_BEST_EFFORT,
                QOS_T800_JOINT, QOS_CORE, T800_TOPICS)
    except Exception:
        return None


def _load_msgs(*pairs):
    """延迟导入消息类。pairs = [(模块路径, 类名), ...]；失败返回 None。"""
    try:
        result = []
        for mod_path, cls_name in pairs:
            import importlib
            mod = importlib.import_module(mod_path)
            result.append(getattr(mod, cls_name))
        return tuple(result)
    except Exception:
        return None


# ══════════════════════════════════════════════════════════════════════════════
# MotionStatePlugin (sensor, tool "motion_state")
# 订阅 /motion/motion_state (BEST_EFFORT) → 缓存 → 2Hz 转发 String JSON 到
# core 域 /{ns}/motion/state，格式 data/json。支持 dispatch action "get" 直查。
# ══════════════════════════════════════════════════════════════════════════════

class MotionStatePlugin:
    """T800 运动状态机传感器卡片。

    数据流: /motion/motion_state (domain 69) → 缓存 → /{ns}/motion/state (domain 42, 2Hz)。
    其他控制类插件各自订阅 /motion/motion_state 做前置检查（避免插件间耦合）。
    """

    PREFIX = "motion_state"

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/motion/state"
        self._running = False
        self._lock = threading.Lock()
        self._latest = None            # {"current_motion_task", "available_transition_motions"}
        self._thread = None            # 2Hz 转发线程

        # ── 延迟初始化 ROS2（无环境时为 stub 模式） ──
        self._sub_node = None
        self._pub_node = None
        self._pub = None
        self._state_sub = None
        self._MotionState = None
        self._String = None
        try:
            contract = _load_ros2_contract()
            if contract is None:
                raise ImportError("ros2 契约不可用")
            _, best_effort_qos, _, core_qos, topics = contract
            msgs = _load_msgs(
                ("interface_protocol.msg", "MotionState"),
                ("std_msgs.msg", "String"),
            )
            if msgs is None:
                raise ImportError("消息类型导入失败")
            self._MotionState, self._String = msgs
            self._sub_node = ros2.make_node_t800("t800_motion_state_sub")
            self._pub_node = ros2.make_node_core("t800_motion_state_pub")
            self._pub = self._pub_node.create_publisher(
                self._String, self._topic, core_qos,
            )
            self._state_topic = topics["motion_state"]
            self._state_qos = best_effort_qos
        except Exception as e:  # noqa: BLE001
            print(f"[MotionStatePlugin] WARNING: ROS2 初始化失败（{e}），进入 stub 模式")

    def get_tool(self) -> dict:
        return {
            "name": "motion_state",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": (
                "T800 运动状态机 — 当前运动任务 current_motion_task 与可用转换白名单 "
                "available_transition_motions（2Hz 转发）。"
                "该卡片是其他控制卡（motion_switcher/loco/arm/joint_bridge）的前置状态来源。"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self) -> None:
        """启动订阅与 2Hz 转发线程。"""
        self._running = True
        if self._sub_node is not None and self._MotionState is not None and self._state_sub is None:
            try:
                self._state_sub = self._sub_node.create_subscription(
                    self._MotionState, self._state_topic, self._on_state, self._state_qos,
                )
                print("[MotionStatePlugin] 已订阅 /motion/motion_state")
            except Exception as e:  # noqa: BLE001
                print(f"[MotionStatePlugin] WARNING: 订阅失败（{e}），stub 模式")
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._publish_loop, daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _on_state(self, msg) -> None:
        try:
            data = {
                "current_motion_task": str(msg.current_motion_task or ""),
                "available_transition_motions": list(msg.available_transition_motions or []),
            }
            with self._lock:
                self._latest = data
        except Exception as e:  # noqa: BLE001
            print(f"[MotionStatePlugin] 回调异常: {e}")

    def _produce(self) -> dict | None:
        """当前状态 + 时间戳；无数据返回 None。"""
        with self._lock:
            data = self._latest
        if not data:
            return None
        payload = dict(data)
        payload["timestamp_ms"] = int(time.time() * 1000)
        return payload

    def _publish_loop(self) -> None:
        """2Hz 转发到 core 域 /{ns}/motion/state。"""
        while self._running:
            time.sleep(0.5)
            payload = self._produce()
            if payload is None:
                continue
            if self._pub is not None and self._String is not None:
                try:
                    out = self._String()
                    out.data = json.dumps(payload)
                    self._pub.publish(out)
                except Exception as e:  # noqa: BLE001
                    print(f"[MotionStatePlugin] 发布异常: {e}")

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return {
                "state": "running" if self._running else "idle",
                "topic_out": [{"topic": self._topic, "format": "data/json"}],
                "last": self._produce(),
            }
        if action in ("get", "read"):
            payload = self._produce()
            if payload is None:
                return {"state": "error", "error": "NO_FEEDBACK",
                        "message": "尚未收到 /motion/motion_state 数据，请确认机器人运控单元已启动"}
            return payload
        return None


# ══════════════════════════════════════════════════════════════════════════════
# MotionSwitcherPlugin (actuator, tool "motion_switcher")
# 订阅 /motion/motion_state 缓存白名单；switch 时校验 mode ∈ available_transition_motions，
# 发布 /motion/set_motion_state (RELIABLE)，轮询确认最多 3 秒。
# ══════════════════════════════════════════════════════════════════════════════

class MotionSwitcherPlugin:
    """T800 运动状态切换卡片。

    数据流: 订阅 /motion/motion_state 维护白名单 → 发布 MotionStateRequest 到
    /motion/set_motion_state → 轮询当前状态确认切换成功（3 秒超时）。
    mode 支持中文别名（如 "自然行走" → rl_basic），description 中列出官方全量状态名。
    """

    PREFIX = "motion_switcher"

    SWITCH_TIMEOUT = 3.0   # 官方切换超时

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._lock = threading.Lock()
        self._latest = None

        self._node = None
        self._state_sub = None
        self._req_pub = None
        self._MotionState = None
        self._MotionStateRequest = None
        try:
            contract = _load_ros2_contract()
            if contract is None:
                raise ImportError("ros2 契约不可用")
            reliable_qos, best_effort_qos, _, _, topics = contract
            msgs = _load_msgs(
                ("interface_protocol.msg", "MotionState"),
                ("interface_protocol.msg", "MotionStateRequest"),
            )
            if msgs is None:
                raise ImportError("消息类型导入失败")
            self._MotionState, self._MotionStateRequest = msgs
            self._node = ros2.make_node_t800("t800_motion_switcher")
            self._req_pub = self._node.create_publisher(
                self._MotionStateRequest, topics["set_motion_state"], reliable_qos,
            )
            self._state_topic = topics["motion_state"]
            self._state_qos = best_effort_qos
        except Exception as e:  # noqa: BLE001
            print(f"[MotionSwitcherPlugin] WARNING: ROS2 初始化失败（{e}），stub 模式")

    def get_tool(self) -> dict:
        return {
            "name": "motion_switcher",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "T800 运动状态切换 — 目标状态必须位于当前 available_transition_motions 白名单内，"
                "切换确认最长 3 秒。官方状态机: idle=空闲, passive=阻力模式, pd_stand=PD站立, "
                "rl_basic=自然行走, lower_body_balance=下肢平衡（上肢运动前置）, "
                "joint_bridge=关节透传, pd_sitground=坐着起身, walk_server=行走服务端, "
                "rl_mimic_supine_to_stance=仰卧起身(模仿), rl_mimic_prone_to_stance=俯卧起身(模仿), "
                "rl_mimic_stance_to_supine=站立躺下(模仿), rl_mimic_stance_to_sitdown=站立坐下(模仿), "
                "rl_mimic_sitdown_to_stance=坐着起身(模仿)。"
                "mode 支持中文别名: 空闲/阻力模式/PD站立/自然行走/下肢平衡/关节透传/坐着起身/行走服务端。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["switch", "get_state", "list_available"],
                        "description": "要执行的动作",
                    },
                    "mode": {
                        "type": "string",
                        "description": "目标运动状态名或中文别名（如 rl_basic / 自然行走）",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "switch": {"params": ["mode"],
                               "description": "切换至目标运动状态（白名单校验，等待确认最长 3 秒）"},
                    "get_state": {"params": [],
                                  "description": "查询当前运动状态与可用转换列表"},
                    "list_available": {"params": [],
                                       "description": "列出当前可切换的运动状态"},
                },
            },
        }

    def start(self) -> None:
        if self._node is not None and self._MotionState is not None and self._state_sub is None:
            try:
                self._state_sub = self._node.create_subscription(
                    self._MotionState, self._state_topic, self._on_state, self._state_qos,
                )
                print("[MotionSwitcherPlugin] 已订阅 /motion/motion_state")
            except Exception as e:  # noqa: BLE001
                print(f"[MotionSwitcherPlugin] WARNING: 订阅失败（{e}），stub 模式")

    def stop(self) -> None:
        pass

    def _on_state(self, msg) -> None:
        try:
            data = {
                "current_motion_task": str(msg.current_motion_task or ""),
                "available_transition_motions": list(msg.available_transition_motions or []),
            }
            with self._lock:
                self._latest = data
        except Exception as e:  # noqa: BLE001
            print(f"[MotionSwitcherPlugin] 回调异常: {e}")

    def _snapshot(self) -> dict | None:
        with self._lock:
            return dict(self._latest) if self._latest else None

    def _do_switch(self, mode: str) -> dict:
        """白名单校验 → 发布请求 → 轮询确认（3 秒超时）。"""
        official = _MODE_ALIASES.get(mode, mode)
        st = self._snapshot()
        if st is None:
            return _err("NO_MOTION_STATE",
                        "尚未收到 /motion/motion_state 反馈，无法确认可切换状态，请稍后重试")
        cur = st.get("current_motion_task")
        avail = st.get("available_transition_motions") or []
        if official == cur:
            return {"state": "ok", "current_motion": official, "elapsed_ms": 0,
                    "already_in_state": True}
        if official not in avail:
            return _err("NOT_AVAILABLE",
                        f"当前状态 {cur} 不允许直接切换到 {official}。"
                        f"当前可用转换: {'、'.join(avail) if avail else '（无）'}")
        if self._req_pub is None or self._MotionStateRequest is None:
            return _err("ROS2_UNAVAILABLE", "ROS2 环境未初始化（stub 模式），无法发布切换请求")
        req = self._MotionStateRequest()
        req.target_motion_name = official
        self._req_pub.publish(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.SWITCH_TIMEOUT:
            time.sleep(0.05)
            st2 = self._snapshot()
            if st2 and st2.get("current_motion_task") == official:
                return {"state": "ok", "current_motion": official,
                        "elapsed_ms": int((time.monotonic() - t0) * 1000)}
        return _err("TIMEOUT",
                    f"3 秒内未确认切换到 {official}（当前仍为 {cur}），请检查运控状态")

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self.start()
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready", "last": self._snapshot()}
        if action == "switch":
            mode = str(args.get("mode", "")).strip()
            if not mode:
                return _err("INVALID_ARGUMENT", "缺少 mode 参数（目标状态名或中文别名）")
            return self._do_switch(mode)
        if action == "get_state":
            st = self._snapshot()
            if st is None:
                return _err("NO_MOTION_STATE", "尚未收到运动状态反馈")
            return st
        if action == "list_available":
            st = self._snapshot()
            if st is None:
                return _err("NO_MOTION_STATE", "尚未收到运动状态反馈")
            return {"current": st.get("current_motion_task"),
                    "available": st.get("available_transition_motions") or []}
        return None


# ══════════════════════════════════════════════════════════════════════════════
# LocoPlugin (actuator, tool "loco")
# 100Hz 持续发布 /motion/body_vel_cmd；速度渐变（每次 tick 向目标步进 20%）；
# duration>0 到时自动发零速；>2s 未收到新命令防御性自停（官方接收端 2 秒无数据即停）。
# ══════════════════════════════════════════════════════════════════════════════

class LocoPlugin:
    """T800 机体速度控制卡片。

    数据流: move(vx, vy, vyaw) → 100Hz 定时器发布 BodyVelCmd 到 /motion/body_vel_cmd，
    linear_velocity=[x, y]、yaw_velocity；header.frame_id="body"。限制 x/y ±1 m/s、
    yaw ±1 rad/s。停止 = 显式发零速度（stop）或定时自停（duration / 2 秒看门狗）。
    前置检查: 需要 rl_basic（自然行走）或 walk_server 模式。
    """

    PREFIX = "loco"

    CONTROL_PERIOD = 0.01     # 100Hz
    WATCHDOG_SEC = 2.0        # 接收端 2 秒未收到命令自动停
    GRADIENT = 0.2            # 每次 tick 向目标速度步进 20%（简单渐变）

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._lock = threading.Lock()
        self._target = None                 # (vx, vy, vyaw) 目标速度；None = 空闲
        self._cur = (0.0, 0.0, 0.0)         # 当前实际发布速度（渐变用）
        self._last_cmd_time = 0.0
        self._deadline = None               # duration 到期时间（monotonic）
        self._timer = None

        self._node = None
        self._pub = None
        self._state_sub = None
        self._BodyVelCmd = None
        self._Header = None
        self._MotionState = None
        try:
            contract = _load_ros2_contract()
            if contract is None:
                raise ImportError("ros2 契约不可用")
            _, best_effort_qos, _, _, topics = contract
            msgs = _load_msgs(
                ("interface_protocol.msg", "BodyVelCmd"),
                ("interface_protocol.msg", "MotionState"),
                ("std_msgs.msg", "Header"),
            )
            if msgs is None:
                raise ImportError("消息类型导入失败")
            self._BodyVelCmd, self._MotionState, self._Header = msgs
            self._node = ros2.make_node_t800("t800_loco")
            # 官方示例 body_vel_cmd 用默认 depth10 QoS
            self._pub = self._node.create_publisher(
                self._BodyVelCmd, topics["body_vel_cmd"], 10,
            )
            self._state_topic = topics["motion_state"]
            self._state_qos = best_effort_qos
        except Exception as e:  # noqa: BLE001
            print(f"[LocoPlugin] WARNING: ROS2 初始化失败（{e}），stub 模式")

    def get_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "T800 机体速度控制 — 100Hz 持续发布 /motion/body_vel_cmd。"
                "限制: x/y 方向 ±1 m/s，yaw ±1 rad/s；duration>0 到时自动停止；"
                "接收端 2 秒未收到命令会自动停下。停止 = 显式发零速度。"
                "速度按每次 20% 渐变平滑到达目标。前置条件: rl_basic（自然行走）或 walk_server 模式。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["move", "stop"],
                        "description": "要执行的动作",
                    },
                    "vx":       {"type": "number", "description": "前进速度 m/s [-1, 1]"},
                    "vy":       {"type": "number", "description": "横向速度 m/s [-1, 1]"},
                    "vyaw":     {"type": "number", "description": "转向角速度 rad/s [-1, 1]"},
                    "duration": {"type": "number",
                                 "description": "运动时长秒；0 或省略 = 持续运动直到 stop"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move": {"params": ["vx", "vy", "vyaw", "duration"],
                             "description": "按指定速度运动（100Hz 持续发布，duration>0 到时自动停）"},
                    "stop": {"params": [],
                             "description": "立即停止运动（显式发布零速度并停定时器）"},
                },
            },
        }

    def start(self) -> None:
        if self._node is not None and self._MotionState is not None and self._state_sub is None:
            try:
                self._state_sub = self._node.create_subscription(
                    self._MotionState, self._state_topic, self._on_state, self._state_qos,
                )
                print("[LocoPlugin] 已订阅 /motion/motion_state")
            except Exception as e:  # noqa: BLE001
                print(f"[LocoPlugin] WARNING: 订阅失败（{e}），stub 模式")

    def stop(self) -> None:
        """系统/工具 stop：立即发零速度并停 timer。"""
        with self._lock:
            self._zero_and_stop_locked()

    def _on_state(self, msg) -> None:
        try:
            data = {"current_motion_task": str(msg.current_motion_task or ""),
                    "available_transition_motions": list(msg.available_transition_motions or [])}
            with self._lock:
                self._motion_state = data
        except Exception as e:  # noqa: BLE001
            print(f"[LocoPlugin] 回调异常: {e}")

    def _snapshot_state(self) -> dict | None:
        with self._lock:
            return dict(self._motion_state) if getattr(self, "_motion_state", None) else None

    def _ensure_timer(self) -> None:
        if self._timer is None and self._node is not None:
            self._timer = self._node.create_timer(self.CONTROL_PERIOD, self._tick)

    def _publish(self, vx: float, vy: float, vyaw: float) -> None:
        if self._pub is None or self._BodyVelCmd is None or self._Header is None:
            return
        try:
            msg = self._BodyVelCmd()
            msg.header = self._Header()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = "body"
            msg.linear_velocity = [vx, vy]
            msg.yaw_velocity = vyaw
            self._pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            print(f"[LocoPlugin] 发布异常: {e}")

    def _zero_and_stop_locked(self) -> None:
        """发零速度 + 停 timer + 置空闲（须持锁调用）。"""
        vx, vy, vyaw = self._cur
        self._publish(0.0, 0.0, 0.0)
        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception:  # noqa: BLE001
                pass
            self._timer = None
        self._target = None
        self._deadline = None
        self._cur = (0.0, 0.0, 0.0)
        print(f"[LocoPlugin] 已停止（原速度 {vx:.2f}, {vy:.2f}, {vyaw:.2f}）")

    def _tick(self) -> None:
        """100Hz 定时器：渐变发布 + duration 到期自停 + 2 秒看门狗。"""
        with self._lock:
            if self._target is None:
                return
            now = time.monotonic()
            # duration 到期 → 自动停（先于看门狗判断，避免看门狗截断 duration>2s 的运动；
            # 100Hz 持续发布本身保证接收端不缺数据）
            if self._deadline is not None and now >= self._deadline:
                print("[LocoPlugin] duration 到期，自动停止")
                self._zero_and_stop_locked()
                return
            # 防御: 仅持续运动（无 duration 到期时间）距上次 move 命令超过 2 秒 →
            # 零速自停（官方接收端 2s 无数据即停；带 duration 的运动以到期自停为准）
            if self._deadline is None and now - self._last_cmd_time > self.WATCHDOG_SEC:
                print("[LocoPlugin] 看门狗: 超过 2 秒无新命令，自动停止")
                self._zero_and_stop_locked()
                return
            tx, ty, tyaw = self._target
            cx, cy, cyaw = self._cur
            # 每次 tick 向目标步进 20%（简单渐变，防突变冲击）
            nx = cx + (tx - cx) * self.GRADIENT
            ny = cy + (ty - cy) * self.GRADIENT
            nyaw = cyaw + (tyaw - cyaw) * self.GRADIENT
            if abs(tx - nx) < 0.001 and abs(ty - ny) < 0.001 and abs(tyaw - nyaw) < 0.001:
                nx, ny, nyaw = tx, ty, tyaw   # 已收敛到目标
            self._cur = (nx, ny, nyaw)
        self._publish(nx, ny, nyaw)

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self.start()
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            with self._lock:
                target = list(self._target) if self._target else None
                cur = list(self._cur)
            return {"state": "moving" if target else "ready",
                    "current": cur, "target": target}
        if action == "move":
            vx = float(args.get("vx", 0.0))
            vy = float(args.get("vy", 0.0))
            vyaw = float(args.get("vyaw", 0.0))
            duration = float(args.get("duration", 0.0))
            if abs(vx) > 1.0 or abs(vy) > 1.0:
                return _err("INVALID_ARGUMENT",
                            f"x/y 方向速度超出限制 ±1 m/s（vx={vx}, vy={vy}）")
            if abs(vyaw) > 1.0:
                return _err("INVALID_ARGUMENT", f"yaw 角速度超出限制 ±1 rad/s（vyaw={vyaw}）")
            # 运动状态前置检查
            st = self._snapshot_state()
            if st is None:
                return _err("NO_MOTION_STATE",
                            "尚未收到运动状态反馈，无法确认当前模式，请先启动 motion_state 卡片并确认机器人已上电")
            cur_mode = st.get("current_motion_task")
            if cur_mode not in _LOCO_REQUIRED_MODES:
                return _err("WRONG_MOTION_STATE",
                            f"当前运动状态为 {cur_mode}，机体速度控制需要行走类模式"
                            f"（rl_basic 自然行走 / walk_server 行走服务端），请先用 motion_switcher 切换")
            with self._lock:
                self._target = (vx, vy, vyaw)
                self._last_cmd_time = time.monotonic()
                self._deadline = self._last_cmd_time + duration if duration > 0 else None
                self._ensure_timer()
            result = {"state": "ok", "vx": vx, "vy": vy, "vyaw": vyaw, "duration": duration}
            if duration > 0:
                result["scheduled_stop_at_ms"] = int((self._deadline or 0) * 1000)
            return result
        return None


# ══════════════════════════════════════════════════════════════════════════════
# ArmMotionPlugin (actuator, tool "arm")
# 上肢运动规划：发布 /motion/joint_motion_plan/request (RELIABLE)，订阅
# /motion/joint_motion_plan/state (BEST_EFFORT) 缓存 {request_id, status, progress}，
# 并 5Hz 转发到 core 域 /{ns}/arm/state。手势以任务队列后台顺序执行（不阻塞 dispatch）。
# 前置检查: current_motion_task == "lower_body_balance"。
# ══════════════════════════════════════════════════════════════════════════════

# 官方预设手势（target_positions/duration/stiffness/damping 取自 config/t800/*.yaml）
_ARM_PRESETS = {
    "extend_right_hand": [
        # 注意: 官方 YAML 中该动作拼写为 extand_right_hand，此处保留官方名并注明
        {"name": "extand_right_hand", "positions": list(_ARM_EXTEND_POSITIONS),
         "duration": 2.0, "stiffness": list(_UPPER_STIFFNESS_HAND),
         "damping": list(_UPPER_DAMPING_HAND), "gravity": True},
    ],
    "shake_hand": [
        {"name": "extand_right_hand", "positions": list(_ARM_EXTEND_POSITIONS),
         "duration": 2.0, "stiffness": list(_UPPER_STIFFNESS_HAND),
         "damping": list(_UPPER_DAMPING_HAND), "gravity": True},
        {"name": "withdraw_right_hand", "positions": list(_ARM_WITHDRAW_POSITIONS),
         "duration": 2.0, "stiffness": list(_UPPER_STIFFNESS_HAND),
         "damping": list(_UPPER_DAMPING_HAND), "gravity": True},
    ],
    "wave_hand": [
        {"name": "init", "positions": list(_WAVE_INIT_POSITIONS),
         "duration": 1.0, "stiffness": list(_UPPER_STIFFNESS_WAVE_INIT),
         "damping": list(_UPPER_DAMPING_HAND), "gravity": True},
        {"name": "raise_hands", "positions": list(_WAVE_RAISE_POSITIONS),
         "duration": 2.0, "stiffness": list(_UPPER_STIFFNESS_WAVE_INIT),
         "damping": list(_UPPER_DAMPING_HAND), "gravity": True},
        {"name": "wave_hands_1", "positions": list(_WAVE_HANDS_POSITIONS),
         "duration": 0.3, "stiffness": list(_UPPER_STIFFNESS_WAVE_STEP),
         "damping": list(_UPPER_DAMPING_HAND), "gravity": True},
        {"name": "wave_hands_2", "positions": list(_WAVE_RAISE_POSITIONS),
         "duration": 0.3, "stiffness": list(_UPPER_STIFFNESS_WAVE_STEP),
         "damping": list(_UPPER_DAMPING_HAND), "gravity": True},
        {"name": "wave_hands_3", "positions": list(_WAVE_HANDS_POSITIONS),
         "duration": 0.3, "stiffness": list(_UPPER_STIFFNESS_WAVE_STEP),
         "damping": list(_UPPER_DAMPING_HAND), "gravity": True},
        {"name": "wave_hands_4", "positions": list(_WAVE_RAISE_POSITIONS),
         "duration": 0.3, "stiffness": list(_UPPER_STIFFNESS_WAVE_STEP),
         "damping": list(_UPPER_DAMPING_HAND), "gravity": True},
        {"name": "wave_hands_5", "positions": list(_WAVE_HANDS_POSITIONS),
         "duration": 0.3, "stiffness": list(_UPPER_STIFFNESS_WAVE_END),
         "damping": list(_UPPER_DAMPING_HAND), "gravity": True},
        # 官方 yaml 末尾的复位任务：挥手结束后回到默认姿态
        {"name": "reset", "request_type": "RESET"},
    ],
}


class ArmMotionPlugin:
    """T800 上肢运动规划卡片。

    数据流: 预设手势任务入队 → 后台线程等待规划器 IDLE 后依次发布
    JointMotionPlanRequest（request_id 递增）→ 订阅 JointMotionPlanState 缓存
    状态/进度 → 5Hz 转发 /{ns}/arm/state (data/json)。
    前置检查: 需 lower_body_balance（下肢平衡）模式。
    """

    PREFIX = "arm"

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._state_topic_out = f"/{namespace}/arm/state"
        self._lock = threading.Lock()
        self._state = None            # {"request_id", "status", "progress"}
        self._motion_state = None
        self._request_id = 0
        self._last_sent_request_id = None
        self._last_sent_at = 0.0
        self._expected_ms = 0         # 期望完成时长（execution_time + 余量）
        self._queue = queue.Queue()
        self._worker: threading.Thread | None = None
        self._running = False
        self._last_error = None
        self._last_core_pub = 0.0

        self._node = None
        self._core_node = None
        self._req_pub = None
        self._core_pub = None
        self._state_sub = None
        self._motion_state_sub = None
        self._JointMotionPlanRequest = None
        self._JointMotionPlanState = None
        self._MotionState = None
        self._String = None
        try:
            contract = _load_ros2_contract()
            if contract is None:
                raise ImportError("ros2 契约不可用")
            reliable_qos, best_effort_qos, _, core_qos, topics = contract
            msgs = _load_msgs(
                ("interface_protocol.msg", "JointMotionPlanRequest"),
                ("interface_protocol.msg", "JointMotionPlanState"),
                ("interface_protocol.msg", "MotionState"),
                ("std_msgs.msg", "String"),
            )
            if msgs is None:
                raise ImportError("消息类型导入失败")
            (self._JointMotionPlanRequest, self._JointMotionPlanState,
             self._MotionState, self._String) = msgs
            self._node = ros2.make_node_t800("t800_arm")
            self._req_pub = self._node.create_publisher(
                self._JointMotionPlanRequest, topics["motion_plan_request"], reliable_qos,
            )
            self._core_node = ros2.make_node_core("t800_arm_state_pub")
            self._core_pub = self._core_node.create_publisher(
                self._String, self._state_topic_out, core_qos,
            )
            self._state_topic = topics["motion_plan_state"]
            self._state_qos = best_effort_qos
            self._motion_topic = topics["motion_state"]
        except Exception as e:  # noqa: BLE001
            print(f"[ArmMotionPlugin] WARNING: ROS2 初始化失败（{e}），stub 模式")

    def get_tool(self) -> dict:
        return {
            "name": "arm",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "T800 上肢运动规划 — 预设手势: extend_right_hand=伸右手, shake_hand=握手"
                "（伸右手→收右手）, wave_hand=挥手（官方 wave_hands 全任务）; "
                "reset=复位到默认姿态, cancel=取消当前任务, get_state=查询规划状态/进度。"
                "前置条件: lower_body_balance（下肢平衡）模式。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["extend_right_hand", "shake_hand", "wave_hand",
                                 "reset", "cancel", "get_state"],
                        "description": "要执行的动作",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "extend_right_hand": {"params": [], "description": "伸出右手（官方 extand_right_hand 姿态）"},
                    "shake_hand":        {"params": [], "description": "握手（依次执行 伸右手 → 收右手）"},
                    "wave_hand":         {"params": [], "description": "挥手（执行官方 wave_hands 全部任务，含结尾复位）"},
                    "reset":             {"params": [], "description": "复位上肢到默认姿态"},
                    "cancel":            {"params": [], "description": "取消当前规划任务并清空队列"},
                    "get_state":         {"params": [], "description": "查询规划器状态与进度"},
                },
            },
            "topic_out": [{"topic": self._state_topic_out, "format": "data/json"}],
        }

    def start(self) -> None:
        """启动订阅与后台任务队列线程。"""
        self._running = True
        if self._node is not None and self._state_sub is None:
            try:
                self._state_sub = self._node.create_subscription(
                    self._JointMotionPlanState, self._state_topic,
                    self._on_plan_state, self._state_qos,
                )
                self._motion_state_sub = self._node.create_subscription(
                    self._MotionState, self._motion_topic,
                    self._on_motion_state, self._state_qos,
                )
                print("[ArmMotionPlugin] 已订阅 /motion/joint_motion_plan/state 与 /motion/motion_state")
            except Exception as e:  # noqa: BLE001
                print(f"[ArmMotionPlugin] WARNING: 订阅失败（{e}），stub 模式")
        if self._worker is None or not self._worker.is_alive():
            self._worker = threading.Thread(target=self._worker_loop, daemon=True)
            self._worker.start()

    def stop(self) -> None:
        """停止: 清空队列、退出 worker、发送取消请求（安全兜底）。"""
        self._running = False
        if self._worker is not None:
            try:
                self._queue.put_nowait(None)   # 哨兵
            except queue.Full:
                pass
            self._worker.join(timeout=2)
            self._worker = None
        self._send_cancel_request()

    # ── 订阅回调 ─────────────────────────────────────────────────────────────

    def _on_plan_state(self, msg) -> None:
        try:
            rid = int(msg.request_id)
            with self._lock:
                self._state = {"request_id": rid, "status": int(msg.status),
                               "progress": float(msg.progress)}
                # 以最新 state 的 request_id 为基准递增（官方示例同此逻辑）
                if rid >= self._request_id:
                    self._request_id = rid
            # 5Hz 节流转发到 core 域 /{ns}/arm/state
            now = time.monotonic()
            if self._core_pub is not None and self._String is not None and now - self._last_core_pub >= 0.2:
                self._last_core_pub = now
                payload = dict(self._state)
                payload["timestamp_ms"] = int(now * 1000)
                out = self._String()
                out.data = json.dumps(payload)
                self._core_pub.publish(out)
        except Exception as e:  # noqa: BLE001
            print(f"[ArmMotionPlugin] 规划状态回调异常: {e}")

    def _on_motion_state(self, msg) -> None:
        try:
            with self._lock:
                self._motion_state = {
                    "current_motion_task": str(msg.current_motion_task or ""),
                    "available_transition_motions": list(msg.available_transition_motions or []),
                }
        except Exception as e:  # noqa: BLE001
            print(f"[ArmMotionPlugin] 运动状态回调异常: {e}")

    # ── 前置检查 ─────────────────────────────────────────────────────────────

    def _check_precondition(self) -> dict | None:
        """返回错误 dict 或 None（通过）。要求 lower_body_balance 模式。"""
        with self._lock:
            st = dict(self._motion_state) if self._motion_state else None
        if st is None:
            return _err("NO_MOTION_STATE",
                        "尚未收到运动状态反馈，无法确认当前模式，请先确认机器人已上电")
        cur = st.get("current_motion_task")
        if cur != "lower_body_balance":
            return _err("WRONG_MOTION_STATE",
                        f"当前运动状态为 {cur}，上肢运动规划需要 lower_body_balance（下肢平衡）模式，"
                        f"请先用 motion_switcher 切换")
        return None

    # ── 后台任务队列（顺序发送，不阻塞 dispatch） ────────────────────────────

    def _worker_loop(self) -> None:
        """从队列取任务，等待规划器 IDLE 后发送；None 为停止哨兵。"""
        while self._running:
            try:
                task = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if task is None:
                break
            try:
                if not self._wait_planner_idle(20.0):
                    self._last_error = "规划器未在 20 秒内回到 IDLE，任务未发送"
                    print(f"[ArmMotionPlugin] {self._last_error}: {task.get('name')}")
                    continue
                if task.get("request_type") == "RESET":
                    self._send_reset_request()   # 复位任务走 REQUEST_RESET
                else:
                    self._send_plan_task(task)
                self._last_error = None
            except Exception as e:  # noqa: BLE001
                self._last_error = str(e)
                print(f"[ArmMotionPlugin] 任务执行异常: {e}")

    def _wait_planner_idle(self, timeout: float) -> bool:
        """轮询缓存状态，等待规划器 IDLE（status==1）。"""
        deadline = time.monotonic() + timeout
        while self._running and time.monotonic() < deadline:
            with self._lock:
                status = self._state.get("status") if self._state else None
            if status == 1:   # IDLE
                return True
            time.sleep(0.1)
        return False

    def _send_plan_task(self, task: dict) -> None:
        """构建并发布 REQUEST_PLAN_EXECUTE 请求。"""
        if self._req_pub is None or self._JointMotionPlanRequest is None:
            self._last_error = "ROS2 环境未初始化（stub 模式），无法发送规划请求"
            print(f"[ArmMotionPlugin] {self._last_error}")
            return
        req = self._JointMotionPlanRequest()
        with self._lock:
            self._request_id += 1
            rid = self._request_id
        req.request_id = rid
        req.request_type = self._JointMotionPlanRequest.REQUEST_PLAN_EXECUTE
        req.use_gravity_compensation = bool(task.get("gravity", True))
        req.joint_indices = list(UPPER_BODY_JOINT_INDICES)
        req.target_positions = [float(p) for p in task["positions"]]
        req.target_velocities = []          # 空数组 = 自动规划
        req.execution_time = float(task["duration"])
        req.stiffness = [float(s) for s in (task.get("stiffness") or [])]
        req.damping = [float(d) for d in (task.get("damping") or [])]
        self._req_pub.publish(req)
        with self._lock:
            self._last_sent_request_id = rid
            self._last_sent_at = time.monotonic()
            # 期望完成时长 = execution_time + 2 秒余量（供 get_state 估算进度）
            self._expected_ms = int(float(task["duration"]) * 1000) + 2000
        print(f"[ArmMotionPlugin] 已发布规划任务 {task.get('name')} request_id={rid}")

    def _send_cancel_request(self) -> None:
        """发布 REQUEST_CANCEL（携带最新 request_id）。"""
        if self._req_pub is None or self._JointMotionPlanRequest is None:
            return
        try:
            with self._lock:
                self._request_id += 1
                rid = self._request_id
            req = self._JointMotionPlanRequest()
            req.request_id = rid
            req.request_type = self._JointMotionPlanRequest.REQUEST_CANCEL
            self._req_pub.publish(req)
            print(f"[ArmMotionPlugin] 已发布取消请求 request_id={rid}")
        except Exception as e:  # noqa: BLE001
            print(f"[ArmMotionPlugin] 取消请求失败: {e}")

    def _send_reset_request(self) -> None:
        """发布 REQUEST_RESET（joint_indices 置空 = 复位默认姿态）。"""
        if self._req_pub is None or self._JointMotionPlanRequest is None:
            return
        try:
            with self._lock:
                self._request_id += 1
                rid = self._request_id
            req = self._JointMotionPlanRequest()
            req.request_id = rid
            req.request_type = self._JointMotionPlanRequest.REQUEST_RESET
            req.joint_indices = []          # 空数组表示复位默认姿态
            req.target_positions = []
            req.target_velocities = []
            req.execution_time = 0.0
            req.stiffness = []
            req.damping = []
            self._req_pub.publish(req)
            with self._lock:
                self._last_sent_request_id = rid
                self._last_sent_at = time.monotonic()
                self._expected_ms = 3000    # 复位按 3 秒估算
            print(f"[ArmMotionPlugin] 已发布复位请求 request_id={rid}")
        except Exception as e:  # noqa: BLE001
            print(f"[ArmMotionPlugin] 复位请求失败: {e}")

    def _enqueue(self, preset_name: str) -> dict:
        """将预设手势全部任务入队（队列为空时立即返回，不阻塞）。"""
        tasks = _ARM_PRESETS.get(preset_name)
        if not tasks:
            return _err("INVALID_ARGUMENT", f"未知手势: {preset_name}")
        for t in tasks:
            self._queue.put(dict(t))
        return {"state": "ok", "gesture": preset_name,
                "queued_tasks": [t["name"] for t in tasks],
                "queue_remaining": self._queue.qsize()}

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self.start()
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready",
                    "topic_out": [{"topic": self._state_topic_out, "format": "data/json"}],
                    "last": dict(self._state) if self._state else None}
        if action in ("extend_right_hand", "shake_hand", "wave_hand"):
            err = self._check_precondition()
            if err:
                return err
            return self._enqueue(action)
        if action == "reset":
            err = self._check_precondition()
            if err:
                return err
            # 复位也走队列，确保在上一任务完成后执行
            self._queue.put({"name": "reset", "request_type": "RESET"})
            return {"state": "ok", "gesture": "reset", "queued": True,
                    "queue_remaining": self._queue.qsize()}
        if action == "cancel":
            # 清空待执行队列
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
            self._send_cancel_request()
            return {"state": "ok", "cancelled_request_id": self._request_id,
                    "queue_cleared": True}
        if action == "get_state":
            with self._lock:
                s = dict(self._state) if self._state else None
                last_sent = self._last_sent_request_id
                sent_at = self._last_sent_at
                expected_ms = self._expected_ms
            queue_remaining = self._queue.qsize()
            if s is None:
                return {"state": "idle", "status": "UNKNOWN", "request_id": self._request_id,
                        "queue_remaining": queue_remaining,
                        "message": "尚未收到规划状态反馈（规划器可能未启动）"}
            status = _PLAN_STATUS_NAMES.get(s.get("status"), "UNKNOWN")
            progress = s.get("progress", 0.0)
            # 若缓存状态不是本次发送的任务（或未回馈），按期望完成时间估算进度
            if last_sent is not None and s.get("request_id") != last_sent:
                elapsed_ms = (time.monotonic() - sent_at) * 1000
                progress = min(1.0, max(0.0, elapsed_ms / max(expected_ms, 1)))
            return {"state": "ok", "status": status, "progress": round(progress, 3),
                    "request_id": s.get("request_id"), "queue_remaining": queue_remaining,
                    "last_error": self._last_error}
        return None


# ══════════════════════════════════════════════════════════════════════════════
# JointBridgePlugin (actuator, tool "joint_bridge")
# 500Hz 全身 25 关节 PD 位置控制：订阅 /hardware/joint_state 取当前位置为插值起点，
# 线性插值（步长=(目标-当前)/(步数-1)，官方 joint_bridge_example 公式）逐拍发布
# JointCommand 到 /hardware/joint_command。config 默认关闭，仅限专业人员。
# ══════════════════════════════════════════════════════════════════════════════

class JointBridgePlugin:
    """T800 关节透传卡片（危险操作）。

    数据流: set_pose(positions[25]) → 以最新 /hardware/joint_state 位置为起点，
    按 num_steps 线性插值，500Hz 定时器逐拍发布 JointCommand
    （position=插值位, velocity=0, feed_forward_torque=0, torque=0, stiffness, damping,
    parallel_parser_type=CLASSIC_PARSER(0)）。
    前置检查: joint_bridge（关节透传）模式。config.yaml 默认关闭。
    """

    PREFIX = "joint_bridge"

    CONTROL_PERIOD = 1.0 / 500.0    # 500Hz

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        # config 默认关闭；仅 config.yaml enabled: true 时才会被实例化并放行操作
        self._enabled = bool((plugin_config or {}).get("enabled", False))
        self._ns = namespace
        self._ros2 = ros2
        self._lock = threading.Lock()
        self._joint_positions = None      # 最新 /hardware/joint_state 位置
        self._motion_state = None
        self._interp = None               # 每关节插值轨迹 [[pos0, pos1, ...], ...]
        self._num_steps = 0
        self._step_idx = 0
        self._stiffness = list(_JB_DEFAULT_STIFFNESS)
        self._damping = list(_JB_DEFAULT_DAMPING)
        self._timer = None
        self._subs_created = False

        self._node = None
        self._cmd_pub = None
        self._joint_state_sub = None
        self._motion_state_sub = None
        self._JointCommand = None
        self._JointState = None
        self._MotionState = None
        self._Header = None
        try:
            contract = _load_ros2_contract()
            if contract is None:
                raise ImportError("ros2 契约不可用")
            _, best_effort_qos, joint_qos, _, topics = contract
            msgs = _load_msgs(
                ("interface_protocol.msg", "JointCommand"),
                ("interface_protocol.msg", "JointState"),
                ("interface_protocol.msg", "MotionState"),
                ("std_msgs.msg", "Header"),
            )
            if msgs is None:
                raise ImportError("消息类型导入失败")
            self._JointCommand, self._JointState, self._MotionState, self._Header = msgs
            self._node = ros2.make_node_t800("t800_joint_bridge")
            # 官方 joint_bridge_example: depth3 BEST_EFFORT VOLATILE — 500Hz 高保真，
            # joint_state 订阅与 joint_command 发布用 QOS_T800_JOINT（depth3），
            # motion_state 订阅保持 QOS_T800_BEST_EFFORT（depth1）即可
            self._cmd_pub = self._node.create_publisher(
                self._JointCommand, topics["joint_command"], joint_qos,
            )
            self._joint_state_topic = topics["joint_state"]
            self._motion_topic = topics["motion_state"]
            self._state_qos = best_effort_qos
            self._joint_qos = joint_qos
        except Exception as e:  # noqa: BLE001
            print(f"[JointBridgePlugin] WARNING: ROS2 初始化失败（{e}），stub 模式")

    def get_tool(self) -> dict:
        return {
            "name": "joint_bridge",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "T800 关节透传 — 500Hz 全身 25 关节 PD 位置控制（J00~J24 全部关节）。"
                "危险操作: 仅限专业人员使用，必须处于 joint_bridge（关节透传）模式，"
                "并配合安全吊架/清场！config.yaml 默认关闭（enabled: false）。"
                "关节命令数组长度必须为 25。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["set_pose", "stop"],
                        "description": "要执行的动作",
                    },
                    "positions": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "目标关节位置 rad，长度必须为 25（J00~J24）",
                    },
                    "stiffness": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "刚度 kp 长度 25，缺省用官方 pd_joint_test 值",
                    },
                    "damping": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "阻尼 kd 长度 25，缺省用官方 pd_joint_test 值",
                    },
                    "num_steps": {
                        "type": "integer",
                        "description": "插值步数（默认 1500，对应约 3 秒 @500Hz）",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "set_pose": {"params": ["positions", "stiffness", "damping", "num_steps"],
                                 "description": "按 25 关节目标位置执行线性插值到位（500Hz）"},
                    "stop": {"params": [], "description": "立即停止插值（保持当前命令停止发布）"},
                },
            },
        }

    def start(self) -> None:
        if self._node is not None and not self._subs_created:
            try:
                self._joint_state_sub = self._node.create_subscription(
                    self._JointState, self._joint_state_topic,
                    self._on_joint_state, self._joint_qos,
                )
                self._motion_state_sub = self._node.create_subscription(
                    self._MotionState, self._motion_topic,
                    self._on_motion_state, self._state_qos,
                )
                self._subs_created = True
                print("[JointBridgePlugin] 已订阅 /hardware/joint_state 与 /motion/motion_state")
            except Exception as e:  # noqa: BLE001
                print(f"[JointBridgePlugin] WARNING: 订阅失败（{e}），stub 模式")

    def stop(self) -> None:
        """停止插值：取消 500Hz 定时器，清空轨迹。"""
        with self._lock:
            self._interp = None
            if self._timer is not None:
                try:
                    self._timer.cancel()
                except Exception:  # noqa: BLE001
                    pass
                self._timer = None

    def _on_joint_state(self, msg) -> None:
        try:
            with self._lock:
                self._joint_positions = [float(p) for p in (msg.position or [])]
        except Exception as e:  # noqa: BLE001
            print(f"[JointBridgePlugin] 关节状态回调异常: {e}")

    def _on_motion_state(self, msg) -> None:
        try:
            with self._lock:
                self._motion_state = {
                    "current_motion_task": str(msg.current_motion_task or ""),
                    "available_transition_motions": list(msg.available_transition_motions or []),
                }
        except Exception as e:  # noqa: BLE001
            print(f"[JointBridgePlugin] 运动状态回调异常: {e}")

    def _check_precondition(self) -> dict | None:
        """要求 joint_bridge 模式。返回错误 dict 或 None。"""
        with self._lock:
            st = dict(self._motion_state) if self._motion_state else None
        if st is None:
            return _err("NO_MOTION_STATE",
                        "尚未收到运动状态反馈，无法确认当前模式，请先确认机器人已上电")
        if st.get("current_motion_task") != "joint_bridge":
            return _err("WRONG_MOTION_STATE",
                        f"当前运动状态为 {st.get('current_motion_task')}，关节透传需要 "
                        f"joint_bridge（关节透传）模式，请先用 motion_switcher 切换。"
                        f"危险操作，请确保安全吊架/清场")
        return None

    def _publish_cmd(self, positions: list) -> None:
        if self._cmd_pub is None or self._JointCommand is None or self._Header is None:
            return
        try:
            cmd = self._JointCommand()
            cmd.header = self._Header()
            cmd.header.stamp = self._node.get_clock().now().to_msg()
            cmd.header.frame_id = ""
            cmd.position = [float(p) for p in positions]
            cmd.velocity = [0.0] * JOINT_COUNT
            cmd.feed_forward_torque = [0.0] * JOINT_COUNT
            cmd.torque = [0.0] * JOINT_COUNT
            cmd.stiffness = self._stiffness
            cmd.damping = self._damping
            cmd.parallel_parser_type = 0    # CLASSIC_PARSER
            self._cmd_pub.publish(cmd)
        except Exception as e:  # noqa: BLE001
            print(f"[JointBridgePlugin] 发布异常: {e}")

    def _tick(self) -> None:
        """500Hz 定时器：按插值轨迹逐拍发布 JointCommand。"""
        with self._lock:
            if self._interp is None:
                return
            idx = self._step_idx
            last_idx = self._num_steps - 1
            positions = [self._interp[j][idx if idx < last_idx else last_idx]
                         for j in range(JOINT_COUNT)]
            done = idx >= last_idx
            if not done:
                self._step_idx = idx + 1
        self._publish_cmd(positions)
        if done:
            with self._lock:
                self._interp = None
                if self._timer is not None:
                    try:
                        self._timer.cancel()
                    except Exception:  # noqa: BLE001
                        pass
                    self._timer = None
            print("[JointBridgePlugin] 插值完成，已停止发布")

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self.start()
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            with self._lock:
                running = self._interp is not None
                step = self._step_idx
            return {"state": "running" if running else "ready",
                    "interp_step": step, "enabled": self._enabled}
        if action == "set_pose":
            if not self._enabled:
                return _err("DISABLED",
                            "joint_bridge 卡片默认关闭（config.yaml enabled: false），"
                            "仅限专业人员启用后使用")
            # 先做参数/数组长度校验（快速失败，避免被状态错误掩盖）
            if "positions" not in args:
                return _err("INVALID_ARGUMENT", "缺少 positions 参数（25 个关节目标位置）")
            positions = [float(x) for x in args["positions"]]
            if len(positions) != JOINT_COUNT:
                return _err("INVALID_LENGTH",
                            f"positions 长度必须为 {JOINT_COUNT}（J00~J24），当前 {len(positions)}")
            stiffness = args.get("stiffness")
            damping = args.get("damping")
            if stiffness is not None:
                stiffness = [float(x) for x in stiffness]
                if len(stiffness) != JOINT_COUNT:
                    return _err("INVALID_LENGTH",
                                f"stiffness 长度必须为 {JOINT_COUNT}，当前 {len(stiffness)}")
            else:
                stiffness = list(_JB_DEFAULT_STIFFNESS)
            if damping is not None:
                damping = [float(x) for x in damping]
                if len(damping) != JOINT_COUNT:
                    return _err("INVALID_LENGTH",
                                f"damping 长度必须为 {JOINT_COUNT}，当前 {len(damping)}")
            else:
                damping = list(_JB_DEFAULT_DAMPING)
            try:
                num_steps = max(2, int(args.get("num_steps", 1500)))
            except (TypeError, ValueError):
                return _err("INVALID_ARGUMENT", "num_steps 必须是整数")
            # 运动状态前置检查（要求 joint_bridge 模式）
            err = self._check_precondition()
            if err:
                return err
            with self._lock:
                cur = list(self._joint_positions) if self._joint_positions else None
            if cur is None or len(cur) != JOINT_COUNT:
                return _err("NO_STATE",
                            "尚未收到 /hardware/joint_state（25 关节位置），无法确定插值起点")
            # 官方公式: 步长 = (目标 - 当前) / (步数 - 1)
            interp = []
            for i in range(JOINT_COUNT):
                delta = (positions[i] - cur[i]) / (num_steps - 1)
                interp.append([cur[i] + delta * j for j in range(num_steps)])
            with self._lock:
                self._interp = interp
                self._num_steps = num_steps
                self._step_idx = 0
                self._stiffness = stiffness
                self._damping = damping
                if self._timer is None and self._node is not None:
                    self._timer = self._node.create_timer(self.CONTROL_PERIOD, self._tick)
            print(f"[JointBridgePlugin] set_pose 开始: {num_steps} 步 @500Hz")
            return {"state": "ok", "num_steps": num_steps,
                    "positions": positions, "safety": "确认安全吊架/清场！"}
        return None


__all__ = [
    "MotionStatePlugin",
    "MotionSwitcherPlugin",
    "LocoPlugin",
    "ArmMotionPlugin",
    "JointBridgePlugin",
]
