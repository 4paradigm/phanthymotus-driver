#!/usr/bin/env python3
"""
engineai/t800/ros2.py — 众擎 T800 开发版 driver 双 rclpy context 管理（骨架）。

数据流：
  ctx_t800  (domain 69, rmw_cyclonedds_cpp, ROS_LOCALHOST_ONLY=0)
            —— 直连 T800 机器人运控单元，订阅/发布机器人话题
            （/motion/*、/hardware/*）
  ctx_core  (domain 42, rmw_fastrtps_cpp)
            —— 与 agent-core / dashboard / perception 通信，
            传感器转发插件通过 core 节点发布 /{ns}/xxx（std_msgs/String JSON）

进程内 RMW 切换：每个 context 创建前设置环境变量
（RMW_IMPLEMENTATION / ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY），
并在 rclpy.init 的 args 中显式传 __rmw 与 __domain_id remap 双重保证，
创建完成后恢复原环境变量（与 x-humanoid/tianyi2.0 双 context 模式一致）。

模块级只 import 标准库；rclpy 相关导入全部在方法内延迟进行，
保证本模块在没有 ROS2 环境的机器上可被纯 import 测试。

对外常量：
  T800_TOPICS            —— 机器人侧话题名表（domain 69）
  QOS_T800_RELIABLE      —— depth1 RELIABLE VOLATILE
  QOS_T800_BEST_EFFORT   —— depth1 BEST_EFFORT VOLATILE
  QOS_T800_JOINT         —— depth3 BEST_EFFORT VOLATILE（joint_state/joint_command 官方用）
  QOS_CORE               —— depth10 BEST_EFFORT VOLATILE（core 域发布）
  QOS_DEFAULT            —— depth10 默认 QoS（body_vel_cmd / led_control 官方用）
"""

from __future__ import annotations

import os
import threading

# ── 双域常量 ──────────────────────────────────────────────────────────────────

T800_DOMAIN_ID = 69   # 机器人侧（运控单元）
CORE_DOMAIN_ID = 42   # core 侧（agent-core / dashboard / perception）

# 机器人侧话题（domain 69），消息类型见 interface_protocol/msg/*.msg
T800_TOPICS = {
    "motion_state":        "/motion/motion_state",              # MotionState
    "set_motion_state":    "/motion/set_motion_state",          # MotionStateRequest
    "joint_state":         "/hardware/joint_state",             # JointState
    "joint_command":       "/hardware/joint_command",           # JointCommand
    "body_vel_cmd":        "/motion/body_vel_cmd",              # BodyVelCmd
    "motion_plan_request": "/motion/joint_motion_plan/request", # JointMotionPlanRequest
    "motion_plan_state":   "/motion/joint_motion_plan/state",   # JointMotionPlanState
    "led":                 "/hardware/led_control",             # LedControl
    "imu":                 "/hardware/imu_info",                # ImuInfo
    "power":               "/hardware/power_info",              # PowerInfo
    "gamepad":             "/hardware/gamepad_keys",            # GamepadKeys
}

# ── 惰性 QoS 常量（首次访问时创建，避免模块级导入 rclpy）─────────────────────

_QOS_PROFILES: dict = {}


def _build_qos_profiles() -> dict:
    """按官方例程（interface_example/scripts）创建 QoSProfile 常量表。"""
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    return {
        # 官方规定：/motion/set_motion_state、/motion/joint_motion_plan/request 发布用
        "QOS_T800_RELIABLE": QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        ),
        # 官方规定：/motion/motion_state、/motion/joint_motion_plan/state 订阅用
        "QOS_T800_BEST_EFFORT": QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        ),
        # 官方 joint 例程：/hardware/joint_state、/hardware/joint_command 用 depth 3
        "QOS_T800_JOINT": QoSProfile(
            depth=3,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        ),
        # core 域发布（/{ns}/xxx）：depth 10 BEST_EFFORT VOLATILE
        "QOS_CORE": QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        ),
        # 官方默认 QoS（/motion/body_vel_cmd、/hardware/led_control 直接用 depth 10）
        "QOS_DEFAULT": QoSProfile(depth=10),
    }


def __getattr__(name: str):
    """PEP 562 惰性暴露 QOS_* 常量：首次访问时才 import rclpy 构造 QoSProfile。"""
    global _QOS_PROFILES
    if name in ("QOS_T800_RELIABLE", "QOS_T800_BEST_EFFORT",
                "QOS_T800_JOINT", "QOS_CORE", "QOS_DEFAULT"):
        if not _QOS_PROFILES:
            _QOS_PROFILES = _build_qos_profiles()
        return _QOS_PROFILES[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


# ── 双 context 管理 ───────────────────────────────────────────────────────────


class Ros2Contexts:
    """T800 双 rclpy context 管理。

    - ctx_t800：domain 69 + rmw_cyclonedds_cpp，直连机器人话题
    - ctx_core：domain 42 + rmw_fastrtps_cpp，与 agent-core 通信
    每个 context 配一个 SingleThreadedExecutor，start() 起两个 daemon spin 线程。
    """

    def __init__(self, namespace: str):
        self.namespace = namespace
        self.ctx_t800 = None
        self.ctx_core = None
        self.executor_t800 = None
        self.executor_core = None
        self._spin_threads: list = []
        self._rclpy = None
        self._init_contexts()

    # ── 初始化 ─────────────────────────────────────────────────────────────

    def _init_contexts(self) -> None:
        """按 context 切换 RMW 实现并分别 rclpy.init（环境变量方案，与 tianyi2.0 一致）。"""
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        self._rclpy = rclpy

        # 保存原环境变量，context 创建后恢复，避免影响进程内其他组件
        saved = {
            k: os.environ.get(k)
            for k in ("RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")
        }
        try:
            # ctx_t800：domain 69 + CycloneDDS（机器人侧）
            os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
            os.environ["ROS_DOMAIN_ID"] = str(T800_DOMAIN_ID)
            os.environ["ROS_LOCALHOST_ONLY"] = "0"
            self.ctx_t800 = Context()
            rclpy.init(
                args=["--ros-args", "-r", f"__rmw:=rmw_cyclonedds_cpp",
                      "-r", f"__domain_id:={T800_DOMAIN_ID}"],
                context=self.ctx_t800,
            )
            self.executor_t800 = SingleThreadedExecutor(context=self.ctx_t800)

            # ctx_core：domain 42 + FastDDS（core 侧；本地回环约束不强制）
            os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
            os.environ["ROS_DOMAIN_ID"] = str(CORE_DOMAIN_ID)
            os.environ.pop("ROS_LOCALHOST_ONLY", None)
            self.ctx_core = Context()
            rclpy.init(
                args=["--ros-args", "-r", f"__rmw:=rmw_fastrtps_cpp",
                      "-r", f"__domain_id:={CORE_DOMAIN_ID}"],
                context=self.ctx_core,
            )
            self.executor_core = SingleThreadedExecutor(context=self.ctx_core)
        finally:
            for k, v in saved.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v

    # ── 节点创建 ───────────────────────────────────────────────────────────

    def make_node_t800(self, name: str):
        """在 ctx_t800（domain 69 / CycloneDDS）创建节点并加入 executor_t800。"""
        node = self._rclpy.node.Node(name, context=self.ctx_t800)
        self.executor_t800.add_node(node)
        return node

    def make_node_core(self, name: str):
        """在 ctx_core（domain 42 / FastDDS）创建节点并加入 executor_core。"""
        node = self._rclpy.node.Node(name, context=self.ctx_core)
        self.executor_core.add_node(node)
        return node

    # ── 生命周期 ───────────────────────────────────────────────────────────

    def start(self) -> None:
        """两个 executor 各起 daemon 线程 spin。"""
        def _spin(executor, label: str) -> None:
            try:
                while self._rclpy.ok(context=executor.context):
                    executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                print(f"[ros2] {label} spin error: {e}", flush=True)
            print(f"[ros2] {label} spin exited", flush=True)

        t1 = threading.Thread(target=_spin, args=(self.executor_t800, "t800(69)"), daemon=True)
        t2 = threading.Thread(target=_spin, args=(self.executor_core, "core(42)"), daemon=True)
        t1.start()
        t2.start()
        self._spin_threads = [t1, t2]

    def shutdown(self) -> None:
        """销毁节点、executor 与 context（幂等，异常吞掉）。"""
        for ex in (self.executor_t800, self.executor_core):
            if ex is None:
                continue
            # 尽力销毁已注册节点（插件 stop() 已销毁的节点此处会静默跳过）
            for node in list(getattr(ex, "_nodes", []) or []):
                try:
                    node.destroy_node()
                except Exception:
                    pass
            try:
                ex.shutdown()
            except Exception:
                pass
        if self._rclpy is not None:
            for ctx in (self.ctx_t800, self.ctx_core):
                try:
                    self._rclpy.shutdown(context=ctx)
                except Exception:
                    pass
        print("[ros2] contexts shut down", flush=True)
