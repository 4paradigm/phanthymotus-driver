#!/usr/bin/env python3
"""Continuous joint control for the RM75 — the first consumer of motus.control/1.

`joint_control` in device.py is the call-shaped path: one absolute target per
`tools/call`, an ACP completion, a `confirm_motion` per request. That is right
for an LLM deciding to move an arm somewhere, and wrong for an execution model,
which produces tens of commands per second and is not asking a question each
time.

This card is the stream-shaped path. It subscribes to a `control/joint` topic,
runs every message through `common.control.ControlSink`, and passes what
survives to the RealMan passthrough API (`rm_movej_canfd`). The arm's safety
properties come from the sink, not from this file: freshness, arbitration
between sources, step clamping, hard limits, watchdog, and escalation all live
there and are tested without a robot.

What is specific to this arm, and therefore here:

- **Units.** The descriptor is in radians, matching the skeleton publisher this
  driver already produces. The SDK passthrough takes degrees. The conversion
  happens in one place, at the boundary, immediately before the SDK call.
- **The motion gate.** `RM_MOTION_ENABLED` and an explicit confirmation guard
  every movement in this driver. A card that streams commands does not get to
  skip that because the commands are frequent — it is checked once at `start`,
  which is the point where a human is present.
- **Stopping.** `rm_set_arm_slow_stop` is what the watchdog and abort paths
  call, the same call the existing plugin's interrupt hook uses.
- **The gripper is part of the action space, not a separate card.** See below.
- **Proprioception comes out of this card**, on the same `state/joint` topic a
  VLA binds as its observation input. See `_publish_state`.
- **Six-axis force feeds the sink's abort.** RM75-6F-V has an integrated F/T
  sensor, so chain step 7 is real here rather than declared absent. See
  `_sample_force`.

## 夹爪是第 8 维，不是旁路字段

`device.py` 的 `GripperPlugin` 是 call-shaped 的那条路：`block=True`、最长 30 s、
一次一个动作。执行模型在同一拍输出「手移到这里 + 夹爪闭合」，那条路接不上 ——
所以夹爪在这里是 `values` 的第 8 维，和 G1 (`unitree/g1/servo.py`)、天轶
(`x-humanoid/tianyi2.0/servo.py`) 同一个做法。

两张卡不会打架：`_start` 把 `client.motion_gate` 持有到卡片停止，于是 call-shaped
的那张会被响亮地拒掉（"another arm operation is active"）。

**极性：闭合度，0 = 张开，1 = 闭合。** 和天轶一致 —— 它是这个仓库里唯一明确声明过
极性并且有测试钉住往返的驱动。SDK 的 `position` 是**开口度** 1~1000，方向相反
（`rm_gripper_state_t.mode` 的枚举：`1 夹爪张开到最大`、`2 夹爪闭合到最小`），换算
见 `closure_to_position` / `position_to_closure`，两者互为逆运算且有测试钉住。

**模型那一维到底是闭合度还是开口度，协商层验证不了。** `motus.vla/1` 对夹爪是原样
透传的，而且这是有意的 —— `phanthymotus-cloud/runtimes/common/normalize.py` 写着
「把它归一化到 0..1 需要知道这只夹爪的完整行程，而那是机器人的知识，不是模型的」。
所以接反了不会报错：在物体周围执行「张开」会变成「夹紧」，也就是危险的那个方向。
**本文件的极性尚未在真机上验证过**，第一次上真机要先看一眼夹爪的实际开合。
G1 至今留着同一个未解决的矛盾（`servo.py` 声明 0..1，`servo_eef.py` 按 checkpoint
的统计量是 0..4.5）。
"""

from __future__ import annotations

import json
import math
import threading
import time

from common.control import ControlSink, Verdict, parse_descriptor, rotation
from common.vendor_runtime import action_schema, tool

from device import (GRIPPER_POSITION_MAX, GRIPPER_POSITION_MIN, JOINT_LIMITS_DEG,
                    JOINT_MAX_SPEED_DEG_S, JOINT_NAMES)

# Passthrough rate. The vendor requires a period under 10 ms for high-follow
# mode; we run low-follow at a much calmer rate, so `follow=False`.
DEFAULT_EXPECTED_HZ = 30.0
MAX_HZ = 100.0
WATCHDOG_MS = 200
# An observation older than this cannot be acted on. Remote inference routinely
# produces a command generated just now from a much older picture; see
# README_dev § "Continuous Control".
MAX_OBS_AGE_MS = 300

# 归一化闭合度每秒的变化上限。和手臂的限速同一个性质：取一个保守的、说得出理由
# 的数，而不是把硬件行程当成许可。2.0 = 半秒走完全行程,与 G1 那张卡一致。
GRIPPER_MAX_VELOCITY = 2.0
# 夹爪是控制器后面的串行设备，不是一个 30 Hz 的关节。这两个值把 30 Hz 的指令流
# 降到它能消化的速率，并且**都要进 `info()`** —— README_dev 的 "no silent caps"。
DEFAULT_GRIPPER_DEADBAND = 0.02        # ≈ 20 counts of the 1~1000 SDK range
DEFAULT_GRIPPER_MAX_HZ = 10.0

# 本体状态的发布频率。对齐 config.yaml 的 `ros.skeleton_publish_hz`；VLA 的
# `max_obs_age_ms` 是 300，所以 100 ms 很宽裕。比这更快没有收益，而每一拍都是
# 两次 SDK 往返，走的是和 30 Hz 透传同一把 `hardware.py:_lock`。
DEFAULT_STATE_HZ = 10.0
# 六维力的采样频率。和控制回路同阶，不跟着 state 走 —— 10 Hz 意味着撞上之后
# 手臂还能再走 100 ms。
DEFAULT_FORCE_HZ = 30.0

# **阈值是猜的，这里说出来。** 厂商文档给的是额定力 200 N、额定力矩 8 Nm
# (`vendor/Robotic_Arm/rm_robot_interface.py` 的 `Force` 类)。这两个默认值刻意
# **偏松**：偏紧会让手臂不断 abort，而那正是一道保护被人关掉的典型路径；默认
# 关闭更糟 —— 关掉的保护没人会去开。真机上第一次用要按 README 的流程收紧。
DEFAULT_FORCE_N = 50.0
DEFAULT_TORQUE_NM = 5.0


def closure_to_position(closure: float) -> int:
    """闭合度 (0 张开 … 1 闭合) → SDK 的开口度 1~1000。

    方向是反的，这是整个换算的全部内容，也是唯一会接反的地方。
    `position_to_closure` 必须是它的严格逆运算 —— 两者放在同一个文件里，并且有
    一条往返测试钉住，理由见模块 docstring。
    """
    clamped = max(0.0, min(1.0, float(closure)))
    span = GRIPPER_POSITION_MAX - GRIPPER_POSITION_MIN
    return int(round(GRIPPER_POSITION_MAX - clamped * span))


def position_to_closure(position) -> float:
    """SDK 的开口度 1~1000 → 闭合度 (0 张开 … 1 闭合)。"""
    span = GRIPPER_POSITION_MAX - GRIPPER_POSITION_MIN
    closure = (GRIPPER_POSITION_MAX - float(position)) / span
    return max(0.0, min(1.0, closure))


def build_descriptor(expected_hz: float = DEFAULT_EXPECTED_HZ,
                     gripper: bool = True,
                     force_torque=None) -> dict:
    """The action space this arm accepts: 7 joints in radians + 1 gripper.

    Derived from the same vendor limits `joint_control` enforces, so the two
    paths cannot disagree about what this arm can do.

    **`gripper=False` really is a 7-dimension card**, not an 8-dimension one
    that quietly ignores the last value. Keeping 8 would let a model with a
    gripper negotiate successfully and then have that degree of freedom go
    nowhere — the exact silent failure `dof` exists to catch. An RM75 whose
    gripper is offline is a different action space, and saying so is what lets
    the model be refused instead of half-executed. (Which it is gets decided by
    asking the robot, in `detect_gripper`, not by trusting the config.)

    `force_torque` is a 6-list of per-axis absolute thresholds, or None for
    "this driver is not reading the sensor". It is a parameter rather than a
    constant because an uncalibrated sensor must not be presented as a working
    protection — see `_sample_force`.
    """
    joint_names = list(JOINT_NAMES)
    lower = [math.radians(low) for low, _ in JOINT_LIMITS_DEG]
    upper = [math.radians(high) for _, high in JOINT_LIMITS_DEG]
    max_velocity = [math.radians(speed) for speed in JOINT_MAX_SPEED_DEG_S]
    groups = [{"name": "arm", "offset": 0, "count": len(JOINT_NAMES),
               "unit": "rad", "resource": "arm"}]
    if gripper:
        joint_names += ["gripper"]
        lower += [0.0]
        upper += [1.0]
        max_velocity += [GRIPPER_MAX_VELOCITY]
        groups += [{"name": "gripper", "offset": len(JOINT_NAMES), "count": 1,
                    "unit": "normalized", "resource": "gripper"}]
    # One step may not exceed what the joint could travel in one period at its
    # rated speed. Anything larger is a jump the arm cannot follow anyway; the
    # sink clamps it rather than refusing, because it is usually one noisy
    # sample rather than a mistake.
    period = 1.0 / expected_hz
    max_delta = [speed * period for speed in max_velocity]
    return {
        "control_interface": "motus.control/1",
        "mode": "joint_position",
        "dof": len(joint_names),
        "joint_names": joint_names,
        # 混合单位是必然的，`groups` 才是说清楚哪段是哪种的地方。
        "units": ({"angle": "rad", "normalized": "0-1", "time": "s"}
                  if gripper else {"angle": "rad", "time": "s"}),
        "limits": {
            "lower": lower,
            "upper": upper,
            "max_velocity": max_velocity,
            "max_delta_per_step": max_delta,
        },
        "groups": groups,
        "frame": "base_link",
        "rate": {
            "max_hz": MAX_HZ,
            "expected_hz": expected_hz,
            "watchdog_ms": WATCHDOG_MS,
            "max_obs_age_ms": MAX_OBS_AGE_MS,
        },
        # None means "this driver is not reading the sensor", and it stays
        # declared rather than omitted so that a missing protection is visible
        # instead of assumed. RM75-6F-V *has* a six-axis sensor, so on this arm
        # a None here is a statement about the driver or the calibration, never
        # about the hardware. See common/control/descriptor.py.
        "force_torque": list(force_torque) if force_torque is not None else None,
        "urdf_ref": "model",
    }


def detect_gripper(state) -> str:
    """机器上到底有没有一只能用的夹爪。返回 `""`（没有）或原因名。

    **问机器人，不问配置** —— 和 G1 的 `arm_sdk.detect_dex1` 同一条规矩，理由也
    一样：配置是人填的，而这里填错不报错。少这一维不会有任何异常，卡片照样声明
    8 维、照样把第 8 个数发给一只不存在的夹爪。

    `state` 是 `rm_get_gripper_state()` 的字典。`status` 是在线状态（0 离线），
    `enable_state` 是使能标志（0 未使能）—— 两个都要，一只在线但没使能的夹爪
    收下指令之后不会动。
    """
    if not isinstance(state, dict):
        return ""
    if int(state.get("status", 0) or 0) != 1:
        return ""
    if int(state.get("enable_state", 0) or 0) != 1:
        return ""
    return "online"


class RM75ServoPlugin:
    """Stream-shaped joint control. One card, one input topic, one sink."""

    PREFIX = "servo"

    def __init__(self, client, config, namespace="rm75", ros2=None):
        self.client = client
        self._ros2 = ros2
        self._namespace = namespace.strip("/") or "rm75"
        servo_config = config.get("servo", {}) or {}
        self._expected_hz = float(servo_config.get("expected_hz", DEFAULT_EXPECTED_HZ))
        if not math.isfinite(self._expected_hz) or not 0 < self._expected_hz <= MAX_HZ:
            raise ValueError(f"servo.expected_hz must be in (0, {MAX_HZ}]")

        self._gripper_configured = bool(servo_config.get("gripper", True))
        self._gripper_deadband = float(
            servo_config.get("gripper_deadband", DEFAULT_GRIPPER_DEADBAND))
        self._gripper_max_hz = float(
            servo_config.get("gripper_max_hz", DEFAULT_GRIPPER_MAX_HZ))
        if not math.isfinite(self._gripper_deadband) or self._gripper_deadband < 0:
            raise ValueError("servo.gripper_deadband must be a non-negative number")
        if not math.isfinite(self._gripper_max_hz) or self._gripper_max_hz <= 0:
            raise ValueError("servo.gripper_max_hz must be greater than 0")
        # 看门狗/abort 时松不松夹爪。默认**不松** —— 松开会把手里的东西扔到下面
        # 的任何东西上，而这个文件无从判断「扔掉」和「继续握着一个没人在决定的
        # 东西」哪个更糟。天轶的 `release_hands_on_watchdog` 同一条推理。
        self._release_gripper_on_watchdog = bool(
            servo_config.get("release_gripper_on_watchdog", False))

        self._state_hz = float(servo_config.get("state_hz", DEFAULT_STATE_HZ))
        self._force_hz = float(servo_config.get("force_hz", DEFAULT_FORCE_HZ))
        if not math.isfinite(self._state_hz) or self._state_hz <= 0:
            raise ValueError("servo.state_hz must be greater than 0")
        if not math.isfinite(self._force_hz) or self._force_hz <= 0:
            raise ValueError("servo.force_hz must be greater than 0")
        self._force_thresholds = self._parse_force_config(
            servo_config.get("force_torque", {}))

        # 启动时才知道的两件事：夹爪在不在（问机器人）、六维力读数可不可信。
        # 所以 descriptor 在 `_start` 里按探测结果重建；这里先按配置建一份，
        # 因为画布在 start **之前**就要读 `get_tools()` / `info()`。
        self._gripper_enabled = self._gripper_configured
        self._force_enabled = False
        self._force_note = "not sampled yet"
        self._rebuild_descriptor()

        self._state_topic = f"/{self._namespace}/servo/state"

        # `start`/`stop`/`config` arrive on separate threads (ThreadingHTTPServer).
        # The lock guards the bookkeeping only — never a start(), a stop() or an
        # SDK call, or a stop would queue behind the start it is meant to cancel.
        self._lock = threading.RLock()
        self._sink: ControlSink | None = None
        self._node = None
        self._input_topic = ""
        self._running = False
        # 抢到了运动门、但还在探测硬件的那段窗口。见 `_start`。
        self._starting = False
        self._last_outcome: dict | None = None
        self._rejects: list[str] = []
        # Paused means subscribed but not applying. See `_halt`.
        self._paused = False
        self._motion_gate_held = False

        # 夹爪的单槽信箱：最新值覆盖。`_apply` 只写它，worker 线程读它。
        # 为什么不内联见 `_apply`。
        self._gripper_lock = threading.Lock()
        self._gripper_wanted: float | None = None
        self._gripper_sent: float | None = None
        self._gripper_sent_ms = 0.0
        self._gripper_wake = threading.Event()
        self._gripper_thread = None
        self._gripper_stop = threading.Event()
        self._gripper_error = ""

        # 本体状态 + 六维力的轮询
        self._poll_thread = None
        self._poll_stop = threading.Event()
        self._state_pub = None
        self._last_state = None
        self._last_force = None
        self._poll_error = ""

    # ── 配置 ─────────────────────────────────────────────────────────────────

    @staticmethod
    def _parse_force_config(raw):
        """`servo.force_torque` → 6 个阈值，或 None（= 不读传感器）。

        接受三种写法：`false`/`null` 关闭；一个 6 元列表；或
        `{force_n: …, torque_nm: …}` 两个标量（最常用，因为前三轴是力、后三轴
        是力矩，单位不同而同类的三轴没有理由各写一个数）。
        """
        if raw is None or raw is False:
            return None
        if isinstance(raw, (list, tuple)):
            values = [float(v) for v in raw]
            if len(values) != 6:
                raise ValueError(
                    f"servo.force_torque as a list must have 6 entries "
                    f"(Fx Fy Fz Mx My Mz), got {len(values)}")
        else:
            config = raw if isinstance(raw, dict) else {}
            if config.get("enabled") is False:
                return None
            force = float(config.get("force_n", DEFAULT_FORCE_N))
            torque = float(config.get("torque_nm", DEFAULT_TORQUE_NM))
            values = [force] * 3 + [torque] * 3
        for value in values:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    "servo.force_torque thresholds must all be finite and > 0")
        return values

    def _rebuild_descriptor(self):
        """按当前探测结果重建 descriptor。`_start` 在探测之后调它。"""
        self._descriptor_raw = build_descriptor(
            self._expected_hz,
            gripper=self._gripper_enabled,
            force_torque=self._force_thresholds if self._force_enabled else None,
        )
        self._descriptor = parse_descriptor(self._descriptor_raw)

    # ── tools ────────────────────────────────────────────────────────────────

    def get_tools(self):
        schema = action_schema(
            {
                "pause": ([], "Stop driving the arm and hold the current pose, "
                              "staying subscribed; resume continues"),
                "resume": ([], "Continue applying commands"),
                # `reset` 是给模型的，和 pause/resume 同一类：撞到东西之后 sink
                # 锁存，而只有看过现场的那一方能决定可以继续。它**不是**一个更
                # 软的 resume —— 锁存没清之前 resume 什么都恢复不了。
                "reset": ([], "Clear a force-torque abort after the contact has "
                              "been dealt with; does nothing otherwise"),
            },
            {
                "input_topic": {"type": "string",
                                "description": "control/joint topic to consume"},
            },
        )
        # `start`/`stop`/`info` are deliberately absent from the action list
        # above, and that is what decides who may call them: agent-core splits a
        # tool into one LLM-callable function per action (mcp_client.py
        # `_to_openai_schema`), so an action not listed is reachable by the
        # canvas and not by the model. `stop` belongs to the project lifecycle —
        # a model calling it would take this card out of a running project
        # without the project knowing, and could not put it back, since `start`
        # needs the input topic and the downstream descriptor that only
        # agent-core has.
        #
        # `pause` is therefore the model's halt, and what the interrupt hooks
        # fire. It is not a weaker stop: the arm stops just as immediately. It
        # differs in staying subscribed, so the project's view of what is
        # running stays true and `resume` can continue.
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "pause"},
                             "on_interrupt_all": {"action": "pause"}}
        schema["x-is-dangerous"] = True
        # No x-completion: this card has no end. It streams until stopped, so
        # there is nothing for an ACP pending action to wait for — holding one
        # open would block every other actuator for as long as the card runs.
        schema["x-resource"] = "arm"
        width = self._descriptor.dof
        return [
            tool(
                "servo",
                "actuator",
                "Continuous RM75 joint control from a motus.control/1 stream "
                f"(passthrough at up to {self._expected_hz:g} Hz)",
                schema,
                topic_in=[{"format": "control/joint",
                           "desc": f"motus.control/1, {width} 维"
                                   + (" (7 臂 rad + 1 夹爪 闭合度)"
                                      if self._gripper_enabled else " (7 臂 rad)")}],
                # 本体状态和它接受的指令出自**同一份** build_descriptor()：同样的
                # 关节顺序、同样的单位、同样的极性。想只改一边做不到，因为它们是
                # 同一个列表 —— 这正是把状态放在这张卡上、而不是另建一张卡的理由。
                #
                # topic 在这里声明而不是等启动后再报：画布上这条线要接回 vla 卡，
                # 形成 vla → servo →(state)→ vla 的回环，而环里谁都不能从「已经
                # 启动的上游」学到自己的输入话题，持久化的声明是唯一剩下的东西。
                topic_out=[{"topic": self._state_topic, "format": "state/joint",
                            "desc": f"{width} 维本体状态，与指令同序同单位；"
                                    "带 eef 当前末端位姿（增量模型的基准）"}],
            )
        ]

    def dispatch(self, action, args):
        if action == "start":
            return self._start(args)
        if action == "stop":
            return self._stop()
        if action == "pause":
            return self._halt(True)
        if action == "resume":
            return self._halt(False)
        if action == "reset":
            return self._reset()
        if action == "info":
            return self._info()
        return None

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Bundle lifecycle. Deliberately does nothing.

        A card that streams motion must not come up streaming because the
        container restarted. It starts when agent-core starts the project,
        which is an operator action, and not before.
        """

    def stop(self):
        self._stop()

    # ── actions ──────────────────────────────────────────────────────────────

    def _start(self, args):
        if not self.client.motion_enabled:
            return {"state": "error",
                    "message": "RM_MOTION_ENABLED is not set; this driver is read-only"}
        # Arguments before environment: a caller who left out the topic should
        # be told that, whether or not the arm happens to be plugged in.
        topic = (args.get("input_topic") or "").strip()
        if not topic:
            topic = (args.get("input_topics") or [""])[0].strip()
        if not topic:
            return {"state": "error",
                    "message": "input_topic is required — wire a control/joint "
                               "source to this card on the canvas"}

        if not self.client.connected:
            return {"state": "error", "message": "RM75 SDK is not connected"}
        if self._ros2 is None:
            return {"state": "error", "message": "no ROS context; cannot subscribe"}

        # **抢到运动门之后才碰 SDK。** 探测要读控制器，而「另一张卡正持有这条
        # 手臂」必须在任何一次 SDK 调用之前就判出来 —— 顺序反了的话，探测会在
        # 别人正在用的手臂上发查询，而失败信息说的是「读不到夹爪状态」，指向
        # 夹爪而不是指向那张卡。
        with self._lock:
            if self._running or self._starting:
                return {"state": "error", "message": f"already running on {self._input_topic}"}
            if not self.client.motion_gate.acquire(blocking=False):
                return {"state": "error", "message": "another arm operation is active"}
            # `_starting` 而不是 `_running`：这段窗口里卡片还没有 sink，报
            # `running` 会让 `info()` 说一张跑着的卡而 `sink` 是 null。它同时
            # 挡住重复 start 泄漏共享的门。
            self._starting = True
            self._motion_gate_held = True

        # 探测在**锁外**：它是若干次 SDK 往返，而这把锁只护簿记。持着它做 SDK
        # 调用会让一个 stop 排在探测后面，而 stop 恰恰是要取消它的那个。
        try:
            failure = self._probe_hardware()
        except Exception as exc:
            failure = {"state": "error",
                       "message": f"hardware probe failed: {type(exc).__name__}: {exc}"}

        # descriptor 的宽度和 force_torque 都由探测结果决定，而 sink 把
        # descriptor 当不变量缓存（frozen，理由见 common/control/descriptor.py），
        # 所以 sink 必须在探测之后才建 —— 否则是拿一份过期的声明去检查每一条指令。
        sink = None if failure is not None else ControlSink(
            self._descriptor,
            self._apply,
            on_watchdog=self._on_watchdog,
            on_abort=self._on_watchdog,
        )
        with self._lock:
            self._starting = False
            if failure is None and not self._motion_gate_held:
                # 探测期间有人调了 stop：门已经被它放掉了。不接着启动，也不能
                # 再放一次 —— 第二次 release 会把门送给一张没有申请过的卡片。
                failure = {"state": "error", "message": "start cancelled by stop"}
            if failure is not None:
                if self._motion_gate_held:
                    self._motion_gate_held = False
                    self.client.motion_gate.release()
                return failure
            self._sink = sink
            self._input_topic = topic
            self._running = True
            self._paused = False
            try:
                self._subscribe(topic)
            except Exception as exc:
                self._running = False
                self._sink = None
                self._input_topic = ""
                self._motion_gate_held = False
                self.client.motion_gate.release()
                return {"state": "error", "message": f"subscribe failed: {exc}"}

        # 两个 worker 在订阅成功之后才起：起早了它们会对着一张还没在跑的卡片
        # 发指令、发状态。
        self._start_workers()

        print(f"[rm75] servo streaming from {topic}", flush=True)
        return {"state": "running", "input": topic,
                "gripper": self._gripper_enabled,
                "force_torque": self._force_note,
                "control_interface": self._descriptor_raw}

    def _probe_hardware(self):
        """问机器人夹爪在不在、六维力读数可不可信，然后重建 descriptor。

        返回 None 表示可以继续，返回一个 error 字典表示拒绝启动。
        """
        gripper_detected = ""
        if self._gripper_configured:
            try:
                state = self.client.call("rm_get_gripper_state")
            except Exception as exc:
                return {"state": "error",
                        "message": f"cannot read the gripper state: {exc}. "
                                   "Set servo.gripper to false to run this card "
                                   "as a 7-dimension arm-only action space"}
            gripper_detected = detect_gripper(state)
            if not gripper_detected:
                # **拒绝，而不是降级。** 悄悄降成 7 维会让一个带夹爪的模型协商
                # 通过，然后那一维去向虚空 —— 和 G1 那条拒绝分支同一个理由。
                return {"state": "error",
                        "message": "servo.gripper is true but this arm reports no "
                                   f"usable gripper (state={state!r}; status must be 1 "
                                   "and enable_state must be 1). Set servo.gripper to "
                                   "false — that makes this a 7-dimension card, so a "
                                   "model with a gripper fails negotiation instead of "
                                   "being half-executed"}
        self._gripper_enabled = bool(gripper_detected)

        self._force_enabled, self._force_note = self._probe_force()
        self._rebuild_descriptor()
        return None

    def _probe_force(self):
        """六维力能不能当一道保护用。返回 `(enabled, 人读的理由)`。

        **不自动标零。** `rm_clear_force_data` 会把当前状态定为零位，而在手臂
        已经碰着东西的时候标零等于把那个接触烘焙成零点 —— 之后真正的碰撞反而
        读不出来。标零是操作员在确认手臂悬空之后做的事，不是启动流程的一部分。

        这里只做一件事：读一次，确认读数**像**一个已标零的传感器。读不出来或者
        六轴全零，就不启用并把原因说出来 —— 一道假的保护比没有保护更糟，因为
        descriptor 会宣称它存在。
        """
        if self._force_thresholds is None:
            return False, "disabled by servo.force_torque"
        try:
            readings = self._read_force()
        except Exception as exc:
            return False, f"not enabled: cannot read the sensor ({exc})"
        if readings is None:
            return False, "not enabled: rm_get_force_data has no zero_force_data"
        if all(abs(value) < 1e-9 for value in readings):
            # 全零可能是真的悬空且刚标零，也可能是传感器没接、或者字段没填。
            # 分不开，所以不赌 —— 不启用，并说清楚怎么让它启用。
            return False, ("not enabled: zero_force_data reads exactly zero on all "
                           "six axes, which cannot be told apart from an unwired or "
                           "unpopulated sensor. Zero the sensor with the arm hanging "
                           "free (rm_clear_force_data), confirm a non-zero reading, "
                           "then restart this card")
        return True, f"enabled, thresholds={self._force_thresholds}"

    def _halt(self, halted: bool):
        """`pause` and `resume`. Stops the arm; keeps the subscription."""
        with self._lock:
            if not self._running:
                return {"state": "idle", "message": "card is not running"}
            self._paused = bool(halted)
        if halted:
            self._slow_stop()
            # **信箱也要清。** `pause` 是模型的 halt，也是中断钩子打的那个 ——
            # 手臂立刻停，而一个还压在信箱里的夹爪目标会在暂停之后被 worker
            # 发出去，于是「暂停」变成「手臂停了但手还在合」。和 `_on_message`
            # 丢掉暂停期间指令同一个理由：那个目标出自一个已经过去的世界。
            self._clear_gripper_target()
        return {"state": "paused" if halted else "running",
                "input": self._input_topic}

    def _reset(self):
        """清掉 force-torque 的锁存。链条第 7 步是**锁存**的，只有 `reset()` 能出来。

        做成一个 action 而不是让操作员 stop→start：重建卡片会连带放掉运动门、
        丢掉订阅，而这里要的只是「我看过手臂了，可以继续」。
        """
        with self._lock:
            sink = self._sink
            running = self._running
        if sink is None or not running:
            return {"state": "idle", "message": "card is not running"}
        sink.reset()
        # 锁存期间攒下的夹爪目标不该在恢复的瞬间被发出去：它是撞击之前算出来的。
        self._clear_gripper_target()
        print("[rm75] servo sink reset (force-torque latch cleared)", flush=True)
        return {"state": "paused" if self._paused else "running",
                "input": self._input_topic, "sink": sink.stats()}

    def _stop(self):
        with self._lock:
            node, self._node = self._node, None
            self._sink = None
            was_running = self._running
            self._running = False
            topic, self._input_topic = self._input_topic, ""

        self._stop_workers()
        try:
            if node is not None:
                try:
                    self._ros2.executor_core.remove_node(node)
                finally:
                    node.destroy_node()
            if was_running:
                self._slow_stop()
        finally:
            # Teardown can fail; the shared gate must be released even then.
            with self._lock:
                gate_held = self._motion_gate_held
                self._motion_gate_held = False
            if gate_held:
                self.client.motion_gate.release()
        if was_running:
            print(f"[rm75] servo stopped ({topic})", flush=True)
        return {"state": "idle"}

    def _info(self):
        with self._lock:
            sink = self._sink
            running = self._running
            topic = self._input_topic
            last = dict(self._last_outcome) if self._last_outcome else None
            rejects = list(self._rejects)
            paused = self._paused
        return {
            # Paused is its own state, not running: an operator reading
            # "running" beside a motionless arm goes looking for a fault that
            # is not there.
            "state": ("paused" if paused else "running") if running else "idle",
            "input": topic,
            "control_interface": self._descriptor_raw,
            # The window this card commits to the controller before it can be
            # revoked. One passthrough period: the arm is never holding more
            # than a single point, so e-stop latency is bounded by the period
            # rather than by a chunk length. See README_dev § "Continuous Control".
            "committed_window_ms": int(1000.0 / self._expected_hz),
            "motion_enabled": self.client.motion_enabled,
            "sink": sink.stats() if sink is not None else None,
            "last_outcome": last,
            # Rejections are contract or limit violations — somebody wired
            # something wrong — so they are surfaced, unlike drops.
            "recent_rejects": rejects,
            # 夹爪那一维的全部真相，包括两个会**悄悄**限制覆盖范围的值：
            # 死区和限频。README_dev 的 "no silent caps" —— 一个被限掉的频率
            # 如果不报出来，读到的人会以为模型的每一拍都到了夹爪。
            "gripper": {
                "configured": self._gripper_configured,
                "enabled": self._gripper_enabled,
                "deadband": self._gripper_deadband,
                "max_hz": self._gripper_max_hz,
                "release_on_watchdog": self._release_gripper_on_watchdog,
                "last_sent_closure": self._gripper_sent,
                "error": self._gripper_error,
                # 极性在真机上没验证过，而读到这里的人正是该去验证的人。
                "polarity": "0 = open, 1 = closed (UNVERIFIED on hardware)",
            },
            "force_torque": {
                "enabled": self._force_enabled,
                "note": self._force_note,
                "thresholds": self._force_thresholds,
                "source": "zero_force_data (external force, tool weight compensated)",
                "last_reading": self._last_force,
            },
            "state_out": {
                "topic": self._state_topic,
                "hz": self._state_hz,
                "last": self._last_state,
                "error": self._poll_error,
                # 位姿在控制器**当前生效的工具坐标系**下。驱动不拥有这个设置，
                # 所以它是个必须被看见的假设，而不是一个可以悄悄补偿的偏移。
                # 见 config.yaml 的 cartesian.tool_length_mm 注释。
                "eef_frame": "controller's active tool frame, relative to base_link",
            },
        }

    # ── the stream ───────────────────────────────────────────────────────────

    def _subscribe(self, topic: str):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                      # a queued command is a stale command
            durability=DurabilityPolicy.VOLATILE,
        )
        node = Node("rm75_servo", context=self._ros2.ctx_core)
        node.create_subscription(String, topic, self._on_message, qos)
        # The watchdog needs a heartbeat of its own: silence produces no
        # callbacks, and silence is exactly what it exists to notice.
        node.create_timer(WATCHDOG_MS / 2000.0, self._tick)
        self._ros2.executor_core.add_node(node)
        with self._lock:
            self._node = node

    def _on_message(self, message):
        try:
            payload = json.loads(message.data)
        except Exception as exc:
            self._record(Verdict.REJECTED.value, f"undecodable payload: {exc}")
            return
        # Hold the lifecycle lock through admission and application. stop()
        # and pause() therefore cannot issue slow-stop and return while an
        # already-admitted callback is still able to command the arm.
        with self._lock:
            sink = self._sink
            if sink is None or self._paused:
                # Dropped, not queued: a command held through a pause was
                # computed from a world that has moved on.
                return
            outcome = sink.submit(payload)
        self._record(outcome.verdict.value, outcome.reason, outcome.warnings)

    def _tick(self):
        with self._lock:
            sink = self._sink
            if sink is None or self._paused:
                return
            outcome = sink.tick()
        if outcome is not None:
            self._record(outcome.verdict.value, outcome.reason)

    def _record(self, verdict: str, reason: str = "", warnings=None):
        entry = {"verdict": verdict, "reason": reason,
                 "at_ms": int(time.time() * 1000)}
        if warnings:
            entry["warnings"] = list(warnings)
        with self._lock:
            self._last_outcome = entry
            if verdict in (Verdict.REJECTED.value, Verdict.ABORTED.value):
                self._rejects.append(f"{verdict}: {reason}")
                del self._rejects[:-10]

    # ── the arm ──────────────────────────────────────────────────────────────

    def _apply(self, values, gripper):
        """Only reached by commands that passed every check in the sink.

        **`gripper` 这个形参不用**，夹爪走 `values` 的第 8 维 —— 和 G1、天轶一致。
        它是 `common/control/sink.py:261` 的一个旁路字段，而 actucore 里没有任何
        生产者（`plugins/vla/` 全文没有它）。维度一律走 `values`，因为只有走
        `values` 才会经过 sink 的限位、步长和速度检查；旁路字段一道检查都不过。
        """
        degrees = [math.degrees(value) for value in values[:len(JOINT_NAMES)]]
        # follow=False — low-follow mode. High follow requires a period under
        # 10 ms, which this card does not promise.
        self.client.command("rm_movej_canfd", degrees, False)
        if self._gripper_enabled and len(values) > len(JOINT_NAMES):
            self._want_gripper(values[len(JOINT_NAMES)])

    # ── 夹爪 ─────────────────────────────────────────────────────────────────

    def _want_gripper(self, closure: float):
        """只写单槽信箱，立刻返回。真正的 SDK 调用在 worker 线程里。

        **为什么不在这里直接调 SDK。** `hardware.py:command_trajectory` 已经记着
        这台硬件上「三线程 SDK 的 block=0 本应立即返回，但部分真机版本会一直停在
        Python/C 包装调用中」。内联的话，一次这样的停顿就卡住 30 Hz 的手臂回路 ——
        而手臂回路是这张卡里唯一不能停的东西。夹爪迟到一拍只是迟到；手臂停一拍
        是指令流断了。

        信箱是**单槽、最新值覆盖**：攒着的夹爪目标是从一个已经过去的世界算出来
        的，和 `_on_message` 对 pause 期间指令的处理同一个理由。
        """
        with self._gripper_lock:
            self._gripper_wanted = max(0.0, min(1.0, float(closure)))
        self._gripper_wake.set()

    def _clear_gripper_target(self):
        with self._gripper_lock:
            self._gripper_wanted = None

    def _gripper_loop(self):
        """取出信箱里的最新目标，按死区和限频决定发不发。"""
        min_period = 1.0 / self._gripper_max_hz
        while not self._gripper_stop.is_set():
            # 醒来靠事件而不是轮询，但带超时：限频把一次发送推迟到以后，而那个
            # 「以后」没有新的事件去唤醒它。
            self._gripper_wake.wait(timeout=min_period)
            self._gripper_wake.clear()
            if self._gripper_stop.is_set():
                return
            # **暂停要在这里也查一次，不只在 `_on_message` 里。** 手臂那条路是
            # 同步的：暂停之后就没有新的 `_apply`。夹爪这条路是异步的，清信箱和
            # worker 取出目标之间有一个窗口，于是光靠清信箱会漏掉已经取出来的
            # 那一个 —— 表现是暂停之后夹爪又动了一次。
            with self._lock:
                if self._paused or not self._running:
                    continue
            with self._gripper_lock:
                wanted = self._gripper_wanted
                sent = self._gripper_sent
                last_ms = self._gripper_sent_ms
            if wanted is None:
                continue
            # 死区：夹爪是控制器后面的串行设备，把 30 Hz 里每一个浮点噪声都发
            # 下去只会占满那条链路，而它根本跟不上。
            if sent is not None and abs(wanted - sent) < self._gripper_deadband:
                continue
            now = time.monotonic()
            if sent is not None and now - last_ms < min_period:
                continue
            position = closure_to_position(wanted)
            try:
                # block=False, timeout=0 —— 厂商文档：「非阻塞模式：0-发送后
                # 立即返回」。worker 线程的全部理由见 `_want_gripper`。
                self.client.command("rm_set_gripper_position", position, False, 0)
            except Exception as exc:
                # 一次失败不该让这个线程退出：夹爪掉线而手臂还在走，是一个要被
                # 看见（`info()` 里的 gripper.error）而不是让卡片垮掉的状态。
                with self._gripper_lock:
                    self._gripper_error = f"{type(exc).__name__}: {exc}"
                continue
            with self._gripper_lock:
                self._gripper_sent = wanted
                self._gripper_sent_ms = now
                self._gripper_error = ""

    def _on_watchdog(self):
        """看门狗和 abort 都落在这里：停手臂，默认**不动夹爪**。

        松开夹爪是另一个显而易见的选择 —— 策略已经沉默了，而手还握着一个没人在
        决定的东西 —— 但松开会把它扔到下面的任何东西上。哪个更糟取决于机器人正在
        做什么，而这个文件无从知道。所以它是 opt-in
        （`release_gripper_on_watchdog`），不是一个会让人惊讶一次的默认值。
        天轶的 `release_hands_on_watchdog` 同一条推理。

        无论松不松，**信箱都要清**：攒着的目标是沉默之前算出来的，不该在沉默
        期间继续被发出去。
        """
        self._slow_stop()
        self._clear_gripper_target()
        if not self._release_gripper_on_watchdog or not self._gripper_enabled:
            return
        # 0 = 张开。走和正常指令同一条 worker 路径，于是它同样不会阻塞调用方
        # （这里的调用方是 sink 的 tick，也就是 ROS 的定时器线程）。
        self._want_gripper(0.0)

    def _slow_stop(self):
        if not self.client.connected:
            return
        try:
            self.client.command("rm_set_arm_slow_stop")
        except Exception as exc:
            print(f"[rm75] servo stop failed: {exc}", flush=True)

    # ── workers ──────────────────────────────────────────────────────────────

    def _start_workers(self):
        """夹爪 worker + 状态/六维力轮询。两个都是守护线程，绑卡片生命周期。"""
        with self._lock:
            if self._gripper_enabled and self._gripper_thread is None:
                self._gripper_stop.clear()
                self._gripper_wake.clear()
                self._gripper_thread = threading.Thread(
                    target=self._gripper_loop, name="rm75-servo-gripper", daemon=True)
                self._gripper_thread.start()
            if self._poll_thread is None:
                self._poll_stop.clear()
                self._poll_thread = threading.Thread(
                    target=self._poll_loop, name="rm75-servo-poll", daemon=True)
                self._poll_thread.start()

    def _stop_workers(self):
        with self._lock:
            gripper_thread, self._gripper_thread = self._gripper_thread, None
            poll_thread, self._poll_thread = self._poll_thread, None
        self._gripper_stop.set()
        self._gripper_wake.set()
        self._poll_stop.set()
        self._clear_gripper_target()
        # join 带超时：一个卡在厂商包装调用里的 worker 不该把 `stop` 一起拖住，
        # 而 `stop` 还要去放掉共享的运动门。线程是守护的，进程退出时会走。
        for thread in (gripper_thread, poll_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        with self._lock:
            self._state_pub = None

    # ── 本体状态与六维力 ─────────────────────────────────────────────────────

    def _read_force(self):
        """六轴**系统外受力**，或 None（字段不在）。

        **读 `zero_force_data`，不读 `force_data`。** `rm_force_data_t` 有四个
        六元向量，`force_data` 是传感器**原始**读数、含工具自重 —— 对它设阈值
        的结果是某些姿态下被重力直接触发、另一些姿态下永不触发，而两者都看着
        像「阈值没调好」。`zero_force_data` 是厂商做完重力/负载补偿之后的
        「系统外受力」，也就是「有没有外界在推我」这个问题的答案。
        """
        data = self.client.call("rm_get_force_data")
        if not isinstance(data, dict):
            return None
        readings = data.get("zero_force_data")
        if readings is None:
            return None
        values = [float(v) for v in readings]
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            return None
        return values

    def _sample_state(self):
        """一次 SDK 往返拿到关节角**和**末端位姿，再一次拿夹爪。

        `rm_get_current_arm_state` 同时给 `joint`（度）和 `pose`
        （xyz 米 + rx/ry/rz 弧度），所以手臂那边是一次往返而不是两次。
        """
        # 时刻在**第一次 SDK 调用之前**取。VLA 用它算观测年龄，而取在发布之前
        # 会把两次往返的耗时算成零 —— 等于宣称这路通道比实际更新鲜，这正是
        # `max_obs_age_ms` 要挡的那件事。
        stamp_ms = int(time.time() * 1000)
        state = self.client.call("rm_get_current_arm_state")
        if not isinstance(state, dict):
            return None
        degrees = [float(v) for v in (state.get("joint") or [])]
        pose = [float(v) for v in (state.get("pose") or [])]
        if len(degrees) != len(JOINT_NAMES) or len(pose) != 6:
            return None
        if not all(math.isfinite(v) for v in degrees + pose):
            return None
        values = [math.radians(v) for v in degrees]

        closure = None
        if self._gripper_enabled:
            gripper = self.client.call("rm_get_gripper_state")
            if not isinstance(gripper, dict) or "actpos" not in gripper:
                return None
            closure = position_to_closure(gripper["actpos"])
            values.append(closure)

        quaternion = rotation.rpy_to_quat(pose[3], pose[4], pose[5])
        eef = [pose[0], pose[1], pose[2], *quaternion]
        if closure is not None:
            eef.append(closure)
        return {"values": values, "eef": eef, "stamp_ms": stamp_ms}

    def _poll_loop(self):
        """一个线程，两个周期：六维力按 `force_hz`，状态按 `state_hz`。

        六维力不跟着状态走 —— 10 Hz 的碰撞检测意味着撞上之后手臂还能再走
        100 ms。状态慢一些没有代价：VLA 的 `max_obs_age_ms` 是 300。
        """
        force_period = 1.0 / self._force_hz
        state_period = 1.0 / self._state_hz
        next_state = 0.0
        while not self._poll_stop.wait(timeout=force_period):
            with self._lock:
                running = self._running
            if not running:
                return
            if self._force_enabled:
                self._sample_force()
            now = time.monotonic()
            if now >= next_state:
                next_state = now + state_period
                self._publish_state()

    def _sample_force(self):
        try:
            readings = self._read_force()
        except Exception as exc:
            with self._lock:
                self._poll_error = f"force read failed: {type(exc).__name__}: {exc}"
            return
        if readings is None:
            return
        with self._lock:
            self._last_force = readings
            sink = self._sink
        if sink is None:
            return
        # 超阈值 → ABORTED 并**锁存**：撞到东西之后，下一条指令恰恰是最不该跑的
        # 那一条。出路只有 `reset` action。
        outcome = sink.force_torque(readings)
        if outcome is not None:
            self._record(outcome.verdict.value, outcome.reason)
            print(f"[rm75] servo force-torque abort: {outcome.reason}", flush=True)

    def _publish_state(self):
        """关节角 + 夹爪闭合度 + **当前末端位姿**，一路话题、一份载荷。

        `eef` 这个字段是给增量模型（OpenVLA）当基准位姿用的：它输出的是末端
        delta，「相对于哪儿」只有机器人知道。

        **和关节状态放在同一路话题里，不新开一路。** agent-core 的 VLA 卡片
        （`actucore/plugins/vla/plugin.py:_bind_inputs`）是按 **ROS 消息类型**
        分派角色的，两路 `String` 它分不开 —— 新开一路会变成一个按连线顺序赌
        运气的绑定。

        **位姿的坐标系是控制器当前生效的工具坐标系**，不一定是夹爪 TCP。见
        `config.yaml` 的 `cartesian.tool_length_mm` 注释：它只在控制器已被设成
        夹爪 TCP 时才是 222.5。这个假设在 `info()` 的 `state_out.eef_frame` 里
        报出来，而不是在这里偷偷加一个 222.5 mm 的偏移 —— 补偿一个我们并不知道
        生效没生效的设置，比不补偿更危险。
        """
        publisher = self._ensure_state_publisher()
        if publisher is None:
            return
        try:
            sample = self._sample_state()
        except Exception as exc:
            with self._lock:
                self._poll_error = f"state read failed: {type(exc).__name__}: {exc}"
            return
        if sample is None:
            # **缺一路就整个不发，不补零。** 补进去的零在弧度里是「手臂伸直」、
            # 在闭合度里是「夹爪张开」，两个都是看着合理的读数，策略无从分辨。
            # 宁可让下游拿不到观测（它会因此不发指令），也不给它一个编造的世界。
            with self._lock:
                self._poll_error = "state incomplete; published nothing"
            return
        stamp_ms = sample["stamp_ms"]

        from std_msgs.msg import String

        payload = String()
        payload.data = json.dumps({
            "schema": "motus.control/1",
            "kind": "joint_state",
            "dof": self._descriptor.dof,
            "joint_names": list(self._descriptor_raw["joint_names"]),
            # `values` 是本体感受（和指令同序同单位），`eef` 是标准布局的当前
            # 末端位姿。两者是两回事：前者的宽度和含义由机器人决定，后者由协议
            # 定义（7 维位姿 + 夹爪，和 phanthymotus-cloud 的
            # normalize.SINGLE_ARM_EEF_GROUPS 同构）。
            "values": sample["values"],
            "eef": sample["eef"],
            "stamp_ms": stamp_ms,
        }, ensure_ascii=False)
        try:
            publisher.publish(payload)
        except Exception as exc:
            with self._lock:
                self._poll_error = f"state publish failed: {type(exc).__name__}: {exc}"
            return
        with self._lock:
            self._last_state = {"values": sample["values"], "eef": sample["eef"],
                                "stamp_ms": stamp_ms}
            self._poll_error = ""

    def _ensure_state_publisher(self):
        with self._lock:
            if self._state_pub is not None:
                return self._state_pub
            node = self._node
        if node is None:
            return None
        from std_msgs.msg import String

        publisher = node.create_publisher(String, self._state_topic, 10)
        with self._lock:
            self._state_pub = publisher
        return publisher
