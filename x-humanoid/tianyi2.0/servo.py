#!/usr/bin/env python3
"""Continuous bimanual control for Tianyi 2.0 — arms and dexterous hands.

`arm` and `hand` in device.py are the call-shaped paths: one pose per
`tools/call`, feedback verification, a gesture preset. Right for an LLM posing a
robot, wrong for an execution model, which emits a whole action vector tens of
times a second and is not waiting for an answer to each one.

This card is the stream-shaped path for both at once. A bimanual policy does not
produce "an arm command" and separately "a hand command" — it produces one
vector covering everything it controls, and splitting that across two cards
would mean two topics, two arrival times and two independent watchdogs for what
the model intended as one instant.

So the action space is 26 dimensions:

      0..6    left arm,   radians
      7..13   right arm,  radians
     14..19   left hand,  0 = open .. 1 = closed
     20..25   right hand, same

which is why `groups` exists in the descriptor (common/control/descriptor.py):
one `units` mapping cannot say "radians here, normalised closure there", and the
arms and the hands are different physical channels that an ACP barrier should be
able to tell apart.

Three things here are Tianyi-specific and are where a Tianyi-shaped mistake
would live:

- **Two DDS domains.** Commands arrive from agent-core on domain 42
  (`ctx_core`) and the robot's own controllers live on domain 0 (`ctx_tianyi`).
  Subscribing on the wrong one produces a card that starts cleanly, reports
  running, and never receives anything.

- **The hand's polarity is inverted at the wire.** The hardware takes 1.0 for
  open and 0.0 for closed; this descriptor is the other way round, matching the
  existing `hand` tool where 0 is open and 100 is closed. Getting that backwards
  turns "open the hand" into "clench", which around an object is the dangerous
  direction, so it is converted in one place and tested.

- **Left and right arms do not share limits.** Shoulder roll is (-15, 150) on
  the left and (-150, 15) on the right. A descriptor built from one side and
  mirrored would authorise the wrong half of each range.
"""

from __future__ import annotations

import json
import math
import threading
import time

from common.control import ControlSink, Verdict, parse_descriptor

from device import (
    _RATED_MOTOR_CURRENT_A,
    _RELIABLE_QOS,
    ArmPlugin,
    HandPlugin,
)

# Vendor-sanctioned joint speed range for the arms is [0.2, 1.5] rad/s (see
# ArmPlugin's schema). The streaming path takes the top of it as the hard limit
# and sends a calmer value as the per-command speed.
ARM_MAX_VELOCITY = 1.5
ARM_COMMAND_SPEED = 0.5
# Fingers: full travel in half a second. There is no vendor figure for this, so
# it is deliberately conservative rather than invented precision.
HAND_MAX_VELOCITY = 2.0

DEFAULT_EXPECTED_HZ = 30.0
MAX_HZ = 50.0
WATCHDOG_MS = 200
MAX_OBS_AGE_MS = 300

FINGER_NAMES = HandPlugin._FINGER_NAMES
ARM_JOINTS = ArmPlugin._JOINT_NAMES

LEFT_ARM = slice(0, 7)
RIGHT_ARM = slice(7, 14)
LEFT_HAND = slice(14, 20)
RIGHT_HAND = slice(20, 26)
DOF = 26

# 本体反馈的来源，都在域 0（机器人自己的控制器那一侧）。
# 直接订原始话题，不经 device.py 里的 ArmGesturePlugin / HandStatePlugin ——
# 这张卡拿不到那两个实例的引用，而且更要紧的是：这样两个方向的单位换算落在
# 同一个文件里，线上反馈→descriptor 单位 和 descriptor 单位→线上指令 可以并排
# 读、并排测。它们必须互为逆运算，分散在两处就没人保证得了。
ARM_STATUS_TOPIC = "/arm/status"
HAND_STATE_TOPICS = {"left": "/inspire_hand/state/left_hand",
                     "right": "/inspire_hand/state/right_hand"}
# 手臂电机 id：左 11..17、右 21..27，与 _publish_arms 里的 base_id 同源。
ARM_MOTOR_BASE = {"left": 11, "right": 21}
STATE_MAX_HZ = 30.0


def build_descriptor(expected_hz: float = DEFAULT_EXPECTED_HZ) -> dict:
    """The 26-dimension action space, derived from what `arm`/`hand` enforce.

    Taken from the same vendor limits the call-shaped tools use, so the two
    paths to the same motors cannot disagree about what this robot can do.
    """
    joint_names = (
        [f"left_{name}" for name in ARM_JOINTS]
        + [f"right_{name}" for name in ARM_JOINTS]
        + [f"left_{name}" for name in FINGER_NAMES]
        + [f"right_{name}" for name in FINGER_NAMES]
    )

    lower, upper = [], []
    for limits in (ArmPlugin._LEFT_POSE_LIMITS, ArmPlugin._RIGHT_POSE_LIMITS):
        for low, high in limits:
            lower.append(math.radians(low))
            upper.append(math.radians(high))
    # Both hands: normalised closure, 0 open .. 1 closed.
    lower.extend([0.0] * 12)
    upper.extend([1.0] * 12)

    max_velocity = [ARM_MAX_VELOCITY] * 14 + [HAND_MAX_VELOCITY] * 12
    period = 1.0 / expected_hz
    max_delta = [speed * period for speed in max_velocity]

    return {
        "control_interface": "motus.control/1",
        "mode": "joint_position",
        "dof": DOF,
        "joint_names": joint_names,
        # Mixed by necessity; `groups` is what says which is which.
        "units": {"angle": "rad", "normalized": "0-1", "time": "s"},
        "limits": {
            "lower": lower,
            "upper": upper,
            "max_velocity": max_velocity,
            "max_delta_per_step": max_delta,
        },
        "groups": [
            {"name": "arm_l", "offset": 0, "count": 7,
             "unit": "rad", "resource": "arm_l"},
            {"name": "arm_r", "offset": 7, "count": 7,
             "unit": "rad", "resource": "arm_r"},
            {"name": "hand_l", "offset": 14, "count": 6,
             "unit": "normalized", "resource": "hand_l"},
            {"name": "hand_r", "offset": 20, "count": 6,
             "unit": "normalized", "resource": "hand_r"},
        ],
        "frame": "base_link",
        "rate": {
            "max_hz": MAX_HZ,
            "expected_hz": expected_hz,
            "watchdog_ms": WATCHDOG_MS,
            "max_obs_age_ms": MAX_OBS_AGE_MS,
        },
        # Tianyi's arms report no force-torque to this driver. Declared null
        # rather than omitted, so the missing protection is visible.
        "force_torque": None,
    }


class TianyiServoPlugin:
    """One card, one input topic, one sink, four publishers."""

    PREFIX = "servo"

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        config = plugin_config or {}
        self._expected_hz = float(config.get("expected_hz", DEFAULT_EXPECTED_HZ))
        if not math.isfinite(self._expected_hz) or not 0 < self._expected_hz <= MAX_HZ:
            raise ValueError(f"servo.expected_hz must be in (0, {MAX_HZ}]")
        self._arm_speed = float(config.get("arm_speed", ARM_COMMAND_SPEED))
        # Off by default — see _hold for why this is a decision and not a
        # setting with an obvious answer.
        self._release_hands = bool(config.get("release_hands_on_watchdog", False))
        # Paused means subscribed but not applying. See `_halt`.
        self._paused = False

        # 本体状态。独立的锁：状态回调来自域 0 的执行器线程，和指令回调
        # （域 42）是两条路，共用 _lock 会让一路的回调排在另一路后面。
        self._state_lock = threading.RLock()
        self._state_topic = f"/{namespace}/servo/state"
        self._state_pub = None
        self._arm_pos: dict = {}
        self._arm_seen_ms = 0
        self._hand_closure: dict = {"left": {}, "right": {}}
        self._hand_seen_ms: dict = {"left": 0, "right": 0}
        self._last_state_publish_at = 0.0
        self._descriptor_raw = build_descriptor(self._expected_hz)
        self._descriptor = parse_descriptor(self._descriptor_raw)

        self._lock = threading.RLock()
        self._sink = None
        self._sub_node = None            # domain 42 — where commands arrive
        self._pub_node = None            # domain 0 — where the robot listens
        self._arm_pub = None
        self._left_hand_pub = None
        self._right_hand_pub = None
        self._input_topic = ""
        self._running = False
        self._last_outcome = None
        self._rejects: list = []

    # ── tool ─────────────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "servo",
            "type": "actuator",
            "description": (
                "天轶 2.0 双臂 + 双灵巧手的连续控制：订阅一路 motus.control/1 "
                f"指令流（26 维，≤{self._expected_hz:g} Hz）驱动执行。"
                "普通摆姿势用 arm / hand，这张卡片是给执行模型用的。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "stop", "pause", "resume",
                                        "info"]},
                    "input_topic": {"type": "string",
                                    "description": "control/joint 指令流话题"},
                },
                "required": ["action"],
                # `start`/`stop` are missing from here on purpose, and that is
                # what decides who may call them: agent-core splits a tool into
                # one LLM-callable function per entry (mcp_client.py
                # `_to_openai_schema`), so an action absent here is reachable by
                # the canvas and not by the model. `stop` in particular belongs
                # to the project lifecycle — a model calling it would take the
                # card out of a running project without the project knowing, and
                # could not put it back, since `start` needs the input topic and
                # the downstream descriptor that only agent-core has.
                #
                # So `pause` is the model's halt, and it is what the interrupt
                # hooks fire. It is not a weaker `stop`: the arms stop just as
                # immediately. It differs in staying subscribed, so the
                # project's view of what is running stays true and `resume` can
                # continue.
                "x-action-params": {
                    "pause": {"params": [],
                              "description": "立即停止执行并保持当前姿态；"
                                             "仍然订阅着，resume 可继续"},
                    "resume": {"params": [], "description": "继续执行"},
                },
                "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                            "on_interrupt_all": {"action": "pause"}},
                "x-is-dangerous": True,
                # Every channel this card occupies. The arms and the hands are
                # independent degrees of freedom, which is why `hand` already
                # declares its two separately — but this card holds all four,
                # so nothing else may drive any of them while it runs.
                "x-resource": list(self._descriptor.resources),
                # No x-completion: a stream has no end. A pending action held
                # open for the life of the card would block every other
                # actuator behind the ACP barrier.
            },
            "topic_in": [{"format": "control/joint",
                          "desc": "motus.control/1，26 维（14 臂 rad + 12 指 归一化）"}],
            # 本体状态和它接受的指令出自**同一份** build_descriptor()：同样的
            # 关节顺序、同样的单位、同样的极性。想只改一边做不到，因为它们是
            # 同一个列表。这正是把状态放在这张卡上、而不是另建一张卡的理由 ——
            # 一致性从"我们保证同步维护"变成结构性的。
            #
            # topic 在这里声明而不是等启动后再报：画布上这条线要接回 vla 卡，
            # 形成 vla → servo →(state)→ vla 的回环，而环里谁都不能从"已经启动
            # 的上游"学到自己的输入话题，持久化的声明是唯一剩下的东西。
            "topic_out": [{"topic": self._state_topic, "format": "state/joint",
                           "desc": "26 维本体状态，与指令同序同单位"}],
        }

    def dispatch(self, action: str, args: dict):
        if action == "start":
            return self._start(args)
        if action == "stop":
            return self._stop()
        if action == "pause":
            return self._halt(True)
        if action == "resume":
            return self._halt(False)
        if action == "info":
            return self._info()
        return None

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Bundle lifecycle. Deliberately does nothing.

        A card that streams motion must not come up streaming because the
        container restarted. It subscribes when agent-core starts the project,
        which is an operator action, and not before.
        """

    def stop(self):
        self._stop()

    # ── actions ──────────────────────────────────────────────────────────────

    def _start(self, args: dict):
        topic = (args.get("input_topic") or "").strip()
        if not topic:
            topics = args.get("input_topics") or [""]
            topic = (topics[0] or "").strip()
        if not topic:
            return {"state": "error",
                    "message": "缺少 input_topic —— 请在画布上把一路 control/joint "
                               "源连到这张卡片"}

        if self._ros2 is None:
            return {"state": "error", "message": "没有 ROS 上下文，无法订阅"}

        sink = ControlSink(
            self._descriptor,
            self._apply,
            on_watchdog=self._hold,
            on_abort=self._hold,
        )
        # Registered before started, so a concurrent stop can find and cancel
        # it — same rule as every other plugin holding per-instance state.
        with self._lock:
            if self._running:
                return {"state": "error",
                        "message": f"已经在运行（{self._input_topic}）"}
            self._sink = sink
            self._input_topic = topic
            self._running = True
            self._paused = False

        try:
            self._open(topic)
        except Exception as exc:
            with self._lock:
                self._running = False
                self._sink = None
            return {"state": "error", "message": f"启动失败: {exc}"}

        print(f"[servo] streaming from {topic}", flush=True)
        return {"state": "running", "input": topic,
                "control_interface": self._descriptor_raw}

    def _halt(self, halted: bool):
        """`pause` and `resume`. Stops the arms; keeps the subscription.

        Holds rather than releases, for the same reason the watchdog does (see
        `_hold`): these joints keep their last target, so not publishing *is*
        the hold — and a pause that dropped whatever the hands are carrying
        would put it on whatever is underneath, which is a worse answer to
        "wait" than stillness.
        """
        with self._lock:
            if not self._running:
                return {"state": "idle", "message": "卡片未在运行"}
            self._paused = bool(halted)
        if halted:
            self._hold()
        return {"state": "paused" if halted else "running",
                "input": self._input_topic}

    def _stop(self):
        with self._state_lock:
            self._state_pub = None
            self._last_state_publish_at = 0.0
        with self._lock:
            sub_node, self._sub_node = self._sub_node, None
            pub_node, self._pub_node = self._pub_node, None
            self._sink = None
            self._arm_pub = None
            self._left_hand_pub = None
            self._right_hand_pub = None
            was_running, self._running = self._running, False
            topic, self._input_topic = self._input_topic, ""

        for node, executor in ((sub_node, getattr(self._ros2, "executor_core", None)),
                               (pub_node, getattr(self._ros2, "executor_tianyi", None))):
            if node is None:
                continue
            try:
                if executor is not None:
                    executor.remove_node(node)
            finally:
                # destroy_node, not only remove_node: otherwise the publisher
                # and the ROS node name leak and a restart collides with itself.
                node.destroy_node()

        if was_running:
            print(f"[servo] stopped ({topic})", flush=True)
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
            # "running" beside two motionless arms goes looking for a fault
            # that is not there.
            "state": ("paused" if paused else "running") if running else "idle",
            "input": topic,
            "control_interface": self._descriptor_raw,
            # One command is committed at a time, so the window an e-stop has to
            # wait out is one period rather than a chunk length.
            "committed_window_ms": int(1000.0 / self._expected_hz),
            "sink": sink.stats() if sink is not None else None,
            "last_outcome": last,
            "recent_rejects": rejects,
        }

    # ── the stream ───────────────────────────────────────────────────────────

    def _open(self, topic: str):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from sensor_msgs.msg import JointState
        from bodyctrl_msgs.msg import CmdSetMotorPosition

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                       # a queued command is a stale command
            durability=DurabilityPolicy.VOLATILE,
        )

        # 本体反馈订阅在域 0，状态发布在域 42 —— 和指令那条路正好相反。
        from bodyctrl_msgs.msg import MotorStatusMsg

        # Commands arrive on domain 42 …
        sub_node = Node("tianyi2_servo_sub", context=self._ros2.ctx_core)
        sub_node.create_subscription(String, topic, self._on_message, qos)
        # The watchdog needs its own heartbeat: silence produces no callbacks,
        # and silence is exactly what it exists to notice.
        sub_node.create_timer(WATCHDOG_MS / 2000.0, self._tick)
        self._ros2.executor_core.add_node(sub_node)

        # … and the robot listens on domain 0.
        pub_node = Node("tianyi2_servo_pub", context=self._ros2.ctx_tianyi)
        arm_pub = pub_node.create_publisher(
            CmdSetMotorPosition, "/arm/cmd_pos", _RELIABLE_QOS)
        left_hand = pub_node.create_publisher(
            JointState, "/inspire_hand/ctrl/left_hand", _RELIABLE_QOS)
        right_hand = pub_node.create_publisher(
            JointState, "/inspire_hand/ctrl/right_hand", _RELIABLE_QOS)
        # 本体反馈：订在域 0（机器人自己发的），发布到域 42（agent-core 那侧）。
        pub_node.create_subscription(
            MotorStatusMsg, ARM_STATUS_TOPIC, self._on_arm_status, _RELIABLE_QOS)
        for side, hand_topic in HAND_STATE_TOPICS.items():
            pub_node.create_subscription(
                JointState, hand_topic,
                lambda message, s=side: self._on_hand_state(s, message),
                _RELIABLE_QOS)
        state_pub = sub_node.create_publisher(String, self._state_topic, qos)

        self._ros2.executor_tianyi.add_node(pub_node)

        with self._lock:
            self._sub_node, self._pub_node = sub_node, pub_node
            self._arm_pub = arm_pub
            self._left_hand_pub = left_hand
            self._right_hand_pub = right_hand
        with self._state_lock:
            self._state_pub = state_pub

    def _on_message(self, message):
        sink = self._sink
        if sink is None or self._paused:
            # Dropped, not queued: a command held through a pause was computed
            # from a world that has moved on, and applying it at resume would
            # be a jump from stale data.
            return
        try:
            payload = json.loads(message.data)
        except Exception as exc:
            self._record(Verdict.REJECTED.value, f"无法解析的载荷: {exc}")
            return
        outcome = sink.submit(payload)
        self._record(outcome.verdict.value, outcome.reason, outcome.warnings)

    def _tick(self):
        sink = self._sink
        if sink is None:
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

    # ── the robot ────────────────────────────────────────────────────────────

    def _apply(self, values, gripper):
        """Only reached by commands that passed every check in the sink."""
        self._publish_arms(values[LEFT_ARM], values[RIGHT_ARM])
        self._publish_hand(self._left_hand_pub, values[LEFT_HAND])
        self._publish_hand(self._right_hand_pub, values[RIGHT_HAND])

    def _publish_arms(self, left, right):
        publisher = self._arm_pub
        if publisher is None:
            return
        from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition

        message = CmdSetMotorPosition()
        commands = []
        for base_id, pose in ((11, left), (21, right)):
            for i, radians in enumerate(pose):
                command = SetMotorPosition()
                motor_id = base_id + i
                command.name = motor_id
                # Already radians: the descriptor is in the wire's own unit, so
                # there is no conversion here to get backwards.
                command.pos = float(radians)
                command.spd = self._arm_speed
                command.cur = _RATED_MOTOR_CURRENT_A[motor_id]
                commands.append(command)
        message.cmds = commands
        publisher.publish(message)

    def _publish_hand(self, publisher, closure):
        if publisher is None:
            return
        from sensor_msgs.msg import JointState

        message = JointState()
        message.name = [str(i + 1) for i in range(6)]
        # Inverted at the wire: the hardware reads 1.0 as open and 0.0 as
        # closed, while this descriptor — like the existing `hand` tool — reads
        # 0 as open. Backwards here turns "open" into "clench", which around an
        # object is the dangerous direction.
        message.position = [1.0 - float(value) for value in closure]
        publisher.publish(message)

    # ── 本体状态 ─────────────────────────────────────────────────────────────

    def _on_arm_status(self, message):
        """域 0 的 MotorStatusMsg。pos 已经是弧度，与 descriptor 同单位。"""
        with self._state_lock:
            for motor in message.status:
                self._arm_pos[int(motor.name)] = float(motor.pos)
            self._arm_seen_ms = int(time.time() * 1000)
        self._publish_state()

    def _on_hand_state(self, side, message):
        """域 0 的 JointState。

        `position` 是**张开比例**（1.0 张开、0.0 闭合），而 descriptor 用的是
        闭合度（0 张开、1 闭合）—— 所以这里取 1-x，正好是 `_publish_hand` 发出
        去时做的那次取反的逆运算。两处必须互为逆运算，所以放在同一个文件里，
        并且有一条测试把这个往返钉住。

        这个极性今天刚在别处咬过人：device.py 的 hand_state 卡把同一个数字读反
        了，于是 LLM 问"手张开了吗"得到的是相反的答案。
        """
        names = list(getattr(message, "name", []) or [])
        positions = list(getattr(message, "position", []) or [])
        values = {}
        for index, raw_name in enumerate(names):
            if index >= len(positions):
                break
            try:
                finger_id = int(raw_name)
            except (TypeError, ValueError):
                finger_id = index + 1
            values[finger_id] = 1.0 - float(positions[index])
        with self._state_lock:
            self._hand_closure[side] = values
            self._hand_seen_ms[side] = int(time.time() * 1000)
        self._publish_state()

    def state_vector(self):
        """26 维本体状态，或 None —— 有任何一路还没到就返回 None。

        缺一路就整个不发，而不是补零：补进去的零在弧度里是"手臂伸直"、在闭合度
        里是"手张开"，两个都是看着合理的读数，策略无从分辨。宁可让下游拿不到
        观测（它会因此不发指令），也不给它一个编造的世界。

        公开且不碰 ROS，所以顺序、单位、极性可以脱离机器人测。
        """
        with self._state_lock:
            arm_pos = dict(self._arm_pos)
            hands = {side: dict(values) for side, values in self._hand_closure.items()}

        values = []
        for side in ("left", "right"):
            base = ARM_MOTOR_BASE[side]
            for offset in range(len(ARM_JOINTS)):
                position = arm_pos.get(base + offset)
                if position is None:
                    return None
                values.append(position)
        for side in ("left", "right"):
            side_values = hands.get(side) or {}
            for finger_id in range(1, len(FINGER_NAMES) + 1):
                closure = side_values.get(finger_id)
                if closure is None:
                    return None
                values.append(max(0.0, min(1.0, closure)))
        return values

    def _publish_state(self):
        publisher = self._state_pub
        if publisher is None:
            return
        now = time.time()
        if now - self._last_state_publish_at < 1.0 / STATE_MAX_HZ:
            return
        values = self.state_vector()
        if values is None:
            return
        self._last_state_publish_at = now

        from std_msgs.msg import String

        with self._state_lock:
            stamp_ms = min(self._arm_seen_ms or 0,
                           *(v for v in self._hand_seen_ms.values() if v)) \
                if self._arm_seen_ms and any(self._hand_seen_ms.values()) else 0
        payload = String()
        # stamp_ms 取各路反馈里**最旧**的那个，不是现在。一个 VLA 用它算观测年龄，
        # 报现在等于宣称所有通道都刚刚更新过，而实际最旧的那路可能已经很陈旧。
        payload.data = json.dumps({
            "schema": "motus.control/1",
            "kind": "joint_state",
            "dof": DOF,
            "joint_names": self._descriptor_raw["joint_names"],
            "values": values,
            "stamp_ms": stamp_ms or int(now * 1000),
        }, ensure_ascii=False)
        publisher.publish(payload)

    def _hold(self):
        """Watchdog and abort both land here. Holds; does not release.

        These joints are position-controlled: the controller keeps the last
        target it was given, so *not publishing* is already a hold, and that is
        the whole of the default behaviour.

        Releasing the hands would be the other obvious choice — a hand still
        gripping after the policy has gone silent is holding something nobody is
        deciding about any more — but it drops whatever is being carried, onto
        whatever is underneath. Which of those is worse depends on what the
        robot is doing, and this file cannot know. So it is opt-in
        (`release_hands_on_watchdog`) rather than a default that surprises
        someone once.
        """
        if not self._release_hands:
            return
        for publisher in (self._left_hand_pub, self._right_hand_pub):
            try:
                self._publish_hand(publisher, [0.0] * 6)
            except Exception as exc:
                print(f"[servo] hand release failed: {exc}", flush=True)
