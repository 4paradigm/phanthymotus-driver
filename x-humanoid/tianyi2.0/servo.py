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
                               "enum": ["start", "stop", "info"]},
                    "input_topic": {"type": "string",
                                    "description": "control/joint 指令流话题"},
                    "confirm_motion": {
                        "type": "boolean",
                        "description": "必须为 true —— 启动这张卡片即授权持续运动",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": ["input_topic", "confirm_motion"],
                              "description": "订阅指令流并开始驱动双臂与双手"},
                    "stop": {"params": [], "description": "停止订阅并让机器人静止"},
                    "info": {"params": [],
                             "description": "动作空间 descriptor、检查链计数与最近一次结果"},
                },
                "x-hooks": {"on_interrupt_motion": {"action": "stop"},
                            "on_interrupt_all": {"action": "stop"}},
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
        }

    def dispatch(self, action: str, args: dict):
        if action == "start":
            return self._start(args)
        if action == "stop":
            return self._stop()
        if action == "info":
            return self._info()
        return None

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Bundle lifecycle. Deliberately does nothing.

        A card that streams motion must not come up streaming because the
        container restarted. It subscribes when someone wires it and confirms.
        """

    def stop(self):
        self._stop()

    # ── actions ──────────────────────────────────────────────────────────────

    def _start(self, args: dict):
        if not args.get("confirm_motion"):
            return {"state": "error",
                    "message": "confirm_motion 必须为 true —— 这张卡片在运行期间"
                               "持续驱动双臂与双手"}

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

    def _stop(self):
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
        return {
            "state": "running" if running else "idle",
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
        self._ros2.executor_tianyi.add_node(pub_node)

        with self._lock:
            self._sub_node, self._pub_node = sub_node, pub_node
            self._arm_pub = arm_pub
            self._left_hand_pub = left_hand
            self._right_hand_pub = right_hand

    def _on_message(self, message):
        sink = self._sink
        if sink is None:
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
