#!/usr/bin/env python3
"""Continuous bimanual control for Unitree G1 — 14 arm joints + 2 Dex1 grippers.

`arm` in device.py is the call-shaped path: one named gesture per `tools/call`,
chosen from a fixed list. Right for an LLM posing a robot, wrong for an
execution model, which emits a whole action vector tens of times a second and is
not waiting for an answer to each one.

This card is the stream-shaped path. The action space is 16 dimensions:

      0..6    left arm,     radians
      7..13   right arm,    radians
     14       left gripper, 0 = open .. 1 = closed
     15       right gripper, same

which is what UnifoLM-WMA-0's G1 checkpoints emit (`G1_29_JointArmIndex` 15..28
plus the two Dex1 grippers — `agent_action_dim` 16). It is **not** what
UnifoLM-VLA-0's G1 checkpoint emits: that one is 23-dimensional `EE_R6_G1`,
absolute end-effector pose, which needs IK and a mode `motus.control/1` does not
have yet. Those are two different action spaces and therefore two different
cards; the negotiation refuses the mismatch by comparing `control_mode` rather
than trusting that 16 and 23 are different enough to notice.

── 这条下发通道选的是 `rt/arm_sdk`，不是 `rt/lowcmd` ────────────────────────

G1 上有两条低层通道，选错的那条会让机器人抖。

**`rt/lowcmd`** 接管全部 29 个关节。宇树官方文档《底层运动开发》写得很清楚：

    一旦 G1 开机，内置的运动控制程序会自动启动……如果您在这种状态下使用 SDK 进行
    底层开发，可能会导致指令冲突，从而使 G1 出现抖动的情况。因此……请务必确保
    G1 已经进入调试模式（L2+A），以停止运动控制程序发送指令。

也就是说这条路要求人先把平衡控制器关掉。宇树自己的 `unifolm-world-model-action`
部署代码（`robot_devices/arm/g1_arm.py`）走的正是它 —— 那是给调试态准备的，抄进
一个会被 agent-core 随时 start 的卡片里是危险的。

**`rt/arm_sdk`** 是高层运控服务提供的上肢接口，官方文档《手臂控制例程》：**无需
进入调试模式**，机器人保持锁定站立即可。它靠一个混合权重和内置控制器共存：

    msg.motor_cmd[kNotUsedJoint].q = weight     # 0→1 接管，1→0 交还
    msg.motor_cmd[arm_joint].q/dq/kp/kd/tau     # 只写手臂那 14 个

所以这张卡用 arm_sdk。代价是接管和交还都要**渐变**，不能瞬间切 —— 见 `_ramp`。

── 三个 G1 特有的地方，也是 G1 形状的错误会住进来的地方 ────────────────────

- **交还必须渐出。** 权重从 1 直接掉到 0，手臂瞬间脱力自由落体。官方例程用 2 秒
  把它降下来，这里照做，并且 `stop` 会**等**这条渐出跑完再返回。

- **左右肩 roll 不对称。** 左 [-1.588, 2.252]，右 [-2.252, 1.588]。拿一侧镜像出
  另一侧，会授权错半个行程 —— 和天轶 servo 记的是同一个坑。限位取自仓库里已有的
  `resource/g1_model.urdf`，不是抄来的数字。

- **腕比肩弱得多。** URDF 里肩肘 effort 25 / velocity 37，腕 effort 5 /
  velocity 22。默认增益因此分两档（官方 WMA 配置里也是 kp 300 对 kp 40）。

── 两件还没在真机上确认的事 ────────────────────────────────────────────────

都做成「启动时显式检查并拒绝」，不做成默认假设：

1. **`rt/arm_sdk` 要不要算 CRC。** C++ 例程没设，而 WMA 的 Python 代码对
   `rt/lowcmd` 设了。`send_crc` 配置项默认 True（多算一次无害，少算会被丢弃）。
2. **Dex1 夹爪的消息类型。** 它是独立话题、不受 arm_sdk 权重影响，但 SDK 里的
   idl 名字没有文档。`_open` 会真的去 import，import 不到就**拒绝启动**并把缺的
   名字写出来，而不是让一张 16 维的卡片只驱动 14 维。
"""

from __future__ import annotations

import json
import math
import threading
import time

from common.control import ControlSink, Verdict, parse_descriptor

# ── 关节与话题，全部来自官方文档与仓库里的 URDF ─────────────────────────────

ARM_SDK_TOPIC = "rt/arm_sdk"
DEX1_CMD_TOPICS = {"left": "rt/dex1/left/cmd", "right": "rt/dex1/right/cmd"}
LOW_STATE_TOPIC = "rt/lowstate"

# G1_29_JointArmIndex：左臂 15..21，右臂 22..28。
ARM_MOTOR_IDS = list(range(15, 29))
# kNotUsedJoint —— SDK 把这个位置的 q 当作过渡权重，不是一个真关节。
WEIGHT_MOTOR_ID = 29

JOINT_NAMES = [
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]

# 逐字取自 resource/g1_model.urdf 的 <limit lower= upper=>。**左右不对称**，
# 见模块文档；镜像会授权错半个行程。
ARM_LIMITS = [
    (-3.0892, 2.6704), (-1.5882, 2.2515), (-2.618, 2.618), (-1.0472, 2.0944),
    (-1.9722, 1.9722), (-1.6144, 1.6144), (-1.6144, 1.6144),
    (-3.0892, 2.6704), (-2.2515, 1.5882), (-2.618, 2.618), (-1.0472, 2.0944),
    (-1.9722, 1.9722), (-1.6144, 1.6144), (-1.6144, 1.6144),
]

# URDF 的 velocity 是**硬件极限**（肩肘 37 rad/s，腕 22），不是安全值 —— 一条
# 以它为上限的指令流可以在一个控制周期里把手臂甩到对侧。和天轶那张卡同样的处理：
# 取一个保守的、说得出理由的数，而不是把硬件极限当成许可。
ARM_MAX_VELOCITY = 1.5
GRIPPER_MAX_VELOCITY = 2.0

# 增益分两档，取自宇树 WMA 部署配置（G1ArmConfig）：肩肘 kp 300/kd 3，腕
# kp 40/kd 1.5。腕的 effort 只有肩肘的五分之一，同一档增益会把它推过头。
WRIST_MOTOR_IDS = {19, 20, 21, 26, 27, 28}
KP_ARM, KD_ARM = 300.0, 3.0
KP_WRIST, KD_WRIST = 40.0, 1.5

DEFAULT_EXPECTED_HZ = 30.0
MAX_HZ = 50.0
WATCHDOG_MS = 200
MAX_OBS_AGE_MS = 300

# 接管与交还的时间。交还那一条是安全参数，不是手感参数：权重从 1 直接归零，
# 手臂瞬间脱力。官方例程用 2 秒。
TAKEOVER_S = 1.0
HANDBACK_S = 2.0
RAMP_HZ = 100.0

LEFT_ARM = slice(0, 7)
RIGHT_ARM = slice(7, 14)
LEFT_GRIPPER = 14
RIGHT_GRIPPER = 15
DOF = 16


def build_descriptor(expected_hz: float = DEFAULT_EXPECTED_HZ,
                     grippers: bool = True) -> dict:
    """The action space, derived from the URDF this repo ships.

    Limits come from `resource/g1_model.urdf` rather than from a datasheet
    transcribed by hand, so the descriptor and the skeleton renderer cannot
    disagree about what this robot is.

    **`grippers=False` really is a 14-dimension card**, not a 16-dimension one
    that quietly ignores two values. Keeping 16 here would let a 16-dim model
    negotiate successfully and then have two of its degrees of freedom go
    nowhere — the exact silent failure `control_mode` and `dof` exist to catch.
    A G1 without Dex1 grippers is a different action space, and saying so is
    what lets the model be refused instead of half-executed.
    """
    joint_names = list(JOINT_NAMES)
    lower = [low for low, _ in ARM_LIMITS]
    upper = [high for _, high in ARM_LIMITS]
    max_velocity = [ARM_MAX_VELOCITY] * 14
    groups = [
        {"name": "arm_l", "offset": 0, "count": 7,
         "unit": "rad", "resource": "arm_l"},
        {"name": "arm_r", "offset": 7, "count": 7,
         "unit": "rad", "resource": "arm_r"},
    ]
    if grippers:
        joint_names += ["left_gripper", "right_gripper"]
        lower += [0.0, 0.0]
        upper += [1.0, 1.0]
        max_velocity += [GRIPPER_MAX_VELOCITY] * 2
        groups += [
            {"name": "gripper_l", "offset": 14, "count": 1,
             "unit": "normalized", "resource": "gripper_l"},
            {"name": "gripper_r", "offset": 15, "count": 1,
             "unit": "normalized", "resource": "gripper_r"},
        ]

    period = 1.0 / expected_hz
    max_delta = [speed * period for speed in max_velocity]

    return {
        "control_interface": "motus.control/1",
        "mode": "joint_position",
        "dof": len(joint_names),
        "joint_names": joint_names,
        # 混合单位是必然的，`groups` 才是说清楚哪段是哪种的地方。
        "units": ({"angle": "rad", "normalized": "0-1", "time": "s"}
                  if grippers else {"angle": "rad", "time": "s"}),
        "limits": {
            "lower": lower,
            "upper": upper,
            "max_velocity": max_velocity,
            "max_delta_per_step": max_delta,
        },
        "groups": groups,
        "frame": "pelvis",
        "rate": {
            "max_hz": MAX_HZ,
            "expected_hz": expected_hz,
            "watchdog_ms": WATCHDOG_MS,
            "max_obs_age_ms": MAX_OBS_AGE_MS,
        },
        # G1 的手臂不向这个驱动报力矩反馈。显式声明为 null 而不是省略，
        # 这样「缺这项保护」是看得见的，不是被默认掉的。
        "force_torque": None,
    }


class G1ServoPlugin:
    """One card, one input topic, one sink, one arm_sdk publisher + two grippers."""

    PREFIX = "servo"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 arm_client=None):
        self._ns = namespace
        self._executor = executor
        config = plugin_config or {}

        self._expected_hz = float(config.get("expected_hz", DEFAULT_EXPECTED_HZ))
        if not math.isfinite(self._expected_hz) or not 0 < self._expected_hz <= MAX_HZ:
            raise ValueError(f"servo.expected_hz must be in (0, {MAX_HZ}]")
        # 见模块文档「两件还没确认的事」。默认算 —— 多算一次无害，少算会被静默丢弃。
        self._send_crc = bool(config.get("send_crc", True))
        self._grippers_enabled = bool(config.get("grippers", True))

        self._descriptor_raw = build_descriptor(self._expected_hz,
                                                self._grippers_enabled)
        self._descriptor = parse_descriptor(self._descriptor_raw)

        self._lock = threading.RLock()
        self._sink = None
        self._sub_node = None
        self._arm_pub = None
        self._gripper_pubs: dict = {}
        self._crc = None
        self._msg = None
        self._input_topic = ""
        self._running = False
        self._paused = False
        self._weight = 0.0
        self._last_target = None
        self._last_outcome = None
        self._rejects: list = []

    # ── tool ─────────────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "servo",
            "type": "actuator",
            "description": (
                "G1 双臂 + 双夹爪的连续控制：订阅一路 motus.control/1 指令流"
                f"（{self._descriptor.dof} 维，≤{self._expected_hz:g} Hz）驱动执行。"
                "普通摆姿势用 arm 的预设手势，这张卡片是给执行模型用的。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "stop", "pause", "resume", "info"]},
                    "input_topic": {"type": "string",
                                    "description": "control/joint 指令流话题"},
                },
                "required": ["action"],
                # `start`/`stop` 故意不在这里 —— agent-core 把每个条目展开成一个
                # LLM 可调函数，不在这里的动作只有画布能调。`stop` 属于项目生命
                # 周期：模型调它会把卡片从运行中的项目里摘掉而项目并不知情，而且
                # 它自己放不回去（`start` 需要输入话题和下游 descriptor）。
                "x-action-params": {
                    "pause": {"params": [],
                              "description": "立即停止执行并保持当前姿态；"
                                             "仍然订阅着，resume 可继续"},
                    "resume": {"params": [], "description": "继续执行"},
                },
                "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                            "on_interrupt_all": {"action": "pause"}},
                "x-is-dangerous": True,
                "x-resource": list(self._descriptor.resources),
            },
            "topic_in": [{"format": "control/joint",
                          "desc": ("motus.control/1，"
                                   + ("16 维（14 臂 rad + 2 夹爪 归一化）"
                                      if self._grippers_enabled
                                      else "14 维（臂 rad，未接夹爪）"))}],
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

        一张会让手臂动起来的卡片，不能因为容器重启就自己开始动。它在 agent-core
        启动项目时才订阅，而那是一个人的动作。
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
        if self._executor is None:
            return {"state": "error", "message": "没有 ROS 上下文，无法订阅"}

        sink = ControlSink(
            self._descriptor,
            self._apply,
            on_watchdog=self._hold,
            on_abort=self._hold,
        )
        # 先登记再启动，这样一个并发的 stop 找得到它、取消得掉 —— 和这个项目里
        # 每个持有实例状态的插件同一条规矩。
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

        # 接管：权重 0→1。**在这之前一条关节指令都没发过**，所以渐入期间手臂停在
        # 内置控制器给它的位置上，不会先跳到一个目标再开始受控。
        self._ramp(0.0, 1.0, TAKEOVER_S)
        print(f"[servo] streaming from {topic}", flush=True)
        return {"state": "running", "input": topic,
                "control_interface": self._descriptor_raw}

    def _halt(self, halted: bool):
        """`pause` 与 `resume`。停止执行，保持订阅，**权重不动**。

        权重留在 1：这些关节保持最后的目标，所以「不再发新指令」本身就是保持。
        把权重降下去等于让手臂在暂停的瞬间脱力，而暂停的语义是「等一下」，不是
        「松手」—— 手上有东西的时候后者是更坏的那个答案。
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
        with self._lock:
            if not self._running:
                return {"state": "idle"}
            self._running = False
            self._paused = False
            sink, node = self._sink, self._sub_node
            self._sink = None
            self._sub_node = None

        # 交还：权重 1→0，两秒。**同步等它跑完**再往下拆东西 —— 中途把发布器
        # 撤掉，权重就停在半路，而那是一个既不受我们控制也没完全交还的状态。
        try:
            self._ramp(self._weight, 0.0, HANDBACK_S)
        except Exception as exc:  # noqa: BLE001 —— 交还失败要说出来，但不能挡住清理
            print(f"[servo] handback failed: {exc}", flush=True)

        if node is not None:
            try:
                self._executor.remove_node(node)
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        with self._lock:
            self._arm_pub = None
            self._gripper_pubs = {}
            self._last_target = None
        del sink
        return {"state": "stopped"}

    def _info(self):
        with self._lock:
            return {
                "state": ("running" if self._running and not self._paused else
                          "paused" if self._running else "idle"),
                "input": self._input_topic,
                "weight": round(self._weight, 3),
                "grippers": self._grippers_enabled,
                "control_interface": self._descriptor_raw,
                "last": self._last_outcome,
                "rejects": list(self._rejects),
            }

    # ── wiring ───────────────────────────────────────────────────────────────

    def _open(self, topic: str):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                       # 排着的指令就是过期的指令
            durability=DurabilityPolicy.VOLATILE,
        )

        node = Node("g1_servo", context=None)
        node.create_subscription(String, topic, self._on_message, qos)
        # 看门狗要自己的心跳：没有消息就没有回调，而没有消息正是它要发现的东西。
        node.create_timer(WATCHDOG_MS / 2000.0, self._tick)
        self._executor.add_node(node)

        arm_pub = ChannelPublisher(ARM_SDK_TOPIC, LowCmd_)
        arm_pub.Init()

        gripper_pubs = {}
        if self._grippers_enabled:
            # 真的去 import，import 不到就**拒绝启动**。让一张声明 16 维的卡片
            # 只驱动 14 维，是协商通过之后再静默少动两个自由度 —— 正是这套协议
            # 存在的理由。
            try:
                from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
            except ImportError as exc:
                raise RuntimeError(
                    "Dex1 夹爪的消息类型 unitree_go.msg.dds_.MotorCmds_ 不在这个 "
                    f"SDK 里（{exc}）。要么装上带这个 idl 的 unitree_sdk2py，"
                    "要么在 config.yaml 里设 servo.grippers=false —— 那会把这张卡"
                    "降成 14 维，而 16 维的模型将协商失败，这是对的"
                ) from exc
            for side, gripper_topic in DEX1_CMD_TOPICS.items():
                publisher = ChannelPublisher(gripper_topic, MotorCmds_)
                publisher.Init()
                gripper_pubs[side] = publisher

        message = unitree_hg_msg_dds__LowCmd_()
        for motor_id in ARM_MOTOR_IDS:
            if motor_id in WRIST_MOTOR_IDS:
                message.motor_cmd[motor_id].kp = KP_WRIST
                message.motor_cmd[motor_id].kd = KD_WRIST
            else:
                message.motor_cmd[motor_id].kp = KP_ARM
                message.motor_cmd[motor_id].kd = KD_ARM
            message.motor_cmd[motor_id].tau = 0.0
            message.motor_cmd[motor_id].dq = 0.0

        crc = None
        if self._send_crc:
            from unitree_sdk2py.utils.crc import CRC
            crc = CRC()

        with self._lock:
            self._sub_node = node
            self._arm_pub = arm_pub
            self._gripper_pubs = gripper_pubs
            self._msg = message
            self._crc = crc

    # ── the stream ───────────────────────────────────────────────────────────

    def _on_message(self, message):
        sink = self._sink
        if sink is None or self._paused:
            # 丢掉而不是排队：一条跨过暂停被留下来的指令，算它的时候世界还没动，
            # resume 时应用它就是从过期数据往前跳一步。
            return
        try:
            payload = json.loads(message.data)
        except Exception as exc:  # noqa: BLE001
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
        """只有通过了 sink 里每一道检查的指令才会到这里。"""
        with self._lock:
            self._last_target = list(values)
        self._publish_arms(values[LEFT_ARM], values[RIGHT_ARM])
        if self._grippers_enabled:
            self._publish_gripper("left", values[LEFT_GRIPPER])
            self._publish_gripper("right", values[RIGHT_GRIPPER])

    def _publish_arms(self, left, right):
        publisher, message = self._arm_pub, self._msg
        if publisher is None or message is None:
            return
        for motor_id, radians in zip(ARM_MOTOR_IDS, list(left) + list(right)):
            # 已经是弧度：descriptor 就用线上的单位，所以这里没有换算可以弄反。
            message.motor_cmd[motor_id].q = float(radians)
        message.motor_cmd[WEIGHT_MOTOR_ID].q = float(self._weight)
        if self._crc is not None:
            message.crc = self._crc.Crc(message)
        publisher.Write(message)

    def _publish_gripper(self, side: str, closure):
        publisher = self._gripper_pubs.get(side)
        if publisher is None:
            return
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_

        command = unitree_go_msg_dds__MotorCmd_()
        # Dex1 的行程就是 0..1，和 descriptor 同向 —— 天轶那两只手是反的，这里
        # 不是。不反的时候不要"为了对称"加一个取反，那正是把"张开"变成"攥紧"的方式。
        command.q = float(closure)
        command.kp = 5.0
        command.kd = 0.1
        message = MotorCmds_(cmds=[command])
        publisher.Write(message)

    def _ramp(self, start: float, end: float, seconds: float):
        """把 arm_sdk 的混合权重从 start 渐变到 end，并在这期间持续发布。

        接管和交还都必须渐变，理由不对称：

        - **接管**（0→1）渐入，是因为内置控制器还握着这些电机，权重一跳到 1 等于
          在一个控制周期里完成交接。官方例程用 `weight*weight`，这里照抄那条曲线
          而不是线性 —— 它前段更慢。
        - **交还**（1→0）渐出，是因为权重归零的那一刻手臂脱力。两秒是官方例程的值。

        渐变期间发的是**最后一个被接受的目标**，没有目标就只发权重（关节字段保持
        上一次的值，电机因此停在原地）。
        """
        publisher, message = self._arm_pub, self._msg
        if publisher is None or message is None:
            return
        steps = max(1, int(seconds * RAMP_HZ))
        period = 1.0 / RAMP_HZ
        for i in range(1, steps + 1):
            fraction = i / steps
            weight = start + (end - start) * fraction
            # 接管走平方曲线（前段更慢），交还走线性 —— 和官方例程一致。
            blended = weight * weight if end > start else weight
            self._weight = max(0.0, min(1.0, blended))
            target = self._last_target
            if target is not None:
                for motor_id, radians in zip(ARM_MOTOR_IDS, target[:14]):
                    message.motor_cmd[motor_id].q = float(radians)
            message.motor_cmd[WEIGHT_MOTOR_ID].q = float(self._weight)
            if self._crc is not None:
                message.crc = self._crc.Crc(message)
            publisher.Write(message)
            time.sleep(period)
        self._weight = max(0.0, min(1.0, end))

    def _hold(self):
        """看门狗与 abort 的处置：停止发布，**不动权重**。

        这些关节保持最后的目标，所以不发新指令本身就是保持。把权重降下去会让手臂
        在指令流断掉的瞬间脱力 —— 而指令流断掉最常见的原因是上游模型卡了一拍，
        那时静止是正确的答案，脱力不是。

        真要松手是 `stop`，它是一个人的决定，并且带两秒的渐出。
        """
        with self._lock:
            self._last_target = None


PROVIDER = G1ServoPlugin
