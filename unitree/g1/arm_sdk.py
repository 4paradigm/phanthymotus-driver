"""G1 上肢的下发通道：`rt/arm_sdk` 的权重混合、增益、渐变，以及两只 Dex1。

从 `servo.py` 里抽出来的，因为现在有**两张**卡片要用它：`servo`（16 维关节空间）
和 `servo_eef`（19 维末端位姿，解完 IK 之后落到同样这 14 个关节）。

抽出来而不是抄第二份，理由不是整洁：这里面有安全逻辑。交还时的两秒渐出、看门狗
「不动权重」的处置、接管前不发任何关节指令 —— 抄成两份之后，改动只落在其中一份的
那天不会有任何报错，而分叉的那一份正在某台机器人上跑着。

── 这条通道选的是 `rt/arm_sdk`，不是 `rt/lowcmd` ──────────────────────────

**`rt/lowcmd`** 接管全部 29 个关节。宇树官方文档《底层运动开发》：

    一旦 G1 开机，内置的运动控制程序会自动启动……如果您在这种状态下使用 SDK 进行
    底层开发，可能会导致指令冲突，从而使 G1 出现抖动的情况。因此……请务必确保
    G1 已经进入调试模式（L2+A），以停止运动控制程序发送指令。

也就是这条路要求人先把平衡控制器关掉。宇树自己的 `unifolm-world-model-action`
部署代码（`robot_devices/arm/g1_arm.py`）走的正是它 —— 那是给调试态准备的，抄进
一个会被 agent-core 随时 start 的卡片里是危险的。

**`rt/arm_sdk`** 是高层运控服务提供的上肢接口，官方文档《手臂控制例程》：**无需
进入调试模式**，机器人保持锁定站立即可。它靠一个混合权重和内置控制器共存：

    msg.motor_cmd[kNotUsedJoint].q = weight     # 0→1 接管，1→0 交还
    msg.motor_cmd[arm_joint].q/dq/kp/kd/tau     # 只写手臂那 14 个

代价是接管和交还都要**渐变**，不能瞬间切 —— 见 `ramp`。
"""

from __future__ import annotations

import threading
import time

ARM_SDK_TOPIC = "rt/arm_sdk"
DEX1_CMD_TOPICS = {"left": "rt/dex1/left/cmd", "right": "rt/dex1/right/cmd"}

# G1_29_JointArmIndex：左臂 15..21，右臂 22..28。
ARM_MOTOR_IDS = list(range(15, 29))
# kNotUsedJoint —— SDK 把这个位置的 q 当作过渡权重，不是一个真关节。
WEIGHT_MOTOR_ID = 29

# 腰。**只有 yaw 在官方 arm7 例程的可写列表里**；roll 与 pitch 能不能经 arm_sdk 写
# 没有文档，而且不少 G1 的这两个轴本来就是锁死的。所以卡片侧用限位把它们卡住，
# 这里只负责「给了就写」。
WAIST_MOTOR_IDS = {"yaw": 12, "roll": 13, "pitch": 14}

# 增益分两档，取自宇树 WMA 部署配置（G1ArmConfig）：肩肘 kp 300/kd 3，腕
# kp 40/kd 1.5。腕的 effort 只有肩肘的五分之一，同一档增益会把它推过头。
WRIST_MOTOR_IDS = {19, 20, 21, 26, 27, 28}
KP_ARM, KD_ARM = 300.0, 3.0
KP_WRIST, KD_WRIST = 40.0, 1.5
KP_WAIST, KD_WAIST = 200.0, 5.0

# 接管与交还的时间。交还那一条是**安全参数**，不是手感参数：权重从 1 直接归零，
# 手臂瞬间脱力。官方例程用 2 秒。
TAKEOVER_S = 1.0
HANDBACK_S = 2.0
RAMP_HZ = 100.0


class ArmSdkChannel:
    """一条 arm_sdk 发布链路，外加两只 Dex1。

    不订阅任何东西、不认识 descriptor、不知道动作空间是关节角还是位姿 —— 卡片把
    「最终要发下去的 14 个关节角」交给它。`servo_eef` 解完 IK 之后交的和 `servo`
    直接收到的是同一种东西，这正是它能被两张卡片共用的原因。
    """

    def __init__(self, *, send_crc: bool = True, grippers: bool = True,
                 waist: bool = False):
        self._send_crc = bool(send_crc)
        self._grippers = bool(grippers)
        self._waist = bool(waist)
        self._lock = threading.RLock()
        self._arm_pub = None
        self._gripper_pubs: dict = {}
        self._message = None
        self._crc = None
        self._weight = 0.0
        self._last_target = None

    # ── 生命周期 ─────────────────────────────────────────────────────────────

    def open(self):
        """建发布器。Dex1 的 idl 缺席时**拒绝启动**，不降级。

        让一张声明了 16 维的卡片只驱动 14 维，是协商通过之后再静默少动两个自由度
        —— 正是这套协议存在的理由。
        """
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

        arm_pub = ChannelPublisher(ARM_SDK_TOPIC, LowCmd_)
        arm_pub.Init()

        gripper_pubs = {}
        if self._grippers:
            try:
                from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
            except ImportError as exc:
                raise RuntimeError(
                    "Dex1 夹爪的消息类型 unitree_go.msg.dds_.MotorCmds_ 不在这个 "
                    f"SDK 里（{exc}）。要么装上带这个 idl 的 unitree_sdk2py，"
                    "要么在 config.yaml 里把 grippers 设成 false —— 那会把卡片降成"
                    "不带夹爪的宽度，而带夹爪的模型将协商失败，这是对的"
                ) from exc
            for side, topic in DEX1_CMD_TOPICS.items():
                publisher = ChannelPublisher(topic, MotorCmds_)
                publisher.Init()
                gripper_pubs[side] = publisher

        message = unitree_hg_msg_dds__LowCmd_()
        motor_ids = list(ARM_MOTOR_IDS)
        if self._waist:
            motor_ids += sorted(WAIST_MOTOR_IDS.values())
        for motor_id in motor_ids:
            if motor_id in WRIST_MOTOR_IDS:
                message.motor_cmd[motor_id].kp = KP_WRIST
                message.motor_cmd[motor_id].kd = KD_WRIST
            elif motor_id in WAIST_MOTOR_IDS.values():
                message.motor_cmd[motor_id].kp = KP_WAIST
                message.motor_cmd[motor_id].kd = KD_WAIST
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
            self._arm_pub = arm_pub
            self._gripper_pubs = gripper_pubs
            self._message = message
            self._crc = crc

    def close(self):
        with self._lock:
            self._arm_pub = None
            self._gripper_pubs = {}
            self._last_target = None

    # ── 状态 ─────────────────────────────────────────────────────────────────

    @property
    def weight(self) -> float:
        return self._weight

    @property
    def last_target(self):
        return self._last_target

    def forget_target(self):
        """看门狗与 abort 的处置：忘掉目标，**不动权重**。

        这些关节是位置控制的，控制器保持最后一个目标，所以「不再发新指令」本身就是
        保持。把权重降下去会让手臂在指令流断掉的瞬间脱力 —— 而指令流断掉最常见的
        原因是上游模型卡了一拍，那时静止是正确的答案，脱力不是。

        真要松手是 `stop`，那是一个人的决定，并且带两秒的渐出。
        """
        with self._lock:
            self._last_target = None

    # ── 发布 ─────────────────────────────────────────────────────────────────

    def publish_arms(self, radians, waist=None):
        """14 个臂关节角（弧度），可选 3 个腰关节角 `(roll, pitch, yaw)`。"""
        with self._lock:
            publisher, message = self._arm_pub, self._message
            self._last_target = list(radians)
        if publisher is None or message is None:
            return
        for motor_id, value in zip(ARM_MOTOR_IDS, radians):
            # 已经是弧度：descriptor 用的就是线上的单位，这里没有换算可以弄反。
            message.motor_cmd[motor_id].q = float(value)
        if waist is not None and self._waist:
            roll, pitch, yaw = (float(v) for v in waist)
            message.motor_cmd[WAIST_MOTOR_IDS["roll"]].q = roll
            message.motor_cmd[WAIST_MOTOR_IDS["pitch"]].q = pitch
            message.motor_cmd[WAIST_MOTOR_IDS["yaw"]].q = yaw
        self._write(message)

    def publish_gripper(self, side: str, closure):
        publisher = self._gripper_pubs.get(side)
        if publisher is None:
            return
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_

        command = unitree_go_msg_dds__MotorCmd_()
        # Dex1 的行程就是 0..1，和 descriptor 同向 —— 天轶那两只手是反的，这里
        # 不是。不反的时候不要"为了对称"加一个取反，那正是把"张开"变成"攥紧"的方式。
        command.q = float(closure)
        command.kp = 5.0
        command.kd = 0.1
        publisher.Write(MotorCmds_(cmds=[command]))

    def ramp(self, start: float, end: float, seconds: float):
        """把混合权重从 start 渐变到 end，并在这期间持续发布。

        接管和交还都必须渐变，理由不对称：

        - **接管**（0→1）渐入，是因为内置控制器还握着这些电机，权重一跳到 1 等于
          在一个控制周期里完成交接。官方例程用 `weight*weight`，这里照抄那条曲线
          而不是线性 —— 它前段更慢。
        - **交还**（1→0）渐出，是因为权重归零的那一刻手臂脱力。两秒是官方例程的值。

        渐变期间发的是**最后一个被接受的目标**，没有目标就只发权重（关节字段保持
        上一次的值，电机因此停在原地）。
        """
        publisher, message = self._arm_pub, self._message
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
                for motor_id, value in zip(ARM_MOTOR_IDS, target[:14]):
                    message.motor_cmd[motor_id].q = float(value)
            self._write(message)
            time.sleep(period)
        self._weight = max(0.0, min(1.0, end))

    def _write(self, message):
        message.motor_cmd[WEIGHT_MOTOR_ID].q = float(self._weight)
        if self._crc is not None:
            message.crc = self._crc.Crc(message)
        publisher = self._arm_pub
        if publisher is not None:
            publisher.Write(message)
