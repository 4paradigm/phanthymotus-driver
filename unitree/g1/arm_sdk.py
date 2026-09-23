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

── `rt/arm_sdk` 要求**运控服务正在运行**，这是它唯一的前置 ──────────────

这条是真机上花了最久才定位的，而且中途走错过一次，错法值得留下来。

`rt/arm_sdk` **不是**一条直接写电机的通道，它是高层运控服务提供的接口，由那个服务
做混合：`实际执行 = 运控指令 × (1 − weight) + arm_sdk 指令 × weight`。运控服务没在
跑，就没有人订阅 `rt/arm_sdk` —— 权重、增益、CRC、IK 全都正确，消息发进虚空，手臂
纹丝不动，**没有任何一处报错**。

判据是问 loco 服务要一次状态，不是看日志：

    GetFsmId() -> (3102, None)
    [Writer] no subscriber matched rt/api/sport/request — is the peer service running?

FSM 500（Regular Mode，1 自由度腰）下同一条指令立刻生效，实测左肘 −0.0437 rad
（指令 −0.05）。

**和 `rt/lowcmd` 的前置正好相反**：那条要求进调试模式、把运控停掉。选错通道加选错
模式，两次都不会报错。

**注意 FSM 801（AI Run）不要用。** 宇树 issue #182 记录了在 locomotion 模式下使用
`rt/arm_sdk` 触发单向不可恢复的 FSM 转换，之后机器人走不了路。

── `mode_machine` 回传：按文档做，但**实测它对 arm_sdk 没有影响** ──────────

`LowCmd_` 有 `mode_machine`/`mode_pr` 两个字段，默认 0，官方文档说控制端要把
`rt/lowstate` 读到的值原样填回去，否则机器人忽略低层指令。

我们照做了，但要说清楚：**这条在 arm_sdk 上被实测证伪。** FSM 500 下做过 A/B ——
回传 4 和回传 0，左肘位移分别是 −0.0437 和 −0.0436 rad，没有差别。

保留回传是因为它零成本且符合文档（那条要求很可能是针对 `rt/lowcmd` 的），但**不要
把它当成"指令不生效"的排查方向** —— 曾经有人（我）在这上面找错了方向，真正的原因
是上面那条。

── 这台机器人有几个关节，要问它，不要问 URDF ──────────────────────────────

`rt/lowstate` 里每个电机有一个 `mode` 字段，0 表示这个关节**不存在或未使能**。
实测的这台 G1：两条手臂的 `wrist_pitch`/`wrist_yaw` 都是 `mode=0`、q 恒为 0，腰只有
yaw 是 1 —— 也就是 **5 自由度手臂 + 1 自由度腰**（官方叫 23dof / arm5），而仓库里的
`g1_model.urdf` 是 29dof（arm7 + 3 自由度腰）。

差别不会报错。IK 在 7 自由度模型里解，把 +0.0845 rad 放进一个物理上不存在的
`wrist_pitch`，残差报 0.26 mm（在**模型**里确实解出来了），然后机器人做不到那个位姿。
**残差校验的是求解器，不是机器人。**

所以 `open()` 会核对每一个要驱动的关节的 `mode`，有 0 就拒绝启动并点名 —— 把一次
静默的「姿态对不上」变成一次启动期的响亮失败。

── 接管之前必须先知道手臂在哪 ──────────────────────────────────────────────

`LowCmd_` 这个消息里每个 `motor_cmd[i].q` 的默认值是 **0.0**，而 0 不是「不控制」，
是「去零位」。所以 `open()` 如果只填 kp/kd 就开始渐入权重，发出去的是一条「双臂去
全零位」的指令 —— 零位是手臂伸直贴体侧，而卡片启动时手臂通常不在那儿。表现是
**一 start 两条胳膊就以 kp=300 在一秒内甩直**，而日志里什么都没有。

所以 `open()` 先订一帧 `rt/lowstate`，拿实测关节角给 `q` 打底，并把它设成
`_last_target`。**读不到就拒绝启动**：不知道手臂在哪就接管，正是最危险的那一种，
而「拒绝」是唯一能让它变成一次可观察失败的处置。
"""

from __future__ import annotations

import threading
import time

ARM_SDK_TOPIC = "rt/arm_sdk"
DEX1_CMD_TOPICS = {"left": "rt/dex1/left/cmd", "right": "rt/dex1/right/cmd"}
# Dex1 有**两种接法**，而只探一种等于没探。
#
# - 外接：夹爪走 USB-485 小板，`dex1_1_service` 把串口桥成 DDS，于是
#   `rt/dex1/<side>/{cmd,state}` 存在。这是 `DEX1_CMD_TOPICS` 假定的那条路。
# - 内部走线：夹爪直接挂在机器人内部总线上，**没有那两个话题**，而是占用
#   `LowState_.motor_state` 的 31（左）/ 33（右）—— 宇树 `xr_teleoperate` 的
#   `--ee dex1_internal` 走的就是这条。
#
# 判存在必须两条都问：内部走线的机器人在外接话题上永远是静默的，反之亦然。
DEX1_INTERNAL_MOTOR_IDS = {"left": 31, "right": 33}
DEX1_STATE_TOPICS = {"left": "rt/dex1/left/state", "right": "rt/dex1/right/state"}
# 外接那条路要等一帧。夹爪以 1 kHz 报状态，一秒足够；这一步只在内部走线已经判否
# 之后才跑，所以装了内部夹爪的机器人不为它付任何时间。
DEX1_STATE_TIMEOUT_S = 1.0
# 实测关节角。接管前必须读到一帧，见 `_read_measured_arms`。
LOW_STATE_TOPIC = "rt/lowstate"

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


def read_low_state_once(timeout_s: float = 2.0):
    """等一帧 `rt/lowstate` 并**立刻关掉订阅**。纯读，不发布任何东西。

    抽成模块函数，是因为有两个调用方：通道要用它拿接管前的实测角，卡片要用它判
    机器人型号（arm5 还是 arm7）。各建一条用完即关的订阅，比让一方去问另一方要
    缓存清楚 —— 缓存会过期，而这两处都要的是「此刻」。

    读完就关：留着意味着一条 500 Hz 的订阅活到进程结束，而这个仓库里「孤儿订阅」
    的历史是回调还在跑、ROS 图看着健康，直到某次 teardown 顺序出错把整层带走。
    """
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    received: dict = {}
    subscriber = ChannelSubscriber(LOW_STATE_TOPIC, LowState_)
    subscriber.Init(lambda message: received.setdefault("m", message), 10)
    try:
        deadline = time.monotonic() + timeout_s
        while "m" not in received and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        try:
            subscriber.Close()
        except Exception:  # noqa: BLE001
            pass
    if "m" not in received:
        raise RuntimeError(
            f"{timeout_s:g} 秒内没有收到 {LOW_STATE_TOPIC}。"
            "检查 NETWORK_INTERFACE 和机器人是否上电"
        )
    return received["m"]


def detect_dex1(motor_state, timeout_s: float = DEX1_STATE_TIMEOUT_S) -> str:
    """机器人上到底有没有 Dex1。返回 `""`（没有）或接法名。

    **问机器人，不问配置** —— 和 `servo_eef.detect_variant` 同一条规矩，理由也
    一样：配置是人填的，而这里填错不报错。办公室那台 G1 的 `config.yaml` 写着
    `grippers: true`，机器人上两条接法一条都没有，于是卡片多声明两维、往虚空发
    两条夹爪指令，协商照过。

    先看内部走线（`motor_state` 已经在手上，零成本），没有再去等外接话题。
    """
    present = [side for side, motor_id in DEX1_INTERNAL_MOTOR_IDS.items()
               if motor_id < len(motor_state)
               and int(getattr(motor_state[motor_id], "mode", 0)) != 0]
    if present:
        return "internal"

    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
    except ImportError:
        # idl 缺席在 `open()` 里是一条独立的、更具体的拒绝。这里只负责「探不到」。
        return ""

    seen: dict = {}
    subscribers = []
    try:
        for side, topic in DEX1_STATE_TOPICS.items():
            subscriber = ChannelSubscriber(topic, MotorStates_)
            subscriber.Init(lambda m, side=side: seen.setdefault(side, m), 10)
            subscribers.append(subscriber)
        deadline = time.monotonic() + timeout_s
        while not seen and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        for subscriber in subscribers:
            try:
                subscriber.Close()
            except Exception:  # noqa: BLE001 —— 关不掉也不能挡住启动
                pass
    return "external" if seen else ""


class ArmSdkChannel:
    """一条 arm_sdk 发布链路，外加两只 Dex1。

    不订阅任何东西、不认识 descriptor、不知道动作空间是关节角还是位姿 —— 卡片把
    「最终要发下去的 14 个关节角」交给它。`servo_eef` 解完 IK 之后交的和 `servo`
    直接收到的是同一种东西，这正是它能被两张卡片共用的原因。
    """

    # 等一帧 `rt/lowstate` 的上限。它以 500 Hz 发，正常情况下第一帧在几毫秒内就到；
    # 给到 2 秒是为了容忍 DDS 发现（joining 一个已有的 domain 要握手）。超时不重试
    # —— 超时本身就说明这条链路不通，而在不通的链路上接管手臂是最坏的选择。
    LOW_STATE_TIMEOUT_S = 2.0

    def __init__(self, *, send_crc: bool = True, grippers: bool = True,
                 waist: bool = False, driven_arm_ids=None,
                 driven_waist_names=None):
        self._send_crc = bool(send_crc)
        self._grippers = bool(grippers)
        self._waist = bool(waist)
        # 这台机器人真正要驱动的臂关节。23dof/arm5 的机器上 20/21/27/28 不存在，
        # 把它们算进来会让下面的 mode 核对误判成「机器人坏了」。
        self._driven_arm_ids = list(driven_arm_ids or ARM_MOTOR_IDS)
        # 腰同理：23dof 的机器上只有 yaw 存在（roll/pitch 报 mode=0）。写成名字
        # 而不是下标，是因为调用方按 roll/pitch/yaw 思考，而下标是 13/14/12 ——
        # 那个顺序每次都要回去查表，而查错不报错。
        self._driven_waist = tuple(driven_waist_names
                                   if driven_waist_names is not None
                                   else WAIST_MOTOR_IDS)
        self._lock = threading.RLock()
        self._arm_pub = None
        self._gripper_pubs: dict = {}
        self._message = None
        self._crc = None
        self._weight = 0.0
        self._last_target = None
        self._measured_waist: dict = {}
        self._mode_machine = 0
        self._mode_pr = 0
        # `open()` 探到的 Dex1 接法（"internal" / "external"），供状态回报用。
        self._dex1 = ""

    # ── 生命周期 ─────────────────────────────────────────────────────────────

    def open(self):
        """建发布器，并用**实测关节角**给指令打底。

        两处都会拒绝启动而不是降级：Dex1 的 idl 缺席（让一张声明 16 维的卡片只驱动
        14 维，是协商通过之后再静默少动两个自由度），以及读不到 `rt/lowstate`
        （不知道手臂在哪就接管 —— 见模块文档）。
        """
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

        arm_pub = ChannelPublisher(ARM_SDK_TOPIC, LowCmd_)
        # Retain partially initialized publishers so a failed stream claim can
        # close them explicitly instead of leaving an unowned DDS writer.
        with self._lock:
            self._arm_pub = arm_pub
        arm_pub.Init()

        gripper_pubs = {}
        with self._lock:
            self._gripper_pubs = gripper_pubs
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
                gripper_pubs[side] = publisher
                publisher.Init()

        # **先读实测，再建消息。** 顺序是有意的：读失败就在建立任何发布器之后、
        # 发出任何一条指令之前抛出，而 `_start` 会把卡片回滚成 idle。
        measured = self._read_measured_arms()

        message = unitree_hg_msg_dds__LowCmd_()
        # **硬件型号握手。** 不回传这两个字段，机器人静默忽略整条指令 —— 见模块文档。
        message.mode_machine = self._mode_machine
        message.mode_pr = self._mode_pr
        motor_ids = list(ARM_MOTOR_IDS)
        if self._waist:
            motor_ids += [WAIST_MOTOR_IDS[name] for name in self._driven_waist]
        motor_ids = [i for i in motor_ids
                     if i in self._driven_arm_ids or i in WAIST_MOTOR_IDS.values()]
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
        # q 必须逐个填成实测值。漏掉它，默认的 0.0 就是「去零位」。
        for motor_id, value in zip(self._driven_arm_ids, measured):
            message.motor_cmd[motor_id].q = float(value)
        if self._waist:
            # 腰同理。这里读到的是实测腰角，第一条真正的指令会覆盖它。
            for name in self._driven_waist:
                message.motor_cmd[WAIST_MOTOR_IDS[name]].q = float(
                    self._measured_waist.get(name, 0.0))

        crc = None
        if self._send_crc:
            from unitree_sdk2py.utils.crc import CRC
            crc = CRC()

        with self._lock:
            self._arm_pub = arm_pub
            self._gripper_pubs = gripper_pubs
            self._message = message
            self._crc = crc
            # 渐入期间 `ramp` 重发的就是它 —— 也就是手臂此刻所在的位置。
            self._last_target = list(measured)

    def close(self):
        with self._lock:
            publishers = [self._arm_pub, *self._gripper_pubs.values()]
            self._arm_pub = None
            self._gripper_pubs = {}
            self._last_target = None
        for publisher in publishers:
            close = getattr(publisher, 'Close', None)
            if close is not None:
                close()

    def _read_measured_arms(self):
        """等一帧 `rt/lowstate`，返回 14 个臂关节的实测角（左 7 + 右 7）。

        **纯读，不发布任何东西。** 它唯一的作用是回答「接管的那一刻，手臂在哪」，
        而这个问题没有安全的默认答案：`LowCmd_` 里 `q` 的默认值 0.0 是「去零位」，
        不是「不动」。

        读不到就抛 —— 调用方（`_start`）会把卡片回滚成 idle 并把原因报出去。这比
        「用零位兜底」好，理由和这个仓库里每一条「拒绝而不是猜」一样：猜错的代价
        不是一条报错，是手臂以 kp=300 甩到一个没人要求过的位置。
        """
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        received: dict = {}

        def _on_state(message):
            if "m" not in received:
                received["m"] = message

        # **只要一帧，拿到就关。** 留着它意味着一条 500 Hz 的订阅活到进程结束，
        # 而这个仓库里「孤儿订阅」的历史是：卡片 stop 之后回调还在跑，图看着健康，
        # 全栈订阅却在某一次 teardown 顺序错误时一起停摆。
        subscriber = ChannelSubscriber(LOW_STATE_TOPIC, LowState_)
        subscriber.Init(_on_state, 10)
        try:
            deadline = time.monotonic() + self.LOW_STATE_TIMEOUT_S
            while "m" not in received and time.monotonic() < deadline:
                time.sleep(0.01)
        finally:
            # SDK 的 `Close()` 是 `del self.__reader` 然后置 None，而 DDS 的回调
            # 可能正在飞 —— 那会在日志里留一条 `'NoneType' object has no attribute
            # 'take'`。**这是 SDK 自己的 teardown 竞态，不是我们的错**，而且确定性地
            # 关掉比让它被 GC 回收要好：窗口从「不确定」缩到几微秒，且只有一次。
            try:
                subscriber.Close()
            except Exception:  # noqa: BLE001 —— 关不掉也不能挡住启动
                pass

        if "m" not in received:
            raise RuntimeError(
                f"{self.LOW_STATE_TIMEOUT_S:g} 秒内没有收到 {LOW_STATE_TOPIC} —— "
                "拿不到实测关节角就不能接管手臂（指令里 q 的默认值 0.0 是「去零位」，"
                "不是「不动」）。检查 NETWORK_INTERFACE 和机器人是否上电"
            )

        frame = received["m"]
        # 型号握手用的两个字段，和实测角来自同一帧。
        self._mode_machine = int(getattr(frame, "mode_machine", 0))
        self._mode_pr = int(getattr(frame, "mode_pr", 0))

        motors = frame.motor_state
        # **这台机器人真有这些关节吗。** `mode == 0` 表示不存在或未使能，而往一个
        # mode=0 的关节写目标不会报错，只会什么都不发生 —— 然后整条手臂到不了 IK
        # 解出来的那个位姿，而残差（在模型里算的）一切正常。
        absent = [motor_id for motor_id in self._driven_arm_ids
                  if int(getattr(motors[motor_id], "mode", 1)) == 0]
        if self._waist:
            absent += [WAIST_MOTOR_IDS[name] for name in self._driven_waist
                       if int(getattr(motors[WAIST_MOTOR_IDS[name]], "mode", 1)) == 0]
        if absent:
            raise RuntimeError(
                f"要驱动的关节里有 mode=0（不存在或未使能）：{absent}。"
                f"本机 mode_machine={self._mode_machine}。往不存在的关节写目标不报错，"
                "只会让手臂到不了 IK 解出来的位姿，而残差一切正常 —— 所以这里拒绝"
            )

        # **夹爪同理，而且它此前是唯一一处还在信配置的。** 型号（arm5/arm7）和
        # 关节使能都是问机器人得来的，夹爪装没装却只看 `config.yaml` —— 同一类坑
        # 修了一个漏了一个。少这一维不会报错：卡片照样声明 19 维、照样往
        # `rt/dex1/*/cmd` 发，而那个话题在没装夹爪的机器上没有任何订阅者。
        if self._grippers:
            self._dex1 = detect_dex1(motors)
            if not self._dex1:
                raise RuntimeError(
                    f"配置里 grippers=true，但这台机器人两条接法都探不到 Dex1："
                    f"内部走线的 motor[{DEX1_INTERNAL_MOTOR_IDS['left']}]/"
                    f"[{DEX1_INTERNAL_MOTOR_IDS['right']}] 都是 mode=0，外接的 "
                    f"{DEX1_STATE_TIMEOUT_S:g} 秒内也没有 rt/dex1/*/state。"
                    "把 config.yaml 的 grippers 设成 false —— 那会让这张卡少声明"
                    "两维，于是带夹爪的模型**协商失败**，而不是被执行一半"
                )
            if self._dex1 == "internal":
                # 探到了，但我们驱动不了它：发布器指向 `rt/dex1/*/cmd`，而内部走线
                # 的夹爪不听那个话题，它要写进 LowCmd_ 的 31/33 号电机。没实现的
                # 路径要响亮地说没实现，而不是往一个没人订阅的话题上发。
                raise RuntimeError(
                    f"这台机器人的 Dex1 是**内部走线**（motor "
                    f"{DEX1_INTERNAL_MOTOR_IDS['left']}/"
                    f"{DEX1_INTERNAL_MOTOR_IDS['right']} 在报状态），而这里只实现了"
                    f"外接那条路（发 {DEX1_CMD_TOPICS['left']}）。内部走线的夹爪不听"
                    "那个话题，发下去不会报错也不会动。先把 grippers 设成 false，"
                    "或者把这条链路补成写 LowCmd_ 的 31/33 号电机"
                )

        measured = [float(motors[i].q) for i in self._driven_arm_ids]
        # **三个轴都读，不只是要驱动的那些。** 一个腰不被驱动的调用方
        # （`servo_eef` 的 advisory 腰）恰恰最需要知道腰**实际在哪** —— 它要把这
        # 个值挂进 IK 链。只读 `_driven_waist` 的话那里会拿到空字典、退化成零，
        # 而零是「躯干笔直」，不是「躯干此刻的样子」。
        self._measured_waist = {
            name: float(motors[motor_id].q)
            for name, motor_id in WAIST_MOTOR_IDS.items()
        }
        return measured

    # ── 状态 ─────────────────────────────────────────────────────────────────

    @property
    def weight(self) -> float:
        return self._weight

    @property
    def measured_waist(self) -> dict:
        """接管那一刻读到的腰角，`{"roll"|"pitch"|"yaw": rad}`。三个轴都有。

        给 advisory 腰的 IK 用：躯干不会被驱动，所以它停在这里，而链上必须挂它
        实际所在的角度，不是指令要求的角度。
        """
        return dict(self._measured_waist)

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
        for motor_id, value in zip(self._driven_arm_ids, radians):
            # 已经是弧度：descriptor 用的就是线上的单位，这里没有换算可以弄反。
            message.motor_cmd[motor_id].q = float(value)
        if waist is not None and self._waist:
            # 入参永远是标准布局的 [roll, pitch, yaw]，只写这台机器人真有的那些。
            values = dict(zip(("roll", "pitch", "yaw"), (float(v) for v in waist)))
            for name in self._driven_waist:
                message.motor_cmd[WAIST_MOTOR_IDS[name]].q = values[name]
        return self._write(message)

    def publish_servo_position(self, radians, *, weight):
        """Use the existing servo gains and position writer, without RNEA.

        The execution owner supplies takeover/handback weight. This is not
        trajectory interpolation; each accepted target is written unchanged.
        """
        import math
        if len(radians) != len(self._driven_arm_ids) or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in radians):
            raise ValueError('invalid_servo_position')
        if type(weight) not in (int, float) or not math.isfinite(weight) or not 0 <= weight <= 1:
            raise ValueError('invalid_servo_weight')
        with self._lock:
            if self._arm_pub is None or self._message is None:
                raise RuntimeError('arm_stream_not_open')
            self._weight = float(weight)
            for motor_id in self._driven_arm_ids:
                cmd = self._message.motor_cmd[motor_id]
                cmd.kp, cmd.kd = (40.0, 1.5) if motor_id in WRIST_MOTOR_IDS else (80.0, 3.0)
        return self.publish_arms(radians)

    def publish_stream(self, radians, tau_ff, *, weight):
        """Strict arm-only stream write with a real SDK write receipt.

        Legacy servo entry points retain their existing behavior. The stream
        passes final, validated q and gravity compensation, never an IK target.
        tau_ff=None is reserved for hold/handback, retaining the last gains/tau.
        """
        import math
        n = len(self._driven_arm_ids)
        for name, values in (('q', radians), ('tau', tau_ff)):
            if values is None and name == 'tau':
                continue
            if (not isinstance(values, (list, tuple)) or len(values) != n
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
                raise ValueError('invalid_stream_'+name)
        if type(weight) not in (int, float) or not math.isfinite(weight) or not 0 <= weight <= 1:
            raise ValueError('invalid_stream_weight')
        with self._lock:
            if self._arm_pub is None or self._message is None:
                raise RuntimeError('arm_stream_not_open')
            for i, motor_id in enumerate(self._driven_arm_ids):
                cmd = self._message.motor_cmd[motor_id]
                # G1_23 visible-follow hardware.py (2026-09-19): arm weak
                # motors use 80/3, wrist roll 40/1.5. Keep the legacy servo
                # defaults above unchanged; the new stream retains its baseline.
                cmd.mode = 1
                cmd.kp, cmd.kd = (40.0, 1.5) if motor_id in WRIST_MOTOR_IDS else (80.0, 3.0)
                cmd.q, cmd.dq = float(radians[i]), 0.0
                if tau_ff is not None:
                    cmd.tau = float(tau_ff[i])
            self._weight = float(weight)
            if self._write(self._message) is not True:
                raise RuntimeError('arm_sdk_write_failed')
            self._last_target = list(radians)
            return True

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

        渐变期间发的是**最后一个被接受的目标**。接管时那就是 `open()` 从
        `rt/lowstate` 读回来的实测角 —— 也就是手臂此刻所在的位置，所以渐入期间它
        停在原地。看门狗清掉目标之后（`forget_target`）则只发权重，关节字段保持
        消息里上一次的值，同样是停在原地。
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
                for motor_id, value in zip(self._driven_arm_ids, target):
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
            return publisher.Write(message)
        return False
