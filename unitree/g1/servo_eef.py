#!/usr/bin/env python3
"""G1 的末端位姿卡片：收 19 维标准动作，解 IK，落到 `rt/arm_sdk`。

`servo` 收的是关节角（UnifoLM-WMA-0 那种），这张收的是**绝对末端位姿**。两者是
两个动作空间，因此是两张卡片 —— 协商靠比 `control_mode` 把接错的那一种当场拒掉，
而不是指望「16 和 19 差得够明显，总会有人发现」。

## 动作空间：19 维

      0..6    左末端  [x, y, z, qx, qy, qz, qw]   米 + 单位四元数（xyzw）
      7       左夹爪  Dex1 的关节值，**不是 0..1 的归一化闭合度**（见下）
      8..14   右末端  同上
      15      右夹爪
     16..18   腰      [roll, pitch, yaw] 弧度

这是云端规范化之后的标准布局，不是任何一个模型的原生布局。UnifoLM-VLA 的 G1
checkpoint 原生吐 23 维 `EE_R6_G1`，由 `phanthymotus-cloud` 的
`runtimes/common/normalize.py` 抹平 —— 那边的模块文档里记着真实的 23 维排法，
以及它和上游枚举注释不一致的地方（两个夹爪都在尾部，而且**右在前**）。

## 夹爪的单位是 0..4.5，不是 0..1 —— 而 `servo` 那张卡说的是 0..1

这一条是拿 checkpoint 自带的 `dataset_statistics.json` 核出来的，不是推的：
`g1_stack_block` 的 23 维统计量里，第 18、19 维的范围是 **[0.019, 4.5]**，而其余
21 维全部落在 ±1 以内。那两维就是夹爪（布局见云端 `normalize.py`），4.5 是 Dex1
的行程，不是归一化闭合度。

**所以限位不能写 0..1。** 写了的话 sink 会把每一条指令都拒掉 —— 响亮，但整条管线
跑不起来，而报错说的是「夹爪超限」，指向模型而不是这份声明。

**一个没有解决的矛盾，留在这里而不是挑一个答案：** 同一对物理夹爪，`servo.py`
（接 UnifoLM-WMA-0）声明的是 0..1 归一化，这张卡（接 UnifoLM-VLA-0）按统计量是
0..4.5。两个 checkpoint 对同一个执行器的单位约定不同，而两张卡都把收到的数直接
写进 `Dex1MotorCmd.q`。**只有一个能是对的**，而哪个对要真机上看一次夹爪的实际
开合才知道。云端的规范化层对夹爪是原样透传的，也应当如此：把它归一化到 0..1 需要
知道这只夹爪的完整行程，而那是机器人的知识，不是模型的。

## 腰：声明满 3 维，不可动的轴用限位卡死

腰在 URDF 里就在 pelvis 和两条手臂之间（`pelvis → waist_yaw → waist_roll →
waist_pitch → torso → shoulder …`），所以解一个 pelvis 系下的末端位姿**绕不过它**。
而模型确实会输出腰的三个值。

问题是这三个轴能不能经 `rt/arm_sdk` 写，只有 yaw 有依据（官方 arm7 例程的可写
列表里有 motor 12），roll 和 pitch 没有文档，而且不少 G1 的这两个轴本来就是锁死的。

三条路，选了第三条：

1. 声明 16 维、不接腰 —— 诚实，但 19 维的模型协商失败，整条管线跑不通。
2. 三个轴都写，启动时探测 —— 探测本身就要真的动机器人，而这张卡一次真机都没跑过。
3. **声明满 19 维，yaw 真写，roll/pitch 的限位收到 ±0.02 rad。** 模型真要弯腰
   就被 sink 响亮地 REJECT，而不是发下去看看会怎样；不弯腰的任务照常跑。

IK 把**指令**腰角当作链上的固定值（不是实测值）：指令是这一拍要去的地方，而末端
目标和它是同一拍算出来的。拿实测腰角去解，等于把手臂的目标算在一个已经过时的
躯干姿态上。

**实测（2026-09-21，真 GPU 上的 `unifolm-vla-g1`）：这一条把它整个挡在门外。**
那个 checkpoint 输出的腰 roll 落在 0.087–0.148 rad、pitch 落在 0.135–0.154 rad，
换了四组完全不同的输入（随机噪点／全黑／全白／空指令）再问，**没有一次接近零**
—— 躯干前倾是它训练分布的一部分，不是探针输入不干净。于是 25 步全数被
`waist_roll at 0.1380 outside [-0.02, 0.02]` 拒掉，一步都到不了 IK。

守卫是对的（腰确实动不了），但结论是：**要在 1 自由度腰的 G1 上跑这个 checkpoint，
需要云端把腰声明成可选**，而不是放宽这里的限位 —— 放宽等于让 IK 按一个机器人永远
到不了的躯干姿态去解手臂，那是个静默的系统性偏移。

### 真机验证（2026-09-21，办公室 G1 `10.100.129.168`，23dof，悬挂，FSM 500）

advisory 这条路在真机上跑过了，三件事分别有独立证据：

**1. 守卫拦在启动处。** `waist_advisory: false` → 卡片拒绝启动并点名开关，零动作。
注意这和旧代码的表现不同：旧代码会起来、看着在跑，然后每条指令在硬限位处被拒。
旧症状现在复现不了，因为它被前移了。

**2. 腰真的没被写。** 嗅探 `rt/arm_sdk` 共 274 帧（含一次 18° 的双臂运动），腰
roll/pitch/yaw 的 `motor_cmd.q` **全程 0.0、`kp` 全程 0.0**，而指令里一直带着
`roll=0.1380`。kp 为 0 是第二层保险：就算哪天 q 被写进去也不会施力。

**3. IK 挂的是实测腰角，不是指令腰角。** 这条要专门设计才能测出来，而它正是
advisory 和「干脆放宽限位」的分界。做法：末端目标取**实测关节角的正解**（即「回到
此刻所在的位置」），而指令腰 yaw 填 **0.9 rad**，实测只有 0.0012。

    IK 若挂指令腰角 → 以为躯干转了 51°，手臂必须改几十度才能够到同一个 pelvis
                      系位姿 → 手会动
    IK 若挂实测腰角 → 解回原处 → 手不动

实测臂关节指令首末最大变化 **0.0086 / 0.0155 rad（≈0.5°）**，而那 0.5° 是从实测
种子出发的数值收敛加编码器抖动。**这条用例里「不动」是阳性结果。**

另外量到的：`verdict: applied`、残差 0.32/0.26 mm；双臂前伸 8 cm 时残差 2.8/2.5 mm
（arm5 的 x 向包络约 11 cm，所以接近边缘），跟踪误差 0.011 rad。流停后 1332 ms
看门狗 → `aborted` → 保持。

**没验到的**：29dof 上腰被真正驱动的那一支（手上没有 arm7 机器），以及端到端接
`unifolm-vla-g1`（那台机器还差两道独立的门，见下一节）。

## 末端 frame 是「装了什么手」的事，不是「几个关节」的事

`VARIANTS` 按 arm5/arm7 选 URDF 和 tip link，而这两件事只有后者是型号决定的：

      g1_model.urdf   (29dof)  → hand_palm_link          Dex3 三指手，腕外 12.5 cm
      g1_23dof.urdf   (23dof)  → wrist_roll_rubber_hand  光腕，**没有任何末端执行器**

**两份 URDF 都没有建模 Dex1。** 所以一台装了 Dex1 的 arm5 G1，真实末端比这里算的
远出一截，而 IK 会把这一截当成够不着 —— 正是 `kinematics.py` 文档里警告的那种
「恒定偏移，看起来像标定问题」。

量过它值多少：拿那 25 步真轨迹，沿腕滚转轴加虚拟延长扫长度，**0 mm 时最坏残差
48.8 mm，60 mm 时全部 25 步可解**。也就是说「够不着」和「全可解」之间只隔着一只手
的长度。办公室那台（10.100.129.168）确认是光腕 —— 内部走线的 motor 31/33 都是
mode=0，外接的 `rt/dex1/*/state` 也没有消息，`arm_sdk.detect_dex1` 现在会在启动时
问这件事。

对 `unifolm-vla-g1` 的结论：**它的目标机是 arm7。** arm5 上用 40 万点撒样加投影
梯度下降（不依赖求解器）求工作空间到目标的最近距离，是 7–47 mm，目标在包络外面；
同一段轨迹 arm7 解到 0.01–0.25 mm、2–4 次迭代。这不是求解器或任务优先级的问题：
把姿态完全放弃、只解位置，也还差 39.6 mm。

### 那台 G1 跑不了这个 checkpoint，有**三个互相独立**的原因

腰只是其中一个，而且是唯一修掉的那个。按开火顺序：

1. **维度。** 那台机器没有夹爪（内部走线的 motor 31/33 与外接的 `rt/dex1/*/state`
   都空），于是 `grippers=false`、卡片是 **17 维**，而模型是 19 维。协商在维度这一
   条就拒了，**advisory/optional 那道门根本轮不到开火**。这是对的行为。
2. **腰。** 本节修的。
3. **够不着。** 上面那段，要 arm7。

把真 capabilities 和真 descriptor 送进真协商函数跑过：17 维时报的是「分成 5 段 /
3 段」加「19 维 / 17 维」；把 `grippers` 强行设成 true 凑到 19 维时，报的才是
「下游把 'waist' 声明成 advisory，而模型没有把 'waist' 声明成 optional」。

## 解不出来必须是响亮的

`ControlSink` 的每一道检查都在**关节空间之外**：它检查的是收到的 19 个数，而 IK
是在那之后。所以「这个位姿解不出来」这件事，只有这张卡自己能发现。残差超阈值 →
不发布 + 走 `on_watchdog` 的处置（保持），和看门狗同一个出口。

不这么做的话：CLIK 不收敛时返回的是**某个**关节角 —— 每个分量都在限位内、能通过
下游每一道检查、离目标半米远。那是这条链路上最安静的一种错。
"""

from __future__ import annotations

import json
import math
import threading
import time

from common.control import ControlSink, Verdict, parse_descriptor
from common.control.kinematics import ArmChain, KinematicsError

from arm_sdk import HANDBACK_S, LOW_STATE_TOPIC, TAKEOVER_S, ArmSdkChannel

# ── 两种 G1，启动时按实测决定用哪一套 ──────────────────────────────────────
#
# 办公室那台是 **arm5 + 1 自由度腰**（23dof）：`rt/lowstate` 里 wrist_pitch /
# wrist_yaw 与腰 roll/pitch 的 `mode` 全是 0，也就是这些关节不存在或未使能。而
# 仓库原本只有 29dof 的 URDF（arm7 + 3 自由度腰）。
#
# **这个差别不报错。** 在 7 自由度模型里解一个 5 自由度手臂够不着的位姿，求解器会
# 把解分配给不存在的 wrist_pitch（实测 +0.0845 rad），然后报残差 0.26 mm —— 在
# **模型**里它确实解出来了。残差校验的是求解器，不是机器人。
#
# 所以型号由 `motor_state[].mode` 决定，不由配置决定：配置会写错，而实测不会。
URDF_29DOF = "/work/resource/g1_model.urdf"
URDF_23DOF = "/work/resource/g1_23dof.urdf"

# 末端取手掌根，不是腕。腕之后还有一段固定变换，拿腕当末端会让整条轨迹系统性地
# 差掉那一段 —— 一个恒定偏移，看起来像标定问题。
_ARM7 = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
         "wrist_roll", "wrist_pitch", "wrist_yaw")
_ARM5 = _ARM7[:5]                       # 没有 wrist_pitch / wrist_yaw

# 末端取手掌根而不是腕：腕之后还有一段固定变换，拿腕当末端会让整条轨迹系统性地差
# 掉那一段 —— 一个恒定偏移，看起来像标定问题。两份 URDF 里那个 link 叫法不同。
VARIANTS = {
    "29dof": {
        "urdf": URDF_29DOF,
        "arm": _ARM7,
        "tips": {"left": "left_hand_palm_link", "right": "right_hand_palm_link"},
        "motors": {"left": list(range(15, 22)), "right": list(range(22, 29))},
    },
    "23dof": {
        "urdf": URDF_23DOF,
        "arm": _ARM5,
        "tips": {"left": "left_wrist_roll_rubber_hand",
                 "right": "right_wrist_roll_rubber_hand"},
        "motors": {"left": list(range(15, 20)), "right": list(range(22, 27))},
    },
}
# 29dof 的腰是三个关节，23dof 只有 yaw。描述符两种情况下**都声明 3 维**（模型输出
# 就是 3 维），差别只在 IK 链里挂几个 —— 不可动的那两个本来就已经用限位卡死。
WAIST_JOINTS_BY_VARIANT = {
    "29dof": ("waist_roll_joint", "waist_pitch_joint", "waist_yaw_joint"),
    "23dof": ("waist_yaw_joint",),
}

# 向后兼容的别名：外部（测试、其它模块）按老名字引用。
TIP_LINKS = VARIANTS["29dof"]["tips"]
ARM_JOINTS = {side: [f"{side}_{n}_joint" for n in _ARM7]
              for side in ("left", "right")}
WAIST_JOINTS = WAIST_JOINTS_BY_VARIANT["29dof"]


def detect_variant(motor_state) -> str:
    """看 `rt/lowstate` 的电机 `mode` 判型号。`mode == 0` = 不存在或未使能。

    问机器人，不问配置：配置会写错，而实测不会。两侧腕 pitch/yaw 缺席就是 arm5。
    """
    absent = [i for i in (20, 21, 27, 28)
              if int(getattr(motor_state[i], "mode", 1)) == 0]
    return "23dof" if len(absent) == 4 else "29dof"

# G1_29 的关节下标，从 rt/lowstate 里读实测角用。
ARM_MOTOR_IDS = VARIANTS["29dof"]["motors"]
WAIST_MOTOR_IDS = (13, 14, 12)          # roll, pitch, yaw —— 和标准向量同序

# 布局。**不是常量** —— 不带夹爪时整条向量往前挪两格，而一组写死的下标在那种
# 配置下会把腰读成右末端四元数的尾巴，且不报错。`_layout` 从同一个开关算出来，
# 和 `build_descriptor` 用的是同一条规则。
DOF = 19


def _layout(grippers: bool) -> dict:
    """`{left_pose, left_gripper, right_pose, right_gripper, waist}` 的下标。"""
    offset = 0
    out: dict = {}
    out["left_pose"] = slice(offset, offset + 7)
    offset += 7
    out["left_gripper"] = offset if grippers else None
    offset += 1 if grippers else 0
    out["right_pose"] = slice(offset, offset + 7)
    offset += 7
    out["right_gripper"] = offset if grippers else None
    offset += 1 if grippers else 0
    out["waist"] = slice(offset, offset + 3)
    return out

# 腰的限位。yaw 取 URDF 的真实行程；roll/pitch 卡死在零附近 —— 见模块文档。
WAIST_YAW_LIMIT = (-2.618, 2.618)
WAIST_LOCKED_LIMIT = (-0.02, 0.02)

# 工作空间。不是精确的可达域（那是 IK 的事），是一个「明显跑飞了」的围栏：一个
# 落在两米开外的目标，与其让 IK 迭代 50 次再报残差，不如当场拒掉。
WORKSPACE = ((-0.6, 0.9), (-0.9, 0.9), (-0.6, 1.0))

# 步长与速度。位置是米，姿态那一格是**弧度**（整段姿态共用它，另外三格不读 ——
# 见 common/control/sink.py 的 `_clamp_step`）。
MAX_LINEAR_VELOCITY = 0.5        # m/s
MAX_ANGULAR_VELOCITY = 2.0       # rad/s
# Dex1 的行程，取自 checkpoint 统计量里第 18/19 维的 max（见模块文档）。
GRIPPER_RANGE = (0.0, 4.5)
GRIPPER_MAX_VELOCITY = 6.0       # 单位同上，不是 0..1/s
WAIST_MAX_VELOCITY = 1.0

# 残差阈值。比收敛判据松一档：收敛判据是求解器停下来的条件，这是「停下来的地方
# 还算不算数」。1 cm / 0.05 rad 之外的解，执行它不如保持。
MAX_POSITION_RESIDUAL = 0.01     # m
MAX_ROTATION_RESIDUAL = 0.05     # rad

DEFAULT_EXPECTED_HZ = 30.0
MAX_HZ = 50.0
WATCHDOG_MS = 200
MAX_OBS_AGE_MS = 300
STATE_MAX_HZ = 30.0


def build_descriptor(expected_hz: float = DEFAULT_EXPECTED_HZ,
                     grippers: bool = True,
                     waist_advisory: bool = False) -> dict:
    """19 维（不带夹爪时 17 维）。

    `grippers=False` 真的是一张 17 维的卡片，不是一张 19 维却悄悄忽略两个值的卡
    片 —— 和 `servo` 同样的理由：留着宽度会让一个 19 维模型协商通过，然后两个自由
    度静默不动。

    `waist_advisory=True` 则相反：**宽度留着，并且把腰那一段标成 advisory**，也就
    是「收下但不执行」。这不是同一件事的两种写法，区别在于**谁给的许可**——
    advisory 在协商时会被拿去和生产者的 `optional` 对质，生产者没声明就拒绝启动。
    详见 `common/control/descriptor.py` 的 `Group.advisory`。

    为什么腰不走 `grippers` 那条「少声明几维」的路：模型输出的 19 维里腰是**中
    间**的三维（16..18），少声明它就少了三维宽度，19 维的模型直接协商失败——而
    那正是我们要解决的问题本身，不是解法。
    """
    joint_names = (
        ["eef_l_x", "eef_l_y", "eef_l_z", "eef_l_qx", "eef_l_qy", "eef_l_qz", "eef_l_qw"]
        + (["gripper_l"] if grippers else [])
        + ["eef_r_x", "eef_r_y", "eef_r_z", "eef_r_qx", "eef_r_qy", "eef_r_qz", "eef_r_qw"]
        + (["gripper_r"] if grippers else [])
        + ["waist_roll", "waist_pitch", "waist_yaw"]
    )

    period = 1.0 / expected_hz
    lower: list[float] = []
    upper: list[float] = []
    max_velocity: list[float] = []
    groups: list[dict] = []
    offset = 0

    for side in ("l", "r"):
        for axis in range(3):
            lower.append(WORKSPACE[axis][0])
            upper.append(WORKSPACE[axis][1])
            max_velocity.append(MAX_LINEAR_VELOCITY)
        # 四元数分量的上下界没有物理意义（任何单位四元数四个分量都在 [-1, 1]，而
        # 同一个朝向可以把它们全取反），sink 也不检查它们。写 ±1 是为了让描述符
        # 自洽，真正生效的是 `qx` 那一格的角速度上限。
        lower += [-1.0] * 4
        upper += [1.0] * 4
        max_velocity += [MAX_ANGULAR_VELOCITY] * 4
        groups.append({"name": f"eef_{side}", "offset": offset, "count": 7,
                       "unit": "m+quat", "resource": f"arm_{side}",
                       "mode": "eef_pose"})
        offset += 7
        if grippers:
            lower.append(GRIPPER_RANGE[0])
            upper.append(GRIPPER_RANGE[1])
            max_velocity.append(GRIPPER_MAX_VELOCITY)
            groups.append({"name": f"gripper_{side}", "offset": offset, "count": 1,
                           # `dex1` 而不是 `normalized`：这个数不是 0..1，写
                           # normalized 会让读描述符的人（和下一张照抄的卡片）
                           # 以为它是。见模块文档。
                           "unit": "dex1", "resource": f"gripper_{side}",
                           "mode": "joint_position"})
            offset += 1

    # 腰。两种声明，取决于这台机器人驱不驱得动它整段：
    #
    # - **驱得动**（29dof，三个轴都在）：roll/pitch 卡死在 ±0.02、yaw 真实行程。
    #   那对 roll/pitch 卡死的限位此时仍然有意义——三个轴都会被真的写下去，而
    #   roll/pitch 能不能经 arm_sdk 写没有依据，所以先按不能算。
    # - **驱不动**（23dof，roll/pitch 电机 mode=0）：整段标 advisory，限位不再被
    #   检查（sink 跳过 advisory 段），所以这里填的是物理行程而不是 ±0.02——
    #   留着 ±0.02 会让读描述符的人以为那个约束还在生效。
    #
    # **advisory 是整段的，所以 23dof 上连 yaw 也不再驱动。** 这是有意的取舍：
    # 模型给的是一个协调的躯干位姿，只执行其中三分之一会得到一个模型从未要求过的
    # 姿势；而末端目标是绝对的，IK 会自己补偿。相对现状也没有损失——现状是整条
    # 指令在 roll 的硬限位处被拒，yaw 同样一动不动。
    waist_limits = ((WAIST_YAW_LIMIT,) * 3 if waist_advisory
                    else (WAIST_LOCKED_LIMIT, WAIST_LOCKED_LIMIT, WAIST_YAW_LIMIT))
    for low, high in waist_limits:
        lower.append(low)
        upper.append(high)
        max_velocity.append(WAIST_MAX_VELOCITY)
    waist_group = {"name": "waist", "offset": offset, "count": 3,
                   "unit": "rad", "resource": "waist", "mode": "joint_position"}
    if waist_advisory:
        waist_group["advisory"] = True
    groups.append(waist_group)

    return {
        "control_interface": "motus.control/1",
        # 顶层 mode 取 `eef_pose`：它是这条向量的主体，也是协商时第一眼比的东西。
        # 混合的部分由 `groups` 逐段说清楚。
        "mode": "eef_pose",
        "dof": len(joint_names),
        "joint_names": joint_names,
        "units": ({"length": "m", "rotation": "quat_xyzw", "angle": "rad",
                   "dex1": f"{GRIPPER_RANGE[0]:g}-{GRIPPER_RANGE[1]:g}",
                   "time": "s"} if grippers else
                  {"length": "m", "rotation": "quat_xyzw", "angle": "rad",
                   "time": "s"}),
        "limits": {
            "lower": lower,
            "upper": upper,
            "max_velocity": max_velocity,
            "max_delta_per_step": [speed * period for speed in max_velocity],
        },
        "groups": groups,
        # 位姿是在这个系下表达的。改它等于改每一条指令的含义，所以它和
        # `joint_names` 一样是描述符里不能默认的东西。
        "frame": "pelvis",
        "end_effector": {"left": TIP_LINKS["left"], "right": TIP_LINKS["right"]},
        "rate": {
            "max_hz": MAX_HZ,
            "expected_hz": expected_hz,
            "watchdog_ms": WATCHDOG_MS,
            "max_obs_age_ms": MAX_OBS_AGE_MS,
        },
        "force_torque": None,
    }


class G1ServoEefPlugin:
    """一张卡片、一路输入、一个 sink、两条 IK 链、一条 arm_sdk 通道。"""

    PREFIX = "servo_eef"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 arm_client=None):
        self._ns = namespace
        self._executor = executor
        config = plugin_config or {}

        self._expected_hz = float(config.get("expected_hz", DEFAULT_EXPECTED_HZ))
        if not math.isfinite(self._expected_hz) or not 0 < self._expected_hz <= MAX_HZ:
            raise ValueError(f"servo_eef.expected_hz must be in (0, {MAX_HZ}]")
        self._send_crc = bool(config.get("send_crc", True))
        self._grippers_enabled = bool(config.get("grippers", True))
        # 只在测试里用：真机上型号由 `detect_variant` 从实测电机 mode 得出，
        # 不由配置决定 —— 配置会写错，而实测不会。
        self._urdf_override = str(config.get("urdf", "")) or None
        self._variant = ""
        self._max_position_residual = float(
            config.get("max_position_residual", MAX_POSITION_RESIDUAL))
        self._max_rotation_residual = float(
            config.get("max_rotation_residual", MAX_ROTATION_RESIDUAL))

        # **必须在这里定，不能等 `_start` 探完型号。** 协商读的是 agent-core 在
        # 卡片绑定时拿到的那份 descriptor，而那份来自静态的工具 schema。等到
        # `_start` 才把腰标成 advisory，意味着一个没声明 `optional` 的模型可以先
        # 协商通过、再在运行时被悄悄丢掉三维——正好是 advisory 要防的那件事。
        #
        # 所以它是配置项，而**机器人负责证伪**：`_start` 探到型号之后对质，不一致
        # 就拒绝启动。和 `grippers` 同一条规矩——声明在配置里，真相在机器人那儿。
        self._waist_advisory = bool(config.get("waist_advisory", False))
        self._descriptor_raw = build_descriptor(self._expected_hz,
                                                self._grippers_enabled,
                                                self._waist_advisory)
        self._descriptor = parse_descriptor(self._descriptor_raw)
        self._layout = _layout(self._grippers_enabled)

        self._lock = threading.RLock()
        self._sink = None
        self._sub_node = None
        self._state_pub = None
        # 通道在 `_start` 里定完型号才建 —— 它要知道这台机器人真有哪些臂关节。
        self._channel = ArmSdkChannel(send_crc=self._send_crc,
                                      grippers=self._grippers_enabled,
                                      waist=True)
        self._chains: dict = {}
        self._arm_motor_ids = VARIANTS["29dof"]["motors"]
        self._input_topic = ""
        self._running = False
        self._paused = False
        # 上一拍解出来的关节角，下一拍的种子。**这是边端解 IK 的全部收益**：
        # 7 自由度手臂对同一个手部位姿有无穷多组解，靠「离种子最近」挑一支，而
        # 边端的种子永远是 33 ms 前的，不是云端那个 833 ms 前的。
        self._seed = {"left": None, "right": None}
        self._measured: dict = {}
        self._measured_ms = 0
        self._low_state_sub = None
        self._last_outcome = None
        self._rejects: list = []
        self._last_residual: dict = {}

    # ── tool ─────────────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "servo_eef",
            "type": "actuator",
            "description": (
                "G1 双臂的末端位姿控制：订阅一路 motus.control/1 指令流"
                f"（{self._descriptor.dof} 维绝对末端位姿，≤{self._expected_hz:g} Hz），"
                "在本机解 IK 后驱动执行。关节空间的模型用 servo 那张卡片。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "stop", "pause", "resume", "info"]},
                    "input_topic": {"type": "string",
                                    "description": "control/eef 指令流话题"},
                },
                "required": ["action"],
                # `start`/`stop` 不在这里 —— agent-core 把每一项展开成一个 LLM 可调
                # 函数，而 stop 属于项目生命周期：模型调它会把卡片从运行中的项目里
                # 摘掉而项目并不知情，且它自己放不回去。和 servo 同一条规矩。
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
            "topic_in": [{"format": "control/eef",
                          "desc": f"motus.control/1，{self._descriptor.dof} 维"
                                  "（末端 xyz + 四元数 xyzw、夹爪、腰）"}],
            "topic_out": [{"format": "data/json",
                           "desc": "关节角与**当前末端位姿**（标准布局）"}],
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
        """Bundle lifecycle. 故意什么都不做 —— 一张会让手臂动起来的卡片，不能因为
        容器重启就自己开始动。它在 agent-core 启动项目时才订阅，而那是一个人的动作。
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
                    "message": "缺少 input_topic —— 请在画布上把一路 control/eef "
                               "源连到这张卡片"}
        if self._executor is None:
            return {"state": "error", "message": "没有 ROS 上下文，无法订阅"}

        try:
            # **先问机器人是什么型号，再建链。** 顺序是有意的：拿 29dof 的链去解
            # 一台 arm5 的手臂不会报错，只会把解分配给不存在的关节。
            variant = self._detect_variant_from_robot()
            chains = self._build_chains(variant)
        except KinematicsError as exc:
            # 拒绝启动，不降级。一张声明了 eef_pose 的卡片解不了 IK，就是收下位姿
            # 然后什么都不做 —— 而画布上它看起来是在跑的。
            return {"state": "error", "message": f"IK 建链失败: {exc}"}

        # **配置说的腰，和机器人真有的腰，必须一致。** 声明在 `__init__` 定死了
        # （协商要读它），这里是唯一能证伪它的地方。
        drives_whole_waist = variant == "29dof"
        if not drives_whole_waist and not self._waist_advisory:
            return {"state": "error", "message": (
                f"这台机器人是 {variant}，腰 roll/pitch 的电机 mode=0，驱动不了 —— "
                "而 descriptor 把腰那一段声明成会执行（waist_advisory=false），"
                "于是任何一条带非零 roll/pitch 的指令都会在硬限位 ±0.02 处被**整条**"
                "拒掉。实测 unifolm-vla-g1 的腰 roll 恒在 0.087–0.148 rad，25 步全拒。"
                "在 config.yaml 里设 servo_eef.waist_advisory: true —— 那会把腰声明成"
                "「收下但不执行」，而模型必须在 capabilities 里声明 waist 是 optional，"
                "否则协商拒绝。这两道一起才让「丢掉三维」是双方都同意过的")}
        if drives_whole_waist and self._waist_advisory:
            # 反过来不拦：你可能就是想让这台 29dof 别动腰。但要说出来。
            print("[servo_eef] 注意：这台 29dof 的腰三个轴都在，"
                  "但 waist_advisory=true —— 腰不会被驱动", flush=True)


        sink = ControlSink(
            self._descriptor,
            self._apply,
            on_watchdog=self._hold,
            on_abort=self._hold,
        )
        with self._lock:
            if self._running:
                return {"state": "error",
                        "message": f"已经在运行（{self._input_topic}）"}
            self._chains = chains
            self._variant = variant
            self._arm_motor_ids = VARIANTS[variant]["motors"]
            self._channel = ArmSdkChannel(
                send_crc=self._send_crc, grippers=self._grippers_enabled,
                # advisory 的腰是真的不碰：通道连腰的发布槽位都不建，
                # 所以「不执行」不是靠调用方记得不传值来保证的。
                waist=not self._waist_advisory,
                driven_arm_ids=(VARIANTS[variant]["motors"]["left"]
                                + VARIANTS[variant]["motors"]["right"]),
                driven_waist_names=(() if self._waist_advisory else
                                    ("roll", "pitch", "yaw")))
            self._sink = sink
            self._input_topic = topic
            self._running = True
            self._paused = False

        try:
            self._open(topic)
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._running = False
                self._sink = None
            return {"state": "error", "message": f"启动失败: {exc}"}

        # 接管：权重 0→1。**在这之前一条关节指令都没发过**，所以渐入期间手臂停在
        # 内置控制器给它的位置上，不会先跳到一个目标再开始受控。
        self._channel.ramp(0.0, 1.0, TAKEOVER_S)
        print(f"[servo_eef] streaming from {topic}", flush=True)
        print(f"[servo_eef] variant={variant} chains={len(chains)}", flush=True)
        return {"state": "running", "input": topic, "variant": variant,
                "waist": "advisory" if self._waist_advisory else "driven",
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

        # 交还：权重 1→0，两秒。**同步等它跑完**再往下拆东西 —— 中途把发布器撤掉，
        # 权重就停在半路，而那是一个既不受我们控制也没完全交还的状态。
        try:
            self._channel.ramp(self._channel.weight, 0.0, HANDBACK_S)
        except Exception as exc:  # noqa: BLE001
            print(f"[servo_eef] handback failed: {exc}", flush=True)

        if node is not None:
            try:
                self._executor.remove_node(node)
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        self._channel.close()
        # **必须关。** 不关的话每一次 start 都留下一条 500 Hz 的 rt/lowstate 订阅，
        # 而画布做 start/stop 是家常便饭 —— 这正是这个仓库里「孤儿订阅」那一类故障
        # 的形状：图看着健康，回调还在跑，直到某次 teardown 顺序出错把整层带走。
        subscriber, self._low_state_sub = self._low_state_sub, None
        if subscriber is not None:
            try:
                subscriber.Close()
            except Exception:  # noqa: BLE001 —— 清理失败不能挡住 stop 返回
                pass
        with self._lock:
            self._state_pub = None
            self._chains = {}
            self._seed = {"left": None, "right": None}
        del sink
        return {"state": "stopped"}

    def _info(self):
        with self._lock:
            return {
                "state": ("running" if self._running and not self._paused else
                          "paused" if self._running else "idle"),
                "input": self._input_topic,
                "weight": round(self._channel.weight, 3),
                "grippers": self._grippers_enabled,
                # 「收下但不驱动」必须在状态里看得见。一个被丢掉的自由度如果只有
                # descriptor 里一个布尔知道，那它在运维眼里就是「腰怎么不动」。
                "waist": "advisory" if self._waist_advisory else "driven",
            "variant": self._variant,
                "control_interface": self._descriptor_raw,
                "residual": dict(self._last_residual),
                "last": self._last_outcome,
                "rejects": list(self._rejects),
            }

    # ── wiring ───────────────────────────────────────────────────────────────

    def _build_chains(self, variant: str) -> dict:
        """按型号建两条链，各建各的模型。

        共享一份 `pinocchio.Data` 的两条手臂会在求解过程里互相踩 —— 表现是偶发的、
        和负载相关的错误解，而不是异常。两份模型多占几 MB，换的是这个。
        """
        spec = VARIANTS[variant]
        urdf = self._urdf_override or spec["urdf"]
        return {
            side: ArmChain(urdf, tip_link=spec["tips"][side],
                           joint_names=[f"{side}_{n}_joint" for n in spec["arm"]])
            for side in ("left", "right")
        }

    def _detect_variant_from_robot(self) -> str:
        """读一帧 `rt/lowstate`，按电机 `mode` 判型号。

        放在这里而不是 `ArmSdkChannel` 里：通道只管发，型号决定的是**运动学**，
        而那是这张卡的事。两边各读一帧，代价是几毫秒，换的是职责不混。
        """
        from arm_sdk import read_low_state_once

        try:
            frame = read_low_state_once()
        except Exception as exc:  # noqa: BLE001
            raise KinematicsError(
                f"读不到 rt/lowstate，无法判断机器人型号（{exc}）。"
                "不判型号就建链，等于赌它是 29dof —— 赌错了不报错，"
                "只会把 IK 的解分配给不存在的关节"
            ) from exc
        return detect_variant(frame.motor_state)

    def _open(self, topic: str):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                       # 排着的指令就是过期的指令
            durability=DurabilityPolicy.VOLATILE,
        )

        node = Node("g1_servo_eef", context=None)
        node.create_subscription(String, topic, self._on_message, qos)
        node.create_timer(WATCHDOG_MS / 2000.0, self._tick)
        state_pub = node.create_publisher(
            String, f"/{self._ns.strip('/') or 'g1'}/servo_eef/state", 1)
        self._executor.add_node(node)

        self._subscribe_low_state()
        self._channel.open()
        with self._lock:
            self._sub_node = node
            self._state_pub = state_pub

    def _subscribe_low_state(self):
        """订 `rt/lowstate` 拿实测关节角。

        用途有两个，而且只有这两个：正解出**当前**末端位姿报给上游（增量模型的
        基准位姿），以及第一拍的 IK 种子。之后的种子用上一拍的解，不用实测值 ——
        实测值滞后一拍，拿它当种子会让解在两支之间来回跳。
        """
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        subscriber = ChannelSubscriber(LOW_STATE_TOPIC, LowState_)
        subscriber.Init(self._on_low_state, 10)
        self._low_state_sub = subscriber

    def _on_low_state(self, message):
        try:
            motors = message.motor_state
            measured = {
                side: [float(motors[i].q) for i in self._arm_motor_ids[side]]
                for side in ("left", "right")
            }
            measured["waist"] = [float(motors[i].q) for i in WAIST_MOTOR_IDS]
        except Exception:  # noqa: BLE001 —— 一帧坏数据不该拖垮流
            return
        with self._lock:
            self._measured = measured
            self._measured_ms = int(time.time() * 1000)

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
        if sink is not None:
            outcome = sink.tick()
            if outcome is not None:
                self._record(outcome.verdict.value, outcome.reason)
        self._publish_state()

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
        """sink 放行之后才到这里。IK 在这一步，**不在 sink 里**。

        sink 检查的是收到的那 19 个数（单位四元数、工作空间、步长、限位），它对
        「这个位姿这台机器人够不够得着」一无所知 —— 那需要运动学，而 sink 是
        ROS-free、SDK-free、也 URDF-free 的。所以可达性在这里判，判不过就保持。
        """
        values = list(values)
        waist = values[self._layout["waist"]]
        solutions = {}
        residuals = {}

        for side in ("left", "right"):
            span = self._layout[f"{side}_pose"]
            chain = self._chains.get(side)
            if chain is None:
                return
            seed = self._seed_for(side)
            extra = self._waist_extra(waist)
            try:
                joints, position_residual, rotation_residual, _ = chain.solve(
                    values[span], seed, extra)
            except KinematicsError as exc:
                self._record(Verdict.REJECTED.value, f"{side} IK 失败: {exc}")
                self._hold()
                return
            if (position_residual > self._max_position_residual
                    or rotation_residual > self._max_rotation_residual):
                # **保持，不是执行一个近似解。** 迭代法不收敛时返回的是某个关节角
                # ——每个分量都在限位内、能通过下游每一道检查、离目标很远。
                self._record(
                    Verdict.REJECTED.value,
                    f"{side} 末端位姿解不出来：位置残差 {position_residual * 1000:.1f} mm、"
                    f"姿态残差 {rotation_residual:.3f} rad（上限 "
                    f"{self._max_position_residual * 1000:.0f} mm / "
                    f"{self._max_rotation_residual:.2f} rad）。"
                    "目标可能在工作空间外，或者被限位挡住")
                with self._lock:
                    self._last_residual = {
                        side: {"position_m": round(position_residual, 5),
                               "rotation_rad": round(rotation_residual, 5)}}
                self._hold()
                return
            solutions[side] = joints
            residuals[side] = {"position_m": round(position_residual, 5),
                               "rotation_rad": round(rotation_residual, 5)}

        with self._lock:
            self._seed = dict(solutions)
            self._last_residual = residuals

        self._channel.publish_arms(
            list(solutions["left"]) + list(solutions["right"]),
            waist=None if self._waist_advisory else waist)
        if self._grippers_enabled:
            for side in ("left", "right"):
                self._channel.publish_gripper(
                    side, values[self._layout[f"{side}_gripper"]])

    def _waist_extra(self, waist) -> dict:
        """把腰映射到这个型号真有的关节，给 IK 当链上的固定值。

        **关键是挂哪一组数**，而这取决于腰会不会被执行：

        - 会执行（`waist_advisory=False`）：挂**指令**腰角。指令是这一拍要去的
          地方，而末端目标和它是同一拍算出来的；挂实测值等于把手臂的目标算在一个
          已经过时的躯干姿态上。
        - 不执行（advisory）：挂**实测**腰角。躯干不会动，所以这一拍它就在那儿。
          挂指令值才是真正危险的那个选择——IK 会按一个机器人永远到不了的躯干姿态
          去解手臂，每一拍都差同样一点，看起来像标定问题。这正是「放宽限位」和
          「声明 advisory」的全部区别：前者两件事都做错了，后者只是不做。
        """
        names = WAIST_JOINTS_BY_VARIANT.get(self._variant or "29dof")
        if self._waist_advisory:
            measured = self._channel.measured_waist
            # `waist_yaw_joint` → `yaw`，即 `WAIST_MOTOR_IDS` 的键。
            return {name: float(measured.get(name[len("waist_"):-len("_joint")], 0.0))
                    for name in names}
        if len(names) == 1:                       # 23dof：只有 yaw，取第三个
            return {names[0]: float(waist[2])}
        return dict(zip(names, (float(v) for v in waist)))

    def _seed_for(self, side: str):
        """上一拍的解；还没有就用实测值；再没有就用零位。

        零位是最后的退路而不是默认值：它是一个胳膊垂下来的构型，从那儿解一个抬起
        来的目标，求解器很可能挑到另一支肘部朝向。第一拍用实测值就避开了它。
        """
        with self._lock:
            seed = self._seed.get(side)
            measured = self._measured.get(side)
        if seed is not None:
            return seed
        if measured is not None:
            return measured
        return [0.0] * len(self._arm_motor_ids[side])

    def _publish_state(self):
        """关节角 + **当前末端位姿**，一路话题、一份载荷。

        `eef` 这个字段是给增量模型（OpenVLA）当基准位姿用的：它输出的是末端 delta，
        「相对于哪儿」只有机器人知道。

        **和关节状态放在同一路话题里，不新开一路。** agent-core 的 VLA 卡片
        （`actucore/plugins/vla/plugin.py::_bind_inputs`）是按 **ROS 消息类型**分派
        角色的，两路 `String` 它分不开 —— 新开一路会变成一个按连线顺序赌运气的绑定。
        """
        publisher = self._state_pub
        if publisher is None:
            return
        now = time.time()
        with self._lock:
            measured = dict(self._measured)
            measured_ms = self._measured_ms
            chains = dict(self._chains)
        if not measured or not chains:
            return

        waist = measured.get("waist") or [0.0, 0.0, 0.0]
        extra = self._waist_extra(waist)
        try:
            poses = {side: chains[side].forward(measured[side], extra)
                     for side in ("left", "right")}
        except (KinematicsError, KeyError):
            return

        from std_msgs.msg import String

        payload = String()
        payload.data = json.dumps({
            "schema": "motus.control/1",
            "kind": "joint_state",
            "dof": self._descriptor.dof,
            "joint_names": self._descriptor_raw["joint_names"],
            # `values` 是关节角（本体感受），`eef` 是标准布局的当前末端位姿。
            # 两者是两回事：前者的宽度和含义由机器人决定，后者由协议定义。
            "values": list(measured["left"]) + list(measured["right"]) + list(waist),
            # 和这张卡片声明的布局同构：不带夹爪时那两格也不在。
            #
            # 夹爪那两格是 0.0 占位：这个驱动不订 Dex1 的状态话题，所以报不出
            # 实测开合度。它只服务于增量模型的位姿基准，而没有哪个增量模型的
            # delta 是叠在夹爪上的（夹爪那一维本来就是绝对的）。真要报实测值，
            # 得先订上 Dex1 的 state —— 那是另一件事。
            "eef": ([*poses["left"]]
                    + ([0.0] if self._grippers_enabled else [])
                    + [*poses["right"]]
                    + ([0.0] if self._grippers_enabled else [])
                    + [*waist]),
            # 取实测那一帧的时刻，不是现在。一个 VLA 用它算观测年龄，报现在等于
            # 宣称通道刚刚更新过，而实际可能已经很陈旧。
            "stamp_ms": measured_ms or int(now * 1000),
        }, ensure_ascii=False)
        publisher.publish(payload)

    def _hold(self):
        """看门狗、abort、解不出来 —— 三者同一个处置：停止发布，**不动权重**。

        理由见 `arm_sdk.ArmSdkChannel.forget_target`。种子**不清**：它是构型的记忆，
        而机器人在保持期间并没有移动，下一条有效指令从此刻的真实构型解起才对。
        """
        self._channel.forget_target()


PROVIDER = G1ServoEefPlugin
