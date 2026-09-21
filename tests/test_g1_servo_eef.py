"""G1 的末端位姿卡片：19 维标准动作 → IK → `rt/arm_sdk`。

分工：`ControlSink` 的检查链在 test_control_sink.py，权重渐变在
test_g1_arm_sdk.py，四元数数学在 test_control_rotation.py。这里只测这张卡片自己
的东西，也就是**关节空间之外的检查看不见的那些**：

  - 描述符的形状，以及腰那两个被限位卡死的轴
  - IK 解不出来时**保持**，而不是执行一个近似解
  - 种子来自上一拍的解，这是边端解 IK 的全部收益
  - 状态载荷里带 `eef`，而且和关节状态在**同一路话题**里

IK 的数值用例对着仓库自带的 `unitree/g1/resource/g1_model.urdf`，缺 pinocchio 就
skip —— 照 actucore 那份 SmolVLA provider 测试的做法，把版本敏感面收到最小。

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_g1_servo_eef.py -q
"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.control import Verdict, parse_descriptor  # noqa: E402

URDF = ROOT / "unitree" / "g1" / "resource" / "g1_model.urdf"

pinocchio = pytest.importorskip(
    "pinocchio",
    reason="解 IK 要 pinocchio（约 200 MB）。卡片的形状与拒绝逻辑在下面用不到它的"
           "那些用例里照样跑；数值部分在 Orin 6 与镜像里验证。",
)


def _load():
    bundle = ROOT / "unitree" / "g1"
    sys.path.insert(0, str(bundle))
    try:
        spec = importlib.util.spec_from_file_location(
            "g1_servo_eef", bundle / "servo_eef.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(bundle))


servo_eef = _load()

IDENTITY = [0.0, 0.0, 0.0, 1.0]


# ── 描述符 ───────────────────────────────────────────────────────────────────


def test_the_descriptor_is_accepted_by_the_shared_parser():
    parsed = parse_descriptor(servo_eef.build_descriptor())
    assert parsed.mode == "eef_pose"
    assert parsed.dof == 19 == len(parsed.joint_names)
    assert [g.name for g in parsed.groups] == [
        "eef_l", "gripper_l", "eef_r", "gripper_r", "waist"]
    assert parsed.frame == "pelvis"


def test_the_two_orientations_are_where_the_sink_will_look_for_them():
    """`eef_quat_offsets` 是 sink 的三处逐值检查用来避开四元数的索引。

    段划错一格，单位范数校验就会去查三个位置分量加一个夹爪 —— 那组数几乎必然不
    满足 |q| = 1，于是**每一条指令都被拒**，而报错说的是「不是单位四元数」。
    """
    parsed = parse_descriptor(servo_eef.build_descriptor())
    assert parsed.eef_quat_offsets == (3, 11)


def test_the_waist_roll_and_pitch_are_pinned_and_the_yaw_is_not():
    """三个轴里只有 yaw 在官方 arm7 例程的可写列表里。

    卡死的做法是把限位收到 ±0.02 —— 模型真要弯腰就被 sink 响亮地 REJECT，而不是
    发下去看看会怎样。声明成 16 维不接腰也是诚实的，但那样 19 维的模型协商失败，
    整条管线跑不通。
    """
    parsed = parse_descriptor(servo_eef.build_descriptor())
    roll, pitch, yaw = 16, 17, 18
    for locked in (roll, pitch):
        assert (parsed.lower[locked], parsed.upper[locked]) == (-0.02, 0.02)
    assert parsed.lower[yaw] < -2.0 and parsed.upper[yaw] > 2.0


def test_disabling_grippers_shifts_every_index_and_the_card_follows():
    """不带夹爪时整条向量往前挪两格。

    一组写死的下标在那种配置下会把腰读成右末端四元数的尾巴，且不报错 —— 这条钉的
    是布局由同一个开关算出来，不是两处各写一遍。
    """
    parsed = parse_descriptor(servo_eef.build_descriptor(grippers=False))
    assert parsed.dof == 17
    assert parsed.eef_quat_offsets == (3, 10)
    assert [g.name for g in parsed.groups] == ["eef_l", "eef_r", "waist"]
    card = servo_eef.G1ServoEefPlugin({"grippers": False}, "g1", executor=None)
    assert card._layout["waist"] == slice(14, 17)
    assert card._layout["left_gripper"] is None


def test_the_gripper_range_matches_the_checkpoint_and_is_not_0_to_1():
    """UnifoLM-VLA 的 G1 checkpoint 输出的夹爪值是 **0..4.5**，不是归一化闭合度。

    来源是 checkpoint 自带的 `dataset_statistics.json`：`g1_stack_block` 那 23 维
    统计量里第 18/19 维的范围是 [0.019, 4.5]，而其余 21 维全部落在 ±1 以内。
    kai 上的真模型跑一次也印证了（那两维 2.67..4.47，其余都很小）。

    声明成 0..1 的后果不是静默的，但整条管线跑不起来：sink 会把每一条指令都拒掉，
    而报错说的是「夹爪超限」，指向模型而不是这份声明。

    `servo.py`（接 WMA checkpoint）对**同一对物理夹爪**声明的是 0..1，两者只有一个
    能是对的 —— 那要真机上看一次开合才知道，见模块文档。这条测试钉的是这张卡与它
    自己那个 checkpoint 一致。
    """
    parsed = parse_descriptor(servo_eef.build_descriptor())
    for gripper in (7, 15):
        assert (parsed.lower[gripper], parsed.upper[gripper]) == (0.0, 4.5)
    # 单位名也不能写 normalized —— 下一个照抄这张卡的人会以为它是 0..1。
    assert "normalized" not in parsed.units
    assert parsed.units["dex1"] == "0-4.5"


def test_an_out_of_range_rate_is_refused():
    with pytest.raises(ValueError, match="expected_hz"):
        servo_eef.G1ServoEefPlugin({"expected_hz": 500}, "g1", executor=None)


def test_starting_without_an_input_topic_is_refused():
    card = servo_eef.G1ServoEefPlugin({}, "g1", executor=None)
    assert card._start({})["state"] == "error"


# ── IK ───────────────────────────────────────────────────────────────────────


@pytest.fixture
def chains():
    from common.control.kinematics import ArmChain

    return {
        side: ArmChain(str(URDF), tip_link=servo_eef.TIP_LINKS[side],
                       joint_names=servo_eef.ARM_JOINTS[side])
        for side in ("left", "right")
    }


def test_the_solver_round_trips_its_own_forward_kinematics(chains):
    """正解出来的位姿，逆解回去必须落在原处。差一点都说明两头的约定不一致
    （四元数 wxyz/xyzw、旋转矩阵行列、末端取腕还是手掌）。"""
    chain = chains["left"]
    seed = (0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0)
    pose = chain.forward(seed)
    _, position_residual, rotation_residual, _ = chain.solve(pose, seed)
    assert position_residual < 1e-6
    assert rotation_residual < 1e-6


def test_the_waist_is_part_of_the_chain(chains):
    """腰在 URDF 里就在 pelvis 和手臂之间，绕不过去。

    把它当零处理会让正解报出一个机器人从未处于过的位姿 —— 而那个位姿会被当成
    增量模型的基准，于是每一条 delta 都叠在错误的地方。
    """
    chain = chains["left"]
    seed = (0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0)
    straight = chain.forward(seed)
    twisted = chain.forward(seed, extra={"waist_yaw_joint": 0.5})
    assert max(abs(a - b) for a, b in zip(straight[:3], twisted[:3])) > 0.01


def test_an_unreachable_target_comes_back_with_a_large_residual(chains):
    """**这是整个文件里最重要的一条。**

    CLIK 不收敛时返回的是**某个**关节角 —— 每个分量都在限位内、能通过下游每一道
    检查、离目标很远。残差是唯一能把它和一个好解区分开的东西，所以它是返回值而
    不是异常，而调用方必须看。
    """
    chain = chains["left"]
    seed = (0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0)
    target = list(chain.forward(seed))
    target[0] += 2.0                       # 两米开外，绝无可能
    joints, position_residual, _, _ = chain.solve(target, seed)

    assert position_residual > 0.5
    # 而返回的那组关节角**看起来完全正常** —— 这正是它危险的地方。
    for value, low, high in zip(joints, chain.lower, chain.upper):
        assert low - 1e-9 <= value <= high + 1e-9


def test_a_zero_quaternion_target_is_refused_rather_than_solved(chains):
    from common.control.kinematics import KinematicsError

    with pytest.raises(KinematicsError, match="零四元数"):
        chains["left"].solve([0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0],
                             (0.0,) * 7)


def test_a_misspelt_joint_name_is_refused_at_build_time():
    """拼错一个关节名不会报错，只会少解一个自由度 —— 除非建链时就拒。"""
    from common.control.kinematics import ArmChain, KinematicsError

    names = list(servo_eef.ARM_JOINTS["left"])
    names[3] = "left_elbow"                # 少了 `_joint`
    with pytest.raises(KinematicsError, match="left_elbow"):
        ArmChain(str(URDF), tip_link=servo_eef.TIP_LINKS["left"], joint_names=names)


# ── 卡片把 IK 接上去之后 ────────────────────────────────────────────────────


class _FakeChannel:
    def __init__(self):
        self.arms = []
        self.grippers = []
        self.forgotten = 0
        self.weight = 0.0

    def publish_arms(self, radians, waist=None):
        self.arms.append((list(radians), None if waist is None else list(waist)))

    def publish_gripper(self, side, closure):
        self.grippers.append((side, closure))

    def forget_target(self):
        self.forgotten += 1


def _card(**config):
    # 默认路径是容器里的 /work/resource/…，在这儿不成立。
    config.setdefault("urdf", str(URDF))
    card = servo_eef.G1ServoEefPlugin(config, "g1", executor=None)
    card._channel = _FakeChannel()
    card._chains = card._build_chains()
    return card


def _pose_command(card, *, left=None, right=None, waist=(0.0, 0.0, 0.0)):
    """一条落在可达域内的 19 维指令，由正解自己算出来。

    手写坐标会让这些用例变成「我猜的那个点够不够得着」的测试。
    """
    seed = (0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0)
    extra = dict(zip(servo_eef.WAIST_JOINTS, waist))
    poses = {side: list(card._chains[side].forward(seed, extra))
             for side in ("left", "right")}
    if left is not None:
        poses["left"] = list(left)
    if right is not None:
        poses["right"] = list(right)
    return poses["left"] + [0.0] + poses["right"] + [0.0] + list(waist)


def test_a_reachable_command_reaches_the_arm_sdk_channel():
    card = _card()
    card._apply(_pose_command(card), None)

    assert len(card._channel.arms) == 1
    radians, waist = card._channel.arms[0]
    assert len(radians) == 14
    assert waist == [0.0, 0.0, 0.0]
    assert [side for side, _ in card._channel.grippers] == ["left", "right"]


def test_an_unreachable_command_holds_instead_of_moving():
    """sink 放行了它 —— 四元数是单位的、在工作空间的框里、步长没超。可达性只有
    这张卡自己知道，而「差不多的解」在 30 Hz 上会一直差下去。"""
    card = _card()
    command = _pose_command(card)
    command[0] = 0.85                       # 框内，但胳膊够不到

    card._apply(command, None)

    assert card._channel.arms == [], "解不出来就不该发布任何东西"
    assert card._channel.forgotten == 1, "应当走和看门狗同一个出口：保持"
    assert card._last_outcome["verdict"] == Verdict.REJECTED.value
    assert "解不出来" in card._last_outcome["reason"]
    assert "mm" in card._last_outcome["reason"], "报错要带上残差的数量级"


def test_the_seed_is_the_previous_solution_not_the_measured_state():
    """边端解 IK 的全部收益就在这里。

    7 自由度手臂对同一个手部位姿有无穷多组解（肘部可上可下），求解器靠「离种子
    最近」挑一支。种子过时就可能跳到另一支 —— 手的位姿是对的，整条手臂甩过去。
    """
    card = _card()
    assert card._seed_for("left") == [0.0] * 7          # 还没有任何解

    card._apply(_pose_command(card), None)
    after = card._seed_for("left")
    assert after is not None and len(after) == 7
    assert after != [0.0] * 7

    # 实测值到了也不该顶掉上一拍的解 —— 实测滞后一拍，拿它当种子会让解来回跳。
    card._measured = {"left": [0.9] * 7, "right": [0.9] * 7,
                      "waist": [0.0, 0.0, 0.0]}
    assert card._seed_for("left") == after


def test_the_first_seed_falls_back_to_the_measured_configuration():
    """零位是最后的退路，不是默认值：那是一个胳膊垂下来的构型，从那儿解一个抬起
    来的目标，很可能挑到另一支肘部朝向。"""
    card = _card()
    card._measured = {"left": [0.1] * 7, "right": [0.2] * 7,
                      "waist": [0.0, 0.0, 0.0]}
    assert card._seed_for("left") == [0.1] * 7


def test_holding_does_not_forget_the_seed():
    """种子是构型的记忆，而机器人在保持期间并没有移动。

    清掉它会让恢复后的第一解从零位开始 —— 一次没人要求过的大幅重构型。
    """
    card = _card()
    card._apply(_pose_command(card), None)
    remembered = card._seed_for("left")
    card._hold()
    assert card._seed_for("left") == remembered


# ── 状态上报：增量模型的基准位姿 ────────────────────────────────────────────


class _FakeStatePublisher:
    def __init__(self):
        self.published = []

    def publish(self, message):
        self.published.append(json.loads(message.data))


def test_the_state_payload_carries_both_the_joints_and_the_end_effector_pose():
    """`eef` 是给 OpenVLA 那类增量模型当基准位姿用的：它输出末端 delta，
    「相对于哪儿」只有机器人知道。

    **和关节状态在同一路话题里。** agent-core 的 VLA 卡片按 ROS 消息类型分派角色，
    两路 `String` 它分不开 —— 新开一路会变成按连线顺序赌运气的绑定。
    """
    pytest.importorskip("std_msgs", reason="状态发布要 ROS 的 String 消息类型")
    card = _card()
    card._state_pub = _FakeStatePublisher()
    card._measured = {"left": [0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0],
                      "right": [0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0],
                      "waist": [0.0, 0.0, 0.0]}
    card._measured_ms = 1_700_000_000_000

    card._publish_state()

    payload = card._state_pub.published[-1]
    assert payload["kind"] == "joint_state"
    assert len(payload["values"]) == 17          # 14 臂 + 3 腰
    assert len(payload["eef"]) == 19             # 和这张卡声明的布局同构
    # 两个四元数都要是单位的，否则上游拿它当基准会越叠越偏。
    for span in (slice(3, 7), slice(11, 15)):
        assert math.isclose(sum(v * v for v in payload["eef"][span]), 1.0,
                            abs_tol=1e-9)
    # 时间戳取实测那一帧，不是现在 —— 报现在等于宣称通道刚刚更新过。
    assert payload["stamp_ms"] == 1_700_000_000_000
