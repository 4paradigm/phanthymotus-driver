"""`rt/arm_sdk` 的权重渐变 —— G1 上肢下发链路里最危险的一处。

这些用例原本在 test_g1_servo.py 里，随 `ArmSdkChannel` 一起搬过来：现在有两张卡片
（`servo` 的 16 维关节空间、`servo_eef` 的 19 维末端位姿）走同一条通道，而通道里
的东西是安全逻辑 —— 权重从 1 直接归零就是手臂瞬间脱力，栈里没有别的地方会注意到。
被测的行为一个字没变，只是不再从卡片里伸手去够。

不需要机器人、不需要 ROS、不需要宇树 SDK：`arm_sdk.py` 在模块顶层不 import 它们
中的任何一个，所以整条渐变逻辑能在笔记本上跑。一个只能在 G1 上跑的测试，是在手臂
已经动过之后才跑的测试。

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_g1_arm_sdk.py -q
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _load():
    bundle = ROOT / "unitree" / "g1"
    spec = importlib.util.spec_from_file_location("g1_arm_sdk", bundle / "arm_sdk.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


arm_sdk = _load()


class _FakePublisher:
    def __init__(self):
        self.writes = []

    def Write(self, message):  # noqa: N802 —— SDK 的大小写
        self.writes.append(message.motor_cmd[arm_sdk.WEIGHT_MOTOR_ID].q)


class _FakeMotorCmd:
    def __init__(self):
        self.q = 0.0
        self.dq = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.tau = 0.0


class _FakeMessage:
    def __init__(self):
        self.motor_cmd = [_FakeMotorCmd() for _ in range(35)]
        self.crc = 0


def _channel(monkeypatch, **config):
    monkeypatch.setattr(arm_sdk.time, "sleep", lambda _s: None)
    channel = arm_sdk.ArmSdkChannel(**config)
    channel._arm_pub = _FakePublisher()
    channel._message = _FakeMessage()
    channel._crc = None
    return channel


# ── 渐变 ─────────────────────────────────────────────────────────────────────


def test_handback_reaches_exactly_zero_and_never_jumps(monkeypatch):
    """权重从 1 直接归零 = 手臂瞬间脱力自由落体。

    这条同时钉两件事：**终点必须正好是 0**（停在 0.03 意味着没完全交还），
    以及**中间不能有大台阶**（一步跨过去等于没有渐变）。
    """
    channel = _channel(monkeypatch)
    channel._weight = 1.0
    channel.ramp(1.0, 0.0, arm_sdk.HANDBACK_S)

    written = channel._arm_pub.writes
    assert written, "渐变期间必须持续发布，否则权重根本传不出去"
    assert written[-1] == 0.0
    assert channel.weight == 0.0
    biggest_step = max(abs(b - a) for a, b in zip(written, written[1:]))
    assert biggest_step < 0.05, f"权重出现了 {biggest_step:.3f} 的台阶"


def test_handback_takes_the_vendor_two_seconds(monkeypatch):
    """两秒是官方例程的值，不是手感参数。步数少了就等于把渐变压缩掉了。"""
    channel = _channel(monkeypatch)
    channel._weight = 1.0
    channel.ramp(1.0, 0.0, arm_sdk.HANDBACK_S)
    assert len(channel._arm_pub.writes) == int(arm_sdk.HANDBACK_S * arm_sdk.RAMP_HZ)


def test_takeover_is_slower_at_the_start_than_linear(monkeypatch):
    """接管走 `weight*weight`，和官方例程一致 —— 前段更慢。

    线性接管会在交接的前几十毫秒里就把大部分权重给出去，而那正是内置控制器
    还握着这些电机的时候。
    """
    channel = _channel(monkeypatch)
    channel.ramp(0.0, 1.0, arm_sdk.TAKEOVER_S)
    written = channel._arm_pub.writes
    half = written[len(written) // 2]
    assert half < 0.5, f"接管到一半时权重已经是 {half:.3f}，比线性还快"
    assert written[-1] == pytest.approx(1.0)


def test_the_ramp_keeps_sending_the_last_accepted_target(monkeypatch):
    """渐变期间发的是最后一个被接受的目标，不是零位。

    发零位等于在交还的两秒里把手臂拉回零位再松开 —— 一个没人要求过的动作。
    """
    channel = _channel(monkeypatch)
    channel._last_target = [0.3] * 14
    channel._weight = 1.0
    channel.ramp(1.0, 0.0, 0.1)
    for motor_id in arm_sdk.ARM_MOTOR_IDS:
        assert channel._message.motor_cmd[motor_id].q == pytest.approx(0.3)


def test_forgetting_the_target_does_not_touch_the_weight(monkeypatch):
    """看门狗与 abort 的处置。降权重会让手臂在流中断的瞬间松开手上的东西，
    而流中断最常见的原因是上游模型卡了一拍。"""
    channel = _channel(monkeypatch)
    channel._weight = 1.0
    channel._last_target = [0.1] * 14
    channel.forget_target()
    assert channel.weight == 1.0
    assert channel.last_target is None
    assert channel._arm_pub.writes == [], "不发就是保持"


# ── 发布 ─────────────────────────────────────────────────────────────────────


def test_the_arm_joints_go_out_in_radians_with_no_conversion(monkeypatch):
    channel = _channel(monkeypatch)
    values = [0.1 * i for i in range(14)]
    channel.publish_arms(values)
    for motor_id, expected in zip(arm_sdk.ARM_MOTOR_IDS, values):
        assert channel._message.motor_cmd[motor_id].q == pytest.approx(expected)


def test_the_waist_is_only_written_when_the_channel_was_opened_for_it(monkeypatch):
    """腰的三个轴里只有 yaw 在官方 arm7 例程的可写列表里，roll/pitch 没有文档。

    所以「写不写腰」是一个显式的开关，不是「给了就写」——一条默认会去动腰的通道，
    在一台腰被锁死的 G1 上会安静地什么都不发生，而在另一台上会动。
    """
    without = _channel(monkeypatch, waist=False)
    without.publish_arms([0.0] * 14, waist=(0.1, 0.2, 0.3))
    assert without._message.motor_cmd[arm_sdk.WAIST_MOTOR_IDS["yaw"]].q == 0.0

    with_waist = _channel(monkeypatch, waist=True)
    with_waist.publish_arms([0.0] * 14, waist=(0.1, 0.2, 0.3))
    motors = with_waist._message.motor_cmd
    assert motors[arm_sdk.WAIST_MOTOR_IDS["roll"]].q == pytest.approx(0.1)
    assert motors[arm_sdk.WAIST_MOTOR_IDS["pitch"]].q == pytest.approx(0.2)
    assert motors[arm_sdk.WAIST_MOTOR_IDS["yaw"]].q == pytest.approx(0.3)


def test_the_wrists_get_the_weaker_gains(monkeypatch):
    """腕的 effort 只有肩肘的五分之一，同一档增益会把它推过头。

    增益是在 `open()` 里填的，而 `open()` 要 SDK —— 所以这里直接调那段填充逻辑
    做不到。改为断言常量本身的关系：这条真正要防的是有人把两档并成一档。
    """
    assert arm_sdk.KP_WRIST < arm_sdk.KP_ARM / 2
    assert arm_sdk.KD_WRIST < arm_sdk.KD_ARM
    assert arm_sdk.WRIST_MOTOR_IDS == {19, 20, 21, 26, 27, 28}
    assert arm_sdk.WRIST_MOTOR_IDS < set(arm_sdk.ARM_MOTOR_IDS)


# ── 接管之前必须先知道手臂在哪 ──────────────────────────────────────────────


class _FakeMotorState:
    def __init__(self, q, mode=1):
        self.q = q
        self.mode = mode


class _FakeLowState:
    """29 个关节，每个给一个能认出来的值。"""

    def __init__(self):
        self.motor_state = [_FakeMotorState(i / 100.0) for i in range(35)]


def _stub_low_state(monkeypatch, deliver=True):
    """替掉 `ChannelSubscriber`，模拟 rt/lowstate 到达或者不到达。"""
    import types

    class _Subscriber:
        def __init__(self, topic, kind):
            self.topic = topic

        def Init(self, callback, depth):  # noqa: N802 —— SDK 的大小写
            if deliver:
                callback(_FakeLowState())

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = _Subscriber
    channel.ChannelPublisher = lambda *a, **k: types.SimpleNamespace(
        Init=lambda: None, Write=lambda m: None)
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    dds.LowState_ = object
    dds.LowCmd_ = object
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.unitree_hg.msg.dds_", dds)


def test_the_measured_arm_angles_are_read_before_anything_is_published(monkeypatch):
    """**这条测的是那个会让手臂甩起来的默认值。**

    `LowCmd_` 里每个 `motor_cmd[i].q` 默认是 0.0，而 0 不是「不控制」是「去零位」。
    只填 kp/kd 就渐入权重，发出去的是「双臂去全零位」—— 手臂以 kp=300 在一秒内甩
    直，而日志里什么都没有。所以 `open()` 必须先拿实测角把 q 填上。
    """
    _stub_low_state(monkeypatch)
    channel = arm_sdk.ArmSdkChannel(grippers=False)
    measured = channel._read_measured_arms()

    assert len(measured) == 14
    # ARM_MOTOR_IDS 是 15..28，桩里第 i 个关节的 q 是 i/100。
    assert measured[0] == pytest.approx(0.15)
    assert measured[-1] == pytest.approx(0.28)


def test_without_a_low_state_frame_it_refuses_to_open(monkeypatch):
    """不知道手臂在哪就接管，是这条链路上最危险的一种。

    用零位兜底会让它「看起来能启动」，然后甩臂。拒绝是唯一能把它变成一次可观察
    失败的处置 —— `_start` 会把卡片回滚成 idle 并报出原因。
    """
    _stub_low_state(monkeypatch, deliver=False)
    channel = arm_sdk.ArmSdkChannel(grippers=False)
    monkeypatch.setattr(channel.__class__, "LOW_STATE_TIMEOUT_S", 0.05)

    with pytest.raises(RuntimeError, match="rt/lowstate"):
        channel._read_measured_arms()


def test_the_ramp_holds_the_arms_where_they_were_found(monkeypatch):
    """接管渐入期间重发的，必须是手臂此刻所在的位置。

    这是模块文档一直声称、而代码此前并没有做到的那件事。
    """
    monkeypatch.setattr(arm_sdk.time, "sleep", lambda _s: None)
    _stub_low_state(monkeypatch)
    channel = arm_sdk.ArmSdkChannel(grippers=False)
    measured = channel._read_measured_arms()
    channel._last_target = list(measured)
    channel._arm_pub = _FakePublisher()
    channel._message = _FakeMessage()
    channel._crc = None

    channel.ramp(0.0, 1.0, arm_sdk.TAKEOVER_S)

    for motor_id, expected in zip(arm_sdk.ARM_MOTOR_IDS, measured):
        assert channel._message.motor_cmd[motor_id].q == pytest.approx(expected)
        assert channel._message.motor_cmd[motor_id].q != 0.0


def test_the_low_state_subscriber_is_closed_after_the_one_frame(monkeypatch):
    """只要一帧，拿到就关。

    留着它意味着一条 500 Hz 的订阅活到进程结束。这个仓库里「孤儿订阅」的历史是：
    卡片 stop 之后回调还在跑，ROS 图看着健康，直到某次 teardown 顺序出错把整层
    订阅一起带走 —— 而那时候症状指向的是别的地方。
    """
    import types

    closed = []

    class _Subscriber:
        def __init__(self, topic, kind):
            pass

        def Init(self, callback, depth):  # noqa: N802
            callback(_FakeLowState())

        def Close(self):  # noqa: N802
            closed.append(True)

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = _Subscriber
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    dds.LowState_ = object
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.unitree_hg.msg.dds_", dds)

    arm_sdk.ArmSdkChannel(grippers=False)._read_measured_arms()
    assert closed == [True]


def test_the_subscriber_is_closed_even_when_no_frame_arrives(monkeypatch):
    """超时路径同样要关 —— 那条路上我们已经建了订阅，只是没等到数据。"""
    import types

    closed = []

    class _Subscriber:
        def __init__(self, topic, kind):
            pass

        def Init(self, callback, depth):  # noqa: N802
            pass                       # 什么都不送

        def Close(self):  # noqa: N802
            closed.append(True)

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = _Subscriber
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    dds.LowState_ = object
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.unitree_hg.msg.dds_", dds)

    channel_obj = arm_sdk.ArmSdkChannel(grippers=False)
    monkeypatch.setattr(channel_obj.__class__, "LOW_STATE_TIMEOUT_S", 0.05)
    with pytest.raises(RuntimeError):
        channel_obj._read_measured_arms()
    assert closed == [True]


def test_the_machine_type_is_echoed_back_or_the_robot_ignores_us(monkeypatch):
    """`mode_machine` 是**硬件型号握手**，不回传机器人就静默忽略整条指令。

    真机上证实过：G1 报 `mode_machine=4`，我们发默认的 0，于是权重渐入正常、
    IK 残差 0.26 mm、`rt/arm_sdk` 照发，而手臂一动不动 —— 链路每一环都「成功」。
    """
    import types

    class _Frame(_FakeLowState):
        mode_machine = 4
        mode_pr = 0

    class _Subscriber:
        def __init__(self, topic, kind):
            pass

        def Init(self, callback, depth):  # noqa: N802
            callback(_Frame())

        def Close(self):  # noqa: N802
            pass

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = _Subscriber
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    dds.LowState_ = object
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.unitree_hg.msg.dds_", dds)

    c = arm_sdk.ArmSdkChannel(grippers=False)
    c._read_measured_arms()
    assert c._mode_machine == 4


def test_a_joint_the_robot_does_not_have_refuses_the_start(monkeypatch):
    """`mode == 0` 表示这个关节不存在或未使能。

    实测的那台 G1 是 23dof/arm5：两条手臂的 wrist_pitch/wrist_yaw 都是 mode=0。
    往它们写目标不报错，只会让手臂到不了 IK 解出来的位姿，而残差（在**模型**里
    算的）一切正常 —— 残差校验的是求解器，不是机器人。
    """
    import types

    class _Arm5Frame(_FakeLowState):
        mode_machine = 10

        def __init__(self):
            super().__init__()
            for i in (20, 21, 27, 28):        # 两侧 wrist_pitch / wrist_yaw
                self.motor_state[i].mode = 0

    class _Subscriber:
        def __init__(self, topic, kind):
            pass

        def Init(self, callback, depth):  # noqa: N802
            callback(_Arm5Frame())

        def Close(self):  # noqa: N802
            pass

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = _Subscriber
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    dds.LowState_ = object
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.unitree_hg.msg.dds_", dds)

    with pytest.raises(RuntimeError, match="mode=0"):
        arm_sdk.ArmSdkChannel(grippers=False)._read_measured_arms()


def test_the_waist_check_follows_the_variant_too(monkeypatch):
    """23dof 的机器上腰只有 yaw —— roll/pitch 报 mode=0。

    真机上就是在这里挂的一次：臂关节按型号放行了，腰还在按三个要，于是卡片拒绝
    启动并点名 `[13, 14]`。守卫没错，是通道的腰处理没跟着型号走。
    """
    import types

    class _Arm5Frame(_FakeLowState):
        mode_machine = 4

        def __init__(self):
            super().__init__()
            for i in (20, 21, 27, 28, 13, 14):     # 腕 pitch/yaw + 腰 roll/pitch
                self.motor_state[i].mode = 0

    class _Subscriber:
        def __init__(self, topic, kind):
            pass

        def Init(self, callback, depth):  # noqa: N802
            callback(_Arm5Frame())

        def Close(self):  # noqa: N802
            pass

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = _Subscriber
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    dds.LowState_ = object
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.unitree_hg.msg.dds_", dds)

    arm5_ids = list(range(15, 20)) + list(range(22, 27))

    # 腰按 29dof 要三个 → 应当拒绝，并点名 13/14
    with pytest.raises(RuntimeError, match="13"):
        arm_sdk.ArmSdkChannel(grippers=False, waist=True,
                              driven_arm_ids=arm5_ids)._read_measured_arms()

    # 腰按 23dof 只要 yaw → 应当通过
    c = arm_sdk.ArmSdkChannel(grippers=False, waist=True,
                              driven_arm_ids=arm5_ids,
                              driven_waist_names=("yaw",))
    assert len(c._read_measured_arms()) == 10


def test_only_the_waist_joints_the_robot_has_get_written(monkeypatch):
    """入参永远是标准布局的 [roll, pitch, yaw]，写的只是这台真有的那些。"""
    c = _channel(monkeypatch, waist=True, driven_waist_names=("yaw",))
    c.publish_arms([0.0] * 14, waist=(0.9, -0.9, 0.42))
    motors = c._message.motor_cmd
    assert motors[arm_sdk.WAIST_MOTOR_IDS["yaw"]].q == pytest.approx(0.42)
    assert motors[arm_sdk.WAIST_MOTOR_IDS["roll"]].q == 0.0      # 没写
    assert motors[arm_sdk.WAIST_MOTOR_IDS["pitch"]].q == 0.0


# ── 夹爪装没装，也要问机器人 ────────────────────────────────────────────────
#
# 这一组的由来：办公室那台 G1 的 config.yaml 写着 `grippers: true`，而机器上两条
# 接法一条都没有。卡片照样声明 19 维、照样往 `rt/dex1/left/cmd` 发 —— 一个没有任何
# 订阅者的话题。协商过、消息校验过、两个自由度去了虚空。
#
# 型号（arm5/arm7）和关节使能早就是问机器人得来的，夹爪是最后一处还在信配置的。


def _stub_dex1(monkeypatch, *, internal, external, seen=None):
    """按话题分派的假订阅：lowstate 一定给，dex1 state 看 `external`。

    `internal` 决定 motor 31/33 的 mode —— 两条接法在 DDS 上长得完全不一样，
    所以假件也必须把它们分开，否则测的是一个现实中不存在的机器人。
    """
    import types

    class _Frame(_FakeLowState):
        mode_machine = 4

        def __init__(self):
            super().__init__()
            for i in arm_sdk.DEX1_INTERNAL_MOTOR_IDS.values():
                self.motor_state[i].mode = 1 if internal else 0

    class _Subscriber:
        def __init__(self, topic, kind):
            self.topic = topic
            if seen is not None:
                seen.append(topic)

        def Init(self, callback, depth):  # noqa: N802
            if self.topic == arm_sdk.LOW_STATE_TOPIC:
                callback(_Frame())
            elif external:
                callback(types.SimpleNamespace(states=[types.SimpleNamespace(q=0.0)]))

        def Close(self):  # noqa: N802
            pass

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = _Subscriber
    hg = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    hg.LowState_ = object
    go = types.ModuleType("unitree_sdk2py.idl.unitree_go.msg.dds_")
    go.MotorStates_ = object
    go.MotorCmds_ = object
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.unitree_hg.msg.dds_", hg)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.unitree_go.msg.dds_", go)
    monkeypatch.setattr(arm_sdk, "DEX1_STATE_TIMEOUT_S", 0.05)


def test_a_gripper_the_robot_does_not_have_refuses_the_start(monkeypatch):
    """两条接法都探不到 → 拒绝，并说清楚去改哪个开关。

    降级成「悄悄少两维」是**不行的**：卡片已经把 19 报给了协商，改宽度等于让
    `dof` 这个字段撒谎，而它存在的全部理由就是抓这件事。
    """
    _stub_dex1(monkeypatch, internal=False, external=False)
    with pytest.raises(RuntimeError, match="grippers"):
        arm_sdk.ArmSdkChannel(grippers=True)._read_measured_arms()


def test_an_internally_wired_gripper_is_refused_rather_than_published_at(monkeypatch):
    """探到了，但驱动不了 —— 这也是拒绝，不是「试试看」。

    内部走线的 Dex1 不听 `rt/dex1/*/cmd`，它要写 LowCmd_ 的 31/33 号电机。往前者
    发不会报错也不会动，和「没装」的症状一模一样。
    """
    _stub_dex1(monkeypatch, internal=True, external=False)
    with pytest.raises(RuntimeError, match="内部走线"):
        arm_sdk.ArmSdkChannel(grippers=True)._read_measured_arms()


def test_an_externally_wired_gripper_is_accepted(monkeypatch):
    """这条是我们真的实现了的那一条。"""
    _stub_dex1(monkeypatch, internal=False, external=True)
    channel = arm_sdk.ArmSdkChannel(grippers=True)
    assert len(channel._read_measured_arms()) == 14
    assert channel._dex1 == "external"


def test_the_internal_check_costs_nothing_when_it_answers(monkeypatch):
    """内部走线探到了就不去等外接话题 —— 否则每次启动白等一秒。"""
    topics: list[str] = []
    _stub_dex1(monkeypatch, internal=True, external=False, seen=topics)
    with pytest.raises(RuntimeError):
        arm_sdk.ArmSdkChannel(grippers=True)._read_measured_arms()
    assert not [t for t in topics if t.startswith("rt/dex1/")]


def test_without_grippers_nothing_is_probed_at_all(monkeypatch):
    """`grippers=False` 是一张 14/17 维的卡片，它跟 Dex1 没有任何关系。"""
    topics: list[str] = []
    _stub_dex1(monkeypatch, internal=False, external=False, seen=topics)
    assert len(arm_sdk.ArmSdkChannel(grippers=False)._read_measured_arms()) == 14
    assert topics == [arm_sdk.LOW_STATE_TOPIC]
