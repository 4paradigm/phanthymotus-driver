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
