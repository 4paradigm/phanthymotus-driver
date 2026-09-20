"""The G1 servo card — bimanual continuous control over `rt/arm_sdk`, 16 dims.

`ControlSink` is tested on its own in test_control_sink.py. This file covers
what is G1-shaped, which is where a G1-shaped mistake would be:

  - **the handback ramp**, because a weight that drops to 0 in one step is arms
    that go limp in one step, and nothing else in the stack would notice
  - the descriptor's own shape and its left/right asymmetry, since a mirrored
    limit authorises half the wrong range and no command ever gets rejected
  - that a missing Dex1 message type is refused rather than silently dropping
    two of the sixteen dimensions

No robot, no ROS, no Unitree SDK: this module imports none of them at module
level, so the card can be loaded and its logic exercised on a laptop. That is
deliberate — a test that only runs on a G1 is a test that runs after the arms
have already moved.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_g1_servo.py -q
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _load():
    spec = importlib.util.spec_from_file_location(
        "g1_servo", ROOT / "unitree" / "g1" / "servo.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


servo = _load()


# ── descriptor ───────────────────────────────────────────────────────────────


def test_the_descriptor_is_accepted_by_the_shared_parser():
    """A descriptor that does not parse is a card that cannot start, and the
    error would surface at `start` on a robot rather than here."""
    parsed = servo.parse_descriptor(servo.build_descriptor())
    assert parsed.mode == "joint_position"
    assert parsed.dof == 16 == len(parsed.joint_names)
    assert sorted(parsed.resources) == ["arm_l", "arm_r", "gripper_l", "gripper_r"]


def test_the_shoulders_are_not_mirrored():
    """左 roll [-1.588, 2.252]、右 roll [-2.252, 1.588] —— 取自仓库里的 URDF。

    镜像一侧得到另一侧会授权错半个行程，而那**不会被任何一条指令拒绝**：每一条
    都落在（错误的）限位内。这是这个描述符里最容易错且最安静的一处。
    """
    parsed = servo.parse_descriptor(servo.build_descriptor())
    left_low, left_high = parsed.lower[1], parsed.upper[1]
    right_low, right_high = parsed.lower[8], parsed.upper[8]
    assert (left_low, left_high) == pytest.approx((-1.5882, 2.2515))
    assert (right_low, right_high) == pytest.approx((-2.2515, 1.5882))
    assert left_low != right_low and left_high != right_high


def test_the_speed_limit_is_not_the_urdf_hardware_limit():
    """URDF 写的 37 rad/s 是硬件极限，不是安全值。

    以它为上限，一个控制周期就能把手臂甩到对侧 —— 而 sink 会放行，因为它确实
    在限位内。这条钉的是「我们自己选了一个保守值」，不是「我们抄了 URDF」。
    """
    parsed = servo.parse_descriptor(servo.build_descriptor())
    assert all(v == servo.ARM_MAX_VELOCITY for v in parsed.max_velocity[:14])
    assert servo.ARM_MAX_VELOCITY < 5.0


def test_the_per_step_delta_follows_the_declared_rate():
    at30 = servo.parse_descriptor(servo.build_descriptor(30.0))
    at10 = servo.parse_descriptor(servo.build_descriptor(10.0))
    assert at10.max_delta_per_step[0] == pytest.approx(3 * at30.max_delta_per_step[0])


# ── 权重渐变：这张卡上最危险的一处 ───────────────────────────────────────────


class _FakePublisher:
    def __init__(self):
        self.writes = []

    def Write(self, message):  # noqa: N802 —— SDK 的大小写
        self.writes.append(message.motor_cmd[servo.WEIGHT_MOTOR_ID].q)


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


def _card(monkeypatch, **config):
    monkeypatch.setattr(servo.time, "sleep", lambda _s: None)
    card = servo.G1ServoPlugin(config, "g1", executor=None)
    card._arm_pub = _FakePublisher()
    card._msg = _FakeMessage()
    card._crc = None
    return card


def test_handback_reaches_exactly_zero_and_never_jumps(monkeypatch):
    """权重从 1 直接归零 = 手臂瞬间脱力自由落体。

    这条同时钉两件事：**终点必须正好是 0**（停在 0.03 意味着没完全交还），
    以及**中间不能有大台阶**（一步跨过去等于没有渐变）。
    """
    card = _card(monkeypatch)
    card._weight = 1.0
    card._ramp(1.0, 0.0, servo.HANDBACK_S)

    written = card._arm_pub.writes
    assert written, "渐变期间必须持续发布，否则权重根本传不出去"
    assert written[-1] == 0.0
    assert card._weight == 0.0
    biggest_step = max(abs(b - a) for a, b in zip(written, written[1:]))
    assert biggest_step < 0.05, f"权重出现了 {biggest_step:.3f} 的台阶"


def test_handback_takes_the_vendor_two_seconds(monkeypatch):
    """两秒是官方例程的值，不是手感参数。步数少了就等于把渐变压缩掉了。"""
    card = _card(monkeypatch)
    card._weight = 1.0
    card._ramp(1.0, 0.0, servo.HANDBACK_S)
    assert len(card._arm_pub.writes) == int(servo.HANDBACK_S * servo.RAMP_HZ)


def test_takeover_is_slower_at_the_start_than_linear(monkeypatch):
    """接管走 `weight*weight`，和官方例程一致 —— 前段更慢。

    线性接管会在交接的前几十毫秒里就把大部分权重给出去，而那正是内置控制器
    还握着这些电机的时候。
    """
    card = _card(monkeypatch)
    card._ramp(0.0, 1.0, servo.TAKEOVER_S)
    written = card._arm_pub.writes
    half = written[len(written) // 2]
    assert half < 0.5, f"接管到一半时权重已经是 {half:.3f}，比线性还快"
    assert written[-1] == pytest.approx(1.0)


def test_the_ramp_keeps_sending_the_last_accepted_target(monkeypatch):
    """渐变期间发的是最后一个被接受的目标，不是零位。

    发零位等于在交还的两秒里把手臂拉回零位再松开 —— 一个没人要求过的动作。
    """
    card = _card(monkeypatch)
    card._last_target = [0.3] * 14 + [0.0, 0.0]
    card._weight = 1.0
    card._ramp(1.0, 0.0, 0.1)
    for motor_id in servo.ARM_MOTOR_IDS:
        assert card._msg.motor_cmd[motor_id].q == pytest.approx(0.3)


# ── hold：看门狗不松手 ───────────────────────────────────────────────────────


def test_the_watchdog_holds_rather_than_releasing(monkeypatch):
    """指令流断掉最常见的原因是上游卡了一拍，那时静止是对的，脱力不是。

    `_hold` 必须**不动权重** —— 降权重会让手臂在流中断的瞬间松开手上的东西。
    """
    card = _card(monkeypatch)
    card._weight = 1.0
    card._last_target = [0.1] * 16
    card._hold()
    assert card._weight == 1.0, "看门狗不该改权重"
    assert card._arm_pub.writes == [], "看门狗不该发布任何东西；不发就是保持"


def test_pause_does_not_release_either(monkeypatch):
    card = _card(monkeypatch)
    card._running = True
    card._weight = 1.0
    assert card._halt(True)["state"] == "paused"
    assert card._weight == 1.0


# ── 配置与拒绝 ───────────────────────────────────────────────────────────────


def test_an_out_of_range_rate_is_refused():
    with pytest.raises(ValueError, match="expected_hz"):
        servo.G1ServoPlugin({"expected_hz": 500}, "g1", executor=None)


def test_starting_without_an_input_topic_is_refused(monkeypatch):
    card = _card(monkeypatch)
    assert card._start({})["state"] == "error"


def test_disabling_grippers_really_declares_a_14_dof_card():
    """关掉夹爪必须改变**声明的维度**，否则就是在骗协商。

    留在 16 维的话，一个 16 维模型照样协商通过，然后两个自由度静默不动 —— 正是
    `control_mode` 和 `dof` 存在要抓的那种失败。一台没装 Dex1 的 G1 是另一个动作
    空间，说出来才能让模型被拒绝，而不是被执行一半。

    这条测试第一版写的是「已知缺口」，断言 dof 仍然是 16。那是把一个真问题钉成
    了期望行为 —— 改掉了。
    """
    with_grippers = servo.G1ServoPlugin({}, "g1", executor=None)
    without = servo.G1ServoPlugin({"grippers": False}, "g1", executor=None)

    assert with_grippers._descriptor.dof == 16
    assert without._descriptor.dof == 14
    assert sorted(without._descriptor.resources) == ["arm_l", "arm_r"]
    # 单位表也要跟着掉 —— 一个没有归一化维度的描述符不该还声明 normalized。
    assert "normalized" not in without._descriptor.units
