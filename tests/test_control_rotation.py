"""The four quaternion helpers the `eef_pose` action space rests on.

Separate from test_control_sink.py because these are pure arithmetic with no
clock, no descriptor and no sink — and because the properties worth asserting
here are the ones that make the *sink's* checks correct: q ≡ -q, slerp keeps
unit length, and the angle is the rotation angle rather than the half-angle.

Run: python3 -m pytest tests/test_control_rotation.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.control import rotation  # noqa: E402

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def about_z(angle: float):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


# ── is_unit ──────────────────────────────────────────────────────────────────

def test_the_tolerance_admits_float32_noise_and_rejects_another_representation():
    """两边都要成立才有意义。

    太紧：一个从 float32 动作头出来、又过了一趟 JSON 的合法四元数会被拒。
    太松：一个 rpy 三元组加一个漏进来的数也能混过去，而那正是这条检查要抓的。
    """
    assert rotation.is_unit((0.0, 0.0, 0.0, 1.0 - 5e-4))
    assert not rotation.is_unit((0.1, 0.2, 0.3, 0.4))      # rpy + 一个 R6 的数
    assert not rotation.is_unit((0.0, 0.0, 0.0, 0.0))      # 全零，最常见的未初始化


def test_a_zero_quaternion_cannot_be_normalized():
    with pytest.raises(ValueError):
        rotation.normalize((0.0, 0.0, 0.0, 0.0))


# ── angle_between ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("angle", [0.0, 0.3, 1.2, math.pi / 2, 3.0])
def test_the_angle_is_the_rotation_angle_not_the_half_angle(angle):
    """差一个 2 倍就是钳位和限速全部少一半或者多一倍，而两种都不报错。"""
    assert rotation.angle_between(IDENTITY, about_z(angle)) == pytest.approx(angle)


def test_the_negated_quaternion_is_the_same_orientation():
    """`q` 与 `-q` 是同一个朝向。策略翻个号不要钱，按分量算就是 180°。"""
    flipped = tuple(-v for v in IDENTITY)
    assert rotation.angle_between(IDENTITY, flipped) == pytest.approx(0.0, abs=1e-9)


# ── slerp_limit ──────────────────────────────────────────────────────────────

def test_a_step_within_the_limit_passes_through_untouched():
    want = about_z(0.1)
    result, limited = rotation.slerp_limit(IDENTITY, want, 0.2)
    assert limited is False
    assert result == want                 # 逐位原样，不重新归一化


def test_a_step_over_the_limit_travels_exactly_the_limit():
    result, limited = rotation.slerp_limit(IDENTITY, about_z(1.2), 0.2)
    assert limited is True
    assert rotation.angle_between(IDENTITY, result) == pytest.approx(0.2)


def test_the_clamped_result_is_still_a_unit_quaternion():
    """这条是整个 slerp 存在的理由。

    逐分量钳位会得到一个不再是单位长度的向量，于是下一条消息在 sink 的契约检查
    处被拒 —— 症状是「策略一动快就全被拒」，而那条报错一个字都不提钳位。
    """
    for angle in (0.5, 1.2, 2.5, 3.0):
        result, _ = rotation.slerp_limit(IDENTITY, about_z(angle), 0.2)
        assert rotation.norm(result) == pytest.approx(1.0, abs=1e-12)


def test_the_clamp_takes_the_short_way_round():
    """转 350° 应当是往回 10°，不是往前 350°。"""
    result, limited = rotation.slerp_limit(
        IDENTITY, about_z(math.radians(350)), math.radians(4))
    assert limited is True
    # 往回走：绕 z 的分量为负。
    assert result[2] < 0


def test_two_identical_orientations_do_not_divide_by_zero():
    """sin(θ) → 0 是 slerp 的奇点，而「没动」是这条流上最常见的一帧。"""
    result, limited = rotation.slerp_limit(IDENTITY, IDENTITY, 0.0)
    assert limited is False
    assert result == IDENTITY
