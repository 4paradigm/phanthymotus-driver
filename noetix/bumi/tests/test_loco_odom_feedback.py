"""Optional state/odom feedback for loco_relative_move (position only)."""
import json
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
import types

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_distance_control import ExternalOdom  # noqa: E402


def sample(x, y, yaw_rad=0.0, *, segment='seg-a', status='tracking',
           pose_reference='body_axes'):
    half = yaw_rad/2.0
    return json.dumps(dict(
        schema='motus.odom/1', stamp_ms=123,
        pose=dict(position=[x, y, 0.0],
                  quaternion=[0.0, 0.0, math.sin(half), math.cos(half)],
                  frame_id='seg_odom', child_frame_id='seg_base',
                  covariance=[0.0]*36),
        vendor=dict(status=status, segment_id=segment,
                    pose_reference=pose_reference)))


def started(x=0.0, y=0.0, yaw=0.0, now=0.0):
    odom = ExternalOdom()
    odom.update(sample(x, y, yaw), now)
    odom.begin(now, timeout_s=0)
    return odom


def test_body_position_is_relative_to_the_move_start_frame():
    odom = started()
    odom.update(sample(0.5, 0.0), 0.5)
    assert odom.body_position(0.5) == pytest.approx((0.5, 0.0))
    # Origin heading 90°: world +y movement is body +x.
    turned = started(yaw=math.pi/2)
    turned.update(sample(0.0, 0.5), 0.5)
    assert turned.body_position(0.5) == pytest.approx((0.5, 0.0))


def test_stale_or_segment_reset_feedback_raises():
    odom = started()
    odom.update(sample(0.1, 0.0, segment='seg-b'), 0.2)
    with pytest.raises(ValueError, match='segment changed'):
        odom.body_position(0.2)
    stale = started()
    with pytest.raises(ValueError, match='stale'):
        stale.body_position(1.0)


def test_non_tracking_or_optical_frames_are_rejected():
    odom = ExternalOdom()
    odom.update(sample(0.0, 0.0, status='lost'), 100.0)
    assert odom.fresh(100.0) is False
    odom.update(sample(0.0, 0.0, pose_reference='rgb_optical'), 100.0)
    assert odom.fresh(100.0) is False


def test_position_jump_is_rejected():
    odom = started()
    odom.update(sample(0.5, 0.0), 0.5)
    odom.update(sample(5.0, 0.0), 0.6)
    assert odom.fresh(0.6) is False
