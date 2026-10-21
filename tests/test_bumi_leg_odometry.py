"""Leg odometry kinematics and stance-foot integration (no hardware)."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'noetix' / 'bumi'))
import leg_odometry as module

LEFT = module.LEG_CHAINS['left']
RIGHT = module.LEG_CHAINS['right']
IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
STANDING_LEFT = [0.0301, 0.0995, -0.4802]
STANDING_RIGHT = [0.0301, -0.0995, -0.4802]


def stand_feet():
    return [list(STANDING_LEFT), list(STANDING_RIGHT)]


def test_standing_pose_puts_the_contact_at_the_sole_centre():
    left = module.foot_position([0.0]*6, LEFT)
    right = module.foot_position([0.0]*6, RIGHT)
    assert left == pytest.approx(STANDING_LEFT, abs=1e-3)
    assert right == pytest.approx(STANDING_RIGHT, abs=1e-3)
    # The ankle-roll link origin is the foot at z=0; the sole is 5.11 cm below.
    assert left[2] == pytest.approx(-0.4802, abs=1e-3)


def test_contact_point_follows_the_foot_roll_to_the_heel_or_toe():
    flat = module.foot_position([0.0]*6, LEFT)
    toe_down = module.foot_position([0.0, 0.0, 0.0, 0.0, 0.3, 0.0], LEFT)
    heel_down = module.foot_position([0.0, 0.0, 0.0, 0.0, -0.3, 0.0], LEFT)
    assert flat[0] == pytest.approx(0.0301, abs=1e-3)
    # Rolling the foot moves the lowest sole corner by the foot length; a fixed
    # point under the ankle would not capture that.
    assert toe_down[0] > flat[0]+0.05
    assert heel_down[0] < flat[0]-0.05


def test_forward_translation_from_both_stance_feet():
    estimator = module.LegOdometry()
    feet = stand_feet()
    estimator.observe_feet(feet, IDENTITY, 0.0)
    estimator.observe_feet([[foot[0]-0.005, foot[1], foot[2]] for foot in feet],
                           IDENTITY, 0.02)
    assert estimator.position == pytest.approx([0.005, 0.0, 0.0], abs=1e-9)
    assert estimator.velocity == pytest.approx([0.25, 0.0, 0.0], abs=1e-9)
    assert estimator.stance_samples == 1


def test_the_swing_foot_is_not_used_for_the_body_step():
    estimator = module.LegOdometry()
    feet = stand_feet()
    estimator.observe_feet(feet, IDENTITY, 0.0)
    # Left foot is planted: it implies a 5 mm body step. Right foot swings
    # forward: it implies a 20 mm step in the opposite direction.
    estimator.observe_feet([[feet[0][0]-0.005, feet[0][1], feet[0][2]],
                            [feet[1][0]+0.020, feet[1][1], feet[1][2]]],
                           IDENTITY, 0.02)
    assert estimator.position == pytest.approx([0.005, 0.0, 0.0], abs=1e-9)


def test_yaw_is_applied_when_projecting_the_step_into_the_start_frame():
    estimator = module.LegOdometry()
    feet = stand_feet()
    estimator.observe_feet(feet, IDENTITY, 0.0)
    yaw = math.radians(90)
    rotation = module._axis_rotation((0.0, 0.0, 1.0), yaw)
    # World displacement +x for a 90 deg yaw: the body-frame foot position
    # becomes inverse(R) * (p_prev - d_world).
    step = [0.005, 0.0, 0.0]
    inverse = module._axis_rotation((0.0, 0.0, 1.0), -yaw)
    estimator.observe_feet([module._apply(inverse, [foot[i]-step[i] for i in range(3)])
                            for foot in feet], rotation, 0.02)
    assert estimator.position == pytest.approx([0.005, 0.0, 0.0], abs=1e-6)


def test_sdk_motor_state_objects_are_accepted():
    motors = [NS(pos=0.0) for _ in range(21)]
    angles = module.joint_angles(motors)
    assert angles == [0.0]*12
    estimator = module.LegOdometry()
    estimator.observe(motors, (0.0, 0.0, 0.0, 1.0), 0.0)
    estimator.observe(motors, (0.0, 0.0, 0.0, 1.0), 0.02)
    assert estimator.position == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    assert estimator.samples == 2


def test_implausible_steps_are_ignored_instead_of_integrating_noise():
    estimator = module.LegOdometry()
    feet = stand_feet()
    estimator.observe_feet(feet, IDENTITY, 0.0)
    # 10 cm in 20 ms is 5 m/s: not a body step, discard.
    estimator.observe_feet([[foot[0]-0.10, foot[1], foot[2]] for foot in feet],
                           IDENTITY, 0.02)
    assert estimator.position == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    assert estimator.velocity == [0.0, 0.0, 0.0]


def test_invalid_sampling_interval_and_joint_count_are_rejected():
    estimator = module.LegOdometry()
    estimator.observe_feet(stand_feet(), IDENTITY, 0.0)
    with pytest.raises(ValueError, match='sampling interval'):
        estimator.observe_feet(stand_feet(), IDENTITY, 0.2)
    with pytest.raises(ValueError, match='21 SDK joint positions'):
        module.joint_angles([0.0]*5)


def test_quaternion_rotation_requires_a_finite_normalized_value():
    rotation = module.rotation_from_quaternion((0.0, 0.0, math.sin(0.1), math.cos(0.1)))
    assert rotation[0][0] == pytest.approx(math.cos(0.2), abs=1e-9)
    with pytest.raises(ValueError, match='quaternion'):
        module.rotation_from_quaternion((0.0, 0.0, 0.0, 0.0))
