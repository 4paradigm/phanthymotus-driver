import math

import pytest

from deep_robotics.s10_pro.relative_motion import RelativeController, RelativeMotionError


def pose(sequence=1, t=0., x=0., y=0., yaw=0., vx=0., vy=0., yaw_rate=0.):
    return {'fresh': True, 'sequence': sequence, 'received_monotonic': t,
            'x': x, 'y': y, 'yaw': yaw, 'vx': vx, 'vy': vy, 'yaw_rate': yaw_rate}


def test_forward_uses_starting_body_frame_and_settles():
    controller = RelativeController(pose(), direction='forward', target=.3,
                                    max_speed=.1, now=0.)
    command, status = controller.command(pose(2, .1, x=.05, vx=.08), .1)
    assert command == (.1, 0., 0.)
    assert status['phase'] == 'moving'
    controller.command(pose(3, .2, x=.13, vx=.08), .2)
    command, status = controller.command(pose(4, .3, x=.24, vx=.03), .3)
    assert command == (0., 0., 0.) and status['phase'] == 'settling'
    command, status = controller.command(pose(5, .8, x=.25), .8)
    assert command == (0., 0., 0.) and status['phase'] == 'completed'


def test_lateral_target_respects_initial_heading():
    initial = pose(yaw=math.pi / 2)
    controller = RelativeController(initial, direction='left', target=.3,
                                    max_speed=.1, now=0.)
    # At yaw=90 degrees, body-left is negative world X.
    command, status = controller.command(pose(2, .1, x=-.1, yaw=math.pi / 2, vy=.08), .1)
    assert command == (0., .1, 0.)
    assert status['progress'] == pytest.approx(.1)


def test_turn_unwraps_pi_boundary():
    initial = pose(yaw=math.radians(179))
    controller = RelativeController(initial, direction='turn_left', target=math.radians(30),
                                    max_speed=.15, now=0.)
    command, status = controller.command(
        pose(2, .1, yaw=math.radians(-176), yaw_rate=.1), .1)
    assert command == (0., 0., .15)
    assert status['progress'] == pytest.approx(math.radians(5))


def test_odometry_jump_fails_closed():
    controller = RelativeController(pose(), direction='forward', target=.3,
                                    max_speed=.1, now=0.)
    with pytest.raises(RelativeMotionError, match='safe bound') as caught:
        controller.command(pose(2, .1, x=1.), .1)
    assert caught.value.reason == 'odometry_position_jump'


def test_reverse_progress_fails_closed():
    controller = RelativeController(pose(), direction='forward', target=.3,
                                    max_speed=.1, now=0.)
    controller.command(pose(2, .1, x=.08, vx=.08), .1)
    with pytest.raises(RelativeMotionError) as caught:
        controller.command(pose(3, .2, x=.02, vx=-.08), .2)
    assert caught.value.reason == 'motion_reversed'


@pytest.mark.parametrize('sample,reason', [
    (pose(2, .1, x=0., y=.13), 'cross_track_exceeded'),
    (pose(2, .1, x=.02, yaw=math.radians(11)), 'odometry_heading_jump'),
])
def test_deviation_is_rejected(sample, reason):
    controller = RelativeController(pose(), direction='forward', target=.3,
                                    max_speed=.1, now=0.)
    with pytest.raises(RelativeMotionError) as caught:
        controller.command(sample, .1)
    assert caught.value.reason == reason


def test_overshoot_never_commands_more_forward_motion():
    controller = RelativeController(pose(), direction='forward', target=.3,
                                    max_speed=.1, now=0.)
    for seq, x in enumerate((.10, .20, .30), 2):
        controller.command(pose(seq, (seq - 1) * .1, x=x), (seq - 1) * .1)
    with pytest.raises(RelativeMotionError) as caught:
        controller.command(pose(5, .4, x=.40), .4)
    assert caught.value.reason == 'target_overshot'
