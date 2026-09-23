"""Real executor with fake SDK I/O; these tests never connect to a robot."""
import pytest
from test_arm_stream import rig, packet


def setup_servo(tmp_path):
    arm, sample, channel, advance, gravity, sdk = rig(tmp_path, servo=True)
    channel.positions = []
    channel.forgotten = 0
    def publish(q, *, weight):
        channel.positions.append((list(q), weight))
        return True
    def forget():
        channel.forgotten += 1
    channel.publish_servo_position = publish
    channel.forget_target = forget
    return arm, sample, channel, advance, gravity, sdk


def test_new_target_written_once_without_gravity(tmp_path):
    arm, sample, channel, advance, gravity, sdk = setup_servo(tmp_path)
    assert arm._gravity is None
    assert arm.accept_control(packet(arm, target=.5))
    for _ in range(5):
        advance(10); arm.tick()
    assert len(channel.positions) == 1
    assert channel.positions[0][0] == pytest.approx([.5]*10)
    assert gravity == channel.writes == sdk == []


def test_timeout_holds_without_new_target_and_fresh_input_resumes(tmp_path):
    arm, sample, channel, advance, gravity, sdk = setup_servo(tmp_path)
    arm.accept_control(packet(arm)); advance(10); arm.tick()
    sample['q'] = [.5]*10
    advance(301); arm.tick()
    advance(10); arm.tick()
    assert arm.status()['continuation_ready']
    assert channel.forgotten == 1 and len(channel.positions) == 1
    session = arm.session_id
    arm.accept_control(packet(arm, seq=2, target=.1)); advance(10); arm.tick()
    assert arm.session_id == session and arm.applied_seq == 2
    assert channel.positions[-1][0] == pytest.approx([.1]*10)
    assert gravity == sdk == []


def test_failed_write_is_not_reported_applied(tmp_path):
    arm, sample, channel, advance, gravity, sdk = setup_servo(tmp_path)
    channel.publish_servo_position = lambda *args, **kwargs: False
    arm.accept_control(packet(arm)); advance(10); arm.tick()
    assert arm.applied_seq == -1
    assert arm.status()['reason'] == 'arm_sdk_write_failed'


def test_first_ik_rejection_does_not_wait_for_uncommanded_motion(tmp_path):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    session=arm.session_id
    sample['dq']=[.035]*10
    arm.hold('ik_recoverable',recoverable=True)
    advance(10);arm.tick()
    assert arm.status()['continuation_ready']
    assert not channel.positions
    assert arm.accept_control(packet(arm,seq=1,target=.1))
    advance(10);arm.tick()
    assert arm.session_id==session and arm.applied_seq==1
    assert len(channel.positions)==1
    arm.hold('ik_recoverable',recoverable=True)
    advance(10);arm.tick();advance(10);arm.tick()
    assert arm.status()['continuation_ready']
    assert not arm.status()['hold_confirmed']  # No fabricated physical stop.
    assert arm.accept_control(packet(arm,seq=2,target=.2))
    advance(10);arm.tick()
    assert arm.applied_seq==2 and arm.session_id==session
    arm.hold('operator_pause',recoverable=False)
    advance(10);arm.tick()
    assert not arm.status()['continuation_ready']
    with pytest.raises(ValueError,match='hold_not_resumable'):
        arm.accept_control(packet(arm,seq=3,target=.3))
