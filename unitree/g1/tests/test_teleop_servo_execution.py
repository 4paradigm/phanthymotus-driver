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


def test_new_target_written_once_with_smoothing_without_gravity(tmp_path):
    arm, sample, channel, advance, gravity, sdk = setup_servo(tmp_path)
    assert arm._gravity is None
    assert arm.accept_control(packet(arm, target=.5))
    for _ in range(5):
        advance(10); arm.tick()
    assert len(channel.positions) == 1
    assert channel.positions[0][0] == pytest.approx([.01]*10)
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
    assert all(0 < x < .1 for x in channel.positions[-1][0])
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


def test_regrip_can_resume_while_moving_and_large_gap_is_smoothed(tmp_path):
    arm, sample, channel, advance, _, _ = setup_servo(tmp_path)
    arm.accept_control(packet(arm,seq=1,target=.5));advance(10);arm.tick()
    previous=channel.positions[-1][0][:]
    arm.hold('operator_pause',recoverable=False)
    sample['dq']=[.1]*10
    advance(10);arm.tick()
    assert not arm.status()['hold_confirmed'] and arm.status()['resume_ready']
    result=arm.dispatch('resume',{'session_id':arm.session_id,'secret':arm.secret})
    assert result['state']=='ready'
    advance(2000)
    assert arm.accept_control(packet(arm,seq=1,target=-.5))
    advance(10);arm.tick()
    current=channel.positions[-1][0]
    assert all(-.5 < x < p for x,p in zip(current,previous))
    assert max(abs(x-p) for x,p in zip(current,previous)) < .2
    assert not arm.status()['stop_confirmed']


def test_smoothing_converges_without_resetting_to_measured(tmp_path):
    arm,sample,channel,advance,_,_=setup_servo(tmp_path)
    for seq in range(1,41):
        assert arm.accept_control(packet(arm,seq=seq,target=.5))
        advance(50,follow=False);arm.tick()
    values=[x[0][0] for x in channel.positions]
    assert all(a < b < .5 for a,b in zip(values,values[1:]))
    assert values[-1] == pytest.approx(.5,abs=1e-5)
    assert sample['q']==[0.]*10


def test_joint_reference_speed_bound_with_irregular_inputs_and_gaps(tmp_path):
    arm,sample,channel,advance,_,_=setup_servo(tmp_path)
    previous=[0.]*10
    for seq,ms in enumerate([5,20,50,150,10,80],1):
        advance(ms,follow=False)
        target=.8 if seq%2 else -.8
        assert arm.accept_control(packet(arm,seq=seq,target=target))
        arm.tick()
        current=channel.positions[-1][0]
        assert max(abs(a-b) for a,b in zip(current,previous)) <= min(ms/1000,.05)+1e-12
        previous=current
