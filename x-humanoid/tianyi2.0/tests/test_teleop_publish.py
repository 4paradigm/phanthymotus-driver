"""Exercise the actual ArmPlugin/HandPlugin writers with recording ROS stubs."""
import math
from types import SimpleNamespace
import pytest
from test_servo import device_mod, ros_stubs  # noqa: F401 - scoped ROS fixture
from teleop_executor import TeleopExecutor


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_executor_uses_current_vendor_writers_with_correct_units_and_hand_polarity(ros_stubs):
    arm = device_mod.ArmPlugin.__new__(device_mod.ArmPlugin)
    arm._pos_publisher = Recorder()
    hand = device_mod.HandPlugin.__new__(device_mod.HandPlugin)
    hand._left_pub, hand._right_pub = Recorder(), Recorder()
    executor = TeleopExecutor({}, 'test', None, arm, hand, [])
    executor._output_ready = True
    executor.gate.velocity = .6
    executor.profile = {"hands": {s: {"open": [0]*6, "closed": [80]*6}
                                   for s in ("left", "right")}}
    # Deliberately asymmetric targets expose mirroring and ordering errors.
    target = [math.radians(i) for i in range(-7, 7)]
    executor._emit(target, [.25, .75])
    commands = arm._pos_publisher.messages[-1].cmds
    assert [c.name for c in commands] == list(range(11,18)) + list(range(21,28))
    assert [c.pos for c in commands] == pytest.approx(target)
    assert [c.spd for c in commands] == [.6]*14
    assert [c.cur for c in commands] == [35,23,8,8,8,5,5]*2
    assert hand._left_pub.messages[-1].position == pytest.approx([.8]*6)
    assert hand._right_pub.messages[-1].position == pytest.approx([.4]*6)
    executor._emit(target, None)
    assert len(hand._left_pub.messages) == len(hand._right_pub.messages) == 1
    # Arm-only profile must suppress even explicit full-close targets at the
    # vendor writer, without requiring invented open/closed calibration.
    executor.profile = {'hands_enabled': False}
    executor._emit(target, [1., 1.])
    assert len(arm._pos_publisher.messages) == 3
    assert len(hand._left_pub.messages) == len(hand._right_pub.messages) == 1


def test_vendor_writer_failure_is_not_success(ros_stubs):
    executor = TeleopExecutor.__new__(TeleopExecutor)
    executor._output_ready = True
    executor.gate = SimpleNamespace(velocity=.2)
    executor.arm = device_mod.ArmPlugin.__new__(device_mod.ArmPlugin)
    executor.arm._pos_publisher = None
    with pytest.raises(ValueError, match="arm_publish_failed"):
        executor._emit([0.]*14, None)


@pytest.mark.parametrize('hands_enabled', [False, True])
def test_hand_diagnostics_only_gate_enabled_hands(ros_stubs, monkeypatch, hands_enabled):
    import sys
    import time
    monkeypatch.setitem(sys.modules, 'device', device_mod)
    executor = TeleopExecutor({}, 'test', None, None, None, [])
    now = time.monotonic_ns()
    maps = {'head': device_mod._HEAD_JOINTS, 'waist': device_mod._WAIST_JOINTS,
            'leg': device_mod._LEG_JOINTS}
    executor.profile = {'hands_enabled': hands_enabled, 'fixed_motor_positions_rad': {
        str(mid): 0. for ids in maps.values() for mid in ids}}
    for part, ids in maps.items():
        executor._streams[part] = (now, {mid: (0., 0., 0) for mid in ids})
    executor._streams['arm'] = (now, {mid: (0., 0., 0) for mid in
                                    list(range(11,18)) + list(range(21,28))})
    executor._power = (now, True, False)
    feedback = executor.feedback()
    assert feedback['hand_ns'] == 0  # Missing is not fabricated into fresh.
    assert len(feedback['hand_fault_reasons']) == 2
    assert feedback['fault'] == hands_enabled
    # Actual arm hardware errors are never hidden by arms-only mode.
    executor._streams['arm'][1][11] = (0., 0., 4)
    assert 'arm_motor_11_error_4' in executor.feedback()['fault_reasons']

def test_session_reference_tracks_actual_body_and_detects_later_change(ros_stubs, monkeypatch):
    import sys,time
    monkeypatch.setitem(sys.modules,'device',device_mod)
    item=TeleopExecutor({},'test',None,None,None,[])
    item.profile={'hands_enabled':False,'fixed_motor_positions_rad':{'2':0.}}
    item.gate.hands_enabled=False
    now=time.monotonic_ns()
    for part,ids in [('head',device_mod._HEAD_JOINTS),('waist',device_mod._WAIST_JOINTS),('leg',device_mod._LEG_JOINTS)]:
        item._streams[part]=(now,{mid:(-.366 if mid==2 else 0.,0.,0) for mid in ids})
    item._streams['arm']=(now,{mid:(0.,0.,0) for mid in list(range(11,18))+list(range(21,28))})
    item._power=(now,True,False)
    with item.gate.lock:item._capture_fixed_baseline()
    assert item.feedback()['fixed_body']
    assert item.feedback()['fixed_reference_positions_rad']['2']==-.366
    assert item.profile['fixed_motor_positions_rad']['2']==0.
    item._streams['head'][1][2]=(-.30,0.,0)
    assert not item.feedback()['fixed_body']
    with pytest.raises(ValueError,match='calibrated_body_joints_changed'):item.gate._feedback()
    assert item.feedback()['fixed_reference_positions_rad']['2']==-.366
