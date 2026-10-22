"""Recorded slow-IK timing and fail-closed stream recovery, with no ROS/robot."""
import pytest

from test_motion_stream import rig


def timeout_hold():
    gate, state, writes, advance, packet = rig()
    # First solve takes 58 ms of its 100 ms input budget.
    advance(58)
    gate.accept(packet(seq=8, valid_for_ms=42))
    gate.tick()
    advance(43)
    gate.tick()
    assert gate.state == 'hold' and gate.reason == 'command_timeout'
    assert writes[-1] == (state['q'], None)
    return gate, state, writes, advance, packet


def test_next_63ms_solve_continues_same_session_after_confirmed_hold():
    gate, state, writes, advance, packet = timeout_hold()
    session = gate.session_id
    assert gate.latest is None and not gate.status()['continuation_ready']
    # New packet before hold feedback: discard, never queue it for later.
    assert gate.accept(packet(seq=9, valid_for_ms=50)) is False
    assert gate.seq == 8 and gate.latest is None
    advance(20)
    gate.tick()
    assert gate.status()['continuation_ready']
    before = len(writes)
    # Next solve finished at 63 ms, with 37 ms of its own budget left.
    assert gate.accept(packet(seq=10, valid_for_ms=37))
    assert len(writes) == before
    gate.tick()
    assert gate.session_id == session and gate.applied_seq == 10 and gate.state == 'active'
    assert max(abs(q-m) for q,m in zip(writes[-1][0],state['q'])) <= .004+1e-12
    assert gate.diagnostics['continuations'] == 1
    assert gate.diagnostics['first_hold']['command']['valid_for_ms'] == 42


@pytest.mark.parametrize('change', [dict(valid_for_ms=101), dict(generated_ns=2_000_000_000),
                                   dict(seq=8), dict(q=[9.]*14), dict(session_id='old')])
def test_bad_packet_cannot_use_continuation(change):
    gate, _, writes, advance, packet = timeout_hold()
    advance(20);gate.tick()
    before = len(writes)
    deadline = gate._continuation_deadline
    with pytest.raises(ValueError):
        gate.accept(packet(**{'seq':9, **change}))
    assert len(writes) == before and gate.latest is None
    if 'session_id' in change:
        assert gate.status()['continuation_allowed'] and gate._continuation_deadline == deadline
        advance(300);gate.tick()
        assert not gate.status()['continuation_allowed']
    else:
        assert not gate.status()['continuation_allowed']


@pytest.mark.parametrize('release', [False, True])
def test_explicit_pause_or_release_closes_existing_timeout_window(release):
    gate, _, writes, advance, packet = timeout_hold()
    gate.hold('operator_pause', release=release)
    advance(20);gate.tick()
    assert not gate.status()['continuation_allowed']
    before = len(writes)
    with pytest.raises(ValueError, match='motion_not_armed'):
        gate.accept(packet(seq=9))
    assert len(writes) == before


def test_300ms_gap_requires_explicit_reenable_and_does_not_replay():
    gate, _, writes, advance, packet = timeout_hold()
    advance(20);gate.tick()
    advance(237);gate.tick()  # 300 ms since last valid command receipt.
    assert not gate.status()['continuation_allowed'] and gate.latest is None
    before = len(writes)
    with pytest.raises(ValueError, match='motion_not_armed'):
        gate.accept(packet(seq=9))
    assert len(writes) == before
    old = gate.session_id
    gate.resume()
    assert gate.session_id != old and gate.latest is None


@pytest.mark.parametrize('field', ['arm_ns','power_ns','fixed_ns','hand_ns'])
def test_200ms_feedback_gap_holds_then_requires_new_target(field):
    gate, state, writes, advance, packet = rig()
    gate.accept(packet());advance(20);gate.tick()
    state[field] -= 200_000_000
    before = len(writes)
    gate.tick()
    assert gate.state == 'hold' and gate.reason == field+'_stale'
    assert not gate.status()['hold_confirmed'] and gate.latest is None
    assert len(writes) == before  # Never publish a hold from stale feedback.
    advance(20);gate.tick();advance(20);gate.tick()
    assert gate.status()['continuation_ready']
    count = len(writes)
    gate.tick()
    assert len(writes) == count  # Recovery alone does not replay a target.
    gate.accept(packet(seq=1));gate.tick()
    assert gate.applied_seq == 1 and gate.state == 'active'


def test_first_fault_is_not_overwritten_and_records_exact_feedback_age():
    gate, state, _, advance, packet = rig()
    gate.accept(packet());advance(20);gate.tick()
    state['fixed_ns'] -= 301_000_000
    gate.tick()
    first = gate.status()['diagnostics']['first_fault']
    assert first['code'] == 'fixed_ns_stale' and first['feedback_age_ms']['fixed_ns'] == 301.
    state['arm_ns'] -= 401_000_000
    gate.tick()
    assert gate.reason == 'fixed_ns_stale'
    assert gate.diagnostics['last_fault']['code'] == 'arm_ns_stale'
    assert gate.diagnostics['first_fault'] == first
    advance(20);gate.tick();advance(20);gate.tick()
    assert gate.state == 'fault' and not gate.status()['continuation_allowed']
    with pytest.raises(ValueError):gate.accept(packet(seq=1))


@pytest.mark.parametrize('change', [dict(estop=True),dict(power_on=False),dict(fault=True)])
def test_real_safety_fault_does_not_wait_for_feedback_grace(change):
    gate, state, writes, advance, packet = rig()
    gate.accept(packet());advance(20);gate.tick()
    state['arm_ns'] -= 101_000_000
    state.update(change)
    before = len(writes)
    gate.tick()
    assert gate.state == 'fault' and gate.reason == 'robot_safety_not_ready'
    assert len(writes) == before and not gate.status()['continuation_allowed']


@pytest.mark.parametrize('age', [-1, 301])
def test_invalid_or_over_budget_feedback_faults_immediately(age):
    gate, state, _, advance, packet = rig()
    gate.accept(packet());advance(20);gate.tick()
    state['power_ns'] -= int(age*1e6)
    gate.tick()
    assert gate.state == 'fault'


def test_continuation_rechecks_fixed_body_and_acceptance():
    gate, state, _, advance, packet = timeout_hold()
    advance(20);gate.tick()
    state['fixed_body'] = False
    with pytest.raises(ValueError, match='calibrated_body_joints_changed'):
        gate.accept(packet(seq=9))
    assert not gate.status()['continuation_allowed']
    gate, _, _, advance, packet = timeout_hold()
    advance(20);gate.tick()
    gate.acceptance_check = lambda: False
    with pytest.raises(ValueError, match='live_acceptance_missing'):
        gate.accept(packet(seq=9))


def test_repeated_slow_targets_remain_bounded_with_a_lagging_plant():
    gate, state, writes, advance, packet = rig()
    for seq in range(200):
        gate.accept(packet(seq=seq, valid_for_ms=37))
        gate.tick()
        assert gate.state == 'active'
        assert max(abs(q-m) for q,m in zip(writes[-1][0],state['q'])) <= .004+1e-12
        advance(43);gate.tick()
        assert gate.state == 'hold' and gate.latest is None
        advance(20);gate.tick()
        assert gate.status()['continuation_ready']
    assert gate.diagnostics['continuations'] == 199 and gate.applied_seq == 199
    assert state['q'] == [0.]*14  # Deliberately never teleport measured feedback to target.


def test_expired_wire_packet_is_dropped_without_extending_continuation():
    import json
    from types import SimpleNamespace
    from teleop_executor import TeleopExecutor
    gate, state, writes, advance, packet = rig()
    gate.accept(packet());advance(20);gate.tick()
    deadline = gate._continuation_deadline
    executor = TeleopExecutor.__new__(TeleopExecutor)
    executor.gate = gate
    executor._trace_enabled = False
    before = len(writes)
    executor._command(SimpleNamespace(data=json.dumps(packet(seq=1, generated_ns=1))))
    assert gate.reason == 'command_expired' and gate.latest is None and len(writes) == before
    assert gate.status()['continuation_allowed'] and gate._continuation_deadline == deadline
    gate.tick();advance(20);gate.tick()
    assert gate.status()['continuation_ready']
    gate.accept(packet(seq=2));gate.tick()
    assert gate.applied_seq == 2
    # Corrupt JSON is a different failure and must close continuation.
    gate.lease_deadline = gate.clock()-1;gate.tick()
    executor._command(SimpleNamespace(data='{'))
    assert not gate.status()['continuation_allowed']


def test_recoverable_hold_keeps_session_without_replaying_during_long_ik_gap():
    gate,_,writes,advance,packet=rig()
    gate.accept(packet());advance(20);gate.tick()
    old=gate.session_id
    gate.hold('ik_recoverable',recoverable=True)
    assert gate.latest is None
    assert gate.accept(packet(seq=1)) is False  # Not yet physically held.
    gate.tick();advance(20);gate.tick()
    count=len(writes)
    for _ in range(20):advance(100);gate.tick()
    assert gate.status()['continuation_ready'] and len(writes)==count
    assert gate.accept(packet(seq=2));gate.tick()
    assert gate.session_id==old and gate.applied_seq==2
    gate.hold();gate.tick();advance(20);gate.tick()
    with pytest.raises(ValueError,match='hold_not_resumable'):
        gate.hold('ik_recoverable',recoverable=True)
    assert not gate.status()['continuation_allowed']


def test_recoverable_hold_cannot_override_hardware_fault():
    gate,state,writes,advance,packet=rig()
    gate.hold('ik_recoverable',recoverable=True)
    state['estop']=True;gate.tick()
    count=len(writes)
    with pytest.raises(ValueError,match='hold_not_resumable'):
        gate.hold('ik_recoverable',recoverable=True)
    with pytest.raises(ValueError,match='motion_not_armed'):gate.accept(packet())
    assert len(writes)==count


def test_unseen_pre_hold_packet_cannot_restart_same_session():
    gate,_,writes,advance,packet=rig()
    gate.accept(packet());advance(20);gate.tick()
    delayed=packet(seq=1)
    gate.hold('ik_recoverable',recoverable=True)
    gate.tick();advance(20);gate.tick()
    assert gate.status()['continuation_ready']
    count=len(writes)
    assert gate.accept(delayed) is False
    gate.tick();assert len(writes)==count and gate.state=='hold' and gate.latest is None
    assert gate.accept(packet(seq=2));gate.tick()
    assert gate.applied_seq==2
