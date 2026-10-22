"""Recoverable holds with a lagging plant; no ROS or physical commands."""
import pytest
from test_motion_stream import rig


def confirmed_hold(gate, advance, reason="operator_pause"):
    gate.hold(reason)
    advance(20)
    gate.tick()
    assert not gate.status()["hold_confirmed"]
    advance(20)
    gate.tick()
    assert gate.status()["hold_confirmed"]


def test_lagging_arm_stays_active_with_bounded_short_targets():
    gate, state, writes, advance, packet = rig()
    gate.velocity = .6
    # A plant that cannot follow is deliberately NOT made equal to the target.
    # 200 ms travel remains bounded, with at most 20 ms command increments.
    previous = list(state["q"])
    for seq in range(100):
        gate.accept(packet(seq=seq, q=[.8]*14))
        advance(20)
        gate.tick()
        assert gate.state == "active"
        assert max(abs(a-b) for a,b in zip(writes[-1][0], state["q"])) <= .12 + 1e-12
        assert max(abs(a-b) for a,b in zip(writes[-1][0], previous)) <= .012 + 1e-12
        previous = list(writes[-1][0])
    assert previous == pytest.approx([.12]*14)
    assert gate.applied_seq == 99


def test_resume_rotates_session_and_rebases_on_measured_pose():
    gate, state, writes, advance, packet = rig()
    stale = packet()
    gate.accept(stale)
    advance(20)
    gate.tick()
    state["q"] = [.003]*14
    confirmed_hold(gate, advance)
    old_session, old_secret = gate.session_id, gate.secret
    lease = gate.resume()
    assert lease["session_id"] != old_session and lease["secret"] != old_secret
    assert gate.last_q == state["q"]
    assert gate.latest is None and gate.applied_seq == -1
    assert gate.state == "ready" and not gate.output_active
    count = len(writes)
    with pytest.raises(ValueError):
        gate.accept(stale)
    assert len(writes) == count


def test_command_timeout_holds_then_can_resume_without_dropping_owner():
    gate, state, writes, advance, packet = rig()
    gate.accept(packet())
    advance(20)
    gate.tick()
    advance(101)
    gate.tick()
    assert gate.reason == "command_timeout" and gate.session_id
    with pytest.raises(ValueError, match="hold_not_resumable"):
        gate.resume()
    advance(20)
    gate.tick()
    gate.resume()
    assert gate.state == "ready" and gate.session_id
    assert writes[-1][1] is None  # Hold never opens the hands.


@pytest.mark.parametrize("reason", ["invalid_command_mac", "external_motion_publishers_present"])
def test_nonrecoverable_hold_cannot_resume(reason):
    gate, _, _, advance, _ = rig()
    confirmed_hold(gate, advance, reason)
    with pytest.raises(ValueError, match="hold_not_resumable"):
        gate.resume()


def test_fault_remains_latched_even_after_feedback_recovers():
    gate, _, _, advance, _ = rig()
    advance(301, fresh=False)
    gate.tick()
    assert gate.state == "fault"
    advance(20)
    gate.tick()
    advance(20)
    gate.tick()
    with pytest.raises(ValueError, match="hold_not_resumable"):
        gate.resume()
    assert gate.session_id


def test_servo_running_blocks_claim():
    from types import SimpleNamespace
    from teleop_executor import TeleopExecutor
    executor = TeleopExecutor.__new__(TeleopExecutor)
    executor.plugins = [SimpleNamespace(PREFIX="servo", _running=True)]
    assert executor.legacy_busy()


def test_info_does_not_start_any_execution_resources():
    from teleop_executor import TeleopExecutor
    executor = TeleopExecutor.__new__(TeleopExecutor)
    executor.info = lambda: {"state": "idle"}
    executor.start = lambda: pytest.fail("info must remain read-only")
    assert executor.dispatch("info", {}) == {"state": "idle"}


def test_body_pose_change_blocks_motion_but_not_confirmed_arm_stop():
    gate, state, writes, advance, packet = rig()
    gate.accept(packet())
    advance(20); gate.tick()
    state['fixed_body'] = False
    advance(20); gate.tick()
    assert gate.state == 'fault' and gate.reason == 'calibrated_body_joints_changed'
    before = len(writes)
    gate.hold(release=True)
    advance(20); gate.tick()
    assert len(writes) == before+1 and writes[-1] == (state['q'], None)
    advance(20); gate.tick()
    assert gate.status()['stop_confirmed'] and not gate.session_id
