"""Command telemetry is distinct from real joint feedback and stop evidence."""
import copy

import pytest

from test_motion_stream import rig


def test_command_state_tracks_issued_positions_and_actual_cycle_time():
    gate, feedback, writes, advance, packet = rig()
    initial = gate.status()['command_state']
    assert initial['kind'] == 'stationary_seed' and not initial['published']
    for seq, (target, ms) in enumerate(((.1, 20), (.1, 30), (-.1, 20))):
        gate.accept(packet(seq=seq, q=[target]*14))
        advance(ms)
        gate.tick()
        state = gate.status()['command_state']
        assert state['target_sequence'] == seq and state['q'] == writes[-1][0]
        assert state['dt_s'] == pytest.approx(ms/1000)
        assert state['published'] and state['limited']
        feedback['q'] = list(writes[-1][0])
    assert state['dq'][0] == pytest.approx(-.2)
    assert state['derivatives'] == 'finite_difference'
    # Feedback and telemetry have different sources even if this test's plant
    # happens to follow immediately.
    state['q'][0] = 99
    assert gate.status()['command_state']['q'][0] != 99


def test_successfully_issued_exact_target_is_not_labelled_limited():
    gate, _, writes, advance, packet = rig()
    gate.accept(packet(q=[.001]*14))
    advance(20)
    gate.tick()
    assert gate.status()['command_state']['limited'] is False
    assert writes[-1][0] == [.001]*14


def test_failed_publish_cannot_advance_command_evidence():
    gate, _, _, advance, packet = rig()
    original = copy.deepcopy(gate.status()['command_state'])
    def fail(*args):raise OSError('injected_publish_failure')
    gate.emit = fail
    gate.accept(packet())
    advance(20)
    gate.tick()
    assert gate.state == 'fault'
    assert gate.status()['command_state'] == original


def test_hold_never_fabricates_zero_velocity_or_stop_confirmation():
    gate, _, _, advance, packet = rig()
    gate.accept(packet())
    advance(20)
    gate.tick()
    gate.hold(recoverable=True)
    advance(20)
    gate.tick()
    state = gate.status()
    assert state['command_state']['kind'] == 'hold'
    assert state['command_state']['dq'] is state['command_state']['ddq'] is None
    assert not state['stop_confirmed']
    advance(20)
    gate.tick()
    assert gate.status()['hold_confirmed']


def test_long_gap_invalidates_derivative_estimates():
    gate, _, _, _, _ = rig()
    gate._record_command([.01]*14, gate.last_emit+150_000_000)
    assert gate.command_state['dq'] is gate.command_state['ddq'] is None


def test_publish_duration_is_included_in_command_sample_timestamp():
    gate, _, writes, advance, packet = rig()
    original = gate.emit
    def delayed(q, hands):
        advance(5)
        original(q, hands)
    gate.emit = delayed
    gate.accept(packet(q=[.001]*14))
    advance(20)
    gate.tick()
    state = gate.status()['command_state']
    assert state['q'] == writes[-1][0]
    assert state['sample_ns'] == gate.clock()
    assert state['dt_s'] == pytest.approx(.025)
    assert state['dq'][0] == pytest.approx(.001/.025)
