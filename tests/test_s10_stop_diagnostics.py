"""Classify missing stop evidence without changing stop acceptance rules."""
import threading
from types import SimpleNamespace

import pytest
from deep_robotics.s10_pro import canvas_posture as module
from test_s10_loco_move import loco, call, terminal


def check(**updates):
    return {'ready': False, 'feedback_fresh': True, 'motion_state': 17,
            'matching_samples': 1, 'required_samples': 3,
            'stable_duration_s': .1, 'required_duration_s': .3,
            'linear_tolerance_mps': .05, 'yaw_tolerance_radps': .06,
            'reported_velocity': {'vx': 0., 'vy': 0., 'yaw_rate': 0.}, **updates}


@pytest.mark.parametrize('sample,expected', [
    (check(feedback_fresh=False), 'stop_feedback_stale_or_missing'),
    (check(motion_state=1), 'stop_motion_state_mismatch'),
    (check(reported_velocity={'vx': .06, 'vy': 0., 'yaw_rate': 0.}),
     'reported_velocity_above_stop_tolerance'),
    (check(), 'insufficient_stable_feedback')])
def test_stop_failure_retains_specific_evidence(monkeypatch, sample, expected):
    now = [10.]
    p = object.__new__(module.CanvasPosturePlugin)
    p._clock = lambda: now[0]
    p._lock = threading.RLock()
    p._active = None
    count = [0]
    def feedback():
        count[0] += 1
        return {'stopped': False, 'sequence': count[0], 'stationary_check': sample}
    p._backend = SimpleNamespace(stop_distance=lambda *a: {'sent': True}, distance_stopped=feedback)
    monkeypatch.setattr(module.time, 'sleep', lambda seconds: now.__setitem__(0, now[0] + seconds))
    assert p._stop_fixed_distance('move_2s', 'test', 15.) == (False, True)
    d = p._stop_diagnostics
    assert d['failure_kind'] == expected
    assert d['last_stationary_check'] == sample
    assert d['available_budget_s'] == pytest.approx(1.2)
    assert d['zero_commands_sent'] >= 2
    assert len(d['recent_distinct_feedback']) <= 16


def test_send_failure_is_not_reported_as_stop_success(monkeypatch):
    p = object.__new__(module.CanvasPosturePlugin)
    p._clock = lambda: 10.
    p._lock = threading.RLock()
    p._active = None
    def send(*args):
        raise TimeoutError('private implementation detail')
    p._backend = SimpleNamespace(stop_distance=send)
    assert p._stop_fixed_distance('move_2s', 'test', 15.) == (False, False)
    assert p._stop_diagnostics['failure_kind'] == 'zero_command_send_failed'
    assert p._stop_diagnostics['error_type'] == 'TimeoutError'
    assert 'private implementation detail' not in str(p._stop_diagnostics)


def test_healthy_explicit_stand_down_can_recover_failed_channel(loco):
    p, backend, bridge, _, transports = loco
    bridge.state = 17
    backend.abort()
    assert backend.safety_status()['failed_closed']
    assert call(p, 'stand_down', request_id='down-after-protection')['accepted']
    assert terminal(p, 'down-after-protection')['completed']
    assert [state for t in transports for state, _ in t.states] == [4]
    assert not any(t.velocity_commands for t in transports)


def test_fault_still_blocks_stand_down_recovery(loco):
    p, backend, bridge, _, transports = loco
    backend.abort()
    bridge.faults = [{'code': 33025}]
    result = call(p, 'stand_down', request_id='down-with-fault')
    assert not result['accepted']
    assert 'fresh_clear_fault_snapshot_required' in result['blocked_by']
    assert not any(t.states or t.velocity_commands for t in transports)
