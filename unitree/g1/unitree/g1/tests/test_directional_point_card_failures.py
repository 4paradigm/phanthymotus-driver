import _point_test_helpers
"""Startup error evidence belongs to the originating point action."""
import json
import threading

from directional_pointing import DirectionalPointing
from _point_test_helpers import Backend


def test_failed_start_receipt_keeps_own_executor_and_release_evidence(tmp_path):
    backend = Backend(str(tmp_path))
    failure = RuntimeError('directional_release_unconfirmed:release_timeout; cause=arm_feedback_stale')
    failure.startup_failure = {'phase': 'validate_claimed_path', 'same_session': True,
                               'state': 'hold', 'reason': 'arm_feedback_stale',
                               'sequence': -1, 'applied_sequence': -1}
    failure.release_failure = {'operation_id': 'this-action-release', 'state': 'unknown',
                               'timed_out_state': 'confirming_handback',
                               'zero_writes': 5, 'feedback': {'dq': [.025] + [0.] * 9}}
    backend.error = failure
    notices = []
    done = threading.Event()

    def notify(ident, state, data, **kwargs):
        notices.append((ident, state, data))
        done.set()

    card = DirectionalPointing(backend, notify)
    try:
        ticket = card.dispatch('point', {'yaw_deg': 20, 'pitch_deg': 20, 'hold_s': 1})
        assert ticket['state'] == 'accepted'
        assert done.wait(2)
        ident, state, compact = notices[0]
        assert (ident, state) == (ticket['action_id'], 'error')
        assert compact['failure_phase'] == 'validate_claimed_path'
        assert compact['release_phase'] == 'confirming_handback'
        assert 'cause=arm_feedback_stale' in compact['reason']
        assert len(json.dumps(compact)) <= 600
        receipt = json.loads((tmp_path / (ident + '.json')).read_text())
        assert receipt['result']['startup_failure'] == failure.startup_failure
        assert receipt['result']['release_failure'] == failure.release_failure
    finally:
        card.stop()


def test_compact_failure_keeps_phase_when_direction_fields_overflow():
    source = {'state': 'error', 'frame': 'torso_link', 'actual_direction': [.8, .3, .3],
              'angular_error_deg': 97.123456789, 'sample_phase': 'taking_over',
              'reason': 'directional_release_unconfirmed:release_timeout; cause=directional_guard:directional_tracking_error',
              'execution_mode': 'bounded_ik', 'external_accuracy': 'pending',
              'actual_yaw_deg': 35.6789123456, 'actual_pitch_deg': -69.567891234,
              'left_hold': {'max_error_rad': .00899234},
              'envelope_failure': {'predicate': 'measured_tracking_error', 'motor_id': 26,
                                   'failing_value_rad': -.004329875, 'lower_rad': -.002,
                                   'upper_rad': .002},
              'startup_failure': {'phase': 'submit_first_hold'},
              'release_failure': {'state': 'unknown', 'timed_out_state': 'confirming_handback'}}
    compact = DirectionalPointing._acp_summary(source)
    assert len(json.dumps(compact)) <= 600
    assert compact['failure_phase'] == 'submit_first_hold'
    assert compact['release_phase'] == 'confirming_handback'
    assert compact['envelope_failure']['motor_id'] == 26
    assert source['actual_direction'] == [.8, .3, .3]
