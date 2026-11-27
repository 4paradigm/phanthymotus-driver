import _point_test_helpers
"""Point card request/receipt compatibility and direction-hold diagnostics."""
import json
import math
import threading

import pytest

from directional_pointing import DirectionalPointing
from _point_test_helpers import Backend as LegacyBackend


class Backend(LegacyBackend):
    def __init__(self, directory):
        super().__init__(str(directory))
        self.kinematics.angular_tolerance_deg = 5.
        self.kinematics.evaluate = lambda q,goal: {'direction': goal, 'angular_error_deg': q[0]}
        self.requested = None
        self.complete = True

    def execute(self, target, hold, stop, observe, *, planned_start_q5, target_direction=None):
        self.calls.append(planned_start_q5)
        self.requested = target_direction
        observe('moving', [5.2,0.,0.,0.,0.])
        if self.error:
            raise self.error
        observe('holding', [4.3051,0.,0.,0.,0.])
        observe('returning', None)
        observe('releasing', None)
        return {'left_hold': {'max_error_rad': .00936}, 'arrival_diagnostics': {
            'criterion': 'measured_direction', 'target_direction': list(target_direction),
            'angular_error_deg': 4.3051, 'tolerance_deg': 5., 'required_hold_s': hold,
            'hold_elapsed_s': hold if self.complete else hold/2,
            'longest_hold_s': hold if self.complete else hold/2, 'hold_completed': self.complete,
            'joint_error_rad': [.071,.012,.02,.009,.1], 'max_abs_joint_error_rad': .1,
            'joint_velocity_rad_s': [0.,0.,0.,0.,.1],
            'feedback_ns': 1_234_567_890, 'feedback_age_ns': 7_477_000}}


def invoke(backend):
    notices = []
    done = threading.Event()

    def notify(ident, state, data, **kwargs):
        notices.append((ident,state,data))
        done.set()

    card = DirectionalPointing(backend, notify)
    try:
        ticket = card.dispatch('point', {'yaw_deg': 20., 'pitch_deg': 20., 'hold_s': 1.})
        assert ticket.get('state') == 'accepted', ticket
        assert done.wait(2)
        assert notices[0][0] == ticket['action_id']
        return notices[0], card.get_tool()
    finally:
        card.stop()


def test_exact_requested_direction_and_five_degree_hold_are_visible_in_receipt(tmp_path):
    backend = Backend(tmp_path)
    (ident,state,compact), tool = invoke(backend)
    angle = math.radians(20)
    expected = [math.cos(angle)**2, math.sin(angle)*math.cos(angle), math.sin(angle)]
    assert backend.requested == pytest.approx(expected)
    assert state == 'completed' and compact['angular_error_deg'] == 4.3051
    assert compact['arrival']['limit_deg'] == 5. and compact['arrival']['completed']
    assert compact['arrival']['held_s'] == compact['arrival']['required_s'] == 1.
    assert len(json.dumps(compact,ensure_ascii=True)) <= 600
    receipt = json.loads((tmp_path/(ident+'.json')).read_text(encoding='utf8'))['result']
    arrival = receipt['arrival_diagnostics']
    assert arrival['joint_error_rad'] == [.071,.012,.02,.009,.1]
    assert arrival['feedback_ns'] == 1_234_567_890
    assert '5 度' in tool['description'] and '连续保持 hold_s' in tool['description']
    assert tool['name'] == 'g1_directional_pointing'
    assert tool['inputSchema']['x-action-params']['point']['params'] == ['yaw_deg','pitch_deg','hold_s']


def test_incomplete_continuous_hold_cannot_become_completed(tmp_path):
    backend = Backend(tmp_path)
    backend.complete = False
    (ident,state,compact), _ = invoke(backend)
    assert state == 'error' and compact['reason'] == 'directional_hold_not_completed'
    assert compact['arrival']['completed'] is False
    receipt = json.loads((tmp_path/(ident+'.json')).read_text(encoding='utf8'))['result']
    assert receipt['arrival_diagnostics']['hold_elapsed_s'] == .5


def test_legacy_execute_without_direction_keyword_remains_compatible(tmp_path):
    backend = LegacyBackend(str(tmp_path))
    (_,state,_), _ = invoke(backend)
    assert state == 'completed' and len(backend.calls) == 1


def test_execute_type_error_is_not_retried_as_a_legacy_signature(tmp_path):
    backend = Backend(tmp_path)
    backend.error = TypeError('execute-internal-type-error')
    (_,state,compact), _ = invoke(backend)
    assert state == 'error' and compact['reason'] == 'execute-internal-type-error'
    assert len(backend.calls) == 1


def test_arrival_diagnostics_survive_failure_and_canvas_size_limit(tmp_path):
    backend = Backend(tmp_path)
    failure = RuntimeError('directional_target_not_reached')
    failure.arrival_diagnostics = {
        'tolerance_deg': 5., 'required_hold_s': 1., 'hold_elapsed_s': 0.,
        'longest_hold_s': .6, 'hold_completed': False, 'angular_error_deg': 5.2,
        'joint_error_rad': [.08]*5, 'max_abs_joint_error_rad': .08,
        'feedback_ns': 999_999_999, 'feedback_age_ns': 18_000_000}
    backend.error = failure
    (ident,state,compact), _ = invoke(backend)
    assert state == 'error' and compact['arrival']['limit_deg'] == 5.
    assert compact['arrival']['completed'] is False
    assert len(json.dumps(compact,ensure_ascii=True)) <= 600
    receipt = json.loads((tmp_path/(ident+'.json')).read_text(encoding='utf8'))['result']
    assert receipt['arrival_diagnostics'] == failure.arrival_diagnostics


@pytest.mark.parametrize('reason', [
    'directional_release_unconfirmed:release_timeout; cause=directional_guard:motion_envelope_joint_limit',
    'directional_observer:' + '故障'*70])
def test_compact_combined_hard_limit_failure_keeps_identity_and_arrival_contract(reason):
    source = {'state': 'error', 'frame': 'torso_link', 'actual_direction': [.8,.3,.3],
              'angular_error_deg': 5.123456789, 'sample_phase': 'moving',
              'reason': reason,
              'execution_mode': 'bounded_ik', 'external_accuracy': 'pending',
              'actual_yaw_deg': 20.12345678, 'actual_pitch_deg': 14.56789123,
              'left_hold': {'max_error_rad': .00987654321},
              'startup_failure': {'phase': 'first_reference'},
              'release_failure': {'state': 'unknown', 'timed_out_state': 'confirming_handback'},
              'envelope_failure': {'predicate': 'measured_joint_limit', 'motor_id': 23,
                                   'failing_value_rad': -.00312345, 'lower_rad': -.002, 'upper_rad': .002},
              'arrival_diagnostics': {'tolerance_deg': 5., 'required_hold_s': 5.,
                                      'hold_elapsed_s': 0., 'longest_hold_s': 4.987654,
                                      'hold_completed': False, 'max_abs_joint_error_rad': .01234567,
                                      'feedback_age_ns': 199_000_000}}
    compact = DirectionalPointing._acp_summary(source)
    assert len(json.dumps(compact,ensure_ascii=True)) <= 600
    assert compact['arrival']['limit_deg'] == 5. and compact['arrival']['required_s'] == 5.
    assert compact['arrival']['completed'] is False
    assert compact['envelope_failure']['motor_id'] == 23
    assert compact['failure_phase'] == 'first_reference'
    assert compact['release_phase'] == 'confirming_handback'
    assert compact['reason'].startswith(reason.split(':')[0]+':')
    assert source['arrival_diagnostics']['longest_hold_s'] == 4.987654
