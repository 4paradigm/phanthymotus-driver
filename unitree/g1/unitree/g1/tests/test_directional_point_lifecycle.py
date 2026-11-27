"""Public angle/action contract with synthetic feedback; no motor commands."""
import math
import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from directional_kinematics import DirectionalKinematics
from directional_pointing import DirectionalPointing


class SimulatedBackend:
    def __init__(self):
        self.entered = threading.Event()
        self.proceed = threading.Event()
        self.feedback = [0, 0, 0, 0.01, 0]
        self.release_error = None
        self.ready_error = None
        self.checked = []
        self.stop_event = None
        self.target = None
        self.hold = None

    def check_ready(self, target, hold):
        self.checked.append((list(target), hold))
        if self.ready_error:
            raise RuntimeError(self.ready_error)

    def execute(self, target, hold, stop, observe):
        self.target = list(target)
        self.hold = hold
        self.stop_event = stop
        self.entered.set()
        # Cancellation is requested immediately, but the simulated handback
        # completes only when the test releases this gate.
        if not self.proceed.wait(2):
            raise RuntimeError('test backend timeout')
        if self.release_error:
            raise RuntimeError(self.release_error)
        if stop.is_set():
            raise RuntimeError('cancelled')
        if self.feedback is not None:
            observe('holding', self.feedback)


class RecordingModel:
    """Capture conversion before deliberately refusing to execute a pose."""
    angular_tolerance_deg = 3.0

    def __init__(self):
        self.requests = []

    def solve(self, direction):
        self.requests.append(list(direction))
        raise ValueError('synthetic solver refuses execution')


class PointingActions(unittest.TestCase):
    def setUp(self):
        model = DirectionalKinematics({})
        # This pose exists only in this test's simulated calibration catalogue.
        q = [0.0] * 5
        self.target = model.direction(q)
        self.angles = {
            'yaw_deg': math.degrees(math.atan2(self.target[1], self.target[0])),
            'pitch_deg': math.degrees(math.atan2(self.target[2],
                                                math.hypot(*self.target[:2]))),
        }
        cfg = {'verified_poses': [{'q_rad': q, 'name': 'fixture', 'verified': True,
                                  'validation_id': 'simulation', 'model_sha256': model.model_sha256}]}
        self.backend = SimulatedBackend()
        self.backend.kinematics = DirectionalKinematics(cfg)
        self.done = threading.Event()
        self.notifications = []

        def notify(*args, **kwargs):
            self.notifications.append((args, kwargs))
            self.done.set()

        self.card = DirectionalPointing(self.backend, notify)
        self.addCleanup(self.card.stop)
        self.addCleanup(self.backend.proceed.set)

    def begin(self, **extra):
        ticket = self.card.dispatch('point', {**self.angles, **extra})
        self.assertEqual(ticket.get('state'), 'accepted', ticket)
        self.assertTrue(self.backend.entered.wait(2))
        return {'action_id': ticket['action_id']}

    def terminal(self, ticket):
        self.assertTrue(self.done.wait(2))
        self.assertEqual(len(self.notifications), 1)
        args, kwargs = self.notifications[0]
        self.assertEqual(args[0], ticket['action_id'])
        output = args[2]
        self.assertEqual(args[1], output['state'])
        self.assertEqual(kwargs.get('tool'), self.card.PREFIX)
        for field in ('actual_direction', 'angular_error_deg', 'reason'):
            self.assertIn(field, output)
        if output['state'] == 'completed':
            self.assertIsNone(output['reason'])
        else:
            self.assertIsInstance(output['reason'], str)
            self.assertTrue(output['reason'])
        return output

    def finish(self, ticket):
        self.backend.proceed.set()
        return self.terminal(ticket)

    def request_stop_while_backend_is_busy(self, ticket):
        returned = threading.Event()
        results = []

        def stop():
            try:
                results.append(self.card.dispatch('stop', ticket))
            finally:
                returned.set()

        thread = threading.Thread(target=stop, daemon=True)
        thread.start()
        self.addCleanup(thread.join, 2)
        self.addCleanup(self.backend.proceed.set)
        self.assertTrue(self.backend.stop_event.wait(2))
        self.assertFalse(returned.wait(0.05), 'stop returned before backend handback finished')
        self.backend.proceed.set()
        self.assertTrue(returned.wait(2))
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(results), 1)
        return results[0]

    def test_public_schema_exposes_angles_and_scoped_stop(self):
        tool = self.card.get_tool()
        schema = tool['inputSchema']
        props = schema['properties']
        actions = props['action']['enum']
        self.assertIn('point', actions)
        self.assertIn('stop', actions)
        for removed in ('preview', 'action_status', 'cancel'):
            self.assertNotIn(removed, actions)
        self.assertNotIn('direction', props)
        self.assertEqual((props['yaw_deg']['minimum'], props['yaw_deg']['maximum']), (-180, 180))
        self.assertEqual((props['pitch_deg']['minimum'], props['pitch_deg']['maximum']), (-90, 90))
        self.assertEqual(props['hold_s']['default'], 1.0)
        self.assertEqual(schema['x-action-params']['point']['params'],
                         ['yaw_deg', 'pitch_deg', 'hold_s'])
        self.assertEqual(schema['x-action-params']['point']['required'], ['yaw_deg', 'pitch_deg'])
        self.assertEqual(schema['x-action-params']['stop']['params'], ['action_id'])
        self.assertEqual(schema['x-action-params']['stop']['required'], ['action_id'])
        expected_hook = {'action': 'stop', 'params': {'_interrupt_active': True}}
        self.assertEqual(schema['x-hooks']['on_interrupt_motion'], expected_hook)
        self.assertEqual(schema['x-hooks']['on_interrupt_all'], expected_hook)

    def test_bounded_card_accepts_different_angles_and_keeps_both_driver_receipts(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg = {'solver_mode': 'bounded_ik', 'verified_poses': [],
                   'verified_start_q_rad': [.29, -.22, .02, .976, -.034],
                   'ik_reference_q_rad': [-.268, -.096, .005, .041, 1.50],
                   'receipt_directory': directory}
            class PlannedBackend(SimulatedBackend):
                def plan(self, direction, hold):
                    left = [.2875854671,.2309480011,-.0180961806,.9727954865,.0932013169]
                    start = cfg['verified_start_q_rad']
                    target = self.kinematics.solve(direction,start_q5=start,left_q5=left)
                    return {'start_q_rad':list(start),'target_q_rad':target,'planned_direction':direction}

                def execute(self, target, hold, stop, observe, *, planned_start_q5, target_direction=None):
                    return super().execute(target,hold,stop,observe)

            backend = PlannedBackend()
            backend.config = cfg
            backend.kinematics = DirectionalKinematics(cfg)
            terminal = threading.Event()
            callbacks = []

            def notify(ident, status, data, **kwargs):
                callbacks.append((ident, status, data))
                terminal.set()

            card = DirectionalPointing(backend, notify)
            self.addCleanup(card.stop)
            self.addCleanup(backend.proceed.set)
            props = card.get_tool()['inputSchema']['properties']
            self.assertNotIn('operator_code', props)
            self.assertEqual((props['yaw_deg']['minimum'], props['yaw_deg']['maximum']), (-180, 180))
            self.assertEqual((props['pitch_deg']['minimum'], props['pitch_deg']['maximum']), (-90, 90))
            for yaw, pitch in ((20, 20), (90, 0)):
                terminal.clear()
                backend.entered.clear()
                backend.proceed.clear()
                result = card.dispatch('point', {'yaw_deg': yaw, 'pitch_deg': pitch, 'hold_s': 1})
                self.assertEqual(result.get('state'), 'accepted', result)
                self.assertTrue(backend.entered.wait(2))
                backend.feedback = backend.target
                backend.proceed.set()
                self.assertTrue(terminal.wait(2))
                ident, state, data = callbacks[-1]
                self.assertEqual((ident, state), (result['action_id'], 'completed'))
                self.assertAlmostEqual(data['actual_yaw_deg'], yaw, places=2)
                self.assertAlmostEqual(data['actual_pitch_deg'], pitch, places=2)
                stored = json.loads((Path(directory) / (ident + '.json')).read_text(encoding='utf-8'))
                self.assertEqual((stored['action_id'],stored['status'],stored['source']),
                                 (ident,state,'driver_worker'))
                self.assertEqual(stored['result']['state'],'completed')
                self.assertLess(stored['result']['angular_error_deg'],.01)
                self.assertAlmostEqual(stored['result']['actual_yaw_deg'],data['actual_yaw_deg'],places=3)
            self.assertNotEqual(callbacks[0][0], callbacks[1][0])

    def test_angle_conversion_signs_degrees_and_inclusive_boundaries(self):
        model = RecordingModel()
        backend = SimulatedBackend()
        backend.kinematics = model
        card = DirectionalPointing(backend)
        self.addCleanup(card.stop)
        for yaw, pitch, expected in (
            (0, 0, [1, 0, 0]), (90, 0, [0, 1, 0]), (-90, 0, [0, -1, 0]),
            (180, 0, [-1, 0, 0]), (-180, 0, [-1, 0, 0]),
            (0, 90, [0, 0, 1]), (0, -90, [0, 0, -1]),
            (45, 30, [math.sqrt(6) / 4, math.sqrt(6) / 4, 0.5]),
        ):
            with self.subTest(yaw=yaw, pitch=pitch):
                count = len(model.requests)
                result = card.dispatch('point', {'yaw_deg': yaw, 'pitch_deg': pitch})
                self.assertIn('synthetic solver', result['error'])
                self.assertEqual(len(model.requests), count + 1)
                for actual, wanted in zip(model.requests[-1], expected):
                    self.assertAlmostEqual(actual, wanted, places=12)
        self.assertFalse(backend.entered.is_set())

    def test_invalid_angles_are_rejected_before_solving_or_claiming(self):
        model = RecordingModel()
        backend = SimulatedBackend()
        backend.kinematics = model
        card = DirectionalPointing(backend)
        self.addCleanup(card.stop)
        for field, invalid in (
            ('yaw_deg', (-180.01, 180.01, True, None, '0', float('nan'), float('inf'))),
            ('pitch_deg', (-90.01, 90.01, False, None, '0', float('nan'), float('-inf'))),
        ):
            for value in invalid:
                with self.subTest(field=field, value=value):
                    args = {'yaw_deg': 0, 'pitch_deg': 0, field: value}
                    self.assertIn('error', card.dispatch('point', args))
        for args in ({}, {'yaw_deg': 0}, {'pitch_deg': 0}, {'direction': [1, 0, 0]}):
            self.assertIn('error', card.dispatch('point', args))
        self.assertEqual(model.requests, [])
        self.assertFalse(backend.entered.is_set())

    def test_removed_actions_and_invalid_hold_never_claim(self):
        for action in ('preview', 'action_status', 'cancel'):
            self.assertIn('error', self.card.dispatch(action, self.angles))
        for hold in (False, 0, 10, float('nan'), float('inf'), '1'):
            self.assertIn('error', self.card.dispatch('point', {**self.angles, 'hold_s': hold}))
        self.assertIn('error', self.card.dispatch('point', {'yaw_deg': 180, 'pitch_deg': 0}))
        self.assertFalse(self.backend.entered.is_set())

    def test_default_hold_and_feedback_based_acp_completion(self):
        ticket = self.begin()
        self.assertEqual(self.backend.hold, 1.0)
        duplicate = self.card.dispatch('point', self.angles)
        self.assertIn('error', duplicate)
        self.assertNotIn('action_id', duplicate)
        self.assertEqual(self.backend.checked[0], ([0.0] * 5, 1.0))
        self.assertEqual(self.notifications, [])
        output = self.finish(ticket)
        self.assertEqual(output['state'], 'completed')
        self.assertGreater(output['angular_error_deg'], 0)
        self.assertEqual(output['actual_direction'], self.card.model.direction(self.backend.feedback))
        first = self.card.dispatch('stop', ticket)
        self.assertEqual(first['state'], 'completed')
        self.assertEqual(self.card.dispatch('stop', ticket), first)

    def test_explicit_hold_is_passed_to_backend(self):
        self.assertEqual(self.finish(self.begin(hold_s=0.5))['state'], 'completed')
        self.assertEqual(self.backend.hold, 0.5)

    def test_readiness_failure_has_no_action_id_or_pending_completion(self):
        self.backend.ready_error = 'synthetic stale feedback'
        result = self.card.dispatch('point', self.angles)
        self.assertIn('synthetic stale feedback', result['error'])
        self.assertNotIn('action_id', result)
        self.assertFalse(self.backend.entered.is_set())
        self.assertEqual(self.notifications, [])

    def test_invalid_angle_failure_has_no_action_id(self):
        result = self.card.dispatch('point', {'yaw_deg': 181, 'pitch_deg': 0})
        self.assertIn('error', result)
        self.assertNotIn('action_id', result)
        self.assertEqual(self.backend.checked, [])
        self.assertEqual(self.notifications, [])

    def test_measured_error_fails_with_terminal_evidence(self):
        self.backend.feedback = [0, 0, 0, 0.25, 0]
        output = self.finish(self.begin())
        self.assertEqual(output['state'], 'error')
        self.assertGreater(output['angular_error_deg'], 3)
        self.assertIsNotNone(output['actual_direction'])

    def test_absent_feedback_never_completes(self):
        self.backend.feedback = None
        output = self.finish(self.begin())
        self.assertEqual(output['state'], 'error')
        self.assertIsNone(output['actual_direction'])
        self.assertIsNone(output['angular_error_deg'])

    def test_stop_waits_for_handback_is_scoped_and_idempotent(self):
        ticket = self.begin()
        self.assertIn('error', self.card.dispatch('stop', {'action_id': 'wrong'}))
        self.assertFalse(self.backend.stop_event.is_set())
        output = self.request_stop_while_backend_is_busy(ticket)
        self.assertEqual(output['state'], 'cancelled')
        terminal = self.terminal(ticket)
        for field in ('state', 'actual_direction', 'angular_error_deg', 'reason'):
            self.assertEqual(output[field], terminal[field])
        self.assertEqual(self.card.dispatch('stop', ticket), output)
        self.assertEqual(len(self.notifications), 1)
        # Scoped stop leaves the card enabled for the next independent action.
        self.done.clear()
        self.notifications.clear()
        self.backend.entered.clear()
        self.backend.proceed.clear()
        self.assertEqual(self.finish(self.begin())['state'], 'completed')

    def test_failed_handback_is_returned_by_stop_and_acp(self):
        self.backend.release_error = 'handback unconfirmed'
        ticket = self.begin()
        stopped = self.request_stop_while_backend_is_busy(ticket)
        self.assertEqual(stopped['state'], 'error')
        self.assertIn('handback unconfirmed', stopped['reason'])
        self.assertEqual(self.terminal(ticket)['state'], 'error')
        self.assertEqual(self.card.dispatch('stop', ticket), stopped)

    def test_stop_timeout_does_not_claim_terminal_success_and_can_be_retried(self):
        ticket = self.begin()
        # Simulate expiration of only the completion wait, without a 15-second
        # test delay or changing the independently running backend worker.
        with patch.object(self.card._active['done'], 'wait', return_value=False):
            stopped = self.card.dispatch('stop', ticket)
        self.assertEqual(stopped['state'], 'stopping')
        self.assertEqual(stopped['error'], 'stop_not_yet_confirmed')
        self.assertTrue(self.backend.stop_event.is_set())
        self.assertEqual(self.notifications, [])
        self.assertEqual(self.finish(ticket)['state'], 'cancelled')
        self.assertEqual(self.card.dispatch('stop', ticket)['state'], 'cancelled')

    def test_malformed_action_id_cannot_turn_into_lifecycle_stop(self):
        ticket = self.begin()
        for ident in ('', None, False, 123, []):
            with self.subTest(action_id=ident):
                self.assertIn('error', self.card.dispatch('stop', {'action_id': ident}))
                self.assertFalse(self.backend.stop_event.is_set())
        self.assertEqual(self.finish(ticket)['state'], 'completed')

    def test_framework_interrupt_requests_stop_and_keeps_card_enabled(self):
        ticket = self.begin()
        result = self.card.dispatch('stop', {'_interrupt_active': True})
        self.assertEqual(result['state'], 'stopping')
        self.assertTrue(self.backend.stop_event.is_set())
        self.assertFalse(self.done.is_set())
        self.assertEqual(self.finish(ticket)['state'], 'cancelled')
        self.done.clear()
        self.notifications.clear()
        self.backend.entered.clear()
        self.backend.proceed.clear()
        self.assertEqual(self.finish(self.begin())['state'], 'completed')

    def test_project_stop_exposes_stopping_until_backend_finishes(self):
        ticket = self.begin()
        result = self.card.dispatch('stop', {})
        self.assertEqual(result['state'], 'stopping')
        self.assertTrue(self.backend.stop_event.is_set())
        self.assertEqual(self.card.dispatch('info', {}), {'state': 'stopping'})
        self.assertIn('error', self.card.dispatch('point', self.angles))
        self.assertIn('error', self.card.dispatch('start', {}))
        self.assertEqual(self.finish(ticket)['state'], 'cancelled')
        self.assertEqual(self.card.dispatch('info', {}), {'state': 'idle'})
        self.assertIn('error', self.card.dispatch('point', self.angles))

    def test_project_stop_failure_remains_visible_to_framework(self):
        self.backend.release_error = 'handback unconfirmed'
        ticket = self.begin()
        self.assertEqual(self.card.dispatch('stop', {})['state'], 'stopping')
        self.assertEqual(self.finish(ticket)['state'], 'error')
        info = self.card.dispatch('info', {})
        self.assertEqual(info['state'], 'error')
        self.assertIn('handback unconfirmed', info['error'])

    def test_framework_info_stop_and_start_remain_available(self):
        info = self.card.dispatch('info', {})
        self.assertNotIn('error', info)
        self.assertEqual(set(info), {'state'})
        self.card.dispatch('stop', {})
        self.assertIn('error', self.card.dispatch('point', self.angles))
        self.assertNotIn('error', self.card.dispatch('start', {}))
        self.assertEqual(self.finish(self.begin())['state'], 'completed')


if __name__ == '__main__':
    unittest.main()
