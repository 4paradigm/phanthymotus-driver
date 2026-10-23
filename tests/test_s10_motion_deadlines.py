"""Velocity deadline races using a fictional clock and backend only."""
import threading
import time

import pytest

from deep_robotics.s10_pro.motion import MotionPlugin


class Clock:
    def __init__(self):
        self._now = 0.
        self._lock = threading.Lock()

    def __call__(self):
        with self._lock:
            return self._now

    def advance_to(self, value):
        with self._lock:
            self._now = max(self._now, value)


class Backend:
    def __init__(self, clock, outcome):
        self.clock, self.outcome = clock, outcome
        self.sends, self.stops = [], []

    def contract(self):
        return {'verified': True, 'evidence': {key: 'ISOLATED DEADLINE TEST FICTION' for key in
            ('identity', 'firmware', 'authority', 'feedback', 'stop', 'watchdog')},
            'robot_identity_matched': True, 'firmware_matched': True, 'robot_deadman_verified': True,
            'robot_deadman_s': .3, 'send_timeout_s': .05,
            'velocity_units': {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'},
            'actions': ['stand', 'lie', 'velocity', 'stopmotion'],
            'stop_semantics': 'zero_velocity_until_feedback',
            # Fictional permission to stop after authority loss lets the test
            # distinguish a successful stop from successful motion completion.
            'stop_on_authority_loss': True}

    def control_status(self):
        owned = not (self.outcome == 'authority_lost' and self.clock() >= .1)
        if not owned:
            # Ownership becomes unavailable during an observation that crosses
            # the original deadline. It must remain an interrupted outcome.
            self.clock.advance_to(.2)
        return {'owned': owned, 'fresh': True, 'feedback_fresh': True, 'age_s': 0.}

    def send(self, action, request, cancel):
        assert not cancel.is_set()
        self.sends.append(action)
        if len(self.sends) == 2:
            # The last refresh returns after duration=.15. Both accepted and
            # rejected responses exercise the same deadline crossing.
            self.clock.advance_to(.2)
            if self.outcome == 'rejected':
                return {'accepted': False}
        return {'accepted': None}

    def stop(self, reason, cancel):
        assert not cancel.is_set()
        self.stops.append(reason)
        return {'accepted': None}


class Recorder:
    def __init__(self):
        self.payloads = []

    def submit(self, payload):
        self.payloads.append(payload)

    def status(self, action_id):
        return {'acknowledged': False, 'delivery_state': 'pending', 'action_id': action_id}


@pytest.mark.parametrize('outcome,expected_status,expected_reason', [
    ('rejected', 'error', 'rejected'),
    ('authority_lost', 'error', 'authority_unavailable'),
    ('natural', 'completed', 'duration_elapsed'),
])
def test_crossing_duration_never_relabels_interruption_as_success(outcome, expected_status, expected_reason):
    clock = Clock()
    backend = Backend(clock, outcome)
    recorder = Recorder()

    def feedback(action, request_id):
        if action == 'velocity':
            clock.advance_to(.1)
        return {'confirmed': bool(backend.stops) if action == 'stopmotion' else True,
                'fresh': True, 'age_s': 0., 'received_monotonic': clock(),
                'request_id': request_id}

    plugin = MotionPlugin({'enabled': True, 'max_duration_s': .3}, 'isolated', backend,
        feedback=feedback, execution_authorized=True, notifier=recorder,
        clock=clock, autostart_worker=False)
    try:
        plugin.start()
        receipt = plugin.execute('velocity', {'request_id': 'deadline-' + outcome, 'vx': .1, 'duration': .15})
        assert receipt['accepted'] and not receipt['completed']
        deadline = time.monotonic() + 1.
        while plugin.status()['last_result'] is None and time.monotonic() < deadline:
            time.sleep(.005)
        result = plugin.status()['last_result']
        assert result is not None, 'The isolated fake clock must not leave a stop loop waiting.'
        plugin._worker.join(.5)
        assert not plugin._worker.is_alive()
        assert result['status'] == expected_status
        assert result['reason'] == expected_reason
        assert result['completed'] is (expected_status == 'completed')
        assert result['stop_confirmed'] is True
        assert len(recorder.payloads) == 1
        assert recorder.payloads[0]['action_id'] == receipt['action_id']
        assert recorder.payloads[0]['status'] == expected_status
        if outcome == 'rejected':
            assert result['command_accepted'] is False
            assert len(backend.sends) == 2
        if outcome == 'authority_lost':
            assert len(backend.sends) == 1
    finally:
        plugin.close()
