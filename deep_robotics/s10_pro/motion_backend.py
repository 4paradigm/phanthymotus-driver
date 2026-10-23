"""Documented high-level DDS action sequencing with passive production defaults.

This module never grants control ownership from a graph or feedback topic. A
trusted lease provider and machine-specific contract must be injected separately.
The platform process uses an opt-in child transport, never its own vendor DDS.
"""
from __future__ import annotations

import copy
import math
import threading
import time

try:
    from .motion import REQUIRED_EVIDENCE, validate_contract
    from .motion_feedback import MotionFeedback
except ImportError:
    from motion import REQUIRED_EVIDENCE, validate_contract
    from motion_feedback import MotionFeedback


class DDSMotionBackend:
    def __init__(self, bridge, transport=None, *, contract=None, authority_provider=None,
                 execution_authorized=False, clock=time.monotonic):
        if type(execution_authorized) is not bool:
            raise ValueError('execution_authorized must be boolean')
        self._bridge, self._transport, self._clock = bridge, transport, clock
        self._policy = copy.deepcopy(contract or {})
        self._authority = authority_provider
        self._authorized = execution_authorized
        self._lock = threading.RLock()
        self._observing = self._closed = False
        self._request = None
        self._generation = 0
        self._stage = None
        self.feedback = MotionFeedback(lambda: bridge.snapshot('motion_info'), clock=clock)

    def contract(self):
        if not self._policy:
            return {'verified': False, 'transport': 'ros2_dds', 'tls_required': False,
                'missing': list(REQUIRED_EVIDENCE),
                'protocol_reference': 'S10 software guide V1.0.2 §2.2/2.3'}
        return copy.deepcopy(self._policy)

    def start_observation(self):
        with self._lock:
            if self._closed:
                raise RuntimeError('DDS motion backend is closed')
            if not self._observing:
                self._bridge.acquire('motion_observer', {'motion_info'}, {'refresh_hz': 10., 'stale_after_s': .5})
                self._observing = True

    def stop_observation(self):
        with self._lock:
            if self._observing:
                self._bridge.release('motion_observer')
                self._observing = False

    def diagnostics(self):
        result = self.feedback.preflight()
        result.update(transport='ros2_dds', tls_required=False,
            protocol_implemented='S10 software guide V1.0.2 §2.2/2.3',
            production_backend='dds_sequencer_locked', observation_active=self._observing,
            command_worker_running=self._transport_running(),
            command_worker_required=True, authority_provider_bound=callable(self._authority),
            backend_execution_authorized=self._authorized,
            transport_execution_enabled=self._transport_status().get('execution_enabled'),
            control_ownership_inferred=False,
            explanation='High-level ROS 2 DDS is implemented without TLS. Control ownership, navigation mode and loss-of-command stopping still require verification.')
        return result

    def _transport_status(self):
        try:
            value = self._transport.status() if self._transport is not None else {}
            return value if isinstance(value, dict) else {}
        except Exception:
            return {}

    def _transport_running(self):
        return self._transport_status().get('worker_running') is True

    def _authority_snapshot(self):
        try:
            value = self._authority() if self._authority is not None else {}
            return copy.deepcopy(value) if isinstance(value, dict) else {}
        except Exception:
            return {}

    def control_status(self):
        lease = self._authority_snapshot()
        observed = self.feedback.observation()
        now, age, received = self._clock(), lease.get('age_s'), lease.get('received_monotonic')
        finite = lambda value: type(value) in (int, float) and math.isfinite(value)
        valid = (lease.get('owned') is True and lease.get('fresh') is True
            and finite(age) and 0 <= age <= .5 and finite(received)
            and 0 <= now - received <= .5 and abs((now - received) - age) <= .1)
        if 'expires_monotonic' in lease:
            expiry = lease['expires_monotonic']
            valid = valid and finite(expiry) and now < expiry
        transport = self._transport_status()
        return {'owned': lease.get('owned') is True, 'fresh': lease.get('fresh') is True,
            'age_s': lease.get('age_s'), 'feedback_fresh': observed['fresh'],
            'navigation_mode_confirmed': lease.get('navigation_mode_confirmed') is True,
            'authority_provider_bound': callable(self._authority), 'authority_valid': bool(valid),
            'feedback_age_s': observed['age_s'], 'feedback_reason': observed['reason'],
            'command_worker_required': True, 'command_worker_running': transport.get('worker_running') is True,
            'backend_execution_authorized': self._authorized,
            'transport_execution_enabled': transport.get('execution_enabled')}

    def readiness_status(self):
        """Read current observations without acquiring authority or starting I/O.

        The injected authority provider must expose its already received status
        snapshot. The production default remains unbound, with no inference from
        motion feedback, graph discovery or a newly generated timestamp.
        """
        return self.control_status()

    def _permit(self, request_id, *, navigation=False, action_deadline=None):
        if self._closed or not self._observing or not self._authorized or not validate_contract(self.contract()):
            raise PermissionError('DDS production execution is not authorized')
        if not self._transport_running():
            raise RuntimeError('DDS command worker has not explicitly started')
        lease = self._authority_snapshot()
        now = self._clock()
        age = lease.get('age_s')
        received = lease.get('received_monotonic')
        if (lease.get('owned') is not True or lease.get('fresh') is not True
                or isinstance(age, bool) or not isinstance(age, (int, float)) or not math.isfinite(age) or not 0 <= age <= .5
                or isinstance(received, bool) or not isinstance(received, (int, float)) or not math.isfinite(received)
                or not 0 <= now - received <= .5 or abs((now - received) - age) > .1):
            raise PermissionError('control authority is unavailable')
        if navigation and lease.get('navigation_mode_confirmed') is not True:
            raise PermissionError('navigation mode has not been confirmed')
        expires = received + .5
        if 'expires_monotonic' in lease:
            source_expiry = lease['expires_monotonic']
            if (type(source_expiry) not in (int, float) or not math.isfinite(source_expiry)
                    or source_expiry <= now):
                raise PermissionError('control authority has expired or has an invalid deadline')
            expires = min(expires, source_expiry)
        if action_deadline is not None:
            if (type(action_deadline) not in (int, float) or not math.isfinite(action_deadline)
                    or now >= action_deadline):
                raise TimeoutError('motion action deadline expired')
            expires = min(expires, action_deadline)
        # Never manufacture a new authority timestamp from an old observation.
        return {**lease, 'request_id': request_id, 'expires_monotonic': expires}

    @staticmethod
    def _deadline(request):
        value = request.get('expires_monotonic')
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('an explicit action deadline is required')
        return value

    def send(self, action, request, cancel_event):
        with self._lock:
            if action not in ('stand', 'lie', 'velocity') or request.get('action') != action:
                raise ValueError('unsupported or mismatching DDS action')
            rid = request.get('request_id')
            if not isinstance(rid, str) or not rid or len(rid) > 128:
                raise ValueError('invalid local request identifier')
            if cancel_event.is_set():
                raise InterruptedError('motion request cancelled')
            permit = self._permit(rid, navigation=action == 'velocity', action_deadline=self._deadline(request))
            first = self._request is None
            if not first and self._request['request_id'] != rid:
                raise RuntimeError('DDS backend is busy')
            if first:
                self._generation += 1
                self._request = copy.deepcopy(request)
                self._stage = 'stand_pose' if action == 'stand' else action
                self.feedback.begin(self._stage, rid, request if action == 'velocity' else {}, generation=self._generation)
            elif self._request != request:
                raise ValueError('cannot retarget an active DDS request')
            if action == 'velocity':
                return self._transport.publish_velocity(request.get('vx', 0), request.get('vy', 0), request.get('yaw_rate', 0),
                    authority=permit, cancel_event=cancel_event)
            if not first:
                raise RuntimeError('posture requests must not be replayed')
            return self._transport.publish_state(1 if action == 'stand' else 4, authority=permit, cancel_event=cancel_event)

    def progress(self, action, request, cancel_event):
        """Only the bounded action loop may advance DDS stand from 1 to 17."""
        with self._lock:
            if (cancel_event.is_set() or self._request is None or self._request['request_id'] != request['request_id']
                    or action != 'stand' or self._stage != 'stand_pose'):
                return
            self._permit(request['request_id'], action_deadline=self._deadline(request))
            if self.feedback.poll('stand_pose', request['request_id'])['confirmed']:
                self._stage = 'stand'
                self.feedback.begin('stand', request['request_id'], generation=self._generation)
                if cancel_event.is_set():
                    raise InterruptedError('stand cancelled before RL transition')
                permit = self._permit(request['request_id'], action_deadline=self._deadline(request))
                return self._transport.publish_state(17, authority=permit, cancel_event=cancel_event)

    def stop(self, reason, cancel_event):
        with self._lock:
            if self._request is None:
                return {'accepted': False, 'confirmed': False, 'reason': 'no_application_motion'}
            rid = self._request['request_id']
            permit = self._permit(rid, navigation=True)
            # A NAV_CMD zero is only valid in navigation mode. Do not invent a
            # posture cancel/damping fallback or switch operating mode here.
            if self._stage != 'stopmotion':
                self._stage = 'stopmotion'
                self.feedback.begin('stopmotion', rid, generation=self._generation)
            return self._transport.publish_velocity(0., 0., 0., authority=permit, cancel_event=cancel_event)

    def finish(self, request_id):
        with self._lock:
            if self._request is None or self._request['request_id'] != request_id:
                return
            self.feedback.invalidate(request_id)
            self._request, self._stage = None, None

    def close(self):
        with self._lock:
            self._closed = True
            self._generation += 1
            self.feedback.invalidate()
            self._request = None
        self.stop_observation()
        if self._transport is not None:
            self._transport.close()
