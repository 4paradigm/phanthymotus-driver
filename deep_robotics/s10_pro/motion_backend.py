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
    from .motion import REQUIRED_EVIDENCE, validate_contract, CONTROL_CONTRACT_V2
    from .motion_feedback import MotionFeedback
except ImportError:
    from motion import REQUIRED_EVIDENCE, validate_contract, CONTROL_CONTRACT_V2
    from motion_feedback import MotionFeedback


class _ActionFeedback:
    """Route posture interruption to its verified adapter, never to zero speed."""

    def __init__(self, normal, posture_stop, posture_context, clock):
        self._normal, self._adapter = normal, posture_stop
        self._context, self._clock = posture_context, clock
        self._marker = None
        self._lock = threading.RLock()

    def __getattr__(self, name):
        return getattr(self._normal, name)

    def begin_posture_stop(self, request_id, action):
        with self._lock:
            self._marker = {'request_id': request_id, 'action': action,
                'after': self._clock(), 'sent': False, 'last_sequence': 0,
                'last_received': None, 'matching_samples': 0}

    def mark_posture_stop_sent(self):
        with self._lock:
            if self._marker is not None:
                self._marker['sent'] = True
                self._marker['after'] = self._clock()

    def invalidate(self, request_id=None, *, generation=None):
        with self._lock:
            if self._marker is not None and (request_id is None or self._marker['request_id'] == request_id):
                self._marker = None
        return self._normal.invalidate(request_id, generation=generation)

    def poll(self, action, request_id):
        if action != 'stopmotion' or not self._context():
            return self._normal.poll(action, request_id)
        result = {'confirmed': False, 'fresh': False, 'age_s': None,
            'received_monotonic': None, 'request_id': request_id, 'action': action,
            'reason': 'posture_interruption_unconfirmed', 'matching_samples': 0,
            'required_samples': 2, 'correlation': 'verified_posture_adapter'}
        with self._lock:
            marker = self._marker
            if marker is None or not marker['sent'] or marker['request_id'] != request_id:
                return result
            try:
                value = self._adapter.feedback(request_id, marker['action'])
            except Exception:
                marker['matching_samples'] = 0
                return result
            if not isinstance(value, dict):
                marker['matching_samples'] = 0
                return result
            now, age, received, sequence = self._clock(), value.get('age_s'), value.get('received_monotonic'), value.get('sequence')
            finite = lambda number: type(number) in (int, float) and math.isfinite(number)
            valid = (value.get('request_id') == request_id and value.get('fresh') is True
                and finite(age) and 0 <= age <= .5 and finite(received)
                and marker['after'] <= received <= now and 0 <= now - received <= .5
                and abs((now - received) - age) <= .1
                and type(sequence) is int and sequence > 0)
            if not valid:
                marker['matching_samples'] = 0
                return result
            result.update(fresh=True, age_s=age, received_monotonic=received)
            if sequence < marker['last_sequence'] or (marker['last_received'] is not None and received < marker['last_received']):
                marker['matching_samples'] = 0
                return result
            if sequence > marker['last_sequence']:
                if marker['last_received'] is not None and received <= marker['last_received']:
                    marker['matching_samples'] = 0
                    return result
                marker['matching_samples'] = marker['matching_samples'] + 1 if value.get('confirmed') is True else 0
                marker['last_sequence'], marker['last_received'] = sequence, received
            result.update(confirmed=marker['matching_samples'] >= 2,
                          matching_samples=marker['matching_samples'])
            return result

    def __call__(self, action, request_id):
        return self.poll(action, request_id)


class DDSMotionBackend:
    def __init__(self, bridge, transport=None, *, contract=None, authority_provider=None,
                 execution_authorized=False, authorization_provider=None,
                 posture_stop=None, runtime_status=None, clock=time.monotonic):
        if type(execution_authorized) is not bool:
            raise ValueError('execution_authorized must be boolean')
        self._bridge, self._transport, self._clock = bridge, transport, clock
        self._policy = copy.deepcopy(contract or {})
        self._authority = authority_provider
        self._authorized = execution_authorized
        if authorization_provider is not None and not callable(authorization_provider):
            raise ValueError('authorization_provider must be callable')
        self._authorization_provider = authorization_provider
        if runtime_status is not None and not callable(runtime_status):
            raise ValueError('runtime_status must be callable')
        self._runtime_status = runtime_status
        self._posture_stop = posture_stop if (callable(getattr(posture_stop, 'send', None))
            and callable(getattr(posture_stop, 'feedback', None))) else None
        self._prepare_failed = False
        self._posture_stop_receipt = None
        self._posture_stop_attempted = False
        self._lock = threading.RLock()
        self._observing = self._closed = False
        self._request = None
        self._generation = 0
        self._stage = None
        normal = MotionFeedback(lambda: bridge.snapshot('motion_info'), clock=clock)
        self.feedback = _ActionFeedback(normal, self._posture_stop, self._posture_context, clock)

    def _posture_context_for_action(self, action):
        return self._policy.get('schema') == CONTROL_CONTRACT_V2 and action in ('stand', 'lie')

    def _posture_context(self):
        return (self._policy.get('schema') == CONTROL_CONTRACT_V2
            and self._request is not None and self._request.get('action') in ('stand', 'lie'))

    def _execution_allowed(self):
        try:
            return self._authorized and (self._authorization_provider is None or self._authorization_provider() is True)
        except Exception:
            return False

    def contract(self):
        if not self._policy:
            return {'verified': False, 'transport': 'ros2_dds', 'tls_required': False,
                'missing': list(REQUIRED_EVIDENCE),
                'protocol_reference': 'S10 software guide V1.0.2 §2.2/2.3'}
        policy = copy.deepcopy(self._policy)
        if policy.get('schema') == CONTROL_CONTRACT_V2 and self._posture_stop is None:
            actions = policy.get('actions', {})
            if isinstance(actions, dict):
                for action in ('stand', 'lie'):
                    if isinstance(actions.get(action), dict):
                        actions[action].update(verified=False, unavailable_reason='posture_interruption_adapter_unbound')
        return policy

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
            production_backend='dds_sequencer_bound' if self._execution_allowed() and validate_contract(self.contract()) else 'dds_sequencer_locked', observation_active=self._observing,
            command_worker_running=self._transport_running(),
            command_worker_required=True, authority_provider_bound=callable(self._authority),
            backend_execution_authorized=self._execution_allowed(),
            transport_execution_enabled=self._transport_status().get('execution_enabled'),
            control_ownership_inferred=False,
            explanation='High-level ROS 2 DDS is implemented without TLS. Control ownership, navigation mode and loss-of-command stopping still require verification.')
        if self._runtime_status is not None:
            try:
                binding = self._runtime_status()
                result['execution_binding'] = copy.deepcopy(binding) if isinstance(binding, dict) else {'status': 'unavailable'}
            except Exception:
                result['execution_binding'] = {'status': 'unavailable'}
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
        if self._policy.get('schema') == CONTROL_CONTRACT_V2:
            valid = valid and self._identity_matches(lease)
        transport = self._transport_status()
        return {'owned': lease.get('owned') is True, 'fresh': lease.get('fresh') is True,
            'age_s': lease.get('age_s'), 'feedback_fresh': observed['fresh'],
            'navigation_mode_confirmed': lease.get('navigation_mode_confirmed') is True,
            'regular_mode_confirmed': lease.get('regular_mode_confirmed') is True,
            'allowed_actions': copy.deepcopy(lease.get('allowed_actions')),
            'authority_provider_bound': callable(self._authority), 'authority_valid': bool(valid),
            'feedback_age_s': observed['age_s'], 'feedback_reason': observed['reason'],
            'command_worker_required': True, 'command_worker_running': transport.get('worker_running') is True,
            'backend_execution_authorized': self._execution_allowed(),
            'transport_execution_enabled': transport.get('execution_enabled')}

    def action_readiness(self, action):
        """Describe whether explicit action preparation could start a worker."""
        contract, lease = self.contract(), self._authority_snapshot()
        valid_contract = validate_contract(contract, action)
        v2 = contract.get('schema') == CONTROL_CONTRACT_V2
        allowed = (not v2 or (isinstance(lease.get('allowed_actions'), list)
            and action in lease['allowed_actions']))
        mode = (lease.get('navigation_mode_confirmed') is True if action == 'velocity'
            else not v2 or lease.get('regular_mode_confirmed') is True)
        control, transport = self.control_status(), self._transport_status()
        identity = (not v2 or self._identity_matches(lease))
        startable = (self._execution_allowed() and self._observing and not self._closed
            and valid_contract and control['authority_valid'] and control['feedback_fresh']
            and allowed and mode and identity and not self._prepare_failed
            and transport.get('execution_enabled') is not False
            and (self._transport_running() or callable(getattr(self._transport, 'start', None))))
        return {'action_contract_valid': valid_contract, 'action_mode_confirmed': bool(mode),
            'authority_action_allowed': bool(allowed), 'authority_valid': control['authority_valid'] and identity,
            'command_worker_running': self._transport_running(),
            'command_worker_startable': bool(startable), 'command_worker_preparable': bool(startable)}

    def _identity_matches(self, lease):
        return all(isinstance(self._policy.get(key), str) and bool(self._policy[key])
            and lease.get(key) == self._policy[key] for key in ('robot_identity', 'firmware_identity'))

    def prepare_execution(self, action):
        """Explicit action lifecycle only; observation/status never starts I/O."""
        with self._lock:
            if self._prepare_failed:
                raise RuntimeError('motion worker startup failed; a new authorized runtime is required')
            self._permit('prepare-' + action, action=action, require_worker=False)
            if not self.feedback.observation()['fresh']:
                raise PermissionError('fresh motion feedback is required before worker startup')
            if self._transport_status().get('execution_enabled') is False:
                raise PermissionError('DDS command transport execution is disabled')
            preparation_deadline = self._clock() + 5.
            if action in ('stand', 'lie') and self._posture_context_for_action(action):
                prepare_posture = getattr(self._posture_stop, 'prepare', None)
                if callable(prepare_posture):
                    try:
                        prepare_posture(timeout_s=max(.05, preparation_deadline - self._clock()))
                        self._permit('prepare-' + action, action=action, require_worker=False)
                    except Exception:
                        self._prepare_failed = True
                        self._posture_stop.close()
                        if self._transport is not None:
                            self._transport.close()
                        raise
            if not self._transport_running():
                start = getattr(self._transport, 'start', None)
                if not callable(start):
                    raise RuntimeError('DDS command worker cannot be explicitly started')
                try:
                    remaining = preparation_deadline - self._clock()
                    if remaining < .05:
                        raise TimeoutError('explicit motion preparation deadline expired')
                    start(timeout_s=min(5., remaining))
                    self._permit('prepare-' + action, action=action)
                    if not self.feedback.observation()['fresh']:
                        raise PermissionError('motion feedback expired during worker startup')
                except Exception:
                    self._prepare_failed = True
                    if self._transport is not None:
                        self._transport.close()
                    raise
            return {'prepared': True, 'action': action, 'command_worker_running': True}

    def readiness_status(self):
        """Read current observations without acquiring authority or starting I/O.

        The injected authority provider must expose its already received status
        snapshot. The production default remains unbound, with no inference from
        motion feedback, graph discovery or a newly generated timestamp.
        """
        return self.control_status()

    def _permit(self, request_id, *, navigation=False, action=None, action_deadline=None, require_worker=True):
        if self._closed or not self._observing or not self._execution_allowed() or not validate_contract(self.contract(), action):
            raise PermissionError('DDS production execution is not authorized')
        if require_worker and not self._transport_running():
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
        if self._policy.get('schema') == CONTROL_CONTRACT_V2:
            if (not self._identity_matches(lease) or not isinstance(lease.get('allowed_actions'), list)
                    or action not in lease['allowed_actions']):
                raise PermissionError('control authority does not cover this machine action')
            if action in ('stand', 'lie') and lease.get('regular_mode_confirmed') is not True:
                raise PermissionError('regular mode has not been confirmed')
            if action == 'velocity' and lease.get('navigation_mode_confirmed') is not True:
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
        if not self._execution_allowed():
            raise PermissionError('deployment authorization was revoked during authority observation')
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
            permit = self._permit(rid, navigation=action == 'velocity', action=action, action_deadline=self._deadline(request))
            first = self._request is None
            if not first and self._request['request_id'] != rid:
                raise RuntimeError('DDS backend is busy')
            if first:
                self._generation += 1
                self._request = copy.deepcopy(request)
                self._posture_stop_receipt = None
                self._posture_stop_attempted = False
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
            self._permit(request['request_id'], action=action, action_deadline=self._deadline(request))
            if self.feedback.poll('stand_pose', request['request_id'])['confirmed']:
                self._stage = 'stand'
                self.feedback.begin('stand', request['request_id'], generation=self._generation)
                if cancel_event.is_set():
                    raise InterruptedError('stand cancelled before RL transition')
                permit = self._permit(request['request_id'], action=action, action_deadline=self._deadline(request))
                return self._transport.publish_state(17, authority=permit, cancel_event=cancel_event)

    def stop(self, reason, cancel_event):
        with self._lock:
            if self._request is None:
                return {'accepted': False, 'confirmed': False, 'reason': 'no_application_motion'}
            rid = self._request['request_id']
            action = self._request['action']
            if self._posture_context():
                permit = self._permit(rid, action=action)
                if self._posture_stop_receipt is not None:
                    return copy.deepcopy(self._posture_stop_receipt)
                if self._posture_stop_attempted:
                    return {'sent': None, 'accepted': None, 'confirmed': False,
                        'completed': False, 'reason': 'posture_interruption_send_unconfirmed'}
                self._stage = 'stopmotion'
                self._posture_stop_attempted = True
                self.feedback.begin_posture_stop(rid, action)
                receipt = self._posture_stop.send(rid, action, reason, permit, cancel_event)
                if (not isinstance(receipt, dict) or receipt.get('sent') is not True
                        or receipt.get('accepted') is False):
                    raise RuntimeError('posture interruption was not sent')
                # A send/acknowledgement is never physical interruption proof.
                self._posture_stop_receipt = {**receipt, 'confirmed': False, 'completed': False}
                self.feedback.mark_posture_stop_sent()
                return copy.deepcopy(self._posture_stop_receipt)
            permit = self._permit(rid, navigation=True, action=action)
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
            self._request, self._stage, self._posture_stop_receipt = None, None, None

    def close(self):
        with self._lock:
            self._closed = True
            self._generation += 1
            self.feedback.invalidate()
            self._request = None
        self.stop_observation()
        if self._transport is not None:
            self._transport.close()
        close_posture = getattr(self._posture_stop, 'close', None)
        if callable(close_posture):
            close_posture()
