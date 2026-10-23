"""Finite on-site posture tests with explicit operator visual confirmation.

Operator permission is distinct from a vendor control lease. This path sends
only one state 1 or 4, never RL state 17, navigation velocity, or an invented
posture stop. A failure closes our worker; it cannot claim the robot stopped.
"""
from __future__ import annotations

import copy
import math
import threading
import time

try:
    from .motion import MotionPlugin
    from .motion_feedback import MotionFeedback
    from .parameters import META_FIELDS, validate_empty_arguments
except ImportError:
    from motion import MotionPlugin
    from motion_feedback import MotionFeedback
    from parameters import META_FIELDS, validate_empty_arguments


MODE = 'operator_supervised_posture_test'
PERMIT_SCHEMA = 's10_pro.operator_permit/1'
POSTURE_STATES = {'stand': 1, 'lie': 4}
CONFIRMATION_SOURCE = 'operator_visual_and_feedback'


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class SupervisedPostureBackend:
    """One-shot posture publisher using the existing independent observer."""

    def __init__(self, bridge, transport, authorization, *, clock=time.monotonic):
        self.bridge, self.transport, self.authorization = bridge, transport, authorization
        self._clock = clock
        self._lock = threading.RLock()
        self._observing = self._closed = self._failed = False
        self._request = None
        self._sent_at = None
        self._used = set()
        self.feedback = MotionFeedback(lambda: bridge.snapshot('motion_info'), clock=clock)

    def contract(self):
        return {'verified': False, 'mode': MODE,
                'note': 'Operator-supervised test; no vendor authority or verified autonomous stop contract.'}

    def start_observation(self):
        with self._lock:
            if self._closed:
                raise RuntimeError('supervised posture backend is closed')
            if not self._observing:
                self.bridge.acquire('supervised_posture_observer', {'motion_info', 'hard_estop'},
                                    {'refresh_hz': 10., 'stale_after_s': .5})
                self._observing = True

    def stop_observation(self):
        with self._lock:
            if self._observing:
                self.bridge.release('supervised_posture_observer')
                self._observing = False

    def safety_status(self):
        try:
            authorized = self.authorization.authorized_now() is True
        except Exception:
            authorized = False
        observed = self.feedback.observation()
        try:
            hes = self.bridge.snapshot('hard_estop') or {}
        except Exception:
            hes = {}
        now, age, received = self._clock(), hes.get('age_s'), hes.get('received_monotonic')
        hes_fresh = (hes.get('fresh') is True and not hes.get('error')
            and _finite(age) and 0 <= age <= .5 and _finite(received)
            and 0 <= now - received <= .5 and abs((now - received) - age) <= .1)
        data = hes.get('data')
        value = data.get('value') if isinstance(data, dict) else None
        hes_clear = hes_fresh and type(value) is int and value == 0
        return {'mode': MODE, 'operator_authorized': authorized,
                'vendor_control_owned': False, 'vendor_control_contract_verified': False,
                'feedback_fresh': observed['fresh'], 'feedback_age_s': observed['age_s'],
                'hard_estop_fresh': bool(hes_fresh), 'hard_estop_clear': bool(hes_clear),
                'hard_estop_value': value if type(value) is int else None,
                'hard_estop_age_s': age if _finite(age) else None,
                'observing': self._observing, 'failed_closed': self._failed or self._closed,
                'ready': bool(authorized and observed['fresh'] and hes_clear
                              and self._observing and not self._failed and not self._closed)}

    def _require_ready(self, action):
        if (action not in POSTURE_STATES or not self.safety_status()['ready']
                or action not in self.authorization.public_status().get('allowed_actions', [])):
            raise PermissionError('supervised posture conditions are unavailable')

    def prepare_execution(self, action):
        with self._lock:
            self._require_ready(action)
            status = self.transport.status()
            if status.get('closed') is True:
                self._failed = True
                raise RuntimeError('closed supervised worker cannot restart')
            if status.get('worker_running') is not True:
                try:
                    self.transport.start(timeout_s=5.)
                    self._require_ready(action)
                except Exception:
                    self.abort()
                    raise

    def send(self, action, request, cancel_event):
        """Consume the durable request once; no replay, RL transition or stop."""
        with self._lock:
            self._require_ready(action)
            rid, deadline = request.get('request_id'), request.get('expires_monotonic')
            if not isinstance(rid, str) or not rid.strip() or len(rid) > 128:
                raise ValueError('invalid supervised request identity')
            if not _finite(deadline) or self._clock() >= deadline:
                raise TimeoutError('supervised posture action expired')
            if cancel_event.is_set():
                raise InterruptedError('supervised posture cancelled before send')
            if rid in self._used or self._request is not None:
                raise RuntimeError('supervised posture request cannot be replayed')
            permit = self.authorization.permit(rid, action, deadline)
            if (not isinstance(permit, dict) or permit.get('schema') != PERMIT_SCHEMA
                    or permit.get('request_id') != rid or permit.get('action') != action):
                raise PermissionError('operator permit does not match the posture request')
            # Consume before handing the single command to the transport. An
            # uncertain publication must not be retried after a timeout.
            self._used.add(rid)
            self._request = {'request_id': rid, 'action': action}
            self._require_ready(action)
            receipt = self.transport.publish_state(POSTURE_STATES[action], authority=permit,
                                                   cancel_event=cancel_event)
            if not isinstance(receipt, dict) or receipt.get('sent') is not True:
                raise RuntimeError('supervised posture publication is unconfirmed')
            self._sent_at = self._clock()
            # Deliberately set the baseline after publish returns. Frames that
            # arrived during a slow send cannot satisfy post-send evidence.
            self.feedback.begin('stand_pose' if action == 'stand' else 'lie', rid)
            return {**receipt, 'confirmed': False, 'completed': False,
                    'mode': MODE, 'sent_monotonic': self._sent_at}

    def target_feedback(self, action, request_id):
        with self._lock:
            if (self._request != {'request_id': request_id, 'action': action}
                    or self._sent_at is None):
                return {'confirmed': False, 'fresh': False, 'reason': 'not_sent'}
            return self.feedback.poll('stand_pose' if action == 'stand' else 'lie', request_id)

    def sent_at(self, request_id):
        with self._lock:
            return self._sent_at if self._request and self._request['request_id'] == request_id else None

    def finish(self, request_id):
        with self._lock:
            if self._request and self._request['request_id'] == request_id:
                self.feedback.invalidate(request_id)
                self._request, self._sent_at = None, None

    def abort(self):
        # Closing only our command worker is not a physical stop operation.
        self._failed = True
        try:
            self.transport.close()
        except Exception:
            pass

    def close(self):
        self._closed = True
        self.abort()
        self.stop_observation()


class SupervisedMotionPlugin(MotionPlugin):
    """Reuse task persistence/ACP while requiring a current human pose check."""

    def __init__(self, config, namespace, backend, **kwargs):
        self._operator_confirmation = None
        kwargs.setdefault('feedback', backend.feedback)
        kwargs.setdefault('execution_authorized', True)
        kwargs.setdefault('authorization_provider', backend.authorization.authorized_now)
        super().__init__(config, namespace, backend=backend, **kwargs)

    def _gate(self, action=None, *, for_stop=False):
        blocked = []
        if self._closed.is_set() and not for_stop:
            blocked.append('controller_closed')
        if not self._execution_authorized_now():
            blocked.append('operator_session_unavailable')
        if self._backend.safety_status()['failed_closed']:
            blocked.append('operator_intervention_required')
        if action is not None and action not in POSTURE_STATES:
            blocked.append('supervised_posture_only')
        if self._feedback is None:
            blocked.append('fresh_feedback_adapter')
        if self._notifier is None:
            blocked.append('acp_notifier')
        return blocked

    def _control_ok(self, action=None):
        return ((action is None or action in self._backend.authorization.public_status().get('allowed_actions', []))
            and (action is None or action in POSTURE_STATES) and self._backend.safety_status()['ready'])

    def _readiness(self, *, contract=None, diagnostics=None, action=None):
        safety = self._backend.safety_status()
        blocked = self._gate(action)
        if not self._started:
            blocked.append('card_not_started')
        if not self._settings['enabled']:
            blocked.append('card_disabled')
        if not safety['feedback_fresh']:
            blocked.append('fresh_motion_feedback')
        if not safety['hard_estop_clear']:
            blocked.append('fresh_clear_hard_estop_required')
        return {'mode': MODE, 'ready': not blocked, 'card_started': self._started,
                'card_enabled': self._settings['enabled'], 'operator_supervision_required': True,
                'operator_session': self._backend.authorization.public_status(),
                'vendor_control_owned': False, 'vendor_control_contract_verified': False,
                'command_worker': {'running': self._backend.transport.status().get('worker_running') is True,
                    'preparable': bool(safety['ready'])},
                'observation': safety, 'blocked_by': blocked,
                'actions': {candidate: {'ready': not blocked and candidate in self._backend.authorization.public_status().get('allowed_actions', []), 'requires_operator_visual_confirmation': True}
                            for candidate in POSTURE_STATES},
                'unavailable_actions': ['velocity'],
                'next_steps': [{'code': value, 'instruction': value.replace('_', ' ')} for value in blocked]}

    def _unavailable(self, blocked_by, action=None):
        blocked_by = ['supervised_observation_unavailable' if item == 'fresh_control_authority' else item
                      for item in blocked_by]
        return super()._unavailable(blocked_by, action)

    def status(self):
        status = super().status()
        status.update(mode=MODE, title='Supervised Posture Test', operator_supervision_required=True,
            vendor_control_owned=False, vendor_control_contract_verified=False,
            confirmation_source=CONFIRMATION_SOURCE, supported_actions=['prepare_test', 'stand', 'lie', 'confirm_pose', 'stopmotion'],
            stop_semantics='close_our_worker_physical_stop_unverified',
            operator_confirmation=copy.deepcopy(self._operator_confirmation),
            availability_detail={'production_backend': MODE,
                'explanation': 'Finite on-site posture test. Stand sends state 1 only; lie sends state 4 only. Operator visual confirmation and fresh post-send feedback are both required. Worker closure does not stop the robot.'})
        return status

    def get_tool(self):
        tool = super().get_tool()
        schema = tool['inputSchema']
        schema['properties']['action']['enum'].remove('velocity')
        schema['properties']['action']['enum'].extend(['prepare_test', 'confirm_pose'])
        schema['properties']['observed_action'] = {'type': 'string', 'enum': ['stand', 'lie'],
            'description': 'The posture the on-site operator explicitly reports seeing.'}
        schema['allOf'].append({'if': {'properties': {'action': {'const': 'confirm_pose'}}, 'required': ['action']},
            'then': {'required': ['request_id', 'observed_action'],
                'not': {'anyOf': [{'required': [key]} for key in schema['properties']
                    if key not in {'action', 'request_id', 'observed_action', *META_FIELDS}]}},
            'else': {'not': {'required': ['observed_action']}}})
        schema['allOf'].append({'if': {'properties': {'action': {'const': 'prepare_test'}}, 'required': ['action']},
            'then': {'not': {'anyOf': [{'required': [key]} for key in schema['properties']
                if key not in {'action', *META_FIELDS}]}}})
        schema['x-completion'] = {'actions': ['stand', 'lie'], 'timeout': 90, 'silent_completion': False}
        schema['x-action-params'].update({
            'prepare_test': {'params': [], 'description': 'Prepare the isolated command worker after operator authorization. Sends no robot command.'},
            'stand': {'params': [], 'description': 'Send state 1 once. Requires the on-site operator to confirm the standing pose; no RL transition.'},
            'lie': {'params': [], 'description': 'Send state 4 once. Requires the on-site operator to confirm the lying pose.'},
            'confirm_pose': {'params': ['request_id', 'observed_action'],
                'description': 'Invoke only after the on-site operator explicitly confirms the physical pose for this request. Never infer visual confirmation from telemetry.'},
            'velocity': {'params': [], 'description': 'Unavailable in supervised posture test mode.'},
            'stopmotion': {'params': [], 'description': 'Cancel the test and close our worker. Physical state becomes unknown; use the on-site operator and handheld. This does not claim the robot stopped.'}})
        tool.update(title='Supervised Posture Test',
            description='On-site operator-supervised finite posture test. No vendor ownership lease is asserted. Cancellation, timeout or observation loss requires operator intervention; no programmatic posture stop is claimed.')
        return tool

    def confirm_pose(self, arguments):
        if not isinstance(arguments, dict) or set(arguments) - {'request_id', 'observed_action', *META_FIELDS}:
            raise ValueError('confirm_pose requires only request_id and observed_action')
        rid, action = arguments.get('request_id'), arguments.get('observed_action')
        if not isinstance(rid, str) or not rid.strip() or len(rid) > 128 or action not in POSTURE_STATES:
            raise ValueError('confirm_pose requires a valid request_id and posture action')
        for field in META_FIELDS & set(arguments):
            if not isinstance(arguments[field], str) or not arguments[field].strip():
                raise ValueError('invalid routing metadata')
        with self._lock:
            active = self._active
            if not active or active['request_id'] != rid or active['action'] != action:
                return {'accepted': False, 'completed': False, 'reason': 'active_posture_request_mismatch'}
            if self._cancel.is_set() or self._gate(action) or not self._control_ok(action):
                return {'accepted': False, 'completed': False, 'reason': 'supervised_conditions_unavailable'}
            if self._clock() >= active['expires_monotonic']:
                return {'accepted': False, 'completed': False, 'reason': 'posture_confirmation_expired'}
            observed = self._backend.target_feedback(action, rid)
            if observed.get('confirmed') is not True or observed.get('fresh') is not True:
                return {'accepted': False, 'completed': False, 'reason': 'fresh_post_send_target_not_confirmed'}
            self._operator_confirmation = {'request_id': rid, 'observed_action': action,
                'confirmed_monotonic': self._clock(), 'confirmation_source': CONFIRMATION_SOURCE}
            self._event('operator_pose_confirmed', request_id=rid, observed_action=action,
                        confirmation_source=CONFIRMATION_SOURCE)
            return {'accepted': True, 'completed': False, 'state': 'operator_confirmation_recorded',
                    'request_id': rid, 'confirmation_source': CONFIRMATION_SOURCE}

    def dispatch(self, action, args):
        if action == 'confirm_pose':
            return self.confirm_pose(args)
        if action == 'prepare_test':
            validate_empty_arguments(args)
            with self._lock:
                if self._active is not None:
                    return {'accepted': False, 'state': 'busy'}
                allowed = self._backend.authorization.public_status().get('allowed_actions', [])
                if self._gate() or not self._started or not self._settings['enabled'] or not allowed:
                    return self._unavailable(['operator_session_unavailable'])
                self._backend.prepare_execution(allowed[0])
                return {'prepared': True, 'physical_command_sent': False,
                    'command_worker_running': self._backend.transport.status().get('worker_running') is True}
        return super().dispatch(action, args)

    def _run_action(self, record, request, started):
        action, rid, deadline = request['action'], request['request_id'], request['expires_monotonic']
        receipt, completed, reason = {}, False, 'posture_confirmation_timeout'
        try:
            with self._io_lock:
                if self._cancel.is_set() or self._gate(action) or not self._control_ok(action) or not self._started:
                    raise PermissionError('supervised conditions lost before send')
                if self._wall() * 1000 > request['issued_at_ms'] + request['ttl_ms']:
                    raise TimeoutError('request expired before send')
                receipt = self._backend.send(action, request, self._cancel)
                self._last_command = self._clock()
            with self._lock:
                self._phase = 'awaiting_operator_pose_confirmation'
            self._event('sent', request_id=rid, action_id=record['action_id'],
                        command_accepted=None, confirmed=False, mode=MODE)
            while self._clock() < deadline:
                if self._cancel.is_set():
                    reason = self._cancel_reason or 'operator_cancelled'
                    break
                if self._gate(action) or not self._control_ok(action):
                    reason = 'supervised_observation_or_session_lost'
                    break
                observed = self._backend.target_feedback(action, rid)
                with self._lock:
                    confirmation = self._operator_confirmation
                    matching = (confirmation is not None and confirmation['request_id'] == rid
                        and confirmation['observed_action'] == action
                        and self._backend.sent_at(rid) is not None
                        and confirmation['confirmed_monotonic'] >= self._backend.sent_at(rid))
                    if (matching and observed.get('confirmed') is True and observed.get('fresh') is True
                            and not self._cancel.is_set() and not self._gate(action) and self._control_ok(action)):
                        completed, reason = True, 'operator_visual_and_feedback_confirmed'
                        break
                self._cancel.wait(.02)
        except Exception as exc:
            reason = 'supervised_execution_failed_' + type(exc).__name__
        finally:
            with self._lock:
                if self._clock() >= deadline or self._cancel.is_set() or self._gate(action) or not self._control_ok(action):
                    completed = False
                    reason = self._cancel_reason or 'supervised_observation_or_session_lost'
                if not completed:
                    self._backend.abort()
                result = {'request_id': rid, 'action_id': record['action_id'], 'action': action,
                    'accepted': True, 'sent': receipt.get('sent') is True,
                    'command_accepted': None, 'confirmed': completed, 'completed': completed,
                    'stop_confirmed': False, 'state': 'completed' if completed else 'error',
                    'status': 'completed' if completed else 'error', 'reason': reason,
                    'mode': MODE, 'confirmation_source': CONFIRMATION_SOURCE if completed else None,
                    'physical_state_unknown': not completed, 'operator_intervention_required': not completed,
                    'vendor_control_owned': False, 'finished_at': self._wall()}
                try:
                    if self._tasks.finish(rid, result):
                        self._last = result
                        self._phase = result['state'] if self._started else 'idle'
                        self._event('finished', **result)
                except Exception:
                    self._error = 'terminal_record_failed_restart_recovery_required'
                    self._backend.abort()
                self._active = None
            try:
                self._backend.finish(rid)
                if not self._started:
                    self._backend.stop_observation()
            finally:
                self._busy.release()
            self._flush_notifications()
