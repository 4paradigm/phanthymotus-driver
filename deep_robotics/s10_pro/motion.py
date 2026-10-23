"""Basic motion lifecycle with an explicit, fail-closed execution boundary.

No SDK or transport is constructed here. The default backend cannot send. A
real backend requires robot-specific evidence for authority, feedback and the
robot-side loss-of-command stop guarantee, plus process-level authorization.
"""
from __future__ import annotations

import copy
import math
import threading
import time
from collections import deque

try:
    from .parameters import (motion_defaults, validate_motion_settings, motion_config_schema,
        motion_action_schema, validate_motion_request, normalize_config_updates, validate_empty_arguments)
    from .motion_tasks import MotionTasks
except ImportError:
    from parameters import (motion_defaults, validate_motion_settings, motion_config_schema,
        motion_action_schema, validate_motion_request, normalize_config_updates, validate_empty_arguments)
    from motion_tasks import MotionTasks


ACTIONS = ('stand', 'lie', 'velocity', 'stopmotion')
REQUIRED_EVIDENCE = ('identity', 'firmware', 'authority', 'feedback', 'stop', 'watchdog')


class UnsupportedBackend:
    def contract(self):
        return {'verified': False, 'missing': list(REQUIRED_EVIDENCE)}

    def control_status(self):
        return {'owned': False, 'fresh': False, 'age_s': None}


def _number(value, minimum, maximum, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be finite and within {minimum}..{maximum}')
    return float(value)


CONTROL_CONTRACT_V2 = 's10_pro.control_contract/2'
EXECUTION_ACTIONS = ('stand', 'lie', 'velocity')


def _evidence_present(evidence, keys):
    return isinstance(evidence, dict) and all(
        isinstance(evidence.get(key), str) and evidence[key].strip() for key in keys)


def _valid_action_v2(contract, action):
    if (not isinstance(contract, dict) or contract.get('schema') != CONTROL_CONTRACT_V2
            or contract.get('verified') is not True
            or contract.get('robot_identity_matched') is not True
            or contract.get('firmware_matched') is not True
            or not _evidence_present(contract.get('evidence'), ('identity', 'firmware', 'authority', 'feedback'))):
        return False
    actions = contract.get('actions')
    item = actions.get(action) if isinstance(actions, dict) else None
    if (action not in EXECUTION_ACTIONS or not isinstance(item, dict)
            or item.get('verified') is not True
            or not _evidence_present(item.get('evidence'), ('stop', 'watchdog'))
            or item.get('stop_on_authority_loss') is not False):
        return False
    try:
        timeout = _number(item.get('send_timeout_s'), .01, .2, 'send_timeout_s')
        if action in ('stand', 'lie'):
            uncontrolled = _number(item.get('max_uncontrolled_s'), .1, 1., 'max_uncontrolled_s')
            return (item.get('mode') == 'regular'
                and item.get('stop_semantics') == 'verified_posture_adapter'
                and item.get('interruption_verified') is True and timeout * 2 < uncontrolled)
        deadman = _number(item.get('robot_deadman_s'), .1, 1., 'robot_deadman_s')
        return (item.get('mode') == 'navigation'
            and item.get('robot_deadman_verified') is True
            and item.get('units') == {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'}
            and item.get('stop_semantics') == 'zero_velocity_until_feedback'
            and timeout * 2 < deadman)
    except ValueError:
        return False


def validate_contract(contract, action=None):
    """Evidence must describe this machine, rather than a generic verified flag."""
    if isinstance(contract, dict) and contract.get('schema') == CONTROL_CONTRACT_V2:
        if action is None or action == 'stopmotion':
            return any(_valid_action_v2(contract, candidate) for candidate in EXECUTION_ACTIONS)
        return _valid_action_v2(contract, action)
    if action is not None and action not in ACTIONS:
        return False
    if not isinstance(contract, dict) or contract.get('verified') is not True:
        return False
    evidence = contract.get('evidence')
    if not isinstance(evidence, dict) or any(not isinstance(evidence.get(k), str) or not evidence[k].strip() for k in REQUIRED_EVIDENCE):
        return False
    if contract.get('robot_identity_matched') is not True or contract.get('firmware_matched') is not True:
        return False
    if contract.get('robot_deadman_verified') is not True:
        return False
    if contract.get('velocity_units') != {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'}:
        return False
    if not isinstance(contract.get('actions'), list) or not set(ACTIONS) <= set(contract['actions']):
        return False
    try:
        deadman = _number(contract.get('robot_deadman_s'), .1, 1., 'robot_deadman_s')
        send_timeout = _number(contract.get('send_timeout_s'), .01, .2, 'send_timeout_s')
    except ValueError:
        return False
    return (send_timeout * 2 < deadman and contract.get('stop_semantics') == 'zero_velocity_until_feedback'
        and isinstance(contract.get('stop_on_authority_loss'), bool))


def action_contract(contract, action):
    """Return one verified policy, without inheriting another action's claims."""
    if not validate_contract(contract, action):
        return {}
    if contract.get('schema') != CONTROL_CONTRACT_V2:
        return copy.deepcopy(contract)
    if action not in EXECUTION_ACTIONS:
        return {}
    item = copy.deepcopy(contract['actions'][action])
    return {**copy.deepcopy(contract), **item, 'action': action,
        'actions': [action], 'evidence': {**contract['evidence'], **item['evidence']}}


def contract_readiness(contract):
    """Describe missing machine evidence separately from application policy.

    The ranges below are requirements of this controller, never measurements or
    claims about the robot. Acceptance continues to use ``validate_contract``.
    """
    if contract.get('schema') == CONTROL_CONTRACT_V2:
        evidence = contract.get('evidence')
        evidence = evidence if isinstance(evidence, dict) else {}
        required = ('identity', 'firmware', 'authority', 'feedback')
        return {'accepted_by_application': validate_contract(contract), 'schema': CONTROL_CONTRACT_V2,
            'machine_evidence': {'required': list(required),
                'missing': [key for key in required if not _evidence_present(evidence, (key,))],
                'robot_identity_matched': contract.get('robot_identity_matched') is True,
                'firmware_matched': contract.get('firmware_matched') is True},
            'application_requirements': {'actions': {
                action: {'accepted': validate_contract(contract, action),
                    'mode': 'navigation' if action == 'velocity' else 'regular',
                    'stop_semantics': 'zero_velocity_until_feedback' if action == 'velocity' else 'verified_posture_adapter'}
                for action in EXECUTION_ACTIONS}},
            'note': 'Each action requires its own verified interruption and loss-of-command evidence. An unavailable navigation action does not block verified posture actions.'}
    evidence = contract.get('evidence')
    evidence = evidence if isinstance(evidence, dict) else {}
    missing = [key for key in REQUIRED_EVIDENCE
        if not isinstance(evidence.get(key), str) or not evidence[key].strip()]
    facts = {key: contract.get(key) is True for key in
        ('robot_identity_matched', 'firmware_matched', 'robot_deadman_verified')}
    unmet = []
    if contract.get('verified') is not True:
        unmet.append('contract_not_verified')
    if not isinstance(contract.get('actions'), list) or not set(ACTIONS) <= set(contract['actions']):
        unmet.append('complete_action_coverage')
    if contract.get('velocity_units') != {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'}:
        unmet.append('velocity_units')
    values = {}
    for key, low, high in [('robot_deadman_s', .1, 1.), ('send_timeout_s', .01, .2)]:
        try:
            values[key] = _number(contract.get(key), low, high, key)
        except ValueError:
            unmet.append(key)
    if len(values) == 2 and values['send_timeout_s'] * 2 >= values['robot_deadman_s']:
        unmet.append('send_timeout_below_half_deadman')
    if contract.get('stop_semantics') != 'zero_velocity_until_feedback':
        unmet.append('stop_semantics')
    if not isinstance(contract.get('stop_on_authority_loss'), bool):
        unmet.append('stop_on_authority_loss_policy')
    return {'accepted_by_application': validate_contract(contract),
        'machine_evidence': {'required': list(REQUIRED_EVIDENCE), 'missing': missing,
            **facts, 'unmatched': [key for key, matched in facts.items() if not matched]},
        'application_requirements': {'unmet': unmet, 'actions': list(ACTIONS),
            'velocity_units': {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'},
            'robot_deadman_range_s': [.1, 1.], 'send_timeout_range_s': [.01, .2],
            'stop_semantics': 'zero_velocity_until_feedback'},
        'note': 'Application requirements do not establish robot capabilities. Machine evidence must match this robot.'}


class MotionPlugin:
    name = PREFIX = 'motion'

    def __init__(self, config, namespace, backend=None, *, feedback=None, publish=None,
                 execution_authorized=False, clock=time.monotonic, wall_time=time.time,
                 autostart_worker=True, notifier=None, tasks=None, authorization_provider=None):
        if not isinstance(execution_authorized, bool):
            raise ValueError('execution_authorized must be boolean')
        self._backend = backend or UnsupportedBackend()
        self._feedback = feedback
        self._publish = publish or (lambda *args: None)
        self._authorized = execution_authorized
        if authorization_provider is not None and not callable(authorization_provider):
            raise ValueError('authorization_provider must be callable')
        self._authorization_provider = authorization_provider
        self._notifier = notifier
        self._tasks = tasks if tasks is not None else MotionTasks()
        self._owns_tasks = tasks is None
        self._tasks.recover_interrupted(wall_time)
        self._worker = None
        self._shutdown_done = False
        self._cancel_reason = None
        self._clock, self._wall = clock, wall_time
        self._lock = threading.RLock()
        self._io_lock = threading.Lock()
        self._notification_lock = threading.Lock()
        self._busy = threading.Lock()
        self._cancel = threading.Event()
        self._closed = threading.Event()
        self._started = False
        self._phase = 'idle'
        self._active = None
        self._generation = 0
        self._last_command = clock()
        self._last = None
        self._events = deque(maxlen=64)
        self._error = None
        self._last_tick = clock()
        self._settings = self._validate(config or {})
        base = '/' + '/'.join(p for p in (str(namespace).strip('/'), 's10_pro') if p)
        self.status_topic, self.events_topic = base + '/motion_status', base + '/motion_events'
        self._thread = threading.Thread(target=self._monitor, name='s10-motion-watchdog', daemon=True)
        if autostart_worker:
            self._thread.start()

    @staticmethod
    def _validate(settings):
        return validate_motion_settings(settings)

    def _ports(self):
        return [{'topic': self.status_topic, 'format': 'data/json'}, {'topic': self.events_topic, 'format': 'data/json'}]

    def get_tool(self):
        with self._lock:
            schema = motion_action_schema(self._settings)
            schema['x-completion'] = {'actions': ['stand', 'lie', 'velocity'],
                'timeout': 90, 'silent_completion': False}
            schema['x-resource'] = 'base'
            return {'name': self.name, 'title': 'Basic Motion', 'type': 'actuator', 'multiInstance': False,
                'description': 'Stand, lie down, finite velocity and application stop. Physical execution remains locked. Accepted tasks complete through ACP; application stop is not a hardware emergency stop.',
                'inputSchema': schema, 'configSchema': motion_config_schema(self._settings),
                'topic_in': [], 'topic_out': self._ports()}

    def get_tools(self):
        return [self.get_tool()]

    def _contract(self):
        try:
            contract = self._backend.contract()
            return copy.deepcopy(contract) if isinstance(contract, dict) else {}
        except Exception:
            return {}

    def _execution_authorized_now(self):
        if not self._authorized:
            return False
        if self._authorization_provider is None:
            return True
        try:
            return self._authorization_provider() is True
        except Exception:
            return False

    def _gate(self, action=None, *, for_stop=False):
        missing = []
        if self._closed.is_set() and not for_stop:
            missing.append('controller_closed')
        if not self._execution_authorized_now():
            missing.append('execution_authorization')
        if not validate_contract(self._contract(), action):
            missing.append('robot_control_contract')
        if self._feedback is None:
            missing.append('fresh_feedback_adapter')
        if self._notifier is None:
            missing.append('acp_notifier')
        return missing

    def _authority_ok(self):
        try:
            value = self._backend.control_status()
            age = value.get('age_s')
            return (value.get('owned') is True and value.get('fresh') is True
                and value.get('authority_valid', True) is True
                and isinstance(age, (int, float)) and not isinstance(age, bool) and math.isfinite(age) and 0 <= age <= .5)
        except Exception:
            return False

    def _control_ok(self, action=None):
        try:
            control = self._backend.control_status()
            if not self._authority_ok() or control.get('feedback_fresh') is not True:
                return False
            if action is not None and self._contract().get('schema') == CONTROL_CONTRACT_V2:
                permitted = control.get('allowed_actions')
                if not isinstance(permitted, (list, tuple, set)) or action not in permitted:
                    return False
                mode = 'navigation_mode_confirmed' if action == 'velocity' else 'regular_mode_confirmed'
                return control.get(mode) is True
            return True
        except Exception:
            return False

    def _readiness(self, *, contract=None, diagnostics=None, action=None):
        contract = self._contract() if contract is None else contract
        diagnostics = diagnostics or {}
        try:
            # Use the backend's explicit read-only diagnostic interface. Do not
            # invoke the action-side authority adapter merely to render status;
            # an unknown adapter remains unavailable rather than being probed.
            reader = getattr(self._backend, 'readiness_status', None)
            control = reader() if callable(reader) else {}
            control = control if isinstance(control, dict) else {}
        except Exception:
            control = {}
        def age(value):
            return float(value) if type(value) in (int, float) and math.isfinite(value) and value >= 0 else None
        authority_age = age(control.get('age_s'))
        authority_fresh = (control.get('owned') is True and control.get('fresh') is True
            and authority_age is not None and authority_age <= .5
            and control.get('authority_valid', True) is True)
        authority_bound = control.get('authority_provider_bound', diagnostics.get('authority_provider_bound'))
        feedback_age = age(control.get('feedback_age_s', diagnostics.get('age_s')))
        feedback_fresh = (self._feedback is not None and control.get('feedback_fresh') is True
            and (feedback_age is None or feedback_age <= .5))
        # A backend reporting the observation age must not hide an invalid value
        # behind a fresh boolean. Older injected test adapters omit the field.
        if 'feedback_age_s' in control and feedback_age is None:
            feedback_fresh = False
        worker_required = control.get('command_worker_required', diagnostics.get('command_worker_required')) is True
        worker_running = control.get('command_worker_running', diagnostics.get('command_worker_running'))
        backend_authorized = control.get('backend_execution_authorized', diagnostics.get('backend_execution_authorized'))
        transport_enabled = control.get('transport_execution_enabled', diagnostics.get('transport_execution_enabled'))
        blocked = self._gate(action)
        if backend_authorized is False:
            blocked.append('backend_execution_authorization')
        if transport_enabled is False:
            blocked.append('transport_execution_authorization')
        if not self._started:
            blocked.append('card_not_started')
        if not self._settings['enabled']:
            blocked.append('card_disabled')
        if authority_bound is False:
            blocked.append('authority_provider_unbound')
        if not authority_fresh:
            blocked.append('fresh_control_authority')
        if not feedback_fresh:
            blocked.append('fresh_motion_feedback')
        if worker_required and worker_running is not True:
            blocked.append('command_worker_not_running')
        action_availability = {}
        for candidate in EXECUTION_ACTIONS:
            reasons = [reason for reason in blocked if reason != 'robot_control_contract']
            if not validate_contract(contract, candidate):
                reasons.append('robot_control_contract')
            # Runtime scope and mode are separate facts from the static
            # machine policy. Legacy injected adapters may omit these fields.
            permitted = control.get('allowed_actions')
            if isinstance(permitted, (list, tuple, set)) and candidate not in permitted:
                reasons.append('action_authority_unavailable')
            if contract.get('schema') == CONTROL_CONTRACT_V2:
                if not isinstance(permitted, (list, tuple, set)):
                    reasons.append('action_authority_unavailable')
                mode_field = 'navigation_mode_confirmed' if candidate == 'velocity' else 'regular_mode_confirmed'
                if control.get(mode_field) is not True:
                    reasons.append('operating_mode_unconfirmed')
            can_prepare = False
            action_status = getattr(self._backend, 'action_readiness', None)
            if callable(action_status):
                try:
                    observed = action_status(candidate)
                    can_prepare = (isinstance(observed, dict)
                        and observed.get('command_worker_startable') is True)
                except Exception:
                    pass
            action_availability[candidate] = {'contract_verified': validate_contract(contract, candidate),
                'ready': not reasons, 'blocked_by': reasons, 'command_worker_preparable': can_prepare}
        instructions = {
            'controller_closed': 'Use a running driver instance; this controller is closed.',
            'execution_authorization': 'Authorize the deployment explicitly after verifying machine evidence. Canvas settings cannot grant process authorization.',
            'backend_execution_authorization': 'Bind an explicitly authorized motion backend after machine evidence has been verified.',
            'transport_execution_authorization': 'Authorize the command transport through the deployment process; card configuration cannot enable it.',
            'robot_control_contract': 'Verify this robot identity, firmware, control ownership, stopping and watchdog evidence, then bind its control contract.',
            'fresh_feedback_adapter': 'Bind the documented motion feedback adapter.',
            'acp_notifier': 'Bind the durable ACP completion notifier.',
            'card_not_started': 'Start Basic Motion to begin read-only feedback observation.',
            'card_disabled': 'Enable Basic Motion when the other execution requirements have been met.',
            'authority_provider_unbound': 'Bind a documented live control authority adapter. A visible feedback topic does not grant ownership.',
            'fresh_control_authority': 'Obtain valid live control ownership within the 0.5 second application limit; do not synthesize a lease from feedback.',
            'fresh_motion_feedback': 'Restore fresh /MOTION_INFO feedback within the existing 0.5 second limit.',
            'command_worker_not_running': 'Start the command worker only through an authorized deployment after contract and live authority checks pass.',
            'action_authority_unavailable': 'Obtain live control authority scoped to the requested action.',
            'operating_mode_unconfirmed': 'Confirm the action-specific operating mode from its verified live source.',
        }
        if action in action_availability:
            blocked = action_availability[action]['blocked_by']
        ready = not blocked
        if action is None and contract.get('schema') == CONTROL_CONTRACT_V2:
            ready = any(item['ready'] for item in action_availability.values())
        plugin_authorized = self._execution_authorized_now()
        return {'ready': ready, 'card_started': self._started,
            'card_enabled': self._settings['enabled'],
            'process_authorization': {'granted': plugin_authorized and backend_authorized is not False and transport_enabled is not False,
                'plugin_authorized': plugin_authorized, 'backend_authorized': backend_authorized,
                'transport_enabled': transport_enabled, 'canvas_configurable': False},
            'contract': contract_readiness(contract),
            'authority': {'provider_bound': authority_bound, 'owned': control.get('owned') is True,
                'fresh': authority_fresh, 'age_s': authority_age, 'max_age_s': .5,
                'navigation_mode_confirmed': control.get('navigation_mode_confirmed') is True},
            'feedback': {'adapter_bound': self._feedback is not None, 'fresh': feedback_fresh,
                'age_s': feedback_age, 'max_age_s': .5,
                'reason': control.get('feedback_reason', diagnostics.get('reason'))},
            'command_worker': {'required': worker_required, 'running': worker_running},
            'actions': action_availability,
            'blocked_by': blocked,
            'next_steps': [{'code': reason, 'instruction': instructions.get(reason, 'Review the action-specific readiness details.')} for reason in blocked]}

    def _unavailable(self, blocked_by, action=None):
        readiness = self._readiness(action=action)
        return {'accepted': False, 'confirmed': False, 'completed': False,
            'state': 'unavailable', 'blocked_by': blocked_by,
            'readiness': readiness, 'next_steps': readiness['next_steps']}

    def _event(self, kind, **values):
        with self._lock:
            event = {'card': 'motion', 'event': kind, 'timestamp_ms': int(self._wall() * 1000), **values}
            self._events.append(event)
            if self._started:
                try:
                    self._publish(self.events_topic, 'data/json', copy.deepcopy(event))
                except Exception:
                    self._error = 'status_publish_failed'

    def status(self):
        with self._lock:
            missing = self._gate()
            contract = self._contract()
            diagnostics = {}
            if hasattr(self._backend, 'diagnostics'):
                try:
                    diagnostics = self._backend.diagnostics()
                except Exception:
                    diagnostics = {'preflight_error': 'observation_unavailable'}
            readiness = self._readiness(contract=contract, diagnostics=diagnostics)
            return {'card': 'motion', 'state': 'running' if self._started else 'idle', 'phase': self._phase,
                'execution_available': readiness['ready'], 'execution_authorized': self._execution_authorized_now(),
                'contract_verified': validate_contract(contract), 'blocked_by': missing,
                'readiness': readiness, 'next_steps': readiness['next_steps'],
                'availability_detail': {
                    'protocol_implemented': 'S10 software guide V1.0.2',
                    'production_backend': 'unbound',
                    'missing_evidence': contract.get('missing', []),
                    'explanation': 'Robot version, control ownership and loss-of-command stopping remain unverified. Canvas settings cannot authorize execution.',
                    **diagnostics},
                'settings': dict(self._settings), 'active_request': copy.deepcopy(self._active),
                'last_result': copy.deepcopy(self._last), 'last_error': self._error,
                'completion_delivery': self._delivery_status(),
                'topic_in': [], 'topic_out': self._ports(), 'supported_actions': list(ACTIONS),
                'stop_semantics': 'application_stop_not_emergency_stop', 'events': list(self._events)}

    def start(self):
        with self._lock:
            if self._closed.is_set():
                return self.status()
            if hasattr(self._backend, 'start_observation'):
                self._backend.start_observation()
            self._started = True
            self._phase = 'locked' if self._gate() or not self._settings['enabled'] else 'ready'
        return self.status()

    def stop(self):
        with self._lock:
            self._started = False
            self._phase = 'idle'
        result = self.cancel('card_stopped')
        with self._lock:
            if self._active is None and hasattr(self._backend, 'stop_observation'):
                self._backend.stop_observation()
        return {**self.status(), 'cancellation': result}

    def close(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self._closed.set()
        self.stop()
        worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(2)
        if hasattr(self._backend, 'close'):
            self._backend.close()
        if worker is not None and worker is not threading.current_thread() and worker.is_alive():
            worker.join(1)
        if self._thread.ident is not None and self._thread is not threading.current_thread():
            self._thread.join(1)
        self._flush_notifications()
        if self._owns_tasks and (worker is None or not worker.is_alive()):
            self._tasks.close()

    def configure(self, options):
        updates = normalize_config_updates(options, motion_defaults())
        with self._lock:
            updated = self._validate({**self._settings, **updates})
            # Core reapplies saved settings before forwarding tool actions.
            # Repeating a configuration is not a control change and must not
            # cancel the task whose result the user is inspecting.
            if updated == self._settings:
                return self.status()
            active = self._active is not None
            self._generation += 1
            self._settings = updated
        if active:
            self.cancel('configuration_changed')
        return self.status()

    def _delivery_status(self):
        if self._notifier is None:
            return {'delivery_state': 'unavailable', 'acknowledged': False}
        task = self._active or self._last
        if task is None:
            return {'delivery_state': 'idle', 'acknowledged': False}
        try:
            return self._notifier.status(task['action_id'])
        except Exception:
            return {'delivery_state': 'status_unavailable', 'acknowledged': False}

    def _flush_notifications(self):
        if self._notifier is None or not self._notification_lock.acquire(blocking=False):
            return
        try:
            for record in self._tasks.pending_notifications():
                result = record['result']
                payload = {'action_id': record['action_id'], 'request_id': record['request_id'],
                    'status': result['status'], 'result': result, 'tool': 'motion', 'ts': result['finished_at']}
                try:
                    self._notifier.submit(payload)
                    self._tasks.mark_enqueued(record['request_id'])
                except Exception:
                    self._error = 'completion_enqueue_failed_retained_for_retry'
        finally:
            self._notification_lock.release()

    def _stop_backend(self, reason, *, request_id, authority_lost=False):
        with self._lock:
            if self._active is None or self._active.get('request_id') != request_id:
                return {'accepted': False, 'confirmed': False, 'reason': 'request_replaced'}
            action = self._active.get('action')
        if self._gate(action, for_stop=True):
            return {'accepted': False, 'confirmed': False, 'reason': 'unavailable'}
        contract = action_contract(self._contract(), action)
        if authority_lost and not contract['stop_on_authority_loss']:
            return {'accepted': None, 'confirmed': False, 'reason': 'authority_released'}
        if not self._io_lock.acquire(timeout=.25):
            return {'accepted': None, 'confirmed': False, 'reason': 'stop_transport_busy'}
        try:
            with self._lock:
                if self._active is None or self._active['request_id'] != request_id:
                    return {'accepted': False, 'confirmed': False, 'reason': 'request_replaced'}
            # Recheck ownership inside the transport critical section. Never
            # send a delayed stop over a later request or manual takeover.
            if not self._authority_ok() and not contract['stop_on_authority_loss']:
                return {'accepted': None, 'confirmed': False, 'reason': 'authority_released'}
            # An already-set movement cancellation token must not cancel its own stop.
            return self._backend.stop(reason, threading.Event())
        except Exception:
            return {'accepted': None, 'confirmed': False, 'reason': 'stop_transport_failed'}
        finally:
            self._io_lock.release()

    def cancel(self, reason='user_stop'):
        # This lock never surrounds transport I/O. The action worker owns stop
        # and the sole terminal commit; the caller never waits for feedback.
        with self._lock:
            active = copy.deepcopy(self._active)
            if active is None:
                return {'accepted': False, 'confirmed': False, 'completed': False,
                        'reason': 'no_application_motion'}
            first = not self._cancel.is_set()
            self._cancel.set()
            if first:
                self._cancel_reason = reason
                self._generation += 1
                self._event('cancel_requested', request_id=active['request_id'],
                    original_action_id=active['action_id'], reason=reason, stop_confirmed=False)
            return {'accepted': True, 'confirmed': False, 'completed': False,
                'state': 'cancellation_requested', 'request_id': active['request_id'],
                'original_action_id': active['action_id'], 'reason': self._cancel_reason,
                'stop_confirmed': False}

    def _confirmation(self, action, request_id, after):
        try:
            value = self._feedback(action, request_id)
            age = value.get('age_s')
            received = value.get('received_monotonic')
            now = self._clock()
            return (value.get('confirmed') is True and value.get('fresh') is True
                and isinstance(age, (int, float)) and not isinstance(age, bool) and math.isfinite(age) and 0 <= age <= .5
                and isinstance(received, (int, float)) and not isinstance(received, bool) and math.isfinite(received)
                and after <= received <= now and abs((now - received) - age) <= .1
                and value.get('request_id') == request_id)
        except Exception:
            return False

    @staticmethod
    def _receipt(record, replayed=False):
        if record['result'] is not None:
            # Returning action_id here would make Core register a fresh pending
            # after the original barrier has reaped it. Keep identity nested.
            result = copy.deepcopy(record['result'])
            result['original_action_id'] = result.pop('action_id')
            return {**result, 'replayed': True, 'terminal': True}
        return {'action_id': record['action_id'], 'request_id': record['request_id'],
                'action': record['request']['action'], 'accepted': True,
                'confirmed': False, 'completed': False, 'stop_confirmed': False,
                'state': 'accepted', 'status': 'running', 'replayed': replayed}

    def execute(self, action, arguments):
        if action not in ('stand', 'lie', 'velocity'):
            raise ValueError('Unsupported motion action')
        with self._lock:
            settings, generation = dict(self._settings), self._generation
            request = validate_motion_request(action, arguments, settings, self._wall, check_freshness=False)
            if self._closed.is_set():
                return self._unavailable(['controller_closed'], action)
            previous = self._tasks.get(request['request_id'])
            if previous:
                if self._tasks.fingerprint(request) != previous['fingerprint']:
                    raise ValueError('request_id already belongs to a different motion target')
                return self._receipt(previous, replayed=True)
            # Only first acceptance is subject to request TTL. Replays keep the
            # original identity even when their original acceptance TTL elapsed.
            request = validate_motion_request(action, {**arguments, 'request_id': request['request_id']},
                                              settings, self._wall)
            missing = self._gate(action)
            if missing or not self._started or not settings['enabled']:
                return self._unavailable(missing + ([] if self._started and settings['enabled'] else ['card_not_enabled']), action)
            if not self._control_ok(action):
                return self._unavailable(['fresh_control_authority'], action)
            if not self._busy.acquire(blocking=False):
                return {'accepted': False, 'confirmed': False, 'completed': False, 'state': 'busy'}
            if generation != self._generation or not self._started or self._closed.is_set():
                self._busy.release()
                return {'accepted': False, 'confirmed': False, 'completed': False,
                        'state': 'cancelled', 'reason': 'lifecycle_changed'}
            prepare = getattr(self._backend, 'prepare_execution', None)
            if callable(prepare):
                try:
                    prepare(action)
                except Exception:
                    self._busy.release()
                    return self._unavailable(['command_worker_not_running'], action)
                if (generation != self._generation or not self._started or self._closed.is_set()
                        or self._gate(action) or not self._control_ok(action)):
                    self._busy.release()
                    return self._unavailable(['fresh_control_authority'], action)
            if self._wall() * 1000 > request['issued_at_ms'] + request['ttl_ms']:
                self._busy.release()
                return self._unavailable(['stale_request'], action)
            try:
                record, replayed = self._tasks.accept(request)
            except Exception:
                self._busy.release()
                raise
            if replayed:
                self._busy.release()
                return self._receipt(record, replayed=True)
            started = self._clock()
            request['expires_monotonic'] = started + (request['duration'] if action == 'velocity' else settings['action_timeout_s'])
            self._cancel, self._cancel_reason = threading.Event(), None
            self._active = {**request, 'action_id': record['action_id'], 'started_monotonic': started}
            self._last_command, self._phase = started, 'active'
            receipt = self._receipt(record)
            self._worker = threading.Thread(target=self._run_action,
                args=(record, request, started), name='s10-action-' + record['action_id'], daemon=True)
            self._event('requested', request_id=request['request_id'], action_id=record['action_id'], action=action)
            try:
                self._worker.start()
            except Exception:
                result = {'request_id': request['request_id'], 'action_id': record['action_id'],
                    'action': action, 'status': 'error', 'state': 'error', 'accepted': True,
                    'confirmed': False, 'stop_confirmed': False, 'completed': False,
                    'reason': 'action_worker_start_failed', 'finished_at': self._wall()}
                self._tasks.finish(request['request_id'], result)
                self._last, self._active, self._phase = result, None, 'error'
                self._busy.release()
        self._flush_notifications()
        return receipt

    def _wait_for_stop(self, reason, request_id, interval):
        with self._lock:
            if self._active and self._active['request_id'] == request_id:
                self._phase = 'stopping'
        after = self._clock()
        receipt = self._stop_backend(reason, request_id=request_id, authority_lost=not self._authority_ok())
        deadline, next_send = self._clock() + 1., self._clock() + interval
        while self._clock() < deadline and receipt.get('reason') != 'authority_released':
            if self._confirmation('stopmotion', request_id, after):
                return True
            if self._clock() >= next_send:
                receipt = self._stop_backend(reason, request_id=request_id, authority_lost=not self._authority_ok())
                next_send = self._clock() + interval
            time.sleep(.02)
        return False

    def _run_action(self, record, request, started):
        action, rid = request['action'], request['request_id']
        deadline = request['expires_monotonic']
        receipt, confirmed, stop_confirmed = {'accepted': None}, False, False
        reason, stop_attempted = 'feedback_timeout', False
        interval = .1
        try:
            contract = action_contract(self._contract(), action)
            if action == 'velocity':
                interval = min(.1, (contract['robot_deadman_s'] - contract['send_timeout_s']) / 3)
            next_send = started
            while self._clock() < deadline:
                if self._cancel.is_set():
                    reason = self._cancel_reason or 'cancelled'
                    break
                if self._gate(action) or not self._control_ok(action):
                    reason = 'authority_unavailable'
                    break
                if self._clock() >= next_send:
                    with self._io_lock:
                        if self._cancel.is_set() or self._gate(action) or not self._control_ok(action) or not self._started:
                            reason = self._cancel_reason or 'authority_unavailable'
                            break
                        if next_send == started and self._wall() * 1000 > request['issued_at_ms'] + request['ttl_ms']:
                            reason = 'stale_request'
                            break
                        if self._clock() >= deadline:
                            reason = 'action_deadline_expired'
                            break
                        receipt = self._backend.send(action, request, self._cancel)
                        self._last_command = self._clock()
                    if receipt.get('accepted') is False:
                        reason = 'rejected'
                        break
                    next_send = self._clock() + interval if action == 'velocity' else float('inf')
                    self._event('sent', request_id=rid, action_id=record['action_id'],
                                command_accepted=receipt.get('accepted'), confirmed=False)
                if self._confirmation(action, rid, started):
                    confirmed = True
                    if action != 'velocity':
                        reason = 'feedback_confirmed'
                        break
                if hasattr(self._backend, 'progress') and self._clock() < deadline:
                    with self._io_lock:
                        if not self._cancel.is_set() and not self._gate(action) and self._control_ok(action) and self._clock() < deadline:
                            advanced = self._backend.progress(action, request, self._cancel)
                            if advanced is not None:
                                receipt = advanced
                                if receipt.get('accepted') is False:
                                    reason = 'rejected'
                                    break
                self._cancel.wait(.02)
            else:
                # Only a natural deadline exit completes a finite velocity.
                # A rejected send or lost authority can also advance time past
                # the deadline; retain its failure instead of hiding it.
                if action == 'velocity':
                    reason = 'duration_elapsed'
            if action == 'velocity' or not confirmed or self._cancel.is_set():
                stop_confirmed = self._wait_for_stop(reason, rid, interval)
                stop_attempted = True
        except Exception as exc:
            reason = 'execution_failed_' + type(exc).__name__
            confirmed = False
            try:
                stop_confirmed = self._wait_for_stop(reason, rid, interval)
            except Exception:
                stop_confirmed = False
            stop_attempted = True
        finally:
            # Cancel and completion compete at this one commit lock. A late
            # cancellation first gets a real stop attempt, never a fake success.
            while True:
                with self._lock:
                    need_stop = self._cancel.is_set() and not stop_attempted
                    if not need_stop:
                        cancelled = self._cancel.is_set()
                        if cancelled:
                            reason = self._cancel_reason or 'cancelled'
                        completed = (not cancelled and confirmed
                            and reason in ('feedback_confirmed', 'duration_elapsed')
                            and (action != 'velocity' or stop_confirmed))
                        status = ('completed' if completed else 'cancelled'
                                  if cancelled and stop_confirmed else 'error')
                        if (cancelled or action == 'velocity') and not stop_confirmed:
                            reason += '_stop_unconfirmed'
                        result = {'request_id': rid, 'action_id': record['action_id'], 'action': action,
                            'accepted': True, 'command_accepted': receipt.get('accepted'),
                            'confirmed': confirmed and not cancelled, 'stop_confirmed': stop_confirmed,
                            'completed': completed, 'state': status, 'status': status,
                            'reason': reason, 'finished_at': self._wall()}
                        try:
                            if self._tasks.finish(rid, result):
                                self._last = result
                                self._phase = status if self._started else 'idle'
                                self._event('finished', **result)
                        except Exception:
                            self._error = 'terminal_record_failed_restart_recovery_required'
                        self._active = None
                        break
                stop_confirmed = self._wait_for_stop(self._cancel_reason or 'cancelled', rid, interval)
                stop_attempted = True
            try:
                if hasattr(self._backend, 'finish'):
                    self._backend.finish(rid)
                if not self._started and hasattr(self._backend, 'stop_observation'):
                    self._backend.stop_observation()
            finally:
                self._busy.release()
            self._flush_notifications()

    def dispatch(self, action, args):
        if action in ('info', 'status'):
            validate_empty_arguments(args)
            return self.status()
        if action == 'start':
            updates = normalize_config_updates(args, motion_defaults())
            if updates:
                self.configure(updates)
            return self.start()
        if action == 'stop':
            validate_empty_arguments(args)
            return self.stop()
        if action == 'config':
            return self.configure(args)
        if action == 'stopmotion':
            validate_motion_request(action, args, self._settings, self._wall)
            return self.cancel()
        return self.execute(action, args)

    def _monitor(self):
        while not self._closed.wait(.05):
            self._flush_notifications()
            now = self._clock()
            with self._lock:
                active = self._active is not None
                started = self._started
                overdue = now - self._last_tick > .5
                if active and self._active['action'] == 'velocity' and self._phase == 'active':
                    policy = action_contract(self._contract(), 'velocity')
                    overdue = overdue or now - self._last_command > policy.get('robot_deadman_s', .3) / 2
                self._last_tick = now
            with self._lock:
                action = self._active.get('action') if self._active else None
            if active and (overdue or self._gate(action) or not self._control_ok(action)):
                self.cancel('watchdog_late' if overdue else 'authority_unavailable')
            if started and now >= getattr(self, '_next_status', 0):
                with self._lock:
                    if self._started:
                        try:
                            payload = self.status()
                            payload['timestamp_ms'] = int(self._wall() * 1000)
                            self._publish(self.status_topic, 'data/json', payload)
                        except Exception:
                            self._error = 'status_publish_failed'
                        self._next_status = now + 1. / self._settings['refresh_hz']
