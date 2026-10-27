"""Finite on-site posture requests with automatic reported-state observation.

Operator permission is distinct from a vendor control lease. This path sends
only one explicitly selected state 1, 4 or 17, never navigation velocity or an
invented posture stop. A failure closes our worker; it cannot claim the robot stopped.
"""
from __future__ import annotations

import copy
import math
import threading
import time

try:
    from .motion import MotionPlugin
    from .motion_feedback import MotionFeedback
    from .motion_dds import CommandChannelNotReady
    from .stationary_feedback import stationary_velocity
    from .parameters import META_FIELDS, validate_empty_arguments
except ImportError:
    from motion import MotionPlugin
    from motion_feedback import MotionFeedback
    from motion_dds import CommandChannelNotReady
    from stationary_feedback import stationary_velocity
    from parameters import META_FIELDS, validate_empty_arguments


MODE = 'operator_supervised_posture_test'
PERMIT_SCHEMA = 's10_pro.operator_permit/1'
PERMIT_SCHEMAS = {PERMIT_SCHEMA, 's10_pro.operator_permit/2'}
POSTURE_STATES = {'stand': 1, 'lie': 4, 'move_mode': 17}
FEEDBACK_ACTIONS = {'stand': 'stand_pose', 'lie': 'lie_idle', 'move_mode': 'stand'}
COMPLETION_STATES = {'stand': 1, 'lie': 0, 'move_mode': 17}
COMPLETION_LEVEL = 'reported_motion_state'
COMPLETION_BASIS = 'post_send_target_state_feedback'


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
        self._channel = {'publisher_prepared': False, 'matched_subscribers': 0,
                         'ready': False, 'checked_monotonic': None}
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
    def transition_blockers(self, action):
        """Validate only transitions observed in this robot's native FSM logs."""
        blocked = []
        observed = self.feedback.observation()
        state = observed.get('motion_state')
        if not observed.get('fresh'):
            return ['fresh_motion_feedback']
        if action == 'move_mode':
            if state == 17:
                blocked.append('already_in_move_mode')
            elif state != 1:
                blocked.append('stand_first_required')
            if not stationary_velocity(observed):
                blocked.append('fresh_stationary_feedback_required')
        elif action == 'stand':
            if state == 17:
                blocked.append('exit_move_with_lie_required')
            elif state not in (0, 1):
                blocked.append('wait_for_idle_required')
        elif action == 'lie':
            if state == 4:
                blocked.append('wait_for_idle_required')
            elif state not in (0, 1, 17):
                blocked.append('unsupported_transition_state')
        return blocked

    def mode_blockers(self, *, require_standing=False):
        # Compatibility for status callers; move_mode always requires state 1.
        return self.transition_blockers('move_mode')

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
            # Discovery is a preparation operation, outside the .2s send RPC.
            # Polling this graph never constructs or publishes a motion message.
            try:
                self._wait_for_channel(action)
            except CommandChannelNotReady:
                # Nothing was published or accepted. A later explicit prepare
                # may observe a receiver; this is not an automatic action retry.
                raise
            except Exception:
                self.abort()
                raise

    def _wait_for_channel(self, action):
        deadline = self._clock() + 2.
        while True:
            self._require_ready(action)
            channel = self.transport.graph_snapshot().get('state', {})
            matched = channel.get('matched_subscribers')
            ready = (channel.get('publisher_prepared') is True
                     and type(matched) is int and matched > 0)
            self._channel = {'publisher_prepared': channel.get('publisher_prepared') is True,
                             'matched_subscribers': matched if type(matched) is int and matched >= 0 else 0,
                             'ready': ready, 'checked_monotonic': self._clock()}
            if ready:
                self._require_ready(action)
                return
            if self._clock() >= deadline:
                raise CommandChannelNotReady('no matched posture receiver after bounded preparation')
            time.sleep(.02)

    def command_channel_status(self):
        with self._lock:
            channel = copy.deepcopy(self._channel)
            checked = channel.pop('checked_monotonic')
            return {**channel, 'checked_age_s': None if checked is None else max(0., self._clock() - checked),
                    'receiver_acceptance_verified': False}

    def send(self, action, request, cancel_event):
        """Consume the durable request once; no replay or implicit extra action."""
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
            transition_blocked = self.transition_blockers(action)
            if transition_blocked:
                raise PermissionError('unsupported posture transition: ' + ','.join(transition_blocked))
            permit = self.authorization.permit(rid, action, deadline)
            if (not isinstance(permit, dict) or permit.get('schema') not in PERMIT_SCHEMAS
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
            self.feedback.begin(FEEDBACK_ACTIONS[action], rid)
            return {**receipt, 'confirmed': False, 'completed': False,
                    'mode': MODE, 'sent_monotonic': self._sent_at}

    def target_feedback(self, action, request_id):
        with self._lock:
            if (self._request != {'request_id': request_id, 'action': action}
                    or self._sent_at is None):
                return {'confirmed': False, 'fresh': False, 'reason': 'not_sent'}
            return self.feedback.poll(FEEDBACK_ACTIONS[action], request_id)

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
    """Send one posture request and observe the reported target state via ACP."""

    execution_actions = (*POSTURE_STATES, 'velocity')

    def __init__(self, config, namespace, backend, **kwargs):
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
        transition_blocked = {candidate: self._backend.transition_blockers(candidate)
                              for candidate in POSTURE_STATES}
        if action in transition_blocked:
            blocked.extend(transition_blocked[action])
        conditions_ready = not blocked
        worker_running = self._backend.transport.status().get('worker_running') is True
        channel = self._backend.command_channel_status()
        if not worker_running or not channel['publisher_prepared']:
            blocked.append('command_channel_not_prepared')
        elif not channel['ready']:
            blocked.append('command_receiver_unmatched')
        return {'mode': MODE, 'ready': not blocked, 'card_started': self._started,
                'conditions_ready': conditions_ready,
                'card_enabled': self._settings['enabled'], 'operator_supervision_required': True,
                'operator_session': self._backend.authorization.public_status(),
                'vendor_control_owned': False, 'vendor_control_contract_verified': False,
                'command_worker': {'running': worker_running,
                    'preparable': bool(safety['ready']),
                    'channel': channel},
                'observation': safety, 'blocked_by': blocked,
                'actions': {candidate: {'ready': not blocked and candidate in self._backend.authorization.public_status().get('allowed_actions', [])
                                       and not transition_blocked[candidate],
                                       'blocked_by': transition_blocked[candidate],
                                       'experimental': candidate == 'move_mode',
                                       'completion_level': COMPLETION_LEVEL}
                            for candidate in POSTURE_STATES},
                'unavailable_actions': ['velocity'],
                'next_steps': [{'code': value, 'instruction': (
                    'Prepare the supervised command channel; no posture command is sent.'
                    if value == 'command_channel_not_prepared' else value.replace('_', ' '))} for value in blocked]}

    def _unavailable(self, blocked_by, action=None):
        blocked_by = ['supervised_observation_unavailable' if item == 'fresh_control_authority' else item
                      for item in blocked_by]
        result = super()._unavailable(blocked_by, action)
        if 'command_receiver_unmatched' in blocked_by:
            readiness = result['readiness']
            readiness['ready'] = False
            readiness['blocked_by'] = list(dict.fromkeys([*readiness['blocked_by'], 'command_receiver_unmatched']))
            step = {'code': 'command_receiver_unmatched',
                    'instruction': 'Check the /MOTION_STATE receiver, type and QoS; no posture command was sent.'}
            readiness['next_steps'] = [s for s in readiness['next_steps'] if s['code'] != 'command_receiver_unmatched']
            readiness['next_steps'].append(step)
            result['next_steps'] = readiness['next_steps']
        return result

    def status(self):
        status = super().status()
        status['blocked_by'] = list(status['readiness']['blocked_by'])
        if status['phase'] == 'ready' and not status['readiness']['ready']:
            status['phase'] = 'not_ready'
        status.update(mode=MODE, title='Posture Requests', operator_supervision_required=True,
            vendor_control_owned=False, vendor_control_contract_verified=False,
            completion_level=COMPLETION_LEVEL, completion_basis=COMPLETION_BASIS,
            physical_pose_completion_verified=False,
            supported_actions=['prepare_test', 'stand', 'lie', 'move_mode', 'stopmotion'],
            stop_semantics='close_our_worker_physical_stop_unverified',
            availability_detail={'production_backend': MODE,
                'explanation': 'Send one stand (1), lie (4), or experimental move_mode (17) request and observe fresh post-send state. Move mode requires stationary state 1 first; it sends no gait or velocity and does not establish navigation readiness. Worker closure does not stop the robot.'})
        return status

    def get_tool(self):
        tool = super().get_tool()
        schema = tool['inputSchema']
        schema['properties']['action']['enum'].remove('velocity')
        schema['properties']['action']['enum'].append('prepare_test')
        schema['properties']['action']['enum'].append('move_mode')
        schema['allOf'].append({'if': {'properties': {'action': {'const': 'move_mode'}}, 'required': ['action']},
            'then': {'not': {'anyOf': [{'required': [key]} for key in schema['properties']
                if key not in {'action', 'request_id', 'issued_at_ms', 'ttl_ms', *META_FIELDS}]}}})
        schema['allOf'].append({'if': {'properties': {'action': {'const': 'prepare_test'}}, 'required': ['action']},
            'then': {'not': {'anyOf': [{'required': [key]} for key in schema['properties']
                if key not in {'action', *META_FIELDS}]}}})
        schema['x-completion'] = {'actions': list(POSTURE_STATES), 'timeout': 90, 'silent_completion': False,
            'completion_level': COMPLETION_LEVEL,
            'description': 'Completion means fresh post-send reported target state observed; physical pose completion is not independently verified.'}
        schema['x-action-params'].update({
            'move_mode': {'params': [], 'description': '移动模式（实验性）：机器狗已静止站定且反馈为 state 1 后，单次发送原生 state 17。不自动站立、不切步态、不发送速度。'},
            'prepare_test': {'params': [], 'description': 'Prepare the isolated command worker after operator authorization. Sends no robot command.'},
            'stand': {'params': [], 'description': 'Send stand request (state 1) once and automatically observe the reported target state. Completion means reported state observed, not independently verified physical pose completion. No RL transition.'},
            'lie': {'params': [], 'description': 'Send lie request (state 4) once, then wait until the vendor state machine finishes LieDown and reports final idle state 0. Intermediate state 4 is not completion.'},
            'velocity': {'params': [], 'description': 'Unavailable in supervised posture test mode.'},
            'stopmotion': {'params': [], 'description': 'Cancel the test and close our worker. Physical state becomes unknown; use the on-site operator and handheld. This does not claim the robot stopped.'}})
        tool.update(title='Posture Requests',
            description='Stand, lie, or experimental move_mode (native state 17, stand first). Each click sends one state only. No gait or velocity is sent. Existing on-site supervision applies.')
        return tool

    def dispatch(self, action, args):
        if action == 'prepare_test':
            validate_empty_arguments(args)
            with self._lock:
                if self._active is not None:
                    return {'accepted': False, 'state': 'busy'}
                allowed = self._backend.authorization.public_status().get('allowed_actions', [])
                if self._gate() or not self._started or not self._settings['enabled'] or not allowed:
                    return self._unavailable(['operator_session_unavailable'])
                try:
                    self._backend.prepare_execution(allowed[0])
                except CommandChannelNotReady:
                    return self._unavailable(['command_receiver_unmatched'])
                return {'prepared': True, 'physical_command_sent': False,
                    'command_worker_running': self._backend.transport.status().get('worker_running') is True}
        return super().dispatch(action, args)

    def execute(self, action, arguments):
        reply = super().execute(action, arguments)
        if reply.get('state') == 'accepted' and not reply.get('replayed'):
            reply.update(completion_level=COMPLETION_LEVEL,
                completion_basis=COMPLETION_BASIS, physical_pose_completion_verified=False)
        if reply.get('state') == 'busy':
            with self._lock:
                active = copy.deepcopy(self._active)
            if active is not None:
                reply.update(reason='active_posture_task_in_progress',
                    active_request_id=active['request_id'], active_action=active['action'],
                    instruction='The current posture request is awaiting reported target state. Wait for its result before sending another posture request.')
        return reply

    def _observation_failure(self, action):
        """Keep distinct causes instead of labelling every timeout session loss."""
        if self._cancel.is_set():
            return self._cancel_reason or 'operator_cancelled'
        if not self._started:
            return 'card_stopped'
        safety = self._backend.safety_status()
        if not safety['operator_authorized']:
            return 'operator_session_unavailable'
        if not safety['feedback_fresh']:
            return 'motion_feedback_stale'
        if not safety['hard_estop_fresh']:
            return 'hard_estop_feedback_stale'
        if not safety['hard_estop_clear']:
            return 'hard_estop_triggered'
        if self._gate(action) or not self._control_ok(action):
            return 'supervised_conditions_unavailable'
        return None

    def _run_action(self, record, request, started):
        action, rid, deadline = request['action'], request['request_id'], request['expires_monotonic']
        receipt, completed, reason = {}, False, 'reported_state_timeout'
        observed = {}
        try:
            with self._io_lock:
                if self._cancel.is_set() or self._gate(action) or not self._control_ok(action) or not self._started:
                    raise PermissionError('supervised conditions lost before send')
                if self._wall() * 1000 > request['issued_at_ms'] + request['ttl_ms']:
                    raise TimeoutError('request expired before send')
                receipt = self._backend.send(action, request, self._cancel)
                self._last_command = self._clock()
            with self._lock:
                self._phase = 'awaiting_reported_motion_state'
            self._event('sent', request_id=rid, action_id=record['action_id'],
                        command_accepted=None, confirmed=False, mode=MODE)
            while self._clock() < deadline:
                failure = self._observation_failure(action)
                if failure:
                    reason = failure
                    break
                observed = self._backend.target_feedback(action, rid)
                with self._lock:
                    if (observed.get('confirmed') is True and observed.get('fresh') is True
                            and not self._cancel.is_set() and not self._gate(action) and self._control_ok(action)):
                        completed, reason = True, 'target_state_observed'
                        break
                self._cancel.wait(.02)
        except CommandChannelNotReady:
            reason = 'command_receiver_unmatched'
        except Exception as exc:
            reason = 'supervised_execution_failed_' + type(exc).__name__
        finally:
            with self._lock:
                failure = self._observation_failure(action)
                if failure or self._clock() >= deadline:
                    was_completed = completed
                    completed = False
                    # A specific send failure must survive a coincident deadline.
                    if failure:
                        reason = failure
                    elif reason == 'reported_state_timeout' or was_completed:
                        reason = 'reported_state_timeout'
                if not completed:
                    self._backend.abort()
                result = {'request_id': rid, 'action_id': record['action_id'], 'action': action,
                    'accepted': True, 'sent': receipt.get('sent') is True,
                    'command_accepted': None, 'confirmed': completed, 'completed': completed,
                    'stop_confirmed': False, 'state': 'completed' if completed else 'error',
                    'status': 'completed' if completed else 'error', 'reason': reason,
                    'mode': MODE, 'completion_level': COMPLETION_LEVEL,
                    'completion_basis': COMPLETION_BASIS if completed else None,
                    'reported_state_confirmed': completed,
                    'reported_command_state': POSTURE_STATES[action] if receipt.get('sent') is True else None,
                    'reported_target_state': COMPLETION_STATES[action] if completed else None,
                    'reported_state_evidence': {key: observed.get(key) for key in (
                        'sequence', 'received_monotonic', 'age_s', 'motion_state',
                        'matching_samples', 'required_samples', 'correlation', 'fresh', 'reason')} if observed else None,
                    'command_receipt': {key: receipt[key] for key in (
                        'topic', 'frame_id', 'sent_monotonic', 'matched_subscribers') if key in receipt},
                    'physical_pose_completion_verified': False, 'physical_state_unknown': True,
                    'operator_intervention_required': not completed,
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
