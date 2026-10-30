"""Unified locomotion UI and one durable, feedback-gated stand-up sequence."""
from __future__ import annotations
import copy
import hashlib
try:
    from .canvas_posture import CanvasPostureBackend, CanvasPosturePlugin, DISTANCE_ACTION, RELATIVE_ACTION, RELATIVE_MOVE_TOOL
    from .motion import MotionPlugin
    from .parameters import validate_empty_arguments, validate_motion_request
except ImportError:
    from canvas_posture import CanvasPostureBackend, CanvasPosturePlugin, DISTANCE_ACTION, RELATIVE_ACTION, RELATIVE_MOVE_TOOL
    from motion import MotionPlugin
    from parameters import validate_empty_arguments, validate_motion_request

LOCO_TOOL, STAND_UP, SIT_DOWN = 'loco_move', 'stand_up', 'sitdown'
PUBLIC_ACTIONS = ('info', 'start', 'stop', 'status', 'config', STAND_UP, SIT_DOWN, 'move', 'stop_move')
ACTION_NAMES = {'lie': SIT_DOWN, 'stand_down': SIT_DOWN, DISTANCE_ACTION: 'move'}
MOVE_FIELDS = frozenset(('direction', 'speed', 'duration'))

class StandUpFailure(RuntimeError):
    def __init__(self, reason, blocked_by=()):
        super().__init__(reason)
        self.reason, self.blocked_by = reason, list(blocked_by)

class LocoMoveBackend(CanvasPostureBackend):
    def prepare_execution(self, action):
        if action == STAND_UP:
            self._require_ready('stand')
            self._require_ready('move_mode')
            return super().prepare_execution('stand')
        return super().prepare_execution(action)

class LocoMovePlugin(CanvasPosturePlugin):
    name = LOCO_TOOL
    execution_actions = (*CanvasPosturePlugin.execution_actions, STAND_UP)

    def __init__(self, config, *args, **kwargs):
        # An existing site/card override remains binding. Defaults do not
        # change the selected command speed or grant physical authority.
        config = {'max_vx': 1., 'max_yaw_rate': 1., **(config or {})}
        super().__init__(config, *args, **kwargs)
        base = self.status_topic.rsplit('/', 1)[0]
        self.status_topic = self.timed_status_topic = base + '/loco_move_status'
        self.events_topic = self.timed_events_topic = base + '/loco_move_events'

    def _ports(self):
        # Status remains available through info/status; completion uses ACP.
        return []

    def get_tool(self):
        posture = super().get_tool()
        timed = self._timed_move_tool()['inputSchema']
        # Canvas renders fields in schema insertion order; action stays first
        # while x-action-params reveals movement fields underneath it.
        props = {'action': {'type': 'string', 'enum': list(PUBLIC_ACTIONS),
                            'description': 'Locomotion action; sitdown keeps the original lie-down behavior. Wait for ACP after an accepted motion instead of polling.'}}
        props.update({k: copy.deepcopy(v) for k, v in posture['inputSchema']['properties'].items()
                      if k in ('request_id', 'issued_at_ms', 'ttl_ms', 'instance_id', '_tool_name')})
        props.update(copy.deepcopy(timed['properties']))
        props.update(copy.deepcopy(posture['configSchema']['properties']))
        props['config'] = copy.deepcopy(posture['configSchema'])
        metadata = {'request_id', 'issued_at_ms', 'ttl_ms', 'instance_id', '_tool_name', 'action'}
        groups = [(['move'], metadata | {'direction', 'speed', 'duration'}),
                  ([STAND_UP, SIT_DOWN, 'stop_move'], metadata),
                  (['start', 'config'], metadata | set(posture['configSchema']['properties']) | {'config'}),
                  (['info', 'status', 'stop'], {'action', 'instance_id', '_tool_name'})]
        conditions = []
        for actions, allowed in groups:
            # Core's canvas submits nonempty fields even when x-action-params
            # hides them. These three fields have meaning only for move.
            allowed = allowed | MOVE_FIELDS
            disallowed = [{'required': [k]} for k in props if k not in allowed]
            rule = {'not': {'anyOf': disallowed}} if disallowed else {}
            if actions == ['move']: rule['required'] = ['direction', 'speed', 'duration']
            conditions.append({'if': {'properties': {'action': {'enum': actions}}, 'required': ['action']}, 'then': rule})
        schema = {'type': 'object', 'properties': props, 'required': ['action'],
                  'additionalProperties': False, 'allOf': conditions,
                  'x-action-params': {
                      STAND_UP: {'params': [], 'description': 'Stand up, confirm fresh stable state 1, then enter move mode 17. Already-ready state 17 sends no command. No velocity is sent.'},
                      SIT_DOWN: {'params': [], 'description': 'Lie down and wait for final idle state 0.'},
                      'move': {'params': ['direction', 'speed', 'duration'], 'description': 'Finite forward/backward movement or turn; requires stationary move mode and confirms stopping.'},
                      'stop_move': {'params': [], 'description': 'Cancel the active application task. Finite movement sends zero velocity and checks stopping; cancelling posture is not a physical stop guarantee.'}},
                  'x-resource': 'base', 'x-completion': {
                      'actions': [STAND_UP, SIT_DOWN, 'move', 'stand_down'], 'timeout': 20, 'silent_completion': False,
                      'description': 'Returns action_id after acceptance; completion arrives once through ACP. Do not poll or repeat an accepted command. stand_up completes only at reported move mode.'}}
        return {'name': LOCO_TOOL, 'title': 'S10 Locomotion', 'type': 'actuator', 'multiInstance': False,
                'description': 'Stand up, sitdown (original lie-down), finite move, or cancel. Motion returns action_id and completes through ACP; do not poll after acceptance. status is concise; info is detailed diagnostics. Direction/speed/duration apply only to move.',
                'inputSchema': schema, 'configSchema': posture['configSchema'], 'topic_in': [], 'topic_out': self._ports()}

    @staticmethod
    def _action_card(action):
        return RELATIVE_MOVE_TOOL if action == RELATIVE_ACTION else LOCO_TOOL

    def _notification_payload(self, record):
        payload = super()._notification_payload(record)
        payload['tool'] = RELATIVE_MOVE_TOOL if record['request']['action'] == RELATIVE_ACTION else LOCO_TOOL
        payload['result'] = (self._public_record(payload['result']) if payload['tool'] == RELATIVE_MOVE_TOOL
                             else self._display_result(payload['result'], record['request']['action']))
        return payload

    def get_tools(self):
        return [self.get_tool(), self._relative_move_tool()]

    @staticmethod
    def _public_record(record):
        result = copy.deepcopy(record)
        if isinstance(result, dict) and result.get('action') in ACTION_NAMES:
            result['action'] = ACTION_NAMES[result['action']]
        return result

    def _loco_status(self, source=None):
        result = copy.deepcopy(source if source is not None else self.status())
        result.update(card=LOCO_TOOL, title='S10 Locomotion', topic_out=self._ports(),
                      supported_actions=[STAND_UP, SIT_DOWN, 'move', 'stop_move'])
        result['events'] = [self._public_record(e) for e in result['events'] if e.get('card') != RELATIVE_MOVE_TOOL]
        active = result.get('active_request')
        result['shared_controller_busy'] = active is not None
        result['active_request'] = None if active and active['action'] == RELATIVE_ACTION else self._public_record(active)
        last = result.get('last_result')
        result['last_result'] = (self._public_record(last) if not last or last['action'] != RELATIVE_ACTION
                               else next((e for e in reversed(result['events']) if e.get('event') == 'finished'), None))
        readiness = result['readiness']; old = readiness['actions']
        blockers = list(readiness['blocked_by'])
        if not {'stand', 'move_mode'} <= set(self._backend.authorization.public_status().get('allowed_actions', [])):
            blockers.append('stand_and_move_mode_authorization_required')
        if self._backend.feedback.observation().get('motion_state') not in (0, 1, 17):
            blockers.append('wait_for_idle_required')
        readiness['actions'] = {STAND_UP: {**old['stand'], 'ready': not blockers, 'blocked_by': list(dict.fromkeys(blockers))},
                                SIT_DOWN: old['lie'], 'move': old[DISTANCE_ACTION]}
        return result

    def _publish_status(self, payload):
        self._publish(self.status_topic, 'data/json', self._loco_status(payload))
        self._publish(self.relative_status_topic, 'data/json', self._card_status(payload, card=RELATIVE_MOVE_TOOL))

    def _control_ok(self, action=None):
        if action == STAND_UP:
            return all(super(LocoMovePlugin, self)._control_ok(stage) for stage in ('stand', 'move_mode'))
        return super()._control_ok(action)

    def dispatch(self, action, args):
        if action == 'stand_down':
            action = SIT_DOWN  # Saved canvases retain the old alias.
        if args.get('_tool_name') == RELATIVE_MOVE_TOOL: return super().dispatch(action, args)
        if args.get('_tool_name') not in (None, LOCO_TOOL): raise ValueError('Retired tool; migrate to loco_move')
        if action not in PUBLIC_ACTIONS and action not in ('prepare', 'prepare_automatic'):
            raise ValueError('Unsupported loco_move action')
        if action in PUBLIC_ACTIONS and action != 'move':
            # Do not let hidden values left by a previous move selection block
            # posture, cancellation, status or lifecycle calls. All remaining
            # parameters still pass through the original strict validators.
            args = {key: value for key, value in args.items() if key not in MOVE_FIELDS}
        if action in ('info', 'status'):
            validate_empty_arguments(args)
            status = self._loco_status()
            return status if action == 'info' else self._summary(status)
        if action in ('config', 'start', 'stop'):
            super().dispatch(action, args); return self._summary(self._loco_status(), include_settings=True)
        if action == 'stop_move':
            validate_motion_request('stopmotion', args, self._settings, self._wall); return self._display_result(self.cancel(), action)
        if action == STAND_UP: return self._display_result(self.execute(STAND_UP, args), action)
        if action == SIT_DOWN: return self._display_result(self.execute('lie', args), action)
        if action == 'move': return self._display_result(self.execute(DISTANCE_ACTION, args), action)
        if args.get('_tool_name') is not None: raise ValueError('Prepare is private to the supervisor')
        return super().dispatch(action, args)

    @staticmethod
    def _blocker_text(blockers):
        messages = {
            'card_not_enabled': 'Enable and start the card first.',
            'operator_session_unavailable': 'Waiting for an authorized Prepare session.',
            'fresh_clear_fault_snapshot_required': 'Robot fault feedback is missing, stale or reports a fault; check robot diagnostics.',
            'driver_error_requires_review': 'The previous failure requires review before another motion.',
            'operator_intervention_required': 'Review the previous failed motion and confirm the robot is stationary.',
            'fresh_stationary_feedback_required': 'Waiting for fresh stationary feedback.',
            'stationary_move_mode_required': 'Movement requires stationary move mode.',
            'move_mode_required': 'Run stand_up and wait for its ACP completion first.',
            'active_motion_task_in_progress': 'A motion is still running; wait for ACP or cancel it.',
        }
        return ' '.join(dict.fromkeys(messages.get(b, b.replace('_', ' ') + '.') for b in blockers))

    def _display_result(self, record, action):
        """Keep protocol identity and outcome; leave full evidence in durable info."""
        record = self._public_record(record)
        keys = ('action_id', 'original_action_id', 'request_id', 'action', 'accepted',
                'state', 'status', 'completed', 'confirmed', 'stop_confirmed',
                'reason', 'blocked_by', 'failed_stage', 'failure_phase', 'replayed', 'terminal', 'physical_command_sent')
        result = {k: record[k] for k in keys if k in record}
        result.setdefault('action', ACTION_NAMES.get(action, action))
        if result.get('action_id') and result.get('status') == 'running':
            result['message'] = 'Command accepted. Wait for its ACP completion; do not poll or resend.'
        elif result.get('completed'):
            result['message'] = 'Command completed; required robot feedback was confirmed.'
        elif result.get('state') == 'busy':
            result['message'] = 'Another motion is running. Wait for ACP or use stop_move.'
        elif result.get('blocked_by'):
            result['message'] = self._blocker_text(result['blocked_by'])
        elif result.get('reason') == 'reported_stop_not_confirmed':
            result['message'] = 'Stop was not confirmed. Check the robot and use info for diagnostics.'
        else:
            result['message'] = str(result.get('reason', result.get('state', 'Command result'))).replace('_', ' ')
        return result

    def _summary(self, status, *, include_settings=False):
        readiness = status['readiness']
        blocked = readiness['blocked_by']
        result = {'card': LOCO_TOOL, 'state': status['state'],
                'enabled': status['settings']['enabled'], 'ready': readiness['ready'],
                'topic_out': [],
                'active_request': ({k: self._public_record(status['active_request']).get(k)
                                    for k in ('action_id', 'request_id', 'action')}
                                   if status.get('active_request') else None),
                'last_result': (self._display_result(status['last_result'], status['last_result']['action'])
                                if status.get('last_result') else None),
                'blocked_by': blocked,
                'message': self._blocker_text(blocked) if blocked else 'Ready. Motion completion is reported through ACP.'}
        if include_settings:
            result['settings'] = status['settings']
        return result

    def execute(self, action, arguments):
        if action != STAND_UP: return super().execute(action, arguments)
        args = dict(arguments or {})
        with self._lock:
            request = validate_motion_request(action, args, self._settings, self._wall,
                check_freshness=False, allowed_actions=self.execution_actions)
            if self._tasks.get(request['request_id']): return MotionPlugin.execute(self, action, args)
            if self._active is not None: return {'accepted': False, 'state': 'busy', 'reason': 'active_motion_task_in_progress'}
            if self._closed.is_set() or not self._started or not self._settings['enabled']:
                return self._unavailable(['card_not_enabled'], action)
            if self._error or self._backend.safety_status()['failed_closed']:
                return self._unavailable(['driver_error_requires_review'], action)
            self._backend.start_observation()
            check = self._backend.preflight(stationary_states=(0, 1, 17))
            if self._backend.feedback.observation().get('motion_state') not in (0, 1, 17):
                check['blocked_by'].append('wait_for_idle_required')
            if check['blocked_by']: return self._check_unavailable(check, action)
            return MotionPlugin.execute(self, action, {**args, 'request_id': request['request_id']})

    def _run_action(self, record, request, started):
        if request['action'] != STAND_UP: return super()._run_action(record, request, started)
        rid, deadline = request['request_id'], request['expires_monotonic']
        stages, stage_ids = [], []
        completed, reason, sent = False, 'stand_up_incomplete', False
        stage, phase, blocked_by, error_type = None, 'plan', [], None
        try:
            state = self._backend.feedback.observation().get('motion_state')
            if state not in (0, 1, 17): raise StandUpFailure('unexpected_posture_state')
            plan = ('stand', 'move_mode') if state == 0 else ('move_mode',) if state == 1 else ()
            if not plan:
                check = self._backend.preflight(stationary_states=(17,))
                if check['blocked_by'] or self._observation_failure(STAND_UP):
                    raise StandUpFailure('move_mode_not_stationary', check['blocked_by'])
            for stage in plan:
                phase = 'preflight'
                if self._clock() >= deadline or self._cancel.is_set() or self._observation_failure(STAND_UP):
                    raise StandUpFailure('sequence_conditions_lost')
                check = self._backend.preflight(stationary_states=(0,) if stage == 'stand' else (1,))
                if check['blocked_by']: raise StandUpFailure('stage_preflight_unavailable', check['blocked_by'])
                sid = 'loco_' + hashlib.sha256((rid + ':' + stage).encode()).hexdigest()
                stage_ids.append(sid)
                child = {**request, 'request_id': sid, 'action': stage}
                with self._io_lock:
                    if self._cancel.is_set() or self._clock() >= deadline or self._observation_failure(STAND_UP):
                        raise StandUpFailure('sequence_conditions_lost')
                    phase = 'prepare_channel'
                    self._backend.prepare_execution(stage)
                    if self._clock() >= deadline or self._cancel.is_set(): raise TimeoutError('Stage expired')
                    phase = 'send'
                    receipt = self._backend.send(stage, child, self._cancel)
                    sent = sent or receipt.get('sent') is True
                    self._last_command = self._clock()
                self._event('stage_sent', action=STAND_UP, stage=stage, request_id=rid, action_id=record['action_id'])
                confirmed = False
                phase = 'wait_feedback'
                while self._clock() < deadline:
                    failure = self._observation_failure(STAND_UP)
                    if failure: reason = failure; break
                    observed = self._backend.target_feedback(stage, sid)
                    if observed.get('confirmed') is True and observed.get('fresh') is True:
                        confirmed = True; break
                    self._cancel.wait(.02)
                if not confirmed: raise StandUpFailure('stage_feedback_unconfirmed')
                stages.append({'action': stage, 'reported_target_state': 1 if stage == 'stand' else 17, 'confirmed': True})
                self._backend.finish(sid)
            completed = not self._observation_failure(STAND_UP) and self._clock() < deadline
            reason = 'move_mode_ready' if completed else 'sequence_conditions_lost'
        except Exception as exc:
            error_type = type(exc).__name__
            blocked_by = exc.blocked_by if isinstance(exc, StandUpFailure) else []
            reason = self._observation_failure(STAND_UP) or (
                exc.reason if isinstance(exc, StandUpFailure) else 'stage_' + phase + '_failed')
        finally:
            if self._cancel.is_set() or self._clock() >= deadline or self._observation_failure(STAND_UP):
                completed = False; reason = self._observation_failure(STAND_UP) or 'stand_up_deadline_exceeded'
            if not completed: self._backend.abort()
            for sid in stage_ids: self._backend.finish(sid)
            result = {'request_id': rid, 'action_id': record['action_id'], 'action': STAND_UP,
                      'accepted': True, 'completed': bool(completed), 'confirmed': bool(completed),
                      'state': 'completed' if completed else 'error', 'status': 'completed' if completed else 'error',
                      'reason': reason, 'stages': stages, 'physical_command_sent': sent, 'stop_confirmed': False,
                      'reported_target_state': 17 if completed else None, 'completion_level': 'reported_motion_state',
                      'physical_pose_completion_verified': False, 'finished_at': self._wall()}
            if not completed:
                result.update(failed_stage=stage, failure_phase=phase,
                              blocked_by=blocked_by, error_type=error_type)
            try:
                if self._tasks.finish(rid, result):
                    with self._lock:
                        self._last, self._phase = result, result['state'] if self._started else 'idle'
                        self._event('finished', **result)
            except Exception: self._error = 'terminal_record_failed_restart_recovery_required'
            finally:
                with self._lock: self._active = None
                if not self._started: self._backend.stop_observation()
                self._busy.release()
            self._flush_notifications()
