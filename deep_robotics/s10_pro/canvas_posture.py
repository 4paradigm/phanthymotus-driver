"""Canvas checks, explicit prepare session renewal, and idle worker recovery."""
from __future__ import annotations

import copy
import math
import threading
import time

try:
    from .supervised_motion import SupervisedMotionPlugin, SupervisedPostureBackend, POSTURE_STATES
    from .motion_dds import CommandChannelNotReady
    from .parameters import META_FIELDS, validate_empty_arguments
    from .stationary_feedback import StationaryWindow
except ImportError:
    from supervised_motion import SupervisedMotionPlugin, SupervisedPostureBackend, POSTURE_STATES
    from motion_dds import CommandChannelNotReady
    from parameters import META_FIELDS, validate_empty_arguments
    from stationary_feedback import StationaryWindow


INSTRUCTIONS = {
    'command_channel_not_prepared': '通道尚未准备；点击站立/卧倒时会自动准备健康通道，不需要先发动作解锁。',
    'operator_session_unavailable': '操作会话尚未建立或已到期；点击 prepare，通过状态检查后建立新的60分钟会话。',
    'joints_not_enabled': '关节状态未恢复；请使用厂家遥控器正常唤醒，不要重复发送站立。',
    'joint_data_not_advancing': '关节数据未持续更新，可能仍在休眠；DDS消息新鲜不代表电机在线。',
    'fresh_joint_feedback_required': '等待新鲜、完整的16关节反馈。',
    'fresh_clear_fault_snapshot_required': '故障快照过期或有活动故障，禁止发送。',
    'stationary_idle_or_standing_required': '需要在状态0、1或17连续稳定0.3秒：平移合速度低于0.05m/s、转动低于0.06rad/s。',
    'stand_first_required': '移动模式必须在机器狗已经静止站定、反馈为 state 1 时使用。',
    'fresh_stationary_feedback_required': '移动模式需要新鲜且速度为零的运动反馈。',
    'already_in_move_mode': '当前已经处于移动模式 state 17，无需重复进入；退出请使用 lie。',
    'exit_move_with_lie_required': '厂家状态机在 state 17 会忽略 stand；请先使用 lie，等待回到 state 0 后再 stand。',
    'wait_for_idle_required': '机器狗仍在姿态切换中；请等待 lie 完整回到 state 0 后再执行下一动作。',
    'unsupported_transition_state': '当前厂家运动状态不支持这个姿态切换，未发送指令。',
    'operator_intervention_required': '上次动作失败已锁定；下次新动作点击会在静止和无故障检查通过后自动恢复，不会重发旧动作。',
    'move_mode_required': '定时移动只能在 move_mode（state 17）中执行。',
    'stationary_move_mode_required': '开始定时移动前，需在move_mode中连续稳定0.3秒：平移合速度低于0.05m/s、转动低于0.06rad/s。',
    'timed_command_channel_not_prepared': '定时移动速度通道尚未准备，未发送移动指令。',
}


DISTANCE_ACTION = 'move_2s'
DISTANCE_ACTIONS = (DISTANCE_ACTION,)
TIMED_MOVE_TOOL = 'timed_move'
DISTANCE_DIRECTIONS = {'forward': 1., 'backward': -1.}
DISTANCE_SCOPES = {'forward': 'forward_30cm', 'backward': 'backward_30cm'}
TIMED_SPEED_MPS = .08
TIMED_DURATION_S = 2.
TIMED_RESPONSE_MPS = .05


def odometry_observation(record, now):
    """Parse ODOM for monitoring compatibility; timed moves do not depend on it."""
    result = {'fresh': False, 'sequence': 0, 'x': None, 'y': None, 'yaw': None,
              'received_monotonic': None, 'age_s': None, 'reason': 'fresh_odometry_required'}
    if not isinstance(record, dict) or not fresh(record, now, .35):
        return result
    raw = record.get('data')
    pose = raw.get('pose') if isinstance(raw, dict) else None
    pose = pose.get('pose') if isinstance(pose, dict) else None
    position = pose.get('position') if isinstance(pose, dict) else None
    orientation = pose.get('orientation') if isinstance(pose, dict) else None
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return result
    twist = raw.get('twist', {})
    twist = twist.get('twist', {}) if isinstance(twist, dict) else {}
    linear = twist.get('linear', {}) if isinstance(twist, dict) else {}
    speeds = [linear.get(key) for key in ('x', 'y', 'z')] if isinstance(linear, dict) else []
    if len(speeds) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in speeds):
        return result
    if math.sqrt(sum(v * v for v in speeds)) > .30:
        return {**result, 'reason': 'odometry_velocity_inconsistent'}
    values = [position.get('x'), position.get('y'), orientation.get('x'),
              orientation.get('y'), orientation.get('z'), orientation.get('w')]
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
        return result
    norm = math.sqrt(sum(value * value for value in values[2:]))
    if not .5 <= norm <= 1.5:
        return result
    qx, qy, qz, qw = (value / norm for value in values[2:])
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    sequence = record.get('sequence')
    if type(sequence) is not int or sequence <= 0:
        return result
    return {'fresh': True, 'sequence': sequence, 'x': float(values[0]),
            'y': float(values[1]), 'yaw': yaw,
            'received_monotonic': float(record['received_monotonic']),
            'age_s': float(record['age_s'])}


def fresh(record, now, limit):
    age, received = record.get('age_s'), record.get('received_monotonic')
    return (record.get('fresh') is True and not record.get('error')
            and type(age) in (float, int) and math.isfinite(age) and 0 <= age <= limit
            and type(received) in (float, int) and math.isfinite(received)
            and 0 <= now - received <= limit and abs(now - received - age) <= .1)


class CanvasPostureBackend(SupervisedPostureBackend):
    def __init__(self, *args, transport_factory=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._transport_factory = transport_factory
        self.last_preflight = None
        self._distance_channel = {'publisher_prepared': False, 'matched_subscribers': 0,
                                  'ready': False, 'checked_monotonic': None}
        self._distance_permit = None
        self._stationary = StationaryWindow(clock=self._clock)
        self._distance_stop_started = False

    def start_observation(self):
        super().start_observation()
        self.bridge.acquire('canvas_posture_preflight', {'joints', 'fault_aggregator'},
                            {'refresh_hz': 10., 'stale_after_s': 2.})

    def stop_observation(self):
        self.bridge.release('canvas_posture_preflight')
        super().stop_observation()

    def snapshot_checks(self, *, require_authorization=True):
        blocked, ids = [], None
        try:
            joints = self.bridge.snapshot('joints') or {}
            faults = self.bridge.snapshot('fault_aggregator') or {}
        except Exception:
            joints, faults = {}, {}
        now = self._clock()
        raw = joints.get('data')
        data = raw.get('data', {}) if isinstance(raw, dict) else {}
        rows = data.get('joints_data') if isinstance(data, dict) else None
        if (not fresh(joints, now, .5) or not isinstance(rows, list) or len(rows) != 16
                or any(not isinstance(r, dict) or type(r.get('data_id')) is not int
                       or not isinstance(r.get('name'), (list, str)) for r in rows)):
            blocked.append('fresh_joint_feedback_required')
        else:
            ids = {str(r['name']): r['data_id'] for r in rows}
            if len(ids) != 16:
                blocked.append('fresh_joint_feedback_required')
            # Empirical allow-list from this robot, not a generic bit decoder.
            if any(type(r.get('status_word')) is not int or r['status_word'] != 1 for r in rows):
                blocked.append('joints_not_enabled')
        fault_data = faults.get('data')
        if (not fresh(faults, now, 2.) or not isinstance(fault_data, dict)
                or fault_data.get('active_faults') != []):
            blocked.append('fresh_clear_fault_snapshot_required')
        safety = self.safety_status()
        if require_authorization and not safety['operator_authorized']:
            blocked.append('operator_session_unavailable')
        if not safety['feedback_fresh']:
            blocked.append('fresh_motion_feedback')
        if not safety['hard_estop_clear']:
            blocked.append('fresh_clear_hard_estop_required')
        return blocked, ids

    def stationary_status(self, states=(0, 1, 17)):
        return self._stationary.observe(self.feedback.observation(), states)

    def wait_stationary(self, states, timeout_s=.65):
        deadline = self._clock() + timeout_s
        while True:
            result = self.stationary_status(states)
            if result['ready'] or self._clock() >= deadline:
                return result
            time.sleep(.04)

    def transition_blockers(self, action):
        blocked = super().transition_blockers(action)
        if action == 'move_mode' and not self.stationary_status((1,))['ready']:
            blocked.append('fresh_stationary_feedback_required')
        return list(dict.fromkeys(blocked))

    def preflight(self, *, recovering=False, require_authorization=True, stationary_states=None):
        """Two read-only observations; no publication, wake or permission change."""
        self.stationary_status()
        blocked, first = self.snapshot_checks(require_authorization=require_authorization)
        if not blocked:
            time.sleep(.25)
            blocked, second = self.snapshot_checks(require_authorization=require_authorization)
            if first is None or second is None or first.keys() != second.keys() or any(
                    second[key] == value for key, value in first.items()):
                blocked.append('joint_data_not_advancing')
        states = (0, 1, 17) if recovering else stationary_states
        stationary = self.wait_stationary(states) if states is not None and not blocked else self.stationary_status()
        if states is not None and not stationary['ready']:
            blocked.append('stationary_idle_or_standing_required' if recovering else
                'stationary_move_mode_required' if states == (17,) else 'fresh_stationary_feedback_required')
        self.last_preflight = {'ready': not blocked, 'blocked_by': list(dict.fromkeys(blocked)),
            'checked_at_ms': int(time.time() * 1000), 'physical_command_sent': False,
            'automatic_wake': False, 'automatic_authorization_renewal': False,
            'stationary_check': stationary}
        return copy.deepcopy(self.last_preflight)

    def prepare_operator_session(self):
        """Idle explicit prepare: check first, renew, replace old private channel."""
        with self._lock:
            if self._closed or self._request is not None or self._transport_factory is None:
                raise RuntimeError('prepare session requires an idle open backend')
            # A restarted read-only observer needs bounded time to receive all
            # sources. Never create a session from missing/stale telemetry.
            deadline = self._clock() + 2.
            while True:
                check = self.preflight(recovering=True, require_authorization=False)
                if check['ready'] or self._clock() >= deadline:
                    break
                time.sleep(.1)
            if not check['ready']:
                return check
            self.authorization.prepare_session()
            try:
                # A new session must never reuse receipts from an old worker.
                self.transport.close()
                self.transport = self._transport_factory()
                self._failed = False
                self._channel = {'publisher_prepared': False, 'matched_subscribers': 0,
                                 'ready': False, 'checked_monotonic': None}
                self._distance_channel = {'publisher_prepared': False, 'matched_subscribers': 0,
                                         'ready': False, 'checked_monotonic': None}
                allowed = self.authorization.public_status().get('allowed_actions', [])
                self.prepare_execution(allowed[0])
            except Exception:
                self.authorization.clear_session('prepare_failed_retry_required')
                self.abort()
                raise
            return {**check, 'authorization_renewed': True,
                'operator_session': self.authorization.public_status(), 'old_action_retried': False}

    def _require_ready(self, action):
        super()._require_ready(action)
        blocked, _ = self.snapshot_checks()
        if blocked:
            raise PermissionError('canvas preflight conditions changed before send: ' + ','.join(blocked))

    def distance_blockers(self, action):
        if action not in DISTANCE_ACTIONS:
            return ['unsupported_distance_action']
        blocked = []
        observed = self.feedback.observation()
        if not observed.get('fresh') or observed.get('motion_state') != 17:
            blocked.append('move_mode_required')
        if not self.stationary_status((17,))['ready']:
            blocked.append('stationary_move_mode_required')
        return list(dict.fromkeys(blocked))

    def _require_distance_ready(self, action, *, before_start=False):
        internal_scopes = set(DISTANCE_SCOPES.values())
        if (action not in DISTANCE_ACTIONS or not self.safety_status()['ready']
                or not internal_scopes <= set(self.authorization.public_status().get('allowed_actions', []))):
            raise PermissionError('timed-motion conditions are unavailable')
        blocked, _ = self.snapshot_checks()
        if before_start:
            blocked.extend(self.distance_blockers(action))
        else:
            observed = self.feedback.observation()
            if not observed.get('fresh') or observed.get('motion_state') != 17:
                blocked.append('move_mode_required')
        if blocked:
            raise PermissionError('timed-motion safety conditions changed: ' + ','.join(dict.fromkeys(blocked)))

    def prepare_execution(self, action):
        if action not in DISTANCE_ACTIONS:
            return super().prepare_execution(action)
        with self._lock:
            self._require_distance_ready(action, before_start=True)
            status = self.transport.status()
            if status.get('closed') is True:
                self._failed = True
                raise RuntimeError('closed supervised worker cannot restart')
            if status.get('worker_running') is not True:
                try:
                    self.transport.start(timeout_s=5.)
                    self._require_distance_ready(action, before_start=True)
                except Exception:
                    self.abort()
                    raise
            deadline = self._clock() + 2.
            while True:
                self._require_distance_ready(action, before_start=True)
                channel = self.transport.graph_snapshot().get('velocity', {})
                matched = channel.get('matched_subscribers')
                ready = channel.get('publisher_prepared') is True and type(matched) is int and matched > 0
                self._distance_channel = {'publisher_prepared': channel.get('publisher_prepared') is True,
                    'matched_subscribers': matched if type(matched) is int and matched >= 0 else 0,
                    'ready': ready, 'checked_monotonic': self._clock()}
                if ready:
                    return
                if self._clock() >= deadline:
                    raise CommandChannelNotReady('no matched NAV_CMD receiver after bounded preparation')
                time.sleep(.02)

    def distance_channel_status(self):
        with self._lock:
            result = copy.deepcopy(self._distance_channel)
            checked = result.pop('checked_monotonic')
            result['checked_age_s'] = None if checked is None else max(0., self._clock() - checked)
            result['receiver_acceptance_verified'] = False
            return result

    def begin_distance(self, action, request, cancel_event):
        with self._lock:
            self._require_distance_ready(action, before_start=True)
            rid, deadline = request.get('request_id'), request.get('expires_monotonic')
            if (not isinstance(rid, str) or not rid.strip() or rid in self._used
                    or self._request is not None or type(deadline) not in (int, float)
                    or not math.isfinite(deadline) or self._clock() >= deadline
                    or cancel_event.is_set()):
                raise RuntimeError('timed-motion request cannot start')
            direction = request.get('direction')
            if direction not in DISTANCE_DIRECTIONS:
                raise ValueError('invalid timed-motion direction')
            permit = self.authorization.permit(rid, DISTANCE_SCOPES[direction], deadline)
            self._used.add(rid)
            self._request = {'request_id': rid, 'action': action, 'direction': direction}
            self._distance_permit = permit
            self._sent_at = None
            self._distance_stop_started = False
            self._stationary.reset()
            return {'started': True, 'direction': direction}

    def send_distance_velocity(self, action, speed, request_id, cancel_event):
        with self._lock:
            if (not self._request or self._request.get('request_id') != request_id
                    or self._request.get('action') != action):
                raise RuntimeError('timed-motion request identity changed')
            self._require_distance_ready(action, before_start=False)
            direction = DISTANCE_DIRECTIONS[self._request['direction']]
            speed = float(speed)
            if speed != TIMED_SPEED_MPS:
                raise ValueError('timed-motion speed outside fixed validated value')
            receipt = self.transport.publish_velocity(direction * speed, 0., 0.,
                authority=self._distance_permit, cancel_event=cancel_event)
            if receipt.get('sent') is not True:
                raise RuntimeError('timed-motion velocity publication unconfirmed')
            if self._sent_at is None:
                self._sent_at = self._clock()
            return receipt

    def stop_distance(self, action, request_id):
        with self._lock:
            if (not self._request or self._request.get('request_id') != request_id
                    or self._request.get('action') != action):
                raise RuntimeError('timed-motion stop identity changed')
            if not self._distance_stop_started:
                self._stationary.reset()
                self._distance_stop_started = True
            return self.transport.publish_velocity(0., 0., 0., authority=self._distance_permit,
                                                    cancel_event=threading.Event())

    def distance_stopped(self):
        result = self.stationary_status((17,))
        return {'stopped': result['ready'], 'sequence': result['sequence'],
                'stationary_check': result}

    def finish(self, request_id):
        super().finish(request_id)
        with self._lock:
            self._distance_permit = None

    def recover_worker(self):
        """Only called by an explicit human-selected recover_test operation."""
        with self._lock:
            if self._closed or self._request is not None or self._transport_factory is None:
                raise RuntimeError('recovery unavailable for closed or active backend')
            check = self.preflight(recovering=True)
            if not check['ready']:
                return check
            self.transport.close()
            self.transport = self._transport_factory()
            self._failed = False
            self._channel = {'publisher_prepared': False, 'matched_subscribers': 0,
                             'ready': False, 'checked_monotonic': None}
            self._distance_channel = {'publisher_prepared': False, 'matched_subscribers': 0,
                                      'ready': False, 'checked_monotonic': None}
            allowed = self.authorization.public_status().get('allowed_actions', [])
            if not allowed:
                self.abort()
                raise PermissionError('no authorized posture action')
            try:
                self.prepare_execution(allowed[0])
            except Exception:
                self.abort()
                raise
            return {**check, 'recovered': True, 'old_action_retried': False}


class CanvasPosturePlugin(SupervisedMotionPlugin):
    execution_actions = (*POSTURE_STATES, *DISTANCE_ACTIONS)
    streaming_actions = tuple(DISTANCE_ACTIONS)

    def __init__(self, *args, **kwargs):
        autostart = kwargs.pop('autostart_worker', True)
        super().__init__(*args, autostart_worker=False, **kwargs)
        base = self.status_topic.rsplit('/', 1)[0]
        self.status_topic, self.events_topic = base + '/posture_status', base + '/posture_events'
        self.timed_status_topic = base + '/timed_move_status'
        self.timed_events_topic = base + '/timed_move_events'
        if autostart:
            self._thread.start()

    def _timed_ports(self):
        return [{'topic': self.timed_status_topic, 'format': 'data/json'},
                {'topic': self.timed_events_topic, 'format': 'data/json'}]

    def _event(self, kind, **values):
        with self._lock:
            action = values.get('action', values.get('requested_action'))
            if action is None and self._active:
                action = self._active.get('action')
            if action is None and values.get('request_id'):
                record = self._tasks.get(values['request_id'])
                action = record['request']['action'] if record else None
            timed = action in DISTANCE_ACTIONS
            event = {'card': TIMED_MOVE_TOOL if timed else 'motion', 'event': kind,
                     'timestamp_ms': int(self._wall() * 1000), **values}
            self._events.append(event)
            if self._started:
                try:
                    self._publish(self.timed_events_topic if timed else self.events_topic,
                                  'data/json', copy.deepcopy(event))
                except Exception:
                    self._error = 'status_publish_failed'

    def _card_status(self, source, *, timed):
        result = copy.deepcopy(source)
        card = TIMED_MOVE_TOOL if timed else 'motion'
        result['card'] = card
        result['title'] = 'S10 前进/后退（2秒验证版）' if timed else 'Posture Control'
        result['topic_out'] = self._timed_ports() if timed else self._ports()
        result['supported_actions'] = ['move'] if timed else ['prepare', *POSTURE_STATES]
        result['events'] = [e for e in result['events'] if e.get('card') == card]
        active = result.get('active_request')
        if active and (active.get('action') in DISTANCE_ACTIONS) != timed:
            result['active_request'] = None
            result['phase'] = 'ready' if result['readiness']['ready'] else 'not_ready'
        result['shared_controller_busy'] = active is not None
        last = result.get('last_result')
        if last and (last.get('action') in DISTANCE_ACTIONS) != timed:
            result['last_result'] = next((e for e in reversed(result['events'])
                if e.get('event') == 'finished'), None)
        readiness = result['readiness']
        readiness['actions'] = {key: value for key, value in readiness['actions'].items()
                               if (key in DISTANCE_ACTIONS) == timed}
        if timed:
            blockers = list(dict.fromkeys([*source['blocked_by'],
                *readiness['actions'][DISTANCE_ACTION]['blocked_by']]))
            result['blocked_by'] = readiness['blocked_by'] = blockers
            readiness['ready'] = result['execution_available'] = not blockers
            readiness['next_steps'] = [{'code': b, 'instruction': INSTRUCTIONS.get(b, b.replace('_', ' '))}
                                       for b in blockers]
            result['next_steps'] = copy.deepcopy(readiness['next_steps'])
        return result

    def _publish_status(self, payload):
        self._publish(self.status_topic, 'data/json', self._card_status(payload, timed=False))
        self._publish(self.timed_status_topic, 'data/json', self._card_status(payload, timed=True))

    def _gate(self, action=None, *, for_stop=False):
        blocked = super()._gate(None, for_stop=for_stop)
        if action is not None and action not in self.execution_actions:
            blocked.append('supervised_motion_only')
        return blocked

    def _control_ok(self, action=None):
        if action in DISTANCE_ACTIONS:
            return (set(DISTANCE_SCOPES.values()) <= set(
                        self._backend.authorization.public_status().get('allowed_actions', []))
                    and self._backend.safety_status()['ready']
                    and not self._backend.snapshot_checks()[0])
        return super()._control_ok(action) and not self._backend.snapshot_checks()[0]

    def status(self):
        result = super().status()
        result['supported_actions'] = [
            'prepare' if action == 'prepare_test' else action
            for action in result['supported_actions']
            if action not in ('check_test', 'recover_test', 'stopmotion')]
        for action in DISTANCE_ACTIONS:
            if action not in result['supported_actions']:
                result['supported_actions'].append(action)
        result['title'] = 'Posture & Validated 2s Move'
        result['execution_available'] = result['execution_available'] and result['readiness']['ready']
        return result

    def _readiness(self, **kwargs):
        requested_action = kwargs.get('action')
        result = super()._readiness(**kwargs)
        blocked, _ = self._backend.snapshot_checks()
        result['blocked_by'] = list(dict.fromkeys([*result['blocked_by'], *blocked]))
        result['ready'] = not result['blocked_by']
        result['next_steps'] = [{'code': b, 'instruction': INSTRUCTIONS.get(b, b.replace('_', ' '))}
                                for b in result['blocked_by']]
        for action in result['actions'].values():
            action['ready'] = action['ready'] and not blocked
        authorization = self._backend.authorization.public_status().get('allowed_actions', [])
        distance_channel = self._backend.distance_channel_status()
        for candidate in DISTANCE_ACTIONS:
            reasons = self._backend.distance_blockers(candidate)
            if not set(DISTANCE_SCOPES.values()) <= set(authorization):
                reasons.append('operator_session_unavailable')
            if not distance_channel['ready']:
                reasons.append('timed_command_channel_not_prepared')
            reasons = list(dict.fromkeys([*blocked, *reasons]))
            result['actions'][candidate] = {'ready': not reasons, 'blocked_by': reasons,
                'directions': list(DISTANCE_DIRECTIONS), 'speed_mps': TIMED_SPEED_MPS,
                'duration_s': TIMED_DURATION_S,
                'completion_level': 'reported_velocity_response_and_reported_stop'}
        if requested_action in DISTANCE_ACTIONS:
            result['blocked_by'] = list(dict.fromkeys(
                [*result['blocked_by'], *result['actions'][requested_action]['blocked_by']]))
            result['ready'] = not result['blocked_by']
            result['next_steps'] = [{'code': b, 'instruction': INSTRUCTIONS.get(b, b.replace('_', ' '))}
                                    for b in result['blocked_by']]
        result['timed_command_channel'] = distance_channel
        result['timed_move'] = {'directions': list(DISTANCE_DIRECTIONS),
            'speed_mps': TIMED_SPEED_MPS, 'duration_s': TIMED_DURATION_S,
            'requires_motion_state': 17}
        result['click_preflight'] = copy.deepcopy(self._backend.last_preflight)
        result['authorization_renewed_automatically'] = False
        result['safe_recovery_on_next_action'] = True
        result['consolidated_maintenance_action'] = 'prepare'
        result['authorization_renewal_action'] = ('prepare' if getattr(
            self._backend.authorization, 'supports_prepare', False) else None)
        result['stationary_check'] = self._backend.stationary_status()
        return result

    def get_tool(self):
        tool = super().get_tool()
        schema = tool['inputSchema']
        schema['properties']['action']['enum'].remove('prepare_test')
        schema['properties']['action']['enum'].remove('stopmotion')
        schema['properties']['action']['enum'].append('prepare')
        schema['allOf'].append({'if': {'properties': {'action': {'const': 'prepare'}}, 'required': ['action']},
            'then': {'not': {'anyOf': [{'required': [k]} for k in schema['properties']
                if k not in {'action', *META_FIELDS}]}}})
        schema['x-action-params'].pop('prepare_test', None)
        schema['x-action-params'].pop('stopmotion', None)
        schema['x-action-params'].update({
            'prepare': {'params': [], 'description': '准备控制：检查静止状态、关节、急停及故障；通过后建立或续期60分钟操作会话并准备通道。沿用设备预设动作权限，不发送动作。'},
        })
        completion = schema.get('x-completion', {})
        completion['actions'] = [value for value in completion.get('actions', [])
                                 if value not in DISTANCE_ACTIONS]
        completion['description'] = '姿态完成依赖新鲜的动作状态反馈。'
        schema['x-completion'] = completion
        tool['title'] = 'Posture Control'
        tool['description'] = '站立、卧倒与进入移动模式。点击动作前自动执行安全检查。'
        for action in POSTURE_STATES:
            schema['x-action-params'][action]['description'] += ' 点击前自动刷新安全检查；若上次失败且当前静止无故障，会先安全恢复通道再执行本次新动作，不重放旧动作。'
        return tool

    def _timed_move_tool(self):
        base_tool = super().get_tool()
        direction = {
            'type': 'string', 'enum': ['forward', 'backward'],
            'oneOf': [{'const': 'forward', 'title': '前进 2 秒'},
                      {'const': 'backward', 'title': '后退 2 秒'}],
            'title': '移动方向', 'description': '选择已完成实机验证的前进或后退。'}
        schema = {'type': 'object', 'properties': {'direction': direction},
                  'required': ['direction'], 'additionalProperties': False}
        schema['x-completion'] = {'actions': [], 'timeout': 15, 'silent_completion': False,
            'description': '完成需要观察到方向一致的速度响应，并在归零后观察到停止反馈。'}
        schema['x-resource'] = 'base'
        return {'name': TIMED_MOVE_TOOL, 'title': 'S10 前进/后退（2秒验证版）',
            'type': 'actuator', 'multiInstance': False,
            'description': '独立移动卡片：选择前进或后退，执行已经实机验证的固定2秒指令。',
            'inputSchema': schema, 'configSchema': base_tool['configSchema'],
            'topic_in': [], 'topic_out': self._timed_ports()}

    def get_tools(self):
        return [self.get_tool(), self._timed_move_tool()]

    def _timed_move_status(self):
        return self._card_status(self.status(), timed=True)

    def _check_unavailable(self, checked, action=None):
        result = self._unavailable(checked['blocked_by'], action)
        result['click_preflight'] = checked
        result['physical_command_sent'] = False
        return result

    def dispatch(self, action, args):
        if isinstance(args, dict) and args.get('_tool_name') == TIMED_MOVE_TOOL:
            if action in ('info', 'status'):
                validate_empty_arguments(args)
                return self._timed_move_status()
            if action == 'start':
                super().dispatch(action, args)
                return self._timed_move_status()
            if action == 'stop':
                super().dispatch(action, args)
                return self._timed_move_status()
            if action == 'config':
                super().dispatch(action, args)
                return self._timed_move_status()
            if action not in ('move', TIMED_MOVE_TOOL):
                return None
            arguments = dict(args)
            arguments.pop('_tool_name', None)
            return self.execute(DISTANCE_ACTION, arguments)
        if action in ('info', 'status'):
            validate_empty_arguments(args)
            return self._card_status(self.status(), timed=False)
        if action != 'prepare':
            return super().dispatch(action, args)
        validate_empty_arguments(args)
        with self._lock:
            if self._active is not None or (self._worker is not None and self._worker.is_alive()):
                return {'accepted': False, 'state': 'busy', 'physical_command_sent': False}
            if self._closed.is_set() or not self._started or not self._settings['enabled']:
                return self._unavailable(['card_not_enabled'])
            if self._error:
                return self._unavailable(['driver_error_requires_review'])
            self._backend.start_observation()
            recovered = False
            if getattr(self._backend.authorization, 'supports_prepare', False):
                try:
                    checked = self._backend.prepare_operator_session()
                except Exception:
                    return self._unavailable(['prepare_session_failed_requires_review'])
                if not checked['ready']:
                    return self._check_unavailable(checked)
                self._phase = 'ready'
                readiness = self._readiness()
                self._event('operator_session_prepared', physical_command_sent=False,
                            expires_at_ms=checked['operator_session']['expires_at_ms'])
                return {**checked, 'ready': readiness['ready'], 'blocked_by': readiness['blocked_by'],
                    'checked': True, 'prepared': readiness['ready'], 'physical_command_sent': False,
                    'old_action_retried': False, 'readiness': readiness}
            if self._backend.safety_status()['failed_closed']:
                try:
                    checked = self._backend.recover_worker()
                except Exception:
                    return self._unavailable(['recovery_failed_requires_review'])
                if not checked['ready']:
                    return self._check_unavailable(checked)
                recovered = True
                self._event('channel_recovered', physical_command_sent=False, old_action_retried=False)
            else:
                checked = self._backend.preflight()
            if not checked['ready']:
                return self._check_unavailable(checked)
            if not recovered:
                allowed = self._backend.authorization.public_status().get('allowed_actions', [])
                if not allowed:
                    return self._unavailable(['operator_session_unavailable'])
                try:
                    self._backend.prepare_execution(allowed[0])
                except CommandChannelNotReady:
                    return self._unavailable(['command_receiver_unmatched'])
                except Exception:
                    return self._unavailable(['command_worker_not_running'])
            self._phase = 'ready'
            readiness = self._readiness()
            return {**checked, 'ready': readiness['ready'], 'blocked_by': readiness['blocked_by'],
                    'checked': True, 'prepared': readiness['ready'], 'recovered': recovered,
                    'physical_command_sent': False, 'old_action_retried': False,
                    'readiness': readiness}

    def execute(self, action, arguments):
        if action not in self.execution_actions:
            return super().execute(action, arguments)
        with self._lock:
            # Preserve durable deduplication, even when a replay's current
            # conditions differ. Never re-arm a completed request identity.
            rid = arguments.get('request_id') if isinstance(arguments, dict) else None
            if rid and self._tasks.get(rid):
                return super().execute(action, arguments)
            if self._active is not None:
                return {'accepted': False, 'state': 'busy', 'reason': 'active_motion_task_in_progress',
                    'active_request_id': self._active['request_id'], 'active_action': self._active['action']}
            if self._closed.is_set() or not self._started or not self._settings['enabled']:
                return self._unavailable(['card_not_enabled'], action)
            if self._error:
                return self._unavailable(['driver_error_requires_review'], action)
            self._backend.start_observation()
            auto_recovered = False
            if self._backend.safety_status()['failed_closed']:
                try:
                    checked = self._backend.recover_worker()
                except Exception:
                    return self._unavailable(['recovery_failed_requires_review'], action)
                if not checked['ready']:
                    return self._check_unavailable(checked, action)
                auto_recovered = True
                self._phase = 'ready'
                self._event('channel_auto_recovered', requested_action=action,
                            physical_command_sent=False, old_action_retried=False)
            else:
                checked = self._backend.preflight(stationary_states=
                    (17,) if action in DISTANCE_ACTIONS else (1,) if action == 'move_mode' else None)
            transition_blocked = (self._backend.distance_blockers(action) if action in DISTANCE_ACTIONS
                                  else self._backend.transition_blockers(action))
            if transition_blocked:
                checked['ready'] = False
                checked['blocked_by'] = list(dict.fromkeys(
                    [*checked['blocked_by'], *transition_blocked]))
            if not checked['ready']:
                return self._check_unavailable(checked, action)
            # Existing task acceptance checks the protected grant again. A
            # failed worker was replaced above only after the same strict
            # recovery preflight; the old action identity is never replayed.
            reply = super().execute(action, arguments)
            if auto_recovered:
                reply['channel_auto_recovered'] = True
                reply['old_action_retried'] = False
            return reply

    def _stop_fixed_distance(self, action, request_id, request_deadline):
        with self._lock:
            if self._active and self._active.get('request_id') == request_id:
                self._phase = 'stopping'
        stop_deadline = min(self._clock() + 1.2, request_deadline)
        next_send, stable, last_sequence, send_count = self._clock(), 0, None, 0
        sent = False
        while self._clock() < stop_deadline:
            if self._clock() >= next_send:
                try:
                    receipt = self._backend.stop_distance(action, request_id)
                    sent = sent or receipt.get('sent') is True
                    send_count += receipt.get('sent') is True
                except Exception:
                    return False, sent
                next_send = self._clock() + .1
            observation = self._backend.distance_stopped()
            sequence = observation.get('sequence')
            if type(sequence) is int and sequence != last_sequence:
                last_sequence = sequence
                stable = stable + 1 if observation.get('stopped') else 0
                if stable >= 2 and send_count >= 2:
                    return True, sent
            time.sleep(.02)
        return False, sent

    def _run_action(self, record, request, started):
        if request['action'] not in DISTANCE_ACTIONS:
            return super()._run_action(record, request, started)
        action, rid = request['action'], request['request_id']
        direction = DISTANCE_DIRECTIONS[request['direction']]
        deadline = request['expires_monotonic'] - 1.25
        receipt = {'accepted': None}
        confirmed = stop_confirmed = completed = False
        sent = pulse_completed = False
        command_frames = 0
        min_vx = max_vx = None
        peak_signed_vx = None
        reason = 'timed_motion_incomplete'
        try:
            if deadline <= started + TIMED_DURATION_S:
                raise TimeoutError('timed-motion request leaves no stop reserve')
            self._backend.begin_distance(action, request, self._cancel)
            pulse_start = self._clock()
            pulse_end = pulse_start + TIMED_DURATION_S
            next_send = pulse_start
            while self._clock() < pulse_end:
                if self._cancel.is_set():
                    reason = self._cancel_reason or 'cancelled'
                    break
                if self._gate(action) or not self._control_ok(action):
                    reason = 'authority_unavailable'
                    break
                sample = self._backend.feedback.observation()
                velocity = sample.get('velocity') if sample.get('fresh') else None
                if (sample.get('motion_state') == 17 and isinstance(velocity, dict)
                        and type(velocity.get('vx')) in (int, float)
                        and math.isfinite(velocity['vx'])):
                    vx = float(velocity['vx'])
                    min_vx = vx if min_vx is None else min(min_vx, vx)
                    max_vx = vx if max_vx is None else max(max_vx, vx)
                    signed_vx = direction * vx
                    peak_signed_vx = signed_vx if peak_signed_vx is None else max(peak_signed_vx, signed_vx)
                    confirmed = confirmed or signed_vx >= TIMED_RESPONSE_MPS
                if self._clock() >= next_send:
                    receipt = self._backend.send_distance_velocity(
                        action, TIMED_SPEED_MPS, rid, self._cancel)
                    sent = sent or receipt.get('sent') is True
                    command_frames += receipt.get('sent') is True
                    self._last_command = self._clock()
                    # Advance on the original 10 Hz timeline so validation and
                    # feedback processing do not shorten the effective pulse.
                    next_send += .1
                    self._event('sent', request_id=rid, action_id=record['action_id'],
                                command_accepted=receipt.get('accepted'), confirmed=confirmed,
                                direction=request['direction'], elapsed_s=self._clock() - pulse_start)
                self._cancel.wait(.02)
            else:
                pulse_completed = True
            stop_confirmed, stop_sent = self._stop_fixed_distance(
                action, rid, request['expires_monotonic']) if sent else (True, False)
            sent = sent or stop_sent
            completed = (pulse_completed and confirmed and stop_confirmed
                         and not self._cancel.is_set())
            reason = ('timed_motion_completed' if completed else
                      'motion_response_not_observed' if pulse_completed and not confirmed else
                      'reported_stop_not_confirmed' if pulse_completed and confirmed and not stop_confirmed
                      else reason)
        except Exception as exc:
            reason = 'execution_failed_' + type(exc).__name__
            try:
                stop_confirmed, stop_sent = self._stop_fixed_distance(action, rid, request['expires_monotonic']) if sent else (False, False)
                sent = sent or stop_sent
            except Exception:
                stop_confirmed = False
            completed = False
        finally:
            cancelled = self._cancel.is_set()
            if cancelled:
                reason = self._cancel_reason or 'cancelled'
            if not completed:
                self._backend.abort()
            status = 'completed' if completed else 'cancelled' if cancelled and stop_confirmed else 'error'
            result = {'request_id': rid, 'action_id': record['action_id'], 'action': action,
                'accepted': True, 'command_accepted': receipt.get('accepted'),
                'confirmed': confirmed and not cancelled, 'stop_confirmed': stop_confirmed,
                'completed': completed, 'state': status, 'status': status, 'reason': reason,
                'direction': request['direction'], 'commanded_speed_mps': TIMED_SPEED_MPS,
                'commanded_duration_s': TIMED_DURATION_S, 'command_frames': command_frames,
                'reported_min_vx_mps': min_vx, 'reported_max_vx_mps': max_vx,
                'reported_peak_signed_vx_mps': peak_signed_vx,
                'completion_level': 'reported_velocity_response_and_reported_stop',
                'physical_command_sent': sent, 'finished_at': self._wall()}
            try:
                if self._tasks.finish(rid, result):
                    with self._lock:
                        self._last, self._phase = result, status if self._started else 'idle'
                        self._event('finished', **result)
            except Exception:
                self._error = 'terminal_record_failed_restart_recovery_required'
            with self._lock:
                self._active = None
            try:
                self._backend.finish(rid)
                if not self._started:
                    self._backend.stop_observation()
            finally:
                self._busy.release()
            self._flush_notifications()
