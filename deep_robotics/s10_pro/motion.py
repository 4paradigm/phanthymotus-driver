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
import uuid
from collections import deque


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


def validate_contract(contract):
    """Evidence must describe this machine, rather than a generic verified flag."""
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


class MotionPlugin:
    name = PREFIX = 'motion'

    def __init__(self, config, namespace, backend=None, *, feedback=None, publish=None,
                 execution_authorized=False, clock=time.monotonic, wall_time=time.time,
                 autostart_worker=True):
        if not isinstance(execution_authorized, bool):
            raise ValueError('execution_authorized must be boolean')
        self._backend = backend or UnsupportedBackend()
        self._feedback = feedback
        self._publish = publish or (lambda *args: None)
        self._authorized = execution_authorized
        self._clock, self._wall = clock, wall_time
        self._lock = threading.RLock()
        self._io_lock = threading.Lock()
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
        self._used_ids = deque(maxlen=128)
        self._error = None
        self._last_tick = clock()
        self._settings = self._validate({'enabled': False, 'refresh_hz': 2., 'max_duration_s': 2.,
            'action_timeout_s': 8., 'max_vx': .2, 'max_vy': .1, 'max_yaw_rate': .2, **dict(config or {})})
        base = '/' + '/'.join(p for p in (str(namespace).strip('/'), 's10_pro') if p)
        self.status_topic, self.events_topic = base + '/motion_status', base + '/motion_events'
        self._thread = threading.Thread(target=self._monitor, name='s10-motion-watchdog', daemon=True)
        if autostart_worker:
            self._thread.start()

    @staticmethod
    def _validate(settings):
        allowed = {'enabled', 'refresh_hz', 'max_duration_s', 'action_timeout_s', 'max_vx', 'max_vy', 'max_yaw_rate'}
        if set(settings) - allowed or not isinstance(settings.get('enabled'), bool):
            raise ValueError('unsupported motion settings')
        result = dict(settings)
        for key, low, high in [('refresh_hz', .5, 10), ('max_duration_s', .1, 5),
                ('action_timeout_s', .2, 15), ('max_vx', .01, .3), ('max_vy', .01, .15), ('max_yaw_rate', .01, .3)]:
            result[key] = _number(result.get(key), low, high, key)
        return result

    def _ports(self):
        return [{'topic': self.status_topic, 'format': 'data/json'}, {'topic': self.events_topic, 'format': 'data/json'}]

    def get_tool(self):
        props = {
            'action': {'type': 'string', 'enum': ['info', 'start', 'stop', 'status', 'config', *ACTIONS]},
            'vx': {'type': 'number', 'minimum': -.3, 'maximum': .3, 'title': '前后速度 (m/s)', 'default': 0},
            'vy': {'type': 'number', 'minimum': -.15, 'maximum': .15, 'title': '横向速度 (m/s)', 'default': 0},
            'yaw_rate': {'type': 'number', 'minimum': -.3, 'maximum': .3, 'title': '转向速度 (rad/s)', 'default': 0},
            'duration': {'type': 'number', 'minimum': .1, 'maximum': 5, 'default': 1, 'title': '持续秒数'},
            'request_id': {'type': 'string', 'maxLength': 128},
            'issued_at_ms': {'type': 'integer', 'description': '请求生成时间；过期请求被拒绝。'},
            'ttl_ms': {'type': 'integer', 'minimum': 100, 'maximum': 2000, 'default': 1000},
        }
        settings = {'enabled': {'type': 'boolean', 'default': False, 'title': '启用卡片状态',
            'description': '只开启卡片状态；不能解锁真机执行。'}}
        for key, low, high, title in [('refresh_hz', .5, 10, '状态频率 (Hz)'),
                ('max_duration_s', .1, 5, '最长移动时间 (秒)'), ('action_timeout_s', .2, 15, '动作确认超时 (秒)'),
                ('max_vx', .01, .3, '前后限速 (m/s)'), ('max_vy', .01, .15, '横向限速 (m/s)'),
                ('max_yaw_rate', .01, .3, '转向限速 (rad/s)')]:
            settings[key] = {'type': 'number', 'minimum': low, 'maximum': high, 'default': self._settings[key], 'title': title}
        return {'name': self.name, 'title': 'S10 Pro · 基础动作', 'type': 'actuator', 'multiInstance': False,
            'description': '起立、趴下、限时速度与停止。当前真机执行锁定；发出请求不等于动作完成。停止不是硬急停。',
            'inputSchema': {'type': 'object', 'properties': props, 'required': ['action'], 'additionalProperties': False,
                'x-action-params': {'stand': {'params': [], 'description': '起立并等待新鲜状态确认'},
                    'lie': {'params': [], 'description': '趴下并等待新鲜状态确认'},
                    'velocity': {'params': ['vx', 'vy', 'yaw_rate', 'duration'], 'description': '限时移动，到期零速并确认停止'},
                    'stopmotion': {'params': [], 'description': '取消本方动作并请求停止；不是硬急停'}}},
            'configSchema': {'type': 'object', 'properties': settings, 'additionalProperties': False},
            'topic_in': [], 'topic_out': self._ports()}

    def get_tools(self):
        return [self.get_tool()]

    def _contract(self):
        try:
            contract = self._backend.contract()
            return copy.deepcopy(contract) if isinstance(contract, dict) else {}
        except Exception:
            return {}

    def _gate(self, *, for_stop=False):
        missing = []
        if self._closed.is_set() and not for_stop:
            missing.append('controller_closed')
        if not self._authorized:
            missing.append('execution_authorization')
        if not validate_contract(self._contract()):
            missing.append('robot_control_contract')
        if self._feedback is None:
            missing.append('fresh_feedback_adapter')
        return missing

    def _control_ok(self):
        try:
            value = self._backend.control_status()
            age = value.get('age_s')
            return (value.get('owned') is True and value.get('fresh') is True
                and value.get('feedback_fresh') is True
                and isinstance(age, (int, float)) and not isinstance(age, bool) and math.isfinite(age) and 0 <= age <= .5)
        except Exception:
            return False

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
            return {'card': 'motion', 'state': 'running' if self._started else 'idle', 'phase': self._phase,
                'execution_available': not missing and self._settings['enabled'], 'execution_authorized': self._authorized,
                'contract_verified': validate_contract(self._contract()), 'blocked_by': missing,
                'availability_detail': {
                    'protocol_implemented': 'S10 software guide V1.0.2',
                    'production_backend': 'unbound',
                    'missing_evidence': self._contract().get('missing', []),
                    'explanation': '需确认本机适用版本、TLS信任、控制权与失联停止规则；界面启用不会解除执行锁。'},
                'settings': dict(self._settings), 'active_request': copy.deepcopy(self._active),
                'last_result': copy.deepcopy(self._last), 'last_error': self._error,
                'topic_in': [], 'topic_out': self._ports(), 'supported_actions': list(ACTIONS),
                'stop_semantics': 'application_stop_not_emergency_stop', 'events': list(self._events)}

    def start(self):
        with self._lock:
            if self._closed.is_set():
                return self.status()
            self._started = True
            self._phase = 'locked' if self._gate() or not self._settings['enabled'] else 'ready'
        return self.status()

    def stop(self):
        with self._lock:
            self._started = False
            self._phase = 'idle'
        result = self.cancel('card_stopped')
        return {**self.status(), 'cancellation': result}

    def close(self):
        self._closed.set()
        self.stop()
        if self._thread.ident is not None and self._thread is not threading.current_thread():
            self._thread.join(1)

    def configure(self, options):
        options = {k: v for k, v in options.items() if k not in ('instance_id', '_tool_name')}
        if isinstance(options.get('config'), dict):
            options = {**options.pop('config'), **options}
        with self._lock:
            updated = self._validate({**self._settings, **options})
            active = self._active is not None
            self._generation += 1
            self._settings = updated
            if active:
                self._cancel.set()
        if active:
            self.cancel('configuration_changed')
        return self.status()

    def _stop_backend(self, reason, *, request_id, authority_lost=False):
        if self._gate(for_stop=True):
            return {'accepted': False, 'confirmed': False, 'reason': 'unavailable'}
        contract = self._contract()
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
            if not self._control_ok() and not contract['stop_on_authority_loss']:
                return {'accepted': None, 'confirmed': False, 'reason': 'authority_released'}
            # An already-set movement cancellation token must not cancel its own stop.
            return self._backend.stop(reason, threading.Event())
        except Exception:
            return {'accepted': None, 'confirmed': False, 'reason': 'stop_transport_failed'}
        finally:
            self._io_lock.release()

    def cancel(self, reason='user_stop'):
        self._cancel.set()  # Priority over ordinary requests, before any locks.
        with self._lock:
            self._generation += 1
            self._cancel.set()
            active = copy.deepcopy(self._active)
        if active is None:
            return {'accepted': False, 'confirmed': False, 'completed': False, 'reason': 'no_application_motion'}
        receipt = self._stop_backend(reason, request_id=active['request_id'], authority_lost=not self._control_ok())
        self._event('cancel_requested', request_id=active['request_id'], reason=reason,
                    stop_accepted=receipt.get('accepted'), stop_confirmed=False)
        return {'accepted': receipt.get('accepted'), 'confirmed': False, 'completed': False,
                'request_id': active['request_id'], 'reason': reason}

    def _confirmation(self, action, request_id, after):
        try:
            value = self._feedback(action, request_id)
            age = value.get('age_s')
            received = value.get('received_monotonic')
            return (value.get('confirmed') is True and value.get('fresh') is True
                and isinstance(age, (int, float)) and not isinstance(age, bool) and math.isfinite(age) and 0 <= age <= .5
                and isinstance(received, (int, float)) and math.isfinite(received) and received >= after
                and value.get('request_id') == request_id)
        except Exception:
            return False

    def execute(self, action, arguments):
        if action not in ('stand', 'lie', 'velocity'):
            raise ValueError('unsupported motion action')
        with self._lock:
            missing = self._gate()
            if missing or not self._started or not self._settings['enabled']:
                return {'accepted': False, 'confirmed': False, 'completed': False, 'state': 'unavailable',
                        'blocked_by': missing + ([] if self._started and self._settings['enabled'] else ['card_not_enabled'])}
            generation = self._generation
        values = {k: v for k, v in arguments.items() if k not in ('_tool_name', 'instance_id')}
        allowed = {'request_id', 'issued_at_ms', 'ttl_ms'} | ({'vx', 'vy', 'yaw_rate', 'duration'} if action == 'velocity' else set())
        if set(values) - allowed:
            raise ValueError('unsupported motion parameters')
        now_ms = self._wall() * 1000
        issued = _number(values.get('issued_at_ms', now_ms), 0, 1e15, 'issued_at_ms')
        ttl = _number(values.get('ttl_ms', 1000), 100, 2000, 'ttl_ms')
        if now_ms - issued > ttl or issued > now_ms + 100:
            raise ValueError('stale or future motion request')
        rid = values.get('request_id', uuid.uuid4().hex)
        if not isinstance(rid, str) or not rid.strip() or len(rid) > 128:
            raise ValueError('invalid request_id')
        duration = (_number(values.get('duration', min(1, self._settings['max_duration_s'])), .1,
                            self._settings['max_duration_s'], 'duration') if action == 'velocity' else 0)
        request = {'request_id': rid, 'action': action, 'duration': duration}
        if action == 'velocity':
            for key, limit in [('vx', self._settings['max_vx']), ('vy', self._settings['max_vy']), ('yaw_rate', self._settings['max_yaw_rate'])]:
                request[key] = _number(values.get(key, 0), -limit, limit, key)
        if not self._busy.acquire(blocking=False):
            return {'accepted': False, 'confirmed': False, 'completed': False, 'state': 'busy'}
        receipt, confirmed, stop_confirmed, reason = {'accepted': None}, False, False, 'feedback_timeout'
        started = self._clock()
        result = None
        try:
            with self._lock:
                if generation != self._generation or not self._started or not self._settings['enabled'] or self._gate():
                    return {'accepted': False, 'confirmed': False, 'completed': False, 'state': 'cancelled', 'reason': 'lifecycle_changed'}
                if rid in self._used_ids:
                    raise ValueError('duplicate request_id')
                self._used_ids.append(rid)
                self._cancel = threading.Event()
                self._active = {**request, 'started_monotonic': started}
                self._last_command = started
                self._phase = 'active'
            self._event('requested', request_id=rid, action=action)
            deadline = started + (duration if action == 'velocity' else self._settings['action_timeout_s'])
            next_send = started
            contract = self._contract()
            refresh_interval = min(.1, (contract['robot_deadman_s'] - contract['send_timeout_s']) / 3)
            while self._clock() < deadline:
                if self._cancel.is_set():
                    reason = 'cancelled'
                    break
                if not self._control_ok():
                    reason = 'authority_unavailable'
                    self._cancel.set()
                    break
                if self._clock() >= next_send:
                    with self._io_lock:
                        if self._cancel.is_set() or not self._control_ok():
                            reason = 'cancelled'
                            break
                        if not self._started or (not self._active or self._active['request_id'] != rid):
                            reason = 'cancelled'
                            break
                        if not self._used_ids or (self._wall() * 1000 > issued + ttl and next_send == started):
                            reason = 'stale_request'
                            break
                        receipt = self._backend.send(action, request, self._cancel)
                        with self._lock:
                            self._last_command = self._clock()
                    if receipt.get('accepted') is False:
                        reason = 'rejected'
                        break
                    if action != 'velocity':
                        next_send = float('inf')
                    else:
                        next_send = self._clock() + refresh_interval
                    self._event('sent', request_id=rid, accepted=receipt.get('accepted'), confirmed=False)
                if self._confirmation(action, rid, started):
                    confirmed = True
                    if action != 'velocity':
                        reason = 'feedback_confirmed'
                        break
                self._cancel.wait(.02)
            if action == 'velocity' and not self._cancel.is_set() and self._clock() >= deadline:
                reason = 'duration_elapsed'
            if action == 'velocity' or not confirmed:
                stop_after = self._clock()
                stop_receipt = self._stop_backend(reason, request_id=rid, authority_lost=not self._control_ok())
                stop_deadline = self._clock() + 1.
                next_stop = self._clock() + refresh_interval
                while self._clock() < stop_deadline and stop_receipt.get('reason') != 'authority_released':
                    if self._confirmation('stopmotion', rid, stop_after):
                        stop_confirmed = True
                        break
                    if self._clock() >= next_stop:
                        stop_receipt = self._stop_backend(reason, request_id=rid, authority_lost=not self._control_ok())
                        next_stop = self._clock() + refresh_interval
                    time.sleep(.02)
                # A cancelled posture must never be reported as successful posture.
                if action != 'velocity':
                    confirmed = False
            completed = confirmed and reason in ('feedback_confirmed', 'duration_elapsed') and (action != 'velocity' or stop_confirmed)
            result = {'request_id': rid, 'accepted': receipt.get('accepted'), 'confirmed': confirmed,
                'stop_confirmed': stop_confirmed, 'completed': completed,
                'state': 'completed' if completed else 'cancelled' if reason == 'cancelled' else 'failed',
                'reason': reason, 'action': action}
        except Exception as exc:
            self._cancel.set()
            self._stop_backend('execution_failed', request_id=rid, authority_lost=not self._control_ok())
            result = {'request_id': rid, 'accepted': receipt.get('accepted'), 'confirmed': False,
                'completed': False, 'state': 'failed', 'reason': type(exc).__name__, 'action': action}
        finally:
            with self._lock:
                if result is not None:
                    self._last, self._phase = result, result['state'] if self._started else 'idle'
                self._active = None
                if result is not None:
                    self._event('finished', **result)
            self._busy.release()
        return copy.deepcopy(result)

    def dispatch(self, action, args):
        if action in ('info', 'status'):
            return self.status()
        if action == 'start':
            return self.start()
        if action == 'stop':
            return self.stop()
        if action == 'config':
            return self.configure(args)
        if action == 'stopmotion':
            return self.cancel()
        return self.execute(action, args)

    def _monitor(self):
        while not self._closed.wait(.05):
            now = self._clock()
            with self._lock:
                active = self._active is not None
                started = self._started
                overdue = now - self._last_tick > .5
                if active and self._active['action'] == 'velocity':
                    overdue = overdue or now - self._last_command > self._contract().get('robot_deadman_s', .3) / 2
                self._last_tick = now
            if active and (overdue or not self._control_ok()):
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
