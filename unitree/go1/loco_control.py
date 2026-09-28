"""Confirmed HIGHLEVEL loco actions. Never equate request acceptance with motion.

No native hardware imports: all device access goes through the shared SdkProxy.
This is a software stop, not a safety-rated emergency stop.
"""
import copy
import json
import math
import os
import ssl
import threading
import time
import urllib.request
import uuid
from collections import OrderedDict


class ControlError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


POSTURES = {'stand_up': 6, 'stand_down': 5, 'balance_stand': 1,
            'recovery_stand': 8, 'damp': 7}


def number(value, name, low, high):
    if isinstance(value, bool):
        raise ControlError('INVALID_ARGUMENT', name + ' must be a number')
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ControlError('INVALID_ARGUMENT', name + ' must be a number')
    if not math.isfinite(value) or not low <= value <= high:
        raise ControlError('INVALID_ARGUMENT', '%s outside [%s, %s]' % (name, low, high))
    return value


class ConfirmedLocoPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._lock = threading.RLock()
        self._active = None
        self._held = None  # Completed posture still owns the SDK until explicit stop.
        self._stopping = 0
        self._closed = False
        self._history = OrderedDict()
        cfg = plugin_config or {}
        self._control_enabled = cfg.get('control_enabled', False) is True
        self._stop_timeout = number(cfg.get('stop_timeout_sec', 3), 'stop_timeout_sec', .5, 5)
        self._posture_timeout = number(cfg.get('posture_timeout_sec', 8), 'posture_timeout_sec', 2, 15)
        self._max_duration = number(cfg.get('max_duration_sec', 10), 'max_duration_sec', .5, 300)
        self._poll = .05
        self._grace = .3
        self._settle = .15

    def get_tool(self):
        actions = ['start', 'stop', 'move', 'stop_move'] + list(POSTURES) + ['status']
        params = {'move': {'params': ['vx', 'vy', 'vyaw', 'duration'],
                           'description': '有限时运动；短动作同步确认，长动作返回 accepted 和 action_id'},
                  'start': {'params': [], 'description': '卡片生命周期就绪；不启用控制、不发运动命令'},
                  'stop': {'params': [], 'description': '卡片生命周期停止；idle 不代表真机停稳，停稳确认用 stop_move'},
                  'stop_move': {'params': [], 'description': '优先请求运动停止并确认新鲜遥测已停稳；中断入口'},
                  'status': {'params': ['action_id'], 'description': '查询当前或最近动作的最终结果'}}
        for action in POSTURES:
            params[action] = {'params': ['confirm'] if action in ('damp', 'recovery_stand') else [],
                              'description': '姿态确认后保持控制权；后续动作前先 stop_move；非紧急停止'}
        tool = {'name': 'loco_confirmed', 'type': 'actuator', 'multiInstance': False,
                'description': 'Go1 有反馈运动控制；accepted 仅表示已接收，completed 才表示观察到结果。'
                               'stop_move 无遥测确认时返回 STOP_UNCONFIRMED，须遥控器接管；'
                               'start/stop 仅返回生命周期状态，不证明真机运动或停稳。',
                'inputSchema': {'type': 'object', 'required': ['action'],
                    'properties': {
                        'action': {'type': 'string', 'enum': actions},
                        'vx': {'type': 'number', 'minimum': -1., 'maximum': 1., 'description': '前后 m/s；0 或绝对值≥0.05'},
                        'vy': {'type': 'number', 'minimum': -.6, 'maximum': .6, 'description': '横向 m/s；0 或绝对值≥0.05'},
                        'vyaw': {'type': 'number', 'minimum': -90., 'maximum': 90., 'description': '偏航 °/s；0 或绝对值≥6'},
                        'duration': {'type': 'number', 'minimum': 0., 'maximum': self._max_duration,
                                     'description': '秒；省略或 0 为 0.5 秒；不自动延长'},
                        'confirm': {'type': 'boolean', 'description': '恢复站立/阻尼须显式 true'},
                        'action_id': {'type': 'string'}},
                    'x-action-params': params,
                    'x-resource': 'base',
                    'x-completion': {'actions': ['move'] + list(POSTURES),
                                     'timeout': max(self._max_duration, self._posture_timeout) + 15},
                    'x-hooks': {'on_interrupt_motion': {'action': 'stop_move'},
                                'on_interrupt_all': {'action': 'stop_move'}}}}
        if not self._control_enabled:
            tool['inputSchema'].pop('x-hooks', None)
            tool['description'] += ' 当前仅展示/查询；control_enabled=false，不能控制机器人。'
        return tool

    def start(self):
        # Lifecycle readiness is independent of SDK availability/control opt-in.
        # Never clear a pending cancellation or rearm the SDK stop latch here.
        with self._lock:
            self._closed = False

    def stop(self):
        with self._lock:
            self._closed = True
            if self._active:
                self._active['cancel'].set()
            if self._control_enabled:
                self._client.request_stop()
                self._release_held_after_stop()

    def _release_held_after_stop(self):
        # Caller holds _lock and has already latched stop. Never release a live
        # posture first: legacy writes must remain blocked even during cleanup.
        if self._held:
            job = self._held
            job['cancel'].set()
            self._client.release_control(job['id'])
            self._held = None  # Retain on RPC failure for explicit stop retry.

    def _result(self, action, status, **fields):
        return dict(card='loco_confirmed', action=action, status=status,
                    ok=status in ('accepted', 'completed', 'ready'),
                    control_level='HIGHLEVEL', timestamp_ms=int(time.time() * 1000), **fields)

    def _snapshot(self):
        snap = self._client.snapshot()
        age = snap.get('telemetry_age_sec')
        if (not snap.get('fresh') or not isinstance(age, (int, float))
                or not math.isfinite(age) or not 0 <= age <= .5
                or not isinstance(snap.get('sample_seq'), int) or snap['sample_seq'] <= 0):
            raise ControlError('TELEMETRY_STALE', 'Fresh, independently received HighState required')
        self._speeds(snap)
        return snap

    @staticmethod
    def _speeds(snap):
        try:
            values = [float(snap['velocity'][0]), float(snap['velocity'][1]), float(snap['yaw_speed'])]
            if not all(math.isfinite(v) for v in values):
                raise ValueError()
            return values
        except (KeyError, IndexError, TypeError, ValueError):
            raise ControlError('TELEMETRY_INVALID', 'Missing or invalid velocity')

    @staticmethod
    def _upright(snap):
        try:
            rpy = snap['imu']['rpy_rad']
            return all(math.isfinite(float(x)) and abs(float(x)) < .6 for x in rpy[:2]) and len(rpy) >= 2
        except (KeyError, TypeError, ValueError):
            return False

    def _preflight(self, action):
        if not self._client.available:
            raise ControlError('STUB_MODE', 'SDK unavailable')
        snap = self._snapshot()
        sent_age = snap.get('last_send_age_sec')
        if not isinstance(sent_age, (int, float)) or not math.isfinite(sent_age) or not 0 <= sent_age <= .5:
            raise ControlError('UDP_UNAVAILABLE', 'No recent successful UDP send')
        if action == 'move' and (snap.get('mode') not in (1, 2)
                                 or not self._upright(snap)
                                 or not .18 <= float(snap.get('body_height', 0)) <= .5):
            raise ControlError('PRECONDITION_FAILED', 'Move requires an upright, standing robot')
        return snap

    def dispatch(self, action, args):
        args = args or {}
        # Framework lifecycle markers must work even in observation-only mode.
        if action == 'start':
            self.start()
            return {'state': 'ready'}
        if action == 'stop':
            try:
                self.stop()  # Cancel/latch without waiting for physical feedback.
            except Exception as exc:
                return {'state': 'idle', 'ok': False,
                        'code': getattr(exc, 'code', 'CONTROL_ERROR'),
                        'message': str(exc), 'stop_confirmed': False,
                        'recommended_action': 'remote_stop_and_inspect'}
            return {'state': 'idle'}  # Card state, NOT stop confirmation.
        if not self._control_enabled and action not in ('status', 'info'):
            return self._result(action, 'error', code='CONTROL_DISABLED',
                                message='New card is observation-only until controlled acceptance')
        if action == 'stop_move':
            return self._stop_action(action)
        if action in ('status', 'info'):
            with self._lock:
                aid = args.get('action_id')
                job = self._history.get(aid) if aid else self._active
                if not job and not aid and self._history:
                    job = next(reversed(self._history.values()))
                if not job:
                    return self._result(action, 'idle' if not aid else 'error', code='NO_ACTION',
                                        control_enabled=self._control_enabled)
                result = copy.deepcopy(job['result'])
                result['holding_control'] = self._held is job
                return result
        if action != 'move' and action not in POSTURES:
            return None  # bundle converts this to JSON-RPC Unknown tool
        try:
            with self._lock:
                if self._closed:
                    raise ControlError('CARD_STOPPED', 'Start the card lifecycle before requesting motion')
            # Validate before cancelling or otherwise affecting an existing action.
            requested = {}
            if action == 'move':
                requested = {k: number(args.get(k, 0.), k, -limit, limit)
                             for k, limit in [('vx', 1.), ('vy', .6), ('vyaw', 90.)]}
                duration = number(args.get('duration', .5), 'duration', 0., self._max_duration) or .5
                requested['duration'] = duration
                if all(requested[k] == 0 for k in ('vx', 'vy', 'vyaw')):
                    return self._stop_action('stop_move')
                for k, minimum in [('vx', .05), ('vy', .05), ('vyaw', 6.)]:
                    if 0 < abs(requested[k]) < minimum:
                        raise ControlError('INVALID_ARGUMENT', k + ' is below reliable feedback threshold')
            else:
                duration = self._posture_timeout
            if action in ('damp', 'recovery_stand') and args.get('confirm') is not True:
                raise ControlError('PRECONDITION_FAILED', 'Explicit confirm=true required')
            generation = self._client.control_epoch
            snap = self._preflight(action)
            asynchronous = action in POSTURES or duration > 2
            aid = 'go1_loco_confirmed_' + uuid.uuid4().hex
            with self._lock:
                if self._closed or self._active or self._stopping or self._held:
                    raise ControlError('RESOURCE_BUSY', 'Action/stopping or held posture active; use stop_move first')
                if generation != self._client.control_epoch:
                    raise ControlError('CANCELLED', 'Stop occurred during preflight')
                job = {'id': aid, 'action': action, 'args': requested, 'duration': duration,
                       'epoch': generation, 'cancel': threading.Event(), 'async': asynchronous,
                       'baseline': snap['sample_seq'],
                       'send_errors': snap.get('send_error_count', 0),
                       'result': self._result(action, 'accepted', command_id=aid,
                                              executed=False, requested=requested)}
                if asynchronous:
                    job['result']['action_id'] = aid
                self._active = job
                self._history[aid] = job
                while len(self._history) > 32:
                    self._history.popitem(last=False)
                accepted = copy.deepcopy(job['result'])
                if asynchronous:
                    thread = threading.Thread(target=self._run, args=(job,), daemon=True,
                                              name='go1_loco_action')
                    try:
                        thread.start()
                    except Exception:
                        self._active = None
                        del self._history[aid]
                        raise
            if asynchronous:
                return accepted
            self._run(job)
            return copy.deepcopy(job['result'])
        except Exception as exc:
            return self._result(action, 'error', code=getattr(exc, 'code', 'CONTROL_ERROR'), message=str(exc))

    def _check_cancel(self, job):
        if job['cancel'].is_set() or job['epoch'] != self._client.control_epoch:
            raise ControlError('CANCELLED', 'Action interrupted by stop')

    def _run(self, job):
        failure = None
        observed = None
        stop_result = None
        may_own_control = False
        try:
            self._check_cancel(job)
            # A timeout has an unknown outcome and must fail safe. An explicit
            # rejection, however, must not stop a different card's movement.
            may_own_control = True
            try:
                self._client.claim_control(job['id'], job['epoch'])
            except Exception as exc:
                if getattr(exc, 'code', None) in ('RESOURCE_BUSY', 'CANCELLED', 'STUB_MODE'):
                    may_own_control = False
                raise
            observed = self._monitor(job)
            self._check_cancel(job)
        except Exception as exc:
            failure = exc
        finally:
            held = False
            # Publish held ownership atomically against lifecycle/physical stop.
            # A completed posture is NOT an idle SDK: its command remains latched.
            with self._lock:
                if may_own_control and job['action'] in POSTURES and failure is None:
                    try:
                        self._check_cancel(job)
                    except Exception as exc:
                        failure = exc
                    else:
                        self._held = job
                        held = True
            # Successful posture is intentionally held. Every move/error/cancel
            # requests a stop before checking telemetry or delivering callbacks.
            if may_own_control and (job['action'] == 'move' or failure is not None):
                self._client.request_stop()
                stop_result = self._confirm_stop()
            try:
                if may_own_control and not held:
                    self._client.release_control(job['id'])
            except Exception as exc:
                failure = failure or exc
                self._client.request_stop()
            status = 'completed'
            fields = {'command_id': job['id'], 'requested': job['args'],
                      'observed': observed or job.get('last_observed')}
            if failure:
                code = getattr(failure, 'code', 'CONTROL_ERROR')
                status = 'cancelled' if code == 'CANCELLED' else 'error'
                fields.update(code=code, message=str(failure))
            if stop_result is not None:
                fields['stop'] = stop_result
                if not stop_result['stop_confirmed']:
                    status = 'error'
                    if failure:
                        fields['cause_code'] = getattr(failure, 'code', 'CONTROL_ERROR')
                    fields.update(code='STOP_UNCONFIRMED', recommended_action='remote_stop_and_inspect')
            with self._lock:
                if job['cancel'].is_set() and status == 'completed':
                    status = 'cancelled'
                    fields.update(code='CANCELLED', message='Interrupted before action settled')
                job['result'] = self._result(job['action'], status, **fields)
                if job['async']:
                    job['result']['action_id'] = job['id']
                if self._active is job:
                    self._active = None
            if job['async']:
                self._notify(job)

    def _monitor(self, job):
        started = time.monotonic()
        deadline = started + job['duration']
        seq = job['baseline']
        last_match = started
        matched = 0
        posture_since = None
        observed = None
        while time.monotonic() < deadline:
            self._check_cancel(job)
            if job['action'] == 'move':
                a = job['args']
                self._client.control_move(job['id'], job['epoch'], a['vx'], a['vy'], math.radians(a['vyaw']), deadline)
            else:
                self._client.control_posture(job['id'], job['epoch'], POSTURES[job['action']])
            self._check_cancel(job)
            snap = self._snapshot()
            if snap.get('send_error_count', 0) > job['send_errors']:
                raise ControlError('UDP_SEND_FAILED', 'UDP error during action')
            if snap['sample_seq'] > seq:
                seq = snap['sample_seq']
                speeds = self._speeds(snap)
                observed = {'sample_seq': seq, 'vx': speeds[0], 'vy': speeds[1],
                            'yaw_rad_s': speeds[2], 'mode': snap.get('mode'),
                            'body_height': snap.get('body_height'),
                            'gait_type': snap.get('gait_type')}
                job['last_observed'] = observed
                if job['action'] == 'move':
                    if not self._upright(snap):
                        raise ControlError('UNSAFE_POSTURE', 'Excessive tilt or missing IMU')
                    wanted = [a['vx'], a['vy'], math.radians(a['vyaw'])]
                    checks = [actual * (1 if target > 0 else -1) >= min(.03, abs(target) * .3)
                              for actual, target in zip(speeds, wanted) if target != 0]
                    if all(checks):
                        matched += 1
                        last_match = time.monotonic()
                    elif time.monotonic() - last_match > self._grace:
                        raise ControlError('MOTION_NOT_OBSERVED', 'Requested axes not observed in matching directions')
                elif self._posture_matches(job['action'], snap):
                    matched += 1
                    if posture_since is None:
                        posture_since = time.monotonic()
                    if matched >= 3 and time.monotonic() - posture_since >= self._settle:
                        return observed
                else:
                    matched = 0
                    posture_since = None
            job['cancel'].wait(self._poll)
        self._check_cancel(job)
        if job['action'] == 'move' and matched >= 2 and time.monotonic() - last_match <= self._grace:
            return observed
        raise ControlError('MOTION_NOT_OBSERVED' if job['action'] == 'move' else 'POSTURE_UNCONFIRMED',
                           'No sufficient fresh matching samples before deadline')

    def _posture_matches(self, action, snap):
        vx, vy, yaw = self._speeds(snap)
        still = math.hypot(vx, vy) <= .03 and abs(yaw) <= .08
        height = float(snap.get('body_height', 0))
        mode = snap.get('mode')
        if action == 'damp':
            return mode == 7 and still
        if action == 'stand_down':
            return mode in (0, 5) and 0 < height <= .16 and still
        if action in ('stand_up', 'recovery_stand'):
            return mode in (1, 6) and .2 <= height <= .5 and self._upright(snap) and still
        return mode == 1 and .18 <= height <= .5 and self._upright(snap) and still

    def _confirm_stop(self):
        deadline = time.monotonic() + self._stop_timeout
        first = None
        count = 0
        seq = None
        last_error = 'No new stop telemetry'
        while time.monotonic() < deadline:
            try:
                snap = self._snapshot()
                if seq is None:
                    seq = snap['sample_seq']  # baseline must be AFTER stop request
                elif snap['sample_seq'] > seq:
                    seq = snap['sample_seq']
                    vx, vy, yaw = self._speeds(snap)
                    if math.hypot(vx, vy) <= .03 and abs(yaw) <= .08:
                        count += 1
                        first = time.monotonic() if first is None else first
                        if count >= 3 and time.monotonic() - first >= self._settle:
                            return {'stop_confirmed': True, 'sample_seq': seq,
                                    'vx': vx, 'vy': vy, 'yaw_rad_s': yaw}
                    else:
                        first, count = None, 0
                        last_error = 'Robot still moving'
            except Exception as exc:
                first, count = None, 0
                last_error = str(exc)
            time.sleep(self._poll)
        return {'stop_confirmed': False, 'code': 'STOP_UNCONFIRMED', 'message': last_error,
                'recommended_action': 'remote_stop_and_inspect'}

    def _stop_action(self, action):
        with self._lock:
            self._stopping += 1
        try:
            with self._lock:
                if self._active:
                    self._active['cancel'].set()
                self._client.request_stop()
                self._release_held_after_stop()
            result = self._confirm_stop()
            return self._result(action, 'completed' if result['stop_confirmed'] else 'error', **result)
        finally:
            with self._lock:
                self._stopping -= 1

    def _notify(self, job):
        result = copy.deepcopy(job['result'])
        payload = json.dumps({'action_id': job['id'], 'tool': 'loco_confirmed',
                              'status': result['status'], 'result': result, 'ts': time.time()}).encode()
        # ACP is the platform's internal callback (self-signed local HTTPS).
        url = os.environ.get('AGENT_CORE_URL', 'https://localhost:15678').rstrip('/') + '/api/acp/complete'
        context = ssl.create_default_context()
        if url.startswith(('https://localhost:', 'https://127.0.0.1:')):
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
        error = None
        # One POST only: even an unmatched result may enqueue a platform event.
        # Retain delivery failures for status queries, never blindly replay.
        for delay in (.1,):
            time.sleep(delay)
            try:
                req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
                with urllib.request.urlopen(req, timeout=2, context=context) as response:
                    answer = json.loads(response.read())
                if answer.get('matched') is False:
                    error = 'ACP action not registered yet'
                    continue
                if answer.get('ok') is False:
                    raise RuntimeError('ACP callback rejected')
                error = None
                break
            except Exception as exc:
                error = str(exc)
                break
        with self._lock:
            job['result']['completion_delivery'] = 'failed' if error else 'delivered'
            if error:
                job['result']['completion_error'] = error


def make_loco_confirmed(plugin_config, namespace, executor, client):
    return ConfirmedLocoPlugin(plugin_config, namespace, executor, client)
