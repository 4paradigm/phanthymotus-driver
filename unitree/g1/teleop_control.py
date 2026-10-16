"""G1 two-card operator session; only ArmStreamExecutor emits hardware commands.

Session admission derives from Tianyi PR321 49997cc6 (Apache-2.0).
G1 uses ten arm joints and the existing release/action 99, not Tianyi homing.

Mapping derives from the former ActuCore DualArmMapping (Apache-2.0). A grip is
an enable signal, never a new coordinate calibration. Numerical work remains in
MotionControl's isolated worker, and DDS in the existing local-profile helper.
"""
import copy
import html
import math
import secrets
import threading
import time

from common.teleop_contract import validate_input, FEEDBACK_SCHEMA
from common.motion.protocol import envelope
from trajectory_control import TrajectoryControl


def _mul(a, b):
    x,y,z,w = a; X,Y,Z,W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def _inverse(q): return [-q[0], -q[1], -q[2], q[3]]


def _rotate(q, p): return _mul(_mul(q, [*p, 0]), _inverse(q))[:3]


class RelativeMapping:
    def __init__(self, scale=.5):
        self.scale, self.anchor, self.epoch = scale, None, 0
        self.set_controller_offsets({})

    def set_controller_offsets(self, offsets):
        """Preserve r4 OpenXR-local controller-to-palm calibration.

        Device input has already changed both parent and child axes by B:
        (x,y,z)->(-z,-x,y). Conjugate the stored local offset by the same B;
        applying an unconverted local translation would rotate the wrong lever.
        """
        converted = {}
        for side in ('left', 'right'):
            pose = offsets.get(side, {'position': [0.,0.,0.], 'orientation': [0.,0.,0.,1.]})
            p, q = pose['position'], pose['orientation']
            if (len(p) != 3 or len(q) != 4
                    or any(type(v) not in (int,float) or not math.isfinite(v) for v in (*p,*q))
                    or abs(sum(v*v for v in q)-1.) > .002):
                raise ValueError('invalid_controller_to_palm')
            converted[side] = ([-p[2], -p[0], p[1]], [-q[2], -q[0], q[1], q[3]])
        self.controller_offsets = converted

    def _palm(self, frame, side):
        pose = frame[side]
        offset, rotation = self.controller_offsets[side]
        lever = _rotate(pose['orientation_xyzw'], offset)
        return ([a+b for a,b in zip(pose['position'], lever)],
                _mul(pose['orientation_xyzw'], rotation))

    def calibrate(self, frame, poses):
        forward = _rotate(frame['head_reference']['orientation_xyzw'], [1, 0, 0])
        yaw = math.atan2(forward[1], forward[0])
        alignment = [0, 0, math.sin(-yaw/2), math.cos(-yaw/2)]
        self.anchor = (copy.deepcopy(frame), copy.deepcopy(poses), alignment)
        self.epoch += 1

    def targets(self, frame):
        if self.anchor is None: raise ValueError('needs_calibration')
        origin, poses, alignment = self.anchor
        values = []
        for side, robot in zip(('left', 'right'), poses):
            current, start = self._palm(frame, side), self._palm(origin, side)
            delta = _rotate(alignment, [a-b for a,b in zip(current[0], start[0])])
            rotation = _mul(current[1], _inverse(start[1]))
            aligned = _mul(_mul(alignment, rotation), _inverse(alignment))
            values.extend(a+self.scale*b for a,b in zip(robot[:3], delta))
            values.extend(_mul(aligned, robot[3:]))
        return values


class TeleopControl:
    CAPABILITIES = ['dual_arm']
    INPUT_HOLDS = ('input_missing', 'input_stale', 'tracking_lost')

    def __init__(self, cfg, motion, *, clock=time.monotonic_ns, notify=None):
        self._validate_config(cfg)
        self.cfg, self.motion, self.executor = cfg, motion, motion.executor
        self.clock, self.clock_id = clock, motion.clock_id
        self.server_epoch = secrets.token_hex(16)
        self.mapping = RelativeMapping(cfg.get('position_scale', 1.))
        self._lock = threading.RLock()
        self._begin_lock = threading.Lock()
        self._closed, self._wake = threading.Event(), threading.Event()
        self._thread = None
        self.binding = None
        self.transport_status = None
        self.instance_id = 'teleop_control'
        self._latest = None
        self._input_identity = None
        self._input_sequence = -1
        self._mapping_identity = None
        self._processed = None
        self._generation = self._sequence = self._feedback_sequence = 0
        self._operator = None
        self._release_seen = False
        self._gripped = False
        self._enabled = False
        self._ready_on_release = False
        self._engage_sequence = None
        self._paused = False
        self._hold_kind = None
        self._needs_calibration = False
        self._state, self._reason = 'idle', None
        self.executor.cfg['live_enabled'] = self.live
        self.executor.cfg['operator_session_enabled'] = self.live
        self.trajectory = TrajectoryControl(self, notify)
        self._input_changed = False
        self._buttons = {}
        self._reset_operation = None

    @property
    def live(self): return self.cfg.get('mode', 'live') == 'live'

    def get_tool(self):
        fields = {'usage_guide': {'type': 'string', 'title': '使用说明（固定，无需配置）',
                    'enum': ['无需配置'], 'default': '无需配置', 'scope': 'instance'}}
        return {'name': 'teleop_control', 'type': 'actuator', 'multiInstance': False,
            'description': 'G1双臂遥操：先安装遥操设备 Driver，在画布添加 teleop_device 并连接本卡；在设备卡齿轮页获取 App 下载和配对入口。启动项目后双方双臂下垂，松开双握把并长按 X+A 一秒确认基准，松键后重新双握接通。Y+B 长按一秒复用原厂收臂复位；完成后重新确认基准。松握暂停；对齐误差仅用于显示。支持轨迹录制和检查；play 只需文件名，调用后直接限速移动到起点并回放，请先确认运动空间无人、无障碍。停止项目停止遥操；收臂使用 arm release。',
            'configSchema': {'type': 'object', 'additionalProperties': False, 'properties': fields},
            'inputSchema': {'type': 'object', 'required': ['action'],
                'properties': {'action': {'type': 'string', 'enum': ['info', 'config', 'start', 'stop', 'record_start', 'record_stop', 'recordings', 'inspect', 'start_difference', 'play', 'action_status']},
                    'input_topic': {'type': 'string'}, 'filename': {'type': 'string'},
                    'action_id': {'type': 'string'}},
                'x-completion': {'actions': ['play'], 'timeout': 620},
                'x-resource': ['arm_l', 'arm_r'],
                'x-action-params': {'info': {'params': []}, 'config': {'params': []},
                    'start': {'params': ['input_topic']}, 'stop': {'params': []},
                    'record_start': {'params': []}, 'record_stop': {'params': []}, 'recordings': {'params': []},
                    'inspect': {'params': ['filename']}, 'start_difference': {'params': ['filename']},
                    'play': {'params': ['filename']},
                    'action_status': {'params': ['action_id']}}},
            'topic_in': [{'port_id': 'command', 'format': 'data/teleop-cmd',
                **({'topic': self.binding['command_topic']} if self.binding else {})}],
            'topic_out': [{'port_id': 'state', 'format': 'data/teleop-state', 'topic': '/teleop/state'}]}

    @staticmethod
    def _validate_config(candidate):
        if candidate.get('mode', 'live') not in ('shadow', 'live'):
            raise ValueError('invalid_mode')
        scale = candidate.get('position_scale', 1.)
        if type(scale) not in (int, float) or not math.isfinite(scale) or not 0 < scale <= 1.:
            raise ValueError('invalid_position_scale')

    def dispatch(self, action, args):
        try:
            if action in ('info', 'config'): return self.info()
            if action == 'start':
                with self.trajectory.lock:
                    if self.trajectory.busy: raise ValueError('replay_active')
                    return self.start(args)
            if action == 'stop': return self.stop()
            return self.trajectory.dispatch(action, args)
        except (ValueError, RuntimeError, OSError) as exc:
            return {'state': 'error', 'code': str(exc), 'error': str(exc)}

    def start(self, args=None):
        args = args or {}
        if not args.get('input_topic'):
            return {**self.info(), 'state': 'waiting_binding', 'reason': 'input_topic_required'}
        binding = {'command_topic': args['input_topic'], 'feedback_topic': '/teleop/state'}
        with self._lock:
            same_topics = bool(self.binding and all(self.binding.get(k) == binding[k]
                               for k in ('command_topic', 'feedback_topic')))
            if same_topics:
                if not self._closed.is_set() and self._thread and self._thread.is_alive():
                    return self.info()
            if self.binding and not same_topics and self._operator:
                raise ValueError('binding_requires_idle')
            if not same_topics or self._closed.is_set():
                self._latest = self._input_identity = None
                self._input_sequence = -1
                self.mapping.anchor = None
            self.binding = binding
            self.instance_id = args.get('instance_id') or 'teleop_control'
            self._closed.clear()
            self._buttons.clear()
            self._state, self._reason = 'ready', 'waiting_baseline_confirmation'
        self.motion.start()
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._run, daemon=True, name='g1-teleop-control')
                self._thread.start()
        return self.info()

    @staticmethod
    def _identity(frame):
        return frame['device_id'], frame['connection_epoch'], frame['space_epoch']

    def receive(self, value):
        if not self.binding:return False
        if value.get('kind') != 'input':
            raise ValueError('device_input_only')
        if self._closed.is_set():return False
        frame = validate_input(value, clock_id=self.clock_id,
                               now_ns=self.clock())
        identity = self._identity(frame)
        with self._lock:
            old = self._input_identity
            if old and identity[0] == old[0] and (identity[1] < old[1] or identity[2] < old[2]):
                raise ValueError('old_input_generation')
            if identity == old and frame['sequence'] <= self._input_sequence: return False
            self._input_identity, self._input_sequence = identity, frame['sequence']
            source_changed = self._latest and self._latest['instance_id'] != frame['instance_id']
            self._latest = frame
            if old and (identity != old or source_changed):
                self._buttons.clear()
                self._needs_calibration = True
                self._input_changed = True
                self._ready_on_release = False
            if self._mapping_identity and (identity[0], identity[2]) != self._mapping_identity:
                self._needs_calibration = True
            self._wake.set()
        return True

    def _check_generation(self, generation):
        if generation != self._generation or self._closed.is_set(): raise ValueError('operation_cancelled')

    def _latest_fresh(self):
        frame = copy.deepcopy(self._latest)
        if frame is None: raise ValueError('input_missing')
        frame = validate_input(frame, clock_id=self.clock_id,
                                now_ns=self.clock())
        if not all(frame[k]['tracked'] for k in ('head_reference','left','right')):
            raise ValueError('tracking_lost')
        return frame

    def _begin(self, generation, action, identity=None):
        # Only begin/calibrate serialize here. Stop never waits for cold setup.
        with self._begin_lock:
            return self._begin_locked(generation, action, identity)

    def _begin_locked(self, generation, action, identity):
        with self._lock:
            self._check_generation(generation)
            identity = identity or self._identity(self._latest_fresh())
            if identity != self._input_identity:raise ValueError('operation_input_generation_changed')
            if action == 'begin' and self._operator and not self._needs_calibration:
                return {'operator_session_id': self._operator, 'state': self._state}
            recalibrate = action == 'calibrate' or self._needs_calibration
            if recalibrate:self._operator = None
        if recalibrate:
            self._stop_motion(operator_confirmation=True)
            self._check_generation(generation)
        if (self.cfg.get('calibration_path')
                and self.executor.cfg.get('calibration_path') != self.cfg['calibration_path']):
            self.motion.configure({k:self.cfg[k] for k in ('calibration_path','joint_velocity_rad_s') if k in self.cfg})
        calibrated = self.motion.dispatch('calibrate', {})
        if calibrated.get('error'): raise ValueError(calibrated.get('code', calibrated['error']))
        with self._lock:
            self._check_generation(generation)
            if identity != self._input_identity:raise ValueError('operation_input_generation_changed')
        prepared = False
        try:
            if self.live:
                result = self.motion.dispatch('prepare_operator_session', {})
                if result.get('error'): raise ValueError(result.get('code', result['error']))
            result = self.motion.dispatch('prepare_preview', {})
            if result.get('error'):raise ValueError(result.get('code', result['error']))
            prepared = True
            self._check_generation(generation)
            # Both expensive model preparation AND execution preparation are
            # complete. FK and the source sample must be refreshed after them.
            self.motion._refresh_snapshot()
            with self._lock:
                self._check_generation(generation)
                frame = self._latest_fresh()
                if identity != self._identity(frame) or self._input_changed:
                    raise ValueError('operation_input_generation_changed')
                if not self._pressed(frame, 'x', 'a') or any(frame[s]['grip'] >= .5 for s in ('left', 'right')):
                    raise ValueError('baseline_confirmation_released')
                self.executor._fresh()
                snap = copy.deepcopy(self.motion._snapshot)
                if not snap or not 0 <= self.clock()-snap['monotonic_ns'] <= 100_000_000:
                    raise ValueError('arm_feedback_stale')
                self.mapping.set_controller_offsets(self.motion.solver.profile.get('controller_to_palm', {}))
                self.mapping.calibrate(frame, snap['poses'])
                self._mapping_identity = (frame['device_id'], frame['space_epoch'])
                self._needs_calibration = False
                self._operator = secrets.token_hex(16)
                self._release_seen = not all(frame[s]['grip'] >= .5 for s in ('left','right'))
                self._paused = False
                self._gripped = self._enabled = self._ready_on_release = False
                self._engage_sequence = None
                self._processed = None
                self._hold_kind = None
                self._sequence = 0
                self._state, self._reason = 'ready', 'release_buttons_then_regrip'
                return {'operator_session_id': self._operator, 'mapping_epoch': self.mapping.epoch,
                        'state': 'ready', 'output_active': False}
        except Exception:
            if prepared:self._stop_motion()
            raise

    def _credentials(self):
        try: return self.motion._lease()
        except ValueError: return {}

    def _hold(self, reason, *, grip=False):
        with self._lock:
            self._state, self._reason = 'hold', reason
            self._enabled = self._ready_on_release = False
            self._engage_sequence = None
            if self.trajectory.recorder:
                self.trajectory.recorder.pause()
            if grip: self._paused = True
            kind = 'pause' if grip else 'recoverable_hold'
            if self._hold_kind == (kind, reason): return
            self.motion.cancel_pending()
            credentials = self._credentials()
            if credentials:
                result = self.motion.dispatch(kind, credentials)
                if result.get('error'): raise ValueError(result.get('code', result['error']))
            self._hold_kind = (kind, reason)

    @staticmethod
    def _pressed(frame, left, right):
        def pressed(side, name):
            value = frame[side].get('controls', {}).get('buttons', {}).get(name, {})
            return value.get('available') is True and value.get('pressed') is True
        return pressed('left', left) and pressed('right', right)

    def _button_event(self, frame, name, pressed):
        # Count distinct fresh samples; one held/replayed sample cannot confirm.
        if not pressed:
            self._buttons.pop(name, None)
            return False
        stamp = frame['received_monotonic_ns']
        start, last, fired = self._buttons.get(name, (stamp, stamp, False))
        if stamp-last > 300_000_000: start = stamp
        event = not fired and stamp-start >= 1_000_000_000
        self._buttons[name] = (start, stamp, fired or event)
        return event

    def _start_reset(self):
        if not self.live: raise ValueError('reset_requires_live')
        if self.trajectory.busy: raise ValueError('replay_active')
        self._generation += 1
        self._operator = None
        self._needs_calibration = True
        self.mapping.anchor = None
        self._enabled = self._ready_on_release = False
        self._engage_sequence = None
        self.motion.cancel_pending()
        with self.motion._lock:
            self.motion._preview = None
            self.motion._preview_state = 'idle'
        self.trajectory.record_stop()
        self.executor.cancel_preparation()
        result = self.executor.dispatch('operator_reset', {})
        if result.get('error'): raise ValueError(result['error'])
        self._reset_operation = result['operation_id']
        self._state, self._reason = 'hold', 'resetting'

    def _poll_reset(self):
        result = self.executor.dispatch('finish_status', {'operation_id': self._reset_operation})
        if result.get('error') or result['state'] in ('failed', 'unknown'):
            self._reset_operation = None
            raise ValueError(result.get('reason') or result.get('error') or 'reset_failed')
        if result['state'] == 'completed':
            if not result.get('request_sent') or not result.get('authority_released'):
                raise ValueError('reset_request_unconfirmed')
            self._reset_operation = None
            stamp = self._latest['received_monotonic_ns']
            self._buttons['confirm'] = (stamp, stamp, True)
            self._state, self._reason = 'ready', 'waiting_baseline_confirmation'
        else:
            self._state, self._reason = 'hold', 'resetting'

    def step(self):
        try:
            with self._lock:
                if self._closed.is_set() or not self.binding: return
                generation = self._generation
                frame = self._latest_fresh()
                if self._input_changed:
                    self._input_changed = False
                    if self._operator: self._hold('input_generation_changed', grip=True)
                released = all(frame[s]['grip'] < .5 for s in ('left', 'right'))
                reset = self._pressed(frame, 'y', 'b')
                confirm = self._pressed(frame, 'x', 'a')
                reset_event = self._button_event(frame, 'reset', reset and released and not confirm)
                confirm_event = self._button_event(frame, 'confirm', confirm and released and not reset)
                if reset_event:
                    self._start_reset()
                    return
                if self._reset_operation and not confirm_event:
                    self._poll_reset()
                    return
                initialize = confirm_event or not self._operator or self._needs_calibration
                if initialize and not confirm_event:
                    if self._state != 'hold' or self._reason in self.INPUT_HOLDS:
                        self._state, self._reason = 'ready', 'waiting_baseline_confirmation'
                    return
                if not initialize and (reset or confirm):
                    if self._enabled: self._hold('operator_pause', grip=True)
                    self._gripped = self._ready_on_release = False
                    self._engage_sequence = None
                    self._state, self._reason = 'ready', 'release_buttons_then_regrip'
                    return
            if initialize:
                self._begin(generation, 'calibrate' if confirm_event or self._needs_calibration else 'begin')
                return
        except (ValueError, RuntimeError, OSError) as exc:
            with self._lock:
                reason = str(exc)
                if self._reset_operation:
                    stopped = self.executor.dispatch('stop', {})
                    self._reset_operation = None
                    reason = stopped.get('reason') or stopped.get('error') or 'reset_interrupted'
                # A later input gap must not replace an unresolved SDK/reset failure.
                if (reason in self.INPUT_HOLDS and self._state == 'hold'
                        and self._reason not in self.INPUT_HOLDS):
                    return
                if self._operator: self._hold(reason, grip=True)
                else: self._state, self._reason = 'hold', reason
            return
        try:
            with self._lock:
                self._check_generation(generation)
                if self._needs_calibration: raise ValueError('needs_calibration')
                frame = self._latest_fresh()
                held = all(frame[s]['grip'] >= .5 for s in ('left', 'right'))
                rising = held and not self._gripped
                self._gripped = held
                key = (*self._identity(frame), frame['sequence'])
                decision = self.motion._decision
                solved = (decision.get('state') == 'preview'
                    and 0 <= self.clock()-decision.get('monotonic_ns', 0) <= 300_000_000)
                if not held:
                    if self._enabled:
                        self._hold('operator_pause', grip=True)
                    self._engage_sequence = None
                    self._ready_on_release = True
                    self._release_seen = True
                elif not self._enabled:
                    if rising:
                        self._engage_sequence = frame['sequence'] if self._ready_on_release else None
                        self._ready_on_release = False
                    elif self._engage_sequence is not None:
                        # The grip-edge pose must pass IK before live output.
                        if decision.get('source_seq', -1) >= self._engage_sequence:
                            self._enabled = solved
                            if solved: self._processed = None
                            self._engage_sequence = None
                if self._enabled and self.motion._decision.get('state') in ('hold', 'error'):
                    self._hold(self.motion._decision.get('reason') or 'ik_unavailable', grip=True)
                if not self._enabled:
                    self._state, self._reason = 'ready', 'regrip_required'
                    if key != self._processed and self._engage_sequence in (None, frame['sequence']):
                        self._submit(frame, preview=True)
                    return
                if key == self._processed: return
            # SDK acquisition/resume must not own the input/operation lock:
            # cancellation can fence us while a channel is still opening.
            state = self.motion.gate.status()
            if self.live:
                if state['state'] == 'fault': raise ValueError('driver_fault')
                if not state['ownership_held']:
                    claimed = self.motion.dispatch('claim', {})
                    if claimed.get('error'): raise ValueError(claimed.get('code', claimed['error']))
                    with self._lock: self._sequence = 0
                elif state['state'] == 'hold':
                    if not state['hold_confirmed'] and not state.get('continuation_ready') and not state.get('resume_ready'):
                        with self._lock: self._state, self._reason = 'hold', 'waiting_hold_confirmation'
                        return
                    if self._paused or not state.get('continuation_ready'):
                        resumed = self.motion.dispatch('resume', self._credentials())
                        if resumed.get('error'): raise ValueError(resumed.get('code', resumed['error']))
                        with self._lock: self._sequence = 0
            elif self._paused and self.motion._preview_state == 'hold':
                resumed = self.motion.dispatch('resume', self._credentials())
                if resumed.get('error'): raise ValueError(resumed.get('code', resumed['error']))
                with self._lock: self._sequence = 0
            with self._lock:
                self._check_generation(generation)
                self._submit(frame, preview=False)
                self._paused, self._hold_kind = False, None
                self._state, self._reason = 'active', None
        except (ValueError, RuntimeError, OSError) as exc:
            with self._lock:
                if generation != self._generation or self._closed.is_set(): return
            code = str(exc)
            self._hold(code, grip=True)

    def _submit(self, frame, *, preview):
        now = self.clock()
        self._sequence += 1
        command = envelope(self.motion._lease(), seq=self._sequence,
            source_seq=frame['sequence'], mapping_epoch=self.mapping.epoch,
            generated_ns=now, valid_until_ns=min(now+300_000_000, frame['received_monotonic_ns']+300_000_000),
            mode='eef_pose', values=self.mapping.targets(frame), **self.motion.versions)
        self.motion.receive_eef(command, preview_only=preview)
        self._processed = (*self._identity(frame), frame['sequence'])

    def display_feedback(self):
        visual = copy.deepcopy(getattr(self.motion, '_visualization', {'available': False}))
        alignment = visual.pop('alignment', {'state': 'unavailable', 'ready': False})
        now = self.clock()
        try:
            self._latest_fresh()
            valid = (not self._closed.is_set() and not self._needs_calibration
                and not self._input_changed
                and visual.get('mapping_epoch') == self.mapping.epoch
                and 0 <= now-visual.get('sampled_ns', 0) <= 100_000_000
                and now < visual.get('valid_until_ns', 0))
        except ValueError:
            valid = False
        if not valid:
            visual.update(available=False, ik=[])
            alignment = {'state': 'unavailable', 'ready': False}
        else:
            visual['intent_age_ms'] = max(0., (now-visual['intent_received_ns'])/1e6)
            if self._enabled and alignment['state'] not in ('unavailable', 'unreachable'):
                alignment.update(state='active', ready=False)
        return {'visualization': visual, 'alignment': alignment}

    def _run(self):
        while not self._closed.is_set():
            self._wake.wait(.05); self._wake.clear()
            try: self.step()
            except Exception as exc:
                with self._lock: self._state, self._reason = 'fault', str(exc)
            self._closed.wait(.005)

    def _stop_motion(self, *, operator_confirmation=False):
        self.motion.cancel_pending()
        with self.motion._lock:
            self.motion._preview = None
            self.motion._preview_state = 'idle'
        # Independent of the model-management lock. This also fences a claim
        # still opening its SDK channel and a cold begin not yet prepared.
        self.executor.cancel_preparation()
        state = self.executor.status()
        if not state['ownership_held'] and not self.executor.opening and not self._reset_operation:
            result = {'state': 'idle', 'authority_released': True, 'return_required': False}
        else:
            confirmed = operator_confirmation and self.executor.confirm_operator_reset()
            result = {} if confirmed else self.executor.dispatch('stop', {})
            if result.get('error'): raise ValueError(result.get('code', result['error']))
            deadline = time.monotonic()+12.
            while self.executor.status()['ownership_held'] and time.monotonic() < deadline:
                if confirmed:self.executor.confirm_operator_reset()
                time.sleep(.02)
            state = self.executor.status()
            if state['ownership_held']: raise ValueError(state['reason'] or 'stop_unconfirmed')
            result = {'state': 'idle', 'authority_released': True, 'return_required': False}
        with self._lock:
            self._reset_operation = None
            self._state, self._reason = 'idle', None
            self._paused = False
        return result

    def cancel_for_arm_release(self):
        self.trajectory.stop()
        self.trajectory.record_stop()
        # The existing arm.release is an explicit operator cancellation, not a
        # global controller arbiter. Stop accepting teleop before SDK handback.
        with self._lock:
            self._generation += 1
            self._enabled = self._ready_on_release = False
            self._engage_sequence = None
            self._operator = None
            self._latest = None
            self._closed.set()
            self._wake.set()
            self._state, self._reason = 'idle', 'arm_release'
        self.motion.cancel_pending()

    def stop(self):
        with self._lock:
            self.trajectory.stop()
            self._buttons.clear()
            self._generation += 1
            self._enabled = self._ready_on_release = False
            self._engage_sequence = None
            self._operator = None
            self._closed.set(); self._wake.set()
        try:
            result = self._stop_motion()
        finally:
            self.trajectory.record_stop()
        worker = self._thread
        if worker and worker is not threading.current_thread(): worker.join(.5)
        return result

    def input_status(self):
        transport = self.transport_status() if self.transport_status else None
        age = None if self._latest is None else max(0., (self.clock()-self._latest['received_monotonic_ns'])/1e6)
        if not self.binding:
            state, hint = 'waiting_binding', 'Driver 重启后请在 Canvas 停止并重新启动项目；无需重新连线'
        elif self._closed.is_set():
            state, hint = 'stopped', '请从 Canvas 启动项目'
        elif transport and not transport['executor']['healthy']:
            state, hint = 'transport_error', 'ROS 接收线程异常，请查看 transport 诊断'
        elif age is None:
            state, hint = 'waiting_input', '已绑定，等待 teleop_device 新输入'
        elif age > 300.:
            state, hint = 'input_stale', '没有新的有效输入，请查看收帧序号与拒绝原因'
        else:
            state, hint = 'fresh', ''
        return {'state': state, 'hint': hint, 'age_ms': age, 'transport': transport}

    def feedback(self):
        with self._lock:
            self._feedback_sequence += 1
            state = self.motion.gate.status()
            execution = {k:copy.deepcopy(state[k]) for k in ('state','reason','output_active','ownership_held',
                'hold_confirmed','stop_confirmed','applied_sequence','feedback')}
            execution.update(armed=bool(self.binding and not self._closed.is_set()),
                             started=bool(self._operator), mode=self.cfg.get('mode', 'live'))
            execution['decision'] = copy.deepcopy(self.motion._decision)
            execution['finish'] = copy.deepcopy(self.motion._finish)
            input_status = self.input_status()
            return {'schema': FEEDBACK_SCHEMA,
                'text': html.escape(f"遥操：{self._state} | 输入：{input_status['state']} | 输入帧：{self._input_sequence} | 执行：{state['state']} | 已下发：{state['applied_sequence']} | 输出：{state['output_active']} | 原因：{self._reason or state['reason'] or '无'}"),
                'instance_id': self._latest['instance_id'] if self._latest else '',
                'control_instance_id': self.instance_id, 'server_epoch': self.server_epoch,
                'sequence': self._feedback_sequence, 'emitted_monotonic_ns': self.clock(),
                'clock_id': self.clock_id, 'source_sequence': self._input_sequence,
                'connection_epoch': self._input_identity[1] if self._input_identity else 0,
                'space_epoch': self._input_identity[2] if self._input_identity else 0,
                'operator_session_id': self._operator, 'mapping_epoch': self.mapping.epoch,
                'state': self._state, 'reason': self._reason, 'capabilities': list(self.CAPABILITIES),
                'input_status': input_status, 'execution': execution, 'receipts': [], **self.display_feedback()}

    def can_record(self):
        if not self.live or self._closed.is_set() or not self._operator or not self.mapping.anchor or self._needs_calibration:
            return False
        try:
            self._latest_fresh()
            return not self._input_changed and self._state in ('ready', 'active')
        except ValueError:
            return False

    def info(self):
        with self._lock:
            return {'state': self._state, 'reason': self._reason, 'mode': self.cfg.get('mode', 'live'),
                'config': copy.deepcopy(self.cfg),
                'effective_config': {k:self.cfg.get(k,v.get('default')) for k,v in self.get_tool()['configSchema']['properties'].items()},
                'binding': copy.deepcopy(self.binding),
                'input_status': self.input_status(),
                'mapping_epoch': self.mapping.epoch, 'operator_session_id': self._operator,
                **self.trajectory.info(), 'input_fresh': self.input_status()['state'] == 'fresh',
                'mapped': bool(self.mapping.anchor and not self._needs_calibration), 'can_record': self.can_record(),
                'capabilities': list(self.CAPABILITIES), 'feedback': self.feedback() if self.binding else None,
                **{k:self.get_tool()[k] for k in ('topic_in','topic_out')}}
