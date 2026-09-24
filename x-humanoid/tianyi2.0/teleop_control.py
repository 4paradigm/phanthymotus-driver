"""Two-card Tianyi operator session; only the existing MotionGate emits motion.

Mapping derives from the former ActuCore DualArmMapping (Apache-2.0). A grip is
an enable signal, never a new coordinate calibration. Numerical work remains in
MotionControl's isolated worker, and DDS in the existing local-profile helper.
"""
import copy
import json
import math
import os
import tempfile
from pathlib import Path
import secrets
import threading
import time

from common.teleop_contract import (binding_from_topic, topics, validate_input, canonical_instance,
                                    FEEDBACK_SCHEMA)
from tianyi_motion.protocol import envelope


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

    def __init__(self, cfg, motion, *, clock=time.monotonic_ns):
        self.cfg, self.motion, self.executor = cfg, motion, motion.executor
        self.clock, self.clock_id = clock, motion.clock_id
        self.server_epoch = secrets.token_hex(16)
        self.mapping = RelativeMapping(cfg.get('position_scale', .5))
        self._lock = threading.RLock()
        self._begin_lock = threading.Lock()
        self._closed, self._wake = threading.Event(), threading.Event()
        self._thread = None
        self.binding = None
        self.instance_id = 'teleop_control'
        self._latest = None
        self._input_identity = None
        self._input_sequence = -1
        self._mapping_identity = None
        self._processed = None
        self._generation = self._sequence = self._feedback_sequence = 0
        self._operator = None
        self._release_seen = False
        self._paused = False
        self._hold_kind = None
        self._needs_calibration = False
        self._state, self._reason = 'idle', None
        self._config_error = None
        self._auto_retry_after_ns = 0
        self._config_path = Path(cfg.get('state_path', '/opt/phanthy-motus/data/teleop-control.json'))
        try:
            if self._config_path.is_file():
                saved = json.loads(self._config_path.read_text())
                keys = {'mode','calibration_path','position_scale','joint_velocity_rad_s','joint_acceleration_rad_s2','trajectory_smoothing','usage_guide'}
                if not isinstance(saved, dict) or set(saved)-set(keys):raise ValueError('invalid_saved_config')
                # Deployment presets own motion parameters; legacy UI values are ignored.
        except (ValueError, OSError) as exc:
            self._config_error = str(exc)
        self.motion.gate.live_enabled = self.live
        self.executor.cfg['live_enabled'] = self.live
        self.executor.cfg['operator_session_enabled'] = self.live
        self.motion.gate.smoothing_seconds = .12
        self.motion.gate.resume_without_settle = True

    @property
    def live(self): return self.cfg.get('mode', 'live') == 'live'

    def get_tool(self):
        fields = {'usage_guide': {'type': 'string', 'title': '使用说明（固定，无需配置）',
                    'enum': ['无需配置'], 'default': '无需配置', 'scope': 'instance'}}
        return {'name': 'teleop_control', 'type': 'actuator', 'multiInstance': False,
            'description': '天轶双臂遥操：先安装遥操设备 Driver，在画布添加 teleop_device 并连接本卡；在设备卡齿轮页获取 App 下载和配对入口。启动项目后松开双握把建立初始基准，再按住双握把跟随。松握保持，再握不重标定；在监控面板查看执行状态。停止项目停止遥操；收臂使用 arm_gesture reset（both）。',
            'configSchema': {'type': 'object', 'additionalProperties': False, 'properties': fields},
            'inputSchema': {'type': 'object', 'required': ['action'],
                'properties': {'action': {'type': 'string', 'enum': ['info', 'config', 'start', 'stop']},
                    'input_topic': {'type': 'string'}},
                'x-resource': ['arm_l', 'arm_r'],
                'x-action-params': {'info': {'params': []}, 'config': {'params': []},
                    'start': {'params': ['input_topic']}, 'stop': {'params': []}}},
            'topic_in': [{'port_id': 'command', 'format': 'data/teleop-cmd',
                **({'topic': self.binding['command_topic']} if self.binding else {})}],
            'topic_out': [{'port_id': 'state', 'format': 'data/teleop-state', 'topic': '/teleop/state'}]}

    @staticmethod
    def _validate_config(candidate):
        if candidate.get('mode', 'shadow') not in ('shadow', 'live'): raise ValueError('invalid_mode')
        for name, default, upper in [('position_scale', .5, 1.), ('joint_velocity_rad_s', 1., 1.5),
                                      ('joint_acceleration_rad_s2', 2., 10.)]:
            value = candidate.get(name, default)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= upper:
                raise ValueError('invalid_'+name)
        if type(candidate.get('trajectory_smoothing', False)) is not bool: raise ValueError('invalid_trajectory_smoothing')

    def configure(self, args):
        keys = self.get_tool()['configSchema']['properties']
        candidate = {**self.cfg, **{k:v for k,v in args.items() if k in keys}}
        self._validate_config(candidate)
        with self._lock:
            if self._operator or self.motion.gate.session_id:
                raise ValueError('configuration_requires_idle')
            old_motion = self.motion.config_snapshot()
            self._config_path.parent.mkdir(parents=True, exist_ok=True)
            fd, temporary = tempfile.mkstemp(prefix='.teleop-config-', dir=self._config_path.parent)
            try:
                with os.fdopen(fd, 'w') as stream:
                    json.dump({k:candidate[k] for k in keys if k in candidate}, stream, allow_nan=False)
                    stream.flush(); os.fsync(stream.fileno())
            except Exception:
                Path(temporary).unlink(missing_ok=True)
                raise
            try:
                if candidate.get('calibration_path'):
                    self.motion.configure({k:candidate[k] for k in ('calibration_path','joint_velocity_rad_s') if k in candidate})
                os.replace(temporary, self._config_path)
            except Exception:
                Path(temporary).unlink(missing_ok=True)
                if old_motion.get('calibration_path'):
                    self.motion.configure(old_motion)
                raise
            # Values are applied only after the model/limits validation succeeds.
            self.cfg.clear(); self.cfg.update(candidate)
            self.mapping.scale = candidate.get('position_scale', .5)
            self.motion.gate.live_enabled = self.live
            self.executor.cfg['live_enabled'] = self.live
            self.executor.cfg['operator_session_enabled'] = self.live
            self.motion.gate.acceleration = (candidate.get('joint_acceleration_rad_s2', 2.)
                                             if candidate.get('trajectory_smoothing', False) else None)
            self.mapping.anchor = None
            self._config_error = None
            return self.info()

    def dispatch(self, action, args):
        try:
            if action == 'info': return self.info()
            if action == 'config': return self.configure(args)
            if action == 'start': return self.start(args)
            if action == 'stop': return self.stop()
            raise ValueError('unknown_action')
        except (ValueError, RuntimeError, OSError) as exc:
            if action == 'config': self._config_error = str(exc)
            return {'state': 'error', 'code': str(exc), 'error': str(exc)}

    def start(self, args=None):
        args = args or {}
        if not args.get('input_topic'):
            return {**self.info(), 'state': 'waiting_binding', 'reason': 'input_topic_required'}
        if args['input_topic'] == '/teleop/command':
            namespace, source_instance = '', None
            command_topic, feedback_topic = '/teleop/command', '/teleop/state'
        else:
            namespace, source_instance = binding_from_topic(args['input_topic'])
            command_topic, _ = topics(namespace, source_instance)
            feedback_topic = '/teleop/state'
        binding = {'namespace': namespace, 'instance_id': source_instance,
                   'command_topic': command_topic, 'feedback_topic': feedback_topic}
        with self._lock:
            same_topics = bool(self.binding and all(self.binding.get(k) == binding[k]
                               for k in ('command_topic', 'feedback_topic')))
            if same_topics:
                # Source pinning is not a transport rebind. Repeated Canvas
                # start must not erase identity or remap an active operator.
                binding['instance_id'] = self.binding['instance_id']
                if not self._closed.is_set() and self._thread and self._thread.is_alive():
                    return self.info()
            if self.binding and not same_topics and self._operator:
                raise ValueError('binding_requires_idle')
            if not same_topics or self._closed.is_set():
                self._latest = self._input_identity = None
                self._input_sequence = -1
                self.mapping.anchor = None
                self._needs_calibration = False
                self._mapping_identity = None
            self.binding = binding
            self.instance_id = args.get('instance_id') or 'teleop_control'
            self._closed.clear()
            self._state, self._reason = 'ready', 'waiting_input'
        self.motion.start()
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._run, daemon=True, name='tianyi-teleop-control')
                self._thread.start()
        return self.info()

    @staticmethod
    def _identity(frame):
        return frame['device_id'], frame['connection_epoch'], frame['space_epoch']

    def receive(self, value):
        if not self.binding:return False
        # The single-device topic carries identity in JSON, not in its path.
        if value.get('kind') != 'input':
            raise ValueError('device_input_only')
        with self._lock:
            if self.binding['instance_id'] is None:
                source = canonical_instance(value.get('instance_id'))
                validate_input(value, instance_id=source, clock_id=self.clock_id, now_ns=self.clock())
                self.binding['instance_id'] = source
        if self._closed.is_set():return False
        frame = validate_input(value, instance_id=self.binding['instance_id'], clock_id=self.clock_id,
                               now_ns=self.clock())
        identity = self._identity(frame)
        with self._lock:
            old = self._input_identity
            if old and (identity[0] != old[0] and self._operator or
                        identity[0] == old[0] and (identity[1] < old[1] or identity[2] < old[2])):
                raise ValueError('old_input_generation')
            if identity == old and frame['sequence'] <= self._input_sequence: return False
            self._input_identity, self._input_sequence = identity, frame['sequence']
            self._latest = frame
            if self._mapping_identity and (identity[0], identity[2]) != self._mapping_identity:
                self._needs_calibration = True
            self._wake.set()
        return True


    def _check_generation(self, generation):
        if generation != self._generation or self._closed.is_set(): raise ValueError('operation_cancelled')


    def _latest_fresh(self):
        frame = copy.deepcopy(self._latest)
        if frame is None: raise ValueError('input_missing')
        frame = validate_input(frame, instance_id=self.binding['instance_id'], clock_id=self.clock_id,
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
            self._stop_motion()
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
            action_name = 'prepare_operator_session' if self.live else 'prepare_preview'
            result = self.motion.dispatch(action_name, {})
            if result.get('error'):raise ValueError(result.get('code', result['error']))
            prepared = True
            self._check_generation(generation)
            # Both expensive model preparation AND execution preparation are
            # complete. FK and the source sample must be refreshed after them.
            self.motion._refresh_snapshot()
            with self._lock:
                self._check_generation(generation)
                frame = self._latest_fresh()
                if identity != self._identity(frame):raise ValueError('operation_input_generation_changed')
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
                self._processed = None
                self._hold_kind = None
                self._sequence = 0
                self._state, self._reason = 'ready', None
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
            if grip: self._paused = True
            kind = 'pause' if grip else 'recoverable_hold'
            if self._hold_kind == kind: return
            self.motion.cancel_pending()
            credentials = self._credentials()
            if credentials:
                result = self.motion.dispatch(kind, credentials)
                if result.get('error'): raise ValueError(result.get('code', result['error']))
            self._hold_kind = kind

    def step(self):
        with self._lock:
            if self._closed.is_set() or not self.binding: return
            generation = self._generation
            initialize = not self._operator
            if self._needs_calibration:
                self._state, self._reason = "hold", "needs_calibration"
                return
        if initialize:
            if self.clock() < self._auto_retry_after_ns: return
            try:
                frame = self._latest_fresh()
                if any(frame[s]['grip'] >= .5 for s in ('left', 'right')):
                    with self._lock: self._state, self._reason = 'ready', 'release_grips_before_enable'
                    return
                self._begin(generation, 'begin')
            except (ValueError, RuntimeError, OSError) as exc:
                with self._lock: self._state, self._reason = 'hold', str(exc)
                self._auto_retry_after_ns = self.clock() + 500_000_000
            return
        try:
            with self._lock:
                self._check_generation(generation)
                if self._needs_calibration: raise ValueError('needs_calibration')
                frame = self._latest_fresh()
                if not all(frame[s]['grip'] >= .5 for s in ('left','right')):
                    self._release_seen = True
                    hold_reason = 'operator_pause'
                elif not self._release_seen:
                    hold_reason = 'release_grips_before_enable'
                else: hold_reason = None
                key = (*self._identity(frame), frame['sequence'])
                if not hold_reason and key == self._processed: return
            if hold_reason:
                self._hold(hold_reason, grip=True); return
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
                now = self.clock()
                values = self.mapping.targets(frame)
                self._sequence += 1
                command = envelope(self.motion._lease(), seq=self._sequence,
                    source_seq=frame['sequence'], mapping_epoch=self.mapping.epoch,
                    generated_ns=now, valid_until_ns=min(now+300_000_000, frame['received_monotonic_ns']+300_000_000),
                    mode='eef_pose', values=values, **self.motion.versions)
                self.motion.receive_eef(command)
                self._processed = key
                self._paused, self._hold_kind = False, None
                self._state, self._reason = 'active', None
        except (ValueError, RuntimeError, OSError) as exc:
            with self._lock:
                if generation != self._generation or self._closed.is_set(): return
            code = str(exc)
            self._hold(code, grip=code in ('driver_fault','needs_calibration'))

    def _run(self):
        while not self._closed.is_set():
            self._wake.wait(.05); self._wake.clear()
            try: self.step()
            except Exception as exc:
                with self._lock: self._state, self._reason = 'fault', str(exc)
            self._closed.wait(.005)

    def _stop_motion(self):
        self.motion.cancel_pending()
        self.motion._finish_cancel.set()
        # Do not acquire MotionControl's management lock: model load can own
        # it for seconds. The executor release path is independent of IK.
        with self.motion._lock:
            if self.motion._preview is not None:
                self.motion._preview = None
                self.motion._preview_state = 'idle'
        result = self.executor.dispatch('release', self._credentials())
        if result.get('error'): raise ValueError(result.get('code', result['error']))
        deadline = time.monotonic()+3.
        while self.motion.gate.session_id and time.monotonic() < deadline:
            time.sleep(.02)
        state = self.motion.gate.status()
        if state['ownership_held']: raise ValueError('stop_unconfirmed')
        ended = self.executor.dispatch('end_operator_session', {})
        if ended.get('error'): raise ValueError(ended.get('code', ended['error']))
        with self._lock:
            self._state, self._reason = 'idle', None
            self._paused = False
        return {'state': 'idle', 'authority_released': True, 'return_required': False}


    def stop(self):
        with self._lock:
            self._generation += 1
            self._operator = None
            self._closed.set(); self._wake.set()
        result = self._stop_motion()
        worker = self._thread
        if worker and worker is not threading.current_thread(): worker.join(.5)
        return result

    def feedback(self):
        with self._lock:
            self._feedback_sequence += 1
            state = self.motion.gate.status()
            execution = {k:copy.deepcopy(state[k]) for k in ('state','reason','output_active','ownership_held',
                'hold_confirmed','stop_confirmed','applied_sequence','feedback')}
            execution.update(armed=bool(self.binding and not self._closed.is_set()),
                             started=bool(self._operator), mode=self.cfg.get('mode', 'live'))
            execution['decision'] = copy.deepcopy(self.motion._decision)
            execution['transport'] = copy.deepcopy(getattr(self.executor, '_bus_health', {}))
            execution['finish'] = copy.deepcopy(self.motion._finish)
            return {'schema': FEEDBACK_SCHEMA, 'instance_id': self.binding['instance_id'] if self.binding else '',
                'control_instance_id': self.instance_id, 'server_epoch': self.server_epoch,
                'sequence': self._feedback_sequence, 'emitted_monotonic_ns': self.clock(),
                'clock_id': self.clock_id, 'source_sequence': self._input_sequence,
                'connection_epoch': self._input_identity[1] if self._input_identity else 0,
                'space_epoch': self._input_identity[2] if self._input_identity else 0,
                'operator_session_id': self._operator, 'mapping_epoch': self.mapping.epoch,
                'state': self._state, 'reason': self._reason, 'capabilities': list(self.CAPABILITIES),
                'execution': execution, 'receipts': []}

    def info(self):
        with self._lock:
            return {'state': self._state, 'reason': self._reason, 'mode': self.cfg.get('mode', 'live'),
                'config': copy.deepcopy(self.cfg),
                'effective_config': {k:self.cfg.get(k,v.get('default')) for k,v in self.get_tool()['configSchema']['properties'].items()},
                'config_error': self._config_error, 'binding': copy.deepcopy(self.binding),
                'mapping_epoch': self.mapping.epoch, 'operator_session_id': self._operator,
                'capabilities': list(self.CAPABILITIES), 'feedback': self.feedback() if self.binding else None,
                **{k:self.get_tool()[k] for k in ('topic_in','topic_out')}}
