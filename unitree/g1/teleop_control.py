"""G1 two-card operator session; only ArmStreamExecutor emits hardware commands.

Session admission/receipts derive from Tianyi PR321 49997cc6 (Apache-2.0).
G1 uses ten arm joints and the existing release/action 99, not Tianyi homing.

Mapping derives from the former ActuCore DualArmMapping (Apache-2.0). A grip is
an enable signal, never a new coordinate calibration. Numerical work remains in
MotionControl's isolated worker, and DDS in the existing local-profile helper.
"""
from collections import OrderedDict
import copy
import html
import json
import math
import os
import tempfile
from pathlib import Path
import secrets
import threading
import time

from common.teleop_contract import (binding_from_topic, topics, validate_input, validate_operation,
                                    FEEDBACK_SCHEMA, canonical_instance)
from common.motion.protocol import envelope


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
        self._input_sequence = self._operation_sequence = -1
        self._operation_identity = None
        self._mapping_identity = None
        self._processed = None
        self._generation = self._sequence = self._feedback_sequence = 0
        self._operator = None
        self._release_seen = False
        self._paused = False
        self._hold_kind = None
        self._needs_calibration = False
        self._state, self._reason = 'idle', None
        self._receipts = OrderedDict()
        self._operation_packets = {}
        self._active_operation = None
        self._operation_threads = set()
        self._stop_operation = None
        self._stop_aliases = set()
        self._auto_retry_after_ns = 0
        self._config_error = None
        self._config_path = Path(cfg.get('state_path', '/opt/phanthy-motus/data/g1-teleop-control.json'))
        try:
            if self._config_path.is_file():
                saved = json.loads(self._config_path.read_text())
                # Legacy Canvas preferences must not override the robot deployment preset.
                keys = {'mode', 'calibration_path', 'position_scale', 'usage_guide'}
                if not isinstance(saved, dict) or set(saved)-keys:raise ValueError('invalid_saved_config')
        except (ValueError, OSError) as exc:
            self._config_error = str(exc)
        self.executor.cfg['live_enabled'] = self.live
        self.executor.cfg['operator_session_enabled'] = self.live

    @property
    def live(self): return self.cfg.get('mode', 'live') == 'live'

    def get_tool(self):
        fields = {'usage_guide': {'type': 'string', 'title': '使用说明（固定，无需配置）',
                    'enum': ['无需配置'], 'default': '无需配置', 'scope': 'instance'}}
        return {'name': 'teleop_control', 'type': 'actuator', 'multiInstance': False,
            'description': 'G1双臂遥操：先安装遥操设备 Driver，在画布添加 teleop_device 并连接本卡；在设备卡齿轮页获取 App 下载和配对入口。启动项目后松开双握把建立初始基准，再按住双握把跟随。松握保持，再握不重标定；在监控面板查看执行状态。停止项目停止遥操；收臂使用 arm release。',
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
        for name, default, upper in [('position_scale', 1., 1.)]:
            value = candidate.get(name, default)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= upper:
                raise ValueError('invalid_'+name)

    def configure(self, args):
        keys = self.get_tool()['configSchema']['properties']
        candidate = {**self.cfg, **{k:v for k,v in args.items() if k in keys}}
        self._validate_config(candidate)
        with self._lock:
            if self._operator or self._active_operation or self.motion.gate.session_id:
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
            self.mapping.scale = candidate.get('position_scale', 1.)
            self.executor.cfg['live_enabled'] = self.live
            self.executor.cfg['operator_session_enabled'] = self.live
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
            if self.binding and not same_topics and (self._operator or self._active_operation):
                raise ValueError('binding_requires_idle')
            if not same_topics or self._closed.is_set():
                self._latest = self._input_identity = self._operation_identity = None
                self._input_sequence = self._operation_sequence = -1
                self._receipts.clear(); self._operation_packets.clear()
                self.mapping.anchor = None
            self.binding = binding
            self.instance_id = args.get('instance_id') or 'teleop_control'
            self._closed.clear()
            self._state, self._reason = 'ready', 'waiting_input'
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
        # The single-device topic carries identity in JSON, not in its path.
        if value.get('kind') != 'input':
            raise ValueError('device_input_only')
        with self._lock:
            if self.binding['instance_id'] is None:
                source = canonical_instance(value.get('instance_id'))
                validate_input(value, instance_id=source, clock_id=self.clock_id, now_ns=self.clock())
                self.binding['instance_id'] = source
        is_stop = value.get('kind') == 'operation' and value.get('action') == 'stop'
        if self._closed.is_set() and not is_stop:return False
        if value.get('kind') == 'operation':
            try:return self._receive_operation(value)
            except ValueError as exc:
                # Admission failures of a valid request are terminal receipts;
                # do not make the user wait for a generic transport timeout.
                with self._lock:
                    if value.get('request_id') in self._receipts:raise
                    validate_operation(value, instance_id=self.binding['instance_id'],
                                       clock_id=self.clock_id, now_ns=self.clock())
                    key=value['request_id']
                    self._receipts[key]={'request_id': key, 'action': value['action'],
                        **{k:value[k] for k in ('device_id','connection_epoch','space_epoch')},
                        'status': 'failed', 'error': str(exc), 'result': {}}
                    self._operation_packets[key]=copy.deepcopy(value)
                    while len(self._receipts)>32:
                        removed,_=self._receipts.popitem(last=False);self._operation_packets.pop(removed,None)
                        self._stop_aliases.discard(removed)
                return False
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

    def _receive_operation(self, value):
        with self._lock:
            existing = self._receipts.get(value.get('request_id'))
            original = getattr(self, '_operation_packets', {}).get(value.get('request_id'))
            if existing is not None:
                if original != value: raise ValueError('operation_identity_conflict')
                return True
        op = validate_operation(value, instance_id=self.binding['instance_id'], clock_id=self.clock_id,
                                now_ns=self.clock())
        identity = self._identity(op)
        is_stop = op['action'] == 'stop'
        with self._lock:
            request = op['request_id']
            if request in self._receipts:
                if self._receipts[request]['action'] != op['action']: raise ValueError('operation_identity_conflict')
                return True
            old = self._operation_identity
            if not is_stop and old and identity[0] == old[0] and (identity[1] < old[1] or identity[2] < old[2]):
                raise ValueError('old_operation_generation')
            if not is_stop and self._input_identity and identity != self._input_identity:
                raise ValueError('operation_input_generation_mismatch')
            if not is_stop and identity == old and op['sequence'] <= self._operation_sequence:
                raise ValueError('old_operation_sequence')
            if op['action'] != 'stop' and (self._active_operation or self._operation_threads):
                raise ValueError('operation_busy')
            # Stop needs a valid bound request, not a live pose or grip. It may
            # arrive over WSS after RTC reconnect or shutdown; never lower the
            # ordinary operation watermark as a side effect of accepting it.
            if (not is_stop or old is None or
                    identity == old and op['sequence'] > self._operation_sequence or
                    identity[0] == old[0] and identity[1] >= old[1] and identity[2] >= old[2] and identity != old):
                self._operation_identity, self._operation_sequence = identity, op['sequence']
            self._receipts[request] = {'request_id': request, 'action': op['action'], 'status': 'accepted',
                                       **{k:op[k] for k in ('device_id','connection_epoch','space_epoch')},
                                       'error': None, 'result': {}}
            self._operation_packets[request] = copy.deepcopy(op)
            while len(self._receipts) > 32:
                removed, _ = self._receipts.popitem(last=False)
                self._operation_packets.pop(removed, None)
                self._stop_aliases.discard(removed)
            if op['action'] == 'stop' and self._stop_operation is not None:
                # Multiple callers can stop, but they do not allocate unbounded
                # workers or repeat release. Each gets the same final outcome.
                self._stop_aliases.add(request)
                return True
            self._active_operation = request
            self._generation += 1
            generation = self._generation
            if op['action'] in ('stop', 'finish'):
                self._operator = None  # Disarm before any blocking management/IK operation.
            if op['action'] == 'stop':self._stop_operation = request
            thread = threading.Thread(target=self._operation, args=(op, generation), daemon=True,
                                      name='g1-teleop-'+op['action'])
            self._operation_threads.add(thread)
            thread.start()
        return True

    def _check_generation(self, generation):
        if generation != self._generation or self._closed.is_set(): raise ValueError('operation_cancelled')

    def _operation(self, op, generation):
        try:
            if op['action'] in ('begin', 'calibrate'): result = self._begin(generation, op['action'], self._identity(op))
            elif op['action'] == 'finish': result = self._finish(generation)
            else: result = self._stop_motion()
            status, error = 'completed', None
        except Exception as exc:
            status, error, result = 'failed', str(exc), {}
        with self._lock:
            receipt = self._receipts.get(op['request_id'])
            if receipt: receipt.update(status=status, error=error, result=result)
            if self._stop_operation == op['request_id']:
                for alias in self._stop_aliases:
                    receipt = self._receipts.get(alias)
                    if receipt:receipt.update(status=status, error=error, result=copy.deepcopy(result))
                self._stop_operation = None
                self._stop_aliases.clear()
            if self._active_operation == op['request_id']: self._active_operation = None
            if error and generation == self._generation:
                self._state, self._reason = 'fault', error
            self._operation_threads.discard(threading.current_thread())

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
            if self._closed.is_set() or self._active_operation or not self.binding: return
            generation = self._generation
            initialize = not self._operator or self._needs_calibration
        if initialize:
            if self.clock() < self._auto_retry_after_ns: return
            try:
                frame = self._latest_fresh()
                if any(frame[s]['grip'] >= .5 for s in ('left', 'right')):
                    with self._lock: self._state, self._reason = 'ready', 'release_grips_before_enable'
                    return
                self._begin(generation, 'calibrate' if self._needs_calibration else 'begin')
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
            # the stop receipt can fence us while a channel is still opening.
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
        with self.motion._lock:
            self.motion._preview = None
            self.motion._preview_state = 'idle'
        # Independent of the model-management lock. This also fences a claim
        # still opening its SDK channel and a cold begin not yet prepared.
        self.executor.cancel_preparation()
        state = self.executor.status()
        if not state['ownership_held'] and not self.executor.opening:
            result = {'state': 'idle', 'authority_released': True, 'return_required': False}
        else:
            result = self.executor.dispatch('stop', {})
            if result.get('error'): raise ValueError(result.get('code', result['error']))
            deadline = time.monotonic()+4.
            while self.executor.status()['ownership_held'] and time.monotonic() < deadline:
                time.sleep(.02)
            state = self.executor.status()
            if state['ownership_held']: raise ValueError(state['reason'] or 'stop_unconfirmed')
            result = {'state': 'idle', 'authority_released': True, 'return_required': False}
        with self._lock:
            self._state, self._reason = 'idle', None
            self._paused = False
        return result

    def _finish(self, generation):
        with self._lock:
            self._check_generation(generation)
            if not self.live:
                result = self.motion.dispatch('finish', self._credentials())
            else:
                # G1's existing arm.release/action 99 is the return path.
                # Admission is short; the SDK handback and RPC run in arm.
                result = self.executor.dispatch('finish', self._credentials())
        deadline = time.monotonic()+11.
        while result.get('state') not in ('idle', 'completed', 'failed', 'unknown', 'error') and time.monotonic() < deadline:
            self._check_generation(generation)
            time.sleep(.02)
            result = self.executor.dispatch('finish_status', {'operation_id': result['operation_id']})
        if not result.get('return_completed') or not result.get('authority_released'):
            raise ValueError(result.get('error') or result.get('reason') or 'return_unconfirmed')
        with self._lock:
            self._check_generation(generation)
            self._state, self._reason = 'idle', None
        return result

    def stop(self):
        with self._lock:
            self._generation += 1
            self._operator = None
            self._closed.set(); self._wake.set()
        result = self._stop_motion()
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
                'instance_id': self.binding['instance_id'] if self.binding else '',
                'control_instance_id': self.instance_id, 'server_epoch': self.server_epoch,
                'sequence': self._feedback_sequence, 'emitted_monotonic_ns': self.clock(),
                'clock_id': self.clock_id, 'source_sequence': self._input_sequence,
                'connection_epoch': self._input_identity[1] if self._input_identity else 0,
                'space_epoch': self._input_identity[2] if self._input_identity else 0,
                'operator_session_id': self._operator, 'mapping_epoch': self.mapping.epoch,
                'state': self._state, 'reason': self._reason, 'capabilities': list(self.CAPABILITIES),
                'input_status': input_status, 'execution': execution, 'receipts': copy.deepcopy(list(self._receipts.values()))}

    def info(self):
        with self._lock:
            return {'state': self._state, 'reason': self._reason, 'mode': self.cfg.get('mode', 'live'),
                'config': copy.deepcopy(self.cfg),
                'effective_config': {k:self.cfg.get(k,v.get('default')) for k,v in self.get_tool()['configSchema']['properties'].items()},
                'config_error': self._config_error, 'binding': copy.deepcopy(self.binding),
                'input_status': self.input_status(),
                'mapping_epoch': self.mapping.epoch, 'operator_session_id': self._operator,
                'capabilities': list(self.CAPABILITIES), 'feedback': self.feedback() if self.binding else None,
                **{k:self.get_tool()[k] for k in ('topic_in','topic_out')}}
