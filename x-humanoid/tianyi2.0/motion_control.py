"""Tianyi end-effector controller in the Driver, sharing one hardware MotionGate.

The numerical worker never publishes vendor messages. Its authenticated joint
envelope returns through the local DDS arm input; only TeleopExecutor's watchdog
can emit. Preview credentials can never authorize that input.
"""
import copy
import math
import secrets
import threading
import time
from types import SimpleNamespace

from motion_stream import PROTOCOL, sign, vector
from tianyi_motion.protocol import SCHEMA, envelope, validate, validate_descriptor


class MotionControl:
    def __init__(self, cfg, executor, *, solver_factory=None):
        self.cfg, self.executor, self.gate = cfg, executor, executor.gate
        self.ns = executor.ns
        self.topic = f'/{self.ns}/motion/control'
        self.arm_topic = f'/{self.ns}/motion/arm/command'
        self.solver_factory = solver_factory
        self.solver = None
        self._lock = threading.RLock()
        self._management_lock = threading.RLock()
        self._wake, self._closed = threading.Event(), threading.Event()
        self._worker = None
        self._pending = None
        self._generation = 0
        self._lifecycle_generation = 0
        self._preview = None
        self._preview_state = 'idle'
        self._live_session = None
        self._used_live = False
        self._last_seq = self._joint_seq = self._last_epoch = -1
        self._state_seq = 0
        self._snapshot = None
        self._visualization = {'schema': 'motus.tianyi-visualization.v1', 'available': False,
                               'reason': 'calibration_missing'}
        self._decision = {}
        self._finish = {'state': 'idle', 'return_completed': False, 'authority_released': False}
        self._finish_thread = None
        self._finish_auth = None
        self._finish_cancel = threading.Event()
        self._last_joint = None

    @property
    def versions(self):
        if self.solver is None:
            raise ValueError('calibration_missing')
        return {'model_version': self.solver.profile['urdf_sha256'],
                'calibration_version': self.solver.profile_sha256,
                'frame': self.solver.profile['torso_frame']}

    def get_tool(self):
        actions = ['info', 'config', 'start', 'stop', 'calibrate', 'prepare_preview', 'claim',
                   'resume', 'pause', 'recoverable_hold', 'release', 'prepare_operator_session',
                   'end_operator_session', 'finish', 'finish_status']
        target = {'protocol_version': 2, 'robot_profile': 'tianyi2', 'namespace': self.ns,
                  'command_topic': self.topic+'/command',
                  'feedback_topic': self.executor.topic+'/feedback'}
        return {'name': 'motion_control', 'type': 'actuator', 'multiInstance': False,
            'description': '天轶双末端运控：IK、碰撞、预览和收臂在 Driver 内执行；不接收 PICO 协议。',
            'x-teleop-target': target,
            'x-motion-control': {**target, 'execution_tool': 'arm', 'execution_command_topic': self.arm_topic},
            'configSchema': {'type': 'object', 'additionalProperties': False, 'properties': {
                'calibration_path': {'type': 'string', 'scope': 'shared', 'x-sensitive': True,
                    'description': 'Driver 已挂载的天轶标定 JSON 路径，包含 URDF/TCP/碰撞边界；仅空闲时导入。'},
                'joint_velocity_rad_s': {'type': 'number', 'scope': 'shared', 'default': 1.0,
                    'exclusiveMinimum': 0, 'maximum': 1.5,
                    'description': '双臂运行速度 rad/s，同时受标定 URDF 速度上限约束。'}}},
            'inputSchema': {'type': 'object', 'properties': {
                'action': {'type': 'string', 'enum': actions},
                'session_id': {'type': 'string'}, 'secret': {'type': 'string', 'format': 'password'},
                'request_id': {'type': 'string'}, 'request_valid_until_ns': {'type': 'integer'},
                'operation_id': {'type': 'string'}, 'input_topic': {'type': 'string'},
                'instance_id': {'type': 'string'}, 'control_interface': {'type': 'object'},
                'calibration_path': {'type': 'string', 'x-sensitive': True},
                'joint_velocity_rad_s': {'type': 'number'},
                'control_interfaces': {'type': 'object'}, 'execution_binding': {'type': 'object'}},
                'required': ['action'], 'additionalProperties': False,
                'x-resource': ['arm_l', 'arm_r'],
                'x-action-params': {a: {'params': ['session_id', 'secret', 'request_id',
                    'request_valid_until_ns'] if a in ('claim', 'resume', 'release') else
                    ['session_id', 'secret'] if a in ('pause', 'stop', 'recoverable_hold', 'finish') else
                    ['operation_id'] if a == 'finish_status' else
                    ['calibration_path', 'joint_velocity_rad_s'] if a == 'config' else
                    ['input_topic', 'instance_id', 'control_interface', 'control_interfaces', 'execution_binding'] if a == 'start' else []} for a in actions}},
            'topic_in': [{'port_id': 'targets', 'topic': self.topic+'/command', 'format': 'control/eef'}],
            'topic_out': [{'port_id': 'joints', 'topic': self.arm_topic, 'format': 'control/joint'},
                          {'port_id': 'feedback', 'topic': self.executor.topic+'/feedback', 'format': 'data/json'}]}

    def arm_metadata(self):
        return {'x-control-target': {'protocol_version': 2, 'robot_profile': 'tianyi2',
            'namespace': self.ns, 'command_topic': self.arm_topic,
            'feedback_topic': self.executor.topic+'/feedback', 'resources': ['arm_l', 'arm_r']},
            'topic_in': [{'port_id': 'targets', 'topic': self.arm_topic, 'format': 'control/joint'}]}

    def control_interface(self, mode='eef_pose'):
        if mode not in ('eef_pose', 'joint_position'):
            raise ValueError('invalid_control_interface')
        profile = self.solver.profile if self.solver else (self.executor.profile or {})
        versions = self.versions if self.solver else {'model_version': profile.get('urdf_sha256'),
            'calibration_version': self.executor.profile_sha256, 'frame': profile.get('torso_frame')}
        result = {'control_interface': SCHEMA, 'schema': SCHEMA, 'protocol_version': 2,
            'mode': mode, 'dof': 14, **versions,
            'units': {'position': 'm', 'orientation': 'xyzw', 'time': 's'} if mode == 'eef_pose'
                     else {'angle': 'rad', 'time': 's'},
            'groups': [{'name': 'arm_'+suffix, 'offset': offset, 'count': 7,
                'mode': mode, 'unit': 'pose' if mode == 'eef_pose' else 'rad', 'resource': 'arm_'+suffix}
                for suffix, offset in (('l', 0), ('r', 7))],
            'rate': {'max_hz': 50, 'expected_hz': 50, 'watchdog_ms': 300 if mode == 'eef_pose' else 100}}
        if mode == 'eef_pose':
            result['effector_ids'] = ['left', 'right']
        else:
            result['joint_names'] = list(profile.get('arm_joint_names', []))
            result['limits'] = {'lower': [v[0] for v in self.gate.limits],
                                'upper': [v[1] for v in self.gate.limits],
                                'max_velocity': [self.gate.velocity]*14}
        return result

    def start(self):
        with self._lock:
            generation = self._lifecycle_generation
            if self._closed.is_set() and any(worker and worker.is_alive()
                    for worker in (self._worker, self._finish_thread)):
                raise RuntimeError('motion_control_thread_stop_unconfirmed')
        self.executor.start()
        with self._lock:
            if generation != self._lifecycle_generation:
                raise RuntimeError('motion_control_start_cancelled')
            if self._worker and self._worker.is_alive():
                return
            self._closed.clear()
            self._worker = threading.Thread(target=self._run, daemon=True, name='tianyi-motion-ik')
            self._worker.start()

    def stop(self):
        with self._lock:
            self._lifecycle_generation += 1
            self.cancel_pending()
            self._finish_cancel.set()
            self._closed.set()
            self._wake.set()
            workers = (self._worker, self._finish_thread)
        for worker in workers:
            if worker and worker is not threading.current_thread():
                worker.join(.5)
        if any(worker and worker.is_alive() for worker in workers):
            raise RuntimeError('motion_control_thread_stop_unconfirmed')
        with self._lock:
            if self._worker is workers[0]:
                self._worker = None
            if self._finish_thread is workers[1]:
                self._finish_thread = None

    def cancel_pending(self):
        with self._lock:
            self._generation += 1
            self._pending = None

    def _lease(self):
        if self._preview is not None:
            if self.gate.session_id:
                raise ValueError('preview_revoked_by_execution')
            return dict(self._preview)
        if self._live_session != self.gate.session_id or not self.gate.session_id:
            raise ValueError('invalid_lease')
        return dict(boot_id=self.gate.boot_id, session_id=self.gate.session_id, secret=self.gate.secret)

    @staticmethod
    def _authorized(args, lease):
        import hmac
        return bool(lease and args.get('session_id') == lease['session_id'] and
                    hmac.compare_digest(str(args.get('secret', '')), lease['secret']))

    def _fresh(self):
        state = self.gate.status()
        feedback = state['feedback']
        q = vector(feedback.get('q'), 14, 'q')
        vector(feedback.get('dq'), 14, 'dq')
        age = self.gate.clock()-feedback.get('arm_ns', 0)
        if not 0 <= age <= 100_000_000:
            raise ValueError('arm_feedback_stale')
        return state, q

    def _solver_for(self, path, cfg=None):
        factory = self.solver_factory
        if factory is None:
            from tianyi_motion.kinematics import TianyiIK
            factory = TianyiIK
        solver = factory(path)
        if solver.hands_enabled:
            raise ValueError('motion_control_requires_arms_only')
        cfg = self.cfg if cfg is None else cfg
        velocity = cfg.get('joint_velocity_rad_s', solver.velocity)
        if (type(velocity) not in (int, float) or not math.isfinite(velocity)
                or not 0 < velocity <= min(1.5, float(min(solver.model.velocityLimit)))):
            raise ValueError('joint_velocity_limit')
        solver.velocity = float(velocity)
        return solver

    def configure(self, args):
        from teleop_executor import load_profile
        candidate = dict(self.cfg)
        for key in ('calibration_path', 'joint_velocity_rad_s'):
            if key in args:candidate[key] = args[key]
        path = candidate.get('calibration_path') or self.executor.cfg.get('calibration_path')
        if not isinstance(path, str) or not path.strip():
            raise ValueError('calibration_missing')
        with self._management_lock, self.gate.legacy():
            if (self._preview or self.executor._operator_prepared or self.executor.legacy_busy()
                    or self._finish_thread and self._finish_thread.is_alive()):
                raise ValueError('configuration_requires_idle')
            # Validate the entire candidate before changing any live object.
            profile, limits, digest = load_profile(path)
            solver = self._solver_for(path, candidate)
            if solver.profile_sha256 != digest:
                raise ValueError('calibration_changed_during_load')
            with self._lock:
                self.cancel_pending()
                self.cfg.update(candidate)
                self.executor.cfg['calibration_path'] = path
                self.executor.profile, self.executor.profile_sha256 = profile, digest
                self.executor.profile_error = None
                self.executor._fixed_baseline = None
                self.gate.limits, self.gate.velocity = limits, solver.velocity
                self.gate.hands_enabled = False
                self.solver = None  # A new actual-state calibration is still required.
                self._snapshot = None
                self._visualization = {'schema': 'motus.tianyi-visualization.v1',
                                       'available': False, 'reason': 'calibration_required'}
            return {'state': 'configured', 'calibrated': False, 'config': self.config_snapshot(),
                    'control_interface': self.control_interface()}

    def config_snapshot(self):
        return {'calibration_path': self.cfg.get('calibration_path') or self.executor.cfg.get('calibration_path', ''),
                'joint_velocity_rad_s': self.cfg.get('joint_velocity_rad_s',
                    (self.executor.profile or {}).get('joint_velocity_rad_s', 1.0))}

    def calibrate(self):
        with self._management_lock:
            if self.gate.session_id or self._finish_thread and self._finish_thread.is_alive():
                raise ValueError('calibration_requires_idle')
            path = self.cfg.get('calibration_path') or self.executor.cfg.get('calibration_path')
            if not path:
                raise ValueError('calibration_missing')
            solver = self._solver_for(path)
            if solver.profile_sha256 != self.executor.profile_sha256:
                raise ValueError('executor_calibration_mismatch')
            _, q = self._fresh()
            solver.self_test(q)
            with self._lock:
                self.cancel_pending()
                self._preview = None
                self.solver = solver
                # Constructor validates this against the actual URDF limits.
                self.gate.velocity = solver.velocity
            self._refresh_snapshot()
            return {'calibrated': True, **self.versions, 'effector_ids': ['left', 'right'],
                    'eef_snapshot': copy.deepcopy(self._snapshot)}

    def _refresh_snapshot(self):
        with self._lock:
            generation = self._generation
            solver = self.solver
            versions = self.versions if solver is not None else None
        if solver is None:
            return
        from scipy.spatial.transform import Rotation
        from tianyi_motion.tianyi_visualization import snapshot
        state = self.gate.status()
        feedback = state['feedback']
        if 0 <= self.gate.clock()-feedback.get('arm_ns', 0) <= 100_000_000:
            q = vector(feedback.get('q'), 14, 'q')
            poses = [t[:3, 3].tolist()+Rotation.from_matrix(t[:3, :3]).as_quat().tolist()
                     for t in solver.palms(q)]
            with self._lock:
                if generation != self._generation or solver is not self.solver:
                    return
                if not self._snapshot or self._snapshot['monotonic_ns'] != feedback['arm_ns']:
                    self._state_seq += 1
                self._snapshot = {'poses': poses, **versions, 'state_seq': self._state_seq,
                                  'monotonic_ns': feedback['arm_ns']}
        adapter = SimpleNamespace(solver=solver, link=SimpleNamespace(feedback=lambda: state),
            output={'state': self._decision.get('state', state['state']), 'code': self._decision.get('reason')},
            hardware_output=bool(self._live_session and self.gate.session_id))
        value = snapshot(adapter)
        with self._lock:
            if generation == self._generation:
                self._visualization = value

    def feedback_fields(self):
        with self._lock:
            if self._preview and self.gate.session_id:
                # A legacy v1 claimant shares the same gate. Never hide its
                # real ownership behind a stale preview overlay.
                self.cancel_pending()
                self._preview = None
                self._decision = {'state': 'hold', 'reason': 'preview_revoked_by_execution'}
            data = {'control_interface': self.control_interface(),
                'control_interfaces': {'joints': self.control_interface('joint_position')},
                'eef_snapshot': copy.deepcopy(self._snapshot), 'visualization': copy.deepcopy(self._visualization),
                'control_decision': copy.deepcopy(self._decision), 'finish': copy.deepcopy(self._finish),
                'preview': bool(self._preview)}
            if self._preview:
                data.update(session_id=self._preview['session_id'], state=self._preview_state,
                    ownership_held=False, output_active=False, hold_confirmed=self._preview_state == 'hold',
                    stop_confirmed=self._preview_state == 'hold', sequence=self._last_seq, applied_sequence=-1)
            return data

    def info(self):
        tool = self.get_tool()
        return {**self.executor.info(), **self.feedback_fields(), 'config': self.config_snapshot(),
                **{k: tool[k] for k in ('x-teleop-target', 'x-motion-control', 'topic_in', 'topic_out')}}

    def dispatch(self, action, args):
        try:
            with self._management_lock:
                return self._dispatch(action, args)
        except (ValueError, RuntimeError, OSError) as exc:
            return {'state': 'error', 'code': str(exc), 'error': str(exc)}

    def _dispatch(self, action, args):
        if action == 'info':
            return self.info()
        if action == 'config':
            return self.configure(args)
        if action == 'start':
            if args.get('input_topic') not in (None, self.topic+'/command'):
                raise ValueError('motion_control_input_topic_mismatch')
            try:
                interfaces = args.get('control_interfaces', {})
                if not isinstance(interfaces, dict):
                    raise ValueError('invalid_control_descriptor')
                for source, name in ((args, 'control_interface'), (interfaces, 'joints')):
                    if name in source:
                        validate_descriptor(source[name], self.control_interface('joint_position'))
            except ValueError:
                raise ValueError('motion_control_execution_binding_mismatch') from None
            binding = args.get('execution_binding')
            if binding is not None:
                expected = self.arm_metadata()['x-control-target']
                if (not isinstance(binding, dict) or binding.get('tool') != 'arm'
                        or any(binding.get(k) != v for k, v in expected.items())):
                    raise ValueError('motion_control_execution_binding_mismatch')
            self.start()
            return {**self.info(), 'execution_state': self.gate.state, 'state': 'ready'}
        if action == 'calibrate':
            self.start()
            return self.calibrate()
        if action == 'finish_status':
            with self._lock:
                if args.get('operation_id') not in (None, self._finish.get('operation_id')):
                    raise ValueError('unknown_finish_operation')
                return copy.deepcopy(self._finish)
        if action == 'finish':
            return self._start_finish(args)
        if action == 'prepare_preview':
            self.start()
            if self.gate.session_id:
                raise ValueError('preview_requires_idle')
            if self.solver is None:
                self.calibrate()
            self._fresh()
            with self._lock:
                self.cancel_pending()
                self._preview = dict(boot_id=self.gate.boot_id, session_id=secrets.token_hex(16), secret=secrets.token_hex(32))
                self._preview_state = 'ready'
                self._last_seq = self._last_epoch = -1
                return {**self._preview, 'preview': True, 'state': 'ready', **self.versions}
        if self._preview and action in ('pause', 'recoverable_hold', 'resume', 'release', 'stop', 'end_operator_session'):
            framework_stop = action in ('stop', 'end_operator_session') and not args.get('session_id') and not args.get('secret')
            if not framework_stop and not self._authorized(args, self._preview):
                raise ValueError('invalid_lease')
            self.cancel_pending()
            if action == 'resume':
                return self._dispatch('prepare_preview', {})
            if action in ('release', 'stop', 'end_operator_session'):
                self._preview = None
                self._preview_state = 'idle'
                if action == 'stop':
                    self.stop()
                    if self.gate.session_id:
                        # A concurrent legacy claimant still owns the shared gate.
                        return self.info()
                return {'state': 'idle', 'stop_confirmed': True, 'ownership_held': False, 'preview': True}
            self._preview_state = 'hold'
            return {**self.info(), 'hold_confirmed': True}
        if action in ('claim', 'resume', 'prepare_operator_session'):
            self.start()
            if self.solver is None:
                self.calibrate()
            if self._finish_thread and self._finish_thread.is_alive():
                raise ValueError('return_in_progress')
        if (action in ('stop', 'release', 'pause') and self._finish_auth
                and self._authorized(args, self._finish_auth)
                and self.gate.session_id == self._live_session
                and self._finish_thread and self._finish_thread.is_alive()):
            # The return worker may rotate its internal lease. The original
            # accepted finish credentials can only cancel that same operation.
            self._finish_cancel.set()
            args = self._lease()
        previous_session = self._live_session
        result = self.executor.dispatch(action, args)
        if not result.get('error') and action in ('claim', 'resume', 'prepare_operator_session',
                'pause', 'release', 'stop', 'recoverable_hold', 'end_operator_session'):
            if action not in ('claim', 'resume') or result.get('session_id') != previous_session:
                self.cancel_pending()
            self._preview = None
            if action in ('stop', 'release'):
                self._finish_cancel.set()
            if action == 'stop':
                self.stop()
                # Stopping the IK card must not stop the shared feedback/watchdog.
                # Its real hold/fault remains visible until ownership is released.
                result = self.info()
        if not result.get('error') and action in ('claim', 'resume'):
            with self._lock:
                changed = self._live_session != result['session_id']
                self._live_session = result['session_id']
                self._used_live = True
                if changed:
                    self._last_seq = self._joint_seq = self._last_epoch = -1
                    self._finish_auth = None
            return {**result, **self.versions, 'preview': False}
        return result

    def receive_eef(self, packet):
        with self._lock:
            if self._closed.is_set():
                raise ValueError('motion_control_stopped')
            lease = self._lease()
            body = validate(packet, lease, mode='eef_pose', now=self.gate.clock(),
                previous_seq=self._last_seq, previous_epoch=self._last_epoch, **self.versions)
            if self._finish_thread and self._finish_thread.is_alive():
                raise ValueError('return_in_progress')
            if self._preview and self._preview_state == 'hold':
                raise ValueError('preview_paused')
            if body['mapping_epoch'] > self._last_epoch:
                self.cancel_pending()
                # A clutch establishes a different target frame. Retire the
                # old epoch's display and numerical seed as well as its queue.
                self.solver.visualization_sample = None
                self.solver.last_valid_visualization = None
                self._visualization = {**self._visualization, 'ik': [], 'held_ik': [], 'targets': []}
            self._last_seq, self._last_epoch = body['seq'], body['mapping_epoch']
            self._pending = (body, lease, self._generation, bool(self._preview))
            self._wake.set()
            return True

    def receive_joint(self, packet):
        with self._lock:
            if self._closed.is_set():
                raise ValueError('motion_control_stopped')
            if self._preview:
                raise ValueError('preview_cannot_execute')
            lease = self._lease()
            body = validate(packet, lease, mode='joint_position', now=self.gate.clock(),
                previous_seq=self.gate.seq, previous_epoch=self._last_epoch, **self.versions)
            # Keep the preexisting hardware gate and its independent watchdog.
            # Floor, never extend, the original absolute deadline at v1 boundary.
            ttl = (body['valid_until_ns']-body['generated_ns'])//1_000_000
            old = dict(protocol=PROTOCOL, boot_id=body['boot_id'], session_id=body['session_id'],
                seq=body['seq'], generated_ns=body['generated_ns'], valid_for_ms=ttl,
                q=body['values'], hands=[0., 0.])
            accepted = self.gate.accept({**old, 'mac': sign(old, lease['secret'])})
            if accepted:
                self._last_joint = body
            return accepted

    def rejected(self, route, packet, reason):
        with self._lock:
            self._decision = {'state': 'rejected', 'route': route, 'reason': reason,
                              'monotonic_ns': self.gate.clock()}
            if isinstance(packet, dict):
                self._decision.update({k: packet[k] for k in ('seq', 'source_seq', 'mapping_epoch')
                    if type(packet.get(k)) is int and 0 <= packet[k] < 2**53})

    def _publish_joint(self, body, lease, q):
        now = self.gate.clock()
        if now >= body['valid_until_ns']:
            raise ValueError('command_expired')
        self._joint_seq = max(self._joint_seq+1, self.gate.seq+1, body['seq'])
        # A joint target is produced now, but cannot outlive its source input.
        command = envelope(lease, seq=self._joint_seq, source_seq=body['source_seq'],
            mapping_epoch=body['mapping_epoch'], generated_ns=now,
            valid_until_ns=min(body['valid_until_ns'], now+100_000_000),
            mode='joint_position', values=q, **self.versions)
        self.executor.publish_joint_command(command)
        return command

    def process_latest(self):
        with self._lock:
            work, self._pending = self._pending, None
        if work is None:
            return False
        body, lease, generation, preview = work
        try:
            from tianyi_motion.kinematics import transform
            state, q = self._fresh()
            if not preview and state['state'] == 'hold' and (
                    not state.get('continuation_ready') or
                    body['generated_ns'] < (state.get('hold_confirmed_ns') or 2**63)):
                with self._lock:
                    self._decision = {'state': 'hold', 'reason': 'waiting_fresh_input_after_hold',
                                      'source_seq': body['source_seq'], 'seq': body['seq']}
                return False
            values = body['values']
            targets = [transform({'position': values[i:i+3], 'orientation': values[i+3:i+7]})
                       for i in (0, 7)]
            result = self.solver.solve(targets, q, state.get('commanded_q'),
                deadline_monotonic=body['valid_until_ns']/1e9)
            with self._lock:
                if (generation != self._generation or body['mapping_epoch'] != self._last_epoch
                        or lease != self._lease()):
                    self.solver.visualization_sample = None
                    self.solver.last_valid_visualization = None
                    return False
                if self.gate.clock() >= body['valid_until_ns']:
                    raise ValueError('command_expired')
                if preview:
                    self._preview_state = 'active'
                else:
                    self._publish_joint(body, lease, result)
                self._decision = {'state': 'preview' if preview else 'target_published', 'reason': None,
                    'source_seq': body['source_seq'], 'seq': body['seq'], 'mapping_epoch': body['mapping_epoch'],
                    'source_generated_ns': body['generated_ns'], 'source_valid_until_ns': body['valid_until_ns'],
                    'monotonic_ns': self.gate.clock(), 'ik_ms': self.solver.last_ms}
            return True
        except (ValueError, RuntimeError) as exc:
            with self._lock:
                if generation == self._generation:
                    self._decision = {'state': 'hold', 'reason': str(exc), 'seq': body['seq'],
                        'source_seq': body['source_seq'], 'monotonic_ns': self.gate.clock()}
                    if not preview and self.gate.session_id == lease['session_id']:
                        # Failed results are never replayed. A later valid result
                        # can continue the same session after physical hold.
                        try:self.gate.hold('ik_recoverable', recoverable=True)
                        except ValueError:pass
            return False
        finally:
            if generation == self._generation:
                self._refresh_snapshot()

    def _run(self):
        while not self._closed.is_set():
            self._wake.wait(.02)
            self._wake.clear()
            try:
                if not self.process_latest():
                    self._refresh_snapshot()
            except Exception as exc:
                with self._lock:
                    self._decision = {'state': 'error', 'reason': str(exc)[:160] or type(exc).__name__,
                                      'error_type': type(exc).__name__}

    def _start_finish(self, args):
        with self._lock:
            if self._finish_thread and self._finish_thread.is_alive():
                if not self._authorized(args, self._finish_auth or self._lease()):
                    raise ValueError('invalid_lease')
                return copy.deepcopy(self._finish)
            if self._preview:
                if not self._authorized(args, self._preview):
                    raise ValueError('invalid_lease')
                self.cancel_pending()
                self._preview = None
                self._finish = {'state': 'idle', 'return_completed': True, 'authority_released': True,
                                'preview': True, 'operation_id': secrets.token_hex(16)}
                return dict(self._finish)
            retry_authorized = (self._finish.get('state') == 'error' and self._finish_auth
                and self.gate.session_id == self._live_session and self._authorized(args, self._finish_auth))
            if self.gate.session_id and not (retry_authorized or self._authorized(args, self._lease())):
                raise ValueError('invalid_lease')
            if not self._used_live:
                self._finish = {'state': 'idle', 'return_completed': True, 'authority_released': True,
                    'motion_requested': False, 'operation_id': secrets.token_hex(16)}
                return dict(self._finish)
            self.cancel_pending()
            self._finish_cancel.clear()
            self._finish_auth = {'session_id': args.get('session_id'), 'secret': str(args.get('secret', ''))}
            self._finish = {'state': 'returning', 'return_completed': False, 'authority_released': False,
                            'operation_id': secrets.token_hex(16)}
            self._finish_thread = threading.Thread(target=self._return_arms, daemon=True, name='tianyi-return-arms')
            self._finish_thread.start()
            return dict(self._finish)

    def _return_arms(self):
        import numpy as np
        deadline = time.monotonic()+45.
        settled = None
        try:
            solver = self.solver
            if solver is None or solver.hands_enabled:
                raise ValueError('return_requires_calibrated_arms_only')
            lower = solver.model.lowerPositionLimit[solver.indices]
            upper = solver.model.upperPositionLimit[solver.indices]
            if np.any(lower > 0) or np.any(upper < 0):
                raise ValueError('neutral_outside_limits')
            if not self.gate.session_id:
                prepared = self.executor.dispatch('prepare_operator_session', {})
                if prepared.get('error'):raise ValueError(prepared['code'])
                result = self.executor.dispatch('claim', {})
                if result.get('error'):raise ValueError(result['code'])
                self._live_session = result['session_id']
            elif self.gate.state in ('hold', 'fault'):
                result = self.executor.dispatch('resume', self._lease())
                if result.get('error'):
                    # An explicit retry may reconcile a previously latched
                    # fault, but only after fresh physical stop and release.
                    if not self.gate.status()['stop_confirmed']:
                        raise ValueError(result['code'])
                    release = self.executor.dispatch('release', self._lease())
                    if release.get('error'):raise ValueError(release['code'])
                    stop_deadline = min(deadline, time.monotonic()+2.5)
                    while self.gate.session_id and time.monotonic() < stop_deadline:
                        if self._finish_cancel.is_set():raise ValueError('return_cancelled')
                        time.sleep(.02)
                    if self.gate.session_id or not self.gate.status()['stop_confirmed']:
                        raise ValueError('stop_unconfirmed')
                    prepared = self.executor.dispatch('prepare_operator_session', {})
                    if prepared.get('error'):raise ValueError(prepared['code'])
                    result = self.executor.dispatch('claim', {})
                    if result.get('error'):raise ValueError(result['code'])
                self._live_session = result['session_id']
            self._last_epoch = max(0, self._last_epoch)
            while time.monotonic() < deadline:
                if self._finish_cancel.is_set():raise ValueError('return_cancelled')
                state, q = self._fresh()
                q = np.asarray(q)
                dq = np.asarray(state['feedback']['dq'])
                if state['state'] == 'fault':raise ValueError('return_driver_fault')
                if np.max(np.abs(q)) <= .02 and np.max(np.abs(dq)) <= .02:
                    settled = time.monotonic() if settled is None else settled
                    if time.monotonic()-settled >= .1:
                        result = self.executor.dispatch('release', self._lease())
                        if result.get('error'):raise ValueError(result['code'])
                        stop_deadline = min(deadline, time.monotonic()+2.5)
                        while time.monotonic() < stop_deadline:
                            receipt = self.gate.status()
                            if receipt['stop_confirmed'] and not receipt['ownership_held']:
                                with self._lock:
                                    self._finish.update(state='idle', return_completed=True,
                                        authority_released=True, max_error_rad=float(np.max(np.abs(q))))
                                    self._used_live = False
                                return
                            time.sleep(.02)
                        raise ValueError('stop_unconfirmed')
                else:
                    settled = None
                if state['state'] == 'hold':
                    if not state['hold_confirmed'] or not state.get('continuation_allowed'):
                        raise ValueError('return_driver_hold:'+str(state.get('reason')))
                previous = np.asarray(state.get('commanded_q') or q.tolist())
                step = previous+np.clip(-previous, -solver.velocity*.02, solver.velocity*.02)
                step = np.clip(step, q-solver.velocity*.2, q+solver.velocity*.2)
                step = np.clip(step, lower, upper)
                cycle = min(deadline, time.monotonic()+.1)
                def budget():
                    if self._finish_cancel.is_set():raise ValueError('return_cancelled')
                    if time.monotonic() >= cycle:raise ValueError('return_geometry_timeout')
                with solver.lock:
                    solver._safe_transition(q, step, budget)
                    solver._safe_transition(previous, step, budget)
                with self._lock:
                    budget()
                    now = self.gate.clock()
                    body = dict(seq=self._joint_seq+1, source_seq=self._joint_seq+1,
                        mapping_epoch=self._last_epoch, generated_ns=now, valid_until_ns=now+100_000_000)
                    self._publish_joint(body, self._lease(), step.tolist())
                time.sleep(.02)
            raise ValueError('return_timeout')
        except Exception as exc:
            if self.gate.session_id == self._live_session:
                try:self.gate.hold('operator_pause')
                except ValueError:pass
            with self._lock:
                self._finish.update(state='error', code=str(exc), error=str(exc),
                    return_completed=False, authority_released=not bool(self.gate.session_id))
