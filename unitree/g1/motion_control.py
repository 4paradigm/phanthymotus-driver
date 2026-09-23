"""G1_23 end-effector card; numerical work is isolated from arm execution.

The executor adapter owns hardware, authority and continuous interpolation. This
card owns one bounded latest input slot, calibration and the numerical child.
"""
import copy
from pathlib import Path
import secrets
import threading
import time

from common.motion.envelope import IDENTITY, MotionEnvelope
from common.motion.protocol import SCHEMA, envelope, validate, validate_descriptor, vector
from g1_motion.worker import NumericalWorker

JOINT_NAMES = tuple(f'{side}_{joint}_joint' for side in ('left', 'right') for joint in
                    ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll'))


class MotionControl:
    def __init__(self, cfg, executor, *, solver_factory=None):
        self.cfg, self.executor, self.gate = cfg, executor, executor.gate
        self.ns = executor.ns
        self.topic = f'/{self.ns}/motion/control'
        self.arm_topic = f'/{self.ns}/motion/arm/command'
        self.feedback_topic = f'/{self.ns}/motion/teleop/feedback'
        self.solver_factory = solver_factory
        self.solver = None
        self._lock = threading.RLock()
        self._management_lock = threading.RLock()
        self._closed, self._wake = threading.Event(), threading.Event()
        self._worker = None
        self._pending = None
        self._generation = self._last_seq = self._last_epoch = self._joint_seq = 0
        self._last_seq = self._last_epoch = self._joint_seq = -1
        self._preview = None
        self._preview_state = 'idle'
        self._live_session = None
        self._snapshot = None
        self._visualization = {'available': False, 'reason': 'calibration_missing'}
        self._decision = {}
        self._state_seq = 0
        self._finish = {'state':'idle','return_completed':False,'authority_released':False}
        clock_file = Path('/proc/sys/kernel/random/boot_id')
        self.clock_id = clock_file.read_text().strip() if clock_file.exists() else None

    @property
    def versions(self):
        if self.solver is None: raise ValueError('calibration_missing')
        return {'model_version': self.solver.profile['urdf_sha256'],
                'calibration_version': self.solver.profile_sha256,
                'frame': self.solver.profile['torso_frame']}

    def control_interface(self, mode='eef_pose'):
        if mode not in ('eef_pose', 'joint_position'): raise ValueError('invalid_control_interface')
        solver = self.solver
        profile = solver.profile if solver else {}
        versions = self.versions if solver else {'model_version': None, 'calibration_version': None, 'frame': None}
        dof, count = (14, 7) if mode == 'eef_pose' else (10, 5)
        value = {'control_interface': SCHEMA, 'schema': SCHEMA, 'protocol_version': 2,
            'mode': mode, 'dof': dof, **versions, 'force_torque': None,
            'units': {'position': 'm', 'orientation': 'xyzw', 'time': 's'} if mode == 'eef_pose'
                else {'angle': 'rad', 'time': 's'},
            'groups': [{'name': 'arm_'+s, 'resource': 'arm_'+s, 'offset': i*count, 'count': count,
                        'mode': mode, 'unit': 'pose' if mode == 'eef_pose' else 'rad'}
                       for i,s in enumerate(('l','r'))],
            'rate': {'max_hz': 50, 'expected_hz': 20, 'watchdog_ms': 300}}
        if mode == 'eef_pose': value['effector_ids'] = ['left', 'right']
        else:
            value['joint_names'] = list(profile.get('arm_joint_names', JOINT_NAMES))
            value['limits'] = {'lower': list(solver.lower) if solver else [],
                'upper': list(solver.upper) if solver else [],
                'max_velocity': [solver.velocity]*10 if solver else []}
        return value

    def get_tool(self):
        actions = ['info', 'config', 'start', 'stop', 'calibrate', 'prepare_preview',
            'prepare_operator_session', 'end_operator_session', 'claim', 'resume',
            'pause', 'recoverable_hold', 'release', 'finish', 'finish_status']
        target = {'protocol_version': 2, 'robot_profile': 'unitree_g1_23_dual_arm_relative_v1',
            'namespace': self.ns, 'command_topic': self.topic+'/command', 'feedback_topic': self.feedback_topic}
        return {'name': 'motion_control', 'type': 'actuator', 'multiInstance': False,
            'description': 'G1_23双末端运控：独立进程IK与碰撞，连续执行及停止交给arm。',
            'x-teleop-target': target,
            'x-motion-control': {**target, 'execution_tool': 'arm', 'execution_command_topic': self.arm_topic},
            'configSchema': {'type': 'object', 'additionalProperties': False, 'properties': {
                'calibration_path': {'type': 'string', 'scope': 'shared', 'x-sensitive': True},
                'joint_velocity_rad_s': {'type': 'number', 'scope': 'shared', 'default': 1., 'exclusiveMinimum': 0}}},
            'inputSchema': {'type': 'object', 'required': ['action'], 'properties': {
                'action': {'type': 'string', 'enum': actions}, 'instance_id': {'type': 'string'},
                'input_topic': {'type': 'string'}, 'control_interface': {'type': 'object'},
                'control_interfaces': {'type': 'object'}, 'execution_binding': {'type': 'object'},
                'session_id': {'type': 'string'}, 'secret': {'type': 'string', 'format': 'password'},
                'request_id': {'type': 'string'}, 'request_valid_until_ns': {'type': 'integer'},
                'operation_id': {'type': 'string'}, 'retry': {'type': 'boolean'}, 'calibration_path': {'type': 'string', 'x-sensitive': True},
                'joint_velocity_rad_s': {'type': 'number'}}, 'additionalProperties': False,
                'x-resource': ['arm_l','arm_r']},
            'topic_in': [{'port_id': 'targets', 'topic': self.topic+'/command', 'format': 'control/eef'}],
            'topic_out': [{'port_id': 'joints', 'topic': self.arm_topic, 'format': 'control/joint'},
                          {'port_id': 'feedback', 'topic': self.feedback_topic, 'format': 'data/json'}]}

    def arm_metadata(self):
        return {'x-control-target': {'protocol_version': 2,
            'robot_profile': 'unitree_g1_23_dual_arm_relative_v1', 'namespace': self.ns,
            'command_topic': self.arm_topic, 'feedback_topic': self.feedback_topic,
            'resources': ['arm_l','arm_r']},
            'topic_in': [{'port_id': 'targets', 'topic': self.arm_topic, 'format': 'control/joint'}]}

    def _fresh(self):
        state = self.gate.status()
        feedback = state['feedback']
        q = vector(feedback.get('q'), 10, 'q')
        vector(feedback.get('dq'), 10, 'dq')
        if not 0 <= self.gate.clock()-feedback.get('arm_ns', 0) <= 100_000_000:
            raise ValueError('arm_feedback_stale')
        return state, q

    def _lease(self):
        if self._preview:
            if self.gate.session_id: raise ValueError('preview_revoked_by_execution')
            return dict(self._preview)
        if not self.gate.session_id or self._live_session != self.gate.session_id:
            raise ValueError('invalid_lease')
        return {'boot_id': self.gate.boot_id, 'session_id': self.gate.session_id, 'secret': self.gate.secret}

    def cancel_pending(self):
        with self._lock:
            self._generation += 1
            self._pending = None

    def start(self):
        self.executor.start()
        with self._lock:
            if self._worker and self._worker.is_alive(): return
            self._closed.clear()
            self._worker = threading.Thread(target=self._run, name='g1-motion-coordinator', daemon=True)
            self._worker.start()

    def stop(self):
        with self._lock:
            self.cancel_pending()
            self._closed.set()
            self._wake.set()
            worker, solver = self._worker, self.solver
        if solver: solver.interrupt()
        if worker and worker is not threading.current_thread(): worker.join(.5)
        if worker and worker.is_alive(): raise RuntimeError('motion_control_thread_stop_unconfirmed')
        if solver: solver.close()
        with self._lock: self._worker = self.solver = None

    def calibrate(self, candidate=None):
        if self.gate.session_id or self._preview: raise ValueError('calibration_requires_idle')
        cfg = self.cfg if candidate is None else candidate
        path = cfg.get('calibration_path')
        if not isinstance(path, str) or not path: raise ValueError('calibration_missing')
        factory = self.solver_factory or NumericalWorker
        solver = factory(path, cfg.get('joint_velocity_rad_s', 1.))
        try:
            if solver.profile.get('arm_joint_names') != list(JOINT_NAMES): raise ValueError('g1_joint_order')
            _, q = self._fresh()
            solver.self_test(q)
            # Candidate numerical checks finish before changing the idle arm's
            # profile. Execution keeps its own independent model and Data.
            self.executor.configure_profile(path, velocity=solver.velocity,
                expected_sha256=solver.profile_sha256)
            if getattr(self.executor, 'profile_sha256', None) != solver.profile_sha256:
                raise ValueError('executor_calibration_mismatch')
        except BaseException:
            solver.close()
            raise
        with self._lock:
            self.cancel_pending()
            old, self.solver = self.solver, solver
            self.cfg.update(cfg)
        if old: old.close()
        self._refresh_snapshot()
        return {'calibrated': True, 'effector_ids': ['left', 'right'], **self.versions, 'eef_snapshot': copy.deepcopy(self._snapshot)}

    def config_snapshot(self):
        return {k: self.cfg[k] for k in ('calibration_path', 'joint_velocity_rad_s') if k in self.cfg}

    def configure(self, values):
        with self._management_lock:
            result = self.calibrate({**self.cfg, **values})
            self.executor.cfg.update(self.config_snapshot())
            return result

    def feedback_fields(self):
        with self._lock:
            if self._preview and self.gate.session_id:
                self.cancel_pending(); self._preview = None
                self._decision = {'state': 'hold', 'reason': 'preview_revoked_by_execution'}
            result = {'schema': 'motus.motion.feedback/1', 'clock_id': self.clock_id,
                'control_interface': self.control_interface(),
                'control_interfaces': {'joints': self.control_interface('joint_position')},
                'eef_snapshot': copy.deepcopy(self._snapshot),
                'visualization': copy.deepcopy(self._visualization),
                'control_decision': copy.deepcopy(self._decision), 'finish': copy.deepcopy(self._finish),
                'collision_checks_enabled': getattr(self.solver, 'collision_checks_enabled', None),
                'preview': bool(self._preview)}
            if self._preview:
                result.update(session_id=self._preview['session_id'], state=self._preview_state,
                    ownership_held=False, output_active=False, hold_confirmed=self._preview_state == 'hold',
                    stop_confirmed=self._preview_state == 'hold', sequence=self._last_seq, applied_sequence=-1)
            return result

    def info(self):
        tool = self.get_tool()
        return {**self.executor.info(), **self.feedback_fields(), 'config': dict(self.cfg),
            **{key: tool[key] for key in ('x-teleop-target','x-motion-control','topic_in','topic_out')}}

    def dispatch(self, action, args):
        try:
            with self._management_lock: return self._dispatch(action, args)
        except (ValueError, RuntimeError, OSError) as exc:
            return {'state': 'error', 'code': str(exc), 'error': str(exc)}

    def _dispatch(self, action, args):
        if action == 'info': return self.info()
        if action == 'finish_status' and self._finish.get('preview'):
            if args.get('operation_id') not in (None,self._finish.get('operation_id')): raise ValueError('unknown_finish_operation')
            return dict(self._finish)
        if action == 'config':
            candidate = dict(self.cfg)
            candidate.update({k: args[k] for k in ('calibration_path','joint_velocity_rad_s') if k in args})
            result = self.calibrate(candidate)
            return {**result, 'state': 'configured', 'config': dict(self.cfg)}
        if action == 'calibrate': return self.calibrate()
        if action == 'start':
            if args.get('input_topic') not in (None, self.topic+'/command'): raise ValueError('motion_control_input_topic_mismatch')
            interfaces = args.get('control_interfaces', {})
            if not isinstance(interfaces, dict): raise ValueError('invalid_control_descriptor')
            for source, key in ((args,'control_interface'),(interfaces,'joints')):
                if key in source: validate_descriptor(source[key], self.control_interface('joint_position'))
            binding = args.get('execution_binding')
            if binding is not None:
                if not isinstance(binding,dict) or binding.get('tool') != 'arm' or any(
                    binding.get(k) != v for k,v in self.arm_metadata()['x-control-target'].items()):
                    raise ValueError('motion_control_execution_binding_mismatch')
            self.start(); return {**self.info(), 'execution_state': self.gate.status()['state'], 'state': 'ready'}
        if action == 'prepare_preview':
            if self.gate.session_id: raise ValueError('preview_requires_idle')
            if self.solver is None: self.calibrate()
            self.start(); self._fresh(); self.cancel_pending()
            self._preview = {'boot_id': self.gate.boot_id, 'session_id': secrets.token_hex(16), 'secret': secrets.token_hex(32)}
            self._preview_state = 'ready'; self._last_seq = self._last_epoch = -1
            return {**self._preview, **self.versions, 'preview': True, 'state': 'ready'}
        if self._preview and action in ('pause','recoverable_hold','resume','release','stop','finish','end_operator_session'):
            import hmac
            framework_stop = action in ('stop','end_operator_session') and not args.get('session_id') and not args.get('secret')
            if not framework_stop and (args.get('session_id') != self._preview['session_id'] or
                    not hmac.compare_digest(str(args.get('secret','')),self._preview['secret'])):
                raise ValueError('invalid_lease')
            self.cancel_pending()
            if action == 'resume': return self._dispatch('prepare_preview', {})
            if action in ('pause','recoverable_hold'):
                self._preview_state = 'hold'; return {**self.info(), 'hold_confirmed': True}
            self._preview = None; self._preview_state = 'idle'
            if action == 'stop': self.stop()
            self._finish = {'state': 'idle', 'stop_confirmed': True, 'ownership_held': False,
                'return_completed': True, 'authority_released': True, 'preview': True,
                'operation_id': args.get('request_id') or secrets.token_hex(16)}
            return dict(self._finish)
        if action in ('prepare_operator_session','claim','resume'):
            if self.solver is None: self.calibrate()
            self.start()
        # Arm owns SDK handback and existing release/action 99. No neutral trajectory here.
        result = self.executor.dispatch(action, args)
        if not result.get('error'):
            if action in ('claim','resume'):
                if result.get('session_id') != self._live_session:
                    self.cancel_pending(); self._last_seq = self._last_epoch = self._joint_seq = -1
                self._live_session = result['session_id']; self._preview = None; self._finish = {}
                return {**result, **self.versions, 'preview': False}
            if action in ('stop','release','pause','recoverable_hold','finish','end_operator_session'):
                self.cancel_pending()
            if action == 'stop': self.stop()
        return result

    def receive_eef(self, packet):
        with self._lock:
            if self._closed.is_set(): raise ValueError('motion_control_stopped')
            lease = self._lease()
            body = validate(packet, lease, mode='eef_pose', now=self.gate.clock(),
                previous_seq=self._last_seq, previous_epoch=self._last_epoch, **self.versions)
            if self._preview and self._preview_state == 'hold': raise ValueError('preview_paused')
            if body['mapping_epoch'] > self._last_epoch: self.cancel_pending()
            self._last_seq, self._last_epoch = body['seq'], body['mapping_epoch']
            self._pending = body, lease, self._generation, bool(self._preview)
            self._wake.set()
        return True

    def rejected(self, route, packet, reason):
        with self._lock:
            self._decision = {'state': 'rejected','route':route,'reason':reason,'monotonic_ns':self.gate.clock()}

    def _refresh_snapshot(self):
        with self._lock: solver, generation = self.solver, self._generation
        if solver is None: return
        state = self.gate.status()
        rendered = solver.render(state, {'state':self._decision.get('state',state['state']),
            'code':self._decision.get('reason')}, bool(self._live_session and self.gate.session_id), generation)
        with self._lock:
            if solver is not self.solver or generation != self._generation: return
            if rendered['poses'] is not None:
                stamp = state['feedback']['arm_ns']
                if not self._snapshot or self._snapshot['monotonic_ns'] != stamp: self._state_seq += 1
                self._snapshot = {'poses':rendered['poses'],**self.versions,
                    'monotonic_ns':stamp,'state_seq':self._state_seq}
            self._visualization = rendered['visualization']

    def process_latest(self):
        with self._lock: work,self._pending = self._pending,None
        if work is None:
            self._refresh_snapshot(); return False
        body,lease,generation,preview = work
        try:
            state,_ = self._fresh()
            result = self.solver.solve_frame(body,state,generation)
            result = vector(result,10,'ik_q')
            with self._lock:
                if generation != self._generation or body['mapping_epoch'] != self._last_epoch or lease != self._lease(): return False
                now = self.gate.clock()
                if now >= body['valid_until_ns']: raise ValueError('command_expired')
                current,_ = self._fresh()
                if not preview:
                    if current['state'] == 'hold' and (not current.get('continuation_ready') or
                            body['generated_ns'] < (current.get('continuation_after_ns') or current.get('hold_confirmed_ns') or 2**63)):
                        raise ValueError('waiting_fresh_input_after_hold')
                    if current['state'] not in ('ready','active','hold'): raise ValueError('motion_not_armed')
                    self._joint_seq = max(self._joint_seq+1,self.gate.seq+1,body['seq'])
                    command = envelope(lease,seq=self._joint_seq,source_seq=body['source_seq'],
                        mapping_epoch=body['mapping_epoch'],generated_ns=now,valid_until_ns=body['valid_until_ns'],
                        mode='joint_position',values=result,dof=10,**self.versions)
                    record = {**{k:command[k] for k in IDENTITY},'schema':'motus.motion.envelope/1',
                              **copy.deepcopy(self.solver.last_envelope)}
                    proof = MotionEnvelope.from_record(record,dof=10)
                    if not proof.contains(current['feedback']['q']) or not proof.contains(current.get('commanded_q') or current['feedback']['q']):
                        raise ValueError('motion_envelope_stale_start')
                    self.executor.install_motion_envelope(record)
                    self.executor.publish_joint_command(command)
                else: self._preview_state = 'active'
                self._decision = {'state':'preview' if preview else 'target_published','reason':None,
                    'seq':body['seq'],'source_seq':body['source_seq'],'mapping_epoch':body['mapping_epoch'],
                    'monotonic_ns':now,'ik_ms':self.solver.last_ms}
            return True
        except (ValueError,RuntimeError) as exc:
            with self._lock:
                if generation == self._generation:
                    if str(exc).startswith('ik_worker_'): self.cancel_pending()
                    self._decision = {'state':'hold','reason':str(exc),'seq':body['seq'],
                                      'source_seq':body['source_seq'],'monotonic_ns':self.gate.clock()}
                    if not preview and self.gate.session_id == lease['session_id']:
                        try: self.gate.hold('ik_recoverable',recoverable=True)
                        except ValueError: pass
            return False
        finally:
            if generation == self._generation: self._refresh_snapshot()

    def _run(self):
        while not self._closed.is_set():
            self._wake.wait(.02); self._wake.clear()
            try: self.process_latest()
            except Exception as exc:
                with self._lock:
                    self._decision = {'state':'error','reason':str(exc)[:160] or type(exc).__name__}
