"""G1_23 arm execution: no IK, FK or collision search in the actuator loop.

The reduced gravity model follows the frozen Unitree G1_23 implementation;
see g1_motion/NOTICE.md for Apache-2.0 source provenance. SDK and Pinocchio
imports are lazy. Construction and offline tests do not connect to a robot.
"""
from collections import OrderedDict
from contextlib import contextmanager
import copy
import hashlib
import hmac
import json
import math
from pathlib import Path
import re
import secrets
import threading
import time
import xml.etree.ElementTree as ET

from common.motion.envelope import MotionEnvelope
from common.motion.protocol import validate, validate_descriptor, vector

JOINT_NAMES = tuple(f'{s}_{j}_joint' for s in ('left', 'right') for j in
                   ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll'))
MOTOR_IDS = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
LOCKED_MOTOR_IDS = tuple(range(13))
LOCKED_NAMES = tuple(f'{s}_{j}_joint' for s in ('left', 'right') for j in
                    ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')) + ('waist_yaw_joint',)
PROFILE = 'unitree_g1_23_dual_arm_relative_v1'


class GravityCompensation:
    """Private Pinocchio Data; RNEA(q_final, 0, 0), without CasADi/IK."""
    def __init__(self, profile, model_path):
        import numpy as np
        import pinocchio as pin
        model = pin.buildModelFromUrdf(str(model_path))
        locked = profile['locked_joints']
        if set(locked) != set(LOCKED_NAMES):
            raise ValueError('g1_locked_joint_mapping')
        q = pin.neutral(model)
        for name, value in locked.items():
            if not model.existJointName(name): raise ValueError('g1_locked_joint_missing')
            i = model.joints[model.getJointId(name)].idx_q
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or not model.lowerPositionLimit[i] <= value <= model.upperPositionLimit[i]):
                raise ValueError('g1_locked_joint_limit')
            q[i] = value
        self.model = pin.buildReducedModel(model, [model.getJointId(n) for n in LOCKED_NAMES], q)
        if self.model.nq != 10 or self.model.nv != 10 or tuple(self.model.names)[1:] != JOINT_NAMES:
            raise ValueError('g1_gravity_model_mismatch')
        self.data, self.pin, self.np = self.model.createData(), pin, np
        self.zero = np.zeros(10)

    def __call__(self, q):
        return self.pin.rnea(self.model, self.data, self.np.asarray(q), self.zero, self.zero).tolist()


class ArmStreamExecutor:
    def __init__(self, cfg, namespace, arm_client, *, legacy_busy=lambda: False,
                 snapshot=None, channel=None, gravity=None, clock=time.monotonic_ns):
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,63}', namespace):
            raise ValueError('invalid_robot_namespace')
        self.cfg, self.ns, self.arm_client = cfg, namespace, arm_client
        self.gate, self.clock = self, clock
        self.lock = threading.RLock()
        self._management_lock = threading.RLock()
        self._cancelled_management = {}
        self._management_receipt = None
        self._legacy_lock = threading.Lock()
        self._legacy_busy, self._legacy_pending = legacy_busy, False
        self._snapshot_provider, self._channel = snapshot, channel
        self._gravity_override, self._gravity = gravity, gravity
        self._feedback = {'q': [], 'dq': [], 'arm_ns': 0, 'fault': True}
        self._source_tick = None
        self._subscriber = None
        self._fsm_subscriber = self._arm_observer = None
        self._fsm = {'arm_ns': 0}
        self._observer_started_ns = None
        self._closed = threading.Event()
        self._thread = None
        self._rpc_thread = None
        self._publisher = None
        self.motion_control = None
        self.profile = None
        self.profile_sha256 = None
        self.profile_error = 'calibration_missing'
        self._model_path = None
        self.limits, self.torque_limits, self.velocity = [], [], 1.
        self.boot_id = secrets.token_hex(16)
        self.session_id = self.secret = None
        self.seq = self.applied_seq = self._epoch = -1
        self.state, self.reason = 'idle', None
        self.latest, self.last_q = None, None
        self.last_emit = self.clock()
        self._proofs = OrderedDict()
        self._proof = None
        self._prepared = False
        self._opening = False
        self._channel_open = channel is not None
        self._generation = 0
        self._takeover_ns = None
        self._weight = 0.
        self._recoverable = False
        self._hold_ns = self._hold_confirmed_ns = None
        self._hold_confirmed = False
        self._hold_q = None
        self.release_requested = False
        self._operation = None
        self._operations = OrderedDict()
        self._settle = None
        self._last_management = None
        self.first_fault = None
        if cfg.get('calibration_path'):
            self.configure_profile(cfg['calibration_path'], cfg.get('joint_velocity_rad_s', 1.))

    def configure_profile(self, path, velocity=1., *, expected_sha256=None):
        with self.lock:
            if self.session_id or self._opening or self._legacy_pending or self._operation and self._operation['state']!='completed':
                raise ValueError('calibration_requires_idle')
        raw = Path(path).read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if expected_sha256 is not None and digest != expected_sha256:
            raise ValueError('executor_calibration_mismatch')
        p = json.loads(raw)
        if p.get('schema') != 'motus.g1-calibration.v1' or p.get('profile_id') != PROFILE:
            raise ValueError('g1_calibration_schema')
        if p.get('arm_joint_names') != list(JOINT_NAMES): raise ValueError('g1_joint_order')
        safety = p.get('safety', {})
        modes = safety.get('allowed_fsm_ids', [safety.get('fsm_id')])
        if not isinstance(modes, list) or not modes or any(type(v) is not int or v not in (500, 801) for v in modes):
            raise ValueError('invalid_allowed_fsm_ids')
        model_path = Path(p['urdf_path'])
        if not model_path.is_absolute(): model_path = Path(path).parent/model_path
        data = model_path.read_bytes()
        if hashlib.sha256(data).hexdigest() != p['urdf_sha256']: raise ValueError('calibration_model_changed')
        joints = {j.attrib['name']: j for j in ET.fromstring(data).findall('joint')}
        limits = [(float(joints[n].find('limit').attrib['lower']), float(joints[n].find('limit').attrib['upper'])) for n in JOINT_NAMES]
        vmax = min(float(joints[n].find('limit').attrib['velocity']) for n in JOINT_NAMES)
        effort = [float(joints[n].find('limit').attrib['effort']) for n in JOINT_NAMES]
        if type(velocity) not in (int,float) or not math.isfinite(velocity) or not 0 < velocity <= vmax:
            raise ValueError('joint_velocity_limit')
        ff = p.get('feedforward_limit_nm')
        if type(ff) in (int,float): ff = [ff]*10
        torque = vector(ff, 10, 'feedforward_limit') if ff is not None else effort
        if any(t <= 0 or t > e for t,e in zip(torque,effort)): raise ValueError('feedforward_limit')
        gravity = None if self.cfg.get('servo_position') else self._gravity_override or GravityCompensation(p, model_path)
        with self.lock:
            if self.session_id or self._opening or self._legacy_pending or self._operation and self._operation['state']!='completed':
                raise ValueError('calibration_requires_idle')
            self.profile, self.profile_sha256 = p, digest
            self._model_path, self._gravity = model_path, gravity
            self.limits, self.torque_limits, self.velocity = limits, torque, float(velocity)
            self.profile_error = None

    @property
    def versions(self):
        if self.profile is None: raise ValueError('calibration_missing')
        return {'model_version': self.profile['urdf_sha256'],
                'calibration_version': self.profile_sha256, 'frame': self.profile['torso_frame']}

    @property
    def opening(self):
        with self.lock:
            return self._opening

    def cancel_preparation(self):
        """Cancel cold acquisition without waiting for IK or invoking action 99."""
        with self.lock:
            self._generation += 1
            self._prepared = False

    def _on_state(self, msg):
        try:
            tick = int(msg.tick)
            if self._source_tick is not None and not 0 < ((tick-self._source_tick) & 0xffffffff) < 0x80000000: return
            motors = msg.motor_state
            q = vector([float(motors[i].q) for i in MOTOR_IDS],10,'feedback_q')
            dq = vector([float(motors[i].dq) for i in MOTOR_IDS],10,'feedback_dq')
            # Raw SDK telemetry is observable, not a custom hardware interlock.
            telemetry = [{'id':i, 'mode':int(motors[i].mode),
                          'motorstate':int(motors[i].motorstate),
                          'voltage':float(motors[i].vol),
                          'temperature':list(motors[i].temperature)} for i in MOTOR_IDS]
            with self.lock:
                self._source_tick = tick
                self._feedback = {'q':q,'dq':dq,'arm_ns':self.clock(),
                    'source_tick':tick, 'motor_telemetry':telemetry,
                    'locked_joints': dict(zip(LOCKED_NAMES,
                        vector([float(motors[i].q) for i in LOCKED_MOTOR_IDS], 13, 'body_q')))}
        except Exception as exc:
            with self.lock:
                self._feedback = {**self._feedback,'error':type(exc).__name__}

    def _on_fsm(self, msg):
        with self.lock:
            self._fsm = {'arm_ns':self.clock(),'fsm_id':int(msg.fsm_id),'fsm_mode':int(msg.fsm_mode)}

    def _snapshot(self):
        return copy.deepcopy(self._snapshot_provider() if self._snapshot_provider else self._feedback)

    def _fresh(self):
        s = self._snapshot()
        q, dq = vector(s.get('q'),10,'feedback_q'), vector(s.get('dq'),10,'feedback_dq')
        if type(s.get('arm_ns')) is not int or not 0 <= self.clock()-s['arm_ns'] <= 100_000_000:
            raise ValueError('arm_feedback_stale')
        if s.get('error'): raise ValueError('arm_feedback_invalid')
        if s.get('external_control') is True: raise ValueError('external_control_owned')
        if self._snapshot_provider is None:
            if not 0 <= self.clock()-self._fsm['arm_ns'] <= 100_000_000:
                raise ValueError('motion_feedback_stale')
            safety = (self.profile or {}).get('safety', {})
            if self._fsm.get('fsm_id') not in safety.get('allowed_fsm_ids', [safety.get('fsm_id')]):
                raise ValueError('motion_mode_unavailable')
        return s,q,dq

    def start(self):
        with self.lock:
            if self._thread and self._thread.is_alive(): return
            if self.session_id: raise ValueError('executor_restart_requires_confirmed_release')
            if self._snapshot_provider is None and self._subscriber is None:
                from unitree_sdk2py.core.channel import ChannelSubscriber
                from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
                self._subscriber = ChannelSubscriber('rt/lowstate', LowState_)
                self._subscriber.Init(self._on_state,1)
                from sport_mode_state import SportModeState_
                from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
                self._fsm_subscriber = ChannelSubscriber('rt/sportmodestate', SportModeState_)
                self._fsm_subscriber.Init(self._on_fsm,1)
                self._arm_observer = ChannelSubscriber('rt/arm_sdk', LowCmd_)
                self._arm_observer.Init(lambda msg:None,0)
                self._observer_started_ns = self.clock()
            self._closed.clear()
            self._thread = threading.Thread(target=self._run,name='g1-arm-stream',daemon=True)
            self._thread.start()

    def _run(self):
        while not self._closed.is_set():
            begin = time.monotonic()
            self.tick()
            self._closed.wait(max(0., .004-(time.monotonic()-begin)))

    def set_joint_publisher(self, callback):
        self._publisher = callback

    def publish_joint_command(self, packet):
        if self._publisher is None: raise ValueError('local_dds_unavailable')
        return self._publisher(packet)

    def install_motion_envelope(self, record):
        proof = MotionEnvelope.from_record(record,dof=10)
        with self.lock:
            if record['session_id'] != self.session_id or record['boot_id'] != self.boot_id:
                raise ValueError('motion_envelope_session')
            if not proof.matches(record,now_ns=self.clock()): raise ValueError('motion_envelope_expired')
            self._proofs[record['seq']] = proof
            while len(self._proofs)>8: self._proofs.popitem(last=False)

    def accept_control(self, packet):
        with self.lock:
            if not self.session_id or self.state not in ('ready','active','hold') or self.release_requested:
                raise ValueError('motion_not_armed')
            lease = dict(boot_id=self.boot_id,session_id=self.session_id,secret=self.secret)
            body = validate(packet,lease,mode='joint_position',dof=10,now=self.clock(),
                previous_seq=self.seq,previous_epoch=self._epoch,**self.versions)
            if self.state == 'hold':
                if not self._recoverable: raise ValueError('hold_not_resumable')
                ready_after = self._continuation_after()
                if ready_after is None or body['generated_ns'] < ready_after: return False
            if any(not lo <= q <= hi for q,(lo,hi) in zip(body['values'],self.limits)):
                self.hold('joint_limit'); raise ValueError('joint_limit')
            proof = self._proofs.get(body['seq'])
            if proof is None or not proof.matches(body,now_ns=self.clock()):
                self.hold('motion_envelope_unavailable',recoverable=True)
                raise ValueError('motion_envelope_unavailable')
            self._fresh()
            self.latest, self._proof = body, proof
            self.seq, self._epoch = body['seq'],body['mapping_epoch']
            self.state,self.reason = 'ready',None
            self._recoverable = self._hold_confirmed = False
            self._hold_ns = self._hold_q = None
            return True

    def hold(self, reason='operator_pause', release=False, *, recoverable=False):
        with self.lock:
            if recoverable and (self.state=='fault' or self.release_requested
                    or self.state=='hold' and not self._recoverable): raise ValueError('hold_not_resumable')
            self._generation += 1
            self.latest = self._proof = None
            self.release_requested |= release
            if self.session_id or self._channel_open:
                if self.state not in ('hold','fault'):
                    self._hold_ns = self._hold_q = None
                    self._hold_confirmed = False
                if self.state != 'fault': self.state,self.reason = 'hold',reason
            self._recoverable = bool(recoverable and not self.release_requested and self.state!='fault')
            return self.status()

    @contextmanager
    def legacy(self):
        if not self._legacy_lock.acquire(False): raise ValueError('legacy_motion_pending')
        try:
            with self.lock:
                if self.session_id or self._opening or self._operation and self._operation['state']!='completed':
                    raise ValueError('motion_owned_by_teleop')
                self._legacy_pending = True
            yield
        finally: self._legacy_lock.release()

    def _authorized(self, args):
        return (self.session_id and args.get('session_id')==self.session_id
                and isinstance(args.get('secret'),str) and hmac.compare_digest(args['secret'],self.secret))

    def _claim(self):
        with self.lock:
            if self.session_id or self._opening: raise ValueError('motion_owned')
            if self._operation and self._operation['state']!='completed': raise ValueError('release_pending')
            if self._legacy_pending or self._legacy_busy(): raise ValueError('legacy_motion_pending')
            if not self._prepared or self.cfg.get('live_enabled') is not True: raise ValueError('live_not_prepared')
            if self.profile is None or (not self.cfg.get('servo_position') and self._gravity is None): raise ValueError('calibration_missing')
            s,q,dq = self._fresh()
            if max(map(abs,dq))>.02: raise ValueError('robot_not_stopped')
            generation = self._generation
            self._opening = True
        try:
            if not self._channel_open:
                from arm_sdk import ArmSdkChannel
                self._channel = self._channel or ArmSdkChannel(grippers=False,waist=False,driven_arm_ids=MOTOR_IDS)
                self._channel.open()
            with self.lock:
                self._channel_open = True
                if generation != self._generation: raise ValueError('claim_cancelled')
                _,q,dq = self._fresh()
                if max(map(abs,dq))>.02: raise ValueError('robot_not_stopped')
                self.session_id,self.secret = secrets.token_hex(16),secrets.token_hex(32)
                self.seq,self.applied_seq,self._epoch = -1,-1,-1
                self._proofs.clear(); self.latest=self._proof=None
                self.state,self.reason = 'ready',None
                self.last_q,self.last_emit = q,self.clock()
                self._weight=0.; self._takeover_ns=self.clock()
                self._hold_confirmed=False;self.release_requested=False
                self._operation=None
                return {'state':'ready','boot_id':self.boot_id,'session_id':self.session_id,'secret':self.secret}
        except Exception:
            if self._channel and not self.session_id:
                self._channel.close();self._channel_open=False
            raise
        finally:
            with self.lock:self._opening=False

    def _release(self, args, *, action99=True, cancel_pending=False):
        ident = args.get('operation_id') or args.get('request_id')
        if ident is not None and (not isinstance(ident,str) or not 1<=len(ident)<=128):
            raise ValueError('invalid_operation_id')
        with self.lock:
            if (cancel_pending and not action99 and not self.session_id
                    and not self._opening and not self._channel_open and not self._legacy_pending
                    and not (self._rpc_thread and self._rpc_thread.is_alive())
                    and (self._operation is None or self._operation['state']=='completed')):
                # Canvas can stop both arm and teleop_control. If no SDK control
                # was acquired, there is nothing to hand back or physically confirm.
                # Do not create an operation that would itself manufacture ownership.
                return {'state':'completed','authority_released':True,'return_required':False,
                        'return_completed':False,'physical_confirmed':False,'no_op':True}
            if cancel_pending and self._operation and self._operation['state'] not in ('completed','failed','unknown'):
                op=self._operation
                op['cancel_requested']=True
                if (op['state'] in ('holding','handback','confirming_handback')
                        or op['state']=='awaiting_feedback' and op['action_id'] is None):
                    # Continue the safe SDK handback, but do not start a vendor
                    # gesture after an explicit stop cancelled its pending work.
                    op['action_id']=None
                else:
                    # ExecuteAction has no cancel/status API. Once 99 was sent,
                    # a local stop cannot honestly report the vendor action as
                    # cancelled or acquire its control back silently.
                    op.update(state='unknown',reason='release_action_stop_unconfirmed')
                return copy.deepcopy(op)
            if ident in self._operations: return copy.deepcopy(self._operations[ident])
            if self._operation and self._operation['state'] not in ('completed','failed','unknown'):
                return copy.deepcopy(self._operation)
            if self._rpc_thread and self._rpc_thread.is_alive(): raise ValueError('release_result_unknown')
            if self._operation and self._operation['state']=='unknown':
                # An unknown SDK outcome is not permission to issue action 99
                # again. An operator must explicitly retry after fresh rest.
                if args.get('retry') is not True: raise ValueError('release_result_unknown')
                _,_,dq=self._fresh()
                if max(map(abs,dq))>.02: raise ValueError('robot_not_stopped')
            if self._legacy_busy(): raise ValueError('external_control_owned')
            self._fresh()
            self.hold('release_requested',release=True)
            self._prepared=False
            ident = ident or secrets.token_hex(16)
            self._operation = {'operation_id':ident,'state':'holding','started_ns':self.clock(),
                'action_id':99 if action99 else None,'sdk_return':None,'sdk_returned_ns':None,
                'zero_writes':0,'handback_confirmed':False,'physical_confirmed':False,
                'return_completed':False,'authority_released':False}
            self._operations[ident]=self._operation
            while len(self._operations)>32:self._operations.popitem(last=False)
            self._settle=None
            return copy.deepcopy(self._operation)

    def dispatch(self, action, args):
        if action in ('stop', 'end_operator_session'):
            # Claim may wait for a low-state sample in SDK open(). Stop fences
            # that acquisition immediately, without waiting for its RPC lock.
            self.cancel_preparation()
            return self._dispatch(action, args)
        # Management retries/cancellation follow the same bounded nonce contract
        # as Tianyi. A lost claim/resume reply must never create a second lease.
        with self._management_lock:
            try:
                ident=args.get('request_id')
                receipt=self._management_receipt
                now=self.clock()
                self._cancelled_management={k:v for k,v in self._cancelled_management.items() if v>now}
                if ident is not None and action in ('claim','resume','release'):
                    if not isinstance(ident,str) or not re.fullmatch('[0-9a-f]{32}',ident):
                        raise ValueError('invalid_management_request')
                    until=args.get('request_valid_until_ns')
                    if type(until) is not int: raise ValueError('invalid_management_deadline')
                    if action!='release' and ident in self._cancelled_management: raise ValueError('management_cancelled')
                    if receipt and ident==receipt['id']:
                        original=receipt['args']
                        if (until!=original['request_valid_until_ns'] or args.get('session_id')!=original.get('session_id')
                                or not hmac.compare_digest(str(args.get('secret','')),str(original.get('secret','')))):
                            raise ValueError('invalid_management_request')
                        if action=='release':
                            if self.session_id not in (None,receipt['result']['session_id']): raise ValueError('management_owner_changed')
                            receipt['cancelled']=True
                            credentials={k:receipt['result'][k] for k in ('session_id','secret')} if self.session_id else {}
                            return self._dispatch('release',{**args,**credentials})
                        if receipt['cancelled']: raise ValueError('management_cancelled')
                        if action!=receipt['action']: raise ValueError('invalid_management_request')
                        if now>=until: raise ValueError('management_request_expired')
                        if self.session_id!=receipt['result']['session_id']: raise ValueError('management_lease_released')
                        return dict(receipt['result'])
                    if action=='release':
                        if len(self._cancelled_management)>=64: raise ValueError('management_cancel_queue_full')
                        self._cancelled_management[ident]=now+300_000_000
                        if not self.session_id and not self._legacy_pending and not self._channel_open:
                            return {**self.status(),'cancelled_request_id':ident,'cancelled_without_output':True}
                    elif not 0<until-now<=300_000_000:
                        raise ValueError('management_request_expired' if until<=now else 'invalid_management_deadline')
                result=self._dispatch(action,args)
                if not result.get('error') and ident is not None and action in ('claim','resume'):
                    if receipt:self._cancelled_management[receipt['id']]=receipt['args']['request_valid_until_ns']
                    self._management_receipt={'id':ident,'action':action,'args':dict(args),'result':dict(result),'cancelled':False}
                elif not result.get('error') and receipt and action in ('pause','release','stop','finish','end_operator_session'):
                    receipt['cancelled']=True
                return result
            except (ValueError,RuntimeError,OSError) as exc:
                return {'state':'error','code':str(exc),'error':str(exc)}

    def _dispatch(self, action, args):
        try:
            if action=='info':return self.info()
            if action=='start':
                if args.get('input_topic') not in (None,f'/{self.ns}/motion/arm/command'):
                    raise ValueError('arm_input_topic_mismatch')
                interfaces=args.get('control_interfaces',{})
                if not isinstance(interfaces,dict):raise ValueError('invalid_control_descriptor')
                for source,key in ((args,'control_interface'),(interfaces,'joints')):
                    if key in source:
                        if self.motion_control is None:raise ValueError('control_descriptor_unavailable')
                        validate_descriptor(source[key],self.motion_control.control_interface('joint_position'))
                self.start();return {'state':'ready'}
            if action=='prepare_operator_session':
                if self.cfg.get('live_enabled') is not True:raise ValueError('live_disabled')
                # Admission uses live robot state, not manually signed acceptance records.
                self._fresh();self._prepared=True;return {'state':'ready','prepared':True}
            if action=='claim':return self._claim()
            if action=='resume':
                if not self._authorized(args):raise ValueError('invalid_lease')
                with self.lock:
                    if self.state!='hold' or self.release_requested or not (self._hold_confirmed or self.cfg.get('servo_position') and self._hold_ns is not None):raise ValueError('hold_not_resumable')
                    self._fresh();self.state,self.reason='ready',None
                    self.session_id,self.secret=secrets.token_hex(16),secrets.token_hex(32)
                    self.seq=self.applied_seq=self._epoch=-1
                    self._proofs.clear()
                    self.latest=self._proof=None;self._hold_confirmed=False
                    self.last_emit=self.clock()
                    return {'state':'ready','boot_id':self.boot_id,'session_id':self.session_id,'secret':self.secret}
            if action in ('pause','recoverable_hold'):
                if not self._authorized(args):raise ValueError('invalid_lease')
                return self.hold('ik_recoverable' if action=='recoverable_hold' else 'operator_pause',recoverable=action=='recoverable_hold')
            if action=='finish_status':
                with self.lock:
                    ident=args.get('operation_id')
                    op=self._operations.get(ident) if ident else self._operation
                    if op is None:raise ValueError('unknown_finish_operation')
                    return copy.deepcopy(op)
            alias = action=='execute' and (args.get('action_id')==99 or str(args.get('gesture','')).strip().lower()=='release arm')
            if action in ('release','finish','stop','end_operator_session') or alias:
                # Direct arm/Canvas release intentionally survives a dead input
                # client. Another active source is still fenced by the bundle.
                ident=args.get('operation_id') or args.get('request_id')
                if action not in ('stop','end_operator_session') and isinstance(ident,str) and ident in self._operations:
                    return copy.deepcopy(self._operations[ident])
                if args.get('session_id') or args.get('secret'):
                    if not self._authorized(args):raise ValueError('invalid_lease')
                return self._release(args,action99=self._legacy_pending or action not in ('stop','end_operator_session'),
                                     cancel_pending=action in ('stop','end_operator_session'))
            raise ValueError('unknown_action')
        except (ValueError,RuntimeError,OSError) as exc:
            return {'state':'error','code':str(exc),'error':str(exc)}

    def _write(self, q, tau, weight):
        if not self._channel_open: raise ValueError('arm_stream_not_open')
        if self.cfg.get('servo_position'):
            written = self._channel.publish_servo_position(q, weight=weight)
        else:
            written = self._channel.publish_stream(q,tau,weight=weight)
        if written is not True:
            raise ValueError('arm_sdk_write_failed')
        self.last_q,self.last_emit,self._weight = list(q),self.clock(),weight

    def _stationary(self, s, q, dq, after):
        if s['arm_ns']<=after or max(map(abs,dq))>.02:
            self._settle=None;return False
        if self._settle is None or max(abs(a-b) for a,b in zip(q,self._settle[1]))>.002:
            self._settle=(s['arm_ns'],q);return False
        return s['arm_ns']-self._settle[0]>=100_000_000

    def _invoke99(self, op):
        try:
            ret=self.arm_client.ExecuteAction(99)
            with self.lock:
                op['sdk_return'],op['sdk_returned_ns']=ret,self.clock()
                if op['state']=='unknown':return  # Do not overwrite an observed unknown result.
                success=type(ret) is int and ret==0
                op['state']='awaiting_feedback' if success else 'failed'
                if not success:op['reason']='release_sdk_error'
                self._settle=None
        except Exception as exc:
            with self.lock:op.update(state='unknown',reason=type(exc).__name__)

    def _release_tick(self,s,q,dq):
        op=self._operation
        if op is None or op['state'] in ('completed','failed','unknown'):return
        now=self.clock()
        if now-op['started_ns']>10_000_000_000:
            op.update(state='unknown',reason='release_timeout');return
        if op['state']=='holding':
            if self._channel_open:
                self._write(q,None,self._weight)
            else:
                # No SDK stream was acquired (e.g. an existing vendor gesture).
                # There are no zero-weight writes to claim. Action 99 itself
                # cancels that gesture; only subsequent measured rest completes.
                op['handback_confirmed']=True
                op['sdk_stream_present']=False
                if op['action_id']==99:
                    op['state']='sdk_pending'
                    self._rpc_thread=threading.Thread(target=self._invoke99,args=(op,),name='g1-arm-release',daemon=True)
                    self._rpc_thread.start()
                else:
                    op['state']='awaiting_feedback';op['sdk_returned_ns']=now;self._settle=None
                return
            op.update(state='handback',handback_started_ns=now,initial_weight=self._weight)
            return
        if op['state']=='handback':
            weight=max(0.,op['initial_weight']*(1-(now-op['handback_started_ns'])/2e9))
            if self._channel_open:self._write(q,None,weight)
            if weight==0:
                op['zero_writes']+=1
                if op['zero_writes']>=5:
                    op.update(state='confirming_handback',zero_confirmed_ns=now)
                    self._settle=None
            return
        if op['state']=='confirming_handback':
            if not self._stationary(s,q,dq,op['zero_confirmed_ns']):return
            op['handback_confirmed']=True
            if op['action_id']==99:
                op['state']='sdk_pending'
                self._rpc_thread=threading.Thread(target=self._invoke99,args=(op,),name='g1-arm-release',daemon=True)
                self._rpc_thread.start()
            else:op['state']='awaiting_feedback';op['sdk_returned_ns']=now;self._settle=None
            return
        if op['state']=='awaiting_feedback' and self._stationary(s,q,dq,op['sdk_returned_ns']):
            op.update(state='completed',physical_confirmed=True,return_completed=True,
                      authority_released=True,completed_ns=now)
            self.session_id=self.secret=None
            self._legacy_pending=False;self._prepared=False
            self.state,self.reason='idle','release_confirmed'
            self._hold_confirmed=True
            if self._channel_open:self._channel.close();self._channel_open=False

    def tick(self):
        with self.lock:
            if not self.session_id and not (self._operation and self._operation['state'] not in ('completed','failed','unknown')):return
            try:
                s,q,dq=self._fresh()
                if self._operation and self._operation['state'] not in ('completed','failed','unknown'):
                    self._release_tick(s,q,dq);return
                if self.state=='fault':return
                now=self.clock()
                if self.latest and now>=self.latest['valid_until_ns']:
                    self.hold('command_timeout',recoverable=True)
                if self.state=='hold':
                    if self._hold_ns is None:
                        if self.cfg.get('servo_position'):
                            self._channel.forget_target()
                            self._hold_q,self._hold_ns=q,self.clock()
                            return
                        self._write(q,None,self._weight)
                        self._hold_q,self._hold_ns=q,self.clock();return
                    if not self._hold_confirmed and s['arm_ns']>self._hold_ns and max(map(abs,dq))<=.02 and max(abs(a-b) for a,b in zip(q,self._hold_q))<=.02:
                        self._hold_confirmed=True;self._hold_confirmed_ns=now
                    return
                if self.latest is None:return
                servo = self.cfg.get('servo_position')
                if servo and self.applied_seq == self.latest['seq']:return
                dt=max(0.,min((now-self.last_emit)/1e9,.02))
                limit=self.velocity*dt
                if servo:
                    # Continuity/reference handoff follows jsmy-CTH's PR #322:
                    # https://github.com/4paradigm/phanthymotus-driver/pull/322
                    # Use the last successful output, not measured-position noise.
                    # The 120ms exponential filter is an addition in this PR.
                    # Cap dt after input gaps so
                    # a regrip cannot skip smoothing by accumulating idle time.
                    smooth_dt=max(0.,min((now-self.last_emit)/1e9,.05))
                    alpha=-math.expm1(-smooth_dt/.12)
                    # 1 rad/s reference slew limit, independent for each joint.
                    max_step=1.0*smooth_dt
                    next_q=[p+max(-max_step,min(max_step,alpha*(t-p)))
                            for p,t in zip(self.last_q,self.latest['values'])]
                else:
                    next_q=[p+max(-limit,min(limit,t-p)) for p,t in zip(self.last_q,self.latest['values'])]
                if (not self._proof or not self._proof.allows(self.latest,q,self.last_q,next_q,now_ns=now)
                        or any(not lo<=x<=hi for x,(lo,hi) in zip(next_q,self.limits))):
                    self.hold('motion_envelope_exceeded',recoverable=True);return
                tau = None if servo else vector(self._gravity(next_q),10,'gravity_tau')
                if tau is not None and any(abs(t)>lim for t,lim in zip(tau,self.torque_limits)):raise ValueError('gravity_torque_limit')
                if self.clock()>=self.latest['valid_until_ns']:
                    self.hold('command_timeout',recoverable=True);return
                weight=min(1.,max(0.,(now-self._takeover_ns)/1e9))**2
                self._write(next_q,tau,weight)
                self.applied_seq=self.latest['seq'];self.state,self.reason='active',None
            except Exception as exc:
                snapshot=self._snapshot()
                age=self.clock()-(self._fsm.get('arm_ns',0) if str(exc)=='motion_feedback_stale' else snapshot.get('arm_ns',0))
                if str(exc) in ('arm_feedback_stale','motion_feedback_stale') and 0 <= age <= 300_000_000 and self.state!='fault':
                    try:self.hold(str(exc),recoverable=self.state in ('ready','active') or self._recoverable)
                    except ValueError:pass
                    self._hold_confirmed=False
                    return
                self.latest=self._proof=None
                self.state,self.reason='fault',str(exc)
                if self.first_fault is None:self.first_fault={'reason':str(exc),'monotonic_ns':self.clock()}
                if self._operation and self._operation['state'] not in ('completed','failed','unknown'):
                    self._operation.update(state='failed',reason=str(exc))

    def _continuation_after(self):
        if self.state != 'hold' or not self._recoverable or self.release_requested:
            return None
        # Resuming servo input is not a claim that the robot is physically
        # stopped. A transient IK gap only discards pending work; a fresh
        # target may follow the existing motion without a stop/restart cycle.
        if self.cfg.get('servo_position') and self._hold_ns is not None:
            return self._hold_ns
        return self._hold_confirmed_ns if self._hold_confirmed else None

    def status(self):
        with self.lock:
            return {'state':self.state,'reason':self.reason,'boot_id':self.boot_id,'session_id':self.session_id,
                'sequence':self.seq,'applied_sequence':self.applied_seq,'feedback':self._snapshot(),
                'commanded_q':copy.deepcopy(self.last_q),'ownership_held':bool(self.session_id or self._opening or self._legacy_pending
                    or self._operation and self._operation['state']!='completed'),
                'output_active':self.state=='active','stop_confirmed':self._hold_confirmed,
                'hold_confirmed':self._hold_confirmed,'hold_confirmed_ns':self._hold_confirmed_ns,
                'continuation_ready':self._continuation_after() is not None,
                'continuation_after_ns':self._continuation_after(),
                'resume_ready':self.state=='hold' and not self.release_requested and bool(self._hold_confirmed or self.cfg.get('servo_position') and self._hold_ns is not None),
                'release_requested':self.release_requested,'weight':self._weight,'monotonic_ns':self.clock(),
                'timing_policy':{'target_max_ms':300,'feedback_hold_ms':100,'feedback_fault_timeout_ms':300,
                                 'recoverable_hold':True,'management_retry_ms':300},
                'first_fault':copy.deepcopy(self.first_fault),'release':copy.deepcopy(self._operation)}

    def info(self):
        fields=self.motion_control.feedback_fields() if self.motion_control else {}
        return {**self.status(),**fields,'calibration_sha256':self.profile_sha256,'calibration_error':self.profile_error}

    def stop(self):
        with self.lock:
            complete=self._operation and self._operation.get('authority_released') and not self.session_id
            idle=not self.session_id and not self._legacy_pending and not self._channel_open and not self._opening and self._operation is None
            result=copy.deepcopy(self._operation) if complete else {'state':'idle','authority_released':True} if idle else None
        if result is None:result=self.dispatch('stop',{})
        # Keep the loop alive until handback has physical evidence. Closing a
        # publisher or clearing a session is never a stop receipt.
        if result.get('error') or not result.get('authority_released'):return result
        self._closed.set()
        if self._thread and self._thread is not threading.current_thread():self._thread.join(.5)
        for name in ('_subscriber','_fsm_subscriber','_arm_observer'):
            sub=getattr(self,name)
            if sub:sub.Close();setattr(self,name,None)
        return result
