"""FR3 wrist observation and RealMan-compatible pick-and-place actions."""
import copy
import logging
import math
import threading
import time
from uuid import uuid4

import numpy as np
from common.vendor_runtime import action_schema, tool
from hardware import JOINT_LOWER, JOINT_UPPER, JOINT_MARGIN
from .inputs import ObservationInputs

log = logging.getLogger(__name__)


def joint_target(value):
    if not isinstance(value, str):
        raise ValueError('observation_joints_deg must be seven comma-separated angles')
    values = [float(x.strip()) for x in value.split(',')]
    if len(values) != 7 or any(not math.isfinite(v) or not lo+JOINT_MARGIN <= math.radians(v) <= hi-JOINT_MARGIN
                             for v, lo, hi in zip(values, JOINT_LOWER, JOINT_UPPER)):
        raise ValueError('Observation angles exceed FR3 joint limits')
    return values


def pose_vector(matrix):
    t = np.asarray(matrix, dtype=float).reshape(4, 4, order='F')
    if not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-6):
        raise RuntimeError('Invalid SDK end-effector transform')
    r = t[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-4) or np.linalg.det(r) < .999:
        raise RuntimeError('Invalid SDK end-effector rotation')
    cy = math.hypot(r[0, 0], r[1, 0])
    ry = math.atan2(-r[2, 0], cy)
    rx, rz = ((math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0]))
              if cy > 1e-9 else (0., math.atan2(-r[0, 1], r[1, 1])))
    return [*t[:3, 3].tolist(), rx, ry, rz]


class PickPlacePlugin:
    PREFIX = 'vision_pick_and_drop'

    def __init__(self, config, manager, camera, ros2, callback=None, inputs=None, gripper_manager=None):
        profile = config.get('vision_pick_and_drop', {})
        self.addresses = config.get('motion_arm_ips', {})
        self.profiles = {'right': profile, **profile.get('arm_profiles', {})}
        self.arm = 'right'
        self.arm_labels = {arm: f'{self.addresses[arm]}（{name}）'
                           for arm, name in (('left', '左手'), ('right', '右手'))
                           if arm in self.profiles and self.addresses.get(arm)}
        self.address = self.addresses.get(self.arm)
        self.serial = profile.get('camera_serial', '')
        self.gripper_manager = gripper_manager
        self.manager = manager
        self.planning_profile = profile.get('planning')
        self.inputs = inputs or ObservationInputs(ros2, camera, self.serial)
        self.config = dict(arm=self.arm_labels.get(self.arm, 'right'),
                           observation_joints_deg=profile.get('observation_joints_deg', ''),
                           x_compensation_mm=0, y_compensation_mm=0, z_compensation_mm=0,
                           observe_after_transfer=profile.get('observe_after_transfer', False),
                           force_n=profile.get('force_n'))
        self.arm_configs = {self.arm: dict(self.config)}
        if callback is None:
            from device import JointControlPlugin
            callback = lambda aid, status, result: JointControlPlugin._post_completion(self, aid, status, result)
        self.callback = callback
        self.lock = threading.RLock()
        self.active = self.last_result = self.observation = None
        self.enabled = False

    def get_tools(self):
        schema = action_schema({
            'observe': (['confirm_motion'], 'Move to the configured observation pose, settle and retain one synchronized RGB-D/VOP photograph. Uses a fixed 2 deg/s observation speed. Opens the unloaded hand to its maximum permitted width. Wait for ACP completion.'),
            'grab_to': (['start_point_x','start_point_y','target_point_x','target_point_y','rotation_deg','speed_deg_s','speed_mm_s','confirm_motion'], 'Pick and place at normalized coordinates in the same observation photo, release and return to observation.'),
            'grab_by': (['start_point_x','start_point_y','delta_x','delta_y','rotation_deg','speed_deg_s','speed_mm_s','confirm_motion'], 'Pick and translate along the observation image right/down directions in actual table millimetres, release and return to observation.'),
            'cancel': ([], 'Cancel observation and stop only this card\'s active motion. Never opens the gripper.'),
        }, {'confirm_motion': {'type': 'boolean', 'const': True},
            'speed_deg_s': {'type': 'number', 'minimum': .1, 'maximum': 5,
                'description': 'Required arm joint speed limit in degrees per second, including the return path.'},
            'speed_mm_s': {'type': 'number', 'minimum': .1, 'maximum': 100,
                'description': 'Required Franka Hand opening and closing speed in mm/s.'},
            'start_point_x': {'type': 'number', 'minimum': -1, 'maximum': 1},
            'start_point_y': {'type': 'number', 'minimum': -1, 'maximum': 1},
            'target_point_x': {'type':'number','minimum':-1,'maximum':1},
            'target_point_y': {'type':'number','minimum':-1,'maximum':1},
            'delta_x': {'type':'number','description':'Actual table displacement in mm; positive toward observation image right.'},
            'delta_y': {'type':'number','description':'Actual table displacement in mm; positive toward observation image down.'},
            'rotation_deg': {'type':'number','minimum':-180,'maximum':180,'default':0,
                'description':'Extra rotation after lifting, about the gripper downward axis; positive clockwise viewed from above. Default 0.'},
        })
        schema['allOf'] = [{'if': {'properties': {'action': {'enum': ['observe', 'grab_to', 'grab_by']}}},
                            'then': {'required': ['confirm_motion']}}]
        schema.update({'x-completion': {'actions': ['observe', 'grab_to', 'grab_by'], 'timeout': 1200}, 'x-resource': 'arm',
                       'x-is-dangerous': True,
                       'x-hooks': {'on_interrupt_motion': {'action': 'cancel'},
                                   'on_interrupt_all': {'action': 'cancel'}}})
        definition = tool(self.PREFIX, 'actuator',
            'FR3 wrist pick and place. Run observe, then select objects[].position in [-1,1]: X right, Y down. '
            'grab_to uses a placement point in the same photograph; grab_by uses actual table millimetres aligned with that photograph. '
            'Returns running/action_id followed by ACP completion or failure. Successful transfers release and return to observation. '
            'observe_after_transfer optionally captures a new observation after returning. Check transfer_completed, return_completed, '
            'recovery_required and observation_required before starting another task. '
            'Rotation defaults to zero; only request extra rotation when needed. Force is read from configuration; observe uses 2 deg/s; supply arm speed_deg_s and hand speed_mm_s on each transfer. FR3 force is in newtons, not a RealMan raw register value.',
            schema, topic_in=[dict(format=fmt, desc=desc, **({'topic': self.inputs.topics[i]} if self.inputs.topics else {}))
                              for i, fmt, desc in ((1, 'image/depth-zlib', 'Depth image'),
                                  (0, 'image/jpeg', 'RGB image'), (2, 'data/json', 'VOP objects'))])
        definition['configSchema'] = {'type': 'object', 'additionalProperties': False, 'required': ['force_n'], 'properties': {
            'arm': {'type': 'string', 'enum': list(self.arm_labels.values()),
                'default': self.config['arm'],
                'description': 'Robot IP and arm. Switching selects its wrist camera, calibration and saved observation pose; observe again before transferring.'},
            **{f'{arm}_observation_joints_deg': {
                'type': 'string', 'title': f'{arm.title()} observation joints',
                'default': self.arm_configs.get(arm, self.profiles[arm]).get('observation_joints_deg', ''),
                'x-show-when': {'arm': label},
                'description': 'J1–J7 in degrees, comma-separated.'}
                for arm, label in self.arm_labels.items()},
            **{f'{axis}_compensation_mm': {
                'type': 'number', 'minimum': -50, 'maximum': 50, 'default': 0,
                'description': description}
                for axis, description in (
                    ('x', 'Additional offset in mm. Positive toward image right along the table.'),
                    ('y', 'Additional offset in mm. Positive toward image down along the table.'),
                    ('z', 'Additional depth in mm. Positive downward along the tool axis.'))},
            'force_n': {'type': 'number', 'minimum': 30, 'maximum': 70,
                'description': 'Required Franka Hand grasp force in newtons; configure explicitly before transferring.'},
            'observe_after_transfer': {'type': 'boolean', 'default': False,
                'description': 'Capture a new RGB-D/VOP observation after returning; otherwise observe again before the next transfer.'}}}
        return [definition]

    def start(self):
        pass  # Inputs start only after the canvas supplies the three topics.

    def info(self):
        with self.lock:
            return dict(state='running' if self.active else 'ready' if self.enabled else 'idle',
                        arm=self.arm, robot_ip=self.address, camera_serial=self.serial,
                        config=dict(self.config),
                        topic_in=self.get_tools()[0]['topic_in'],
                        stage=self.active.get('stage') if self.active else None,
                        action_id=self.active.get('action_id') if self.active else None,
                        observation_id=(self.observation or {}).get('result', {}).get('observation_id'),
                        observation_required=self.observation is None,
                        observation=copy.deepcopy((self.observation or {}).get('result')),
                        topic_out=[], inputs=self.inputs.info(),
                        last_result=copy.deepcopy(self.last_result))

    def stop(self):
        self.cancel()
        with self.lock: self.enabled = False; self.observation = None
        self.inputs.stop()

    def cancel(self):
        with self.lock:
            active = self.active
            if not active: return dict(state='idle', observation_required=self.observation is None)
            active['cancel'].set()
            if active.get('session') and active.get('command_id'):
                active['session'].stop_motion(owner=active['owner'], command_id=active['command_id'])
            return dict(state='stopping', action_id=active['action_id'])

    def dispatch(self, action, args):
        if action == 'info': return self.info()
        if action == 'cancel': return self.cancel()
        if action == 'stop': self.stop(); return self.info()
        with self.lock:
            if self.active: return dict(state='error', code='ACTION_IN_PROGRESS', message='Wait for the current action')
            if action == 'config':
                updates = {k: v for k, v in args.items() if k not in ('_tool_name', 'instance_id')}
                # Ignore the removed persistent hand-speed field from older saved canvases.
                updates.pop('speed_mm_s', None)
                updates.pop('speed_percent', None)
                legacy_depth = updates.pop('depth_compensation_mm', None)
                poses = {arm: updates.pop(f'{arm}_observation_joints_deg', None)
                         for arm in self.arm_labels}
                if updates.keys()-self.config.keys():
                    return dict(ok=False,code='INVALID_CONFIG',message='Unknown vision_pick_and_drop configuration field')
                try:
                    label = updates.get('arm', self.config['arm'])
                    arm = next((a for a, value in self.arm_labels.items() if value == label), None)
                    if arm is None:
                        raise ValueError('Select a configured robot IP and arm')
                    profile = self.profiles[arm]
                    defaults = dict(arm=label, observation_joints_deg=profile.get('observation_joints_deg', ''),
                                    x_compensation_mm=0, y_compensation_mm=0, z_compensation_mm=0,
                                    observe_after_transfer=profile.get('observe_after_transfer', False),
                                    force_n=profile.get('force_n', self.config['force_n']))
                    saved = self.arm_configs.get(arm, defaults)
                    if poses[arm] is not None:
                        updates['observation_joints_deg'] = poses[arm]
                    pose = updates.get('observation_joints_deg')
                    other = [self.arm_configs.get(key, self.profiles[key])
                             for key in self.profiles if key != arm]
                    if poses[arm] is None and pose and any(
                        old.get('observation_joints_deg') and
                        joint_target(pose) == joint_target(old['observation_joints_deg'])
                        for old in other
                    ):
                        updates.pop('observation_joints_deg')
                        for axis in 'xyz':
                            updates.pop(f'{axis}_compensation_mm', None)
                        legacy_depth = None
                    if legacy_depth is not None and 'z_compensation_mm' not in updates:
                        baseline = (profile.get('planning') or {}).get('grasp_depth_offset_mm', 0)
                        if type(legacy_depth) not in (int, float):
                            raise ValueError('Invalid legacy depth compensation')
                        updates['z_compensation_mm'] = legacy_depth - baseline
                    cfg = {**saved, **updates}
                    joint_target(cfg['observation_joints_deg'])
                    handeye = (profile.get('planning') or {}).get('handeye')
                    if handeye and (handeye.get('robot_ip') != self.addresses[arm] or
                                    handeye.get('camera_serial') != profile.get('camera_serial')):
                        raise ValueError('Selected arm hand-eye identity does not match its robot and camera')
                    if type(cfg['observe_after_transfer']) is not bool:
                        raise ValueError('observe_after_transfer must be a boolean')
                    for name, low, high in (('force_n', 30, 70), *((f'{axis}_compensation_mm', -50, 50) for axis in 'xyz')):
                        value = cfg[name]
                        if name == 'force_n' and value is None: continue
                        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                            raise ValueError(f'{name} must be configured in [{low}, {high}]')
                except (ValueError, OverflowError) as exc:
                    return dict(ok=False,code='INVALID_CONFIG',message=str(exc))
                if cfg != self.config:
                    self.arm_configs[self.arm] = dict(self.config)
                    if arm != self.arm:
                        self.inputs.select_serial(profile.get('camera_serial', ''))
                    self.arm, self.address = arm, self.addresses[arm]
                    self.serial = profile.get('camera_serial', '')
                    self.planning_profile = copy.deepcopy(profile.get('planning'))
                    self.config = cfg
                    self.arm_configs[arm] = dict(cfg)
                    self.observation = None
                return dict(ok=True, **cfg)
            if action == 'start':
                previous_topics=tuple(self.inputs.topics)
                was_enabled=self.enabled
                self.inputs.start(args)
                if not was_enabled or previous_topics != tuple(self.inputs.topics):
                    self.observation = None
                if self.observation:
                    try:
                        same_source=self.inputs.binding()==self.observation['metadata']['calibration']
                    except (RuntimeError,KeyError):
                        same_source=False
                    if not same_source:self.observation=None
                self.enabled = True
                return self.info()
            if action in ('grab_to','grab_by'):
                if args.get('confirm_motion') is not True:
                    return dict(state='error',code='CONFIRMATION_REQUIRED',message='confirm_motion=true is required')
                from .execution import parameters
                from .transfer import parameters as transfer_parameters
                try:
                    options = parameters(self.config, args)
                    transfer = transfer_parameters(action,args)
                except (ValueError, OverflowError) as exc:
                    return dict(state='error',message=str(exc))
                if not self.observation:
                    return dict(state='error',code='OBSERVATION_REQUIRED',observation_required=True,
                        message='Run observe and wait for completed before transferring. The snapshot was consumed or its configuration/source changed.')
                if not self.planning_profile:
                    return dict(state='error',code='PLANNING_NOT_CONFIGURED',message='Driver hand-eye planning profile is missing')
                if self.gripper_manager is None:
                    return dict(state='error',code='GRIPPER_NOT_CONFIGURED',message='Driver gripper manager is missing')
                active = dict(action_id='vision_pick_and_drop_'+action+'_'+uuid4().hex,
                    action=action,transfer=transfer,config=dict(self.config),
                    owner='vision_pick_and_drop_'+uuid4().hex, cancel=threading.Event(),
                    stage='planning', session=None, command_id=None, hand_command_id=None,
                    observation=self.observation, position=[args.get('start_point_x'),args.get('start_point_y')],
                    profile={**copy.deepcopy(self.planning_profile),
                             **{f'{axis}_compensation_mm': self.config[f'{axis}_compensation_mm'] for axis in 'xyz'}}, options=options)
                self.active = active
                self.observation = None
                threading.Thread(target=self.run_transfer, args=(active,), daemon=True, name='fr3-transfer').start()
                return dict(state='running', action_id=active['action_id'])
            if action != 'observe': return None
            if args.get('confirm_motion') is not True:
                return dict(state='error', code='CONFIRMATION_REQUIRED', message='confirm_motion=true is required')
            if not self.enabled: return dict(state='error', message='Start the three observation inputs first')
            if not self.address or not self.serial: raise ValueError('Configure the selected arm and wrist camera before observation')
            target = joint_target(self.config['observation_joints_deg'])
            try:
                self.inputs.prepare()
                self.inputs.binding()
            except (RuntimeError, ValueError) as exc:
                return dict(state='error', code='CAMERA_NOT_READY', message=str(exc))
            active = dict(action_id='vision_pick_and_drop_observe_'+uuid4().hex, cancel=threading.Event(),
                          owner='vision_pick_and_drop_'+uuid4().hex, target=target, speed_deg_s=2., config=dict(self.config),
                          stage='checking', session=None, command_id=None)
            self.observation = None; self.active = active
            threading.Thread(target=self.run, args=(active,), daemon=True, name='fr3-observe').start()
            return dict(state='running', action_id=active['action_id'])

    def state(self, active):
        if active['cancel'].is_set(): raise RuntimeError('Observation cancelled')
        s = active['session'].snapshot()
        if (not s.get('connected') or (s.get('data_age_ms') or 0) > 200 or
                s.get('robot_mode') != 1 or not s.get('O_T_EE')):
            raise RuntimeError(s.get('error') or 'Fresh stationary Idle robot state is required')
        if max(map(abs, s['dq'])) > .01: raise RuntimeError('Arm is not stationary')
        pose_vector(s['O_T_EE'])
        from .hand_collision import frame
        frame(self.planning_profile or {}, s.get('F_T_EE'))
        return s

    def run(self, active):
        status, result = 'error', {}
        hand = None
        try:
            session = self.manager.acquire(self.address, active['owner']); active['session'] = session
            deadline = time.monotonic()+5
            while not session.snapshot().get('connected'):
                if active['cancel'].wait(.02): raise RuntimeError('Observation cancelled')
                if time.monotonic() >= deadline: raise RuntimeError('Robot state connection timed out')
            before = self.state(active)
            from .hand_collision import check_observation_path
            return_path = check_observation_path(before, active['target'], self.planning_profile or {})
            if self.gripper_manager is not None:
                hand = self.gripper_manager.acquire(self.address, active['owner'])
                deadline = time.monotonic() + 5
                while not hand.snapshot().get('connected'):
                    if active['cancel'].wait(.02):
                        raise RuntimeError('Observation cancelled')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Hand connection timed out')
                from .execution import PickExecution
                executor = PickExecution(active, session, hand, {'speed_mm_s': 20})
                state = executor.fresh(hand)
                if state['is_grasped']:
                    raise RuntimeError('Hand reports a held object; release it before observing')
                opening = min(state['max_width'], .08)
                if opening <= 0:
                    raise RuntimeError('Invalid calibrated hand opening')
                if abs(state['width'] - opening) > .001:
                    active['stage'] = 'observe_open'
                    command = hand.command(active['owner'], 'set_position',
                                           width_m=opening, speed_m_s=.02, timeout_s=15)
                    try:
                        executor.wait(hand, command, 17, 'hand_command_id')
                        if abs(executor.fresh(hand)['width'] - opening) > .001:
                            raise RuntimeError('Hand did not reach its maximum opening')
                    except Exception:
                        executor.stop_pending()
                        raise
            target = [math.radians(v) for v in active['target']]
            # At the already-selected observation pose, capture without sending a redundant motion.
            if max(abs(a-b) for a,b in zip(before['q'], target)) > math.radians(.2):
                active['stage'] = 'move_to_observation'
                if not before.get('control_ready'): raise RuntimeError(before.get('control_error') or 'Robot control is not ready')
                motion_args = dict(speed_rad_s=math.radians(active['speed_deg_s']),
                                   tolerance_rad=math.radians(.2), timeout_s=360)
                if return_path is None:
                    active['command_id'] = session.move(active['owner'], target, **motion_args)
                else:
                    active['command_id'] = session.move_path(active['owner'], return_path.tolist(), **motion_args)
                while not active['cancel'].wait(.05):
                    motion = session.motion(active['command_id'])
                    if motion['status'] in ('succeeded', 'failed', 'cancelled'):
                        if motion['status'] != 'succeeded': raise RuntimeError(motion.get('error') or motion['status'])
                        break
                if active['cancel'].is_set(): raise RuntimeError('Observation cancelled')
            result = self.capture(active)
            status = 'completed'
        except Exception as exc:
            if active['session'] and active['command_id']:
                try: active['session'].stop_motion(owner=active['owner'], command_id=active['command_id'])
                except Exception as stop_exc: log.warning('Observation stop failed: %s', stop_exc)
            status = 'cancelled' if active['cancel'].is_set() else 'error'
            result = dict(ok=False, message=str(exc), stage=active['stage'])
        finally:
            if hand is not None:
                try:
                    self.gripper_manager.release(self.address, active['owner'])
                except Exception as exc:
                    status = 'error'
                    result.update(ok=False, stop_error=str(exc))
                    with self.lock:
                        self.observation = None
            if active['session']:
                try: self.manager.release(self.address, active['owner'])
                except Exception as exc:
                    status = 'error'; result.update(ok=False, stop_error=str(exc))
                    with self.lock: self.observation = None
        self.complete(active, status, result)

    def capture(self, active):
        target = [math.radians(v) for v in active['target']]
        active['stage'] = 'settling'
        anchor = self.state(active)
        after = time.time()
        def check():
            current = self.state(active)
            if max(abs(a-b) for a,b in zip(current['q'], target)) > math.radians(.2):
                raise RuntimeError('Arm left the observation joint target')
            if max(abs(a-b) for a,b in zip(current['q'], anchor['q'])) > math.radians(.05):
                raise RuntimeError('Arm moved during observation')
            if np.linalg.norm(np.array(current['O_T_EE'])[12:15]-np.array(anchor['O_T_EE'])[12:15]) > .0003:
                raise RuntimeError('End effector moved during observation')
        check(); active['stage'] = 'observing'
        photo = self.inputs.snapshot(after, active['cancel'], check); check()
        oid = 'vision_pick_and_drop_observe_'+uuid4().hex
        result = dict(ok=True, media_type='photo', observation_id=oid,
            captured_at=photo['captured_at'], width=photo['width'], height=photo['height'],
            pose=pose_vector(anchor['O_T_EE']), objects=photo['objects'], count=len(photo['objects']),
            objects_timestamp=photo['objects_timestamp'], input_identity=photo['input_identity'],
            synchronization=photo['synchronization'])
        metadata = {k:v for k,v in photo.items() if k not in ('jpeg', 'depth_zlib', 'objects')}
        metadata.update(F_T_EE=anchor.get('F_T_EE'), O_T_EE=anchor['O_T_EE'], joint_degree=[math.degrees(v) for v in anchor['q']],
                        arm_endpoint=self.address, config=active['config'], observation_id=oid,
                        depth_encoding='zlib/uint16-le', depth_aligned_to='color',
                        pose_units=dict(position='m', angle='rad'))
        with self.lock:
            check(); self.observation = dict(result=copy.deepcopy(result), metadata=metadata,
                jpeg=photo['jpeg'], depth_zlib=photo['depth_zlib'])
        return result

    def complete(self, active, status, result):
        with self.lock:
            terminal = dict(state=status, action_id=active['action_id'], result=result,
                            observation_required=self.observation is None, callback='pending')
            self.last_result = terminal
        for attempt in range(3):
            try:
                self.callback(active['action_id'], status, {**result, 'observation_required':terminal['observation_required']})
                terminal['callback'] = 'accepted'; break
            except Exception as exc:
                terminal.update(callback='failed', callback_error=str(exc))
                if attempt < 2: time.sleep(.5)
        if terminal['callback'] == 'failed':
            log.warning('ACP completion delivery failed action_id=%s: %s',active['action_id'],terminal['callback_error'])
        with self.lock:
            terminal['action']=active.get('action','observe')
            self.active = None

    def run_transfer(self, active):
        from .execution import PickExecution
        from .planning import preview
        from .scene import estimate
        status, result, executor, hand = 'error', {}, None, None
        try:
            scene = estimate(active['observation'], active['position'], active['profile']['handeye'])
            plan = preview(active['observation'], active['position'], scene['diameter_mm'],
                           {**active['profile'], **scene}, scene)
            plan['scene'] = scene
            def check():
                if active['cancel'].is_set():
                    raise RuntimeError('Pick cancelled')
            check()
            from .transfer import plan_transfer
            transfer = plan_transfer(active['observation'],plan,active['profile']['handeye'],
                                     active['transfer'],check, active['profile'])
            check()
            active['session'] = self.manager.acquire(self.address, active['owner'])
            hand = self.gripper_manager.acquire(self.address, active['owner'])
            deadline = time.monotonic()+5
            while not (active['session'].snapshot().get('connected') and hand.snapshot().get('connected')):
                if active['cancel'].wait(.02):
                    raise RuntimeError('Pick cancelled')
                if time.monotonic() > deadline:
                    raise TimeoutError('Robot/hand connection timed out')
            from .hand_collision import frame
            frame(active['profile'], active['session'].snapshot().get('F_T_EE'))
            live_pose = np.asarray(active['session'].snapshot()['O_T_EE']).reshape(4,4,order='F')
            saved_pose = np.asarray(active['observation']['metadata']['O_T_EE']).reshape(4,4,order='F')
            pose_vector(live_pose.flatten(order='F').tolist())
            relative = live_pose[:3,:3].T @ saved_pose[:3,:3]
            angle = math.acos(np.clip((np.trace(relative)-1)/2,-1,1))
            if np.linalg.norm(live_pose[:3,3]-saved_pose[:3,3]) > .002 or angle > math.radians(1):
                raise RuntimeError('End-effector pose or tool frame changed since observation')
            executor = PickExecution(active, active['session'], hand, active['options'])
            executor.run(plan)
            result = dict(executor.transfer(transfer),observation_id=plan['observation_id'])
            if active['config']['observe_after_transfer']:
                active['target'] = joint_target(active['config']['observation_joints_deg'])
                result['observation'] = self.capture(active)
            status = 'completed'
        except Exception as exc:
            status = 'cancelled' if active['cancel'].is_set() else 'error'
            # A rejected target before session acquisition has not changed the scene.
            # Keep the photograph so the user can correct coordinates without recapturing.
            if status=='error' and active['session'] is None:
                with self.lock:self.observation=active['observation']
            result = dict(result, ok=False, message=str(exc), stage=active['stage'],
                          transfer_completed=active.get('transfer_completed',False),
                          return_completed=active.get('return_completed',False),
                          observe_after_transfer=active['config']['observe_after_transfer'],
                          observation=(dict(ok=False,message=str(exc)) if active.get('transfer_completed') and active['config']['observe_after_transfer']
                              else dict(skipped=True,reason='transfer_incomplete' if active['config']['observe_after_transfer'] else 'disabled')),
                          observation_id=active['observation'].get('result',{}).get('observation_id'),
                          last_completed_stage=active.get('last_completed_stage'), feedback_recoveries=0,
                          holding_object_possible=active.get('holding_object_possible',False),
                          recovery_required=active.get('holding_object_possible',False),
                          release_completed=active.get('release_completed',False),
                          rotation_deg=(active.get('transfer') or {}).get('rotation_deg',0),
                          rotation_completed=active.get('rotation_completed',False))
            result.pop('final_pose', None)
            if executor:
                try:
                    executor.stop_pending()
                except Exception as stop_exc:
                    status = 'error'
                    result.update(stop_error=str(stop_exc), recovery_required=True)
        finally:
            for manager, session in ((self.manager, active['session']), (self.gripper_manager, hand)):
                if session is not None:
                    try:
                        manager.release(self.address, active['owner'])
                    except Exception as exc:
                        status = 'error'
                        result.update(ok=False, release_error=str(exc), recovery_required=True)
                        with self.lock: self.observation = None
        self.complete(active, status, result)
