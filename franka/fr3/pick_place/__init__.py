"""FR3 wrist observation and RealMan-compatible pick-and-place actions."""
import copy
import logging
from contextlib import contextmanager
import math
import threading
import time
from uuid import uuid4

import numpy as np
from common.vendor_runtime import action_schema, tool
from hardware import JOINT_LOWER, JOINT_UPPER, JOINT_MARGIN
from .execution import OBSERVATION_TOLERANCE_DEG, SPEED_CONFIG, PickExecution
from .inputs import ObservationInputs, resolve_topics
from franka.fr3.arm_ips import mapping

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

    def __init__(self, config, manager, camera, ros2, callback=None, inputs=None, gripper_manager=None,
                 controller_configuration=None, acquire_gripper=None):
        profile = config.get('vision_pick_and_drop', {})
        self.addresses = dict(config.get('motion_arm_ips', {}))
        self.arm = profile.get('default_arm', 'right')
        self.profiles = copy.deepcopy({self.arm: profile, **profile.get('arm_profiles', {})})
        self.arm_labels = {arm: arm
                           for arm, name in (('left', '左手'), ('right', '右手'))
                           if arm in self.profiles and self.addresses.get(arm)}
        self.address = self.addresses.get(self.arm)
        self.serial = profile.get('camera_serial', '')
        self.gripper_manager = gripper_manager
        self.acquire_gripper = acquire_gripper
        self.manager = manager
        self.planning_profile = profile.get('planning')
        self.inputs = inputs or ObservationInputs(ros2, camera, self.serial)
        self.config = dict(arm=self.arm_labels.get(self.arm, 'right'),
                           observation_joints_deg=profile.get('observation_joints_deg', ''),
                           x_compensation_mm=0, y_compensation_mm=0, z_compensation_mm=0,
                           observe_after_transfer=profile.get('observe_after_transfer', False),
                           force_n=profile.get('force_n'),
                           **{key: spec[2] for key, spec in SPEED_CONFIG.items()})
        self.arm_configs = {self.arm: dict(self.config)}
        if callback is None:
            from device import JointControlPlugin
            callback = lambda aid, status, result: JointControlPlugin._post_completion(self, aid, status, result)
        self.callback = callback
        self.lock = threading.RLock()
        self.active = self.last_result = self.observation = None
        self.enabled = False
        self.hand_initialized = False
        self.controller_configuration = controller_configuration

    def apply_arm_mapping(self, addresses):
        addresses = mapping(addresses)
        if addresses == self.addresses:
            return
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('Vision card is busy; retry the arm IP update')
        try:
            if addresses == self.addresses:
                return
            if self.active:
                raise RuntimeError('An observation or transfer is active; arm IP update rejected')
            result = (self.last_result or {}).get('result', {})
            if result.get('holding_object_possible') or result.get('recovery_required'):
                raise RuntimeError('Resolve the possibly held object before changing arm IPs')
            manager = self.gripper_manager
            if manager is not None and hasattr(manager, '_sessions'):
                with manager._lock:
                    sessions = [entry[0] for ip, entry in manager._sessions.items() if ip in self.addresses.values()]
                if any(session.snapshot().get('is_grasped') for session in sessions):
                    raise RuntimeError('The gripper is holding an object; arm IP update rejected')
            self.addresses = dict(addresses)
            self.hand_initialized = False
            self.arm_labels = {side: side
                               for side, name in (('left', '左手'), ('right', '右手')) if side in self.profiles}
            for side, profile in self.profiles.items():
                handeye = (profile.get('planning') or {}).get('handeye')
                if handeye:
                    handeye['robot_ip'] = addresses[side]
                if side in self.arm_configs:
                    self.arm_configs[side]['arm'] = self.arm_labels[side]
            self.address = addresses[self.arm]
            self.config['arm'] = self.arm_labels[self.arm]
            self.planning_profile = copy.deepcopy(self.profiles[self.arm].get('planning'))
            self.observation = None
        finally:
            self.lock.release()

    def get_tools(self):
        schema = action_schema({
            'observe': (['confirm_motion'], 'Move to the configured observation pose, settle and synchronously return one RGB-D/VOP observation. Uses the configured arm speed and opens the unloaded hand.'),
            'grab_to': (['start_point_x','start_point_y','target_point_x','target_point_y','rotation_deg','confirm_motion'], 'Pick and place using coordinates from the same image (X right, Y down). Observes automatically if needed, returns an action ID, then releases and returns to the observation pose.'),
            'grab_by': (['start_point_x','start_point_y','delta_x','delta_y','rotation_deg','confirm_motion'], 'Pick and translate in table millimetres along the image axes (X right, Y down). Observes automatically if needed, returns an action ID, then releases and returns to the observation pose.'),
            'cancel': ([], 'Cancel observation and stop only this card\'s active motion. Never opens the gripper.'),
        }, {'confirm_motion': {'type': 'boolean', 'const': True},
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
        schema.update({'x-completion': {'actions': ['grab_to', 'grab_by'], 'timeout': 1200}, 'x-resource': 'arm',
                       'x-is-dangerous': True,
                       'x-hooks': {'on_interrupt_motion': {'action': 'cancel'},
                                   'on_interrupt_all': {'action': 'cancel'}}})
        definition = tool(self.PREFIX, 'actuator',
            'Pick and place with the FR3 wrist camera. Connect RGB, depth and object detections. '
            'The hand homes before each requested or automatic pre-grasp observation. Remove any held object before starting.',
            schema, topic_in=[dict(format=fmt, desc=desc, **({'topic': self.inputs.topics[i]} if self.inputs.topics else {}))
                              for i, fmt, desc in ((1, 'image/depth-zlib', 'Depth image'),
                                  (0, 'image/jpeg', 'RGB image'), (2, 'data/json', 'VOP objects'))])
        definition['configSchema'] = {'type': 'object', 'additionalProperties': False, 'required': ['force_n'], 'properties': {
            'arm': {'type': 'string', 'title': 'Arm', 'scope': 'instance', 'enum': list(self.arm_labels.values()),
                'enumNames': ['Left hand' if side == 'left' else 'Right hand' for side in self.arm_labels],
                'default': self.config['arm'],
                'description': 'Select the left or right arm.'},
            **{name: {'type': 'number', 'minimum': low,
                      **({'maximum': high} if math.isfinite(high) else {}),
                      'default': default, 'title': title, 'scope': 'instance'}
               for name, (low, high, default, title) in SPEED_CONFIG.items()},
            'force_n': {'type': 'number', 'minimum': 30, 'maximum': 70, 'scope': 'instance',
                'description': 'Set the grasp force in newtons before running a transfer.'},
            'observe_after_transfer': {'type': 'boolean', 'default': False, 'scope': 'instance',
                'description': 'Capture a fresh observation after returning to the observation pose.'}}}
        return [definition]

    def start(self):
        pass  # Inputs start only after the canvas supplies the three topics.

    def info(self):
        with self.lock:
            return dict(state='running' if self.active else 'ready' if self.enabled else 'idle',
                        arm=self.arm, camera_serial=self.serial, config=dict(self.config),
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
        with self.lock:
            self.enabled = False
            self.observation = None
            self.hand_initialized = False
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
                updates = {k: v for k, v in args.items()
                           if k not in ('_tool_name', 'instance_id', 'username', 'password')}
                # Ignore the removed persistent hand-speed field from older saved canvases.
                updates.pop('speed_percent', None)
                for key in ('depth_compensation_mm', 'observation_joints_deg',
                            'left_observation_joints_deg', 'right_observation_joints_deg',
                            *(f'{axis}_compensation_mm' for axis in 'xyz')):
                    updates.pop(key, None)
                if updates.keys()-self.config.keys():
                    return dict(ok=False,code='INVALID_CONFIG',message='Unknown vision_pick_and_drop configuration field')
                try:
                    label = updates.get('arm', self.config['arm'])
                    arm = next((a for a, value in self.arm_labels.items() if value == label), None)
                    # Migrate old saved labels by physical side, never by their stale IP.
                    if arm is None and isinstance(label, str):
                        arm = next((side for side, suffix in (('left', '（左手）'), ('right', '（右手）'))
                                    if label.endswith(suffix) and side in self.arm_labels), None)
                    if arm is None:
                        raise ValueError('Select left or right arm')
                    updates['arm'] = label = self.arm_labels[arm]
                    profile = self.profiles[arm]
                    defaults = dict(arm=label, observation_joints_deg=profile.get('observation_joints_deg', ''),
                                    x_compensation_mm=0, y_compensation_mm=0, z_compensation_mm=0,
                                    observe_after_transfer=profile.get('observe_after_transfer', False),
                                    force_n=profile.get('force_n', self.config['force_n']),
                                    **{key: self.config[key] for key in SPEED_CONFIG})
                    saved = self.arm_configs.get(arm, defaults)
                    cfg = {**saved, **updates}
                    joint_target(cfg['observation_joints_deg'])
                    handeye = (profile.get('planning') or {}).get('handeye')
                    if handeye and (handeye.get('robot_ip') != self.addresses[arm] or
                                    handeye.get('camera_serial') != profile.get('camera_serial')):
                        raise ValueError('Selected arm hand-eye identity does not match its robot and camera')
                    if type(cfg['observe_after_transfer']) is not bool:
                        raise ValueError('observe_after_transfer must be a boolean')
                    for name, low, high in (('force_n', 30, 70), *((name, spec[0], spec[1]) for name, spec in SPEED_CONFIG.items()), *((f'{axis}_compensation_mm', -50, 50) for axis in 'xyz')):
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
                        self.hand_initialized = False
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
                # Old canvases may still send the retired IP topic with camera inputs.
                topics = args.get('input_topics') or []
                if isinstance(topics, list):
                    topics = [topic for topic in topics if not (
                        isinstance(topic, str) and topic.endswith('/arm_ip_sync'))]
                camera_topics = resolve_topics(dict(input_topics=topics))
                config_fields = ('arm', *SPEED_CONFIG, 'force_n', 'observe_after_transfer')
                updates = {key: args[key] for key in config_fields if key in args}
                if updates:
                    configured = self.dispatch('config', updates)
                    if not configured['ok']:
                        return dict(configured, state='error')
                try:
                    self.inputs.start(dict(args, input_topics=camera_topics))
                except Exception:
                    self.enabled = False
                    self.observation = None
                    raise
                if not was_enabled or previous_topics != tuple(self.inputs.topics):
                    self.observation = None
                    self.hand_initialized = False
                if self.observation:
                    try:
                        same_source=self.inputs.binding()==self.observation['metadata']['calibration']
                    except (RuntimeError,KeyError):
                        same_source=False
                    if not same_source:self.observation=None
                self.enabled = bool(camera_topics)
                return self.info()
            if action in ('observe', 'grab_to', 'grab_by'):
                if args.get('confirm_motion') is not True:
                    return dict(state='error', code='CONFIRMATION_REQUIRED', message='confirm_motion=true is required')
                connection = None
                if self.controller_configuration is not None:
                    try:
                        host, addresses, username, password = self.controller_configuration.snapshot()
                        self.apply_arm_mapping(addresses)
                        connection = host, username, password
                    except RuntimeError as error:
                        return dict(state='error', code='CONTROLLER_CONFIGURATION_NOT_READY', message=str(error))
            if action in ('grab_to','grab_by'):
                from .execution import parameters
                from .transfer import parameters as transfer_parameters
                try:
                    options = parameters(self.config)
                    transfer = transfer_parameters(action,args)
                except (ValueError, OverflowError) as exc:
                    return dict(state='error',message=str(exc))
                if not self.planning_profile:
                    return dict(state='error',code='PLANNING_NOT_CONFIGURED',message='Driver hand-eye planning profile is missing')
                if self.planning_profile.get('handeye', {}).get('accepted') is not True:
                    return dict(state='error', code='CALIBRATION_NOT_ACCEPTED',
                                message='The selected arm calibration must be accepted before grasping')
                if not self.observation and not self.enabled:
                    return dict(state='error', message='Start the three observation inputs first')
                if self.gripper_manager is None:
                    return dict(state='error',code='GRIPPER_NOT_CONFIGURED',message='Driver gripper manager is missing')
                active = dict(action_id='vision_pick_and_drop_'+action+'_'+uuid4().hex,
                    action=action,transfer=transfer,config=dict(self.config), controller_connection=connection,
                    owner='vision_pick_and_drop_'+uuid4().hex, cancel=threading.Event(),
                    stage='planning', target=joint_target(self.config['observation_joints_deg']),
                    session=None, command_id=None, hand_command_id=None,
                    observation=self.observation, position=[args.get('start_point_x'),args.get('start_point_y')],
                    profile={**copy.deepcopy(self.planning_profile),
                             **{f'{axis}_compensation_mm': self.config[f'{axis}_compensation_mm'] for axis in 'xyz'}}, options=options)
                self.active = active
                self.observation = None
                threading.Thread(target=self.run_transfer, args=(active,), daemon=True, name='fr3-transfer').start()
                return dict(state='running', action_id=active['action_id'])
            if action != 'observe': return None
            if not self.enabled: return dict(state='error', message='Start the three observation inputs first')
            if not self.address or not self.serial: raise ValueError('Configure the selected arm and wrist camera before observation')
            target = joint_target(self.config['observation_joints_deg'])
            try:
                self.inputs.prepare()
                self.inputs.binding()
            except (RuntimeError, ValueError, TimeoutError) as exc:
                return dict(state='error', code='CAMERA_NOT_READY', message=str(exc))
            active = dict(action_id='vision_pick_and_drop_observe_'+uuid4().hex, cancel=threading.Event(),
                          owner='vision_pick_and_drop_'+uuid4().hex, target=target, config=dict(self.config),
                          stage='checking', session=None, command_id=None, controller_connection=connection)
            self.observation = None; self.active = active
        status, result = self.run_observation(active)
        if status == 'completed':
            result = {'objects': [
                {'name': obj['name'], 'position': [round(value, 3) for value in obj['position']],
                 'confidence': obj['confidence']}
                for obj in result['objects']
            ]}
        with self.lock:
            terminal = dict(state=status, action_id=active['action_id'], result=result,
                            observation_required=self.observation is None, action='observe')
            self.last_result = terminal
            self.active = None
        return terminal

    def state(self, active, *, settling=False, sample=None):
        if active['cancel'].is_set(): raise RuntimeError('Observation cancelled')
        s = active['session'].snapshot() if sample is None else sample
        if (not s.get('connected') or (s.get('data_age_ms') or 0) > 200 or
                s.get('robot_mode') not in (1, 2) or not s.get('control_ready') or not s.get('O_T_EE')):
            if settling and not s.get('error') and s.get('robot_mode') in (1, 2):
                return None
            raise RuntimeError(s.get('error') or f"Fresh control-ready robot state required (mode={s.get('robot_mode')}, age_ms={s.get('data_age_ms')})")
        if max(map(abs, s['dq'])) > .01:
            if settling:
                return None
            raise RuntimeError('Arm is not stationary')
        pose_vector(s['O_T_EE'])
        from .hand_collision import frame
        frame(self.planning_profile or {}, s.get('F_T_EE'))
        return s

    @contextmanager
    def controller(self, active):
        from .controller_session import ControllerSession
        if active.get('controller_connection') is None:
            raise RuntimeError('PNP controller configuration is not ready')
        endpoint, username, password = active['controller_connection']
        owner = active['owner']
        local = self.manager.acquire(self.address, owner, exclusive=True)
        try:
            deadline = time.monotonic() + 8
            while not local.snapshot().get('control_ready'):
                if active['cancel'].wait(.025):
                    raise RuntimeError('Pick cancelled')
                if time.monotonic() >= deadline:
                    raise RuntimeError('Orin native Franka control connection is not ready')
            with ControllerSession(endpoint, username, password, active['cancel'],
                                   local=local).start(self.address) as controller:
                yield controller
        finally:
            self.manager.release(self.address, owner)

    def observe_on_controller(self, active, arm, hand):
        from .remote_execution import RemoteExecution
        active['session'] = arm
        before = self.state(active)
        target = [math.radians(v) for v in active['target']]
        if max(abs(a-b) for a,b in zip(before['q'], target)) > math.radians(OBSERVATION_TOLERANCE_DEG):
            options = dict(active['config'])
            executor = RemoteExecution(active, arm, hand, options, before, self.planning_profile or {})
            if executor.fresh(hand)['is_grasped']:
                raise RuntimeError('Hand reports a held object; release it before moving to the observation pose')
            step = executor.prepare('move_to_observation', target, joint=True)
            try:
                executor.open_hand('observe_open')
                executor.move_remote(step)
            except Exception:
                executor.stop_pending()
                raise
        return self.capture(active)

    def acquire_hand(self, active):
        if self.gripper_manager is None:
            raise RuntimeError('Driver gripper manager is missing')
        owner = active['owner']
        hand = (self.acquire_gripper(self.address, owner) if self.acquire_gripper is not None
                else self.gripper_manager.acquire(self.address, owner, exclusive=True))
        try:
            deadline = time.monotonic() + 5
            while not hand.snapshot().get('connected'):
                if active['cancel'].wait(.025):
                    raise RuntimeError('Visual action cancelled')
                if time.monotonic() >= deadline:
                    raise TimeoutError('Hand connection timed out')
            before_observation = active.get('observation') is None
            if before_observation or not self.hand_initialized:
                self.initialize_hand(active, hand, force=before_observation)
                with self.lock:
                    if active['cancel'].is_set():
                        raise RuntimeError('Visual action cancelled')
                    self.hand_initialized = True
            return hand
        except Exception:
            self.gripper_manager.release(self.address, active['owner'])
            raise

    def initialize_hand(self, active, hand, *, force=False):
        executor = PickExecution(active, None, hand, active['config'])
        sample = executor.fresh(hand)
        if not force and not sample['is_grasped'] and sample['max_width'] > 0:
            return
        stage = active['stage']
        active['stage'] = 'hand_homing'
        previous = (self.last_result or {}).get('result', {})
        if previous.get('holding_object_possible') or previous.get('recovery_required'):
            active.update(holding_object_possible=previous.get('holding_object_possible', False),
                          recovery_required=True)
            raise RuntimeError('Resolve the previous failed transfer before homing the hand')
        requested_at = time.monotonic_ns()
        command = hand.command(active['owner'], 'homing', timeout_s=30)
        try:
            executor.wait(hand, command, 32, 'hand_command_id')
        except Exception:
            if active.get('hand_command_id'):
                try:
                    executor.stop_pending()
                except Exception:
                    active['recovery_required'] = True
                    raise
            raise

        deadline = time.monotonic() + 1
        while True:
            executor.check()
            sample = hand.snapshot()
            if (sample.get('connected') and sample.get('control_ready')
                    and sample.get('sdk_received_ns', 0) > requested_at
                    and 0 <= sample['data_age_ms'] <= 220
                    and not sample['is_grasped'] and sample['max_width'] > 0):
                active['stage'] = stage
                return
            if time.monotonic() >= deadline:
                raise RuntimeError('Hand homing did not confirm a calibrated, released hand')
            active['cancel'].wait(.02)

    def run_observation(self, active):
        hand = None
        try:
            hand = self.acquire_hand(active)
            with self.controller(active) as arm:
                return 'completed', self.observe_on_controller(active, arm, hand)
        except Exception as exc:
            result = dict(ok=False, message=str(exc), stage=active['stage'])
            for key in ('holding_object_possible', 'recovery_required'):
                if active.get(key):
                    result[key] = True
            return 'cancelled' if active['cancel'].is_set() else 'error', result
        finally:
            active.update(session=None, command_id=None)
            if hand is not None:
                self.gripper_manager.release(self.address, active['owner'])

    def capture(self, active):
        target = [math.radians(v) for v in active['target']]
        active['stage'] = 'settling'
        deadline = time.monotonic() + 5
        while True:
            sample = active['session'].snapshot()
            anchor = self.state(active, settling=True, sample=sample)
            errors = [abs(math.degrees(a - b)) for a, b in zip(sample['q'], target)]
            worst = max(range(len(errors)), key=errors.__getitem__)
            if anchor is not None and errors[worst] <= OBSERVATION_TOLERANCE_DEG:
                break
            if time.monotonic() >= deadline:
                velocity = math.degrees(max(map(abs, sample['dq'])))
                raise RuntimeError(
                    f'Observation did not settle: joint {worst + 1} error={errors[worst]:.3f} deg '
                    f'(target={math.degrees(target[worst]):.3f}, actual={math.degrees(sample["q"][worst]):.3f}, '
                    f'limit={OBSERVATION_TOLERANCE_DEG:.3f}); max speed={velocity:.3f} deg/s (limit=0.573); '
                    f'mode={sample.get("robot_mode")}, age_ms={sample.get("data_age_ms")}'
                )
            if active['cancel'].wait(.025):
                raise RuntimeError('Observation cancelled')
        after = time.time()
        def check():
            current = self.state(active)
            if max(abs(a-b) for a,b in zip(current['q'], target)) > math.radians(OBSERVATION_TOLERANCE_DEG):
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
        from .remote_execution import GraspMiss, RemoteExecution
        from .hand_collision import TableClearanceError
        from .planning import grasp_targets
        from .scene import estimate
        from .transfer import destination, moveit_targets
        status, result, executor, hand = 'error', {}, None, None
        try:
            started = time.monotonic()
            hand = self.acquire_hand(active)
            hand_ready = time.monotonic()
            with self.controller(active) as arm:
                controller_ready = time.monotonic()
                active['session'] = arm
                if not active['observation']:
                    active['stage'] = 'auto_observe'
                    self.inputs.prepare()
                    self.inputs.binding()
                    self.observe_on_controller(active, arm, hand)
                    with self.lock:
                        active['observation'], self.observation = self.observation, None
                active['stage'] = 'planning'
                photo, profile = active['observation'], active['profile']
                scene = estimate(photo, active['position'], profile['handeye'])
                candidates = grasp_targets(photo, scene, profile)
                live = self.state(active)
                live_pose = np.asarray(live['O_T_EE']).reshape(4,4,order='F')
                saved_pose = np.asarray(photo['metadata']['O_T_EE']).reshape(4,4,order='F')
                relative = live_pose[:3,:3].T @ saved_pose[:3,:3]
                angle = math.acos(np.clip((np.trace(relative)-1)/2,-1,1))
                if np.linalg.norm(live_pose[:3,3]-saved_pose[:3,3]) > .002 or angle > math.radians(1):
                    raise RuntimeError('End-effector pose changed since observation')
                executor = RemoteExecution(active, arm, hand, active['options'], live, profile, scene)
                if executor.fresh(hand)['is_grasped']:
                    raise RuntimeError('Hand already reports a held object')
                opening = min(executor.fresh(hand)['max_width'], .08)
                errors = []
                deadline = time.monotonic() + 120
                print(f'[fr3 planning start] action={active["action_id"]} '
                      f'hand_s={hand_ready-started:.3f} controller_s={controller_ready-hand_ready:.3f} '
                      f'observation_and_target_s={time.monotonic()-controller_ready:.3f}', flush=True)
                executor.planning_deadline = deadline
                delta = destination(photo, scene, profile['handeye'], active['transfer'])
                ranked = []
                for pick in candidates:
                    executor.check()
                    if time.monotonic() >= deadline:
                        break
                    executor.q = list(live['q'])
                    try:
                        if not 0 < pick['grasp_width_mm'] / 1000 < opening:
                            raise ValueError('Target does not fit the calibrated opening')
                        targets = moveit_targets(photo, pick, scene, profile['handeye'], active['transfer'], delta=delta)
                        descents = []
                        for retreat_mm in range(6):
                            executor.check()
                            try:
                                candidate, goals = pick, targets
                                if retreat_mm:
                                    candidate = grasp_targets(photo, scene, profile, extra_retreat_mm=retreat_mm,
                                                              only_yaw=pick['yaw_deg'])[0]
                                    goals = moveit_targets(photo, candidate, scene, profile['handeye'], active['transfer'], delta=delta)
                                width = candidate['grasp_width_mm'] / 1000
                                if not 0 < width < opening:
                                    raise ValueError('Target does not fit the calibrated opening')
                                executor.check_descent_target(goals['descent'], 'descend')
                            except (RuntimeError, ValueError) as exc:
                                errors.append(f'{pick["yaw_deg"]:.1f} deg (+{retreat_mm} mm): {exc}')
                                continue
                            descents.append((retreat_mm, candidate, goals, width))
                        if descents:
                            requested = time.monotonic()
                            approaches = executor.approach_candidates(targets['approach'])
                            print(f'[fr3 approach planning] action={active["action_id"]} '
                                  f'yaw_deg={pick["yaw_deg"]:.1f} rpc_s={time.monotonic()-requested:.3f}',
                                  flush=True)
                            ranked.extend((travel, pick, targets, plan, descents)
                                          for travel, plan in approaches)
                    except (RuntimeError, ValueError, TimeoutError) as exc:
                        executor.check()
                        errors.append(f'{pick["yaw_deg"]:.1f} deg approach: {exc}')

                for _, pick, targets, plan, descents in sorted(ranked, key=lambda item: item[0]):
                    executor.check()
                    executor.q = list(live['q'])
                    try:
                        approach = executor.prepare_trajectory('approach', targets['approach'], plan,
                                                               False, False)
                    except (RuntimeError, ValueError) as exc:
                        errors.append(f'{pick["yaw_deg"]:.1f} deg approach check: {exc}')
                        continue
                    approach_end = list(executor.q)
                    descent = None
                    for retreat_mm, candidate, goals, width in descents:
                        executor.check()
                        executor.q = list(approach_end)
                        try:
                            descent = executor.prepare('descend', [goals['descent']], cartesian=True)
                        except (RuntimeError, ValueError) as exc:
                            errors.append(f'{pick["yaw_deg"]:.1f} deg (+{retreat_mm} mm): {exc}')
                            if not retreat_mm and not isinstance(exc, TableClearanceError):
                                break
                        else:
                            break
                    if descent is None:
                        continue
                    try:
                        lift = executor.prepare('lift', [goals['lift']], cartesian=True)
                        carry = executor.prepare('move_to_place', goals['carry'][:-1], cartesian=True)
                        land = goals['carry'][-1]
                        normal = np.asarray(scene['table_normal_base'])
                        carry_end = list(executor.q)
                        for place_retreat_mm in range(6):
                            executor.q = list(carry_end)
                            landing = land.copy()
                            landing[:3, 3] += normal * (place_retreat_mm / 1000)
                            try:
                                executor.check_descent_target(landing, 'place_descend')
                                place_descent = executor.prepare('place_descend', [landing], cartesian=True)
                                break
                            except TableClearanceError:
                                if place_retreat_mm == 5:
                                    raise
                        release_lift = goals['place_lift'].copy()
                        release_lift[:3, 3] += normal * (place_retreat_mm / 1000)
                        place_lift = executor.prepare('place_lift', [release_lift], cartesian=True)
                        home = executor.prepare('return_to_observation',
                                                list(map(math.radians, active['target'])), joint=True)
                    except (RuntimeError, ValueError) as exc:
                        errors.append(f'{pick["yaw_deg"]:.1f} deg after descend: {exc}')
                        continue
                    executor.goals['approach'] = (targets['approach'], {'use_ik': True})
                    pick, targets = candidate, goals
                    break
                else:
                    reason = 'MoveIt candidate planning deadline' if time.monotonic() >= deadline else 'No complete MoveIt plan'
                    raise RuntimeError(reason + ': ' + '; '.join(errors))
                if time.monotonic() >= deadline:
                    raise RuntimeError('MoveIt candidate planning deadline: ' + '; '.join(errors))
                if retreat_mm:
                    print(f"[fr3 descent retreat] action={active['action_id']} retreat_mm={retreat_mm} "
                          f"minimum_clearance_bound_mm={descent[1]['descent_gap_bound_mm']:.3f}", flush=True)
                executor.planning_deadline = None
                if place_retreat_mm:
                    print(f'[fr3 place retreat] action={active["action_id"]} retreat_mm={place_retreat_mm}',
                          flush=True)
                active['grasp_retreat_mm'] = retreat_mm
                executor.open_hand('pick_open')
                executor.move_remote(approach)
                executor.move_remote(descent)
                try:
                    executor.grasp(width)
                except GraspMiss as miss:
                    active['stage'] = 'miss_recovery_planning'
                    executor.q = list(executor.idle()['q'])
                    retreat = executor.prepare('miss_retreat', [targets['approach']], cartesian=True)
                    home = executor.prepare('return_to_observation',
                                            list(map(math.radians, active['target'])), joint=True)
                    if executor.fresh(hand)['is_grasped']:
                        raise RuntimeError('Hand reports a held object; recovery withheld')
                    executor.open_hand('miss_open')
                    active['holding_object_possible'] = False
                    active['recovery_required'] = True
                    executor.move_remote(retreat)
                    executor.move_remote(home)
                    active.update(return_completed=True, recovery_required=False)
                    try:
                        result['observation'] = self.capture(active)
                    except Exception as exc:
                        result['observation'] = dict(ok=False, message=str(exc))
                        raise
                    active['stage'] = 'pick_close'
                    raise RuntimeError('Grasp missed; hand opened and a new observation is available') from miss
                executor.move_remote(lift)
                if not executor.fresh(hand)['is_grasped']:
                    raise RuntimeError('Grasp feedback lost during lift')
                executor.move_remote(carry)
                executor.move_remote(place_descent)
                executor.open_hand('place_open')
                active.update(holding_object_possible=False, release_completed=True, transfer_completed=True)
                executor.move_remote(place_lift)
                executor.move_remote(home)
                active['return_completed'] = True
                rotation = active['transfer']['rotation_deg']
                result = dict(ok=True, stage='completed', transfer_completed=True, return_completed=True,
                              release_completed=True, holding_object_possible=False, recovery_required=False,
                              grasp_retreat_mm=retreat_mm,
                              observation_id=pick['observation_id'], rotation_deg=rotation,
                              rotation_completed=bool(rotation), grasp_checked=True,
                              last_completed_stage='return_to_observation', arm_path_commands=executor.arm_commands,
                              feedback_recoveries=active.get('feedback_recoveries', 0),
                              observe_after_transfer=active['config']['observe_after_transfer'],
                              observation=dict(skipped=True, reason='disabled'),
                              final_pose=pose_vector(self.state(active)['O_T_EE']))
                if active['config']['observe_after_transfer']:
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
                          observation_id=(active['observation'] or {}).get('result',{}).get('observation_id'),
                          last_completed_stage=active.get('last_completed_stage'),
                          feedback_recoveries=active.get('feedback_recoveries', 0),
                          holding_object_possible=active.get('holding_object_possible',False),
                          recovery_required=(active.get('holding_object_possible', False)
                                             or active.get('recovery_required', False)),
                          release_completed=active.get('release_completed',False),
                          rotation_deg=(active.get('transfer') or {}).get('rotation_deg',0),
                          rotation_completed=active.get('rotation_completed',False))
            if active.get('transfer_completed') and active['config']['observe_after_transfer']:
                result['observation'] = dict(ok=False, message=str(exc))
            else:
                result.setdefault('observation', dict(skipped=True,
                    reason='transfer_incomplete' if active['config']['observe_after_transfer'] else 'disabled'))
            result.pop('final_pose', None)
            if executor:
                try:
                    executor.stop_pending()
                except Exception as stop_exc:
                    status = 'error'
                    result.update(stop_error=str(stop_exc), recovery_required=True)
        finally:
            if hand is not None:
                try:
                    self.gripper_manager.release(self.address, active['owner'])
                except Exception as exc:
                    status = 'error'
                    result.update(ok=False, release_error=str(exc), recovery_required=True)
                    with self.lock:
                        self.observation = None
        self.complete(active, status, result)
