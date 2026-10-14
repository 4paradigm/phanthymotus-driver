"""User-intent control. Each request owns preparation and execution together."""
from copy import deepcopy
from contextlib import contextmanager
from dataclasses import asdict
import math
import re
import threading
import time

from common.vendor_runtime import action_schema, tool
from agilex.piper.device import PiperPlugin
from agilex.piper.runtime import PiperRuntime
from agilex.piper.intent_motion import DIRECTIONS, checked_state, end_pose, move_cartesian
from agilex.piper.intent_teaching import exit_teaching_and_hold

CONTROL = 'arm_control'
TASK_ACTIONS = {'move', 'open_gripper', 'close_gripper', 'set_gripper_opening', 'exit_teaching_and_hold'}


def public_error(error):
    message = str(error)
    if not re.search(r'[\u3400-\u9fff]', message):
        return message
    for fragment, english in (
            ('取消', 'The action was cancelled.'), ('超时', 'The action timed out; check the actual arm state.'),
            ('未确认到位', 'Gripper target was not reached; check for contact or obstruction.'),
            ('未更新', 'Device feedback is stale; check the CAN connection.'),
            ('正在执行', 'The arm or camera is busy; wait or use Stop.'),
            ('已停止', 'Arm control is stopped; start the card before retrying.'),
            ('连接', 'The CAN connection is unavailable.'),
            ('故障', 'The device reports a fault; check the robot.'),
            ('初始化', 'Arm preparation failed; check the actual robot state.'),
            ('夹爪', 'Gripper feedback or limits are invalid; check the gripper.'),
            ('限位', 'The requested motion exceeds effective limits.'),
            ('照片', 'Photo operation failed; check the image and camera settings.'),
            ('相机', 'Camera operation failed; check the camera settings.')):
        if fragment in message:
            return english
    return 'Device operation failed; check feedback and controller status.'


def english_values(value):
    if isinstance(value, str):
        return public_error(value)
    if isinstance(value, dict):
        return {key: english_values(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [english_values(item) for item in value]
    return value


class IntentControlPlugin(PiperPlugin):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.interface_language = 'en'
        self.task = {'state': 'idle', 'action': None}
        self.task_lock = threading.Lock()
        self.monitor_service = None
        self.gripper_torque = self.config.get('intent_control', {}).get('gripper_torque_nm', .5)
        if isinstance(self.gripper_torque, bool) or not isinstance(self.gripper_torque, (int, float)) or not math.isfinite(self.gripper_torque) or not .001 <= self.gripper_torque <= 1:
            raise ValueError('gripper_torque_nm must be between 0.001 and 1 N m.')

    def _width_bounds(self):
        # Tool metadata never waits for CAN initialization. Read its cache only
        # when the shared connection is immediately available.
        low, high = self.config.get('gripper_limits_mm', (0., 70.))
        if self.connection_lock.acquire(blocking=False):
            try:
                if self.driver is not None:
                    low, high = self.driver.gripper_range()
            finally:
                self.connection_lock.release()
        return low, high

    def get_tools(self):
        low, high = self._width_bounds()
        actions = {'select_action': ([], 'Choose an action'),
                   'move': (['direction', 'distance_mm', 'speed_percent'], 'Move the end effector; keep its orientation'),
                   'stop_motion': ([], 'Stop the current action; keep motor torque'),
                   'open_gripper': ([], 'Open the gripper'), 'close_gripper': ([], 'Close the gripper'),
                   'set_gripper_opening': (['width_mm'], 'Set the gripper opening'),
                   'exit_teaching_and_hold': ([], 'Exit teaching and hold the measured pose. Keep the arm stationary; '
                       'requires PiPER firmware >= 1.8.5 and all arm motors enabled. No reset or homing.'),
                   'start': ([], 'Ready the card; no motor commands'),
                   'stop': ([], 'Stop the card and cancel its action'), 'info': ([], 'Read feedback and the last task result')}
        schema = action_schema(actions, {
            'direction': {'type': 'string', 'enum': list(DIRECTIONS), 'default': 'up', 'title': 'Direction',
                          'description': 'Base frame: forward +X, left +Y, up +Z.',
                          'oneOf': [{'const': d, 'title': d.capitalize()} for d in DIRECTIONS]},
            'distance_mm': {'type': 'number', 'minimum': 1, 'maximum': 10, 'default': 5,
                            'title': 'Distance (mm)', 'description': '1–10 mm; default 5 mm.'},
            'speed_percent': {'type': 'integer', 'minimum': 1, 'maximum': 10, 'default': 5,
                              'title': 'Movement speed (%)', 'description': '1–10%; default 5%. Not a fixed mm/s.'},
            'width_mm': {'type': 'number', 'minimum': low, 'maximum': high, 'default': min(high, max(low, 20)),
                         'title': 'Gripper opening (mm)', 'description': f'{low:g}–{high:g} mm; effective limits.'}})
        labels = {'select_action': 'Choose an action', 'move': 'Move', 'stop_motion': 'Stop',
                  'open_gripper': 'Open gripper', 'close_gripper': 'Close gripper',
                  'set_gripper_opening': 'Set gripper opening', 'exit_teaching_and_hold': 'Exit teaching and hold',
                  'start': 'Start', 'stop': 'Stop card', 'info': 'Info'}
        schema['properties']['action'].update(title='Action', default='select_action',
            oneOf=[{'const': a, 'title': labels[a]} for a in actions])
        schema.update({'additionalProperties': False, 'x-manual-execution': True, 'x-resource': 'arm',
                       'x-is-dangerous': True,
                       'x-hooks': {h: {'action': 'stop_motion'} for h in ('on_interrupt_motion', 'on_interrupt_all')},
                       'x-completion': {'actions': sorted(TASK_ACTIONS), 'timeout': 45}})
        schema['x-action-params'].pop('select_action')
        namespace = self.topic.rsplit('/', 1)[0]
        arm = tool(CONTROL, 'actuator',
                   'Move the end effector in the fixed robot base frame, control the gripper, or stop. '
                   'Normal mode and motor preparation are automatic. Exit teaching and hold is an explicit action: '
                   'keep the arm stationary; no reset or homing. Check physical clearance before executing.',
                   schema, topic_out=[{'topic': namespace + '/state/joints', 'format': 'sensor/skeleton'},
                                      {'topic': namespace + '/state/parameters', 'format': 'data/json'}])
        arm['title'] = 'Arm control'
        photo_schema = action_schema({'start': ([], 'Ready without capture'), 'stop': ([], 'Cancel photo operation'),
            'info': ([], 'Photo status'), 'capture': (['capture_confirmed'], 'Capture one current RGB photo'),
            'import_photo': (['filename'], 'Import a staged JPEG or PNG')}, {
            'capture_confirmed': {'type': 'boolean', 'enum': [False, True], 'default': False,
                                 'title': 'Allow this capture', 'description': 'Confirm capture of the current camera view.'},
            'filename': {'type': 'string', 'title': 'Image filename', 'description': 'JPEG/PNG in the imports directory.'}})
        photo_schema.update({'additionalProperties': False, 'x-resource': ['arm', 'camera'],
                             'x-manual-execution': True, 'x-completion': {'actions': ['capture', 'import_photo'], 'timeout': 20}})
        photo_schema['properties']['action'].update(title='Action', default='info',
            oneOf=[{'const': a, 'title': a.replace('_', ' ').capitalize()}
                   for a in photo_schema['properties']['action']['enum']])
        photo = tool('piper_photo', 'actuator', 'Capture a current RGB photo or import an image; no arm motion.',
                     photo_schema, topic_out=[{'topic': self.topic, 'format': 'image/jpeg'}])
        photo['title'] = 'Camera and photos'
        return [arm, photo]

    def _args(self, name, action, args):
        definition = next(t for t in self.get_tools() if t['name'] == name)
        schema = definition['inputSchema']
        if action not in schema['properties']['action']['enum']:
            raise ValueError('This action has been replaced; refresh Arm control and choose a user action.')
        allowed = set(schema['x-action-params'].get(action, {}).get('params', []))
        supplied = set(args) - {'_tool_name', 'instance_id', 'action'}
        if supplied - allowed:
            raise ValueError('Unexpected parameters for this action: ' + ', '.join(sorted(supplied - allowed)))
        result = dict(args)
        for key in allowed:
            prop = schema['properties'][key]
            if result.get(key) == '' or key not in result:
                if 'default' in prop:
                    result[key] = prop['default']
                else:
                    raise ValueError(f'{key} is required.')
            value = result[key]
            if prop['type'] in {'number', 'integer'}:
                if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                        or prop['type'] == 'integer' and type(value) is not int
                        or not prop['minimum'] <= value <= prop['maximum']):
                    raise ValueError(f'{key} must be in [{prop["minimum"]}, {prop["maximum"]}].')
            elif prop['type'] == 'boolean' and type(value) is not bool:
                raise ValueError(f'{key} must be a boolean.')
            elif prop['type'] == 'string' and not isinstance(value, str):
                raise ValueError(f'{key} must be text.')
            if 'enum' in prop and value not in prop['enum']:
                raise ValueError(f'Invalid {key}.')
        return result

    def _phase(self, state, **values):
        with self.task_lock:
            self.task = {**self.task, 'state': state, **values}

    @contextmanager
    def operation(self, name='piper_arm', action=None):
        nested = (getattr(self.worker_context, 'reserved', None) == (name, action)
                  or getattr(self.worker_context, 'owned', None) == (name, action))
        with super().operation(name, action):
            if not nested and name == CONTROL and action in TASK_ACTIONS:
                # Publish preparation at reservation time, before an async
                # worker starts; Stop cannot mistake the previous task for it.
                self._phase('preparing', action=action, started_at=time.time(), result=None, error=None)
            yield

    def dispatch(self, action, args):
        name = args.get('_tool_name', CONTROL)
        args = self._args(name, action, args)
        previous = getattr(self.worker_context, 'operation_object', None)
        self.worker_context.operation_object = 'gripper' if 'gripper' in action else 'arm'
        try:
            if self.config.get('async_actions', True) and (action in TASK_ACTIONS or name == 'piper_photo' and action in {'capture', 'import_photo'}):
                result = self.run_async(name, action, args)
                if result.get('state') == 'running':
                    result['reminder'] = 'Task accepted. Check Info for actual completion or use Stop.'
                return english_values(result)
            return self.dispatch_sync(action, args)
        except (ValueError, RuntimeError, OSError) as exc:
            raise type(exc)(public_error(exc)) from exc
        finally:
            self.worker_context.operation_object = previous

    def dispatch_sync(self, action, args):
        name = args.get('_tool_name', CONTROL)
        if name == 'piper_photo':
            try:
                return english_values(PiperRuntime.dispatch_sync(self, action, args))
            except Exception as exc:
                raise RuntimeError(public_error(exc)) from exc
        if action == 'select_action':
            return {'executed': False, 'message': 'Choose an action first.'}
        if action == 'start':
            self.start(name)
            return {'state': 'ready', 'executed': False}
        if action in {'stop', 'stop_motion'}:
            if action == 'stop':
                self.request_stop(name)
            return self.stop_motion()
        if action == 'info':
            with self.task_lock:
                task = deepcopy(self.task)
            return {'state': task['state'], 'task': task, 'snapshot': self.read_monitor_snapshot()}
        if action not in TASK_ACTIONS:
            raise ValueError('Unsupported user action; refresh Arm control.')
        try:
            return self._execute_task(action, args)
        except BaseException:
            if self.cancel.is_set() and getattr(self.worker_context, 'reserved', None) == (CONTROL, action):
                self._phase('cancelled', error='The action was cancelled.')
            raise

    def _execute_task(self, action, args):
        with self.operation(CONTROL, action):
            self._phase('preparing', action=action, started_at=time.time(), result=None, error=None)
            try:
                driver = self.connected(cancel_event=self.cancel)
                if action == 'exit_teaching_and_hold':
                    result = exit_teaching_and_hold(driver, on_switch=lambda: self._phase('moving'))
                    with self.lifecycle:
                        driver._check_cancelled()
                        self._phase('completed', result=result)
                    return {**result, 'state': 'completed'}
                state = checked_state(driver)
                # Validate end feedback/actual gripper limits BEFORE enabling.
                if action == 'move':
                    end_pose(driver)
                else:
                    grip = driver.get_gripper()
                    if grip.error_code:
                        raise RuntimeError('The gripper reports a fault.')
                    low, high = driver.gripper_range()
                    width = high if action == 'open_gripper' else low if action == 'close_gripper' else args['width_mm']
                    if not low <= width <= high:
                        raise ValueError(f'Gripper opening must be in [{low}, {high}] mm.')
                driver._check_cancelled()
                if not all(state.enabled):
                    driver.prepare_control(execute=True, workspace_clear=True, allow_limit_adjustment=False,
                                           speed_pct=args.get('speed_percent', 5))
                checked_state(driver, ready=True)
                driver._check_cancelled()
                self._phase('moving')
                if action == 'move':
                    result = move_cartesian(driver, args['direction'], args['distance_mm'], args['speed_percent'])
                else:
                    grip = driver.get_gripper()
                    if not grip.enabled:
                        driver.move_gripper(grip.width_mm, self.gripper_torque)
                    driver._check_cancelled()
                    result = {'executed': True, 'gripper': asdict(driver.move_gripper(width, self.gripper_torque))}
                # Stop owns lifecycle as well: completion cannot overwrite a
                # cancellation published between the final check and result.
                with self.lifecycle:
                    driver._check_cancelled()
                    self._phase('completed', result=result)
                return {**result, 'state': 'completed'}
            except BaseException as exc:
                self._phase('cancelled' if self.cancel.is_set() else 'failed', error=public_error(exc))
                raise RuntimeError(public_error(exc)) from exc

    def stop_motion(self):
        with self.lifecycle:
            with self.task_lock:
                terminal = self.task['state'] in {'completed', 'cancelled', 'failed'}
            affected = self.gate.locked() and self.active_card == CONTROL and not terminal
            if affected:
                self.cancel.set()
                if self.driver is not None:
                    self.driver.cancel_motion()
                self._phase('stopping')
        return {'state': 'stopping' if affected else 'stopped', 'operation_stopping': affected,
                'motor_torque_released': False}

    def read_monitor_snapshot(self):
        result = super().read_monitor_snapshot()
        with self.task_lock:
            result['task'] = deepcopy(self.task)
        result['control_policy'] = 'automatic_preparation'
        result['device_name'] = 'PiPER ' + self.config.get('can_interface', 'can0')
        if self.connection_lock.acquire(blocking=False):
            try:
                if self.driver is not None and not self.closed:
                    try:
                        result['end_pose'] = asdict(end_pose(self.driver))
                    except Exception as exc:
                        result['reasons']['end_pose'] = public_error(exc)
            finally:
                self.connection_lock.release()
        return english_values(result)
