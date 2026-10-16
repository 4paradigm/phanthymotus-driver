"""User-intent control. Each request owns preparation and execution together."""
from copy import deepcopy
from contextlib import contextmanager
from dataclasses import asdict
import math
import json
import re
import threading
import time
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from agilex.piper.device import PiperPlugin
from agilex.piper.runtime import PiperRuntime
from agilex.piper.intent_motion import (DIRECTIONS, JOG_SPEED_PERCENT, SMALL_STEP_MM,
    DirectionUnavailable, checked_state, end_pose, move_cartesian)
from agilex.piper.intent_teaching import exit_teaching_and_hold

CONTROL = 'arm_control'
DIRECTION_ACTIONS = {f'move_{direction}': direction for direction in DIRECTIONS}
TASK_ACTIONS = {*DIRECTION_ACTIONS, 'open_gripper', 'close_gripper', 'set_gripper_opening', 'exit_teaching_and_hold'}


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
    task_actions = TASK_ACTIONS
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.interface_language = 'en'
        self.task = {'state': 'idle', 'action': None}
        self.task_lock = threading.Lock()
        self.audit_lock = threading.Lock()
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
                   **{action: ([], f'Move the gripper {direction} by 1 mm at 1% speed; keep its orientation. '
                       'Directions are fixed to the robot base. Small steps do not guarantee reachability.')
                      for action, direction in DIRECTION_ACTIONS.items()},
                   'stop_motion': ([], 'Stop the current action; keep motor torque'),
                   'open_gripper': ([], 'Open the gripper'), 'close_gripper': ([], 'Close the gripper'),
                   'set_gripper_opening': (['width_mm'], 'Set the gripper opening'),
                   'exit_teaching_and_hold': ([], 'Exit teaching and hold the measured pose. Keep the arm stationary; '
                       'requires PiPER firmware >= 1.8.5 and all arm motors enabled. No reset or homing.'),
                   'start': ([], 'Ready the card; no motor commands'),
                   'stop': ([], 'Stop the card and cancel its action'), 'info': ([], 'Read feedback and the last task result')}
        schema = action_schema(actions, {
            'width_mm': {'type': 'number', 'minimum': low, 'maximum': high, 'default': min(high, max(low, 20)),
                         'title': 'Gripper opening (mm)', 'description': f'{low:g}–{high:g} mm; effective limits.'}})
        labels = {'select_action': 'Choose an action',
                  **{action: ('Raise' if direction == 'up' else 'Lower' if direction == 'down'
                              else 'Move ' + direction) for action, direction in DIRECTION_ACTIONS.items()},
                  'stop_motion': 'Stop',
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
        outputs = [{'topic': namespace + '/state/joints', 'format': 'sensor/skeleton'},
                   {'topic': namespace + '/state/parameters', 'format': 'data/json'}]
        # The existing background camera owns acquisition and publication.
        # Declare its stream here so Monitor discovers it with only this card.
        if self.config.get('camera', {}).get('live', {}).get('enabled', False):
            outputs.append({'topic': namespace + '/camera/live', 'format': 'image/jpeg'})
        arm = tool(CONTROL, 'actuator',
                   'Choose a direction to move the gripper by one small step (1 mm, 1% speed), control the gripper, or stop. '
                   'Forward +X, backward -X, left +Y, right -Y, raise +Z, lower -Z in the fixed robot base frame; '
                   'directions do not follow the camera. Keep the workspace clear. '
                   'Gripper orientation stays fixed. Exit teaching and hold requires a stationary arm.',
                   schema, topic_out=outputs)
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
        result = dict(args)
        # Older Core submits hidden fields retained from previous actions.
        # Width has no meaning outside Set gripper opening: discard only this
        # declared field, preserving strict rejection of all unknown inputs.
        if name == CONTROL and action != 'set_gripper_opening':
            supplied.discard('width_mm')
            result.pop('width_mm', None)
        # Core's generic output-topic discovery adds an empty input_topic to
        # Info for input-less cards. Accept only that read-only framework value;
        # movement requests retain strict parameter rejection.
        if action == 'info' and args.get('input_topic') == '':
            supplied.discard('input_topic')
        if supplied - allowed:
            raise ValueError('Unexpected parameters for this action: ' + ', '.join(sorted(supplied - allowed)))
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

    def _audit(self, event, details=None):
        """Bounded per-device journal; only validated request/feedback fields.

        Initial/target writes are required before control. Terminal write failure
        must not hide a controller error or claim an unsuccessful task succeeded.
        """
        with self.task_lock:
            task = deepcopy(self.task)
        entry = {'time': time.time(), 'event': event, 'task': task}
        if details is not None:
            entry['details'] = details
        line = json.dumps(entry, ensure_ascii=True, allow_nan=False) + '\n'
        with self.audit_lock:
            self.data_dir.mkdir(parents=True, exist_ok=True)
            path = self.data_dir / 'arm-control-events.jsonl'
            if path.exists() and path.stat().st_size >= 2 * 1024 * 1024:
                path.replace(path.with_suffix('.jsonl.1'))
            with path.open('a', encoding='utf-8') as stream:
                stream.write(line)

    def _motion_report(self, details):
        with self.task_lock:
            self.task['diagnostics'] = deepcopy(details)
        self._audit('motion', details)

    def _terminal_audit(self):
        try:
            self._audit('terminal')
        except Exception:
            with self.task_lock:
                self.task['diagnostic_warning'] = 'The terminal event could not be saved; check diagnostic storage.'

    @contextmanager
    def operation(self, name='piper_arm', action=None):
        nested = (getattr(self.worker_context, 'reserved', None) == (name, action)
                  or getattr(self.worker_context, 'owned', None) == (name, action))
        with super().operation(name, action):
            if not nested and name == CONTROL and action in self.task_actions:
                # Publish preparation at reservation time, before an async
                # worker starts; Stop cannot mistake the previous task for it.
                self._phase('preparing', action=action, started_at=time.time(), result=None, error=None,
                            diagnostics=None, outcome=None, diagnostic_warning=None, task_id=uuid4().hex, request=None)
            yield

    def dispatch(self, action, args):
        name = args.get('_tool_name', CONTROL)
        args = self._args(name, action, args)
        previous = getattr(self.worker_context, 'operation_object', None)
        self.worker_context.operation_object = 'gripper' if 'gripper' in action else 'arm'
        try:
            if self.config.get('async_actions', True) and (action in self.task_actions or name == 'piper_photo' and action in {'capture', 'import_photo'}):
                result = self.run_async(name, action, args)
                if result.get('state') == 'running':
                    result['reminder'] = 'Task accepted, not completed. ACP will report the final outcome; Get status also shows progress.'
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
            return self.start(name) or {'state': 'ready', 'executed': False}
        if action in {'stop', 'stop_motion'}:
            if action == 'stop':
                self.request_stop(name)
            return self.stop_motion()
        if action == 'info':
            with self.task_lock:
                task = deepcopy(self.task)
            return {'state': task['state'], 'task': task, 'snapshot': self.read_monitor_snapshot()}
        if action not in self.task_actions:
            raise ValueError('Unsupported user action; refresh Arm control.')
        try:
            return self._execute_task(action, args)
        except BaseException:
            if self.cancel.is_set() and getattr(self.worker_context, 'reserved', None) == (CONTROL, action):
                self._phase('cancelled', error='The action was cancelled.')
            raise

    def start(self, name=None):
        """Core precedes each manual execution (including Stop) with Start.

        While this card owns a task, acknowledge that harmless lifecycle call
        without changing readiness/cancellation or the operation gate. The next
        action remains responsible for rejecting a new task or stopping it.
        """
        if name != CONTROL:
            return super().start(name)
        with self.lifecycle:
            if self.gate.locked() and self.active_card == CONTROL:
                stopping = self.closed or name in self.stopped_cards or self.cancel.is_set()
                return {'state': 'stopping' if stopping else 'busy', 'executed': False,
                        'operation_stopping': stopping,
                        'message': 'The current action is stopping; no restart was performed.' if stopping else
                            'An action is still running. Wait for completion or choose Stop; new actions are not queued.'}
            self.closed = False
            self.stopped_cards.discard(name)
            return {'state': 'ready', 'executed': False}

    def require_motion_permission(self):
        # This is the operator's deployment gate, independent of the removed
        # per-action request-control UI. Start never opens it.
        if not self.motion_enabled:
            raise RuntimeError('Motion is disabled by deployment configuration. '
                               'An operator must set PIPER_MOTION_ENABLED=1 before retrying.')

    def _execute_task(self, action, args):
        with self.operation(CONTROL, action):
            self._phase('preparing', action=action, started_at=time.time(), result=None, error=None,
                        diagnostics=None, outcome=None, diagnostic_warning=None,
                        request={'action': action, **{key: value for key, value in args.items() if key == 'width_mm'}})
            try:
                self._audit('request')
                self.require_motion_permission()
                driver = self.connected(cancel_event=self.cancel)
                observed = driver.get_state()
                self._motion_report({'stage': 'preflight', 'controller_status': observed.arm_status,
                    'control_mode': observed.control_mode, 'error_code': observed.error_code,
                    'joints_deg': [q if math.isfinite(q) else None for q in observed.joints_deg],
                    'joint_feedback_time': observed.feedback_time if math.isfinite(observed.feedback_time) else None})
                self.require_motion_permission()
                if action == 'exit_teaching_and_hold':
                    result = exit_teaching_and_hold(driver, on_switch=lambda: self._phase('moving'),
                                                    require_permission=self.require_motion_permission)
                    with self.lifecycle:
                        driver._check_cancelled()
                        self._phase('completed', result=result)
                    self._terminal_audit()
                    return {**result, 'state': 'completed'}
                state = checked_state(driver)
                # Validate end feedback/actual gripper limits BEFORE enabling.
                if action in DIRECTION_ACTIONS:
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
                if state.control_mode.startswith('STANDBY') or not all(state.enabled):
                    self.require_motion_permission()
                    driver.prepare_control(execute=True, workspace_clear=True, allow_limit_adjustment=False,
                                           speed_pct=JOG_SPEED_PERCENT)
                checked_state(driver, ready=True)
                driver._check_cancelled()
                self.require_motion_permission()
                self._phase('moving')
                if action in DIRECTION_ACTIONS:
                    result = move_cartesian(driver, DIRECTION_ACTIONS[action], SMALL_STEP_MM,
                                            JOG_SPEED_PERCENT, report=self._motion_report)
                else:
                    grip = driver.get_gripper()
                    if not grip.enabled:
                        self.require_motion_permission()
                        driver.move_gripper(grip.width_mm, self.gripper_torque)
                    driver._check_cancelled()
                    self.require_motion_permission()
                    result = {'executed': True, 'gripper': asdict(driver.move_gripper(width, self.gripper_torque))}
                # Stop owns lifecycle as well: completion cannot overwrite a
                # cancellation published between the final check and result.
                with self.lifecycle:
                    driver._check_cancelled()
                    self._phase('completed', result=result)
                self._terminal_audit()
                return {**result, 'state': 'completed'}
            except BaseException as exc:
                self._phase('cancelled' if self.cancel.is_set() else 'failed', error=public_error(exc),
                            outcome='cancelled' if self.cancel.is_set() else
                                'direction_unavailable' if isinstance(exc, DirectionUnavailable) else 'operation_failed')
                self._terminal_audit()
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
