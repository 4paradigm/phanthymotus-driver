"""One user card, modular arm/gripper operations on one shared runtime.

RealMan's shared-client / serialized submission / ACP completion architecture
is retained through IntentControlPlugin; hardware commands remain PiPER SDK.
"""
from copy import deepcopy
from dataclasses import asdict
import time

from common.vendor_runtime import action_schema
from agilex.piper.intent_control import CONTROL, IntentControlPlugin, public_error
from agilex.piper.intent_motion import checked_state, end_pose, JOG_SPEED_PERCENT
from agilex.piper.intent_teaching import exit_teaching_and_hold
from agilex.piper.path_capability import path_capability, adapter_serial
from agilex.piper.preset import read_preset, write_preset, capture_position
from agilex.piper.user_motion import vector, target_pose, joint_target, execute_target, tcp_transform, Transform


USER_TASKS = {'move', 'pose', 'set_joints', 'set_position', 'reset', 'open_gripper', 'close_gripper', 'set_gripper_opening'}


def array_property(title, description, size, default=None, *, allow_null=False):
    result = {'type': ['array', 'string'], 'items': {'type': ['number', 'null'] if allow_null else 'number'}, 'minItems': size, 'maxItems': size,
              'title': title, 'description': description, 'x-vector-length': size}
    if default is not None:
        # Core text inputs stringify JS arrays as "1,0,0". A JSON string default
        # round-trips through that form and through LLM/API callers unchanged.
        import json
        result['default'] = json.dumps(default, separators=(',', ':'))
    return result


class UnifiedControlPlugin(IntentControlPlugin):
    task_actions = USER_TASKS
    def return_preset(self):
        return read_preset(self.data_dir, self.config,
                           adapter_serial(self.config.get('can_interface', 'can0')))

    def capabilities(self):
        cfg = self.config.get('intent_control', {})
        calibrated = False
        try:
            tcp_transform(self.config)
            calibrated = True
        except (RuntimeError, ValueError):
            pass
        preset = False
        preset_reason = None
        try:
            self.return_preset()
            preset = True
        except ValueError as exc:
            preset_reason = str(exc)
        firmware = None
        if self.connection_lock.acquire(blocking=False):
            try:
                if self.driver is not None and not self.closed:
                    firmware = self.driver.firmware()
            finally:
                self.connection_lock.release()
        point = path_capability(self.config, serial=adapter_serial(self.config.get('can_interface', 'can0')),
                                firmware=firmware)
        return {'pose': {'available': calibrated, 'reason': None if calibrated else
                         'Calibrate and verify the installed gripper-tip TCP.'},
                'rotation': {'available': calibrated, 'reason': None if calibrated else
                             'Calibrate and verify the installed gripper-tip TCP.'},
                'return_preset': {'available': preset, 'reason': None if preset else
                                 preset_reason},
                'point_to_point': point}

    def require_path_permission(self):
        capability = self.capabilities()['point_to_point']
        if not capability['available']:
            raise RuntimeError(capability['reason'])

    def get_tools(self):
        tools = deepcopy(super().get_tools())
        arm = next(t for t in tools if t['name'] == CONTROL)
        low, high = self._width_bounds()
        caps = self.capabilities()
        calibrated = caps['pose']['available']
        actions = {
            'select_action': ([], 'Choose an action'),
            'move': (['translation_mm', 'rotation_deg', 'speed_percent'],
                     'Relative tool-frame motion. Without TCP calibration: SDK end axes, translation only, unchanged orientation.'),
            'pose': (['pose_mm_deg', 'motion_mode', 'speed_percent'], 'Absolute base-frame gripper-tip target; null keeps the measured component.'),
            'set_joints': (['joints_deg', 'joint_mode', 'speed_percent'], 'Six joint angles in degrees. Absolute: null keeps measured angle. Relative: 0 keeps angle.'),
            'set_position': ([], 'Exit stationary teaching and hold, then save the measured six-joint position for Reset.'),
            'reset': (['speed_percent'], 'Return all six joints to the saved Set position. Gripper unchanged; no firmware reset or zero calibration.'),
            'stop_motion': ([], 'Cancel preparation or motion; keep motor torque.'),
            'open_gripper': ([], 'Open to the effective opening upper limit.'),
            'close_gripper': ([], 'Close to the effective opening lower limit.'),
            'set_gripper_opening': (['width_mm'], 'Set gripper opening in millimetres.'),
            'get_status': ([], 'Read actual pose, joints, limits, mode, faults and the task result; no control commands.'),
            'start': ([], 'Connect and read; no mode, enable or motion commands.'),
            'stop': ([], 'Stop the card and cancel its current task.'),
            'info': ([], 'Read actual state and the current task result.'),
        }
        props = {
            'translation_mm': array_property('Tool displacement (mm)', 'Max 10 mm; [dx,dy,dz] in tool axes. Without TCP calibration, axes are parallel to the SDK end frame.', 3, [1, 0, 0]),
            'rotation_deg': array_property('Tip rotation (deg)', '[0,0,0] without TCP calibration; otherwise each -5 to 5 deg [roll,pitch,yaw].', 3, [0, 0, 0]),
            'pose_mm_deg': array_property('Base-frame tip pose', '[x,y,z,roll,pitch,yaw]; mm/deg. null keeps measured value; at least one number.', 6, allow_null=True),
            'joints_deg': array_property('Joint angles (deg)', '[J1,J2,J3,J4,J5,J6]; absolute: null keeps measured angle; relative: numbers only. Limits in Get status.', 6, allow_null=True),
            'joint_mode': {'type': 'string', 'enum': ['absolute', 'relative'], 'default': 'absolute',
                           'title': 'Joint target mode', 'description': 'Absolute targets or relative angle increments.'},
            'speed_percent': {'type': 'integer', 'minimum': 1, 'maximum': 10, 'default': 1,
                              'title': 'Speed (%)', 'description': '1–10%; SDK speed percentage, not a fixed mm/s.'},
            'width_mm': {'type': 'number', 'minimum': low, 'maximum': high, 'default': min(high, max(low, 20)),
                         'title': 'Gripper opening (mm)', 'description': f'{low:g}–{high:g} mm; effective limits.'},
        }
        if not calibrated:
            actions.pop('pose')
            actions['move'] = (['translation_mm', 'speed_percent'],
                               'Relative translation along SDK end axes; keep orientation. 0 means no displacement.')
            props.pop('rotation_deg')
            props.pop('pose_mm_deg')
        else:
            paths = ['linear'] + (['point_to_point'] if caps['point_to_point']['available'] else [])
            props['motion_mode'] = {'type': 'string', 'enum': paths, 'default': 'linear',
                'title': 'Path', 'description': 'Linear TCP path, or accepted point-to-point: final pose only; no obstacle avoidance.',
                'oneOf': [{'const': p, 'title': 'Linear' if p == 'linear' else 'Point-to-point'} for p in paths]}
        if not caps['return_preset']['available']:
            actions.pop('reset')
        schema = action_schema(actions, props)
        labels = {'select_action': 'Choose an action', 'move': 'Move relative', 'pose': 'Move to pose', 'set_joints': 'Set joints',
                  'set_position': 'Set position', 'reset': 'Reset', 'stop_motion': 'Stop', 'open_gripper': 'Open gripper',
                  'close_gripper': 'Close gripper', 'set_gripper_opening': 'Set gripper opening',
                  'get_status': 'Get status', 'start': 'Start', 'stop': 'Stop card', 'info': 'Info'}
        schema['properties']['action'].update(title='Action', default='select_action',
            oneOf=[{'const': a, 'title': labels[a]} for a in actions])
        schema.update({'additionalProperties': False, 'x-manual-execution': True, 'x-resource': 'arm',
                       'x-is-dangerous': True, 'x-hooks': {h: {'action': 'stop_motion'}
                            for h in ('on_interrupt_motion', 'on_interrupt_all')},
                       'x-completion': {'actions': sorted(USER_TASKS.intersection(actions)), 'timeout': 60}})
        schema['x-action-params'].pop('select_action')
        arm['inputSchema'] = schema
        arm['description'] = ('One arm: tool-relative Move, absolute/relative joint targets, gripper, Stop and Get status. '
                              'Every control action first exits stationary teaching and holds measured joints when supported, '
                              'then computes and executes the requested target. Failed takeover cancels the action. '
                              'Relative Move uses a linear path. 0 means no increment. '
                              + ('Calibrated tip-relative rotation and absolute base-frame Move to pose are available. '
                                 if calibrated else 'Uncalibrated Move translates along SDK end axes with unchanged orientation. ')
                              + 'Set position exits teaching and holds before saving measured joints. '
                              + ('Reset returns to the saved operator position. ' if 'reset' in actions else '')
                              + 'Keep the workspace clear; a numeric target may still be unreachable.')
        return tools

    def _args(self, name, action, args):
        if name != CONTROL:
            return super()._args(name, action, args)
        # Public capability hiding and backend rejection share the same state.
        caps = self.capabilities()
        if action == 'pose' and not caps['pose']['available']:
            raise RuntimeError('The gripper-tip reference is not calibrated. Configure and verify tcp_offset_mm_deg first.')
        if action == 'reset' and not caps['return_preset']['available']:
            raise RuntimeError(caps['return_preset']['reason'])
        schema = self.get_tools()[0]['inputSchema']
        if action not in schema['properties']['action']['enum']:
            raise ValueError('This action has been replaced; refresh Arm control and choose an available action.')
        if action == 'pose' and args.get('motion_mode') == 'point_to_point':
            self.require_path_permission()
        # Older cards retain rotation=[0,0,0]; accept zero only. Never discard
        # an explicitly requested nonzero rotation when calibration is absent.
        if action == 'move' and not caps['rotation']['available'] and args.get('rotation_deg') not in (None, ''):
            if any(vector(args['rotation_deg'], 3, 'rotation_deg')):
                raise RuntimeError('Gripper-tip rotation requires TCP calibration. Use rotation_deg=[0,0,0] for uncalibrated translation.')
        allowed = set(schema['x-action-params'].get(action, {}).get('params', []))
        declared = set(schema['properties']) | {'rotation_deg', 'pose_mm_deg', 'motion_mode'}
        unknown = set(args) - declared - {'action', '_tool_name', 'instance_id'}
        if action in {'info', 'get_status'} and args.get('input_topic') == '':
            unknown.discard('input_topic')
        if unknown:
            raise ValueError('Unexpected parameters for this action: ' + ', '.join(sorted(unknown)))
        result = {k: v for k, v in args.items() if k in {'action', '_tool_name', 'instance_id'} or k in allowed}
        # Resolve enums first: nullable joints depend on absolute/relative mode.
        for key in sorted(allowed, key=lambda k: 'x-vector-length' in schema['properties'][k]):
            prop = schema['properties'][key]
            if key not in result or result[key] == '':
                if 'default' not in prop:
                    raise ValueError(f'{key} is required.')
                result[key] = prop['default']
            if 'x-vector-length' in prop:
                nullable = key == 'pose_mm_deg' or key == 'joints_deg' and result['joint_mode'] == 'absolute'
                result[key] = vector(result[key], prop['x-vector-length'], key, allow_null=nullable)
            elif prop['type'] in {'number', 'integer'}:
                value = result[key]
                try:
                    vector([value], 1, key)
                except ValueError as exc:
                    raise ValueError(f'{key} must be a finite number.') from exc
                if (prop['type'] == 'integer' and type(value) is not int
                        or not prop['minimum'] <= value <= prop['maximum']):
                    raise ValueError(f'{key} must be within [{prop["minimum"]}, {prop["maximum"]}].')
            elif result[key] not in prop['enum']:
                raise ValueError(f'Invalid {key}.')
        if action == 'move':
            import math
            result.setdefault('rotation_deg', (0., 0., 0.))
            if math.dist(result['translation_mm'], (0, 0, 0)) > 10 or any(abs(v) > 5 for v in result['rotation_deg']):
                raise ValueError('Move allows at most 10 mm translation and 5 degrees per rotation axis.')
        if action == 'pose':
            from agilex.piper.user_motion import validate_numeric_pose
            supplied = [v for v in result['pose_mm_deg'] if v is not None]
            validate_numeric_pose(supplied)
            if any(v is not None and abs(v) > 180 for v in result['pose_mm_deg'][3:]):
                raise ValueError('Pose orientation must be within [-180, 180] degrees.')
        return result

    def dispatch_sync(self, action, args):
        if args.get('_tool_name', CONTROL) == CONTROL and action == 'get_status':
            return super().dispatch_sync('info', args)
        return super().dispatch_sync(action, args)

    def _enter_action_control(self, driver):
        """One guarded takeover before resolving any user movement/gripper target.

        Framework Start also precedes Info and Stop, so it must remain passive.
        Each nonzero control request owns this takeover and its following action
        under the same operation reservation and cancellation token.
        """
        state = checked_state(driver, allow_teaching=True)
        if state.control_mode.startswith('TEACHING_MODE'):
            self._motion_report({'stage': 'exiting_teaching', 'control_mode': state.control_mode,
                                 'joints_deg': state.joints_deg})
            result = exit_teaching_and_hold(driver, require_permission=self.require_motion_permission)
            self.firmware_version = result.get('firmware')
            self._phase('preparing', preparation={'teaching_exit': result})
            # Relative targets and null fields use fresh feedback AFTER takeover,
            # never a pose sampled while the operator was still teaching.
            return checked_state(driver, ready=True)
        return state

    def _execute_task(self, action, args):
        with self.operation(CONTROL, action):
            self._phase('preparing', action=action, started_at=time.time(), result=None, error=None,
                        diagnostics=None, outcome=None, diagnostic_warning=None, preparation=None,
                        request={k: v for k, v in args.items() if k not in {'_tool_name', 'instance_id'}})
            try:
                self._audit('request')
                noop = (action == 'move' and not any((*args['translation_mm'], *args['rotation_deg']))
                        or action == 'set_joints' and args['joint_mode'] == 'relative' and not any(args['joints_deg']))
                if noop:
                    with self.lifecycle:
                        if self.cancel.is_set():
                            raise RuntimeError('The action was cancelled.')
                        result = {'executed': False, 'message': 'No movement requested.', 'motion_mode': 'none'}
                        self._phase('completed', result=result)
                    self._terminal_audit()
                    return {**result, 'state': 'completed'}
                if action == 'pose' and args['motion_mode'] == 'point_to_point':
                    self.require_path_permission()
                self.require_motion_permission()
                driver = self.connected(cancel_event=self.cancel)
                # Recheck the operator gate inside every primitive's cancellation
                # check, including prepare loops and gripper follow-up frames.
                original_check = driver._check_cancelled
                original_permission = getattr(driver, 'require_control_permission', None)
                driver.require_control_permission = self.require_motion_permission
                def check():
                    if self.cancel.is_set():
                        raise RuntimeError('The action was cancelled; motor torque is not released.')
                    original_check()
                    self.require_motion_permission()
                driver._check_cancelled = check
                try:
                    state = self._enter_action_control(driver)
                    if action == 'set_position':
                        record = capture_position(driver, interface=self.config.get('can_interface', 'can0'),
                                                  serial=adapter_serial(self.config.get('can_interface', 'can0')))
                        with self.lifecycle:
                            if self.cancel.is_set():
                                raise RuntimeError('Set position was cancelled; previous position was retained.')
                            driver._check_cancelled()
                            write_preset(self.data_dir, record)
                            with self.task_lock:
                                preparation = deepcopy(self.task.get('preparation') or {})
                            result = {'executed': preparation.get('teaching_exit', {}).get('executed', False),
                                      'return_motion_executed': False, 'teaching_exit': preparation.get('teaching_exit'),
                                      'position_saved': True, 'saved_joints_deg': record['joints_deg'],
                                      'saved_at': record['saved_at'], 'message': 'Current position saved for Reset. Teaching exit and hold were confirmed when required.',
                                      'refresh_tools': True}
                            self._phase('completed', result=result)
                        self._terminal_audit()
                        return {**result, 'state': 'completed'}
                    target = tip_goal = None
                    width = None
                    if action in {'move', 'pose'}:
                        target, tip_goal = target_pose(driver, action, args, self.config)
                    elif action in {'set_joints', 'reset'}:
                        config = self.config
                        if action == 'reset':
                            config = {**self.config, 'intent_control': {**self.config.get('intent_control', {}),
                                      'preset_joints_deg': self.return_preset()['joints_deg']}}
                        target = joint_target(driver, action, args, config, current=state)
                    else:
                        grip = driver.get_gripper()
                        low, high = driver.gripper_range()
                        width = high if action == 'open_gripper' else low if action == 'close_gripper' else args['width_mm']
                        if grip.error_code or not low <= width <= high:
                            raise RuntimeError('The gripper target or feedback is invalid.')
                    self._motion_report({'stage': 'validated', 'control_mode': state.control_mode,
                                         'joints_deg': state.joints_deg, 'tip_target_mm_deg': tip_goal,
                                         'joint_target_deg': target if action in {'set_joints', 'reset'} else None})
                    if state.control_mode.startswith('STANDBY') or not all(state.enabled):
                        driver.prepare_control(execute=True, workspace_clear=True, allow_limit_adjustment=False,
                                               speed_pct=JOG_SPEED_PERCENT)
                    ready = checked_state(driver, ready=True)
                    # Targets are relative to one reference, never silently
                    # recomputed after preparation or a mode change.
                    if max(abs(q-b) for q, b in zip(ready.joints_deg, state.joints_deg)) > .5:
                        raise RuntimeError('The arm moved during preparation; the original target was cancelled.')
                    driver._check_cancelled()
                    self._phase('moving')
                    if action in {'move', 'pose', 'set_joints', 'reset'}:
                        result = execute_target(driver, target, args['speed_percent'],
                            'pose' if action in {'move', 'pose'} else 'joints',
                            require_permission=self.require_motion_permission, report=self._motion_report,
                            tcp_offset=tcp_transform(self.config) if tip_goal is not None else None,
                            tip_goal=tip_goal, motion_mode=args.get('motion_mode', 'linear'),
                            require_path_permission=self.require_path_permission)
                        if action in {'set_joints', 'reset'}:
                            result['reference_frame'] = 'joint_angles'
                        if tip_goal is not None:
                            result['tip_target_mm_deg'] = tip_goal
                            result['reference_frame'] = 'calibrated_gripper_tip'
                        elif action == 'move':
                            result['reference_frame'] = 'sdk_end_axes'
                            result['tcp_verified'] = False
                            result['message'] = ('Tool-frame translation completed with unchanged orientation; '
                                                 'absolute gripper-tip position remains uncalibrated.')
                    else:
                        grip = driver.get_gripper()
                        if not grip.enabled:
                            driver.move_gripper(grip.width_mm, self.gripper_torque)
                        driver._check_cancelled()
                        result = {'executed': True, 'gripper': asdict(driver.move_gripper(width, self.gripper_torque))}
                    with self.lifecycle:
                        driver._check_cancelled()
                        self._phase('completed', result=result)
                    self._terminal_audit()
                    return {**result, 'state': 'completed'}
                finally:
                    driver._check_cancelled = original_check
                    driver.require_control_permission = original_permission
            except BaseException as exc:
                self._phase('cancelled' if self.cancel.is_set() else 'failed', error=public_error(exc),
                            outcome='cancelled' if self.cancel.is_set() else 'operation_failed')
                self._terminal_audit()
                raise RuntimeError(public_error(exc)) from exc

    def read_monitor_snapshot(self):
        snapshot = super().read_monitor_snapshot()
        snapshot['capabilities'] = self.capabilities()
        snapshot['return_preset_configured'] = snapshot['capabilities']['return_preset']['available']
        snapshot['tcp_verified'] = snapshot['capabilities']['pose']['available']
        try:
            snapshot['return_position'] = self.return_preset()
        except ValueError:
            snapshot['return_position'] = None
        pose = snapshot.get('end_pose')
        if pose:
            try:
                value = Transform.from_pose((*pose['position_mm'], *pose['orientation_deg'])).compose(tcp_transform(self.config))
                snapshot['tip_pose_mm_deg'] = value.values()
            except (RuntimeError, ValueError):
                snapshot['tip_pose_mm_deg'] = None
        return snapshot
