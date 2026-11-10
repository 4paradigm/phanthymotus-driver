"""Named joint targets on the same runtime, operation gate and ACP as Arm control."""
from contextlib import contextmanager
from dataclasses import asdict

from common.vendor_runtime import action_schema, tool
from agilex.piper.intent_motion import checked_state
from agilex.piper.path_capability import adapter_serial
from agilex.piper.preset import capture_position
from agilex.piper.unified_control import UnifiedControlPlugin
from agilex.piper.user_motion import require_controller_idle
from agilex.piper.waypoints import WaypointStore, waypoint_name

WAYPOINTS = 'waypoints'


class WaypointControlPlugin(UnifiedControlPlugin):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.waypoints = WaypointStore(self.data_dir)

    def get_tools(self):
        schema = action_schema({
            'select_action': ([], 'Choose an action'),
            'decision_control': ([], 'Ready the card to await decision_core instructions; no hardware connection, mode, enable or motion commands. Canvas binding and intelligent control must be configured separately.'),
            'capture': (['name', 'overwrite'], 'Save measured joints after 0.5 seconds stationary; no movement.'),
            'list': ([], 'List saved waypoints without connecting to the arm'),
            'read': (['name'], 'Read a saved waypoint and gripper observation'),
            'move_to': (['name', 'speed_percent'], 'Move six joints to the named waypoint and verify arrival. Gripper unchanged.'),
            'delete': (['name'], 'Delete the named waypoint'),
            'start': ([], 'Ready the card without motor commands'),
            'stop': ([], 'Stop this card and cancel its current operation; keep motor torque'),
            'stop_motion': ([], 'Stop the current arm action; keep motor torque'),
            'info': ([], 'Read card readiness and last completion without connecting')}, {
                'name': {'type': 'string', 'title': 'Waypoint name', 'minLength': 1, 'maxLength': 80},
                'overwrite': {'type': 'boolean', 'default': False, 'title': 'Replace existing waypoint'},
                'speed_percent': {'type': 'integer', 'minimum': 1, 'maximum': 10, 'default': 1,
                                  'title': 'Speed (%)'}})
        schema['properties']['action'].update(title='Action', default='select_action',
            oneOf=[{'const': a, 'title': '智能托管（decision_core）' if a == 'decision_control'
                    else a.replace('_', ' ').capitalize()}
                   for a in schema['properties']['action']['enum']])
        schema['x-action-params'].pop('select_action')
        schema.update({'additionalProperties': False, 'x-resource': 'arm', 'x-manual-execution': True,
                       'x-is-dangerous': True, 'x-completion': {'actions': ['capture', 'move_to'], 'timeout': 90},
                       'x-hooks': {h: {'action': 'stop_motion'} for h in ('on_interrupt_motion', 'on_interrupt_all')}})
        card = tool(WAYPOINTS, 'actuator',
            'Capture, list, read, move to or delete named joint waypoints bound to the USB-CAN serial and interface. '
            'Decision control only readies this card to await instructions; it does not verify hardware or canvas binding, or grant motion permission. '
            'Moves use joint interpolation; verify the path on site. Gripper observations are not replayed. Reset is independent.',
            schema, topic_out=[])
        card['title'] = 'Waypoints'
        return [*super().get_tools(), card]

    def _waypoint_args(self, action, args):
        # The common parser validates per-action fields, defaults and types.
        args = super()._args(WAYPOINTS, action, args)
        if 'name' in args:
            args['name'] = waypoint_name(args['name'])
        return args

    def dispatch(self, action, args):
        if args.get('_tool_name') != WAYPOINTS:
            return super().dispatch(action, args)
        args = self._waypoint_args(action, args)
        if action in {'capture', 'move_to'} and self.config.get('async_actions', True):
            return self.run_async(WAYPOINTS, action, args)
        return self.dispatch_sync(action, args)

    def dispatch_sync(self, action, args):
        if args.get('_tool_name') != WAYPOINTS:
            return super().dispatch_sync(action, args)
        args = self._waypoint_args(action, args)
        if action == 'select_action':
            return {'executed': False, 'message': 'Choose an action first.'}
        if action == 'decision_control':
            # This is a passive readiness shortcut, not an exclusive control
            # mode. Serialize with task reservation and Stop, without clearing
            # cancellation or touching a driver/adapter or deployment gate.
            with self.lifecycle:
                if self.gate.locked():
                    raise RuntimeError('The arm or camera is busy; wait for completion or use Stop.')
                self.closed = False
                self.stopped_cards.discard(WAYPOINTS)
                return {'state': 'ready', 'executed': False, 'control_mode': 'decision_core',
                        'waiting_for_instruction': True, 'manual_actions_available': True,
                        'hardware_verified': False, 'canvas_binding_verified': False,
                        'motion_enabled': self.motion_enabled,
                        'message': 'Card ready to await decision_core instructions. Connect its canvas execution binding '
                                   'and enable platform intelligent control separately. Hardware readiness was not checked; '
                                   'deployment motion permission and all hardware checks still apply to movement.'}
        if action == 'start':
            return self.start(WAYPOINTS)
        if action == 'stop_motion':
            return self.stop_motion()
        if action == 'stop':
            return self.request_stop(WAYPOINTS)
        if action == 'info':
            return {'state': 'stopped' if self.closed or WAYPOINTS in self.stopped_cards else 'ready',
                    'busy': self.gate.locked(), 'active_card': self.active_card,
                    'last_action': self.last_actions.get(WAYPOINTS), 'storage_path': str(self.waypoints.path)}
        if self.closed or WAYPOINTS in self.stopped_cards:
            raise RuntimeError('Waypoints is stopped; start the card before retrying.')
        if action == 'list':
            return {'waypoints': self.waypoints.list()}
        if action == 'read':
            return self.waypoints.read(args['name'])
        previous = getattr(self.worker_context, 'public_call', None)
        self.worker_context.public_call = (WAYPOINTS, action)
        try:
            with self.operation(WAYPOINTS, action):
                try:
                    return self._execute_waypoint(action, args)
                except Exception as exc:
                    if action == 'move_to':
                        self._phase('cancelled' if self.cancel.is_set() else 'failed', error=str(exc))
                    raise
        except Exception:
            # A reserved worker may be cancelled before nested entry. It still
            # owns the gate; a rejected competing caller must never edit task state.
            if (action == 'move_to' and self.cancel.is_set()
                    and getattr(self.worker_context, 'reserved', None) == (WAYPOINTS, action)):
                self._phase('cancelled', error='Waypoint movement was cancelled.')
            raise
        finally:
            self.worker_context.public_call = previous

    def _execute_waypoint(self, action, args):
        if action == 'delete':
            return self.waypoints.delete(args['name'])
        interface = self.config.get('can_interface', 'can0')
        serial = adapter_serial(interface)
        if not serial:
            raise ValueError('USB-CAN identity is unavailable; waypoint operation refused.')
        if action == 'capture':
            driver = self.connected(cancel_event=self.cancel)
            first = checked_state(driver, allow_teaching=True)
            require_controller_idle(driver)
            record = capture_position(driver, interface=interface, serial=serial)
            require_controller_idle(driver)
            if record['feedback_time'] <= first.feedback_time:
                raise RuntimeError('Joint feedback did not advance during capture; no waypoint was saved.')
            try:
                record['gripper'] = asdict(driver.get_gripper())
            except (RuntimeError, ValueError):
                record['gripper'] = None
            record['gripper_replayed'] = False
            @contextmanager
            def commit_guard():
                with self.lifecycle:
                    driver._check_cancelled()
                    if self.cancel.is_set():
                        raise RuntimeError('Waypoint capture was cancelled.')
                    if adapter_serial(interface) != serial:
                        raise RuntimeError('Arm connection changed during capture; no waypoint was saved.')
                    final = checked_state(driver, allow_teaching=True)
                    if (final.control_mode != first.control_mode or final.enabled != first.enabled
                            or max(abs(q-b) for q, b in zip(final.joints_deg, record['joints_deg'])) > .2):
                        raise RuntimeError('Arm posture or mode changed before saving; capture the waypoint again.')
                    yield
            point = self.waypoints.capture(args['name'], record, overwrite=args['overwrite'],
                                          check_cancelled=driver._check_cancelled, commit_guard=commit_guard)
            return {'state': 'completed', 'executed': False, 'waypoint': point}
        point = self.waypoints.read(args['name'])
        if point['device_serial'] != serial or point['can_interface'] != interface:
            raise ValueError('Waypoint belongs to another arm connection; capture it again for this connection.')
        # Keep the outer reservation throughout target lookup, preparation,
        # execution and feedback verification. Never enqueue a second task.
        result = self._execute_task('set_joints', {
            'joints_deg': point['joints_deg'], 'joint_mode': 'absolute', 'speed_percent': args['speed_percent']})
        return {**result, 'waypoint': point, 'gripper_replayed': False}

    @contextmanager
    def operation(self, name='piper_arm', action=None):
        nested = (getattr(self.worker_context, 'reserved', None) == (name, action)
                  or getattr(self.worker_context, 'owned', None) == (name, action))
        with super().operation(name, action):
            if name == WAYPOINTS and action == 'move_to' and not nested:
                self._phase('preparing', action='move_to', error=None, result=None)
            yield

    def start(self, name=None):
        if name != WAYPOINTS:
            return super().start(name)
        with self.lifecycle:
            if self.gate.locked() and self.active_card == WAYPOINTS:
                return {'state': 'stopping' if self.cancel.is_set() else 'busy', 'executed': False}
            self.closed = False
            self.stopped_cards.discard(name)
            return {'state': 'ready', 'executed': False}

    def stop_motion(self):
        with self.lifecycle:
            with self.task_lock:
                terminal = self.task['state'] in {'completed', 'cancelled', 'failed'}
            affected = self.gate.locked() and self.active_card == WAYPOINTS
            if affected:
                self.cancel.set()
                if self.driver is not None:
                    self.driver.cancel_motion()
                if self.active_action == 'move_to' and not terminal:
                    self._phase('stopping')
                return {'state': 'stopping', 'operation_stopping': True, 'motor_torque_released': False}
        return super().stop_motion()
