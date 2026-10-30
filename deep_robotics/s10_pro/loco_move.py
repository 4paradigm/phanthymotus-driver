"""One bounded S10 controller for posture, timed and relative motion.

The platform owns ACP scheduling; this plugin serializes all base writes and
checks current hardware feedback. It has no policy sessions or persistent lock.
"""
from __future__ import annotations

import copy
import logging
import math
import threading
import time
from collections import OrderedDict
from uuid import uuid4

try:
    from .acp import complete
    from .motion_feedback import fresh, motion_observation, odometry_observation
    from .motion_schema import tools
    from .parameters import (motion_defaults, motion_config_schema, validate_motion_settings,
        normalize_config_updates, validate_empty_arguments, validate_motion_request)
    from .relative_motion import RelativeController
    from .stationary_feedback import StationaryWindow
except ImportError:
    from acp import complete
    from motion_feedback import fresh, motion_observation, odometry_observation
    from motion_schema import tools
    from parameters import (motion_defaults, motion_config_schema, validate_motion_settings,
        normalize_config_updates, validate_empty_arguments, validate_motion_request)
    from relative_motion import RelativeController
    from stationary_feedback import StationaryWindow

DIRECTIONS = {'forward': (1., 0., 0.), 'backward': (-1., 0., 0.),
              'turn_left': (0., 0., 1.), 'turn_right': (0., 0., -1.)}
ALIASES = {'sitdown': 'sit_down', 'stand_down': 'sit_down'}


class MotionUnavailable(RuntimeError):
    pass


class LocoMovePlugin:
    name = 'loco_move'

    def __init__(self, config, namespace, bridge, transport_factory, *, callback=complete,
                 clock=time.monotonic, wall_time=time.time, host_enabled=True):
        self._settings = validate_motion_settings({'max_vx': 1., 'max_yaw_rate': 1., 'action_timeout_s': 15., **config})
        self._host_enabled = host_enabled
        self._bridge, self._factory = bridge, transport_factory
        self._transport = None
        self._callback, self._clock, self._wall = callback, clock, wall_time
        self._lock, self._io_lock = threading.RLock(), threading.Lock()
        self._active = self._worker = self._last = None
        self._cancel = threading.Event()
        self._started = self._closed = False
        self._history = OrderedDict()
        self._stationary = StationaryWindow(clock=clock)
        self._channel = {}
        self._navigation_stop = threading.Event()
        self._navigation_thread = None
        self._navigation_attempted = False
        self._automatic_navigation = {'state': 'idle', 'physical_command_sent': False}
        self._relative_topic = '/' + '/'.join(p for p in (namespace.strip('/'), 's10_pro', 'relative_move_status') if p)

    def get_tools(self):
        return tools(motion_config_schema(self._settings), self._relative_topic)

    def _snapshot(self, source):
        try:
            record = self._bridge.snapshot(source)
            return record if isinstance(record, dict) else {}
        except Exception:
            return {}

    def _observe(self):
        return motion_observation(self._snapshot('motion_info'), self._clock())

    def _odometry(self):
        return odometry_observation(self._snapshot('odometry'), self._clock())

    def _stationary_status(self, states=(0, 1, 17)):
        return self._stationary.observe(self._observe(), states, self._odometry())

    def _blockers(self, *, states=None):
        blocked = []
        if not self._host_enabled or not self._settings['enabled']:
            blocked.append('motion_disabled')
        if not self._started or self._closed:
            blocked.append('card_not_started')
        observed = self._observe()
        if not observed['fresh']:
            blocked.append('fresh_motion_feedback_required')
        hes = self._snapshot('hard_estop')
        hes_data = hes.get('data')
        if not fresh(hes, self._clock(), .5) or not isinstance(hes_data, dict) or type(hes_data.get('value')) is not int or hes_data['value'] != 0:
            blocked.append('fresh_clear_hard_estop_required')
        faults = self._snapshot('fault_aggregator')
        fault_data = faults.get('data')
        if not fresh(faults, self._clock(), 2.) or not isinstance(fault_data, dict) or fault_data.get('active_faults') != []:
            blocked.append('fresh_clear_fault_snapshot_required')
        joints = self._snapshot('joints')
        joint_data = joints.get('data')
        joint_data = joint_data.get('data') if isinstance(joint_data, dict) else None
        rows = joint_data.get('joints_data') if isinstance(joint_data, dict) else None
        if (not fresh(joints, self._clock(), .5) or not isinstance(rows, list) or len(rows) != 16
                or any(not isinstance(r, dict) or type(r.get('status_word')) is not int or r['status_word'] != 1 for r in rows)):
            blocked.append('fresh_enabled_joint_feedback_required')
        if states is not None:
            if observed['motion_state'] not in states:
                blocked.append('move_mode_required' if states == (17,) else 'posture_transition_not_ready')
            if not self._stationary_status(states)['ready']:
                blocked.append('stationary_feedback_required')
        return blocked

    def _check(self, *, states=None, cancelled=True):
        if cancelled and self._cancel.is_set():
            raise MotionUnavailable('cancelled')
        blocked = self._blockers(states=states)
        if blocked:
            raise MotionUnavailable(','.join(blocked))

    def _wait_stationary(self, states, deadline):
        while True:
            self._check()
            observed = self._observe()
            if observed['motion_state'] not in states:
                raise MotionUnavailable('move_mode_required' if states == (17,) else 'posture_transition_not_ready')
            if self._stationary_status(states)['ready']:
                return
            if self._clock() >= deadline:
                raise MotionUnavailable('stationary_feedback_required')
            self._cancel.wait(.04)

    def _ensure_transport(self):
        with self._io_lock:
            if self._transport is None or self._transport.status().get('closed'):
                if self._transport is not None:
                    self._transport.close()
                self._transport = self._factory()
            if not self._transport.status().get('worker_running'):
                self._transport.start(timeout_s=5.)

    def _send(self, *, state=None, velocity=None, stopping=False):
        with self._io_lock:
            if not stopping:
                self._check()
            cancel = threading.Event() if stopping else self._cancel
            if state is not None:
                receipt = self._transport.publish_state(state, cancel_event=cancel)
            else:
                # A zero frame is also a navigation command, only valid in move mode.
                if self._observe()['motion_state'] != 17:
                    raise MotionUnavailable('move_mode_required')
                receipt = self._transport.publish_velocity(*velocity, cancel_event=cancel)
            if receipt.get('sent') is not True:
                raise MotionUnavailable('command_not_sent')
            return receipt

    def start(self):
        with self._lock:
            if self._closed:
                return {'state': 'idle'}
            self._bridge.acquire('s10_motion', {'motion_info', 'hard_estop', 'joints', 'fault_aggregator', 'odometry'},
                                 {'refresh_hz': 10., 'stale_after_s': 2.})
            if self._started:
                return {'state': 'ready'}
            self._started = True
            self._navigation_stop = threading.Event()
            self._navigation_attempted = False
            # If standing is already known, return the accepted transition ID
            # so Core can track this start-triggered action through normal ACP.
            transition = self._maybe_enter_navigation(notify=True)
            self._navigation_thread = threading.Thread(target=self._navigation_loop,
                args=(self._navigation_stop,), name='s10-auto-navigation', daemon=True)
            self._navigation_thread.start()
            return {'state': 'ready', **transition}

    def _maybe_enter_navigation(self, *, notify=False):
        if not self._settings['auto_navigation'] or not self._started or self._closed:
            self._automatic_navigation = {'state': 'disabled'}
            return {}
        if self._active:
            return {}
        state = self._observe()['motion_state']
        if state == 0:
            self._navigation_attempted = False
            self._automatic_navigation = {'state': 'waiting_for_manual_stand'}
        elif state == 17:
            self._navigation_attempted = False
            self._automatic_navigation = {'state': 'navigation_ready'}
        elif state == 1 and not self._navigation_attempted:
            blocked = self._blockers(states=(1,))
            self._automatic_navigation = {'state': 'waiting_for_standing_feedback', 'blocked_by': blocked}
            if not blocked:
                reply = self.execute('enter_navigation', {}, notify=notify)
                if reply.get('accepted'):
                    self._navigation_attempted = True
                    self._automatic_navigation = {'state': 'entering_navigation'}
                    return {'action_id': reply['action_id']} if notify else {}
        return {}

    def _navigation_loop(self, stopped):
        while not stopped.wait(.1):
            with self._lock:
                if stopped.is_set() or not self._started or self._closed:
                    return
                try:
                    self._maybe_enter_navigation()
                except Exception as error:
                    self._automatic_navigation = {'state': 'unavailable', 'reason': type(error).__name__}

    def stop(self):
        with self._lock:
            self._started = False
            self._navigation_stop.set()
            self._automatic_navigation = {'state': 'idle'}
            self._cancel.set()
            idle = self._active is None
        if idle:
            self._bridge.release('s10_motion')
        return {'state': 'idle'}

    def close(self):
        self._closed = True
        self.stop()
        if self._navigation_thread is not None:
            self._navigation_thread.join(1.)
        if self._worker is not None:
            self._worker.join(3.)
        if self._transport is not None:
            self._transport.close()
        self._bridge.release('s10_motion')

    def _info(self, relative=False):
        with self._lock:
            blocked = self._blockers()
            move = list(dict.fromkeys([*blocked, *self._blockers(states=(17,))]))
            if self._active:
                move.append('active_motion_task_in_progress')
            relative_blocked = list(move)
            if not self._odometry()['fresh']:
                relative_blocked.append('fresh_odometry_required')
            actions = {'stand_up': {'ready': not blocked and not self._active, 'blocked_by': blocked},
                       'sit_down': {'ready': not blocked and not self._active, 'blocked_by': blocked},
                       'move': {'ready': not move, 'blocked_by': move}}
            if relative:
                actions = {'relative_move': {'ready': not relative_blocked, 'blocked_by': relative_blocked}}
                blocked = relative_blocked
            return {'card': 'relative_move' if relative else self.name,
                'state': 'ready' if self._started else 'idle',
                'ready': not blocked and not self._active, 'blocked_by': blocked,
                'settings': dict(self._settings), 'active_request': copy.deepcopy(self._active),
                'last_result': copy.deepcopy(self._last),
                'readiness': {'ready': not blocked and not self._active, 'blocked_by': blocked,
                    'actions': actions, 'motion_state': self._observe()['motion_state'],
                    'stationary_check': self._stationary_status(), 'command_channel': dict(self._channel),
                    'automatic_navigation': copy.deepcopy(self._automatic_navigation)}}

    def _request(self, action, arguments):
        values = {k: v for k, v in arguments.items() if k not in ('_tool_name', 'instance_id', 'action')}
        if action in ('stand_up', 'sit_down'):
            # Core can retain hidden move fields while changing the selection.
            for key in ('direction', 'speed', 'duration'):
                values.pop(key, None)
        internal = {'sit_down': 'lie', 'move': 'move_2s', 'enter_navigation': 'stand_up'}.get(action, action)
        if action == 'relative_move':
            turning = values.get('direction', '').startswith('turn_')
            value = values.get('angle' if turning else 'distance')
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('relative target must be finite')
            if turning and (type(value) is not int or value not in (30, 60) or 'distance' in values):
                raise ValueError('relative turn requires angle 30 or 60')
            if not turning and ('angle' in values or not .3 <= value <= .6):
                raise ValueError('relative distance must be .3 through .6 metres')
            values['target'] = math.radians(value) if turning else value
        request = validate_motion_request(internal, values, self._settings, self._wall,
            allowed_actions=('stand_up', 'lie', 'move_2s', 'relative_move'))
        request['action'] = action
        return request

    def execute(self, action, arguments, *, notify=True):
        request = self._request(action, arguments)
        signature = {k: v for k, v in request.items() if k not in ('request_id', 'issued_at_ms', 'ttl_ms')}
        with self._lock:
            previous = self._history.get(request['request_id'])
            if previous:
                if previous['signature'] != signature:
                    raise ValueError('request_id already belongs to another action')
                # A finished replay has no action_id, so Core creates no new pending.
                return copy.deepcopy(previous.get('result') or previous['receipt'])
            if self._active is not None:
                return {'state': 'busy', 'accepted': False, 'reason': 'active_motion_task_in_progress'}
            blocked = self._blockers()
            if action in ('move', 'relative_move') and self._observe()['motion_state'] != 17:
                blocked.append('move_mode_required')
            if blocked:
                return {'state': 'unavailable', 'accepted': False, 'blocked_by': blocked}
            action_id = 's10_motion_' + uuid4().hex
            receipt = {'state': 'moving', 'accepted': True, 'action_id': action_id}
            record = {'signature': signature, 'receipt': receipt}
            self._history[request['request_id']] = record
            while len(self._history) > 256:
                self._history.popitem(last=False)
            self._cancel = threading.Event()
            self._active = {'request_id': request['request_id'], 'action_id': action_id, 'action': action}
            deadline = self._clock() + self._settings['action_timeout_s']
            self._worker = threading.Thread(target=self._run, args=(request, record, deadline, notify),
                                            name='s10-motion', daemon=True)
            try:
                self._worker.start()
            except Exception:
                self._active = None
                self._history.pop(request['request_id'])
                raise
            return receipt

    def cancel(self):
        with self._lock:
            active = self._active is not None
            self._cancel.set()
        return {'state': 'cancellation_requested' if active else 'idle', 'accepted': active}

    def dispatch(self, action, args):
        if args.get('_tool_name') not in (None, 'loco_move', 'relative_move'):
            raise ValueError('unsupported motion tool')
        relative = args.get('_tool_name') == 'relative_move'
        if not relative and action in ('start', 'config', 'info', 'status', 'stop', 'stop_move'):
            args = {k: v for k, v in args.items() if k not in ('direction', 'speed', 'duration')}
        action = ALIASES.get(action, action)
        if action == 'start':
            updates = normalize_config_updates(args, motion_defaults())
            if updates:
                self._configure(updates)
            return self.start()
        if action == 'stop':
            validate_empty_arguments(args)
            return self.stop()
        if action == 'config':
            self._configure(normalize_config_updates(args, motion_defaults()))
            return self._info(relative)
        if action in ('info', 'status'):
            validate_empty_arguments(args)
            return self._info(relative)
        if action == 'stop_move':
            return self.cancel()
        if relative and action in ('move', 'relative_move'):
            return self.execute('relative_move', args)
        if action not in ('stand_up', 'sit_down', 'move'):
            raise ValueError('unsupported locomotion action')
        return self.execute(action, args)

    def _configure(self, updates):
        with self._lock:
            changed = validate_motion_settings({**self._settings, **updates})
            if self._active and changed != self._settings:
                self._cancel.set()
            self._settings = changed

    def _posture(self, state, target, deadline):
        self._check()
        self._wait_stationary((0,) if state == 1 else (1,) if state == 17 else (0, 1, 17),
                              min(deadline, self._clock() + .9))
        marker = self._observe()['sequence']
        self._send(state=state)
        stable = 0
        previous = marker
        while self._clock() < deadline:
            self._check()
            observed = self._observe()
            if observed['sequence'] != previous:
                stable = stable + 1 if observed['motion_state'] == target else 0
                previous = observed['sequence']
                if stable >= 2:
                    return
            self._cancel.wait(.02)
        raise MotionUnavailable('posture_feedback_timeout')

    def _stop_velocity(self, deadline):
        self._stationary.reset()
        deadline = min(deadline, self._clock() + 2.)
        zeros = stable = 0
        previous = None
        while self._clock() < deadline:
            try:
                self._send(velocity=(0., 0., 0.), stopping=True)
                zeros += 1
            except Exception:
                return False
            observed = self._observe()
            if observed['sequence'] != previous:
                check = self._stationary_status((17,))
                stable = stable + 1 if check['ready'] else 0
                previous = observed['sequence']
            if zeros >= 2 and stable >= 2:
                return True
            time.sleep(.1)
        return False

    def _move(self, request, deadline, result):
        self._wait_stationary((17,), min(deadline, self._clock() + .9))
        controller = None
        if request['action'] == 'relative_move':
            initial = self._odometry()
            if not initial['fresh']:
                raise MotionUnavailable('fresh_odometry_required')
            controller = RelativeController(initial, direction=request['direction'], target=request['target'],
                                            max_speed=request['speed'], now=self._clock())
        end = min(self._clock() + request['duration'], deadline - 2.05) if controller is None else deadline - 2.05
        if controller is None and end < self._clock() + request['duration'] - .01:
            raise MotionUnavailable('insufficient_stop_budget')
        response = False
        next_frame = self._clock()
        while self._clock() < end:
            self._check()
            if self._clock() - next_frame > .3:
                raise MotionUnavailable('command_stream_late')
            if self._observe()['motion_state'] != 17:
                raise MotionUnavailable('move_mode_lost')
            if controller is not None:
                pose = self._odometry()
                if not pose['fresh']:
                    raise MotionUnavailable('fresh_odometry_required')
                if pose['sequence'] == controller.last_sequence:
                    self._cancel.wait(.02)
                    continue
                velocity, progress = controller.command(pose, self._clock())
                result['progress'] = progress
                if progress['phase'] == 'completed':
                    return
            else:
                velocity = tuple(v * request['speed'] for v in DIRECTIONS[request['direction']])
            result['physical_command_sent'] = True
            self._send(velocity=velocity)
            result['command_frames'] = result.get('command_frames', 0) + 1
            observed = self._observe()['velocity']
            axes = DIRECTIONS.get(request['direction'])
            if axes is not None:
                projected = sum(observed.get(k, 0.) * axis for k, axis in zip(('vx', 'vy', 'yaw_rate'), axes))
                response |= projected >= min(.03, request['speed'] * .3)
            next_frame += .1
            self._cancel.wait(max(0., next_frame - self._clock()))
        if controller is not None:
            raise MotionUnavailable('relative_motion_timeout')
        if not response:
            raise MotionUnavailable('motion_response_not_observed')

    def _run(self, request, record, deadline, notify):
        action_id = record['receipt']['action_id']
        action = request['action']
        navigation = action in ('move', 'relative_move')
        result = {'action': action, 'request_id': request['request_id'], 'completed': False,
                  'physical_command_sent': False, 'stop_confirmed': False}
        status = 'error'
        try:
            self._check()
            self._ensure_transport()
            kind = 'velocity' if navigation else 'state'
            while self._clock() < deadline:
                self._check()
                self._channel = self._transport.graph_snapshot()
                if self._channel.get(kind, {}).get('matched_subscribers', 0) > 0:
                    break
                self._cancel.wait(.02)
            else:
                raise MotionUnavailable('command_receiver_unmatched')
            if action in ('stand_up', 'enter_navigation'):
                state = self._observe()['motion_state']
                if state not in (0, 1, 17):
                    raise MotionUnavailable('posture_transition_not_ready')
                if action == 'enter_navigation' and state != 1:
                    raise MotionUnavailable('standing_state_required')
                if state == 0:
                    result['physical_command_sent'] = True
                    self._posture(1, 1, deadline)
                    state = 1
                if state == 1:
                    result['physical_command_sent'] = True
                    self._posture(17, 17, deadline)
                self._wait_stationary((17,), min(deadline, self._clock() + .9))
            elif action == 'sit_down':
                if self._observe()['motion_state'] != 0:
                    result['physical_command_sent'] = True
                    self._posture(4, 0, deadline)
            else:
                self._move(request, deadline, result)
            self._check()
            if self._clock() >= deadline:
                raise MotionUnavailable('action_timeout')
            result['completed'] = True
            status = 'completed'
        except Exception as error:
            status = 'cancelled' if self._cancel.is_set() else 'error'
            result['reason'] = str(error) if isinstance(error, MotionUnavailable) else getattr(error, 'reason', type(error).__name__)
        finally:
            if navigation and result['physical_command_sent']:
                result['stop_confirmed'] = self._stop_velocity(deadline)
                if not result['stop_confirmed']:
                    result['completed'] = False
                    result['reason'] = 'reported_stop_not_confirmed'
                    status = 'error'
            if self._cancel.is_set() and status == 'completed':
                result['completed'] = False
                status = 'cancelled'
            result.update(status=status, state=status, finished_at=self._wall())
            with self._lock:
                record['result'] = copy.deepcopy(result)
                self._last = result
                self._active = None
                if action == 'enter_navigation':
                    self._automatic_navigation = {'state': 'navigation_ready' if status == 'completed' else 'error',
                                                  'result': copy.deepcopy(result)}
                if not self._started:
                    self._bridge.release('s10_motion')
            # One worker, one terminal callback. No outbox polling or receipt protocol.
            try:
                # Background standing detection has no dispatched Core call;
                # report it in status rather than inventing an ACP pending ID.
                if notify:
                    self._callback(action_id, status, result, 'relative_move' if action == 'relative_move' else self.name)
            except Exception as error:
                logging.warning('ACP callback failed (%s)', type(error).__name__)
