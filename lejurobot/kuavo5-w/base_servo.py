"""ActuCore motus.control/1 twist sink for the wheeled base (DDS domain 42).

The ROS2 callback only retains the latest frame. A dedicated worker owns all
control transitions and rechecks freshness after potentially slow preparation.
"""
import json
import math
import threading
import time

from common.control import ControlSink
from base_limits import LINEAR_LIMIT, ANGULAR_LIMIT


def footprint_from_polygon(points, padding=0.):
    """Bound the live factory costmap polygon, explicitly as an estimate."""
    if isinstance(points, str):
        if len(points) > 8192:
            raise ValueError('oversized footprint')
        points = json.loads(points)
    if (not isinstance(points, list) or not 3 <= len(points) <= 64
            or isinstance(padding, bool) or not isinstance(padding, (int, float))
            or not math.isfinite(padding) or not 0 <= padding <= .5):
        raise ValueError('invalid footprint')
    if any(not isinstance(p, (list, tuple)) or len(p) != 2
           or any(isinstance(x, bool) or not isinstance(x, (int, float))
                  or not math.isfinite(x) or abs(x) > 3 for x in p) for p in points):
        raise ValueError('invalid footprint vertices')
    xs, ys = zip(*points)
    if not min(xs) < 0 < max(xs) or not min(ys) < 0 < max(ys):
        raise ValueError('footprint must contain the base origin')
    return {'shape': 'box', 'half_width': max(abs(min(ys)), abs(max(ys))) + padding,
            'front': max(xs) + padding, 'rear': -min(xs) + padding, 'height': None,
            'source': 'estimate', 'arms': 'at-rest',
            'provenance': '/lqr_path_track/local_costmap/footprint + footprint_padding; '
                          'factory planner envelope, not measured full-body clearance'}


def build_descriptor(footprint=None):
    descriptor = {'control_interface': 'motus.control/1', 'mode': 'twist', 'dof': 6,
            'joint_names': ['vx', 'vy', 'vz', 'wx', 'wy', 'wz'],
            'units': {'linear': 'm/s', 'angular': 'rad/s', 'time': 's'},
            'frame': 'base_link',
            'limits': {'lower': [-LINEAR_LIMIT, -LINEAR_LIMIT, 0., 0., 0., -ANGULAR_LIMIT],
                       'upper': [LINEAR_LIMIT, LINEAR_LIMIT, 0., 0., 0., ANGULAR_LIMIT],
                       'max_delta_per_step': [.05, .05, 1e-6, 1e-6, 1e-6, .1]},
            'groups': [{'name': 'base', 'offset': 0, 'count': 6, 'unit': 'm/s', 'resource': 'base'}],
            'rate': {'expected_hz': 10., 'max_hz': 20., 'watchdog_ms': 300, 'max_obs_age_ms': 500},
            'force_torque': None}
    if footprint:
        descriptor['footprint'] = footprint
    return descriptor


class BaseServo:
    PREFIX = 'base_servo'

    def __init__(self, bundle, node):
        self.bundle, self.node = bundle, node
        self._lock = bundle.tracker.lock
        self._subscription = None
        self._subscription_generation = 0
        self._thread = None
        self._closed = threading.Event()
        self._latest = None
        self._paused = True
        self._topic = ''
        self._sink = None
        self._lease = None
        self._error = None
        self._last = None
        self._applied = 0
        self._epoch = None
        self._session = 0

    def tool(self):
        return {'name': self.PREFIX, 'type': 'actuator', 'multiInstance': False,
                'description': '≤0.8 m/s、1 rad/s；断流300ms发零速。ActuCore navi速度流经上位机MPC执行，保持BaseArm；与其他底盘动作互斥，显式底盘/导航命令会暂停本流。',
                'topic_in': [{'format': 'control/velocity', 'desc': '6维机身速度[vx,vy,vz,wx,wy,wz]；m/s、rad/s'}],
                'inputSchema': {'type': 'object', 'required': ['action'],
                    'properties': {'action': {'type': 'string', 'enum': ['start', 'stop', 'pause', 'resume', 'info']},
                                   'input_topic': {'type': 'string'}},
                    'x-resource': 'base', 'x-is-dangerous': True,
                    'x-action-params': {'pause': {'params': [], 'description': '暂停速度流并发零速'},
                                        'resume': {'params': [], 'description': '恢复接收新的速度指令'}},
                    'x-hooks': {'on_interrupt_motion': {'action': 'pause'},
                                'on_interrupt_all': {'action': 'pause'}}}}

    def _new_sink(self):
        return ControlSink(self._descriptor(), self._apply, on_watchdog=self._hold, on_abort=self._hold)

    def _descriptor(self):
        return build_descriptor(self.bundle.peripherals.footprint())

    def info(self):
        with self._lock:
            startup = self.bundle.spatial.startup_status()
            state = ('idle' if self._subscription is None else 'paused' if self._paused
                     else 'loading' if startup['state'] == 'loading' else 'running')
            return {'state': state,
                    'input': self._topic, 'control_interface': self._descriptor(),
                    'applied': self._applied, 'last': self._last, 'error': self._error,
                    'startup': startup, 'control_ownership': self.bundle.control_ownership()}

    def _start_reply(self):
        status = self.info()
        return {**status, 'state': 'ready', 'stream_state': status['state']}

    def dispatch(self, action, args):
        try:
            if action == 'info':
                return self.info()
            if action == 'start':
                topic = args.get('input_topic') or (args.get('input_topics') or [''])[0]
                if not isinstance(topic, str) or not topic.strip():
                    raise ValueError('Connect a control/velocity source: input_topic is required')
                with self._lock:
                    if self._closed.is_set():
                        raise RuntimeError('driver is shutting down')
                    if self._subscription is not None:
                        if topic != self._topic:
                            raise RuntimeError('stop before changing input_topic')
                        return self._start_reply()
                    joining_start = self.bundle.spatial.chassis_start_in_progress()
                    if self.bundle._transition is not None and not joining_start:
                        raise RuntimeError('base is busy; stop the current action first')
                    defer_control = ('base' in self.bundle.tracker._active
                                     or (self.bundle.spatial.unresolved() and not joining_start))
                    self.bundle.state.require_motion_enabled()
                    self._sink = self._new_sink()
                    from std_msgs.msg import String
                    self._subscription_generation += 1
                    generation = self._subscription_generation
                    self._subscription = self.node.create_subscription(
                        String, topic, lambda msg: self._receive(msg, generation), 1)
                    self._topic, self._paused = topic, defer_control
                    self._epoch = self.bundle._stop_epoch
                    if self._thread is None:
                        self._thread = threading.Thread(target=self._run, daemon=True, name='base-servo')
                        self._thread.start()
                if defer_control:
                    return {**self._start_reply(), 'message': 'Input connected and paused; resume after the current base operation finishes.'}
                preparation = self.bundle.spatial.start_chassis()
                if preparation.get('state') == 'error':
                    with self._lock:
                        self.mark_paused()
                        self._error = preparation.get('error')
                    return preparation
                return self._start_reply()
            if action in ('pause', 'stop'):
                self.preempt()
                if action == 'stop':
                    with self._lock:
                        if self._subscription is not None:
                            self.node.destroy_subscription(self._subscription)
                            self._subscription = None
                        self._topic = ''
                return self.info()
            if action == 'resume':
                with self._lock:
                    if self._subscription is None:
                        raise RuntimeError('start with input_topic first')
                    joining_start = self.bundle.spatial.chassis_start_in_progress()
                    if ((self.bundle._transition is not None and not joining_start)
                            or 'base' in self.bundle.tracker._active
                            or (self.bundle.spatial.unresolved() and not joining_start)):
                        raise RuntimeError('base is busy; stop the current action first')
                    self._sink = self._new_sink()
                    self._lease = None
                    self._latest = None
                    self._paused, self._error = False, None
                    self._epoch = self.bundle._stop_epoch
                preparation = self.bundle.spatial.start_chassis()
                if preparation.get('state') == 'error':
                    with self._lock:
                        self.mark_paused()
                        self._error = preparation.get('error')
                    return preparation
                return self.info()
            raise ValueError('unknown action')
        except Exception as exc:
            return {'state': 'error', 'error': str(exc).encode('unicode_escape').decode('ascii')[:400]}

    def _receive(self, message, generation=None):
        try:
            if len(message.data) > 16384:
                return
            command = json.loads(message.data)
            if not isinstance(command, dict):
                return
        except (ValueError, TypeError):
            return
        with self._lock:
            if (not self._paused and self._subscription is not None
                    and (generation is None or generation == self._subscription_generation)):
                self._latest = command

    def _run(self):
        while not self._closed.wait(.05):
            with self._lock:
                if self._paused or self._subscription is None:
                    continue
                if self._epoch != self.bundle._stop_epoch:
                    self.preempt()
                    continue
                startup = self.bundle.spatial.startup_status()
                if self._lease is None and startup['state'] != 'running':
                    if startup['state'] == 'error':
                        self._error = startup.get('error')
                        self._paused = True
                    continue
                command, self._latest = self._latest, None
                sink = self._sink
                session = self._session
            try:
                if command is not None:
                    # submit may need ROS1 enable/mode services, so no callback
                    # or driver lock is held across this operation.
                    self._command = command
                    self._command_session = session
                    outcome = sink.submit(command)
                    with self._lock:
                        self._last = {'verdict': outcome.verdict.value, 'reason': outcome.reason}
                with self._lock:
                    if self._sink is sink and not self._paused:
                        sink.tick()
                        if self._lease is not None:
                            self.bundle.require_base_control(self.PREFIX)
                            if self.bundle.ros1.connection != (True, self._lease[2]):
                                raise ConnectionError('connection changed during base servo')
            except Exception as exc:
                with self._lock:
                    if self._session == session:
                        self._error = str(exc).encode('unicode_escape').decode('ascii')[:400]
                        self.mark_paused()
                # Includes uncertain initial sends; never discard the tracked
                # action before the bundle's normal cancellation/pause path.
                aid = getattr(exc, 'action_id', None) or (self._lease[0] if self._lease else None)
                if aid is not None:
                    self.bundle.pause('base_servo_failure', status='error', expected=('base', aid))
                with self._lock:
                    self._lease = None

    def _current(self):
        if (self._paused or self._closed.is_set() or self._epoch != self.bundle._stop_epoch
                or self._command_session != self._session):
            raise InterruptedError('base servo superseded by stop')

    def _fresh(self):
        self._current()
        command = self._command
        now = time.time()*1000
        if (now-command['stamp_ms'] > min(command['ttl_ms'], 300)
                or now-command.get('obs_stamp_ms', command['stamp_ms']) > 500):
            return False
        return True

    def _apply(self, values, _gripper=None):
        # Zero/lease acquisition uses the same guarded path as base_drive.
        if self._lease is None:
            lease = self.bundle.start_motion('base', self.PREFIX, (3,),
                lambda: self.bundle.twist('/cmd_vel'), self._current,
                request_epoch=self._epoch, notify=False)
            with self._lock:
                self._lease = lease
        with self._lock:
            if not self._fresh():
                self._hold()
                return
            aid, _, generation, *_ = self._lease
            if not self.bundle.tracker.current('base', aid):
                raise InterruptedError('base servo ownership was cancelled')
            self.bundle.require_base_control(self.PREFIX)
            if self.bundle.ros1.connection != (True, generation):
                raise ConnectionError('connection changed during base servo')
            scale = min(1., LINEAR_LIMIT / max(math.hypot(values[0], values[1]), 1e-12))
            self.bundle.twist('/cmd_vel', x=values[0]*scale, y=values[1]*scale, yaw=values[5])
            self._applied += 1

    def _hold(self):
        # Watchdog hold is resumable on a new fresh frame. Explicit pause is
        # latched separately. Do not publish zeros over another base owner's task.
        if self._lease is not None:
            aid = self._lease[0]
            if self.bundle.tracker.current('base', aid):
                self.bundle.twist('/cmd_vel')
                self.bundle.tracker.finish('base', aid, self.PREFIX, 'cancelled', {'reason': 'servo_hold'})
            self._lease = None

    def mark_paused(self):
        # Called by bundle.pause while holding tracker.lock; no IO here.
        self._paused = True
        self._latest = None
        self._session += 1

    def preempt(self):
        with self._lock:
            self.mark_paused()
            aid = self._lease[0] if self._lease else None
            token = self.bundle._transition
            pending = token if token is not None and token.get('tool') == self.PREFIX else None
            try:
                self._hold()
                failed = False
            except Exception:
                self.bundle.state.motion_send_uncertain = True
                self.bundle.state.motion_blocked = True
                failed = True
        if pending is not None:
            # A zero/target lease does not exist yet while enable is pending.
            # Cancel the transition through the bundle's remote-resume ordering,
            # rather than reporting paused before an older enable is resolved.
            result = self.bundle.pause('base_servo_preparation_stopped', expected_transition=pending)
            if result.get('error'):
                raise RuntimeError(result['error'])
        if failed:
            self.bundle.pause('base_servo_stop_send_failed', status='error', expected=('base', aid))
            raise RuntimeError('base servo zero velocity send failed; inspect stop confirmation')

    def close(self):
        self._closed.set()
        self.preempt()
        if self._subscription is not None:
            self.node.destroy_subscription(self._subscription)
            self._subscription = None
        if self._thread is not None:
            self._thread.join(timeout=1)
