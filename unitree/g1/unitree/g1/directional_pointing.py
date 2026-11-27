"""One-shot, body-relative indications with the G1_23 fixed right hand."""
import json
import inspect
import logging
import math
import threading
import time
import uuid
from collections import OrderedDict

from directional_commissioning import CommissioningTrial
from directional_receipts import save_terminal


class DirectionalPointing:
    PREFIX = 'g1_directional_pointing'
    MAX_HOLD_S = 5.0
    HISTORY_SIZE = 32

    def __init__(self, backend, notify=None):
        self.model = backend.kinematics
        self.tolerance = self.model.angular_tolerance_deg
        self.backend = backend
        self.trial = CommissioningTrial.from_backend(backend)
        self.bounded_ik = getattr(self.model, 'solver_mode', None) == 'bounded_ik'
        if self.bounded_ik and self.trial:
            raise ValueError('directional_conflicting_solver_modes')
        self.receipt_directory = getattr(backend, 'config', {}).get('receipt_directory')
        self.notify = notify
        self._lock = threading.RLock()
        self._active = None
        self._project_stop = None
        self._history = OrderedDict()
        self._enabled = True
        self._closed = False

    def get_tool(self):
        tool = {
            'name': self.PREFIX, 'type': 'actuator', 'multiInstance': False,
            'description': '静止 G1_23 的固定右手方向示意。使用已验证姿态。'
                           '方向位于 torso_link，X 向前、Y 向左、Z 向上。'
                           'point 验证角度后执行，终态由 ACP 回报；stop 按 ID 停止任务。',
            'inputSchema': {
                'type': 'object', 'required': ['action'],
                'properties': {
                    'action': {'type': 'string', 'enum': ['point', 'stop']},
                    'yaw_deg': {'type': 'number', 'minimum': -180, 'maximum': 180,
                                'description': '偏向角（度）：正前为 0，向左为正。'},
                    'pitch_deg': {'type': 'number', 'minimum': -90, 'maximum': 90,
                                  'description': '俯仰角（度）：水平为 0，向上为正。'},
                    'hold_s': {'type': 'number', 'minimum': 0.1, 'maximum': self.MAX_HOLD_S,
                               'default': 1.0, 'description': 'Time to hold the measured pose (s).'},
                    'action_id': {'type': 'string', 'description': 'ID returned by point.'},
                },
                'x-action-params': {
                    'point': {'params': ['yaw_deg', 'pitch_deg', 'hold_s'],
                              'required': ['yaw_deg', 'pitch_deg'],
                              'description': '校验并执行右手指向，受理后返回 action_id。'},
                    'stop': {'params': ['action_id'], 'required': ['action_id'],
                             'description': '停止指定任务，返回停止结果。'},
                },
                'x-hooks': {hook: {'action': 'stop', 'params': {'_interrupt_active': True}}
                            for hook in ('on_interrupt_motion', 'on_interrupt_all')},
                'x-resource': ['arm_r', 'base'],
                'x-completion': {'actions': ['point'], 'timeout': 120},
            },
        }
        if self.bounded_ik:
            yaw_min, yaw_max = self.model.yaw_limits_deg
            pitch_min, pitch_max = self.model.pitch_limits_deg
            props = tool['inputSchema']['properties']
            props['yaw_deg'].update(minimum=yaw_min, maximum=yaw_max,
                                    description=f'偏航角 {yaw_min:g} 到 {yaw_max:g} 度，正前为 0，向左为正。')
            props['pitch_deg'].update(minimum=pitch_min, maximum=pitch_max,
                                      description=f'俯仰角 {pitch_min:g} 到 {pitch_max:g} 度，水平为 0，向上为正。')
            props['hold_s']['description'] = '达到指向姿态后保持的秒数，默认 1 秒。'
            tool['description'] = ('常规运控模式下，G1_23 用固定右手按输入角度指向，'
                                   '左臂保持调用起始时的实测姿态。每次从当前姿态规划，'
                                   '方向相对 torso_link，随躯干朝向变化。任务占用双臂，'
                                   f'反馈指向误差不超过 {self.model.angular_tolerance_deg:g} 度'
                                   '并连续保持 hold_s 后完成，'
                                   '通过 ACP 回报指向误差、左臂保持偏差和执行结果。')
            tool['inputSchema']['x-resource'] = ['arm_l', 'arm_r']
        elif self.trial:
            tool['description'] = ('G1_23 现场单次试跑。指定右手方向，保持 1 秒；'
                                   '指向精度待外测。需要现场操作员提供一次性试跑码。')
            schema = tool['inputSchema']
            schema['properties']['operator_code'] = {
                'type': 'string', 'writeOnly': True, 'format': 'password', 'x-sensitive': True,
                'description': '仅用于本次调用的现场试跑码，请勿保存到画布配置。'}
            point = schema['x-action-params']['point']
            point['params'].append('operator_code')
            point['required'] += ['hold_s', 'operator_code']
        return tool

    def start(self):
        pass  # Bundle startup does not claim the arm.

    def stop(self):
        self._stop_active(shutdown=True)

    def _project_state(self):
        job = self._project_stop
        if job is not None:
            if not job['done'].is_set():
                return {'state': 'stopping'}
            if job['result']['state'] == 'error':
                return {'state': 'error', 'error': job['result']['reason']}
        return {'state': 'running' if self._enabled else 'idle'}

    def _stop_active(self, shutdown=False):
        with self._lock:
            self._enabled = False
            self._closed |= shutdown
            job = self._active
            if job is not None:
                self._project_stop = job
                job.setdefault('stop_reason', 'driver_shutdown' if shutdown else 'project_stopped')
                job['stop'].set()
            if not shutdown:
                return self._project_state()
        if job is not None:  # Driver shutdown must precede teardown of shared hardware.
            job['worker'].join()

    def _stop_task(self, args):
        with self._lock:
            job = self._active
            if 'action_id' in args:
                ident = args['action_id']
                if not isinstance(ident, str) or ident not in self._history:
                    return {'error': 'unknown action_id'}
                if job is None or job['id'] != ident:
                    return dict(self._history[ident])
                reason = 'stop_requested'
            else:
                if job is None:
                    return {'state': 'idle'}
                reason = 'motion_interrupted'
            job.setdefault('stop_reason', reason)
            job['stop'].set()
            if 'action_id' not in args:
                return {'state': 'stopping', 'action_id': job['id']}
        # Let the worker finish handback without holding the dispatch lock.
        # Completion of the HTTP callback is independent of physical handback.
        if not job['done'].wait(15):
            return {'action_id': job['id'], 'state': 'stopping', 'error': 'stop_not_yet_confirmed'}
        return dict(job['result'])

    def _plan(self, args, hold):
        if args.get('frame', 'torso_link') != 'torso_link' or args.get('side', 'right') != 'right':
            raise ValueError('only the right arm and torso_link frame are supported')
        if 'direction' in args:
            raise ValueError('point accepts yaw_deg and pitch_deg')
        yaw = args.get('yaw_deg')
        pitch = args.get('pitch_deg')
        if type(yaw) not in (int, float) or not -180 <= yaw <= 180:
            raise ValueError('yaw_deg must be a number between -180 and 180')
        if type(pitch) not in (int, float) or not -90 <= pitch <= 90:
            raise ValueError('pitch_deg must be a number between -90 and 90')
        azimuth, elevation = math.radians(yaw), math.radians(pitch)
        horizontal = math.cos(elevation)
        target = [horizontal * math.cos(azimuth), horizontal * math.sin(azimuth), math.sin(elevation)]
        if self.trial:
            joints = self.trial.target(args)
            self.trial.authorize(args.get('operator_code'))
        else:
            if 'operator_code' in args:
                raise ValueError('directional_commissioning_inactive')
            if self.bounded_ik and callable(getattr(self.backend, 'plan', None)):
                return {'frame': 'torso_link', 'target_direction': target,
                        **self.backend.plan(target, hold)}
            joints = self.model.solve(target)
        return {'frame': 'torso_link', 'target_direction': target, 'target_q_rad': joints,
                'planned_direction': self.model.direction(joints)}

    def dispatch(self, action, args):
        if action == 'stop':
            if 'action_id' in args or args.get('_interrupt_active') is True:
                return self._stop_task(args)
            return self._stop_active()
        with self._lock:
            if action == 'start':
                if self._closed:
                    return {'error': 'card is shut down'}
                if self._project_stop is not None and not self._project_stop['done'].is_set():
                    return {'state': 'stopping', 'error': 'handback pending'}
                self._project_stop = None
                self._enabled = True
                return {'state': 'ready'}
            if action == 'info':
                return self._project_state()
            if action != 'point':
                return {'error': 'unknown action: ' + str(action)}
            if not self._enabled or self._closed:
                return {'error': 'card is stopped'}
            if self._active:
                return {'error': 'pointing already active'}
            try:
                hold = args.get('hold_s', 1.0)
                if type(hold) not in (float, int) or not 0.1 <= hold <= self.MAX_HOLD_S:
                    raise ValueError('hold_s must be a number between 0.1 and 5')
                plan = self._plan(args, hold)
                if 'start_q_rad' not in plan:
                    self.backend.check_ready(plan['target_q_rad'], hold)
                ident = uuid.uuid4().hex
                if self.trial:
                    self.trial.authorize(args.get('operator_code'), consume_for=ident)
            except (TypeError, ValueError, RuntimeError, OverflowError) as exc:
                return {'error': str(exc)}
            result = {'action_id': ident, 'state': 'accepted', **plan, 'actual_direction': None,
                      'angular_error_deg': None, 'sample_phase': None, 'reason': None}
            mode_fields = {}
            if self.trial:
                mode_fields = {'execution_mode': 'commissioning_trial', 'external_accuracy': 'pending',
                               'trial_id': self.trial.record['trial_id']}
            elif self.bounded_ik:
                mode_fields = {'execution_mode': 'bounded_ik', 'external_accuracy': 'pending'}
            result.update(mode_fields)
            job = {'id': ident, 'stop': threading.Event(), 'done': threading.Event(), 'result': result}
            worker = threading.Thread(target=self._run, args=(job, hold), name='g1-point', daemon=True)
            job['worker'] = worker
            self._active = job
            self._history[ident] = result
            if len(self._history) > self.HISTORY_SIZE:
                self._history.popitem(last=False)
            try:
                worker.start()
            except RuntimeError as exc:
                self._active = None
                del self._history[ident]
                return {'error': str(exc)}
            return {'ret': 0, 'state': 'accepted', 'action_id': ident, **mode_fields}

    def _run(self, job, hold):
        result = job['result']
        measured_hold = False
        envelope_failure = None
        left_hold = None
        arrival_diagnostics = None
        lifecycle_evidence = {}

        def observe(phase, q5):
            nonlocal measured_hold
            with self._lock:
                result['state'] = phase
                if q5 is None:
                    return
                measured = self.model.evaluate(q5, result['target_direction'])
                error = measured['angular_error_deg']
                result.update(actual_direction=measured['direction'], angular_error_deg=error,
                              sample_phase=phase, sample_time_s=time.time())
                if phase == 'holding':
                    if error > self.model.angular_tolerance_deg:
                        raise RuntimeError('measured direction outside tolerance')
                    measured_hold = True

        try:
            observe('preparing', None)
            execution_args = {}
            if 'start_q_rad' in result:
                execution_args['planned_start_q5'] = result['start_q_rad']
            if self.bounded_ik:
                # New backends consume the original requested direction. Older
                # fake/legacy backends retain their existing call contract.
                try:
                    parameters = inspect.signature(self.backend.execute).parameters.values()
                    accepts_direction = any(
                        p.kind == inspect.Parameter.VAR_KEYWORD or
                        (p.name == 'target_direction' and p.kind in
                         (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY))
                        for p in parameters)
                except (TypeError, ValueError):
                    accepts_direction = False
                if accepts_direction:
                    execution_args['target_direction'] = list(result['target_direction'])
            execution = self.backend.execute(result['target_q_rad'], hold, job['stop'], observe, **execution_args)
            if isinstance(execution, dict):
                left_hold = execution.get('left_hold')
                arrival_diagnostics = execution.get('arrival_diagnostics')
                if not job['stop'].is_set() and isinstance(arrival_diagnostics, dict) and not (
                        arrival_diagnostics.get('hold_completed') is True and
                        type(arrival_diagnostics.get('hold_elapsed_s')) in (int, float) and
                        arrival_diagnostics['hold_elapsed_s'] >= hold):
                    raise RuntimeError('directional_hold_not_completed')
            if not measured_hold and not job['stop'].is_set():
                raise RuntimeError('no verified pointing feedback')
            error = None
        except Exception as exc:
            error = str(exc) or type(exc).__name__
            envelope_failure = getattr(exc, 'envelope_failure', None)
            left_hold = getattr(exc, 'left_hold', left_hold)
            arrival_diagnostics = getattr(exc, 'arrival_diagnostics', arrival_diagnostics)
            for key in ('startup_failure', 'release_failure'):
                evidence = getattr(exc, key, None)
                if isinstance(evidence, dict):
                    lifecycle_evidence[key] = evidence

        with self._lock:
            if job['stop'].is_set() and error in (None, 'cancelled'):
                result.update(state='cancelled', reason=job['stop_reason'])
            elif error is not None:
                result.update(state='error', error=error, reason=error)
            else:
                result['state'] = 'completed'
            summary = {k: result[k] for k in ('state', 'frame', 'actual_direction',
                                               'angular_error_deg', 'sample_phase', 'reason')}
            if result['state'] == 'error' and envelope_failure is not None:
                result['envelope_failure'] = summary['envelope_failure'] = envelope_failure
            if left_hold is not None:
                result['left_hold'] = summary['left_hold'] = left_hold
            if isinstance(arrival_diagnostics, dict):
                result['arrival_diagnostics'] = summary['arrival_diagnostics'] = arrival_diagnostics
            result.update(lifecycle_evidence)
            summary.update(lifecycle_evidence)
            if self.trial:
                for key in ('execution_mode', 'external_accuracy', 'trial_id'):
                    summary[key] = result[key]
            elif self.bounded_ik:
                summary.update(execution_mode='bounded_ik', external_accuracy='pending')
                if result['actual_direction'] is not None:
                    x, y, z = result['actual_direction']
                    summary.update(actual_yaw_deg=math.degrees(math.atan2(y, x)),
                                   actual_pitch_deg=math.degrees(math.atan2(z, math.hypot(x, y))))
            if summary['reason']:
                summary['reason'] = summary['reason'][:200]
        if self.trial:
            try:
                self.trial.record_terminal(job['id'], summary)
            except Exception as exc:
                logging.getLogger(__name__).error('Trial terminal persistence failed for %s (%s)',
                                                  job['id'], type(exc).__name__)
        elif self.bounded_ik and self.receipt_directory:
            try:
                save_terminal(self.receipt_directory, job['id'], summary)
            except (OSError, ValueError, TypeError) as exc:
                logging.getLogger(__name__).error('Point terminal persistence failed for %s (%s)',
                                                  job['id'], type(exc).__name__)
        with self._lock:
            self._active = None
            job['done'].set()
        if self.notify:
            output = self._acp_summary(summary) if self.bounded_ik else summary
            self.notify(job['id'], result['state'], output, tool=self.PREFIX)

    @staticmethod
    def _acp_summary(summary):
        """Keep the canvas terminal small; the saved receipt contains full evidence."""
        compact = dict(summary)
        arrival = compact.pop('arrival_diagnostics', None)
        if isinstance(arrival, dict):
            fields = {'limit_deg': 'tolerance_deg', 'required_s': 'required_hold_s',
                      'held_s': 'hold_elapsed_s', 'best_s': 'longest_hold_s',
                      'completed': 'hold_completed', 'joint_error_rad': 'max_abs_joint_error_rad'}
            compact['arrival'] = {small: round(arrival[key], 4) if type(arrival.get(key)) in (float,int)
                                  else arrival[key] for small,key in fields.items() if key in arrival}
            if type(arrival.get('feedback_age_ns')) is int:
                compact['arrival']['feedback_age_ms'] = round(arrival['feedback_age_ns']/1e6, 2)
        startup = compact.pop('startup_failure', None)
        release = compact.pop('release_failure', None)
        if startup and startup.get('phase'):
            compact['failure_phase'] = startup['phase']
        if release:
            compact['release_phase'] = release.get('timed_out_state') or release.get('state')
        left = compact.pop('left_hold', None)
        if left is not None:
            compact['left_max_error_rad'] = round(left['max_error_rad'], 6)
        failure = compact.get('envelope_failure')
        if failure is not None:
            compact['envelope_failure'] = {
                key: round(value, 6) if type(value) is float else value
                for key, value in failure.items()
                if key in ('predicate', 'motor_id', 'failing_value_rad', 'lower_rad', 'upper_rad')}
        if compact.get('actual_direction') is not None:
            compact['actual_direction'] = [round(value, 6) for value in compact['actual_direction']]
        for key in ('angular_error_deg', 'actual_yaw_deg', 'actual_pitch_deg'):
            if type(compact.get(key)) in (float, int):
                compact[key] = round(compact[key], 4)
        if compact.get('reason'):
            compact['reason'] = compact['reason'][:120]
        # Direction vector/angles are redundant in the small canvas terminal.
        # Keep failure identity and phase visible; full values remain in receipt.
        for key in ('actual_direction', 'actual_yaw_deg', 'actual_pitch_deg'):
            if len(json.dumps(compact, ensure_ascii=True)) > 600:
                compact.pop(key, None)
        # Full evidence lives in the receipt. Keep the direction threshold and
        # hold completion visible even when another failure adds bulky details.
        for key in ('feedback_age_ms', 'joint_error_rad', 'best_s'):
            if len(json.dumps(compact, ensure_ascii=True)) > 600:
                compact.get('arrival', {}).pop(key, None)
        if len(json.dumps(compact, ensure_ascii=True)) > 600:
            compact.pop('frame', None)
        if len(json.dumps(compact, ensure_ascii=True)) > 600:
            compact.pop('left_max_error_rad', None)
        if compact.get('reason') and len(json.dumps(compact, ensure_ascii=True)) > 600:
            # Escaped JSON length is not Unicode character count. Find the
            # longest fitting prefix so a Chinese exception cannot erase the
            # leading machine-readable failure code.
            reason = compact['reason']
            low, high = 0, len(reason)
            while low < high:
                middle = (low+high+1)//2
                compact['reason'] = reason[:middle]
                if len(json.dumps(compact, ensure_ascii=True)) <= 600:
                    low = middle
                else:
                    high = middle-1
            compact['reason'] = reason[:low]
        return compact
