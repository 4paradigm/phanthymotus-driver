"""Closed-loop relative pose movement through a direct, no-reply SDK client."""
import math
import json
import os
from pathlib import Path
from collections import deque
import threading
import time
from uuid import uuid4

try:
    from .relative_feedback import RelativeController, MotionSafety, SafetyViolation
except ImportError:
    from relative_feedback import RelativeController, MotionSafety, SafetyViolation


class MotionTrace:
    """Recent preview plus full history for the time-limited task; no per-tick stdout."""
    def __init__(self, initial, target, settings):
        self.started = time.monotonic()
        self.wall_time = time.time()
        self.records = deque(maxlen=400)
        self.total = 0
        self.all_records = []
        self.initial, self.target, self.settings = dict(initial), dict(target), settings

    def add(self, event, **fields):
        now = time.monotonic()
        record = dict(event=event, elapsed_s=now - self.started, **fields)
        self.records.append(record)
        self.all_records.append(record)
        self.total += 1
        return record

    def export(self, full=False):
        def clean(value):
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {key: clean(item) for key, item in value.items()}
            if isinstance(value, (list, tuple, deque)):
                return [clean(item) for item in value]
            return value
        return clean(dict(version=1, started_unix_s=self.wall_time,
                          started_monotonic_s=self.started,
                          time_basis='local_callback_receive; source_capture_time_unavailable',
                          initial=self.initial, requested_target=self.target, settings=self.settings,
                          record_count=self.total, dropped_records=0 if full else max(0, self.total - len(self.records)),
                          records=self.all_records if full else self.records))


class LocoRelativeMovePlugin:
    PREFIX = "loco_relative_move"
    TIMEOUT = 300
    MAX_MOTION_SECONDS = 120
    MIN_DISTANCE = 0.3
    MAX_DISTANCE = 5.
    HANDOFF_TIMEOUT = 4.

    def __init__(self, config, loco, notify, can_start=None, admission=None):
        self.loco = loco
        self.notify = notify
        self.can_start = can_start
        self.admission = admission
        self.diagnostics_dir = Path(config.get('diagnostics_dir', '/tmp/g1-relative-motion'))
        self.command_lock = threading.RLock()
        # Serialize callers across the entire handoff, including the feedback
        # wait. The worker only takes command_lock and can therefore finish.
        self.handoff_lock = threading.RLock()
        self.active = None
        self.faulted = False
        self.closed = False
        self.speed = float(config.get("translation_speed", 0.3))
        self.yaw_speed = float(config.get("rotation_speed", 0.5))
        self.min_speed = float(config.get("min_translation_speed", 0.2))
        self.min_yaw_speed = float(config.get("min_rotation_speed", 0.3))
        if not (math.isfinite(self.speed) and 0 < self.speed <= 1 and
                math.isfinite(self.yaw_speed) and 0 < self.yaw_speed <= 2):
            raise ValueError("loco_relative_move speeds must be finite: translation (0,1], rotation (0,2]")
        if not math.isfinite(self.min_speed) or not 0 < self.min_speed <= self.speed:
            raise ValueError('Minimum translation speed must be positive and no higher than translation_speed')
        if not math.isfinite(self.min_yaw_speed) or not 0 < self.min_yaw_speed <= self.yaw_speed:
            raise ValueError('Minimum rotation speed must be positive and no higher than rotation_speed')
        self.safety_config = dict(config.get('safety', {}))
        MotionSafety(self.speed, self.yaw_speed, self.safety_config)  # Validate settings at startup.

    @property
    def busy(self):
        return self.active is not None

    def get_tools(self):
        return [{
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Closed-loop relative position and heading control using odometry only. "
                           "Translate to the target and settle first, then rotate and correct position drift. "
                           "Do not interrupt translation for ordinary heading error; correct heading at the endpoint. Never command rotation and translation together. "
                           "Pure turns correct their position drift only after rotation. "
                           "Calls SDK directly without SmartMotion sensor safety checks. "
                           "Local checks stop and latch on stale feedback, pose jumps, tilt and overspeed. "
                           "Targets use the starting body frame. Completion requires position within "
                           "8 cm and heading within 5 degrees, stable after stopping. "
                           "Nonzero translation components have a configurable minimum (default 0.2 m/s); "
                           "rotation has a configurable minimum (default 0.3 rad/s).",
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum": ["move", "stop_move"]},
                    "x": {"type": "number", "title": "X (m)", "default": 0,
                          "minimum": -self.MAX_DISTANCE, "maximum": self.MAX_DISTANCE,
                          "anyOf": [{"const": 0}, {"minimum": self.MIN_DISTANCE}, {"maximum": -self.MIN_DISTANCE}],
                          "description": "Relative longitudinal distance (m): 0 requests no displacement on this axis; otherwise absolute value must be 0.3–5 m. Positive forward, negative backward (same as loco vx)."},
                    "y": {"type": "number", "title": "Y (m, +left / -right)", "default": 0,
                          "minimum": -self.MAX_DISTANCE, "maximum": self.MAX_DISTANCE,
                          "anyOf": [{"const": 0}, {"minimum": self.MIN_DISTANCE}, {"maximum": -self.MIN_DISTANCE}],
                          "description": "Lateral displacement (m) in the starting body frame: positive left, negative right (same as loco vy). Zero requests no displacement on this axis; nonzero absolute values must be 0.3–5 m."},
                    "angle": {"type": "integer", "title": "Angle (°)", "enum": [0, 30, 60, 90, 180], "default": 0,
                              "description": "Final heading change from the starting heading (degrees); 0 holds starting heading."},
                    "direction": {"type": "string", "enum": ["left", "right"], "default": "left",
                                  "description": "Left uses positive loco vyaw, right uses negative vyaw."},
                },
                "x-action-params": {
                    "move": {"params": ["x", "y", "angle", "direction"],
                             "description": "Translate to relative x/y, stop, then rotate and correct drift. Pure turns rotate directly. Correct heading only at the position endpoint after settling. Maximum execution time 120 seconds."},
                    "stop_move": {"params": [], "description": "Cancel the entire pose goal and stop movement."},
                },
                "x-resource": "base",
                "x-completion": {"actions": ["move"], "timeout": self.TIMEOUT},
            },
        }]

    def plan(self, args):
        x, y = float(args.get("x", 0)), float(args.get("y", 0))
        angle = args.get("angle", 0)
        direction = args.get("direction", "left")
        if not all(math.isfinite(v) for v in (x, y)):
            raise ValueError("x and y must be finite")
        for axis, distance in (("x", x), ("y", y)):
            if distance != 0 and not self.MIN_DISTANCE <= abs(distance) <= self.MAX_DISTANCE:
                raise ValueError(f"{axis} must be 0 or have an absolute value between 0.3 and 5 m")
        # Canvas enum controls may serialize numeric selections as strings.
        if isinstance(angle, bool):
            raise ValueError("angle must be 0, 30, 60, 90 or 180 degrees")
        try:
            angle = float(angle)
        except (ValueError, TypeError, OverflowError):
            raise ValueError("angle must be 0, 30, 60, 90 or 180 degrees") from None
        if angle not in (0, 30, 60, 90, 180):
            raise ValueError("angle must be 0, 30, 60, 90 or 180 degrees")
        angle = int(angle)
        if direction not in ("left", "right"):
            raise ValueError("direction must be left or right")
        return {"x": x, "y": y, "yaw": math.radians(angle) * (1 if direction == "left" else -1)}

    def start(self):
        pass

    def stop(self):
        with self.handoff_lock:
            self._stop()

    def _stop(self):
        self.closed = True
        job = self.active
        if job or self.faulted:
            self.cancel_and_stop()
        if job:
            worker = job.get("worker")
            if worker and worker is not threading.current_thread():
                worker.join(12.)
        close = getattr(self.loco, "close", None)
        if close:
            close()

    def dispatch(self, action, args):
        with self.handoff_lock:
            return self._dispatch(action, args)

    def _dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            # Lifecycle state is separate from the physical stop acknowledgement.
            # Retain any stop failure internally to prevent another movement.
            self.cancel_and_stop()
            return {"state": "idle"}
        if action == "stop_move":
            return self.cancel_and_stop()
        if action != "move":
            return None
        try:
            target = self.plan(args)
        except (ValueError, TypeError, OverflowError) as exc:
            return {"error": str(exc)}
        with self.command_lock:
            if self.closed:
                return {"error": "loco_relative_move is shut down"}
            if self.faulted:
                return {"error": "Previous stop was not confirmed; retry stop_move first"}
            if self.busy:
                return {"error": "loco_relative_move is already running"}
            if target['x'] == 0 and target['y'] == 0 and target['yaw'] == 0:
                return {"ret": 0, "state": "idle", "noop": True}
            if self.can_start:
                try:
                    ownership = self.can_start()
                except Exception as exc:
                    return {"error": f"Could not verify base ownership: {exc}"}
                if not isinstance(ownership, dict) or ownership.get('available') is not True:
                    return {"error": "Base is busy or unconfirmed; stop existing loco/navigation movement first",
                            "owner_state": ownership}
            try:
                admission = self._check_admission()
            except SafetyViolation as exc:
                return {'error': str(exc), 'safety_reason': exc.reason, **exc.details}
            try:
                initial = self.loco.snapshot()
                safety = MotionSafety(self.speed, self.yaw_speed, self.safety_config)
                safety.check(initial, time.monotonic())
            except SafetyViolation as exc:
                return {"error": str(exc), "safety_reason": exc.reason}
            except Exception as exc:
                return {"error": str(exc)}
            job = dict(id=f"g1_point_{uuid4().hex[:8]}", cancel=threading.Event(),
                       motion_done=threading.Event(), stop_confirmed=False,
                       deadline=time.monotonic() + self.MAX_MOTION_SECONDS,
                       initial=initial, safety=safety, admission=admission)
            self.active = job
            try:
                job["worker"] = threading.Thread(target=self._run, args=(job, target), daemon=True)
                job["worker"].start()
            except Exception:
                self.active = None
                raise
        return {"ret": 0, "state": "moving", "action_id": job["id"], "target": target, "control": "closed_loop"}

    def _check_admission(self):
        try:
            state = self.admission() if self.admission else None
        except Exception as exc:
            raise SafetyViolation('locomotion_not_ready', f'Could not verify locomotion FSM: {exc}') from exc
        if not isinstance(state, dict) or state.get('admitted') is not True:
            raise SafetyViolation('locomotion_not_ready',
                'Fresh upright balanced locomotion FSM required; stand up before moving',
                {'locomotion_admission': state})
        return state

    def dispatch_base(self, plugin, action, args):
        with self.handoff_lock:
            return self._dispatch_base(plugin, action, args)

    def _dispatch_base(self, plugin, action, args):
        stopping = action in ("stop_move", "emergency_stop", "stop", "interrupt_motion") or (
            action == "switch_mode" and args.get("mode") == "emergency_stop")
        cancelled = None
        if stopping and (self.busy or self.faulted):
            cancelled = self.cancel_and_stop()
        querying = action.startswith("get_") or action in ("list", "list_tags", "list_maps", "check")
        with self.command_lock:
            if (self.busy or self.faulted) and not stopping and not querying:
                return {"error": "loco_relative_move is running; stop it before another base action"}
            result = plugin.dispatch(action, args)
            if cancelled is not None and cancelled.get("ret") != 0:
                return {**cancelled, "peer_result": result}
            return result

    def cancel_and_stop(self):
        with self.handoff_lock:
            return self._cancel_and_stop()

    def _cancel_and_stop(self):
        with self.command_lock:
            job = self.active
            if job:
                job["cancel"].set()
            result = self._send_stop()
            if not job:
                stopped = result['ret'] == 0 and self._confirm_stop()
                if stopped:
                    try:
                        pose = self.loco.snapshot()
                        safety = MotionSafety(self.speed, self.yaw_speed, self.safety_config)
                        safety.check(pose, time.monotonic())
                        if max(abs(pose['roll']), abs(pose['pitch'])) >= math.radians(safety.config['tilt_threshold_deg']):
                            stopped = False
                    except Exception:
                        stopped = False
                self.faulted = not stopped
                result.update(state='idle' if stopped else 'stop_failed',
                              stop_confirmed_by_feedback=stopped)
                if not stopped:
                    result.update(ret=None, error='Stop could not be confirmed from fresh feedback')
        if job:
            # Never wait under command_lock: the worker needs it to send zero
            # and publish its final result. motion_done means no more SDK writes.
            finished = job['motion_done'].wait(self.HANDOFF_TIMEOUT)
            with self.command_lock:
                stopped = finished and job['stop_confirmed'] and not self.faulted
                if not stopped:
                    self.faulted = True
                result.update(ret=0 if stopped else None,
                              state='idle' if stopped else 'stop_failed',
                              stop_confirmed_by_feedback=bool(finished and job['stop_confirmed']),
                              error=None if stopped else 'Cancelled motion did not release the base with a confirmed stop')
        return result

    def _save_diagnostics(self, action_id, status, result):
        """Keep the complete result locally and keep ACP below its 600-char cap."""
        if status != 'error' and not result.get('error'):
            result = {key: value for key, value in result.items() if key != 'motion_trace'}
        payload = json.dumps(dict(action_id=action_id, status=status, result=result),
                             ensure_ascii=True, allow_nan=False)
        summary = {key: result[key] for key in (
            'safety_reason', 'stop_confirmed_by_feedback') if key in result}
        if 'error' in result:
            summary['error'] = str(result['error'])[:120]
        for key in ('position_error_m', 'angle_error_deg', 'phase'):
            if key in result:
                value = result[key]
                summary[key] = round(value, 4) if isinstance(value, float) else value
        temporary = None
        try:
            self.diagnostics_dir.mkdir(parents=True, exist_ok=True)
            destination = self.diagnostics_dir / (action_id + '.json')
            temporary = destination.with_suffix('.' + uuid4().hex + '.tmp')
            with temporary.open('x', encoding='utf-8') as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, destination)
            summary['diagnostics_file'] = str(destination)
            # An unusually long configured path must not break the ACP limit.
            if len(json.dumps(summary, ensure_ascii=True)) > 560:
                summary = dict(diagnostics_file=str(destination))
            if len(json.dumps(summary, ensure_ascii=True)) > 560:
                summary = dict(diagnostics='See container log for file path', action_id=action_id)
            if status == 'error' or result.get('error'):
                print(f'[RelativeMotion] action_id={action_id} status={status} error={summary.get("error")} diagnostics_file={destination}', flush=True)
        except Exception as exc:
            if temporary is not None:
                try:
                    temporary.unlink(missing_ok=True)
                except OSError:
                    pass
            # Numbered, small records allow recovery via docker logs on disk failure.
            chunks = [payload[i:i+700] for i in range(0, len(payload), 700)]
            print(f'[RelativeMotion] action_id={action_id} diagnostics_write_failed={str(exc)[:160]} chunks={len(chunks)}', flush=True)
            for index, chunk in enumerate(chunks):
                print(f'[RelativeMotionData] {action_id} {index+1}/{len(chunks)} {chunk}', flush=True)
            summary['diagnostics_error'] = 'File write failed; full numbered data in docker logs'
        return summary

    def _send_stop(self):
        trace = self.active.get('trace') if self.active else None
        record = trace.add('stop_send', command=[0., 0., 0.]) if trace else None
        started = time.monotonic()
        try:
            ret = self.loco.command(0., 0., 0.)
            if record is not None:
                record.update(send_ret=ret, send_elapsed_s=time.monotonic() - started)
            return {"ret": ret, "state": "stop_pending" if ret == 0 else "stop_failed",
                    "stop_acknowledged": False, "error": None if ret == 0 else f"Stop send failed: {ret}"}
        except Exception as exc:
            if record is not None:
                record.update(send_error=str(exc), send_elapsed_s=time.monotonic() - started)
            return {"ret": None, "state": "stop_failed", "error": str(exc), "stop_acknowledged": False}

    def _confirm_stop(self):
        deadline, stable = time.monotonic() + 3., None
        while time.monotonic() < deadline:
            with self.command_lock:
                if self._send_stop()['ret'] != 0:
                    return False
            try:
                pose = self.loco.snapshot()
            except Exception as exc:
                if self.active and self.active.get('trace'):
                    self.active['trace'].add('stop_feedback_error', error=str(exc))
                return False
            if self.active and self.active.get('trace'):
                self.active['trace'].add('stop_feedback', pose=dict(pose), stable_since_monotonic_s=stable)
            if (pose['speed'] <= RelativeController.STOP_SPEED
                    and pose['angular_speed'] <= RelativeController.STOP_YAW_RATE):
                if stable is None:
                    stable = time.monotonic()
                if pose['odom_stamp'] - stable >= .5:
                    return True
            else:
                stable = None
            time.sleep(.05)
        return False

    def _run(self, job, target):
        status, result = "error", {}
        controller = RelativeController(job['initial'], target, self.speed, self.yaw_speed,
                                        time.monotonic(), min_speed=self.min_speed, min_yaw_speed=self.min_yaw_speed)
        trace = MotionTrace(job['initial'], target, dict(
            translation_speed=self.speed, rotation_speed=self.yaw_speed,
            min_translation_speed=self.min_speed, min_rotation_speed=self.min_yaw_speed,
            safety=dict(job['safety'].config), deadline_monotonic_s=job['deadline'],
            command_duration_s=getattr(self.loco, 'COMMAND_DURATION', None)))
        job['trace'] = trace
        trace.add('start', absolute_target=dict(x=controller.x, y=controller.y, yaw=controller.yaw),
                  locomotion_admission=job['admission'])
        tick, previous_tick = None, None
        last_velocity = (0., 0., 0.)
        try:
            while True:
                with self.command_lock:
                    if job['cancel'].is_set():
                        break
                    now = time.monotonic()
                    tick = trace.add('control', phase=controller.phase,
                                     loop_interval_s=None if previous_tick is None else now - previous_tick,
                                     previous_command=list(last_velocity))
                    previous_tick = now
                    if now >= job['deadline']:
                        raise TimeoutError('Relative movement deadline expired')
                    tick['locomotion_admission'] = self._check_admission()
                    snapshot_started = time.monotonic()
                    pose = self.loco.snapshot()
                    observed = time.monotonic()
                    tick.update(pose=dict(pose), snapshot_elapsed_s=observed - snapshot_started,
                                odom_age_s=observed - pose['odom_stamp'], feedback_source='loco/state')
                    if all(math.isfinite(pose.get(key, float('nan'))) for key in ('x', 'y', 'yaw')):
                        controller.update_error(pose)
                        tick['error'] = dict(controller.error)
                        result['error_sample'] = dict(basis='latest_control_feedback_not_final_pose',
                            odom_receive_monotonic_s=pose['odom_stamp'])
                    try:
                        job['safety'].check(pose, time.monotonic())
                    except SafetyViolation as exc:
                        if exc.reason == 'attitude_jump':
                            controller.update_error(pose)
                            result['error_sample'] = {
                                'basis': 'safety_trigger_feedback_not_final_pose',
                                'odom_receive_monotonic_s': pose['odom_stamp'],
                            }
                        raise
                    velocity, done = controller.step(pose, now)
                    tick.update(command=list(velocity), phase_after_step=controller.phase,
                                error=dict(controller.error), done=done)
                    send_started = time.monotonic()
                    try:
                        ret = self.loco.command(*velocity)
                        tick['send_ret'] = ret
                    finally:
                        tick['send_elapsed_s'] = time.monotonic() - send_started
                    last_velocity = velocity
                    if ret != 0:
                        raise RuntimeError(f'Velocity send failed: {ret}')
                    if done:
                        status = 'completed'
                        break
                job['cancel'].wait(.05)
            if job['cancel'].is_set():
                status = 'cancelled'
        except Exception as exc:
            trace.add('exception', error=str(exc), exception_type=type(exc).__name__)
            if tick is not None:
                tick['exception'] = str(exc)
            result['error'] = str(exc)
            job['cancel'].set()
            if isinstance(exc, SafetyViolation):
                result['safety_reason'] = exc.reason
                result.update(exc.details)
                if 'odom_heading_jump' in result:
                    result['odom_heading_jump']['commanded_vyaw_rad_s'] = last_velocity[2]
        finally:
            if status != 'completed':
                stopped = self._confirm_stop()
                if not stopped:
                    status = 'error'
                    result.setdefault('error', 'Stop could not be confirmed from fresh feedback')
            else:
                stopped = True  # Controller already observed stable zero-speed feedback.
            result.update(controller.error)
            result['stop_confirmed_by_feedback'] = stopped
            trace.add('terminal', status=status, stop_confirmed_by_feedback=stopped, error=result.get('error'))
            if status == 'error' or result.get('error'):
                result['motion_trace'] = trace.export(full=True)
            result['target'] = dict(x=controller.x, y=controller.y, yaw=controller.yaw)
            with self.command_lock:
                self.faulted = not stopped or 'safety_reason' in result
                job['stop_confirmed'] = stopped
                if self.active is job:
                    self.active = None
                job['motion_done'].set()
            summary = self._save_diagnostics(job['id'], status, result)
            self.notify(job['id'], status, summary, tool=self.PREFIX)


class PointBackend:
    """No-reply SDK writes: no request futures and no response-wait threads."""
    COMMAND_DURATION = .3

    def __init__(self, client, feedback, close=None):
        self.client = client
        self.feedback = feedback
        self._close = close

    def snapshot(self):
        return self.feedback.snapshot()

    def command(self, vx, vy, vyaw):
        import json
        from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_SET_VELOCITY
        parameter = json.dumps({'velocity': [vx, vy, vyaw], 'duration': self.COMMAND_DURATION})
        return self.client._CallNoReply(ROBOT_API_ID_LOCO_SET_VELOCITY, parameter)

    def close(self):
        if self._close:
            self._close()
