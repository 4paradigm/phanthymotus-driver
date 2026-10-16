"""Closed-loop relative pose movement through a direct, no-reply SDK client."""
import math
import threading
import time
from uuid import uuid4

try:
    from .relative_feedback import RelativeController, MotionSafety, SafetyViolation
except ImportError:
    from relative_feedback import RelativeController, MotionSafety, SafetyViolation


class LocoRelativeMovePlugin:
    PREFIX = "loco_relative_move"
    TIMEOUT = 300
    MAX_MOTION_SECONDS = 120
    MIN_DISTANCE = 0.3
    MAX_DISTANCE = 5.
    HANDOFF_TIMEOUT = 4.

    def __init__(self, config, loco, notify, can_start=None):
        self.loco = loco
        self.notify = notify
        self.can_start = can_start
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
        if not (math.isfinite(self.speed) and 0 < self.speed <= 1 and
                math.isfinite(self.yaw_speed) and 0 < self.yaw_speed <= 2):
            raise ValueError("loco_relative_move speeds must be finite: translation (0,1], rotation (0,2]")
        if not math.isfinite(self.min_speed) or not 0 < self.min_speed <= self.speed:
            raise ValueError('Minimum translation speed must be positive and no higher than translation_speed')
        self.safety_config = dict(config.get('safety', {}))
        MotionSafety(self.speed, self.yaw_speed, self.safety_config)  # Validate settings at startup.

    @property
    def busy(self):
        return self.active is not None

    def get_tools(self):
        return [{
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Closed-loop relative position and heading control using odometry and IMU. "
                           "Translate to the target and settle first, then rotate and correct position drift. "
                           "Never command rotation and translation together. "
                           "Pure turns correct their position drift only after rotation. "
                           "Calls SDK directly without SmartMotion sensor safety checks. "
                           "Local checks stop and latch on stale feedback, pose jumps, tilt and overspeed. "
                           "Targets use the starting body frame. Completion requires position within "
                           "5 cm and heading within 3 degrees, stable after stopping. "
                           "Nonzero translation components have a configurable minimum (default 0.2 m/s).",
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum": ["move", "stop_move"]},
                    "x": {"type": "number", "title": "X (m)", "default": 0,
                          "minimum": -self.MAX_DISTANCE, "maximum": self.MAX_DISTANCE,
                          "anyOf": [{"const": 0}, {"minimum": self.MIN_DISTANCE}, {"maximum": -self.MIN_DISTANCE}],
                          "description": "Relative longitudinal distance (m): 0 requests no displacement on this axis; otherwise absolute value must be 0.3–5 m. Positive forward, negative backward (same as loco vx)."},
                    "y": {"type": "number", "title": "Y (m，正向左 / 负向右)", "default": 0,
                          "minimum": -self.MAX_DISTANCE, "maximum": self.MAX_DISTANCE,
                          "anyOf": [{"const": 0}, {"minimum": self.MIN_DISTANCE}, {"maximum": -self.MIN_DISTANCE}],
                          "description": "横移距离（m）：相对于机器人运动开始时的朝向，正值向左、负值向右（同 loco vy）；0 表示该轴目标位移为零，非零绝对值为 0.3–5 m。"},
                    "angle": {"type": "integer", "title": "Angle (°)", "enum": [0, 30, 60, 90, 180], "default": 0,
                              "description": "Final heading change from the starting heading (degrees); 0 holds starting heading."},
                    "direction": {"type": "string", "enum": ["left", "right"], "default": "left",
                                  "description": "Left uses positive loco vyaw, right uses negative vyaw."},
                },
                "x-action-params": {
                    "move": {"params": ["x", "y", "angle", "direction"],
                             "description": "Translate to relative x/y, stop, then rotate and correct drift. Pure turns rotate directly. Stop before any heading correction. Maximum execution time 120 seconds."},
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
            if self.can_start:
                try:
                    ownership = self.can_start()
                except Exception as exc:
                    return {"error": f"Could not verify base ownership: {exc}"}
                if not isinstance(ownership, dict) or ownership.get('available') is not True:
                    return {"error": "Base is busy or unconfirmed; stop existing loco/navigation movement first",
                            "owner_state": ownership}
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
                       initial=initial, safety=safety)
            self.active = job
            try:
                job["worker"] = threading.Thread(target=self._run, args=(job, target), daemon=True)
                job["worker"].start()
            except Exception:
                self.active = None
                raise
        return {"ret": 0, "state": "moving", "action_id": job["id"], "target": target, "control": "closed_loop"}

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

    def _send_stop(self):
        try:
            ret = self.loco.command(0., 0., 0.)
            return {"ret": ret, "state": "stop_pending" if ret == 0 else "stop_failed",
                    "stop_acknowledged": False, "error": None if ret == 0 else f"Stop send failed: {ret}"}
        except Exception as exc:
            return {"ret": None, "state": "stop_failed", "error": str(exc), "stop_acknowledged": False}

    def _confirm_stop(self):
        deadline, stable = time.monotonic() + 3., None
        while time.monotonic() < deadline:
            with self.command_lock:
                if self._send_stop()['ret'] != 0:
                    return False
            try:
                pose = self.loco.snapshot()
            except Exception:
                return False
            if pose['speed'] <= .03 and pose['angular_speed'] <= .03:
                if stable is None:
                    stable = time.monotonic()
                if min(pose['odom_stamp'], pose['imu_stamp']) - stable >= .5:
                    return True
            else:
                stable = None
            time.sleep(.05)
        return False

    def _run(self, job, target):
        status, result = "error", {}
        controller = RelativeController(job['initial'], target, self.speed, self.yaw_speed,
                                        time.monotonic(), min_speed=self.min_speed)
        try:
            while True:
                with self.command_lock:
                    if job['cancel'].is_set():
                        break
                    now = time.monotonic()
                    if now >= job['deadline']:
                        raise TimeoutError('Relative movement deadline expired')
                    pose = self.loco.snapshot()
                    job['safety'].check(pose, time.monotonic())
                    velocity, done = controller.step(pose, now)
                    ret = self.loco.command(*velocity)
                    if ret != 0:
                        raise RuntimeError(f'Velocity send failed: {ret}')
                    if done:
                        status = 'completed'
                        break
                job['cancel'].wait(.05)
            if job['cancel'].is_set():
                status = 'cancelled'
        except Exception as exc:
            result['error'] = str(exc)
            job['cancel'].set()
            if isinstance(exc, SafetyViolation):
                result['safety_reason'] = exc.reason
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
            result['target'] = dict(x=controller.x, y=controller.y, yaw=controller.yaw)
            with self.command_lock:
                self.faulted = not stopped or 'safety_reason' in result
                job['stop_confirmed'] = stopped
                if self.active is job:
                    self.active = None
                job['motion_done'].set()
            self.notify(job['id'], status, result, tool=self.PREFIX)


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
