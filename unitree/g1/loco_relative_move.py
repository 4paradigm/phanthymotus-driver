"""Sequential relative movement through the shared SmartMotion safety owner."""
import math
import threading
import time
from uuid import uuid4


class LocoRelativeMovePlugin:
    PREFIX = "loco_relative_move"
    TIMEOUT = 300
    MAX_MOTION_SECONDS = 120
    # Refined from compensated turns observed at ~80 and ~175 degrees.
    # Calibrated only at 0.5 rad/s; smaller angles keep theoretical duration.
    ROTATION_TIME_FACTORS = {90: (90. / 75.) * (90. / 80.),
                             180: (180. / 135.) * (180. / 175.)}
    # Rough observed travel was 60% of the requested displacement at 0.3 m/s.
    # Lateral travel then overshot by ~20%; shorten only its compensated time.
    TRANSLATION_TIME_FACTORS = (1. / 0.6, 1. / 0.6 / 1.2)
    MIN_DISTANCE = 0.3
    MAX_DISTANCE = 5.

    def __init__(self, config, loco, notify, prepare=None):
        self.loco = loco
        self.notify = notify
        self.prepare = prepare
        self.command_lock = threading.RLock()
        self.active = None
        self.faulted = False
        self.closed = False
        self.speed = float(config.get("translation_speed", 0.3))
        self.yaw_speed = float(config.get("rotation_speed", 0.5))
        if not (math.isfinite(self.speed) and 0 < self.speed <= 1 and
                math.isfinite(self.yaw_speed) and 0 < self.yaw_speed <= 2):
            raise ValueError("loco_relative_move speeds must be finite: translation (0,1], rotation (0,2]")

    @property
    def busy(self):
        return self.active is not None

    def get_tools(self):
        return [{
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Sequential relative movement: straight along x, sideways along y, then rotate. "
                           "Uses loco velocity/time control; distances and angles are approximate, "
                           "without position feedback. At 0.3 m/s, translation uses empirical time compensation; "
                           "at 0.5 rad/s, 90/180 degree turns use empirical time compensation. "
                           "Axes refer to the starting body heading.",
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum": ["move", "stop_move"]},
                    "x": {"type": "number", "title": "X (m)", "default": 0,
                          "minimum": -self.MAX_DISTANCE, "maximum": self.MAX_DISTANCE,
                          "anyOf": [{"const": 0}, {"minimum": self.MIN_DISTANCE}, {"maximum": -self.MIN_DISTANCE}],
                          "description": "Relative longitudinal distance (m): 0 skips this stage; otherwise absolute value must be 0.3–5 m. Positive forward, negative backward (same as loco vx)."},
                    "y": {"type": "number", "title": "Y (m)", "default": 0,
                          "minimum": -self.MAX_DISTANCE, "maximum": self.MAX_DISTANCE,
                          "anyOf": [{"const": 0}, {"minimum": self.MIN_DISTANCE}, {"maximum": -self.MIN_DISTANCE}],
                          "description": "Relative lateral distance (m): 0 skips this stage; otherwise absolute value must be 0.3–5 m. Positive left, negative right (same as loco vy)."},
                    "angle": {"type": "integer", "title": "Angle (°)", "enum": [0, 30, 60, 90, 180], "default": 0,
                              "description": "Final rotation in degrees; 0 skips rotation."},
                    "direction": {"type": "string", "enum": ["left", "right"], "default": "left",
                                  "description": "Left uses positive loco vyaw, right uses negative vyaw."},
                },
                "x-action-params": {
                    "move": {"params": ["x", "y", "angle", "direction"],
                             "description": "Execute x, y, rotation in order, stopping between stages. Maximum planned motion time 120 seconds."},
                    "stop_move": {"params": [], "description": "Cancel remaining stages and stop movement."},
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
        stages = []
        for name, distance, axis in (("straight", x, 0), ("lateral", y, 1)):
            if distance:
                velocity = [0., 0., 0.]
                velocity[axis] = math.copysign(self.speed, distance)
                stages.append(dict(stage=name, vx=velocity[0], vy=velocity[1], vyaw=0.,
                                   duration=abs(distance) / self.speed *
                                   (self.TRANSLATION_TIME_FACTORS[axis] if self.speed == 0.3 else 1.)))
        if angle:
            factor = self.ROTATION_TIME_FACTORS.get(angle, 1.) if self.yaw_speed == 0.5 else 1.
            stages.append(dict(stage="rotation", vx=0., vy=0.,
                               vyaw=self.yaw_speed if direction == "left" else -self.yaw_speed,
                               duration=math.radians(angle) / self.yaw_speed * factor))
        if sum(s["duration"] for s in stages) > self.MAX_MOTION_SECONDS:
            raise ValueError("Planned motion exceeds 120 seconds; reduce distances")
        return stages

    def start(self):
        pass

    def stop(self):
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
            return {"error": f"Unknown loco_relative_move action: {action}"}
        try:
            stages = self.plan(args)
        except (ValueError, TypeError, OverflowError) as exc:
            return {"error": str(exc)}
        if not stages:
            return {"ret": 0, "state": "idle", "stages": []}
        with self.command_lock:
            if self.closed:
                return {"error": "loco_relative_move is shut down"}
            if self.faulted:
                return {"error": "Previous stop was not confirmed; retry stop_move first"}
            if self.busy:
                return {"error": "loco_relative_move is already running"}
            job = dict(id=f"g1_point_{uuid4().hex[:8]}", cancel=threading.Event(),
                       stop_done=threading.Event(), stop_result=None,
                       deadline=time.monotonic() + self.TIMEOUT - 15)
            self.active = job
            try:
                job["worker"] = threading.Thread(target=self._run, args=(job, stages), daemon=True)
                job["worker"].start()
            except Exception:
                self.active = None
                raise
        return {"ret": 0, "state": "moving", "action_id": job["id"], "stages": stages}

    def dispatch_base(self, plugin, action, args):
        stopping = action in ("stop_move", "emergency_stop", "stop") or (
            action == "switch_mode" and args.get("mode") == "emergency_stop")
        if stopping and self.busy:
            self.cancel_and_stop()
        querying = action.startswith("get_") or action in ("list", "list_tags", "list_maps", "check")
        with self.command_lock:
            if (self.busy or self.faulted) and not stopping and not querying:
                return {"error": "loco_relative_move is running; stop it before another base action"}
            return plugin.dispatch(action, args)

    def cancel_and_stop(self):
        job = self.active
        if job:
            job["cancel"].set()
        # Order stop after an in-progress segment submission. Response waits
        # can delay this explicit stop; timed segment stops remain independent.
        with self.command_lock:
            job = self.active
            if job:
                job["cancel"].set()
            try:
                result = self.loco._stop_motion()
            except Exception as exc:
                result = {"ret": None, "error": str(exc), "state": "stop_failed"}
            self.faulted = result.get("ret") != 0
            if not job and not self.faulted:
                finish = getattr(self.loco, "finish_sequence", None)
                if finish:
                    try:
                        finish()
                    except Exception as exc:
                        self.faulted = True
                        result = {"ret": None, "error": str(exc), "state": "stop_failed"}
            if job:
                job["stop_result"] = result
                job["stop_done"].set()
            return result

    def _run(self, job, stages):
        status, result = "completed", {"completed_stages": []}
        try:
            if self.prepare:
                with self.command_lock:
                    if not job["cancel"].is_set():
                        prepared = self.prepare()
                        if prepared.get("ret") != 0:
                            raise RuntimeError(prepared.get("error") or "Could not stop preceding loco movement")
            for index, stage in enumerate(stages):
                with self.command_lock:
                    if job["cancel"].is_set():
                        break
                    if time.monotonic() >= job["deadline"]:
                        raise TimeoutError("loco_relative_move deadline expired")
                    segment_id = f"{job['id']}_{index}"
                    begin = getattr(self.loco, "_begin_motion", self.loco._start_motion)
                    pending = begin(stage["vx"], stage["vy"], stage["vyaw"],
                                    stage["duration"], segment_id)
                reply = pending() if callable(pending) else pending
                if reply.get("error") or reply.get("ret") != 0:
                    raise RuntimeError(reply.get("error") or f"Move failed: {reply.get('ret')}")
                while not job["cancel"].is_set():
                    if time.monotonic() >= job["deadline"]:
                        raise TimeoutError("loco_relative_move deadline expired")
                    outcome = self.loco._motion_result(segment_id)
                    if outcome.get("error"):
                        raise RuntimeError(outcome["error"])
                    terminal = outcome.get("status")
                    if terminal in ("completed", "cancelled", "error"):
                        if terminal != "completed":
                            status = terminal
                            result["segment_result"] = outcome.get("result", {})
                        break
                    job["cancel"].wait(0.1)
                if job["cancel"].is_set() or status != "completed":
                    break
                result["completed_stages"].append(stage["stage"])
            if job["cancel"].is_set():
                if not job["stop_done"].wait(max(0., job["deadline"] - time.monotonic())):
                    raise TimeoutError("Cancellation stop was not confirmed")
                stop = job["stop_result"]
                status = "cancelled" if stop.get("ret") == 0 else "error"
                result["stop_result"] = stop
        except Exception as exc:
            status = "error"
            result["error"] = str(exc)
            result["stop_result"] = self.cancel_and_stop()
        finally:
            finish = getattr(self.loco, "finish_sequence", None)
            if finish:
                try:
                    finish()
                except Exception as exc:
                    self.faulted = True
                    status = "error"
                    result["error"] = f"Sequence release failed: {exc}"
            with self.command_lock:
                if self.active is job:
                    self.active = None
            self.notify(job["id"], status, result, tool=self.PREFIX)


class PointBackend:
    """Point sequence adapter; all velocity and stop RPCs belong to SmartMotion."""
    def __init__(self, smart_motion):
        if smart_motion is None:
            raise ValueError("loco_relative_move requires the SmartMotion safety harness")
        self.smart_motion = smart_motion
        self.sequence_id = None

    def _begin_motion(self, vx, vy, vyaw, duration, action_id):
        self.sequence_id = action_id.rsplit("_", 1)[0]
        return self.smart_motion.point_move(vx, vy, vyaw, duration, action_id, self.sequence_id)

    def _start_motion(self, vx, vy, vyaw, duration, action_id):
        return self._begin_motion(vx, vy, vyaw, duration, action_id)

    def _motion_result(self, action_id):
        return self.smart_motion.point_result(action_id, self.sequence_id)

    def _stop_motion(self):
        return self.smart_motion.point_stop(self.sequence_id)

    def finish_sequence(self):
        if self.sequence_id is not None:
            result = self.smart_motion.point_finish(self.sequence_id)
            if result.get("ret") != 0:
                raise RuntimeError(result.get("error") or "Point ownership release failed")
            self.sequence_id = None

    def close(self):
        # A failed stop deliberately retains ownership in the safety process.
        # Shutdown must still allow the bundle to stop that process afterwards.
        try:
            self.finish_sequence()
        except RuntimeError:
            pass
