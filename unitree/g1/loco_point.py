"""Sequential relative movement with its own deadline controller and velocity RPC path."""
import math
import threading
import time
from uuid import uuid4


class LocoPointPlugin:
    PREFIX = "loco_point"
    TIMEOUT = 300
    MAX_MOTION_SECONDS = 120

    def __init__(self, config, loco, notify, prepare=None):
        self.loco = loco
        self.notify = notify
        self.prepare = prepare
        self.command_lock = threading.RLock()
        self.active = None
        self.speed = float(config.get("translation_speed", 0.3))
        self.yaw_speed = float(config.get("rotation_speed", 0.5))
        if not (math.isfinite(self.speed) and 0 < self.speed <= 1 and
                math.isfinite(self.yaw_speed) and 0 < self.yaw_speed <= 2):
            raise ValueError("loco_point speeds must be finite: translation (0,1], rotation (0,2]")

    @property
    def busy(self):
        return self.active is not None

    def get_tools(self):
        return [{
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Relative movement: straight along x, sideways along y, then rotate. "
                           "Uses loco velocity/time control; distances and angles are approximate, "
                           "without feedback or compensation. Axes refer to the starting body heading.",
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum": ["move", "stop_move"]},
                    "x": {"type": "number", "default": 0,
                          "description": "Relative longitudinal distance in metres: positive forward, negative backward (same as loco vx)."},
                    "y": {"type": "number", "default": 0,
                          "description": "Relative lateral distance in metres: positive left, negative right (same as loco vy)."},
                    "angle": {"type": "integer", "enum": [0, 30, 45, 60, 90, 180], "default": 0,
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
        if isinstance(angle, bool) or angle not in (0, 30, 45, 60, 90, 180):
            raise ValueError("angle must be 0, 30, 45, 60, 90 or 180 degrees")
        if direction not in ("left", "right"):
            raise ValueError("direction must be left or right")
        stages = []
        for name, distance, axis in (("straight", x, 0), ("lateral", y, 1)):
            if distance:
                velocity = [0., 0., 0.]
                velocity[axis] = math.copysign(self.speed, distance)
                stages.append(dict(stage=name, vx=velocity[0], vy=velocity[1], vyaw=0.,
                                   duration=abs(distance) / self.speed))
        if angle:
            stages.append(dict(stage="rotation", vx=0., vy=0.,
                               vyaw=self.yaw_speed if direction == "left" else -self.yaw_speed,
                               duration=math.radians(angle) / self.yaw_speed))
        if sum(s["duration"] for s in stages) > self.MAX_MOTION_SECONDS:
            raise ValueError("Planned motion exceeds 120 seconds; reduce distances")
        return stages

    def start(self):
        pass

    def stop(self):
        if self.busy:
            self.cancel_and_stop()

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action in ("stop", "stop_move"):
            return self.cancel_and_stop()
        if action != "move":
            return {"error": f"Unknown loco_point action: {action}"}
        try:
            stages = self.plan(args)
        except (ValueError, TypeError, OverflowError) as exc:
            return {"error": str(exc)}
        if not stages:
            return {"ret": 0, "state": "idle", "stages": []}
        with self.command_lock:
            if self.busy:
                return {"error": "loco_point is already running"}
            job = dict(id=f"g1_point_{uuid4().hex[:8]}", cancel=threading.Event(),
                       stop_done=threading.Event(), stop_result=None,
                       deadline=time.monotonic() + self.TIMEOUT - 15)
            self.active = job
            try:
                threading.Thread(target=self._run, args=(job, stages), daemon=True).start()
            except Exception:
                self.active = None
                raise
        return {"ret": 0, "state": "moving", "action_id": job["id"], "stages": stages}

    def dispatch_base(self, plugin, action, args):
        stopping = action in ("stop_move", "emergency_stop", "stop") or (
            action == "switch_mode" and args.get("mode") == "emergency_stop")
        if stopping and self.busy:
            self.cancel_and_stop()
        with self.command_lock:
            if self.busy and not stopping:
                return {"error": "loco_point is running; stop it before another base action"}
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
                            raise RuntimeError("Could not stop preceding loco movement")
            for index, stage in enumerate(stages):
                with self.command_lock:
                    if job["cancel"].is_set():
                        break
                    if time.monotonic() >= job["deadline"]:
                        raise TimeoutError("loco_point deadline expired")
                    segment_id = f"{job['id']}_{index}"
                    reply = self.loco._start_motion(stage["vx"], stage["vy"], stage["vyaw"],
                                                    stage["duration"], segment_id)
                if reply.get("error") or reply.get("ret") != 0:
                    raise RuntimeError(reply.get("error") or f"Move failed: {reply.get('ret')}")
                while not job["cancel"].is_set():
                    if time.monotonic() >= job["deadline"]:
                        raise TimeoutError("loco_point deadline expired")
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
            with self.command_lock:
                if self.active is job:
                    self.active = None
            self.notify(job["id"], status, result, tool=self.PREFIX)


class PointBackend:
    """Card-owned controller; no dependency on LocoPlugin or its revisions."""
    def __init__(self):
        from point_motion import PointMotion
        from point_rpc import PointRpc
        self.motion = PointMotion(PointRpc())

    def _start_motion(self, vx, vy, vyaw, duration, action_id):
        return self.motion.start(vx, vy, vyaw, duration, action_id)

    def _motion_result(self, action_id):
        return self.motion.get_result(action_id)

    def _stop_motion(self):
        return self.motion.stop()
