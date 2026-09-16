"""Cancelable, reusable AS2W velocity-motion tools."""

import json
import math
import os
from pathlib import Path
import re
import threading
import time
import uuid


_CONTROL_PERIOD = 0.1
_MAX_TRAJECTORY_SECONDS = 60.0
_MAX_RECORDING_SECONDS = 300.0
_MAX_RECORDING_FRAMES = 6000
_DEFAULT_RECORDINGS_DIR = "/opt/phanthy-motus/data/as2w-motion-recordings"


def _finite(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _bounded(value, name, minimum, maximum):
    result = _finite(value, name)
    if result < minimum or result > maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return result


def _velocity(args):
    return (
        _bounded(args.get("vx", 0), "vx", -1.5, 1.5),
        _bounded(args.get("vy", 0), "vy", -1.0, 1.0),
        _bounded(args.get("vyaw", 0), "vyaw", -2.0, 2.0),
    )


class MotionExecutor:
    """Own the one background SportClient velocity stream allowed at a time."""

    def __init__(self, proxy):
        self._proxy = proxy
        self._lock = threading.RLock()
        self._thread = None
        self._stop_event = None
        self._owner = None
        self._action_id = None
        self._started_at = None
        self._last_result = None

    def start(self, owner, worker):
        stopped = self.stop()
        if stopped.get("still_stopping"):
            return {"error": "previous motion has not stopped", "code": "MOTION_BUSY"}
        stop_event = threading.Event()
        action_id = f"as2w_{owner}_{uuid.uuid4().hex[:12]}"

        def run():
            try:
                result = worker(stop_event) or {"state": "completed"}
            except Exception as exc:
                result = {"state": "failed", "error": str(exc)}
            finally:
                stop_ret = self._proxy.StopMove()
                with self._lock:
                    if self._action_id == action_id:
                        self._last_result = {**result, "stop_ret": stop_ret}
                        self._thread = None
                        self._stop_event = None
                        self._owner = None
                        self._action_id = None

        thread = threading.Thread(target=run, daemon=True, name=action_id)
        with self._lock:
            self._thread = thread
            self._stop_event = stop_event
            self._owner = owner
            self._action_id = action_id
            self._started_at = time.monotonic()
            self._last_result = None
        thread.start()
        return {"state": "running", "action_id": action_id, "owner": owner}

    def stop(self, owner=None):
        with self._lock:
            if owner is not None and self._owner not in (None, owner):
                return {"state": "idle", "active_owner": self._owner}
            event, thread, active_owner = self._stop_event, self._thread, self._owner
            if event is not None:
                event.set()
        stop_ret = self._proxy.StopMove()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        alive = bool(thread and thread.is_alive())
        return {
            "state": "stopping" if alive else "idle",
            "stopped_owner": active_owner,
            "stop_ret": stop_ret,
            "still_stopping": alive,
        }

    def status(self, owner=None):
        with self._lock:
            active = self._thread is not None and self._thread.is_alive()
            matches = owner is None or self._owner == owner
            result = {
                "state": "running" if active and matches else "idle",
                "active_owner": self._owner if active else None,
                "action_id": self._action_id if active else None,
                "elapsed": round(time.monotonic() - self._started_at, 3)
                if active and self._started_at is not None else 0.0,
                "last_result": self._last_result,
            }
        return result


class TrajectoryMotionPlugin:
    PREFIX = "trajectory_motion"

    def __init__(self, config, proxy, executor, stop_loco):
        self._proxy = proxy
        self._executor = executor
        self._stop_loco = stop_loco

    def get_tool(self):
        actions = ["circle", "figure_eight", "slalom", "status", "stop"]
        return {
            "name": self.PREFIX,
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "AS2W parameterized velocity trajectories: circle, figure-eight, and slalom. "
                "Every trajectory is time-limited and cancelable."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": actions},
                    "radius": {"type": "number", "minimum": 0.3, "maximum": 3.0},
                    "speed": {"type": "number", "minimum": 0.05, "maximum": 0.6},
                    "loops": {"type": "number", "minimum": 0.25, "maximum": 3.0},
                    "direction": {"type": "string", "enum": ["left", "right"]},
                    "duration": {"type": "number", "minimum": 1.0, "maximum": 30.0},
                    "yaw_amplitude": {"type": "number", "minimum": 0.1, "maximum": 1.2},
                    "period": {"type": "number", "minimum": 1.0, "maximum": 10.0},
                },
                "required": ["action"],
                "x-is-dangerous": True,
                "x-action-params": {
                    "circle": {"params": ["radius", "speed", "loops", "direction"]},
                    "figure_eight": {"params": ["radius", "speed", "loops", "direction"]},
                    "slalom": {"params": ["speed", "duration", "yaw_amplitude", "period"]},
                    "status": {"params": []},
                    "stop": {"params": []},
                },
                "x-completion": {"actions": ["circle", "figure_eight", "slalom"], "timeout": 70},
                "x-hooks": {"on_interrupt_motion": {"action": "stop"}},
            },
        }

    def start(self):
        pass

    def stop(self):
        return self._executor.stop(self.PREFIX)

    def _run(self, duration, command):
        def worker(stop_event):
            started = time.monotonic()
            samples = 0
            while not stop_event.is_set():
                elapsed = time.monotonic() - started
                if elapsed >= duration:
                    break
                vx, vy, vyaw = command(elapsed)
                ret = self._proxy.Move(vx, vy, vyaw)
                if ret != 0:
                    return {"state": "failed", "ret": ret, "samples": samples}
                samples += 1
                stop_event.wait(_CONTROL_PERIOD)
            return {"state": "cancelled" if stop_event.is_set() else "completed",
                    "samples": samples, "duration": round(time.monotonic() - started, 3)}

        self._stop_loco()
        return self._executor.start(self.PREFIX, worker)

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready", **self._executor.status(self.PREFIX)}
        if action == "status":
            return self._executor.status(self.PREFIX)
        if action == "stop":
            return self.stop()
        try:
            speed = _bounded(args.get("speed", 0.25), "speed", 0.05, 0.6)
            if action in ("circle", "figure_eight"):
                radius = _bounded(args.get("radius", 0.75), "radius", 0.3, 3.0)
                loops = _bounded(args.get("loops", 1.0), "loops", 0.25, 3.0)
                direction = args.get("direction", "left")
                if direction not in ("left", "right"):
                    raise ValueError("direction must be left or right")
                sign = -1.0 if direction == "right" else 1.0
                circle_seconds = 2 * math.pi * radius / speed
                duration = circle_seconds * loops * (2 if action == "figure_eight" else 1)
                if duration > _MAX_TRAJECTORY_SECONDS:
                    raise ValueError("trajectory duration exceeds 60 seconds")
                if action == "circle":
                    return self._run(duration, lambda _: (speed, 0.0, sign * speed / radius))
                return self._run(
                    duration,
                    lambda elapsed: (
                        speed, 0.0,
                        sign * speed / radius * (-1.0 if int(elapsed / circle_seconds) % 2 else 1.0),
                    ),
                )
            if action == "slalom":
                duration = _bounded(args.get("duration", 10.0), "duration", 1.0, 30.0)
                amplitude = _bounded(args.get("yaw_amplitude", 0.6), "yaw_amplitude", 0.1, 1.2)
                period = _bounded(args.get("period", 4.0), "period", 1.0, 10.0)
                return self._run(
                    duration,
                    lambda elapsed: (speed, 0.0, amplitude * math.sin(2 * math.pi * elapsed / period)),
                )
        except ValueError as exc:
            return {"error": str(exc), "code": "INVALID_ARGUMENT"}
        return {"error": f"unknown trajectory_motion action: {action}"}


class MotionRecorderPlugin:
    """Record velocity commands submitted through this card and replay them."""

    PREFIX = "motion_recorder"

    def __init__(self, config, proxy, executor, stop_loco):
        self._proxy = proxy
        self._executor = executor
        self._stop_loco = stop_loco
        self._directory = Path(config.get("recordings_dir") or _DEFAULT_RECORDINGS_DIR)
        self._lock = threading.RLock()
        self._recording = False
        self._frames = []
        self._record_name = None
        self._record_started = None
        self._record_timer = None

    def get_tool(self):
        actions = ["record_start", "drive", "record_stop", "play", "stop_playback",
                   "list", "delete", "status"]
        return {
            "name": self.PREFIX,
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Record timestamped AS2W velocity commands and replay them. Commands are recorded "
                "only when sent through motion_recorder.drive."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": actions},
                    "label": {"type": "string", "maxLength": 48},
                    "name": {"type": "string", "maxLength": 96},
                    "duration": {"type": "number", "minimum": 0, "maximum": 300},
                    "speed_scale": {"type": "number", "minimum": 0.25, "maximum": 2.0},
                    "vx": {"type": "number", "minimum": -1.5, "maximum": 1.5},
                    "vy": {"type": "number", "minimum": -1.0, "maximum": 1.0},
                    "vyaw": {"type": "number", "minimum": -2.0, "maximum": 2.0},
                },
                "required": ["action"],
                "x-is-dangerous": True,
                "x-action-params": {
                    "record_start": {"params": ["label", "duration"]},
                    "drive": {"params": ["vx", "vy", "vyaw"]},
                    "record_stop": {"params": []},
                    "play": {"params": ["name", "speed_scale"]},
                    "stop_playback": {"params": []},
                    "list": {"params": []},
                    "delete": {"params": ["name"]},
                    "status": {"params": []},
                },
                "x-completion": {"actions": ["play"], "timeout": 620},
                "x-hooks": {
                    "on_interrupt_motion": {"action": "stop_playback"},
                    "on_interrupt_recording": {"action": "record_stop"},
                },
            },
        }

    def start(self):
        self._directory.mkdir(parents=True, exist_ok=True)

    def stop(self):
        recording = self._finish_recording("lifecycle") if self._recording else None
        playback = self._executor.stop(self.PREFIX)
        return {"state": "idle", "recording": recording, "playback": playback}

    @staticmethod
    def _safe_name(value):
        name = re.sub(r"[^a-zA-Z0-9_-]+", "-", str(value or "motion")).strip("-_")
        if not name:
            name = "motion"
        return name[:48]

    def _path(self, name):
        safe = self._safe_name(name)
        if safe != name:
            raise ValueError("invalid recording name")
        return self._directory / f"{safe}.json"

    def _start_recording(self, args):
        duration = _bounded(args.get("duration", 0), "duration", 0, _MAX_RECORDING_SECONDS)
        with self._lock:
            if self._recording:
                return {"state": "recording", "name": self._record_name,
                        "frames": len(self._frames), "already_recording": True}
        self._stop_loco()
        self._executor.stop()
        now_ms = int(time.time() * 1000)
        label = self._safe_name(args.get("label", "motion"))
        with self._lock:
            self._directory.mkdir(parents=True, exist_ok=True)
            self._recording = True
            self._frames = []
            self._record_name = f"{label}-{now_ms}"
            self._record_started = time.monotonic()
            if duration > 0:
                self._record_timer = threading.Timer(duration, self._finish_recording,
                                                     kwargs={"reason": "duration"})
                self._record_timer.daemon = True
                self._record_timer.start()
            return {"state": "recording", "name": self._record_name, "frames": 0}

    def _drive(self, args):
        try:
            velocity = _velocity(args)
        except ValueError as exc:
            return {"error": str(exc), "code": "INVALID_ARGUMENT"}
        with self._lock:
            if not self._recording:
                return {"error": "record_start must be called before drive", "code": "NOT_RECORDING"}
            if len(self._frames) >= _MAX_RECORDING_FRAMES:
                return {"error": "recording frame limit reached", "code": "RECORDING_LIMIT"}
            elapsed_ms = int((time.monotonic() - self._record_started) * 1000)
            if elapsed_ms > _MAX_RECORDING_SECONDS * 1000:
                return {"error": "recording duration limit reached", "code": "RECORDING_LIMIT"}
        ret = self._proxy.Move(*velocity)
        if ret != 0:
            self._proxy.StopMove()
            return {"state": "failed", "ret": ret, "frames": len(self._frames)}
        with self._lock:
            if not self._recording:
                self._proxy.StopMove()
                return {"error": "recording stopped while drive was running",
                        "code": "NOT_RECORDING"}
            self._frames.append({"timestamp_ms": elapsed_ms, "vx": velocity[0],
                                 "vy": velocity[1], "vyaw": velocity[2]})
            frame_count = len(self._frames)
        return {"state": "recording", "ret": ret, "frames": frame_count,
                "velocity": {"vx": velocity[0], "vy": velocity[1], "vyaw": velocity[2]}}

    def _finish_recording(self, reason="manual"):
        with self._lock:
            if not self._recording:
                return {"state": "idle", "saved": False}
            self._recording = False
            if self._record_timer is not None:
                self._record_timer.cancel()
                self._record_timer = None
            frames = list(self._frames)
            name = self._record_name
            duration_ms = frames[-1]["timestamp_ms"] if frames else 0
            payload = {"version": 1, "name": name, "reason": reason,
                       "duration_ms": duration_ms, "frames": frames}
            self._frames = []
            self._record_name = None
            self._record_started = None
        stop_ret = self._proxy.StopMove()
        if not frames:
            return {"state": "idle", "saved": False, "reason": "no_frames",
                    "stop_ret": stop_ret}
        path = self._directory / f"{name}.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(payload, separators=(",", ":")))
        os.replace(temporary, path)
        return {"state": "saved", "saved": True, "name": name,
                "frames": len(frames), "duration_ms": duration_ms, "stop_ret": stop_ret}

    def _load(self, name):
        payload = json.loads(self._path(name).read_text())
        frames = payload.get("frames")
        if payload.get("version") != 1 or not isinstance(frames, list):
            raise ValueError("unsupported recording format")
        if not frames or len(frames) > _MAX_RECORDING_FRAMES:
            raise ValueError("recording has an invalid frame count")
        normalized = []
        previous = -1
        for frame in frames:
            timestamp = int(frame["timestamp_ms"])
            if timestamp < previous or timestamp > _MAX_RECORDING_SECONDS * 1000:
                raise ValueError("recording timestamps are invalid")
            velocity = _velocity(frame)
            normalized.append((timestamp, velocity))
            previous = timestamp
        return payload, normalized

    def _play(self, args):
        if self._recording:
            return {"error": "cannot play while recording", "code": "RECORDING_ACTIVE"}
        try:
            name = str(args.get("name", ""))
            speed_scale = _bounded(args.get("speed_scale", 1), "speed_scale", 0.25, 2.0)
            _, frames = self._load(name)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            return {"error": str(exc), "code": "INVALID_RECORDING"}

        def worker(stop_event):
            started = time.monotonic()
            sent = 0
            for timestamp_ms, velocity in frames:
                target = timestamp_ms / 1000.0 / speed_scale
                remaining = target - (time.monotonic() - started)
                if remaining > 0 and stop_event.wait(remaining):
                    break
                if stop_event.is_set():
                    break
                ret = self._proxy.Move(*velocity)
                if ret != 0:
                    return {"state": "failed", "ret": ret, "frames_sent": sent}
                sent += 1
            return {"state": "cancelled" if stop_event.is_set() else "completed",
                    "name": name, "frames_sent": sent}

        self._stop_loco()
        result = self._executor.start(self.PREFIX, worker)
        if "error" not in result:
            result.update({"name": name, "frames": len(frames), "speed_scale": speed_scale})
        return result

    def _list(self):
        self._directory.mkdir(parents=True, exist_ok=True)
        recordings = []
        for path in sorted(self._directory.glob("*.json")):
            try:
                payload = json.loads(path.read_text())
                recordings.append({"name": path.stem, "frames": len(payload.get("frames", [])),
                                   "duration_ms": payload.get("duration_ms", 0)})
            except (OSError, json.JSONDecodeError):
                continue
        return {"state": "ready", "recordings": recordings}

    def _delete(self, args):
        try:
            path = self._path(str(args.get("name", "")))
            path.unlink()
        except FileNotFoundError:
            return {"error": "recording not found", "code": "NOT_FOUND"}
        except (OSError, ValueError) as exc:
            return {"error": str(exc), "code": "INVALID_RECORDING"}
        return {"state": "deleted", "name": path.stem}

    def _status(self):
        with self._lock:
            recording = self._recording
            name = self._record_name
            frames = len(self._frames)
        return {"state": "recording" if recording else self._executor.status(self.PREFIX)["state"],
                "recording": recording, "name": name, "frames": frames,
                "playback": self._executor.status(self.PREFIX)}

    def dispatch(self, action, args):
        if action in ("start", "info"):
            status = self._status()
            status["activity_state"] = status.pop("state")
            status["state"] = "ready"
            return status
        if action == "status":
            return self._status()
        if action == "stop":
            return self.stop()
        if action == "record_start":
            try:
                return self._start_recording(args)
            except ValueError as exc:
                return {"error": str(exc), "code": "INVALID_ARGUMENT"}
        if action == "drive":
            return self._drive(args)
        if action == "record_stop":
            return self._finish_recording()
        if action == "play":
            return self._play(args)
        if action == "stop_playback":
            return self._executor.stop(self.PREFIX)
        if action == "list":
            return self._list()
        if action == "delete":
            return self._delete(args)
        return {"error": f"unknown motion_recorder action: {action}"}
