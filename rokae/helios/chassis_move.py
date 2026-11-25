"""Guarded chassis movement card (`base_drive`) for the Helios wheeled base.

Same contract as the AgileX Bunker `base_drive` card: a bounded `move` with an
explicit duration, an immediate `stop`, `x-completion` via Agent Core ACP and
`x-resource: base`. Speed reaches the chassis through the vendor VMR AMR SDK
third-party speed control (`nav.set_third_speed_ctrl` /
`send_third_expected_speed` / `close_third_speed_ctrl`), which runs in a
separate worker process (see chassis_sdk_worker.py). The vendor SDK is not part
of this repository; the Dockerfile fetches it from the project COS (hash-pinned).
"""

from __future__ import annotations

import json
import math
import os
import ssl
import shutil
import subprocess
import sys
import threading
import time
import urllib.request
from uuid import uuid4

from common.vendor_runtime import tool

MOVE_MIN_DURATION_S = 0.05
MOVE_MAX_DURATION_S = 10.0
MOVE_COMPLETION_TIMEOUT_S = 20
HARD_MAX_LINEAR_M_S = 0.5
HARD_MAX_ANGULAR_DEG_S = 30.0
WORKER_READY_TIMEOUT_S = 8.0
# The vendor SDK has no synchronous getter for the exception level, only the
# exception_state subscription; a move waits this long for its first report.
EXCEPTION_STATE_TIMEOUT_S = 3.0
WATCHDOG_MARGIN_S = 3.0
# Before the worker reports move_acquired it may still be retrying set_third_speed_ctrl
# while the chassis wakes its motors (3 attempts x 3 s RPC timeout, see the worker).
ACQUIRE_BUDGET_S = 10.0
# After acquisition the worker may wait up to this long for idle motors to wake and the
# steerable wheels to turn before the move's duration starts (worker START_GRACE_S).
START_GRACE_S = 5.0


def acp_notify(action_id, status, result):
    """Report an asynchronous terminal result to Agent Core (same API as G1/Bunker)."""
    url = os.environ.get("AGENT_CORE_URL", "https://127.0.0.1:15678").rstrip("/")
    body = json.dumps({"action_id": action_id, "status": status, "result": result,
                       "tool": "base_drive", "ts": time.time()}).encode()
    request = urllib.request.Request(url + "/api/acp/complete", data=body,
                                     headers={"Content-Type": "application/json"}, method="POST")
    context = ssl.create_default_context()
    if url.startswith(("https://localhost:", "https://127.0.0.1:")):
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    try:
        with urllib.request.urlopen(request, timeout=5, context=context):
            pass
    except Exception as exc:
        print(f"[helios-move] ACP completion callback failed ({type(exc).__name__})", flush=True)


def _number(args, name, default=0.0):
    value = args.get(name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _limit(value, maximum, name, unit):
    if abs(value) > maximum + 1e-9:
        raise ValueError(f"{name}={value:g} {unit} exceeds the configured limit ±{maximum:g} {unit}")
    return value


class SdkWorkerLink:
    """Owns the vendor SDK worker process and its JSON-lines protocol."""

    def __init__(self, config, on_move_done, popen=subprocess.Popen, on_move_acquired=None):
        self._config = config
        self._on_move_done = on_move_done
        self._on_move_acquired = on_move_acquired or (lambda move_id: None)
        self._popen = popen
        self._lock = threading.Lock()
        self._proc = None
        self._ready = threading.Event()
        self._fatal = None
        self.exception_level = None
        self.exception_supported = None  # worker reports whether the SDK has an exception device
        self._exception_seen = threading.Event()
        self.exceptions = []  # last reported [{"module", "code"}] from the chassis
        self.exception_history = []  # recent (time, level, exceptions) for diagnosis
        self.odometry = None  # latest {"x","y","theta","vx","vy","w","age_s"} from the worker

    def sdk_status(self):
        sdk_dir = self._config.get("chassis_sdk_dir", "")
        lib_dir = os.path.join(sdk_dir, self._config.get("chassis_sdk_lib_subdir", "lib"))
        if not sdk_dir or not os.path.isdir(lib_dir):
            return False, f"vendor chassis SDK not mounted at {lib_dir}"
        return True, lib_dir

    def ensure_started(self, timeout_s=WORKER_READY_TIMEOUT_S):
        with self._lock:
            if self._proc is not None and self._proc.poll() is None and self._ready.is_set():
                return
            if self._proc is not None:
                self._terminate_locked()
            available, lib_dir = self.sdk_status()
            if not available:
                raise RuntimeError(lib_dir)
            env = dict(os.environ)
            # Only the vendor modules and this repository: the ROS Humble (CPython 3.10)
            # site-packages must not leak into the SDK interpreter.
            repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            env["PYTHONPATH"] = os.pathsep.join((lib_dir, repo_root))
            env["LD_LIBRARY_PATH"] = os.pathsep.join(p for p in (lib_dir, env.get("LD_LIBRARY_PATH", "")) if p)
            worker_config = json.dumps({
                "host": os.environ.get("HELIOS_CHASSIS_SDK_HOST", self._config["chassis_sdk_host"]),
                "port": int(os.environ.get("HELIOS_CHASSIS_SDK_PORT", self._config["chassis_sdk_port"])),
                "rate_hz": float(self._config.get("chassis_command_hz", 50)),
            })
            python = self._config.get("chassis_sdk_python") or sys.executable
            if not (os.path.isfile(python) or shutil.which(python)):
                raise RuntimeError(f"chassis SDK Python interpreter not found: {python}")
            self._ready.clear()
            self._fatal = None
            self.exception_level = None
            self.exception_supported = None
            self._exception_seen.clear()
            self._proc = self._popen(
                [python, "-m", "rokae.helios.chassis_sdk_worker", worker_config],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None,
                text=True, bufsize=1, env=env)
            threading.Thread(target=self._read_events, args=(self._proc,), daemon=True,
                             name="helios-chassis-sdk-events").start()
        if not self._ready.wait(timeout_s):
            self.close()
            raise RuntimeError(self._fatal or "chassis SDK worker did not become ready")
        if self._fatal:
            raise RuntimeError(self._fatal)

    def _read_events(self, proc):
        for line in proc.stdout:
            try:
                event = json.loads(line)
            except ValueError:
                continue
            kind = event.get("event")
            if kind == "ready":
                self.exception_supported = bool(event.get("exception_state"))
                self._ready.set()
            elif kind == "fatal":
                self._fatal = event.get("message", "chassis SDK worker failed")
                self._ready.set()
            elif kind == "exception":
                self.exception_level = event.get("level")
                self.exceptions = event.get("exceptions") or []
                self._exception_seen.set()
                self.exception_history = (self.exception_history + [{
                    "time": time.strftime("%H:%M:%S"), "level": self.exception_level,
                    "exceptions": self.exceptions}])[-10:]
            elif kind == "odom":
                self.odometry = {k: event[k] for k in ("x", "y", "theta", "vx", "vy", "w") if k in event}
                self.odometry["received_monotonic"] = time.monotonic()
            elif kind == "move_acquired":
                self._on_move_acquired(event.get("id"))
            elif kind == "move_done":
                self._on_move_done(event.get("id"), event.get("status", "error"), event.get("message", ""),
                                   event.get("odometry"), event.get("start_delay_s"))
        # Worker exited: the chassis stops on missing speed commands.
        self._on_move_done(None, "error", self._fatal or "chassis SDK worker exited")

    def wait_exception_state(self, timeout_s=EXCEPTION_STATE_TIMEOUT_S):
        """True once the chassis has reported its exception level since the worker started."""
        return self._exception_seen.wait(timeout_s)

    def send(self, message):
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None:
                raise RuntimeError("chassis SDK worker is not running")
            proc.stdin.write(json.dumps(message) + "\n")
            proc.stdin.flush()

    def running(self):
        proc = self._proc
        return proc is not None and proc.poll() is None and self._ready.is_set() and not self._fatal

    def _terminate_locked(self):
        proc, self._proc = self._proc, None
        if proc is None:
            return
        try:
            if proc.poll() is None:
                proc.stdin.write(json.dumps({"op": "quit"}) + "\n")
                proc.stdin.flush()
                proc.stdin.close()
                proc.wait(timeout=3)
        except Exception:
            pass
        if proc.poll() is None:
            proc.kill()
            try:
                proc.wait(timeout=2)
            except Exception:
                pass

    def kill(self):
        """Last resort: the vendor stops the base when commands stop arriving."""
        with self._lock:
            proc, self._proc = self._proc, None
        if proc is not None and proc.poll() is None:
            proc.kill()

    def close(self):
        with self._lock:
            self._terminate_locked()


class HeliosChassisMove:
    PREFIX = "base_drive"

    def __init__(self, config, link=None, notify=acp_notify, clock=time.monotonic):
        self._config = config
        self._notify = notify
        self._clock = clock
        self._lock = threading.Lock()
        self._active = None  # {"id", "deadline"}
        self._watchdog_stop = threading.Event()
        self._watchdog = None
        self._max_linear = float(config.get("chassis_max_linear_m_s", 0.2))
        self._max_angular_deg = float(config.get("chassis_max_angular_deg_s", 15.0))
        if not 0 < self._max_linear <= HARD_MAX_LINEAR_M_S:
            raise ValueError(f"chassis_max_linear_m_s must be in (0, {HARD_MAX_LINEAR_M_S}]")
        if not 0 < self._max_angular_deg <= HARD_MAX_ANGULAR_DEG_S:
            raise ValueError(f"chassis_max_angular_deg_s must be in (0, {HARD_MAX_ANGULAR_DEG_S}]")
        self._allow_lateral = bool(config.get("chassis_allow_lateral", False))
        enabled = os.environ.get("HELIOS_CHASSIS_MOTION_ENABLED")
        # Off unless the deployment opts in: charging interlock, controller command timeout and
        # obstacle-avoidance behaviour under SDK speed control are not yet verified.
        self._enabled = (enabled == "1") if enabled is not None else bool(config.get("chassis_motion_enabled", False))
        self.link = link or SdkWorkerLink(config, self._on_move_done, on_move_acquired=self._on_move_acquired)

    # ── tool definition ──────────────────────────────────────────────────
    def get_tools(self):
        lin, ang = self._max_linear, self._max_angular_deg
        vx = {"type": "number", "minimum": -lin, "maximum": lin, "default": 0,
              "description": "Forward velocity in m/s (positive = forward)"}
        vy = {"type": "number", "minimum": -lin, "maximum": lin, "default": 0,
              "description": "Lateral velocity in m/s (positive = left)"}
        props = {
            "action": {"type": "string", "enum": ["move", "stop_move", "stop", "info"], "description": "Action to perform"},
            "vx": vx,
            # The canvas renders fields in property order: keep vy right below vx.
            **({"vy": vy} if self._allow_lateral else {}),
            "vyaw": {"type": "number", "minimum": -ang, "maximum": ang, "default": 0,
                     "description": "Yaw rate in degrees/s (positive = counter-clockwise / left)"},
            "duration_s": {"type": "number", "minimum": MOVE_MIN_DURATION_S, "maximum": MOVE_MAX_DURATION_S,
                           "description": "Move duration in seconds; the base stops when it elapses"},
        }
        move_params = ["vx", "vy", "vyaw", "duration_s"] if self._allow_lateral else ["vx", "vyaw", "duration_s"]
        schema = {
            "type": "object", "properties": props, "required": ["action"],
            "x-action-params": {
                "move": {"params": move_params,
                         "description": "Move for a bounded duration; vx m/s, vyaw degrees/s, then stop"},
                # Core hides start/stop/info/config from the canvas action list (system
                # actions), so manual stopping needs a public action — same as G1 loco.
                "stop_move": {"params": [], "description": "Stop the base immediately (cancels the current move)"},
                "stop": {"params": [], "description": "Stop the base immediately"},
                "info": {"params": [], "description": "Limits, SDK link and chassis exception state"},
            },
            "x-completion": {"actions": ["move"], "timeout": MOVE_COMPLETION_TIMEOUT_S},
            "x-resource": "base",
        }
        return [tool("base_drive", "actuator",
                     f"Helios 底盘移动：限时低速前进/后退/旋转（≤{lin:g} m/s、≤{ang:g}°/s、≤{MOVE_MAX_DURATION_S:g} s），"
                     "到时自动停车；stop_move 立即停车。经厂家 SDK 第三方控速下发，断线/异常时底盘自动停止。",
                     schema)]

    # ── lifecycle ────────────────────────────────────────────────────────
    def start(self):
        # Do not connect or acquire speed control at Driver startup.
        pass

    def stop(self):
        self._stop_motion("driver stopping")
        self._watchdog_stop.set()
        self.link.close()

    # ── dispatch ─────────────────────────────────────────────────────────
    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action in ("stop", "stop_move"):
            return {**self._stop_motion("stop requested"), "state": "idle"}
        if action == "info":
            return self._info()
        if action == "move":
            return self._move(args)
        raise ValueError(f"unknown base_drive action: {action}")

    def _info(self):
        available, detail = self.link.sdk_status()
        connect_error = None
        if available and not self.link.running():
            # Read-only: connects and subscribes to odometry/exception state; never
            # acquires speed control (that happens only inside a move).
            try:
                self.link.ensure_started()
            except Exception as exc:
                connect_error = str(exc)
        with self._lock:
            active = dict(self._active) if self._active else None
        return {
            "connect_error": connect_error,
            "state": "moving" if active else "ready",
            "motion_enabled": self._enabled,
            "sdk_available": available,
            "sdk_detail": detail if not available else None,
            "worker_running": self.link.running(),
            "chassis_exception_level": self.link.exception_level,
            "chassis_exceptions": getattr(self.link, "exceptions", []),
            "chassis_exception_history": getattr(self.link, "exception_history", []),
            "odometry": self._odometry(),
            "limits": {"max_linear_m_s": self._max_linear, "max_angular_deg_s": self._max_angular_deg,
                       "max_duration_s": MOVE_MAX_DURATION_S, "lateral_enabled": self._allow_lateral},
            "active_move": active["id"] if active else None,
        }

    def _validate(self, args):
        duration = _number(args, "duration_s", None) if args.get("duration_s") is not None else None
        if duration is None or not MOVE_MIN_DURATION_S <= duration <= MOVE_MAX_DURATION_S:
            raise ValueError(f"duration_s is required, between {MOVE_MIN_DURATION_S:g} and {MOVE_MAX_DURATION_S:g} s")
        vx = _limit(_number(args, "vx"), self._max_linear, "vx", "m/s")
        vy = _number(args, "vy")
        if vy and not self._allow_lateral:
            raise ValueError("lateral movement (vy) is disabled by chassis_allow_lateral")
        vy = _limit(vy, self._max_linear, "vy", "m/s")
        vyaw = _limit(_number(args, "vyaw"), self._max_angular_deg, "vyaw", "deg/s")
        if vx == 0 and vy == 0 and vyaw == 0:
            raise ValueError("move needs a non-zero vx, vy or vyaw")
        return vx, vy, vyaw, duration

    def _move(self, args):
        if not self._enabled:
            raise RuntimeError("chassis movement is disabled (chassis_motion_enabled / HELIOS_CHASSIS_MOTION_ENABLED)")
        vx, vy, vyaw, duration = self._validate(args)
        with self._lock:
            if self._active is not None:
                return {"error": "a timed base move is already running; send stop first"}
        self.link.ensure_started()
        # Fail closed: never drive a chassis whose exception level is unknown.
        if not self.link.exception_supported:
            raise RuntimeError("chassis SDK has no exception device; cannot confirm the chassis is "
                               "safe to move, movement refused")
        if not self.link.wait_exception_state():
            raise RuntimeError("chassis exception state not received yet; movement refused, retry shortly")
        level = self.link.exception_level
        if level != 0:
            raise RuntimeError(f"chassis reports exception level {level} "
                               f"{getattr(self.link, 'exceptions', [])}; movement refused")
        action_id = f"helios_move_{uuid4().hex[:12]}"
        with self._lock:
            if self._active is not None:
                return {"error": "a timed base move is already running; send stop first"}
            # Until the worker holds speed control, allow for the chassis waking its motors.
            self._active = {"id": action_id, "duration": duration,
                            "deadline": self._clock() + ACQUIRE_BUDGET_S + START_GRACE_S + duration
                            + WATCHDOG_MARGIN_S}
        try:
            self.link.send({"op": "move", "id": action_id, "vx": vx, "vy": vy,
                            "wz": math.radians(vyaw), "duration_s": duration})
        except Exception:
            with self._lock:
                self._active = None
            raise
        self._ensure_watchdog()
        return {"state": "running", "action_id": action_id, "duration_s": duration,
                "vx_m_s": vx, "vy_m_s": vy, "vyaw_deg_s": vyaw}

    def _finish(self, action_id, status, result):
        with self._lock:
            active = self._active
            if active is None or (action_id is not None and active["id"] != action_id):
                return
            self._active = None
        self._notify(active["id"], status, result)

    def _odometry(self):
        odom = getattr(self.link, "odometry", None)
        if not odom:
            return None
        age = time.monotonic() - odom.get("received_monotonic", 0)
        out = {k: v for k, v in odom.items() if k != "received_monotonic"}
        out["age_s"] = round(age, 2)
        return out if age < 3.0 else {**out, "stale": True}

    def _on_move_acquired(self, action_id):
        """Speed control acquired: the move runs for its duration from now."""
        with self._lock:
            active = self._active
            if active is not None and active["id"] == action_id:
                active["deadline"] = self._clock() + START_GRACE_S + active["duration"] + WATCHDOG_MARGIN_S

    def _on_move_done(self, action_id, status, message, odometry=None, start_delay_s=None):
        result = {"message": message} if message else {}
        if odometry:
            result["odometry"] = odometry
        if start_delay_s:  # time spent turning the wheels before the move's duration started
            result["start_delay_s"] = start_delay_s
        if status == "error" and message:
            result["error"] = message
        self._finish(action_id, status, result)

    def _stop_motion(self, reason):
        with self._lock:
            active = self._active
        try:
            if self.link.running():
                self.link.send({"op": "stop"})
        except Exception:
            self.link.kill()
        if active is not None:
            # The worker reports move_done(cancelled); this is the fallback.
            threading.Timer(2.0, self._finish, args=(active["id"], "cancelled", {"message": reason})).start()
        return {"stopped": True, "cancelled_move": active["id"] if active else None}

    def _ensure_watchdog(self):
        if self._watchdog is not None and self._watchdog.is_alive():
            return
        self._watchdog_stop.clear()
        self._watchdog = threading.Thread(target=self._watch, daemon=True, name="helios-move-watchdog")
        self._watchdog.start()

    def _watch(self):
        while not self._watchdog_stop.wait(0.2):
            with self._lock:
                active = self._active
            if active is None:
                return
            if self._clock() > active["deadline"]:
                # Worker failed to report completion in time: cut it off. The
                # vendor stops the base when speed commands stop arriving.
                self.link.kill()
                self._finish(active["id"], "error", {"error": "move completion watchdog expired"})
                return
