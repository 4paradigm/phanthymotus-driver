"""Out-of-process bridge to the chassis vendor SDK (VMR AMR SDK, `nav` module).

Run as ``python3 -m rokae.helios.chassis_sdk_worker '<json config>'`` by
:class:`rokae.helios.chassis_move.SdkWorkerLink`. One JSON object per line on
stdin/stdout. The vendor extension modules are compiled for one CPython ABI and
are loaded only here, so the Driver process never imports vendor binaries and a
crash in them cannot take the joint/battery cards down.

Safety properties kept in this process, independent of the parent:

* Every move has its own deadline; the worker stops sending speed when it
  expires even if the parent is unresponsive.
* stdin EOF (parent exited or was killed) stops any move and exits.
* Every exit path sends zero speed several times and releases SDK speed control
  (`close_third_speed_ctrl`). The vendor documents that the chassis stops when
  speed commands stop arriving, so killing this process also stops the base.
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time

ZERO_REPEAT = 5
MAX_SEND_FAILURES = 3
# Acquiring SDK speed control can time out while the chassis wakes and enables its
# idle motors (seen on the robot after a restart: "RPC call timeout: set_third_speed_ctrl").
# Each attempt is bounded by the RPC call timeout (3 s), so acquisition takes at most ~10 s.
ACQUIRE_ATTEMPTS = 3
ACQUIRE_RETRY_DELAY_S = 0.5
# The Helios base has steerable wheels: when the direction changes (e.g. forward after a
# lateral move) it first turns the wheels and only then drives. duration_s counts from
# the moment odometry shows the base moving, waiting at most STEER_GRACE_S for that.
STEER_GRACE_S = 2.0
# Motion is judged by distance actually covered, not by speed: while the wheels turn,
# odometry shows short speed spikes (measured 0.016-0.037 m/s for a 0.05 m/s command)
# with only 2-5 mm of real displacement.
START_DISTANCE_M = 0.01
START_YAW_DEG = 1.5


def _twist(nav, vx, vy, wz):
    twist = nav.Twist()
    twist.linear = nav.Vector3()
    twist.linear.x, twist.linear.y, twist.linear.z = float(vx), float(vy), 0.0
    twist.angular = nav.Vector3()
    twist.angular.x, twist.angular.y, twist.angular.z = 0.0, 0.0, float(wz)
    return twist


class SpeedSession:
    """One bounded speed command: acquire control, stream, zero, release."""

    def __init__(self, nav, rate_hz, clock=time.monotonic, sleep=time.sleep):
        if not 20 < rate_hz <= 100:
            raise ValueError("command rate must be above 20 Hz (vendor requirement) and at most 100 Hz")
        self._nav = nav
        self._period = 1.0 / rate_hz
        self._clock = clock
        self._sleep = sleep

    def _acquire(self, stop_event):
        """Return (acquired, reason). Retries; an SDK exception is a failed attempt, not a crash."""
        reason = "set_third_speed_ctrl failed"
        for attempt in range(ACQUIRE_ATTEMPTS):
            if stop_event.is_set():
                return False, "cancelled"
            try:
                if self._nav.set_third_speed_ctrl():
                    return True, ""
                reason = "set_third_speed_ctrl returned false"
            except Exception as exc:
                reason = f"set_third_speed_ctrl: {type(exc).__name__}: {exc}"
            if attempt + 1 < ACQUIRE_ATTEMPTS:
                self._sleep(ACQUIRE_RETRY_DELAY_S)
        return False, reason

    def run(self, vx, vy, wz, duration_s, stop_event, blocked=lambda: None, on_acquired=lambda: None,
            moving=None):
        """Return (status, message); status is completed, cancelled or error.

        `moving()` reports whether odometry shows the base following the command (None:
        no odometry, time from the first command). The duration starts once it does, or
        after STEER_GRACE_S at most; `self.start_delay_s` records the wait.
        """
        self.start_delay_s = 0.0
        for value in (vx, vy, wz, duration_s):
            if not math.isfinite(value):
                return "error", "non-finite command"
        acquired, reason = self._acquire(stop_event)
        if not acquired:
            try:  # release anything a timed-out call may have half-acquired; no speed was sent
                self._nav.close_third_speed_ctrl()
            except Exception:
                pass
            if reason == "cancelled":
                return "cancelled", ""
            return "error", (f"could not acquire SDK speed control after {ACQUIRE_ATTEMPTS} attempts "
                             f"({reason}); the chassis may still be waking its motors, retry shortly")
        on_acquired()
        status, message = "completed", ""
        try:
            twist = _twist(self._nav, vx, vy, wz)
            first = self._clock()
            deadline = None if moving is not None else first + duration_s
            failures = 0
            while True:
                if stop_event.is_set():
                    status = "cancelled"
                    break
                reason = blocked()
                if reason:
                    status, message = "error", reason
                    break
                now = self._clock()
                if deadline is None and (now - first >= STEER_GRACE_S or moving()):
                    deadline = now + duration_s
                    self.start_delay_s = round(now - first, 2)
                if deadline is not None and now >= deadline:
                    break
                if self._nav.send_third_expected_speed(twist):
                    failures = 0
                else:
                    failures += 1
                    if failures >= MAX_SEND_FAILURES:
                        status, message = "error", "send_third_expected_speed failed repeatedly"
                        break
                self._sleep(self._period)
        except Exception as exc:  # Never leave the base moving on an SDK exception.
            status, message = "error", f"{type(exc).__name__}: {exc}"
        finally:
            self.zero_and_release()
        return status, message

    def zero_and_release(self):
        zero = None
        try:
            zero = _twist(self._nav, 0.0, 0.0, 0.0)
        except Exception:
            pass
        for _ in range(ZERO_REPEAT):
            try:
                if zero is not None:
                    self._nav.send_third_expected_speed(zero)
            except Exception:
                pass
            self._sleep(self._period)
        try:
            self._nav.close_third_speed_ctrl()
        except Exception:
            pass


class OdomTracker:
    """Latest vendor odometry (x/y m, theta rad, vx/vy m/s, w rad/s), sent to the parent at ~2 Hz."""

    FIELDS = ("x", "y", "theta", "vx", "vy", "w")

    def __init__(self, emit, clock=time.monotonic, period_s=0.5):
        self._emit = emit
        self._clock = clock
        self._period = period_s
        self._lock = threading.Lock()
        self._latest = None
        self._last_emit = None

    def on_odom(self, info):
        sample = {}
        for name in self.FIELDS:
            value = getattr(info, name, None)
            if isinstance(value, (int, float)) and math.isfinite(value):
                sample[name] = float(value)
        if "x" not in sample or "y" not in sample or "theta" not in sample:
            return
        now = self._clock()
        with self._lock:
            self._latest = (now, sample)
            due = self._last_emit is None or now - self._last_emit >= self._period
            if due:
                self._last_emit = now
        if due:
            self._emit({"event": "odom", **{k: round(v, 4) for k, v in sample.items()}})

    def snapshot(self, max_age_s=1.0):
        with self._lock:
            latest = self._latest
        if latest is None or self._clock() - latest[0] > max_age_s:
            return None
        return dict(latest[1])

    @staticmethod
    def displacement(start, end):
        """Movement between two samples, expressed in the start pose's body frame."""
        if not start or not end:
            return None
        dx, dy = end["x"] - start["x"], end["y"] - start["y"]
        c, s = math.cos(start["theta"]), math.sin(start["theta"])
        dtheta = math.atan2(math.sin(end["theta"] - start["theta"]), math.cos(end["theta"] - start["theta"]))
        return {"forward_m": round(c * dx + s * dy, 4), "left_m": round(-s * dx + c * dy, 4),
                "distance_m": round(math.hypot(dx, dy), 4), "yaw_deg": round(math.degrees(dtheta), 2)}


class Worker:
    def __init__(self, nav, emit, rate_hz, exception_module=None, odom=None):
        self._session = SpeedSession(nav, rate_hz)
        self._emit = emit
        self._lock = threading.Lock()
        self._move = None  # (id, stop_event, thread)
        self._odom = odom
        self.exception_level = None
        self._requires_exception_state = exception_module is not None
        if exception_module is not None:
            exception_module.exception_state(self._on_exception)

    def _on_exception(self, info):
        level = getattr(info.robot_exception, "value", info.robot_exception)
        self.exception_level = int(level)
        active = []
        for item in getattr(info, "exceptions", None) or []:
            module = getattr(item, "exception_moudle", getattr(item, "exception_module", None))  # vendor spelling
            code = getattr(item, "exception_code", None)
            if isinstance(module, int) and isinstance(code, int):
                active.append({"module": module, "code": code})
        self._emit({"event": "exception", "level": self.exception_level, "exceptions": active[:20]})

    def _blocked(self):
        if self._requires_exception_state and self.exception_level is None:
            return "chassis exception state not received yet"
        if self.exception_level not in (None, 0):
            return f"chassis exception level {self.exception_level}"
        return None

    def handle(self, message):
        op = message.get("op")
        if op == "move":
            return self._start_move(message)
        if op == "stop":
            return self.stop()
        raise ValueError(f"unknown op {op!r}")

    def _start_move(self, message):
        move_id = str(message["id"])
        with self._lock:
            if self._move is not None and self._move[2].is_alive():
                self._emit({"event": "move_done", "id": move_id, "status": "error",
                            "message": "another move is running"})
                return
            stop_event = threading.Event()
            thread = threading.Thread(target=self._run, name="chassis-move", daemon=True, args=(
                move_id, stop_event, float(message["vx"]), float(message["vy"]),
                float(message["wz"]), float(message["duration_s"])))
            self._move = (move_id, stop_event, thread)
            thread.start()

    def _run(self, move_id, stop_event, vx, vy, wz, duration_s):
        start = self._odom.snapshot() if self._odom else None
        self._emit({"event": "move_started", "id": move_id})
        try:
            status, text = self._session.run(
                vx, vy, wz, duration_s, stop_event, self._blocked,
                on_acquired=lambda: self._emit({"event": "move_acquired", "id": move_id}),
                moving=self._moving_check(vx, vy, wz) if self._odom else None)
        except Exception as exc:  # always report: a silent thread death left the Driver waiting
            status, text = "error", f"{type(exc).__name__}: {exc}"
        if self._odom:
            time.sleep(0.3)  # let the final odometry sample arrive after zero speed
        end = self._odom.snapshot() if self._odom else None
        self._emit({"event": "move_done", "id": move_id, "status": status, "message": text,
                    "odometry": OdomTracker.displacement(start, end),
                    "start_delay_s": getattr(self._session, "start_delay_s", 0.0)})

    def _moving_check(self, vx, vy, wz):
        """True once odometry shows the base has covered START_DISTANCE_M along the command
        (or turned START_YAW_DEG); True if odometry is stale, so a move is never extended on a guess."""
        linear = math.hypot(vx, vy)
        origin = []

        def moving():
            sample = self._odom.snapshot(max_age_s=0.5)
            if sample is None:
                return True
            if not origin:
                origin.append(sample)
                return False
            moved = OdomTracker.displacement(origin[0], sample)
            if linear > 0:
                along = (moved["forward_m"] * vx + moved["left_m"] * vy) / linear
                if along >= START_DISTANCE_M:
                    return True
            return wz != 0 and math.copysign(1.0, wz) * moved["yaw_deg"] >= START_YAW_DEG
        return moving

    def stop(self, timeout_s=3.0):
        with self._lock:
            move = self._move
        if move is not None:
            move[1].set()
            move[2].join(timeout_s)
        else:
            self._session.zero_and_release()
        self._emit({"event": "stopped"})


def main(argv):
    # A separate interpreter does not inherit the Driver's stdout wrapper; install it
    # before the vendor SDK loads so its diagnostics are line-atomic as well.
    try:
        from common import logsafe
        logsafe.install()
    except Exception as exc:  # never block movement safety on the log wrapper
        sys.stderr.write(f"logsafe unavailable: {exc}\n")
    config = json.loads(argv[1])
    out_lock = threading.Lock()

    def emit(payload):
        with out_lock:
            sys.stdout.write(json.dumps(payload) + "\n")
            sys.stdout.flush()

    try:
        import py_client_common as rpc
        import nav
    except Exception as exc:
        emit({"event": "fatal", "message": f"vendor SDK import failed ({type(exc).__name__}: {exc}); "
                                           f"check the SDK directory and that its Python ABI matches "
                                           f"{sys.version_info.major}.{sys.version_info.minor}"})
        return 2
    conf = rpc.ClientConfig()
    conf.ip = config["host"]
    conf.port = int(config["port"])
    conf.connect_timeout_ms = int(config.get("connect_timeout_ms", 3000))
    conf.call_timeout_ms = int(config.get("call_timeout_ms", 3000))
    if not rpc.init_client(conf):
        emit({"event": "fatal", "message": f"cannot connect to chassis RPC {conf.ip}:{conf.port}"})
        return 3
    worker = None
    try:
        if not nav.has_nav_dev():
            emit({"event": "fatal", "message": "chassis reports no navigation device"})
            return 4
        exception_module = None
        try:
            import exception as exception_module  # noqa: F811 - vendor module name
            if not exception_module.has_exception_dev():
                exception_module = None
        except Exception:
            exception_module = None
        odom = None
        try:
            import odometry
            if odometry.has_odom_dev():
                odom = OdomTracker(emit)
                odometry.odom_state(odom.on_odom)
        except Exception:
            odom = None
        worker = Worker(nav, emit, float(config.get("rate_hz", 50)), exception_module, odom)
        emit({"event": "ready", "exception_state": exception_module is not None,
              "odometry": odom is not None})
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
                if message.get("op") == "quit":
                    break
                worker.handle(message)
            except Exception as exc:
                emit({"event": "error", "message": f"{type(exc).__name__}: {exc}"})
        return 0
    finally:
        if worker is not None:
            worker.stop()
        try:
            rpc.close_client()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main(sys.argv))
