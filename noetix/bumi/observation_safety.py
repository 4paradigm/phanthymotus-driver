"""Bounded observation sessions around LocoPlugin's existing rotate worker.

No SDK command publisher lives here. The owner remains responsible for rotation,
ACP and stop_move. Session admission shares its chassis lock. Telemetry settling
is deliberately conservative; it is not an independent hardware emergency stop.
"""
from collections import deque
from copy import deepcopy
import math
import threading
import time
import uuid

MAX_SINGLE_ROTATION_DEG = 10.0
MAX_ROTATION_COUNT = 3
MAX_TOTAL_ABSOLUTE_ROTATION_DEG = 30.0
ACTION_TIMEOUT_S = 20.0
STOP_TIMEOUT_S = 2.0
SETTLE_INTERVAL_S = .05
SETTLE_SAMPLES = 3
MAX_JOINT_SPEED = .10
MAX_ANGULAR_SPEED = .10


class ObservationSafety:
    def __init__(self, owner, *, action_timeout_s=ACTION_TIMEOUT_S, stop_timeout_s=STOP_TIMEOUT_S):
        self.owner = owner
        self.lock = owner._rotate_lock
        self.stop_lock = threading.Lock()
        self.end_lock = threading.Lock()
        self.token = object()
        self.session = None
        self.admitting = False
        self.monitor = None
        self.action_timeout_s = action_timeout_s
        self.stop_timeout_s = stop_timeout_s
        self.receipts = deque(maxlen=32)

    @property
    def reserved(self):
        session = self.session
        return self.admitting or (session is not None and not session.get("released", False))

    def _error(self, code, message):
        return {"state": "error", "code": code, "error": message, "command_sent": False}

    def _valid(self, session_id):
        return self.reserved and isinstance(session_id, str) and session_id == self.session["session_id"]

    def _snapshot(self):
        ctrl = self.owner._high_ctrl
        mode = int(ctrl.get_mode())
        if mode != 2:
            raise ValueError(f"observation requires walking mode=2; current={mode}")
        yaw, wz = self.owner._read_rotate_imu()
        imu = ctrl.get_imu_data()
        angular = [float(v) for v in imu.angular_vel]
        joints = ctrl.get_joint_state()
        if len(joints) != 21 or len(angular) != 3:
            raise ValueError("incomplete motion feedback")
        if any(int(j.error) != 0 for j in joints):
            raise ValueError("motor fault (including undocumented nonzero error)")
        speeds = [float(j.vel) for j in joints]
        if not all(math.isfinite(v) for v in [yaw, wz, *speeds, *angular]):
            raise ValueError("nonfinite motion feedback")
        if max(map(abs, speeds)) > MAX_JOINT_SPEED or max(map(abs, angular)) > MAX_ANGULAR_SPEED:
            raise ValueError("motion feedback is not stationary")
        return yaw

    def _idle_error(self):
        owner = self.owner
        if (owner._rotate_state["state"] == "running" or owner._body_action
                or (owner._move_thread and owner._move_thread.is_alive())):
            return "another chassis action is active"
        servo = owner._servo
        if servo is not None:
            state = servo._info()
            if state.get("state") not in ("idle", "paused"):
                return "streaming chassis control must be stopped or paused first"
        return None

    def begin(self):
        with self.lock:
            if self.reserved:
                return self._error("SESSION_ACTIVE", "end the existing observation session; budgets cannot be reset")
            # Reserve before reading the servo/SDK. A servo start that already
            # passed its first busy check must recheck at its own commit point.
            self.admitting = True
            try:
                conflict = self._idle_error()
                if conflict:
                    self.admitting = False
                    return self._error("CHASSIS_BUSY", conflict)
                self._snapshot()
            except Exception as exc:
                self.admitting = False
                return self._error("UNSAFE_STATE", str(exc))
            self.session = {"session_id": uuid.uuid4().hex, "state": "ready", "released": False,
                            "cancelled": False, "rotation_count": 0, "total_absolute_rotation_deg": 0.,
                            "action_id": None, "last_result": None, "error": None,
                            "stop_confirmed": True, "observation_after_ns": time.time_ns()}
            self.admitting = False
            self.receipts.clear()
            return self.status(self.session["session_id"])

    def rotate(self, args):
        with self.lock:
            if not self._valid(args.get("session_id")):
                return self._error("INVALID_SESSION", "valid session_id required")
            session = self.session
            if session["cancelled"] or session["state"] != "ready":
                return self._error("SESSION_NOT_READY", "session is running, cancelled or locked")
            raw = args.get("angle_deg")
            if (type(raw) not in (float, int) or not math.isfinite(raw)
                    or not .05 <= abs(raw) <= MAX_SINGLE_ROTATION_DEG):
                return self._error("SINGLE_ROTATION_LIMIT", "angle_deg must be finite, nonzero and within ±10 degrees")
            request_id = args.get("request_id")
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
                return self._error("INVALID_REQUEST", "unique request_id required for retry deduplication")
            if request_id in self.receipts:
                return self._error("DUPLICATE_REQUEST", "request was already admitted; query observation_status")
            if session["rotation_count"] >= MAX_ROTATION_COUNT:
                return self._error("ROTATION_COUNT_LIMIT", "at most three rotations per observation session")
            if session["total_absolute_rotation_deg"] + abs(raw) > MAX_TOTAL_ABSOLUTE_ROTATION_DEG + 1e-9:
                return self._error("TOTAL_ROTATION_LIMIT", "absolute rotation budget exhausted")
            conflict = self._idle_error()
            if conflict:
                return self._error("CHASSIS_BUSY", conflict)
            try:
                self._snapshot()
                _, refusal = self.owner._rotate_preflight({"angle_deg": raw})
                if refusal:
                    return refusal
            except Exception as exc:
                return self._error("UNSAFE_STATE", str(exc))
            # Reserve before opening a worker. Failures consume the reservation;
            # alternating signs, retries and failed starts cannot gain more budget.
            session["rotation_count"] += 1
            session["total_absolute_rotation_deg"] += abs(raw)
            session.update(state="running", stop_confirmed=False, action_id=None)
            self.receipts.append(request_id)
            try:
                result = self.owner._do_destination({"angle_deg": raw, "_observation_token": self.token}, "rotate")
            except Exception as exc:
                result = self._error("ROTATE_EXCEPTION", str(exc))
            action_id = result.get("action_id") if isinstance(result, dict) else None
            if not isinstance(action_id, str) or not action_id or result.get("state") != "running":
                session.update(state="locked", cancelled=True, error="rotation admission failed or action_id missing")
            else:
                session["action_id"] = action_id
                self.monitor = threading.Thread(target=self._watch, args=(session, raw), daemon=True,
                                                name="bumi_observation_monitor")
                try:
                    self.monitor.start()
                except Exception as exc:
                    session.update(state="locked", cancelled=True, error=f"monitor start failed: {exc}")
                else:
                    return {**result, "session_id": session["session_id"], "stop_confirmed": False}
        self._stop_and_verify(session)
        return self.status(session["session_id"])

    def on_stop(self):
        # Called before owner.stop_move publishes its brake, including interrupts.
        with self.lock:
            if self.reserved:
                self.session["cancelled"] = True
                self.session["stop_confirmed"] = False
                if self.session["state"] != "locked":
                    self.session["state"] = "stopping"

    def _settled(self):
        deadline = time.monotonic() + self.stop_timeout_s
        previous, samples, detail = None, 0, "no stationary feedback"
        while time.monotonic() < deadline:
            try:
                yaw = self._snapshot()
                if previous is not None:
                    delta = ((yaw-previous+180) % 360)-180
                    samples = samples+1 if abs(delta) <= .3 else 0
                previous = yaw
                if samples >= SETTLE_SAMPLES:
                    return True, None
            except Exception as exc:
                previous, samples, detail = None, 0, str(exc)
            time.sleep(SETTLE_INTERVAL_S)
        return False, detail

    def _stop_and_verify(self, session):
        with self.stop_lock:
            error = None
            try:
                self.owner._stop_move()
                if not self.owner._wait_for_rotate_idle(timeout_s=self.stop_timeout_s):
                    raise RuntimeError("rotation worker has not stopped")
                confirmed, error = self._settled()
            except Exception as exc:
                confirmed, error = False, str(exc)
            with self.lock:
                if self.session is session:
                    session["stop_confirmed"] = confirmed
                    if not confirmed:
                        session.update(state="locked", error=error, cancelled=True)
                    elif session["state"] != "locked":
                        session["state"] = "cancelled"
                    return confirmed
            return False

    def _watch(self, session, requested):
        try:
            deadline = time.monotonic()+self.action_timeout_s
            while time.monotonic() < deadline:
                with self.lock:
                    if self.session is not session or session["cancelled"]:
                        break
                status = self.owner._rotate_status()
                if status["state"] == "idle":
                    result = status.get("last_result")
                    if (status.get("last_action_id") != session["action_id"]
                            or not isinstance(result, dict) or result.get("state") != "done"):
                        raise RuntimeError("rotation terminal result is missing, mismatched or unsuccessful")
                    confirmed, error = self._settled()
                    if not confirmed:
                        raise RuntimeError("stop not confirmed: "+str(error))
                    turned = result.get("turned_deg")
                    if type(turned) not in (int, float) or not math.isfinite(turned):
                        raise RuntimeError("actual turn feedback missing")
                    with self.lock:
                        if session["cancelled"]:
                            break
                        # Never refund undershoot; account conservatively for overshoot.
                        session["total_absolute_rotation_deg"] += max(0., abs(turned)-abs(requested))
                        if session["total_absolute_rotation_deg"] > MAX_TOTAL_ABSOLUTE_ROTATION_DEG+1e-9:
                            raise RuntimeError("actual absolute rotation exceeded the session budget")
                        session.update(state="ready", last_result=deepcopy(result), stop_confirmed=True,
                                       observation_after_ns=time.time_ns())
                    return
                time.sleep(.02)
            else:
                raise TimeoutError("observation rotation timeout")
        except Exception as exc:
            with self.lock:
                session.update(state="locked", error=str(exc), cancelled=True)
        self._stop_and_verify(session)

    def status(self, session_id):
        with self.lock:
            if self.session is None or session_id != self.session["session_id"]:
                return self._error("INVALID_SESSION", "unknown session_id")
            return {**deepcopy(self.session),
                    "limits": {"single_rotation_deg": MAX_SINGLE_ROTATION_DEG,
                               "rotation_count": MAX_ROTATION_COUNT,
                               "total_absolute_rotation_deg": MAX_TOTAL_ABSOLUTE_ROTATION_DEG},
                    "stop_confirmation_source": "SDK joint/IMU stationary samples; no independent hardware acknowledgement"}

    def end(self, session_id):
        # Serialize the complete release operation, not just brake publication.
        # A second end for an old session must never stop a newly opened one.
        with self.end_lock:
            return self._end(session_id)

    def _end(self, session_id):
        with self.lock:
            if self.session is None or session_id != self.session["session_id"]:
                return self._error("INVALID_SESSION", "unknown session_id")
            session = self.session
            if session["released"]:
                return self.status(session_id)
            was_error = session["error"] is not None
            session["cancelled"] = True
            if not was_error:
                session["state"] = "stopping"
        confirmed = self._stop_and_verify(session)
        monitor = self.monitor
        if monitor is not None and monitor is not threading.current_thread():
            monitor.join(timeout=self.stop_timeout_s)
        with self.lock:
            if confirmed and (monitor is None or not monitor.is_alive()):
                session.update(state="error" if session["error"] else "ended", released=True)
            else:
                session.update(state="locked", error=session["error"] or "stop/monitor exit not confirmed")
            return self.status(session_id)
