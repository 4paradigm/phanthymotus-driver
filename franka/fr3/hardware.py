"""One native state/control connection per Franka IP, shared by canvas cards."""

from __future__ import annotations

import copy
import ipaddress
import json
import math
import os
import selectors
import subprocess
import threading
import time
import uuid


# Match the FR3 URDF and native controller; leave a margin at each hard limit.
JOINT_LOWER = (-2.9007, -1.8361, -2.9007, -3.077, -2.8763, 0.4398, -3.0508)
JOINT_UPPER = (2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508)
JOINT_MARGIN = 0.01
TERMINAL_STATUSES = frozenset(("succeeded", "cancelled", "failed", "rejected"))


def _joints(values, name):
    if (not isinstance(values, (list, tuple)) or len(values) != 7
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in values)):
        raise ValueError(f"{name} must contain seven finite numbers")
    return list(values)


def canonical_ip(value):
    """Validate an IP literal; equivalent addresses share one hardware session."""
    if not isinstance(value, str) or not value.strip() or "%" in value:
        raise ValueError("robot_ip must be an IPv4 or IPv6 address without a port or scope")
    try:
        address = ipaddress.ip_address(value.strip())
    except ValueError as exc:
        raise ValueError("robot_ip must be a valid IPv4 or IPv6 address") from exc
    return str(getattr(address, "ipv4_mapped", None) or address)


class RobotSession:
    """Supervise a single SDK owner and arbitrate one finite motion at a time."""

    def __init__(self, address, *, reader_path=None, process_factory=subprocess.Popen,
                 connect_timeout=5.0, stale_after=2.0, retry_delay=1.0,
                 retry_max=10.0, stop_timeout=0.5, motion_stop_timeout=3.0):
        self.address = canonical_ip(address)
        self._reader_path = reader_path or os.environ.get(
            "FRANKA_STATE_READER", "/usr/local/bin/franka-state-reader")
        self._process_factory = process_factory
        for name, value in (("connect_timeout", connect_timeout), ("stale_after", stale_after),
                            ("retry_delay", retry_delay), ("retry_max", retry_max),
                            ("stop_timeout", stop_timeout),
                            ("motion_stop_timeout", motion_stop_timeout)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self._connect_timeout, self._stale_after = connect_timeout, stale_after
        self._retry_delay, self._retry_max = retry_delay, max(retry_delay, retry_max)
        self._stop_timeout = stop_timeout
        self._motion_stop_timeout = motion_stop_timeout
        self._lock = threading.Lock()
        self._changed = threading.Condition(self._lock)
        self._close_lock = threading.Lock()
        self._stop = threading.Event()
        self._closing = False
        self._shutdown_motion = None
        self._close_checked = False
        self._had_motion = False
        self._connection_broken = False
        self._owners = set()
        self._motions = {}
        self._active = None
        self._latest_motion = None
        self._sample = None
        self._received = None
        self._process = None
        self._state = "connecting"
        self._error = None
        self._thread = threading.Thread(target=self._run, name=f"franka-{self.address}", daemon=True)
        self._thread.start()

    def snapshot(self):
        """Return a copy without blocking on robot I/O; never expose stale joints."""
        with self._lock:
            age = None if self._received is None else max(0.0, time.monotonic() - self._received)
            state = self._state
            if state == "connected" and (age is None or age > self._stale_after):
                state = "stale"
            result = {"robot_ip": self.address, "state": state,
                      "connected": state == "connected",
                      "data_age_ms": None if age is None else round(age * 1000),
                      "error": "Robot state stream is stale" if state == "stale" else self._error}
            result.update(control_ready=False, control_error=result["error"])
            if state == "connected" and self._sample is not None:
                result.update(copy.deepcopy(self._sample))
            if self._closing:
                result.update(control_ready=False, control_error="Franka session is stopping")
            result["motion"] = self._motion_locked(self._latest_motion)
            return result

    def close(self):
        with self._close_lock:
            self.begin_close()
            with self._lock:
                command_id = None if self._close_checked else self._shutdown_motion
            stop_error = None
            if command_id is not None:
                try:
                    result = self.wait(command_id, self._motion_stop_timeout)
                except TimeoutError:
                    with self._changed:
                        self._break_locked("Motion stop was not acknowledged before shutdown")
                        result = self._motion_locked(command_id)
                if result["status"] == "failed":
                    stop_error = "Physical motion stop was not confirmed: " + str(result["error"])
            self._stop.set()
            grace = self._motion_stop_timeout if self._had_motion else self._stop_timeout
            self._thread.join(timeout=grace + self._stop_timeout + 1.0)
            if self._thread.is_alive():
                raise RuntimeError(f"Reader for {self.address} did not stop within its deadline")
            if self._process is not None and self._process.poll() is None:
                raise RuntimeError(f"Reader for {self.address} is still terminating")
            self._close_checked = True
            if stop_error is not None:
                self._status("stopped", stop_error)
                raise RuntimeError(stop_error)

    def begin_close(self):
        """Block new moves and ask the physical backend to stop before exit."""
        with self._changed:
            self._closing = True
            if self._active is not None:
                self._shutdown_motion = self._active
                try:
                    self._stop_motion_locked(self._active)
                except RuntimeError:
                    pass  # A broken pipe already fails the action; still reap it.

    def register_owner(self, owner):
        hash(owner)
        with self._lock:
            if self._closing or self._stop.is_set():
                raise RuntimeError("Franka session is stopping")
            self._owners.add(owner)

    def release_owner(self, owner):
        """Release a card, stopping only motion that card owns, even with readers left."""
        with self._changed:
            self._owners.discard(owner)
            command_id = self._active
            if command_id is None or self._motions[command_id]["owner"] != owner:
                return
            try:
                self._stop_motion_locked(command_id)
            except RuntimeError:
                pass  # The supervisor will close the broken connection.
        try:
            result = self.wait(command_id, self._motion_stop_timeout)
        except TimeoutError:
            with self._changed:
                self._break_locked("Motion stop was not acknowledged after owner release")
            raise RuntimeError("Motion stop timed out; resetting the native connection") from None
        if result["status"] == "failed":
            raise RuntimeError("Physical motion stop was not confirmed: " + str(result["error"]))

    def move(self, owner, target_rad, speed_rad_s=math.radians(5),
             tolerance_rad=math.radians(0.5), timeout_s=120.0):
        """Submit one target in radians; never queue or replay a robot command."""
        target = _joints(target_rad, "target_rad")
        for index, (value, low, high) in enumerate(zip(target, JOINT_LOWER, JOINT_UPPER), 1):
            if not low + JOINT_MARGIN <= value <= high - JOINT_MARGIN:
                raise ValueError(f"J{index} target is outside the permitted FR3 range")
        for name, value, low, high in (
                ("speed_rad_s", speed_rad_s, math.radians(0.1), math.radians(10)),
                ("tolerance_rad", tolerance_rad, 0.0, math.radians(1)),
                ("timeout_s", timeout_s, 1.0, 600.0)):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < low or value > high
                    or (name == "tolerance_rad" and value == 0)):
                raise ValueError(f"{name} is outside its permitted range")
        fields = (speed_rad_s, tolerance_rad, timeout_s, *target)
        return self._submit_motion(owner, target, "MOVE", fields, timeout_s)

    def move_path(self, owner, points, speed_rad_s=math.radians(2),
                  tolerance_rad=math.radians(.1), timeout_s=600):
        """Submit one continuous, bounded cubic spline; never queue or replay it."""
        points = [_joints(point, "path point") for point in points]
        if not 2 <= len(points) <= 21:
            raise ValueError("Path requires 2 to 21 control points")
        for point in points:
            if any(not low+JOINT_MARGIN <= q <= high-JOINT_MARGIN
                   for q,low,high in zip(point,JOINT_LOWER,JOINT_UPPER)):
                raise ValueError("Path point is outside FR3 joint limits")
        if (not math.isfinite(speed_rad_s) or not math.radians(.1) <= speed_rad_s <= math.radians(5)
                or not math.isfinite(tolerance_rad) or not 0 < tolerance_rad <= math.radians(1)
                or not math.isfinite(timeout_s) or not 1 <= timeout_s <= 600):
            raise ValueError("Invalid continuous path speed, tolerance or timeout")
        fields = (speed_rad_s,tolerance_rad,timeout_s,len(points),
                  *(v for point in points for v in point))
        return self._submit_motion(owner, points[-1], "PATH", fields, timeout_s)

    def _submit_motion(self, owner, target, verb, fields, timeout_s, *, validate=None, stop=False):
        """Atomically validate ownership/state and submit one non-replayed command."""
        with self._changed:
            if owner not in self._owners:
                raise RuntimeError("Control card does not own this session")
            if self._closing or self._stop.is_set():
                raise RuntimeError("Franka session is stopping")
            if stop and self._active is not None:
                command_id = self._active
                self._motions[command_id]["_stop_ack_required"] = True
                self._stop_motion_locked(command_id)
                return command_id
            if not stop and (self._state != "connected" or self._connection_broken
                    or self._received is None
                    or time.monotonic() - self._received > self._stale_after):
                raise RuntimeError("Fresh robot state is required before motion")
            if not stop and not self._sample.get("control_ready", False):
                raise RuntimeError(self._sample.get("control_error")
                                   or "Native backend does not support joint control")
            if self._active is not None:
                raise RuntimeError("This robot already has an active motion; stop it before another move")
            if validate is not None:
                validate(self._sample)
            # Keep unconsumed terminal records for ACP waiters; bound memory even
            # if a caller submits commands and never collects their results.
            for key in list(self._motions):
                if len(self._motions) < 64:
                    break
                if self._motions[key]["consumed"]:
                    del self._motions[key]
            if len(self._motions) >= 256:
                raise RuntimeError("Collect previous motion results before submitting another move")
            command_id = uuid.uuid4().hex
            self._motions[command_id] = {
                "command_id": command_id, "status": "submitted", "target": target,
                "owner": owner, "error": None, "consumed": False,
                "_deadline": time.monotonic() + timeout_s + self._motion_stop_timeout,
            }
            self._active = self._latest_motion = command_id
            self._had_motion = True
            if stop:
                self._motions[command_id].update(
                    status="stopping", _stop_deadline=time.monotonic() + self._motion_stop_timeout)
            self._send_locked(verb + " " + command_id + " " + " ".join(format(v, ".17g") for v in fields))
            return command_id

    def _motion_locked(self, command_id):
        if command_id is None:
            return {"status": "idle"}
        if command_id not in self._motions:
            raise ValueError("Unknown motion command_id")
        return copy.deepcopy({key: value for key, value in self._motions[command_id].items()
                              if key not in ("owner", "consumed") and not key.startswith("_")})

    def motion(self, command_id=None):
        with self._lock:
            return self._motion_locked(command_id or self._latest_motion)

    def wait(self, command_id, timeout=None):
        """Wait for actual backend completion, not acceptance or a stop request."""
        if timeout is not None and (not math.isfinite(timeout) or timeout < 0):
            raise ValueError("timeout must be finite and non-negative")
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._changed:
            while True:
                result = self._motion_locked(command_id)
                if result["status"] in TERMINAL_STATUSES:
                    self._motions[command_id]["consumed"] = True
                    return result
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    raise TimeoutError("Motion has not reached a terminal state")
                self._changed.wait(remaining)

    def stop_motion(self, owner=None, command_id=None):
        """Send STOP to the backend; owner=None is an explicit robot-wide stop."""
        with self._changed:
            selected = command_id or self._active
            if selected is None:
                return {"status": "idle"}
            result = self._motion_locked(selected)
            if owner is not None and self._motions[selected]["owner"] != owner:
                raise RuntimeError("This motion belongs to another control card")
            if result["status"] not in TERMINAL_STATUSES:
                self._stop_motion_locked(selected)
            return self._motion_locked(selected)

    def _stop_motion_locked(self, command_id):
        record = self._motions[command_id]
        if record["status"] == "stopping" or record["status"] in TERMINAL_STATUSES:
            return
        record["status"] = "stopping"
        record["_stop_deadline"] = time.monotonic() + self._motion_stop_timeout
        self._send_locked("STOP " + command_id)

    def _send_locked(self, command):
        try:
            if self._connection_broken or self._process is None or self._process.stdin is None:
                raise ConnectionError("Native command channel is unavailable")
            encoded = (command + "\n").encode("ascii")
            # Commands are shorter than POSIX PIPE_BUF. A nonblocking atomic
            # write cannot hold a card callback behind a wedged SDK process.
            if os.write(self._process.stdin.fileno(), encoded) != len(encoded):
                raise ConnectionError("Native command channel accepted a partial command")
        except (OSError, ValueError) as exc:
            self._break_locked(f"Native command channel failed: {exc}")
            raise RuntimeError(self._error) from exc

    def _finish_locked(self, status, error=None):
        if self._active is not None:
            record = self._motions[self._active]
            record.update(status=status, error=error)
            self._active = None
            self._changed.notify_all()

    def _break_locked(self, error):
        self._connection_broken = True
        self._state, self._error = "disconnected", str(error)[-4096:]
        self._finish_locked("failed", self._error)

    @property
    def close_deadline(self):
        return 2 * self._motion_stop_timeout + 2 * self._stop_timeout + 1.0

    @property
    def stopping(self):
        return self._closing or self._stop.is_set()

    @property
    def stopped(self):
        return (self.stopping and not self._thread.is_alive()
                and (self._process is None or self._process.poll() is not None))

    def _status(self, state, error=None):
        with self._lock:
            self._state, self._error = state, error

    def _accept(self, line):
        record = json.loads(line)
        if not isinstance(record, dict):
            raise ValueError("Reader state must be a JSON object")
        if "event" in record:
            self._accept_event(record)
            return False
        sample = {}
        for name in ("q", "dq", "tau_J"):
            values = record.get(name)
            sample[name] = _joints(values, "Reader " + name)
        ready = record.get("control_ready", False)
        if not isinstance(ready, bool):
            raise ValueError("Reader control_ready must be boolean")
        control_error = record.get("control_error")
        if control_error is not None and not isinstance(control_error, str):
            raise ValueError("Reader control_error must be text or null")
        sample.update(control_ready=ready, control_error=control_error)
        for name in ("rt_kernel", "sched_fifo"):
            if name in record:
                if not isinstance(record[name], bool):
                    raise ValueError(f"Reader {name} must be boolean")
                sample[name] = record[name]
        for name in ("q_d", "target"):
            if name in record or (name == "q_d" and ready):
                sample[name] = _joints(record.get(name), "Reader " + name)
        mode = record.get("robot_mode")
        if isinstance(mode, bool) or not isinstance(mode, int) or not 0 <= mode <= 6:
            raise ValueError("Reader robot_mode must be a libfranka RobotMode value")
        sample.update(robot_mode=mode, timestamp_ms=int(time.time() * 1000))
        for name in ("O_T_EE", "F_T_EE"):
            if name in record:
                pose = record[name]
                if (not isinstance(pose, list) or len(pose) != 16 or
                        any(type(v) not in (int, float) or not math.isfinite(v) for v in pose)):
                    raise ValueError(f"Reader {name} must contain sixteen finite numbers")
                sample[name] = list(pose)
        with self._lock:
            if self._stop.is_set() or self._connection_broken:
                return False
            self._sample, self._received = sample, time.monotonic()
            self._state, self._error = "connected", None
        return True

    def _accept_event(self, event):
        kind, status, command_id = (event.get(key) for key in ("event", "status", "command_id"))
        valid = {"motion": {"accepted", "running", *TERMINAL_STATUSES},
                 "stop": {"stopped", "idle", "failed"}}
        if (not isinstance(kind, str) or kind not in valid
                or not isinstance(status, str) or status not in valid[kind]
                or not isinstance(command_id, str)):
            raise ValueError("Invalid native motion event")
        error = event.get("error")
        if error is not None and not isinstance(error, str):
            raise ValueError("Motion error must be text or null")
        maximum = event.get("max_error")
        if maximum is not None and (isinstance(maximum, bool)
                                    or not isinstance(maximum, (int, float))
                                    or not math.isfinite(maximum) or maximum < 0):
            raise ValueError("Motion max_error must be finite and non-negative")
        with self._changed:
            if command_id != self._active:
                return  # Delayed/duplicate events can never complete a new move.
            record = self._motions[command_id]
            if maximum is not None:
                record["max_error"] = maximum
            if kind == "stop":
                if status == "failed":
                    self._break_locked(error or "Native backend failed to stop motion")
                else:
                    record["stop_status"] = status
            elif status in TERMINAL_STATUSES:
                self._finish_locked(status, error)
            elif record["status"] != "stopping":
                record["status"] = status
            self._changed.notify_all()

    def _stream(self, process):
        """Drain both pipes; cap malformed stdout and stderr instead of blocking."""
        started = time.monotonic()
        last_valid = None
        pending = bytearray()
        stderr = bytearray()
        with selectors.DefaultSelector() as selector:
            for pipe, label in ((process.stdout, "stdout"), (process.stderr, "stderr")):
                os.set_blocking(pipe.fileno(), False)
                selector.register(pipe, selectors.EVENT_READ, label)
            while not self._stop.is_set():
                for key, _ in selector.select(timeout=0.05):
                    chunk = os.read(key.fileobj.fileno(), 4096)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        continue
                    if key.data == "stderr":
                        stderr.extend(chunk)
                        del stderr[:-4096]
                        continue
                    pending.extend(chunk)
                    while b"\n" in pending:
                        line, _, remainder = pending.partition(b"\n")
                        pending = bytearray(remainder)
                        if len(line) > 8192:
                            raise ValueError("Reader state exceeds 8192 bytes")
                        if self._accept(line):
                            last_valid = time.monotonic()
                    if len(pending) > 8192:
                        raise ValueError("Reader state exceeds 8192 bytes")
                if process.poll() is not None:
                    detail = stderr.decode("utf-8", errors="replace").strip()
                    raise ConnectionError(detail or f"Reader exited with status {process.returncode}")
                now = time.monotonic()
                with self._lock:
                    if self._connection_broken:
                        raise ConnectionError(self._error)
                    if self._active is not None:
                        motion = self._motions[self._active]
                        if now > motion.get("_stop_deadline", math.inf):
                            raise TimeoutError("Motion stop was not acknowledged before its deadline")
                        if now > motion["_deadline"]:
                            raise TimeoutError("Motion completion was not acknowledged before its deadline")
                if last_valid is None and now - started > self._connect_timeout:
                    raise TimeoutError("Timed out waiting for the first robot state; check IP, FCI and SDK compatibility")
                if last_valid is not None and now - last_valid > self._stale_after:
                    raise TimeoutError("Robot state stream is stale")

    def _dispose(self, process):
        try:
            if process.poll() is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass
            try:
                grace = self._motion_stop_timeout if self._had_motion else self._stop_timeout
                process.wait(timeout=grace)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
                process.wait(timeout=self._stop_timeout)
        finally:
            for pipe in (process.stdin, process.stdout, process.stderr):
                if pipe is not None:
                    pipe.close()

    def _run(self):
        delay = self._retry_delay
        while not self._stop.is_set():
            process = None
            attempt_started = time.monotonic()
            self._status("connecting", self._error)
            try:
                process = self._process_factory(
                    [self._reader_path, self.address], stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
                os.set_blocking(process.stdin.fileno(), False)
                with self._lock:
                    self._connection_broken = False
                    self._had_motion = False
                    self._process = process
                self._stream(process)
            except (OSError, ValueError, TypeError, OverflowError, RecursionError) as exc:
                with self._changed:
                    self._break_locked(str(exc))
            finally:
                with self._changed:
                    self._finish_locked("failed", "Native connection closed before motion completed")
                if process is not None:
                    try:
                        self._dispose(process)
                    except (OSError, subprocess.TimeoutExpired) as exc:
                        # Never start a second reader while an old SDK process
                        # is still alive, even if the OS cannot reap it promptly.
                        self._status("error", f"Could not stop reader: {exc}"[-4096:])
                        return
            if self._stop.is_set():
                break
            # A recovered stream resets backoff; consecutive failures are bounded.
            with self._lock:
                received = self._received
            if received is not None and received >= attempt_started:
                delay = self._retry_delay
            if self._stop.wait(delay):
                break
            delay = min(delay * 2, self._retry_max)
        self._status("stopped", self._error)


class SessionManager:
    """Share sessions by IP and retain each one until its last card releases it."""

    def __init__(self, *, session_factory=None, **session_options):
        self._session_factory = session_factory
        self._options = session_options
        self._lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._sessions = {}
        self._retiring = {}
        self._closed = False

    def acquire(self, address, owner):
        address = canonical_ip(address)
        hash(owner)
        while True:
            with self._lock:
                if self._closed:
                    raise RuntimeError("Franka session manager is closed")
                retired = self._retiring.get(address)
                if retired is None:
                    existing = self._sessions.get(address)
                    if existing is not None and existing[0].stopping:
                        if not existing[0].stopped:
                            raise RuntimeError(f"Previous reader for {address} is still stopping; retry shortly")
                        del self._sessions[address]
                    if address not in self._sessions:
                        factory = self._session_factory or RobotSession
                        self._sessions[address] = (factory(address, **self._options), set())
                    session, owners = self._sessions[address]
                    session.register_owner(owner)
                    owners.add(owner)
                    return session
            # Wait only for this address; stopping another robot cannot block us.
            if not retired[1].wait(timeout=retired[0].close_deadline):
                raise RuntimeError(f"Previous reader for {address} is still stopping; retry shortly")

    def release(self, address, owner):
        address = canonical_ip(address)
        with self._lock:
            entry = self._sessions.get(address)
            if entry is None:
                return
            session, owners = entry
            owners.discard(owner)
            last_owner = not owners
            if last_owner:
                del self._sessions[address]
                retired = threading.Event()
                self._retiring[address] = (session, retired)
        if not last_owner:
            session.release_owner(owner)
            return
        try:
            try:
                session.release_owner(owner)
            finally:
                session.close()
        except RuntimeError:
            with self._lock:
                if not self._closed:
                    self._sessions[address] = (session, set())
            raise
        finally:
            with self._lock:
                del self._retiring[address]
                retired.set()

    def close(self):
        # All callers observe completed cleanup, including a concurrent second
        # shutdown after the first caller has removed the session dictionary.
        with self._close_lock:
            self._close_sessions()

    def _close_sessions(self):
        with self._lock:
            self._closed = True
            sessions = [(address, entry[0]) for address, entry in self._sessions.items()]
            self._sessions.clear()
            retiring = list(self._retiring.items())
        for _, session in sessions:
            session.begin_close()
        for _, (session, _) in retiring:
            session.begin_close()
        failures = []
        for address, (session, retired) in retiring:
            if not retired.wait(timeout=session.close_deadline):
                failures.append(RuntimeError(f"Reader for {address} is still stopping"))
            sessions.append((address, session))
        for address, session in sessions:
            try:
                session.close()
            except RuntimeError as exc:
                failures.append(exc)
                with self._lock:
                    self._sessions[address] = (session, set())
        if failures:
            raise failures[0]
