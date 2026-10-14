"""Read-only Franka sessions shared by cards that specify the same robot IP."""

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
    """One supervised native reader, with no SDK work on a card callback thread."""

    def __init__(self, address, *, reader_path=None, process_factory=subprocess.Popen,
                 connect_timeout=5.0, stale_after=2.0, retry_delay=1.0,
                 retry_max=10.0, stop_timeout=0.5):
        self.address = canonical_ip(address)
        self._reader_path = reader_path or os.environ.get(
            "FRANKA_STATE_READER", "/usr/local/bin/franka-state-reader")
        self._process_factory = process_factory
        for name, value in (("connect_timeout", connect_timeout), ("stale_after", stale_after),
                            ("retry_delay", retry_delay), ("retry_max", retry_max),
                            ("stop_timeout", stop_timeout)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self._connect_timeout, self._stale_after = connect_timeout, stale_after
        self._retry_delay, self._retry_max = retry_delay, max(retry_delay, retry_max)
        self._stop_timeout = stop_timeout
        self._lock = threading.Lock()
        self._stop = threading.Event()
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
            if state == "connected" and self._sample is not None:
                result.update(copy.deepcopy(self._sample))
            return result

    def close(self):
        self._stop.set()
        self._thread.join(timeout=self.close_deadline)
        if self._thread.is_alive():
            raise RuntimeError(f"Reader for {self.address} did not stop within its deadline")
        if self._process is not None and self._process.poll() is None:
            raise RuntimeError(f"Reader for {self.address} is still terminating")

    @property
    def close_deadline(self):
        return 2 * self._stop_timeout + 1.0

    @property
    def stopping(self):
        return self._stop.is_set()

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
        sample = {}
        for name in ("q", "dq", "tau_J"):
            values = record.get(name)
            if (not isinstance(values, list) or len(values) != 7
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) for v in values)):
                raise ValueError(f"Reader {name} must contain seven finite numbers")
            sample[name] = values
        mode = record.get("robot_mode")
        if isinstance(mode, bool) or not isinstance(mode, int) or not 0 <= mode <= 6:
            raise ValueError("Reader robot_mode must be a libfranka RobotMode value")
        sample.update(robot_mode=mode, timestamp_ms=int(time.time() * 1000))
        with self._lock:
            if self._stop.is_set():
                return
            self._sample, self._received = sample, time.monotonic()
            self._state, self._error = "connected", None

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
                        self._accept(line)
                        last_valid = time.monotonic()
                    if len(pending) > 8192:
                        raise ValueError("Reader state exceeds 8192 bytes")
                if process.poll() is not None:
                    detail = stderr.decode("utf-8", errors="replace").strip()
                    raise ConnectionError(detail or f"Reader exited with status {process.returncode}")
                now = time.monotonic()
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
                process.wait(timeout=self._stop_timeout)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
                process.wait(timeout=self._stop_timeout)
        finally:
            for pipe in (process.stdout, process.stderr):
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
                    [self._reader_path, self.address], stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
                self._process = process
                self._stream(process)
            except (OSError, ValueError, TypeError, OverflowError, RecursionError) as exc:
                self._status("disconnected", str(exc)[-4096:])
            finally:
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
        self._status("stopped")


class SessionManager:
    """Share sessions by IP and retain each one until its last card releases it."""

    def __init__(self, **session_options):
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
                        self._sessions[address] = (RobotSession(address, **self._options), set())
                    session, owners = self._sessions[address]
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
            if owners:
                return
            del self._sessions[address]
            retired = threading.Event()
            self._retiring[address] = (session, retired)
        try:
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
            session._stop.set()
        for _, (session, _) in retiring:
            session._stop.set()
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
