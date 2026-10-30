"""Supervise an independent, subscription-only vendor ROS process.

Fast DDS profile selection is process global. The platform process never joins
the robot domain, and the child never receives the platform loopback profile.
"""
from __future__ import annotations

import base64
import copy
import json
import logging
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    from .reader import DEFAULT_SOURCES
except ImportError:  # Direct execution in the driver container.
    from reader import DEFAULT_SOURCES

LOG = logging.getLogger("s10.reader")
SOURCES = frozenset(DEFAULT_SOURCES)
MAX_LINE_BYTES = 2_000_000


def reader_environment(config, inherited=None):
    env = dict(os.environ if inherited is None else inherited)
    for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
                "FASTDDS_BUILTIN_TRANSPORTS", "CYCLONEDDS_URI", "ROS_LOCALHOST_ONLY",
                "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_STATIC_PEERS"):
        env.pop(key, None)
    reader = config.get("reader", {})
    env.update(ROS_DOMAIN_ID=str(int(reader.get("domain_id", 0))),
               RMW_IMPLEMENTATION="rmw_fastrtps_cpp", PYTHONUNBUFFERED="1",
               PYTHONDONTWRITEBYTECODE="1", RCUTILS_COLORIZED_OUTPUT="0",
               RCUTILS_LOGGING_USE_STDOUT="0")
    # Only an explicitly supplied vendor profile is forwarded. Never modify it.
    profile = reader.get("vendor_profile")
    if env.get("S10_REQUIRE_VENDOR_PROFILE") == "1" and not profile:
        raise ValueError("deployment requires an explicit reader vendor profile")
    if profile:
        if not Path(profile).is_file():
            raise ValueError("configured reader vendor profile is not a file")
        env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile)
    return env


class ReaderBridge:
    def __init__(self, config, config_path, *, process_factory=subprocess.Popen, clock=time.monotonic):
        self.config = config
        self.config_path = str(Path(config_path).resolve())
        self._factory = process_factory
        self._clock = clock
        self._lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._owners = {}
        self._activation = {}
        self._cache = {}
        self._sequence = 0
        self._process = None
        self._last_error = None
        self._closed = threading.Event()
        self._wake = threading.Event()
        self._thread = threading.Thread(target=self._supervise, name="s10-reader-supervisor", daemon=True)
        self._thread.start()

    def acquire(self, owner, sources, options=None):
        sources = set(sources)
        if not sources or not sources <= SOURCES:
            raise ValueError("unsupported S10 state source")
        with self._lock:
            if self._closed.is_set():
                raise RuntimeError("reader bridge is closed")
            previous = self._active_locked()
            self._owners[str(owner)] = (sources, dict(options or {}))
            # Old samples from a previous activation cannot be labelled live.
            for source in sources - previous:
                self._cache.pop(source, None)
                self._activation[source] = self._clock()
        self._wake.set()

    def release(self, owner):
        with self._lock:
            self._owners.pop(str(owner), None)
            active = self._active_locked()
            self._cache = {k: v for k, v in self._cache.items() if k in active}
            self._activation = {k: v for k, v in self._activation.items() if k in active}
        self._wake.set()

    def _active_locked(self):
        return set().union(*(entry[0] for entry in self._owners.values())) if self._owners else set()

    def _control_locked(self):
        active = self._active_locked()
        rates = {}
        max_points = 40000
        for sources, options in self._owners.values():
            rate = float(options.get("refresh_hz", 5))
            if not math.isfinite(rate) or not 0.1 <= rate <= 30:
                raise ValueError("refresh_hz must be within 0.1..30")
            for source in sources:
                rates[source] = max(rates.get(source, 0), rate)
            if "lidar" in sources:
                max_points = int(options.get("max_points", 40000))
        return {"active": sorted(active), "rates_hz": rates,
                "max_points": max(100, min(40000, max_points))}

    def snapshot(self, source):
        with self._lock:
            record = self._cache.get(source)
            error = self._last_error
            if record is None:
                return {"data": None, "received_at_ms": None, "age_s": None,
                        "fresh": False, "error": error, "sequence": 0, "binary": None}
            result = copy.deepcopy(record)
            age = max(0, self._clock() - result["received_monotonic"])
            threshold = max((float(opts.get("stale_after_s", 5)) for sources, opts in self._owners.values()
                             if source in sources), default=5)
            result.update(age_s=age, fresh=age <= threshold, error=error)
            return result

    def status(self):
        with self._lock:
            process = self._process
            return {"reader_running": process is not None and process.poll() is None,
                    "reader_pid": process.pid if process is not None else None,
                    "active_sources": sorted(self._active_locked()), "last_error": self._last_error,
                    "read_only": True}

    def _accept_event(self, event):
        key = event.get("key")
        now = self._clock()
        stamp = event.get("received_monotonic")
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp > now + 1:
            return False
        # Parent and reader are on the same host and share CLOCK_MONOTONIC.
        # Reject old pipe backlog even if the child remains alive.
        if now - stamp > 10 or not isinstance(event.get("data"), dict):
            return False
        binary = None
        if key == "lidar":
            try:
                binary = base64.b64decode(event.get("binary_base64", ""), validate=True)
            except (ValueError, TypeError):
                return False
            if len(binary) > 640008 or len(binary) < 8:
                return False
        with self._lock:
            if key not in self._active_locked():
                return False
            if stamp < self._activation.get(key, float("inf")):
                return False
            previous = self._cache.get(key)
            if previous and stamp < previous["received_monotonic"]:
                return False
            self._sequence += 1
            self._cache[key] = {"data": event["data"], "binary": binary,
                                "received_monotonic": stamp,
                                "received_at_ms": event.get("received_at_ms"),
                                "sequence": self._sequence}
            self._last_error = None
        return True

    def _read_output(self, process):
        try:
            while not self._closed.is_set():
                line = process.stdout.readline(MAX_LINE_BYTES + 1)
                if not line:
                    break
                if len(line) > MAX_LINE_BYTES:
                    process.terminate()
                    raise ValueError("reader exceeded IPC line size limit")
                try:
                    self._accept_event(json.loads(line))
                except (ValueError, TypeError, AttributeError):
                    LOG.warning("discarded malformed reader event")
        except Exception as exc:
            with self._lock:
                self._last_error = type(exc).__name__

    def _read_errors(self, process):
        for line in process.stderr:
            LOG.warning("%s", line.rstrip()[:600])

    def _start_process(self):
        reader = self.config.get("reader", {})
        python = str(reader.get("python", sys.executable))
        command = [python, str(Path(__file__).with_name("reader.py")), "--config", self.config_path]
        process = self._factory(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, bufsize=1,
                                env=reader_environment(self.config), start_new_session=True)
        with self._lock:
            self._process = process
            self._last_error = None
        for target, label in ((self._read_output, "events"), (self._read_errors, "stderr")):
            threading.Thread(target=target, args=(process,), daemon=True, name=f"s10-reader-{label}").start()
        return process

    @staticmethod
    def _terminate(process):
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        for stream in (process.stdin, process.stdout, process.stderr):
            if stream:
                stream.close()

    def _supervise(self):
        previous = None
        while not self._closed.is_set():
            self._wake.wait(timeout=0.5)
            self._wake.clear()
            with self._lock:
                active = bool(self._owners)
                process = self._process
                control = self._control_locked()
            if not active:
                self._terminate(process)
                with self._lock:
                    self._process = None
                previous = None
                continue
            try:
                if process is None or process.poll() is not None:
                    self._terminate(process)
                    process = self._start_process()
                    previous = None
                if control != previous:
                    with self._write_lock:
                        process.stdin.write(json.dumps(control) + "\n")
                        process.stdin.flush()
                    previous = control
            except Exception as exc:
                with self._lock:
                    self._last_error = f"reader startup failed: {type(exc).__name__}"
                LOG.warning("reader unavailable: %s", type(exc).__name__)
                self._closed.wait(timeout=2)
        with self._lock:
            process, self._process = self._process, None
        self._terminate(process)

    def close(self):
        self._closed.set()
        self._wake.set()
        self._thread.join(timeout=7)
