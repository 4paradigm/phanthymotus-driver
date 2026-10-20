"""Bounded IPC to the Slamware C++ SDK worker; no motion on construction/connect.

One request in flight, no command queue and no automatic reconnect/replay. The
worker owns its velocity action and a one-second firmware heartbeat. It also
checks the command lease independently of ROS and of this Python process.
"""

from __future__ import annotations

import math
import os
import select
import subprocess
import threading
import time


class SlamtecVelocityBackend:
    def __init__(self, config):
        self._config = dict(config)
        self._process = None
        self._lock = threading.RLock()
        self._buffer = b""
        self._sequence = 0
        self._faulted = False
        self._armed = False
        self._unsafe_until = 0.0
        self._error = ""
        self._action_id = 0
        self._state = "disconnected"
        self._timeout = float(config.get("sdk_reply_timeout", 0.25))
        if not math.isfinite(self._timeout) or not 0.05 <= self._timeout <= 0.3:
            raise ValueError("sdk_reply_timeout must be between 0.05 and 0.3 seconds")
        self._connect_ms = int(config.get("sdk_connect_timeout_ms", 1500))
        if not 100 <= self._connect_ms <= 5000:
            raise ValueError("sdk_connect_timeout_ms must be between 100 and 5000")
        self._host = str(config.get("sdk_host", "192.168.11.1")).strip()
        self._port = int(config.get("sdk_port", 1445))
        if not self._host or not 1 <= self._port <= 65535:
            raise ValueError("invalid Slamware SDK host/port")

    def connect(self):
        with self._lock:
            if self._faulted:
                raise RuntimeError(f"SDK backend faulted; stop/start the card: {self._error}")
            if self._process is not None:
                if self._process.poll() is None:
                    return
                self._fail("SDK worker exited")
            binary = str(self._config.get("sdk_binary", "/usr/local/bin/tianyi-loco-sdk"))
            args = [binary, self._host, str(self._port), str(self._connect_ms),
                    str(self._config.get("max_vx", 0.3)),
                    str(self._config.get("max_wz", 0.6))]
            try:
                self._process = subprocess.Popen(
                    args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    # SDK diagnostics go to container logs, never an unread pipe.
                    stderr=None, bufsize=0, close_fds=True)
                os.set_blocking(self._process.stdout.fileno(), False)
                self._reply(0, self._connect_ms / 1000 + 1.0)
            except Exception as exc:
                self._fail(f"SDK connection failed ({self._host}:{self._port}): {exc}")

    def _reply(self, sequence, timeout):
        deadline = time.monotonic() + timeout
        while True:
            while b"\n" in self._buffer:
                line, self._buffer = self._buffer.split(b"\n", 1)
                if not line.startswith(b"MOTUS_SDK "):
                    continue  # SDK versions may log to stdout.
                fields = line.decode("ascii").split()
                if len(fields) < 4 or int(fields[1]) != sequence:
                    raise RuntimeError("SDK reply sequence mismatch")
                if fields[2] != "OK":
                    message = bytes.fromhex(fields[3]).decode("utf-8", "replace")
                    raise RuntimeError(message)
                self._action_id = int(fields[3])
                self._state = fields[4] if len(fields) > 4 else "ready"
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("SDK worker reply timed out")
            readable, _, _ = select.select([self._process.stdout], [], [], remaining)
            if not readable:
                raise TimeoutError("SDK worker reply timed out")
            chunk = os.read(self._process.stdout.fileno(), 4096)
            if not chunk:
                raise RuntimeError("SDK worker closed its output")
            self._buffer += chunk
            if len(self._buffer) > 65536:
                raise RuntimeError("SDK worker reply exceeded size limit")

    def _request(self, command):
        if self._faulted:
            raise RuntimeError(self._error or "SDK worker is not connected")
        if self._process is None or self._process.poll() is not None:
            self._fail("SDK worker is not connected")
        self._sequence += 1
        try:
            data = f"{self._sequence} {command}\n".encode("ascii")
            self._process.stdin.write(data)
            self._reply(self._sequence, self._timeout)
        except Exception as exc:
            self._fail(str(exc))

    def write(self, values, deadline_ms):
        with self._lock:
            if len(values) != 6 or any(not math.isfinite(v) for v in values):
                raise ValueError("SDK velocity requires six finite values")
            if any(values[i] for i in (1, 2, 3, 4)):
                raise ValueError("SDK adapter supports vx/wz only")
            if not any(values):
                return self.stop()
            if deadline_ms <= time.time() * 1000:
                raise ValueError("SDK command lease expired")
            # Even an unacknowledged first write might have reached the chassis.
            self._armed = True
            self._request(f"SET {int(deadline_ms)} {values[0]:.17g} {values[5]:.17g}")

    def stop(self):
        with self._lock:
            if self._faulted:
                # A killed worker cannot refresh its firmware heartbeat. Do not
                # hand over to another card during that outstanding lease.
                if time.monotonic() < self._unsafe_until:
                    raise RuntimeError("SDK stop unconfirmed; waiting for firmware heartbeat expiry")
                self._armed = False
                return
            if self._process is not None:
                self._request("STOP")
            self._armed = False

    def _fail(self, message):
        self._error = message
        self._faulted = True
        self._state = "error"
        if self._armed:
            self._unsafe_until = max(self._unsafe_until, time.monotonic() + 1.5)
        if self._process is not None and self._process.poll() is None:
            self._process.kill()
            self._process.wait(timeout=1)
        raise RuntimeError(message)

    def close(self):
        with self._lock:
            try:
                self.stop()
            finally:
                if self._process is not None:
                    if self._process.poll() is None:
                        self._process.kill()
                        self._process.wait(timeout=1)
                    self._process.stdin.close()
                    self._process.stdout.close()
                    self._process = None

    def info(self):
        with self._lock:
            alive = self._process is not None and self._process.poll() is None
            return {"type": "slamware_sdk", "host": self._host, "port": self._port,
                    "connected": alive and not self._faulted,
                    "state": self._state if alive or self._faulted else "disconnected",
                    "action_id": self._action_id, "error": self._error,
                    "firmware_heartbeat_timeout_s": 1,
                    "handover_wait_ms": max(0, int((self._unsafe_until - time.monotonic()) * 1000))}
