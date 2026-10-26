"""Isolate vendor FastDDS services from the bundle's CycloneDDS contexts.

The child owns only the domain-2 SoundSource service path. Probe is a standard
read-only parameter request, not a zero-degree motion disguised as a probe.
"""
from __future__ import annotations

import concurrent.futures
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import time
from types import SimpleNamespace


class HeadServiceTransport:
    def __init__(self):
        root = Path(__file__).resolve().parent
        env = dict(os.environ)
        env.update(RMW_IMPLEMENTATION="rmw_fastrtps_cpp", ROS_DOMAIN_ID="2",
                   FASTRTPS_DEFAULT_PROFILES_FILE=str(root / "head-dds-local.xml"),
                   FASTDDS_DEFAULT_PROFILES_FILE=str(root / "head-dds-local.xml"))
        env.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
        env.pop("CYCLONEDDS_URI", None)
        self._lock = threading.Lock()
        self._inflight = False
        self._process = subprocess.Popen(
            [sys.executable, str(root / "head_service_worker.py")],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            env=env, bufsize=0)
        self._buffer = b""

    @property
    def inflight(self):
        return self._inflight

    def _rpc(self, command, timeout):
        with self._lock:
            process = self._process
            if process.poll() is not None:
                raise RuntimeError("head service transport exited; restart required")
            process.stdin.write(json.dumps(command).encode() + b"\n")
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if b"\n" in self._buffer:
                    line, self._buffer = self._buffer.split(b"\n", 1)
                    # Ignore library startup diagnostics; protocol has a fixed prefix.
                    if not line.startswith(b"U1_HEAD_REPLY "):
                        continue
                    result = json.loads(line[len(b"U1_HEAD_REPLY "):])
                    if not result.get("ok"):
                        raise RuntimeError(result.get("error", "head service transport error"))
                    return result
                ready, _, _ = select.select([process.stdout], [], [], min(.2, max(0, deadline - time.monotonic())))
                if ready:
                    data = os.read(process.stdout.fileno(), 4096)
                    if not data:
                        raise RuntimeError("head service transport disconnected")
                    self._buffer += data
                    if len(self._buffer) > 65536:
                        raise RuntimeError("head service transport response exceeded limit")
            # Never reuse a stream whose response can arrive after this request.
            self.close()
            raise TimeoutError("head service transport timed out; physical outcome unknown")

    def wait_for_service(self, timeout_sec=1):
        result = self._rpc({"operation": "probe"}, 10)
        return bool(result.get("ready"))

    def call_async(self, request):
        future = concurrent.futures.Future()
        command = {"operation": "turn", "degree": request.degree,
                   "max_vel": request.revolute_setup.max_vel,
                   "max_acc": request.revolute_setup.max_acc}
        self._inflight = True
        def call():
            try:
                result = self._rpc(command, 65)
            except Exception as exc:
                self._inflight = False
                future.set_exception(exc)
            else:
                self._inflight = False
                future.set_result(SimpleNamespace(**result["response"]))
        threading.Thread(target=call, daemon=True, name="u1-head-service-result").start()
        return future

    def close(self):
        process = self._process
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        for stream in (process.stdin, process.stdout):
            if stream:
                stream.close()
