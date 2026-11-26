"""Cancellable connection to the read-only odometry SDK worker."""

import asyncio
import json
import math
import os
import threading
import time
from pathlib import Path


class SdkOdometryClient:
    def __init__(self, config, stale_timeout_s):
        self._python = Path(os.getenv("HELIOS_ODOMETRY_SDK_PYTHON", "/opt/sdk-python/bin/python3.12"))
        self._lib = Path(os.getenv("HELIOS_ODOMETRY_SDK_LIB", "/opt/vmr-sdk/lib"))
        self._host = config.get("odometry_sdk_host", "192.168.0.100")
        self._port = int(config.get("odometry_sdk_port", 9003))
        self._timeout = stale_timeout_s
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._latest = None

    @property
    def installed(self):
        return self._python.is_file() and self._lib.is_dir()

    def start(self):
        if self.installed and (self._thread is None or not self._thread.is_alive()):
            self._stop.clear()
            self._thread = threading.Thread(target=lambda: asyncio.run(self._run()),
                                            name="helios-odom-sdk", daemon=True)
            self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=4)
        with self._lock:
            self._latest = None

    def snapshot(self):
        with self._lock:
            if self._latest is not None:
                pose, received_s = self._latest
                age_s = time.monotonic() - received_s
                if not self._stop.is_set() and age_s <= self._timeout:
                    return {**pose, "age_ms": round(age_s * 1000)}
        return None

    async def _run(self):
        worker_path = Path(__file__).with_name("sdk_odometry_worker.py")
        env = dict(os.environ, PYTHONPATH=str(self._lib), PYTHONUNBUFFERED="1")
        env["LD_LIBRARY_PATH"] = os.pathsep.join(
            part for part in (str(self._lib), env.get("LD_LIBRARY_PATH")) if part)
        while not self._stop.is_set():
            process = None
            try:
                process = await asyncio.create_subprocess_exec(
                    # Without -P the sibling Driver odometry.py shadows the SDK extension.
                    str(self._python), "-P", str(worker_path), self._host, str(self._port),
                    env=env, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL,
                )
                await self._read(process)
            except OSError:
                # Missing/incompatible runtime degrades to the existing speed stream.
                pass
            finally:
                with self._lock:
                    self._latest = None
                if process is not None and process.returncode is None:
                    await self._terminate(process)
            for _ in range(4):
                if self._stop.is_set():
                    break
                await asyncio.sleep(0.25)

    @staticmethod
    async def _terminate(process):
        try:
            process.terminate()
            await asyncio.wait_for(process.communicate(), 1)
        except asyncio.TimeoutError:
            try:
                process.kill()
            except ProcessLookupError:
                pass
            await process.communicate()
        except ProcessLookupError:
            await process.wait()

    async def _read(self, process):
        deadline = time.monotonic() + max(4.0, self._timeout)
        last_stamp = 0
        prefix = b"MOTUS_ODOM "
        while not self._stop.is_set() and time.monotonic() < deadline:
            try:
                line = await asyncio.wait_for(process.stdout.readline(), 0.25)
            except asyncio.TimeoutError:
                continue
            except ValueError:
                return  # Unbounded/malformed worker output.
            if not line:
                return
            if not line.startswith(prefix):
                continue
            try:
                reading = json.loads(line[len(prefix):])
                if not isinstance(reading, dict):
                    continue
                stamp = reading.get("source_stamp_ns")
                if type(stamp) is not int or stamp <= last_stamp:
                    continue
                pose = {key: float(reading[key]) for key in ("x", "y", "yaw")}
                if not all(math.isfinite(value) for value in pose.values()):
                    continue
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            received_s = time.monotonic()
            pose.update(source_stamp_ns=stamp, stamp_ms=time.time_ns() // 1_000_000)
            with self._lock:
                self._latest = (pose, received_s)
            last_stamp = stamp
            deadline = received_s + self._timeout
