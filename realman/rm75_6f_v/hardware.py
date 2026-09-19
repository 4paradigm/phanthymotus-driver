"""Driver-owned SDK connection and exclusive hardware access; no card logic."""

from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
import tempfile
import threading

from common.vendor_runtime import jsonable


JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
SDK_LIBRARY_PATH = Path("/work/Robotic_Arm/libs/linux_arm/libapi_c.so")
JOINT_LIMITS_DEG = [(-178.0, 178.0), (-130.0, 130.0), (-178.0, 178.0),
                    (-135.0, 135.0), (-178.0, 178.0), (-128.0, 128.0),
                    (-360.0, 360.0)]
JOINT_MAX_SPEED_DEG_S = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]


def _sdk_result(name, result):
    if not isinstance(result, tuple) or not result:
        raise RuntimeError(f"{name} returned an invalid SDK result: {result!r}")
    code = int(result[0])
    if code != 0:
        raise RuntimeError(f"{name} failed with RealMan SDK code {code}")
    if len(result) == 2:
        return jsonable(result[1])
    return jsonable(result[1:])


class RM75SDKClient:
    """Own one SDK handle and serialize all access to the vendor library."""

    def __init__(self, config):
        self.ip = os.environ.get("RM_ARM_IP", str(config.get("arm_ip", "")).strip())
        self.port = int(os.environ.get("RM_TCP_PORT", config.get("tcp_port", 8080)))
        self.enabled = os.environ.get("RM_DRIVER_ENABLED", "0") == "1"
        self.motion_enabled = os.environ.get("RM_MOTION_ENABLED", "0") == "1"
        self._lock = threading.RLock()
        self.motion_lock = threading.Lock()
        self._robot = None
        self._handle = None
        self._protected_owner = None

    @property
    def connected(self):
        return self._handle is not None and int(getattr(self._handle, "id", -1)) >= 0

    def start(self):
        if not self.enabled:
            print("[rm75] SDK connection disabled; set RM_DRIVER_ENABLED=1 and RM_ARM_IP after safety checks", flush=True)
            return
        if not self.ip:
            raise ValueError("RM_ARM_IP is required when RM_DRIVER_ENABLED=1")
        if not SDK_LIBRARY_PATH.is_file():
            raise FileNotFoundError(
                "RealMan API2 ARM64 library is missing; mount RM_API2_LIB_DIR "
                "to /work/Robotic_Arm/libs/linux_arm"
            )
        from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

        with self._lock:
            if self.connected:
                return
            self._robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            self._handle = self._robot.rm_create_robot_arm(self.ip, self.port)
            if not self.connected:
                bad_id = getattr(self._handle, "id", None)
                self._handle = None
                self._robot = None
                raise ConnectionError(f"RealMan SDK could not connect to {self.ip}:{self.port}; handle={bad_id}")
            print(f"[rm75] SDK connected to {self.ip}:{self.port} handle={self._handle.id}", flush=True)

    def stop(self):
        with self._lock:
            if self._protected_owner is not None:
                raise RuntimeError("Cannot disconnect SDK while an exclusive action owns it")
            robot, self._robot = self._robot, None
            self._handle = None
            if robot is not None:
                robot.rm_delete_robot_arm()

    def status(self):
        return {
            "state": "connected" if self.connected else "disabled" if not self.enabled else "disconnected",
            "endpoint": f"{self.ip}:{self.port}" if self.ip else None,
            "read_only": not self.motion_enabled,
            "motion_enabled": self.motion_enabled,
        }

    def call(self, method, *args):
        with self._lock:
            if not self.connected or self._robot is None:
                raise ConnectionError("RM75 SDK is not connected")
            return _sdk_result(method, getattr(self._robot, method)(*args))

    def call_dict(self, method):
        with self._lock:
            if not self.connected or self._robot is None:
                raise ConnectionError("RM75 SDK is not connected")
            result = getattr(self._robot, method)()
            if not isinstance(result, dict) or "return_code" not in result:
                raise RuntimeError(f"{method} returned an invalid SDK result: {result!r}")
            code = int(result["return_code"])
            if code != 0:
                raise RuntimeError(f"{method} failed with RealMan SDK code {code}")
            return jsonable(result)

    def joint_states(self):
        degrees = self.call("rm_get_joint_degree")
        if not isinstance(degrees, list) or len(degrees) != 7:
            raise RuntimeError(f"rm_get_joint_degree returned {len(degrees) if isinstance(degrees, list) else 'invalid'} joints")
        radians = [math.radians(float(value)) for value in degrees]
        return {"name": JOINT_NAMES, "position": radians, "position_unit": "rad", "raw_degree": degrees}

    def command(self, method, *args, _owner=None):
        with self._lock:
            if self._protected_owner is not None and self._protected_owner is not _owner:
                raise RuntimeError("Device is reserved by another card")
            if _owner is not None and self._protected_owner is not _owner:
                raise RuntimeError("Exclusive SDK command requires device ownership")
            if not self.connected or self._robot is None:
                raise ConnectionError("RM75 SDK is not connected")
            code = int(getattr(self._robot, method)(*args))
            if code != 0:
                raise RuntimeError(f"{method} failed with RealMan SDK code {code}")
            return code

    def get_tools(self):
        # The bundle owns connection lifetime, independently of all cards.
        return []

    def exclusive_client(self):
        return ExclusiveSDKClient(self)

    def shared_client(self):
        return SharedSDKClient(self)


class SharedSDKClient:
    """Borrow a Driver-owned connection without owning its lifecycle.

    Existing cards can keep their start/stop contract; the bundle starts the
    connection before cards and closes it after all cards have stopped.
    """

    def __init__(self, client):
        self._client = client

    def __getattr__(self, name):
        return getattr(self._client, name)

    def start(self):
        pass

    def stop(self):
        pass


class ExclusiveSDKClient:
    """A capability whose writes are accepted only while it owns the device."""

    def __init__(self, client):
        self._client = client
        self.motion_lock = self
        self._token = object()

    @property
    def connected(self):
        return self._client.connected

    @property
    def motion_enabled(self):
        return self._client.motion_enabled

    def acquire(self, blocking=False):
        if not self._client.motion_lock.acquire(blocking=blocking):
            return False
        with self._client._lock:
            self._client._protected_owner = self._token
        return True

    def release(self):
        with self._client._lock:
            if self._client._protected_owner is not self._token:
                raise RuntimeError("Cannot release another owner's device")
            self._client._protected_owner = None
            self._client.motion_lock.release()

    def call(self, method, *args):
        return self._client.call(method, *args)

    def call_dict(self, method):
        return self._client.call_dict(method)

    def command(self, method, *args):
        return self._client.command(method, *args, _owner=self._token)

    def status(self):
        return self._client.status()


class CameraLease:
    """Exclude participating camera pipelines within this runtime.

    Unrelated camera implementations need not use this lease; native SDK open
    errors and frame freshness checks must still be enforced by the caller.
    """

    def __init__(self, serial):
        self.serial = serial
        self._file = None

    def __enter__(self):
        import fcntl

        name = hashlib.sha256(self.serial.encode()).hexdigest()
        handle = open(Path(tempfile.gettempdir()) / f"realman-camera-{name}.lock", "a")
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            handle.close()
            raise RuntimeError("RealSense camera is in use by another action or card") from exc
        self._file = handle
        return self

    def __exit__(self, *args):
        if self._file is not None:
            self._file.close()
            self._file = None
