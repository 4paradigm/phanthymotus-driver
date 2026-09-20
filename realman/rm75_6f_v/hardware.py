"""Driver-owned SDK connection and exclusive hardware access; no card logic."""

from __future__ import annotations

import math
import os
from pathlib import Path
import threading

from common.vendor_runtime import jsonable


# Keep the names identical to the official RM75 URDF.  Canvas uses the
# joint name to associate each q value with the corresponding URDF joint.
JOINT_NAMES = [f"joint_{i}" for i in range(1, 8)]
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
        self.motion_gate = threading.Lock()
        # Every card, including pick_place, reserves the same physical arm.
        self.motion_lock = self.motion_gate
        self._protected_owner = None
        self._robot = None
        self._handle = None
        self._last_connection_error = None
        self._trajectory_event = threading.Event()
        self._trajectory_lock = threading.Lock()
        self._trajectory_state = "idle"
        # Kept for compatibility with diagnostics written before the explicit
        # idle/waiting/draining state machine was introduced.
        self._trajectory_waiting = False
        self._trajectory_result = None
        self._trajectory_callback = None
        self._trajectory_generation = 0
        # ctypes 回调必须由 Python 对象持有，否则可能被 GC 后导致 C SDK 回调失效。
        self._event_callback = None

    @property
    def connected(self):
        return self._handle is not None and int(getattr(self._handle, "id", -1)) >= 0

    def start(self):
        if not self.enabled:
            print("[rm75] SDK connection disabled; set RM_DRIVER_ENABLED=1 and RM_ARM_IP after safety checks", flush=True)
            return
        self.ensure_connected()

    def ensure_connected(self):
        """Create the one shared SDK handle when a request needs it.

        Driver startup and the arm controller do not always become ready in the
        same order.  The bundle deliberately keeps serving its MCP tools after
        a plugin start failure, so a later request must be able to recover that
        missing initial connection instead of remaining disconnected until the
        container is restarted.
        """
        if not self.enabled:
            raise ConnectionError(
                "RM75 SDK connection is disabled; set RM_DRIVER_ENABLED=1"
            )
        if not self.ip:
            raise ValueError("RM_ARM_IP is required when RM_DRIVER_ENABLED=1")
        if not SDK_LIBRARY_PATH.is_file():
            raise FileNotFoundError(
                "RealMan API2 ARM64 library is missing; mount RM_API2_LIB_DIR "
                "to /work/Robotic_Arm/libs/linux_arm"
            )
        from Robotic_Arm.rm_robot_interface import RoboticArm, rm_event_callback_ptr, rm_thread_mode_e

        with self._lock:
            if self.connected and self._robot is not None:
                return False
            robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            handle = robot.rm_create_robot_arm(self.ip, self.port)
            if handle is None or int(getattr(handle, "id", -1)) < 0:
                bad_id = getattr(handle, "id", None)
                self._handle = None
                self._robot = None
                self._last_connection_error = (
                    f"RealMan SDK could not connect to {self.ip}:{self.port}; handle={bad_id}"
                )
                raise ConnectionError(self._last_connection_error)
            self._robot = robot
            self._handle = handle
            self._event_callback = rm_event_callback_ptr(self._on_arm_event)
            self._robot.rm_get_arm_event_call_back(self._event_callback)
            self._last_connection_error = None
            print(f"[rm75] SDK connected to {self.ip}:{self.port} handle={self._handle.id}", flush=True)
            return True

    def stop(self):
        # Deleting the SDK handle is a definitive event-stream boundary, so no
        # late event from this handle can be confused with a later connection.
        with self._lock:
            if self._protected_owner is not None:
                raise RuntimeError("Cannot disconnect SDK while an exclusive action owns it")
            self.discard_trajectory_wait()
            robot, self._robot = self._robot, None
            self._handle = None
            self._event_callback = None
            if robot is not None:
                robot.rm_delete_robot_arm()

    def status(self):
        return {
            "state": "connected" if self.connected else "disabled" if not self.enabled else "disconnected",
            "endpoint": f"{self.ip}:{self.port}" if self.ip else None,
            "last_connection_error": self._last_connection_error,
            "read_only": not self.motion_enabled,
            "motion_enabled": self.motion_enabled,
        }

    def call(self, method, *args):
        if not self.connected or self._robot is None:
            self.ensure_connected()
        with self._lock:
            if not self.connected or self._robot is None:
                raise ConnectionError("RM75 SDK is not connected")
            return _sdk_result(method, getattr(self._robot, method)(*args))

    def upload_recording(self, path, speed, slot, run=False):
        self._check_owner()
        from Robotic_Arm.rm_robot_interface import rm_send_project_t

        project = rm_send_project_t(
            project_path=str(path), plan_speed=speed, only_save=0 if run else 1,
            save_id=slot, step_flag=0, auto_start=0, project_type=0,
        )
        # rm_send_project returns (status, error_line).  Keep the second
        # value when status is non-zero; the generic call() helper raises
        # first and otherwise hides the controller's useful line number.
        with self._lock:
            if not self.connected or self._robot is None:
                raise ConnectionError("RM75 SDK is not connected")
            self._check_owner()
            result = self._robot.rm_send_project(project)
        if not isinstance(result, tuple) or len(result) < 2:
            raise RuntimeError(f"rm_send_project returned an invalid SDK result: {result!r}")
        code, error_line = int(result[0]), int(result[1])
        if code != 0:
            raise RuntimeError(
                f"rm_send_project failed with RealMan SDK code {code}; "
                f"controller error line={error_line}"
            )
        if error_line != -1:
            raise RuntimeError(f"trajectory project rejected at line {error_line}")

    def call_dict(self, method):
        if not self.connected or self._robot is None:
            self.ensure_connected()
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

    def _check_owner(self, owner=None):
        if self._protected_owner is not None and self._protected_owner is not owner:
            raise RuntimeError("Device is reserved by another card")
        if owner is not None and self._protected_owner is not owner:
            raise RuntimeError("Exclusive SDK command requires device ownership")

    def command(self, method, *args, _owner=None):
        if not self.connected or self._robot is None:
            self.ensure_connected()
        with self._lock:
            self._check_owner(_owner)
            if not self.connected or self._robot is None:
                raise ConnectionError("RM75 SDK is not connected")
            code = int(getattr(self._robot, method)(*args))
            if code != 0:
                raise RuntimeError(f"{method} failed with RealMan SDK code {code}")
            return code

    def _on_arm_event(self, data):
        """接收三线程 API2 的规划轨迹到位事件。"""
        try:
            if int(data.event_type) != 1 or int(data.device) != 0:
                return
            handle_id = int(getattr(data, "handle_id", -1))
            if self.connected and handle_id != int(self._handle.id):
                return
            callback = None
            drained = False
            with self._trajectory_lock:
                if self._trajectory_state == "idle":
                    return
                if self._trajectory_state == "draining":
                    # A timeout/cancellation already ended the owning action.
                    # The SDK event has no command id, so consume this terminal
                    # event before another command is allowed to register.
                    self._trajectory_state = "idle"
                    self._trajectory_waiting = False
                    self._trajectory_result = None
                    self._trajectory_callback = None
                    self._trajectory_event.set()
                    drained = True
                else:
                    trajectory_state = bool(data.trajectory_state)
                    self._trajectory_result = trajectory_state
                    self._trajectory_event.set()
                    # 控制器已经给出该轨迹的终态，下一条轨迹可以登记自己的等待者。
                    # 旧的 Python SDK 包装调用即使仍未返回，也不能继续占用逻辑槽位。
                    self._trajectory_state = "idle"
                    self._trajectory_waiting = False
                    # 事件可能在 rm_movel_offset(block=0) 的 Python 包装调用返回前到达。
                    # 回调只取一次，实际 ACP 收尾由插件放到独立线程执行，不能阻塞 SDK
                    # 的 ctypes 回调线程。
                    callback, self._trajectory_callback = self._trajectory_callback, None
            if drained:
                print("[rm75] late controller trajectory event drained", flush=True)
                return
            if callback is not None:
                callback(trajectory_state)
            outcome = "completed" if trajectory_state else "failed"
            print(f"[rm75] controller trajectory event: {outcome}", flush=True)
        except Exception as exc:
            print(f"[rm75] controller trajectory event ignored: {exc}", flush=True)

    def command_trajectory(self, method, *args, completion_callback=None):
        """非阻塞下发轨迹，并在下发前准备官方到位事件。"""
        # 状态发布或预检可能卡在 SDK 查询并长期持有 _lock。三线程模式的原生
        # block=0 运动不能因此被饿死；只做原子引用快照，生命周期 stop 会先停止
        # 各插件，不会与正常的新运动并发销毁句柄。
        if not self.connected or self._robot is None:
            self.ensure_connected()
        # The calling card holds motion_gate for its action. Keep this path
        # independent of the general SDK lock, as required by controller events.
        self._check_owner()
        robot = self._robot
        if not self.connected or robot is None:
            raise ConnectionError("RM75 SDK is not connected")
        with self._trajectory_lock:
            self._ensure_trajectory_ready_locked()
            self._trajectory_generation += 1
            generation = self._trajectory_generation
            self._trajectory_result = None
            self._trajectory_callback = completion_callback
            self._trajectory_event.clear()
            self._trajectory_state = "waiting"
            self._trajectory_waiting = True
        try:
            # 三线程 SDK 的 block=0 本应立即返回，但部分真机版本会一直停在
            # Python/C 包装调用中，尽管到位事件已经送达。这里不持有通用 SDK 锁，
            # 让事件完成后下一条命令仍可进入控制器。
            code = int(getattr(robot, method)(*args))
            if code != 0:
                raise RuntimeError(f"{method} failed with RealMan SDK code {code}")
            return code
        except Exception:
            self._discard_trajectory_wait(generation)
            raise

    def poll_trajectory(self, timeout_seconds=0.0):
        """读取官方到位事件；尚未收到时保留等待状态并返回 None。"""
        self._trajectory_event.wait(timeout=max(0.0, float(timeout_seconds)))
        with self._trajectory_lock:
            # 回调可能恰好在 Event.wait 超时与取得锁之间到达；以锁内结果为准，
            # 避免把已经收到的成功事件误判为超时。
            result = self._trajectory_result
            if result is not None:
                self._trajectory_waiting = False
                self._trajectory_result = None
                self._trajectory_callback = None
                self._trajectory_event.clear()
        return result

    def wait_trajectory(self, timeout_seconds):
        """等待官方 current trajectory state 回调；None 表示超时。"""
        self._trajectory_event.wait(timeout=max(0.0, float(timeout_seconds)))
        with self._trajectory_lock:
            result = self._trajectory_result
            if result is not None:
                self._trajectory_result = None
                self._trajectory_event.clear()
                return result
            if self._trajectory_state == "waiting":
                # Make timeout and event delivery mutually exclusive.  Once
                # this transition wins, the old untagged event can only drain
                # this generation and cannot reach a later callback.
                self._trajectory_callback = None
                self._trajectory_state = "draining"
                self._trajectory_waiting = False
                self._trajectory_event.clear()
            return None

    def _ensure_trajectory_ready_locked(self):
        if self._trajectory_state == "draining":
            raise RuntimeError(
                "previous controller trajectory completion is still draining"
            )
        if self._trajectory_state == "waiting":
            raise RuntimeError("another controller trajectory wait is active")

    def ensure_trajectory_ready(self):
        """Reject a new action until an old untagged SDK event is drained."""
        with self._trajectory_lock:
            self._ensure_trajectory_ready_locked()
            return True

    def trajectory_wait_state(self):
        with self._trajectory_lock:
            return self._trajectory_state

    def discard_trajectory_wait(self, expected_generation=None):
        with self._trajectory_lock:
            if (expected_generation is not None
                    and expected_generation != self._trajectory_generation):
                return False
            self._trajectory_state = "idle"
            self._trajectory_waiting = False
            self._trajectory_result = None
            self._trajectory_callback = None
            self._trajectory_event.clear()
            return True

    # 兼容类内旧调用名称。
    _discard_trajectory_wait = discard_trajectory_wait

    def cancel_trajectory_wait(self):
        """结束动作回调并保留事件槽，直到旧控制器终态被排空。"""
        with self._trajectory_lock:
            if self._trajectory_state != "waiting":
                return False
            self._trajectory_callback = None
            self._trajectory_result = False
            self._trajectory_state = "draining"
            self._trajectory_waiting = False
            self._trajectory_event.set()
            return True

    def command_interrupt(self, method, *args):
        """运动命令为非阻塞模式，停止命令可安全使用同一串行 SDK 入口。"""
        return self.command(method, *args)

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
