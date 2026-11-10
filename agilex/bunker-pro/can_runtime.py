"""SocketCAN lifecycle and guarded Bunker Pro base commands."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from pathlib import Path

import protocol

STATUS_TIMEOUT_S = 0.5
COMMAND_HZ = 50.0


def find_gs_usb_interface(sys_class_net: str = "/sys/class/net") -> str:
    matches = []
    for entry in Path(sys_class_net).glob("*"):
        try:
            driver = (entry / "device" / "driver").resolve().name
        except OSError:
            continue
        if driver == "gs_usb":
            matches.append(entry.name)
    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one SocketCAN interface bound to gs_usb; "
            f"found {matches or 'none'}"
        )
    return matches[0]


class BunkerCAN:
    def __init__(self, config: dict, interface: str, on_odom=None):
        self.config = config
        self.interface = interface
        self._on_odom = on_odom
        self._bus = None
        self._lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._system_state: dict | None = None
        self._system_state_at = 0.0
        self._motion_state: dict | None = None
        self._motion_state_at = 0.0
        self._wheel_odometry: dict | None = None
        self._odom_samples = deque()
        self._target: tuple[float, float] | None = None
        self._target_deadline = 0.0
        self._tx_active = False
        self._frame_count = 0
        self._last_rx_at = 0.0
        self._last_error = ""
        self._last_tx_at = 0.0
        self._last_tx_error = ""
        self._requested_mode = None

    def start(self) -> None:
        import can

        self._bus = can.Bus(interface="socketcan", channel=self.interface,
                            receive_own_messages=False)
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._receive_loop, daemon=True, name="bunker-can-rx"),
            threading.Thread(target=self._transmit_loop, daemon=True, name="bunker-can-tx"),
        ]
        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        self.stop_motion()
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads.clear()
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._bus.recv(timeout=0.1)
                if frame is None or frame.is_extended_id or frame.is_remote_frame:
                    continue
                now = time.monotonic()
                with self._lock:
                    self._frame_count += 1
                    self._last_rx_at = now
                if frame.arbitration_id == protocol.SYSTEM_STATE_ID:
                    value = protocol.decode_system_state(bytes(frame.data))
                    with self._lock:
                        self._system_state = value
                        self._system_state_at = now
                elif frame.arbitration_id == protocol.MOTION_STATE_ID:
                    value = protocol.decode_motion_state(bytes(frame.data))
                    with self._lock:
                        self._motion_state = value
                        self._motion_state_at = now
                        self._odom_samples.append((
                            now,
                            [value["linear_velocity_m_s"], None, None, None, None,
                             value["angular_velocity_rad_s"]],
                        ))
                elif frame.arbitration_id == protocol.ODOMETRY_ID:
                    value = protocol.decode_odometry(bytes(frame.data))
                    with self._lock:
                        self._wheel_odometry = value
            except Exception as exc:  # transport failure must be visible in info()
                if not self._stop.is_set():
                    with self._lock:
                        self._last_error = f"{type(exc).__name__}: {exc}"
                    time.sleep(0.05)

    def _transmit_loop(self) -> None:
        period = 1.0 / COMMAND_HZ
        while not self._stop.wait(period):
            now = time.monotonic()
            with self._lock:
                target = self._target
                active = target is not None and now < self._target_deadline
                system = self._system_state
                status_fresh = now - self._system_state_at <= STATUS_TIMEOUT_S
                was_active = self._tx_active
                if not active:
                    self._target = None
                    self._tx_active = False
                elif status_fresh and system and system["vehicle_state"] == protocol.VEHICLE_NORMAL \
                        and system["control_mode"] == protocol.CONTROL_CAN:
                    self._tx_active = True
                else:
                    self._target = None
                    self._tx_active = False
                    active = False
            if active and target is not None:
                self._send_motion(*target)
            elif was_active:
                # One explicit zero on stop/lease expiry; silence then lets the
                # chassis' own 500 ms communication watchdog remain a backup.
                self._send_motion(0.0, 0.0)

    def _send_frame(self, arbitration_id: int, data: bytes) -> None:
        import can

        if self._bus is None:
            raise RuntimeError("CAN bus is not open")
        frame = can.Message(arbitration_id=arbitration_id, data=data,
                            is_extended_id=False)
        with self._send_lock:
            self._bus.send(frame, timeout=0.05)
        with self._lock:
            self._last_tx_at = time.monotonic()
            self._last_tx_error = ""

    def _send_motion(self, linear: float, angular: float) -> None:
        try:
            data = protocol.encode_motion_command(linear, angular)
            self._send_frame(protocol.MOTION_COMMAND_ID, data)
        except Exception as exc:
            with self._lock:
                self._last_tx_error = f"{type(exc).__name__}: {exc}"
                self._target = None
                self._tx_active = False

    def _fresh_system_state(self) -> dict:
        with self._lock:
            state = dict(self._system_state or {})
            age = time.monotonic() - self._system_state_at
        if not state or age > STATUS_TIMEOUT_S:
            raise RuntimeError("CAN system-state feedback is missing or stale")
        return state

    def request_control_mode(self, mode: int, timeout_s: float = 2.0) -> dict:
        state = self._fresh_system_state()
        if state["vehicle_state"] != protocol.VEHICLE_NORMAL:
            raise RuntimeError("底盘状态异常或急停；拒绝切换控制模式")
        if state["control_mode"] == mode:
            return {"changed": False, "control_mode": mode}
        self.stop_motion()
        self._send_frame(protocol.CONTROL_MODE_CONFIG_ID,
                         protocol.encode_control_mode(mode))
        self._requested_mode = mode
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                state = self._fresh_system_state()
            except RuntimeError:
                break
            if state["vehicle_state"] != protocol.VEHICLE_NORMAL:
                raise RuntimeError("切换模式期间底盘报告异常或急停")
            if state["control_mode"] == mode:
                return {"changed": True, "control_mode": mode}
            time.sleep(0.02)
        raise RuntimeError("未收到目标控制模式反馈；遥控器可能仍拥有控制权")

    def set_target(self, linear: float, angular: float, lease_s: float = 0.35) -> None:
        state = self._fresh_system_state()
        if state["vehicle_state"] != protocol.VEHICLE_NORMAL:
            raise RuntimeError("底盘状态异常或急停；拒绝运动指令")
        if state["control_mode"] != protocol.CONTROL_CAN:
            raise RuntimeError("底盘不在 CAN 控制模式；请显式调用 enable_can")
        linear = self._limit(linear, float(self.config.get("max_linear_m_s", 0.4)), "linear")
        angular = self._limit(angular, float(self.config.get("max_angular_rad_s", 0.8)), "angular")
        with self._lock:
            self._target = (linear, angular)
            self._target_deadline = time.monotonic() + lease_s

    @staticmethod
    def _limit(value, limit, name):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} velocity must be finite")
        if abs(value) > limit:
            raise ValueError(f"{name} velocity exceeds configured limit {limit}")
        return float(value)

    def stop_motion(self) -> dict:
        with self._lock:
            was_active = self._tx_active or self._target is not None
            self._target = None
            self._target_deadline = 0.0
            self._tx_active = False
            state = dict(self._system_state or {})
            fresh = time.monotonic() - self._system_state_at <= STATUS_TIMEOUT_S
        sent = False
        if was_active and fresh and state.get("control_mode") == protocol.CONTROL_CAN:
            self._send_motion(0.0, 0.0)
            sent = True
        return {"state": "stopped", "zero_command_sent": sent}

    def move_for(self, linear: float, angular: float, duration_s: float) -> dict:
        if isinstance(duration_s, bool) or not isinstance(duration_s, (int, float)) \
                or not math.isfinite(duration_s) or not 0.05 <= duration_s <= 10.0:
            raise ValueError("duration_s must be between 0.05 and 10 seconds")
        self.stop_motion()
        self.set_target(linear, angular, lease_s=float(duration_s))
        deadline = time.monotonic() + float(duration_s)
        while time.monotonic() < deadline:
            with self._lock:
                error = self._last_tx_error
                target = self._target
            if error:
                self.stop_motion()
                raise RuntimeError(f"CAN command transmission failed: {error}")
            if target is None:
                raise RuntimeError("CAN motion target was cancelled by the safety watchdog")
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
        return self.stop_motion()

    def odometry_window(self, now: float | None = None) -> tuple[list, dict | None]:
        now = time.monotonic() if now is None else now
        with self._lock:
            rows = [values for stamp, values in self._odom_samples if stamp <= now]
            self._odom_samples.clear()
            motion = dict(self._motion_state) if self._motion_state else None
            motion_fresh = now - self._motion_state_at <= STATUS_TIMEOUT_S
        return rows, motion if motion_fresh else None

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            system = dict(self._system_state) if self._system_state else None
            system_age = now - self._system_state_at if system else None
            motion = dict(self._motion_state) if self._motion_state else None
            motion_age = now - self._motion_state_at if motion else None
            return {
                "can_interface": self.interface,
                "connected": self._bus is not None,
                "rx_frames": self._frame_count,
                "last_rx_age_ms": None if self._last_rx_at == 0 else int((now - self._last_rx_at) * 1000),
                "system_state": system,
                "system_state_age_ms": None if system_age is None else int(system_age * 1000),
                "motion_state": motion,
                "motion_state_age_ms": None if motion_age is None else int(motion_age * 1000),
                "wheel_odometry_raw": dict(self._wheel_odometry) if self._wheel_odometry else None,
                "last_error": self._last_error,
                "last_tx_error": self._last_tx_error,
                "requested_mode": self._requested_mode,
            }
