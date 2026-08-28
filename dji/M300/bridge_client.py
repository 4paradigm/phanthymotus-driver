"""
bridge_client.py — Python IPC client for the C psdk_bridge process.

In mock mode, returns simulated data for development without DJI hardware.
In live mode, communicates with the psdk_bridge C process via Unix domain socket.

Architecture mirrors Go2's rpc_proxy.py pattern but uses Unix socket instead of
multiprocessing.Queue (since the C bridge is a separate executable, not a Python subprocess).
"""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
import time
import math
import random


# ── IPC wire format ─────────────────────────────────────────────────────────
# Each message: 4-byte big-endian length prefix + JSON payload (UTF-8)

def _send_msg(sock: socket.socket, data: dict):
    payload = json.dumps(data, separators=(",", ":")).encode("utf-8")
    sock.sendall(struct.pack(">I", len(payload)) + payload)


def _recv_msg(sock: socket.socket) -> dict | None:
    hdr = b""
    while len(hdr) < 4:
        chunk = sock.recv(4 - len(hdr))
        if not chunk:
            return None
        hdr += chunk
    length = struct.unpack(">I", hdr)[0]
    buf = b""
    while len(buf) < length:
        chunk = sock.recv(length - len(buf))
        if not chunk:
            return None
        buf += chunk
    return json.loads(buf.decode("utf-8"))


# ── Mock data generators ───────────────────────────────────────────────────

class _MockState:
    """Shared simulated aircraft state for mock mode."""

    def __init__(self):
        self.lat = 39.9042
        self.lon = 116.4074
        self.alt = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.battery_percent = 85
        self.flight_status = "on_ground"  # on_ground / in_air
        self.flight_mode = "normal"
        self.motor_on = False
        self.satellites = 18
        self.obstacle_avoidance = True


_mock = _MockState()


def _mock_telemetry() -> dict:
    t = time.time()
    return {
        "timestamp": t,
        "position": {
            "latitude": _mock.lat + math.sin(t * 0.01) * 0.00001,
            "longitude": _mock.lon + math.cos(t * 0.01) * 0.00001,
            "altitude": _mock.alt,
            "altitude_fused": _mock.alt + 0.1,
            "home_altitude": 0.0,
        },
        "attitude": {
            "quaternion": [1.0, 0.0, 0.0, 0.0],
            "yaw": _mock.yaw,
            "pitch": _mock.pitch,
            "roll": _mock.roll,
        },
        "velocity": {
            "vx": _mock.vx,
            "vy": _mock.vy,
            "vz": _mock.vz,
        },
        "battery": {
            "percent": _mock.battery_percent,
            "voltage": 22.8,
            "current": 5.2,
            "temperature": 35,
        },
        "gps": {
            "satellites": _mock.satellites,
            "fix_type": 5,
        },
        "compass": {
            "heading": _mock.yaw,
        },
        "obstacles": {
            "front": round(random.uniform(5.0, 20.0), 1),
            "back": round(random.uniform(5.0, 20.0), 1),
            "left": round(random.uniform(5.0, 20.0), 1),
            "right": round(random.uniform(5.0, 20.0), 1),
            "up": round(random.uniform(5.0, 20.0), 1),
            "down": max(0.0, _mock.alt),
        },
        "rc": {
            "left_stick_x": 0, "left_stick_y": 0,
            "right_stick_x": 0, "right_stick_y": 0,
        },
        "flight_status": _mock.flight_status,
        "flight_mode": _mock.flight_mode,
    }


# ── BridgeClient ───────────────────────────────────────────────────────────

class BridgeClient:
    """
    Client for the psdk_bridge C process.

    In mock mode (default for development), simulates all PSDK responses.
    In live mode, connects to the Unix domain socket and forwards commands.
    """

    def __init__(self, socket_path: str = "/tmp/psdk_bridge.sock",
                 mock_mode: bool = True):
        self._socket_path = socket_path
        self._mock_mode = mock_mode
        self._sock: socket.socket | None = None
        # Keep a perception call's response validation and possible reconnect
        # atomic with the underlying request.  Telemetry runs concurrently
        # and must not reuse a socket while a late perception reply is being
        # discarded.
        self._lock = threading.RLock()
        self._push_callbacks: dict[str, list] = {}
        self._reader_thread: threading.Thread | None = None
        self._running = False
        self._closed = False
        self._request_id = 0

        if not mock_mode:
            self._connect()

    def _connect(self):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(self._socket_path)
        except Exception:
            sock.close()
            raise
        sock.settimeout(10.0)
        self._sock = sock
        self._running = True
        # Note: no reader thread — _call() does synchronous send/recv
        print(f"[BridgeClient] connected to {self._socket_path}", flush=True)

    def _invalidate_connection(self):
        """Drop a connection whose receive stream may contain a late reply.

        The bridge uses one length-prefixed stream and does not include a
        request id in its reply.  If a command times out, its eventual reply
        would otherwise be consumed as the reply to the next command.  Close
        the stream so the bridge discards that late reply before we reconnect.
        """
        sock = self._sock
        self._sock = None
        self._running = False
        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def _ensure_connected(self) -> bool:
        if self._closed:
            return False
        if self._sock is not None and self._running:
            return True
        try:
            self._connect()
            return True
        except Exception as e:
            print(f"[BridgeClient] reconnect failed: {e}", flush=True)
            return False

    def _handle_push(self, msg: dict):
        push_type = msg.get("push")
        if not push_type:
            return
        for cb in self._push_callbacks.get(push_type, []):
            try:
                cb(msg.get("data"))
            except Exception as e:
                print(f"[BridgeClient] push callback error: {e}", flush=True)

    def _recv_response(self) -> dict | None:
        """Read the next command reply, ignoring asynchronous push frames."""
        while True:
            msg = _recv_msg(self._sock)
            if msg is None:
                return None
            if "push" in msg:
                self._handle_push(msg)
                continue
            return msg

    def _reader_loop(self):
        """Background thread: reads push messages from the C bridge."""
        while self._running:
            try:
                msg = _recv_msg(self._sock)
                if msg is None:
                    print("[BridgeClient] bridge disconnected", flush=True)
                    break
                if "push" in msg:
                    self._handle_push(msg)
            except Exception as e:
                if self._running:
                    print(f"[BridgeClient] reader error: {e}", flush=True)
                break

    def on_push(self, push_type: str, callback):
        """Register a callback for push messages (telemetry, frame, hms, etc.)."""
        self._push_callbacks.setdefault(push_type, []).append(callback)

    def _call(self, cmd: str, args: dict | None = None, timeout: float = 10.0) -> dict:
        """Send a command and wait for response."""
        if self._mock_mode:
            return self._mock_dispatch(cmd, args or {})

        with self._lock:
            if not self._ensure_connected():
                return {"ok": False, "error": "bridge unavailable"}
            self._request_id += 1
            req = {"id": self._request_id, "cmd": cmd, "args": args or {}}
            try:
                self._sock.settimeout(timeout)
                _send_msg(self._sock, req)
                msg = self._recv_response()
                if msg is None:
                    self._invalidate_connection()
                    return {"ok": False, "error": "bridge disconnected"}
                return msg
            except socket.timeout:
                self._invalidate_connection()
                return {"ok": False, "error": "bridge timeout"}
            except Exception as e:
                self._invalidate_connection()
                return {"ok": False, "error": str(e)}

    def stop(self):
        with self._lock:
            self._closed = True
            self._running = False
            self._invalidate_connection()

    # ── Mock dispatch ──────────────────────────────────────────────────────

    def _mock_dispatch(self, cmd: str, args: dict) -> dict:
        handler = getattr(self, f"_mock_{cmd}", None)
        if handler:
            return handler(args)
        return {"ok": True, "data": {"ret": 0, "note": f"mock: {cmd}"}}

    def _mock_get_telemetry(self, args: dict) -> dict:
        return {"ok": True, "data": _mock_telemetry()}

    def _mock_takeoff(self, args: dict) -> dict:
        _mock.flight_status = "in_air"
        _mock.alt = 1.2
        _mock.motor_on = True
        return {"ok": True, "data": {"ret": 0}}

    def _mock_land(self, args: dict) -> dict:
        _mock.flight_status = "on_ground"
        _mock.alt = 0.0
        _mock.motor_on = False
        return {"ok": True, "data": {"ret": 0}}

    def _mock_go_home(self, args: dict) -> dict:
        _mock.flight_mode = "go_home"
        return {"ok": True, "data": {"ret": 0}}

    def _mock_cancel_go_home(self, args: dict) -> dict:
        _mock.flight_mode = "normal"
        return {"ok": True, "data": {"ret": 0}}

    def _mock_joystick_move(self, args: dict) -> dict:
        _mock.vx = args.get("vx", 0)
        _mock.vy = args.get("vy", 0)
        _mock.vz = args.get("vz", 0)
        return {"ok": True, "data": {"ret": 0}}

    def _mock_emergency_brake(self, args: dict) -> dict:
        _mock.vx = _mock.vy = _mock.vz = 0
        return {"ok": True, "data": {"ret": 0}}

    def _mock_set_home_point(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_set_obstacle_avoidance(self, args: dict) -> dict:
        _mock.obstacle_avoidance = args.get("enabled", True)
        return {"ok": True, "data": {"ret": 0}}

    def _mock_obtain_joystick_authority(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_release_joystick_authority(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_get_hms_info(self, args: dict) -> dict:
        return {"ok": True, "data": {"alerts": []}}

    def _mock_hms_inject(self, args: dict) -> dict:
        code = args.get("code", 0x1E020001)
        level = args.get("level", 1)
        return {"ok": True, "data": {"ret": 0, "code": f"0x{code:08X}", "note": f"mock injected 0x{code:08X} level={level}"}}

    def _mock_hms_eliminate(self, args: dict) -> dict:
        code = args.get("code", 0x1E020001)
        return {"ok": True, "data": {"ret": 0, "code": f"0x{code:08X}", "note": f"mock eliminated 0x{code:08X}"}}

    def _mock_start_liveview(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_stop_liveview(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_start_perception(self, args: dict) -> dict:
        source = args.get("source") or args.get("direction", "front_left")
        return {"ok": True, "data": {"ret": 0, "source": source}}

    def _mock_stop_perception(self, args: dict) -> dict:
        source = args.get("source") or args.get("direction", "front_left")
        return {"ok": True, "data": {"ret": 0, "source": source}}

    def _mock_get_aircraft_info(self, args: dict) -> dict:
        return {"ok": True, "data": {
            "product_name": "Matrice 300 RTK",
            "firmware_version": "M300-PSDK",
            "serial_number": "MOCK0000000001",
        }}

    # ── Public API (convenience wrappers) ──────────────────────────────────

    # Flight control
    def takeoff(self):
        return self._call("takeoff")

    def land(self, auto_confirm: bool = False):
        if auto_confirm:
            return self._call("land", {"auto_confirm": True})
        return self._call("land")

    def confirm_landing(self):
        return self._call("confirm_landing")

    def go_home(self):
        return self._call("go_home")

    def cancel_go_home(self):
        return self._call("cancel_go_home")

    def joystick_move(self, vx: float = 0, vy: float = 0, vz: float = 0,
                      vyaw: float = 0, duration: float = -1):
        return self._call("joystick_move", {"vx": vx, "vy": vy, "vz": vz, "vyaw": vyaw, "duration": duration})

    def stop_move(self):
        return self._call("stop_move")

    def emergency_brake(self):
        return self._call("emergency_brake")

    def turn_on_motors(self):
        return self._call("rotate_start")

    def turn_off_motors(self):
        return self._call("rotate_stop")

    def slow_rotate_start(self):
        return self._call("slow_rotate_start")

    def slow_rotate_stop(self):
        return self._call("slow_rotate_stop")

    def set_home_point(self, lat: float, lon: float):
        return self._call("set_home_point", {"lat": lat, "lon": lon})

    def set_obstacle_avoidance(self, enabled: bool, direction: str = "all"):
        return self._call("set_obstacle_avoidance",
                          {"enabled": enabled, "direction": direction})

    def obtain_joystick_authority(self):
        return self._call("obtain_joystick_authority")

    def release_joystick_authority(self):
        return self._call("release_joystick_authority")

    # Telemetry
    def get_telemetry(self):
        return self._call("get_telemetry")

    # HMS
    def get_hms_info(self):
        return self._call("get_hms_info")

    def hms_inject_error(self, error_code: int = 0x1E020001, error_level: int = 1):
        return self._call("hms_inject", {"code": error_code, "level": error_level})

    def hms_eliminate_error(self, error_code: int = 0x1E020001):
        return self._call("hms_eliminate", {"code": error_code})

    # Liveview
    def start_liveview(self, camera: str = "fpv"):
        return self._call("start_liveview", {"camera": camera})

    def stop_liveview(self, camera: str = "fpv"):
        return self._call("stop_liveview", {"camera": camera})

    # Perception
    def start_perception(self, source: str = "front_left", direction: str | None = None):
        """Start one physical perception camera.

        ``direction`` remains accepted as a compatibility alias for older
        callers; direction-only values are interpreted as the left camera by
        the bridge.
        """
        if direction is not None:
            source = direction
        return self._perception_call("start_perception", source)

    def stop_perception(self, source: str = "front_left", direction: str | None = None):
        if direction is not None:
            source = direction
        return self._perception_call("stop_perception", source)

    def _perception_call(self, cmd: str, source: str) -> dict:
        """Call perception and reject a reply belonging to another request.

        Perception replies always echo the requested source.  This gives the
        client a cheap guard against a late reply left by an earlier timed-out
        IPC command; retrying start/stop is safe because both operations are
        idempotent for an already matching source.
        """
        args = {"source": source}
        with self._lock:
            response = self._call(cmd, args)
            data = response.get("data")
            reply_source = data.get("source") if isinstance(data, dict) else None
            if reply_source == source:
                return response
            if response.get("ok") is False and not reply_source:
                return response

            self._invalidate_connection()
            retry = self._call(cmd, args)
            retry_data = retry.get("data")
            retry_source = retry_data.get("source") if isinstance(retry_data, dict) else None
            if retry_source == source:
                return retry
            return {
                "ok": False,
                "error": f"bridge response mismatch for {cmd}: expected {source}, got {reply_source or retry_source or 'unknown'}",
            }

    # Aircraft info
    def get_aircraft_info(self):
        return self._call("get_aircraft_info")

    def get_aircraft_time(self):
        return self._call("get_aircraft_time")

    def sync_clock(self):
        return self._call("sync_clock")
