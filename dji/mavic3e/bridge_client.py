"""
bridge_client.py — Python IPC client for the C psdk_bridge process.

In mock mode, returns simulated data for development without DJI hardware.
In live mode, communicates with the psdk_bridge C process via Unix domain socket.

Architecture mirrors Go2's rpc_proxy.py pattern but uses Unix socket instead of
multiprocessing.Queue (since the C bridge is a separate executable, not a Python subprocess).
"""

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
        self.gimbal_pitch = 0.0
        self.gimbal_yaw = 0.0
        self.gimbal_roll = 0.0
        self.battery_percent = 85
        self.flight_status = "on_ground"  # on_ground / in_air
        self.flight_mode = "normal"
        self.motor_on = False
        self.satellites = 18
        self.recording_video = False
        self.waypoint_state = "idle"  # idle / executing / paused
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
        self._lock = threading.Lock()
        self._push_callbacks: dict[str, list] = {}
        self._reader_thread: threading.Thread | None = None
        self._running = False
        self._request_id = 0

        if not mock_mode:
            self._connect()

    def _connect(self):
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.connect(self._socket_path)
        self._sock.settimeout(10.0)
        self._running = True
        # Note: no reader thread — _call() does synchronous send/recv
        print(f"[BridgeClient] connected to {self._socket_path}", flush=True)

    def _reader_loop(self):
        """Background thread: reads push messages from the C bridge."""
        while self._running:
            try:
                msg = _recv_msg(self._sock)
                if msg is None:
                    print("[BridgeClient] bridge disconnected", flush=True)
                    break
                if "push" in msg:
                    push_type = msg["push"]
                    for cb in self._push_callbacks.get(push_type, []):
                        try:
                            cb(msg.get("data"))
                        except Exception as e:
                            print(f"[BridgeClient] push callback error: {e}", flush=True)
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
            self._request_id += 1
            req = {"id": self._request_id, "cmd": cmd, "args": args or {}}
            try:
                _send_msg(self._sock, req)
                msg = _recv_msg(self._sock)
                if msg is None:
                    return {"ok": False, "error": "bridge disconnected"}
                return msg
            except socket.timeout:
                return {"ok": False, "error": "bridge timeout"}
            except Exception as e:
                return {"ok": False, "error": str(e)}

    def stop(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

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

    def _mock_take_photo(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0, "photo_index": 1}}

    def _mock_start_video(self, args: dict) -> dict:
        _mock.recording_video = True
        return {"ok": True, "data": {"ret": 0}}

    def _mock_stop_video(self, args: dict) -> dict:
        _mock.recording_video = False
        return {"ok": True, "data": {"ret": 0}}

    def _mock_set_camera_mode(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_set_zoom(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0, "zoom_factor": args.get("factor", 1.0)}}

    def _mock_set_focus(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_set_exposure(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_get_storage(self, args: dict) -> dict:
        return {"ok": True, "data": {"total_mb": 128000, "free_mb": 95000}}

    def _mock_gimbal_rotate(self, args: dict) -> dict:
        _mock.gimbal_pitch = max(-90, min(35, args.get("pitch", _mock.gimbal_pitch)))
        _mock.gimbal_yaw = max(-40, min(40, args.get("yaw", _mock.gimbal_yaw)))
        return {"ok": True, "data": {"ret": 0}}

    def _mock_gimbal_reset(self, args: dict) -> dict:
        _mock.gimbal_pitch = _mock.gimbal_yaw = _mock.gimbal_roll = 0
        return {"ok": True, "data": {"ret": 0}}

    def _mock_gimbal_set_mode(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_gimbal_get_angles(self, args: dict) -> dict:
        return {"ok": True, "data": {
            "pitch": _mock.gimbal_pitch,
            "yaw": _mock.gimbal_yaw,
            "roll": _mock.gimbal_roll,
        }}

    def _mock_waypoint_upload(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_waypoint_start(self, args: dict) -> dict:
        _mock.waypoint_state = "executing"
        return {"ok": True, "data": {"ret": 0}}

    def _mock_waypoint_pause(self, args: dict) -> dict:
        _mock.waypoint_state = "paused"
        return {"ok": True, "data": {"ret": 0}}

    def _mock_waypoint_resume(self, args: dict) -> dict:
        _mock.waypoint_state = "executing"
        return {"ok": True, "data": {"ret": 0}}

    def _mock_waypoint_stop(self, args: dict) -> dict:
        _mock.waypoint_state = "idle"
        return {"ok": True, "data": {"ret": 0}}

    def _mock_waypoint_status(self, args: dict) -> dict:
        return {"ok": True, "data": {"state": _mock.waypoint_state, "progress": 0.0}}

    def _mock_speaker_play(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_speaker_set_volume(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_speaker_stop(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_get_power_state(self, args: dict) -> dict:
        return {"ok": True, "data": {
            "battery_percent": _mock.battery_percent,
            "voltage": 22.8,
            "current": 5.2,
            "eport_power": True,
        }}

    def _mock_get_hms_info(self, args: dict) -> dict:
        return {"ok": True, "data": {"alerts": []}}

    def _mock_start_liveview(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_stop_liveview(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_start_perception(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_stop_perception(self, args: dict) -> dict:
        return {"ok": True, "data": {"ret": 0}}

    def _mock_get_aircraft_info(self, args: dict) -> dict:
        return {"ok": True, "data": {
            "product_name": "Mavic 3 Enterprise",
            "firmware_version": "07.01.20.01",
            "serial_number": "MOCK0000000001",
        }}

    # ── Public API (convenience wrappers) ──────────────────────────────────

    # Flight control
    def takeoff(self):
        return self._call("takeoff")

    def land(self):
        return self._call("land")

    def go_home(self):
        return self._call("go_home")

    def cancel_go_home(self):
        return self._call("cancel_go_home")

    def joystick_move(self, vx: float = 0, vy: float = 0, vz: float = 0,
                      vyaw: float = 0):
        return self._call("joystick_move", {"vx": vx, "vy": vy, "vz": vz, "vyaw": vyaw})

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

    # Camera
    def take_photo(self, mode: str = "single"):
        return self._call("take_photo", {"mode": mode})

    def start_video(self):
        return self._call("start_video")

    def stop_video(self):
        return self._call("stop_video")

    def set_camera_mode(self, mode: str):
        return self._call("set_camera_mode", {"mode": mode})

    def set_zoom(self, factor: float):
        return self._call("set_zoom", {"factor": factor})

    def set_focus(self, x: float, y: float):
        return self._call("set_focus", {"x": x, "y": y})

    def set_exposure(self, iso: int = 0, aperture: float = 0,
                     shutter_speed: float = 0, ev: float = 0):
        return self._call("set_exposure", {
            "iso": iso, "aperture": aperture,
            "shutter_speed": shutter_speed, "ev": ev,
        })

    def get_storage(self):
        return self._call("get_storage")

    def ir_temp_point(self, x: float = 0.5, y: float = 0.5):
        return self._call("ir_temp_point", {"x": x, "y": y})

    def ir_temp_area(self, ltx: float = 0.25, lty: float = 0.25,
                     rbx: float = 0.75, rby: float = 0.75):
        return self._call("ir_temp_area", {"ltx": ltx, "lty": lty, "rbx": rbx, "rby": rby})

    # Gimbal
    def gimbal_rotate(self, pitch: float = 0, yaw: float = 0, roll: float = 0,
                      mode: str = "absolute", duration: float = 1.0):
        return self._call("gimbal_rotate", {
            "pitch": pitch, "yaw": yaw, "roll": roll,
            "mode": mode, "duration": duration,
        })

    def gimbal_reset(self):
        return self._call("gimbal_reset")

    def gimbal_set_mode(self, mode: str):
        return self._call("gimbal_set_mode", {"mode": mode})

    def gimbal_get_angles(self):
        return self._call("gimbal_get_angles")

    # Waypoint
    def waypoint_upload(self, kmz_path: str):
        return self._call("waypoint_upload", {"kmz_path": kmz_path})

    def waypoint_start(self):
        return self._call("waypoint_start")

    def waypoint_pause(self):
        return self._call("waypoint_pause")

    def waypoint_resume(self):
        return self._call("waypoint_resume")

    def waypoint_stop(self):
        return self._call("waypoint_stop")

    def waypoint_status(self):
        return self._call("waypoint_status")

    # Speaker
    def speaker_play(self, text: str = "", file_path: str = ""):
        return self._call("speaker_play", {"text": text, "file_path": file_path})

    def speaker_set_volume(self, volume: int):
        return self._call("speaker_set_volume", {"volume": volume})

    def speaker_stop(self):
        return self._call("speaker_stop")

    # Telemetry
    def get_telemetry(self):
        return self._call("get_telemetry")

    # Power
    def get_power_state(self):
        return self._call("get_power_state")

    # HMS
    def get_hms_info(self):
        return self._call("get_hms_info")

    # Liveview
    def start_liveview(self, camera: str = "wide"):
        return self._call("start_liveview", {"camera": camera})

    def stop_liveview(self):
        return self._call("stop_liveview")

    # Perception
    def start_perception(self, direction: str = "front"):
        return self._call("start_perception", {"direction": direction})

    def stop_perception(self, direction: str = "front"):
        return self._call("stop_perception", {"direction": direction})

    # Aircraft info
    def get_aircraft_info(self):
        return self._call("get_aircraft_info")

    def get_aircraft_time(self):
        return self._call("get_aircraft_time")
