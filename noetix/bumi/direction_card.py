"""Independent Bumi sound-direction card using the vendor multichannel capture."""

from __future__ import annotations

import json
import math
import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from audio_msgs.msg import SoundDirection

from sound_direction import SoundActivityGate, estimate_angle, estimate_signature


CALIBRATION_PATH = Path("/opt/phanthy-motus/data/bumi/sound_direction_calibration.json")
DEFAULT_CALIBRATION_PATH = Path(__file__).resolve().parent / "resource/sound_direction_calibration.json"
SETTINGS_PATH = Path("/opt/phanthy-motus/data/bumi/sound_direction_settings.json")
DEFAULT_PARAMETERS = {
    "onset_level": 10.0, "onset_ratio": 1.8,
    "burst_level": 15.0, "burst_ratio": 3.0,
    "update_interval_ms": 100,
}
_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)
_READY_LINE = "__BUMI_DIRECTION_READY__"


def load_calibration() -> dict:
    """Use one complete, valid calibration; never mix signatures across files."""
    for path in (CALIBRATION_PATH, DEFAULT_CALIBRATION_PATH):
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        if (isinstance(data, dict) and all(
                isinstance(data.get(name), list) and len(data[name]) == 3
                and all(type(value) in (int, float) and math.isfinite(value)
                        and abs(value) <= 24 for value in data[name])
                for name in ("front", "right"))):
            return {name: data[name] for name in ("front", "right")}
    return {}


def _valid_parameter(name: str, value) -> bool:
    if name == "update_interval_ms":
        return type(value) is int and 50 <= value <= 1000
    if type(value) not in (int, float) or not math.isfinite(value):
        return False
    if name in ("onset_level", "burst_level"):
        return 1 <= value <= 1000
    if name in ("onset_ratio", "burst_ratio"):
        return 1.05 <= value <= 10
    return False


def load_parameters() -> dict:
    try:
        saved = json.loads(SETTINGS_PATH.read_text())
    except Exception:
        return DEFAULT_PARAMETERS.copy()
    if (not isinstance(saved, dict) or saved.keys() != DEFAULT_PARAMETERS.keys()
            or any(not _valid_parameter(key, value) for key, value in saved.items())):
        return DEFAULT_PARAMETERS.copy()
    return saved


def activity_payload(audio, calibration, audio_window, published_us) -> SoundDirection | None:
    if not {"front", "right"} <= calibration.keys():
        return None
    signature = estimate_signature(audio, 8, 16000)
    if signature is None:
        return None
    angle = estimate_angle(signature, calibration["front"], calibration["right"])
    if angle is None:
        return None
    msg = SoundDirection()
    msg.state = SoundDirection.VALID
    msg.angle_deg = float(angle if angle <= 180 else angle - 360)
    begin, last, last_samples = audio_window
    if begin >= 1_000_000_000_000_000 and last >= begin:
        msg.window_start_us = begin
        msg.window_end_us = last + last_samples * 1_000_000 // 16000
    else:
        msg.window_start_us = 0
        msg.window_end_us = 0
    msg.published_us = published_us
    return msg


def _direction_subprocess(namespace: str) -> None:
    from common import logsafe
    logsafe.install(check_fd=False)

    import os
    import sys
    os.environ.setdefault("CYCLONEDDS_URI", "file:///work/noetix_sdk_bumi/config/dds.xml")
    sys.path.insert(0, "/work/noetix_sdk_bumi/build")
    from mediacontrol_py import MediaController

    media_ctrl = MediaController.instance()
    media_ctrl.init()
    time.sleep(3)
    rclpy.init()
    node = Node("bumi_sound_direction")
    pub = node.create_publisher(SoundDirection, f"/{namespace}/sound_direction", _QOS)
    recent_audio = deque(maxlen=8)
    recent_capture_us = deque(maxlen=8)
    parameters = load_parameters()
    gate = SoundActivityGate(**{key: parameters[key] for key in (
        "onset_level", "onset_ratio", "burst_level", "burst_ratio")})
    # SDK 和 ROS 发布器都就绪后，才允许控制卡报告配置成功。
    print(_READY_LINE, flush=True)
    next_check = 0.0
    last_result_at = None
    next_error_log = 0.0

    while True:
        try:
            if last_result_at is not None and time.monotonic() - last_result_at >= 1.0:
                event = SoundDirection()
                event.state = SoundDirection.NO_EVENT
                event.published_us = time.time_ns() // 1000
                pub.publish(event)
                last_result_at = None
            frame = media_ctrl.get_audio_capture_data()
            if frame.channels != 8 or frame.sample_rate != 16000 or not frame.audio_data:
                time.sleep(0.005)
                continue
            samples = np.asarray(frame.audio_data, dtype=np.int16)
            if samples.size % 8:
                continue
            capture_us = getattr(frame, "timestamp_us", 0)
            if type(capture_us) is not int or capture_us < 1_000_000_000_000_000:
                capture_us = 0
            if not gate.accepts(samples):
                # 停顿帧不参与定位，避免机器人的风扇声改变说话者角度。
                recent_audio.clear()
                recent_capture_us.clear()
                continue
            recent_audio.append(samples)
            recent_capture_us.append((capture_us, samples.size // 8))
            if sum(part.size for part in recent_audio) < 8 * 1024:
                continue
            if time.monotonic() < next_check:
                continue
            next_check = time.monotonic() + parameters["update_interval_ms"] / 1000
            valid_clock = all(capture for capture, _ in recent_capture_us)
            result = activity_payload(
                np.concatenate(tuple(recent_audio)), load_calibration(),
                (recent_capture_us[0][0] if valid_clock else 0,
                 recent_capture_us[-1][0] if valid_clock else 0,
                 recent_capture_us[-1][1]), time.time_ns() // 1000)
            if result is not None:
                pub.publish(result)
                last_result_at = time.monotonic()
        except Exception as exc:
            if time.monotonic() >= next_error_log:
                print(f"[direction] capture error: {exc}", flush=True)
                next_error_log = time.monotonic() + 0.5
            time.sleep(0.005)


class SoundDirectionPlugin:
    PREFIX = "sound_direction"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl=None):
        self._namespace = namespace
        self._topic = f"/{namespace}/sound_direction"
        self._media_ctrl = media_ctrl
        self._proc: subprocess.Popen | None = None
        self._process_lock = threading.RLock()
        self._last_direction = None
        self._last_direction_time = 0.0
        self._node = Node("bumi_sound_direction_sub")
        executor.add_node(self._node)
        self._node.create_subscription(SoundDirection, self._topic, self._on_direction, _QOS)
        self._clear_pub = self._node.create_publisher(SoundDirection, self._topic, _QOS)

    def _on_direction(self, msg: SoundDirection) -> None:
        if msg.state == SoundDirection.NO_EVENT:
            with self._process_lock:
                self._last_direction = None
            return
        if msg.state != SoundDirection.VALID:
            return
        if not math.isfinite(msg.angle_deg) or not -180 <= msg.angle_deg <= 180:
            return
        with self._process_lock:
            if self._proc is None or self._proc.poll() is not None:
                return
            self._last_direction = {"state": "fresh", "angle_deg": msg.angle_deg,
                                    "window_start_us": msg.window_start_us,
                                    "window_end_us": msg.window_end_us}
            self._last_direction_time = time.monotonic()

    def get_tool(self) -> dict:
        return {
            "name": "sound_direction", "type": "sensor", "multiInstance": False,
            "description": "Bumi calibrated sound direction from the microphone array",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["info", "start", "stop"]}}},
            "configSchema": {"type": "object", "properties": {
                "onset_level": {"type": "number", "minimum": 1, "maximum": 1000,
                                "default": 10, "scope": "shared"},
                "onset_ratio": {"type": "number", "minimum": 1.05, "maximum": 10,
                                "default": 1.8, "scope": "shared"},
                "burst_level": {"type": "number", "minimum": 1, "maximum": 1000,
                                "default": 15, "scope": "shared"},
                "burst_ratio": {"type": "number", "minimum": 1.05, "maximum": 10,
                                "default": 3.0, "scope": "shared"},
                "update_interval_ms": {"type": "integer", "minimum": 50,
                                       "maximum": 1000, "default": 100,
                                       "scope": "shared"}}},
            "topic_out": [{"topic": self._topic, "format": "sensor/sound-direction-v1"}],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self) -> bool:
        import sys
        with self._process_lock:
            if self._proc is not None and self._proc.poll() is None:
                return True
            self._last_direction = None
            self._last_direction_time = 0.0
            proc = subprocess.Popen(
                [sys.executable, "-c",
                 "import sys; sys.path.insert(0, '/work'); "
                 "from common import logsafe; logsafe.install(check_fd=False); "
                 f"from direction_card import _direction_subprocess; "
                 f"_direction_subprocess({self._namespace!r})"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            )
            self._proc = proc
            ready_event = threading.Event()
            ready = [False]
            def forward():
                for line in proc.stdout:
                    message = line.decode(errors="replace").rstrip()
                    if message == _READY_LINE:
                        ready[0] = True
                        ready_event.set()
                    else:
                        print(message, flush=True)
                ready_event.set()
            threading.Thread(target=forward, daemon=True).start()
            if (not ready_event.wait(timeout=8) or not ready[0]
                    or proc.poll() is not None):
                self.stop()
                return False
            return True

    def stop(self) -> None:
        with self._process_lock:
            if self._proc is not None:
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self._proc.kill()
                    self._proc.wait(timeout=2)
                self._proc = None
            self._last_direction = None
            self._last_direction_time = 0.0
            event = SoundDirection()
            event.state = SoundDirection.NO_EVENT
            event.published_us = time.time_ns() // 1000
            self._clear_pub.publish(event)

    def _set_parameters(self, changes: dict | None) -> dict:
        with self._process_lock:
            previous_parameters = load_parameters()
            if changes is not None:
                changes = {key: value for key, value in changes.items()
                           if key not in ("_tool_name", "action")}
                if any(key not in DEFAULT_PARAMETERS for key in changes):
                    return {"state": "invalid_parameters"}
                changes = {key: value for key, value in changes.items()
                           if value is not None and value != ""}
                if not changes or any(not _valid_parameter(key, value)
                                      for key, value in changes.items()):
                    return {"state": "invalid_parameters"}
                parameters = {**previous_parameters, **changes}
            else:
                parameters = DEFAULT_PARAMETERS.copy()
            running = self._proc is not None and self._proc.poll() is None
            if running:
                try:
                    self.stop()
                except subprocess.TimeoutExpired:
                    return {"state": "error", "message": "direction subprocess did not exit after kill"}
            write_error = None
            try:
                SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
                temporary = SETTINGS_PATH.with_suffix(".tmp")
                temporary.write_text(json.dumps(parameters))
                temporary.replace(SETTINGS_PATH)
            except OSError as exc:
                write_error = exc
            if running:
                try:
                    if not self.start():
                        return {"state": "error", "message": "direction restart failed to initialize",
                                "parameters": load_parameters(),
                                "file_parameters": load_parameters(), "running_parameters": None}
                except Exception as exc:
                    return {"state": "error", "message": f"direction restart failed: {exc}",
                            "parameters": load_parameters(),
                            "file_parameters": load_parameters(), "running_parameters": None}
            if write_error is not None:
                return {"state": "error", "message": str(write_error),
                        "parameters": load_parameters(),
                        "file_parameters": load_parameters(),
                        "running_parameters": previous_parameters if running else None}
            return {"state": "configured", "parameters": parameters}

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action not in ("start", "stop", "info", "config"):
            return None
        if action == "start":
            try:
                if not self.start():
                    return {"state": "error", "message": "direction worker failed to initialize"}
            except (OSError, subprocess.TimeoutExpired) as exc:
                return {"state": "error", "message": f"direction worker failed to start: {exc}"}
            return {"state": "running", "topic_out": self.get_tool()["topic_out"]}
        if action == "stop":
            try:
                self.stop()
            except subprocess.TimeoutExpired:
                return {"state": "error", "message": "direction subprocess did not exit after kill"}
            return {"state": "idle"}
        if action == "info":
            with self._process_lock:
                running = self._proc is not None and self._proc.poll() is None
                if not running:
                    self._last_direction = None
                    self._last_direction_time = 0.0
                direction = self._last_direction
                age = time.monotonic() - self._last_direction_time
            observation = ({"state": "no_event"} if direction is None else
                           {"state": "stale"} if age > 10 else
                           {**direction, "age_ms": round(age * 1000)})
            result = {"state": "running" if running else "idle",
                      "sound_direction": observation,
                      "parameters": load_parameters(),
                      "calibrated_directions": [key for key in ("front", "right")
                                                if key in load_calibration()]}
            result["topic_out"] = self.get_tool()["topic_out"]
            return result
        if action == "config":
            return self._set_parameters(args)
        return None
