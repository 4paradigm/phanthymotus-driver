"""Independent Bumi sound-direction card using the vendor multichannel capture."""

from __future__ import annotations

import json
import math
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

from sound_direction import SoundActivityGate, estimate_angle, estimate_signature, is_voiced_audio


CALIBRATION_PATH = Path("/opt/phanthy-motus/data/bumi/sound_direction_calibration.json")
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
    """Reuse only valid front/right signatures from the existing mic calibration."""
    try:
        data = json.loads(CALIBRATION_PATH.read_text())
    except Exception:
        return {}
    if not isinstance(data, dict):
        return {}
    return {name: signature for name, signature in data.items()
            if name in ("front", "right")
            and isinstance(signature, list) and len(signature) == 3
            and all(type(value) in (int, float) and abs(value) <= 24
                    for value in signature)}


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


def activity_payload(audio, calibration, timestamp_ms, audio_window=None) -> str | None:
    if not {"front", "right"} <= calibration.keys():
        return None
    signature = estimate_signature(audio, 8, 16000)
    if signature is None:
        return None
    angle = estimate_angle(signature, calibration["front"], calibration["right"])
    if angle is None:
        return None
    result = {"state": "fresh", "trigger": "sound_activity",
              "timestamp_ms": int(timestamp_ms), "angle": angle,
              "unit": "deg", "reference": "robot_front_clockwise"}
    if audio_window is not None:
        result.update({"audio_window_start_us": audio_window[0],
                       "audio_window_end_us": audio_window[1]})
    return json.dumps(result, ensure_ascii=False)


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
    pub = node.create_publisher(String, f"/{namespace}/sound_direction", _QOS)
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
                pub.publish(String(data=json.dumps({"state": "no_event"})))
                last_result_at = None
            frame = media_ctrl.get_audio_capture_data()
            if frame.channels != 8 or frame.sample_rate != 16000 or not frame.audio_data:
                time.sleep(0.005)
                continue
            samples = np.asarray(frame.audio_data, dtype=np.int16)
            if samples.size % 8:
                continue
            capture_us = getattr(frame, "timestamp_us", 0)
            if type(capture_us) is not int or capture_us <= 0:
                capture_us = int(time.time() * 1_000_000)
            if not gate.accepts(samples):
                # 停顿帧不参与定位，避免机器人的风扇声改变说话者角度。
                recent_audio.clear()
                recent_capture_us.clear()
                continue
            recent_audio.append(samples)
            recent_capture_us.append(capture_us)
            if sum(part.size for part in recent_audio) < 8 * 1024:
                continue
            if time.monotonic() < next_check:
                continue
            next_check = time.monotonic() + parameters["update_interval_ms"] / 1000
            result = activity_payload(
                np.concatenate(tuple(recent_audio)), load_calibration(),
                time.time() * 1000, (recent_capture_us[0], recent_capture_us[-1]))
            if result is not None:
                pub.publish(String(data=result))
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
        self._node.create_subscription(String, self._topic, self._on_direction, _QOS)

    def _on_direction(self, msg: String) -> None:
        try:
            result = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(result, dict) or result.get("state") != "fresh":
            return
        if type(result.get("angle")) is not int or not 0 <= result["angle"] < 360:
            return
        if any(type(value) not in (str, int, float, bool, type(None))
               for value in result.values()):
            return
        with self._process_lock:
            if self._proc is None or self._proc.poll() is not None:
                return
            self._last_direction = result
            self._last_direction_time = time.monotonic()

    def get_tool(self) -> dict:
        return {
            "name": "sound_direction", "type": "sensor", "multiInstance": False,
            "description": "Bumi calibrated sound direction from the microphone array",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def get_tools(self) -> list:
        return [self.get_tool(), self.get_control_tool()]

    def get_control_tool(self) -> dict:
        return {
            "name": "sound_direction_control", "type": "actuator", "multiInstance": False,
            "description": "Calibrate and configure Bumi sound direction",
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": [
                    "info", "check_direction",
                    "calibrate_front", "calibrate_right",
                    "set_parameters", "reset_parameters"]},
                    "onset_level": {"type": "number", "minimum": 1, "maximum": 1000,
                                    "description": "连续发声最低音量；默认 10"},
                    "onset_ratio": {"type": "number", "minimum": 1.05, "maximum": 10,
                                    "description": "连续发声相对背景倍数；默认 1.8"},
                    "burst_level": {"type": "number", "minimum": 1, "maximum": 1000,
                                    "description": "单次强声最低音量；默认 15"},
                    "burst_ratio": {"type": "number", "minimum": 1.05, "maximum": 10,
                                    "description": "单次强声相对背景倍数；默认 3.0"},
                    "update_interval_ms": {"type": "integer", "minimum": 50, "maximum": 1000,
                                           "description": "方向更新最短间隔，毫秒；默认 100"}},
                "required": ["action"],
                "x-action-params": {
                    "info": {"params": [], "description": "查看状态、最近方向及标定进度。"},
                    "check_direction": {"params": [], "description": "查看最近一次声音方向。"},
                    "calibrate_front": {"params": [], "description": "正前方持续说话约 2 秒。"},
                    "calibrate_right": {"params": [], "description": "正右方持续说话约 2 秒。"},
                    "set_parameters": {"params": [
                        "onset_level", "onset_ratio", "burst_level", "burst_ratio",
                        "update_interval_ms"], "description": "只填写需要调整的参数；保存后重启方向采集。"},
                    "reset_parameters": {"params": [], "description": "恢复当前版本的默认参数。"},
                },
            },
        }

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

    def _calibrate(self, direction: str) -> dict:
        # 标定只暂停本卡采集进程，绝不停止原有 mic 音频卡。
        with self._process_lock:
            running = self._proc is not None and self._proc.poll() is None
            if running:
                self.stop()
            calibration_saved = False
            try:
                frames = []
                seen = set()
                samples = 0
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    frame = self._media_ctrl.get_audio_capture_data()
                    if frame.channels == 8 and frame.sample_rate == 16000 and frame.audio_data:
                        data = np.asarray(frame.audio_data, dtype=np.int16)
                        if data.size % 8 == 0 and data.tobytes() not in seen:
                            seen.add(data.tobytes())
                            frames.append(data)
                            samples += data.size // 8
                    time.sleep(0.005)
                if samples < 16000:
                    return {"state": "no_voice", "message": "未采到足够的新音频，请靠近机器人重试"}
                captured = np.concatenate(frames)
                if not is_voiced_audio(captured, 8, 16000):
                    return {"state": "no_voice", "message": "未采到清晰人声，请靠近机器人重试"}
                signature = estimate_signature(captured, 8, 16000)
                if signature is None:
                    return {"state": "no_voice", "message": "未采到可用于标定的声音，请靠近机器人重试"}
                calibration = load_calibration()
                calibration[direction] = signature
                try:
                    previous = json.loads(CALIBRATION_PATH.read_text())
                except Exception:
                    previous = {}
                if not isinstance(previous, dict):
                    previous = {}
                extras = {key: value for key, value in previous.items()
                          if key not in ("front", "right")}
                CALIBRATION_PATH.parent.mkdir(parents=True, exist_ok=True)
                temporary = CALIBRATION_PATH.with_suffix(".tmp")
                temporary.write_text(json.dumps({**extras, **calibration}))
                temporary.replace(CALIBRATION_PATH)
                calibration_saved = True
                return {"state": "calibrated", "direction": direction,
                        "remaining": [key for key in ("front", "right") if key not in calibration]}
            finally:
                if running:
                    try:
                        if not self.start():
                            return {"state": "error", "message": "direction restart failed to initialize",
                                    "direction": direction, "calibration_saved": calibration_saved}
                    except Exception as exc:
                        return {"state": "error", "message": f"direction restart failed: {exc}",
                                "direction": direction, "calibration_saved": calibration_saved}

    def _set_parameters(self, changes: dict | None) -> dict:
        with self._process_lock:
            if changes is not None:
                changes = {key: value for key, value in changes.items()
                           if key != "_tool_name"}
                if any(key not in DEFAULT_PARAMETERS for key in changes):
                    return {"state": "invalid_parameters"}
                changes = {key: value for key, value in changes.items()
                           if value is not None and value != ""}
                if not changes or any(not _valid_parameter(key, value)
                                      for key, value in changes.items()):
                    return {"state": "invalid_parameters"}
                parameters = {**load_parameters(), **changes}
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
                                "parameters": load_parameters()}
                except Exception as exc:
                    return {"state": "error", "message": f"direction restart failed: {exc}",
                            "parameters": load_parameters()}
            if write_error is not None:
                return {"state": "error", "message": str(write_error),
                        "parameters": load_parameters()}
            return {"state": "configured", "parameters": parameters}

    def dispatch(self, action: str, args: dict) -> dict | None:
        tool_name = args.get("_tool_name")
        if tool_name == "sound_direction_control" and action == "start":
            return {"state": "ready"}
        if tool_name == "sound_direction_control" and action == "stop":
            return {"state": "idle"}
        if tool_name == "sound_direction" and action not in ("start", "stop", "info"):
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
        if action in ("info", "check_direction"):
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
            if tool_name != "sound_direction_control":
                result["topic_out"] = self.get_tool()["topic_out"]
            return result
        if action == "set_parameters":
            return self._set_parameters(args)
        if action == "reset_parameters":
            return self._set_parameters(None)
        if action in ("calibrate_front", "calibrate_right"):
            return self._calibrate(action.removeprefix("calibrate_"))
        return None
