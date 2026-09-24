"""UBTECH U1 Pro ROS 2 adapter.

The cards in this module are Agent capabilities, not a mirror of every SDK
management call. The robot's public ROS graph provides useful contracts:
16 kHz microphone PCM, live speaker PCM input, motion playback, and audio events.
"""

from __future__ import annotations

import json
import mmap
import os
import re
import shutil
import ssl
import struct
import subprocess
import threading
import time
import urllib.request
import uuid
from pathlib import Path
from typing import Any

from common.vendor_runtime import action_schema, jsonable, tool


SERVICE_TIMEOUT = 3.0
MIC_TOPIC = "/sys/device/audio_in/raw"
SPEAKER_TOPIC = "/sys/device/audio_out/raw"
AUDIO_FORMAT = "audio/pcm-16k"
MIC_SAMPLE_FORMATS = {"s16", "s16le", "s16_le", "signed_16", "pcm_s16le", "int16"}
PLAYBACK_TOPIC = "/robo/media/subscribe/playback_state"
VIDEO_METADATA_TOPIC = "/robo/video/subscribe/metadata"
VIDEO_OPEN = "/robo/video/call/open_stream"
VIDEO_STATE = "/robo/video/call/stream_state"
VIDEO_CLOSE = "/robo/video/call/close_stream"

# The U1 Pro SDK document declares all five event topics as
# std_msgs/msg/String.  Their String.data value is a JSON envelope.  Keep
# these wire types separate from the local audio bridge messages below; using
# a custom audio_msgs event type would prevent DDS matching on the robot.
EVENT_TOPICS = {
    "doa_event": "/robo/audio/subscribe/doa_event",
    "playback_state": PLAYBACK_TOPIC,
}


def _sensor_schema() -> dict:
    return {
        "type": "object",
        "properties": {"action": {"type": "string", "enum": ["start", "stop", "info"]}},
        "required": ["action"],
    }


def _event_json(message: Any) -> str:
    if hasattr(message, "data") and isinstance(message.data, str):
        return message.data
    return json.dumps(jsonable(message), ensure_ascii=False)


def _json_value(value: Any) -> Any:
    if isinstance(value, str):
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            return value
    return jsonable(value)


def _event_data(event: Any) -> dict:
    """Unwrap the SDK's JSON event envelope for internal consumers."""
    current = _json_value(event)
    for _ in range(4):
        if not isinstance(current, dict) or "data" not in current:
            break
        nested = _json_value(current["data"])
        if not isinstance(nested, dict):
            break
        current = nested
    return current if isinstance(current, dict) else {}


def _bounded_playback(data: dict) -> dict:
    """Forward only the bounded playback fields needed by Agent Core."""
    allowed = ("uuid", "request_type", "phase", "state", "state_name", "code", "success", "message")
    result = {key: data[key] for key in allowed if key in data}
    if isinstance(result.get("message"), str):
        result["message"] = result["message"][:512]
    if isinstance(result.get("uuid"), str):
        result["uuid"] = result["uuid"][:128]
    return result


def _unwrap_result(value: Any) -> dict:
    """Return the JSON object carried by a Trigger/StringCall response."""
    if isinstance(value, dict):
        result = value
    else:
        result = _decode_vendor_result(value)
    for _ in range(4):
        if not isinstance(result, dict):
            return {}
        nested = result.get("data", result.get("result"))
        if isinstance(nested, str):
            try:
                nested = json.loads(nested)
            except json.JSONDecodeError:
                return result
        if not isinstance(nested, dict):
            return result
        result = nested
    return result


def _stream_config(*values: Any) -> dict:
    """Find the documented shared-memory fields in nested service results."""
    keys = {"stream", "state", "path", "frame_payload_size", "max_frames"}
    merged = {}
    def visit(value):
        if isinstance(value, str):
            try:
                value = json.loads(value)
            except json.JSONDecodeError:
                return
        if isinstance(value, dict):
            merged.update({key: value[key] for key in keys if key in value})
            for child in value.values():
                visit(child)
    for value in values:
        visit(value)
    return merged


def _jpeg_from_frame(payload: bytes, metadata: dict) -> bytes:
    """Convert one SDK raw frame to JPEG using only documented metadata."""
    if payload.startswith(b"\xff\xd8\xff"):
        return payload
    from PIL import Image

    width = int(metadata.get("width", 0))
    height = int(metadata.get("height", 0))
    step = int(metadata.get("step", 0))
    encoding = str(metadata.get("encoding", "")).lower().replace("-", "_")
    if width <= 0 or height <= 0:
        raise ValueError("video metadata must contain positive width and height")
    if encoding in {"rgb8", "8uc3"}:
        mode, rawmode, channels = "RGB", "RGB", 3
    elif encoding == "bgr8":
        mode, rawmode, channels = "RGB", "BGR", 3
    elif encoding == "rgba8":
        mode, rawmode, channels = "RGBA", "RGBA", 4
    elif encoding == "bgra8":
        mode, rawmode, channels = "RGBA", "BGRA", 4
    elif encoding in {"mono8", "8uc1"}:
        mode, rawmode, channels = "L", "L", 1
    elif encoding in {"yuv422_yuy2", "yuy2", "yuyv", "yuv422_yuyv"}:
        import numpy as np

        row_bytes = width * 2
        step = step or row_bytes
        if width % 2 or step < row_bytes or len(payload) < step * height:
            raise ValueError("U1 Pro YUY2 frame payload is smaller than metadata dimensions")
        packed = np.frombuffer(payload, dtype=np.uint8).reshape(height, step)[:, :row_bytes]
        yuyv = packed.reshape(height, width // 2, 4).astype(np.int32)
        y = np.empty((height, width), dtype=np.int32)
        y[:, 0::2], y[:, 1::2] = yuyv[:, :, 0], yuyv[:, :, 2]
        u = np.repeat(yuyv[:, :, 1], 2, axis=1) - 128
        v = np.repeat(yuyv[:, :, 3], 2, axis=1) - 128
        c = np.maximum(y - 16, 0)
        rgb = np.stack(((298 * c + 409 * v + 128) >> 8,
                        (298 * c - 100 * u - 208 * v + 128) >> 8,
                        (298 * c + 516 * u + 128) >> 8), axis=-1)
        image = Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8), "RGB")
        import io
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=85, optimize=False)
        return output.getvalue()
    else:
        raise ValueError(f"unsupported U1 Pro video encoding: {encoding!r}")
    row_bytes = width * channels
    step = step or row_bytes
    if step < row_bytes or len(payload) < step * height:
        raise ValueError("U1 Pro video frame payload is smaller than metadata dimensions")
    # Strip row padding before handing the data to Pillow. The SDK payload is
    # a bounded raw frame, not a ROS Image message, so step is significant.
    packed = b"".join(payload[row * step:row * step + row_bytes] for row in range(height))
    image = Image.frombytes(mode, (width, height), packed, "raw", rawmode)
    if mode == "RGBA":
        image = image.convert("RGB")
    import io
    output = io.BytesIO()
    image.save(output, format="JPEG", quality=85, optimize=False)
    return output.getvalue()


class VideoSharedMemoryReader:
    """Read a U1 SDK fixed-slot video shared-memory ring."""

    _RING_HEADER = struct.Struct("<8Q")  # 64-byte, cache-line-aligned SDK header
    _HEADER = struct.Struct("<8Q")  # alignas(64): sequence, timestamp, payload size, reserved

    def __init__(self, config: dict, metadata_getter, frame_callback):
        self.config = dict(config)
        self.metadata_getter = metadata_getter
        self.frame_callback = frame_callback
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self._last_sequence = -1
        self._error = ""

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._ready.clear()
        self._error = ""
        self._thread = threading.Thread(target=self._run, name="u1-video-reader", daemon=True)
        self._thread.start()
        if not self._ready.wait(SERVICE_TIMEOUT):
            self.stop()
            raise RuntimeError(self._error or "timed out waiting for U1 shared-memory stream")
        if self._error:
            self.stop()
            raise RuntimeError(self._error)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._thread = None

    def _run(self):
        path = str(self.config.get("path", ""))
        payload_size = int(self.config.get("frame_payload_size", 0))
        max_frames = int(self.config.get("max_frames", 0))
        if not path or payload_size <= 0 or max_frames <= 0:
            print("[U1 camera] invalid shared-memory video configuration", flush=True)
            return
        slot_size = self._HEADER.size + payload_size
        try:
            deadline = time.monotonic() + SERVICE_TIMEOUT
            handle = None
            while not self._stop.is_set() and time.monotonic() < deadline:
                try:
                    handle = open(path, "rb")
                    if os.fstat(handle.fileno()).st_size >= self._RING_HEADER.size:
                        break
                    handle.close()
                    handle = None
                except FileNotFoundError:
                    pass
                self._stop.wait(0.05)
            if handle is None:
                raise FileNotFoundError(path)
            with handle:
                with mmap.mmap(handle.fileno(), 0, access=mmap.ACCESS_READ) as shared:
                    data_offset = self._RING_HEADER.size
                    if len(shared) < data_offset + slot_size * max_frames:
                        raise ValueError("video shared-memory file is smaller than configured ring")
                    ring_header = self._RING_HEADER.unpack_from(shared, 0)
                    if ring_header[1] != max_frames or ring_header[2] != payload_size:
                        raise ValueError("shared-memory ring header does not match stream configuration")
                    self._ready.set()
                    while not self._stop.is_set():
                        newest = None
                        ring_header = self._RING_HEADER.unpack_from(shared, 0)
                        write_index, ring_frames, ring_payload = ring_header[:3]
                        if ring_frames != max_frames or ring_payload != payload_size:
                            raise ValueError("shared-memory ring header does not match stream configuration")
                        for index in range(max_frames):
                            offset = data_offset + index * slot_size
                            frame_header = self._HEADER.unpack_from(shared, offset)
                            sequence, timestamp_ns, size = frame_header[:3]
                            if write_index == 0 or sequence >= write_index:
                                continue
                            if sequence <= self._last_sequence or size <= 0 or size > payload_size:
                                continue
                            if newest is None or sequence > newest[0]:
                                newest = (sequence, timestamp_ns, bytes(shared[offset + self._HEADER.size:offset + self._HEADER.size + size]))
                        if newest is not None:
                            self._last_sequence = newest[0]
                            self.frame_callback(newest[2], self.metadata_getter(), newest[1])
                        else:
                            self._stop.wait(0.005)
        except FileNotFoundError:
            self._error = f"U1 shared-memory path is unavailable: {path}"
            print(f"[U1 stream] shared-memory path is unavailable: {path}", flush=True)
        except Exception as exc:
            self._error = str(exc)[:256]
            print(f"[U1 stream] shared-memory reader stopped: {self._error}", flush=True)
            self._ready.set()


def _acp_notify(action_id: str | None, status: str, result: dict, tool_name: str = "audio") -> None:
    if not action_id:
        return
    base = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    payload = json.dumps({
        "action_id": action_id,
        "status": status,
        "result": result,
        "tool": tool_name,
        "ts": time.time(),
    }).encode()
    request = urllib.request.Request(
        f"{base}/api/acp/complete",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    context = ssl._create_unverified_context() if base.startswith("https://") else None
    try:
        urllib.request.urlopen(request, timeout=5, context=context).read()
    except Exception:
        safe_id = str(action_id).encode("unicode_escape").decode("ascii")[:128]
        print(f"[U1 ACP] completion request failed for action_id={safe_id}", flush=True)


def _decode_vendor_result(response: Any) -> Any:
    """Decode robo_sdk's JSON envelope while preserving non-JSON responses."""
    success = getattr(response, "success", None)
    value = getattr(response, "message", None)
    if value is None:
        value = getattr(response, "result", response)
    if isinstance(value, str):
        try:
            decoded = json.loads(value)
            if success is False and isinstance(decoded, dict):
                decoded.setdefault("ok", False)
            return decoded
        except json.JSONDecodeError:
            if success is False:
                return {"ok": False, "message": value}
            return {"result": value}
    if success is False:
        return {"ok": False, "message": str(value)}
    return jsonable(value)


class U1Nodes:
    def __init__(self, config: dict, namespace: str, ros) -> None:
        import rclpy
        import rclpy.executors
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from audio_msgs.msg import AudioChunk, AudioInData, AudioOutData, AudioInfo
        from audio_msgs.srv import EnableAudioIn, EnableAudioOut, SetAudioVolume
        from std_msgs.msg import UInt8
        from robo_sdk.srv import StringCall
        from std_srvs.srv import Trigger
        from sensor_msgs.msg import CompressedImage
        try:
            from shm_msgs.msg import Image6m
        except ImportError as exc:
            raise RuntimeError("U1 Pro camera runtime is missing shm_msgs/Image6m") from exc

        self.robot = Node("u1_pro_driver", context=ros.ctx_robot)
        self.core = Node("u1_pro_bridge", namespace=namespace, context=ros.ctx_core)
        self._rclpy = rclpy
        self._audio_context = Context()
        audio_domain_id = int(config.get("ros", {}).get("audio_device_domain_id", 2))
        rclpy.init(context=self._audio_context, domain_id=audio_domain_id)
        self._audio_executor = rclpy.executors.MultiThreadedExecutor(context=self._audio_context)
        self.audio_device = Node("u1_pro_audio_device", context=self._audio_context)
        self._audio_executor.add_node(self.audio_device)
        self._audio_thread = threading.Thread(target=self._spin_audio_device, daemon=True,
                                              name="u1-audio-device-ros")
        self._audio_thread.start()
        self._executor_robot = ros.executor_robot
        self._executor_core = ros.executor_core
        self._executor_robot.add_node(self.robot)
        self._executor_core.add_node(self.core)
        self._closed = False
        self.config = config
        self.namespace = namespace
        self.mic_topic = f"/{namespace}/mic/audio"
        self.AudioChunk = AudioChunk
        self.AudioInData = AudioInData
        self.EnableAudioIn = EnableAudioIn
        self.EnableAudioOut = EnableAudioOut
        self.AudioOutData = AudioOutData
        self.AudioInfo = AudioInfo
        self.UInt8 = UInt8
        self.String = String
        self.CompressedImage = CompressedImage
        self.Image6m = Image6m
        self._speaker_publisher = self.audio_device.create_publisher(AudioOutData, SPEAKER_TOPIC, 10)
        self._volume = None
        self._volume_subscription = self.audio_device.create_subscription(
            UInt8, "/sys/device/audio_out/current_volume", self._volume_callback, 10)
        self._speaker_subscription = None
        self._speaker_forwarding = False
        self._speaker_uuid = ""
        self._speaker_frames = 0

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._audio_qos = best_effort
        self._sensor_qos = best_effort
        self._mic_publisher = self.core.create_publisher(AudioChunk, self.mic_topic, best_effort)
        self._event_publishers = {}
        self._event_forwarding = {}
        self._mic_forwarding = False
        self._mic_frames = 0
        self._mic_frame_event = threading.Event()
        self._mic_subscription = self.audio_device.create_subscription(
            AudioInData, MIC_TOPIC, self._mic_callback, best_effort)
        self._playback_listeners = []
        self._video_metadata = {}
        self._video_metadata_lock = threading.Lock()
        self._robot_subscriptions = []
        for name, topic in EVENT_TOPICS.items():
            output_topic = f"/{namespace}/u1_pro/{name}"
            self._event_publishers[name] = self.core.create_publisher(String, output_topic, reliable)
            self._robot_subscriptions.append(self.robot.create_subscription(String, topic, self._event_callback(name), reliable))
        self._robot_subscriptions.append(self.robot.create_subscription(String, VIDEO_METADATA_TOPIC, self._metadata_callback, reliable))
        self._clients = {
            "mic_enable": self.audio_device.create_client(EnableAudioIn, "/sys/device/audio_in/enable"),
            "speaker_enable": self.audio_device.create_client(EnableAudioOut, "/sys/device/audio_out/enable"),
            "volume": self.audio_device.create_client(SetAudioVolume, "/sys/device/audio_out/set_volume"),
            "motion_list": self.robot.create_client(StringCall, "/robo/audio/call/get_motion_info_list"),
            "play_action": self.robot.create_client(StringCall, "/robo/audio/call/play_action"),
            "play_text": self.robot.create_client(StringCall, "/robo/audio/call/play_text"),
            "interrupt": self.robot.create_client(Trigger, "/robo/audio/call/interrupt_action_audio"),
            "authorize": self.robot.create_client(StringCall, "/robo/auth/call/authorize"),
            "auth_state": self.robot.create_client(Trigger, "/robo/auth/call/auth_state"),
            "wakeup_enabled": self.robot.create_client(StringCall, "/robo/system/call/set_wakeup_enabled"),
            "wakeup_enabled_state": self.robot.create_client(Trigger, "/robo/system/call/get_wakeup_enabled"),
            "vision_enabled": self.robot.create_client(StringCall, "/robo/system/call/set_vision_enabled"),
            "vision_enabled_state": self.robot.create_client(Trigger, "/robo/system/call/get_vision_enabled"),
            "wakeup_followup": self.robot.create_client(StringCall, "/robo/system/call/set_wakeup_followup"),
            "wakeup_followup_state": self.robot.create_client(Trigger, "/robo/system/call/get_wakeup_followup"),
            "video_open": self.robot.create_client(Trigger, VIDEO_OPEN),
            "video_state": self.robot.create_client(Trigger, VIDEO_STATE),
            "video_close": self.robot.create_client(Trigger, VIDEO_CLOSE),
        }

    def _spin_audio_device(self) -> None:
        while self._rclpy.ok(context=self._audio_context):
            self._audio_executor.spin_once(timeout_sec=0.1)

    def initialize_robot(self) -> None:
        """Authorize the SDK and disable its built-in wake word on startup."""
        try:
            auth_state = self.trigger_call("auth_state")
        except Exception:
            auth_state = None
        if (isinstance(auth_state, dict)
                and auth_state.get("code") == "OK"
                and isinstance(auth_state.get("data"), dict)
                and auth_state["data"].get("authorized") is True):
            print("[U1 init] vendor SDK is already authorized", flush=True)
        else:
            U1Nodes._authorize_from_credentials(self)


    def _authorize_from_credentials(self) -> None:
        env_names = {
            "appid": "U1_PRO_APPID",
            "api_key": "U1_PRO_API_KEY",
            "api_secret": "U1_PRO_API_SECRET",
            "device_id": "U1_PRO_DEVICE_ID",
            "license": "U1_PRO_LICENSE",
        }
        auth_config = self.config.get("auth", {})
        values = {
            key: str(auth_config.get(key) or os.environ.get(env_names[key], ""))
            for key in env_names
        }
        auth_file = os.environ.get("U1_PRO_AUTH_FILE") or self.config.get("auth_file")
        if auth_file and any(not value for value in values.values()):
            try:
                file_path = Path(auth_file).resolve()
                file_config = json.loads(file_path.read_text(encoding="utf-8"))
                license_name = file_config.get("license_file")
                license_path = (file_path.parent / license_name).resolve() if license_name else None
                if license_path is None or file_path.parent not in license_path.parents:
                    raise ValueError("license_file must stay next to the auth file")
                file_values = {
                    "appid": file_config.get("appid", ""),
                    "api_key": file_config.get("api_key", ""),
                    "api_secret": file_config.get("api_secret", ""),
                    "device_id": file_config.get("device_id", ""),
                    "license": license_path.read_text(encoding="utf-8"),
                }
                for key, value in file_values.items():
                    if not values[key] and value:
                        values[key] = str(value)
            except (OSError, ValueError, TypeError, json.JSONDecodeError):
                print("[U1 init] authorization file could not be loaded", flush=True)
        missing = [key for key, value in values.items() if not value]
        if missing:
            raise RuntimeError(f"U1 Pro authorization cannot start; missing fields: {', '.join(missing)}")
        try:
            response = self.string_call("authorize", values)
        except Exception as exc:
            raise RuntimeError("U1 Pro authorization request failed") from exc
        if not (isinstance(response, dict)
                and response.get("ok") is True
                and response.get("code") == "OK"
                and isinstance(response.get("data"), dict)
                and response["data"].get("authorized") is True):
            raise RuntimeError("U1 Pro authorization was rejected")
        print("[U1 init] authorization request completed", flush=True)

    def _event_callback(self, name: str):
        def callback(message):
            output = self.String()
            output.data = _event_json(message)
            if name == "playback_state":
                event = _json_value(output.data)
                for listener in tuple(self._playback_listeners):
                    listener(event)
            if not self._event_forwarding.get(name, False):
                return
            self._event_publishers[name].publish(output)
        return callback

    def _metadata_callback(self, message) -> None:
        value = _json_value(_event_json(message))
        data = _event_data(value)
        with self._video_metadata_lock:
            self._video_metadata = dict(data)

    def video_metadata(self) -> dict:
        with self._video_metadata_lock:
            return dict(self._video_metadata)

    def _volume_callback(self, message) -> None:
        self._volume = int(message.data)

    def get_volume(self) -> dict:
        if self._volume is None:
            raise RuntimeError("U1 Pro speaker volume has not been published yet")
        return {"volume": self._volume}

    def _mic_callback(self, message) -> None:
        if not self._mic_forwarding or message.sample_rate != 16000 or message.channels != 1:
            return
        sample_format = str(getattr(message, "sample_format", "")).strip().lower()
        if sample_format not in MIC_SAMPLE_FORMATS:
            return
        chunk = self.AudioChunk()
        chunk.header = message.header
        chunk.format = AUDIO_FORMAT
        chunk.data = list(message.data.data)
        self._mic_publisher.publish(chunk)
        self._mic_frames += 1
        self._mic_frame_event.set()

    def call(self, name: str, request) -> Any:
        client = self._clients[name]
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        deadline = time.monotonic() + SERVICE_TIMEOUT
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise TimeoutError(f"service timeout: {client.srv_name}")
        return future.result()

    def set_mic_enabled(self, enabled: bool) -> dict:
        self._mic_forwarding = False
        request = self.EnableAudioIn.Request()
        request.header = self._audio_header()
        request.enable = bool(enabled)
        response = self.call("mic_enable", request)
        code = int(getattr(response, "code", -1))
        if code != 0:
            raise RuntimeError(f"U1 Pro microphone enable service failed with code {code}")
        if not enabled:
            return {"state": "idle", "source_topic": MIC_TOPIC}
        self._mic_frames = 0
        self._mic_frame_event.clear()
        self._mic_forwarding = True
        return {"state": "running", "source_topic": MIC_TOPIC}

    def wait_for_mic_frame(self, timeout: float) -> bool:
        return self._mic_frame_event.wait(timeout)

    def stop_mic_reader(self) -> None:
        self._mic_forwarding = False

    def set_event_enabled(self, name: str, enabled: bool) -> None:
        self._event_forwarding[name] = enabled

    def add_playback_listener(self, listener) -> None:
        self._playback_listeners.append(listener)

    def string_call(self, name: str, params: dict) -> dict:
        from robo_sdk.srv import StringCall
        request = StringCall.Request()
        request.params = json.dumps(params, ensure_ascii=False, separators=(",", ":"))
        return _decode_vendor_result(self.call(name, request))

    def trigger_call(self, name: str) -> dict:
        from std_srvs.srv import Trigger
        response = self.call(name, Trigger.Request())
        message = getattr(response, "message", "")
        if isinstance(message, str) and message:
            try:
                return json.loads(message)
            except json.JSONDecodeError:
                pass
        if isinstance(response, dict):
            return response
        return {"success": bool(getattr(response, "success", False)), "message": message}

    def set_system_enabled(self, name: str, enabled: bool) -> dict:
        requested = bool(enabled)
        response = self.string_call(name, {"enabled": requested})
        if isinstance(response, dict) and response.get("ok") is False:
            raise RuntimeError(f"U1 Pro {name} request failed: {response.get('code', 'unknown error')}")
        state_name = name.replace("set_", "") + "_state"
        state = self.get_system_enabled(state_name)
        actual = state.get("data", {}).get("enabled") if isinstance(state, dict) else None
        if actual is not requested:
            raise RuntimeError(f"U1 Pro {name} state mismatch: requested {requested}, got {actual!r}")
        return {"ok": True, "requested": requested, "enabled": actual, "state": state}

    def get_system_enabled(self, name: str) -> dict:
        return self.trigger_call(name)

    def open_video(self) -> dict:
        opened = self.trigger_call("video_open")
        state = self.trigger_call("video_state")
        return {"open": opened, "state": state,
                "stream": _stream_config(opened, state)}

    def close_video(self) -> dict:
        return self.trigger_call("video_close")

    def set_volume(self, volume: int) -> dict:
        from audio_msgs.srv import SetAudioVolume
        request = SetAudioVolume.Request()
        request.volume = max(0, min(100, int(volume)))
        response = self.call("volume", request)
        code = getattr(response, "code", 0)
        if int(code) != 0:
            raise RuntimeError(f"U1 Pro volume service failed with code {code}")
        self._volume = request.volume
        return jsonable(response)

    def close_speaker_subscription(self) -> None:
        self._speaker_forwarding = False
        if self._speaker_subscription is not None:
            self.core.destroy_subscription(self._speaker_subscription)
            self._speaker_subscription = None
            try:
                request = self.EnableAudioOut.Request()
                request.header = self._audio_header()
                request.enable = False
                request.info = self._audio_info()
                request.mode = 0
                request.gain = 0.0
                self.call("speaker_enable", request)
            except Exception:
                pass

    def connect_speaker(self, input_topic: str) -> dict:
        self.close_speaker_subscription()
        request = self.EnableAudioOut.Request()
        request.header = self._audio_header()
        request.enable = True
        request.info = self._audio_info()
        request.mode = 0
        request.gain = 0.0
        response = self.call("speaker_enable", request)
        code = int(getattr(response, "code", -1))
        if code != 0:
            raise RuntimeError(f"U1 Pro speaker enable service failed with code {code}")
        self._speaker_uuid = f"u1-{uuid.uuid4().hex}"
        self._speaker_frames = 0
        self._speaker_subscription = self.core.create_subscription(
            self.AudioChunk, input_topic, self._speaker_callback, self._audio_qos)
        self._speaker_forwarding = True
        return {"state": "running", "input_topic": input_topic, "robot_topic": SPEAKER_TOPIC}

    @staticmethod
    def _audio_header():
        from std_msgs.msg import Header
        return Header()

    def _audio_info(self):
        info = self.AudioInfo()
        info.uuid = self._speaker_uuid or f"u1-{uuid.uuid4().hex}"
        info.channels = 1
        info.sample_rate = 16000
        info.sample_format = "S16LE"
        return info

    def _speaker_callback(self, message) -> None:
        if not self._speaker_forwarding:
            return
        if getattr(message, "format", "") != AUDIO_FORMAT:
            return
        output = self.AudioOutData()
        output.header = message.header
        output.uuid = self._speaker_uuid
        output.data.data = list(message.data)
        self._speaker_publisher.publish(output)
        self._speaker_frames += 1

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._mic_forwarding = False
        self._event_forwarding.clear()
        self.close_speaker_subscription()
        self._audio_executor.remove_node(self.audio_device)
        self._audio_executor.shutdown()
        self.audio_device.destroy_node()
        if self._rclpy.ok(context=self._audio_context):
            self._rclpy.shutdown(context=self._audio_context)
        self._audio_thread.join(timeout=1.0)
        self._executor_robot.remove_node(self.robot)
        self._executor_core.remove_node(self.core)
        self.robot.destroy_node()
        self.core.destroy_node()


class MicPlugin:
    PREFIX = "mic"

    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.running = False
        self._enable_requested = False

    def get_tool(self):
        return tool(self.PREFIX, "sensor", "U1 Pro microphone array: live 16 kHz mono PCM audio for ASR.", _sensor_schema(), topic_out=[{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}])

    def start(self):
        if self.running:
            return {"state": "running", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}
        self._enable_requested = True
        try:
            self.nodes.set_mic_enabled(True)
            if not self.nodes.wait_for_mic_frame(2.0):
                raise TimeoutError("no PCM frames received from the U1 microphone topic")
        except Exception as exc:
            message = f"U1 Pro microphone unavailable: {str(exc)[:256]}"
            try:
                self.nodes.set_mic_enabled(False)
            except Exception as cleanup_exc:
                message += f"; microphone disable failed: {str(cleanup_exc)[:192]}"
            else:
                self._enable_requested = False
            self.running = False
            return {"state": "error", "message": message, "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}
        else:
            self.running = True
            return {"state": "running", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}

    def stop(self):
        if not self._enable_requested:
            return
        self._enable_requested = False
        try:
            self.nodes.set_mic_enabled(False)
        except Exception:
            pass
        finally:
            self.running = False

    def dispatch(self, action, args):
        if action not in {"start", "stop", "info"}:
            return None
        if action == "start":
            return self.start()
        elif action == "stop":
            self.stop()
            return {"state": "idle", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}
        return {"state": "running" if self.running else "idle", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}


class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.running = False
        self.input_topic = ""

    def get_tool(self):
        actions = {
            "start": (["input_topic"], "Start playing the connected PCM audio stream."),
            "set_volume": (["volume"], "Set U1 Pro speaker volume from 0 to 100."),
            "get_volume": ([], "Read the current U1 Pro device speaker volume."),
            "stop": ([], "Stop consuming the connected audio stream."),
            "info": ([], "Read speaker connection state."),
        }
        return {"name": self.PREFIX, "type": "actuator", "multiInstance": False, "description": "U1 Pro speaker output stream. Connect a TTS or other audio/pcm-16k output to this card to play it live. Volume can be read or set from 0 to 100.", "inputSchema": action_schema(actions, {"input_topic": {"type": "string", "description": "Connected audio/pcm-16k input topic, normally supplied by the Agent Core stream connection."}, "volume": {"type": "integer", "minimum": 0, "maximum": 100}}), "topic_in": [{"format": "audio/pcm-16k"}]}

    def start(self):
        # The input topic is supplied by Agent Core when the stream is connected;
        # there is nothing to subscribe to during bundle startup.
        self.nodes.close_speaker_subscription()
        self.running = False
        self.input_topic = ""
        return {"state": "ready"}

    def stop(self):
        self.nodes.close_speaker_subscription()
        self.running = False
        self.input_topic = ""

    def dispatch(self, action, args):
        if action == "start":
            topic = str(args.get("input_topic", "")).strip()
            if not topic:
                return {"state": "waiting_for_input", "message": "Connect an audio/pcm-16k output stream to speaker before starting playback."}
            self.input_topic = topic
            result = self.nodes.connect_speaker(topic)
            self.running = True
            return result
        if action == "set_volume":
            return self.nodes.set_volume(args.get("volume", 100))
        if action == "get_volume":
            return self.nodes.get_volume()
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self.running else "idle", "input_topic": self.input_topic,
                    "frames_received": self.nodes._speaker_frames}
        return None


class AudioPlugin:
    PREFIX = "tts"

    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.running = False
        self._lock = threading.Lock()
        self._active: dict | None = None
        self.nodes.add_playback_listener(self._on_playback_state)

    def get_tool(self):
        actions = {
            "speak": (["text"], "Convert the supplied text to speech and play it through the U1 Pro.",),
            "set_volume": (["volume"], "Set the U1 Pro TTS speaker volume from 0 to 100."),
            "get_volume": ([], "Read the U1 Pro TTS speaker volume."),
            "interrupt": ([], "Interrupt the current U1 Pro text-to-speech playback immediately."),
            "stop": ([], "Interrupt the current U1 Pro text-to-speech playback."),
            "info": ([], "Read TTS readiness and any active playback."),
        }
        properties = {
            "text": {"type": "string", "minLength": 1, "description": "Text to speak."},
            "volume": {"type": "integer", "minimum": 0, "maximum": 100, "description": "Speaker volume from 0 to 100."},
            "action_id": {"type": "string", "description": "Optional caller correlation ID; otherwise a UUID is generated."},
        }
        schema = action_schema(actions, properties)
        schema["x-completion"] = {"actions": ["speak"], "timeout": 120}
        return tool(self.PREFIX, "actuator", "U1 Pro text-to-speech output. Submit text to speak; this card does not expose preset motion or raw audio playback.", schema)

    def start(self):
        self.running = True
        return {"state": "ready"}

    def stop(self):
        with self._lock:
            active = self._active
            self._active = None
        result = {"state": "idle"}
        if active:
            try:
                result["interrupt"] = self.nodes.trigger_call("interrupt")
            except Exception as exc:
                result["interrupt_error"] = str(exc)
            finally:
                _acp_notify(active["action_id"], "cancelled", {"state": "cancelled", "action_id": active["action_id"]}, active["tool_name"])
        self.running = False
        return result

    def _queue(self, kind: str, payload: dict, action_id: str, tool_name: str = "audio") -> dict:
        with self._lock:
            if self._active:
                return {"state": "error", "message": "another U1 Pro audio action is active", "action_id": self._active["action_id"]}
            vendor_uuid = str(uuid.uuid4())
            vendor_payload = dict(payload)
            vendor_payload["uuid"] = vendor_uuid
            self._active = {"action_id": action_id, "vendor_uuid": vendor_uuid, "kind": kind, "tool_name": tool_name}
        try:
            result = self.nodes.string_call(kind, vendor_payload)
        except Exception as exc:
            with self._lock:
                if self._active and self._active["action_id"] == action_id:
                    self._active = None
            _acp_notify(action_id, "error", {"state": "error", "message": str(exc)}, tool_name)
            raise
        return {"state": "queued", "action_id": action_id, "request": result}

    def _on_playback_state(self, event: dict) -> None:
        data = _event_data(event)
        if data.get("phase") != "result":
            return
        event_uuid = data.get("uuid")
        with self._lock:
            active = self._active
            if not active or not event_uuid or event_uuid != active["vendor_uuid"]:
                return
            self._active = None
        state_name = str(data.get("state_name") or data.get("state") or "").upper()
        failed = state_name == "FAILED" or data.get("success") is False
        success = not failed and (data.get("success") is True or state_name == "COMPLETED")
        status = "completed" if success else "error"
        _acp_notify(active["action_id"], status, {"state": state_name.lower() or status, "action_id": active["action_id"], "playback": _bounded_playback(data)}, active["tool_name"])

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "info":
            with self._lock:
                active = dict(self._active) if self._active else None
            return {"state": "ready" if self.running else "idle", "active": active}
        if action == "set_volume":
            return self.nodes.set_volume(args.get("volume", 100))
        if action == "get_volume":
            return self.nodes.get_volume()
        if action == "speak":
            text = str(args.get("text", "")).strip()
            if not text:
                raise ValueError("tts.speak requires text")
            action_id = str(args.get("action_id") or uuid.uuid4())[:128]
            return self._queue("play_text", {"text": text[:4096]}, action_id, "tts")
        if action in ("interrupt", "stop"):
            return self.stop()
        return None


class EventPlugin:
    PREFIX = "event"

    def __init__(self, nodes: U1Nodes, name: str, description: str):
        self.nodes, self.name, self.description = nodes, name, description
        self.PREFIX = name
        self.running = False

    def get_tool(self):
        return tool(self.name, "sensor", self.description, _sensor_schema(), topic_out=[{"topic": f"/{self.nodes.namespace}/u1_pro/{self.name}", "format": "data/json"}])

    def start(self):
        if self.running:
            return
        self.nodes.set_event_enabled(self.name, True)
        self.running = True

    def stop(self):
        if not self.running:
            return
        self.nodes.set_event_enabled(self.name, False)
        self.running = False

    def dispatch(self, action, args):
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        elif action != "info":
            return None
        return {"state": "running" if self.running else "idle", "topic_out": [{"topic": f"/{self.nodes.namespace}/u1_pro/{self.name}", "format": "data/json"}]}


class EyeCameraPlugin:
    """Expose one physical U1 eye camera from its verified ROS image topic.

    The U1 SDK's ``open_stream`` service controls a separate single shared-memory
    stream and does not select the left or right eye. The physical eye cards use
    the vendor's ``Image6m`` DDS topics instead.
    """

    def __init__(self, nodes: U1Nodes, eye: str):
        self.nodes = nodes
        self.eye = eye
        self.PREFIX = f"camera_{eye}"
        self.source_topic = f"/sensor/camera/{eye}_eye/color/raw"
        self.topic = f"/{nodes.namespace}/camera/{eye}"
        self.running = False
        self._publisher = None
        self._subscription = None
        self._frame_ready = threading.Event()
        self._metadata = {}
        self._frame_condition = threading.Condition()
        self._latest_jpeg = None
        self._frame_sequence = 0
        self._frames = 0
        self._last_error = ""

    def get_tool(self):
        return tool(
            self.PREFIX, "sensor",
            f"U1 Pro {self.eye} eye RGB camera. Publishes the physical {self.eye} camera as JPEG images.",
            _sensor_schema(),
            topic_out=[{"topic": self.topic, "format": "image/jpeg"}],
        )

    def start(self):
        if self.running:
            return self._state()
        self._frame_ready.clear()
        try:
            if self._publisher is None:
                self._publisher = self.nodes.core.create_publisher(self.nodes.CompressedImage, self.topic, 1)
            self._subscription = self.nodes.robot.create_subscription(
                self.nodes.Image6m, self.source_topic, self._on_frame, self.nodes._sensor_qos)
            if not self._frame_ready.wait(3.0):
                raise TimeoutError(f"no frames received from {self.source_topic}")
            self.running = True
            self._last_error = ""
        except Exception as exc:
            self._last_error = str(exc)[:256]
            if self._subscription:
                self.nodes.robot.destroy_subscription(self._subscription)
                self._subscription = None
            self.running = False
            self._frame_ready.set()
        return self._state()

    def stop(self):
        if self._subscription:
            self.nodes.robot.destroy_subscription(self._subscription)
            self._subscription = None
        self.running = False
        return self._state()

    def _on_frame(self, frame):
        try:
            metadata = {"width": int(frame.width), "height": int(frame.height),
                        "step": int(frame.step), "encoding": _message_text(frame.encoding),
                        "frame_id": _message_text(frame.header.frame_id)}
            payload = bytes(frame.data[:frame.step * frame.height])
            jpeg = _jpeg_from_frame(payload, metadata)
            message = self.nodes.CompressedImage()
            # Image6m uses shm_msgs/Header; CompressedImage requires std_msgs/Header.
            # Copy the fields explicitly so rclpy does not reject the vendor type.
            from std_msgs.msg import Header
            message.header = Header()
            message.header.stamp.sec = int(getattr(frame.header.stamp, "sec", 0))
            message.header.stamp.nanosec = int(getattr(frame.header.stamp, "nanosec", 0))
            message.header.frame_id = metadata["frame_id"]
            message.format = "jpeg"
            message.data = list(jpeg)
            self._publisher.publish(message)
            with self._frame_condition:
                self._latest_jpeg = jpeg
                self._frame_sequence += 1
                self._frames += 1
                self._frame_condition.notify_all()
            self._frame_ready.set()
        except Exception as exc:
            self._last_error = str(exc)[:256]
            self._frame_ready.set()

    def wait_for_jpeg(self, after_sequence=None, timeout_s=5.0):
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._frame_condition:
            baseline = self._frame_sequence if after_sequence is None else after_sequence
            while self._latest_jpeg is None or self._frame_sequence <= baseline:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, self._frame_sequence
                self._frame_condition.wait(timeout=remaining)
            return self._latest_jpeg, self._frame_sequence

    def frame_sequence(self):
        with self._frame_condition:
            return self._frame_sequence

    def _state(self):
        result = {
            "state": "running" if self.running else ("error" if self._last_error else "idle"),
            "topic_out": [{"topic": self.topic, "format": "image/jpeg"}],
            "frames_published": self._frames,
            "metadata": dict(self._metadata),
        }
        if self._last_error:
            result["message"] = self._last_error
        return result

    def dispatch(self, action, args):
        del args
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._state()
        return None


def _message_text(value):
    value = getattr(value, "data", value)
    if isinstance(value, (list, tuple, bytes, bytearray)):
        return bytes(value).split(b"\0", 1)[0].decode("utf-8", "replace")
    return str(value)


class VisionCapturePlugin:
    """Save fresh U1 JPEG frames and encode them as MP4 for Agent Core."""

    PREFIX = "vision_capture"

    def __init__(self, camera: EyeCameraPlugin, config: dict):
        self.camera = camera
        self.config = dict(config or {})
        self.output_dir = os.path.abspath(str(self.config.get(
            "output_dir", "/opt/phanthy-motus/data/vision_capture/u1_pro")))
        self.channel_dir = str(self.config.get("channel_output_dir") or self.output_dir)
        self.fps = max(1.0, min(30.0, float(self.config.get("video_fps", 15))))
        self.default_seconds = max(1.0, min(60.0, float(self.config.get("default_video_seconds", 5))))
        self.max_seconds = max(self.default_seconds, min(60.0, float(self.config.get("max_video_seconds", 60))))
        self._lock = threading.Lock()
        self._active = None
        self._last_recording = None

    def get_tool(self):
        actions = {
            "capture_image": (["image_name"], "Capture a fresh U1 Pro RGB image as a JPEG."),
            "record_video": (["video_name", "duration"], "Record a fresh U1 Pro RGB video as an MP4; duration defaults to 5 seconds and is capped at 60 seconds."),
            "start_recording": (["video_name"], "Start manual continuous recording; use stop_recording to finalize it. The final result is returned by stop_recording and info, not ACP."),
            "stop_recording": (["recording_id"], "Stop the selected manual recording and finalize its MP4."),
            "list": ([], "List saved U1 Pro photos and videos."),
            "delete": (["name"], "Delete one saved .jpg or .mp4 file by its complete filename."),
            "info": ([], "Show camera readiness, output paths, and recording state."),
            "start": ([], "Prepare the U1 Pro capture card."),
            "stop": ([], "Stop an active recording and release capture state."),
        }
        schema = action_schema(actions, {
            "image_name": {"type": "string", "description": "Optional filename stem without .jpg."},
            "video_name": {"type": "string", "description": "Optional filename stem without .mp4."},
            "duration": {"type": "number", "minimum": 1, "maximum": 60, "default": self.default_seconds, "description": "Video duration in seconds."},
            "name": {"type": "string", "description": "Complete saved filename, ending in .jpg or .mp4."},
            "recording_id": {"type": "string", "description": "Recording ID returned by start_recording."},
        })
        schema["x-completion"] = {"actions": ["record_video"], "timeout": int(self.max_seconds + 15)}
        return tool(
            self.PREFIX, "actuator",
            "U1 Pro RGB photo and video capture using the left-eye camera, saves media under the configured shared data directory, and returns a channel-visible path.",
            schema,
        )

    @staticmethod
    def _safe_stem(value, field):
        if value in (None, ""):
            return None
        value = str(value).strip()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,99}", value):
            raise ValueError(f"{field} must contain only letters, numbers, '.', '_' or '-' and be at most 100 characters")
        return value

    def _path(self, stem, suffix, prefix):
        stem = self._safe_stem(stem, "name") or f"{prefix}_{time.time_ns()}"
        path = os.path.join(self.output_dir, stem + suffix)
        if os.path.exists(path):
            raise ValueError(f"file already exists: {os.path.basename(path)}")
        return path

    def _result_path(self, path, mime, state):
        return {"state": state, "filename": os.path.basename(path), "path": path,
                "channel_reply_path": os.path.join(self.channel_dir, os.path.basename(path)),
                "mime": mime, "size": os.path.getsize(path)}

    def _ensure_camera(self):
        if not self.camera.running:
            state = self.camera.start()
            if state.get("state") == "error":
                raise RuntimeError(state.get("message", "U1 Pro camera is unavailable"))

    def start(self):
        return {"state": "ready"}

    def stop(self):
        result = self._stop_recording()
        return result or {"state": "idle"}

    def _info(self):
        with self._lock:
            active = None
            if self._active:
                active = {key: self._active[key] for key in (
                    "recording_id", "state", "path", "duration", "started_at")}
        return {"state": "recording" if active else "ready", "camera": self.camera._state(),
                "output_dir": self.output_dir, "channel_output_dir": self.channel_dir,
                "photos_dir": self.output_dir, "videos_dir": self.output_dir,
                "fps": self.fps, "active_recording": active,
                "last_recording": self._last_recording}

    def _list(self):
        if not os.path.isdir(self.output_dir):
            return {"state": "listed", "files": []}
        files = []
        for name in sorted(os.listdir(self.output_dir)):
            path = os.path.join(self.output_dir, name)
            if os.path.isfile(path) and os.path.splitext(name)[1].lower() in (".jpg", ".mp4"):
                files.append({"filename": name, "path": path, "size": os.path.getsize(path),
                              "mime": "image/jpeg" if name.lower().endswith(".jpg") else "video/mp4"})
        return {"state": "listed", "files": files}

    def _delete(self, name):
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,99}\.(?:jpg|mp4)", name, re.IGNORECASE):
            return {"state": "error", "message": "name must be a complete .jpg or .mp4 filename"}
        path = os.path.join(self.output_dir, name)
        if not os.path.isfile(path):
            return {"state": "error", "message": f"file not found: {name}"}
        os.remove(path)
        return {"state": "deleted", "filename": name}

    def _capture_image(self, args):
        try:
            self._ensure_camera()
            sequence = self.camera.frame_sequence()
            frame, _ = self.camera.wait_for_jpeg(sequence, 5.0)
            if frame is None:
                return {"state": "error", "message": "no fresh U1 Pro camera frame received"}
            os.makedirs(self.output_dir, exist_ok=True)
            path = self._path(args.get("image_name"), ".jpg", "IMG")
            with open(path, "wb") as handle:
                handle.write(frame)
            return self._result_path(path, "image/jpeg", "captured")
        except Exception as exc:
            return {"state": "error", "message": str(exc)}

    def _start_recording(self, args, duration, action_id=None):
        try:
            self._ensure_camera()
            if shutil.which("ffmpeg") is None:
                return {"state": "error", "message": "ffmpeg is required for MP4 recording"}
            with self._lock:
                if self._active:
                    return {"state": "error", "message": "a U1 Pro recording is already active"}
                os.makedirs(self.output_dir, exist_ok=True)
                path = self._path(args.get("video_name"), ".mp4", "VID")
                active = {"state": "recording", "path": path, "duration": duration,
                          "recording_id": f"u1-recording-{uuid.uuid4().hex}",
                          "action_id": action_id, "cancel": threading.Event(),
                          "continuous": duration is None,
                          "started_at": time.time()}
                self._last_recording = None
                self._active = active
                thread = threading.Thread(target=self._record_worker, args=(active,), daemon=True, name="u1-vision-recording")
                active["thread"] = thread
                thread.start()
            result = {"state": "recording", "filename": os.path.basename(path), "path": path,
                      "channel_reply_path": os.path.join(self.channel_dir, os.path.basename(path)), "mime": "video/mp4"}
            if action_id:
                result["action_id"] = action_id
            else:
                result["recording_id"] = active["recording_id"]
            return result
        except Exception as exc:
            return {"state": "error", "message": str(exc)}

    def _record_worker(self, active):
        process = None
        result = None
        try:
            baseline = self.camera.frame_sequence()
            frame, sequence = self.camera.wait_for_jpeg(baseline, 5.0)
            if frame is None:
                raise RuntimeError("no fresh U1 Pro camera frame received")
            duration = active["duration"]
            total_frames = int(round(duration * self.fps)) if duration else None
            command = ["ffmpeg", "-y", "-loglevel", "error", "-f", "mjpeg", "-framerate", str(self.fps),
                       "-i", "pipe:0", "-an", "-vf", "scale=ceil(iw/2)*2:ceil(ih/2)*2",
                       "-c:v", "libx264", "-pix_fmt", "yuv420p"]
            if total_frames:
                command.extend(["-frames:v", str(total_frames)])
            command.extend(["-movflags", "+faststart", active["path"]])
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            active["process"] = process
            index = 0
            next_tick = time.monotonic()
            while not active["cancel"].is_set() and (total_frames is None or index < total_frames):
                process.stdin.write(frame)
                process.stdin.flush()
                index += 1
                next_tick += 1.0 / self.fps
                remaining = max(0.0, next_tick - time.monotonic())
                if remaining:
                    new_frame, new_sequence = self.camera.wait_for_jpeg(sequence, remaining)
                    if new_frame is not None:
                        frame, sequence = new_frame, new_sequence
                    else:
                        time.sleep(remaining)
            process.stdin.close()
            return_code = process.wait(timeout=15)
            if active["cancel"].is_set() and not active["continuous"]:
                raise RuntimeError("recording cancelled")
            if return_code != 0 or not os.path.isfile(active["path"]):
                error = process.stderr.read().decode("utf-8", "replace")[-512:]
                raise RuntimeError(error or "ffmpeg failed to create MP4")
            result = self._result_path(active["path"], "video/mp4", "recorded")
            result["recording_id"] = active["recording_id"]
        except Exception as exc:
            result = {
                "state": "cancelled" if active["cancel"].is_set() else "error",
                "recording_id": active["recording_id"],
                "message": str(exc),
            }
            try:
                if active.get("path") and os.path.exists(active["path"]):
                    os.remove(active["path"])
            except OSError:
                pass
        finally:
            if process and process.poll() is None:
                process.kill()
                process.wait()
            with self._lock:
                self._last_recording = result
                self._active = None
            if active.get("action_id"):
                status = "completed" if result and result.get("state") == "recorded" else (
                    "cancelled" if result and result.get("state") == "cancelled" else "error")
                _acp_notify(active["action_id"], status, result or {"state": "error"}, "vision_capture")

    def _stop_recording(self, recording_id=None):
        with self._lock:
            active = self._active
        if not active:
            return None
        if recording_id and recording_id != active["recording_id"]:
            return {"state": "error", "message": "recording_id does not match the active recording"}
        active["cancel"].set()
        process = active.get("process")
        active["thread"].join(timeout=15)
        if active["thread"].is_alive() and process and process.poll() is None:
            process.kill()
            active["thread"].join(timeout=2)
        if active["thread"].is_alive():
            return {
                "state": "stopping",
                "recording_id": active["recording_id"],
                "message": "recording stop is still in progress",
            }
        with self._lock:
            return self._last_recording or {"state": "cancelled", "recording_id": active["recording_id"]}

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._info()
        if action == "list":
            return self._list()
        if action == "delete":
            return self._delete(args.get("name"))
        if action == "capture_image":
            return self._capture_image(args)
        if action == "start_recording":
            return self._start_recording(args, None)
        if action == "stop_recording":
            return self._stop_recording(args.get("recording_id")) or {"state": "idle", "message": "no active recording"}
        if action == "record_video":
            try:
                duration = max(1.0, min(self.max_seconds, float(args.get("duration", self.default_seconds))))
            except (TypeError, ValueError):
                return {"state": "error", "message": "duration must be a number between 1 and 60"}
            action_id = str(args.get("action_id") or f"u1-vision-{uuid.uuid4().hex}")[:128]
            return self._start_recording(args, duration, action_id)
        return None


class ExpressionPlugin:
    """Semantic Agent card for vendor-provided face and local motions."""

    PREFIX = "expression"
    EXPRESSION_IDS = {
        "A001", "A002", "A003", "A004", "A005", "A006", "A007", "A008",
        "A009", "A010", "A011", "A012", "A013", "A014", "A017", "A018",
        "A019", "A020", "A021", "A022", "A023", "A024", "A025", "A026",
        "A027", "A028", "A029", "A030", "A031", "A032", "A033", "A034",
    }
    EXPRESSIONS = {
        "blink": ("A001", "眨眼"), "raise_eyebrow": ("A002", "挑眉"),
        "gaze": ("A003", "注视"), "close_eyes": ("A004", "闭眼"),
        "frown": ("A005", "皱眉"), "open_mouth": ("A006", "张嘴"),
        "smile": ("A007", "笑"), "pout": ("A008", "嘟嘴"),
        "blow_kiss": ("A009", "飞吻"), "tilt_head": ("A010", "歪头"),
        "shake_head": ("A011", "摇头"), "look_down": ("A012", "低头"),
        "look_up": ("A013", "抬头"), "nod": ("A014", "点头"),
        "wake_up": ("A017", "苏醒"), "shy": ("A018", "害羞"),
        "affectionate": ("A019", "撒娇"), "angry": ("A020", "生气"),
        "sad": ("A021", "伤心/难过"), "surprised": ("A022", "惊讶"),
        "happy": ("A023", "开心"), "distracted": ("A024", "发呆"),
        "confused": ("A025", "困惑"), "anxious": ("A026", "焦虑"),
        "contempt": ("A027", "轻蔑"), "afraid": ("A028", "恐惧"),
        "thinking": ("A029", "思考"), "got_it": ("A030", "想到了"),
        "sleepy": ("A031", "困"), "good_night": ("A032", "睡吧"),
        "laugh": ("A033", "大笑"), "silly_face": ("A034", "鬼脸"),
    }

    def __init__(self, audio: AudioPlugin):
        self.audio = audio
        self.running = False

    def get_tool(self):
        actions = {
            "start": ([], "Prepare the U1 Pro expression action card."),
            "list_actions": ([], "List available expressions by readable name."),
            "play": (["name"], "Play an expression using its readable name from list_actions."),
            "stop": ([], "Interrupt the current U1 Pro expression or audio motion."),
            "info": ([], "Read the expression card and active playback state."),
        }
        schema = action_schema(actions, {
            "name": {"type": "string", "enum": sorted(self.EXPRESSIONS), "description": "Readable expression name returned by list_actions, such as smile or blink."},
            "action_id": {"type": "string", "description": "Optional caller correlation ID."},
        })
        schema["x-completion"] = {"actions": ["play"], "timeout": 120}
        return tool(self.PREFIX, "actuator", "Control U1 Pro preset face expressions and light gestures, such as smile, blink, nod, or head tilt. Discover available actions first; songs and unrelated motions are excluded.", schema)

    def start(self):
        self.running = True
        return {"state": "ready"}

    def stop(self):
        self.running = False
        return self.audio.stop()

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "list_actions":
            response = self.audio.nodes.string_call("motion_list", {})
            result = self._expression_actions(response)
            return {"actions": [item for item in result["actions"]
                                if item["name"] not in {"tilt_head", "shake_head", "look_down", "look_up", "nod"}]}
        if action == "play":
            name = str(args.get("name", "")).strip().lower()
            if name not in self.EXPRESSIONS:
                raise ValueError("expression.play requires a readable name returned by expression.list_actions")
            available = self.dispatch("list_actions", {})["actions"]
            if name not in {item["name"] for item in available}:
                raise ValueError(f"expression {name!r} is not available on this robot firmware")
            motion_id = self.EXPRESSIONS[name][0]
            action_id = str(args.get("action_id") or uuid.uuid4())[:128]
            return self.audio._queue("play_action", {"action": motion_id}, action_id, "expression")
        if action == "stop":
            return self.stop()
        if action == "info":
            with self.audio._lock:
                active = dict(self.audio._active) if self.audio._active else None
            return {"state": "ready" if self.running else "idle", "active": active}
        return None

    @classmethod
    def _expression_actions(cls, response):
        """Keep only documented expression/gesture IDs from the dynamic vendor list."""
        if isinstance(response, str):
            try:
                response = json.loads(response)
            except json.JSONDecodeError:
                return {"actions": []}
        available_ids = set()
        def collect(value):
            if isinstance(value, str):
                try:
                    collect(json.loads(value))
                except json.JSONDecodeError:
                    pass
                return
            if isinstance(value, list):
                for item in value:
                    collect(item)
            if isinstance(value, dict):
                motion_id = str(value.get("motion_id", value.get("motionId",
                                  value.get("action_id", value.get("actionId", value.get("id", ""))))))
                if motion_id:
                    available_ids.add(motion_id)
                name_id = str(value.get("motion_name", value.get("motionName", "")))
                if name_id:
                    normalized = name_id.strip().lower()
                    for name, (_known_id, label) in cls.EXPRESSIONS.items():
                        if normalized in {name, label.lower()}:
                            available_ids.add(_known_id)
                for child in value.values():
                    if isinstance(child, (dict, list, str)):
                        collect(child)
        collect(response)
        actions = [{"name": name, "label": label}
                   for name, (motion_id, label) in cls.EXPRESSIONS.items()
                   if motion_id in available_ids]
        return {"actions": actions}


class _SystemSwitchPlugin:
    """Expose one documented vendor system switch as a small Agent card."""

    def __init__(self, nodes: U1Nodes, prefix: str, set_name: str, get_name: str,
                 description: str):
        self.nodes = nodes
        self.PREFIX = prefix
        self.set_name = set_name
        self.get_name = get_name
        self.description = description

    def get_tool(self):
        actions = {
            "start": ([], "Prepare this U1 Pro system switch card."),
            "stop": ([], "Stop this U1 Pro system switch card without changing the robot capability setting."),
            "enable": ([], "Enable this U1 Pro system capability."),
            "disable": ([], "Disable this U1 Pro system capability."),
            "status": ([], "Read the current U1 Pro system capability state."),
        }
        return tool(self.PREFIX, "actuator", self.description,
                    action_schema(actions, {}))

    def start(self):
        return {"state": "ready"}

    def stop(self):
        return {"state": "idle"}

    def dispatch(self, action, args):
        del args
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "enable":
            return self.nodes.set_system_enabled(self.set_name, True)
        if action == "disable":
            return self.nodes.set_system_enabled(self.set_name, False)
        if action == "status":
            return self.nodes.get_system_enabled(self.get_name)
        return None


class SystemControlsPlugin:
    """Control the three independent vendor switches from one Agent card."""

    PREFIX = "system_controls"
    SWITCHES = {
        "wakeup": ("wakeup_enabled", "wakeup_enabled_state"),
        "wakeup_followup": ("wakeup_followup", "wakeup_followup_state"),
        "visual_behavior": ("vision_enabled", "vision_enabled_state"),
    }

    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes

    def get_tool(self):
        actions = {
            "start": ([], "Prepare the system controls card."),
            "stop": ([], "Stop the card without changing robot settings."),
            "status": (["control"], "Read one or all current U1 Pro system control states."),
            "enable": (["control"], "Enable one U1 Pro system behavior."),
            "disable": (["control"], "Disable one U1 Pro system behavior."),
        }
        return tool(self.PREFIX, "actuator",
                    "Control built-in wakeup, post-wakeup dialog, and autonomous visual behavior. "
                    "Disabling visual behavior stops vendor visual following/idle behavior, but "
                    "does not prevent head motions explicitly requested through expression/head cards.",
                    action_schema(actions, {"control": {
                        "type": "string", "enum": ["wakeup", "wakeup_followup", "visual_behavior", "all"],
                        "description": "Which independent system behavior to control.",
                    }}))

    def start(self):
        return {"state": "ready"}

    def stop(self):
        return {"state": "idle"}

    def _read(self, control):
        set_name, get_name = self.SWITCHES[control]
        return self.nodes.get_system_enabled(get_name)

    def dispatch(self, action, args):
        if action not in {"start", "stop", "status", "enable", "disable"}:
            return None
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        control = args.get("control")
        targets = list(self.SWITCHES) if control == "all" else [control]
        if any(item not in self.SWITCHES for item in targets):
            raise ValueError("control must be wakeup, wakeup_followup, visual_behavior, or all")
        if action == "status":
            states = {item: self._read(item) for item in targets}
            return states if control == "all" else states[control]
        if action in {"enable", "disable"}:
            enabled = action == "enable"
            results = {item: self.nodes.set_system_enabled(self.SWITCHES[item][0], enabled)
                       for item in targets}
            return results if control == "all" else results[control]
        return None


class HeadPlugin:
    """Play documented, safe preset head motions; raw joint control is unsupported."""

    PREFIX = "head"
    HEAD_ACTIONS = {
        "tilt": ("A010", "歪头"),
        "shake": ("A011", "摇头"),
        "look_down": ("A012", "低头"),
        "look_up": ("A013", "抬头"),
        "nod": ("A014", "点头"),
    }

    def __init__(self, audio: AudioPlugin):
        self.audio = audio
        self.running = False

    def get_tool(self):
        actions = {
            "start": ([], "Prepare the U1 Pro head action card."),
            "list_actions": ([], "List available preset head motions by readable name."),
            "play": (["name"], "Play a documented preset head motion by readable name."),
            "stop": ([], "Interrupt the current head motion."),
            "info": ([], "Read head action card state."),
        }
        schema = action_schema(actions, {
            "name": {"type": "string", "enum": sorted(self.HEAD_ACTIONS),
                     "description": "Readable name returned by list_actions."},
            "action_id": {"type": "string", "description": "Optional caller correlation ID."},
        })
        schema["x-completion"] = {"actions": ["play"], "timeout": 120}
        return tool(self.PREFIX, "actuator",
                    "U1 Pro preset head motions such as nod, shake, tilt, look up, and look down. It does not expose raw joint angles.",
                    schema)

    def start(self):
        self.running = True
        return {"state": "ready"}

    def stop(self):
        self.running = False
        return self.audio.stop()

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "list_actions":
            available = ExpressionPlugin._expression_actions(
                self.audio.nodes.string_call("motion_list", {}))["actions"]
            return {"actions": [item for item in available if item["name"] in self.HEAD_ACTIONS]}
        if action == "play":
            name = str(args.get("name", "")).strip().lower()
            if name not in self.HEAD_ACTIONS:
                raise ValueError("head.play requires a readable name returned by head.list_actions")
            available = {item["name"] for item in self.dispatch("list_actions", {})["actions"]}
            if name not in available:
                raise ValueError(f"head action {name!r} is not available on this robot firmware")
            action_id = str(args.get("action_id") or uuid.uuid4())[:128]
            return self.audio._queue("play_action", {"action": self.HEAD_ACTIONS[name][0]}, action_id, "head")
        if action == "stop":
            return self.stop()
        if action == "info":
            return {"state": "ready" if self.running else "idle"}
        return None


class _LifecyclePlugin:
    """Close the shared ROS nodes after all functional cards have stopped."""

    PREFIX = "lifecycle"

    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.closed = False

    def get_tools(self):
        return []

    def start(self):
        if self.closed:
            raise RuntimeError("U1 Pro lifecycle is already closed")

    def stop(self):
        if self.closed:
            return
        self.closed = True
        self.nodes.close()

    def dispatch(self, action, args):
        del action, args
        return None


def build_plugins(config: dict, namespace: str, ros) -> list:
    nodes = U1Nodes(config, namespace, ros)
    # Authenticate before DriverBundle is created or the MCP endpoint is registered.
    try:
        nodes.initialize_robot()
    except Exception:
        try:
            nodes.close()
        except Exception:
            pass
        try:
            ros.shutdown()
        except Exception:
            pass
        raise
    # Keep cleanup first so DriverBundle.stop_all() runs it last, after every
    # card has disabled its vendor resources and stopped publishing.
    audio = AudioPlugin(nodes)
    camera_left = EyeCameraPlugin(nodes, "left")
    camera_right = EyeCameraPlugin(nodes, "right")
    plugins = [_LifecyclePlugin(nodes), MicPlugin(nodes), SpeakerPlugin(nodes), audio,
               ExpressionPlugin(audio), HeadPlugin(audio),
               SystemControlsPlugin(nodes),
               camera_left, camera_right,
               VisionCapturePlugin(camera_left, config.get("vision_capture", {}))]
    descriptions = {
        "doa_event": "Microphone-array sound direction with azimuth and confidence.",
    }
    plugins.extend(EventPlugin(nodes, name, description) for name, description in descriptions.items())
    return plugins
