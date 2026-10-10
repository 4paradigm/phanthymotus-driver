"""UBTECH U1 Pro ROS 2 adapter.

The cards in this module are Agent capabilities, not a mirror of every SDK
management call. The robot's public ROS graph provides useful contracts:
16 kHz microphone PCM, live speaker PCM input, motion playback, and audio events.
"""

from __future__ import annotations

import json
import mmap
import os
import queue
import re
import ssl
import struct
import threading
import time
import urllib.request
import uuid
from pathlib import Path
from typing import Any

from common.vendor_runtime import action_schema, jsonable, tool


SERVICE_TIMEOUT = 10.0
CAMERA_FRAME_TIMEOUT = 3.0
CAMERA_RIGHT_RETRY_TIMEOUT = 5.0
MIC_TOPIC = "/sys/device/audio_in/raw"
SPEAKER_TOPIC = "/sys/device/audio_out/raw"
AUDIO_FORMAT = "audio/pcm-16k"
PLAYBACK_TOPIC = "/robo/media/subscribe/playback_state"
VIDEO_METADATA_TOPIC = "/robo/video/subscribe/metadata"
SDK_AUDIO_OPEN = "/robo/audio/call/open_stream"
SDK_AUDIO_STATE = "/robo/audio/call/stream_state"
SDK_AUDIO_CLOSE = "/robo/audio/call/close_stream"
ASR_AUDIO_TOPIC = "/audio/sense/audio_data_to_asr"
MOTION_LIST_SERVICE = "/robo/audio/call/get_motion_info_list"
JPEG_MAX_PIXELS = 1280 * 720

EVENT_TOPICS = {"playback_state": PLAYBACK_TOPIC}


def _normalize_pcm16k(message: Any, data: Any) -> bytes:
    """Convert U1 AudioInData PCM to the driver's mono 16 kHz S16LE contract."""
    raw = bytes(data)
    sample_rate = int(getattr(message, "sample_rate", 16000) or 16000)
    channels = max(1, int(getattr(message, "channels", 1) or 1))
    sample_format = str(getattr(message, "sample_format", "S16LE") or "S16LE").upper()
    try:
        import numpy as np
        if sample_format in {"S16LE", "PCM_S16LE", "SIGNED_16"}:
            samples = np.frombuffer(raw[:len(raw) - len(raw) % 2], dtype="<i2").astype(np.float32)
        elif sample_format in {"F32LE", "PCM_F32LE", "FLOAT32"}:
            samples = np.frombuffer(raw[:len(raw) - len(raw) % 4], dtype="<f4") * 32767.0
        else:
            return b""
        if not len(samples):
            return b""
        samples = samples[:len(samples) - len(samples) % channels]
        if channels > 1:
            samples = samples.reshape(-1, channels).mean(axis=1)
        if sample_rate != 16000 and len(samples) > 1:
            target_len = max(1, round(len(samples) * 16000 / sample_rate))
            positions = np.linspace(0, len(samples) - 1, target_len)
            samples = np.interp(positions, np.arange(len(samples)), samples)
        return np.clip(samples, -32768, 32767).astype("<i2").tobytes()
    except Exception:
        return b""


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
    """Extract the SDK shared-memory stream fields from nested envelopes."""
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


def _vendor_action_uuid(result: Any) -> str | None:
    value = _unwrap_result(result)
    for candidate in (value, result):
        if isinstance(candidate, dict):
            data = candidate.get("data")
            if isinstance(data, dict) and data.get("uuid"):
                return str(data["uuid"])[:128]
            if candidate.get("uuid"):
                return str(candidate["uuid"])[:128]
    return None


def _vendor_request_failed(result: Any) -> bool:
    """Recognize both Trigger and SDK JSON error envelopes."""
    if not isinstance(result, dict):
        return False
    if result.get("ok") is False or result.get("success") is False:
        return True
    if result.get("accepted") is False:
        return True
    code = result.get("code")
    if code not in (None, 0, "0", "OK"):
        return True
    data = result.get("data")
    if isinstance(data, dict):
        if data.get("accepted") is False:
            return True
        nested_code = data.get("code")
        if nested_code not in (None, 0, "0", "OK"):
            return True
    return False


class VideoSharedMemoryReader:
    """Read one U1 SDK fixed-slot shared-memory stream."""

    _RING_HEADER = struct.Struct("<8Q")
    _HEADER = struct.Struct("<4Q")

    def __init__(self, config: dict, metadata_getter, frame_callback):
        self.config = dict(config)
        self.metadata_getter = metadata_getter
        self.frame_callback = frame_callback
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self._last_sequence = -1
        self._error = ""

    def start(self, timeout=SERVICE_TIMEOUT):
        self._stop.clear()
        self._ready.clear()
        self._error = ""
        self._thread = threading.Thread(target=self._run, name="u1-video-reader", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout):
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
            self._error = "U1 shared-memory stream configuration is incomplete"
            self._ready.set()
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
                    self._stop.wait(0.05)
            if handle is None:
                raise FileNotFoundError(path)
            with handle, mmap.mmap(handle.fileno(), 0, access=mmap.ACCESS_READ) as shared:
                data_offset = self._RING_HEADER.size
                if len(shared) < data_offset + slot_size * max_frames:
                    raise ValueError("shared-memory ring is smaller than the stream configuration")
                ring = self._RING_HEADER.unpack_from(shared, 0)
                if ring[1] != max_frames or ring[2] != payload_size:
                    raise ValueError("shared-memory ring header does not match stream configuration")
                self._ready.set()
                while not self._stop.is_set():
                    newest = None
                    write_index, ring_frames, ring_payload = self._RING_HEADER.unpack_from(shared, 0)[:3]
                    if ring_frames != max_frames or ring_payload != payload_size:
                        raise ValueError("shared-memory ring header changed unexpectedly")
                    for index in range(max_frames):
                        offset = data_offset + index * slot_size
                        sequence, timestamp_ns, size = self._HEADER.unpack_from(shared, offset)[:3]
                        if (sequence <= self._last_sequence or sequence >= write_index
                                or not 0 < size <= payload_size):
                            continue
                        if newest is None or sequence > newest[0]:
                            start = offset + self._HEADER.size
                            payload = bytes(shared[start:start + size])
                            after = self._HEADER.unpack_from(shared, offset)[:3]
                            latest_write_index = self._RING_HEADER.unpack_from(shared, 0)[0]
                            if after != (sequence, timestamp_ns, size) or sequence >= latest_write_index:
                                continue
                            newest = (sequence, timestamp_ns, payload)
                    if newest is None:
                        self._stop.wait(0.005)
                    else:
                        self._last_sequence = newest[0]
                        self.frame_callback(newest[2], self.metadata_getter(), newest[1])
        except FileNotFoundError:
            self._error = f"U1 shared-memory path is unavailable: {path}"
            self._ready.set()
        except Exception as exc:
            self._error = str(exc)[:256]
            self._ready.set()


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
        # The physical stream is 2048x1536.  Encoding both full-resolution
        # eyes as JPEG is more expensive than the Agent display needs and
        # causes visible queueing.  Subsample before YUV conversion so the
        # expensive RGB allocation is bounded, rather than resizing after it.
        scale = 2 if width * height > JPEG_MAX_PIXELS else 1
        packed = np.frombuffer(payload, dtype=np.uint8).reshape(height, step)[:, :row_bytes]
        yuyv = packed.reshape(height, width // 2, 4)[::scale, ::scale].astype(np.int32)
        out_height = yuyv.shape[0]
        out_width = yuyv.shape[1] * 2
        y = np.empty((out_height, out_width), dtype=np.int32)
        y[:, 0::2], y[:, 1::2] = yuyv[:, :, 0], yuyv[:, :, 2]
        u = np.repeat(yuyv[:, :, 1], 2, axis=1) - 128
        v = np.repeat(yuyv[:, :, 3], 2, axis=1) - 128
        c = np.maximum(y - 16, 0)
        rgb = np.stack(((298 * c + 409 * v + 128) >> 8,
                        (298 * c - 100 * u - 208 * v + 128) >> 8,
                        (298 * c + 516 * u + 128) >> 8), axis=-1)
        image = Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8))
        import io
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=75, optimize=False)
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
    image.save(output, format="JPEG", quality=75, optimize=False)
    return output.getvalue()


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
        try:
            from rclpy.callback_groups import ReentrantCallbackGroup
        except ImportError:
            ReentrantCallbackGroup = None
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from audio_msgs.msg import AudioChunk, AudioInData, AudioInfo, AudioOutData
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
        self.AudioInfo = AudioInfo
        self.AudioOutData = AudioOutData
        self.String = String
        self.CompressedImage = CompressedImage
        self.Image6m = Image6m
        self._speaker_publisher = self.audio_device.create_publisher(AudioOutData, SPEAKER_TOPIC, 10)
        self._speaker_subscription = None
        self._speaker_forwarding = False
        self._speaker_uuid = ""
        self._speaker_frames = 0
        self._speaker_enabled = False
        self._audio_service_lock = threading.Lock()
        # Image6m callbacks perform a bounded JPEG conversion. Keep them in
        # their own re-entrant group so left/right conversion does not block
        # microphone/audio callbacks on the device executor.
        self._camera_callback_group = ReentrantCallbackGroup() if ReentrantCallbackGroup else None
        self._video_users = 0
        self._video_stream = {}
        self._video_lock = threading.Lock()
        self._video_metadata = {}
        self._video_metadata_lock = threading.Lock()

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._audio_qos = best_effort
        self._sensor_qos = best_effort
        self._camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._mic_publisher = self.core.create_publisher(AudioChunk, self.mic_topic, best_effort)
        self._event_publishers = {}
        self._event_forwarding = {}
        self._mic_forwarding = False
        self._mic_frames = 0
        self._mic_frame_event = threading.Event()
        self._mic_reader = None
        self._mic_stream = {}
        self._mic_stream_open = False
        self._mic_subscription = self.audio_device.create_subscription(
            AudioInData, MIC_TOPIC, self._mic_topic_callback, self._audio_qos)
        self._mic_asr_subscription = self.audio_device.create_subscription(
            AudioInData, ASR_AUDIO_TOPIC, self._mic_topic_callback, self._audio_qos)
        self._playback_listeners = []
        self._robot_subscriptions = []
        for name, topic in EVENT_TOPICS.items():
            output_topic = f"/{namespace}/u1_pro/{name}"
            self._event_publishers[name] = self.core.create_publisher(String, output_topic, reliable)
            self._robot_subscriptions.append(self.robot.create_subscription(String, topic, self._event_callback(name), reliable))
        self._robot_subscriptions.append(self.robot.create_subscription(String, VIDEO_METADATA_TOPIC, self._metadata_callback, reliable))
        self._clients = {
            "video_open": self.robot.create_client(Trigger, "/robo/video/call/open_stream"),
            "video_state": self.robot.create_client(Trigger, "/robo/video/call/stream_state"),
            "video_close": self.robot.create_client(Trigger, "/robo/video/call/close_stream"),
            "audio_open": self.robot.create_client(Trigger, SDK_AUDIO_OPEN),
            "audio_state": self.robot.create_client(Trigger, SDK_AUDIO_STATE),
            "audio_close": self.robot.create_client(Trigger, SDK_AUDIO_CLOSE),
            # This is the SDK action endpoint used by the deployed U1
            # adapter. The similarly named typed controller service is not
            # usable on the target firmware.
            "play_action": self.robot.create_client(StringCall, "/robo/audio/call/play_action"),
            "motion_list": self.robot.create_client(StringCall, MOTION_LIST_SERVICE),
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
        }

    def _spin_audio_device(self) -> None:
        while self._rclpy.ok(context=self._audio_context):
            try:
                self._audio_executor.spin_once(timeout_sec=0.1)
            except Exception as exc:
                print(f"[U1 audio ROS] executor callback failed: {str(exc)[:256]}", flush=True)
                time.sleep(0.05)

    def initialize_robot(self) -> None:
        """Authorize the SDK and ensure autonomous behavior switches are off."""
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
        for name in ("wakeup_enabled", "wakeup_followup", "vision_enabled"):
            state_name = f"{name}_state"
            try:
                current = U1Nodes._system_enabled_value(self.get_system_enabled(state_name))
            except Exception:
                current = None
            if current is False:
                continue
            try:
                self.set_system_enabled(name, False)
            except Exception:
                # A firmware can reject a redundant set(false); an authoritative
                # readback is enough to establish the required startup state.
                actual = U1Nodes._system_enabled_value(self.get_system_enabled(state_name))
                if actual is not False:
                    raise
        print("[U1 init] autonomous behavior switches are disabled", flush=True)


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
        with self._video_metadata_lock:
            self._video_metadata = _event_data(value)

    def video_metadata(self) -> dict:
        with self._video_metadata_lock:
            return dict(self._video_metadata)

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
        if not enabled:
            self.stop_mic_reader()
            close_error = None
            if self._mic_stream_open:
                try:
                    response = self.trigger_call("audio_close")
                    if _vendor_request_failed(response):
                        raise RuntimeError("U1 Pro audio stream close was rejected")
                except Exception as exc:
                    close_error = exc
                self._mic_stream_open = False
            if close_error:
                raise close_error
            return {"state": "idle", "source_topic": MIC_TOPIC}
        if self._mic_stream_open and self._mic_reader is not None:
            self._mic_forwarding = True
            return {"state": "running", "stream": dict(self._mic_stream),
                    "source": "U1 SDK audio shared-memory stream"}
        if self._mic_stream_open:
            self.set_mic_enabled(False)
        self._mic_forwarding = False
        self.stop_mic_reader()
        opened = self.trigger_call("audio_open")
        if _vendor_request_failed(opened):
            raise RuntimeError("U1 Pro audio stream open was rejected")
        self._mic_stream_open = True
        try:
            state = self.trigger_call("audio_state")
            if _vendor_request_failed(state):
                raise RuntimeError("U1 Pro audio stream state request was rejected")
            stream = _stream_config(opened, state)
            if not all(stream.get(key) for key in ("path", "frame_payload_size", "max_frames")):
                raise RuntimeError("U1 Pro audio stream state did not provide shared-memory configuration")
            self._mic_frames = 0
            self._mic_frame_event.clear()
            self._mic_stream = stream
            # The device-domain raw topic is a supported fallback on firmware
            # builds where the SDK ring is opened but never populated.
            self._mic_forwarding = True
            # The domain-2 raw topic is the reliable live source on the
            # target adapter. The generic SDK ring may remain empty while it
            # is already publishing on the device topic, so it is optional.
            self._mic_reader = VideoSharedMemoryReader(
                stream, lambda: {}, self._publish_mic_frame)
            try:
                self._mic_reader.start(timeout=2.0)
            except Exception:
                # Some firmware opens the SDK stream but only publishes the
                # domain-2 AudioInData topic. Keep that topic fallback alive.
                self._mic_reader = None
        except Exception as exc:
            self.stop_mic_reader()
            try:
                self.set_mic_enabled(False)
            except Exception as close_exc:
                raise RuntimeError(f"{exc}; microphone stream close failed: {close_exc}") from exc
            raise
        self._mic_forwarding = True
        return {"state": state, "stream": stream,
                "source": "U1 SDK audio shared-memory stream"}

    def _publish_mic_frame(self, payload: bytes, _metadata: dict, timestamp_ns: int) -> None:
        if not self._mic_forwarding:
            return
        chunk = self.AudioChunk()
        chunk.format = AUDIO_FORMAT
        chunk.data = list(payload)
        chunk.header = self._audio_header()
        chunk.header.stamp.sec = timestamp_ns // 1_000_000_000
        chunk.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        self._mic_publisher.publish(chunk)
        self._mic_frames += 1
        self._mic_frame_event.set()

    def _mic_topic_callback(self, message) -> None:
        """Accept the device-domain raw topic when the SDK ring is unavailable."""
        if not self._mic_forwarding:
            return
        data = getattr(getattr(message, "data", None), "data", None)
        if data is None:
            data = getattr(message, "data", None)
        if not data:
            return
        payload = _normalize_pcm16k(message, data)
        if not payload:
            return
        chunk = self.AudioChunk()
        chunk.format = AUDIO_FORMAT
        chunk.data = list(payload)
        header = getattr(message, "header", None)
        chunk.header = header if header is not None else self._audio_header()
        self._mic_publisher.publish(chunk)
        self._mic_frames += 1
        self._mic_frame_event.set()

    def wait_for_mic_frame(self, timeout: float) -> bool:
        if self._mic_frame_event.wait(timeout):
            return True
        reader_error = getattr(self._mic_reader, "_error", "") if self._mic_reader else ""
        if reader_error:
            raise RuntimeError(reader_error)
        return False

    def stop_mic_reader(self) -> None:
        self._mic_forwarding = False
        if self._mic_reader:
            self._mic_reader.stop()
            self._mic_reader = None
        self._mic_stream = {}

    def set_event_enabled(self, name: str, enabled: bool) -> None:
        self._event_forwarding[name] = enabled

    def add_playback_listener(self, listener) -> None:
        self._playback_listeners.append(listener)

    def string_call(self, name: str, params: dict) -> dict:
        from robo_sdk.srv import StringCall
        request = StringCall.Request()
        request.params = json.dumps(params, ensure_ascii=False, separators=(",", ":"))
        return _decode_vendor_result(self.call(name, request))

    def play_motion(self, motion_type: int, motion_name: str, legacy_action: str | None = None) -> dict:
        """Play a declared motion through the deployed SDK StringCall API."""
        del motion_type, motion_name
        if not legacy_action:
            raise ValueError("U1 Pro motion requires a vendor action id")
        return self.string_call("play_action", {"action": str(legacy_action)})

    def motion_catalog(self) -> dict[str, dict[str, Any]]:
        """Read the firmware motion catalog; return an empty map on failure."""
        try:
            response = self.string_call("motion_list", {})
            data = _unwrap_result(response)
            entries = data.get("motion_info_list", [])
            catalog = {}
            for item in entries if isinstance(entries, list) else []:
                if not isinstance(item, dict):
                    continue
                motion_id = str(item.get("motion_id", "")).strip()
                if motion_id:
                    catalog[motion_id] = item
            return catalog
        except Exception as exc:
            print(f"[U1 init] motion catalog unavailable: {str(exc)[:160]}", flush=True)
            return {}

    def trigger_call(self, name: str) -> dict:
        from std_srvs.srv import Trigger
        response = self.call(name, Trigger.Request())
        message = getattr(response, "message", "")
        if isinstance(message, str) and message:
            try:
                value = json.loads(message)
                if getattr(response, "success", True) is False and isinstance(value, dict):
                    value.setdefault("success", False)
                return value
            except json.JSONDecodeError:
                pass
        if isinstance(response, dict):
            return response
        return {"success": bool(getattr(response, "success", False)), "message": message}

    def open_video(self) -> dict:
        with self._video_lock:
            if self._video_users:
                self._video_users += 1
                return {"state": {"state": "OPEN"}, "stream": dict(self._video_stream),
                        "users": self._video_users}
            opened = self.trigger_call("video_open")
            if _vendor_request_failed(opened):
                raise RuntimeError("U1 Pro video stream open was rejected")
            try:
                state = self.trigger_call("video_state")
                if _vendor_request_failed(state):
                    raise RuntimeError("U1 Pro video stream state request was rejected")
                stream = _stream_config(opened, state)
                if not all(stream.get(key) for key in ("path", "frame_payload_size", "max_frames")):
                    raise RuntimeError("U1 Pro video stream state did not provide shared-memory configuration")
                if str(stream.get("state", "OPEN")).upper() == "CLOSED":
                    raise RuntimeError("U1 Pro video stream remained closed after open_stream")
            except Exception:
                try:
                    self.trigger_call("video_close")
                except Exception:
                    pass
                raise
            self._video_stream = stream
            self._video_users = 1
            return {"open": opened, "state": state, "stream": dict(stream), "users": 1}

    def close_video(self) -> dict:
        with self._video_lock:
            if not self._video_users:
                return {"state": "closed", "users": 0}
            self._video_users -= 1
            if self._video_users:
                return {"state": "open", "users": self._video_users}
            try:
                result = self.trigger_call("video_close")
            except Exception:
                self._video_users = 0
                self._video_stream = {}
                raise
            self._video_stream = {}
            if _vendor_request_failed(result):
                raise RuntimeError("U1 Pro video stream close was rejected")
            return {"state": "closed", "users": 0, "vendor": result}

    def set_system_enabled(self, name: str, enabled: bool) -> dict:
        requested = bool(enabled)
        response = self.string_call(name, {"enabled": requested})
        if _vendor_request_failed(response):
            raise RuntimeError(f"U1 Pro {name} request failed: {response.get('code', 'unknown error')}")
        state_name = name.replace("set_", "") + "_state"
        state = self.get_system_enabled(state_name)
        # Some adapter builds report a false ROS Trigger transport status
        # while the JSON business envelope is successful. Prefer the
        # business payload when it contains the authoritative enabled field.
        actual = U1Nodes._system_enabled_value(state)
        if actual is None:
            raise RuntimeError(f"U1 Pro {name} state readback failed")
        if actual is not requested:
            raise RuntimeError(f"U1 Pro {name} state mismatch: requested {requested}, got {actual!r}")
        return {"ok": True, "requested": requested, "enabled": actual, "state": state}

    @staticmethod
    def _system_enabled_value(state: Any) -> bool | None:
        payload = _unwrap_result(state)
        actual = payload.get("enabled") if isinstance(payload, dict) else None
        if actual is None and isinstance(state, dict):
            data = state.get("data")
            if isinstance(data, dict):
                actual = data.get("enabled")
        return actual if isinstance(actual, bool) else None

    def get_system_enabled(self, name: str) -> dict:
        return self.trigger_call(name)

    def close_speaker_subscription(self) -> None:
        self._speaker_forwarding = False
        if self._speaker_subscription is not None:
            self.core.destroy_subscription(self._speaker_subscription)
            self._speaker_subscription = None
        # U1 firmware accepts PCM on the raw topic; audio_out/enable is
        # present in the graph but blocks indefinitely on the target image.
        self._speaker_enabled = False

    def connect_speaker(self, input_topic: str) -> dict:
        self._speaker_forwarding = False
        if self._speaker_subscription is not None:
            self.core.destroy_subscription(self._speaker_subscription)
            self._speaker_subscription = None
        self._speaker_uuid = f"u1-{uuid.uuid4().hex}"
        self._speaker_frames = 0
        self._speaker_subscription = self.core.create_subscription(
            self.AudioChunk, input_topic, self._speaker_callback, self._audio_qos)
        self._speaker_forwarding = True
        return {"state": "running", "input_topic": input_topic, "robot_topic": SPEAKER_TOPIC}

    def _audio_info(self):
        info = self.AudioInfo()
        info.uuid = self._speaker_uuid or f"u1-{uuid.uuid4().hex}"
        info.channels = 1
        info.sample_rate = 16000
        info.sample_format = "S16LE"
        return info

    @staticmethod
    def _audio_header():
        from std_msgs.msg import Header
        return Header()

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
        try:
            self.set_mic_enabled(False)
        except Exception:
            self.stop_mic_reader()
        self._event_forwarding.clear()
        self.close_speaker_subscription()
        if self._mic_subscription is not None:
            self.audio_device.destroy_subscription(self._mic_subscription)
            self._mic_subscription = None
        if self._mic_asr_subscription is not None:
            self.audio_device.destroy_subscription(self._mic_asr_subscription)
            self._mic_asr_subscription = None
        if self._video_users:
            try:
                self.trigger_call("video_close")
            except Exception:
                pass
            self._video_users = 0
            self._video_stream = {}
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
        return tool(self.PREFIX, "sensor", "U1 Pro 麦克风阵列：输出 16 kHz 单声道 PCM 音频流，可供语音识别使用。", _sensor_schema(), topic_out=[{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}])

    def start(self):
        if self.running:
            return {"state": "running", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}
        self._enable_requested = True
        microphone_enabled = False
        try:
            self.nodes.set_mic_enabled(True)
            microphone_enabled = True
            if not self.nodes.wait_for_mic_frame(2.0):
                raise TimeoutError(f"no PCM frames received from {MIC_TOPIC}")
        except Exception as exc:
            message = f"U1 Pro microphone unavailable: {str(exc)[:256]}"
            if microphone_enabled:
                try:
                    self.nodes.set_mic_enabled(False)
                except Exception as cleanup_exc:
                    message += f"; microphone disable failed: {str(cleanup_exc)[:192]}"
                else:
                    self._enable_requested = False
            else:
                self._enable_requested = False
            self.running = False
            return {"state": "error", "message": message, "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}
        else:
            self.running = True
            return {"state": "running", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}

    def stop(self):
        if not self._enable_requested:
            self.running = False
            return {"state": "idle"}
        try:
            self.nodes.set_mic_enabled(False)
        except Exception as exc:
            self.running = False
            return {"state": "error", "message": f"microphone disable failed: {str(exc)[:192]}"}
        self._enable_requested = False
        self.running = False
        return {"state": "idle"}

    def dispatch(self, action, args):
        if action not in {"start", "stop", "info"}:
            return None
        if action == "start":
            return self.start()
        elif action == "stop":
            result = self.stop()
            result["topic_out"] = [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]
            return result
        return {"state": "running" if self.running else "idle", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}


class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.running = False
        self.input_topic = ""

    def get_tool(self):
        actions = {
            "start": (["input_topic"], "播放已连接的 PCM 音频流。"),
            "stop": ([], "停止播放已连接的音频流。"),
            "info": ([], "读取扬声器连接状态。"),
        }
        return {"name": self.PREFIX, "type": "actuator", "multiInstance": False, "description": "U1 Pro 扬声器音频输出。连接 audio/pcm-16k 输入流后尝试播放。", "inputSchema": action_schema(actions, {"input_topic": {"type": "string", "description": "已连接的 audio/pcm-16k 输入话题，通常由 Agent Core 的流连接提供。"}}), "topic_in": [{"format": "audio/pcm-16k"}]}

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
            "start": ([], "启动 U1 Pro 文本转语音卡片。"),
            "speak": (["text"], "将文本转换为语音并通过 U1 Pro 播放。",),
            "interrupt": ([], "立即打断当前 TTS 播放。"),
            "stop": ([], "打断当前 TTS 播放。"),
            "info": ([], "读取 TTS 就绪状态和当前播放任务。"),
        }
        properties = {
            "text": {"type": "string", "minLength": 1, "description": "要播放的文本。"},
            "action_id": {"type": "string", "description": "可选的调用关联 ID；未提供时自动生成 UUID。"},
        }
        schema = action_schema(actions, properties)
        schema["x-completion"] = {"actions": ["speak"], "timeout": 120}
        return tool(self.PREFIX, "actuator", "U1 Pro 文本转语音输出。提交文本即可播报；本卡不提供预设动作或原始音频播放。", schema)

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
        # Some vendor services report rejection in their normal response
        # envelope instead of raising. Do not leave the action barrier active
        # when that happens, because no matching playback event may follow.
        if _vendor_request_failed(result):
            with self._lock:
                if self._active and self._active["action_id"] == action_id:
                    self._active = None
            message = str(result.get("message") or result.get("error") or "vendor rejected the request")
            error = {"state": "error", "message": message[:512], "action_id": action_id}
            _acp_notify(action_id, "error", error, tool_name)
            raise RuntimeError(message)
        returned_uuid = _vendor_action_uuid(result)
        if returned_uuid:
            with self._lock:
                if self._active and self._active["action_id"] == action_id:
                    self._active["vendor_uuid"] = returned_uuid
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
            return {"state": "running"}
        self.nodes.set_event_enabled(self.name, True)
        self.running = True
        return {"state": "running"}

    def stop(self):
        if not self.running:
            return {"state": "idle"}
        self.nodes.set_event_enabled(self.name, False)
        self.running = False
        return {"state": "idle"}

    def dispatch(self, action, args):
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        elif action != "info":
            return None
        return {"state": "running" if self.running else "idle", "topic_out": [{"topic": f"/{self.nodes.namespace}/u1_pro/{self.name}", "format": "data/json"}]}


class EyeCameraPlugin:
    """Expose the U1 SDK video shared-memory stream as an eye-camera card."""

    def __init__(self, nodes: U1Nodes, eye: str):
        self.nodes = nodes
        self.eye = eye
        self.PREFIX = f"camera_{eye}"
        self.source_topic = f"/sensor/camera/{eye}_eye/color/raw"
        self.topic = f"/{nodes.namespace}/camera/{eye}"
        self.running = False
        self._publisher = None
        self._metadata = {}
        self._frame_ready = threading.Event()
        self._frame_condition = threading.Condition()
        self._latest_jpeg = None
        self._frame_sequence = 0
        self._frames = 0
        self._last_error = ""
        self._reader = None
        self._subscription = None
        self._video_open = False
        self._frame_queue = queue.Queue(maxsize=1)
        self._frame_worker = None
        self._frame_worker_stop = threading.Event()

    def get_tool(self):
        return tool(
            self.PREFIX, "sensor",
            f"U1 Pro {('左' if self.eye == 'left' else '右')}眼 RGB 摄像头：以 JPEG 图像流输出物理摄像头画面。",
            _sensor_schema(),
            topic_out=[{"topic": self.topic, "format": "image/jpeg"}],
        )

    def start(self):
        if self.running:
            return self._state()
        self._frame_ready.clear()
        self._frame_worker_stop.clear()
        self._frame_worker = threading.Thread(target=self._process_frames, daemon=True,
                                              name=f"u1-camera-{self.eye}")
        self._frame_worker.start()
        try:
            if self._publisher is None:
                self._publisher = self.nodes.core.create_publisher(self.nodes.CompressedImage, self.topic, 1)
            # The SDK ring is a single stream and has no eye-selection input.
            # Keep each card bound to its verified physical-eye DDS topic;
            # use ring frames only if their metadata explicitly identifies
            # this eye, so a generic stream is never mislabeled as stereo.
            camera_kwargs = {}
            camera_group = getattr(self.nodes, "_camera_callback_group", None)
            if camera_group is not None:
                camera_kwargs["callback_group"] = camera_group
            self._subscription = self.nodes.audio_device.create_subscription(
                self.nodes.Image6m, self.source_topic, self._on_frame,
                getattr(self.nodes, "_camera_qos", self.nodes._sensor_qos), **camera_kwargs)
            # Subscribe before opening the shared stream. The adapter can
            # publish the eye topics immediately after video_open returns;
            # registering first avoids losing that startup window.
            response = self.nodes.open_video()
            self._video_open = True
            stream = dict(response.get("stream") or {})
            if str(stream.get("state", "OPEN")).upper() == "CLOSED":
                raise RuntimeError("U1 Pro video stream remained closed after open_stream")
            self._metadata = self.nodes.video_metadata()
            # Use the SDK ring when it identifies a physical eye. Some
            # firmware builds publish only the eye-specific DDS topics, so a
            # ring that does not become ready is optional.
            self._reader = VideoSharedMemoryReader(
                stream, self.nodes.video_metadata, self._on_shared_frame)
            try:
                self._reader.start(timeout=0.5)
            except Exception:
                self._reader.stop()
                self._reader = None
            ready = self._frame_ready.wait(CAMERA_FRAME_TIMEOUT)
            if not ready and self.eye == "right":
                # The U1 adapter can bring up the right-eye DDS writer after
                # the shared video service has opened. Recreate only this
                # subscription once instead of relabeling another eye's data.
                self.nodes.audio_device.destroy_subscription(self._subscription)
                self._subscription = None
                self._frame_ready.clear()
                camera_kwargs = {}
                camera_group = getattr(self.nodes, "_camera_callback_group", None)
                if camera_group is not None:
                    camera_kwargs["callback_group"] = camera_group
                self._subscription = self.nodes.audio_device.create_subscription(
                    self.nodes.Image6m, self.source_topic, self._on_frame,
                    getattr(self.nodes, "_camera_qos", self.nodes._sensor_qos), **camera_kwargs)
                ready = self._frame_ready.wait(CAMERA_RIGHT_RETRY_TIMEOUT)
            if not ready:
                reader_error = getattr(self._reader, "_error", "")
                raise RuntimeError(reader_error or self._last_error or
                                   f"no valid JPEG frames received from {self.source_topic} "
                                   "or an eye-identified U1 video stream")
            self.running = True
            self._last_error = ""
        except Exception as exc:
            self._last_error = str(exc)[:256]
            if self._reader:
                self._reader.stop()
                self._reader = None
            self._stop_frame_worker()
            if self._subscription:
                self.nodes.audio_device.destroy_subscription(self._subscription)
                self._subscription = None
            if self._video_open:
                try:
                    self.nodes.close_video()
                except Exception as close_exc:
                    self._last_error = f"{self._last_error}; stream close failed: {str(close_exc)[:128]}"[:256]
                self._video_open = False
            self.running = False
        return self._state()

    def stop(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
        self._stop_frame_worker()
        if self._subscription:
            self.nodes.audio_device.destroy_subscription(self._subscription)
            self._subscription = None
        if self._video_open:
            try:
                self.nodes.close_video()
            except Exception as exc:
                self._last_error = str(exc)[:256]
            else:
                self._video_open = False
        self.running = False
        return self._state()

    def _on_shared_frame(self, payload: bytes, metadata: dict, timestamp_ns: int):
        if not isinstance(metadata, dict):
            return
        labels = " ".join(str(metadata.get(key, "")) for key in (
            "eye", "camera", "camera_name", "camera_id", "frame_id", "topic", "stream"))
        labels = labels.lower()
        has_left = "left" in labels
        has_right = "right" in labels
        if has_left == has_right or (self.eye == "left") != has_left:
            return
        self._enqueue_frame(payload, metadata, timestamp_ns)

    def _enqueue_frame(self, payload: bytes, metadata: dict, timestamp_ns: int):
        """Keep only the newest frame so conversion never creates latency."""
        item = (payload, dict(metadata), timestamp_ns)
        try:
            self._frame_queue.put_nowait(item)
        except queue.Full:
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._frame_queue.put_nowait(item)
            except queue.Full:
                pass

    def _process_frames(self):
        while not self._frame_worker_stop.is_set():
            try:
                payload, metadata, timestamp_ns = self._frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self._publish_frame(payload, metadata, timestamp_ns)

    def _stop_frame_worker(self):
        worker = self._frame_worker
        if worker is None:
            return
        self._frame_worker_stop.set()
        worker.join(timeout=1.0)
        self._frame_worker = None
        while True:
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                break

    def _publish_frame(self, payload: bytes, metadata: dict, timestamp_ns: int):
        try:
            jpeg = _jpeg_from_frame(payload, metadata)
            message = self.nodes.CompressedImage()
            from std_msgs.msg import Header
            message.header = Header()
            message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
            message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
            message.header.frame_id = str(metadata.get("frame_id", f"{self.eye}_eye"))
            message.format = "jpeg"
            message.data = list(jpeg)
            self._publisher.publish(message)
            self._metadata = dict(metadata)
            self._last_error = ""
            with self._frame_condition:
                self._latest_jpeg = jpeg
                self._frame_sequence += 1
                self._frames += 1
                self._frame_condition.notify_all()
            self._frame_ready.set()
        except Exception as exc:
            self._last_error = str(exc)[:256]
            with self._frame_condition:
                self._frame_condition.notify_all()

    def _on_frame(self, frame):
        """Compatibility adapter for tests and deployments exposing Image6m."""
        header = getattr(frame, "header", None)
        metadata = {"width": int(frame.width), "height": int(frame.height),
                    "step": int(frame.step), "encoding": _message_text(frame.encoding),
                    "frame_id": _message_text(getattr(header, "frame_id", ""))}
        payload = bytes(frame.data[:frame.step * frame.height])
        stamp = getattr(header, "stamp", None)
        timestamp = int(getattr(stamp, "sec", 0)) * 1_000_000_000
        timestamp += int(getattr(stamp, "nanosec", 0))
        if self._frame_worker is None:
            # Preserve the direct callback contract used by ROS-free tests and
            # by callers that feed a frame before card start.
            self._publish_frame(payload, metadata, timestamp)
        else:
            self._enqueue_frame(payload, metadata, timestamp)

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
    size = getattr(value, "size", None)
    value = getattr(value, "data", value)
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (list, tuple, bytes, bytearray, memoryview)):
        raw = bytes(value)
        if isinstance(size, int) and 0 <= size <= len(raw):
            raw = raw[:size]
        return raw.split(b"\0", 1)[0].decode("utf-8", "replace")
    return str(value)


class ExpressionPlugin:
    """Semantic Agent card for vendor-provided face and local motions."""

    PREFIX = "expression"
    EXPRESSIONS = {
        "blink": ("A001", "眨眼"), "raise_eyebrow": ("A002", "挑眉"),
        "gaze": ("A003", "注视"), "close_eyes": ("A004", "闭眼"),
        "frown": ("A005", "皱眉"), "open_mouth": ("A006", "张嘴"),
        "smile": ("A007", "笑"), "pout": ("A008", "嘟嘴"),
        "blow_kiss": ("A009", "飞吻"), "wake_up": ("A017", "苏醒"),
        "shy": ("A018", "害羞"), "affectionate": ("A019", "撒娇"),
        "angry": ("A020", "生气"), "sad": ("A021", "伤心/难过"),
        "surprised": ("A022", "惊讶"), "happy": ("A023", "开心"),
        "distracted": ("A024", "发呆"), "confused": ("A025", "困惑"),
        "anxious": ("A026", "焦虑"), "contempt": ("A027", "轻蔑"),
        "afraid": ("A028", "恐惧"), "thinking": ("A029", "思考"),
        "got_it": ("A030", "想到了"), "sleepy": ("A031", "困"),
        "good_night": ("A032", "睡吧"), "laugh": ("A033", "大笑"),
        "silly_face": ("A034", "鬼脸"),
    }

    def __init__(self, audio: AudioPlugin, motion_catalog: dict[str, dict[str, Any]] | None = None,
                 catalog_available: bool = False):
        self.audio = audio
        self.running = False
        if catalog_available:
            self.EXPRESSIONS = {name: value for name, value in self.EXPRESSIONS.items()
                                if motion_catalog and value[0] in motion_catalog}

    def get_tool(self):
        actions = {
            "start": ([], "Prepare the U1 Pro expression action card."),
            "play": (["name"], "Play an expression using its declared readable name."),
            "interrupt": ([], "Interrupt the current expression motion."),
            "stop": ([], "Interrupt the current U1 Pro expression or audio motion."),
            "info": ([], "Read the expression card and active playback state."),
        }
        schema = action_schema(actions, {
            "name": {"type": "string", "enum": sorted(self.EXPRESSIONS), "description": "Declared readable expression name, such as smile or blink."},
            "action_id": {"type": "string", "description": "Optional ACP action correlation ID."},
        })
        schema["x-completion"] = {"actions": ["play"], "timeout": 120}
        return tool(self.PREFIX, "actuator", "控制 U1 Pro 预设表情和轻量手势，例如微笑和眨眼；头部动作由 head 卡片提供。", schema)

    def start(self):
        self.running = True
        return {"state": "ready"}

    def stop(self):
        self.running = False
        return self.audio.stop()

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "play":
            name = str(args.get("name", "")).strip().lower()
            if name not in self.EXPRESSIONS:
                raise ValueError("expression.play requires one of the declared expression names")
            legacy_action, _motion_name = self.EXPRESSIONS[name]
            action_id = str(args.get("action_id") or uuid.uuid4())[:128]
            return self.audio._queue("play_action", {"action": legacy_action}, action_id, "expression")
        if action in ("interrupt", "stop"):
            return self.stop()
        if action == "info":
            with self.audio._lock:
                active = dict(self.audio._active) if self.audio._active else None
            return {"state": "ready" if self.running else "idle", "active": active}
        return None

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
        "look_down": ("A012", "低头"),
        "look_up": ("A013", "抬头"),
        "nod": ("A014", "点头"),
        "shake": ("A011", "摇头"),
        "tilt": ("A010", "歪头"),
    }

    def __init__(self, audio: AudioPlugin, motion_catalog: dict[str, dict[str, Any]] | None = None,
                 catalog_available: bool = False):
        self.audio = audio
        self.running = False
        if catalog_available:
            self.HEAD_ACTIONS = {name: value for name, value in self.HEAD_ACTIONS.items()
                                 if motion_catalog and value[0] in motion_catalog}

    def get_tool(self):
        actions = {
            "start": ([], "Prepare the U1 Pro head action card."),
            "play": (["name"], "Play a documented preset head motion by readable name."),
            "interrupt": ([], "Interrupt the current head motion."),
            "stop": ([], "Interrupt the current head motion."),
            "info": ([], "Read head action card state."),
        }
        schema = action_schema(actions, {
            "name": {"type": "string", "enum": sorted(self.HEAD_ACTIONS),
                     "description": "Readable head motion name."},
            "action_id": {"type": "string", "description": "Optional ACP action correlation ID."},
        })
        schema["x-completion"] = {"actions": ["play"], "timeout": 120}
        return tool(self.PREFIX, "actuator",
                    "控制 U1 Pro 预设头部动作，不提供原始关节角度控制。",
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
        if action == "play":
            name = str(args.get("name", "")).strip().lower()
            if name not in self.HEAD_ACTIONS:
                raise ValueError("head.play requires one of the declared head motion names")
            legacy_action, _motion_name = self.HEAD_ACTIONS[name]
            action_id = str(args.get("action_id") or uuid.uuid4())[:128]
            return self.audio._queue("play_action", {"action": legacy_action}, action_id, "head")
        if action in ("interrupt", "stop"):
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
    # The firmware catalog is authoritative.  If it is unavailable, the
    # motion cards remain present but expose no actions instead of advertising
    # firmware-dependent static aliases. Song motions (A1xx) are intentionally
    # not mapped into either semantic card.
    motion_catalog = nodes.motion_catalog() if hasattr(nodes, "motion_catalog") else {}
    catalog_available = hasattr(nodes, "motion_catalog")
    camera_left = EyeCameraPlugin(nodes, "left")
    camera_right = EyeCameraPlugin(nodes, "right")
    plugins = [_LifecyclePlugin(nodes), MicPlugin(nodes), SpeakerPlugin(nodes), audio,
               ExpressionPlugin(audio, motion_catalog, catalog_available),
               HeadPlugin(audio, motion_catalog, catalog_available),
               SystemControlsPlugin(nodes),
               camera_left, camera_right]
    from qr_scan import QrScanPlugin
    plugins.append(QrScanPlugin(nodes, config.get("plugins", {}).get("qr_scan", {})))
    return plugins
