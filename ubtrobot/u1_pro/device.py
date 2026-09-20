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
from typing import Any

from common.vendor_runtime import action_schema, jsonable, tool


SERVICE_TIMEOUT = 3.0
MIC_TOPIC = "/audio/sense/audio_data_to_asr"
SPEAKER_TOPIC = "/sys/device/audio_out/raw"
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
    """Read the U1 SDK's fixed-slot video ring described in PDF section 4.5."""

    _HEADER = struct.Struct("<QQQ")  # sequence, timestamp_ns, payload_size

    def __init__(self, config: dict, metadata_getter, frame_callback):
        self.config = dict(config)
        self.metadata_getter = metadata_getter
        self.frame_callback = frame_callback
        self._stop = threading.Event()
        self._thread = None
        self._last_sequence = -1

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="u1-video-reader", daemon=True)
        self._thread.start()

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
            with open(path, "rb") as handle:
                with mmap.mmap(handle.fileno(), 0, access=mmap.ACCESS_READ) as shared:
                    if len(shared) < slot_size * max_frames:
                        raise ValueError("video shared-memory file is smaller than configured ring")
                    while not self._stop.is_set():
                        newest = None
                        for index in range(max_frames):
                            offset = index * slot_size
                            sequence, timestamp_ns, size = self._HEADER.unpack_from(shared, offset)
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
            print(f"[U1 camera] shared-memory path does not exist: {path}", flush=True)
        except Exception as exc:
            print(f"[U1 camera] shared-memory reader stopped: {exc}", flush=True)


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
    value = getattr(response, "result", response)
    if isinstance(value, str):
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            return {"result": value}
    return jsonable(value)


class U1Nodes:
    def __init__(self, config: dict, namespace: str, ros) -> None:
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from audio_msgs.msg import AudioChunk, AudioInData, AudioOutData
        from audio_msgs.srv import EnableAudioIn, SetAudioVolume
        from robo_sdk.srv import StringCall
        from std_srvs.srv import Trigger
        from sensor_msgs.msg import CompressedImage

        self.robot = Node("u1_pro_driver", context=ros.ctx_robot)
        self.core = Node("u1_pro_bridge", namespace=namespace, context=ros.ctx_core)
        self._executor_robot = ros.executor_robot
        self._executor_core = ros.executor_core
        self._executor_robot.add_node(self.robot)
        self._executor_core.add_node(self.core)
        self._closed = False
        self.config = config
        self.namespace = namespace
        self.mic_topic = f"/{namespace}/mic/audio"
        self.AudioChunk = AudioChunk
        self.AudioOutData = AudioOutData
        self.String = String
        self.CompressedImage = CompressedImage
        self._speaker_publisher = self.robot.create_publisher(AudioOutData, SPEAKER_TOPIC, 10)
        self._speaker_subscription = None
        self._speaker_forwarding = False
        self._speaker_uuid = ""

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._mic_publisher = self.core.create_publisher(AudioChunk, self.mic_topic, best_effort)
        self._event_publishers = {}
        self._event_forwarding = {}
        self._mic_forwarding = False
        self._playback_listeners = []
        self._video_metadata = {}
        self._video_metadata_lock = threading.Lock()
        self._robot_subscriptions = []
        for name, topic in EVENT_TOPICS.items():
            output_topic = f"/{namespace}/u1_pro/{name}"
            self._event_publishers[name] = self.core.create_publisher(String, output_topic, reliable)
            self._robot_subscriptions.append(self.robot.create_subscription(String, topic, self._event_callback(name), reliable))
        self._robot_subscriptions.append(self.robot.create_subscription(String, VIDEO_METADATA_TOPIC, self._metadata_callback, reliable))
        self._robot_subscriptions.append(self.robot.create_subscription(AudioInData, MIC_TOPIC, self._mic_callback, best_effort))

        self._clients = {
            "mic_enable": self.robot.create_client(EnableAudioIn, "/sys/device/audio_in/enable"),
            "volume": self.robot.create_client(SetAudioVolume, "/sys/device/audio_out/set_volume"),
            "motion_list": self.robot.create_client(StringCall, "/robo/audio/call/get_motion_info_list"),
            "play_action": self.robot.create_client(StringCall, "/robo/audio/call/play_action"),
            "play_text": self.robot.create_client(StringCall, "/robo/audio/call/play_text"),
            "interrupt": self.robot.create_client(Trigger, "/robo/audio/call/interrupt_action_audio"),
            "authorize": self.robot.create_client(StringCall, "/robo/auth/call/authorize"),
            "auth_state": self.robot.create_client(Trigger, "/robo/auth/call/auth_state"),
            "wakeup_enabled": self.robot.create_client(StringCall, "/robo/system/call/set_wakeup_enabled"),
            "video_open": self.robot.create_client(Trigger, VIDEO_OPEN),
            "video_state": self.robot.create_client(Trigger, VIDEO_STATE),
            "video_close": self.robot.create_client(Trigger, VIDEO_CLOSE),
        }

    def initialize_robot(self) -> None:
        """Authorize the SDK and disable its built-in wake word on startup."""
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
        missing = [key for key, value in values.items() if not value]
        if missing:
            print(f"[U1 init] authorization skipped; missing fields: {', '.join(missing)}", flush=True)
        else:
            try:
                self.string_call("authorize", values)
                print("[U1 init] authorization request completed", flush=True)
            except Exception:
                print("[U1 init] authorization request failed", flush=True)

        try:
            self.string_call("wakeup_enabled", {"enabled": False})
            print("[U1 init] built-in wake word disable request completed", flush=True)
        except Exception:
            print("[U1 init] built-in wake word disable request failed", flush=True)

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

    def _mic_callback(self, message) -> None:
        if not self._mic_forwarding:
            return
        if message.sample_rate != 16000 or message.channels != 1:
            return
        chunk = self.AudioChunk()
        chunk.format = "audio/pcm-16k"
        chunk.data = list(message.data.data)
        self._mic_publisher.publish(chunk)

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
        from std_msgs.msg import Header
        from audio_msgs.srv import EnableAudioIn
        if not enabled:
            self._mic_forwarding = False
        request = EnableAudioIn.Request()
        request.header = Header()
        request.enable = enabled
        response = jsonable(self.call("mic_enable", request))
        if enabled:
            self._mic_forwarding = True
        return response

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
        return jsonable(self.call("volume", request))

    def close_speaker_subscription(self) -> None:
        self._speaker_forwarding = False
        if self._speaker_subscription is not None:
            self.core.destroy_subscription(self._speaker_subscription)
            self._speaker_subscription = None

    def connect_speaker(self, input_topic: str) -> dict:
        self.close_speaker_subscription()
        self._speaker_uuid = f"u1-{uuid.uuid4().hex}"
        self._speaker_subscription = self.core.create_subscription(self.AudioChunk, input_topic, self._speaker_callback, 10)
        self._speaker_forwarding = True
        return {"state": "running", "input_topic": input_topic, "robot_topic": SPEAKER_TOPIC}

    def _speaker_callback(self, message) -> None:
        if not self._speaker_forwarding:
            return
        output = self.AudioOutData()
        output.uuid = self._speaker_uuid
        output.data.data = list(message.data)
        self._speaker_publisher.publish(output)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._mic_forwarding = False
        self._event_forwarding.clear()
        self.close_speaker_subscription()
        self._executor_robot.remove_node(self.robot)
        self._executor_core.remove_node(self.core)
        self.robot.destroy_node()
        self.core.destroy_node()


class MicPlugin:
    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.running = False
        self._enable_requested = False

    def get_tool(self):
        return tool("mic", "sensor", "U1 Pro microphone array: live 16 kHz mono PCM audio for ASR.", _sensor_schema(), topic_out=[{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}])

    def start(self):
        if self.running:
            return {"state": "running", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}
        self._enable_requested = True
        try:
            self.nodes.set_mic_enabled(True)
        except Exception as exc:
            self._enable_requested = False
            self.running = False
            return {"state": "error", "message": f"U1 Pro microphone unavailable: {str(exc)[:256]}", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}
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
        if action == "start":
            return self.start()
        elif action == "stop":
            self.stop()
            return {"state": "idle", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}
        elif action != "info":
            return None
        return {"state": "running" if self.running else "idle", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}


class SpeakerPlugin:
    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.running = False
        self.input_topic = ""

    def get_tool(self):
        actions = {
            "start": (["input_topic"], "Start playing the connected PCM audio stream."),
            "set_volume": (["volume"], "Set U1 Pro speaker volume from 0 to 100."),
            "stop": ([], "Stop consuming the connected audio stream."),
            "info": ([], "Read speaker connection state."),
        }
        return {"name": "speaker", "type": "actuator", "multiInstance": False, "description": "U1 Pro speaker. Connect an audio/pcm-16k stream such as TTS or mic audio, then start playback.", "inputSchema": action_schema(actions, {"input_topic": {"type": "string", "description": "Connected audio/pcm-16k input topic"}, "volume": {"type": "integer", "minimum": 0, "maximum": 100}}), "topic_in": [{"format": "audio/pcm-16k"}]}

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
                return self.start()
            self.input_topic = topic
            result = self.nodes.connect_speaker(topic)
            self.running = True
            return result
        if action == "set_volume":
            return self.nodes.set_volume(args.get("volume", 100))
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self.running else "idle", "input_topic": self.input_topic}
        return None


class AudioPlugin:
    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.running = False
        self._lock = threading.Lock()
        self._active: dict | None = None
        self.nodes.add_playback_listener(self._on_playback_state)

    def get_tool(self):
        actions = {
            "start": ([], "Prepare the U1 Pro audio action card."),
            "list_actions": ([], "List command motion IDs available on the U1 Pro."),
            "play_action": (["motion_id"], "Play one documented command motion by its motion_id, such as A029."),
            "play_text": (["text"], "Speak text through the U1 Pro voice output, optionally with a motion and persistence."),
            "stop": ([], "Interrupt the current U1 Pro speech or motion."),
            "info": ([], "Read the U1 Pro audio card and active playback state."),
        }
        properties = {
            "motion_id": {"type": "string", "minLength": 1, "description": "Vendor command motion_id returned by list_actions, for example A029."},
            "text": {"type": "string", "minLength": 1, "description": "Text to speak."},
            "motion": {"type": "string", "description": "Optional vendor motion parameter for play_text."},
            "save": {"type": "boolean", "default": False, "description": "Whether the vendor should persist the generated audio."},
            "action_id": {"type": "string", "description": "Optional caller correlation ID; otherwise a UUID is generated."},
        }
        schema = action_schema(actions, properties)
        schema["x-completion"] = {"actions": ["play_action", "play_text"], "timeout": 120}
        return tool("audio", "actuator", "U1 Pro preset motion and text playback. Use list_actions to discover motion IDs; play calls return queued and complete asynchronously from playback_state.", schema)

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
                _acp_notify(active["action_id"], "cancelled", {"state": "cancelled", "action_id": active["action_id"]})
        self.running = False
        return result

    def _queue(self, kind: str, payload: dict, action_id: str, tool_name: str = "audio") -> dict:
        with self._lock:
            if self._active:
                return {"state": "error", "message": "another U1 Pro audio action is active", "action_id": self._active["action_id"]}
            vendor_uuid = str(uuid.uuid4())
            vendor_payload = dict(payload)
            vendor_payload["uuid"] = vendor_uuid
            self._active = {"action_id": action_id, "vendor_uuid": vendor_uuid, "kind": kind}
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
        _acp_notify(active["action_id"], status, {"state": state_name.lower() or status, "action_id": active["action_id"], "playback": _bounded_playback(data)})

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "info":
            with self._lock:
                active = dict(self._active) if self._active else None
            return {"state": "ready" if self.running else "idle", "active": active}
        if action == "list_actions":
            return self.nodes.string_call("motion_list", {})
        if action == "play_action":
            motion_id = str(args.get("motion_id", "")).strip()
            if not motion_id:
                raise ValueError("audio.play_action requires motion_id")
            action_id = str(args.get("action_id") or uuid.uuid4())[:128]
            return self._queue("play_action", {"action": motion_id}, action_id)
        if action == "play_text":
            text = str(args.get("text", "")).strip()
            if not text:
                raise ValueError("audio.play_text requires text")
            action_id = str(args.get("action_id") or uuid.uuid4())[:128]
            payload = {"text": text[:4096], "save": bool(args.get("save", False))}
            if args.get("motion"):
                payload["motion"] = str(args["motion"])
            return self._queue("play_text", payload, action_id)
        if action == "stop":
            return self.stop()
        return None


class EventPlugin:
    def __init__(self, nodes: U1Nodes, name: str, description: str):
        self.nodes, self.name, self.description = nodes, name, description
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


class CameraRgbPlugin:
    """Expose the SDK video shared-memory stream as Agent Core JPEG frames."""

    def __init__(self, nodes: U1Nodes, config: dict):
        self.nodes = nodes
        self.config = config
        self.topic = f"/{nodes.namespace}/camera/rgb"
        self.running = False
        self._publisher = None
        self._reader = None
        self._metadata = {}
        self._frame_condition = threading.Condition()
        self._latest_jpeg = None
        self._frame_sequence = 0
        self._frames = 0
        self._last_error = ""

    def get_tool(self):
        return tool(
            "camera_rgb", "sensor",
            "U1 Pro RGB camera stream. Starts the documented vendor video stream, reads its shared-memory raw frames, and publishes JPEG images on the Agent Core camera topic.",
            _sensor_schema(),
            topic_out=[{"topic": self.topic, "format": "image/jpeg"}],
        )

    def start(self):
        if self.running:
            return self._state()
        try:
            if self._publisher is None:
                self._publisher = self.nodes.core.create_publisher(self.nodes.CompressedImage, self.topic, 1)
            response = self.nodes.open_video()
            stream = dict(response.get("stream") or {})
            if str(stream.get("state", "OPEN")).upper() == "CLOSED":
                raise RuntimeError("U1 Pro video stream remained closed after open_stream")
            self._metadata = self.nodes.video_metadata()
            reader_config = {**stream, **dict(self.config.get("video", {}))}
            reader_config.setdefault("path", stream.get("path"))
            reader_config.setdefault("frame_payload_size", stream.get("frame_payload_size"))
            reader_config.setdefault("max_frames", stream.get("max_frames"))
            missing = [key for key in ("path", "frame_payload_size", "max_frames") if not reader_config.get(key)]
            if missing:
                raise RuntimeError("U1 Pro video stream response is missing: " + ", ".join(missing))
            self._reader = VideoSharedMemoryReader(reader_config, self.nodes.video_metadata, self._publish_frame)
            self._reader.start()
            self.running = True
            self._last_error = ""
        except Exception as exc:
            self._last_error = str(exc)[:256]
            try:
                self.nodes.close_video()
            except Exception:
                pass
        return self._state()

    def stop(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
        if self.running:
            try:
                self.nodes.close_video()
            except Exception as exc:
                self._last_error = str(exc)[:256]
        self.running = False
        return self._state()

    def _publish_frame(self, payload: bytes, metadata: dict, timestamp_ns: int):
        try:
            jpeg = _jpeg_from_frame(payload, metadata)
            message = self.nodes.CompressedImage()
            message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
            message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
            message.format = "jpeg"
            message.data = list(jpeg)
            self._publisher.publish(message)
            with self._frame_condition:
                self._latest_jpeg = jpeg
                self._frame_sequence += 1
                self._frames += 1
                self._frame_condition.notify_all()
        except Exception as exc:
            self._last_error = str(exc)[:256]

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


class VisionCapturePlugin:
    """Save fresh U1 JPEG frames and encode them as MP4 for Agent Core."""

    def __init__(self, camera: CameraRgbPlugin, config: dict):
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
            "vision_capture", "actuator",
            "U1 Pro RGB photo and video capture. Reuses camera_rgb, saves media under the configured shared data directory, and returns a channel-visible path.",
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

    def __init__(self, audio: AudioPlugin):
        self.audio = audio
        self.running = False

    def get_tool(self):
        actions = {
            "start": ([], "Prepare the U1 Pro expression action card."),
            "list_actions": ([], "List vendor-provided command motions that can be used for expressions or light head/face movements."),
            "play": (["motion_id"], "Play one motion_id returned by list_actions. Do not invent motion IDs."),
            "stop": ([], "Interrupt the current U1 Pro expression or audio motion."),
            "info": ([], "Read the expression card and active playback state."),
        }
        schema = action_schema(actions, {
            "motion_id": {"type": "string", "minLength": 1, "description": "Exact vendor motion_id from list_actions, such as A029."},
            "action_id": {"type": "string", "description": "Optional caller correlation ID."},
        })
        schema["x-completion"] = {"actions": ["play"], "timeout": 120}
        return tool("expression", "actuator", "U1 Pro face and light motion control through vendor preset actions. Discover available motion IDs first; the card does not guess aliases because the vendor list is firmware-dependent.", schema)

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
            return self.audio.nodes.string_call("motion_list", {})
        if action == "play":
            motion_id = str(args.get("motion_id", "")).strip()
            if not motion_id:
                raise ValueError("expression.play requires motion_id from expression.list_actions")
            action_id = str(args.get("action_id") or uuid.uuid4())[:128]
            return self.audio._queue("play_action", {"action": motion_id}, action_id, "expression")
        if action == "stop":
            return self.stop()
        if action == "info":
            with self.audio._lock:
                active = dict(self.audio._active) if self.audio._active else None
            return {"state": "ready" if self.running else "idle", "active": active}
        return None


class _LifecyclePlugin:
    """Close the shared ROS nodes after all functional cards have stopped."""

    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.closed = False

    def get_tools(self):
        return []

    def start(self):
        if self.closed:
            raise RuntimeError("U1 Pro lifecycle is already closed")
        self.nodes.initialize_robot()

    def stop(self):
        if self.closed:
            return
        self.closed = True
        self.nodes.close()


def build_plugins(config: dict, namespace: str, ros) -> list:
    nodes = U1Nodes(config, namespace, ros)
    # Keep cleanup first so DriverBundle.stop_all() runs it last, after every
    # card has disabled its vendor resources and stopped publishing.
    audio = AudioPlugin(nodes)
    camera = CameraRgbPlugin(nodes, config)
    plugins = [_LifecyclePlugin(nodes), MicPlugin(nodes), SpeakerPlugin(nodes), audio,
               ExpressionPlugin(audio), camera,
               VisionCapturePlugin(camera, config.get("vision_capture", {}))]
    descriptions = {
        "doa_event": "Microphone-array sound direction with azimuth and confidence.",
    }
    plugins.extend(EventPlugin(nodes, name, description) for name, description in descriptions.items())
    return plugins
