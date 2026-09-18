"""UBTECH U1 Pro ROS 2 adapter.

The cards in this module are Agent capabilities, not a mirror of every SDK
management call. The robot's public ROS graph provides useful contracts:
16 kHz microphone PCM, live speaker PCM input, motion playback, and audio events.
"""

from __future__ import annotations

import json
import os
import ssl
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
    except Exception as exc:
        print(f"[U1 ACP] completion failed for {action_id}: {exc}", flush=True)


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
        self._robot_subscriptions = []
        for name, topic in (
            ("main_wakeup_word", "/robo/audio/subscribe/main_wakeup_word"),
            ("wakeup_event", "/robo/audio/subscribe/wakeup_event"),
            ("wakeup_state", "/robo/audio/subscribe/wakeup_state"),
            ("doa_event", "/robo/audio/subscribe/doa_event"),
            ("playback_state", PLAYBACK_TOPIC),
        ):
            output_topic = f"/{namespace}/u1_pro/{name}"
            self._event_publishers[name] = self.core.create_publisher(String, output_topic, reliable)
            self._robot_subscriptions.append(self.robot.create_subscription(String, topic, self._event_callback(name), reliable))
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
        }

    def _event_callback(self, name: str):
        def callback(message):
            output = self.String()
            output.data = _event_json(message)
            if name == "playback_state":
                try:
                    event = json.loads(output.data)
                except (TypeError, json.JSONDecodeError):
                    event = {}
                for listener in tuple(self._playback_listeners):
                    listener(event)
            if not self._event_forwarding.get(name, False):
                return
            self._event_publishers[name].publish(output)
        return callback

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
        response = self.call(name, None)
        message = getattr(response, "message", "")
        if isinstance(message, str) and message:
            try:
                return json.loads(message)
            except json.JSONDecodeError:
                pass
        if isinstance(response, dict):
            return response
        return {"success": bool(getattr(response, "success", False)), "message": message}

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
            return
        self._enable_requested = True
        try:
            self.nodes.set_mic_enabled(True)
        except Exception:
            self.running = False
            raise
        else:
            self.running = True

    def stop(self):
        if not self._enable_requested:
            return
        self._enable_requested = False
        try:
            self.nodes.set_mic_enabled(False)
        finally:
            self.running = False

    def dispatch(self, action, args):
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        elif action != "info":
            raise ValueError(f"unknown mic action: {action}")
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

    def stop(self):
        self.nodes.close_speaker_subscription()
        self.running = False
        self.input_topic = ""

    def dispatch(self, action, args):
        if action == "start":
            topic = str(args.get("input_topic", "")).strip()
            if not topic:
                raise ValueError("speaker.start requires input_topic")
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
        raise ValueError(f"unknown speaker action: {action}")


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

    def _queue(self, kind: str, payload: dict, action_id: str) -> dict:
        with self._lock:
            if self._active:
                raise RuntimeError(f"audio action already active: {self._active['action_id']}")
            self._active = {"action_id": action_id, "kind": kind, "payload": payload}
        try:
            result = self.nodes.string_call(kind, payload)
        except Exception as exc:
            with self._lock:
                if self._active and self._active["action_id"] == action_id:
                    self._active = None
            _acp_notify(action_id, "error", {"state": "error", "message": str(exc)})
            raise
        return {"state": "queued", "action_id": action_id, "request": result}

    def _on_playback_state(self, event: dict) -> None:
        data = event.get("data") if isinstance(event.get("data"), dict) else event
        if not isinstance(data, dict) or data.get("phase") != "result":
            return
        event_uuid = data.get("uuid")
        with self._lock:
            active = self._active
            if not active or not event_uuid or event_uuid != active["action_id"]:
                return
            self._active = None
        state_name = str(data.get("state_name", "")).upper()
        failed = state_name == "FAILED" or data.get("success") is False
        success = not failed and (data.get("success") is True or state_name == "COMPLETED")
        status = "completed" if success else "error"
        _acp_notify(active["action_id"], status, {"state": state_name.lower() or status, "action_id": active["action_id"], "playback": data})

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
            action_id = str(args.get("action_id") or uuid.uuid4())
            return self._queue("play_action", {"action": motion_id, "uuid": action_id}, action_id)
        if action == "play_text":
            text = str(args.get("text", "")).strip()
            if not text:
                raise ValueError("audio.play_text requires text")
            action_id = str(args.get("action_id") or uuid.uuid4())
            payload = {"text": text, "uuid": action_id, "save": bool(args.get("save", False))}
            if args.get("motion"):
                payload["motion"] = str(args["motion"])
            return self._queue("play_text", payload, action_id)
        if action == "stop":
            return self.stop()
        raise ValueError(f"unknown audio action: {action}")


class AuthPlugin:
    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.running = True

    def get_tool(self):
        actions = {
            "start": ([], "Prepare the U1 Pro authentication card."),
            "authorize": (["appid", "api_key", "api_secret", "device_id", "license"], "Authorize protected U1 Pro SDK services with vendor credentials."),
            "auth_state": ([], "Query whether the U1 Pro SDK is currently authorized."),
            "stop": ([], "Stop the authentication card without changing the vendor authorization state."),
            "info": ([], "Read the authentication card state."),
        }
        properties = {
            "appid": {"type": "string", "description": "Vendor application ID."},
            "api_key": {"type": "string", "format": "password", "description": "Vendor API key."},
            "api_secret": {"type": "string", "format": "password", "description": "Vendor API secret."},
            "device_id": {"type": "string", "description": "U1 Pro device ID."},
            "license": {"type": "string", "format": "password", "description": "Vendor license text."},
        }
        return tool("auth", "actuator", "U1 Pro SDK authentication. Authorize before protected audio or event operations; credentials are never stored by this driver.", action_schema(actions, properties))

    def start(self):
        return {"state": "ready"}

    def stop(self):
        self.running = False
        return {"state": "idle"}

    def dispatch(self, action, args):
        if action == "start":
            self.running = True
            return {"state": "ready"}
        if action == "stop":
            return self.stop()
        if action == "info":
            return {"state": "ready" if self.running else "idle"}
        if action == "authorize":
            env_names = {
                "appid": "U1_PRO_APPID",
                "api_key": "U1_PRO_API_KEY",
                "api_secret": "U1_PRO_API_SECRET",
                "device_id": "U1_PRO_DEVICE_ID",
                "license": "U1_PRO_LICENSE",
            }
            auth_config = self.nodes.config.get("auth", {})
            values = {
                key: str(args.get(key) or auth_config.get(key) or os.environ.get(env_names[key], ""))
                for key in env_names
            }
            missing = [key for key, value in values.items() if not value]
            if missing:
                raise ValueError("missing U1 Pro auth fields: " + ", ".join(missing))
            return self.nodes.string_call("authorize", values)
        if action == "auth_state":
            return self.nodes.trigger_call("auth_state")
        raise ValueError(f"unknown auth action: {action}")


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
            raise ValueError(f"unknown event action: {action}")
        return {"state": "running" if self.running else "idle", "topic_out": [{"topic": f"/{self.nodes.namespace}/u1_pro/{self.name}", "format": "data/json"}]}


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

    def stop(self):
        if self.closed:
            return
        self.closed = True
        self.nodes.close()


def build_plugins(config: dict, namespace: str, ros) -> list:
    nodes = U1Nodes(config, namespace, ros)
    # Keep cleanup first so DriverBundle.stop_all() runs it last, after every
    # card has disabled its vendor resources and stopped publishing.
    plugins = [_LifecyclePlugin(nodes), AuthPlugin(nodes), MicPlugin(nodes), SpeakerPlugin(nodes), AudioPlugin(nodes)]
    descriptions = {
        "main_wakeup_word": "Main wake-word event recognized by the U1 Pro.",
        "wakeup_event": "U1 Pro wake-up recognition event; does not guarantee a follow-up conversation.",
        "wakeup_state": "Current U1 Pro wake-up state.",
        "doa_event": "Microphone-array sound direction with azimuth and confidence.",
        "playback_state": "U1 Pro audio completion or interruption events for current audio UUIDs.",
    }
    plugins.extend(EventPlugin(nodes, name, description) for name, description in descriptions.items())
    return plugins
