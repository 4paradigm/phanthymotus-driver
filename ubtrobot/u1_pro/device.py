"""ROS 2 adapter for the public UBTECH robo_sdk U1 Pro contract.

The vendor document exposes JSON envelopes over ``robo_sdk/srv/StringCall``
and ``std_srvs/srv/Trigger``.  This module deliberately does not invent a
lower-level joint or actuator protocol; all calls and event topics are the
ones documented by the vendor SDK.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any

from common.vendor_runtime import DriverBundle, action_schema, jsonable, tool


CALL_TIMEOUT = 3.0


def _parse_json(value: Any) -> Any:
    if isinstance(value, str):
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            return value
    return jsonable(value)


class U1Nodes:
    def __init__(self, config: dict, namespace: str, ros) -> None:
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        from robo_sdk.srv import StringCall
        from std_srvs.srv import Trigger

        from rclpy.node import Node

        self.robot = Node("u1_pro_driver", context=ros.ctx_robot)
        self.core = Node("u1_pro_bridge", namespace=namespace, context=ros.ctx_core)
        self._ros = ros
        self._lock = threading.Lock()
        self._subscriptions = []
        self._publishers = {}
        self._String = String
        self._topics: dict[str, str] = {}
        self._services = {}

        service_types = {"call": StringCall, "trigger": Trigger}
        self._service_types = service_types
        for name, kind in SERVICE_TYPES.items():
            srv_type = service_types[kind]
            self._services[name] = self.robot.create_client(srv_type, name)

        self._event_topics = dict(EVENT_TOPICS)
        event_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        regular_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        for event_name, robot_topic in self._event_topics.items():
            output_topic = f"/{namespace}/ubtrobot_u1_pro/{event_name}"
            self._topics[event_name] = output_topic
            self._publishers[event_name] = self.core.create_publisher(String, output_topic, event_qos if event_name == "ready_state" else regular_qos)
            qos = event_qos if event_name == "ready_state" else regular_qos
            self._subscriptions.append(
                self.robot.create_subscription(String, robot_topic, self._forward(event_name), qos)
            )

    def _forward(self, event_name: str):
        def callback(message):
            output = self._String()
            output.data = message.data
            self._publishers[event_name].publish(output)

        return callback

    def call(self, service: str, params: dict | None = None) -> dict:
        client = self._services[service]
        if not client.wait_for_service(timeout_sec=CALL_TIMEOUT):
            raise RuntimeError(f"service unavailable: {service}")
        request = self._service_types[SERVICE_TYPES[service]].Request()
        if SERVICE_TYPES[service] == "call":
            request.params = json.dumps(params or {}, ensure_ascii=False, separators=(",", ":"))
        future = client.call_async(request)
        deadline = time.monotonic() + CALL_TIMEOUT
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise TimeoutError(f"service timeout: {service}")
        response = future.result()
        if SERVICE_TYPES[service] == "trigger":
            value = getattr(response, "message", "")
            return _parse_json(value) if value else {"success": bool(getattr(response, "success", False))}
        value = getattr(response, "result", None)
        if value is None:
            value = getattr(response, "message", "")
        parsed = _parse_json(value)
        return parsed if isinstance(parsed, dict) else {"result": parsed}

    def close(self):
        self.robot.destroy_node()
        self.core.destroy_node()


SERVICE_TYPES = {
    "/robo/auth/call/authorize": "call",
    "/robo/auth/call/auth_state": "trigger",
    "/robo/system/call/get_ready_state": "trigger",
    "/robo/system/call/get_serial_number": "trigger",
    "/robo/system/call/get_system_version": "trigger",
    "/robo/system/call/get_soft_version": "trigger",
    "/robo/system/call/set_wakeup_followup": "call",
    "/robo/system/call/get_wakeup_followup": "trigger",
    "/robo/system/call/set_wakeup_enabled": "call",
    "/robo/system/call/get_wakeup_enabled": "trigger",
    "/robo/system/call/set_vision_enabled": "call",
    "/robo/system/call/get_vision_enabled": "trigger",
    "/robo/system/call/set_face_recognition_enabled": "call",
    "/robo/system/call/get_face_recognition_enabled": "trigger",
    "/robo/audio/call/get_motion_info_list": "call",
    "/robo/audio/call/play_action": "call",
    "/robo/audio/call/play_text": "call",
    "/robo/audio/call/interrupt_action_audio": "trigger",
    "/robo/audio/call/open_stream": "trigger",
    "/robo/audio/call/stream_state": "trigger",
    "/robo/audio/call/close_stream": "trigger",
    "/robo/video/call/open_stream": "trigger",
    "/robo/video/call/stream_state": "trigger",
    "/robo/video/call/close_stream": "trigger",
}

EVENT_TOPICS = {
    "ready_state": "/robo/system/subscribe/ready_state",
    "playback_state": "/robo/media/subscribe/playback_state",
    "main_wakeup_word": "/robo/audio/subscribe/main_wakeup_word",
    "wakeup_event": "/robo/audio/subscribe/wakeup_event",
    "wakeup_state": "/robo/audio/subscribe/wakeup_state",
    "doa_event": "/robo/audio/subscribe/doa_event",
    "video_metadata": "/robo/video/subscribe/metadata",
}


def _schema(actions: dict[str, tuple[list[str], str]], properties: dict) -> dict:
    return action_schema(actions, properties)


class ServicePlugin:
    def __init__(self, nodes: U1Nodes, name: str, description: str, actions, properties=None):
        self.nodes, self.name = nodes, name
        self.description, self.actions = description, actions
        self.properties = properties or {}

    def get_tool(self):
        return tool(self.name, "actuator", self.description, _schema(self.actions, self.properties))

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        service, params = ACTIONS[self.name][action]
        if service is None:
            return {"state": "running"}
        payload = {key: args[key] for key in params if key in args}
        if self.name == "speaker" and action == "play_action" and "action_id" in payload:
            payload["action"] = payload.pop("action_id")
        return self.nodes.call(service, payload)


ACTIONS = {
    "auth": {
        "authorize": ("/robo/auth/call/authorize", ["appid", "api_key", "api_secret", "device_id", "license"]),
        "auth_state": ("/robo/auth/call/auth_state", []),
    },
    "system": {name: (path, [] if name.startswith("get_") else ["enabled"]) for name, path in {
        "get_ready_state": "/robo/system/call/get_ready_state",
        "get_serial_number": "/robo/system/call/get_serial_number",
        "get_system_version": "/robo/system/call/get_system_version",
        "get_soft_version": "/robo/system/call/get_soft_version",
    }.items()},
    "wakeup": {
        "set_followup": ("/robo/system/call/set_wakeup_followup", ["enabled"]),
        "get_followup": ("/robo/system/call/get_wakeup_followup", []),
        "set_enabled": ("/robo/system/call/set_wakeup_enabled", ["enabled"]),
        "get_enabled": ("/robo/system/call/get_wakeup_enabled", []),
    },
    "vision": {
        "set_vision_enabled": ("/robo/system/call/set_vision_enabled", ["enabled"]),
        "get_vision_enabled": ("/robo/system/call/get_vision_enabled", []),
        "set_face_recognition_enabled": ("/robo/system/call/set_face_recognition_enabled", ["enabled"]),
        "get_face_recognition_enabled": ("/robo/system/call/get_face_recognition_enabled", []),
    },
    "speaker": {
        "get_motion_info_list": ("/robo/audio/call/get_motion_info_list", []),
        "play_action": ("/robo/audio/call/play_action", ["action_id", "uuid"]),
        "play_text": ("/robo/audio/call/play_text", ["text", "motion", "uuid", "save"]),
        "interrupt": ("/robo/audio/call/interrupt_action_audio", []),
    },
    "audio_stream": {name: (path, []) for name, path in {
        "open": "/robo/audio/call/open_stream", "state": "/robo/audio/call/stream_state", "close": "/robo/audio/call/close_stream"}.items()},
    "video_stream": {name: (path, []) for name, path in {
        "open": "/robo/video/call/open_stream", "state": "/robo/video/call/stream_state", "close": "/robo/video/call/close_stream"}.items()},
}


class EventPlugin:
    def __init__(self, nodes: U1Nodes, name: str, description: str):
        self.nodes, self.name, self.description = nodes, name, description

    def get_tool(self):
        return tool(self.name, "sensor", self.description, topic_out=[{"topic": self.nodes._topics[self.name], "format": "data/json"}])

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        return {"state": "idle" if action == "stop" else "running", "topic_out": [{"topic": self.nodes._topics[self.name], "format": "data/json"}]}


def build_plugins(config: dict, namespace: str, ros) -> list:
    nodes = U1Nodes(config, namespace, ros)
    plugins = []
    schemas = {
        "auth": ({"appid": {"type": "string"}, "api_key": {"type": "string", "format": "password"}, "api_secret": {"type": "string", "format": "password"}, "device_id": {"type": "string"}, "license": {"type": "string", "format": "password"}}, "Authorize the U1 Pro SDK. Authorize before using protected system, audio, video, or event interfaces."),
        "system": ({}, "Read whether the U1 Pro is ready, its serial number, and system/software versions."),
        "wakeup": ({"enabled": {"type": "boolean"}}, "Enable or disable wake-word detection, or allow/block the interaction flow after wake-up."),
        "vision": ({"enabled": {"type": "boolean"}}, "Enable or disable visual behaviors, or independently enable/disable face recognition."),
        "speaker": ({"action_id": {"type": "string"}, "text": {"type": "string"}, "motion": {"type": "string"}, "uuid": {"type": "string"}, "save": {"type": "boolean"}}, "Make the U1 Pro speak, play one vendor motion action, or stop the current speech/action. Use playback_state to observe the real execution result."),
        "audio_stream": ({}, "Advanced U1 Pro microphone shared-memory stream control. Opens or closes the vendor stream and returns its shared-memory configuration; this driver does not convert raw bytes to an ASR audio topic."),
        "video_stream": ({}, "Advanced U1 Pro camera shared-memory stream control. Opens or closes the vendor stream and returns its shared-memory configuration; use video_metadata to interpret frames."),
    }
    for name, (properties, description) in schemas.items():
        plugins.append(ServicePlugin(nodes, name, description, {key: ([], key) for key in ACTIONS[name]}, properties))
    descriptions = {
        "ready_state": "Current U1 Pro device-ready state. Use this before protected operations.",
        "playback_state": "Authoritative speech/action execution states. A speaker call only means the request was accepted; use this stream for the actual result.",
        "main_wakeup_word": "Event emitted when the U1 Pro recognizes its main wake word.", "wakeup_event": "Event emitted for a recognized wake-up; it does not guarantee a follow-up conversation.",
        "wakeup_state": "Current wake-up state event stream, separate from the post-wake interaction-flow switch.", "doa_event": "Direction-of-arrival event stream from the U1 Pro microphone array.",
        "video_metadata": "Metadata for U1 Pro video shared-memory frames: frame ID, encoding, width, height, and row step.",
    }
    plugins.extend(EventPlugin(nodes, name, description) for name, description in descriptions.items())
    return plugins
