#!/usr/bin/env python3
"""
drivers/phanthy/remote_control/plugins.py — 远程控制插件。

三个 actuator 插件：
  RemoteMessagePlugin   — 发送文本消息到 ROS2 topic
  RemoteAudioPlugin     — 发送音频文件（转 PCM-16k）到 ROS2 topic
  RemoteMicPlugin       — 接收浏览器 WebSocket 实时 PCM-16k 流并发布到 ROS2 topic
"""

import audioop
import base64
import io
import json
import struct
import time
import threading
import wave

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Header, String
from audio_msgs.msg import AudioChunk

# ── 常量 ──────────────────────────────────────────────────────────────────────

CHUNK_BYTES = 1024  # bytes per ROS2 publish (~32ms at 16kHz/16bit/mono)

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


# ── RemoteMessagePlugin ──────────────────────────────────────────────────────

class _MessageNode(Node):
    def __init__(self, topic: str):
        super().__init__("remote_message")
        self._pub = self.create_publisher(String, topic, 10)
        self.get_logger().info(f"MessageNode ready — topic: {topic}")

    def publish_message(self, text: str):
        msg = String()
        msg.data = json.dumps({"text": text, "ts": time.time()}, ensure_ascii=False)
        self._pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


class RemoteMessagePlugin:
    PREFIX = "remote_message"

    def __init__(self, namespace: str, executor):
        self._topic = f"/{namespace}/remote_message"
        self._node = _MessageNode(self._topic)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "remote_message",
            "type": "sensor",
            "multiInstance": False,
            "description": "Send a text message via remote control. Publishes JSON to ROS2 topic.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["send_message"],
                        "description": "Action to perform",
                    },
                    "text": {
                        "type": "string",
                        "description": "Message text to send",
                    },
                },
                "required": ["action", "text"],
            },
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "send_message":
            text = args.get("text", "")
            if not text:
                return {"error": "Missing required parameter: text"}
            self._node.publish_message(text)
            return {"status": "sent", "text": text}
        return {"error": f"Unknown action: {action}"}


# ── RemoteAudioPlugin ────────────────────────────────────────────────────────

class _AudioNode(Node):
    def __init__(self, topic: str):
        super().__init__("remote_audio")
        self._pub = self.create_publisher(AudioChunk, topic, _LOW_LAT_QOS)
        self.get_logger().info(f"AudioNode ready — topic: {topic}")

    def publish_pcm(self, pcm_data: bytes):
        """Publish PCM data in CHUNK_BYTES segments."""
        offset = 0
        while offset < len(pcm_data):
            chunk = pcm_data[offset:offset + CHUNK_BYTES]
            offset += CHUNK_BYTES
            msg = AudioChunk()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "audio/pcm-16k"
            msg.data = list(chunk)
            self._pub.publish(msg)


class RemoteAudioPlugin:
    PREFIX = "remote_audio"

    def __init__(self, namespace: str, executor):
        self._topic = f"/{namespace}/remote_audio"
        self._node = _AudioNode(self._topic)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "remote_audio",
            "type": "sensor",
            "multiInstance": False,
            "description": "Send an audio file (converted to PCM-16k mono) via remote control.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["send_audio"],
                        "description": "Action to perform",
                    },
                    "audio_base64": {
                        "type": "string",
                        "description": "Base64-encoded audio file (any format supported by ffmpeg)",
                    },
                },
                "required": ["action", "audio_base64"],
            },
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "send_audio":
            audio_b64 = args.get("audio_base64", "")
            if not audio_b64:
                return {"error": "Missing required parameter: audio_base64"}
            try:
                raw = base64.b64decode(audio_b64)
                pcm = self._convert_to_pcm16k(raw)
                self._node.publish_pcm(pcm)
                return {"status": "sent", "bytes": len(pcm), "duration_ms": int(len(pcm) / 32)}
            except Exception as e:
                return {"error": f"Audio conversion failed: {e}"}
        return {"error": f"Unknown action: {action}"}

    def _convert_to_pcm16k(self, raw_bytes: bytes) -> bytes:
        """Convert WAV audio to 16kHz mono 16-bit PCM using stdlib.

        Expects input as WAV format. For other formats, the browser
        should convert to WAV before base64-encoding.
        """
        with wave.open(io.BytesIO(raw_bytes), 'rb') as wf:
            n_channels = wf.getnchannels()
            sampwidth = wf.getsampwidth()
            framerate = wf.getframerate()
            pcm = wf.readframes(wf.getnframes())

        # Convert to mono
        if n_channels > 1:
            pcm = audioop.tomono(pcm, sampwidth, 1, 1)

        # Convert to 16-bit
        if sampwidth != 2:
            pcm = audioop.lin2lin(pcm, sampwidth, 2)

        # Resample to 16kHz
        if framerate != 16000:
            pcm, _ = audioop.ratecv(pcm, 2, 1, framerate, 16000, None)

        return pcm


# ── RemoteMicPlugin ──────────────────────────────────────────────────────────

class _MicNode(Node):
    def __init__(self, topic: str):
        super().__init__("remote_mic")
        self._pub = self.create_publisher(AudioChunk, topic, _LOW_LAT_QOS)
        self.get_logger().info(f"RemoteMicNode ready — topic: {topic}")

    def publish_frame(self, pcm_bytes: bytes):
        """Publish a single PCM frame from WebSocket."""
        offset = 0
        while offset < len(pcm_bytes):
            chunk = pcm_bytes[offset:offset + CHUNK_BYTES]
            offset += CHUNK_BYTES
            msg = AudioChunk()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "audio/pcm-16k"
            msg.data = list(chunk)
            self._pub.publish(msg)


class RemoteMicPlugin:
    PREFIX = "remote_mic"

    def __init__(self, namespace: str, executor, port: int):
        self._topic = f"/{namespace}/remote_mic"
        self._node = _MicNode(self._topic)
        self._port = port
        self._active = False
        self._ws_clients = set()
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "remote_mic",
            "type": "sensor",
            "multiInstance": False,
            "description": "Stream live microphone audio from browser to robot (PCM-16k mono via WebSocket). Use 'connect' to start, 'disconnect' to stop.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["connect", "disconnect", "status"],
                        "description": "Action to perform",
                    },
                },
                "required": ["action"],
            },
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self):
        pass

    def stop(self):
        self._active = False

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            self._active = False
            return {"state": "idle"}
        if action == "connect":
            self._active = True
            return {
                "status": "listening",
                "ws_url": f"ws://localhost:{self._port + 1}/ws/mic",
                "topic": self._topic,
            }
        elif action == "disconnect":
            self._active = False
            return {"status": "disconnected"}
        elif action == "status":
            return {
                "active": self._active,
                "clients": len(self._ws_clients),
                "topic": self._topic,
            }
        return {"error": f"Unknown action: {action}"}

    def register_ws(self, ws):
        self._ws_clients.add(ws)

    def unregister_ws(self, ws):
        self._ws_clients.discard(ws)

    def on_audio_frame(self, pcm_bytes: bytes):
        """Called from WS handler when browser sends binary PCM frame."""
        if not self._active:
            return
        self._node.publish_frame(pcm_bytes)

    @property
    def active(self):
        return self._active
