#!/usr/bin/env python3
"""
drivers/unitree/g1/ext_devices.py — External mic and camera plugins (multiInstance).

Enumerates system audio/video devices, excluding built-in G1 mic (UDP multicast)
and RealSense cameras. Each external device can be started as an independent
tool instance on the canvas.
"""

import glob
import logging
import subprocess
import threading
import time
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from audio_msgs.msg import AudioChunk

log = logging.getLogger(__name__)

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)

JPEG_QUALITY = 80


# ── Device Enumeration ────────────────────────────────────────────────────────

def _enumerate_ext_mics() -> list[dict]:
    """List external USB microphone input devices (excluding RealSense mic)."""
    try:
        import sounddevice as sd
    except ImportError:
        log.warning("[ext_mic] sounddevice not installed, cannot enumerate mic devices")
        return []

    devices = []
    for i, dev in enumerate(sd.query_devices()):
        if dev['max_input_channels'] < 1:
            continue
        name = dev['name'].lower()
        # Exclude RealSense built-in mic
        if 'realsense' in name or 'intel' in name:
            continue
        devices.append({
            "index": i,
            "name": dev['name'],
            "channels": dev['max_input_channels'],
            "sample_rate": int(dev['default_samplerate']),
        })
    return devices


def _enumerate_ext_cameras() -> list[dict]:
    """List external V4L2 video capture devices (excluding RealSense)."""
    devices = []
    for path in sorted(glob.glob('/dev/video*')):
        try:
            info = subprocess.check_output(
                ['v4l2-ctl', '-d', path, '--info'],
                text=True, timeout=2, stderr=subprocess.DEVNULL
            )
        except Exception:
            continue

        # Exclude RealSense (Intel vendor)
        if 'RealSense' in info or 'Intel(R) RealSense' in info:
            continue
        # Only keep Video Capture devices (not metadata nodes)
        if 'Video Capture' not in info:
            continue

        name = "Unknown"
        for line in info.splitlines():
            if 'Card type' in line:
                name = line.split(':', 1)[-1].strip()
                break

        devices.append({"path": path, "name": name})
    return devices


# ── ROS2 Nodes ────────────────────────────────────────────────────────────────

class _ExtMicNode(Node):
    """Captures audio from a system input device and publishes AudioChunk."""

    def __init__(self, device_index: int, device_name: str, namespace: str, instance_id: str):
        node_name = f"ext_mic_{instance_id.replace('-', '_')}"
        super().__init__(node_name)
        self._device_index = device_index
        self._device_name = device_name
        self._instance_id = instance_id
        self._topic = f"/{namespace}/ext_mic/{instance_id}/audio"
        self._pub = self.create_publisher(AudioChunk, self._topic, _LOW_LAT_QOS)
        self._stream = None
        self.state = "idle"

    def start(self) -> dict:
        if self.state == "running":
            return self._status_dict()
        import sounddevice as sd
        self._stream = sd.InputStream(
            device=self._device_index,
            samplerate=16000, channels=1, dtype='int16',
            blocksize=512, callback=self._audio_cb,
        )
        self._stream.start()
        self.state = "running"
        log.info(f"[ext_mic] started device={self._device_name} → {self._topic}")
        return self._status_dict()

    def stop(self) -> dict:
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None
        self.state = "idle"
        return self._status_dict()

    def _audio_cb(self, indata, frames, time_info, status):
        if status:
            log.debug(f"[ext_mic] sounddevice status: {status}")
        msg = AudioChunk()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "audio/pcm-16k"
        msg.data = list(indata.tobytes())
        self._pub.publish(msg)

    def _status_dict(self) -> dict:
        return {
            "state": self.state,
            "device_name": self._device_name,
            "device_index": self._device_index,
            "topic_in": [],
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k", "desc": ""}],
        }


class _ExtCameraNode(Node):
    """Captures video from a V4L2 device and publishes JPEG CompressedImage."""

    def __init__(self, device_path: str, device_name: str, namespace: str, instance_id: str,
                 fps: int = 15, width: int = 1920, height: int = 1080):
        node_name = f"ext_camera_{instance_id.replace('-', '_')}"
        super().__init__(node_name)
        self._device_path = device_path
        self._device_name = device_name
        self._instance_id = instance_id
        self._topic = f"/{namespace}/ext_camera/{instance_id}/rgb"
        self._pub = self.create_publisher(CompressedImage, self._topic, _LOW_LAT_QOS)
        self._fps = fps
        self._width = width
        self._height = height
        self._cap: Optional[cv2.VideoCapture] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.state = "idle"

    def start(self) -> dict:
        if self.state == "running":
            return self._status_dict()
        self._cap = cv2.VideoCapture(self._device_path)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {self._device_path}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS, self._fps)
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self.state = "running"
        log.info(f"[ext_camera] started device={self._device_path} ({self._width}x{self._height}@{self._fps}) → {self._topic}")
        return self._status_dict()

    def stop(self) -> dict:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
            self._thread = None
        if self._cap:
            self._cap.release()
            self._cap = None
        self.state = "idle"
        return self._status_dict()

    def _capture_loop(self):
        interval = 1.0 / self._fps
        while self._running:
            t0 = time.monotonic()
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = jpeg.tobytes()
            self._pub.publish(msg)
            elapsed = time.monotonic() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

    def _status_dict(self) -> dict:
        return {
            "state": self.state,
            "device_path": self._device_path,
            "device_name": self._device_name,
            "topic_in": [],
            "topic_out": [{"topic": self._topic, "format": "image/jpeg", "desc": ""}],
        }


# ── Plugins ───────────────────────────────────────────────────────────────────

TOOLS_EXT_MIC = [
    {
        "name": "ext_mic",
        "type": "sensor",
        "multiInstance": True,
        "description": "External USB microphone — captures audio and publishes PCM-16k",
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["start", "stop", "info"],
                    "description": "Action to perform",
                },
            },
            "required": ["action"],
        },
        "configSchema": {
            "type": "object",
            "properties": {
                "device_index": {"type": "integer", "description": "音频设备索引", "scope": "instance"},
                "device_name":  {"type": "string", "description": "设备名称", "scope": "instance"},
            },
        },
        "topic_in": [],
        "topic_out": [{"format": "audio/pcm-16k", "desc": "external mic audio"}],
    }
]

TOOLS_EXT_CAMERA = [
    {
        "name": "ext_camera",
        "type": "sensor",
        "multiInstance": True,
        "description": "External camera (action cam / USB cam) — captures JPEG video",
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["start", "stop", "info"],
                    "description": "Action to perform",
                },
            },
            "required": ["action"],
        },
        "configSchema": {
            "type": "object",
            "properties": {
                "device_path": {"type": "string", "description": "设备路径 (如 /dev/video2)", "scope": "instance"},
                "device_name": {"type": "string", "description": "设备名称", "scope": "instance"},
                "fps":         {"type": "integer", "description": "帧率", "default": 15, "scope": "instance"},
                "resolution":  {"type": "string", "description": "分辨率 (如 1920x1080)", "default": "1920x1080", "scope": "instance"},
            },
        },
        "topic_in": [],
        "topic_out": [{"format": "image/jpeg", "desc": "external camera JPEG stream"}],
    }
]


class ExtMicPlugin:
    PREFIX = "ext_mic"

    def __init__(self, plugin_cfg: dict, namespace: str, executor):
        self._namespace = namespace
        self._executor = executor
        self._nodes: dict[str, _ExtMicNode] = {}
        self._available_devices = _enumerate_ext_mics()
        log.info(f"[ext_mic] found {len(self._available_devices)} external mic device(s)")
        for d in self._available_devices:
            log.info(f"  [{d['index']}] {d['name']}")

    def get_tools(self) -> list:
        # Build dynamic configSchema with enumerated devices
        device_options = [{"const": d["index"], "title": d["name"]} for d in self._available_devices]
        tool = dict(TOOLS_EXT_MIC[0])
        tool["configSchema"] = {
            "type": "object",
            "properties": {
                "device_index": {
                    "type": "integer",
                    "description": "音频设备",
                    "scope": "instance",
                    "oneOf": device_options if device_options else [{"const": 0, "title": "无可用设备"}],
                },
                "device_name": {"type": "string", "description": "设备名称", "scope": "instance"},
            },
        }
        return [tool]

    def start(self) -> None:
        pass  # Don't auto-start — wait for canvas to start instances

    def stop(self) -> None:
        for key in list(self._nodes.keys()):
            self._nodes[key].stop()
            self._executor.remove_node(self._nodes[key])
            del self._nodes[key]

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get("instance_id", "")

        if action == "info":
            if instance_id and instance_id in self._nodes:
                return self._nodes[instance_id]._status_dict()
            # Return available devices list
            return {
                "state": "idle",
                "available_devices": self._available_devices,
                "active_instances": list(self._nodes.keys()),
                "topic_in": [],
                "topic_out": [{"topic": "", "format": "audio/pcm-16k", "desc": ""}],
            }

        elif action == "start":
            if not instance_id:
                raise ValueError("instance_id is required for multiInstance tool")
            device_index = args.get("device_index")
            device_name = args.get("device_name", "")
            if device_index is None:
                # Try to pick first available device
                if self._available_devices:
                    device_index = self._available_devices[0]["index"]
                    device_name = self._available_devices[0]["name"]
                else:
                    raise ValueError("No external mic device available")
            if instance_id not in self._nodes:
                node = _ExtMicNode(device_index, device_name, self._namespace, instance_id)
                self._executor.add_node(node)
                self._nodes[instance_id] = node
            return self._nodes[instance_id].start()

        elif action == "stop":
            if instance_id and instance_id in self._nodes:
                result = self._nodes[instance_id].stop()
                self._executor.remove_node(self._nodes[instance_id])
                del self._nodes[instance_id]
                return result
            elif not instance_id:
                # Stop all
                for key in list(self._nodes.keys()):
                    self._nodes[key].stop()
                    self._executor.remove_node(self._nodes[key])
                    del self._nodes[key]
                return {"state": "idle"}
            return {"state": "idle"}

        return None


class ExtCameraPlugin:
    PREFIX = "ext_camera"

    def __init__(self, plugin_cfg: dict, namespace: str, executor):
        self._namespace = namespace
        self._executor = executor
        self._nodes: dict[str, _ExtCameraNode] = {}
        self._available_devices = _enumerate_ext_cameras()
        log.info(f"[ext_camera] found {len(self._available_devices)} external camera device(s)")
        for d in self._available_devices:
            log.info(f"  {d['path']} — {d['name']}")

    def get_tools(self) -> list:
        # Build dynamic configSchema with enumerated devices
        device_options = [{"const": d["path"], "title": f"{d['name']} ({d['path']})"} for d in self._available_devices]
        tool = dict(TOOLS_EXT_CAMERA[0])
        tool["configSchema"] = {
            "type": "object",
            "properties": {
                "device_path": {
                    "type": "string",
                    "description": "摄像头设备",
                    "scope": "instance",
                    "oneOf": device_options if device_options else [{"const": "", "title": "无可用设备"}],
                },
                "device_name": {"type": "string", "description": "设备名称", "scope": "instance"},
                "fps": {"type": "integer", "description": "帧率", "default": 15, "scope": "instance"},
                "resolution": {"type": "string", "description": "分辨率 (如 1920x1080)", "default": "1920x1080", "scope": "instance"},
            },
        }
        return [tool]

    def start(self) -> None:
        pass  # Don't auto-start

    def stop(self) -> None:
        for key in list(self._nodes.keys()):
            self._nodes[key].stop()
            self._executor.remove_node(self._nodes[key])
            del self._nodes[key]

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get("instance_id", "")

        if action == "info":
            if instance_id and instance_id in self._nodes:
                return self._nodes[instance_id]._status_dict()
            return {
                "state": "idle",
                "available_devices": self._available_devices,
                "active_instances": list(self._nodes.keys()),
                "topic_in": [],
                "topic_out": [{"topic": "", "format": "image/jpeg", "desc": ""}],
            }

        elif action == "start":
            if not instance_id:
                raise ValueError("instance_id is required for multiInstance tool")
            device_path = args.get("device_path")
            device_name = args.get("device_name", "")
            if not device_path:
                if self._available_devices:
                    device_path = self._available_devices[0]["path"]
                    device_name = self._available_devices[0]["name"]
                else:
                    raise ValueError("No external camera device available")
            # Parse resolution
            resolution = args.get("resolution", "1920x1080")
            try:
                w, h = resolution.lower().split('x')
                width, height = int(w), int(h)
            except Exception:
                width, height = 1920, 1080
            fps = int(args.get("fps", 15))

            if instance_id not in self._nodes:
                node = _ExtCameraNode(device_path, device_name, self._namespace, instance_id,
                                      fps=fps, width=width, height=height)
                self._executor.add_node(node)
                self._nodes[instance_id] = node
            return self._nodes[instance_id].start()

        elif action == "stop":
            if instance_id and instance_id in self._nodes:
                result = self._nodes[instance_id].stop()
                self._executor.remove_node(self._nodes[instance_id])
                del self._nodes[instance_id]
                return result
            elif not instance_id:
                for key in list(self._nodes.keys()):
                    self._nodes[key].stop()
                    self._executor.remove_node(self._nodes[key])
                    del self._nodes[key]
                return {"state": "idle"}
            return {"state": "idle"}

        return None
