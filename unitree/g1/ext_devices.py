#!/usr/bin/env python3
"""
drivers/unitree/g1/ext_devices.py — External mic and camera plugins (multiInstance).

Enumerates system audio/video devices, excluding built-in G1 mic (UDP multicast)
and RealSense cameras. Each external device can be started as an independent
tool instance on the canvas.
"""

import glob
import logging
import re
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

# Preferred pixel format order when auto-selecting
_FOURCC_PRIORITY = ['MJPG', 'H264', 'YUYV']


# ── Device Enumeration ────────────────────────────────────────────────────────

def _enumerate_ext_mics() -> list[dict]:
    """List external USB microphone input devices via arecord -l, fallback to sounddevice."""
    devices = []

    # Primary: parse arecord -l (works reliably in Docker with /dev/snd mapped)
    try:
        output = subprocess.check_output(['arecord', '-l'], text=True, timeout=5, stderr=subprocess.DEVNULL)
        for line in output.splitlines():
            if not line.startswith('card '):
                continue
            # Format: "card N: NAME [DESC], device M: ..."
            name_lower = line.lower()
            if 'realsense' in name_lower or 'intel' in name_lower:
                continue
            # Skip NVIDIA APE internal devices
            if 'ape' in name_lower or 'tegra' in name_lower:
                continue
            try:
                card_part = line.split(':')[0]  # "card N"
                card_num = int(card_part.split()[1])
                device_part = line.split('device')[1].split(':')[0].strip()
                device_num = int(device_part)
                # Extract description between [ ]
                desc = line.split('[')[1].split(']')[0] if '[' in line else f"card{card_num}"
                alsa_id = f"hw:{card_num},{device_num}"
                devices.append({
                    "index": card_num,
                    "device_num": device_num,
                    "alsa_id": alsa_id,
                    "name": desc,
                })
            except (IndexError, ValueError):
                continue
    except Exception as e:
        log.debug(f"[ext_mic] arecord -l failed: {e}")

    if devices:
        return devices

    # Fallback: sounddevice
    try:
        import sounddevice as sd
    except ImportError:
        log.warning("[ext_mic] sounddevice not installed and arecord unavailable")
        return []

    for i, dev in enumerate(sd.query_devices()):
        if dev['max_input_channels'] < 1:
            continue
        name = dev['name'].lower()
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

        # Probe supported pixel formats and resolutions via v4l2-ctl --list-formats-ext
        formats: list[str] = []
        resolutions: list[str] = []
        try:
            fmt_out = subprocess.check_output(
                ['v4l2-ctl', '-d', path, '--list-formats-ext'],
                text=True, timeout=2, stderr=subprocess.DEVNULL
            )
            formats = re.findall(r"'\s*([A-Z0-9]{4})\s*'", fmt_out)
            for line in fmt_out.splitlines():
                m = re.search(r'Size: Discrete (\d+x\d+)', line)
                if m and m.group(1) not in resolutions:
                    resolutions.append(m.group(1))
        except Exception:
            pass

        devices.append({"path": path, "name": name, "formats": formats, "resolutions": resolutions})
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
        self._topic = f"/{namespace}/ext_mic/{instance_id.replace('-', '_')}/audio"
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
                 fps: int = 15, width: int = 1920, height: int = 1080,
                 pixel_format: str = "auto", available_formats: Optional[list] = None):
        node_name = f"ext_camera_{instance_id.replace('-', '_')}"
        super().__init__(node_name)
        self._device_path = device_path
        self._device_name = device_name
        self._instance_id = instance_id
        self._topic = f"/{namespace}/ext_camera/{instance_id.replace('-', '_')}/rgb"
        self._pub = self.create_publisher(CompressedImage, self._topic, _LOW_LAT_QOS)
        self._fps = fps
        self._width = width
        self._height = height
        self._pixel_format = pixel_format
        self._available_formats: list = available_formats or []
        self._cap: Optional[cv2.VideoCapture] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.state = "idle"

    def _resolve_fourcc(self) -> Optional[int]:
        if self._pixel_format == "auto":
            for f in _FOURCC_PRIORITY:
                if f in self._available_formats:
                    return cv2.VideoWriter_fourcc(*f)
            return None  # let OpenCV negotiate
        try:
            return cv2.VideoWriter_fourcc(*self._pixel_format)
        except Exception:
            return None

    def start(self) -> dict:
        if self.state == "running":
            return self._status_dict()
        # Check if device is already held by another process
        busy_check = subprocess.run(['fuser', self._device_path], capture_output=True, text=True)
        if busy_check.stdout.strip():
            raise RuntimeError(f"Device {self._device_path} is busy (PID: {busy_check.stdout.strip()}). Stop the old instance first.")
        self._cap = cv2.VideoCapture(self._device_path)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {self._device_path}")
        fourcc = self._resolve_fourcc()
        if fourcc is not None:
            self._cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        actual = int(self._cap.get(cv2.CAP_PROP_FOURCC))
        actual_str = "".join([chr((actual >> 8 * i) & 0xFF) for i in range(4)])
        log.info(f"[ext_camera] FOURCC requested={self._pixel_format} actual={actual_str}")
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
        device_options = [{"const": d.get("alsa_id", str(d["index"])), "title": d["name"]} for d in self._available_devices]
        tool = dict(TOOLS_EXT_MIC[0])
        tool["configSchema"] = {
            "type": "object",
            "properties": {
                "device_index": {
                    "type": "string",
                    "description": "音频设备",
                    "scope": "instance",
                    "oneOf": device_options if device_options else [{"const": "", "title": "无可用设备"}],
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
            # Infer topic from namespace + instance_id even before start
            inferred_topic = f"/{self._namespace}/ext_mic/{instance_id.replace('-', '_')}/audio" if instance_id else ""
            return {
                "state": "idle",
                "available_devices": self._available_devices,
                "active_instances": list(self._nodes.keys()),
                "topic_in": [],
                "topic_out": [{"topic": inferred_topic, "format": "audio/pcm-16k", "desc": "external mic audio"}],
            }

        elif action == "start":
            if not instance_id:
                raise ValueError("instance_id is required for multiInstance tool")
            device_id = args.get("device_index")  # alsa_id string like "hw:0,0" or integer index
            device_name = args.get("device_name", "")
            if not device_id:
                # Try to pick first available device
                if self._available_devices:
                    device_id = self._available_devices[0].get("alsa_id", self._available_devices[0]["index"])
                    device_name = self._available_devices[0]["name"]
                else:
                    raise ValueError("No external mic device available")
            # Try to convert to int for sounddevice numeric index, keep string for alsa_id
            try:
                device_id = int(device_id)
            except (ValueError, TypeError):
                pass  # keep as string (alsa_id like "hw:0,0")
            if instance_id not in self._nodes:
                node = _ExtMicNode(device_id, device_name, self._namespace, instance_id)
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
        # Collect all unique formats across devices for pixel_format selector
        all_formats: list[str] = []
        for d in self._available_devices:
            for f in d.get("formats", []):
                if f not in all_formats:
                    all_formats.append(f)
        format_options = [{"const": "auto", "title": "自动"}] + [{"const": f, "title": f} for f in all_formats]
        # Collect all unique resolutions across devices
        all_resolutions: list[str] = []
        for d in self._available_devices:
            for r in d.get("resolutions", []):
                if r not in all_resolutions:
                    all_resolutions.append(r)
        resolution_options = [{"const": r, "title": r} for r in all_resolutions] or [{"const": "1920x1080", "title": "1920x1080"}]
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
                "resolution": {
                    "type": "string",
                    "description": "分辨率",
                    "default": "1920x1080",
                    "scope": "instance",
                    "oneOf": resolution_options,
                },
                "pixel_format": {
                    "type": "string",
                    "description": "像素格式",
                    "default": "auto",
                    "scope": "instance",
                    "oneOf": format_options,
                },
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
            # Infer topic from namespace + instance_id even before start
            inferred_topic = f"/{self._namespace}/ext_camera/{instance_id.replace('-', '_')}/rgb" if instance_id else ""
            return {
                "state": "idle",
                "available_devices": self._available_devices,
                "active_instances": list(self._nodes.keys()),
                "topic_in": [],
                "topic_out": [{"topic": inferred_topic, "format": "image/jpeg", "desc": "external camera JPEG"}],
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
            # Resolve available formats for the selected device
            available_formats: list[str] = []
            for d in self._available_devices:
                if d["path"] == device_path:
                    available_formats = d.get("formats", [])
                    break
            # Parse resolution
            resolution = args.get("resolution", "1920x1080")
            try:
                w, h = resolution.lower().split('x')
                width, height = int(w), int(h)
            except Exception:
                width, height = 1920, 1080
            fps = int(args.get("fps", 15))
            pixel_format = args.get("pixel_format", "auto")

            if instance_id not in self._nodes:
                node = _ExtCameraNode(device_path, device_name, self._namespace, instance_id,
                                      fps=fps, width=width, height=height,
                                      pixel_format=pixel_format, available_formats=available_formats)
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
