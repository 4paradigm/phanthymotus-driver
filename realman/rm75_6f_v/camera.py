#!/usr/bin/env python3
"""External RealSense camera plugin for the RealMan RM75 upper-computer Driver.

RGB capture uses the camera's V4L2 color node. Depth and infrared capture use
the shared pyrealsense2 session in realsense.py, bound to the same physical USB
device rather than to an unstable /dev/video number.
"""

import glob
import logging
import os
from pathlib import Path
import re
import subprocess
import threading
from typing import Any, Optional

log = logging.getLogger(__name__)

def _realsense_usb_path(device_path: str) -> str:
    """Read the selected V4L2 node's physical USB identity, not a camera index."""
    device = Path('/sys/class/video4linux') / Path(device_path).name / 'device'
    try:
        for parent in device.resolve(strict=True).parents:
            if (parent / 'idVendor').is_file():
                return str(parent)
    except OSError:
        # USB removal can race both symlink resolution and ancestor inspection.
        # A failed node must not abort discovery of other connected cameras.
        return ''
    return ''


def _enumerate_ext_cameras() -> list[dict]:
    """List V4L2 cameras, including only the color interfaces of RealSense."""
    devices = []
    for path in sorted(glob.glob('/dev/video*')):
        try:
            info = subprocess.check_output(
                ['v4l2-ctl', '-d', path, '--info'],
                text=True, timeout=2, stderr=subprocess.DEVNULL,
                env={**os.environ, 'LC_ALL': 'C'},
            )
        except Exception:
            continue

        is_realsense = 'realsense' in info.lower()
        # Capabilities describes the whole device, including its sibling nodes.
        # Device Caps describes this node; metadata siblings are not cameras.
        caps = info.split('Device Caps', 1)[-1].split('Media Driver Info', 1)[0]
        if 'Video Capture' not in caps:
            continue

        name = "Unknown"
        for line in info.splitlines():
            if 'Card type' in line:
                name = line.split(':', 1)[-1].strip()
                break

        # Probe supported pixel formats and resolutions via v4l2-ctl --list-formats-ext.
        # If the probe succeeds but returns no formats, the node is not a real capture device
        # (e.g. secondary metadata interface) — skip it.
        formats: list[str] = []
        resolutions: list[str] = []
        fmt_probe_ok = False
        try:
            fmt_out = subprocess.check_output(
                ['v4l2-ctl', '-d', path, '--list-formats-ext'],
                text=True, timeout=2, stderr=subprocess.DEVNULL,
                env={**os.environ, 'LC_ALL': 'C'},
            )
            fmt_probe_ok = True
            formats = list(dict.fromkeys(f.rstrip() for f in re.findall(
                r"\[\d+\]:\s*'([^']{4})'", fmt_out)))
            for line in fmt_out.splitlines():
                m = re.search(r'Size: Discrete (\d+x\d+)', line)
                if m and m.group(1) not in resolutions:
                    resolutions.append(m.group(1))
        except Exception:
            pass

        # Probe succeeded but no formats → secondary/metadata node, not usable for capture
        if fmt_probe_ok and not formats:
            continue

        # The RealSense stereo module exposes depth/IR formats (including UYVY
        # on IR nodes). Accept verified color formats, never an unprobed node.
        if is_realsense:
            if not set(formats).intersection({'YUYV', 'MJPG', 'RGB3', 'BGR3'}):
                continue
            if set(formats).intersection({'Z16', 'GREY', 'Y8I', 'Y12I', 'Y16'}):
                continue
            name += ' (RealSense)'

        usb_path = _realsense_usb_path(path) if is_realsense else ''
        if is_realsense and not usb_path:
            continue
        devices.append({"path": path, "name": name, "formats": formats, "resolutions": resolutions,
                        "realsense": is_realsense,
                        "usb_path": usb_path})
    return devices

class _ExtCameraNode:
    """Manages a subprocess that captures video from a V4L2 device and publishes JPEG."""

    def __init__(self, device_path: str, device_name: str, namespace: str, instance_id: str,
                 fps: int = 15, width: int = 1920, height: int = 1080,
                 pixel_format: str = "auto", available_formats: Optional[list] = None):
        self._device_path = device_path
        self._device_name = device_name
        self._instance_id = instance_id
        self._namespace = namespace
        self._topic = f"/{namespace}/ext_camera/{instance_id.replace('-', '_')}/rgb"
        self._fps = fps
        self._width = width
        self._height = height
        self._pixel_format = pixel_format
        self._available_formats: list = available_formats or []
        self._proc: Optional[Any] = None
        self.state = "idle"

    def start(self) -> dict:
        if self.state == "running" and self._proc is not None and self._proc.is_alive():
            return self._status_dict()
        import multiprocessing as mp
        ctx = mp.get_context("spawn")
        self._proc = ctx.Process(
            target=_run_ext_camera_process,
            args=(self._device_path, self._namespace, self._instance_id,
                  self._fps, self._width, self._height,
                  self._pixel_format, self._available_formats),
            name=f"ext_camera_{self._instance_id}",
            daemon=True,
        )
        self._proc.start()
        self.state = "running"
        print(f"[ext_camera] subprocess started → pid={self._proc.pid} device={self._device_path} "
              f"({self._width}x{self._height}@{self._fps}) → {self._topic}", flush=True)
        return self._status_dict()

    def stop(self) -> dict:
        if self._proc is not None and self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=3.0)
            if self._proc.is_alive():
                self._proc.kill()
                self._proc.join(timeout=2.0)
        self._proc = None
        self.state = "idle"
        return self._status_dict()

    def _status_dict(self) -> dict:
        state = self.state
        if state == "running" and (self._proc is None or not self._proc.is_alive()):
            state = "error"
        return {
            "state": state,
            "channel": "rgb",
            "device_path": self._device_path,
            "device_name": self._device_name,
            "topic_in": [],
            "topic_out": [{"topic": self._topic, "format": "image/jpeg", "desc": ""}],
        }


def _run_ext_camera_process(device_path: str, namespace: str, instance_id: str,
                            fps: int, width: int, height: int,
                            pixel_format: str, available_formats: list) -> None:
    """Ext camera subprocess entry — independent GIL for full throughput."""
    from common import logsafe
    logsafe.install(check_fd=False)
    import cv2
    import rclpy
    from rclpy.node import Node as _Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage as _CompressedImage

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )

    _FOURCC_PRIO = ['MJPG', 'H264', 'YUYV']
    _JPEG_Q = 80

    rclpy.init()
    node_name = f"ext_camera_{instance_id.replace('-', '_')}"
    node = _Node(node_name)
    topic = f"/{namespace}/ext_camera/{instance_id.replace('-', '_')}/rgb"
    pub = node.create_publisher(_CompressedImage, topic, _QOS)

    cap = cv2.VideoCapture(device_path)
    if not cap.isOpened():
        node.get_logger().error(f"[ext_camera] Cannot open device: {device_path}")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Set FOURCC
    fourcc = None
    if pixel_format == "auto":
        for f in _FOURCC_PRIO:
            if f in available_formats:
                fourcc = cv2.VideoWriter_fourcc(*f)
                break
    else:
        try:
            fourcc = cv2.VideoWriter_fourcc(*pixel_format)
        except Exception:
            pass
    if fourcc is not None:
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    actual = int(cap.get(cv2.CAP_PROP_FOURCC))
    actual_str = "".join([chr((actual >> 8 * i) & 0xFF) for i in range(4)])
    mjpg_passthrough = (actual_str == "MJPG")
    if mjpg_passthrough:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    node.get_logger().info(
        f"[ext_camera] capture started — {device_path} {width}x{height}@{fps} "
        f"fourcc={actual_str} passthrough={mjpg_passthrough}"
    )

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                import time
                time.sleep(0.1)
                continue
            if mjpg_passthrough:
                jpeg_bytes = frame.tobytes()
            else:
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, _JPEG_Q])
                jpeg_bytes = jpeg.tobytes()
            msg = _CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = jpeg_bytes
            pub.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()


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
                "device_index": {
                    "type": "string",
                    "description": "音频设备",
                    "scope": "instance",
                },
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
        "inputSchema": {"type": "object", "properties": {}},
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

class _StereoCameraNode:
    def __init__(self, session, instance_id, channel):
        self.session, self.instance_id, self.channel = session, instance_id, channel

    def start(self):
        return self.session.start(self.instance_id, self.channel)

    def stop(self):
        self.session.stop(self.instance_id)
        return self._status_dict()

    def _status_dict(self):
        return self.session.info(self.instance_id, self.channel)


class ExtCameraPlugin:
    PREFIX = "ext_camera"

    def __init__(self, plugin_cfg: dict, namespace: str, executor):
        self._namespace = namespace
        self._executor = executor
        self._lock = threading.RLock()
        self._nodes = {}
        self._instance_configs = {}
        self._sessions = {}
        self._available_devices = _enumerate_ext_cameras()

    def get_tools(self) -> list:
        devices = [{"const": d["path"], "title": f"{d['name']} ({d['path']})"}
                   for d in self._available_devices]
        resolutions = list(dict.fromkeys(r for d in self._available_devices
                                         for r in d.get('resolutions', []))) or ['1280x720']
        formats = ['auto'] + list(dict.fromkeys(f for d in self._available_devices
                                               for f in d.get('formats', [])))
        tool = dict(TOOLS_EXT_CAMERA[0])
        tool['description'] = ('External camera — channel selects RGB, RealSense depth, or left infrared. '
                               'Infrared is light intensity, not temperature.')
        tool['configSchema'] = {'type': 'object', 'properties': {
            'device_path': {'type': 'string', 'description': '摄像头设备', 'scope': 'instance',
                            'oneOf': devices or [{'const': '', 'title': '无可用设备'}]},
            'channel': {'type': 'string', 'title': 'channel', 'scope': 'instance',
                        'enum': ['rgb', 'depth', 'infrared'], 'default': 'rgb',
                        'description': 'rgb 彩色；depth 深度；infrared 左近红外（非热成像）'},
            'fps': {'type': 'integer', 'scope': 'instance', 'default': 15, 'minimum': 1,
                    'maximum': 60, 'description': 'RGB 帧率；深度/红外按 USB 连接自动选择 6 或 15fps'},
            'resolution': {'type': 'string', 'scope': 'instance', 'enum': resolutions,
                           'default': '1280x720' if '1280x720' in resolutions else resolutions[0],
                           'description': 'RGB 分辨率；深度/红外固定 640x480'},
            'pixel_format': {'type': 'string', 'scope': 'instance', 'enum': formats,
                             'default': 'auto', 'description': 'RGB 像素格式；深度 Z16 / 红外 Y8 自动选择'},
        }}
        return [tool]

    def start(self):
        pass

    def stop(self):
        with self._lock:
            for node in self._nodes.values():
                node.stop()
            self._nodes.clear()

    def _topic(self, instance_id, channel):
        topic = f"/{self._namespace}/ext_camera/{instance_id.replace('-', '_')}/{channel}" if instance_id else ''
        return [{'topic': topic, 'format': 'image/depth-zlib' if channel == 'depth' else 'image/jpeg'}]

    def _info(self, instance_id):
        cfg = self._instance_configs.get(instance_id, {})
        channel = cfg.get('channel', 'rgb')
        info = (self._nodes[instance_id]._status_dict() if instance_id in self._nodes else {'state': 'idle'})
        return {**info, 'channel': channel, 'device_path': cfg.get('device_path', ''),
                'topic_in': [], 'topic_out': self._topic(instance_id, channel),
                'available_devices': self._available_devices,
                'active_instances': list(self._nodes)}

    def _validate(self, cfg):
        cfg = dict(cfg)
        channel = cfg.setdefault('channel', 'rgb')
        if channel not in ('rgb', 'depth', 'infrared'):
            raise ValueError('channel must be rgb, depth or infrared')
        self._available_devices = _enumerate_ext_cameras()
        path = cfg.get('device_path') or (self._available_devices[0]['path'] if self._available_devices else '')
        device = next((d for d in self._available_devices if d['path'] == path), None)
        if device is None:
            raise ValueError('Selected external camera is unavailable')
        cfg['device_path'] = path
        if channel != 'rgb':
            if not device.get('realsense') or not device.get('usb_path'):
                raise ValueError('depth/infrared requires a RealSense camera with a resolvable physical USB path')
        else:
            fps = cfg.setdefault('fps', 15)
            if isinstance(fps, bool) or not isinstance(fps, int) or not 1 <= fps <= 60:
                raise ValueError('RGB fps must be an integer between 1 and 60')
            available_resolutions = device.get('resolutions', [])
            default_resolution = ('1280x720' if '1280x720' in available_resolutions
                                  else (available_resolutions[0] if available_resolutions else '1280x720'))
            resolution = cfg.setdefault('resolution', default_resolution)
            if not isinstance(resolution, str) or not re.fullmatch(r'[1-9][0-9]{1,4}x[1-9][0-9]{1,4}', resolution):
                raise ValueError('RGB resolution must be WIDTHxHEIGHT')
            if device.get('resolutions') and resolution not in device['resolutions']:
                raise ValueError('RGB resolution is not advertised by the selected camera')
            fmt = cfg.setdefault('pixel_format', 'auto')
            if fmt != 'auto' and fmt not in device.get('formats', []):
                raise ValueError('RGB pixel_format is not advertised by the selected camera')
        return cfg, device

    def _make_node(self, instance_id, cfg, device):
        channel = cfg['channel']
        if channel == 'rgb':
            for other, node in self._nodes.items():
                other_cfg = self._instance_configs[other]
                if other != instance_id and other_cfg['channel'] == 'rgb' and other_cfg['device_path'] == cfg['device_path']:
                    raise ValueError('RGB device is already in use by another instance')
            width, height = map(int, cfg['resolution'].split('x'))
            return _ExtCameraNode(device['path'], device['name'], self._namespace, instance_id,
                                  fps=cfg['fps'], width=width, height=height,
                                  pixel_format=cfg['pixel_format'], available_formats=device['formats'])
        from realsense import RealSenseSession
        usb_path = device['usb_path']
        if usb_path not in self._sessions:
            self._sessions[usb_path] = RealSenseSession(self._namespace, usb_path)
        return _StereoCameraNode(self._sessions[usb_path], instance_id, channel)

    def dispatch(self, action, args):
        instance_id = args.get('instance_id', '')
        with self._lock:
            if action == 'info':
                return self._info(instance_id)
            if action == 'stop':
                if instance_id:
                    node = self._nodes.pop(instance_id, None)
                    if node is not None:
                        node.stop()
                else:
                    self.stop()
                return self._info(instance_id)
            if action not in ('config', 'start'):
                return None
            if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]{0,127}', instance_id):
                raise ValueError('A valid instance_id is required')
            if any(key != instance_id and key.replace('-', '_') == instance_id.replace('-', '_')
                   for key in self._instance_configs):
                raise ValueError('instance_id collides with an existing ROS topic')
            supplied = {k: v for k, v in args.items() if k in
                        ('channel', 'device_path', 'device_name', 'fps', 'resolution', 'pixel_format')}
            previous = self._instance_configs.get(instance_id, {})
            cfg, device = self._validate({**previous, **supplied})
            changed = cfg != previous
            node = self._nodes.get(instance_id)
            if node is not None and changed:
                # Validate the replacement before releasing the working channel.
                replacement = self._make_node(instance_id, cfg, device)
                node.stop()
                self._nodes[instance_id] = replacement
                self._instance_configs[instance_id] = cfg
                replacement.start()
            else:
                self._instance_configs[instance_id] = cfg
                if action == 'start':
                    if node is None:
                        node = self._make_node(instance_id, cfg, device)
                        self._nodes[instance_id] = node
                    node.start()
            result = self._info(instance_id)
            if action == 'start' and result['state'] in ('starting', 'running'):
                # Lifecycle activation is separate from the first captured frame.
                # Preserve readiness/freshness; do not hide an actual start error.
                result['readiness'] = result['state']
                result['state'] = 'running'
            return result
