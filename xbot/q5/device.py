"""Verified sensor and audio cards for the RobotEra Q5 bundle.

Direct base, arm, head, and hand cards live in ``direct_control.py``. This
module contains the verified state, battery, audio, and D455 camera cards.
"""

from __future__ import annotations

import json
import re
import shlex
import subprocess
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import AudioPlay
from xbot_common_interfaces.srv import SetVolume

# main.py resolves all card classes through this module. Keep the direct
# control cards here as explicit exports while their implementation remains
# consolidated in direct_control.py.
from direct_control import (
    ArmControlPlugin,
    BaseDrivePlugin,
    HandControlPlugin,
    HandGesturePlugin,
    HeadControlPlugin,
)


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_REMOTE_AUDIO_LOCK = threading.Lock()
_REMOTE_AUDIO_READY = False


def _q5_ssh_args(command: str):
    return [
        "sshpass", "-p", "developer", "ssh", "-p", "2222",
        "-o", "PreferredAuthentications=password", "-o", "PubkeyAuthentication=no",
        "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no",
        "-o", "UserKnownHostsFile=/dev/null", "developer@192.168.8.100",
        f"bash -lc {shlex.quote(command)}",
    ]


def _q5_remote_command(command: str, timeout: float = 20.0, stdin=None):
    """Run a noninteractive command in Q5's documented developer container."""
    return subprocess.run(_q5_ssh_args(command), input=stdin, capture_output=True, timeout=timeout)


def _ensure_remote_audio_tools():
    """Install the minimal ALSA tools missing from a stock Q5 developer container."""
    global _REMOTE_AUDIO_READY
    with _REMOTE_AUDIO_LOCK:
        if _REMOTE_AUDIO_READY:
            return
        command = (
            "command -v arecord >/dev/null && command -v aplay >/dev/null || "
            "(echo developer | sudo -S apt-get -o Acquire::Retries=3 update && "
            "echo developer | sudo -S apt-get install -y --no-install-recommends alsa-utils); "
            "arecord -l; aplay -l"
        )
        result = _q5_remote_command(command, timeout=180.0)
        if result.returncode:
            detail = (result.stderr or result.stdout).decode(errors="replace").strip()
            raise RuntimeError(f"Q5 remote audio setup failed: {detail}")
        _REMOTE_AUDIO_READY = True
        print("[Q5Audio] remote ALSA devices:\n" + result.stdout.decode(errors="replace"), flush=True)


class MicPlugin:
    """Q5 developer-container microphone as a 16 kHz PCM stream."""

    def __init__(self, plugin_config, namespace, executor, client):
        del executor
        self._client = client
        self._topic = f"/{namespace}/q5/mic/audio"
        self._device = str(plugin_config.get("device", "default"))
        self._rate = int(plugin_config.get("sample_rate_hz", 16000))
        self._channels = int(plugin_config.get("channels", 1))
        self._process = None
        self._thread = None
        self._running = False
        if self._rate != 16000 or self._channels != 1:
            raise ValueError("Q5 mic only supports the shared 16 kHz mono PCM contract")

    def get_tool(self):
        return {
            "name": "mic", "type": "sensor", "multiInstance": False,
            "description": "Q5 microphone, live PCM 16 kHz/16-bit/mono for ASR.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self):
        if self._running:
            return
        _ensure_remote_audio_tools()
        command = (
            "exec arecord -D " + shlex.quote(self._device) +
            f" -f S16_LE -r {self._rate} -c {self._channels} -t raw"
        )
        self._process = subprocess.Popen(
            _q5_ssh_args(command), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._running = True
        self._thread = threading.Thread(target=self._pump, daemon=True, name="q5_mic_stream")
        self._thread.start()
        print(f"[MicPlugin] capturing Q5 ALSA {self._device} -> {self._topic}", flush=True)

    def _pump(self):
        # 100 ms frames are the same size emitted by perception TTS.
        while self._running and self._process and self._process.stdout:
            chunk = self._process.stdout.read(3200)
            if not chunk:
                break
            sender = getattr(self._client, "publish_audio", None)
            if callable(sender):
                sender(chunk)
        if self._running:
            print("[MicPlugin] remote capture stream ended", flush=True)

    def stop(self):
        self._running = False
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None

    def dispatch(self, action, args):
        del args
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
        return None


class SpeakerPlugin:
    """Play perception's PCM AudioChunk stream on the Q5 developer-container ALSA output."""

    def __init__(self, plugin_config, namespace, executor, client):
        del namespace, executor
        self._client = client
        self._topic = str(plugin_config.get("input_topic", "/perception/tts"))
        self._device = str(plugin_config.get("device", "default"))
        self._rate = int(plugin_config.get("sample_rate_hz", 16000))
        self._channels = int(plugin_config.get("channels", 1))
        self._process = None
        self._thread = None
        self._running = False
        if self._rate != 16000 or self._channels != 1:
            raise ValueError("Q5 speaker only supports the shared 16 kHz mono PCM contract")

    def get_tool(self):
        return {
            "name": "speaker", "type": "actuator", "multiInstance": False,
            "description": "Q5 speaker. Connect a perception TTS audio/pcm-16k output to play live speech.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
                "input_topic": {"type": "string", "description": "PCM 16 kHz AudioChunk topic"},
            }, "required": ["action"], "additionalProperties": False},
            "topic_in": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self, input_topic=None):
        requested = str(input_topic or self._topic)
        if self._running and requested == self._topic:
            return
        self.stop()
        _ensure_remote_audio_tools()
        self._topic = requested
        command = (
            "exec aplay -D " + shlex.quote(self._device) +
            f" -f S16_LE -r {self._rate} -c {self._channels} -t raw"
        )
        self._process = subprocess.Popen(_q5_ssh_args(command), stdin=subprocess.PIPE,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, bufsize=0)
        configure = getattr(self._client, "configure_speaker", None)
        if callable(configure):
            configure(self._topic)
        self._running = True
        self._thread = threading.Thread(target=self._pump, daemon=True, name="q5_speaker_stream")
        self._thread.start()
        print(f"[SpeakerPlugin] subscribed {self._topic} -> Q5 ALSA {self._device}", flush=True)

    def _pump(self):
        while self._running and self._process and self._process.stdin:
            getter = getattr(self._client, "pop_speaker_chunk", None)
            chunk = getter() if callable(getter) else None
            if chunk is None:
                time.sleep(0.005)
                continue
            try:
                self._process.stdin.write(chunk)
                self._process.stdin.flush()
            except (BrokenPipeError, OSError):
                print("[SpeakerPlugin] remote playback stream ended", flush=True)
                break

    def stop(self):
        self._running = False
        if self._process is not None:
            try:
                if self._process.stdin:
                    self._process.stdin.close()
                self._process.wait(timeout=3)
            except (subprocess.TimeoutExpired, OSError):
                self._process.terminate()
            self._process = None

    def dispatch(self, action, args):
        if action == "start":
            self.start(args.get("input_topic"))
        elif action == "stop":
            self.stop()
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_in": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
        return None


class _Q5MediaPlugin:
    """Base for read-only Domain-211 media subscriptions.

    The developer container owns the D455 and SLAM services. These cards only
    subscribe to their DDS output and send bounded, already-processed payloads
    to the existing Domain-42 bridge worker.
    """

    def __init__(self, plugin_config, namespace, executor, client):
        self._ns = namespace
        self._client = client
        self._executor = executor
        self._running = False
        self._last_sent = 0.0
        self._max_hz = max(0.1, float(plugin_config.get("max_hz", 10.0)))
        self._subscription = None
        self._node = Node(self._node_name)
        executor.add_node(self._node)

    def _send_media(self, payload):
        sender = getattr(self._client, "publish_media", None)
        if callable(sender):
            sender(payload)

    def stop(self):
        self._running = False

    def dispatch(self, action, args):
        del args
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "start":
            self.start()
        if action in ("start", "info"):
            result = {
                "state": "running" if self._running else "idle",
                "source_topic": self._source_topic,
                "topic_out": [{"topic": self._topic, "format": self._format}],
            }
            if hasattr(self, "_frames_received"):
                result["diagnostics"] = {
                    "frames_received": self._frames_received,
                    "frames_sent": self._frames_sent,
                }
            return result
        return None


class CameraRgbPlugin(_Q5MediaPlugin):
    """D455 RGB to JPEG, throttled before crossing into Agent Core's DDS domain."""

    _node_name = "q5_camera_rgb"
    _format = "image/jpeg"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get("source_topic", "/camera/camera/color/image_raw"))
        self._topic = f"/{namespace}/q5/camera/rgb"
        self._jpeg_quality = max(20, min(95, int(plugin_config.get("jpeg_quality", 70))))
        self._latest = None
        self._frames_received = 0
        self._frames_sent = 0
        self._lock = threading.Lock()
        self._encoder = None
        self._remote_start = dict(plugin_config.get("remote_start") or {})
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_rgb", "type": "sensor", "multiInstance": False,
            "description": "Q5 D455 RGB camera. The developer-container RealSense driver must already be running.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
            "diagnostics": {"frames_received": self._frames_received, "frames_sent": self._frames_sent},
        }

    def start(self):
        if self._running:
            return
        import cv2
        import numpy as np
        from sensor_msgs.msg import Image

        self._start_remote_realsense_if_configured()
        self._cv2, self._np = cv2, np
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_image, _LATEST_QOS)
        self._encoder = threading.Thread(target=self._encode_loop, daemon=True, name="q5_rgb_encoder")
        self._encoder.start()
        print(f"[CameraRgbPlugin] subscribed {self._source_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _start_remote_realsense_if_configured(self):
        """Optionally start the D455 on its owning developer container via SSH.

        This is intentionally opt-in: XOS can also own the camera, and the
        launch never restarts a live driver. Q5 documents this developer
        account as part of its external-development workflow.
        """
        if not self._remote_start.get("enabled", False):
            return
        host = str(self._remote_start.get("host", "192.168.8.100"))
        user = str(self._remote_start.get("user", "developer"))
        password = str(self._remote_start.get("password", ""))
        try:
            port = int(self._remote_start.get("port", 2222))
        except (TypeError, ValueError):
            raise ValueError("camera_rgb.remote_start.port must be an integer")
        profiles = (
            str(self._remote_start.get("depth_profile", "848x480x30")),
            str(self._remote_start.get("color_profile", "848x480x30")),
        )
        if (not password or not re.fullmatch(r"[A-Za-z0-9.-]+", host) or
                not re.fullmatch(r"[A-Za-z0-9_-]+", user) or
                any(not re.fullmatch(r"[0-9]+x[0-9]+x[0-9]+", value) for value in profiles)):
            raise ValueError("invalid camera_rgb.remote_start configuration")
        remote = (
            "source /opt/ros/humble/setup.bash; "
            "export ROS_DOMAIN_ID=211 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
            "pgrep -f realsense2_camera_node >/dev/null || "
            "nohup ros2 launch realsense2_camera rs_align_depth_launch.py "
            f"depth_module.depth_profile:={profiles[0]} rgb_camera.color_profile:={profiles[1]} "
            ">/tmp/q5-realsense.log 2>&1 &"
        )
        command = ["sshpass", "-p", password, "ssh", "-p", str(port),
                   "-o", "PreferredAuthentications=password", "-o", "PubkeyAuthentication=no",
                   "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no",
                   "-o", "UserKnownHostsFile=/dev/null",
                   f"{user}@{host}", f"bash -lc {shlex.quote(remote)}"]
        result = subprocess.run(command, capture_output=True, text=True, timeout=12)
        if result.returncode:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"remote RealSense launch failed: {detail}")
        print(f"[CameraRgbPlugin] requested remote D455 launch on {user}@{host}:{port}", flush=True)

    def _on_image(self, msg):
        if self._running:
            with self._lock:
                self._latest = msg
                self._frames_received += 1

    def _encode_loop(self):
        while self._running:
            with self._lock:
                msg, self._latest = self._latest, None
            if msg is None or time.monotonic() - self._last_sent < 1.0 / self._max_hz:
                time.sleep(0.005)
                continue
            try:
                channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
                if channels is None or msg.step < msg.width * channels:
                    continue
                raw = self._np.frombuffer(msg.data, dtype=self._np.uint8)
                image = raw[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * channels]
                image = image.reshape(msg.height, msg.width, channels)
                if msg.encoding == "rgb8":
                    image = self._cv2.cvtColor(image, self._cv2.COLOR_RGB2BGR)
                elif msg.encoding == "rgba8":
                    image = self._cv2.cvtColor(image, self._cv2.COLOR_RGBA2BGR)
                elif msg.encoding == "bgra8":
                    image = self._cv2.cvtColor(image, self._cv2.COLOR_BGRA2BGR)
                ok, jpeg = self._cv2.imencode(".jpg", image, [self._cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
                if ok:
                    self._send_media({"kind": "rgb", "data": jpeg.tobytes(),
                                      "width": int(msg.width), "height": int(msg.height),
                                      "encoding": msg.encoding, "timestamp_ms": int(time.time() * 1000)})
                    self._frames_sent += 1
                    self._last_sent = time.monotonic()
            except Exception as exc:
                self._node.get_logger().warn(f"RGB encode failed: {exc}")


class CameraDepthPlugin(_Q5MediaPlugin):
    """D455 aligned depth image, preserving the source Z16/16UC1 measurement unit."""

    _node_name = "q5_camera_depth"
    _format = "image/depth-z16"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get("source_topic", "/camera/camera/aligned_depth_to_color/image_raw"))
        self._topic = f"/{namespace}/q5/camera/depth"
        self._frames_received = 0
        self._frames_sent = 0
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_depth", "type": "sensor", "multiInstance": False,
            "description": "Q5 D455 depth image, aligned to RGB. Values remain Z16/16UC1 millimetres.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
            "diagnostics": {"frames_received": self._frames_received, "frames_sent": self._frames_sent},
        }

    def start(self):
        if self._running:
            return
        from sensor_msgs.msg import Image
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_depth, _LATEST_QOS)
        print(f"[CameraDepthPlugin] subscribed {self._source_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _on_depth(self, msg):
        if not self._running:
            return
        self._frames_received += 1
        if (msg.encoding not in ("16UC1", "mono16") or
                time.monotonic() - self._last_sent < 1.0 / self._max_hz):
            return
        needed = int(msg.height) * int(msg.step)
        if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * 2 or len(msg.data) < needed:
            return
        self._send_media({"kind": "depth", "height": msg.height, "width": msg.width,
                          "encoding": "16UC1", "is_bigendian": msg.is_bigendian,
                          "step": msg.step, "data": bytes(msg.data[:needed])})
        self._frames_sent += 1
        self._last_sent = time.monotonic()


def _wait_for_future(future, timeout_sec: float):
    """Wait for work completed by main.py's shared executor thread."""
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.result() if future.done() else None


class StatePlugin:
    """Read-only joint and Q5 FSM state card."""

    def __init__(self, plugin_config, namespace, executor, client):
        del plugin_config, executor
        self._ns = namespace
        self._client = client

    def get_tool(self):
        return {
            "name": "state", "type": "sensor", "multiInstance": False,
            "description": "Q5 joint feedback and READY/ACTIVE state.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            # q5_bridge_worker publishes these JSON topics in Agent Core's DDS domain.
            "topic_out": [
                {"topic": f"/{self._ns}/q5/joints_state", "format": "data/json"},
                {"topic": f"/{self._ns}/q5/robot_status", "format": "data/json"},
            ],
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        del args
        if action == "stop":
            return {"state": "idle"}
        if action not in ("start", "info"):
            return None
        joint = self._client.snapshot()
        status = self._client.sensor_snapshot("robot_status")
        if not status.get("available"):
            status = self._client.sensor_snapshot("query_state")
        return {
            "state": "running",
            "joint_state": {
                "available": joint.get("available", False),
                "fresh": joint.get("fresh", False),
                "age_ms": joint.get("age_ms"),
                "joint_count": joint.get("joint_count", 0),
                "position_unit": joint.get("position_unit", "rad"),
            },
            "robot_status": {
                "available": status.get("available", False),
                "fresh": status.get("fresh", False),
                "age_ms": status.get("age_ms"),
                "state": status.get("state"),
                "message": status.get("message", ""),
                "source": status.get("source_service", "/xbot_state"),
            },
            "motion_manager_lifecycle": self._client.get_lifecycle_state(),
        }


class BatteryPlugin:
    """Read-only battery state card, including verified board firmware."""

    def __init__(self, plugin_config, namespace, executor, client):
        del plugin_config, executor
        self._ns = namespace
        self._client = client

    def get_tool(self):
        return {
            "name": "battery", "type": "sensor", "multiInstance": False,
            "description": "Q5 battery level, electrical readings, and power-board firmware.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": f"/{self._ns}/battery_state", "format": "data/json"}],
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        del args
        if action == "stop":
            return {"state": "idle"}
        if action not in ("start", "info"):
            return None
        battery = self._client.sensor_snapshot("battery")
        firmware = self._client.sensor_snapshot("battery_version")
        return {
            "state": "running",
            "available": battery.get("available", False),
            "fresh": battery.get("fresh", False),
            "age_ms": battery.get("age_ms"),
            "percentage": battery.get("percentage"),
            "voltage_v": battery.get("voltage"),
            "current_a": battery.get("current"),
            "temperature_c": battery.get("temperature"),
            "power_supply_status": battery.get("power_supply_status"),
            "firmware": firmware.get("components", {}),
        }


class AudioPlugin:
    """Vendor audio playback via /audio_player/play and paired services."""

    def __init__(self, plugin_config, namespace, executor, client):
        del namespace, client
        self._node = Node("q5_audio")
        executor.add_node(self._node)
        self._action_client = ActionClient(self._node, AudioPlay, "/audio_player/play")
        self._srv_volume = self._node.create_client(SetVolume, "/audio_player/set_volume")
        self._srv_stop = self._node.create_client(Trigger, "/audio_player/stop_play")
        self._srv_is_play = self._node.create_client(Trigger, "/audio_player/is_play")
        self._device = plugin_config.get("device", "plughw:2,0")

    def get_tool(self):
        return {
            "name": "audio", "type": "actuator", "multiInstance": False,
            "description": "Q5 vendor audio playback, volume, stop, and status.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "play", "set_volume", "stop_audio", "is_play", "info"]},
                "mode": {"type": "integer", "enum": [0, 1, 2, 3], "description": "0=id, 1=path, 2=item JSON, 3=file name"},
                "id": {"type": "integer"}, "path": {"type": "string"}, "item": {"type": "string"},
                "file_name": {"type": "string"}, "force_play": {"type": "boolean"},
                "timeout": {"type": "integer", "minimum": 0},
                "channel": {"type": "string", "enum": ["default", "channel1", "channel2", "channel3"]},
                "version": {"type": "string", "enum": ["v1", "v2"]},
                "volume": {"type": "integer", "minimum": 0, "maximum": 100},
            }, "required": ["action"], "additionalProperties": False},
        }

    def start(self):
        pass

    def stop(self):
        self._stop_audio()

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready", "action_server": "/audio_player/play", "device": self._device}
        if action == "play":
            return self._play(args)
        if action == "set_volume":
            return self._set_volume(args.get("volume", 50))
        if action == "stop_audio":
            return self._stop_audio()
        if action == "is_play":
            return self._is_playing()
        if action == "stop":
            self._stop_audio()
            return {"state": "idle"}
        return None

    def _play(self, args):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": "/audio_player/play is unavailable"}
        goal = AudioPlay.Goal()
        goal.mode = int(args.get("mode", 1))
        goal.force_play = bool(args.get("force_play", False))
        goal.id = int(args.get("id", 0))
        goal.path = str(args.get("path", ""))
        goal.item = str(args.get("item", ""))
        goal.file_name = str(args.get("file_name", ""))
        goal.channel = str(args.get("channel", "default"))
        goal.timeout = int(args.get("timeout", 0))
        goal.version = str(args.get("version", "v1"))
        goal_handle = _wait_for_future(self._action_client.send_goal_async(goal), 5.0)
        if goal_handle is None:
            return {"state": "error", "message": "audio goal timed out"}
        if not goal_handle.accepted:
            return {"state": "error", "message": "audio goal rejected"}
        response = _wait_for_future(goal_handle.get_result_async(), max(10.0, goal.timeout + 2.0))
        if response is None:
            return {"state": "error", "message": "audio result timed out"}
        return {"state": "ok" if response.result.success else "error", "message": response.result.message}

    def _set_volume(self, value):
        if not self._srv_volume.service_is_ready():
            return {"state": "error", "message": "/audio_player/set_volume is unavailable"}
        req = SetVolume.Request()
        req.volume = max(0, min(100, int(value)))
        response = _wait_for_future(self._srv_volume.call_async(req), 2.0)
        if response is None:
            return {"state": "error", "message": "set-volume request timed out"}
        return {"state": "ok" if response.success else "error", "volume": req.volume, "message": response.message}

    def _stop_audio(self):
        if not self._srv_stop.service_is_ready():
            return {"state": "error", "message": "/audio_player/stop_play is unavailable"}
        response = _wait_for_future(self._srv_stop.call_async(Trigger.Request()), 2.0)
        if response is None:
            return {"state": "error", "message": "stop-audio request timed out"}
        return {"state": "ok" if response.success else "error", "message": response.message}

    def _is_playing(self):
        if not self._srv_is_play.service_is_ready():
            return {"state": "error", "message": "/audio_player/is_play is unavailable"}
        response = _wait_for_future(self._srv_is_play.call_async(Trigger.Request()), 2.0)
        if response is None:
            return {"state": "error", "message": "is-play request timed out"}
        return {"state": "ok", "is_playing": response.success, "message": response.message}
