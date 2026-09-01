"""Q5 visitor-video card: record the newest RGB frames as a bounded MP4."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import subprocess
import threading
import time

try:
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    _HAS_ROS2 = True
except Exception:
    _HAS_ROS2 = False


CARD = "visitor_video"
NODE = "q5_visitor_video"
_CAMERA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
) if _HAS_ROS2 else None


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get(
            "source_topic", "/camera/camera/color/image_raw"))
        self._output_dir = Path(str(plugin_config.get(
            "output_dir", "/work/visitor_videos"))).expanduser()
        self._fps = max(1, min(15, int(plugin_config.get("fps", 10))))
        self._max_duration_s = max(1, min(30, int(plugin_config.get("max_duration_s", 30))))
        self._lock = threading.Condition()
        self._latest = None
        self._latest_at = 0.0
        self._sequence = 0
        self._node = None
        self._subscription = None

        if _HAS_ROS2 and executor is not None:
            self._node = Node(NODE)
            executor.add_node(self._node)

    def get_tool(self):
        return {
            "name": CARD,
            "type": "actuator",
            "multiInstance": False,
            "description": "Record the Q5 RGB camera for 1 to 30 seconds and save an MP4 locally.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    # The canvas startup sequence calls action="start" for
                    # every card.  This is only a readiness check; recording
                    # still creates the temporary RGB subscription on demand.
                    "action": {"type": "string", "enum": ["start", "record", "info", "stop"]},
                    "duration_s": {
                        "type": "integer", "minimum": 1, "maximum": 30,
                        "description": "Recording length in seconds; never more than 30 seconds.",
                    },
                    "visitor_label": {
                        "type": "string",
                        "description": "Optional label used only in the saved filename.",
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
            },
        }

    def get_tools(self):
        return [self.get_tool()]

    def start(self):
        return {"state": "ready" if self._node is not None else "unavailable"}

    def stop(self):
        self._release_subscription()

    def _release_subscription(self):
        if self._node is not None and self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None

    def _ensure_subscription(self):
        if self._node is not None and self._subscription is None:
            # Do not create a second permanent RGB consumer beside the
            # isolated camera worker; subscribe only for an explicit record.
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_image, _CAMERA_QOS)

    def _on_image(self, msg):
        with self._lock:
            self._latest = msg
            self._latest_at = time.time()
            self._sequence += 1
            self._lock.notify_all()

    def _info(self):
        with self._lock:
            age = None if not self._latest_at else round(time.time() - self._latest_at, 2)
        return {
            "ok": self._node is not None,
            "source_topic": self._source_topic,
            "output_dir": str(self._output_dir),
            "fps": self._fps,
            "max_duration_s": self._max_duration_s,
            "latest_frame_age_s": age,
        }

    @staticmethod
    def _safe_label(value):
        value = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(value))
        return value.strip("_")[:40] or "visitor"

    @staticmethod
    def _rgb_bytes(msg):
        import numpy as np
        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
        if channels is None or msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * channels:
            raise ValueError(f"Unsupported RGB message encoding: {msg.encoding}")
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        image = raw[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * channels]
        image = image.reshape(msg.height, msg.width, channels)
        if msg.encoding == "bgr8":
            image = image[:, :, ::-1]
        elif msg.encoding == "rgba8":
            image = image[:, :, :3]
        elif msg.encoding == "bgra8":
            image = image[:, :, [2, 1, 0]]
        return np.ascontiguousarray(image).tobytes(), int(msg.width), int(msg.height)

    def _record(self, args):
        if self._node is None:
            return {"ok": False, "code": "ROS_UNAVAILABLE", "message": "Q5 ROS camera subscription is unavailable"}
        self._ensure_subscription()
        requested = int(args.get("duration_s", 5))
        if not 1 <= requested <= self._max_duration_s:
            return {"ok": False, "code": "INVALID_DURATION", "message": f"duration_s must be between 1 and {self._max_duration_s}"}
        with self._lock:
            if self._latest is None:
                self._lock.wait(timeout=2.0)
            if self._latest is None:
                return {"ok": False, "code": "NO_FRAME", "message": "No RGB frame has arrived yet"}
            msg = self._latest
            sequence = self._sequence
        try:
            first_frame, width, height = self._rgb_bytes(msg)
            self._output_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            label = self._safe_label(args.get("visitor_label", "visitor"))
            path = self._output_dir / f"{stamp}_{label}.mp4"
            command = [
                "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                "-s", f"{width}x{height}", "-r", str(self._fps), "-i", "-",
                "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(path),
            ]
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
            process.stdin.write(first_frame)
            frames = 1
            deadline = time.monotonic() + requested
            while time.monotonic() < deadline:
                with self._lock:
                    remaining = max(0.0, deadline - time.monotonic())
                    self._lock.wait_for(lambda: self._sequence > sequence, timeout=min(1.0 / self._fps, remaining))
                    if self._sequence <= sequence:
                        continue
                    msg, sequence = self._latest, self._sequence
                frame, frame_width, frame_height = self._rgb_bytes(msg)
                if frame_width == width and frame_height == height:
                    process.stdin.write(frame)
                    frames += 1
            process.stdin.close()
            stderr = process.stderr.read().decode("utf-8", "replace")
            if process.wait(timeout=10) != 0 or not path.exists():
                raise RuntimeError(stderr.strip() or "ffmpeg failed to create MP4")
            return {
                "ok": True,
                "file_path": str(path),
                "recorded_duration_s": requested,
                "frames": frames,
                "captured_at": datetime.now().isoformat(timespec="seconds"),
            }
        except Exception as exc:
            try:
                process.kill()
            except Exception:
                pass
            return {"ok": False, "code": "RECORD_FAILED", "message": str(exc)}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready" if self._node is not None else "error",
                    "message": "" if self._node is not None else "Q5 ROS camera subscription is unavailable"}
        if action == "info":
            return self._info()
        if action == "record":
            try:
                return self._record(args)
            finally:
                self._release_subscription()
        if action == "stop":
            self._release_subscription()
            return {"state": "idle"}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
