"""Q5 visitor-photo card: save the newest RGB camera frame as a JPEG.

The card deliberately does not publish images or upload them.  It only writes
an operator-configured local directory and returns the resulting path, so a
Skill can include that path in its current-session visitor record.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import threading
import time

try:
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    _HAS_ROS2 = True
except Exception:
    _HAS_ROS2 = False


CARD = "visitor_photo"
NODE = "q5_visitor_photo"


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get(
            "source_topic", "/camera/camera/color/image_raw"))
        self._output_dir = Path(str(plugin_config.get(
            "output_dir", "/tmp/visitor_photos"))).expanduser()
        self._jpeg_quality = max(20, min(95, int(plugin_config.get("jpeg_quality", 90))))
        self._lock = threading.Lock()
        self._latest = None
        self._latest_at = 0.0
        self._node = None
        self._subscription = None

        if _HAS_ROS2 and executor is not None:
            self._node = Node(NODE)
            executor.add_node(self._node)
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_image, 1)

    def get_tool(self):
        return {
            "name": CARD,
            "type": "actuator",
            "multiInstance": False,
            "description": "Save the newest Q5 RGB camera frame as a visitor JPEG and return its local file path.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["capture", "info"]},
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
        return None

    def _on_image(self, msg):
        with self._lock:
            self._latest = msg
            self._latest_at = time.time()

    def _info(self):
        with self._lock:
            age = None if not self._latest_at else round(time.time() - self._latest_at, 2)
        return {
            "ok": self._node is not None,
            "source_topic": self._source_topic,
            "output_dir": str(self._output_dir),
            "latest_frame_age_s": age,
        }

    @staticmethod
    def _safe_label(value):
        value = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(value))
        return value.strip("_")[:40] or "visitor"

    def _capture(self, args):
        if self._node is None:
            return {"ok": False, "code": "ROS_UNAVAILABLE", "message": "Q5 ROS camera subscription is unavailable"}
        with self._lock:
            msg = self._latest
            frame_age = time.time() - self._latest_at if self._latest_at else None
        if msg is None:
            return {"ok": False, "code": "NO_FRAME", "message": "No RGB frame has arrived yet"}
        if frame_age is not None and frame_age > 3.0:
            return {"ok": False, "code": "STALE_FRAME", "message": "Latest RGB frame is older than 3 seconds", "frame_age_s": round(frame_age, 2)}
        try:
            import numpy as np
            from PIL import Image as PilImage
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
            self._output_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            label = self._safe_label(args.get("visitor_label", "visitor"))
            path = self._output_dir / f"{stamp}_{label}.jpg"
            PilImage.fromarray(np.ascontiguousarray(image), "RGB").save(path, "JPEG", quality=self._jpeg_quality)
            return {
                "ok": True,
                "file_path": str(path),
                "captured_at": datetime.now().isoformat(timespec="seconds"),
                "frame_age_s": round(frame_age or 0.0, 2),
            }
        except Exception as exc:
            return {"ok": False, "code": "CAPTURE_FAILED", "message": str(exc)}

    def dispatch(self, action, args):
        if action == "info":
            return self._info()
        if action == "capture":
            return self._capture(args)
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
