"""Lazy, shared Orbbec RGB-D runtime for Panther.

The first camera card started on the canvas ensures that the host-side Gemini 2
launch is running.  RGB and depth share that source process; stopping a card
only disarms its forwarding path so other host consumers are not interrupted.
"""

from __future__ import annotations

import subprocess
import threading
import time
import zlib


class PantherCameraRuntime:
    COLOR_TOPIC = "/camera/color/image_raw"
    DEPTH_TOPIC = "/camera/depth/image_raw"
    HOST_UNIT = "panther_orbbec.service"
    _NSENTER = ("nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--")

    def __init__(self, node, bridge, config: dict, sensor_qos):
        from sensor_msgs.msg import Image

        self._node = node
        self._bridge = bridge
        self._config = config
        self._active: set[str] = set()
        self._lock = threading.RLock()
        self._source_lock = threading.Lock()
        self._frames = {"rgb": None, "depth": None}
        self._last_frame_at = {"rgb": 0.0, "depth": 0.0}
        self._last_publish_at = {"rgb": 0.0, "depth": 0.0}
        self._armed_at = 0.0
        self._managed_service = False
        self._watchdog_restarts = 0
        self._closed = False

        self._subscriptions = (
            node.create_subscription(Image, self.COLOR_TOPIC, self._on_rgb, sensor_qos),
            node.create_subscription(Image, self.DEPTH_TOPIC, self._on_depth, sensor_qos),
        )
        self._rgb_thread = threading.Thread(target=self._rgb_loop, daemon=True, name="panther_rgb")
        self._depth_thread = threading.Thread(target=self._depth_loop, daemon=True, name="panther_depth")
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True, name="panther_camera_watchdog"
        )
        self._rgb_thread.start()
        self._depth_thread.start()
        self._watchdog_thread.start()

    def info(self, kind: str) -> dict:
        source = self.COLOR_TOPIC if kind == "rgb" else self.DEPTH_TOPIC
        with self._lock:
            active = kind in self._active
            last = self._last_frame_at[kind]
            managed = self._managed_service
        return {
            "state": "running" if active else "idle",
            "source_topic": source,
            "publisher_count": self._node.count_publishers(source),
            "last_frame_age_ms": None if not last else int((time.monotonic() - last) * 1000),
            "host_service": self.HOST_UNIT,
            "host_service_managed": managed,
        }

    def start(self, kind: str) -> dict:
        if kind not in ("rgb", "depth"):
            raise ValueError("camera kind must be rgb or depth")
        with self._lock:
            already_active = kind in self._active
        if already_active:
            return self.info(kind)

        with self._source_lock:
            self._ensure_source()
        deadline = time.monotonic() + float(self._config.get("startup_timeout_s", 30.0))
        required = self.COLOR_TOPIC if kind == "rgb" else self.DEPTH_TOPIC
        while time.monotonic() < deadline and self._node.count_publishers(required) < 1:
            time.sleep(0.2)
        if self._node.count_publishers(required) < 1:
            raise RuntimeError(f"Orbbec started but no publisher appeared on {required}")

        with self._lock:
            if not self._active:
                self._armed_at = time.monotonic()
                self._watchdog_restarts = 0
            self._active.add(kind)
            self._frames[kind] = None
            self._last_frame_at[kind] = 0.0
        return self.info(kind)

    def stop(self, kind: str) -> dict:
        with self._lock:
            self._active.discard(kind)
            self._frames[kind] = None
        return self.info(kind)

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._active.clear()
            self._frames = {"rgb": None, "depth": None}
            managed = self._managed_service
        for thread in (self._rgb_thread, self._depth_thread, self._watchdog_thread):
            thread.join(timeout=2.0)
        if managed:
            try:
                self._run_host(["systemctl", "stop", self.HOST_UNIT], timeout=15.0)
                print(f"[camera] stopped managed host unit {self.HOST_UNIT}", flush=True)
            except Exception as exc:
                print(f"[camera] failed to stop managed host unit: {exc}", flush=True)

    def _run_host(self, command, timeout: float = 15.0, check: bool = True):
        return subprocess.run(
            [*self._NSENTER, *command],
            check=check,
            capture_output=True,
            text=True,
            timeout=timeout,
        )

    def _ensure_source(self) -> None:
        # Any existing camera publisher means another process already owns the
        # device. Do not start a competing Gemini launch; the requested card
        # will wait for its own topic and report a timeout if that stream is not
        # enabled by the existing owner.
        if self._node.count_publishers(self.COLOR_TOPIC) or self._node.count_publishers(self.DEPTH_TOPIC):
            return

        active = self._run_host(
            ["systemctl", "is-active", self.HOST_UNIT], timeout=5.0, check=False
        )
        if active.stdout.strip() == "active":
            # It belonged to the host or another process before this driver.
            # Use it, but never stop or watchdog-restart something we do not own.
            return

        # A failed/inactive transient unit may still be loaded. Starting it is
        # cheaper and safer than attempting to create a duplicate unit name.
        existing = self._run_host(
            ["systemctl", "start", self.HOST_UNIT], timeout=15.0, check=False
        )
        if existing.returncode != 0:
            launch = (
                "cd /home/unix_ai/orbbec_ws && "
                ". install/setup.bash && "
                "export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
                "exec ros2 launch orbbec_camera gemini2.launch.py"
            )
            self._run_host(
                [
                    "systemd-run",
                    f"--unit={self.HOST_UNIT}",
                    "--collect",
                    "--property=Restart=on-failure",
                    "--property=RestartSec=2s",
                    "--uid=unix_ai",
                    "--setenv=HOME=/home/unix_ai",
                    "/bin/bash",
                    "-lc",
                    launch,
                ],
                timeout=15.0,
            )
        with self._lock:
            self._managed_service = True

    def _on_rgb(self, msg) -> None:
        with self._lock:
            if "rgb" not in self._active:
                return
            self._frames["rgb"] = msg
            self._last_frame_at["rgb"] = time.monotonic()

    def _on_depth(self, msg) -> None:
        with self._lock:
            if "depth" not in self._active:
                return
            self._frames["depth"] = msg
            self._last_frame_at["depth"] = time.monotonic()

    def _take_latest(self, kind: str, hz: float):
        with self._lock:
            if kind not in self._active:
                return None
            now = time.monotonic()
            if now - self._last_publish_at[kind] < 1.0 / hz:
                return None
            message = self._frames[kind]
            self._frames[kind] = None
            if message is not None:
                self._last_publish_at[kind] = now
            return message

    @staticmethod
    def _header_payload(message) -> dict:
        return {
            "frame_id": message.header.frame_id,
            "stamp": {
                "sec": int(message.header.stamp.sec),
                "nanosec": int(message.header.stamp.nanosec),
            },
        }

    def _rgb_loop(self) -> None:
        import cv2
        import numpy as np

        hz = max(1.0, min(float(self._config.get("rgb_hz", 10.0)), 20.0))
        quality = max(20, min(int(self._config.get("jpeg_quality", 70)), 95))
        max_width = max(160, int(self._config.get("max_width", 640)))
        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
        while not self._closed:
            message = self._take_latest("rgb", hz)
            if message is None:
                time.sleep(0.01)
                continue
            try:
                encoding = str(message.encoding).lower()
                count = channels.get(encoding)
                width, height, step = int(message.width), int(message.height), int(message.step)
                if count is None or width <= 0 or height <= 0 or step < width * count:
                    raise ValueError(f"unsupported RGB image layout: {message.encoding}")
                raw = np.frombuffer(message.data, dtype=np.uint8)
                if raw.size < height * step:
                    raise ValueError("truncated RGB image")
                image = raw[: height * step].reshape(height, step)[:, : width * count]
                image = image.reshape(height, width) if count == 1 else image.reshape(height, width, count)
                if encoding == "rgb8":
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                elif encoding == "rgba8":
                    image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
                elif encoding == "bgra8":
                    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
                if width > max_width:
                    output_height = max(1, round(height * max_width / width))
                    image = cv2.resize(image, (max_width, output_height), interpolation=cv2.INTER_AREA)
                ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
                if not ok:
                    raise ValueError("JPEG encoder rejected frame")
                payload = self._header_payload(message)
                payload.update({"message_format": "jpeg", "data": encoded.tobytes()})
                self._bridge.publish("camera_rgb", payload)
            except Exception as exc:
                print(f"[camera_rgb] dropped frame: {exc}", flush=True)

    def _depth_loop(self) -> None:
        import cv2
        import numpy as np

        hz = max(1.0, min(float(self._config.get("depth_hz", 8.0)), 15.0))
        while not self._closed:
            message = self._take_latest("depth", hz)
            if message is None:
                time.sleep(0.01)
                continue
            try:
                encoding = str(message.encoding)
                if encoding not in ("16UC1", "mono16", "32FC1"):
                    raise ValueError(f"unsupported depth encoding: {message.encoding}")
                item_size = 4 if encoding == "32FC1" else 2
                width, height, step = int(message.width), int(message.height), int(message.step)
                if width <= 0 or height <= 0 or step < width * item_size:
                    raise ValueError("invalid depth layout")
                raw = np.frombuffer(message.data, dtype=np.uint8)
                if raw.size < height * step:
                    raise ValueError("truncated depth image")
                depth = raw[: height * step].reshape(height, step)[:, : width * item_size]
                byte_order = ">" if message.is_bigendian else "<"
                if encoding == "32FC1":
                    metres = depth.copy().view(byte_order + "f4").reshape(height, width).astype(np.float64)
                    metres[~np.isfinite(metres) | (metres < 0)] = 0.0
                    depth = np.clip(np.rint(metres * 1000.0), 0, 65535).astype("<u2")
                else:
                    depth = depth.copy().view(byte_order + "u2").reshape(height, width).astype("<u2")
                if width * 3 > height * 4:
                    crop_width = height * 4 // 3
                    left = (width - crop_width) // 2
                    depth = depth[:, left:left + crop_width]
                elif width * 3 < height * 4:
                    crop_height = width * 3 // 4
                    top = (height - crop_height) // 2
                    depth = depth[top:top + crop_height, :]
                if depth.shape != (480, 640):
                    depth = cv2.resize(depth, (640, 480), interpolation=cv2.INTER_NEAREST)
                payload = self._header_payload(message)
                payload.update(
                    {
                        "message_format": "16UC1; compressedDepth zlib",
                        "data": zlib.compress(depth.astype("<u2", copy=False).tobytes(), 1),
                    }
                )
                self._bridge.publish("camera_depth", payload)
            except Exception as exc:
                print(f"[camera_depth] dropped frame: {exc}", flush=True)

    def _watchdog_loop(self) -> None:
        first_frame_grace = float(self._config.get("first_frame_grace_s", 30.0))
        stall = float(self._config.get("frame_stall_s", 15.0))
        maximum = int(self._config.get("max_watchdog_restarts", 2))
        while not self._closed:
            time.sleep(1.0)
            with self._lock:
                active = set(self._active)
                managed = self._managed_service
                armed_at = self._armed_at
                last = dict(self._last_frame_at)
                restarts = self._watchdog_restarts
            if not active or not managed or not armed_at:
                continue
            unhealthy = []
            now = time.monotonic()
            for kind in active:
                reference = last[kind] or armed_at
                limit = stall if last[kind] else first_frame_grace
                if now - reference >= limit:
                    unhealthy.append(kind)
            if not unhealthy or restarts >= maximum:
                continue
            try:
                self._run_host(["systemctl", "restart", self.HOST_UNIT], timeout=30.0)
                with self._lock:
                    self._watchdog_restarts += 1
                    self._armed_at = time.monotonic()
                    for kind in self._active:
                        self._last_frame_at[kind] = 0.0
                print(f"[camera] restarted {self.HOST_UNIT}; stalled={unhealthy}", flush=True)
            except Exception as exc:
                print(f"[camera] watchdog restart failed: {exc}", flush=True)
