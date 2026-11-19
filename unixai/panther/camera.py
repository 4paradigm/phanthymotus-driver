"""Lazy, shared Orbbec RGB-D runtime for Panther.

The first camera card started on the canvas ensures that the host-side Gemini 2
launch is running.  RGB and depth share that source process; stopping a card
only disarms its forwarding path so other host consumers are not interrupted.
"""

from __future__ import annotations

import subprocess
import shlex
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
        self._photo_ready = threading.Condition(self._lock)
        self._photo_waiting = False
        self._photo_frame = None
        self._source_lock = threading.Lock()
        self._frames = {"rgb": None, "depth": None}
        self._last_frame_at = {"rgb": 0.0, "depth": 0.0}
        self._last_publish_at = {"rgb": 0.0, "depth": 0.0}
        self._frame_failures = {"rgb": 0, "depth": 0}
        self._armed_at = 0.0
        self._source_unit = None
        self._managed_unit = None
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
            source_unit = self._source_unit
            managed_unit = self._managed_unit
        frame_fresh = bool(
            last
            and time.monotonic() - last < float(self._config.get("frame_stall_s", 15.0))
        )
        publishers = self._node.count_publishers(source)
        return {
            "state": "running" if active and frame_fresh else "unavailable" if active else "idle",
            "available": publishers > 0 and last > 0,
            "fresh": frame_fresh,
            "source_topic": source,
            "publisher_count": publishers,
            "last_frame_age_ms": None if not last else int((time.monotonic() - last) * 1000),
            "host_service": source_unit,
            "host_service_managed": managed_unit is not None,
        }

    def start(self, kind: str) -> dict:
        if kind not in ("rgb", "depth"):
            raise ValueError("camera kind must be rgb or depth")
        required = self.COLOR_TOPIC if kind == "rgb" else self.DEPTH_TOPIC
        with self._lock:
            last = self._last_frame_at[kind]
            already_ready = (
                kind in self._active
                and last > 0
                and time.monotonic() - last < float(self._config.get("frame_stall_s", 15.0))
            )
            if already_ready:
                return self.info(kind)
            if not self._active:
                self._watchdog_restarts = 0
            # The watchdog's first-frame grace starts when this stream is
            # armed, including when the sibling RGB/depth stream is already on.
            self._armed_at = time.monotonic()
            self._active.add(kind)
            self._frames[kind] = None
            self._last_frame_at[kind] = 0.0
        try:
            with self._source_lock:
                self._ensure_source()
            publisher_deadline = time.monotonic() + float(
                self._config.get("startup_timeout_s", 30.0)
            )
            while time.monotonic() < publisher_deadline and self._node.count_publishers(required) < 1:
                time.sleep(0.2)
            if self._node.count_publishers(required) < 1:
                raise RuntimeError(f"no publisher appeared on {required}")

            frame_deadline = time.monotonic() + float(
                self._config.get("first_frame_grace_s", 30.0)
            )
            while time.monotonic() < frame_deadline:
                with self._lock:
                    if self._last_frame_at[kind] > 0:
                        return self.info(kind)
                time.sleep(0.05)
            raise RuntimeError(f"publisher appeared but no frame was received on {required}")
        except Exception:
            with self._lock:
                self._active.discard(kind)
                self._frames[kind] = None
                self._last_frame_at[kind] = 0.0
            raise

    def stop(self, kind: str) -> dict:
        with self._lock:
            self._active.discard(kind)
            self._frames[kind] = None
        return self.info(kind)

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._photo_ready.notify_all()
            self._active.clear()
            self._frames = {"rgb": None, "depth": None}
            managed_unit = self._managed_unit
            self._managed_unit = None
        for thread in (self._rgb_thread, self._depth_thread, self._watchdog_thread):
            thread.join(timeout=2.0)
        if managed_unit:
            try:
                self._run_host(["systemctl", "stop", "--no-block", managed_unit], timeout=15.0)
                print(f"[camera] stopped managed host unit {managed_unit}", flush=True)
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

        active = self._run_host(["systemctl", "is-active", self.HOST_UNIT], timeout=5.0, check=False)
        if active.stdout.strip() == "active":
            # It belonged to the host or another process before this driver.
            # Use it, but never stop or watchdog-restart something we do not own.
            with self._lock:
                self._source_unit = self.HOST_UNIT
            return

        # If the host already defines the fixed unit, start it but do not claim
        # ownership. This matches PR 521 and avoids duplicate Orbbec processes.
        defined = self._run_host(["systemctl", "cat", self.HOST_UNIT], timeout=5.0, check=False)
        if defined.returncode == 0:
            started = self._run_host(["systemctl", "start", self.HOST_UNIT], timeout=30.0, check=False)
            if started.returncode != 0:
                detail = (started.stderr or started.stdout or "unknown systemd error").strip()
                raise RuntimeError(f"failed to start {self.HOST_UNIT}: {detail}")
            with self._lock:
                self._source_unit = self.HOST_UNIT
            return

        workspace = str(self._config.get("workspace", "/home/unix_ai/orbbec_ws"))
        profile = str(
            self._config.get("cyclonedds_profile", "/home/unix_ai/config/cyclonedds.xml")
        )
        for flag, path, label in (("-d", workspace, "Orbbec workspace"),
                                  ("-f", profile, "CycloneDDS profile")):
            checked = self._run_host(["test", flag, path], timeout=5.0, check=False)
            if checked.returncode != 0:
                raise RuntimeError(f"{label} is missing on the host: {path}")

        # Create one stable transient unit. A fixed name makes ownership and
        # process state observable, and prevents repeated starts from creating
        # multiple camera drivers.
        launch = (
            f"cd {shlex.quote(workspace)} && "
            ". install/setup.bash && "
            "export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
            f"CYCLONEDDS_URI={shlex.quote('file://' + profile)} && "
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
            self._source_unit = self.HOST_UNIT
            self._managed_unit = self.HOST_UNIT

    @staticmethod
    def _bounded_error(exc: Exception, limit: int = 240) -> str:
        escaped = str(exc).encode("unicode_escape", "backslashreplace").decode("ascii")
        return escaped if len(escaped) <= limit else escaped[:limit] + "..."

    def _log_frame_failure(self, kind: str, card: str, exc: Exception) -> None:
        with self._lock:
            self._frame_failures[kind] += 1
            count = self._frame_failures[kind]
        if count == 1 or count % 100 == 0:
            detail = self._bounded_error(exc)
            print(f"[{card}] dropped frame #{count}: {detail}", flush=True)

    def _on_rgb(self, msg) -> None:
        with self._lock:
            if "rgb" in self._active:
                self._frames["rgb"] = msg
                self._last_frame_at["rgb"] = time.monotonic()
            if self._photo_waiting and self._photo_frame is None:
                self._photo_frame = msg
                self._photo_ready.notify()

    def rgb_available(self) -> bool:
        return self._node.count_publishers(self.COLOR_TOPIC) > 0

    def capture_jpeg(self, timeout_s: float) -> dict:
        """Use the next RGB frame from a running source, independently of preview."""
        with self._photo_ready:
            if self._closed:
                raise RuntimeError("camera runtime is shutting down")
            if self._photo_waiting:
                raise RuntimeError("another capture is already waiting for an RGB frame")
            if not self.rgb_available():
                raise RuntimeError("no RGB camera publisher; start camera_color first")
            self._photo_waiting = True
            try:
                self._photo_ready.wait_for(lambda: self._photo_frame is not None or self._closed, timeout_s)
                if self._closed:
                    raise RuntimeError("camera runtime is shutting down")
                message = self._photo_frame
                if message is None:
                    raise TimeoutError("no new RGB frame arrived before the capture timeout")
            finally:
                self._photo_waiting = False
                self._photo_frame = None
        return self._encode_rgb(message)

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

    def _encode_rgb(self, message) -> dict:
        import cv2
        import numpy as np

        quality = max(20, min(int(self._config.get("jpeg_quality", 70)), 95))
        max_width = max(160, int(self._config.get("max_width", 640)))
        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
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
        return {**self._header_payload(message), "message_format": "jpeg", "data": encoded.tobytes()}

    def _rgb_loop(self) -> None:
        hz = max(1.0, min(float(self._config.get("rgb_hz", 10.0)), 20.0))
        while not self._closed:
            message = self._take_latest("rgb", hz)
            if message is None:
                time.sleep(0.01)
                continue
            try:
                self._bridge.publish("camera_color", self._encode_rgb(message))
            except Exception as exc:
                self._log_frame_failure("rgb", "camera_color", exc)

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
                self._log_frame_failure("depth", "camera_depth", exc)

    def _watchdog_loop(self) -> None:
        first_frame_grace = float(self._config.get("first_frame_grace_s", 30.0))
        stall = float(self._config.get("frame_stall_s", 15.0))
        maximum = int(self._config.get("max_watchdog_restarts", 2))
        while not self._closed:
            time.sleep(1.0)
            with self._lock:
                active = set(self._active)
                managed_unit = self._managed_unit
                armed_at = self._armed_at
                last = dict(self._last_frame_at)
                restarts = self._watchdog_restarts
            if not active or not managed_unit or not armed_at:
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
                self._run_host(["systemctl", "restart", managed_unit], timeout=30.0)
                with self._lock:
                    self._watchdog_restarts += 1
                    self._armed_at = time.monotonic()
                    for kind in self._active:
                        self._last_frame_at[kind] = 0.0
                print(f"[camera] restarted {managed_unit}; stalled={unhealthy}", flush=True)
            except Exception as exc:
                print(f"[camera] watchdog restart failed: {exc}", flush=True)
