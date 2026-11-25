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
from uuid import uuid4

from common.camera_info import build as build_camera_info
from spatial_inputs import CLOCK_ID, make_geometry, metric_depth, transform_matrix, imu_packet, stamp_ns, mount_offset_matrix


class PantherCameraRuntime:
    COLOR_TOPIC = "/camera/color/image_raw"
    DEPTH_TOPIC = "/camera/depth/image_raw"
    IMU_TOPIC = "/camera/gyro_accel/sample"
    HOST_UNIT = "panther_orbbec.service"
    _NSENTER = ("nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--")

    def __init__(self, node, bridge, config: dict, sensor_qos):
        from sensor_msgs.msg import Image, CameraInfo, Imu
        from tf2_ros import Buffer, TransformListener

        self._node = node
        self._bridge = bridge
        self._config = config
        self._active: set[str] = set()
        self._lock = threading.RLock()
        self._source_lock = threading.Lock()
        self._frames = {"rgb": None, "depth": None, "imu": None}
        self._last_frame_at = {"rgb": 0.0, "depth": 0.0, "imu": 0.0}
        self._last_publish_at = {"rgb": 0.0, "depth": 0.0}
        self._frame_failures = {"rgb": 0, "depth": 0}
        self._armed_at = 0.0
        self._source_unit = None
        self._managed_unit = None
        self._watchdog_restarts = 0
        self._closed = False
        self._infos = {}
        self._geometry = None
        self._geometry_error = "waiting for source CameraInfo"
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._epoch = uuid4().hex
        self._imu_error = "waiting for camera IMU and capture-time TF"
        self._imu_last_stamp = 0
        self._mount_error = "waiting for capture-time body-camera TF"
        self._mount_estimated = False

        self._subscriptions = (
            node.create_subscription(Image, self.COLOR_TOPIC, self._on_rgb, sensor_qos),
            node.create_subscription(Image, self.DEPTH_TOPIC, self._on_depth, sensor_qos),
            node.create_subscription(CameraInfo, "/camera/color/camera_info",
                                     lambda msg: self._on_info("color", msg), sensor_qos),
            node.create_subscription(CameraInfo, "/camera/depth/camera_info",
                                     lambda msg: self._on_info("depth", msg), sensor_qos),
            node.create_subscription(Imu, self.IMU_TOPIC, self._on_camera_imu, sensor_qos),
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
        source = {"rgb": self.COLOR_TOPIC, "depth": self.DEPTH_TOPIC, "imu": self.IMU_TOPIC}[kind]
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
        imu_last = self._last_frame_at["imu"]
        imu_ready = bool("imu" in self._active and imu_last and time.monotonic() - imu_last < .5
                         and self._geometry is not None and not self._imu_error)
        geometry_ready = self._geometry is not None
        return {
            "state": "running" if active and frame_fresh and geometry_ready else "unavailable" if active else "idle",
            "available": publishers > 0 and last > 0,
            "fresh": frame_fresh,
            "source_topic": source,
            "publisher_count": publishers,
            "last_frame_age_ms": None if not last else int((time.monotonic() - last) * 1000),
            "host_service": source_unit,
            "host_service_managed": managed_unit is not None,
            "geometry_ready": self._geometry is not None,
            "geometry_error": self._geometry_error,
            "imu_status": {"available": imu_ready,
                           "reason": self._imu_error or ("" if imu_ready else "camera IMU idle or stale")},
            "body_mount_status": {"available": not self._mount_error, "reason": self._mount_error},
        }

    def start(self, kind: str) -> dict:
        if kind not in ("rgb", "depth", "imu"):
            raise ValueError("camera kind must be rgb, depth or imu")
        required = {"rgb": self.COLOR_TOPIC, "depth": self.DEPTH_TOPIC, "imu": self.IMU_TOPIC}[kind]
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
            self._active.clear()
            self._frames = {"rgb": None, "depth": None}
            managed_unit = self._managed_unit
            self._managed_unit = None
        for thread in (self._rgb_thread, self._depth_thread, self._watchdog_thread):
            thread.join(timeout=2.0)
        self._tf_listener.unregister()
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
            if "rgb" not in self._active:
                return
            self._frames["rgb"] = msg
            self._last_frame_at["rgb"] = time.monotonic()

    def _on_info(self, kind, msg):
        key = (kind, msg.header.frame_id, int(msg.width), int(msg.height),
               tuple(msg.k), tuple(msg.d), msg.distortion_model)
        with self._lock:
            if self._infos.get(kind, (None,))[0] == key:
                return
            self._infos[kind] = (key, msg)
            self._geometry = None
            self._geometry_error = "waiting for both source CameraInfo messages"
            if set(self._infos) != {"color", "depth"}:
                return
            try:
                color, depth = self._infos["color"][1], self._infos["depth"][1]
                geometry = make_geometry(color, depth, registered=self._config.get("registered_depth", True))
                geometry["source_layout"] = {
                    "rgb": (color.width, color.height), "depth": (depth.width, depth.height)}
                self._geometry = geometry
                self._geometry_error = ""
            except Exception as exc:
                self._geometry_error = self._bounded_error(exc)

    def declaration(self, kind, topic):
        with self._lock:
            geometry = self._geometry
            error = self._geometry_error
        vendor = {"camera": "Orbbec Gemini 2 RGB-D", "transport": "ros2",
                  "clock_id": CLOCK_ID, "timestamp_semantics": "source_ros_header",
                  "clock_mapping": {"verified": False, "method": "source_ros_header"},
                  "registration": "registered_rectified" if geometry is not None else "unknown",
                  "geometry_error": error,
                  "body_mount": self.mount_declaration()}
        if kind == "depth":
            vendor.update(depth_unit="mm", depth_source="measured", depth_scale_m=0.001,
                          invalid_value=0, depth_convention="axial_z",
                          source_depth_unit_mm=self._config.get("source_depth_unit_mm", 1.0))
        if geometry is not None:
            vendor["frame_id"] = geometry["frame_id"]
        return build_camera_info(
            topic=topic["topic"], id="unixai/panther/camera/rectified", format=topic["format"],
            width=640, height=480, K=geometry["K"].ravel().tolist() if geometry is not None else None,
            D=[0.] * 5 if geometry is not None else None,
            distortion_model="pinhole" if geometry is not None else "unknown",
            source="derived-from-K" if geometry is not None else "unknown",
            pipeline=["unixai/panther/registered-ray-rectification"], vendor=vendor)

    def mount_declaration(self):
        return dict(schema="motus.camera_mount/1",
                    topic=f"/{self._bridge._namespace}/panther/camera/body_mount",
                    parent_frame=self._config.get("body_frame", "base_link"),
                    child_frame=self._geometry["frame_id"] if self._geometry is not None else None,
                    source="configured_cad_estimate" if getattr(self, "_mount_estimated", False) else "tf",
                    version="capture_tf_v1", epoch=self._epoch, mode="dynamic",
                    length_unit="m", transform_direction="body_from_camera",
                    accuracy_validated=False,
                    provenance={"estimated_mount_config": self._config.get("estimated_body_mount", {}),
                                "native_tf_preferred": True},
                    status="unavailable" if self._mount_error else "available", reason=self._mount_error)

    def _lookup_matrix(self, target, source, message):
        from rclpy.time import Time
        if stamp_ns(message) <= 0 or not target or not source:
            raise ValueError("positive source capture time and frames required; no latest TF fallback")
        result = self._tf_buffer.lookup_transform(target, source, Time.from_msg(message.header.stamp))
        if result.header.frame_id != target or result.child_frame_id != source:
            raise ValueError("TF frames differ from request")
        return transform_matrix(result)

    def _publish_mount(self, message):
        packet = self.mount_declaration()
        packet.pop("topic")
        packet.update(stamp_ns=stamp_ns(message), stamp_source="measurement", clock_id=CLOCK_ID,
                      child_frame=message.header.frame_id)
        try:
            try:
                matrix = self._lookup_matrix(packet["parent_frame"], message.header.frame_id, message)
                self._mount_estimated = False
            except Exception:
                estimate = self._config.get("estimated_body_mount", {})
                if not estimate.get("enabled", False):
                    raise
                # Compose only in this driver. No competing TF authority or
                # changes to the host's robot_description/static TF tree.
                body_from_model = self._lookup_matrix(
                    packet["parent_frame"], estimate["model_frame"], message)
                source_from_optical = self._lookup_matrix(
                    estimate["source_frame"], message.header.frame_id, message)
                matrix = body_from_model @ mount_offset_matrix(estimate) @ source_from_optical
                self._mount_estimated = True
            packet["matrix"] = matrix.tolist()
            packet["source"] = "configured_cad_estimate" if self._mount_estimated else "tf"
            self._mount_error = ""
        except Exception as exc:
            self._mount_error = self._bounded_error(exc)
        packet.update(status="unavailable" if self._mount_error else "available", reason=self._mount_error)
        self._bridge.publish("body_mount", packet)

    def _image_geometry(self, message, kind):
        with self._lock:
            geometry = self._geometry
        if geometry is None:
            raise ValueError(self._geometry_error)
        if (message.header.frame_id != geometry["frame_id"] or
                (message.width, message.height) != geometry["source_layout"][kind]):
            raise ValueError("image frame/size differs from source calibration")
        if stamp_ns(message) <= 0:
            raise ValueError("image capture timestamp unavailable")
        return geometry

    def _on_camera_imu(self, message):
        with self._lock:
            if "imu" not in self._active or self._geometry is None:
                return
            frame = self._geometry["frame_id"]
        try:
            stamp = stamp_ns(message)
            if stamp <= self._imu_last_stamp:
                if stamp < self._imu_last_stamp:
                    self._epoch = uuid4().hex
                    self._imu_last_stamp = 0
                raise ValueError("nonmonotonic IMU source time")
            matrix = self._lookup_matrix(frame, message.header.frame_id, message)
            packet = imu_packet(message, matrix, frame, self._epoch)
            with self._lock:
                if "imu" not in self._active or self._closed:
                    return
                self._bridge.publish("camera_imu", packet)
                self._imu_last_stamp = stamp
                self._imu_error = ""
                self._last_frame_at["imu"] = time.monotonic()
        except Exception as exc:
            self._imu_error = self._bounded_error(exc)

    def arm_imu(self):
        # Optional camera motion must not prevent the existing body IMU display.
        with self._source_lock:
            self._ensure_source()
        with self._lock:
            self._active.add("imu")
            self._armed_at = time.monotonic()
        return self.info("imu")

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
                geometry = self._image_geometry(message, "rgb")
                image = cv2.remap(image, geometry["color_x"], geometry["color_y"], cv2.INTER_LINEAR)
                ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
                if not ok:
                    raise ValueError("JPEG encoder rejected frame")
                payload = self._header_payload(message)
                payload.update({"message_format": "jpeg", "data": encoded.tobytes()})
                with self._lock:
                    if "rgb" in self._active and geometry is self._geometry and not self._closed:
                        self._publish_mount(message)
                        self._bridge.publish("camera_color", payload)
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
                depth = metric_depth(message, self._config.get("source_depth_unit_mm", 1.0))
                geometry = self._image_geometry(message, "depth")
                depth = cv2.remap(depth, geometry["depth_x"], geometry["depth_y"], cv2.INTER_NEAREST)
                payload = self._header_payload(message)
                payload.update(
                    {
                        "message_format": "16UC1; compressedDepth zlib",
                        "data": zlib.compress(depth.astype("<u2", copy=False).tobytes(), 1),
                    }
                )
                with self._lock:
                    if "depth" in self._active and geometry is self._geometry and not self._closed:
                        self._publish_mount(message)
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
                # Optional motion/TF failure must never restart valid RGB-D.
                active = set(self._active) & {"rgb", "depth"}
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
