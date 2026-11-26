"""Split the vendor's raw side-by-side stereo frames for the canvas."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import socket
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

from common.camera_info import build as build_camera_info


EYES = ("left", "right")
EYE_WIDTH, EYE_HEIGHT = 1280, 960


_HOST_ROOT = Path("/proc/1/root")
_VENDOR_ROOT = _HOST_ROOT / "home/unix_ai/work/srt"
_MONITOR_SOCKET = _VENDOR_ROOT / "stream_momitor/.stream_monitor/control.sock"
_VIDEO_ROOT = _VENDOR_ROOT / "video"
_SHARE_KEY = "cam_share_yuv"


def _video_request(command):
    # The manufacturer's monitor can start/restart just the head video stream.
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(8)
        client.connect(str(_MONITOR_SOCKET))
        client.sendall((json.dumps({"cmd": command, "stream": "video"}) + "\n").encode())
        client.shutdown(socket.SHUT_WR)
        with client.makefile("rb") as response:
            reply = json.loads(response.read(65536))
    if not reply.get("ok"):
        raise RuntimeError(reply.get("error") or "vendor video command failed")
    if command == "status":
        return next(row for row in reply["streams"] if row["id"] == "video")


def _ini_value(text, key):
    match = re.search(rf"^[ \t]*{key}[ \t]*=[ \t]*([^#;\r\n]*)", text, re.MULTILINE)
    return match[1].strip() if match else None


def _enable_template(path, text):
    pattern = rf"^([ \t]*{_SHARE_KEY}[ \t]*=)[^\r\n]*"
    updated, replacements = re.subn(pattern, r"\1 1", text, count=1, flags=re.MULTILINE)
    if not replacements:
        # switch_mode.sh overlays recognized keys from the selected template.
        # Insert inside [camera] so this remains an ordinary vendor INI file.
        updated, replacements = re.subn(
            r"^(\[camera\][ \t]*)$", r"\1\ncam_share_yuv = 1", text, count=1, flags=re.MULTILINE
        )
        if not replacements:
            raise RuntimeError(f"{path.name} has no [camera] section")
    metadata = path.stat()
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as replacement:
        staged = Path(replacement.name)
        try:
            replacement.write(updated)
            replacement.flush()
            os.fchown(replacement.fileno(), metadata.st_uid, metadata.st_gid)
            os.fchmod(replacement.fileno(), metadata.st_mode & 0o777)
            staged.replace(path)
        finally:
            staged.unlink(missing_ok=True)


def ensure_vendor_stereo():
    try:
        status = _video_request("status")
    except (FileNotFoundError, ConnectionRefusedError):
        subprocess.run(
            ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--",
             "systemctl", "start", "stream_monitor.service"],
            check=True, timeout=12, capture_output=True,
        )
        deadline = time.monotonic() + 5
        while True:
            try:
                status = _video_request("status")
                break
            except (FileNotFoundError, ConnectionRefusedError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.2)

    # Use the effective template: the vendor may translate the requested mode
    # to another configuration according to the current transport.
    args = status.get("args") or []
    name = args[0] if args else ""
    if not re.fullmatch(r"Config[A-Za-z0-9_-]*\.ini", name):
        raise RuntimeError("could not identify the vendor video configuration")
    path = _VIDEO_ROOT / name
    text = path.read_text()
    defaults = (_VIDEO_ROOT / "Config1.ini").read_text()

    def effective(key):
        return _ini_value(text, key) or _ini_value(defaults, key)

    if tuple(effective(key) for key in ("cam_is_mono", "cam_img_width", "cam_img_height")) != (
        "0", str(2 * EYE_WIDTH), str(EYE_HEIGHT)
    ):
        raise RuntimeError(f"vendor mode {name} does not provide the calibrated stereo resolution")
    if effective(_SHARE_KEY) != "1":
        _enable_template(path, text)

    if not status.get("desired"):
        _video_request("start")
    elif status.get("running") and not status.get("restart_pending"):
        live = _VIDEO_ROOT / "config_send.ini"
        if not live.exists() or _ini_value(live.read_text(), _SHARE_KEY) != "1":
            _video_request("restart")


def load_intrinsics(path):
    root = ET.parse(path).getroot()

    def matrix(name, rows, cols):
        element = root.find(name)
        if element is None or (int(element.findtext("rows", "0")),
                               int(element.findtext("cols", "0"))) != (rows, cols):
            raise ValueError(f"invalid shape for stereo calibration {name}")
        values = [float(value) for value in element.findtext("data", "").split()]
        if len(values) != rows * cols or not all(map(math.isfinite, values)):
            raise ValueError(f"invalid values for stereo calibration {name}")
        return values

    if matrix("size", 1, 2) != [EYE_HEIGHT, EYE_WIDTH]:
        raise ValueError("stereo calibration does not match the native eye resolution")
    intrinsics = {}
    for index, eye in enumerate(EYES, 1):
        k = matrix(f"M{index}", 3, 3)
        if min(k[0], k[4]) <= 0 or k[6:] != [0, 0, 1]:
            raise ValueError(f"invalid intrinsics for the {eye} eye")
        intrinsics[eye] = (k, matrix(f"D{index}", 1, 5))
    return intrinsics


def eye_jpegs(message, eyes, quality):
    import cv2
    import numpy as np

    width = EYE_WIDTH * 2
    if (message.width, message.height) != (width, EYE_HEIGHT):
        raise ValueError(f"expected {width}x{EYE_HEIGHT} side-by-side stereo image")
    encoding = message.encoding.lower()
    if encoding == "i420":
        if message.step != width or len(message.data) != width * EYE_HEIGHT * 3 // 2:
            raise ValueError("invalid stereo I420 stride or data length")
        planar = np.frombuffer(message.data, dtype=np.uint8).reshape(EYE_HEIGHT * 3 // 2, width)
        bgr_pair = cv2.cvtColor(planar, cv2.COLOR_YUV2BGR_I420)
    elif encoding in ("yuyv", "yuv422_yuy2", "yuy2"):
        if message.step < width * 2 or len(message.data) != message.step * EYE_HEIGHT:
            raise ValueError("invalid stereo YUYV stride or data length")
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(EYE_HEIGHT, message.step)
        packed = rows[:, :width * 2].reshape(EYE_HEIGHT, width, 2)
        bgr_pair = None
    else:
        raise ValueError(f"unsupported stereo encoding {encoding!r}")

    images = {}
    for eye in eyes:
        start = EYES.index(eye) * EYE_WIDTH
        columns = slice(start, start + EYE_WIDTH)
        if bgr_pair is None:
            bgr = cv2.cvtColor(packed[:, columns], cv2.COLOR_YUV2BGR_YUY2)
        else:
            bgr = bgr_pair[:, columns]
        ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise ValueError(f"JPEG encoding failed for the {eye} eye")
        images[eye] = encoded.tobytes()
    return images


class StereoCameraRuntime:
    def __init__(self, node, bridge, config, sensor_qos):
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

        self._node = node
        self._group = MutuallyExclusiveCallbackGroup()
        self._bridge = bridge
        self._qos = sensor_qos
        self._manage_vendor = config.get("manage_vendor_source", True)
        self._topic = config.get("source_topic", "/ros_srt")
        hz = float(config.get("publish_hz", 10.0))
        self._quality = int(config.get("jpeg_quality", 75))
        if not 0 < hz <= 30 or not 0 <= self._quality <= 100:
            raise ValueError("stereo_camera publish_hz must be in (0, 30] and jpeg_quality in [0, 100]")
        self._period = 1 / hz
        self._intrinsics = {}
        self._calibration_error = None
        path = config.get("calibration_path") or "/proc/1/root/home/unix_ai/work/srt/video/stereo.yaml"
        try:
            self._intrinsics = load_intrinsics(path)
        except (OSError, ValueError, ET.ParseError) as exc:
            self._calibration_error = str(exc)[:200]

        self._cv = threading.Condition()
        self._active = {}
        self._subscription = None
        self._closed = False
        self._pending = None
        self._last_accepted = float("-inf")
        self._last_output = {}
        self._undistorted = None
        try:
            live_ini = (_VIDEO_ROOT / "config_send.ini").read_text()
            algorithm = _ini_value(live_ini, "cam_use_len_algorithm")
            if algorithm in ("0", "1"):
                self._undistorted = algorithm == "1"
        except OSError:
            pass
        self._error = None
        self._source_error = None
        self._source_task = None
        self._worker = threading.Thread(target=self._run, daemon=True, name="panther_stereo")
        self._worker.start()

    @staticmethod
    def _frame(eye):
        return f"panther_camera_{eye}_optical_frame"

    def declaration(self, eye, port):
        native_k, native_d = self._intrinsics.get(eye, (None, None))
        with self._cv:
            undistorted = self._undistorted
        # The vendor's I420 path uses identity rotation and each eye's own K
        # as the new camera matrix. It removes distortion without aligning
        # the two eyes' epipolar lines.
        d = None
        if native_d is not None and undistorted is not None:
            d = [0.0] * len(native_d) if undistorted else native_d
        return build_camera_info(
            topic=port["topic"], id=f"unixai/panther/camera_{eye}", format=port["format"],
            width=EYE_WIDTH, height=EYE_HEIGHT,
            K=native_k, D=d, distortion_model="pinhole" if native_k is not None else "unknown",
            source="vendor-spec" if native_k is not None else "unknown",
            pipeline=["unixai/panther/split-stereo", "unixai/panther/jpeg"],
            vendor={"camera": "Pixe XYZ 3D", "eye": eye, "frame_id": self._frame(eye),
                    "undistorted": undistorted, "rectified": False,
                    "calibration_key": f"M{EYES.index(eye) + 1}/D{EYES.index(eye) + 1}"},
        )

    def start(self, eye):
        from sensor_msgs.msg import Image

        with self._cv:
            if self._closed:
                return
            first = self._subscription is None
            if first:
                self._subscription = self._node.create_subscription(
                    Image, self._topic, self._on_image, self._qos, callback_group=self._group
                )
                self._last_accepted = float("-inf")
                self._error = None
            # A frame already being encoded cannot publish to a restarted eye.
            self._active.setdefault(eye, object())
            if (self._manage_vendor and (self._source_task is None or not self._source_task.is_alive())
                    and (first or self._source_error)):
                # Discovery/host I/O can take seconds. Keep it off the caller
                # and the frame worker, and coalesce overlapping card starts.
                self._source_error = None
                self._source_task = threading.Thread(
                    target=self._prepare_source, daemon=True, name="panther_stereo_source"
                )
                self._source_task.start()

    def _prepare_source(self):
        try:
            if (self._node.count_publishers(self._topic) or self._frames_arriving(0.6)
                    or self._node.count_publishers(self._topic)):
                return
            ensure_vendor_stereo()
        except Exception as exc:
            with self._cv:
                self._source_error = f"Could not enable vendor stereo: {exc}"[:240]

    def _frames_arriving(self, timeout):
        with self._cv:
            self._cv.wait_for(lambda: not self._active or self._closed
                              or time.monotonic() - self._last_accepted < 1, timeout=timeout)
            return not self._active or self._closed or time.monotonic() - self._last_accepted < 1

    def stop(self, eye):
        with self._cv:
            self._active.pop(eye, None)
            self._last_output.pop(eye, None)
            if not self._active:
                self._unsubscribe()
                self._cv.notify_all()

    def _unsubscribe(self):
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        self._pending = None

    def close(self):
        with self._cv:
            self._closed = True
            self._active.clear()
            self._unsubscribe()
            self._cv.notify_all()
        self._worker.join(timeout=2)

    def info(self, eye):
        with self._cv:
            active = eye in self._active
            live = time.monotonic() - self._last_output.get(eye, float("-inf")) < 3
            error = self._error or self._source_error
        result = {
            "state": ("running" if live else "waiting_for_data") if active else "idle",
            "source_topic": self._topic,
            "calibrated": eye in self._intrinsics,
        }
        if active and not live:
            result["reason"] = error or "Waiting for vendor stereo frames (cam_share_yuv = 1)"
        if self._calibration_error:
            result["calibration_error"] = self._calibration_error
        return result

    def _on_image(self, message):
        now = time.monotonic()
        with self._cv:
            if not self._active or now - self._last_accepted < self._period:
                return
            self._last_accepted = now
            self._pending = (message, dict(self._active))
            self._cv.notify_all()

    def _run(self):
        while True:
            with self._cv:
                self._cv.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                message, active = self._pending
                self._pending = None

            try:
                images = eye_jpegs(message, active, self._quality)
                stamp = {"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec}
                with self._cv:
                    if any(self._active.get(eye) is generation for eye, generation in active.items()):
                        self._undistorted = message.encoding.lower() == "i420"
                    for eye, image in images.items():
                        if self._active.get(eye) is not active[eye]:
                            continue
                        self._bridge.publish(f"camera_{eye}", {
                            "message_format": "jpeg", "data": image,
                            "frame_id": self._frame(eye), "stamp": stamp,
                        })
                        self._last_output[eye] = time.monotonic()
                        self._error = None
            except Exception as exc:
                with self._cv:
                    if any(self._active.get(eye) is generation for eye, generation in active.items()):
                        self._error = str(exc)[:200]
