"""Standalone front-camera depth from calibrated LiDAR and monocular inference.

The output follows the existing 640x480 depth-zlib contract. It is evaluated
at the LiDAR header time, not at an inferred RTSP exposure time. Projection,
hole filling, inference, fusion and compression run in a bounded subprocess.
"""
from __future__ import annotations

import copy
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import re
import tempfile
import threading
import time

from common.camera_info import build as build_camera_info
from common.vendor_runtime import action_schema, tool
from deep_robotics.s10_pro.depth_projection import (
    depth_intrinsics, fill_depth_holes_mm, project_depth_mm, validate_cloud_layout, validate_transform,
)
from deep_robotics.s10_pro.undistortion import CameraUndistortion, half_fov
from deep_robotics.s10_pro.cloud_relay import CloudRelay
from deep_robotics.s10_pro.depth_fusion import fuse_depth_mm
from deep_robotics.s10_pro.monocular_depth import MonocularDepth
from deep_robotics.s10_pro import cpu_kernels

NAME = "camera_front_depth"
FRAME = "s10_pro_camera_front_optical"
FORMAT = "image/depth-zlib"
ROS_FORMAT = "16UC1; compressedDepth zlib"
MAX_DATA_BYTES = 64 * 1024 * 1024
MAX_RESULT_AGE_S = 3.0
MAX_RGB_RECEIPT_AGE_S = .35
SOURCE_TIMEOUT_S = 3.0
CAMERA_ID = "deeprobotics/s10_pro/camera_front"
FRONT_STREAM = "rtsp://10.21.33.103:8554/video4"
BUNDLED_CALIBRATIONS = Path(__file__).with_name("resource") / "calibration"
INPUT_ERROR_CODES = (None, "CLOUD_PUBLISHER_UNAVAILABLE", "CLOUD_DATA_TIMEOUT",
                     "CLOUD_RELAY_FAILED", "CLOUD_RELAY_STALE", "CLOUD_RELAY_DATA_TIMEOUT")
DEFAULTS = {
    "cloud_topic": "/LIDAR/POINTS", "cloud_frame_id": "",
    "camera_optical_from_cloud": None, "allow_standard_extrinsics": False,
    "min_depth_m": 0.1, "max_depth_m": 50.0, "rate_hz": 5.0,
}


class DepthError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def _settings(values):
    if not isinstance(values, dict):
        raise ValueError("depth configuration must be an object")
    unknown = set(values) - set(DEFAULTS)
    if unknown:
        raise ValueError("unsupported depth setting: " + ", ".join(sorted(unknown)))
    out = {**DEFAULTS, **values}
    if type(out["allow_standard_extrinsics"]) is not bool:
        raise ValueError("allow_standard_extrinsics must be a boolean")
    topic = out["cloud_topic"]
    if (not isinstance(topic, str) or len(topic) > 256 or
            not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*", topic)):
        raise ValueError("cloud_topic must be an absolute ROS topic")
    frame = out["cloud_frame_id"]
    if not isinstance(frame, str) or len(frame) > 128 or any(c.isspace() for c in frame):
        raise ValueError("cloud_frame_id must be the exact source frame name")
    matrix = out["camera_optical_from_cloud"]
    out["camera_optical_from_cloud"] = (None if matrix is None or matrix == "" else
                                        validate_transform(matrix))
    for key in ("min_depth_m", "max_depth_m", "rate_hz"):
        number = out[key]
        if isinstance(number, bool) or not isinstance(number, (int, float)):
            raise ValueError(f"{key} must be a finite number")
        try:
            out[key] = float(number)
        except OverflowError as exc:
            raise ValueError(f"{key} must be a finite number") from exc
        if not math.isfinite(out[key]):
            raise ValueError(f"{key} must be a finite number")
    if not 0 <= out["min_depth_m"] < out["max_depth_m"] <= 65.535:
        raise ValueError("depth range must satisfy 0 <= min_depth_m < max_depth_m <= 65.535")
    if not 0 < out["rate_hz"] <= 15:
        raise ValueError("rate_hz must be greater than 0 and at most 15")
    return out


def _put_latest(target, value):
    try:
        target.put_nowait(value)
        return True
    except queue.Full:
        try:
            target.get_nowait()
        except queue.Empty:
            pass
        try:
            target.put_nowait(value)
            return True
        except queue.Full:
            return False


def _project_worker(jobs, results):
    from common import logsafe
    logsafe.install(check_fd=False)
    model = None
    while True:
        job = jobs.get()
        if job is None:
            return
        # Do not send the source point buffer back through the result queue.
        context = {key: job[key] for key in
                   ("generation", "camera_generation", "stamp", "source_stamp", "received_monotonic")}
        try:
            started = time.monotonic()
            raw, stats = project_depth_mm(
                job["cloud"], job["K"], job["transform"],
                min_depth_m=job["min_depth_m"], max_depth_m=job["max_depth_m"])
            stats['projection_ms'] = round((time.monotonic() - started) * 1000, 1)
            fill_started = time.monotonic()
            filled, fill_stats = fill_depth_holes_mm(raw)
            stats.update(fill_stats)
            stats['hole_fill_ms'] = round((time.monotonic() - fill_started) * 1000, 1)
            if model is None:
                model = MonocularDepth()
            inference_started = time.monotonic()
            mono = model.predict_mm(job['jpeg'])
            stats['monocular_inference_ms'] = round((time.monotonic() - inference_started) * 1000, 1)
            fusion_started = time.monotonic()
            fused_packet, fusion_stats = fuse_depth_mm(raw, filled, mono,
                                                  min_depth_m=job['min_depth_m'],
                                                  max_depth_m=job['max_depth_m'])
            stats.update(fusion_stats)
            stats['fusion_ms'] = round((time.monotonic() - fusion_started) * 1000, 1)
            stats["worker_wait_ms"] = round((started - context["received_monotonic"]) * 1000, 1)
            stats['processing_ms'] = round((time.monotonic() - started) * 1000, 1)
            stats['cpu_kernel_backend'] = 'native' if cpu_kernels.AVAILABLE else 'numpy'
            stats["measured_coverage"] = round(stats["valid_pixels"] / (640 * 480), 4)
            stats["filled_coverage"] = round(
                (stats["valid_pixels"] + stats["filled_pixels"]) / (640 * 480), 4)
            _put_latest(results, (context, fused_packet, stats, None))
        except Exception as exc:
            _put_latest(results, (context, None, None, str(exc)[:300]))


class DepthWorker:
    """One bounded projection process, with explicit ownership and shutdown."""

    def __init__(self, on_result, on_error):
        self.on_result, self.on_error = on_result, on_error
        self._ctx = mp.get_context("spawn")
        self._jobs, self._results = self._ctx.Queue(1), self._ctx.Queue(1)
        # Set before the feeder starts. Closing our reader after a failed
        # child exits deliberately breaks an in-flight write during shutdown.
        self._jobs._ignore_epipe = True
        self._process = self._ctx.Process(target=_project_worker,
            args=(self._jobs, self._results), name="s10-front-depth", daemon=True)
        self._closed = threading.Event()
        self._collector = threading.Thread(target=self._collect,
                                           name="s10-front-depth-results", daemon=True)

    def start(self):
        self._process.start()
        self._collector.start()

    def is_alive(self):
        return (not self._closed.is_set() and self._process.is_alive()
                and self._collector.is_alive())

    def submit(self, job):
        return not self._closed.is_set() and _put_latest(self._jobs, job)

    def _collect(self):
        while not self._closed.is_set():
            try:
                context, packet, stats, error = self._results.get(timeout=.1)
            except queue.Empty:
                continue
            except (EOFError, OSError):
                return
            try:
                if error is not None:
                    self.on_error(context, error)
                else:
                    self.on_result(context, packet, stats)
            except Exception as exc:
                self.on_error(context, str(exc)[:300])

    def close(self):
        if self._closed.is_set():
            return
        self._closed.set()
        if self._process.pid is not None:
            # Never read a partially transferred job to make room here.
            try:
                self._jobs.put_nowait(None)
            except queue.Full:
                pass
            self._process.join(timeout=1)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(timeout=1)

        stuck = []
        # Queue.close() alone neither releases these unused pipe endpoints
        # nor interrupts partial reads/writes. Once the child is reaped, EOF
        # releases the collector and EPIPE releases the input feeder. Close
        # the reader BEFORE Queue.close() can send its internal sentinel, so
        # the feeder cannot concurrently close the same descriptor.
        self._results._writer.close()
        self._jobs._reader.close()
        self._jobs.close()
        self._jobs.cancel_join_thread()
        feeder = self._jobs._thread
        if feeder is not None:
            feeder.join(timeout=1)
        if feeder is not None and feeder.is_alive():
            stuck.append("input feeder")
        else:
            # An ignored EPIPE exits the feeder without closing its writer.
            self._jobs._writer.close()
        if self._collector.ident is not None:
            self._collector.join(timeout=1)
        self._results.cancel_join_thread()
        if self._collector.is_alive():
            stuck.append("result collector")
        else:
            self._results.close()
            self._results._reader.close()
        if self._process.is_alive():
            stuck.append("projection process")
        else:
            self._process.close()
        if stuck:
            raise RuntimeError("depth shutdown did not finish: " + ", ".join(stuck))


class FrontDepthPlugin:
    def __init__(self, config, namespace="", nodes=None, *, config_path=None,
                 extrinsics_path=None):
        self.nodes = nodes
        self._relay_config = config
        prefix = f"/{str(namespace).strip('/')}" if namespace else ""
        self.topic = prefix + "/camera_front/depth"
        self.controller = getattr(nodes, "camera_undistortion", None)
        if self.controller is None:
            self.controller = CameraUndistortion(("camera_front",))
        self.path = Path(config_path or os.environ.get("S10_FRONT_DEPTH_CONFIG") or
                         "/opt/phanthy-motus/data/s10_pro/camera_front_depth.json")
        self.extrinsics_path = Path(extrinsics_path or os.environ.get("S10_FRONT_CAMERA_EXTRINSICS") or
                                   "/opt/phanthy-motus/data/s10_pro/camera_front_extrinsics.json")
        self._explicit_extrinsics_path = bool(extrinsics_path or os.environ.get("S10_FRONT_CAMERA_EXTRINSICS"))
        self._front_stream = config.get("camera_rtsp", {}).get("camera_front", FRONT_STREAM)
        self._active_config = None
        self._optical_frame = FRAME
        self._extrinsics_source = "unknown"
        self._lock = threading.RLock()
        self._lifecycle = threading.Lock()
        self._running = False
        self._generation = 0
        self._worker = self._subscription = self._publisher = None
        self._watchdog = None
        self._source_state = "idle"
        self._started_at = self._last_cloud_at = self._observed_frame = None
        self._publisher_count = None
        self._relay = None
        self._input_transport = "ros"
        self._last_stamp = None
        self._last_submit = float("-inf")
        self._last_error = None
        self._error_code = None
        self._stats = {"received": 0, "published": 0, "dropped": 0, "throttled": 0}
        try:
            self._config = _settings(config.get("camera_front_depth", {}))
            if self.path.exists():
                saved = json.loads(self.path.read_text(encoding="utf-8"))
                if not isinstance(saved, dict) or saved.get("version") != 1:
                    raise ValueError("unsupported saved front-depth configuration")
                self._config = _settings({**self._config, **saved["config"]})
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self._config = copy.deepcopy(DEFAULTS)
            self._last_error = f"cannot load front-depth configuration: {exc}"
            self._error_code = "CONFIGURATION_INVALID"

    def _bundled_extrinsics(self):
        path = BUNDLED_CALIBRATIONS / "s10_pro_standard_front.json"
        try:
            return json.loads(path.read_text(encoding="utf-8")), path
        except FileNotFoundError as exc:
            raise DepthError("CALIBRATION_MISSING",
                             f"Standard S10 Pro front-camera calibration is unavailable: {path}") from exc
        except (OSError, ValueError, UnicodeError) as exc:
            raise DepthError("CALIBRATION_INVALID",
                             f"Cannot read standard S10 Pro front-camera calibration {path}: {exc}") from exc

    def _load_extrinsics(self, settings):
        """Read deployment data; a present invalid record never falls back."""
        path, source_prefix = self.extrinsics_path, "file:"
        try:
            record = json.loads(self.extrinsics_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            if settings["camera_optical_from_cloud"] is not None:
                if not settings["cloud_frame_id"]:
                    raise DepthError("CALIBRATION_INVALID", "Calibration is missing cloud_frame_id")
                return settings, FRAME, "maintenance-config"
            if self._explicit_extrinsics_path:
                raise DepthError("CALIBRATION_MISSING",
                    f"The explicitly selected front-camera calibration is unavailable: {path}")
            if not settings["allow_standard_extrinsics"]:
                raise DepthError("CALIBRATION_MISSING",
                    f"Install this robot's front-camera extrinsics at {path}, or explicitly "
                    "enable allow_standard_extrinsics for a matching standard mount.")
            record, path = self._bundled_extrinsics()
            source_prefix = "bundled:"
        except (OSError, ValueError, UnicodeError) as exc:
            raise DepthError("CALIBRATION_INVALID",
                             f"Cannot read front-camera calibration {self.extrinsics_path}: {exc}") from exc
        try:
            if not isinstance(record, dict) or type(record.get("version")) is not int or record["version"] != 1:
                raise ValueError("expected a version 1 calibration object")
            for key, expected in (("camera_id", CAMERA_ID), ("front_camera_stream", self._front_stream),
                                  ("translation_unit", "m"), ("camera_axes", "x_right_y_down_z_forward"),
                                  ("cloud_topic", DEFAULTS["cloud_topic"])):
                if record.get(key) != expected:
                    raise ValueError(f"calibration {key} does not match the active front-camera binding")
            frame = record["camera_optical_frame_id"]
            if not isinstance(frame, str) or not frame or len(frame) > 128 or any(c.isspace() for c in frame):
                raise ValueError("calibration camera_optical_frame_id must name the measured optical frame")
            resolved = _settings({**settings, **{key: record[key] for key in
                ("cloud_topic", "cloud_frame_id", "camera_optical_from_cloud")}})
            if not resolved["cloud_frame_id"] or resolved["cloud_frame_id"] in {"map", "world", "odom"}:
                raise ValueError("calibration cloud_frame_id must name the fixed sensor source")
            if resolved["camera_optical_from_cloud"] is None:
                raise ValueError("calibration camera_optical_from_cloud is missing")
        except (ValueError, KeyError, TypeError) as exc:
            raise DepthError("CALIBRATION_INVALID",
                             f"Invalid front-camera calibration {path}: {exc}") from exc
        provenance = record.get("provenance")
        if (source_prefix == "file:" and isinstance(provenance, dict)
                and provenance.get("kind") == "shared-standard-model-mounting"):
            if not settings["allow_standard_extrinsics"]:
                raise DepthError("STANDARD_EXTRINSICS_DISABLED",
                    f"{path} contains shared standard extrinsics. Install this robot's calibration "
                    "or explicitly enable allow_standard_extrinsics.")
            source_prefix = "standard-file:"
        return resolved, frame, source_prefix + str(path)

    def _camera_info(self):
        calibration, source = self.controller.calibration("camera_front")
        k = depth_intrinsics(calibration) if calibration else None
        horizontal, vertical = half_fov(k, 640, 480) if k else (None, None)
        with self._lock:
            config = copy.deepcopy(self._active_config or self._config)
            optical_frame, extrinsics_source = self._optical_frame, self._extrinsics_source
        return build_camera_info(
            topic=self.topic,
            id="deeprobotics/s10_pro/camera_front", format=FORMAT,
            width=640, height=480, distortion_model="pinhole" if k else "unknown",
            K=k, D=[0.] * 5 if k else None,
            half_fov_rad=horizontal, half_fov_v_rad=vertical,
            source="inherited" if k else "unknown",
            measured_on=calibration.get("measured_on", "") if calibration else "",
            pipeline=["deeprobotics/s10_pro/camera_front", "deeprobotics/s10_pro/undistort",
                      "lidar_projection", "resize:640x360->640x480", "bounded_2d_hole_fill",
                      "yolo26n_depth", "lidar_calibrated_completion"],
            vendor={
                "frame_id": optical_frame, "calibration_source": source,
                "cloud_topic": config["cloud_topic"], "cloud_frame_id": config["cloud_frame_id"],
                "camera_optical_from_cloud": config["camera_optical_from_cloud"],
                "extrinsics_source": extrinsics_source,
                "calibration_warning": ("使用共享标准外参，本机相机与雷达的重投影未经验证，障碍物可能投影到错误像素。"
                                        if extrinsics_source.startswith(("bundled:", "standard-file:")) else None),
                "reprojection_verified": False,
                "depth_unit": "mm", "depth_scale_m": .001, "invalid_depth_value": 0,
                "depth_quantity": "optical_z", "sparse": False, "hole_filling": True,
                "depth_source": "lidar_monocular_fusion", "monocular_completion": True,
                "interpolated_pixels_are_measured": False,
                "registered_view": "camera_front", "depth_aligned_to_rgb": False,
                "rgb_resolution": [640, 360],
                "rgb_pixel_mapping": {"scale": [1., 4 / 3], "offset": [0., 1 / 6],
                                      "convention": "pixel_centres"},
                "timestamp_source": "driver_receive",
                "source_stamp_preserved": False, "hardware_capture_timestamp_available": False,
                "temporal_synchronization": False, "rgb_depth_capture_skew_ms": None,
                "rgb_timestamp_source": "publication_clock", "scan_deskewed": None,
            })

    def _topic_outputs(self):
        return [{"topic": self.topic, "format": FORMAT,
                 "desc": "前相机完整深度图"}]

    def get_tool(self):
        with self._lock:
            settings = copy.deepcopy(self._config)
        properties = {
            "cloud_topic": {"type": "string", "title": "源点云 PointCloud2 话题"},
            "cloud_frame_id": {"type": "string", "title": "雷达消息的 frame_id"},
            "camera_optical_from_cloud": {"type": "string", "title": "雷达到前相机光学坐标的 4×4 外参",
                "description": "Measured rigid transform as a JSON 4x4 array; translation in metres. No default."},
            "allow_standard_extrinsics": {"type": "boolean", "title": "允许使用未验证的标准外参",
                "description": "仅在已确认本机相机、镜头和安装位姿符合标准安装时启用。"},
            "min_depth_m": {"type": "number", "title": "最小深度（米）", "minimum": 0},
            "max_depth_m": {"type": "number", "title": "最大深度（米）", "maximum": 65.535},
            "rate_hz": {"type": "number", "title": "最大输出频率（Hz）", "exclusiveMinimum": 0, "maximum": 15},
        }
        schema = action_schema({
            "start": ([], "自动加载本机标定并启动前相机融合深度；标定缺失时明确报错"),
            "stop": ([], "停止本深度流"), "info": ([], "深度参数、运行状态和时间来源"),
            "config": (list(properties), "保存深度配置；修改后重新启动本卡片"),
        }, properties)
        card = tool(NAME, "sensor", "山猫 S10 Pro 前相机深度图：自动读取标定并融合雷达与单目深度",
                    schema, topic_out=self._topic_outputs())
        card["camera_info"] = [self._camera_info()]
        config_properties = {key: copy.deepcopy(properties[key]) for key in
                             ("min_depth_m", "max_depth_m", "rate_hz", "allow_standard_extrinsics")}
        for key, prop in config_properties.items():
            prop.update(scope="instance", default=settings[key])
        card["configSchema"] = {"type": "object", "properties": config_properties}
        return card

    def start(self):
        """Bundle startup advertises the card; the card action owns capture."""

    def stop(self):
        with self._lifecycle:
            self._stop_locked()

    def _stop_locked(self):
        with self._lock:
            self._running = False
            self._generation += 1
            subscription, worker = self._subscription, self._worker
            watchdog = self._watchdog
            relay, self._relay = self._relay, None
            self._input_transport = "ros"
            self._subscription = self._worker = None
            self._watchdog = None
            self._source_state = "idle"
            self._last_error = self._error_code = None
        try:
            if watchdog is not None:
                self.nodes.robot.destroy_timer(watchdog)
        finally:
            try:
                if subscription is not None:
                    self.nodes.robot.destroy_subscription(subscription)
            finally:
                try:
                    if relay is not None:
                        relay.close()
                finally:
                    if worker is not None:
                        worker.close()

    def _start_capture(self):
        with self._lifecycle:
            if self.nodes is not None:
                executor = self.nodes.robot_executor_status()
                if executor["state"] != "running":
                    raise DepthError("ROS_EXECUTOR_UNAVAILABLE", executor["error"])
            with self._lock:
                if self._running and self._worker is not None and self._worker.is_alive():
                    return
            self._stop_locked()
            with self._lock:
                self._active_config = None
                self._extrinsics_source = "unknown"
                self._optical_frame = FRAME
                settings = copy.deepcopy(self._config)
            settings, optical_frame, extrinsics_source = self._load_extrinsics(settings)
            calibration, reason = self.controller.calibration("camera_front")
            if calibration is None:
                raise DepthError("CALIBRATION_INVALID", f"Front-camera intrinsics unavailable: {reason}")
            if not self.controller.active("camera_front"):
                raise ValueError("camera_front undistort must be enabled with valid calibration")
            if self.nodes is None:
                raise ValueError("ROS 2 runtime is unavailable")
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
            from sensor_msgs.msg import CompressedImage, PointCloud2
            try:
                if self._publisher is None:
                    self._publisher = self.nodes.bridge.create_publisher(NAME, self.topic, CompressedImage)
                with self._lock:
                    self._generation += 1
                    generation = self._generation
                    self._last_stamp, self._last_submit = None, float("-inf")
                    self._last_error = None
                    self._error_code = None
                    self._active_config = settings
                    self._optical_frame = optical_frame
                    self._extrinsics_source = extrinsics_source
                    self._source_state = "waiting"
                    self._started_at = time.monotonic()
                    self._last_cloud_at = self._observed_frame = self._publisher_count = None
                    self._worker = DepthWorker(self._receive_result, self._worker_error)
                    self._worker.start()
                    self._running = True
                self._subscription = self.nodes.robot.create_subscription(
                    PointCloud2, settings["cloud_topic"],
                    lambda message: self._receive_cloud(generation, message), self.nodes._sensor_qos,
                    callback_group=MutuallyExclusiveCallbackGroup())
                self._watchdog = self.nodes.robot.create_timer(
                    1.0, lambda: self._watch_source(generation))
            except Exception:
                self._stop_locked()
                raise

    def _watch_source(self, generation):
        with self._lock:
            if not self._running or generation != self._generation:
                return
            topic = self._active_config["cloud_topic"]
            transport = self._input_transport
        try:
            publishers = (len(self.nodes.robot.get_publishers_info_by_topic(topic))
                          if transport == "ros" else None)
        except RuntimeError:
            publishers = None
        with self._lock:
            if not self._running or generation != self._generation:
                return
            self._publisher_count = publishers
            age = time.monotonic() - (self._last_cloud_at if self._last_cloud_at is not None
                                      else self._started_at)
            if age <= SOURCE_TIMEOUT_S:
                self._source_state = "live" if self._last_cloud_at is not None else "waiting"
                return
            self._source_state = "stale" if self._last_cloud_at is not None or publishers != 0 else "no_publisher"
            if self._error_code not in INPUT_ERROR_CODES:
                return
            if self._input_transport == "ssh":
                if self._error_code not in ("CLOUD_RELAY_FAILED", "CLOUD_RELAY_STALE"):
                    self._error_code = "CLOUD_RELAY_DATA_TIMEOUT"
                    self._last_error = f"No fresh SSH cloud received from {topic} for more than {SOURCE_TIMEOUT_S:g} seconds"
                return
            if publishers == 0:
                self._error_code = "CLOUD_PUBLISHER_UNAVAILABLE"
                self._last_error = (f"No visible publisher for {topic} in the driver ROS domain. "
                    "Check robot-side DDS publishing scope and cross-host reachability; "
                    "a registered topic alone does not prove its publisher is reachable.")
            else:
                self._error_code = "CLOUD_DATA_TIMEOUT"
                self._last_error = f"No valid cloud received from {topic} for more than {SOURCE_TIMEOUT_S:g} seconds"
            # No factory settings change: the remote process only subscribes.
            relay = CloudRelay(self._relay_config, topic, self._active_config["rate_hz"],
                lambda message: self._receive_cloud(generation, message, "ssh"),
                lambda code, reason: self._relay_error(generation, code, reason))
            self._relay, self._input_transport = relay, "ssh"
            try:
                relay.start()
            except (OSError, RuntimeError):
                relay.close()
                self._relay, self._input_transport = None, "ros"
                self._error_code, self._last_error = "CLOUD_RELAY_FAILED", "Could not start the SSH cloud reader"

    def _relay_error(self, generation, code, reason):
        with self._lock:
            if self._running and generation == self._generation and self._input_transport == "ssh":
                self._error_code, self._last_error = code, reason
                self._source_state = "stale"

    def _drop(self, reason, code="INVALID_CLOUD"):
        self._stats["dropped"] += 1
        self._last_error = reason
        self._error_code = code

    def _receive_cloud(self, generation, message, transport="ros"):
        receipt_stamp = divmod(time.time_ns(), 10**9)
        received_monotonic = time.monotonic()
        with self._lock:
            if not self._running or generation != self._generation or transport != self._input_transport:
                return
            self._stats["received"] += 1
            try:
                if not self.controller.active("camera_front"):
                    raise ValueError("camera_front undistort is disabled")
                if self._worker is None or not self._worker.is_alive():
                    raise ValueError("depth worker is not running; restart the card")
                header = message.header
                self._observed_frame = header.frame_id
                config = self._active_config
                if header.frame_id != config["cloud_frame_id"]:
                    raise DepthError("CLOUD_FRAME_MISMATCH",
                        f"unexpected cloud frame {header.frame_id!r}; expected {config['cloud_frame_id']!r}")
                sec, nsec = header.stamp.sec, header.stamp.nanosec
                if type(sec) is not int or type(nsec) is not int or sec < 0 or not 0 <= nsec < 1_000_000_000:
                    raise ValueError("invalid lidar source stamp")
                stamp = sec * 1_000_000_000 + nsec
                if stamp <= 0 or (self._last_stamp is not None and stamp <= self._last_stamp):
                    raise ValueError("missing, repeated or backward lidar stamp; restart after a source clock reset")
                view = memoryview(message.data)
                if view.nbytes > MAX_DATA_BYTES:
                    raise ValueError("point cloud exceeds 64 MiB")
                source = {"fields": [(f.name, f.offset, f.datatype, f.count) for f in message.fields],
                          "is_bigendian": message.is_bigendian, "width": message.width, "height": message.height,
                          "point_step": message.point_step, "row_step": message.row_step, "data": view}
                validate_cloud_layout(source)
                now = time.monotonic()
                self._last_cloud_at = now
                self._source_state = "live"
                if self._error_code in INPUT_ERROR_CODES:
                    self._last_error = self._error_code = None
                if now - self._last_submit < 1 / config["rate_hz"]:
                    self._stats["throttled"] += 1
                    return
                calibration, _ = self.controller.calibration("camera_front")
                camera_generation = self.controller.generation('camera_front')
                frame = self.nodes.front_camera_frame()
                if (frame is None or frame[0] != camera_generation
                        or frame[1] is None or abs(now - frame[1]) > MAX_RGB_RECEIPT_AGE_S):
                    raise DepthError('FRONT_RGB_UNAVAILABLE', 'Waiting for a fresh rectified front-camera image')
                source["data"] = bytes(view)
                job = {
                    "generation": generation, "camera_generation": camera_generation,
                    "stamp": receipt_stamp, "source_stamp": (sec, nsec), "received_monotonic": received_monotonic,
                    "jpeg": frame[2], "rgb_received_monotonic": frame[1],
                    "cloud": source,
                    "K": depth_intrinsics(calibration), "transform": config["camera_optical_from_cloud"],
                    "min_depth_m": config["min_depth_m"], "max_depth_m": config["max_depth_m"],
                }
                if self._worker.submit(job):
                    self._last_stamp, self._last_submit = stamp, now
                else:
                    self._drop("depth queue is busy; newest frame dropped")
            except (ValueError, TypeError, AttributeError) as exc:
                self._drop(str(exc)[:300], getattr(exc, "code", "INVALID_CLOUD"))

    def _worker_error(self, context, reason):
        with self._lock:
            if self._running and context["generation"] == self._generation:
                self._drop(str(reason)[:300])

    def _receive_result(self, context, packet, stats):
        with self._lock:
            if not self._running or context["generation"] != self._generation:
                return
            if (not self.controller.active("camera_front") or
                    context["camera_generation"] != self.controller.generation("camera_front")):
                self._drop("front-camera geometry changed; old depth frame discarded")
                return
            if time.monotonic() - context["received_monotonic"] > MAX_RESULT_AGE_S:
                self._drop("depth result is stale")
                return

            published = False

            def publish():
                nonlocal published
                from sensor_msgs.msg import CompressedImage
                output = CompressedImage()
                output.header.frame_id = self._optical_frame
                output.header.stamp.sec, output.header.stamp.nanosec = context["stamp"]
                output.format, output.data = ROS_FORMAT, packet
                self._publisher.publish(output)
                self._stats.update(stats)
                self._stats["receive_to_enqueue_ms"] = round(
                    (time.monotonic() - context["received_monotonic"]) * 1000, 1)
                self._stats["published"] += 1
                self._stats["last_source_stamp"] = list(context["source_stamp"])
                self._last_error = None
                self._error_code = None
                published = True
            # Hold both guards through enqueue so a stop/config/undistort
            # change cannot let an older generation slip into the DDS queue.
            self.controller.publish_if_current("camera_front", context["camera_generation"], publish,
                                               clear_error=False)
            if not published:
                self._drop("front-camera geometry changed; old depth frame discarded")

    def _configure(self, values):
        with self._lifecycle:
            with self._lock:
                candidate = _settings({**self._config, **values})
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temp = None
            try:
                with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.path.parent,
                                                 prefix=self.path.name + ".", delete=False) as stream:
                    temp = stream.name
                    json.dump({"version": 1, "config": candidate}, stream, allow_nan=False)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temp, self.path)
            finally:
                if temp is not None:
                    Path(temp).unlink(missing_ok=True)
            self._stop_locked()
            with self._lock:
                self._config = candidate
                self._active_config = None
                self._extrinsics_source = "unknown"
                self._optical_frame = FRAME
                self._last_error = None
                self._error_code = None

    def _info(self):
        executor = self.nodes.robot_executor_status() if self.nodes is not None else None
        with self._lock:
            if (self._running and self._error_code == "ROS_EXECUTOR_UNAVAILABLE"
                    and executor is not None and executor["state"] == "running"):
                self._last_error = self._error_code = None
            state = "running" if self._running else "idle"
            error = self._last_error
            code = self._error_code
            if self._running and (self._worker is None or not self._worker.is_alive()):
                error = "depth worker is not running; restart the card"
            if self._running and not self.controller.active("camera_front"):
                error = "camera_front undistort is disabled"
            if self._running and executor is not None and executor["state"] != "running":
                error, code = executor["error"], "ROS_EXECUTOR_UNAVAILABLE"
            if error:
                state = "error"
            result = {"state": state, "config": copy.deepcopy(self._active_config or self._config),
                      "stats": copy.deepcopy(self._stats), "last_error": error,
                      "error_code": code,
                      "topic_out": self._topic_outputs()}
            result["input"] = {"state": self._source_state,
                "executor": executor,
                "transport": self._input_transport,
                "relay": self._relay.status() if self._relay is not None else None,
                "cloud_topic": result["config"]["cloud_topic"],
                "observed_frame_id": self._observed_frame,
                "expected_frame_id": result["config"]["cloud_frame_id"],
                "publisher_count": self._publisher_count, "timeout_s": SOURCE_TIMEOUT_S,
                "last_valid_cloud_age_s": (None if self._last_cloud_at is None else
                                           max(0., time.monotonic() - self._last_cloud_at))}
        result["camera_info"] = [self._camera_info()]
        result["calibration_warning"] = result["camera_info"][0]["vendor"]["calibration_warning"]
        return result

    def dispatch(self, action, args):
        try:
            if action == "start":
                self._start_capture()
            elif action == "stop":
                self.stop()
            elif action == "config":
                nested = args.get("config", {})
                if not isinstance(nested, dict):
                    raise ValueError("config must be an object")
                # Core attaches instance_id to canvas configuration calls;
                # this single-instance card must not persist it as a setting.
                values = {**nested, **{k: v for k, v in args.items()
                                      if k not in {"config", "_tool_name", "instance_id"}}}
                self._configure(values)
                return {**self._info(), "state": "configured", "restart_required": True}
            elif action not in ("info", NAME):
                return None
            return self._info()
        except (ValueError, TypeError, OSError, RuntimeError, ImportError) as exc:
            code = getattr(exc, "code", "DEPTH_START_FAILED" if action == "start" else "CONFIGURATION_INVALID")
            if action == "start":
                with self._lock:
                    self._last_error, self._error_code = str(exc), code
            return {"state": "error", "code": code, "message": str(exc)}
