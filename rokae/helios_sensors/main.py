"""Loopback MCP bundle publishing the two native Gemini sensor streams."""

from array import array
import json
import multiprocessing as mp
import os
import queue
import re
import signal
import threading
import time
from contextlib import closing
from http.server import ThreadingHTTPServer

from common import logsafe
from common.camera_info import build as camera_info
from common.vendor_runtime import DriverBundle, make_handler, start_registration, tool

from .depth_preview import depth_preview_jpeg
from .depth_socket import from_depth_socket
from .orbbec import DEPTH_FORMAT, DEPTH_FPS, HEIGHT, IMU_HZ, WIDTH, capture


DRIVER_ID = "rokae-helios-sensors-driver"
DRIVER_NAME = "ROKAE Helios-2.1 Camera Sensors"
DEFAULT_PORT = 15750
DEPTH_HINT = "image/depth-zlib"
DEPTH_MAX_FRAME_AGE = .2
IMU_FIELDS = {**{f"accel_{axis}_m_s2": None for axis in "xyz"},
              **{f"gyro_{axis}_rad_s": None for axis in "xyz"},
              **{f"orientation_{axis}": None for axis in "xyzw"},
              "orientation_source": None,
              "orientation_reference": None}


class SensorCard:
    def __init__(self, kind, serial, directory, node, namespace, frame_id, depth_socket=""):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String

        self.kind, self.serial, self.directory = kind, serial, directory
        self.depth_socket = depth_socket
        self.node, self.frame_id = node, frame_id
        self.name = "imu" if kind == "imu" else "camera_depth"
        topic = f"/{namespace}/state/imu" if kind == "imu" else f"/{namespace}/camera/depth"
        self.topic = {"topic": topic, "format": "data/json" if kind == "imu" else DEPTH_HINT}
        self.message_type = String if kind == "imu" else CompressedImage
        self.publisher = node.create_publisher(self.message_type, topic, qos_profile_sensor_data)
        self.preview_topic = None
        if kind == "depth":
            self.preview_topic = {"topic": f"{topic}/preview", "format": "image/jpeg"}
            self.preview = node.create_publisher(
                CompressedImage, self.preview_topic["topic"],
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.lock = threading.RLock()
        self.thread = None
        self.stop_capture = threading.Event()
        self.received = self.values = self.device_stamp = self.intrinsics = None
        self.error = None
        self.count = 0
        self.last_publish = 0
        self.worker_context = mp.get_context("spawn")

    def get_tool(self):
        if self.kind == "imu":
            description = (
                "Helios 相机 IMU。读取 Gemini 335Lg 内置传感器，输出相机坐标系下同步采样的"
                "三轴加速度（m/s²）和三轴角速度（rad/s）。"
                f"{IMU_HZ} Hz 采集，约 10 Hz 发布。用加速度计与陀螺仪估算姿态四元数（XYZW）。"
                "横滚和俯仰参考重力，偏航以初始化航向为零点，随时间可能漂移。"
                "初始化完成前四元数留空，停流或过期时清空测量值。"
            )
        else:
            description = (
                "Helios 相机深度。读取 Gemini 335Lg 原生深度图，"
                f"分辨率 {WIDTH}×{HEIGHT}，{DEPTH_FPS} 帧/秒，每个像素的深度单位为毫米。"
                "彩色 JPEG 用于低带宽预览，毫米深度数据中的 0 表示无有效测量。"
                "深度图采用相机光学坐标系，内参缺失时留空。"
            )
        return tool(self.name, "sensor", description,
                    {"type": "object", "properties": {
                        "action": {"type": "string", "enum": ["start", "stop", "info"]}}},
                    topic_out=self.output_topics())

    def output_topics(self):
        return [self.topic, self.preview_topic] if self.preview_topic else [self.topic]

    def payload(self):
        with self.lock:
            age = None if self.received is None else time.monotonic() - self.received
            if not self.serial:
                state = "unconfigured"
            elif self.thread is None or self.stop_capture.is_set():
                state = "idle"
            elif self.error:
                state = "reconnecting"
            elif age is None:
                state = "waiting"
            else:
                state = "online" if age < 1 else "stale"
            result = {"state": state, "available": state == "online",
                      "frame_id": self.frame_id, "frame_count": self.count,
                      "age_ms": None if age is None else round(age * 1000),
                      "device_stamp_us": self.device_stamp if state == "online" else None,
                      "timestamp_ms": int(time.time() * 1000),
                      "error": self.error, "topic_out": self.output_topics()}
            if self.kind == "imu":
                result.update(IMU_FIELDS)
                if state == "online":
                    result.update(self.values)
                result.update({"source_hz": IMU_HZ, "publish_hz": 10})
            else:
                result.update({"width": WIDTH, "height": HEIGHT, "fps": DEPTH_FPS, "unit": "mm"})
            return result

    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            # The canvas asks for an instance's topic before opening the data
            # view. Resume passive preview if Core stopped the previous card.
            if args.get("instance_id"):
                self.start()
            with self.lock:
                result = {**self.get_tool(), **self.payload()}
                if self.kind == "depth":
                    result["preview_topic"] = self.preview_topic["topic"]
                    result["camera_info"] = [camera_info(
                        topic=self.topic["topic"], id="rokae/helios-2.1/gemini335lg-depth",
                        format=DEPTH_HINT, width=WIDTH, height=HEIGHT,
                        K=self.intrinsics, source="derived-from-K" if self.intrinsics else "unknown",
                        pipeline=("native", "mm", "zlib"))]
            return result
        if action in ("read", self.name):
            return self.payload()
        return None

    def publish_status(self):
        if self.kind == "imu":
            msg = self.message_type()
            msg.data = json.dumps(self.payload(), allow_nan=False)
            self.publisher.publish(msg)

    def start(self):
        with self.lock:
            if not self.serial:
                return
            previous = self.thread if self.stop_capture.is_set() else None
        if previous is not None:
            previous.join(timeout=3)
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                if self.stop_capture.is_set():
                    raise RuntimeError("camera is still stopping")
                return
            self.stop_capture = threading.Event()
            self.received = self.values = self.device_stamp = self.intrinsics = self.error = None
            self.count = 0
            self.last_publish = 0
            self.thread = threading.Thread(target=self.run, name=self.name, daemon=True)
            self.thread.start()
            self.publish_status()

    def stop(self):
        with self.lock:
            self.stop_capture.set()
            self.received = self.values = self.device_stamp = self.error = None
            self.publish_status()

    def publish_frame(self, received, stamp, data, geometry):
        with self.lock:
            frame_age_limit = DEPTH_MAX_FRAME_AGE if self.kind == "depth" else 1
            if self.stop_capture.is_set() or time.monotonic() - received >= frame_age_limit:
                return
            self.received, self.device_stamp = received, stamp
            self.count += 1
            self.error = None
            if self.kind == "imu":
                accel, gyro, orientation = data
                self.values = {**{f"accel_{axis}_m_s2": v for axis, v in zip("xyz", accel)},
                               **{f"gyro_{axis}_rad_s": v for axis, v in zip("xyz", gyro)}}
                if orientation is not None:
                    self.values.update({f"orientation_{axis}": v for axis, v in zip("xyzw", orientation)})
                    self.values["orientation_source"] = "accel_gyro_fusion"
                    self.values["orientation_reference"] = "gravity_up_initial_heading"
                if received - self.last_publish >= .095:
                    self.publish_status()
                    self.last_publish = received
            else:
                self.intrinsics = geometry
                msg = self.message_type()
                msg.header.frame_id = self.frame_id
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.format = DEPTH_FORMAT
                msg.data = array("B", data)
                self.publisher.publish(msg)
                jpeg = self.message_type()
                jpeg.header = msg.header
                jpeg.format = "jpeg"
                jpeg.data = array("B", depth_preview_jpeg(data))
                self.preview.publish(jpeg)

    def run_capture(self):
        child_stop = self.worker_context.Event()
        output = self.worker_context.Queue(maxsize=1)
        if self.depth_socket:
            target, args = from_depth_socket, (self.depth_socket, child_stop, output)
        else:
            target, args = capture, (self.kind, self.directory, self.serial, child_stop, output)
        child = self.worker_context.Process(target=target, args=args, daemon=True)
        try:
            child.start()
            deadline = time.monotonic() + 12
            warned_stale = False
            while not self.stop_capture.is_set():
                if time.monotonic() >= deadline:
                    raise TimeoutError("no fresh camera frames")
                try:
                    item = output.get(timeout=.2)
                except queue.Empty:
                    if not child.is_alive():
                        raise RuntimeError(f"camera capture exited ({child.exitcode})")
                    with self.lock:
                        if (not warned_stale and not self.stop_capture.is_set()
                                and self.payload()["state"] == "stale"):
                            warned_stale = True
                            self.publish_status()
                    continue
                if item[0] == "error":
                    raise RuntimeError(item[1])
                self.publish_frame(*item[1:])
                deadline = time.monotonic() + 4
                warned_stale = False
        finally:
            child_stop.set()
            if child.pid is not None:
                child.join(timeout=1.5)
                if child.is_alive():
                    child.kill()
                    child.join(timeout=1)
            output.close()

    def run(self):
        while not self.stop_capture.is_set():
            try:
                self.run_capture()
            except (OSError, RuntimeError, ValueError) as exc:
                # OSError includes capture timeouts, so a stalled camera also retries.
                with self.lock:
                    if self.stop_capture.is_set():
                        return
                    self.error = str(exc)[:300]
                    self.received = self.values = self.device_stamp = None
                    self.publish_status()
                self.stop_capture.wait(3)


def camera_serial(name):
    serial = os.getenv(name, "").strip()
    if serial and not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", serial):
        raise ValueError(f"invalid {name}")
    return serial


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    logsafe.install()
    imu_serial = camera_serial("HELIOS_ORBBEC_IMU_SERIAL")
    depth_serial = camera_serial("HELIOS_ORBBEC_DEPTH_SERIAL")
    if imu_serial and imu_serial == depth_serial:
        raise ValueError("configure different cameras for independent IMU and depth capture")
    directory = os.getenv("HELIOS_ORBBEC_SDK_DIR", "/opt/orbbec-sdk")
    topic_namespace = os.getenv("HELIOS_SENSOR_NAMESPACE", "rokae_helios")
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", topic_namespace):
        raise ValueError("invalid HELIOS_SENSOR_NAMESPACE")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("helios_orbbec_sensors", start_parameter_services=False)
    plugins = [
        SensorCard("imu", imu_serial, directory, node, topic_namespace,
                   os.getenv("HELIOS_ORBBEC_IMU_FRAME_ID", "helios_gemini_imu_frame")),
        SensorCard("depth", depth_serial, directory, node, topic_namespace,
                   os.getenv("HELIOS_ORBBEC_DEPTH_FRAME_ID", "helios_gemini_depth_optical_frame"),
                   os.getenv("HELIOS_ORBBEC_DEPTH_SOCKET", "")),
    ]
    bundle = DriverBundle(plugins)
    port = int(os.getenv("HELIOS_SENSOR_MCP_PORT", str(DEFAULT_PORT)))
    with closing(ThreadingHTTPServer(("127.0.0.1", port), make_handler(
            lambda: bundle, DRIVER_NAME, DRIVER_ID))) as server:
        def shutdown(_signal, _frame):
            threading.Thread(target=server.shutdown, daemon=True).start()

        signal.signal(signal.SIGTERM, shutdown)
        signal.signal(signal.SIGINT, shutdown)
        try:
            bundle.start_all()
            start_registration(port, {"name": DRIVER_NAME}, DRIVER_ID)
            server.serve_forever()
        finally:
            bundle.stop_all()
            for card in plugins:
                if card.thread:
                    card.thread.join(timeout=5)
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
