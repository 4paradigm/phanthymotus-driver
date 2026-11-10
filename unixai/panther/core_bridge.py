"""Process-isolated Agent Core publisher for Panther sensor cards.

The parent process talks to Panther on CycloneDDS/domain 0.  This child owns
FastDDS/domain 42 and the fleet loopback-only profile.  Only normalized sensor
payloads cross the multiprocessing queue; robot command topics never do.
"""

from __future__ import annotations

import json
import multiprocessing as mp
import os
import queue
import sys


TOPIC_SUFFIXES = {
    "joints": ("joints", "sensor/skeleton"),
    "robot_state": ("robot_state", "data/json"),
    "imu": ("imu", "data/json"),
    "odom": ("odom", "state/odom"),
    "laser_scan": ("laser_scan", "data/json"),
    "camera_rgb": ("camera/rgb", "image/jpeg"),
    "camera_depth": ("camera/depth", "image/depth-zlib"),
}


def output_topics(namespace: str) -> dict[str, dict[str, str]]:
    prefix = f"/{namespace}/panther"
    return {
        name: {"topic": f"{prefix}/{suffix}", "format": data_format}
        for name, (suffix, data_format) in TOPIC_SUFFIXES.items()
    }


class BridgeWorker:
    def __init__(self, namespace: str):
        self._ctx = mp.get_context("spawn")
        self._queue = self._ctx.Queue(maxsize=128)
        self._commands = self._ctx.Queue(maxsize=4)
        self._ready = self._ctx.Event()
        self._errors = self._ctx.Queue(maxsize=1)
        self._namespace = namespace
        self._process = None

    def start(self) -> None:
        if self._process is not None and self._process.is_alive():
            return
        self._process = self._ctx.Process(
            target=_run,
            args=(self._queue, self._commands, self._ready, self._errors, self._namespace),
            name="panther_core_bridge",
            daemon=True,
        )
        self._process.start()
        if not self._ready.wait(timeout=5.0):
            error = "core bridge did not become ready"
            try:
                error = self._errors.get_nowait()
            except queue.Empty:
                pass
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=2)
            raise RuntimeError(error)
        print(f"[bridge] Panther core bridge started pid={self._process.pid}", flush=True)

    def publish(self, card: str, payload) -> None:
        if card not in TOPIC_SUFFIXES:
            return
        item = {"card": card, "payload": payload}
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(item)
            except (queue.Empty, queue.Full):
                pass

    def stop(self) -> None:
        try:
            self._commands.put_nowait("shutdown")
        except queue.Full:
            pass
        if self._process is not None:
            self._process.join(timeout=5)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=2)
        print("[bridge] Panther core bridge stopped", flush=True)


def _run(data_queue, command_queue, ready, errors, namespace: str) -> None:
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except ImportError as exc:
        sys.stderr.write(f"[bridge] logsafe unavailable ({exc})\n")

    os.environ["ROS_DOMAIN_ID"] = os.environ.get("AGENT_CORE_ROS_DOMAIN_ID", "42")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    profile = "/opt/phanthy-motus/dds-local.xml"
    if os.path.isfile(profile):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
    else:
        message = f"required Agent Core DDS profile is missing: {profile}"
        try:
            errors.put_nowait(message)
        except queue.Full:
            pass
        raise RuntimeError(message)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    rclpy.init()
    node = Node("panther_core_bridge")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    topics = output_topics(namespace)
    publishers = {}
    for card, spec in topics.items():
        message_type = CompressedImage if spec["format"].startswith("image/") else String
        publishers[card] = node.create_publisher(message_type, spec["topic"], qos)
    print(
        f"[bridge] ready domain={os.environ['ROS_DOMAIN_ID']} profile={profile} "
        f"topics={len(publishers)}",
        flush=True,
    )
    ready.set()
    running = True
    while running and rclpy.ok():
        try:
            if command_queue.get_nowait() == "shutdown":
                running = False
        except queue.Empty:
            pass
        newest = {}
        while True:
            try:
                item = data_queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, dict) and item.get("card") in publishers:
                newest[item["card"]] = item.get("payload") or {}
        for card, payload in newest.items():
            spec = topics[card]
            if spec["format"].startswith("image/"):
                message = CompressedImage()
                message.format = str(payload.get("message_format", "jpeg"))
                message.data = payload.get("data", b"")
                message.header.frame_id = str(payload.get("frame_id", ""))
                stamp = payload.get("stamp") or {}
                message.header.stamp.sec = int(stamp.get("sec", 0))
                message.header.stamp.nanosec = int(stamp.get("nanosec", 0))
            else:
                message = String()
                message.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
            publishers[card].publish(message)
        rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_node()
    rclpy.shutdown()

