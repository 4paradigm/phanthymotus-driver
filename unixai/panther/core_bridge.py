"""Process-isolated, per-card Agent Core publishers for Panther."""

from __future__ import annotations

import json
import multiprocessing as mp
import os
import queue
import sys
import threading


TOPIC_SUFFIXES = {
    "joints": ("joints", "sensor/skeleton"),
    "robot_state": ("robot_state", "data/json"),
    "imu": ("imu", "data/json"),
    "odom": ("odom", "state/odom"),
    "laser_scan": ("laser_scan", "data/json"),
    "laser_view": ("laser_scan/view", "image/jpeg"),
    "camera_rgb": ("camera/rgb", "image/jpeg"),
    "camera_depth": ("camera/depth", "image/depth-zlib"),
}


def output_topics(namespace: str) -> dict[str, dict[str, str]]:
    prefix = f"/{namespace}/panther"
    return {
        name: {"topic": f"{prefix}/{suffix}", "format": data_format}
        for name, (suffix, data_format) in TOPIC_SUFFIXES.items()
    }


def _put_latest(messages, item) -> bool:
    try:
        messages.put_nowait(item)
        return True
    except queue.Full:
        pass
    try:
        messages.get_nowait()
    except queue.Empty:
        pass
    try:
        messages.put_nowait(item)
        return True
    except queue.Full:
        return False


class BridgeWorker:
    """Keep high-rate cards isolated so one stream cannot delay another."""

    def __init__(self, namespace: str):
        self._ctx = mp.get_context("spawn")
        self._namespace = namespace
        self._profile = "/opt/phanthy-motus/dds-local.xml"
        self._domain = int(os.environ.get("AGENT_CORE_ROS_DOMAIN_ID", "42"))
        self._lanes = {}
        self._lock = threading.RLock()
        self._started = False

    def start(self) -> None:
        if not os.path.isfile(self._profile):
            raise RuntimeError(f"required Agent Core DDS profile is missing: {self._profile}")
        self._started = True
        print("[bridge] Panther per-card bridge armed", flush=True)

    def _spawn(self, card: str):
        messages = self._ctx.Queue(maxsize=1)
        process = self._ctx.Process(
            target=_run_lane,
            args=(messages, self._profile, self._domain, self._namespace, card),
            name=f"panther_core_{card}",
            daemon=True,
        )
        process.start()
        previous = self._lanes.get(card)
        state = {
            "queue": messages, "process": process,
            "published": previous["published"] if previous else 0,
            "dropped": previous["dropped"] if previous else 0,
            "errors": previous["errors"] if previous else 0,
            "restarts": (previous["restarts"] + 1) if previous else 0,
            "last_error": previous["last_error"] if previous else None,
        }
        self._lanes[card] = state
        print(f"[bridge] started lane={card} pid={process.pid}", flush=True)
        return state

    def publish(self, card: str, payload) -> None:
        if card not in TOPIC_SUFFIXES:
            return
        with self._lock:
            if not self._started:
                return
            state = self._lanes.get(card)
            if state is None or not state["process"].is_alive():
                state = self._spawn(card)
            try:
                if _put_latest(state["queue"], payload):
                    state["published"] += 1
                else:
                    state["dropped"] += 1
            except Exception as exc:
                state["errors"] += 1
                state["last_error"] = f"{type(exc).__name__}: {exc}"
                if state["errors"] == 1 or state["errors"] % 100 == 0:
                    print(f"[bridge] enqueue failed lane={card}: {exc}", flush=True)

    def stats(self) -> dict:
        with self._lock:
            return {
                card: {
                    "alive": state["process"].is_alive(),
                    "published": state["published"], "dropped": state["dropped"],
                    "errors": state["errors"], "restarts": state["restarts"],
                    "last_error": state["last_error"],
                }
                for card, state in self._lanes.items()
            }

    def stop(self) -> None:
        with self._lock:
            self._started = False
            states = list(self._lanes.values())
        for state in states:
            _put_latest(state["queue"], None)
        for state in states:
            process = state["process"]
            process.join(timeout=3)
            if process.is_alive():
                process.terminate()
                process.join(timeout=2)
        print("[bridge] Panther per-card bridge stopped", flush=True)


def _run_lane(messages, profile: str, domain: int, namespace: str, card: str) -> None:
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except ImportError as exc:
        sys.stderr.write(f"[bridge] logsafe unavailable ({exc})\n")

    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    rclpy.init(domain_id=domain)
    node = Node(f"panther_core_{card}", enable_rosout=False, start_parameter_services=False)
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    spec = output_topics(namespace)[card]
    message_type = CompressedImage if spec["format"].startswith("image/") else String
    publisher = node.create_publisher(message_type, spec["topic"], qos)
    print(f"[bridge] ready lane={card} domain={domain} topic={spec['topic']}", flush=True)
    try:
        while True:
            payload = messages.get()
            if payload is None:
                break
            if message_type is CompressedImage:
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
            publisher.publish(message)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

