"""Process-isolated FastDDS publisher for the A3 core-domain streams."""
from __future__ import annotations

import multiprocessing as mp
import os
import queue
import time


def _type_name(msg_type):
    package = msg_type.__module__.split(".")[0]
    names = {
        "String": "std_msgs/msg/String", "UInt8MultiArray": "std_msgs/msg/UInt8MultiArray",
        "CompressedImage": "sensor_msgs/msg/CompressedImage", "Image": "sensor_msgs/msg/Image",
        "PointCloud2": "sensor_msgs/msg/PointCloud2", "AudioCapture": "audio_msgs/msg/AudioCapture",
    }
    return names.get(msg_type.__name__, f"{package}/msg/{msg_type.__name__}")


class CoreBridge:
    def __init__(self, profile="/opt/phanthy-motus/dds-local.xml", domain=42):
        self._queue = mp.get_context("spawn").Queue(maxsize=32)
        self._ctx = mp.get_context("spawn")
        self._profile = profile
        self._domain = domain
        self._proc = None

    def start(self):
        self._proc = self._ctx.Process(target=_run, args=(self._queue, self._profile, self._domain),
                                       name="a3-core-domain-bridge", daemon=True)
        self._proc.start()
        print(f"[dds-bridge] started pid={self._proc.pid} domain={self._domain}", flush=True)

    def publish(self, topic, msg, msg_type):
        try:
            from rclpy.serialization import serialize_message
            type_name = _type_name(msg_type)
            self._queue.put_nowait((topic, type_name, serialize_message(msg)))
        except (queue.Full, BrokenPipeError, OSError):
            pass

    def stop(self):
        if self._proc is None:
            return
        try:
            self._queue.put_nowait(None)
            self._proc.join(timeout=2)
        except Exception:
            pass
        if self._proc.is_alive():
            self._proc.terminate()
        self._proc = None


class BridgePublisher:
    def __init__(self, bridge, topic, msg_type):
        self.bridge = bridge
        self.topic = topic
        self.msg_type = msg_type
        self.topic_name = topic

    def publish(self, msg):
        self.bridge.publish(self.topic, msg, self.msg_type)


def _run(messages, profile, domain):
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    if os.path.isfile(profile):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = Node("agibot_a3_core_bridge")
    pubs = {}
    types = {}
    for package, names in (("std_msgs.msg", ("String", "UInt8MultiArray")),
                           ("sensor_msgs.msg", ("CompressedImage", "Image", "PointCloud2")),
                           ("audio_msgs.msg", ("AudioCapture",))):
        try:
            module = __import__(package, fromlist=list(names))
        except ImportError:
            continue
        for name in names:
            msg_type = getattr(module, name, None)
            if msg_type is not None:
                types[_type_name(msg_type)] = msg_type
    from rclpy.serialization import deserialize_message
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    try:
        while rclpy.ok():
            try:
                item = messages.get(timeout=0.05)
            except queue.Empty:
                rclpy.spin_once(node, timeout_sec=0.0)
                continue
            if item is None:
                break
            topic, type_name, payload = item
            msg_type = types.get(type_name)
            if msg_type is None:
                continue
            msg = deserialize_message(payload, msg_type)
            pub = pubs.get(topic)
            if pub is None:
                pub = node.create_publisher(msg_type, topic, qos)
                pubs[topic] = pub
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
