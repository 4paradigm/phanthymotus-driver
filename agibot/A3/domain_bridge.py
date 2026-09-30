"""Process-isolated FastDDS publisher for the A3 core-domain streams."""
from __future__ import annotations

import multiprocessing as mp
import os
import queue
import time


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

    def publish(self, topic, msg):
        try:
            self._queue.put_nowait((topic, msg))
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
    def __init__(self, bridge, topic):
        self.bridge = bridge
        self.topic = topic
        self.topic_name = topic

    def publish(self, msg):
        self.bridge.publish(self.topic, msg)


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
            topic, msg = item
            pub = pubs.get(topic)
            if pub is None:
                pub = node.create_publisher(type(msg), topic, qos)
                pubs[topic] = pub
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
