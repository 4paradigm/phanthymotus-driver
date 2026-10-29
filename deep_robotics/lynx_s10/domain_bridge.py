"""Process-isolated FastDDS domain-42 publisher for the Lynx S10 driver."""

from __future__ import annotations

import multiprocessing as mp
import os
import queue


_TYPE_NAMES = {
    "String": "std_msgs/msg/String",
    "UInt8MultiArray": "std_msgs/msg/UInt8MultiArray",
    "CompressedImage": "sensor_msgs/msg/CompressedImage",
}


class CoreBridge:
    """Publish serialized robot-domain samples from loopback-only processes."""

    def __init__(self, profile: str, domain: int = 42):
        self._ctx = mp.get_context("spawn")
        self._profile = profile
        self._domain = int(domain)
        self._queues = {}
        self._processes = {}
        self._counts = {}
        self._dropped = {}

    def create_publisher(self, lane: str, topic: str, msg_type):
        from rclpy.serialization import serialize_message

        if lane not in self._queues:
            messages = self._ctx.Queue(maxsize=1)
            process = self._ctx.Process(
                target=_run_publisher,
                args=(messages, self._profile, self._domain, lane),
                name=f"lynx-s10-domain42-{lane}", daemon=True,
            )
            process.start()
            self._queues[lane] = messages
            self._processes[lane] = process
            self._counts[lane] = 0
            self._dropped[lane] = 0
        return _BridgePublisher(self, lane, topic, msg_type, serialize_message)

    def publish(self, lane, topic, msg_type, serialize_message, message):
        try:
            item = (topic, _TYPE_NAMES[msg_type.__name__], serialize_message(message))
            messages = self._queues[lane]
            try:
                messages.put_nowait(item)
            except queue.Full:
                try:
                    messages.get_nowait()
                except queue.Empty:
                    pass
                messages.put_nowait(item)
                self._dropped[lane] += 1
            self._counts[lane] += 1
        except Exception as exc:
            print(f"[dds-bridge] enqueue failed lane={lane} topic={topic}: {exc}", flush=True)

    def stats(self, lane):
        return {"enqueued": self._counts.get(lane, 0),
                "dropped": self._dropped.get(lane, 0),
                "alive": bool(self._processes.get(lane) and self._processes[lane].is_alive())}

    def stop(self):
        for messages in self._queues.values():
            try:
                messages.put_nowait(None)
            except queue.Full:
                try:
                    messages.get_nowait()
                    messages.put_nowait(None)
                except queue.Empty:
                    pass
        for process in self._processes.values():
            process.join(timeout=2)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1)
        self._processes.clear()
        self._queues.clear()
        self._ctx = None


class _BridgePublisher:
    def __init__(self, bridge, lane, topic, msg_type, serialize_message):
        self._bridge = bridge
        self._lane = lane
        self.topic = topic
        self._msg_type = msg_type
        self._serialize = serialize_message

    def publish(self, message):
        self._bridge.publish(self._lane, self.topic, self._msg_type,
                             self._serialize, message)


def _run_publisher(messages, profile, domain, lane):
    try:
        from common import logsafe
        logsafe.install()
    except ImportError:
        pass
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String, UInt8MultiArray

    types = {"std_msgs/msg/String": String,
             "std_msgs/msg/UInt8MultiArray": UInt8MultiArray,
             "sensor_msgs/msg/CompressedImage": CompressedImage}
    rclpy.init(domain_id=domain)
    node = Node(f"lynx_s10_core_{lane}")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    publishers = {}
    counts = {}
    try:
        while rclpy.ok():
            try:
                item = messages.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            topic, type_name, payload = item
            msg_type = types.get(type_name)
            if msg_type is None:
                print(f"[dds-bridge] unsupported type={type_name} topic={topic}", flush=True)
                continue
            publisher = publishers.get(topic)
            if publisher is None:
                publisher = node.create_publisher(msg_type, topic, qos)
                publishers[topic] = publisher
                print(f"[dds-bridge] publisher created domain={domain} topic={topic} type={type_name}", flush=True)
            try:
                publisher.publish(deserialize_message(payload, msg_type))
                counts[topic] = counts.get(topic, 0) + 1
                if counts[topic] == 1:
                    print(f"[dds-bridge] first sample published domain={domain} topic={topic}", flush=True)
            except Exception as exc:
                print(f"[dds-bridge] publish failed topic={topic}: {exc}", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
