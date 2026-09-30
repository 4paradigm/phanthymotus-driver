"""Process-isolated FastDDS publisher for the A3 core-domain streams."""
from __future__ import annotations

import multiprocessing as mp
import os
import queue


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
        # Keep enough room for bursty images/point clouds. Frames are lossy at
        # the bridge boundary; a full queue drops the newest frame rather than
        # blocking the robot-domain subscription callback.
        self._queues = {lane: mp.get_context("spawn").Queue(maxsize=32)
                        for lane in ("media0", "media1", "media2", "media3",
                                     "pointcloud", "state", "audio")}
        self._ctx = mp.get_context("spawn")
        self._profile = profile
        self._domain = domain
        self._procs = []
        self._sent = 0

    def start(self):
        for lane, messages in self._queues.items():
            proc = self._ctx.Process(target=_run, args=(messages, self._profile, self._domain, lane),
                                     name=f"a3-core-domain-bridge-{lane}", daemon=True)
            proc.start()
            self._procs.append(proc)
        print(f"[dds-bridge] started workers={[p.pid for p in self._procs]} domain={self._domain}", flush=True)

    def publish(self, topic, msg, msg_type):
        try:
            from rclpy.serialization import serialize_message
            type_name = _type_name(msg_type)
            if "lidar" in topic or "pointcloud" in topic:
                lane = "pointcloud"
            elif "camera" in topic:
                # Keep each configured camera on its own process.  The explicit
                # names avoid accidental collisions between the four high-rate
                # streams (left/right head RGB, chest RGB, chest depth).
                camera_lane = (
                    ("head_left", "media0"),
                    ("head_right", "media1"),
                    ("chest_front_d457_rgb", "media2"),
                    ("chest_front_d457_depth", "media3"),
                )
                lane = next((value for marker, value in camera_lane if marker in topic),
                             f"media{sum(topic.encode('utf-8')) % 4}")
            elif "audio" in topic or "mic" in topic:
                lane = "audio"
            else:
                lane = "state"
            self._queues[lane].put_nowait((topic, type_name, serialize_message(msg)))
            self._sent += 1
            if self._sent == 1 or self._sent % 1000 == 0:
                print(f"[dds-bridge] queued={self._sent} topic={topic}", flush=True)
        except queue.Full:
            pass
        except Exception as exc:
            print(f"[dds-bridge] enqueue failed topic={topic}: {exc}", flush=True)

    def stop(self):
        if not self._procs:
            return
        try:
            for q in self._queues.values():
                try:
                    q.put_nowait(None)
                except queue.Full:
                    pass
            for proc in self._procs:
                proc.join(timeout=2)
        except Exception:
            pass
        for proc in self._procs:
            if proc.is_alive():
                proc.terminate()
        self._procs = []


class BridgePublisher:
    def __init__(self, bridge, topic, msg_type):
        self.bridge = bridge
        self.topic = topic
        self.msg_type = msg_type
        self.topic_name = topic

    def publish(self, msg):
        self.bridge.publish(self.topic, msg, self.msg_type)


def _run(messages, profile, domain, lane):
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    if os.path.isfile(profile):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = Node(f"agibot_a3_core_bridge_{lane}")
    pubs = {}
    types = {}
    for package, names in (("std_msgs.msg", ("String", "UInt8MultiArray")),
                           ("sensor_msgs.msg", ("CompressedImage", "Image", "PointCloud2", "JointState")),
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
    # Sensor consumers request BEST_EFFORT. Publishing BEST_EFFORT avoids a
    # reliable writer retaining large camera/point-cloud samples indefinitely.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    try:
        while rclpy.ok():
            try:
                item = messages.get(timeout=0.05)
            except queue.Empty:
                # Keep the node responsive while waiting for the next sample;
                # an empty queue is normal and must not terminate the worker.
                rclpy.spin_once(node, timeout_sec=0.0)
                continue
            if item is None:
                break
            topic, type_name, payload = item
            msg_type = types.get(type_name)
            if msg_type is None:
                print(f"[dds-bridge] unsupported type={type_name} topic={topic}", flush=True)
                continue
            try:
                msg = deserialize_message(payload, msg_type)
            except Exception as exc:
                print(f"[dds-bridge] deserialize failed topic={topic}: {exc}", flush=True)
                continue
            pub = pubs.get(topic)
            if pub is None:
                pub = node.create_publisher(msg_type, topic, qos)
                pubs[topic] = pub
                print(f"[dds-bridge] publisher created topic={topic} type={type_name}", flush=True)
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
