"""Process-isolated FastDDS publisher for the A3 core-domain streams."""
from __future__ import annotations

import multiprocessing as mp
import os
import queue
import threading


def _type_name(msg_type):
    package = msg_type.__module__.split(".")[0]
    names = {
        "String": "std_msgs/msg/String", "UInt8MultiArray": "std_msgs/msg/UInt8MultiArray",
        "CompressedImage": "sensor_msgs/msg/CompressedImage", "Image": "sensor_msgs/msg/Image",
        "PointCloud2": "sensor_msgs/msg/PointCloud2", "AudioCapture": "audio_msgs/msg/AudioCapture",
        "AudioChunk": "audio_msgs/msg/AudioChunk",
    }
    return names.get(msg_type.__name__, f"{package}/msg/{msg_type.__name__}")


class CoreBridge:
    def __init__(self, profile="/opt/phanthy-motus/dds-local.xml", domain=42):
        self._ctx = mp.get_context("spawn")
        self._queues = {
            **{f"media{i}": self._ctx.Queue(maxsize=1) for i in range(12)},
            "pointcloud": self._ctx.Queue(maxsize=1),
            # Audio is an ordered waveform, not a replaceable video frame.
            # Buffer short scheduling bursts; on sustained overload, count and
            # drop the oldest chunk to bound latency.
            # Keep only a short real-time audio tail. A long FIFO makes the
            # dashboard play seconds-old PCM after the bridge is overloaded.
            "audio_mic": self._ctx.Queue(maxsize=4),
            "audio_ext": self._ctx.Queue(maxsize=4),
            # These lanes publish latest-state data. Separate the two combined
            # joint outputs and IMU from one another so maxsize=1 cannot make
            # unrelated topics overwrite each other.
            "state_joints": self._ctx.Queue(maxsize=1),
            "state_joint_state": self._ctx.Queue(maxsize=1),
            "state_imu": self._ctx.Queue(maxsize=1),
            "state_battery": self._ctx.Queue(maxsize=1),
            "state_estop": self._ctx.Queue(maxsize=1),
            "state_motor": self._ctx.Queue(maxsize=1),
            "state_system": self._ctx.Queue(maxsize=16),
        }
        self._profile = profile
        self._domain = domain
        self._procs = []
        self._sent = 0
        self._dropped = 0
        self._topic_counts = {}
        self._metrics_lock = threading.Lock()
        self._subscriptions = set()

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
                    ("head_rear", "media2"),
                    ("head_stereo_left", "media3"),
                    ("head_stereo_right", "media4"),
                    ("armpit_right", "media5"),
                    ("chest_front_d457_rgb", "media6"),
                    ("chest_front_d457_depth", "media7"),
                    ("waist_front_d415_rgb", "media8"),
                    ("waist_front_d415_depth", "media9"),
                    ("wrist_left_d405", "media10"),
                    ("wrist_right_d405", "media11"),
                )
                lane = next((value for marker, value in camera_lane if marker in topic),
                             f"media{sum(topic.encode('utf-8')) % 12}")
            elif "audio" in topic or "mic" in topic:
                lane = "audio_ext" if "ext_mic" in topic else "audio_mic"
            elif topic.endswith("/state/joints"):
                lane = "state_joints"
            elif topic.endswith("/state/joint_state"):
                lane = "state_joint_state"
            elif topic.endswith("/state/imu"):
                lane = "state_imu"
            elif topic.endswith(("/arm_state", "/hand_state", "/neck_state", "/leg_state")):
                lane = "state_motor"
            elif topic.endswith("/battery"):
                lane = "state_battery"
            elif topic.endswith("/estop"):
                lane = "state_estop"
            elif topic.endswith("/skill_status"):
                lane = "state_system"
            else:
                lane = "state_system"
            item = (topic, type_name, serialize_message(msg))
            try:
                self._queues[lane].put_nowait(item)
            except queue.Full:
                # Video, point clouds and latest-state snapshots may replace
                # stale pending values. Audio instead preserves FIFO order.
                try:
                    if lane.startswith("audio_"):
                        self._queues[lane].get_nowait()
                        with self._metrics_lock:
                            self._dropped += 1
                    else:
                        self._queues[lane].get_nowait()
                except queue.Empty:
                    pass
                self._queues[lane].put_nowait(item)
                if not lane.startswith("audio_"):
                    with self._metrics_lock:
                        self._dropped += 1
            with self._metrics_lock:
                self._sent += 1
                count = self._topic_counts.get(topic, 0) + 1
                self._topic_counts[topic] = count
                sent, dropped = self._sent, self._dropped
            log_interval = 300 if "camera" in topic else 5000
            if count == 1 or count % log_interval == 0:
                print(f"[dds-bridge] enqueued_total={self._sent} replaced_stale={self._dropped} "
                      f"lane={lane} topic={topic} topic_count={count}", flush=True)
        except queue.Full:
            with self._metrics_lock:
                self._dropped += 1
        except Exception as exc:
            print(f"[dds-bridge] enqueue failed topic={topic}: {exc}", flush=True)

    def subscribe(self, topic, msg_type, callback, qos=5):
        """Receive a domain-42 topic in an isolated loopback-profiled process."""
        subscription = _BridgeSubscription(
            self._ctx, self._profile, self._domain, topic, msg_type, callback, qos)
        subscription.start()
        self._subscriptions.add(subscription)
        return subscription

    def unsubscribe(self, subscription):
        if subscription in self._subscriptions:
            subscription.stop()
            self._subscriptions.discard(subscription)

    def stop(self):
        for subscription in tuple(self._subscriptions):
            subscription.stop()
        self._subscriptions.clear()
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


class _BridgeSubscription:
    def __init__(self, context, profile, domain, topic, msg_type, callback, qos):
        self._ctx = context
        self._profile = profile
        self._domain = domain
        self._topic = topic
        self._msg_type = msg_type
        self._callback = callback
        self._qos = qos
        self._messages = context.Queue(maxsize=8)
        self._stop = context.Event()
        self._process = context.Process(
            target=_run_subscription,
            args=(self._messages, self._stop, profile, domain, topic,
                  _type_name(msg_type), qos),
            name="a3-core-domain-bridge-subscription", daemon=True)
        self._thread = threading.Thread(target=self._dispatch, daemon=True,
                                        name="a3-core-domain-bridge-dispatch")

    def start(self):
        self._process.start()
        self._thread.start()
        print(f"[dds-bridge] subscription started topic={self._topic} "
              f"pid={self._process.pid} domain={self._domain}", flush=True)

    def _dispatch(self):
        from rclpy.serialization import deserialize_message
        while not self._stop.is_set():
            try:
                payload = self._messages.get(timeout=0.1)
            except queue.Empty:
                continue
            if payload is None:
                return
            try:
                self._callback(deserialize_message(payload, self._msg_type))
            except Exception as exc:
                print(f"[dds-bridge] subscription callback failed topic={self._topic}: {exc}",
                      flush=True)

    def stop(self):
        self._stop.set()
        try:
            self._messages.put_nowait(None)
        except queue.Full:
            pass
        self._process.join(timeout=2)
        if self._process.is_alive():
            self._process.terminate()
        self._thread.join(timeout=1)


def _run_subscription(messages, stop, profile, domain, topic, type_name, qos_depth):
    try:
        from common import logsafe
        logsafe.install()
    except ImportError:
        pass
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    if os.path.isfile(profile):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, UInt8MultiArray
    types = {"sensor_msgs/msg/JointState": JointState,
             "std_msgs/msg/String": String,
             "std_msgs/msg/UInt8MultiArray": UInt8MultiArray}
    try:
        from audio_msgs.msg import AudioChunk
        types["audio_msgs/msg/AudioChunk"] = AudioChunk
    except ImportError:
        pass
    msg_type = types.get(type_name)
    if msg_type is None:
        print(f"[dds-bridge] unsupported subscription type={type_name} topic={topic}",
              flush=True)
        return
    rclpy.init()
    node = Node("agibot_a3_core_bridge_subscription")
    qos = QoSProfile(depth=int(qos_depth), reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(msg_type, topic,
                             lambda msg: messages.put(serialize_message(msg)), qos)
    print(f"[dds-bridge] subscriber created topic={topic} type={type_name}", flush=True)
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _run(messages, profile, domain, lane):
    try:
        from common import logsafe
        logsafe.install()
    except ImportError:
        pass
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
    published = {}
    types = {}
    for package, names in (("std_msgs.msg", ("String", "UInt8MultiArray")),
                           ("sensor_msgs.msg", ("CompressedImage", "Image", "PointCloud2", "JointState")),
                           ("audio_msgs.msg", ("AudioCapture", "AudioPlayback", "AudioChunk"))):
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
                # The bridge node has no subscriptions; spinning an empty
                # worker is unnecessary.  More importantly, Humble can report
                # an invalid wait-set context while the parent is tearing down
                # sibling workers.  Do not turn a normal empty queue into a
                # traceback and an early worker exit.
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
            published[topic] = published.get(topic, 0) + 1
            log_interval = 300 if "camera" in topic else 1000
            if published[topic] == 1 or published[topic] % log_interval == 0:
                print(f"[dds-bridge] published={published[topic]} topic={topic}", flush=True)
            # This node only owns publishers, so no executor spin is needed.
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
