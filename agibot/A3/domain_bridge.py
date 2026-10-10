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
            if count == 1 or count % 5000 == 0:
                print(f"[dds-bridge] enqueued_total={self._sent} replaced_stale={self._dropped} "
                      f"lane={lane} topic={topic} topic_count={count}", flush=True)
        except queue.Full:
            with self._metrics_lock:
                self._dropped += 1
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
            if published[topic] == 1 or published[topic] % 1000 == 0:
                print(f"[dds-bridge] published={published[topic]} topic={topic}", flush=True)
            # This node only owns publishers, so no executor spin is needed.
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
