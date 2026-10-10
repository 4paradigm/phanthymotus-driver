#!/usr/bin/env python3
"""A3 host-side Jazzy -> Humble media relay.

This process deliberately deserializes the robot's large FastDDS samples while
running against the robot's Jazzy installation.  Only small frontend payloads
cross into the Humble domain: JPEG/depth-zlib images and the compact pointcloud
packet used by Phanthy Motus.
"""
from __future__ import annotations

import multiprocessing as mp
import os
import queue
import struct
import threading
import time
import zlib

CAMERAS = {
    "head_left_fisheye": "/hal/head_left_fisheye_camera/rgb",
    "head_right_fisheye": "/hal/head_right_fisheye_camera/rgb",
    "head_rear_fisheye": "/hal/head_rear_fisheye_camera/rgb",
    "head_stereo_left_fisheye": "/hal/head_stereo_left_fisheye_camera/rgb",
    "head_stereo_right_fisheye": "/hal/head_stereo_right_fisheye_camera/rgb",
    "armpit_right_fisheye": "/hal/armpit_right_fisheye_camera/rgb",
    "chest_front_d457_rgb": "/hal/chest_front_d457_camera/rgb",
    "chest_front_d457_depth": "/hal/chest_front_d457_camera/depth",
    "waist_front_d415_rgb": "/hal/waist_front_d415_camera/rgb",
    "waist_front_d415_depth": "/hal/waist_front_d415_camera/depth",
}

# The relay runs on the host, but inherits this environment from the driver
# container. Keep the default set aligned with config.yaml so unused raw
# cameras do not consume CPU. Operators can opt in to an additional key with
# A3_RELAY_STREAMS without changing the relay code.
_requested = {item.strip() for item in os.environ.get(
    "A3_RELAY_STREAMS",
    ",".join(CAMERAS),
).split(",") if item.strip()}
CAMERAS = {key: topic for key, topic in CAMERAS.items() if key in _requested}

_control_file = os.environ.get(
    "A3_RELAY_CONTROL_FILE", "/opt/phanthy-motus/data/a3-relay/active_streams"
)
_default_active = {item.strip() for item in os.environ.get(
    "A3_RELAY_ACTIVE_STREAMS",
    "",
).split(",") if item.strip()}


def _active_streams():
    """Read the driver's latest card activation set without restarting Jazzy."""
    try:
        with open(_control_file, "r", encoding="utf-8") as handle:
            requested = {line.strip() for line in handle if line.strip()}
        return requested & set(CAMERAS)
    except OSError:
        return _default_active & set(CAMERAS)


def _encode_image(msg, key):
    from sensor_msgs.msg import CompressedImage
    out = CompressedImage()
    if key.endswith("_depth"):
        import numpy as np
        width, height = int(msg.width), int(msg.height)
        step = int(msg.step or width * 2)
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        depth = raw[:step * height].reshape(height, step)[:, :width * 2].reshape(height, width, 2)
        out.format = "16UC1; compressedDepth zlib"
        out.data = list(zlib.compress(depth.view(np.uint16).reshape(height, width).tobytes(), 1))
    else:
        import cv2
        import numpy as np
        width, height = int(msg.width), int(msg.height)
        step = int(msg.step or width * 3)
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        image = raw[:step * height].reshape(height, step)[:, :width * 3].reshape(height, width, 3)
        if str(msg.encoding) == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # Full-resolution JPEG encoding for twelve cameras saturates the ADU
        # CPU and makes every callback arrive in bursts.  The dashboard only
        # needs a preview stream; bound the largest dimension before encoding.
        max_dimension = 384
        largest = max(image.shape[:2])
        if largest > max_dimension:
            scale = max_dimension / float(largest)
            image = cv2.resize(image, (max(1, int(image.shape[1] * scale)),
                                       max(1, int(image.shape[0] * scale))),
                               interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 30])
        if not ok:
            return None
        out.format = "jpeg"
        out.data = encoded.tobytes()
    out.header = msg.header
    return out


def _encode_cloud(msg):
    from std_msgs.msg import UInt8MultiArray
    import numpy as np
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        return None
    # PointCloud2 is normally dense and little-endian. A per-point Python loop
    # stalls the relay callback for tens of milliseconds and turns a 10 Hz
    # Livox stream into visible bursts. Build one structured view and stride
    # sample it in C/NumPy; retain a defensive scalar fallback for unusual
    # row/point layouts.
    try:
        count = int(msg.height) * int(msg.width)
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if int(msg.row_step) == int(msg.point_step) * int(msg.width):
            dtype = np.dtype({"names": ["x", "y", "z"],
                              "formats": ["<f4", "<f4", "<f4"],
                              "offsets": [fields[n].offset for n in ("x", "y", "z")],
                              "itemsize": int(msg.point_step)})
            view = np.ndarray((count,), dtype=dtype, buffer=raw[:count * int(msg.point_step)])
            points = np.column_stack((view["x"], view["y"], view["z"]))
            if len(points) > 20000:
                points = points[::max(1, len(points) // 20000)][:20000]
            points = points[np.isfinite(points).all(axis=1)]
        else:
            raise ValueError("non-dense PointCloud2 layout")
    except (ValueError, TypeError, IndexError):
        points = []
        data = bytes(msg.data)
        for row in range(int(msg.height)):
            for col in range(int(msg.width)):
                base = row * int(msg.row_step) + col * int(msg.point_step)
                try:
                    xyz = tuple(float(np.frombuffer(data, dtype=np.float32, count=1, offset=base + fields[n].offset)[0]) for n in ("x", "y", "z"))
                except (ValueError, IndexError):
                    continue
                if all(np.isfinite(xyz)):
                    points.append(xyz)
                if len(points) >= 20000:
                    break
            if len(points) >= 20000:
                break
        points = np.asarray(points, dtype=np.float32)
    points = np.asarray(points, dtype=np.float32)
    # The dashboard's sensor/pointcloud renderer maps its vertical axis as
    # y=-z.  Normalize the ROS lidar convention here so the rendered cloud is
    # upright instead of vertically mirrored.
    if len(points):
        points[:, 2] *= -1.0
    out = UInt8MultiArray()
    out.data = list(struct.pack("<II", 12, len(points)) + points.astype("<f4", copy=False).tobytes())
    return out


def _media_one(key, topic, msg_type, node=None, publisher=None, callback_group=None):
    """Subscribe and convert one raw stream in one Jazzy process.

    Keeping the raw sample inside the callback avoids a second serialization,
    a multiprocessing feeder queue, and a second deserialization before the
    converted (small) sample reaches the domain bridge.
    """
    os.environ["ROS_DOMAIN_ID"] = "232"
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image, PointCloud2
    from std_msgs.msg import UInt8MultiArray
    owns_node = node is None
    if owns_node:
        rclpy.init()
        node = Node(f"a3_jazzy_media_relay_{key}")
    output_topic = f"/agibot_a3/{'lidar_cloud' if key == 'lidar_cloud' else 'camera_' + key}"
    output_type = UInt8MultiArray if key == "lidar_cloud" else CompressedImage
    if publisher is None:
        publisher = node.create_publisher(output_type, output_topic, 1)
    received = 0

    def push(msg):
        nonlocal received
        try:
            received += 1
            if received == 1:
                print(f"[relay] input received key={key}", flush=True)
            if key == "lidar_cloud":
                output = _encode_cloud(msg)
            else:
                output = _encode_image(msg, key)
            if output is not None:
                publisher.publish(output)
        except Exception as exc:
            print(f"[relay] input failed key={key}: {type(exc).__name__}: {exc!r}", flush=True)
    kwargs = {"callback_group": callback_group} if callback_group is not None else {}
    node.create_subscription(msg_type, topic, push, qos_profile_sensor_data, **kwargs)
    print(f"[relay] Jazzy input ready key={key} topic={output_topic} domain=232", flush=True)
    if owns_node:
        rclpy.spin(node)


def _media_all():
    """One Jazzy participant with concurrent callbacks for all raw streams.

    A participant per camera made the ADU spend most of its CPU in DDS and
    Python process management.  OpenCV releases the GIL while encoding, so a
    multi-threaded executor retains parallel conversion without thirteen DDS
    participants or thirteen interpreter processes.
    """
    os.environ["ROS_DOMAIN_ID"] = "232"
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image, PointCloud2
    from std_msgs.msg import UInt8MultiArray

    rclpy.init()
    try:
        import cv2
        cv2.setNumThreads(1)
    except Exception:
        pass
    node = Node("a3_jazzy_media_relay")
    group = ReentrantCallbackGroup()
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    last_emit = {key: 0.0 for key in CAMERAS}
    received = {key: 0 for key in CAMERAS}
    emitted = {key: 0 for key in CAMERAS}
    pending = {key: None for key in CAMERAS}
    subscriptions = {}
    subscriptions_lock = threading.Lock()
    pending_lock = threading.Lock()
    wake = threading.Condition(pending_lock)
    # The callback only replaces a pointer. JPEG work never runs in the DDS
    # callback, so a slow encoder cannot block discovery or state delivery.
    # Ten raw 30 FPS cameras cannot all be JPEG encoded at full rate on the
    # ADU CPU. Keep a fresh latest-frame preview at a bounded 4 FPS per stream;
    # the one-slot pending map means this never accumulates latency.
    min_interval = 1.0 / 6.0
    active_state = {"keys": _active_streams(), "checked": 0.0}

    worker_count = 3

    def sync_subscriptions(active):
        """Receive raw Image samples only for cards currently on the canvas.

        A callback-side active check is insufficient: DDS deserializes an Image
        before invoking the callback. Recreating these best-effort subscriptions
        is what keeps inactive cameras out of the large-sample CPU path.
        """
        with subscriptions_lock:
            wanted = set(active)
            for key in list(subscriptions):
                if key not in wanted:
                    try:
                        node.destroy_subscription(subscriptions.pop(key))
                    except Exception:
                        subscriptions.pop(key, None)
            for key in wanted - set(subscriptions):
                topic = CAMERAS[key]
                pub = publishers[key]
                def push(msg, stream_key=key):
                    try:
                        received[stream_key] += 1
                        if received[stream_key] == 1 or received[stream_key] % 300 == 0:
                            print(f"[relay] input received key={stream_key} count={received[stream_key]}", flush=True)
                        with wake:
                            if stream_key in active_state["keys"]:
                                pending[stream_key] = msg
                            wake.notify()
                    except Exception as exc:
                        print(f"[relay] input failed key={stream_key}: {type(exc).__name__}: {exc!r}", flush=True)
                subscriptions[key] = node.create_subscription(Image, topic, push, qos,
                                                               callback_group=group)
                print(f"[relay] raw subscription enabled key={key}", flush=True)

    def worker(worker_index):
        keys = [key for index, key in enumerate(CAMERAS) if index % worker_count == worker_index]
        while rclpy.ok():
            with wake:
                msg = None
                stream_key = None
                while msg is None and rclpy.ok():
                    now = time.monotonic()
                    if now - active_state["checked"] >= 0.25:
                        # The control file is intentionally polled at low rate;
                        # DDS callbacks stay non-blocking and frame freshness is
                        # still bounded by one preview interval.
                        active_state["keys"] = _active_streams()
                        active_state["checked"] = now
                        sync_subscriptions(active_state["keys"])
                    for key in keys:
                        if key in active_state["keys"] and pending[key] is not None and now - last_emit[key] >= min_interval:
                            stream_key, msg = key, pending[key]
                            pending[key] = None
                            break
                    if msg is None:
                        wake.wait(timeout=0.02)
            if msg is None or stream_key is None:
                continue
            try:
                last_emit[stream_key] = time.monotonic()
                output = _encode_image(msg, stream_key)
                if output is not None:
                    publishers[stream_key].publish(output)
                    emitted[stream_key] += 1
            except Exception as exc:
                print(f"[relay] input failed key={stream_key}: {type(exc).__name__}: {exc!r}", flush=True)

    publishers = {}
    for key, topic in CAMERAS.items():
        output_topic = f"/agibot_a3/camera_{key}"
        pub = node.create_publisher(CompressedImage, output_topic, qos)
        publishers[key] = pub
        print(f"[relay] Jazzy input available key={key} topic={output_topic} domain=232", flush=True)
    sync_subscriptions(active_state["keys"])
    for worker_index in range(worker_count):
        threading.Thread(target=worker, args=(worker_index,), daemon=True,
                         name=f"a3-camera-encoder-{worker_index}").start()
    lidar_pub = node.create_publisher(UInt8MultiArray, "/agibot_a3/lidar_cloud", qos)
    def push_lidar(msg):
        try:
            output = _encode_cloud(msg)
            if output is not None:
                lidar_pub.publish(output)
        except Exception as exc:
            print(f"[relay] input failed key=lidar_cloud: {type(exc).__name__}: {exc!r}", flush=True)
    node.create_subscription(PointCloud2, "/hal/neck_middle_livox_lidar/pointcloud",
                             push_lidar, qos, callback_group=group)
    print("[relay] Jazzy input ready key=lidar_cloud topic=/agibot_a3/lidar_cloud domain=232", flush=True)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    executor.spin()


def _output_one(key, messages):
    # Keep the converted samples in the robot domain.  The existing isolated
    # A3 core bridge then forwards only these small frontend payloads to 42.
    os.environ["ROS_DOMAIN_ID"] = "232"
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, Image, PointCloud2
    from std_msgs.msg import UInt8MultiArray
    from rclpy.serialization import deserialize_message
    rclpy.init()
    node = Node(f"a3_jazzy_media_relay_output_{key}")
    pubs = {}
    while rclpy.ok():
        try:
            _, payload = messages.get(timeout=0.05)
        except queue.Empty:
            rclpy.spin_once(node, timeout_sec=0.0)
            continue
        try:
            if key == "lidar_cloud":
                output = _encode_cloud(deserialize_message(payload, PointCloud2))
                msg_type = UInt8MultiArray
            else:
                output = _encode_image(deserialize_message(payload, Image), key)
                msg_type = CompressedImage
        except Exception as exc:
            print(f"[relay] encode failed key={key}: {exc}", flush=True)
            continue
        if output is None:
            continue
        topic = f"/agibot_a3/{'lidar_cloud' if key == 'lidar_cloud' else 'camera_' + key}"
        if topic not in pubs:
            pubs[topic] = node.create_publisher(msg_type, topic, 1)
            print(f"[relay] publisher created topic={topic}", flush=True)
        pubs[topic].publish(output)
        rclpy.spin_once(node, timeout_sec=0.0)


def main():
    _media_all()


if __name__ == "__main__":
    main()
