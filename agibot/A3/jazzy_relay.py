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
    "wrist_left_d405_rgb": "/hal/wrist_left_d405_camera/rgb",
    "wrist_right_d405_rgb": "/hal/wrist_right_d405_camera/rgb",
}


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
        max_dimension = 640
        largest = max(image.shape[:2])
        if largest > max_dimension:
            scale = max_dimension / float(largest)
            image = cv2.resize(image, (max(1, int(image.shape[1] * scale)),
                                       max(1, int(image.shape[0] * scale))),
                               interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 42])
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
    out = UInt8MultiArray()
    out.data = list(struct.pack("<II", 12, len(points)) + points.astype("<f4", copy=False).tobytes())
    return out


def _media_one(key, topic, msg_type):
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
    rclpy.init()
    node = Node(f"a3_jazzy_media_relay_{key}")
    output_topic = f"/agibot_a3/{'lidar_cloud' if key == 'lidar_cloud' else 'camera_' + key}"
    output_type = UInt8MultiArray if key == "lidar_cloud" else CompressedImage
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
    node.create_subscription(msg_type, topic, push, qos_profile_sensor_data)
    print(f"[relay] Jazzy input ready key={key} topic={output_topic} domain=232", flush=True)
    rclpy.spin(node)


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
    ctx = mp.get_context("spawn")
    keys = list(CAMERAS) + ["lidar_cloud"]
    from sensor_msgs.msg import Image, PointCloud2
    processes = [ctx.Process(target=_media_one,
                             args=(key, topic, Image), daemon=True)
                 for key, topic in CAMERAS.items()]
    processes.append(ctx.Process(target=_media_one, args=(
        "lidar_cloud", "/hal/neck_middle_livox_lidar/pointcloud", PointCloud2), daemon=True))
    for process in processes:
        process.start()
    print(f"[relay] started streams={len(processes)} pids={[p.pid for p in processes]}", flush=True)
    for process in processes:
        process.join()


if __name__ == "__main__":
    main()
