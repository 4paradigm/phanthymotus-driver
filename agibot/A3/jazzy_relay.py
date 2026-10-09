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
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 50])
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
    points = []
    for row in range(int(msg.height)):
        for col in range(int(msg.width)):
            base = row * int(msg.row_step) + col * int(msg.point_step)
            try:
                xyz = tuple(float(np.frombuffer(bytes(msg.data), dtype=np.float32, count=1, offset=base + fields[n].offset)[0]) for n in ("x", "y", "z"))
            except (ValueError, IndexError):
                continue
            if all(np.isfinite(xyz)):
                points.append(xyz)
    points = points[:20000]
    out = UInt8MultiArray()
    out.data = list(struct.pack("<II", 12, len(points)) + b"".join(struct.pack("<fff", *p) for p in points))
    return out


def _input(queues):
    os.environ["ROS_DOMAIN_ID"] = "232"
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2
    from rclpy.serialization import serialize_message
    rclpy.init()
    node = Node("a3_jazzy_media_relay_input")
    def push(key, msg):
        try:
            item = (key, serialize_message(msg))
            queue_out = queues[key]
            try:
                queue_out.put_nowait(item)
            except queue.Full:
                queue_out.get_nowait()
                queue_out.put_nowait(item)
        except Exception as exc:
            print(f"[relay] input failed key={key}: {exc}", flush=True)
    for key, topic in CAMERAS.items():
        node.create_subscription(Image, topic, lambda msg, k=key: push(k, msg), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/hal/neck_middle_livox_lidar/pointcloud", lambda msg: push("lidar_cloud", msg), qos_profile_sensor_data)
    print(f"[relay] Jazzy input ready cameras={len(CAMERAS)} lidar=1 domain=232", flush=True)
    rclpy.spin(node)


def _output(queues):
    # Keep the converted samples in the robot domain.  The existing isolated
    # A3 core bridge then forwards only these small frontend payloads to 42.
    os.environ["ROS_DOMAIN_ID"] = "232"
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, Image, PointCloud2
    from std_msgs.msg import UInt8MultiArray
    from rclpy.serialization import deserialize_message
    rclpy.init()
    node = Node("a3_jazzy_media_relay_output")
    pubs = {}
    keys = list(queues)
    cursor = 0
    while rclpy.ok():
        item = None
        # Poll independent one-slot queues round-robin. A busy 30 Hz camera or
        # lidar must not consume the shared queue and starve the other cameras.
        for _ in range(len(keys)):
            key = keys[cursor]
            cursor = (cursor + 1) % len(keys)
            try:
                item = queues[key].get_nowait()
                break
            except queue.Empty:
                pass
        if item is None:
            rclpy.spin_once(node, timeout_sec=0.005)
            continue
        key, payload = item
        if key == "lidar_cloud":
            output = _encode_cloud(deserialize_message(payload, PointCloud2))
            msg_type = UInt8MultiArray
        else:
            output = _encode_image(deserialize_message(payload, Image), key)
            msg_type = CompressedImage
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
    queues = {key: ctx.Queue(maxsize=1) for key in keys}
    processes = [ctx.Process(target=_input, args=(queues,), daemon=True), ctx.Process(target=_output, args=(queues,), daemon=True)]
    for process in processes:
        process.start()
    print(f"[relay] started input={processes[0].pid} output={processes[1].pid}", flush=True)
    for process in processes:
        process.join()


if __name__ == "__main__":
    main()
