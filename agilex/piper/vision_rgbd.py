"""Use the station's existing aligned RGB-D stream, without owning the camera."""

from collections import deque
from threading import RLock
import json
import math
import time

import numpy as np

from agilex.piper.vision_kinematics import JOINT_NAMES


MAX_AGE_NS = 800_000_000
MAX_SKEW_NS = 100_000_000
MAX_JPEG_BYTES = 8 * 1024 * 1024
MAX_JOINT_BRACKET_NS = 100_000_000
STATIONARY_TOLERANCE_DEG = .15


def stamp_ns(msg):
    stamp = msg.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def decode_depth(msg):
    # RealSense ROS publishes 16UC1 in millimetres. ROS 32FC1 uses metres.
    if msg.encoding not in ("16UC1", "32FC1"):
        raise ValueError("不支持的对齐深度编码")
    dtype = np.dtype((">" if msg.is_bigendian else "<") + ("u2" if msg.encoding == "16UC1" else "f4"))
    if not (1 <= msg.width <= 1920 and 1 <= msg.height <= 1080
            and msg.step >= msg.width * dtype.itemsize and len(msg.data) >= msg.step * msg.height):
        raise ValueError("对齐深度分辨率或行跨度无效")
    depth = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                       strides=(msg.step, dtype.itemsize)).astype(np.float32)
    if msg.encoding == "16UC1":
        depth *= .001
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    depth.setflags(write=False)
    return depth


def camera_point(depth, info, xy):
    if not isinstance(xy, (list, tuple)) or len(xy) != 2 or any(
            type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1 for value in xy):
        raise ValueError("选点须为 [-1,1] 归一化坐标，中心为 (0,0)")
    height, width = depth.shape
    x = min(round((xy[0] + 1) * width / 2), width - 1)
    y = min(round((xy[1] + 1) * height / 2), height - 1)
    patch = depth[max(y - 3, 0):y + 4, max(x - 3, 0):x + 4]
    valid = patch[np.isfinite(patch) & (patch >= .15) & (patch <= 2)]
    if valid.size < max(8, .6 * patch.size):
        raise ValueError("抓取点附近有效深度不足")
    spread = float(np.percentile(valid, 90) - np.percentile(valid, 10))
    if spread > .03:
        raise ValueError("抓取点位于深度边缘，请选择物体内部")
    matrix = np.asarray(info.k, dtype=float).reshape(3, 3)
    distortion = np.asarray(info.d, dtype=float)
    if not np.isfinite(matrix).all() or min(matrix[0, 0], matrix[1, 1]) <= 0 or not np.isfinite(distortion).all():
        raise ValueError("CameraInfo 内参无效")
    if np.any(distortion):
        import cv2

        if info.distortion_model not in ("plumb_bob", "rational_polynomial"):
            raise ValueError("不支持的镜头畸变模型")
        ray = cv2.undistortPoints(np.array([[[x, y]]], dtype=float), matrix, distortion)[0, 0]
    else:
        ray = (np.array([x, y]) - matrix[:2, 2]) / matrix.diagonal()[:2]
    point = np.r_[ray, 1] * float(np.median(valid)) * 1000
    return [x, y], point.tolist(), spread * 1000


class ObservationSource:
    def __init__(self, ros2, config, rgb_output, model=None, *, joint_feedback=None):
        self.ros2 = ros2
        self.base_topic = config.get("camera_topic_base", "/camera_f").rstrip("/")
        self.rgb_output = rgb_output
        self.joint_topic = config.get("joint_state_topic")
        self.joint_feedback = joint_feedback
        self.model = model
        self.lock = RLock()
        self.rgb = deque(maxlen=8)
        self.depth = deque(maxlen=8)
        self.joints = deque(maxlen=32)
        self.info = None
        self.detection = None
        self.nodes = []

    def start(self):
        with self.lock:
            if self.nodes:
                return
            if self.ros2 is None:
                raise RuntimeError("ROS2 尚未初始化")
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data as qos
            from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
            from std_msgs.msg import String

            node_name = "piper_vision_" + self.rgb_output.strip("/").replace("/", "_")
            try:
                core = Node(node_name + "_core", context=self.ros2.ctx_core)
                self.nodes.append((self.ros2.executor_core, core))
                robot = Node(node_name + "_camera", context=self.ros2.ctx_robot)
                self.nodes.append((self.ros2.executor_robot, robot))
                publisher = core.create_publisher(CompressedImage, self.rgb_output, qos)
                core.create_subscription(String, self.rgb_output + "/objects",
                                         lambda msg: self.receive_objects(core, msg), qos)
                robot.create_subscription(
                    CompressedImage, self.base_topic + "/color/image_raw/compressed",
                    lambda msg: self.receive_rgb(robot, msg, publisher), qos)
                robot.create_subscription(Image, self.base_topic + "/aligned_depth_to_color/image_raw",
                                          lambda msg: self.receive(robot, "depth", msg), qos)
                robot.create_subscription(CameraInfo, self.base_topic + "/aligned_depth_to_color/camera_info",
                                          lambda msg: self.receive(robot, "info", msg), qos)
                if self.joint_topic:
                    robot.create_subscription(JointState, self.joint_topic,
                                              lambda msg: self.receive_joints(robot, msg), qos)
                elif self.joint_feedback is not None:
                    robot.create_timer(.02, lambda: self.read_native_joints(robot))
                for executor, node in self.nodes:
                    executor.add_node(node)
            except BaseException:
                self.stop()
                raise

    def stop(self):
        with self.lock:
            nodes, self.nodes = self.nodes, []
            self.rgb.clear()
            self.depth.clear()
            self.joints.clear()
            self.info = self.detection = None
        for executor, node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()

    def receive_rgb(self, node, msg, publisher):
        with self.lock:
            if any(node is active for _, active in self.nodes) and 0 < len(msg.data) <= MAX_JPEG_BYTES:
                self.rgb.append(msg)
                publisher.publish(msg)

    def receive(self, node, stream, message):
        with self.lock:
            if any(node is active for _, active in self.nodes):
                if stream == "info":
                    self.info = message
                else:
                    self.depth.append(message)

    def receive_objects(self, node, msg):
        if len(msg.data) > 128 * 1024:
            return
        try:
            payload = json.loads(msg.data)
            stamp = payload["timestamp"]
            latency = payload.get("latency_ms", 0)
            if (type(stamp) not in (float, int) or type(latency) not in (float, int)
                    or not math.isfinite(stamp) or not 0 <= latency <= 1000):
                return
            objects = []
            for obj in payload["objects"][:50]:
                position = obj["position"]
                if (isinstance(obj["name"], str) and isinstance(position, list) and len(position) == 2
                        and all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 1 for v in position)):
                    objects.append({"name": obj["name"][:100], "position": position})
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            if any(node is active for _, active in self.nodes):
                self.detection = (stamp, latency, objects)

    def receive_joints(self, node, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        by_name = dict(zip(msg.name, msg.position))
        if any(name not in by_name for name in JOINT_NAMES):
            return
        stamp = stamp_ns(msg)
        self.remember_joints(node, np.rad2deg([by_name[name] for name in JOINT_NAMES]), stamp, stamp)

    def read_native_joints(self, node):
        try:
            sample = self.joint_feedback()
        except (OSError, RuntimeError):
            return
        if sample is not None:
            self.remember_joints(node, *sample)

    def remember_joints(self, node, degrees, first_ns, last_ns):
        degrees = np.asarray(degrees, dtype=float)
        if not 0 < first_ns <= last_ns or degrees.shape != (6,) or not np.isfinite(degrees).all():
            return
        with self.lock:
            if any(node is active for _, active in self.nodes) and (
                    not self.joints or last_ns > self.joints[-1][1]):
                self.joints.append((first_ns, last_ns, degrees))

    @staticmethod
    def stationary_angles(joints, rgb_stamp, depth_stamp):
        begin, end = sorted((rgb_stamp, depth_stamp))
        before = max((i for i, (_, last_ns, _) in enumerate(joints) if last_ns < begin), default=-1)
        after = next((i for i, (first_ns, _, _) in enumerate(joints) if first_ns > end), None)
        if before < 0 or after is None or joints[after][1] - joints[before][0] > MAX_JOINT_BRACKET_NS:
            return None
        samples = np.asarray([angles for _, _, angles in joints[before:after + 1]])
        if float(np.max(np.ptp(samples, axis=0))) > STATIONARY_TOLERANCE_DEG:
            return None
        return (samples[0] + samples[-1]) / 2

    def status(self):
        with self.lock:
            now = time.time_ns()
            return {"subscribed": bool(self.nodes), "camera_topic_base": self.base_topic,
                    "rgb_fresh": any(0 <= now - stamp_ns(msg) <= MAX_AGE_NS for msg in self.rgb),
                    "depth_fresh": any(0 <= now - stamp_ns(msg) <= MAX_AGE_NS for msg in self.depth),
                    "camera_info_received": self.info is not None, "objects_topic": self.rgb_output + "/objects",
                    "joint_state_topic": self.joint_topic,
                    "joint_feedback_source": "can" if self.joint_feedback is not None else (
                        "ros" if self.joint_topic else None),
                    "joints_fresh": any(last <= now <= first + MAX_AGE_NS for first, last, _ in self.joints)}

    def snapshot(self):
        import cv2

        with self.lock:
            now = time.time_ns()
            rgb = [msg for msg in self.rgb if 0 <= now - stamp_ns(msg) <= MAX_AGE_NS]
            depth = [msg for msg in self.depth if 0 <= now - stamp_ns(msg) <= MAX_AGE_NS]
            info, detection = self.info, self.detection
            joints = [sample for sample in self.joints if sample[1] <= now <= sample[0] + MAX_AGE_NS]
        if not info or not rgb or not depth:
            raise ValueError("等待有效的 RGB、对齐深度和 CameraInfo")
        pairs = [(color, distance) for color in rgb for distance in depth
                 if color.header.frame_id == distance.header.frame_id == info.header.frame_id
                 and info.header.frame_id and abs(stamp_ns(color) - stamp_ns(distance)) <= MAX_SKEW_NS]
        if not pairs:
            raise ValueError("RGB 与对齐深度的光学坐标系或时间戳不匹配")
        pairs.sort(key=lambda pair: (min(map(stamp_ns, pair)), -abs(stamp_ns(pair[0]) - stamp_ns(pair[1]))),
                   reverse=True)
        color, distance = pairs[0]
        angles = None
        if self.model is not None:
            for color, distance in pairs:
                angles = self.stationary_angles(joints, stamp_ns(color), stamp_ns(distance))
                if angles is not None:
                    break
            if angles is None:
                raise ValueError("No stationary joint samples bracket the RGB-D observation")
        image = cv2.imdecode(np.frombuffer(bytes(color.data), dtype=np.uint8),
                             cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
        if (image is None or image.shape[:2] != (distance.height, distance.width)
                or (distance.height, distance.width) != (info.height, info.width)):
            raise ValueError("RGB、对齐深度和 CameraInfo 的尺寸不匹配")
        rgb_stamp, depth_stamp = stamp_ns(color), stamp_ns(distance)
        objects = []
        if detection:
            completed_at, latency, values = detection
            # Current VOP doesn't echo the source frame ID. Advisory results
            # are for a stationary scene and require confirmation of the pixel.
            estimate_ns = int(completed_at * 1e9 - latency * 1e6)
            if 0 <= now - int(completed_at * 1e9) <= MAX_AGE_NS and abs(estimate_ns - rgb_stamp) <= 350_000_000:
                objects = values
        result = {"depth": decode_depth(distance), "info": info, "rgb_stamp_ns": rgb_stamp,
                  "depth_stamp_ns": depth_stamp, "objects": objects}
        if angles is not None:
            result["base_from_link6_mm"] = self.model.base_from_link6_mm(angles)
            result["joint_degrees"] = angles.tolist()
        return result
