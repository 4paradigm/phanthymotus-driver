"""Read Q1 IMU telemetry and expose it as a Motus sensor card."""

import json
import multiprocessing
import time
from dataclasses import dataclass
from math import hypot, isfinite
from threading import Lock

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from common.vendor_runtime import action_schema, tool
from robot import configure_robot_dds


PUBLISH_INTERVAL = 0.05
STALE_AFTER = 0.5
FULL_IMU = "/aima/hal/imu/state"
TORSO_ORIENTATION = "/torso_imu_orientation"
EMPTY_IMU = {"orientation": None, "angular_velocity": None, "linear_acceleration": None}


def _components(value, axes, covariance=()):
    if len(covariance) and covariance[0] == -1:
        return None
    result = [getattr(value, axis) for axis in axes]
    return result if all(isfinite(component) for component in result) else None


def _orientation(value, covariance=()):
    result = _components(value, "wxyz", covariance)
    if result is None or not 0.0 < hypot(*result) < float("inf"):
        return None
    return result


def full_imu_payload(msg):
    return {
        "orientation": _orientation(msg.orientation, msg.orientation_covariance),
        "angular_velocity": _components(msg.angular_velocity, "xyz", msg.angular_velocity_covariance),
        "linear_acceleration": _components(
            msg.linear_acceleration, "xyz", msg.linear_acceleration_covariance
        ),
    }


def torso_payload(msg):
    return {**EMPTY_IMU, "orientation": _orientation(msg.pose.pose.orientation)}


@dataclass
class ImuSample:
    received: float
    frame_id: str
    stamp: dict
    data: dict


def select_sample(latest, now):
    """Prefer the complete source while it has a fresh, usable sample."""
    for topic in (FULL_IMU, TORSO_ORIENTATION):
        sample = latest.get(topic)
        if sample and 0 <= now - sample.received <= STALE_AFTER:
            if any(value is not None for value in sample.data.values()):
                return topic, sample
    return None, None


def read_imu(domain, output, stopping):
    from common import logsafe
    import rclpy

    logsafe.install(check_fd=False)
    configure_robot_dds()
    rclpy.init(domain_id=domain)
    node = rclpy.create_node("primebot_q1_imu_reader", start_parameter_services=False)
    pending = {}

    def capture(topic, msg, decode):
        header = msg.header
        pending[topic] = ImuSample(
            time.monotonic(),
            header.frame_id,
            {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec},
            decode(msg),
        )

    def flush():
        if pending:
            output.send(pending)
            pending.clear()

    try:
        qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        node.create_subscription(Imu, FULL_IMU, lambda msg: capture(FULL_IMU, msg, full_imu_payload), qos)
        node.create_subscription(
            Odometry, TORSO_ORIENTATION, lambda msg: capture(TORSO_ORIENTATION, msg, torso_payload), qos,
        )
        node.create_timer(PUBLISH_INTERVAL, flush)
        while not stopping.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        output.close()
        node.destroy_node()
        rclpy.try_shutdown()


class ImuCard:
    def __init__(self, namespace, ros, robot_domain):
        self.topic = f"/{namespace}/state/imu"
        self.output = [{"topic": self.topic, "format": "data/json"}]
        self.node = Node("primebot_q1_imu", context=ros.ctx_core)
        self.publisher = self.node.create_publisher(String, self.topic, 5)
        ros.executor_core.add_node(self.node)
        ctx = multiprocessing.get_context("spawn")
        self.receiver, self.sender = ctx.Pipe(duplex=False)
        self.stopping = ctx.Event()
        self.reader = ctx.Process(target=read_imu, args=(robot_domain, self.sender, self.stopping))
        self.lock = Lock()
        self.latest = {}
        self.published = False
        self.last_published = None
        self.timer = None

    def get_tool(self):
        return tool(
            "imu", "sensor",
            "启元 Q1 机身 IMU。高层运控读取姿态四元数，低层运控可读取角速度和加速度。",
            action_schema({"info": ([], "状态与数据话题")}, {}),
            topic_out=self.output,
        )

    def start(self):
        self.reader.start()
        self.sender.close()
        self.timer = self.node.create_timer(PUBLISH_INTERVAL, self._tick)

    def stop(self):
        if self.stopping.is_set():
            return
        self.stopping.set()
        if self.timer:
            self.timer.cancel()
        if self.reader.pid:
            self.reader.join(timeout=3)
            if self.reader.is_alive():
                self.reader.terminate()
                self.reader.join(timeout=1)
        self.sender.close()
        with self.lock:
            self.receiver.close()
            self.node.destroy_node()

    def _tick(self):
        with self.lock:
            if self.stopping.is_set():
                return
            while not self.receiver.closed and self.receiver.poll():
                try:
                    self.latest.update(self.receiver.recv())
                except EOFError:
                    self.receiver.close()
                    break
            _, sample = select_sample(self.latest, time.monotonic())
            if self.published and sample is self.last_published:
                return
            data = sample.data if sample else EMPTY_IMU
            self.publisher.publish(String(data=json.dumps(data, allow_nan=False)))
            self.last_published = sample
            self.published = True

    def dispatch(self, action, args):
        if action not in ("start", "stop", "info", "imu"):
            return None
        with self.lock:
            now = time.monotonic()
            source, sample = select_sample(self.latest, now)
            result = {
                "state": "idle" if action == "stop" else "running",
                "topic_out": self.output,
                "data": sample.data if sample else EMPTY_IMU,
                "source": source,
            }
            if sample:
                result.update(
                    frame_id=sample.frame_id,
                    stamp=sample.stamp,
                    age_s=round(now - sample.received, 3),
                )
            return result
