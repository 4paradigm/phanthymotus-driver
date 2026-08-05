#!/usr/bin/env python3
"""Publish a static MID360/IMU stream with one deliberate IMU gap."""

from __future__ import annotations

from array import array
import json
import struct
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField


IMU_PERIOD_SEC = 0.005
CLOUD_PERIOD_SEC = 0.1
GAP_START_SEC = 4.0
GAP_END_SEC = 4.35
TEST_DURATION_SEC = 10.0


RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=400,
    durability=DurabilityPolicy.VOLATILE,
)


def _static_scene() -> list[tuple[float, float, float, float, int, int]]:
    points: list[tuple[float, float, float, float, int, int]] = []
    for x_index in range(24):
        for y_index in range(20):
            points.append(
                (
                    1.0 + x_index * 0.2,
                    -2.0 + y_index * 0.2,
                    -1.0,
                    20.0,
                    0x10,
                    (x_index + y_index) % 4,
                )
            )
    for y_index in range(20):
        for z_index in range(14):
            points.append(
                (
                    5.0,
                    -2.0 + y_index * 0.2,
                    -1.0 + z_index * 0.15,
                    40.0,
                    0x10,
                    (y_index + z_index) % 4,
                )
            )
    for x_index in range(20):
        for z_index in range(14):
            points.append(
                (
                    1.0 + x_index * 0.2,
                    3.0,
                    -1.0 + z_index * 0.15,
                    60.0,
                    0x10,
                    (x_index + z_index) % 4,
                )
            )
    return points


class ImuGapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fast_livo_imu_gap_smoke")
        self._cloud_publisher = self.create_publisher(
            PointCloud2, "/gap_test/lidar", RELIABLE_QOS
        )
        self._imu_publisher = self.create_publisher(
            Imu, "/gap_test/imu", RELIABLE_QOS
        )
        self._odom_subscription = self.create_subscription(
            Odometry, "/gap_test/odom", self._on_odom, 100
        )
        self._points = _static_scene()
        self._test_start: float | None = None
        self._odom_before_gap = 0
        self._odom_after_gap = 0
        self._odom_total = 0

    def _on_odom(self, _message: Odometry) -> None:
        self._odom_total += 1
        if self._test_start is None:
            return
        elapsed = time.monotonic() - self._test_start
        if elapsed < GAP_START_SEC:
            self._odom_before_gap += 1
        elif elapsed > GAP_END_SEC + 0.5:
            self._odom_after_gap += 1

    @staticmethod
    def _set_stamp(message, stamp) -> None:
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = "livox_frame"

    def _publish_imu(self) -> None:
        message = Imu()
        self._set_stamp(message, self.get_clock().now())
        message.orientation.w = 1.0
        message.orientation_covariance[0] = -1.0
        message.linear_acceleration.z = 1.0
        self._imu_publisher.publish(message)

    def _publish_cloud(self) -> None:
        now = self.get_clock().now()
        header_ns = now.nanoseconds
        payload = bytearray(len(self._points) * 32)
        denominator = max(1, len(self._points) - 1)
        for index, (x, y, z, intensity, tag, line) in enumerate(self._points):
            point_stamp_ns = header_ns + round(index / denominator * 80_000_000)
            struct.pack_into(
                "<fff4xfBB2xd",
                payload,
                index * 32,
                x,
                y,
                z,
                intensity,
                tag,
                line,
                float(point_stamp_ns),
            )

        message = PointCloud2()
        self._set_stamp(message, now)
        message.height = 1
        message.width = len(self._points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=16, datatype=PointField.FLOAT32, count=1
            ),
            PointField(name="tag", offset=20, datatype=PointField.UINT8, count=1),
            PointField(name="line", offset=21, datatype=PointField.UINT8, count=1),
            PointField(
                name="timestamp", offset=24, datatype=PointField.FLOAT64, count=1
            ),
        ]
        message.is_bigendian = False
        message.point_step = 32
        message.row_step = message.point_step * message.width
        message.data = array("B", payload)
        message.is_dense = True
        self._cloud_publisher.publish(message)

    def wait_for_estimator(self, timeout_sec: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.count_subscribers("/gap_test/lidar") >= 1
                and self.count_subscribers("/gap_test/imu") >= 1
            ):
                return
        raise RuntimeError("FAST-LIVO2 subscriptions were not discovered")

    def run(self) -> dict[str, int | float | bool]:
        self.wait_for_estimator()
        self._test_start = time.monotonic()
        next_imu = self._test_start
        next_cloud = self._test_start
        gap_announced = False

        while True:
            now = time.monotonic()
            elapsed = now - self._test_start
            if elapsed >= TEST_DURATION_SEC:
                break

            if now >= next_cloud:
                self._publish_cloud()
                next_cloud = max(next_cloud + CLOUD_PERIOD_SEC, now)

            if GAP_START_SEC <= elapsed < GAP_END_SEC:
                next_imu = now + IMU_PERIOD_SEC
                gap_announced = True
            elif now >= next_imu:
                self._publish_imu()
                next_imu = max(next_imu + IMU_PERIOD_SEC, now)

            rclpy.spin_once(self, timeout_sec=0.001)

        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.01)

        passed = (
            gap_announced
            and self._odom_before_gap >= 3
            and self._odom_after_gap >= 3
        )
        return {
            "passed": passed,
            "intentional_gap_sec": round(GAP_END_SEC - GAP_START_SEC, 3),
            "odom_before_gap": self._odom_before_gap,
            "odom_after_gap": self._odom_after_gap,
            "odom_total": self._odom_total,
        }


def main() -> int:
    rclpy.init()
    node = ImuGapPublisher()
    try:
        result = node.run()
        print(json.dumps(result, sort_keys=True), flush=True)
        return 0 if result["passed"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
