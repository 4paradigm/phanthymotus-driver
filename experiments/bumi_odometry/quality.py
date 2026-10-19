"""Latch odometry faults and export body poses; this process never commands motion."""
import argparse
import csv
import json
import math
from pathlib import Path
import uuid

try:
    from .evaluate import rotation
except ImportError:
    from evaluate import rotation


class TrackingQuality:
    def __init__(self):
        self.samples = {}
        self.segment = None
        self.reason = None
        self.clock = None

    def invalidate(self, reason):
        self.reason = self.reason or reason

    def update(self, kind, stamp_ns, received_ns, segment_id, valid):
        if kind not in ("visual", "imu", "filtered"):
            raise ValueError("unknown source")
        previous = self.samples.get(kind)
        if (not valid or not segment_id or
                (previous is not None and stamp_ns <= previous[0]) or
                (self.segment is not None and segment_id != self.segment) or
                received_ns - stamp_ns > 500_000_000 or stamp_ns - received_ns > 50_000_000):
            self.invalidate("invalid, reset, or unsynchronized " + kind)
        self.segment = segment_id
        self.samples[kind] = stamp_ns, received_ns
        return self.state(received_ns)

    def state(self, now_ns):
        if self.clock is not None and now_ns < self.clock:
            self.invalidate("ROS clock moved backwards")
        self.clock = now_ns
        if any(now_ns - min(stamp, received) > 500_000_000
               for stamp, received in self.samples.values()):
            self.invalidate("sensor stream stale")
        if self.reason:
            return "lost"
        return "tracking" if len(self.samples) == 3 else "initializing"


class TrialCSV:
    """One file is one trial; late faults invalidate it even without more EKF output."""
    def __init__(self, stream, segment):
        self.stream, self.segment = stream, segment
        self.writer = csv.writer(stream)
        self.fault_written = False
        self.writer.writerow(["stamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw",
                              "status", "segment_id", "fault_stamp_ns"])

    def pose(self, stamp_ns, pose, status):
        self.writer.writerow([stamp_ns] + list(pose) + [status, self.segment, ""])

    def fault(self, received_ns, source_ns):
        if not self.fault_written:
            self.writer.writerow([received_ns] + [""]*7 + ["lost", self.segment, source_ns])
            self.stream.flush()
            self.fault_written = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args, ros_args = parser.parse_known_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from rtabmap_msgs.msg import OdomInfo

    def stamp(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def pose_valid(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        try:
            rotation((q.x, q.y, q.z, q.w))
            return (message.header.frame_id == "bumi_odom_odom" and
                    message.child_frame_id == "bumi_odom_base_link" and
                    all(math.isfinite(v) for v in (p.x, p.y, p.z)) and
                    all(math.isfinite(message.pose.covariance[i]) and message.pose.covariance[i] > 0
                        for i in (0, 7, 14, 21, 28, 35)))
        except ValueError:
            return False

    class Exporter(Node):
        def __init__(self, stream):
            super().__init__("quality", namespace="bumi_odom")
            self.guard = TrackingQuality()
            self.segment = uuid.uuid4().hex
            self.info = None
            self.info_distance = None
            self.output = TrialCSV(stream, self.segment)
            self.stream = stream
            self.publisher = self.create_publisher(String, "status", 10)
            for msg_type, topic, callback in (
                    (Odometry, "visual", self.visual), (OdomInfo, "visual_info", self.visual_info),
                    (Imu, "bumi_odom_camera/imu", self.imu), (Odometry, "filtered", self.filtered)):
                self.create_subscription(msg_type, topic, callback, qos_profile_sensor_data)
            self.create_timer(0.1, self.publish_status)

        def now(self):
            return self.get_clock().now().nanoseconds

        def visual(self, msg):
            self.guard.update("visual", stamp(msg), self.now(), self.segment, pose_valid(msg))
            self.record_fault(stamp(msg))

        def visual_info(self, msg):
            now, source = self.now(), stamp(msg)
            if (msg.lost or not math.isfinite(msg.distance_travelled) or
                    (self.info is not None and source <= self.info[0]) or
                    (self.info_distance is not None and msg.distance_travelled < self.info_distance - 1e-6) or
                    now - source > 500_000_000 or source - now > 50_000_000):
                self.guard.invalidate("visual tracking lost, reset, or unsynchronized")
            self.info = source, now
            self.info_distance = msg.distance_travelled
            self.record_fault(source)

        def imu(self, msg):
            w, cov = msg.angular_velocity, msg.angular_velocity_covariance
            valid = (msg.header.frame_id == "bumi_odom_camera_imu_optical_frame" and
                     all(math.isfinite(v) for v in (w.x, w.y, w.z)) and
                     all(math.isfinite(cov[i]) and cov[i] > 0 for i in (0, 4, 8)))
            self.guard.update("imu", stamp(msg), self.now(), self.segment, valid)
            self.record_fault(stamp(msg))

        def status(self):
            now = self.now()
            state = self.guard.state(now)
            if self.info is None:
                return "lost" if state == "lost" else "initializing"
            if now - min(self.info) > 500_000_000:
                self.guard.invalidate("visual diagnostics stale")
            return self.guard.state(now)

        def record_fault(self, source_ns):
            status = self.status()
            if status == "lost":
                self.output.fault(self.now(), source_ns)
            return status

        def publish_status(self):
            msg = String()
            msg.data = json.dumps(dict(status=self.record_fault(self.now()), reason=self.guard.reason,
                                       segment_id=self.segment, stamp_ns=self.now()))
            self.publisher.publish(msg)
            self.stream.flush()

        def filtered(self, msg):
            now, source = self.now(), stamp(msg)
            self.guard.update("filtered", source, now, self.segment, pose_valid(msg))
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.output.pose(source, [p.x, p.y, p.z, q.x, q.y, q.z, q.w], self.record_fault(source))

    # Exclusive creation prevents a second process from truncating an active trial.
    with args.output.open("x", newline="") as stream:
        rclpy.init(args=ros_args)
        node = Exporter(stream)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
