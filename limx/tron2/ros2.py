"""Publish sensor JSON only to Motus's loopback ROS2 bus."""
import json
import os
from pathlib import Path


class JsonPublisher:
    def __init__(self, topics):
        profile = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '')
        if not profile or not Path(profile).is_file():
            raise ValueError('Mount the Core DDS profile before starting this sensor driver')
        if os.environ.get('ROS_DOMAIN_ID') != '42' or \
                os.environ.get('RMW_IMPLEMENTATION') != 'rmw_fastrtps_cpp':
            raise ValueError('Motus sensor bus requires ROS_DOMAIN_ID=42 and FastDDS')
        # The profile must be authoritative even if an inherited base env sets this.
        os.environ.pop('FASTDDS_BUILTIN_TRANSPORTS', None)
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from std_msgs.msg import String
        self.rclpy, self.message_type = rclpy, String
        rclpy.init()
        self.node = None
        try:
            self.node = rclpy.create_node('limx_tron2_readonly')
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
            self.publishers = {topic: self.node.create_publisher(String, topic, qos) for topic in topics}
        except BaseException:
            self.close()
            raise

    def publish(self, topic, data):
        message = self.message_type()
        message.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
        self.publishers[topic].publish(message)

    def close(self):
        if self.node:
            self.node.destroy_node()
            self.node = None
        if self.rclpy.ok():
            self.rclpy.shutdown()
