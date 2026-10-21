"""Publish sensor JSON and three RGB JPEG streams to Motus's loopback ROS2 bus."""
import json
import os
import re
from pathlib import Path

IMAGE_FRAMES = {'/limx_tron2/camera': 'tron2_head_camera',
                '/limx_tron2/camera_left': 'tron2_left_wrist_camera',
                '/limx_tron2/camera_right': 'tron2_right_wrist_camera'}


class JsonPublisher:
    def __init__(self, topics, image_topics=()):
        profile = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '')
        if not profile or not Path(profile).is_file():
            raise ValueError('Mount the Core DDS profile before starting this sensor driver')
        domain = os.environ.get('ROS_DOMAIN_ID', '')
        # The platform default remains strictly42. A nonstandard site must
        # explicitly bind the domain reviewed from its existing Core; merely
        # overriding ROS_DOMAIN_ID does not relax the default contract.
        core_domain = os.environ.get('TRON2_CORE_DOMAIN_ID', '42')
        if not re.fullmatch(r'0|[1-9][0-9]{0,2}', domain) or int(domain) > 232 or \
                domain != core_domain or \
                os.environ.get('RMW_IMPLEMENTATION') != 'rmw_fastrtps_cpp':
            raise ValueError('Motus sensor bus requires domain42 and FastDDS; a reviewed nonstandard Core requires matching TRON2_CORE_DOMAIN_ID')
        # The profile must be authoritative even if an inherited base env sets this.
        os.environ.pop('FASTDDS_BUILTIN_TRANSPORTS', None)
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from std_msgs.msg import String
        self.rclpy, self.message_type = rclpy, String
        self.image_topics = set(image_topics)
        self.image_type = None
        if self.image_topics:
            from sensor_msgs.msg import CompressedImage
            self.image_type = CompressedImage
        rclpy.init()
        self.node = None
        try:
            self.node = rclpy.create_node('limx_tron2_readonly')
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
            self.publishers = {topic: self.node.create_publisher(String, topic, qos) for topic in topics}
            if self.image_topics:
                image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
                self.publishers.update({topic: self.node.create_publisher(self.image_type, topic, image_qos)
                                        for topic in self.image_topics})
        except BaseException:
            self.close()
            raise

    def publish(self, topic, data):
        if topic in self.image_topics:
            message = self.image_type()
            # This stamp is the relay's receipt/publish clock, not the camera's
            # unverified measurement timestamp. JPEG bytes remain unchanged.
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = IMAGE_FRAMES[topic]
            message.format, message.data = 'jpeg', data
        else:
            message = self.message_type()
            message.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
        self.publishers[topic].publish(message)

    def close(self):
        if self.node:
            self.node.destroy_node()
            self.node = None
        if self.rclpy.ok():
            self.rclpy.shutdown()
