"""Same-host DDS wiring for motion_control and arm; no hardware or IK here."""
import json
import os


class MotionBus:
    def __init__(self, namespace, ros_executor, motion, arm):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from std_msgs.msg import String
        if os.environ.get('ROS_DOMAIN_ID') != '42':
            raise ValueError('motion_requires_domain_42')
        self.String, self.executor, self.motion, self.arm = String, ros_executor, motion, arm
        self.node = Node('g1_motion_local_bus', context=ros_executor.context)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        root = f'/{namespace}/motion'
        self.joints = self.node.create_publisher(String, root+'/arm/command', qos)
        self.feedback = self.node.create_publisher(String, root+'/teleop/feedback', qos)
        self.node.create_subscription(String, root+'/control/command', lambda msg: self.receive('eef', msg), qos)
        self.node.create_subscription(String, root+'/arm/command', lambda msg: self.receive('arm', msg), qos)
        self.node.create_timer(.05, self.publish_feedback)
        arm.set_joint_publisher(self.publish_joint)
        ros_executor.add_node(self.node)

    @staticmethod
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result: raise ValueError('duplicate_field')
            result[key] = value
        return result

    def receive(self, route, message):
        packet = None
        try:
            if len(message.data) > 8192: raise ValueError('packet_too_large')
            packet = json.loads(message.data, object_pairs_hook=self.unique)
            if type(packet) is not dict: raise ValueError('invalid_packet')
            if route == 'eef': self.motion.receive_eef(packet)
            else: self.arm.accept_control(packet)
        except (ValueError, TypeError, RuntimeError, RecursionError) as error:
            self.motion.rejected(route, packet, str(error))

    def publish_joint(self, packet):
        message = self.String()
        message.data = json.dumps(packet, allow_nan=False, separators=(',', ':'))
        self.joints.publish(message)

    def publish_feedback(self):
        try:
            message = self.String()
            message.data = json.dumps(self.arm.info(), allow_nan=False, separators=(',', ':'))
            self.feedback.publish(message)
        except (ValueError, TypeError, RuntimeError):
            # Missing feedback must age out; never replace it with invented success.
            self.node.get_logger().warning('motion feedback unavailable')

    def close(self):
        self.arm.set_joint_publisher(None)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
