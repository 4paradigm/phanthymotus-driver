"""Device input and Canvas monitoring. No reverse feedback to the device.

Only the device/control boundary uses DDS. The G1 coordinator and executor
share an in-process admission callback and immutable motion envelopes.
"""
import copy
import json
import threading


class TeleopBus:
    def __init__(self, ros_executor, control):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from std_msgs.msg import String
        if ros_executor.context.get_domain_id() != 42:
            raise ValueError('teleop_requires_domain_42')
        self.executor, self.control, self.String = ros_executor, control, String
        self.node = Node('g1_teleop_control_bus', context=ros_executor.context)
        self.qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self.binding = self.pub = self.sub = None
        self.closed = threading.Event()
        self.last_error = None
        control.executor.set_joint_publisher(control.executor.accept_control)
        ros_executor.add_node(self.node)
        self.thread = threading.Thread(target=self._run, name='g1-teleop-feedback', daemon=True)
        self.thread.start()

    @staticmethod
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate_field')
            result[key] = value
        return result

    def receive(self, message):
        try:
            if len(message.data.encode('utf-8')) > 65536:
                raise ValueError('packet_too_large')
            value = json.loads(message.data, object_pairs_hook=self.unique)
            if not isinstance(value, dict):
                raise ValueError('invalid_packet')
            self.control.receive(value)
        except (ValueError, TypeError, RuntimeError, RecursionError) as exc:
            self.last_error = str(exc)

    def _bind(self, binding):
        if binding == self.binding:
            return
        if self.sub:
            self.node.destroy_subscription(self.sub)
        if self.pub:
            self.node.destroy_publisher(self.pub)
        self.sub = self.pub = None
        self.binding = binding
        if binding:
            self.pub = self.node.create_publisher(self.String, binding['feedback_topic'], self.qos)
            self.sub = self.node.create_subscription(self.String, binding['command_topic'], self.receive, self.qos)

    def _run(self):
        try:
            while not self.closed.wait(.2):
                try:
                    with self.control._lock:
                        binding = copy.deepcopy(self.control.binding)
                    self._bind(binding)
                    if self.pub:
                        message = self.String()
                        message.data = json.dumps(self.control.feedback(), allow_nan=False, separators=(',', ':'))
                        self.pub.publish(message)
                    self.last_error = None
                except Exception as exc:
                    # No unbounded retry queue; each retry uses the newest state.
                    self.last_error = str(exc)
        finally:
            self.executor.remove_node(self.node)
            self.node.destroy_node()

    def close(self):
        self.closed.set()
        self.thread.join(.3)
        # The writer owns final node destruction if middleware is still blocked.
