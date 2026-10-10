"""Latest device input and Canvas monitoring; no reverse device feedback.

A dedicated single-thread executor owns this node's subscriptions and lifecycle.
No other thread creates/destroys/spins its ROS entities, including on rebind.
"""
import copy
import json
import threading
import time
from ros_spin import SpinHealth


class TeleopBus:
    def __init__(self, ros_executor, control):
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from std_msgs.msg import String
        if ros_executor.context.get_domain_id() != 42:
            raise ValueError('teleop_requires_domain_42')
        self.shared_executor, self.control, self.String = ros_executor, control, String
        self.executor = SingleThreadedExecutor(context=ros_executor.context)
        self.node = Node('g1_teleop_control_bus', context=ros_executor.context)
        self.qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self.binding = self.pub = self.sub = None
        self.closed = threading.Event()
        self._status_lock = threading.Lock()
        self.health = SpinHealth()
        self.subscription_generation = 0
        self.receive_error = self.publish_error = None
        self.last_received_sequence = self.last_accepted_sequence = None
        self.last_received_ns = self.last_accepted_ns = None
        self.rejected_count = 0
        control.executor.set_joint_publisher(control.executor.accept_control)
        control.transport_status = self.status
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._run, name='g1-teleop-ros', daemon=True)
        self.thread.start()

    @staticmethod
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate_field')
            result[key] = value
        return result

    @staticmethod
    def binding_key(binding):
        return None if not binding else (binding['command_topic'], binding['feedback_topic'])

    def receive(self, message, generation=None):
        if generation is not None and generation != self.subscription_generation:
            return
        try:
            if len(message.data.encode('utf-8')) > 65536:
                raise ValueError('packet_too_large')
            value = json.loads(message.data, object_pairs_hook=self.unique)
            if not isinstance(value, dict):
                raise ValueError('invalid_packet')
            with self._status_lock:
                self.last_received_sequence = value.get('sequence')
                self.last_received_ns = time.monotonic_ns()
            with self.control._lock:
                if generation is not None and self.binding_key(self.control.binding) != self.binding:
                    return  # Canvas changed topic before the owner processed rebind.
                accepted = self.control.receive(value)
            with self._status_lock:
                if accepted:
                    self.last_accepted_sequence = value.get('sequence')
                    self.last_accepted_ns = time.monotonic_ns()
                    self.receive_error = None
                else:
                    self.rejected_count += 1
                    self.receive_error = 'input_not_admitted'
        except (ValueError, TypeError, RuntimeError, RecursionError) as exc:
            with self._status_lock:
                self.rejected_count += 1
                self.receive_error = str(exc)

    def _bind(self, binding):
        key = self.binding_key(binding)
        if key == self.binding:
            return
        # Only called by this node's single-thread executor owner, between spins.
        self.subscription_generation += 1
        if self.sub:
            self.node.destroy_subscription(self.sub)
        if self.pub:
            self.node.destroy_publisher(self.pub)
        self.sub = self.pub = None
        self.binding = None
        if key:
            generation = self.subscription_generation
            try:
                self.pub = self.node.create_publisher(self.String, key[1], self.qos)
                self.sub = self.node.create_subscription(self.String, key[0],
                    lambda message: self.receive(message, generation), self.qos)
            except Exception:
                if self.pub:
                    self.node.destroy_publisher(self.pub)
                self.pub = None
                raise
        self.binding = key

    def status(self):
        now = time.monotonic_ns()
        with self._status_lock:
            result = {'subscription_generation': self.subscription_generation,
                      'last_received_sequence': self.last_received_sequence,
                      'last_accepted_sequence': self.last_accepted_sequence,
                      'received_age_ms': None if self.last_received_ns is None else (now-self.last_received_ns)/1e6,
                      'accepted_age_ms': None if self.last_accepted_ns is None else (now-self.last_accepted_ns)/1e6,
                      'receive_error': self.receive_error, 'publish_error': self.publish_error,
                      'rejected_count': self.rejected_count}
        result['executor'] = self.health.status()
        shared = getattr(self.shared_executor, '_driver_spin_health', None)
        result['shared_executor'] = shared.status() if shared else None
        return result

    def _run(self):
        from rclpy.impl.implementation_singleton import rclpy_implementation
        InvalidHandle = rclpy_implementation.InvalidHandle
        from rclpy.executors import ExternalShutdownException, ShutdownException
        next_publish = 0.
        try:
            while not self.closed.is_set() and self.executor.context.ok():
                try:
                    with self.control._lock:
                        binding = copy.deepcopy(self.control.binding)
                    self._bind(binding)
                    self.executor.spin_once(timeout_sec=.02)
                    self.health.beat()
                    if self.pub and time.monotonic() >= next_publish:
                        next_publish = time.monotonic()+.2
                        try:
                            message = self.String()
                            message.data = json.dumps(self.control.feedback(), allow_nan=False, separators=(',', ':'))
                            self.pub.publish(message)
                            with self._status_lock:
                                self.publish_error = None
                        except (ValueError, TypeError, RuntimeError) as exc:
                            with self._status_lock:
                                self.publish_error = str(exc)
                except (ExternalShutdownException, ShutdownException):
                    break
                except InvalidHandle as exc:
                    self.health.error(exc, recoverable=True)
                    self.closed.wait(.05)
                except Exception as exc:
                    self.health.error(exc, recoverable=False)
                    break
        finally:
            self.health.stopped()
            self.executor.remove_node(self.node)
            self.executor.shutdown()
            self.node.destroy_node()

    def close(self):
        self.closed.set()
        self.thread.join(.5)
        # If an unexpected callback blocks, the owner still performs destruction.
        # A caller must never race the callback by destroying this node itself.
