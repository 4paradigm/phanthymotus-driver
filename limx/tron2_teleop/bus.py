"""One ROS owner, one pending input, one pending monitor snapshot."""
import json
import os
from pathlib import Path
import threading

from common.teleop_contract import COMMAND_TOPIC, STATE_TOPIC, MAX_WIRE_BYTES


class TeleopBus:
    def __init__(self):
        profile = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '')
        if not profile or not Path(profile).is_file():
            raise ValueError('core_dds_profile_required')
        if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('RMW_IMPLEMENTATION') != 'rmw_fastrtps_cpp':
            raise ValueError('domain42_fastdds_required')
        os.environ.pop('FASTDDS_BUILTIN_TRANSPORTS', None)
        self.lock = threading.Lock()
        self.latest = self.snapshot = None
        self.error = None
        self.ready, self.closed = threading.Event(), threading.Event()
        self.thread = threading.Thread(target=self.run, name='tron2-teleop-dds', daemon=True)
        self.thread.start()
        if not self.ready.wait(5) or self.error:
            self.close()
            raise RuntimeError(self.error or 'dds_start_timeout')

    def receive(self, message):
        raw = message.data
        if len(raw.encode('utf-8')) > MAX_WIRE_BYTES:
            raw = '{}'
        with self.lock:
            self.latest = raw

    def take(self):
        with self.lock:
            value, self.latest = self.latest, None
            return value

    def feedback(self, value):
        with self.lock:
            self.snapshot = json.dumps(value, allow_nan=False, separators=(',', ':'))

    def run(self):
        node = executor = context = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from std_msgs.msg import String
            context = Context()
            rclpy.init(context=context, domain_id=42)
            node = Node('tron2_teleop', context=context)
            executor = SingleThreadedExecutor(context=context)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
            node.create_subscription(String, COMMAND_TOPIC, self.receive, qos)
            publisher = node.create_publisher(String, STATE_TOPIC, qos)
            executor.add_node(node)
            self.ready.set()
            while not self.closed.is_set() and context.ok():
                executor.spin_once(timeout_sec=.005)
                with self.lock:
                    snapshot, self.snapshot = self.snapshot, None
                if snapshot is not None:
                    publisher.publish(String(data=snapshot))
            if not self.closed.is_set():
                raise RuntimeError('dds_context_stopped')
        except Exception as exc:
            self.error = str(exc)[:160] or type(exc).__name__
        finally:
            self.ready.set()
            if executor is not None:
                executor.shutdown(timeout_sec=1)
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.shutdown()

    def close(self):
        self.closed.set()
        self.thread.join(timeout=6)
        if self.thread.is_alive():
            raise RuntimeError('dds_shutdown_timeout')
