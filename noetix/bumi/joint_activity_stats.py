"""Read-only skeleton statistics. No vendor SDK imports; ROS is loaded on start."""
import json
import math
import threading
import time
from collections import deque

JOINT_NAMES = ('l_arm_pitch_joint', 'l_arm_roll_joint', 'l_arm_yaw_joint', 'l_elbow_pitch_joint', 'l_leg_pitch_joint', 'l_leg_roll_joint', 'l_leg_yaw_joint', 'l_knee_pitch_joint', 'l_ankle_pitch_joint', 'l_ankle_roll_joint', 'r_arm_pitch_joint', 'r_arm_roll_joint', 'r_arm_yaw_joint', 'r_elbow_pitch_joint', 'r_leg_pitch_joint', 'r_leg_roll_joint', 'r_leg_yaw_joint', 'r_knee_pitch_joint', 'r_ankle_pitch_joint', 'r_ankle_roll_joint', 'waist_yaw_joint')


def finite_number(value):
    try:
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    except OverflowError:
        return False


def bounded(value, name, low, high):
    if not finite_number(value) or not low <= value <= high:
        raise ValueError(f"{name} must be finite in [{low}, {high}]")
    return float(value)


class JointWindow:
    def __init__(self, window_s=10, clock=time.monotonic):
        self.window_s = bounded(window_s, 'window_s', 0.1, 300)
        self.clock = clock
        self.lock = threading.RLock()
        self.samples = {name: deque() for name in JOINT_NAMES}
        self.last = {name: None for name in JOINT_NAMES}
        self.received = self.invalid_messages = 0
        self.invalid_entries = self.duplicate_entries = 0
        self.capacity_drops = 0

    def _expire(self, now):
        for samples in self.samples.values():
            while samples and samples[0][0] <= now - self.window_s:
                samples.popleft()

    def observe(self, payload):
        with self.lock:
            now = self.clock()
            self._expire(now)
            self.received += 1
            if not isinstance(payload, dict) or not isinstance(payload.get('joints'), list) or len(payload['joints']) > 256:
                self.invalid_messages += 1
                return
            grouped = {}
            for entry in payload['joints']:
                if not isinstance(entry, dict) or not isinstance(entry.get('name'), str) or entry['name'] not in self.samples:
                    self.invalid_entries += 1
                    continue
                grouped.setdefault(entry['name'], []).append(entry.get('q'))
            for name, values in grouped.items():
                if len(values) != 1:
                    self.duplicate_entries += 1
                    continue  # Ambiguous repeated joint names invalidate this joint in this message.
                q = values[0]
                if not finite_number(q):
                    self.invalid_entries += 1
                    continue
                if len(self.samples[name]) >= 30000:
                    self.samples[name].popleft()
                    self.capacity_drops += 1
                self.samples[name].append((now, q))
                self.last[name] = now

    def snapshot(self):
        with self.lock:
            now = self.clock()
            self._expire(now)
            joints = []
            for name, samples in self.samples.items():
                values = [q for _, q in samples]
                low, high = (min(values), max(values)) if values else (None, None)
                joints.append(dict(name=name, min=low, max=high,
                    range=(high-low if math.isfinite(high-low) else None) if values else None, count=len(values),
                    range_valid=bool(values) and math.isfinite(high-low),
                    status=('incomplete' if self.capacity_drops else ('valid' if math.isfinite(high-low) else 'invalid_range')) if values else ('expired' if self.last[name] is not None else 'no_valid_sample'),
                    last_valid_age_s=now-self.last[name] if self.last[name] is not None else None))
            return dict(window_s=self.window_s, window_start_monotonic_s=now-self.window_s,
                window_end_monotonic_s=now, interval='(start, end]', unit='rad',
                time_basis='local_monotonic_receive_time_not_hardware_sample_time',
                deduplication='unavailable_no_verified_sample_id_repeated_messages_counted',
                status='valid' if all(j['status'] == 'valid' for j in joints) else ('partial' if any(j['count'] for j in joints) else 'no_valid_samples'),
                capacity_dropped_samples=self.capacity_drops, window_complete=self.capacity_drops == 0,
                received_messages=self.received, invalid_messages=self.invalid_messages,
                invalid_entries=self.invalid_entries, duplicate_joint_entries=self.duplicate_entries, joints=joints)


class JointActivityStatsPlugin:
    PREFIX = 'joint_activity_stats'

    def __init__(self, plugin_config, namespace, executor, *, clock=time.monotonic, node_factory=None):
        self.window_s = bounded(plugin_config.get('window_s', 10), 'window_s', 0.1, 300)
        self.publish_hz = bounded(plugin_config.get('publish_hz', 1), 'publish_hz', 0.1, 10)
        self.clock, self.executor, self.node_factory = clock, executor, node_factory
        self.output = f'/{namespace}/joint_activity_stats'
        self.lock = threading.RLock()
        self.node = None
        self.topic = None
        self.generation = 0
        self.window = JointWindow(self.window_s, clock)

    def get_tool(self):
        return dict(name=self.PREFIX, type='processor', multiInstance=False,
            description='Read-only per-joint angle ranges in radians over a local receive-time window; no motion events or commands.',
            inputSchema={'type': 'object', 'properties': {
                'action': {'type': 'string', 'enum': ['start', 'stop', 'info']},
                'input_topic': {'type': 'string'}}, 'required': ['action'],
                'x-action-params': {'start': {'params': ['input_topic'], 'description': 'Subscribe to connected skeleton stream'},
                    'stop': {'params': [], 'description': 'Release only this processor'},
                    'info': {'params': [], 'description': 'Read statistics and lifecycle'}}},
            topic_in=[{'format': 'sensor/skeleton'}],
            topic_out=[{'topic': self.output, 'format': 'data/json'}])

    def start(self):
        # Bundle boot has no canvas binding. Wait for explicit input_topic.
        return True

    def stop(self):
        with self.lock:
            node, self.node = self.node, None
            self.topic = None
            self.generation += 1
            self.window = JointWindow(self.window_s, self.clock)
            if node is not None:
                try:
                    self.executor.remove_node(node)
                finally:
                    node.destroy_node()

    def _start(self, args):
        topics = args.get('input_topics')
        if topics is not None and (not isinstance(topics, list) or len(topics) != 1):
            return {'state': 'error', 'error': 'exactly one input topic required'}
        topic = args.get('input_topic')
        if topic and topics and topic != topics[0]:
            return {'state': 'error', 'error': 'conflicting input_topic and input_topics'}
        if not topic:
            topics = args.get('input_topics', [])
            if not isinstance(topics, list) or len(topics) != 1:
                return {'state': 'error', 'error': 'exactly one input topic required'}
            topic = topics[0]
        if not isinstance(topic, str) or not topic.strip() or topic.strip() == self.output:
            return {'state': 'error', 'error': 'valid skeleton input_topic required'}
        topic = topic.strip()
        with self.lock:
            if self.node is not None:
                return {'state': 'running' if topic == self.topic else 'error', 'input_topic': self.topic,
                        'topic_out': self.get_tool()['topic_out'],
                        'message': 'stop before changing input topic'}
            node = None
            try:
                from rclpy.node import Node
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
                from std_msgs.msg import String
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
                node = (self.node_factory or Node)('bumi_joint_activity_stats')
                publisher = node.create_publisher(String, self.output, qos)
                generation = self.generation
                def receive(msg):
                    with self.lock:
                        if self.node is not node or generation != self.generation:
                            return
                        try:
                            payload = json.loads(msg.data) if len(msg.data) <= 262144 else None
                        except (ValueError, TypeError, RecursionError):
                            payload = None
                        self.window.observe(payload)
                def publish():
                    with self.lock:
                        if self.node is not node or generation != self.generation:
                            return
                        msg = String()
                        msg.data = json.dumps(self.window.snapshot(), allow_nan=False)
                        publisher.publish(msg)
                node.create_subscription(String, topic, receive, qos)
                node.create_timer(1/self.publish_hz, publish)
                if self.executor.add_node(node) is False:
                    raise RuntimeError('executor refused processor node')
                self.node, self.topic = node, topic
            except Exception as exc:
                if node is not None:
                    try:
                        self.executor.remove_node(node)
                    finally:
                        node.destroy_node()
                return {'state': 'error', 'error': str(exc)}
            return {'state': 'running', 'input_topic': topic, 'topic_out': self.get_tool()['topic_out']}

    def dispatch(self, action, args):
        if action == 'start':
            return self._start(args)
        if action == 'stop':
            self.stop()
            return {'state': 'idle'}
        if action in ('info', self.PREFIX):
            with self.lock:
                return dict(state='running' if self.node else 'idle', input_topic=self.topic,
                            topic_in=[{'topic': self.topic, 'format': 'sensor/skeleton'}] if self.topic else [],
                            topic_out=self.get_tool()['topic_out'], statistics=self.window.snapshot())
        return None
