"""Optional arm endpoint input, independent of RGB-D acquisition."""
import ipaddress
import json
import threading
import time

from hardware import canonical_ip


def split_topics(args):
    topics = args.get('input_topics') or ([args['input_topic']] if args.get('input_topic') else [])
    if not isinstance(topics, list) or any(not isinstance(t, str) or not t.startswith('/') for t in topics):
        raise ValueError('Input topics must be absolute ROS topic paths')
    ips = [t for t in topics if t.endswith('/arm_ip_sync')]
    if len(ips) > 1 or len(set(topics)) != len(topics):
        raise ValueError('Connect at most one arm IP source and no duplicate inputs')
    return (ips[0] if ips else None), [t for t in topics if t not in ips]


def mapping(value):
    if not isinstance(value, dict) or set(value) != {'left', 'right'}:
        raise ValueError('Arm IP input must contain exactly left and right')
    result = {side: canonical_ip(value[side]) for side in ('left', 'right')}
    if result['left'] == result['right']:
        raise ValueError('Left and right controller IPs must be distinct')
    return result


class ArmIPInput:
    def __init__(self, ros2, apply, name='fr3_pick_arm_ip_input'):
        self.name = name
        self.ros2, self.apply = ros2, apply
        self.lock = threading.Lock()
        self.topic = None
        self.thread = None
        self.quit = threading.Event()
        self.received_at = None
        self.addresses = None
        self.controller_ip = ""
        self.error = None

    def receive(self, raw):
        try:
            if not isinstance(raw, str) or len(raw.encode()) > 4096:
                raise ValueError('Arm IP message is invalid or exceeds 4096 bytes')
            payload = json.loads(raw)
            controller = payload.pop("controller_ip", "") if isinstance(payload, dict) else ""
            if not isinstance(controller, str):
                raise ValueError("Controller IP must be a string")
            controller = str(ipaddress.IPv4Address(controller)) if controller else ""
            addresses = mapping(payload)
            # Never hold the input lock while acquiring the card's lock.
            self.apply(addresses)
            with self.lock:
                self.addresses = addresses
                self.controller_ip = controller
                self.received_at = time.monotonic()
                self.error = None
        except Exception as exc:
            with self.lock:
                self.error = str(exc)

    def info(self):
        with self.lock:
            age = None if self.received_at is None else time.monotonic() - self.received_at
            return dict(topic=self.topic, connected=self.topic is not None,
                        fresh=self.topic is not None and age is not None and age < 3 and not self.error,
                        ready=self.topic is not None and self.addresses is not None and not self.error,
                        controller_ip=self.controller_ip,
                        motion_arm_ips=dict(self.addresses) if self.addresses else None, error=self.error)

    def start(self, topic):
        if topic == self.topic and self.thread and self.thread.is_alive():
            return
        self.stop()
        if topic is None:
            return
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        node = Node(self.name, context=self.ros2.ctx_core)
        executor = SingleThreadedExecutor(context=self.ros2.ctx_core)
        self.topic = topic
        self.quit = threading.Event()
        def receive(msg):
            if len(node.get_publishers_info_by_topic(topic)) != 1:
                with self.lock:
                    self.error = 'Expected exactly one publisher on the arm IP input'
                return
            self.receive(msg.data)
        try:
            node.create_subscription(String, topic, receive, qos_profile_sensor_data)
            executor.add_node(node)
        except Exception:
            executor.shutdown(); node.destroy_node(); self.topic = None
            raise
        def run():
            try:
                while not self.quit.is_set():
                    executor.spin_once(timeout_sec=.1)
            finally:
                executor.shutdown(); node.destroy_node()
        self.thread = threading.Thread(target=run, daemon=True, name='fr3-arm-ip-input')
        self.thread.start()

    def stop(self):
        self.quit.set()
        if self.thread:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                raise RuntimeError('Arm IP subscriber did not stop')
        self.thread = None
        with self.lock:
            self.topic = self.received_at = self.addresses = self.error = None
            self.controller_ip = ""
