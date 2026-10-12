"""Explicitly started, read-only zero-position feedback on the canvas DDS bus."""
import json
import threading
import time


class ZeroCheckStream:
    PERIOD = .2

    def __init__(self, ros2, topic, read):
        self.ros2, self.topic, self.read = ros2, topic, read
        self.lock = threading.RLock()
        self.node = self.publisher = self.latest = None
        self.tolerance = 2.

    def unavailable(self, reason):
        return {'timestamp_ms': int(time.time() * 1000), 'available': False,
                'fresh': False, 'feedback_time': None, 'joints_deg': None,
                'distance_to_zero_deg': None, 'within_tolerance': None,
                'feedback_near_zero': None, 'tolerance_deg': self.tolerance,
                'physical_marks_verified': False, 'reason': reason}

    def start(self, tolerance):
        with self.lock:
            if self.ros2 is None:
                raise RuntimeError('实时角度需要 ROS2 数据通道；当前环境只能单次读取')
            if self.tolerance != tolerance:
                self.latest = None
            self.tolerance = tolerance
            if self.node is None:
                from rclpy.node import Node
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
                from std_msgs.msg import String

                node = Node('piper_zero_check', context=self.ros2.ctx_core)
                try:
                    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.VOLATILE)
                    self.publisher = node.create_publisher(String, self.topic, qos)
                    node.create_timer(self.PERIOD, self.tick)
                    self.ros2.executor_core.add_node(node)
                    self.node = node
                    self.latest = None
                except Exception:
                    self.ros2.executor_core.remove_node(node)
                    node.destroy_node()
                    self.publisher = None
                    raise
            return self.info()

    def publish(self, data):
        from std_msgs.msg import String
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
        self.publisher.publish(msg)

    def tick(self):
        with self.lock:
            if self.node is None:
                return
            try:
                self.latest = self.read(self.tolerance)
            except Exception as exc:
                self.latest = self.unavailable(str(exc))
            try:
                self.publish(self.latest)
            except Exception as exc:
                self.latest = self.unavailable(f'数据推送失败：{exc}')
                self.node.get_logger().error(str(exc))

    def info(self):
        with self.lock:
            data = self.latest or self.unavailable('等待首帧' if self.node else '实时读取已停止')
            if data['fresh'] and abs(time.time() - data['feedback_time']) > .5:
                data = self.unavailable('反馈超过 0.5 秒未更新')
            return {'state': 'running' if self.node else 'idle', 'data': data,
                    'refresh_hz': 1 / self.PERIOD,
                    'topic_out': [{'topic': self.topic, 'format': 'data/json'}] if self.node else []}

    def stop(self):
        with self.lock:
            if self.node is not None:
                self.latest = self.unavailable('实时读取已停止')
                try:
                    self.publish(self.latest)
                except Exception as exc:
                    self.node.get_logger().error(str(exc))
                finally:
                    self.ros2.executor_core.remove_node(self.node)
                    self.node.destroy_node()
                    self.node = self.publisher = None
            return self.info()
