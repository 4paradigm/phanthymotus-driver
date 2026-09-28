"""ROS subscriptions only; no publishers or SDK access."""
import threading

class RosReceiver:
    def __init__(self, service):
        self.service = service
        self.node = None
        self.executor = None
        self.thread = None
        self.stopping = False

    def start(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("bumi_health_check")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscriptions = [
            self.node.create_subscription(String, topic,
                lambda msg, source=source: self.service.receive(source, msg.data), qos)
            for source, topic in self.service.config["topics"].items()]
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception as exc:
            if not self.stopping:
                self.service.transport_error = f"ROS 接收失败：{exc}"
        finally:
            if not self.stopping:
                self.service.transport_error = self.service.transport_error or "ROS 接收已停止"

    def close(self):
        self.stopping = True
        if self.executor:
            self.executor.shutdown(timeout_sec=2)
        if self.thread:
            self.thread.join(timeout=3)
        if self.node:
            self.node.destroy_node()
        if hasattr(self, "rclpy") and self.rclpy.ok():
            self.rclpy.shutdown()
