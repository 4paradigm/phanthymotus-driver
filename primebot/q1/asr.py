"""Bridge Q1's native speech recognition results to the Motus DDS domain."""

import json
import math
import multiprocessing
import time
from threading import Lock


SOURCE_TOPIC = "/aima/agent/asr_result"
OUTPUT_FORMAT = "data/json"


def final_payload(msg):
    text = msg.text.strip()
    if not msg.is_final or not text:
        return None
    return {
        "text": text,
        "event_id": msg.event_id,
        "is_final": True,
        "confidence": msg.confidence if math.isfinite(msg.confidence) else None,
        "language": msg.language,
    }


def read_asr(domain, output, stopping):
    import rclpy
    from aimdk_msgs.msg import AsrResult
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from robot import configure_robot_dds

    configure_robot_dds()
    rclpy.init(domain_id=domain)
    node = rclpy.create_node("primebot_q1_asr_reader", start_parameter_services=False)
    last_publisher_count = None

    def report_publishers():
        nonlocal last_publisher_count
        count = node.count_publishers(SOURCE_TOPIC)
        if count != last_publisher_count:
            output.send(("publishers", count))
            last_publisher_count = count

    def receive(msg):
        payload = final_payload(msg)
        if payload is not None:
            output.send(("result", (time.monotonic_ns(), payload)))

    try:
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        node.create_subscription(AsrResult, SOURCE_TOPIC, receive, qos)
        node.create_timer(1.0, report_publishers)
        while not stopping.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except BrokenPipeError:
        if not stopping.is_set():
            raise
    finally:
        output.close()
        node.destroy_node()
        rclpy.try_shutdown()


class AsrCard:
    def __init__(self, namespace, ros, robot_domain):
        from rclpy.node import Node
        from std_msgs.msg import String

        self.topic = f"/{namespace}/asr/text"
        self.output = [{"topic": self.topic, "format": OUTPUT_FORMAT}]
        self.node = Node("primebot_q1_asr", context=ros.ctx_core)
        self.publisher = self.node.create_publisher(String, self.topic, 10)
        self.message_type = String
        ros.executor_core.add_node(self.node)

        ctx = multiprocessing.get_context("spawn")
        self.receiver, self.sender = ctx.Pipe(duplex=False)
        self.stopping = ctx.Event()
        self.reader = ctx.Process(target=read_asr, args=(robot_domain, self.sender, self.stopping))
        self.lock = Lock()
        self.enabled_at_ns = None
        self.publisher_count = None
        self.last_result_ns = None
        self.timer = None

    def get_tool(self):
        from common.vendor_runtime import tool

        return tool(
            "asr", "sensor", "启元 Q1 原生语音识别，输出机器人识别到的最终文本。",
            topic_out=self.output,
        )

    def start(self):
        if self.stopping.is_set() or self.reader.pid is not None:
            return
        self.enabled_at_ns = time.monotonic_ns()
        self.reader.start()
        self.sender.close()
        self.timer = self.node.create_timer(0.05, self._forward)

    def stop(self):
        if self.stopping.is_set():
            return
        self.stopping.set()
        if self.timer:
            self.timer.cancel()
        with self.lock:
            self.receiver.close()
        self.sender.close()
        if self.reader.pid is not None:
            self.reader.join(timeout=2)
            if self.reader.is_alive():
                self.reader.terminate()
                self.reader.join(timeout=1)
        self.node.destroy_node()

    def _forward(self):
        with self.lock:
            while not self.stopping.is_set() and not self.receiver.closed and self.receiver.poll():
                try:
                    kind, value = self.receiver.recv()
                except EOFError:
                    self.receiver.close()
                    return
                if kind == "publishers":
                    self.publisher_count = value
                    continue
                received_ns, payload = value
                if self.enabled_at_ns is not None and received_ns >= self.enabled_at_ns:
                    body = json.dumps(payload, ensure_ascii=False, allow_nan=False)
                    self.publisher.publish(self.message_type(data=body))
                    self.last_result_ns = received_ns

    def dispatch(self, action, args):
        if action not in ("start", "stop", "info", "asr"):
            return None
        with self.lock:
            if action == "stop":
                self.enabled_at_ns = None
            elif action == "start" and self.enabled_at_ns is None and not self.stopping.is_set():
                self.enabled_at_ns = time.monotonic_ns()

            if self.enabled_at_ns is None or self.stopping.is_set():
                state = "idle"
            elif self.reader.exitcode is not None or self.receiver.closed:
                state = "error"
            else:
                state = "running" if self.publisher_count else "waiting"
            result = {"state": state, "topic_out": self.output, "source_publishers": self.publisher_count}
            if self.last_result_ns is not None:
                result["last_result_age_s"] = round((time.monotonic_ns() - self.last_result_ns) / 1e9, 3)
            if state == "error":
                result["error"] = "Q1 ASR subscriber exited"
            return result
