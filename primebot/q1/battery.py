"""Q1 battery sensor card on the Motus DDS domain."""
import json
import multiprocessing
import time
from threading import Lock

from rclpy.node import Node
from std_msgs.msg import String

from common.vendor_runtime import action_schema, tool
from robot import read_robot


class BatteryCard:
    def __init__(self, namespace, ros, robot_domain):
        self.topic = f"/{namespace}/battery"
        self.output = [{"topic": self.topic, "format": "data/json"}]
        self.node = Node("primebot_q1_battery", context=ros.ctx_core)
        self.publisher = self.node.create_publisher(String, self.topic, 5)
        ros.executor_core.add_node(self.node)
        self.lock = Lock()
        self.latest = None
        self.active = True
        # Fast DDS profiles are process-wide. Only the vendor reader uses the physical LAN.
        ctx = multiprocessing.get_context("spawn")
        self.receiver, self.sender = ctx.Pipe(duplex=False)
        self.stopping = ctx.Event()
        self.reader = ctx.Process(target=read_robot, args=(robot_domain, self.sender, self.stopping))
        self.timer = None

    def get_tool(self):
        return tool("battery", "sensor", "启元 Q1 电池：SOC、电压、电流、功率、温度、剩余容量与循环次数",
                    action_schema({"info": ([], "状态与数据话题"), "start": ([], "开始输出"),
                                   "stop": ([], "暂停输出")}, {}),
                    topic_out=self.output)

    def start(self):
        self.reader.start()
        self.sender.close()
        self.timer = self.node.create_timer(0.5, self.tick)

    def stop(self):
        if self.stopping.is_set():
            return
        self.stopping.set()
        if self.timer:
            self.timer.cancel()
        if self.reader.pid:
            self.reader.join(timeout=3)
            if self.reader.is_alive():
                self.reader.terminate()
                self.reader.join()
        self.sender.close()
        with self.lock:
            self.receiver.close()
            self.node.destroy_node()

    def snapshot(self):
        if self.latest is None:
            return {"battery": {"state": "waiting"}}
        stamp, values = self.latest
        age = max(0, time.monotonic() - stamp)
        return {"battery": {"state": "live" if age <= 5 else "stale",
                            "age_s": round(age, 2), **values}}

    def tick(self):
        with self.lock:
            if self.stopping.is_set():
                return
            while not self.receiver.closed and self.receiver.poll():
                try:
                    self.latest = self.receiver.recv()
                except EOFError:
                    self.receiver.close()
                    break
            if self.active:
                self.publisher.publish(String(data=json.dumps(self.snapshot(), ensure_ascii=False)))

    def dispatch(self, action, args):
        with self.lock:
            if action == "start":
                self.active = True
            elif action == "stop":
                self.active = False
            elif action not in ("info", "battery"):
                raise ValueError(f"Unknown battery action: {action}")
            return {"state": "running" if self.active else "idle", "topic_out": self.output,
                    "data": self.snapshot()}
