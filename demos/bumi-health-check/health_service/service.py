"""Receive-only cache and tool lifecycle. No robot SDK or command client."""
import copy
import json
import math
import re
import threading
import time
from .evaluator import evaluate_health, SOURCE_INTERVALS

INTERVALS = {**SOURCE_INTERVALS, "motion_state": 0.5}

def configuration(namespace, config=None):
    if not isinstance(namespace, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", namespace):
        raise ValueError("BUMI_NAMESPACE 必须填写实际 ROS 命名空间（不含斜杠）")
    config = dict(config or {})
    minimum = config.get("battery_min_soc", 20)
    if not finite(minimum) or not 0 <= minimum <= 100:
        raise ValueError("battery_min_soc 必须为 0—100")
    limits = config.get("temperature_limits", {})
    if not isinstance(limits, dict) or any(k not in ("battery", "joints") or not finite(v) for k,v in limits.items()):
        raise ValueError("temperature_limits 仅接受有限数值 battery/joints")
    intervals = {**INTERVALS, **config.get("intervals", {})}
    if set(intervals) != set(INTERVALS) or any(not finite(v) or v <= 0 for v in intervals.values()):
        raise ValueError("intervals 必须为四类数据的正数发布间隔（秒）")
    topics = {k: f"/{namespace}/state/{k}" for k in SOURCE_INTERVALS}
    topics["motion_state"] = f"/{namespace}/motion/state"
    return {"battery_min_soc": minimum, "temperature_limits": limits,
            "intervals": intervals, "topics": topics}

def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

class HealthService:
    def __init__(self, namespace, config=None):
        self.config = configuration(namespace, config)
        self.samples = {}
        self.lock = threading.RLock()
        self.active = True
        self.transport_error = None

    def receive(self, source, payload, *, now=None, wall=None):
        if source not in self.config["topics"]:
            raise ValueError("未知数据源")
        try:
            data = json.loads(payload, parse_constant=lambda v: None)
        except (ValueError, TypeError):
            data = None
        with self.lock:
            if self.active:
                # Invalid incoming samples replace old valid values, never hide a broken stream.
                self.samples[source] = {
                    "data": data, "received_monotonic": time.monotonic() if now is None else now,
                    "received_at": time.time() if wall is None else wall}

    def tool(self):
        return {"name": "health_check", "type": "actuator", "multiInstance": False,
                "description": "独立 Bumi 健康检查：只订阅状态，报告电量、IMU、关节和保护/故障；不控制机器人。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["check"]}}, "required": ["action"]}}

    def dispatch(self, action):
        with self.lock:
            if action == "start":
                if not self.active:
                    self.samples.clear()
                self.active = True
                return self.info()
            if action == "stop":
                self.active = False
                self.samples.clear()
                return self.info()
            if action == "info":
                return self.info()
            if action != "check":
                raise ValueError("未知 action")
            report = evaluate_health(copy.deepcopy(self.samples), self.config)
            report["topics"] = self.config["topics"]
            report["service_state"] = "running" if self.active else "idle"
            report["freshness_basis"] = "本服务接收 ROS 消息的时间；不能证明 SDK 底层采样时间"
            if self.transport_error:
                report["transport_error"] = self.transport_error
                if report["status"] == "正常":
                    report["status"] = "数据不足"
                    report["summary"] = "ROS 接收不可用，无法完成检查。"
            return report

    def info(self):
        return {"state": "running" if self.active else "idle",
                "topics": self.config["topics"], "transport_error": self.transport_error}
