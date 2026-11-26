"""Read-only joint motor telemetry on the existing PiPER sensor transport."""

from collections.abc import Callable
from dataclasses import asdict
from functools import partial
import threading
import time

from agilex.piper.arm import MOTOR_FEEDBACK_IDS, MotorTelemetry
from agilex.piper.monitor import RosMonitorPublisher
from common.vendor_runtime import tool


TOOL_NAME = "piper_joint_motor_telemetry"
PUBLISH_HZ = 5
MAX_FEEDBACK_AGE_S = 0.5


class JointMotorTelemetryPlugin:
    def __init__(
        self,
        read_feedback: Callable[[], tuple[MotorTelemetry | None, ...]],
        *,
        namespace: str,
        can_interface: str,
        ros2=None,
        clock: Callable[[], float] = time.time,
        publisher_factory=None,
    ):
        self._read_feedback = read_feedback
        self._namespace = namespace
        self._can_interface = can_interface
        self._ros2 = ros2
        self._clock = clock
        self._topic = f"/{namespace}/state/joint_motor_telemetry"
        self._publisher_factory = publisher_factory or partial(
            RosMonitorPublisher, node_prefix="piper_motor_telemetry_",
        )
        self._lock = threading.Lock()
        self._transport = None
        self._running = False
        self._generation = 0
        self._publish_error = None

    def _topics(self):
        return [{"topic": self._topic, "format": "data/json"}]

    def get_tool(self):
        definition = tool(
            TOOL_NAME,
            "sensor",
            "Read-only PiPER motor and driver temperatures, driver voltage and current for each joint.",
            {"type": "object", "properties": {}, "additionalProperties": False},
            topic_out=self._topics(),
        )
        definition["title"] = "Joint Motor Telemetry"
        return definition

    def _sample(self):
        samples = self._read_feedback()
        now = self._clock()
        result = {"can_interface": self._can_interface, "feedback_status": "unavailable"}
        valid_count = 0
        for index in range(len(MOTOR_FEEDBACK_IDS)):
            sample = samples[index] if index < len(samples) else None
            age = None if sample is None else now - sample.received_at_unix_s
            row = {
                "status": "unavailable",
                "motor_temperature_c": None,
                "driver_temperature_c": None,
                "driver_voltage_v": None,
                "driver_current_a": None,
                "received_at_unix_s": None,
            }
            if age is not None and 0 <= age <= MAX_FEEDBACK_AGE_S:
                row.update(asdict(sample), status="fresh")
                valid_count += 1
            elif age is not None and age > MAX_FEEDBACK_AGE_S:
                row["status"] = "stale"
            result[f"J{index + 1}"] = row
        if valid_count:
            result["feedback_status"] = "fresh" if valid_count == len(MOTOR_FEEDBACK_IDS) else "partial"
        return result

    def start(self):
        with self._lock:
            if not self._running:
                self._generation += 1
                generation = self._generation
                if self._ros2 is not None:
                    self._transport = self._publisher_factory(
                        self._ros2, self._namespace, self._topics(), 1 / PUBLISH_HZ,
                        lambda: self._tick(generation),
                    )
                self._running = True
            return {"state": "running", "topic_out": self._topics()}

    def stop(self):
        with self._lock:
            transport, self._transport = self._transport, None
            self._running = False
            self._publish_error = None
        if transport is not None:
            transport.close()
        return {"state": "idle", "topic_out": self._topics()}

    def info(self):
        with self._lock:
            state = "error" if self._publish_error else "running" if self._running else "idle"
            return {
                "state": state,
                "publishing": self._transport is not None,
                "publish_hz": PUBLISH_HZ,
                "topic_out": self._topics(),
                "publish_error": self._publish_error,
                "telemetry": self._sample(),
            }

    def _tick(self, generation):
        with self._lock:
            if self._transport is None or generation != self._generation:
                return
            try:
                self._transport.publish("data/json", self._sample())
                self._publish_error = None
            except Exception as exc:
                self._publish_error = str(exc)

    def dispatch(self, action, args):
        allowed = {"action", "_tool_name", "instance_id", "trace_id", "_trace_id"}
        if args.get("input_topic") == "":
            allowed.add("input_topic")
        if set(args) - allowed:
            raise ValueError("Joint Motor Telemetry only accepts read-only sensor lifecycle parameters.")
        if action == TOOL_NAME:
            action = "info"
        if action in {"start", "stop", "info"}:
            return getattr(self, action)()
        raise ValueError("Joint Motor Telemetry supports only start, stop and info.")
