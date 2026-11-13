"""Two first-PR Watch7 Sensor cards, sharing one DeviceStore and ROS2 bus."""
from __future__ import annotations

import json
import threading


class SensorPlugin:
    def __init__(self, name: str, description: str, topic: str, device, ros_publisher):
        self.name = name
        self.description = description
        self.topic = topic
        self.device = device
        self._publisher = ros_publisher
        self._lock = threading.Lock()
        self._running = False

    def get_tool(self) -> dict:
        return {
            "name": self.name, "type": "sensor", "multiInstance": False,
            "description": self.description,
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"]}},
                "required": ["action"],
            },
            "topic_out": [{"topic": self.topic, "format": "data/json"}],
        }

    def start(self) -> None:
        with self._lock:
            self._running = True

    def stop(self) -> None:
        with self._lock:
            self._running = False

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            with self._lock:
                running = self._running
            snapshot = (self.device.heart_snapshot() if self.name == "heart_rate"
                        else self.device.device_snapshot())
            return {
                "state": "running" if running else "idle",
                "device": "watch7", "snapshot": snapshot,
                "topic_out": [{"topic": self.topic, "format": "data/json"}],
                "on_watch_sampling_required": True,
            }
        return None

    def publish(self, event: dict) -> None:
        # Hold the same lifecycle lock through publication: no message can be
        # emitted after stop() has returned, including from concurrent HTTP calls.
        with self._lock:
            if not self._running:
                return
            self._publisher(json.dumps(event, separators=(",", ":"), ensure_ascii=False))


def build_plugins(device, namespace: str, publish_factory, config: dict) -> list[SensorPlugin]:
    plugins = []
    specs = [
        ("heart_rate", "Live Watch7 heart-rate Sensor. No data is invented when offline.",
         f"/{namespace}/watch7/heart_rate"),
        ("device_state", "Watch7 device heartbeat, wear state, and battery.",
         f"/{namespace}/watch7/device_state"),
    ]
    enabled = config.get("plugins", {})
    for name, description, topic in specs:
        if enabled.get(name, {}).get("enabled", False):
            plugins.append(SensorPlugin(name, description, topic, device, publish_factory(topic)))
    return plugins
