#!/usr/bin/env python3
"""Forward X2 read-only MCP sensor cards into Agent Core's ROS 2 domain.

This bridge polls the local agibot-x2 MCP server for sensor card snapshots and
republishes their read-only payloads on domain 42. It mirrors the Q5 bridge
pattern: the bridge owns the agent-core-facing DDS side, while the driver keeps
the robot-side subscriptions and snapshot generation.
"""

from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Callable

try:
    from common import logsafe
    logsafe.install()
except ImportError as exc:  # running outside the driver image
    import sys
    sys.stderr.write(f"[x2-bridge] logsafe unavailable ({exc}); stdout unprotected\n")

DEFAULT_DRIVER_URL = "http://127.0.0.1:15717/mcp"
DEFAULT_POLL_HZ = 10.0
DEFAULT_REFRESH_SECONDS = 30.0
DEFAULT_TIMEOUT_SECONDS = 2.0
DEFAULT_FAILURE_LOG_INTERVAL = 30.0
DEFAULT_FASTDDS_PROFILE = Path(__file__).with_name("resource") / "fastdds_bridge_local.xml"


def configure_fastdds_transport() -> str | None:
    """Provide a loopback-only Fast DDS profile only when none was injected."""
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
        return None
    configured = os.environ.get("FASTDDS_DEFAULT_PROFILES_FILE")
    if configured:
        return configured
    if not DEFAULT_FASTDDS_PROFILE.is_file():
        return None
    profile = str(DEFAULT_FASTDDS_PROFILE)
    os.environ["FASTDDS_DEFAULT_PROFILES_FILE"] = profile
    os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", profile)
    return profile


def select_sensor_tools(tools: Any) -> dict[str, list[str]]:
    selected: dict[str, list[str]] = {}
    if not isinstance(tools, list):
        return selected
    for tool in tools:
        if not isinstance(tool, dict) or tool.get("type") != "sensor":
            continue
        name = tool.get("name")
        if not isinstance(name, str) or not name:
            continue
        topics = []
        for entry in tool.get("topic_out") or []:
            topic = entry.get("topic") if isinstance(entry, dict) else None
            fmt = entry.get("format") if isinstance(entry, dict) else None
            if isinstance(topic, str) and topic and fmt in {"data/json", "sensor/skeleton"} and topic not in topics:
                topics.append(topic)
        if topics:
            selected[name] = topics
    return selected


def extract_data_payload(response: Any) -> Any:
    if not isinstance(response, dict):
        raise ValueError("MCP response is not an object")
    result = response.get("result")
    content = result.get("content") if isinstance(result, dict) else None
    if not isinstance(content, list) or not content:
        raise ValueError("MCP response has no content")
    first = content[0]
    text = first.get("text") if isinstance(first, dict) else None
    if not isinstance(text, str):
        raise ValueError("MCP response content is not text")
    payload = json.loads(text)
    if not isinstance(payload, dict) or "data" not in payload:
        raise ValueError("MCP tool result has no data field")
    return payload["data"]


class McpClient:
    def __init__(self, url: str, timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS):
        self._url = url
        self._timeout_seconds = timeout_seconds
        self._request_id = 0

    def _call(self, method: str, params: dict[str, Any]) -> dict[str, Any]:
        self._request_id += 1
        payload = json.dumps({
            "jsonrpc": "2.0", "id": self._request_id,
            "method": method, "params": params,
        }).encode("utf-8")
        request = urllib.request.Request(
            self._url, data=payload,
            headers={"Content-Type": "application/json", "Accept": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self._timeout_seconds) as response:
            decoded = json.loads(response.read().decode("utf-8"))
        if not isinstance(decoded, dict):
            raise ValueError("MCP endpoint returned a non-object response")
        if "error" in decoded:
            raise RuntimeError(f"MCP error: {decoded['error']}")
        return decoded

    def list_tools(self) -> list[dict[str, Any]]:
        response = self._call("tools/list", {})
        result = response.get("result")
        tools = result.get("tools") if isinstance(result, dict) else None
        if not isinstance(tools, list):
            raise ValueError("MCP tools/list response has no tools array")
        return tools

    def sensor_info(self, name: str) -> Any:
        response = self._call("tools/call", {"name": name, "arguments": {"action": "info"}})
        return extract_data_payload(response)


class SensorBusBridge:
    def __init__(
        self,
        mcp: Any,
        publish: Callable[[str, str], None],
        log_warning: Callable[[str], None] | None = None,
        log_info: Callable[[str], None] | None = None,
    ):
        self._mcp = mcp
        self._publish = publish
        self._log_warning = log_warning or (lambda message: None)
        self._log_info = log_info or (lambda message: None)
        self._sensors: dict[str, list[str]] = {}
        self._failures: dict[str, tuple[float, str, int]] = {}

    @property
    def sensors(self) -> dict[str, list[str]]:
        return dict(self._sensors)

    def refresh(self) -> dict[str, list[str]]:
        self._sensors = select_sensor_tools(self._mcp.list_tools())
        return self.sensors

    def poll_once(self) -> int:
        published = 0
        for name, topics in self._sensors.items():
            try:
                encoded = json.dumps(self._mcp.sensor_info(name), ensure_ascii=False)
                for topic in topics:
                    self._publish(topic, encoded)
                    published += 1
            except Exception as exc:
                now = time.monotonic()
                message = str(exc).replace("\n", " ")[:240]
                previous = self._failures.get(name)
                count = (previous[2] if previous else 0) + 1
                should_log = (
                    previous is None
                    or previous[1] != message
                    or now - previous[0] >= DEFAULT_FAILURE_LOG_INTERVAL
                )
                if should_log:
                    self._log_warning(
                        f"sensor {name} failed ({count} consecutive polls): {message}"
                    )
                    self._failures[name] = (now, message, count)
                else:
                    self._failures[name] = (previous[0], previous[1], count)
            else:
                previous = self._failures.pop(name, None)
                if previous is not None:
                    self._log_info(
                        f"sensor {name} recovered after {previous[2]} failed polls"
                    )
        return published


def _positive_float(name: str, default: float) -> float:
    try:
        value = float(os.environ.get(name, default))
    except ValueError:
        value = default
    return value if value > 0 else default


def main() -> None:
    fastdds_profile = configure_fastdds_transport()
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit(f"[x2-bridge] ROS 2 String support is required: {exc}")

    driver_url = os.environ.get("X2_DRIVER_URL", DEFAULT_DRIVER_URL)
    poll_hz = _positive_float("X2_BRIDGE_POLL_HZ", DEFAULT_POLL_HZ)
    refresh_seconds = _positive_float("X2_BRIDGE_REFRESH_SECONDS", DEFAULT_REFRESH_SECONDS)
    timeout_seconds = _positive_float("X2_BRIDGE_HTTP_TIMEOUT_SECONDS", DEFAULT_TIMEOUT_SECONDS)

    rclpy.init()
    node = Node("agibot_x2_sensor_bus_bridge")
    publishers: dict[str, Any] = {}

    def publish(topic: str, data: str) -> None:
        publisher = publishers.get(topic)
        if publisher is None:
            publisher = node.create_publisher(String, topic, 10)
            publishers[topic] = publisher
            node.get_logger().info(f"bridging X2 sensor topic {topic}")
        message = String()
        message.data = data
        publisher.publish(message)

    bridge = SensorBusBridge(
        McpClient(driver_url, timeout_seconds),
        publish,
        log_warning=node.get_logger().warning,
        log_info=node.get_logger().info,
    )
    last_refresh = 0.0

    def tick() -> None:
        nonlocal last_refresh
        now = time.monotonic()
        if now - last_refresh >= refresh_seconds:
            try:
                sensors = bridge.refresh()
                last_refresh = now
                node.get_logger().info(f"discovered {len(sensors)} X2 sensor tools")
            except (OSError, urllib.error.URLError, ValueError, RuntimeError) as exc:
                node.get_logger().warning(f"MCP discovery failed: {exc}")
                return
        bridge.poll_once()

    node.create_timer(1.0 / poll_hz, tick)
    print(
        f"[x2-bridge] {driver_url} -> ROS Domain {os.environ.get('ROS_DOMAIN_ID', 'unset')} "
        f"at {poll_hz:g}Hz; Fast DDS profile={fastdds_profile or 'deployment-default'}",
        flush=True,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
