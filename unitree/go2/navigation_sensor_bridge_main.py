#!/usr/bin/env python3
"""Isolated MID360 bridge worker for the Go2 Driver navigation sensor card."""

from __future__ import annotations

try:
    from common import logsafe
    logsafe.install(check_fd=False)
except ImportError:
    pass

import os
import re
import signal
import socket
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml

from navigation_sensor_bridge import _NavigationSensorNode
from unitree_sdk2py.core.channel import ChannelFactoryInitialize


def _load_config() -> dict:
    path = Path(os.environ.get("CONFIG_PATH", Path(__file__).with_name("config.yaml")))
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"navigation bridge config must be a mapping: {path}")
    return config


def _resolve_namespace(config: dict) -> str:
    value = os.environ.get("ROS_NAMESPACE", "").strip()
    if not value:
        value = str(config.get("ros_namespace", "")).strip()
    if not value:
        value = socket.gethostname()
    value = re.sub(r"[^a-zA-Z0-9_]", "_", value.strip("/"))
    if not value:
        raise ValueError("ROS namespace resolves to an empty value")
    return value


def main() -> int:
    network_interface = (
        sys.argv[1] if len(sys.argv) > 1 else os.environ.get("NETWORK_INTERFACE", "eth0")
    )
    config = _load_config()
    plugin_config = config.get("plugins", {}).get("navigation_sensors", {})
    if not plugin_config.get("enabled", False):
        raise RuntimeError("plugins.navigation_sensors.enabled must be true")

    namespace = _resolve_namespace(config)
    ChannelFactoryInitialize(0, network_interface)
    print(
        f"[navigation-sensors-worker] DDS initialized on {network_interface}; "
        f"ROS namespace={namespace}",
        flush=True,
    )

    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=3)
    node = _NavigationSensorNode(plugin_config, namespace)
    executor.add_node(node)
    stop = threading.Event()

    def _request_stop(signum, _frame):
        print(
            f"[navigation-sensors-worker] signal {signum}, shutting down",
            flush=True,
        )
        stop.set()

    signal.signal(signal.SIGTERM, _request_stop)
    signal.signal(signal.SIGINT, _request_stop)

    try:
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=0.2)
    finally:
        node.close()
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
