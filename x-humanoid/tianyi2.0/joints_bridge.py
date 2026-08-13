#!/usr/bin/env python3
"""Dedicated ROS process for the Tianyi joints skeleton stream."""

import os
import re
import signal
import socket
import threading
import time
from pathlib import Path

import yaml
import rclpy
import rclpy.executors
from rclpy.context import Context

from device import StatePlugin


class BridgeROS2:
    def __init__(self):
        dds_profile = "/work/dds_profile.xml"
        if os.path.exists(dds_profile):
            os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = dds_profile
        self.ctx_tianyi = Context()
        rclpy.init(context=self.ctx_tianyi, domain_id=0)
        self.executor_tianyi = rclpy.executors.MultiThreadedExecutor(context=self.ctx_tianyi)
        os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
        self.ctx_core = Context()
        rclpy.init(context=self.ctx_core, domain_id=42)
        self.executor_core = rclpy.executors.MultiThreadedExecutor(context=self.ctx_core)
        self._threads = []

    def start(self):
        def spin(executor):
            while rclpy.ok(context=executor.context):
                executor.spin_once(timeout_sec=0.1)

        for executor in (self.executor_tianyi, self.executor_core):
            thread = threading.Thread(target=spin, args=(executor,), daemon=True)
            thread.start()
            self._threads.append(thread)

    def stop(self):
        self.executor_tianyi.shutdown()
        self.executor_core.shutdown()
        rclpy.shutdown(context=self.ctx_tianyi)
        rclpy.shutdown(context=self.ctx_core)


def main():
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as stream:
        cfg = yaml.safe_load(stream) or {}
    namespace = cfg.get("ros_namespace", "").strip() or socket.gethostname()
    namespace = re.sub(r"[^a-zA-Z0-9_]", "_", namespace)
    state_cfg = dict(cfg.get("plugins", {}).get("state", {}))
    state_cfg.update({"joints_only": True, "publish_joints": True})

    ros2 = BridgeROS2()
    state = StatePlugin(state_cfg, namespace, ros2)
    state.start()
    ros2.start()
    print(f"[joints-bridge] publishing /{namespace}/state/joints", flush=True)
    stop_requested = threading.Event()
    stopped = False

    def stop(*_args):
        stop_requested.set()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while not stop_requested.wait(timeout=1):
            pass
    finally:
        if not stopped:
            stopped = True
            state.stop()
            ros2.stop()


if __name__ == "__main__":
    main()
