"""Real guard-only Bundle/MCP server for an isolated local Docker network.

This validation bootstrap imports main's actual classes, never calls main(),
never loads a vendor controller or Camera Card, and uses the real ROS Runtime.
Run without host networking, devices, Docker/DBus sockets or privileged access.
"""
from __future__ import annotations

import argparse
from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import sys
import threading

os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
os.environ.setdefault("ROS_DOMAIN_ID", "219")

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
import main as bumi_main
import image_quality_guard
import image_quality_runtime


def run(args):
    for module in (bumi_main, image_quality_guard, image_quality_runtime):
        if Path(module.__file__).resolve().parent != Path("/work"):
            raise RuntimeError("validation must load actual image modules from /work")
    event = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: event.set())
    signal.signal(signal.SIGINT, lambda *_: event.set())
    context = Context()
    rclpy.init(context=context)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    bundle = server = None
    ros_thread = http_thread = None
    try:
        bundle = bumi_main.BumiDeviceBundle({"plugins": {
            "image_quality_guard": {"enabled": True}}}, args.namespace, executor, None, None)
        bumi_main._bundle = bundle
        bundle.start_all()
        tools = bundle.get_all_tools()
        assert [tool["name"] for tool in tools] == ["image_quality_guard"], tools
        assert not any(name in sys.modules for name in
                       ("highcontrol_py", "mediacontrol_py", "pyrealsense2"))
        assert executor.get_nodes() == [], "boot hook must not create a subscription"
        server = ThreadingHTTPServer(("0.0.0.0", args.port), bumi_main.make_handler())
        ros_thread = threading.Thread(target=executor.spin, name="iqg-local-ros")
        http_thread = threading.Thread(target=server.serve_forever, name="iqg-local-mcp")
        ros_thread.start()
        http_thread.start()
        print(json.dumps({"probe": "GUARD_MCP_READY", "port": server.server_port,
                          "tools": [tool["name"] for tool in tools],
                          "boot_nodes": 0, "sdk_loaded": False}), flush=True)
        if not event.wait(args.max_seconds):
            raise RuntimeError("bounded local integration server timed out")
    finally:
        cleanup_errors = []
        if bundle is not None:
            bundle.stop_all()
            bundle.stop_all()
        if server is not None:
            if http_thread is not None and http_thread.is_alive():
                server.shutdown()
            server.server_close()
        if http_thread is not None:
            http_thread.join(10)
        nodes = len(executor.get_nodes())
        resources_pending = bool(bundle and bundle._plugins[0].info()["resources_pending"])
        stopped = executor.shutdown(timeout_sec=10)
        if ros_thread is not None:
            ros_thread.join(10)
        http_alive = bool(http_thread and http_thread.is_alive())
        ros_alive = bool(ros_thread and ros_thread.is_alive())
        context.shutdown()
        if nodes or resources_pending or not stopped or http_alive or ros_alive:
            cleanup_errors.append("owned resources or validation worker did not stop")
        print(json.dumps({"probe": "GUARD_MCP_CLEANUP", "nodes": nodes,
                          "resources_pending": resources_pending, "executor_stopped": stopped,
                          "http_worker_alive": http_alive, "ros_worker_alive": ros_alive,
                          "errors": cleanup_errors}), flush=True)
        if cleanup_errors:
            raise RuntimeError("guard-only integration server cleanup failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=15704)
    parser.add_argument("--namespace", default="iqg_local")
    parser.add_argument("--max-seconds", type=float, default=1800)
    args = parser.parse_args()
    if not 0 < args.port < 65536 or not 0 < args.max_seconds <= 7200:
        parser.error("port or maximum runtime is out of range")
    run(args)


if __name__ == "__main__":
    main()
