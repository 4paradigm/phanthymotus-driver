#!/usr/bin/env python3
"""UniX AI Panther MCP driver entry point."""

from __future__ import annotations

import os
import signal
import threading
from http.server import ThreadingHTTPServer

from common import logsafe
from common.vendor_runtime import DriverBundle, load_config, make_handler, resolve_namespace, start_registration

logsafe.install()


def main() -> None:
    import rclpy
    import rclpy.executors

    from device import PantherDevice
    from plugins import build_plugins

    config = load_config(__file__)
    namespace = resolve_namespace(config)
    port = int(config["mcp_port"])
    body_domain = int(
        os.environ.get("PANTHER_ROS_DOMAIN_ID", config.get("ros", {}).get("robot_domain_id", 0))
    )
    os.environ["ROS_DOMAIN_ID"] = str(body_domain)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

    rclpy.init(domain_id=body_domain)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=6)
    device = PantherDevice(config, namespace, executor)
    spin_thread = threading.Thread(target=executor.spin, daemon=True, name="panther_ros")
    spin_thread.start()
    device.start()
    bundle = DriverBundle(build_plugins(config, namespace, device))
    bundle.start_all()

    server = ThreadingHTTPServer(
        ("", port),
        make_handler(
            lambda: bundle,
            "unixai-panther-device-bundle",
            "unixai-panther-driver",
        ),
    )
    # Bind the MCP socket before advertising it. Agent Core immediately calls
    # initialize/tools/list when a driver registers; advertising first creates
    # a race that leaves a persisted offline entry and no cards on the canvas.
    start_registration(port, config, "unixai-panther-driver")
    stopping = threading.Event()

    def shutdown(signum, _frame):
        if stopping.is_set():
            return
        stopping.set()
        print(f"[bundle] signal {signum}; stopping Panther driver", flush=True)
        bundle.stop_all()
        device.close()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print(
        f"[bundle] Panther MCP port={port} namespace={namespace} "
        f"robot_domain={body_domain} core_domain={os.environ.get('AGENT_CORE_ROS_DOMAIN_ID', '42')}",
        flush=True,
    )
    try:
        server.serve_forever()
    finally:
        if not stopping.is_set():
            bundle.stop_all()
            device.close()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

