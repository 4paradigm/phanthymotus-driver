"""Company-compliant Watch7 MCP/ROS2 Driver with a separate Watch ingress.

MCP is loopback-only; the authenticated companion data plane is LAN-facing.
No Windows Relay, ADB, Agent Core patches, or research SDK required at runtime.
"""
from __future__ import annotations

import os
import signal
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path

from common.vendor_runtime import DriverBundle, make_handler, resolve_namespace
from device import DeviceStore
from sensors import build_plugins
from transport import make_device_server

DRIVER_ID = "samsung-watch7-driver"


def resolve_mcp_bind_host() -> str:
    """Only allow LAN-visible MCP in an explicitly isolated test network.

    Never publish an unauthenticated MCP port on a public or shared LAN.
    Production deployment should use the default loopback-only endpoint
    and an Agent Core sharing the host network namespace.
    """
    host = os.environ.get("WATCH7_MCP_BIND_HOST", "127.0.0.1").strip()
    if host == "127.0.0.1":
        return host
    if host == "0.0.0.0" and os.environ.get("WATCH7_PRIVATE_DOCKER_TEST") == "1":
        return host
    raise RuntimeError(
        "Non-loopback MCP requires WATCH7_PRIVATE_DOCKER_TEST=1 "
        "and an unpublished Docker-only bridge endpoint"
    )


def load_device_token() -> str:
    """Fail closed unless the operator provisioned an external device credential."""
    token = os.environ.get("WATCH7_DEVICE_TOKEN", "")
    filename = os.environ.get("WATCH7_DEVICE_TOKEN_FILE")
    if not token and filename:
        try:
            token = Path(filename).read_text(encoding="utf-8").strip()
        except OSError:
            raise RuntimeError("Watch7 device token file unavailable") from None
    if len(token) < 16:
        raise RuntimeError("Set WATCH7_DEVICE_TOKEN (>=16 characters) or token file")
    return token


def load_config(path: str | None = None) -> dict:
    import yaml

    filename = path or os.environ.get("CONFIG_PATH") or str(
        Path(__file__).with_name("config.yaml")
    )
    with open(filename, encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def create_bundle(device: DeviceStore, namespace: str, publish_factory, config: dict):
    plugins = build_plugins(device, namespace, publish_factory, config)
    return DriverBundle(plugins), {plugin.name: plugin for plugin in plugins}


def run() -> None:
    token = load_device_token()

    # Avoid starting ROS2 or any socket if the required watch credential is missing.
    cfg = load_config()
    namespace = resolve_namespace(cfg)
    mcp_port = int(cfg["mcp_port"])
    ingress_port = int(cfg["ingress_port"])
    if not (15700 <= mcp_port <= 15799 and 1 <= ingress_port <= 65535):
        raise ValueError("invalid MCP or ingress port")
    if mcp_port == ingress_port:
        raise ValueError("MCP and Watch ingress ports must differ")
    mcp_bind_host = resolve_mcp_bind_host()

    from common import logsafe
    logsafe.install()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("samsung_watch7_driver")
    publishers: dict[str, object] = {}

    def publish_factory(topic: str):
        pub = publishers.setdefault(
            topic, node.create_publisher(String, topic, qos_profile_sensor_data)
        )

        def publish_json(data: str) -> None:
            msg = String()
            msg.data = data
            pub.publish(msg)

        return publish_json

    plugins: dict[str, object] = {}
    device = DeviceStore(
        on_heart=lambda sample: plugins.get("heart_rate") and plugins["heart_rate"].publish(sample),
        on_device=lambda sample: plugins.get("device_state") and plugins["device_state"].publish(sample),
    )
    bundle, plugins = create_bundle(device, namespace, publish_factory, cfg)
    bundle.start_all()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin, name="watch7-ros2-executor", daemon=True
    )
    watch_server = None
    mcp_server = None
    ingress_thread = None
    try:
        executor_thread.start()
        watch_server = make_device_server(
            os.environ.get("WATCH7_INGRESS_HOST", cfg.get("ingress_host", "0.0.0.0")),
            ingress_port, device, token
        )
        mcp_server = ThreadingHTTPServer(
            (mcp_bind_host, mcp_port),
            make_handler(lambda: bundle, cfg.get("name", DRIVER_ID), DRIVER_ID),
        )
        ingress_thread = threading.Thread(
            target=watch_server.serve_forever, name="watch7-http", daemon=True
        )
        ingress_thread.start()

        # Follow the company's existing registry contract. The connection stays
        # host-local and does not transmit the Watch device token.
        if os.environ.get("WATCH7_REGISTER_AGENT_CORE", "1") == "1":
            from common.vendor_runtime import start_registration
            start_registration(mcp_port, cfg, DRIVER_ID)

        stop_event = threading.Event()

        def stop(_signum, _frame):
            if not stop_event.is_set():
                stop_event.set()
                threading.Thread(target=mcp_server.shutdown, daemon=True).start()

        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        print(
            f"[watch7] mcp={mcp_bind_host}:{mcp_port}, ingress={watch_server.server_address}, "
            f"namespace={namespace}; only heart_rate/device_state enabled",
            flush=True,
        )
        mcp_server.serve_forever(poll_interval=0.1)
    finally:
        if watch_server is not None:
            watch_server.shutdown()
            watch_server.server_close()
        if ingress_thread is not None:
            ingress_thread.join(timeout=3)
        if mcp_server is not None:
            mcp_server.server_close()
        bundle.stop_all()
        executor.shutdown(timeout_sec=2)
        if executor_thread.is_alive():
            executor_thread.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    run()
