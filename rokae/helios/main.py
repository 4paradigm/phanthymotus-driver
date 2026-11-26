"""Single-domain ROS publisher + loopback MCP server; no vendor DDS participant."""

from common import logsafe

logsafe.install()

import os
import signal
import threading
from http.server import ThreadingHTTPServer
from types import SimpleNamespace

from common.vendor_runtime import DriverBundle, load_config, make_handler, resolve_namespace, start_registration
from .device import HeliosJoints
from .battery import HeliosBattery
from .spatial_map import HeliosSpatialMap


def serve(config, plugin_classes, server_name, driver_id):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    context = Context()
    rclpy.init(context=context, domain_id=int(os.environ.get("ROS_DOMAIN_ID", "42")))
    executor = SingleThreadedExecutor(context=context)
    ros2 = SimpleNamespace(ctx_core=context, executor_core=executor)
    namespace = resolve_namespace(config)
    plugins = [plugin(config, namespace, ros2) for plugin in plugin_classes]
    bundle = DriverBundle(plugins)
    port = int(config["mcp_port"])
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(lambda: bundle, server_name, driver_id))
    stopping = threading.Event()

    def shutdown(_signum, _frame):
        if not stopping.is_set():
            stopping.set()
            threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        for plugin in plugins:
            plugin.start()  # Fail startup if model or DDS initialization fails.
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()
        start_registration(port, config, driver_id)
        print(f"[helios] MCP listening on 127.0.0.1:{port}", flush=True)
        server.serve_forever()
    finally:
        executor.shutdown()
        bundle.stop_all()
        server.server_close()
        rclpy.shutdown(context=context)


def main():
    serve(load_config(__file__), (HeliosJoints, HeliosBattery, HeliosSpatialMap),
          "rokae-helios", "rokae-helios-driver")


if __name__ == "__main__":
    main()
