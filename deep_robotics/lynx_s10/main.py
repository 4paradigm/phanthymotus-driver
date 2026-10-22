#!/usr/bin/env python3
"""云深处山猫 S10 Driver 入口；原生协议驱动不启动 ROS。"""

from common import logsafe

logsafe.install()

import signal
from http.server import ThreadingHTTPServer
from threading import Event

from common.vendor_runtime import DriverBundle, load_config, make_handler, start_registration
from device import build_plugins

if __name__ == "__main__":
    driver_id = "deep-robotics-lynx-s10-driver"
    server_name = "deep-robotics-lynx-s10-device-bundle"
    config = load_config(__file__)
    port = int(config["mcp_port"])
    bundle = DriverBundle(build_plugins(config))
    bundle.start_all()
    start_registration(port, config, driver_id)
    server = ThreadingHTTPServer(("", port), make_handler(lambda: bundle, server_name, driver_id))
    stopping = Event()

    def shutdown(signum, _frame):
        if stopping.is_set():
            return
        stopping.set()
        bundle.stop_all()
        server.shutdown()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        server.serve_forever()
    finally:
        bundle.stop_all()
        server.server_close()
