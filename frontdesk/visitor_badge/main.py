#!/usr/bin/env python3
"""Front-desk visitor badge printer MCP driver. No ROS: it only speaks SSH."""

import signal
import threading
from http.server import ThreadingHTTPServer

from common import logsafe
from common.vendor_runtime import DriverBundle, load_config, make_handler, start_registration
from device import build_plugins

DRIVER_ID = "frontdesk-visitor-badge-driver"


def main() -> None:
    logsafe.install()
    config = load_config(__file__)
    port = int(config["mcp_port"])
    bundle = DriverBundle(build_plugins(config))
    start_registration(port, config, DRIVER_ID)
    server = ThreadingHTTPServer(("", port), make_handler(lambda: bundle, "frontdesk-visitor-badge", DRIVER_ID))
    # shutdown() blocks until serve_forever returns, so not on the serving thread.
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    print(f"[bundle] {DRIVER_ID} port={port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
