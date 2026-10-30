#!/usr/bin/env python3
"""S10 Pro single-card HTTP driver with isolated robot DDS processes."""
from __future__ import annotations

import logging
import os
import signal
import threading
from contextlib import ExitStack
from http.server import ThreadingHTTPServer
from pathlib import Path

from common.vendor_runtime import DriverBundle, load_config, make_handler, start_registration
from bridge import ReaderBridge
from motion_process import DDSProcessTransport


def build_runtime_plugins(config, bridge):
    """Construct the sole locomotion card without robot I/O."""
    from loco_move import LocoMovePlugin
    host_enabled = os.environ.get('S10_MOTION_ENABLED', '1') == '1'
    if os.environ.get('S10_EXECUTION_AUTHORIZATION_FILE'):
        logging.warning('Legacy execution mode ignored; motion disabled')
        host_enabled = False
    motion_config = dict(config.get('motion', {}))
    motion_config.pop('auto_navigation', None)  # Ignore old site configuration.
    return [LocoMovePlugin(motion_config, bridge,
        lambda: DDSProcessTransport(config, execution_enabled=host_enabled),
        host_enabled=host_enabled)]


def main():
    from common import logsafe
    logsafe.install()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    config = load_config(__file__)
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).with_name("config.yaml")))
    # Register each cleanup immediately so a later startup failure also reaps
    # the reader supervisor and the resources already created.
    with ExitStack() as cleanup:
        bridge = ReaderBridge(config, config_path)
        cleanup.callback(bridge.close)
        bundle = DriverBundle(build_runtime_plugins(config, bridge))
        for plugin in bundle.plugins:
            if hasattr(plugin, "close"):
                cleanup.callback(plugin.close)
        cleanup.callback(bundle.stop_all)
        port = int(os.environ.get("MCP_PORT", config["mcp_port"]))
        server = ThreadingHTTPServer((str(config.get("bind_host", "127.0.0.1")), port),
                                     make_handler(lambda: bundle, "s10-pro-bundle", "deep-robotics-s10-pro-driver"))
        cleanup.callback(server.server_close)
        server.daemon_threads = True
        stop = threading.Event()

        def shutdown(signum, frame):
            if not stop.is_set():
                stop.set()
                threading.Thread(target=server.shutdown, daemon=True).start()

        signal.signal(signal.SIGTERM, shutdown)
        signal.signal(signal.SIGINT, shutdown)
        # Canvas start/stop owns card lifecycle; startup sends no robot commands.
        if config.get("register", True):
            start_registration(port, config, "deep-robotics-s10-pro-driver")
        logging.info("S10 Pro loco_move; port=%d motion_enabled=%s", port, config.get("motion", {}).get("enabled", False))
        server.serve_forever()


if __name__ == "__main__":
    main()
