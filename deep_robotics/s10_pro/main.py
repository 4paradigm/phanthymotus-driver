#!/usr/bin/env python3
"""S10 Pro M20-style card bundle with isolated read-only observation."""
from __future__ import annotations

import json
import logging
import os
import signal
import threading
from contextlib import ExitStack
from http.server import ThreadingHTTPServer
from pathlib import Path
import xml.etree.ElementTree as ET

from common.vendor_runtime import DriverBundle, load_config, make_handler, resolve_namespace, start_registration
from bridge import ReaderBridge
from device import build_plugins
from motion_process import DDSProcessTransport


def build_runtime_plugins(config, namespace, bridge, publish, *, motion_bridge=None):
    """Bind the one motion controller; construction performs no robot I/O."""
    from loco_move import LocoMovePlugin
    host_enabled = os.environ.get('S10_MOTION_ENABLED', '1') == '1'
    if os.environ.get('S10_EXECUTION_AUTHORIZATION_FILE'):
        logging.warning('Legacy execution mode ignored; motion disabled, monitors available')
        host_enabled = False
    motion_config = dict(config.get('motion', {}))
    motion_config.pop('auto_navigation', None)  # Ignore the removed option in existing site files.
    motion_plugin = LocoMovePlugin(motion_config, namespace,
        motion_bridge or bridge, lambda: DDSProcessTransport(config, execution_enabled=host_enabled),
        host_enabled=host_enabled)
    plugins = build_plugins(config, namespace, bridge, publish, motion=motion_plugin,
                            motion_bridge=motion_bridge)
    return [*plugins, motion_plugin]


def validate_platform_profile(path):
    """Fail closed if the platform profile does not confine UDP to loopback."""
    root = ET.parse(path).getroot()
    # Ignore XML namespace spelling; the repository profile uses a namespace.
    for element in root.iter():
        element.tag = element.tag.split("}")[-1]
    transports = root.findall(".//transport_descriptor")
    allowed = set()
    for transport in transports:
        addresses = [p.text.strip() for p in transport.findall("interfaceWhiteList/address") if p.text]
        if transport.findtext("type") != "UDPv4" or not addresses or set(addresses) != {"127.0.0.1"}:
            raise ValueError("platform DDS transport must be UDPv4 limited to 127.0.0.1")
        allowed.add(transport.findtext("transport_id"))
    defaults = [p for p in root.findall(".//participant") if p.get("is_default_profile") == "true"]
    if not defaults:
        raise ValueError("platform DDS requires a default participant profile")
    for participant in defaults:
        refs = {p.text for p in participant.findall("rtps/userTransports/transport_id")}
        if participant.findtext("rtps/useBuiltinTransports") != "false" or not refs or not refs <= allowed:
            raise ValueError("platform DDS must disable builtin transports and select the loopback transport")


class PlatformBus:
    def __init__(self, domain_id):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String, UInt8MultiArray
        self.rclpy = rclpy
        self.String, self.Bytes = String, UInt8MultiArray
        rclpy.init(domain_id=domain_id)
        self.node = rclpy.create_node("s10_pro_platform", enable_rosout=False, start_parameter_services=False)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.publishers = {}
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.executor.spin, daemon=True, name="s10-platform-dds")
        self.thread.start()

    def publish(self, topic, fmt, payload):
        with self.lock:
            key = (topic, fmt)
            if key not in self.publishers:
                cls = self.Bytes if fmt == "sensor/pointcloud" else self.String
                self.publishers[key] = self.node.create_publisher(cls, topic, 1)
            if fmt == "sensor/pointcloud":
                msg = self.Bytes()
                msg.data = payload
            elif fmt in ("data/json", "sensor/skeleton"):
                msg = self.String()
                msg.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
            else:
                raise ValueError("unsupported platform format")
            self.publishers[key].publish(msg)

    def close(self):
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.node.destroy_node()
        self.rclpy.shutdown()


def main():
    from common import logsafe
    logsafe.install()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    config = load_config(__file__)
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).with_name("config.yaml")))
    profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "/opt/phanthy-motus/dds-local.xml")
    if not Path(profile).is_file():
        raise RuntimeError("platform DDS loopback profile is missing; refusing unisolated startup")
    validate_platform_profile(profile)
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    for key in ("FASTDDS_DEFAULT_PROFILES_FILE", "FASTDDS_BUILTIN_TRANSPORTS"):
        os.environ.pop(key, None)
    domain = int(os.environ.get("ROS_DOMAIN_ID", config.get("core_domain_id", 42)))
    namespace = resolve_namespace(config)
    # Register each cleanup immediately so a later startup failure also reaps
    # both independent reader supervisors and the resources already created.
    with ExitStack() as cleanup:
        bus = PlatformBus(domain)
        cleanup.callback(bus.close)
        bridge = ReaderBridge(config, config_path)
        cleanup.callback(bridge.close)
        motion_bridge = ReaderBridge(config, config_path)
        cleanup.callback(motion_bridge.close)
        bundle = DriverBundle(build_runtime_plugins(config, namespace, bridge, bus.publish,
                                                   motion_bridge=motion_bridge))
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
        logging.info("S10 Pro monitoring cards and gated motion; namespace=%s port=%d domain=%d motion_enabled=%s", namespace, port, domain, config.get("motion", {}).get("enabled", False))
        server.serve_forever()


if __name__ == "__main__":
    main()
