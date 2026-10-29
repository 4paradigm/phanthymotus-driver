#!/usr/bin/env python3
"""云深处山猫 S10 Driver 入口。"""

import fcntl
import os
import socket
import struct
from pathlib import Path

from common.vendor_runtime import run_driver
from device import build_plugins


def configure_fastdds_profile(config):
    """Constrain both FastDDS contexts to loopback and the robot-facing NIC."""
    interface = os.environ.get("NETWORK_INTERFACE") or config.get("ros", {}).get("robot_interface", "eth0")
    if not interface or len(interface.encode()) > 15:
        raise ValueError(f"invalid robot network interface: {interface!r}")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        request = struct.pack("256s", interface.encode())
        address = socket.inet_ntoa(fcntl.ioctl(sock.fileno(), 0x8915, request)[20:24])

    profile_path = Path(os.environ.get("S10_FASTDDS_PROFILE", "/tmp/lynx-s10-fastdds.xml"))
    profile_path.write_text(f'''<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com"><profiles>
  <transport_descriptors><transport_descriptor>
    <transport_id>s10_robot_and_loopback</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>127.0.0.1</address><address>{address}</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="s10_constrained" is_default_profile="true"><rtps>
    <userTransports><transport_id>s10_robot_and_loopback</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps></participant>
</profiles></dds>''', encoding="utf-8")
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile_path)
    print(f"[ros2] FastDDS restricted to loopback and {interface} ({address}); "
          "ensure this interface is on the isolated robot network", flush=True)
    return profile_path

if __name__ == "__main__":
    import yaml

    config_path = Path(os.environ.get("CONFIG_PATH", Path(__file__).with_name("config.yaml")))
    with config_path.open(encoding="utf-8") as config_file:
        configure_fastdds_profile(yaml.safe_load(config_file) or {})
    run_driver(__file__, "deep-robotics-lynx-s10-driver", "deep-robotics-lynx-s10-device-bundle", build_plugins)
