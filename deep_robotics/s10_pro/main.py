#!/usr/bin/env python3
"""云深处山猫 S10 Driver 入口。"""

try:
    from common import logsafe
    logsafe.install()
except ImportError:
    pass

import fcntl
import ipaddress
import os
import socket
import struct
from pathlib import Path

from common.vendor_runtime import run_driver
from device import build_plugins


def _interface_ipv4(interface):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        request = struct.pack("256s", interface.encode())
        address = socket.inet_ntoa(fcntl.ioctl(sock.fileno(), 0x8915, request)[20:24])
        netmask = socket.inet_ntoa(fcntl.ioctl(sock.fileno(), 0x891b, request)[20:24])
    return ipaddress.ip_interface(f"{address}/{netmask}")


def configure_fastdds_profile(config):
    """Configure the robot process and a separate loopback-only core bridge."""
    ros_config = config.get("ros", {})
    interface = os.environ.get("NETWORK_INTERFACE") or ros_config.get("robot_interface")
    if not interface or len(interface.encode()) > 15:
        raise ValueError("NETWORK_INTERFACE must name the isolated S10-facing network interface")
    cidr = os.environ.get("ROBOT_NETWORK_CIDR") or ros_config.get("robot_network_cidr")
    if not cidr:
        raise ValueError("ROBOT_NETWORK_CIDR must explicitly identify the isolated robot subnet")
    try:
        robot_network = ipaddress.ip_network(cidr, strict=True)
        interface_address = _interface_ipv4(interface)
    except (ValueError, OSError) as exc:
        raise ValueError(f"invalid robot network selection: {exc}") from exc
    if not isinstance(robot_network, ipaddress.IPv4Network) or not robot_network.is_private:
        raise ValueError(f"ROBOT_NETWORK_CIDR must be a private IPv4 network: {cidr!r}")
    if robot_network.prefixlen < 24:
        raise ValueError("ROBOT_NETWORK_CIDR must be /24 or narrower to limit DDS exposure")
    if not robot_network.subnet_of(interface_address.network):
        raise ValueError(f"{interface} address {interface_address} is not on configured robot subnet {robot_network}")
    if interface_address.ip not in robot_network:
        raise ValueError(f"{interface} address {interface_address.ip} is outside selected robot subnet {robot_network}")
    robot_host = os.environ.get("S10_BASIC_SERVER_HOST") or config.get("basic_server", {}).get("host")
    try:
        robot_address = ipaddress.ip_address(robot_host)
    except (TypeError, ValueError) as exc:
        raise ValueError("S10 basic_server host must be an IPv4 address on ROBOT_NETWORK_CIDR") from exc
    if not isinstance(robot_address, ipaddress.IPv4Address):
        raise ValueError("S10 basic_server host must be an IPv4 address on ROBOT_NETWORK_CIDR")
    if robot_address not in robot_network or robot_address not in interface_address.network:
        raise ValueError(f"S10 basic_server {robot_address} is outside selected robot subnet {robot_network}")

    profile_path = Path(os.environ.get("S10_FASTDDS_PROFILE", "/tmp/lynx-s10-fastdds.xml"))
    profile_path.write_text(f'''<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com"><profiles>
  <transport_descriptors><transport_descriptor>
    <transport_id>s10_robot_and_loopback</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>127.0.0.1</address><address>{interface_address.ip}</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="s10_constrained" is_default_profile="true"><rtps>
    <userTransports><transport_id>s10_robot_and_loopback</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps></participant>
</profiles></dds>''', encoding="utf-8")
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile_path)
    core_profile = profile_path.with_name("lynx-s10-core-fastdds.xml")
    core_profile.write_text('''<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com"><profiles>
  <transport_descriptors><transport_descriptor>
    <transport_id>s10_core_loopback</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="s10_core_loopback" is_default_profile="true"><rtps>
    <userTransports><transport_id>s10_core_loopback</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps></participant>
</profiles></dds>''', encoding="utf-8")
    os.environ["S10_CORE_FASTDDS_PROFILE"] = str(core_profile)
    print(f"[ros2] robot-domain FastDDS restricted to loopback and validated robot subnet "
          f"{robot_network} via {interface} ({interface_address.ip})", flush=True)
    print("[ros2] Agent Core domain 42 will publish only from loopback-profiled bridge processes",
          flush=True)
    return profile_path

if __name__ == "__main__":
    import yaml

    config_path = Path(os.environ.get("CONFIG_PATH", Path(__file__).with_name("config.yaml")))
    with config_path.open(encoding="utf-8") as config_file:
        configure_fastdds_profile(yaml.safe_load(config_file) or {})
    run_driver(__file__, "deep-robotics-s10-pro-driver", "deep-robotics-s10-pro-device-bundle", build_plugins)
