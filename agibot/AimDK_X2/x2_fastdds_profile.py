#!/usr/bin/env python3
"""Build the robot-facing Fast DDS profile from a Linux interface name."""

from __future__ import annotations

import argparse
import fcntl
import ipaddress
import socket
import struct
from pathlib import Path


SIOCGIFADDR = 0x8915


def interface_ipv4(interface: str) -> str:
    """Return the primary IPv4 address assigned to ``interface``."""
    encoded = interface.encode("utf-8")
    if not encoded or len(encoded) > 15:
        raise ValueError(f"invalid network interface name: {interface!r}")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        result = fcntl.ioctl(sock.fileno(), SIOCGIFADDR, struct.pack("256s", encoded))
    return socket.inet_ntoa(result[20:24])


def render_profile(address: str) -> str:
    address = str(ipaddress.IPv4Address(address))
    addresses = [address]
    if address != "127.0.0.1":
        addresses.append("127.0.0.1")
    whitelist = "\n".join(f"          <address>{item}</address>" for item in addresses)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>x2_robot_udp</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
{whitelist}
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="x2_robot_profile" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>x2_robot_udp</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    parser.add_argument("--address")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    address = args.address or interface_ipv4(args.interface)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(render_profile(address), encoding="utf-8")
    print(
        f"[x2-dds-profile] interface={args.interface} address={address} profile={output}",
        flush=True,
    )


if __name__ == "__main__":
    main()
