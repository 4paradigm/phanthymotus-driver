#!/usr/bin/env python3
"""AgiBot A3 (AimDK v3.2) MCP Driver 入口。"""

import os
import socket


def _robot_subnet_ip() -> str:
    """Find this host's address on the robot subnet (10.42.10.0/24, dev guide §7).

    The A3's three compute units (HDU/ADU/MDU) live at fixed 10.42.10.10-12, so
    the third-party compute unit running this driver is also on 10.42.10.x.
    UDP-connect trick: no packet leaves the machine, the OS just picks the
    route's source address for that destination.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("10.42.10.10", 1))
            return sock.getsockname()[0]
    except OSError:
        return ""


def _select_profile() -> None:
    """Pick this process's single FastDDS profile, before any participant exists.

    Same constraint tianyi2.0 documented the hard way (its main.py
    DualDomainROS2._select_profile): FastDDS reads
    FASTRTPS_DEFAULT_PROFILES_FILE at *participant* creation and caches profiles
    process-wide, so a dual-context driver cannot give the robot context and the
    Agent Core context different profiles — one profile is all the process gets.

    The fleet's loopback-only /opt/phanthy-motus/dds-local.xml would bind every
    participant (including the robot-domain-232 one) to 127.0.0.1 only, and the
    A3's body link is NOT loopback: the HDU/ADU/MDU at 10.42.10.10-12 are
    separate machines reached over eth0. With the fleet profile in force the
    driver would still register over HTTP and look healthy on the dashboard
    while every mirrored topic stayed empty — the exact silent failure tianyi
    measured (visible topics 77 → 33, TTS "succeeding" with no sound).

    So this driver ships its own profile: UDPv4 whitelisted to
    {robot-subnet-IP, 127.0.0.1}. That keeps the body link up (domain 232 via
    eth0) and the Agent Core link up (domain 42 via loopback), while excluding
    every other subnet this host may roam onto. If the robot-subnet IP cannot be
    determined (non-robot host, e.g. a dev machine), no profile is set and
    FastDDS uses its default transports — fine for development, never deployed.
    """
    robot_ip = _robot_subnet_ip()
    if not robot_ip:
        os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
        print("[ros2] no address on the 10.42.10.x robot subnet found — "
              "no DDS profile selected (dev host; on the robot this would isolate "
              "domain 42 to loopback + robot subnet)")
        return
    profile = "/work/agibot-a3/dds-profile.xml"
    content = f"""<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>a3_robot_net</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>{robot_ip}</address>
          <address>127.0.0.1</address>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="a3_process_default" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>a3_robot_net</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
"""
    try:
        with open(profile, "w", encoding="utf-8") as handle:
            handle.write(content)
    except OSError as exc:
        os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
        print(f"[ros2] WARNING cannot write {profile} ({exc}) — no DDS profile. "
              "Both domains will use every interface: domain 42 is NOT isolated.")
        return
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
    print(f"[ros2] process-wide DDS profile: {profile} "
          f"(whitelist {robot_ip} + 127.0.0.1 — robot-domain-232 link to "
          f"10.42.10.x up, other subnets excluded on both contexts)")


def main() -> None:
    # Must run before DualDomainROS2 constructs any participant (see
    # _select_profile: the profile is read at participant creation).
    _select_profile()

    from common.vendor_runtime import run_driver
    from device import build_plugins

    run_driver(__file__, "agibot-a3-driver", "agibot-a3-device-bundle", build_plugins)


if __name__ == "__main__":
    main()
