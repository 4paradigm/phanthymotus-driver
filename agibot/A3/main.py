#!/usr/bin/env python3
"""AgiBot A3 (AimDK v3.2) MCP Driver 入口。"""

# Make every log line one atomic, control-character-free write, so concurrent
# writers cannot tear a Docker log record (README_dev new-driver logging
# contract). Must run before anything prints — _select_profile() below emits
# startup diagnostics long before run_driver() installs it.
try:
    from common import logsafe
    logsafe.install()
except ImportError:  # running outside the container image (dev checkout)
    import sys as _sys
    _sys.stderr.write("[bundle] logsafe unavailable; stdout unprotected\n")

import os
import socket

# Must live inside a directory the Dockerfile actually creates: /work/agibot/A3/
# is COPYied from this repo, /work/agibot-a3/ is not. On the robot a missing
# parent made the profile write fail, the OSError branch cleared the env var,
# and BOTH domains fell back to every interface — silently defeating the
# declared DDS isolation.
PROFILE_PATH = "/work/agibot/A3/dds-profile.xml"


def _robot_subnet_ip() -> str:
    """Find this host's address on the robot subnet (10.42.10.0/24, dev guide §7).

    The A3's three compute units (HDU/ADU/MDU) live at fixed 10.42.10.10-12, so
    the third-party compute unit running this driver is also on 10.42.10.x.
    UDP-connect trick: no packet leaves the machine, the OS just picks the
    route's source address for that destination.

    The selected source is validated: on a host with no specific route to the
    robot subnet, connect() picks the *default-route* address (e.g. an office
    LAN 192.168.x.x), and whitelisting that would silently expose domain 42 on
    the office LAN while domain 232 still cannot reach the A3 units — worse
    than setting no profile at all. Return "" unless the answer is a genuine
    10.42.10.x address.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("10.42.10.10", 1))
            ip = sock.getsockname()[0]
    except OSError:
        return ""
    return ip if ip.startswith("10.42.10.") else ""


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
        raise RuntimeError("A3 requires an address on the 10.42.10.x robot subnet; "
                           "refusing to start without a constrained DDS profile")
    profile = PROFILE_PATH
    content = f"""<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>a3_robot_net</transport_id>
        <type>UDPv4</type>
        <!-- A3 Image/PointCloud2 samples are fragmented at the RTPS layer.
             The defaults in the base image are small enough that a burst of
             fragments can be dropped before Fast-CDR reconstructs the sample,
             producing `sequence size exceeds remaining buffer`. -->
        <sendBufferSize>16777216</sendBufferSize>
        <receiveBufferSize>16777216</receiveBufferSize>
        <maxMessageSize>16777216</maxMessageSize>
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
        raise RuntimeError(f"cannot write constrained DDS profile {profile}: {exc}") from exc
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
    print(f"[ros2] robot DDS profile: {profile} "
          f"(robot-domain-232 whitelist {robot_ip}; domain-42 publishing is "
          "handled by the isolated bridge process)")


def main() -> None:
    # Must run before DualDomainROS2 constructs any participant (see
    # _select_profile: the profile is read at participant creation).
    _select_profile()

    from common.vendor_runtime import run_driver
    from device import build_plugins

    run_driver(__file__, "agibot-a3-driver", "agibot-a3-device-bundle", build_plugins)


if __name__ == "__main__":
    main()
