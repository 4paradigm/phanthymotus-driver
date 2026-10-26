"""Private JSON-line worker; only the parent exposes a card API."""
from common import logsafe
logsafe.install()

import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import sys
import rclpy
from rcl_interfaces.srv import GetParameters
from uworld_motion_msgs.srv import SoundSource


def reply(value):
    print("U1_HEAD_REPLY " + json.dumps(value, allow_nan=False), flush=True)


def wait(node, client, request, timeout):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        client.remove_pending_request(future)
        raise TimeoutError("vendor service response timed out")
    return future.result()


def validate_runtime():
    from rclpy.utilities import get_rmw_implementation_identifier
    profile = Path(__file__).resolve().parent / "head-dds-local.xml"
    if get_rmw_implementation_identifier() != "rmw_fastrtps_cpp" or os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE") != str(profile):
        raise RuntimeError("head worker requires its isolated FastDDS profile")
    root = ET.parse(profile).getroot()
    ns = {"d": "http://www.eprosima.com"}
    addresses = [element.text for element in root.findall(".//d:interfaceWhiteList/d:address", ns)]
    if addresses != ["127.0.0.1"] or root.findtext(".//d:useBuiltinTransports", namespaces=ns) != "false":
        raise RuntimeError("head worker profile must bind only loopback and disable built-in transports")


def main():
    validate_runtime()
    rclpy.init(domain_id=2)
    node = rclpy.create_node("u1_head_service_bridge")
    probe = node.create_client(GetParameters, "/mc/uworld_sound_follower_node/get_parameters")
    motion = node.create_client(SoundSource, "/mc/uworld_sound_source")
    verified = False
    try:
        for line in sys.stdin:
            try:
                if len(line) > 4096:
                    raise ValueError("request too large")
                command = json.loads(line)
                operation = command.get("operation")
                if operation == "probe":
                    verified = False
                    if not probe.wait_for_service(timeout_sec=2) or not motion.wait_for_service(timeout_sec=2):
                        raise RuntimeError("vendor head services unavailable")
                    request = GetParameters.Request()
                    request.names = ["use_sim_time"]
                    response = wait(node, probe, request, 3)
                    if len(response.values) != 1 or response.values[0].type != 1 or response.values[0].bool_value:
                        raise RuntimeError("vendor head clock state unconfirmed or simulated")
                    verified = True
                    reply({"ok": True, "ready": True, "transport": "FastDDS loopback domain 2"})
                elif operation == "turn":
                    if not verified:
                        raise RuntimeError("read-only round-trip probe required before motion")
                    verified = False
                    values = [command.get(key) for key in ("degree", "max_vel", "max_acc")]
                    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
                        raise ValueError("invalid numeric motion input")
                    degree, velocity, acceleration = values
                    if not -20 <= degree <= 20 or not 0 < velocity <= .4 or not 0 < acceleration <= .4:
                        raise ValueError("motion input outside provisional limits")
                    request = SoundSource.Request()
                    request.degree, request.with_waist = float(degree), False
                    request.revolute_setup.max_vel = float(velocity)
                    request.revolute_setup.max_acc = float(acceleration)
                    response = wait(node, motion, request, 60)
                    reply({"ok": True, "response": {"success": response.success,
                          "status": response.status, "message": response.message[:256]}})
                else:
                    raise ValueError("unknown operation")
            except Exception as exc:
                verified = False
                reply({"ok": False, "error": str(exc)[:256]})
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
