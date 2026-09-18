"""Contract tests for the U1 Pro tool inventory without ROS installed."""

from __future__ import annotations

import sys
import types
import unittest


class FakeSrv:
    class Request:
        pass


def _install_stubs():
    rclpy = types.ModuleType("rclpy")
    node = types.ModuleType("rclpy.node")

    class Node:
        def __init__(self, *args, **kwargs):
            pass

    node.Node = Node
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable")
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL="transient_local")
    rclpy.node, rclpy.qos = node, qos
    sys.modules.update({"rclpy": rclpy, "rclpy.node": node, "rclpy.qos": qos})

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {"__init__": lambda self: setattr(self, "data", "")})
    std_msgs.msg = std_msgs_msg
    sys.modules.update({"std_msgs": std_msgs, "std_msgs.msg": std_msgs_msg})

    robo = types.ModuleType("robo_sdk")
    robo_srv = types.ModuleType("robo_sdk.srv")
    robo_srv.StringCall = FakeSrv
    robo.srv = robo_srv
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = FakeSrv
    std_srvs.srv = std_srvs_srv
    sys.modules.update({"robo_sdk": robo, "robo_sdk.srv": robo_srv, "std_srvs": std_srvs, "std_srvs.srv": std_srvs_srv})


_install_stubs()

from device import ACTIONS, EVENT_TOPICS, SERVICE_TYPES  # noqa: E402


class U1ContractTests(unittest.TestCase):
    def test_documented_services_are_mapped(self):
        self.assertIn("/robo/auth/call/authorize", SERVICE_TYPES)
        self.assertIn("/robo/video/call/stream_state", SERVICE_TYPES)
        self.assertEqual(SERVICE_TYPES["/robo/auth/call/auth_state"], "trigger")

    def test_documented_events_are_mapped(self):
        self.assertEqual(EVENT_TOPICS["ready_state"], "/robo/system/subscribe/ready_state")
        self.assertEqual(EVENT_TOPICS["video_metadata"], "/robo/video/subscribe/metadata")

    def test_all_actions_have_vendor_services(self):
        for actions in ACTIONS.values():
            for service, _ in actions.values():
                self.assertIn(service, SERVICE_TYPES)


if __name__ == "__main__":
    unittest.main()
