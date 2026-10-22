"""Canvas target binding against the real DDS setup, without ROS or robot I/O."""
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

import teleop_executor as module
from teleop_executor import TeleopExecutor


@pytest.mark.parametrize("namespace", ["tianyi", "beijing_robot_7"])
def test_target_descriptor_matches_local_bus_topics(monkeypatch, namespace):
    executor = TeleopExecutor({}, namespace, None, None, None, [])
    tool = executor.get_tool()
    descriptor = tool["x-teleop-target"]
    assert descriptor == {
        "protocol_version": 1,
        "robot_profile": "tianyi2",
        "namespace": namespace,
        "command_topic": tool["topic_in"][0]["topic"],
        "feedback_topic": tool["topic_out"][0]["topic"],
    }
    assert tool["topic_in"][0]["format"] == "control/teleop"
    assert tool["topic_out"][0]["format"] == "data/json"

    # Exercise the production bus's actual create_subscription/publisher calls.
    # The stand-ins create no sockets, threads, ROS participants or actuators.
    topics = {"subscriptions": [], "publishers": []}
    domains = []

    class Node:
        def __init__(self, name):
            pass

        def create_subscription(self, message, topic, callback, qos):
            topics["subscriptions"].append(topic)

        def create_publisher(self, message, topic, qos):
            topics["publishers"].append(topic)

        def destroy_node(self):
            pass

    rclpy = ModuleType("rclpy")
    rclpy.init = lambda **kwargs: domains.append(kwargs["domain_id"])
    rclpy.ok = lambda: False
    rclpy.try_shutdown = lambda: None
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules, "rclpy.executors", SimpleNamespace(
        ExternalShutdownException=type("ExternalShutdownException", (Exception,), {})))
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        QoSProfile=lambda **kwargs: kwargs,
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT="best_effort"),
        HistoryPolicy=SimpleNamespace(KEEP_LAST="keep_last"),
        DurabilityPolicy=SimpleNamespace(VOLATILE="volatile")))
    monkeypatch.setitem(sys.modules, "std_msgs", ModuleType("std_msgs"))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=object))
    # This test invokes the child body in pytest itself. Log installation in an
    # actual fresh process is covered by test_teleop_logsafe.py.
    monkeypatch.setitem(sys.modules, "common", SimpleNamespace(
        logsafe=SimpleNamespace(install=lambda **kwargs: None)))
    monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "/isolated/dds.xml")
    monkeypatch.setattr(Path, "read_bytes", lambda self: b"isolated-dds-profile")
    monkeypatch.setattr(module.socket, "socket", lambda **kwargs: SimpleNamespace(
        setblocking=lambda value: None, close=lambda: None))

    module.run_local_bus(123, namespace)
    assert domains == [42]
    assert topics == {
        "subscriptions": [descriptor["command_topic"]],
        "publishers": [descriptor["feedback_topic"]],
    }


def test_info_reports_the_same_target_without_starting_execution(monkeypatch):
    executor = TeleopExecutor({}, "showroom", None, None, None, [])
    # Sensor decoding imports the full robot bundle; metadata needs no samples.
    monkeypatch.setattr(executor.gate, "snapshot", lambda: {})
    monkeypatch.setattr(executor, "start", lambda: pytest.fail("metadata must not start execution"))
    tool = executor.get_tool()
    info = executor.dispatch("info", {})
    assert info["x-teleop-target"] == tool["x-teleop-target"]
    assert info["topic_in"] == tool["topic_in"]
    assert info["topic_out"] == tool["topic_out"]
    assert not info["ownership_held"] and not info["publisher_present"]
    assert executor.node is None and executor._bus_process is None
    assert executor._thread is None and not executor._operator_prepared
