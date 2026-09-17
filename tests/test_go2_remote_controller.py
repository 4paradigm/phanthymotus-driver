"""Go2 remote-controller card contract tests without a ROS 2/DDS runtime."""

from pathlib import Path
import struct
import sys
import types
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


def load_device():
    stubs = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg",
        "audio_msgs", "audio_msgs.msg",
    )}
    stubs["rclpy.node"].Node = object
    stubs["rclpy.qos"].QoSProfile = lambda **kwargs: kwargs
    stubs["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    stubs["rclpy.qos"].HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    stubs["rclpy.qos"].DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    stubs["std_msgs.msg"].String = object
    stubs["audio_msgs.msg"].AudioChunk = object
    path = ROOT / "unitree/go2/device.py"
    module = types.ModuleType("go2_remote_controller_test")
    with mock.patch.dict(sys.modules, stubs):
        exec(compile("from __future__ import annotations\n" + path.read_text(), str(path), "exec"),
             module.__dict__)
    return module


device = load_device()


def remote_bytes():
    return bytearray(40)


def test_parse_all_zero():
    result = device._parse_wireless_remote(remote_bytes())

    assert result["available"] is True
    assert result["fresh"] is True
    assert result["control_level"] == "LOWLEVEL"
    assert result["active"] is False
    assert len(result["buttons"]) == 13
    assert "back" not in result["buttons"]
    assert not any(result["buttons"].values())
    assert result["axes"] == {"lx": 0.0, "rx": 0.0, "ry": 0.0, "ly": 0.0}


def test_parse_button_a_uses_sdk_wire_bit_layout():
    raw = remote_bytes()
    raw[3] = 0b00000001  # A is the least-significant bit in byte 3.

    result = device._parse_wireless_remote(raw)

    assert result["buttons"]["A"] is True
    assert sum(result["buttons"].values()) == 1
    assert result["active"] is True


def test_parse_full_sdk_button_mapping():
    mapping = (
        (2, 5, "LT"), (2, 4, "RT"), (2, 2, "start"),
        (2, 1, "LB"), (2, 0, "RB"),
        (3, 7, "left"), (3, 6, "down"), (3, 5, "right"), (3, 4, "up"),
        (3, 3, "Y"), (3, 2, "X"), (3, 1, "B"), (3, 0, "A"),
    )

    for byte_index, bit, name in mapping:
        raw = remote_bytes()
        raw[byte_index] = 1 << bit
        result = device._parse_wireless_remote(raw)
        assert {key for key, pressed in result["buttons"].items() if pressed} == {name}
        assert result["active"] is True


def test_parse_ignores_reserved_back_bit():
    raw = remote_bytes()
    raw[2] = 0b00001000

    result = device._parse_wireless_remote(raw)

    assert "back" not in result["buttons"]
    assert result["active"] is False


def test_parse_axis_lx_and_deadzone():
    raw = remote_bytes()
    raw[4:8] = struct.pack("f", 0.5)
    assert device._parse_wireless_remote(raw)["axes"]["lx"] == 0.5
    assert device._parse_wireless_remote(raw)["active"] is True

    raw[4:8] = struct.pack("f", 0.05)
    assert device._parse_wireless_remote(raw)["active"] is False


def test_parse_none_is_unavailable():
    assert device._parse_wireless_remote(None) == {
        "available": False,
        "fresh": False,
        "control_level": "LOWLEVEL",
    }


def test_lowstate_publishes_and_caches_remote_snapshot():
    class FakeString:
        pass

    node = object.__new__(device._LowStateNode)
    node._last_imu_time = node._last_joints_time = node._last_bms_time = 100.0
    node._last_remote_time = 0.0
    node._last_remote = None
    node._remote_lock = device.threading.Lock()
    node._remote_pub = mock.Mock()
    raw = remote_bytes()
    raw[3] = 1

    with mock.patch.object(device, "String", FakeString), \
         mock.patch.object(device.time, "monotonic", return_value=100.0), \
         mock.patch.object(device.time, "time", return_value=123.456):
        node._on_state(types.SimpleNamespace(wireless_remote=raw))

    with mock.patch.object(device.time, "monotonic", return_value=100.1):
        cached = node.last_remote
    assert cached["timestamp_ms"] == 123456
    assert cached["buttons"]["A"] is True
    assert cached["fresh"] is True
    assert cached["control_level"] == "LOWLEVEL"
    published = node._remote_pub.publish.call_args.args[0]
    assert '"active": true' in published.data
    with mock.patch.object(device.time, "monotonic", return_value=100.51):
        assert node.last_remote["fresh"] is False


def state_plugin():
    plugin = object.__new__(device.StatePlugin)
    plugin._imu_topic = "/testns/state/imu"
    plugin._battery_topic = "/testns/state/battery"
    plugin._joints_topic = "/testns/state/joints"
    plugin._remote_topic = "/testns/state/remote_controller"
    plugin._node = types.SimpleNamespace(last_remote=None)
    return plugin


def test_tool_contract():
    tool = state_plugin()._remote_tool()

    assert tool["name"] == "remote_controller"
    assert tool["type"] == "sensor"
    assert tool["multiInstance"] is False
    assert tool["inputSchema"]["properties"] == {}
    assert tool["topic_out"] == [{"topic": "/testns/state/remote_controller", "format": "data/json"}]


def test_dispatch_info_and_read():
    plugin = state_plugin()

    assert plugin.dispatch("info", {"_tool_name": "remote_controller"}) == {
        "state": "running",
        "topic_out": [{"topic": "/testns/state/remote_controller", "format": "data/json"}],
    }
    assert plugin.dispatch("read", {"_tool_name": "remote_controller"}) == {
        "state": "running", "data": {"available": False},
    }
    plugin._node.last_remote = {"available": True, "active": True}
    assert plugin.dispatch("read", {"_tool_name": "remote_controller"})["data"] == {
        "available": True, "active": True,
    }
    assert plugin.dispatch("read", {"_tool_name": "imu"}) is None


def test_driver_yaml_cards_synced():
    # Keep this dependency-free: the manifest uses one inline mapping per card.
    manifest = (ROOT / "unitree/go2/driver.yaml").read_text()
    assert "- { name: remote_controller, type: sensor }" in manifest
