"""AS2W remote-controller card contract tests without ROS 2 or DDS."""

from pathlib import Path
import struct
import sys
import types
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


def load_device():
    stubs = {name: types.ModuleType(name) for name in (
        "std_msgs", "std_msgs.msg", "unitree_sdk2py", "unitree_sdk2py.core",
        "unitree_sdk2py.core.channel", "unitree_sdk2py.idl",
        "unitree_sdk2py.idl.unitree_go", "unitree_sdk2py.idl.unitree_go.msg",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    )}
    stubs["std_msgs.msg"].String = object
    stubs["unitree_sdk2py.core.channel"].ChannelSubscriber = object
    stubs["unitree_sdk2py.idl.unitree_go.msg.dds_"].LowState_ = object
    stubs["unitree_sdk2py.idl.unitree_go.msg.dds_"].SportModeState_ = object
    path = ROOT / "unitree/as2w/device.py"
    module = types.ModuleType("as2w_remote_controller_test")
    with mock.patch.dict(sys.modules, stubs):
        exec(compile(path.read_text(), str(path), "exec"), module.__dict__)
    return module


device = load_device()


def remote_bytes():
    return bytearray(40)


def test_parse_all_zero():
    result = device._parse_wireless_remote(remote_bytes())

    assert result["available"] is True
    assert result["fresh"] is True
    assert result["active"] is False
    assert len(result["buttons"]) == 14
    assert not any(result["buttons"].values())
    assert result["axes"] == {"lx": 0.0, "rx": 0.0, "ry": 0.0, "ly": 0.0}


def test_parse_matches_as2_sdk_button_layout():
    mapping = (
        (2, 5, "LT"), (2, 4, "RT"), (2, 3, "back"), (2, 2, "start"),
        (2, 1, "LB"), (2, 0, "RB"), (3, 7, "left"), (3, 6, "down"),
        (3, 5, "right"), (3, 4, "up"), (3, 3, "Y"), (3, 2, "X"),
        (3, 1, "B"), (3, 0, "A"),
    )

    for byte_index, bit, name in mapping:
        raw = remote_bytes()
        raw[byte_index] = 1 << bit
        result = device._parse_wireless_remote(raw)
        assert {key for key, pressed in result["buttons"].items() if pressed} == {name}
        assert result["active"] is True


def test_parse_axes_and_deadzone():
    raw = remote_bytes()
    raw[4:8] = struct.pack("f", 0.5)
    raw[8:12] = struct.pack("f", -0.25)

    result = device._parse_wireless_remote(raw)

    assert result["axes"]["lx"] == 0.5
    assert result["axes"]["rx"] == -0.25
    assert result["active"] is True

    raw[4:12] = struct.pack("f", 0.05) + struct.pack("f", 0.0)
    assert device._parse_wireless_remote(raw)["active"] is False


def test_parse_missing_payload_is_unavailable():
    assert device._parse_wireless_remote(None) == {"available": False, "fresh": False}
    assert device._parse_wireless_remote(bytearray(23)) == {
        "available": False, "fresh": False,
    }


def test_state_node_publishes_and_caches_remote_snapshot():
    class FakeString:
        pass

    node = object.__new__(device._StateNode)
    node._last_remote_time = 0.0
    node._last_remote = None
    node._remote_lock = device.threading.Lock()
    node.remote_controller = mock.Mock()
    node.imu = node.joints = node.joint_state = node.battery = mock.Mock()
    raw = remote_bytes()
    raw[2] = 1 << 3
    msg = types.SimpleNamespace(
        imu_state=None, motor_state=[], bms_state=None, wireless_remote=raw,
    )

    with mock.patch.object(device, "String", FakeString), \
         mock.patch.object(device.time, "monotonic", return_value=100.0), \
         mock.patch.object(device.time, "time", return_value=123.456):
        node._on_low(msg)

    with mock.patch.object(device.time, "monotonic", return_value=100.1):
        cached = node.last_remote
    assert cached["timestamp_ms"] == 123456
    assert cached["buttons"]["back"] is True
    assert cached["fresh"] is True
    published = node.remote_controller.publish.call_args.args[0]
    assert '"active":true' in published.data

    with mock.patch.object(device.time, "monotonic", return_value=100.51):
        assert node.last_remote["fresh"] is False


def state_plugin():
    plugin = object.__new__(device.StatePlugin)
    plugin._namespace = "testns"
    plugin._state = types.SimpleNamespace(last_remote=None)
    return plugin


def test_tool_contract_and_dispatch():
    plugin = state_plugin()
    tool = next(item for item in plugin.get_tools() if item["name"] == "remote_controller")

    assert tool["type"] == "sensor"
    assert tool["multiInstance"] is False
    assert tool["topic_out"] == [
        {"topic": "/testns/state/remote_controller", "format": "data/json"}
    ]
    assert plugin.dispatch("info", {"_tool_name": "remote_controller"}) == {
        "state": "running", "topic_out": tool["topic_out"],
    }
    assert plugin.dispatch("read", {"_tool_name": "remote_controller"}) == {
        "state": "running", "data": {"available": False},
    }
    plugin._state.last_remote = {"available": True, "active": True}
    assert plugin.dispatch("read", {"_tool_name": "remote_controller"})["data"] == {
        "available": True, "active": True,
    }


def test_driver_manifest_registers_remote_controller():
    manifest = (ROOT / "unitree/as2w/driver.yaml").read_text()
    assert "- { name: remote_controller, type: sensor }" in manifest
