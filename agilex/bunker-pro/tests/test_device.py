import json
import sys
import threading
import types
from unittest.mock import patch

import pytest

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))

from device import BunkerProDevice, _validate_move_duration


class FakeCAN:
    def __init__(self):
        self.targets = []
        self.stops = 0

    def validate_target(self, linear, angular):
        return float(linear), float(angular)

    def set_target(self, linear, angular, lease_s):
        self.targets.append((linear, angular, lease_s))

    def snapshot(self):
        return {"last_tx_error": ""}

    def stop_motion(self):
        self.stops += 1
        return {"state": "stopped"}


def _bare_device():
    device = BunkerProDevice.__new__(BunkerProDevice)
    device._motion_enabled = True
    device._motion_lock = threading.Lock()
    device._active_move_id = None
    device._active_move_stop = None
    device._direct_until = 0.0
    device._rejection_log_lock = threading.Lock()
    device._rejection_counts = {}
    device._last_rejection_log_at = 0.0
    device.can = FakeCAN()
    return device


@pytest.mark.parametrize("value", [True, "1", float("nan"), float("inf"), 0, 10.1])
def test_invalid_move_duration_is_rejected_before_motion_state_changes(value):
    device = _bare_device()
    with pytest.raises(ValueError):
        device.dispatch("move", {"duration_s": value, "vx": 0.1, "wz": 0})
    assert device.can.stops == 0
    assert device._active_move_id is None
    assert device._direct_until == 0.0


def test_move_duration_accepts_finite_values():
    assert _validate_move_duration(0.05) == 0.05
    assert _validate_move_duration(10) == 10.0


def test_cancel_before_worker_starts_never_sets_motion_target():
    device = _bare_device()
    event = threading.Event()
    event.set()
    with patch("device._acp_notify") as notify:
        device._run_move("action-1", event, 0.1, 0.0, 1.0)
    assert device.can.targets == []
    assert device.can.stops == 0
    notify.assert_called_once_with("action-1", "cancelled", {})


def test_move_worker_sets_target_and_reports_completion():
    device = _bare_device()
    with patch("device._acp_notify") as notify:
        device._run_move("action-2", threading.Event(), 0.1, -0.1, 0.05)
    assert device.can.targets == [(0.1, -0.1, 0.05)]
    assert device.can.stops == 1
    notify.assert_called_once_with("action-2", "completed", {"state": "stopped"})


def test_stream_rejection_logs_are_throttled(capsys):
    device = _bare_device()
    device._motion_enabled = False
    message = types.SimpleNamespace(data=json.dumps({"values": [0] * 6}))
    for _ in range(20):
        device._on_command(message)
    assert capsys.readouterr().out.count("rejected streamed control messages") == 1
    assert device._rejection_counts == {"motion_disabled": 19}


def test_stream_acceptance_is_submitted_and_applied():
    device = _bare_device()
    calls = []
    device._motion_enabled = True
    device._direct_until = 0.0
    device._sink = types.SimpleNamespace(
        submit=lambda payload: (calls.append(payload) or types.SimpleNamespace(
            applied=True, verdict=types.SimpleNamespace(value="accepted"))),
        tick=lambda: calls.append("tick"),
    )
    payload = {"values": [0.1, 0, 0, 0, 0, 0.2]}
    device._on_command(types.SimpleNamespace(data=json.dumps(payload)))
    assert calls == [payload, "tick"]


def test_move_advertises_async_completion_contract():
    device = _bare_device()
    device.bridge = types.SimpleNamespace(_state_topic="/bunker/state/base",
                                          _odom_topic="/bunker/state/odom")
    device.config = {}
    schema = next(item["inputSchema"] for item in device.get_tools()
                  if item["name"] == "bunker_base")
    assert schema["x-completion"] == {"actions": ["move"], "timeout": 20}


def test_dds_bridge_binds_forwards_and_closes_entities():
    events = []

    class String:
        data = ""

    class Node:
        def __init__(self, name, context=None):
            self.name, self.context = name, context
            self.subscriptions = []
        def create_publisher(self, _type, topic, _depth):
            return types.SimpleNamespace(topic=topic, messages=[], publish=lambda msg: events.append((topic, msg.data)))
        def create_subscription(self, _type, topic, callback, _depth):
            handle = types.SimpleNamespace(topic=topic, callback=callback)
            self.subscriptions.append(handle)
            return handle
        def destroy_subscription(self, handle):
            events.append(("destroy_subscription", handle.topic))
        def destroy_node(self):
            events.append(("destroy_node", self.name))

    class Executor:
        def add_node(self, node): events.append(("add_node", node.name))
        def remove_node(self, node): events.append(("remove_node", node.name))

    node_module = types.ModuleType("rclpy.node")
    node_module.Node = Node
    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.String = String
    with patch.dict(sys.modules, {"rclpy": types.ModuleType("rclpy"),
                                  "rclpy.node": node_module,
                                  "std_msgs": types.ModuleType("std_msgs"),
                                  "std_msgs.msg": std_msgs}):
        from dds_bridge import BunkerDDSBridge
        ros = types.SimpleNamespace(ctx_robot=object(), ctx_core=object(),
                                    executor_robot=Executor(), executor_core=Executor())
        bridge = BunkerDDSBridge(ros, "bunker")
        bridge.bind_control_topic("/agent/control")
        bridge._on_command(types.SimpleNamespace(data='{"values":[]}'))
        bridge._on_state(types.SimpleNamespace(data='{"ok":true}'))
        bridge.close()
    assert ("/bunker/control/velocity", '{"values":[]}') in events
    assert ("/bunker/state/base", '{"ok":true}') in events
    assert ("destroy_node", "bunker_pro_core_bridge") in events
    assert ("destroy_node", "bunker_pro_robot_bridge") in events
