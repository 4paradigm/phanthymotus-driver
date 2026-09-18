import sys
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "realman" / "rm75_6f_v"
sys.path.insert(0, str(DRIVER))
sys.path.insert(0, str(ROOT))

from servo import RM75ServoPlugin


class FakeClient:
    motion_enabled = True
    connected = True

    def __init__(self):
        import threading
        self.motion_gate = threading.Lock()

    def command(self, method, *args):
        return 0


def test_servo_holds_motion_gate_for_active_stream():
    client = FakeClient()
    ros2 = mock.Mock()
    first = RM75ServoPlugin(client, {}, ros2=ros2)
    second = RM75ServoPlugin(client, {}, ros2=ros2)
    first._subscribe = mock.Mock()
    second._subscribe = mock.Mock()

    assert first.dispatch("start", {"input_topic": "/control/a"})["state"] == "running"
    duplicate = first.dispatch("start", {"input_topic": "/control/duplicate"})
    assert duplicate["state"] == "error"
    assert "already running" in duplicate["message"]
    # joint_control and gripper use this same gate, so neither can acquire it
    # while the stream owns the arm for its active lifetime.
    assert client.motion_gate.acquire(blocking=False) is False
    blocked = second.dispatch("start", {"input_topic": "/control/b"})
    assert blocked["state"] == "error"
    assert "another arm operation" in blocked["message"]

    first.dispatch("stop", {})
    assert client.motion_gate.acquire(blocking=False) is True
    client.motion_gate.release()
    assert second.dispatch("start", {"input_topic": "/control/b"})["state"] == "running"
    second.dispatch("stop", {})


def test_servo_stop_releases_gate_when_node_teardown_fails():
    client = FakeClient()
    ros2 = mock.Mock()
    plugin = RM75ServoPlugin(client, {}, ros2=ros2)
    plugin._subscribe = mock.Mock()
    assert plugin.dispatch("start", {"input_topic": "/control/a"})["state"] == "running"

    node = mock.Mock()
    node.destroy_node.side_effect = RuntimeError("teardown failed")
    plugin._node = node
    try:
        plugin.dispatch("stop", {})
    except RuntimeError as exc:
        assert str(exc) == "teardown failed"
    else:
        raise AssertionError("expected teardown failure")

    assert client.motion_gate.acquire(blocking=False) is True
    client.motion_gate.release()
