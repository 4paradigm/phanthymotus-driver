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
    # joint_control and gripper use this same gate, so neither can acquire it
    # while the stream owns the arm for its active lifetime.
    assert client.motion_gate.acquire(blocking=False) is False
    blocked = second.dispatch("start", {"input_topic": "/control/b"})
    assert blocked["state"] == "error"
    assert "another arm operation" in blocked["message"]

    first.dispatch("stop", {})
    assert second.dispatch("start", {"input_topic": "/control/b"})["state"] == "running"
    second.dispatch("stop", {})
