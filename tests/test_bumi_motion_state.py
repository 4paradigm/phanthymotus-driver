"""Read real motion telemetry without ROS, SDK initialization or commands."""

from __future__ import annotations

import importlib.util
import sys
import threading
import types
from pathlib import Path

import pytest


BUMI = Path(__file__).resolve().parents[1] / "noetix" / "bumi"


@pytest.fixture
def motion_node(monkeypatch):
    # Load the whole module so a later duplicate mode map cannot silently
    # override the names tested here. No ROS node is constructed.
    imports = {
        "rclpy": {},
        "rclpy.node": {"Node": object},
        "rclpy.qos": {
            "QoSProfile": lambda **kwargs: None,
            "ReliabilityPolicy": types.SimpleNamespace(BEST_EFFORT=0),
            "HistoryPolicy": types.SimpleNamespace(KEEP_LAST=0),
            "DurabilityPolicy": types.SimpleNamespace(VOLATILE=0),
        },
        "std_msgs": {},
        "std_msgs.msg": {"String": object},
        "audio_msgs": {},
        "audio_msgs.msg": {"AudioChunk": object},
        "sensor_msgs": {},
        "sensor_msgs.msg": {"CompressedImage": object, "Image": object},
    }
    for name, attributes in imports.items():
        stub = types.ModuleType(name)
        stub.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, stub)

    spec = importlib.util.spec_from_file_location(
        "bumi_motion_state_for_test", BUMI / "device.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = object.__new__(module._MotionStateNode)
    node._high_ctrl = ReadOnlyController()
    node._activity_velocity_threshold = 0.15
    return node


class ReadOnlyController:
    mode = 2

    def get_mode(self):
        return self.mode

    def get_imu_data(self):
        return types.SimpleNamespace(
            ori=[0, 0, 0, 1], angular_vel=[0, 0, 0], linear_acc=[0, 0, 9.81])

    def get_joint_state(self):
        return [types.SimpleNamespace(
            motor_id=index, pos=0, vel=0, tau=0, temperature=40, error=0)
            for index in range(21)]

    def __getattr__(self, name):
        raise AssertionError(f"motion_state must not access SDK method {name}")


@pytest.mark.parametrize(("mode", "name", "protection"), [
    (2, "walking", False),
    (3, "whole_body_walking", False),
    (26, "protection", True),
    (777, "unknown", False),
])
def test_feedback_preserves_codes_and_distinguishes_walking_modes(
        motion_node, mode, name, protection):
    motion_node._high_ctrl.mode = mode
    state = motion_node._read_once()
    assert state["workmode"] == {
        "code": mode, "name": name, "protection": protection}
    assert state["fresh"] is True


def test_switch_feedback_does_not_claim_the_robot_is_moving(motion_node):
    for mode in (2, 3, 2, 3, 2):
        motion_node._high_ctrl.mode = mode
        state = motion_node._read_once()
        assert state["workmode"]["code"] == mode
        assert state["activity"] == "stationary"
        assert state["joint_motion"]["moving_joint_count"] == 0


@pytest.fixture
def real_loco(motion_node):
    # Exercise the real reservation and movement guards without constructing
    # a controller, executor, worker thread or ROS publisher.
    cls = motion_node._read_once.__globals__["LocoPlugin"]
    loco = object.__new__(cls)
    loco._rotate_lock = threading.Lock()
    loco._body_action = None
    loco._move_thread = None
    loco._rotate_state = {"state": "idle"}
    return loco


def test_mode_reservation_blocks_actual_move_and_body_action_dispatch(real_loco):
    with real_loco.reserve_walking_mode():
        assert real_loco.is_moving()
        assert real_loco._do_move({})["state"] == "error"
        assert real_loco.dispatch(
            "stand_up", {"_tool_name": "stand_up_lie_prone"})["command_sent"] is False
    assert real_loco.is_moving() is False


def test_reservation_releases_after_transport_failure(real_loco):
    with pytest.raises(OSError), real_loco.reserve_walking_mode():
        raise OSError("App connection lost")
    assert real_loco.is_moving() is False


def test_existing_body_action_or_rotation_prevents_mode_reservation(real_loco):
    real_loco._body_action = "semantic_action.wave"
    with pytest.raises(RuntimeError), real_loco.reserve_walking_mode():
        pytest.fail("must not reserve an active body")
    assert real_loco._body_action == "semantic_action.wave"
    real_loco._body_action = None
    real_loco._rotate_state["state"] = "running"
    with pytest.raises(RuntimeError), real_loco.reserve_walking_mode():
        pytest.fail("must not reserve an active rotation")
