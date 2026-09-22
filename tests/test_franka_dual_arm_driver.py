from concurrent.futures import Future
import copy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / "franka/dual_arm" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


device = load("franka_device_test", "device.py")
hardware = load("franka_hardware_test", "hardware.py")


def done(value):
    future = Future()
    future.set_result(value)
    return future


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancels = 0
        self.cancel_response = SimpleNamespace(goals_canceling=[1])

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        return done(self.cancel_response)


class Arm:
    def __init__(self):
        self.submission = Future()
        self.commands = []
        self.feedback = {"position": [0.] * 7, "velocity": [0.] * 7}
        self.closed = 0

    def ready(self, kind):
        return True

    def send(self, kind, command):
        self.commands.append((kind, copy.deepcopy(command)))
        return self.submission

    def state(self):
        return copy.deepcopy(self.feedback)

    def close(self):
        self.closed += 1


def motion():
    hw = Arm()
    result = device.Motion(hw, "arm", "franka_left_arm", lambda *args: None)
    return result, hw


def terminal(handle, status=4, error_code=0, reached_goal=True):
    handle.result.set_result(SimpleNamespace(status=status, result=SimpleNamespace(
        error_code=error_code, reached_goal=reached_goal)))


def test_stop_before_acceptance_cancels_late_goal():
    slot, hw = motion()
    result = slot.start({}, 30)
    slot.stop()
    assert slot.info()["action_id"] == result["action_id"]
    handle = Handle()
    hw.submission.set_result(handle)
    assert handle.cancels == 1
    assert slot.info()["state"] == "cancel_requested"
    with pytest.raises(RuntimeError, match="busy"):
        slot.start({}, 1)
    terminal(handle, status=5)
    assert slot.info()["action_id"] is None
    assert slot.info()["last_completion"]["status"] == "cancelled"


def test_cancel_rejection_does_not_release_slot():
    slot, hw = motion()
    slot.start({}, 30)
    handle = Handle()
    handle.cancel_response = SimpleNamespace(goals_canceling=[])
    hw.submission.set_result(handle)
    slot.stop()
    assert slot.info()["state"] == "cancel_unconfirmed"
    assert slot.info()["action_id"]
    terminal(handle, status=6)


def test_timeout_waits_for_native_terminal_result():
    slot, hw = motion()
    slot.start({}, 30)
    record = slot.active
    handle = Handle()
    hw.submission.set_result(handle)
    slot._timeout(record)
    assert handle.cancels == 1 and slot.active is record
    terminal(handle, status=5)
    assert slot.info()["last_completion"]["status"] == "failed"


@pytest.mark.parametrize("status,error,expected", [(4, 0, "completed"), (4, -1, "failed"), (6, 0, "failed")])
def test_completion_uses_controller_result(status, error, expected):
    slot, hw = motion()
    slot.start({}, 30)
    handle = Handle()
    hw.submission.set_result(handle)
    assert slot.info()["last_completion"] is None
    terminal(handle, status, error)
    assert slot.info()["last_completion"]["status"] == expected


def test_submission_exception_keeps_motion_locked():
    slot, hw = motion()
    def broken(*args):
        raise ConnectionError("lost after write")
    hw.send = broken
    with pytest.raises(RuntimeError, match="outcome unknown"):
        slot.start({}, 30)
    with pytest.raises(RuntimeError, match="busy"):
        slot.start({}, 30)


def config():
    arms = {}
    for side in ("left", "right"):
        arms[side] = {"joints": [f"joint{i}" for i in range(7)],
                      "trajectory_action": f"/{side}/controller/follow_joint_trajectory",
                      "joint_state_topic": f"/{side}/joint_states", "lower": [-2.] * 7,
                      "upper": [2.] * 7, "max_velocity": [.5] * 7,
                      "max_acceleration": [1.] * 7,
                      "gripper_action": f"/{side}/franka_gripper/gripper_action",
                      "gripper_max_width": .08, "gripper_max_force": 40.}
    return {"motion_enabled": True, "arms": arms}


@pytest.fixture
def plugin():
    hws = {side: Arm() for side in ("left", "right")}
    p = device.FrankaPlugin(config(), hws, lambda *args: None)
    yield p, hws
    for slot in p.motions.values():
        if slot.active and slot.active.get("timer"):
            slot.active["timer"].cancel()


def test_left_and_right_commands_are_isolated(plugin):
    p, hw = plugin
    p.dispatch("move", {"_tool_name": "franka_left_arm", "joint": [.1] * 7, "duration": 2.})
    assert len(hw["left"].commands) == 1
    assert not hw["right"].commands
    assert not p.motions["franka_right_arm"].active


@pytest.mark.parametrize("target,duration", [([3.] * 7, 10), ([1.] * 7, .5), ([float("nan")] * 7, 10), ([True] * 7, 10), ([0.] * 6, 10)])
def test_invalid_target_or_quintic_rate_limit_rejected(plugin, target, duration):
    p, hw = plugin
    with pytest.raises(ValueError):
        p.dispatch("move", {"_tool_name": "franka_left_arm", "joint": target, "duration": duration})
    assert not hw["left"].commands


def test_feedback_must_be_stationary(plugin):
    p, hw = plugin
    hw["left"].feedback["velocity"][0] = .1
    with pytest.raises(RuntimeError, match="stationary"):
        p.dispatch("move", {"_tool_name": "franka_left_arm", "joint": [.1] * 7, "duration": 2.})
    assert not hw["left"].commands


def test_duplicate_endpoints_rejected():
    cfg = config()
    cfg["arms"]["right"]["trajectory_action"] = cfg["arms"]["left"]["trajectory_action"]
    with pytest.raises(ValueError, match="distinct"):
        device.validate_config(cfg)


def test_disabled_motion_does_not_send(plugin):
    p, hw = plugin
    p.cfg["motion_enabled"] = False
    with pytest.raises(RuntimeError, match="disabled"):
        p.dispatch("move", {"_tool_name": "franka_left_arm", "joint": [0.] * 7, "duration": 2.})
    assert not hw["left"].commands


def test_gripper_width_converted_to_single_finger_position():
    class Goal:
        def __init__(self):
            self.command = SimpleNamespace(position=None, max_effort=None)
    sent = []
    hw = object.__new__(hardware.RosArm)
    hw.gripper = SimpleNamespace(send_goal_async=lambda goal: sent.append(goal))
    with patch.dict(sys.modules, {"control_msgs.action": SimpleNamespace(GripperCommand=SimpleNamespace(Goal=Goal))}):
        hw.send("gripper", {"width": .06, "force": 12.})
    assert sent[0].command.position == .03
    assert sent[0].command.max_effort == 12.


def test_measured_state_reorders_joint_names_and_ignores_partial_packet():
    import threading
    hw = object.__new__(hardware.RosArm)
    hw.joints = [f"j{i}" for i in range(7)]
    hw._lock, hw._sample, hw._received = threading.Lock(), None, 0.
    hw._state(SimpleNamespace(name=list(reversed(hw.joints)), position=list(range(7)), velocity=[0.] * 7))
    assert hw.state()["position"] == list(reversed(range(7)))
    hw._state(SimpleNamespace(name=["j0"], position=[99], velocity=[0]))
    assert hw.state()["position"] == list(reversed(range(7)))
    hw._received = 0.
    with pytest.raises(RuntimeError, match="fresh"):
        hw.state()


def test_full_mcp_list_and_dispatch(plugin):
    from common.vendor_runtime import DriverBundle
    p, _ = plugin
    bundle = DriverBundle([p])
    definitions = bundle.get_all_tools()
    assert len(definitions) == 6
    for definition in definitions:
        if definition["type"] == "actuator":
            assert definition["inputSchema"]["x-hooks"]["on_interrupt_motion"]["action"] == "stop"
    assert "position" in bundle.dispatch("franka_right_state", {"action": "get"})


def test_ros_trajectory_contains_quintic_boundary_conditions():
    class Goal:
        def __init__(self):
            self.trajectory = SimpleNamespace(joint_names=[], points=[])
    class Point:
        pass
    sent = []
    hw = object.__new__(hardware.RosArm)
    hw.joints = [f"j{i}" for i in range(7)]
    hw.arm = SimpleNamespace(send_goal_async=lambda goal: sent.append(goal))
    modules = {"control_msgs.action": SimpleNamespace(FollowJointTrajectory=SimpleNamespace(Goal=Goal)),
               "trajectory_msgs.msg": SimpleNamespace(JointTrajectoryPoint=Point),
               "builtin_interfaces.msg": SimpleNamespace(Duration=SimpleNamespace)}
    with patch.dict(sys.modules, modules):
        hw.send("arm", {"start": [0.] * 7, "target": [.1] * 7, "duration": 2.25})
    start, end = sent[0].trajectory.points
    assert start.time_from_start.sec == 0
    assert end.time_from_start.sec == 2 and end.time_from_start.nanosec == 250000000
    assert end.positions == [.1] * 7
    assert start.velocities == end.velocities == start.accelerations == end.accelerations == [0.] * 7


def test_mcp_http_initialize_list_read_and_invalid_motion(plugin):
    from http.server import ThreadingHTTPServer
    import json
    import threading
    import urllib.request
    from common.vendor_runtime import DriverBundle, make_handler
    p, hw = plugin
    bundle = DriverBundle([p])
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(lambda: bundle, "franka", "franka-test"))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    def call(method, params):
        request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp",
            data=json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.loads(response.read())
    try:
        assert call("initialize", {})["result"]["serverInfo"]["name"] == "franka"
        assert len(call("tools/list", {})["result"]["tools"]) == 6
        result = call("tools/call", {"name": "franka_left_state", "arguments": {"action": "get"}})
        assert json.loads(result["result"]["content"][0]["text"])["position"] == [0.] * 7
        result = call("tools/call", {"name": "franka_left_arm", "arguments": {"action": "move", "joint": [99.] * 7, "duration": 2}})
        assert result["error"]["code"] == -32602
        assert not hw["left"].commands
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=1)
