import importlib.util
import json
from pathlib import Path
from queue import Queue
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


client = load("tron_client_test", ROOT / "limx/tron2_edu/client.py")
telemetry = load("tron_telemetry_test", ROOT / "limx/tron2_edu/telemetry.py")
with patch.dict(sys.modules, {"client": client, "telemetry": telemetry}):
    device = load("tron_device_test", ROOT / "limx/tron2_edu/device.py")


class Socket:
    def __init__(self):
        self.incoming = Queue()
        self.outgoing = Queue()

    def recv(self):
        return self.incoming.get(timeout=2)

    def send(self, raw):
        self.outgoing.put(json.loads(raw))

    def close(self):
        self.incoming.put("")


@pytest.fixture
def connected():
    sock = Socket()
    transport = client.TronClient("ws://localhost:5000", "SF_TRON2A_TEST", timeout=.2,
                                  socket_factory=lambda *a, **k: sock)
    transport.connect()
    yield transport, sock
    transport.close()


def test_matches_guid_title_and_identity(connected):
    transport, sock = connected
    outcome = []
    thread = threading.Thread(target=lambda: outcome.append(transport.request("request_get_joint_state")))
    thread.start()
    request = sock.outgoing.get(timeout=1)
    assert request["timestamp"] > 1_000_000_000_000
    for title, guid in [("response_other", request["guid"]), ("response_get_joint_state", "wrong")]:
        transport._receive({"accid": transport.accid, "title": title, "guid": guid,
                            "data": {"result": "success"}})
    assert not outcome
    transport._receive({**request, "title": "response_get_joint_state", "data": {"result": "success", "q": [1]}})
    thread.join(timeout=1)
    assert outcome == [{"result": "success", "q": [1]}]


def test_response_timeout_does_not_retry(connected):
    transport, sock = connected
    with pytest.raises(TimeoutError, match="no retry"):
        transport.request("request_get_joint_state")
    assert sock.outgoing.qsize() == 1
    assert not transport._pending


def test_disconnect_wakes_pending_call(connected):
    transport, sock = connected
    errors = []
    def request():
        try:
            transport.request("request_get_joint_state")
        except Exception as exc:
            errors.append(exc)
    thread = threading.Thread(target=request)
    thread.start()
    sock.outgoing.get(timeout=1)
    transport.close()
    thread.join(timeout=1)
    assert isinstance(errors[0], ConnectionError)


def test_wrong_robot_disconnects_and_invalidates_state(connected):
    transport, sock = connected
    transport._receive({"accid": transport.accid, "title": "notify_robot_info", "data": {"status": "WALK"}})
    sock.incoming.put(json.dumps({"accid": "OTHER", "title": "notify_robot_info", "data": {}}))
    transport._reader.join(timeout=1)
    assert not transport.connected
    with pytest.raises(RuntimeError, match="feedback"):
        transport.notification("notify_robot_info")


class Robot:
    connected = True
    accid = "SF_TRON2A_TEST"
    timeout = .2
    session_id = 1

    def __init__(self):
        self.commands = []
        self.fresh = True
        self.reject = False

    def connect(self):
        self.connected = True

    def close(self):
        self.connected = False

    def notification(self, title):
        if not self.fresh or (title == "notify_twist" and not self.reject):
            raise RuntimeError("stale")
        if title == "notify_twist":
            return {"result": "fail_motor"}
        return {"status": "WALK", "motor": "OK", "imu": "OK"}

    def send(self, title, data):
        if not self.connected:
            raise ConnectionError("offline")
        self.commands.append((title, dict(data)))


def plugin(profile="biped"):
    robot = Robot()
    if profile == "wheeled_biped":
        robot.accid = "WF_TRON2A_TEST"
    result = device.TronPlugin({"profile": profile, "motion_enabled": True}, robot)
    # Drive the same timer callback deterministically instead of spawning a loop.
    result._thread = SimpleNamespace(is_alive=lambda: True, join=lambda **k: None)
    return result, robot


@pytest.mark.parametrize("profile,arm", [("fixed_arms", True), ("mobile_arms", True), ("biped", False), ("wheeled_biped", False)])
def test_configuration_controls_capabilities(profile, arm):
    p, _ = plugin(profile)
    names = {t["name"] for t in p.get_tools()}
    assert ("tron2_joint_states" in names) == arm
    assert ("tron2_velocity" in names) != arm
    assert all("movej" not in str(t).lower() for t in p.get_tools())


def test_lease_expiration_and_stop_cannot_replay_old_target():
    p, robot = plugin()
    p.set_velocity({"x": .1, "lease": .1})
    assert robot.commands[-1][1]["x"] == .1
    p._expires = time.monotonic() - 1
    with p._lock:
        p._tick()
    assert robot.commands[-1][1] == {"x": 0, "y": 0, "z": 0}
    p.set_velocity({"x": .1})
    p.dispatch("stop", {"_tool_name": "tron2_velocity"})
    count = len(robot.commands)
    with p._lock:
        p._tick()
    assert len(robot.commands) == count
    assert robot.commands[-1][1]["x"] == 0


def test_stop_during_preflight_cannot_be_overtaken(monkeypatch):
    p, robot = plugin()
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
    original = device.number
    def number(value, name, low, high):
        if name == "lease":
            entered.set()
            assert release.wait(2)
        return original(value, name, low, high)
    monkeypatch.setattr(device, "number", number)
    mover = threading.Thread(target=lambda: p.set_velocity({"x": .1}))
    def stop():
        p.dispatch("stop", {"_tool_name": "tron2_velocity"})
        stopped.set()
    stopper = threading.Thread(target=stop)
    mover.start()
    assert entered.wait(1)
    stopper.start()
    try:
        assert not stopped.wait(.05)
    finally:
        release.set()
        mover.join(timeout=2)
        stopper.join(timeout=2)
    assert stopped.is_set() and p._target is None
    assert robot.commands[-1][1] == {"x": 0, "y": 0, "z": 0}


@pytest.mark.parametrize("cause", ["stale", "rejected", "disconnected"])
def test_stream_fault_clears_target_and_latches(cause):
    p, robot = plugin()
    p.set_velocity({"x": .1})
    if cause == "stale":
        robot.fresh = False
    elif cause == "rejected":
        robot.reject = True
    else:
        robot.connected = False
    with pytest.raises(Exception):
        with p._lock:
            p._tick()
    assert p._target is None and p._fault
    with pytest.raises(RuntimeError, match="faulted"):
        p.set_velocity({"x": .1})


def test_wrong_profile_and_wheel_lateral_rejected_before_send():
    p, robot = plugin("wheeled_biped")
    with pytest.raises(ValueError, match="lateral"):
        p.set_velocity({"y": .1})
    robot.accid = "SF_TRON2A_TEST"
    with pytest.raises(RuntimeError, match="ACCID"):
        p.set_velocity({"x": .1})
    assert not robot.commands


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True, "0.1", 5])
def test_invalid_velocity_never_sent(value):
    p, robot = plugin()
    with pytest.raises(ValueError):
        p.set_velocity({"x": value})
    assert not robot.commands


def test_arm_validation_checks_limits_without_motion():
    p, robot = plugin("fixed_arms")
    target = [0.] * 14
    assert p.dispatch("validate", {"_tool_name": "tron2_arm_target", "joint": target})["sent"] is False
    target[3] = 1
    with pytest.raises(ValueError, match="outside"):
        p.dispatch("validate", {"_tool_name": "tron2_arm_target", "joint": target})
    assert not robot.commands


def test_shutdown_closes_connection_even_when_zero_cannot_be_delivered():
    p, robot = plugin()
    p.set_velocity({"x": .1})
    robot.connected = False
    with pytest.raises(RuntimeError, match="unconfirmed"):
        p.stop()
    assert not robot.connected and p._target is None


def test_mcp_http_reports_only_selected_capabilities():
    from http.server import ThreadingHTTPServer
    import urllib.request
    from common.vendor_runtime import DriverBundle, make_handler
    p, robot = plugin("fixed_arms")
    bundle = DriverBundle([p])
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(lambda: bundle, "tron2", "tron2-test"))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    def call(method, params):
        request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp",
            data=json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.loads(response.read())
    try:
        assert call("initialize", {})["result"]["serverInfo"]["name"] == "tron2"
        names = {t["name"] for t in call("tools/list", {})["result"]["tools"]}
        assert "tron2_joint_states" in names and "tron2_velocity" not in names
        result = call("tools/call", {"name": "tron2_robot_info", "arguments": {"action": "get"}})
        assert json.loads(result["result"]["content"][0]["text"])["status"] == "WALK"
        assert not robot.commands
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=1)


def test_protocol_sample_preserves_envelope_time_without_retimestamping(connected):
    transport, sock = connected
    transport._receive({"accid": transport.accid, "title": "notify_robot_info", "timestamp": 1234,
                        "data": {"accid": transport.accid, "status": "WALK"}})
    first = transport.notification_sample("notify_robot_info")
    assert first["source_timestamp_ms"] == 1234
    assert first["session_id"] == 1
    assert transport.notification_sample("notify_robot_info") == first
    assert transport.notification("notify_robot_info")["status"] == "WALK"


class SamplingRobot(Robot):
    def __init__(self):
        super().__init__()
        self.requested = []
        self.invalid = False
    def command_sample(self):
        return None
    def sample(self, data):
        return {"data": data, "source_timestamp_ms": 1234,
                "received_at_ns": time.time_ns(), "received_monotonic_ns": time.monotonic_ns(),
                "session_id": self.session_id, "sequence": 1}
    def notification_sample(self, title):
        if title == "notify_imu":
            raise RuntimeError("IMU not enabled upstream")
        return self.sample({"accid": self.accid, "status": "WALK"})
    def request_sample(self, title):
        self.requested.append(title)
        if title == "request_get_joint_state":
            return self.sample({"names": ["j1"], "q": [float("nan") if self.invalid else .1],
                                "dq": [0.], "tau": [1.]})
        if title == "request_get_move_pose":
            return self.sample({"left_position": [0.] * 3, "right_position": [0.] * 3,
                                "left_quat": [1., 0., 0., 0.], "right_quat": [1., 0., 0., 0.]})
        return self.sample({"result": "success", "vendor_field": 1})


def test_poll_channels_follow_installed_configuration():
    robot = SamplingRobot()
    fixed = telemetry.TronTelemetry(robot, {}, "fixed_arms")
    fixed.poll_once()
    assert robot.requested == ["request_get_joint_state", "request_get_move_pose"]
    robot.requested.clear()
    mobile = telemetry.TronTelemetry(robot, {"gripper_state_enabled": True, "mobile_state_enabled": True}, "mobile_arms")
    mobile.poll_once()
    assert robot.requested == ["request_get_joint_state", "request_get_move_pose", "request_get_limx_2fclaw_state",
                               "request_lifter_state", "request_chassis_state"]
    leg = telemetry.TronTelemetry(robot, {"gripper_state_enabled": True, "mobile_state_enabled": True}, "biped")
    assert not leg.queries


def test_poll_invalid_stale_and_previous_connection_samples_are_unavailable():
    robot = SamplingRobot()
    stream = telemetry.TronTelemetry(robot, {}, "fixed_arms")
    stream.poll_once()
    data = stream.snapshot()
    assert data["channels"]["joint_states"]["fresh"]
    assert "accid" not in data["channels"]["robot_info"]["sample"]["data"]
    assert data["channels"]["imu"]["sample"] is None
    old = stream._samples["joint_states"]["received_monotonic_ns"]
    stream._samples["joint_states"]["received_monotonic_ns"] = old - 2_000_000_000
    assert stream.snapshot()["channels"]["joint_states"]["sample"] is None
    stream.poll_once()
    robot.session_id += 1
    assert stream.snapshot()["channels"]["joint_states"]["sample"] is None
    robot.invalid = True
    stream.poll_once()
    data = stream.snapshot()["channels"]["joint_states"]
    assert not data["fresh"] and data["error"]


def test_telemetry_stop_waits_for_one_query_and_sends_no_motion():
    robot = SamplingRobot()
    stream = telemetry.TronTelemetry(robot, {}, "fixed_arms")
    entered, release = threading.Event(), threading.Event()
    original = robot.request_sample
    def request(title):
        entered.set()
        assert release.wait(2)
        return original(title)
    robot.request_sample = request
    stream.start()
    assert entered.wait(1)
    stream._stop.set()
    release.set()
    stream.stop()
    assert robot.requested == ["request_get_joint_state"]
    assert not robot.commands and not stream.info()["polling"]


def test_telemetry_ros_publishes_freshness_envelope_and_stops():
    robot = SamplingRobot()
    nodes = []
    class Node:
        def __init__(self, *args, **kwargs):
            self.messages, self.destroyed = [], False
            nodes.append(self)
        def create_publisher(self, kind, topic, qos):
            return SimpleNamespace(publish=lambda message: self.messages.append(json.loads(message.data)))
        def create_timer(self, period, callback):
            self.tick = callback
        def destroy_node(self):
            self.destroyed = True
    executor = SimpleNamespace(add_node=lambda n: None, remove_node=lambda n: None)
    ros = SimpleNamespace(ctx_core=None, executor_core=executor)
    modules = {"rclpy.node": SimpleNamespace(Node=Node),
               "rclpy.qos": SimpleNamespace(qos_profile_sensor_data=None),
               "std_msgs.msg": SimpleNamespace(String=SimpleNamespace)}
    stream = telemetry.TronTelemetry(robot, {}, "biped", ros2=ros)
    with patch.dict(sys.modules, modules):
        stream.start()
        nodes[0].tick()
        assert nodes[0].messages[0]["channels"]["robot_info"]["sample"]["source_timestamp_ms"] == 1234
        assert not nodes[0].messages[0]["channels"]["imu"]["fresh"]
        stream.stop()
        nodes[0].tick()
        assert nodes[0].destroyed and len(nodes[0].messages) == 1
    assert not robot.commands


def test_command_capture_separates_send_from_physical_execution(connected):
    transport, sock = connected
    guid = transport.send("request_twist", {"x": .1, "y": 0., "z": 0.})
    sample = transport.command_sample()
    assert sample["guid"] == guid and sample["state"] == "sent"
    assert sample["values"]["x"] == .1 and sample["physical_execution_confirmed"] is False
    assert sample["sent_at_ns"] >= sample["submitted_at_ns"]
    transport.send("request_twist", {"x": 0., "y": 0., "z": 0.})
    assert transport.command_sample()["values"]["x"] == 0.


def test_query_sample_uses_response_timestamp_and_round_trip(connected):
    transport, sock = connected
    outcome = []
    worker = threading.Thread(target=lambda: outcome.append(transport.request_sample("request_get_joint_state")))
    worker.start()
    request = sock.outgoing.get(timeout=1)
    transport._receive({**request, "timestamp": 4321, "title": "response_get_joint_state",
                        "data": {"result": "success", "q": [1.]}})
    worker.join(timeout=1)
    assert outcome[0]["source_timestamp_ms"] == 4321
    assert outcome[0]["round_trip_ms"] >= 0
    assert outcome[0]["data"]["q"] == [1.]


def test_failed_velocity_submission_is_recorded_as_unknown(connected):
    transport, sock = connected
    def fail(raw):
        raise ConnectionError("write failed")
    sock.send = fail
    with pytest.raises(ConnectionError, match="outcome unknown"):
        transport.send("request_twist", {"x": .1, "y": 0., "z": 0.})
    assert transport.command_sample()["state"] == "submission_unknown"
    assert not transport.command_sample()["physical_execution_confirmed"]


def test_telemetry_card_dispatch_and_descriptor():
    from common.vendor_runtime import DriverBundle
    robot = SamplingRobot()
    p = device.TronPlugin({"profile": "fixed_arms"}, robot)
    p.telemetry.poll_once()
    bundle = DriverBundle([p])
    definition = next(t for t in bundle.get_all_tools() if t["name"] == "tron2_telemetry")
    assert definition["topic_out"][0]["format"] == "data/json"
    snapshot = bundle.dispatch("tron2_telemetry", {"action": "get"})
    assert snapshot["channels"]["joint_states"]["sample"]["data"]["q"] == [.1]
