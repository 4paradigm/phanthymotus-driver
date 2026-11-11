import sys
import threading
import types
from unittest.mock import patch

import pytest

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))

from device import BunkerMiniDevice, _validate_move_duration


class FakeCAN:
    def __init__(self):
        self.targets = []
        self.stops = 0
        self.target_validation = None

    def request_control_mode(self, mode):
        return {"changed": False, "control_mode": mode}

    def validate_target(self, linear, angular):
        self.target_validation = (linear, angular)
        return float(linear), float(angular)

    def set_target(self, linear, angular, lease_s):
        self.targets.append((linear, angular, lease_s))

    def snapshot(self):
        return {"last_tx_error": ""}

    def stop_motion(self):
        self.stops += 1
        return {"state": "stopped"}


def _bare_device():
    device = BunkerMiniDevice.__new__(BunkerMiniDevice)
    device._motion_enabled = True
    device._motion_lock = threading.Lock()
    device._active_move_id = None
    device._active_move_stop = None
    device._direct_until = 0.0
    device.bridge = types.SimpleNamespace(
        _state_topic="/bunker_mini/state/motor_state",
        _battery_topic="/bunker_mini/state/battery",
        _remote_controller_topic="/bunker_mini/state/remote_controller",
        _odom_topic="/bunker_mini/state/odom",
        _odom_field_topics={
            name: f"/bunker_mini/state/odom/{name}"
            for name in ("vx", "steering_angle_deg", "left_wheel_mm", "right_wheel_mm")},
    )
    device.config = {}
    device.can = FakeCAN()
    return device


@pytest.mark.parametrize("value", [True, "1", float("nan"), float("inf"), 0, 10.1])
def test_invalid_move_duration_is_rejected_before_motion_state_changes(value):
    device = _bare_device()
    with pytest.raises(ValueError):
        device.dispatch("move", {"duration_s": value, "vx": 0.1, "vyaw": 0})
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


def test_move_advertises_async_completion_contract():
    device = _bare_device()
    schema = next(item["inputSchema"] for item in device.get_tools()
                  if item["name"] == "base_drive")
    assert schema["x-completion"] == {"actions": ["move"], "timeout": 20}
    assert schema["properties"]["action"]["enum"] == ["move"]
    assert schema["properties"]["vyaw"]["description"].endswith("degrees/s")
    assert all("topic_in" not in card for card in device.get_tools())


def test_base_drive_uses_degrees_per_second_and_requests_can_mode_internally():
    device = _bare_device()

    class DeferredThread:
        def __init__(self, target, args, **_kwargs):
            self.target, self.args = target, args
        def start(self):
            pass

    with patch("device.threading.Thread", DeferredThread):
        result = device.dispatch("move", {
            "_tool_name": "base_drive", "vx": 0.1, "vyaw": 30, "duration_s": 1})
    assert device.can.target_validation == pytest.approx((0.1, 0.5235987756))
    assert result["state"] == "running"
    assert result["vyaw_deg_s"] == 30


def test_sensor_cards_use_familiar_names_and_flat_odometry_ports():
    cards = {card["name"]: card for card in _bare_device().get_tools()}
    assert set(cards) == {"motor_state", "battery", "remote_controller", "odom", "base_drive"}
    odom_ports = cards["odom"]["topic_out"]
    assert len(odom_ports) == 5
    assert [port["topic"].rsplit("/", 1)[-1] for port in odom_ports[1:]] == [
        "vx", "steering_angle_deg", "left_wheel_mm", "right_wheel_mm"]


def test_motor_state_fields_are_flat_and_freshness_aware():
    snapshot = {
        "connected": True,
        "system_state": {"vehicle_state": 0, "control_mode": 1, "error_flags": 0x85},
        "system_state_age_ms": 20,
        "motion_state": {"linear_velocity_m_s": 0.2, "steering_angle_deg": 12.5},
        "motion_state_age_ms": 25,
        "motor_state": {2: {"speed_rpm": 100, "motor_temperature_c": 40}, 3: {}},
        "motor_state_age_ms": {2: 10, 3: None},
    }
    state = BunkerMiniDevice._motor_state(snapshot)
    assert state["vehicle_state"] == "normal"
    assert state["control_mode"] == "can"
    assert state["remote_lost"] and state["emergency_stop"]
    assert state["motor_2_speed_rpm"] == 100
    assert state["motor_3_speed_rpm"] is None


def test_battery_and_remote_fields_are_flat():
    battery = BunkerMiniDevice._battery_state({
        "battery_state": {"soc_percent": 95, "voltage_v": 48.2},
        "battery_state_age_ms": 400,
        "battery_alarm": {"alarm_status_1": 4, "alarm_status_2": 1},
        "battery_alarm_age_ms": 500,
    })
    assert battery["soc_percent"] == 95
    assert battery["alarm_undervoltage"] is True
    assert battery["alarm_charge_overcurrent"] is True
    assert BunkerMiniDevice._remote_controller_state({
        "remote_controller": {"swa": 2, "left_stick_x": -15},
        "remote_controller_age_ms": 20}) == {
            "swa": 2, "left_stick_x": -15, "feedback_age_ms": 20}
    assert BunkerMiniDevice._remote_controller_state({
        "remote_controller": {"swa": 2, "left_stick_x": -15},
        "remote_controller_age_ms": 501}) is None


def test_publish_loop_drops_stale_remote_and_keeps_odom_receive_timestamp():
    device = _bare_device()
    device._enabled_sensors = {"remote_controller", "odom"}
    device._stop = threading.Event()
    device._state_thread = None
    device._last_battery_publish = 0.0
    snapshot = {
        "remote_controller": {"swa": 2, "left_stick_x": -15},
        "remote_controller_age_ms": 501,
        "wheel_odometry_raw": {},
        "motion_state": {},
    }
    device.can.snapshot = lambda: snapshot
    device.can.odometry_window = lambda: (
        [(1_800_000_000_123, [0.15, None, None, None, None, None])], None)
    published_remote = []
    published_odom = []
    device.bridge.publish_remote_controller = published_remote.append
    device.bridge.publish_odom = lambda sample, fields: published_odom.append((sample, fields))

    class OnePublish:
        calls = 0
        def wait(self, _timeout):
            self.calls += 1
            return self.calls > 1

    device._stop = OnePublish()
    device._publish_loop()

    assert published_remote == []
    assert published_odom[0][0]["stamp_ms"] == 1_800_000_000_123
    assert published_odom[0][0]["twist"][0] == 0.15


def test_actuator_start_and_stop_follow_lifecycle_contract():
    device = _bare_device()
    calls = []
    assert device.dispatch("start", {"_tool_name": "base_drive"}) == {"state": "ready"}
    assert device.dispatch("stop", {"_tool_name": "base_drive"}) == {
        "state": "idle"}
    assert device.dispatch("stop", {}) == {"state": "idle"}


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
        bridge.publish_battery({"voltage_v": 51.0})
        bridge.publish_remote_controller({"swa": 2})
        bridge.publish_odom({"twist": [0.1, None, None, None, None, None]},
                            {"vx": 0.1, "steering_angle_deg": 2.0,
                             "left_wheel_mm": 10, "right_wheel_mm": 11})
        bridge._robot_state_sub.callback(types.SimpleNamespace(data='{"ok":true}'))
        bridge.close()
    assert ("/bunker/state/battery", '{"voltage_v":51.0}') in events
    assert ("/bunker/state/remote_controller", '{"swa":2}') in events
    assert ("/bunker/state/odom", '{"twist":[0.1,null,null,null,null,null]}') in events
    assert ("/bunker/state/odom/vx", "0.1") in events
    assert ("/bunker/state/odom/right_wheel_mm", "11") in events
    assert ("/bunker/state/motor_state", '{"ok":true}') in events
    assert ("destroy_node", "bunker_mini_core_bridge") in events
    assert ("destroy_node", "bunker_mini_robot_bridge") in events
