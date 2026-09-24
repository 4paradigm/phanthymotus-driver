"""ROS wire and bundle admission tests using the existing T800 ROS doubles."""
import copy
import json
import os
from pathlib import Path
import sys
import threading
import types
from unittest import mock
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parents[1]))

from teleop import TeleopControlPlugin
from test_teleop import Rig
from test_contracts import load_main_without_ros
from test_device_contract import CONFIG, FakeNode, FakeRos, install_ros_stubs, load_device


class DeferredThread:
    def __init__(self, **kwargs):
        self.kwargs = kwargs

    def start(self):
        pass

    def join(self, timeout=None):
        pass

    def is_alive(self):
        return False


@pytest.fixture
def plugin():
    with mock.patch.dict(sys.modules):
        install_ros_stubs()
        config = copy.deepcopy(CONFIG)
        config["plugins"] = {"teleop_control": {"enabled": True, "mode": "live"}}
        ros = FakeRos()
        for executor in (ros.executor_core, ros.executor_robot):
            executor.remove_node = lambda node, executor=executor: executor.nodes.remove(node)
        rig = Rig()
        state = types.SimpleNamespace(dispatch=lambda key, args: rig.snapshot["motion" if key == "motion_state" else key])
        instance = TeleopControlPlugin(config, "t800", ros, state)
        instance.control = rig.control
        instance.control.publish = instance._publish
        with mock.patch("threading.Thread", DeferredThread), mock.patch.object(FakeNode, "destroy_node", lambda self: None, create=True):
            instance.start()
            yield instance, rig
            instance.stop()


def test_ros_contract_two_cards_and_only_ten_arm_override_joints(plugin):
    instance, rig = plugin
    tool = instance.get_tool()
    assert tool["name"] == "teleop_control" and tool["multiInstance"] is False
    assert tool["topic_in"][0]["topic"] == "/teleop/command"
    assert tool["topic_out"][0]["topic"] == "/teleop/state"
    assert len(instance._core_node.subscriptions) == 1
    assert not instance._publisher.messages
    instance.dispatch("start", {"input_topic": "/teleop/command"})
    instance._on_input(types.SimpleNamespace(data=json.dumps(rig.frame())))
    rig.control.tick()
    assert not instance._publisher.messages
    rig.step()
    message = instance._publisher.messages[-1]
    assert message.joint_indices == list(range(13, 23))
    assert message.weight == 1.
    assert len(message.position) == len(message.velocity) == 10
    assert message.stiffness == [30., 30., 15., 30., 15., 40., 40., 20., 40., 20.]
    assert message.torque == message.feed_forward_torque == [0.] * 10
    instance._publish_monitor()
    state = json.loads(instance._monitor.messages[-1].data)
    assert state["execution"]["state"] == "following"
    instance.dispatch("stop", {})
    assert instance._publisher.messages[-1].weight == 0.
    assert json.loads(instance._monitor.messages[-1].data)["state"] == "idle"


def test_ros_input_rejects_duplicate_keys_oversize_and_non_object_json(plugin):
    instance, rig = plugin
    rig.start()
    for value in ('{"schema":1,"schema":2}', " " * 65537, "[1,2]", '{"x": NaN}'):
        instance._on_input(types.SimpleNamespace(data=value))
    assert rig.control.rejected == 4
    assert not instance._publisher.messages


def test_adapter_uses_fresh_full_state_and_planner_feedback(plugin):
    instance, rig = plugin
    assert instance._feedback()["planner"]["age_sec"] is None
    instance._on_planner(types.SimpleNamespace(status=1, request_id=7))
    snapshot = instance._feedback()
    assert snapshot["planner"]["request_id"] == 7
    assert snapshot["planner"]["age_sec"] < .1
    assert len(snapshot["joints"]["joints"]) == 25


def bundle_with_control():
    module = load_main_without_ros()
    bundle = module.T800DeviceBundle.__new__(module.T800DeviceBundle)
    rig = Rig()
    teleop = types.SimpleNamespace(
        motion_active=rig.control.motion_active, halt=rig.control.halt,
        dispatch=lambda action, args: rig.control.begin(args) if action == "start" else rig.control.halt(),
        get_tool=lambda: {"name": "teleop_control", "type": "actuator", "inputSchema": {}},
    )
    bundle._teleop = teleop
    bundle._active_plugins = [teleop]
    bundle._motion_instances = {}
    bundle._motion_events = None
    bundle._motion_admission_lock = threading.RLock()
    bundle._motion_inflight = 0
    bundle._motion_interrupt_group = types.SimpleNamespace(
        blocking_outputs=lambda: ["teleop_control"] if teleop.motion_active() else [])
    return bundle, rig


@pytest.mark.parametrize("name", ["joint_override", "joint_bridge", "virtual_gamepad", "joint_plan"])
def test_bundle_rejects_existing_stream_or_pending_planner_before_teleop_start(name):
    bundle, rig = bundle_with_control()
    if name == "joint_plan":
        plugin = types.SimpleNamespace(dispatch=lambda *args: {"status": 1, "request_id": 7,
            "last_request": {"request_id": 8}})
    else:
        plugin = types.SimpleNamespace(_stream=types.SimpleNamespace(
            snapshot=lambda: types.SimpleNamespace(active=True)))
    bundle._motion_instances[name] = plugin
    result = bundle.dispatch("teleop_control", {"action": "start", "input_topic": "/teleop/command"})
    assert name in result["blocking_outputs"] and not rig.control.running


def test_bundle_blocks_other_motion_but_safety_can_interrupt_even_failed_release():
    bundle, rig = bundle_with_control()
    rig.start()
    rig.step()
    calls = []
    bundle._active_plugins.append(types.SimpleNamespace(
        get_tool=lambda: {"name": "safety", "type": "actuator", "inputSchema": {}},
        dispatch=lambda action, args: calls.append(action) or {"state": "requested"}))
    for name in ("joint_override", "head", "native_node_control", "motor_power"):
        assert bundle.dispatch(name, {"action": "command"})["error"] == "teleop_control_owns_motion"
    rig.control.publish = lambda *args: (_ for _ in ()).throw(RuntimeError("offline"))
    assert bundle.dispatch("safety", {"action": "emergency_passive"})["state"] == "requested"
    assert calls == ["emergency_passive"] and not rig.control.running


def test_admission_does_not_race_an_inflight_legacy_motion_request():
    bundle, rig = bundle_with_control()
    entered, finish = threading.Event(), threading.Event()
    def action(name, args):
        entered.set()
        assert finish.wait(2)
        return {"state": "done"}
    bundle._active_plugins.append(types.SimpleNamespace(
        get_tool=lambda: {"name": "joint_plan", "type": "actuator", "inputSchema": {}}, dispatch=action))
    thread = threading.Thread(target=bundle.dispatch, args=("joint_plan", {"action": "plan"}))
    thread.start()
    assert entered.wait(1)
    result = bundle.dispatch("teleop_control", {"action": "start", "input_topic": "/teleop/command"})
    assert result["error"] == "motion_output_busy" and result["inflight"] == 1
    assert not rig.control.running
    finish.set()
    thread.join(2)
    assert bundle._motion_inflight == 0


@pytest.mark.parametrize("name", ["motor_power", "native_node_control"])
def test_read_only_card_lifecycle_start_is_allowed_during_teleop(name):
    bundle, rig = bundle_with_control()
    rig.start()
    bundle._active_plugins.append(types.SimpleNamespace(
        get_tool=lambda: {"name": name, "type": "actuator", "inputSchema": {}},
        dispatch=lambda action, args: {"state": "ready"}))
    assert bundle.dispatch(name, {"action": "start"})["state"] == "ready"
    assert rig.control.running and not rig.messages


def test_actual_bundle_wires_card_into_interrupts_and_safety():
    with mock.patch.dict(sys.modules):
        device = load_device()
        sys.modules["device"] = device
        module = load_main_without_ros()
        config = copy.deepcopy(CONFIG)
        config["plugins"] = {"teleop_control": {"enabled": True}, "safety": {"enabled": True}}
        with mock.patch.object(device, "_t800_acp_preflight", return_value={"state": "ready"}):
            bundle = module.T800DeviceBundle(config, "t800", FakeRos())
        assert bundle._teleop in bundle._plugins
        assert bundle._teleop in bundle._motion_instances["safety"]._controls
        assert "teleop_control" in bundle._motion_interrupt_group._callbacks


def test_dds_profile_keeps_core_loopback_separate_from_robot_interface():
    module = load_main_without_ros()
    config = {"ros": {"robot_interface": "robot0", "robot_domain_id": 69, "core_domain_id": 42},
        "plugins": {"teleop_control": {"enabled": True}}}
    with mock.patch.dict(os.environ, {}, clear=True), mock.patch.object(module.socket, "if_nameindex",
            return_value=[(1, "lo"), (2, "robot0")]):
        module._configure_cyclonedds(config)
        profiles = [ET.fromstring(value) for value in os.environ["CYCLONEDDS_URI"].split(",")]
        domains = {item.attrib["Id"]: item for profile in profiles for item in profile.findall("Domain")}
        assert len(profiles) == 2 and all(len(profile.findall("Domain")) == 1 for profile in profiles)
        robot = domains["69"].find("General/Interfaces/NetworkInterface")
        local = domains["42"].find("General/Interfaces/NetworkInterface")
        assert robot.attrib == {"name": "robot0"}
        assert local.attrib == {"address": "127.0.0.1"}
        assert domains["42"].find("General/AllowMulticast").text == "false"
        assert domains["42"].find("Discovery/Peers/Peer").get("Address") == "127.0.0.1"
        config["ros"]["robot_domain_id"] = 42
        with pytest.raises(ValueError, match="separate robot domain"):
            module._configure_cyclonedds(config)
