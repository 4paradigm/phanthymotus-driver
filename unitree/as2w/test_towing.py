"""Offline contract tests for AS2W towing."""
import json
from http.client import HTTPConnection
import math
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

import pytest
import yaml

from unitree.as2w import rpc_proxy, towing
from unitree.as2w.test_driver import _Proxy, _install_device_stubs, _load
from unitree.as2w.towing import SpeedChannel, TowingConfig, _TowingCard, _derated_speed


ROOT = Path(__file__).parent


class Rig:
    def __init__(self, **config):
        self.now = 0.0
        self.speed = SpeedChannel()
        self.proxy = Mock()
        self.proxy.Move.return_value = 0
        self.proxy.StopMove.return_value = 0
        self.node = Mock()
        self.card = _TowingCard(
            {"enabled": True, **config}, "robot", self.proxy, self.speed,
            clock=lambda: self.now, wait=lambda _: None)
        self.card.bind(self.node)
        self.feed()

    def feed(self, vx=0.3, *, pitch=0.0, speed=True, imu=True):
        if speed:
            self.card.on_speed(self.now, vx, 0.0)
        if imu:
            self.card.on_imu(self.now, pitch)

    def action(self, action, **args):
        return self.card.dispatch(action, {"_tool_name": "towing", **args})

    def start(self, vx=0.3):
        return self.action("start", load_class="cart", vx=vx)

    def tick(self, at, vx=0.3):
        self.now = at
        self.feed(vx)
        self.card.tick()
        return self.action("info")

    def zero_frames(self):
        return sum(call.args == (0.0, 0.0, 0.0) for call in self.proxy.Move.call_args_list)


@pytest.mark.parametrize(("config", "field"), [
    ({"max_speed_mps": -1}, "max_speed_mps"),
    ({"max_speed_mps": 0.51}, "max_speed_mps"),
    ({"max_speed_mps": float("nan")}, "max_speed_mps"),
    ({"drag_warn_ratio": 0.7}, "drag_warn_ratio"),
    ({"zero_frame_count": True}, "zero_frame_count"),
    ({"loop_hz": 1, "publish_hz": 1}, "loop_timeout_ms"),
])
def test_rejects_invalid_configuration(config, field):
    rig = Rig(**config)
    assert rig.start()["field"] == field
    assert rig.action("info")["state"] == "idle"
    rig.proxy.Move.assert_not_called()


@pytest.mark.parametrize("vx", [0.51, 0.01, -0.1, float("nan"), float("inf"), True])
def test_rejects_invalid_speed(vx):
    rig = Rig()
    assert rig.start(vx)["field"] == "vx"
    rig.proxy.Move.assert_not_called()


def test_start_and_stop_with_zero_frames_and_no_late_publication():
    rig = Rig()
    assert rig.start()["state"] == "towing"
    rig.tick(0.05)
    rig.proxy.Move.assert_called_with(0.3, 0.0, 0.0)
    assert rig.action("stop")["state"] == "idle"
    assert rig.zero_frames() == 5
    published = rig.node.create_publisher.return_value.publish.call_args_list
    assert [json.loads(call.args[0].data)["state"] for call in published] == [
        "arming", "towing", "stopping", "idle"]
    rig.tick(0.5)
    assert rig.node.create_publisher.return_value.publish.call_count == 4
    assert rig.zero_frames() == 5


def test_stuck_from_standstill_requires_hold():
    rig = Rig()
    rig.feed(0.0)
    rig.start()
    for step in range(1, 16):
        assert rig.tick(step / 20, 0.0)["state"] == "towing"
    state = rig.tick(0.8, 0.0)
    assert state["flags"]["stuck"]
    assert state["state"] == "idle"
    assert rig.zero_frames() == 5


def test_moving_high_drag_stops_immediately():
    rig = Rig()
    rig.start()
    assert rig.tick(0.05, 0.1)["flags"]["stuck"]
    assert rig.zero_frames() == 5


def test_detach_requires_ratio_drop_and_acceleration():
    rig = Rig(detach_drop_ratio=0.5)
    rig.start(0.25)
    first = rig.tick(0.05, 0.1875)
    assert first["limited_by"] == "drag_derate"
    state = rig.tick(0.10, 0.3375)
    assert first["drag_ratio"] - state["drag_ratio"] == pytest.approx(0.6)
    assert state["flags"]["detached"]
    assert rig.zero_frames() == 5


def test_linear_derate_and_minimum_speed():
    config = TowingConfig()
    outputs = [_derated_speed(config, 0.3, ratio) for ratio in (0.25, 0.3, 0.4, 0.5, 0.6)]
    assert outputs == sorted(outputs, reverse=True)
    assert outputs[0] == 0.3
    assert outputs[-1] == pytest.approx(0.05, abs=1e-6)


def test_publish_rate_and_immediate_derate_event():
    rig = Rig()
    rig.start()
    publish = rig.node.create_publisher.return_value.publish
    for step in range(1, 4):
        rig.tick(step / 20)
    assert publish.call_count == 2
    rig.tick(0.20)
    assert publish.call_count == 3
    rig.tick(0.25, 0.2)
    assert publish.call_count == 4
    payload = json.loads(publish.call_args.args[0].data)
    assert payload["events"][-1]["type"] == "derate"


def test_shared_reservation_busy_then_release_and_restart():
    rig = Rig()
    active = True
    rig.speed.observe("loco", lambda: active)
    assert rig.start() == {"error": "speed channel busy", "owner": "loco"}
    active = False
    assert rig.start()["owner"] == "towing"
    rig.action("stop")
    assert rig.speed.owner is None
    assert rig.start()["owner"] == "towing"


@pytest.mark.parametrize("missing", ["speed", "imu"])
def test_feedback_timeout_stops(missing):
    rig = Rig()
    rig.start()
    rig.now = 0.301
    rig.feed(**{missing: False})
    rig.card.tick()
    state = rig.action("info")
    assert state["events"][-1]["type"] == "fault"
    assert not state["fresh"]
    assert rig.zero_frames() == 5


def test_loop_timeout_with_fresh_feedback():
    rig = Rig()
    rig.start()
    state = rig.tick(0.501)
    assert state["events"][-1]["type"] == "fault"
    assert state["error"] == "loop timeout"
    assert rig.zero_frames() == 5


def test_concurrent_stop_and_shutdown_emit_only_one_stop_sequence():
    rig = Rig()
    rig.start()
    barrier = threading.Barrier(2)
    errors = []

    def run(action):
        try:
            barrier.wait(timeout=2)
            action()
        except Exception as exc:
            errors.append(exc)

    workers = [threading.Thread(target=run, args=(action,))
               for action in (rig.card.stop, rig.card.shutdown)]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join(2)
    assert not any(worker.is_alive() for worker in workers)
    assert not errors
    assert rig.zero_frames() == 5
    assert rig.speed.owner is None
    states = [json.loads(call.args[0].data)["state"]
              for call in rig.node.create_publisher.return_value.publish.call_args_list]
    assert states == ["arming", "towing", "stopping", "idle"]


def test_yaml_and_image_metadata():
    driver = yaml.safe_load((ROOT / "driver.yaml").read_text(encoding="utf-8"))
    config = yaml.safe_load((ROOT / "config.yaml").read_text(encoding="utf-8"))
    assert {"name": "towing", "type": "actuator"} in driver["cards"]
    assert config["towing"] == vars(TowingConfig())
    assert any(line.startswith("COPY ") and "towing.py" in line
               for line in (ROOT / "Dockerfile").read_text().splitlines())


def test_detach_requires_acceleration_even_if_derating_changes_the_ratio():
    rig = Rig()
    rig.start()
    rig.tick(0.05, 0.15)
    state = rig.tick(0.10, 0.16)
    assert state["state"] == "towing"
    assert not state["flags"]["detached"]


def test_initial_acceleration_does_not_trigger_detachment():
    rig = Rig()
    rig.feed(0.0)
    rig.start()
    assert rig.tick(0.05, 0.03)["state"] == "towing"
    state = rig.tick(0.10, 0.3)
    assert state["state"] == "towing"
    assert not state["flags"]["detached"]


def test_move_exception_releases_channel_after_zero_sequence():
    rig = Rig()
    rig.start()
    rig.proxy.Move.side_effect = [OSError("Move failed"), 0, 0, 0, 0, 0]
    state = rig.tick(0.05)
    assert state["events"][-1]["type"] == "fault"
    assert state["state"] == "idle"
    assert rig.zero_frames() == 5
    assert rig.speed.owner is None


def test_failed_zero_sequence_reports_failure_and_attempts_stop_move():
    rig = Rig()
    rig.start()
    rig.proxy.Move.return_value = 3104
    result = rig.action("stop")
    assert rig.zero_frames() == 5
    rig.proxy.StopMove.assert_called_once_with()
    assert "not acknowledged" in result["error"]
    assert rig.speed.owner is None


def test_unacknowledged_stop_blocks_handoff_until_retry():
    rig = Rig()
    rig.start()
    rig.proxy.Move.return_value = 3104
    rig.proxy.StopMove.return_value = 3104
    assert rig.action("stop")["owner"] == "towing_fault"
    assert rig.start() == {"error": "speed channel busy", "owner": "towing_fault"}
    rig.proxy.Move.return_value = 0
    assert rig.action("stop")["owner"] is None
    assert rig.start()["owner"] == "towing"


def test_derate_does_not_toggle_with_unchanged_feedback():
    rig = Rig()
    rig.start()
    samples = [rig.tick(step / 20, 0.2) for step in range(1, 7)]
    assert samples[0]["cmd"]["vx"] < 0.3
    assert all(sample["cmd"] == samples[0]["cmd"] for sample in samples)
    assert [event["type"] for event in samples[-1]["events"]] == ["derate"]
    assert rig.tick(0.35, 0.24)["cmd"]["vx"] > samples[-1]["cmd"]["vx"]


def test_slope_and_current_trend_are_diagnostics_only():
    rig = Rig()
    rig.feed(pitch=10)
    rig.start()
    rig.card.on_current(0.0, -1000)
    rig.tick(0.05)
    rig.card.on_current(0.10, -2000)
    state = rig.tick(0.10)
    assert state["slope_warning"] is True
    assert state["bms_current_magnitude_trend"] == "rising"
    assert state["electrical_power_w"] is None
    assert state["traction_force_n"] is None
    assert state["cmd"]["vx"] == 0.3


@pytest.fixture
def bundle_modules():
    with patch.dict(sys.modules):
        _install_device_stubs()
        sys.modules.setdefault("fcntl", ModuleType("fcntl"))
        sys.modules["rclpy.node"].Node.create_subscription = Mock()
        sys.modules["towing"] = towing
        for name in ("camera_specs", "device", "multimedia", "controlled_spatial",
                     "lidar_backend", "sensor_worker", "lidar", "slam_mapping",
                     "vision_capture", "loco_servo", "main"):
            _load(name, ROOT / f"{name}.py")
        sys.modules["device"]._StateNode = Mock()
        sys.modules["multimedia"]._MicNode = Mock()
        sys.modules["multimedia"]._SpeakerNode = Mock()
        sys.modules["lidar"]._LidarNode = Mock()
        sys.modules["lidar"].LidarPlugin._start_process = Mock()
        sys.modules["controlled_spatial"]._SpatialRpcProxy = Mock()
        sys.modules["slam_mapping"]._SlamMappingNode = Mock()
        yield sys.modules["main"], sys.modules["controlled_spatial"]


def test_registration_preserves_baseline_tools_and_dispatches(bundle_modules):
    main, _ = bundle_modules
    cfg = yaml.safe_load((ROOT / "config.yaml").read_text(encoding="utf-8"))
    del cfg["towing"]
    reference = subprocess.run(
        ["git", "show", "d6509f6726a4d3d39ca1f0632f2424f7999ff2b5:unitree/as2w/main.py"],
        cwd=ROOT, text=True, encoding="utf-8", capture_output=True)
    baseline_bundle = main.Bundle
    if reference.returncode == 0:
        baseline = {"__name__": "as2w_main_baseline", "__file__": str(ROOT / "main.py")}
        exec(compile(reference.stdout, baseline["__file__"], "exec"), baseline)
        baseline_bundle = baseline["Bundle"]
    expected = baseline_bundle(cfg, "robot", Mock(), _Proxy(), "eth0").tools()
    cfg["towing"] = {"enabled": False}
    bundle = main.Bundle(cfg, "robot", Mock(), _Proxy(), "eth0")
    with main.ThreadingHTTPServer(("127.0.0.1", 0), main.handler(bundle)) as server:
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        connection = HTTPConnection(*server.server_address, timeout=2)
        try:
            payload = json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/list"})
            connection.request("POST", "/mcp", body=payload)
            response = connection.getresponse()
            assert response.status == 200
            tools = json.loads(response.read())["result"]["tools"]
        finally:
            connection.close()
            server.shutdown()
            worker.join(2)
    assert [tool for tool in tools if tool["name"] != "towing"] == expected
    assert [tool for tool in tools if tool["name"] == "towing"] == [
        bundle.plugins[0].get_tools()[0]]
    assert bundle.call("towing", {"action": "info"})["state"] == "idle"
    assert bundle.call("towing", {"action": "start", "load_class": "cart", "vx": 0.3})["code"] == "DISABLED"


def test_real_plugin_admission_excludes_both_directions(bundle_modules):
    main, _ = bundle_modules
    proxy = _Proxy()
    cfg = {"towing": {"enabled": True}, "plugins": {
        name: {"enabled": False} for name in (
            "state", "mic", "speaker", "led", "camera", "lidar", "slam_mapping")}}
    cfg["plugins"]["loco_servo"] = {"enabled": True, "dry_run": False}
    bundle = main.Bundle(cfg, "robot", Mock(), proxy, "eth0")
    towing_plugin, loco, servo, _, nav = bundle.plugins
    card = towing_plugin._card
    card._wait = lambda _: None
    card.bind(Mock())
    card.on_speed(time.monotonic(), 0.3, 0.0)
    card.on_imu(time.monotonic(), 0.0)
    start = {"action": "start", "load_class": "cart", "vx": 0.3}
    move = {"action": "move", "vx": 0.2, "vy": 0, "vyaw": 0}

    assert bundle.call("loco", dict(move))["accepted"]
    assert bundle.call("towing", dict(start))["owner"] == "loco"
    loco.stop()
    servo._running = True
    assert bundle.call("towing", dict(start))["owner"] == "loco_servo"
    servo._paused = True
    nav._navigation_active = True
    assert bundle.call("towing", dict(start))["owner"] == "controlled_spatial"
    nav._navigation_active = False

    assert bundle.call("towing", dict(start))["owner"] == "towing"
    assert "towing" in bundle.call("loco", dict(move))["reason"]
    with patch.object(proxy, "GetState", wraps=proxy.GetState) as state_rpc:
        assert bundle.call("loco", {"action": "get_state"})["ret"] == 3104
        state_rpc.assert_not_called()
    assert "towing" in bundle.call("loco_servo", {"action": "resume"})["message"]
    assert "towing" in bundle.call("special_motion", {
        "action": "front_flip", "confirm": True})["reason"]
    assert "towing" in bundle.call("controlled_spatial", {"action": "navigate_to"})["reason"]
    assert "towing" in bundle.call("controlled_spatial", {"action": "stop"})["reason"]
    nav._client.call.assert_not_called()
    bundle.call("towing", {"action": "stop"})
    assert bundle.call("loco", dict(move))["accepted"]
    with patch.object(proxy, "StopMove", return_value=3104):
        assert bundle.call("loco", {"action": "stop"})["ret"] == 3104
    assert bundle.call("towing", dict(start))["owner"] == "loco"
    assert bundle.call("loco", {"action": "stop"})["ret"] == 0
    assert bundle.call("towing", dict(start))["owner"] == "towing"
    bundle.call("towing", {"action": "stop"})


def test_rejected_navigation_replacement_preserves_prior_owner(bundle_modules):
    _, spatial = bundle_modules
    nav = spatial.ControlledSpatialPlugin({}, "robot", Mock(), "eth0")
    channel = SpeedChannel()
    nav._towing_channel = channel
    nav._chassis_lock = channel.lock
    channel.observe("controlled_spatial", nav.is_navigating)
    nav._navigation_active = True
    nav._client.call.return_value = {"code": 123, "response": {}}
    assert nav.dispatch("navigate_to", {})["ret"] == 123
    assert channel.acquire(object()) == "controlled_spatial"
    nav._client.call.return_value = {"code": 0, "response": {}}
    assert nav.dispatch("pause_navigation", {})["ret"] == 0
    assert channel.acquire(object()) is None


def test_raw_dds_adapters_work_without_other_cards(bundle_modules):
    _, _ = bundle_modules
    subscribers = {}

    class Subscriber:
        def __init__(self, topic, _kind):
            subscribers[topic] = self
            self.closed = False

        def Init(self, callback, _queue_length):
            self.callback = callback

        def Close(self):
            self.closed = True

    node = Mock()
    with patch.object(sys.modules["unitree_sdk2py.core.channel"], "ChannelSubscriber", Subscriber), \
            patch.object(sys.modules["rclpy.node"], "Node", return_value=node):
        plugin = towing.TowingPlugin(
            {"towing": {"enabled": True}}, "robot", _Proxy(), True,
            None, None, None, None)
        plugin._card._wait = lambda _: None
        plugin.start()
        try:
            assert set(subscribers) == {"rt/lowstate", "rt/lf/bmsstate", "rt/lf/sportmodestate"}
            angle = math.pi / 4
            imu = SimpleNamespace(quaternion=[math.cos(angle), 0, 0, math.sin(angle)],
                                  gyroscope=[0, 0, 0.1], rpy=[0, 0.2, 0])
            subscribers["rt/lowstate"].callback(SimpleNamespace(imu_state=imu))
            subscribers["rt/lf/sportmodestate"].callback(
                SimpleNamespace(velocity=[0, 0.3, 0], imu_state=imu))
            subscribers["rt/lf/bmsstate"].callback(SimpleNamespace(current=-1200))
            info = plugin.dispatch("start", {"_tool_name": "towing", "load_class": "cart", "vx": 0.3})
            assert info["owner"] == "towing"
            assert info["meas"] == pytest.approx({"vx": 0.3, "wz": 0.1})
            assert info["slope_deg"] == pytest.approx(math.degrees(0.2))
            assert info["bms_reported_ma"] == -1200
        finally:
            plugin.stop()
    assert all(subscriber.closed for subscriber in subscribers.values())
    node.destroy_node.assert_called_once()


def test_towing_rpc_discards_expired_dispatch_before_vendor_move(monkeypatch):
    commands, results = queue.Queue(), queue.Queue()
    client = Mock()
    monkeypatch.setattr(rpc_proxy, "_install_logsafe", lambda: None)
    with patch.dict(sys.modules, {
        "unitree_sdk2py.core.channel": SimpleNamespace(ChannelFactoryInitialize=lambda *_: None),
        "unitree_sdk2py.as2.sport.sport_client": SimpleNamespace(SportClient=lambda: client),
    }):
        commands.put((1, "Move", (0.3, 0.0, 0.0), time.monotonic() - 1))
        commands.put(None)
        rpc_proxy._serve(commands, results, "eth0", "sport")
    assert results.get_nowait() == {"ready": True}
    assert results.get_nowait() == {"request_id": 1, "result": 3104}
    client.Move.assert_not_called()
