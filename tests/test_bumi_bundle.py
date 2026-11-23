"""Bumi bundle plugin wiring, without ROS, SDK modules, or robot hardware."""

from __future__ import annotations

import sys
import types
import importlib.util
import json
import threading
import urllib.request
from contextlib import nullcontext
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / "noetix" / "bumi"


class FakeHighCtrl:
    def publish_cmd(self, *_args):  # pragma: no cover - a safety tripwire
        raise AssertionError("bundle registration must not send motion commands")


class FakeLocoPlugin:
    def __init__(self, plugin_config, namespace, executor, high_ctrl):
        self.plugin_config = plugin_config
        self.namespace = namespace
        self.executor = executor
        self.high_ctrl = high_ctrl
        self.servo = None
        self.walking_mode = None
        self.started = False
        self.stopped = False

    def attach_servo(self, servo):
        self.servo = servo

    def attach_walking_mode(self, walking_mode):
        self.walking_mode = walking_mode

    def is_moving(self):
        return False

    def reserve_walking_mode(self):
        return nullcontext()

    def get_tools(self):
        return [{"name": "loco", "type": "actuator"}]

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def dispatch(self, action, _args):
        return {"state": action}


class FakeSpeakerPlugin:
    def __init__(self, plugin_config, namespace, executor, media_ctrl):
        self.media_ctrl = media_ctrl

    def get_tool(self):
        return {"name": "speaker", "type": "actuator"}

    def start(self):
        pass

    def stop(self):
        pass


@pytest.fixture
def bundle_module(monkeypatch):
    """Load the real bundle and servo card with hardware imports replaced."""
    fake_rclpy = types.ModuleType("rclpy")
    fake_executors = types.ModuleType("rclpy.executors")
    fake_rclpy.executors = fake_executors
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.executors", fake_executors)

    fake_device = types.ModuleType("device")
    fake_device.LocoPlugin = FakeLocoPlugin
    fake_device.SpeakerPlugin = FakeSpeakerPlugin
    monkeypatch.setitem(sys.modules, "device", fake_device)
    monkeypatch.syspath_prepend(str(BUMI))
    previous_loco_servo = sys.modules.pop("loco_servo", None)

    # The host test runner is Python 3.9 while the Bumi image is Python 3.10+.
    # Postponing annotations lets this test execute the unchanged bundle source
    # without weakening the production runtime requirement.
    source = (BUMI / "main.py").read_text()
    if "from __future__ import annotations" not in source:
        source = "from __future__ import annotations\n" + source
    module = types.ModuleType("bumi_main_for_test")
    module.__file__ = str(BUMI / "main.py")
    exec(compile(source, module.__file__, "exec"), module.__dict__)
    yield module
    sys.modules.pop("loco_servo", None)
    if previous_loco_servo is not None:
        sys.modules["loco_servo"] = previous_loco_servo


@pytest.fixture
def locomotion_bundle_module(bundle_module, monkeypatch):
    """Use the actual locomotion cards and bundle with only ROS imports stubbed."""
    imports = {
        "rclpy.node": {"Node": object},
        "rclpy.qos": {
            "QoSProfile": lambda **kwargs: None,
            "ReliabilityPolicy": types.SimpleNamespace(BEST_EFFORT=0),
            "HistoryPolicy": types.SimpleNamespace(KEEP_LAST=0),
            "DurabilityPolicy": types.SimpleNamespace(VOLATILE=0),
        },
        "std_msgs": {}, "std_msgs.msg": {"String": object},
        "audio_msgs": {}, "audio_msgs.msg": {"AudioChunk": object},
        "sensor_msgs": {}, "sensor_msgs.msg": {"CompressedImage": object, "Image": object},
    }
    for name, attributes in imports.items():
        stub = types.ModuleType(name)
        stub.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, stub)
    spec = importlib.util.spec_from_file_location("bumi_locomotion_for_test", BUMI / "device.py")
    device = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(device)
    monkeypatch.setattr(device, "_acp_notify", lambda *_args: None)
    monkeypatch.setattr(device, "_get_default_cmd", lambda: "DEFAULT")
    monkeypatch.setitem(sys.modules, "device", device)
    return bundle_module


def _config(*, loco=True, loco_servo=True, servo_config=None):
    return {
        "plugins": {
            "loco": {"enabled": loco},
            "loco_servo": {
                "enabled": loco_servo,
                **(servo_config or {}),
            },
        }
    }


def test_enabled_loco_servo_registers_and_reuses_loco(bundle_module):
    high_ctrl = FakeHighCtrl()
    executor = object()
    config = _config(servo_config={"dry_run": True, "expected_hz": 10})

    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", executor, high_ctrl, media_ctrl=None)

    loco, servo = bundle._plugins
    assert isinstance(loco, FakeLocoPlugin)
    assert servo.PREFIX == "locoservo"
    assert servo._loco is loco
    assert loco.servo is servo
    assert servo._ns == "bumi_test"
    assert servo._executor is executor
    assert servo._high_ctrl is high_ctrl
    assert servo._dry_run is True
    assert [tool["name"] for tool in bundle.get_all_tools()] == [
        "loco", "loco_servo"]


def test_loco_servo_enablement_is_not_coupled_to_loco_card(bundle_module):
    bundle = bundle_module.BumiDeviceBundle(
        _config(loco=False), "bumi_test", object(), FakeHighCtrl(), None)

    assert [tool["name"] for tool in bundle.get_all_tools()] == ["loco_servo"]
    assert bundle._plugins[0]._loco is None


@pytest.mark.parametrize(
    ("enabled", "high_ctrl"),
    [(False, FakeHighCtrl()), (True, None)],
)
def test_loco_servo_is_skipped_when_disabled_or_controller_missing(
        bundle_module, enabled, high_ctrl):
    bundle = bundle_module.BumiDeviceBundle(
        _config(loco=False, loco_servo=enabled),
        "bumi_test", object(), high_ctrl, None)

    assert bundle.get_all_tools() == []


def test_bundle_lifecycle_and_dispatch_include_loco_servo(bundle_module):
    bundle = bundle_module.BumiDeviceBundle(
        _config(), "bumi_test", object(), FakeHighCtrl(), None)
    loco, servo = bundle._plugins

    bundle.start_all()
    assert loco.started is True
    assert bundle.dispatch("loco_servo", {"action": "info"})["dry_run"] is True

    bundle.stop_all()
    assert loco.stopped is True
    assert servo._running is False


def test_existing_bumi_plugin_loading_is_unchanged(bundle_module):
    config = _config()
    config["plugins"]["speaker"] = {"enabled": True}

    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", object(), FakeHighCtrl(), media_ctrl=object())

    assert [tool["name"] for tool in bundle.get_all_tools()] == [
        "loco", "loco_servo", "speaker"]


def test_packaged_defaults_advertise_walking_mode_without_contacting_robot(
        locomotion_bundle_module, monkeypatch):
    """A fresh image exposes both embedded actions without contacting hardware."""
    import walking_mode

    def unexpected_access(*_args, **_kwargs):
        raise AssertionError("registering a card must not contact the robot")

    class ControllerTripwire:
        def __getattr__(self, _name):
            raise AssertionError("registering a card must not access the SDK")

    monkeypatch.setattr(walking_mode, "discover_host", unexpected_access)
    monkeypatch.setattr(walking_mode, "WalkingModeClient", unexpected_access)
    default = yaml.safe_load((BUMI / "config.yaml").read_text(encoding="utf-8"))
    plugins = default["plugins"]
    config = {"plugins": {name: plugins[name] for name in ("loco", "walking_mode")}}
    bundle = locomotion_bundle_module.BumiDeviceBundle(
        config, "bumi_test", object(), ControllerTripwire(), None)

    bundle.start_all()
    tools = {tool["name"]: tool for tool in bundle.get_all_tools()}
    assert "walking_mode" not in tools
    for name in ("loco", "loco_relative_move"):
        schema = tools[name]["inputSchema"]
        assert schema["required"] == ["action"]
        assert "walking_mode" in schema["properties"]["action"]["enum"]
        assert schema["properties"]["walking_model"]["enum"] == ["lower_body", "whole_body"]
        assert schema["x-action-params"]["walking_mode"]["params"] == ["walking_model"]
        assert schema["allOf"] == [{
            "if": {"properties": {"action": {"const": "walking_mode"}}},
            "then": {"required": ["walking_model"]}}]
        assert "walking_mode" not in schema.get("x-completion", {}).get("actions", [])
    assert tools["loco_relative_move"]["inputSchema"]["x-completion"] == {
        "actions": ["rotate", "move"], "timeout": 120}
    assert bundle._plugins[0]._walking_mode is bundle._walking_mode
    for name in ("loco", "loco_relative_move"):
        assert bundle.dispatch(name, {"action": "start"}) == {"state": "ready"}
    # Existing shutdown braking is independent of declaration/startup checks.
    monkeypatch.setattr(bundle._plugins[0], "_stop_move", lambda: {"state": "stopped"})
    bundle.stop_all()
    driver = yaml.safe_load((BUMI / "driver.yaml").read_text(encoding="utf-8"))
    assert "walking_mode" not in [card["name"] for card in driver["cards"]]


@pytest.mark.parametrize("model", ["lower_body", "whole_body"])
def test_removed_standalone_tool_rejects_calls(bundle_module, model, monkeypatch):
    bundle = bundle_module.BumiDeviceBundle(
        {"plugins": {"loco": {"enabled": True}, "walking_mode": {"enabled": True}}},
        "bumi_test", object(), FakeHighCtrl(), None)
    bundle.start_all()
    loco, = bundle._plugins
    switcher = bundle._walking_mode
    assert switcher._loco is loco
    assert loco.walking_mode is switcher
    def unexpected_switch(_model):
        raise AssertionError("a removed tool must not switch the robot")

    monkeypatch.setattr(switcher, "_switch", unexpected_switch)
    assert [tool["name"] for tool in bundle.get_all_tools()] == ["loco"]
    result = bundle.dispatch("walking_mode", {"walking_model": model})
    assert result is None
    bundle.stop_all()


@pytest.mark.parametrize(
    ("enabled", "high_ctrl", "loco"), [
        (False, FakeHighCtrl(), True), (True, None, True), (True, FakeHighCtrl(), False)])
def test_walking_mode_skipped_when_disabled_or_dependencies_missing(
        bundle_module, enabled, high_ctrl, loco):
    bundle = bundle_module.BumiDeviceBundle(
        {"plugins": {"loco": {"enabled": loco}, "walking_mode": {"enabled": enabled}}},
        "bumi_test", object(), high_ctrl, None)
    assert "walking_mode" not in [tool["name"] for tool in bundle.get_all_tools()]
    assert bundle._walking_mode is None
    assert bundle.dispatch("walking_mode", {"walking_model": "whole_body"}) is None


@pytest.fixture
def locomotion_bundle(locomotion_bundle_module):
    return locomotion_bundle_module.BumiDeviceBundle(
        {"plugins": {"loco": {"enabled": True},
                     "walking_mode": {"enabled": True, "app_host": "127.0.0.1"}}},
        "bumi_test", None, FakeHighCtrl(), None)


def test_embedded_schema_preserves_original_action_fields_and_card_configuration(locomotion_bundle):
    loco, = locomotion_bundle._plugins
    tools = {tool["name"]: tool for tool in loco.get_tools()}
    for original in (loco._loco_tool(), loco._loco_relative_move_tool()):
        tool = tools[original["name"]]
        old, new = original["inputSchema"], tool["inputSchema"]
        assert new["properties"]["action"]["enum"] == old["properties"]["action"]["enum"] + ["walking_mode"]
        for name, schema in old["properties"].items():
            if name != "action":
                assert new["properties"][name] == schema
        for action, fields in old["x-action-params"].items():
            assert new["x-action-params"][action] == fields
        assert tool.get("configSchema") == original.get("configSchema")
        assert new.get("x-hooks") == old.get("x-hooks")
    assert tools == {tool["name"]: tool for tool in loco.get_tools()}


def test_original_actions_keep_their_dispatch_paths_with_mode_switch_enabled(locomotion_bundle, monkeypatch):
    bundle = locomotion_bundle
    loco, = bundle._plugins

    def unexpected_switch(_model):
        raise AssertionError("existing actions must not invoke mode selection")

    monkeypatch.setattr(bundle._walking_mode, "_switch", unexpected_switch)
    monkeypatch.setattr(loco, "_preempt_servo", lambda _reason: False)
    monkeypatch.setattr(loco, "_do_move", lambda args: {"velocity": args["vx"]})
    monkeypatch.setattr(loco, "_do_rotate", lambda args: {"angle": args["angle_deg"]})
    monkeypatch.setattr(loco, "_do_destination", lambda args, action: {"point": [args["x_m"], args["y_m"]]})
    assert bundle.dispatch("loco", {
        "action": "move", "vx": 0.2, "walking_model": "whole_body"}) == {"velocity": 0.2}
    assert bundle.dispatch("loco_relative_move", {
        "action": "rotate", "angle_deg": 30, "walking_model": "whole_body"}) == {"angle": 30}
    assert bundle.dispatch("loco_relative_move", {"action": "move", "x_m": 0.2, "y_m": 0.1}) == {
        "point": [0.2, 0.1]}
    assert bundle.dispatch("loco_relative_move", {"action": "status"})["state"] == "idle"
    assert bundle.dispatch("loco_relative_move", {"action": "config", "slip_forward": 2.0})["state"] == "configured"
    assert loco._slip_scales["forward"] == 2.0


@pytest.mark.parametrize("tool_name", ["loco", "loco_relative_move"])
def test_embedded_switch_owns_chassis_and_either_card_can_cancel(locomotion_bundle, monkeypatch, tool_name):
    bundle = locomotion_bundle
    loco, = bundle._plugins
    switcher = bundle._walking_mode
    entered, release = threading.Event(), threading.Event()
    callbacks, brakes = [], []
    original = switcher._switch

    def delayed_switch(model):
        entered.set()
        assert release.wait(timeout=2)
        return original(model)

    monkeypatch.setattr(switcher, "_switch", delayed_switch)
    monkeypatch.setattr(sys.modules["device"], "_acp_notify", lambda *args: callbacks.append(args))
    monkeypatch.setattr(loco._high_ctrl, "publish_cmd", lambda *args: brakes.append(args))
    results = []
    request = threading.Thread(target=lambda: results.append(bundle.dispatch(
        "loco", {"action": "walking_mode", "walking_model": "whole_body"})))
    request.start()
    try:
        assert entered.wait(timeout=2)
        assert loco.is_moving() is True
        for name, action in (("loco", "move"), ("loco_relative_move", "rotate"),
                             ("loco_relative_move", "move"), ("stand_up_lie_prone", "stand_up")):
            refused = bundle.dispatch(name, {"action": action})
            assert refused["state"] == "error"
        refused = bundle.dispatch("loco_relative_move", {
            "action": "walking_mode", "walking_model": "lower_body"})
        assert refused["code"] == "busy"
        assert "action_id" not in refused
        assert bundle.dispatch(tool_name, {"action": "stop_move"}) == {"state": "stopped"}
        assert brakes == [(0, 0, 0, "DEFAULT", 0)]
    finally:
        release.set()
        request.join(timeout=2)
    assert not request.is_alive()
    assert results[0]["code"] == "cancelled"
    assert "action_id" not in results[0]
    assert callbacks == []
    assert loco.is_moving() is False
    assert loco._rotate_state == {"state": "idle", "last_result": None}


@pytest.mark.parametrize("tool_name", ["loco", "loco_relative_move"])
@pytest.mark.parametrize("phase", ["preflight", "tcp_write"])
def test_stop_move_prevents_late_mode_writes(locomotion_bundle, monkeypatch, tool_name, phase):
    import walking_mode

    bundle = locomotion_bundle
    loco, = bundle._plugins
    blocked, release = threading.Event(), threading.Event()
    stopping, stopped = threading.Event(), threading.Event()
    events, results, stop_results = [], [], []
    state = {"mode": 2}

    def joint_state():
        if phase == "preflight":
            blocked.set()
            assert release.wait(timeout=3)
        return [types.SimpleNamespace(vel=0.0) for _ in range(21)]

    class Client:
        def __init__(self, *_args, **_kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def get_status(self):
            if "tcp_write" in events:
                assert stopped.wait(timeout=3)
            return state["mode"]

        def set_walking_model(self, model):
            assert model == "whole_body"
            blocked.set()
            assert release.wait(timeout=3)
            events.append("tcp_write")
            state["mode"] = 3

    def brake(*args):
        assert args == (0, 0, 0, "DEFAULT", 0)
        events.append("brake")

    def stop():
        stopping.set()
        stop_results.append(bundle.dispatch(tool_name, {"action": "stop_move"}))
        events.append("stop_done")
        stopped.set()

    monkeypatch.setattr(loco._high_ctrl, "get_mode", lambda: state["mode"], raising=False)
    monkeypatch.setattr(loco._high_ctrl, "get_joint_state", joint_state, raising=False)
    monkeypatch.setattr(loco._high_ctrl, "publish_cmd", brake)
    monkeypatch.setattr(walking_mode, "WalkingModeClient", Client)
    request = threading.Thread(target=lambda: results.append(bundle.dispatch(
        "loco", {"action": "walking_mode", "walking_model": "whole_body"})))
    stopper = threading.Thread(target=stop)
    request.start()
    try:
        assert blocked.wait(timeout=3)
        stopper.start()
        assert stopping.wait(timeout=3)
        if phase == "tcp_write":
            assert not stopped.wait(timeout=0.05)
            assert events == []
        else:
            assert stopped.wait(timeout=3)
            assert events == ["brake", "stop_done"]
    finally:
        release.set()
        if stopper.ident is not None:
            stopper.join(timeout=3)
        request.join(timeout=3)
    assert not request.is_alive()
    assert not stopper.is_alive()
    assert stop_results == [{"state": "stopped"}]
    assert results[0]["code"] == "cancelled"
    assert results[0]["command_sent"] is (phase == "tcp_write")
    assert events == (["tcp_write", "brake", "stop_done"] if phase == "tcp_write"
                      else ["brake", "stop_done"])
    assert loco.is_moving() is False


@pytest.mark.parametrize("tool_name", ["loco", "loco_relative_move"])
def test_mode_acceptance_waits_for_the_entire_brake(locomotion_bundle, monkeypatch, tool_name):
    bundle = locomotion_bundle
    loco, = bundle._plugins
    braking, release, requesting, switched = (threading.Event() for _ in range(4))
    events, results = [], []

    def brake(*_args):
        events.append("brake_start")
        braking.set()
        assert release.wait(timeout=3)
        events.append("brake_end")

    def switch(model):
        events.append("mode_write")
        switched.set()
        return {"state": "completed", "walking_model": model}

    def request_switch():
        requesting.set()
        results.append(bundle.dispatch("loco", {
            "action": "walking_mode", "walking_model": "whole_body"}))

    monkeypatch.setattr(loco._high_ctrl, "publish_cmd", brake)
    monkeypatch.setattr(bundle._walking_mode, "_switch", switch)
    stopper = threading.Thread(target=lambda: bundle.dispatch(tool_name, {"action": "stop_move"}))
    request = threading.Thread(target=request_switch)
    stopper.start()
    try:
        assert braking.wait(timeout=3)
        request.start()
        assert requesting.wait(timeout=3)
        assert not switched.wait(timeout=0.05)
    finally:
        release.set()
        stopper.join(timeout=3)
        if request.ident is not None:
            request.join(timeout=3)
    assert not stopper.is_alive()
    assert not request.is_alive()
    assert results[0]["state"] == "completed"
    assert events == ["brake_start", "brake_end", "mode_write"]


@pytest.mark.parametrize("tool_name", ["loco", "loco_relative_move"])
@pytest.mark.parametrize("stop_action", ["stop", "stop_move", "shutdown"])
def test_stop_allows_a_later_switch_from_either_card(
        locomotion_bundle, monkeypatch, tool_name, stop_action):
    bundle = locomotion_bundle
    loco, = bundle._plugins
    entered, release = threading.Event(), threading.Event()
    original = bundle._walking_mode._switch
    results = []

    def switch(model):
        entered.set()
        assert release.wait(timeout=2)
        return original(model)

    monkeypatch.setattr(bundle._walking_mode, "_switch", switch)
    monkeypatch.setattr(loco._high_ctrl, "publish_cmd", lambda *_args: None)
    request = threading.Thread(target=lambda: results.append(bundle.dispatch(
        tool_name, {"action": "walking_mode", "walking_model": "whole_body"})))
    request.start()
    try:
        assert entered.wait(timeout=2)
        if stop_action == "shutdown":
            bundle.stop_all()
        else:
            assert bundle.dispatch(tool_name, {"action": stop_action})["state"] in ("idle", "stopped")
    finally:
        release.set()
        request.join(timeout=2)
    assert not request.is_alive()
    assert results[0]["code"] == "cancelled"
    calls = []

    def complete(model):
        assert not bundle._walking_mode._cancel.is_set()
        calls.append(model)
        return {"state": "completed", "walking_model": model}

    monkeypatch.setattr(bundle._walking_mode, "_switch", complete)
    for name in ("loco", "loco_relative_move"):
        result = bundle.dispatch(name, {"action": "walking_mode", "walking_model": "lower_body"})
        assert result["state"] == "completed"
        assert result["walking_model"] == "lower_body"
    assert calls == ["lower_body", "lower_body"]


def test_slow_command_write_bounds_the_real_stop_move_wait(locomotion_bundle, monkeypatch):
    import walking_mode_client

    bundle = locomotion_bundle
    loco, = bundle._plugins
    bundle._walking_mode._query_timeout = 15.0
    writing, finished = threading.Event(), threading.Event()
    events, results = [], []

    class Socket:
        incoming = walking_mode_client.encode_packet({"cmd": "status", "data": {"status": 2}})

        def settimeout(self, timeout):
            self.timeout = timeout

        def sendall(self, data):
            if json.loads(data[16:-4])["cmd"] == "run":
                assert self.timeout <= 0.1
                writing.set()
                finished.wait(timeout=self.timeout)
                events.append("write_timeout")
                raise TimeoutError("command write timed out")

        def recv(self, size):
            data, self.incoming = self.incoming[:size], self.incoming[size:]
            return data

        def close(self):
            pass

    monkeypatch.setattr(walking_mode_client.socket, "create_connection", lambda *_args: Socket())
    monkeypatch.setattr(loco._high_ctrl, "get_mode", lambda: 2, raising=False)
    monkeypatch.setattr(loco._high_ctrl, "get_joint_state", lambda: [
        types.SimpleNamespace(vel=0.0) for _ in range(21)], raising=False)
    monkeypatch.setattr(loco._high_ctrl, "publish_cmd", lambda *_args: events.append("brake"))
    request = threading.Thread(target=lambda: results.append(bundle.dispatch(
        "loco", {"action": "walking_mode", "walking_model": "whole_body"})))

    def stop():
        bundle.dispatch("loco_relative_move", {"action": "stop_move"})
        finished.set()

    stopper = threading.Thread(target=stop)
    request.start()
    try:
        assert writing.wait(timeout=2)
        stopper.start()
        assert finished.wait(timeout=0.5)
    finally:
        finished.set()
        request.join(timeout=2)
        if stopper.ident is not None:
            stopper.join(timeout=2)
    assert not request.is_alive()
    assert not stopper.is_alive()
    assert events == ["write_timeout", "brake"]
    assert results[0]["command_may_have_been_sent"] is True
    assert results[0]["mode_verified"] is False
    assert not loco.is_moving()


def test_mcp_discovers_both_actions_and_executes_each_mode(locomotion_bundle_module, monkeypatch):
    import walking_mode

    class Controller:
        mode = 2

        def get_mode(self):
            return self.mode

        def get_joint_state(self):
            return [types.SimpleNamespace(vel=0.0) for _ in range(21)]

        def publish_cmd(self, x, y, z, command, index=0):
            sdk_commands.append((x, y, z, command, index))

    controller = Controller()
    commands, callbacks = [], []
    sdk_commands = []

    class Client:
        def __init__(self, host, **kwargs):
            assert host == "127.0.0.1"

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def get_status(self):
            return controller.mode

        def set_walking_model(self, model):
            commands.append(model)
            controller.mode = walking_mode.MODEL_CODES[model]

    monkeypatch.setattr(walking_mode, "WalkingModeClient", Client)
    monkeypatch.setattr(sys.modules["device"], "_acp_notify", lambda *args: callbacks.append(args))
    bundle = locomotion_bundle_module.BumiDeviceBundle(
        {"plugins": {"loco": {"enabled": True},
                     "walking_mode": {"enabled": True, "app_host": "127.0.0.1"}}},
        "bumi_test", None, controller, None)
    monkeypatch.setattr(bundle._walking_mode._cancel, "wait", lambda _timeout: False)
    monkeypatch.setattr(locomotion_bundle_module, "_bundle", bundle)
    server = ThreadingHTTPServer(("127.0.0.1", 0), locomotion_bundle_module.make_handler())
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    def rpc(method, params=None):
        request = urllib.request.Request(
            f"http://127.0.0.1:{server.server_port}/mcp",
            data=json.dumps({"jsonrpc": "2.0", "id": 1, "method": method,
                             "params": params or {}}).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.load(response)["result"]

    try:
        tools = {tool["name"]: tool for tool in rpc("tools/list")["tools"]}
        assert "walking_mode" not in tools
        for tool_name, model in (("loco", "whole_body"), ("loco_relative_move", "lower_body"),
                                 ("loco_relative_move", "whole_body"), ("loco", "lower_body")):
            assert tools[tool_name]["inputSchema"]["x-action-params"]["walking_mode"]["params"] == ["walking_model"]
            response = rpc("tools/call", {"name": tool_name, "arguments": {
                "action": "walking_mode", "walking_model": model,
                "vx": 0.8, "vy": -0.5, "vyaw": 0.4, "duration": 5,
                "angle_deg": 90, "x_m": 0.5, "y_m": 0.2}})
            completed = json.loads(response["content"][0]["text"])
            assert completed["state"] == "completed"
            assert completed["action"] == "walking_mode"
            assert "action_id" not in completed
            assert completed["mode_verified"] is True
            assert completed["walking_model"] == model
            assert bundle._plugins[0]._move_thread is None
            assert bundle._plugins[0]._rotate_worker is None
            assert callbacks == []
        assert commands == ["whole_body", "lower_body", "whole_body", "lower_body"]
        refused = rpc("tools/call", {"name": "loco", "arguments": {"action": "walking_mode"}})
        assert json.loads(refused["content"][0]["text"])["code"] == "invalid_walking_model"
        assert callbacks == []
        for tool_name, model in (("loco", "whole_body"), ("loco_relative_move", "whole_body"),
                                 ("loco", "lower_body"), ("loco_relative_move", "lower_body")):
            before = len(sdk_commands)
            response = rpc("tools/call", {"name": tool_name, "arguments": {
                "action": "walking_mode", "walking_model": model}})
            assert json.loads(response["content"][0]["text"])["state"] == "completed"
            assert len(sdk_commands) == before
            mode_commands = list(commands)
            response = rpc("tools/call", {"name": "loco", "arguments": {
                "action": "move", "vx": 0.6, "duration": 0.04}})
            assert json.loads(response["content"][0]["text"])["state"] == "moving"
            bundle._plugins[0]._move_thread.join(timeout=2)
            assert not bundle._plugins[0].is_moving()
            assert sdk_commands[before] == (0.6, 0, 0, "DEFAULT", 0)
            assert sdk_commands[-1] == (0, 0, 0, "DEFAULT", 0)
            assert controller.mode == walking_mode.MODEL_CODES[model]
            assert commands == mode_commands
            assert callbacks == []
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=2)

