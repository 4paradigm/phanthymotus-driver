"""Offline face_light contract, timing, cancellation and transport checks; no robot IO."""
import sys
import json
import os
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

GO1 = Path(__file__).resolve().parents[1] / "unitree" / "go1"
sys.path.insert(0, str(GO1))
import ext_devices as ext

_REAL_ACP_NOTIFY = ext._face_acp_notify


@pytest.fixture(autouse=True)
def acp_notify(monkeypatch):
    # Offline checks never contact Agent Core or a robot.
    notify = Mock()
    monkeypatch.setattr(ext, "_face_acp_notify", notify)
    return notify


class FakeFaceBackend:
    """Test-only recorder; never contacts a robot."""
    name = "simulated"
    per_led = True

    def __init__(self):
        from collections import deque
        self.frames = deque(maxlen=256)
        self.connected = False

    def start(self):
        self.connected = True

    def write(self, frame):
        if not self.connected:
            raise RuntimeError("simulation backend is stopped")
        self.frames.append((time.monotonic(), frame))

    def close(self):
        self.connected = False


@pytest.fixture
def light():
    plugin = ext.make_face_light({}, "test", None, None)
    plugin._backend = FakeFaceBackend()
    assert plugin.dispatch("start", {})["ok"]
    yield plugin
    plugin.stop()


def wait_until(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "condition timed out"
        time.sleep(0.005)


@pytest.mark.parametrize("action", ext._FACE_EFFECTS)
def test_effect_completion_contract_and_unique_ids(light, acp_notify, action):
    completion = light.get_tool()["inputSchema"]["x-completion"]
    assert set(completion["actions"]) == set(ext._FACE_EFFECTS)
    assert completion["timeout"] > 3600
    assert light.PREFIX == light.get_tool()["name"] == "face_light"
    assert light.get_tool()["inputSchema"]["x-resource"] == "face_light"
    ids = []
    for _ in range(2):
        response = light.dispatch(action, {"r": 100, "duration_s": 0.05})
        assert response["ok"] and response["action_id"]
        ids.append(response["action_id"])
        light._thread.join(1)
        assert not light._thread.is_alive()
    assert len(set(ids)) == 2
    wait_until(lambda: acp_notify.call_count == 2)
    assert acp_notify.call_count == 2
    for call, action_id in zip(acp_notify.call_args_list, ids):
        sent_id, status, result = call.args
        assert sent_id == action_id and status == "completed"
        assert result["ok"] and result["mode"] == "off"
        assert result["state_source"] == "software_record" and not result["hardware_verified"]
    assert "action_id" not in light.dispatch("set_color", {"r": 1})


def test_all_go1_acting_tools_declare_consistent_resources_without_hardware_calls():
    import main
    channels = {
        'face_light': 'face_light', 'beep': 'mouth', 'speaker': 'mouth',
        'loco': 'base', 'body_pose': 'base', 'switch_gait': 'base',
        'gesture': 'base', 'special_motion': 'base',
        'system_health': 'base', 'activity_monitor': 'base',
    }
    config = {'plugins': {name: {'enabled': True} for name in channels}}
    client = Mock()
    # Only construct tools and inspect declarations. Do not start any card.
    bundle = main.Go1Bundle(config, 'offline', None, client)
    tools = {tool['name']: tool for tool in bundle.get_all_tools()}
    assert set(tools) == set(channels)
    for name, channel in channels.items():
        assert tools[name]['type'] == 'actuator'  # preserve existing access policy
        assert tools[name]['inputSchema']['x-resource'] == channel
        if name != 'face_light':
            assert channel != tools['face_light']['inputSchema']['x-resource']
    # Shared speaker and body hardware cannot be labelled as independent channels.
    assert tools['beep']['inputSchema']['x-resource'] == tools['speaker']['inputSchema']['x-resource']
    assert not client.mock_calls


@pytest.mark.parametrize("action,args", [("set_color", {"g": 10}), ("off", {}), ("stop", {}),
                                         ("blink", {"duration_s": 2}),
                                         ("config", {"sdk_exclusive": False})])
def test_effect_cancelled_completion(light, acp_notify, action, args):
    response = light.dispatch("chase", {"r": 100, "duration_s": 2})
    old = light._thread
    assert light.dispatch(action, args)["ok"]
    assert not old.is_alive()
    calls = [call for call in acp_notify.call_args_list if call.args[0] == response["action_id"]]
    wait_until(lambda: any(call.args[0] == response['action_id'] for call in acp_notify.call_args_list))
    calls = [call for call in acp_notify.call_args_list if call.args[0] == response["action_id"]]
    assert len(calls) == 1 and calls[0].args[1] == "cancelled"
    assert calls[0].args[2]["code"] == "CANCELLED"


@pytest.mark.parametrize("final_frame", [False, True])
def test_effect_error_completion(light, acp_notify, final_frame):
    original = light._backend.write
    response = light.dispatch("blink", {"r": 100, "period_s": 1, "duration_s": 0.1})
    def fail(frame):
        if not final_frame or frame == ext._FACE_BLACK:
            raise RuntimeError("offline transport failure")
        original(frame)
    with light._lock:
        light._backend.write = fail
    light._thread.join(1)
    wait_until(lambda: acp_notify.call_count == 1)
    acp_notify.assert_called_once()
    action_id, status, result = acp_notify.call_args.args
    assert action_id == response["action_id"] and status == "error"
    assert not result["ok"] and result["message"] == "offline transport failure"
    assert light._info()["mode"] == "error"


def test_rejected_effect_has_no_pending_completion(light, acp_notify):
    response = light.dispatch("blink", {"duration_s": -1})
    assert not response["ok"] and "action_id" not in response
    light._backend.write = Mock(side_effect=RuntimeError("unavailable"))
    response = light.dispatch("blink", {})
    assert not response["ok"] and "action_id" not in response
    acp_notify.assert_not_called()


def test_acp_http_payload_and_cleanup(monkeypatch):
    monkeypatch.setenv("AGENT_CORE_URL", "https://localhost:15678/")
    response = Mock()
    response.__enter__ = Mock(return_value=response)
    response.__exit__ = Mock(return_value=False)
    urlopen = Mock(return_value=response)
    monkeypatch.setattr(ext.urllib.request, "urlopen", urlopen)
    _REAL_ACP_NOTIFY("face_light_test", "completed", {"hardware_verified": False})
    request = urlopen.call_args.args[0]
    assert request.full_url == "https://localhost:15678/api/acp/complete"
    assert request.get_method() == "POST"
    payload = json.loads(request.data)
    assert payload["action_id"] == "face_light_test" and payload["status"] == "completed"
    assert payload["tool"] == "face_light" and payload["result"] == {"hardware_verified": False}
    assert urlopen.call_args.kwargs["timeout"] == 3
    response.__exit__.assert_called_once()


def test_acp_failure_is_bounded_and_visible(monkeypatch, capsys):
    urlopen = Mock(side_effect=TimeoutError("offline callback timeout"))
    monkeypatch.setattr(ext.urllib.request, "urlopen", urlopen)
    _REAL_ACP_NOTIFY("face_light_failed", "error", {})
    assert urlopen.call_args.kwargs["timeout"] == 3
    output = capsys.readouterr().out
    assert "face_light_failed" in output and "offline callback timeout" in output


@pytest.mark.parametrize('action,args', [('off', {}), ('set_color', {'g': 70}), ('stop', {})])
def test_blocked_acp_cannot_delay_replacement_frame(light, monkeypatch, action, args):
    entered, release = threading.Event(), threading.Event()
    def blocked(*unused):
        entered.set()
        assert release.wait(2)
    monkeypatch.setattr(ext, '_face_acp_notify', blocked)
    light.dispatch('blink', {'r': 50, 'duration_s': 2})
    calls = []
    command = threading.Thread(target=lambda: calls.append(light.dispatch(action, args)))
    try:
        command.start()
        assert entered.wait(1)  # old effect's callback is stalled
        expected = ((0, 70, 0),) * 12 if action == 'set_color' else ext._FACE_BLACK
        wait_until(lambda: light._backend.frames[-1][1] == expected, timeout=0.5)
        count = len(light._backend.frames)
        time.sleep(0.06)
        assert len(light._backend.frames) == count
        if action != 'stop':
            command.join(0.5)
            assert not command.is_alive()  # off/static return despite blocked HTTP
        else:
            assert not light._backend.connected  # closed before waiting for reporter cleanup
    finally:
        release.set()
        command.join(1)
        light.stop()
    assert calls[0]['ok'] and not light._completion_threads


def test_completion_reporters_are_bounded_and_slots_reused(light, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    light._completion_slots = threading.BoundedSemaphore(1)
    def blocked(*unused):
        entered.set()
        assert release.wait(2)
    monkeypatch.setattr(ext, '_face_acp_notify', blocked)
    try:
        first = light.dispatch('blink', {'r': 20, 'duration_s': 0.05})
        assert first['ok'] and entered.wait(1)
        count = len(light._backend.frames)
        result = light.dispatch('chase', {'g': 40})
        assert result['code'] == 'RESOURCE_BUSY' and 'action_id' not in result
        assert len(light._backend.frames) == count and len(light._completion_threads) == 1
        assert light.dispatch('off', {})['ok']
    finally:
        release.set()
        light.stop()
    assert not light._completion_threads
    assert light.start()['ok']
    assert light.dispatch('chase', {'g': 40})['ok']


def test_legacy_static_and_info(light):
    result = light.dispatch("set_color", {"r": 12, "g": 34, "b": 56})
    assert result["applied"]["r"] == 12 and result["simulated"]
    count = len(light._backend.frames)
    time.sleep(0.08)
    assert len(light._backend.frames) == count  # persistent, not periodic / auto-off
    assert light.dispatch("preset", {"name": "BLUE"})["applied"]["rgb"] == [0, 0, 255]
    assert light.dispatch("preset", {})["applied"]["rgb"] == [0, 0, 0]
    assert light.dispatch("set_color", {})["ok"]  # legacy omitted channels default zero
    assert light.dispatch("off", {})["applied"]["rgb"] == [0, 0, 0]
    info = light.dispatch("info", {})
    assert info["mode"] == "off" and not info["running"]
    assert info["state_source"] == "software_record" and not info["hardware_verified"]
    assert not info["capabilities"]["official_sdk_integrated"]
    assert light.dispatch("unknown", {}) is None


@pytest.mark.parametrize("index", [-1, 12, 1.5, True, "2", None])
def test_bad_index(light, index):
    before = len(light._backend.frames)
    result = light.dispatch("set_led", {"index": index, "r": 5})
    assert result["code"] == "INVALID_ARGUMENT" and "index" in result["message"]
    assert len(light._backend.frames) == before


@pytest.mark.parametrize("bad", [-1, 256, 2.0, True, "1", None, float("nan")])
@pytest.mark.parametrize("action", ["set_color", "set_led", "blink"])
def test_bad_rgb(light, bad, action):
    result = light.dispatch(action, {"index": 0, "r": bad})
    assert result["code"] == "INVALID_ARGUMENT"
    assert "RGB" in result["message"]


@pytest.mark.parametrize("colors", [None, [], [[0, 0, 0]] * 11, [[0, 0, 0]] * 13,
                                     [[0, 0]] * 12, [[0, 0, True]] * 12,
                                     [[0, 0, 256]] * 12])
def test_bad_array(light, colors):
    assert light.dispatch("set_leds", {"colors": colors})["code"] == "INVALID_ARGUMENT"


def test_all_12_indices_and_retention(light):
    colors = [[i, i + 10, 255 - i] for i in range(12)]
    assert light.dispatch("set_leds", {"colors": colors})["ok"]
    assert light._backend.frames[-1][1] == tuple(tuple(c) for c in colors)
    for i in range(12):
        result = light.dispatch("set_led", {"index": i, "r": 100 + i})
        assert result["ok"]
        colors[i] = [100 + i, 0, 0]
        assert light.dispatch("info", {})["colors"] == colors
    mapping = light.dispatch("info", {})["led_map"]
    assert [m["index"] for m in mapping] == list(range(12))
    assert [m["viewer_side"] for m in mapping] == ["left"] * 6 + ["right"] * 6
    assert [m["row_from_top"] for m in mapping] == list(range(6)) * 2


@pytest.mark.parametrize("action", ext._FACE_EFFECTS)
def test_render_time_and_live_auto_off(light, action):
    rgb, target = (255, 80, 20), (0, 10, 240)
    first = ext._face_effect_frame(action, rgb, target, 1, 0)
    other = ext._face_effect_frame(action, rgb, target, 1, 0.6)
    assert first != other
    if action == "chase":
        for i in range(12):
            frame = ext._face_effect_frame(action, rgb, target, 12, i + 0.1)
            assert frame[i] == rgb and sum(c != (0, 0, 0) for c in frame) == 1
    assert light.dispatch(action, {"r": 255, "g": 80, "b": 20, "to_b": 240,
                                   "period_s": 0.2, "duration_s": 0.24})["ok"]
    worker = light._thread
    wait_until(lambda: not worker.is_alive())
    assert len(set(frame for _, frame in light._backend.frames)) > 1
    assert light._backend.frames[-1][1] == ext._FACE_BLACK
    count = len(light._backend.frames)
    time.sleep(0.06)
    assert len(light._backend.frames) == count
    assert light._info()["mode"] == "off"


@pytest.mark.parametrize("field,value", [("period_s", 0), ("period_s", 0.19), ("period_s", True),
                                         ("period_s", "1"), ("duration_s", 0), ("duration_s", -1),
                                         ("duration_s", float("inf")), ("duration_s", float("nan")),
                                         ("duration_s", 3601)])
def test_effect_parameter_validation(light, field, value):
    result = light.dispatch("breathe", {field: value})
    assert result["code"] == "INVALID_ARGUMENT" and field in result["message"]


@pytest.mark.parametrize("action,args", [("set_color", {"g": 77}), ("preset", {"name": "cyan"}),
                                         ("set_led", {"index": 5, "b": 90}),
                                         ("set_leds", {"colors": [[1, 2, 3]] * 12}), ("off", {})])
def test_static_preempts_and_never_resends(light, action, args):
    assert light.dispatch("blink", {"r": 255, "period_s": 0.2, "duration_s": 2})["ok"]
    old = light._thread
    assert light.dispatch(action, args)["ok"]
    assert not old.is_alive()
    count = len(light._backend.frames)
    time.sleep(0.12)
    assert len(light._backend.frames) == count


def test_effect_preemption_and_stop_cleanup(light):
    light.dispatch("chase", {"r": 60, "duration_s": 2})
    old = light._thread
    light.dispatch("breathe", {"g": 90, "duration_s": 2})
    assert not old.is_alive()
    worker = light._thread
    assert light.dispatch("stop", {})["ok"]
    assert not worker.is_alive() and light._thread is None
    assert light._backend.frames[-1][1] == ext._FACE_BLACK
    count = len(light._backend.frames)
    time.sleep(0.12)
    assert len(light._backend.frames) == count and not light._backend.connected
    assert light.dispatch("set_color", {"r": 99})["code"] == "NOT_AVAILABLE"
    assert light.stop()["ok"]  # idempotent
    assert light.start()["ok"]
    assert light.dispatch("set_color", {"r": 3})["ok"]


def test_stop_waits_for_in_flight_frame(light):
    entered, release = threading.Event(), threading.Event()
    original = light._backend.write
    block = False

    def write(frame):
        if block and frame != ext._FACE_BLACK:
            entered.set()
            assert release.wait(1)
        original(frame)

    light._backend.write = write
    light.dispatch("blink", {"r": 255, "period_s": 1, "duration_s": 2})
    block = True
    assert entered.wait(1)
    result = []
    stopper = threading.Thread(target=lambda: result.append(light.stop()))
    stopper.start()
    release.set()
    stopper.join(1)
    assert not stopper.is_alive() and result[0]["ok"]
    assert light._backend.frames[-1][1] == ext._FACE_BLACK
    count = len(light._backend.frames)
    time.sleep(0.06)
    assert count == len(light._backend.frames)


def test_background_connection_failure(light):
    light.dispatch("blink", {"r": 255, "duration_s": 1})
    worker = light._thread
    with light._lock:
        light._backend.connected = False
    wait_until(lambda: not worker.is_alive())
    info = light._info()
    assert info["mode"] == "error" and info["last_error"]
    assert not info["running"]
    assert not light.stop()["ok"]
    assert light._thread is None


def test_single_card_bundle_and_packaging():
    import importlib.util
    import yaml
    spec = importlib.util.spec_from_file_location("go1_face_main", GO1 / "main.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    bundle = module.Go1Bundle({"plugins": {"face_light": {"enabled": True}}}, "", None, None)
    bundle._plugins[0]._backend = FakeFaceBackend()
    bundle.start_all()
    try:
        tools = bundle.get_all_tools()
        assert len(tools) == 1 and tools[0]["name"] == "face_light"
        schema = tools[0]["inputSchema"]
        assert set(schema["properties"]["action"]["enum"]) == set(schema["x-action-params"])
        assert bundle.dispatch("face_light", {"action": "set_led", "index": 11, "r": 9})["ok"]
        assert bundle.dispatch("face_light", {"action": "info"})["simulated"]
        config = yaml.safe_load((GO1 / "config.yaml").read_text())
        assert config["plugins"]["face_light"]["enabled"] is True
        assert config["plugins"]["face_light"]["backend"] == "sdk"
        assert config["plugins"]["face_light"]["sdk_exclusive"] is True
        metadata = yaml.safe_load((GO1 / "driver.yaml").read_text())
        assert sum(item["name"] == "face_light" for item in metadata["cards"]) == 1
        assert "COPY ext_devices.py" in (GO1 / "Dockerfile").read_text()
    finally:
        bundle.stop_all()


def test_shipped_system_health_is_enabled_assembled_and_listed():
    import main
    import yaml
    config = yaml.safe_load((GO1 / 'config.yaml').read_text())
    plugins = config['plugins']
    assert plugins['system_health']['enabled'] is True
    assert 'system_health' not in plugins['face_light']
    assert 'mqtt_host' not in plugins['face_light']
    selected = {name: plugins[name] for name in ('face_light', 'system_health')}
    client = Mock()
    bundle = main.Go1Bundle({'plugins': selected}, 'offline', None, client)
    assert {tool['name'] for tool in bundle.get_all_tools()} == {'face_light', 'system_health'}
    metadata = yaml.safe_load((GO1 / 'driver.yaml').read_text())
    assert sum(card['name'] == 'system_health' for card in metadata['cards']) == 1
    assert not client.mock_calls  # declaration check only; no lifecycle/hardware calls


def test_default_face_card_stays_discoverable_when_sdk_setup_is_missing(monkeypatch):
    import main
    import yaml
    config = yaml.safe_load((GO1 / "config.yaml").read_text())
    bundle = main.Go1Bundle({"plugins": {"face_light": config["plugins"]["face_light"]}},
                            "offline", None, None)
    tool = bundle.get_all_tools()[0]
    assert tool["name"] == "face_light"
    assert "backend" not in tool["configSchema"]["properties"]
    plugin = bundle._plugins[0]
    error = "ERROR official faceLight SDK headers are missing; mount the trusted official SDK"
    monkeypatch.setattr(plugin._backend, "start", Mock(side_effect=RuntimeError(error)))
    try:
        bundle.start_all()
        assert [t["name"] for t in bundle.get_all_tools()] == ["face_light"]
        info = bundle.dispatch("face_light", {"action": "info"})
        assert info["ok"] and info["state"] == "idle" and not info["available"]
        assert info["availability_source"] == "software_sdk_process"
        assert info["unavailable_reason"] == info["last_error"] == error
        assert not info["hardware_verified"] and info["colors"] is None
        assert not bundle.dispatch("face_light", {"action": "set_color", "r": 100})["ok"]
        assert plugin._backend.process is None
    finally:
        bundle.stop_all()


def test_info_availability_tracks_process_readiness_and_stop(sdk_helper):
    helper, _ = sdk_helper
    plugin = sdk_test_plugin(helper)
    assert not plugin._info()["available"]
    try:
        assert plugin.start()["ok"]
        info = plugin._info()
        assert info["available"] and info["state"] == "ready"
        assert info["unavailable_reason"] is None and not info["hardware_verified"]
        # A crashed helper is unavailable even before the next write discovers it.
        process = plugin._backend.process
        process.terminate()
        process.wait(timeout=1)
        info = plugin._info()
        assert not info["available"] and info["state"] == "idle"
        assert info["unavailable_reason"] and not info["connected"]
    finally:
        plugin.stop()
    assert not plugin._info()["available"]


def test_concurrent_start_stop_no_resurrection():
    entered, release = threading.Event(), threading.Event()

    def connect(*args):
        entered.set()
        assert release.wait(1)

    plugin = ext.FaceLightPlugin({}, "", None, None)
    plugin._backend = FakeFaceBackend()
    plugin._backend.start = connect
    starter = threading.Thread(target=plugin.start)
    stopper = threading.Thread(target=plugin.stop)
    starter.start()
    assert entered.wait(1)
    stopper.start()
    release.set()
    starter.join(1)
    stopper.join(1)
    assert not starter.is_alive() and not stopper.is_alive()
    assert not plugin._active and not plugin._backend.connected
    assert plugin._info()["mode"] == "stopped"
    assert plugin.dispatch("set_color", {"r": 1})["code"] == "NOT_AVAILABLE"


def test_concurrent_commands_leave_only_one_worker(light):
    barrier = threading.Barrier(5)
    results = []

    def command(action):
        barrier.wait()
        results.append(light.dispatch(action, {"r": 70, "to_g": 90, "duration_s": 2}))

    threads = [threading.Thread(target=command, args=(a,)) for a in ext._FACE_EFFECTS]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join(1)
        assert not thread.is_alive()
    assert all(r["ok"] for r in results)
    assert sum(t.name == "go1-face-light" and t.is_alive() for t in threading.enumerate()) == 1
    light.stop()
    assert not any(t.name == "go1-face-light" and t.is_alive() for t in threading.enumerate())


def test_default_sdk_exclusive_yes_and_explicit_no_guard_do_not_use_mqtt(monkeypatch):
    mqtt_client = Mock()
    monkeypatch.setattr(ext, "mqtt", mqtt_client, raising=False)
    popen = Mock()
    monkeypatch.setattr(ext.subprocess, "Popen", popen)
    plugin = ext.FaceLightPlugin({}, "", None, None)
    assert plugin._backend.name == "sdk"
    schema = plugin.get_tool()["configSchema"]["properties"]
    assert "backend" not in schema
    assert plugin._config["backend"] == "sdk"
    assert schema["sdk_exclusive"]["default"] is True
    assert plugin._backend.exclusive is True
    assert "mqtt_host" not in schema and "mqtt_port" not in schema
    plugin = ext.FaceLightPlugin({"sdk_exclusive": False}, "", None, None)
    result = plugin.start()
    assert not result["ok"] and "sdk_exclusive" in result["message"]
    assert not plugin._active
    popen.assert_not_called()
    assert not mqtt_client.mock_calls


def test_invalid_command_leaves_current_effect(light):
    light.dispatch("blink", {"r": 255, "duration_s": 2})
    worker = light._thread
    assert light.dispatch("set_led", {"index": 12})["code"] == "INVALID_ARGUMENT"
    assert light.dispatch("preset", {"name": "unknown"})["code"] == "INVALID_ARGUMENT"
    assert light._thread is worker and worker.is_alive()


@pytest.fixture
def sdk_helper(tmp_path):
    helper = tmp_path / "sdk_helper"
    log = tmp_path / "frames.txt"
    helper.write_text(f'''#!{sys.executable}
import sys
print("READY face-light-v1", flush=True)
for line in sys.stdin:
    with open({str(log)!r}, "a") as output:
        output.write(line)
    print("SENT", flush=True)
''')
    helper.chmod(0o755)
    return helper, log


def sdk_test_plugin(executable, exclusive=True, sdk_dir=None):
    # Test-only transport injection; production card config cannot select a process.
    plugin = ext.FaceLightPlugin({"backend": "sdk", "sdk_exclusive": exclusive}, "", None, None)
    plugin._backend = ext._FaceSdkBackend(str(executable), exclusive, sdk_dir)
    return plugin


def test_native_harness_uses_test_only_backend_injection(sdk_helper):
    import importlib.util
    path = Path(__file__).parent / 'face_light_native_check.py'
    spec = importlib.util.spec_from_file_location('native_face_harness', path)
    native = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(native)  # importing must not compile, send or inspect host networking
    helper, _ = sdk_helper
    plugin = native.sdk_test_plugin(helper, '/test-only-sdk')
    assert plugin._config['sdk_executable'] == ext._FACE_SDK_EXECUTABLE
    assert plugin._config['sdk_dir'] == ext._FACE_SDK_DIR
    assert plugin._backend.executable == str(helper)
    try:
        assert plugin.start()['ok']
        assert plugin.dispatch('set_color', {'r': 7})['ok']
    finally:
        plugin.stop()


def test_sdk_checksum_failure_prevents_compilation(tmp_path):
    sdk = tmp_path / 'sdk'
    (sdk / 'include').mkdir(parents=True)
    (sdk / 'lib').mkdir()
    for name in ('include/FaceLightClient.h', 'include/LEDPixel.h', 'version.txt',
                 'lib/libfaceLight_SDK_arm64.so'):
        (sdk / name).write_text('untrusted vendor input')
    commands = tmp_path / 'bin'
    commands.mkdir()
    (commands / 'uname').write_text('#!/bin/sh\necho aarch64\n')
    # Portable implementation of the Linux checksum command, using real hashes.
    (commands / 'sha256sum').write_text(f'''#!{sys.executable}
import hashlib, pathlib, sys
valid = True
for line in sys.stdin:
    expected, name = line.strip().split(maxsplit=1)
    valid = valid and hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest() == expected
sys.exit(0 if valid else 1)
''')
    (commands / 'cmake').write_text('#!/bin/sh\necho MUST_NOT_COMPILE\nexit 99\n')
    for command in commands.iterdir():
        command.chmod(0o755)
    env = dict(os.environ, FACE_LIGHT_SDK_DIR=str(sdk), PATH=str(commands) + os.pathsep + os.environ['PATH'])
    result = subprocess.run(['sh', str(GO1 / 'deploy/face_light/run_sdk.sh')],
                            env=env, capture_output=True, text=True, timeout=2)
    assert result.returncode == 1 and 'checksum verification failed' in result.stdout
    assert 'MUST_NOT_COMPILE' not in result.stdout + result.stderr


@pytest.mark.parametrize("action,args", [("stop", {}), ("off", {}), ("set_color", {"g": 20})])
def test_sdk_full_stdin_pipe_has_bounded_preemption_and_cleanup(tmp_path, action, args):
    helper = tmp_path / "no_reader"
    # READY plus one acknowledgement lets the initial effect frame through. The
    # helper never reads stdin, so the next frame must hit a full kernel pipe.
    helper.write_text(f'''#!{sys.executable}
import time
print("READY face-light-v1", flush=True)
print("SENT", flush=True)
time.sleep(30)
''')
    helper.chmod(0o755)
    plugin = sdk_test_plugin(helper)
    assert plugin.start()["ok"]
    backend, process = plugin._backend, plugin._backend.process
    write_entered = threading.Event()
    original_write_request = backend._write_request
    errors = []

    def track_write(payload, deadline):
        write_entered.set()
        try:
            return original_write_request(payload, deadline)
        except RuntimeError as exc:
            errors.append(str(exc))
            raise

    try:
        # Holding the state lock prevents the worker writing before the pipe is full.
        with plugin._lock:
            assert plugin.dispatch("blink", {"r": 100, "duration_s": 10})["ok"]
            worker = plugin._thread
            fd = process.stdin.fileno()
            assert not os.get_blocking(fd)
            filled = 0
            while True:
                try:
                    filled += os.write(fd, b"X" * 4096)
                except BlockingIOError:
                    break
            assert filled > 0
            backend._write_request = track_write
        assert write_entered.wait(1), "worker did not enter the blocked write"
        started = time.monotonic()
        result = plugin.dispatch(action, args)
        assert time.monotonic() - started < backend.IO_TIMEOUT + 1.0
        assert not result["ok"] and result["code"] == "NOT_AVAILABLE"
        assert any("request write timed out" in error for error in errors)
        assert process.poll() is not None and backend.process is None
        assert not worker.is_alive() and plugin._thread is None
        assert process.stdin.closed and process.stdout.closed
    finally:
        plugin.stop()


def test_sdk_partial_and_interrupted_writes_preserve_one_frame(sdk_helper, monkeypatch):
    helper, log = sdk_helper
    plugin = sdk_test_plugin(helper)
    assert plugin.start()["ok"]
    write = os.write
    attempts = []
    fd = plugin._backend.process.stdin.fileno()

    def partial_write(descriptor, data):
        if descriptor != fd:
            return write(descriptor, data)
        attempts.append(len(data))
        if len(attempts) == 1:
            raise InterruptedError()
        if len(attempts) == 2:
            raise BlockingIOError()
        return write(descriptor, data[:3])

    try:
        monkeypatch.setattr(ext.os, "write", partial_write)
        assert plugin.dispatch("set_color", {"r": 12, "g": 34, "b": 56})["ok"]
        assert len(attempts) > 3
        assert log.read_text().splitlines() == [" ".join(["12", "34", "56"] * 12)]
    finally:
        plugin.stop()


def test_sdk_backend_complete_card_path(sdk_helper):
    helper, log = sdk_helper
    plugin = sdk_test_plugin(helper)
    start = plugin.start()
    assert start["ok"], start
    process = plugin._backend.process
    try:
        assert plugin.dispatch("set_color", {"r": 12, "g": 34, "b": 56})["applied"]["r"] == 12
        assert log.read_text().splitlines()[-1].split() == ['12', '34', '56'] * 12
        assert plugin.dispatch("preset", {"name": "BLUE"})["applied"]["rgb"] == [0, 0, 255]
        assert plugin.dispatch("set_color", {})["ok"]
        assert plugin.dispatch("off", {})["applied"]["rgb"] == [0, 0, 0]
        colors = [[i, 10, 255 - i] for i in range(12)]
        assert plugin.dispatch("set_leds", {"colors": colors})["delivery"] == "sdk_udp_socket_sent"
        assert [int(v) for v in log.read_text().splitlines()[-1].split()] == [v for c in colors for v in c]
        assert plugin.dispatch("set_led", {"index": 11, "r": 255})["ok"]
        for action in ext._FACE_EFFECTS:
            assert plugin.dispatch(action, {"g": 20, "duration_s": 0.1, "period_s": 0.2})["ok"]
            wait_until(lambda: not plugin._info()["running"])
        assert plugin._info()["capabilities"]["official_sdk_integrated"]
        assert not plugin._info()["hardware_verified"]
    finally:
        assert plugin.stop()["ok"]
    assert process.poll() is not None and plugin._backend.process is None
    assert [int(v) for v in log.read_text().splitlines()[-1].split()] == [0] * 36


def test_old_mqtt_config_is_rejected_without_effect_or_transport_change(light, monkeypatch):
    light.dispatch('blink', {'r': 30, 'duration_s': 2})
    worker, backend = light._thread, light._backend
    popen = Mock()
    monkeypatch.setattr(ext.subprocess, 'Popen', popen)
    for action in ('config', 'start'):
        result = light.dispatch(action, {'backend': 'mqtt'})
        assert not result['ok'] and result['code'] == 'INVALID_ARGUMENT'
        assert 'official SDK' in result['message']
        assert light._thread is worker and worker.is_alive() and light._backend is backend
    popen.assert_not_called()


def test_sdk_exclusive_guard_and_missing_executable(sdk_helper):
    helper, _ = sdk_helper
    plugin = sdk_test_plugin(helper, exclusive=False)
    result = plugin.start()
    assert not result["ok"] and "sdk_exclusive" in result["message"]
    assert plugin._backend.process is None
    plugin = sdk_test_plugin("/missing/face-light-helper")
    assert not plugin.start()["ok"]
    assert not plugin._active and plugin._backend.process is None


@pytest.mark.parametrize("behavior", ["error", "timeout", "eof", "protocol", "oversized"])
def test_sdk_protocol_failure_and_process_cleanup(tmp_path, behavior):
    helper = tmp_path / "helper"
    responses = {"error": 'print("ERROR SDK UDP send failed: Network is unreachable", flush=True)',
                 "timeout": 'time.sleep(10)', "eof": 'sys.exit(1)',
                 "protocol": 'print("WRONG", flush=True)',
                 "oversized": 'print("X" * 5000, flush=True)'}
    helper.write_text(f'''#!{sys.executable}
import sys, time
print("READY face-light-v1", flush=True)
for line in sys.stdin:
    {responses[behavior]}
''')
    helper.chmod(0o755)
    plugin = sdk_test_plugin(helper)
    start = plugin.start()
    assert start["ok"], start
    process = plugin._backend.process
    result = plugin.dispatch("set_color", {"r": 1})
    assert not result["ok"] and result["code"] == "NOT_AVAILABLE"
    assert process.poll() is not None and plugin._backend.process is None
    assert plugin._info()["colors"] is None
    plugin.stop()


def test_sdk_bad_handshake(tmp_path):
    helper = tmp_path / "helper"
    helper.write_text(f'#!{sys.executable}\nprint("wrong protocol", flush=True)\n')
    helper.chmod(0o755)
    plugin = sdk_test_plugin(helper)
    assert not plugin.start()["ok"]
    assert plugin._backend.process is None


def test_canvas_config_replay_preserves_effect(light):
    light.dispatch('chase', {'g': 60, 'duration_s': 2})
    worker = light._thread
    assert light.dispatch('config', {'backend': 'sdk'})['changed'] is False
    assert light._thread is worker and worker.is_alive()


def test_canvas_backend_config_requires_restart(light, sdk_helper):
    helper, _ = sdk_helper
    light.dispatch('chase', {'g': 60, 'duration_s': 2})
    worker = light._thread
    result = light.dispatch('config', {'backend': 'sdk', 'sdk_exclusive': False})
    assert result['ok'] and result['needs_start']
    assert not worker.is_alive() and not light._active
    assert light._backend.executable == ext._FACE_SDK_EXECUTABLE
    assert light._backend.sdk_dir == ext._FACE_SDK_DIR
    light._backend = ext._FaceSdkBackend(str(helper), True)  # offline transport injection
    assert light.dispatch('start', {})['ok']
    assert light.dispatch('set_led', {'index': 7, 'r': 10})['ok']
    assert light._info()['backend'] == 'sdk'


@pytest.mark.parametrize('config', [{'backend': 'invalid'}, {'backend': 'mqtt'}, {'backend': 'simulated'}, {'sdk_exclusive': 1},
                                    {'sdk_exclusive': 'true'}, {'sdk_dir': ''}, {'sdk_executable': None}])
def test_bad_canvas_config_preserves_backend(light, config):
    backend = light._backend
    assert light.dispatch('config', config)['code'] == 'INVALID_ARGUMENT'
    assert light._backend is backend and light._active


@pytest.mark.parametrize('action', ['config', 'start'])
@pytest.mark.parametrize('override', [
    {'sdk_executable': '/bin/sh'}, {'sdk_executable': '/usr/bin/python3'},
    {'sdk_executable': '/deploy/face_light/../face_light/run_sdk.sh'},
    {'sdk_dir': '/tmp/untrusted-sdk'}, {'sdk_dir': '/opt/phanthy-motus/data/go1/faceLightSDK_Nano/../other'},
    {'sdk_dir': '/opt/phanthy-motus/data/go1/faceLightSDK_Nano/'},
    {'sdk_executable': ['run_sdk.sh']}, {'sdk_dir': None},
])
def test_remote_sdk_path_override_rejected_without_interrupt(light, monkeypatch, action, override):
    light.dispatch('chase', {'r': 50, 'duration_s': 2})
    worker, backend = light._thread, light._backend
    popen = Mock(side_effect=AssertionError('must not launch a process'))
    monkeypatch.setattr(ext.subprocess, 'Popen', popen)
    result = light.dispatch(action, dict(backend='sdk', sdk_exclusive=True, **override))
    assert not result['ok'] and result['code'] == 'INVALID_ARGUMENT'
    assert 'fixed to' in result['message']
    assert light._thread is worker and worker.is_alive()
    assert light._backend is backend and light._active
    popen.assert_not_called()


def test_sdk_paths_not_editable_and_constructor_rejects_override(tmp_path):
    plugin = ext.FaceLightPlugin({}, '', None, None)
    schema = plugin.get_tool()['configSchema']['properties']
    assert 'sdk_executable' not in schema and 'sdk_dir' not in schema
    alias = tmp_path / 'launcher'
    alias.symlink_to(ext._FACE_SDK_EXECUTABLE)
    for config in ({'sdk_executable': str(alias)}, {'sdk_dir': '/tmp/vendor-sdk'}):
        with pytest.raises(ValueError, match='fixed to'):
            ext.FaceLightPlugin(config, '', None, None)


def test_sdk_launch_uses_fixed_paths_and_overrides_inherited_environment(monkeypatch):
    monkeypatch.setenv('FACE_LIGHT_SDK_DIR', '/tmp/untrusted-sdk')
    popen = Mock(side_effect=OSError('offline: no launcher installed'))
    monkeypatch.setattr(ext.subprocess, 'Popen', popen)
    plugin = ext.FaceLightPlugin({'backend': 'sdk', 'sdk_exclusive': True}, '', None, None)
    assert not plugin.start()['ok']
    assert popen.call_args.args[0] == [ext._FACE_SDK_EXECUTABLE]
    assert popen.call_args.kwargs['env']['FACE_LIGHT_SDK_DIR'] == ext._FACE_SDK_DIR
    assert plugin._backend.process is None


def test_sdk_launcher_is_executable_without_docker_permission_churn():
    launcher = GO1 / 'deploy/face_light/run_sdk.sh'
    assert launcher.stat().st_mode & 0o111
    assert 'chmod +x /deploy/face_light/run_sdk.sh' not in (GO1 / 'Dockerfile').read_text()


def test_missing_sdk_runtime_launcher_returns_clear_error():
    launcher = GO1 / 'deploy/face_light/run_sdk.sh'
    plugin = sdk_test_plugin(launcher, sdk_dir='/missing/official-sdk')
    result = plugin.start()
    assert not result['ok'] and 'headers are missing' in result['message']
    assert plugin._backend.process is None


def test_sdk_stubborn_process_is_killed(tmp_path):
    helper = tmp_path / 'helper'
    helper.write_text(f'''#!{sys.executable}
import signal, sys, time
signal.signal(signal.SIGTERM, signal.SIG_IGN)
print("READY face-light-v1", flush=True)
for line in sys.stdin:
    time.sleep(60)
''')
    helper.chmod(0o755)
    plugin = sdk_test_plugin(helper)
    assert plugin.start()['ok']
    process = plugin._backend.process
    assert not plugin.dispatch('set_color', {'r': 1})['ok']
    assert process.poll() is not None
    plugin.stop()
