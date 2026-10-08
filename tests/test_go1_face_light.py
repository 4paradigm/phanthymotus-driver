"""Offline face_light contract, timing, cancellation and transport checks; no robot IO."""
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

GO1 = Path(__file__).resolve().parents[1] / "unitree" / "go1"
sys.path.insert(0, str(GO1))
import ext_devices as ext


@pytest.fixture
def light():
    plugin = ext.make_face_light({"backend": "simulated"}, "test", None, None)
    assert plugin.dispatch("start", {})["ok"]
    yield plugin
    plugin.stop()


def wait_until(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "condition timed out"
        time.sleep(0.005)


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


@pytest.fixture
def fake_mqtt(monkeypatch):
    client = Mock()
    client.is_connected.return_value = True
    message = Mock(rc=0)
    message.is_published.return_value = True
    client.publish.return_value = message
    mqtt = SimpleNamespace(Client=Mock(return_value=client),
                           CallbackAPIVersion=SimpleNamespace(VERSION1=1), MQTT_ERR_SUCCESS=0)
    monkeypatch.setattr(ext, "_HAS_MQTT", True)
    monkeypatch.setattr(ext, "mqtt", mqtt, raising=False)
    return client, message


def test_mqtt_three_bytes_compatibility_and_capabilities(fake_mqtt):
    client, _ = fake_mqtt
    plugin = ext.FaceLightPlugin({}, "", None, None)
    try:
        assert plugin.start()["ok"]
        assert plugin.dispatch("set_color", {"r": 12, "g": 34, "b": 56})["ok"]
        client.publish.assert_called_with("face_light/color", bytes([12, 34, 56]), qos=0, retain=False)
        for action, args in [("set_led", {"index": 0}), ("set_leds", {"colors": [[0, 0, 0]] * 12}), ("chase", {})]:
            assert plugin.dispatch(action, args)["code"] == "UNSUPPORTED_CAPABILITY"
        info = plugin._info()
        assert not info["capabilities"]["per_led"]
        assert info["capabilities"]["effects"] == ["blink", "breathe", "fade"]
        for action in ("blink", "breathe", "fade"):
            assert plugin.dispatch(action, {"r": 20, "to_b": 40, "duration_s": 0.05})["ok"]
            wait_until(lambda: not plugin._info()["running"])
    finally:
        plugin.stop()
    client.disconnect.assert_called_once()
    client.loop_stop.assert_called_once()


@pytest.mark.parametrize("failure", ["disconnected", "rc", "timeout", "exception"])
def test_mqtt_failure_is_not_success(fake_mqtt, failure):
    client, message = fake_mqtt
    plugin = ext.FaceLightPlugin({}, "", None, None)
    plugin.start()
    if failure == "disconnected":
        client.is_connected.return_value = False
    elif failure == "rc":
        message.rc = 4
    elif failure == "timeout":
        message.is_published.return_value = False
    else:
        message.wait_for_publish.side_effect = RuntimeError("socket lost")
    try:
        assert plugin.dispatch("set_color", {"b": 90})["code"] == "NOT_AVAILABLE"
        assert plugin._info()["colors"] is None
        if failure != "disconnected":
            assert plugin._backend.client is None  # failed send cannot replay queued frames
    finally:
        plugin.stop()


def test_missing_mqtt_and_start_failure(monkeypatch, fake_mqtt):
    monkeypatch.setattr(ext, "_HAS_MQTT", False)
    plugin = ext.FaceLightPlugin({}, "", None, None)
    assert not plugin.start()["ok"] and not plugin._active
    assert plugin.dispatch("off", {})["code"] == "NOT_AVAILABLE"
    monkeypatch.setattr(ext, "_HAS_MQTT", True)
    client, _ = fake_mqtt
    client.connect_async.side_effect = RuntimeError("connect failed")
    assert not plugin.start()["ok"]
    assert plugin._backend.client is None
    assert plugin.stop()["ok"]


def test_single_card_bundle_and_packaging():
    import importlib.util
    import yaml
    spec = importlib.util.spec_from_file_location("go1_face_main", GO1 / "main.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    bundle = module.Go1Bundle({"plugins": {"face_light": {"enabled": True, "backend": "simulated"}}}, "", None, None)
    bundle.start_all()
    try:
        tools = bundle.get_all_tools()
        assert len(tools) == 1 and tools[0]["name"] == "face_light"
        schema = tools[0]["inputSchema"]
        assert set(schema["properties"]["action"]["enum"]) == set(schema["x-action-params"])
        assert bundle.dispatch("face_light", {"action": "set_led", "index": 11, "r": 9})["ok"]
        assert bundle.dispatch("face_light", {"action": "info"})["simulated"]
        config = yaml.safe_load((GO1 / "config.yaml").read_text())
        assert config["plugins"]["face_light"]["backend"] == "mqtt"
        metadata = yaml.safe_load((GO1 / "driver.yaml").read_text())
        assert sum(item["name"] == "face_light" for item in metadata["cards"]) == 1
        assert "COPY ext_devices.py" in (GO1 / "Dockerfile").read_text()
    finally:
        bundle.stop_all()


def test_concurrent_start_stop_no_resurrection(fake_mqtt):
    client, _ = fake_mqtt
    entered, release = threading.Event(), threading.Event()

    def connect(*args):
        entered.set()
        assert release.wait(1)

    client.connect_async.side_effect = connect
    plugin = ext.FaceLightPlugin({}, "", None, None)
    starter = threading.Thread(target=plugin.start)
    stopper = threading.Thread(target=plugin.stop)
    starter.start()
    assert entered.wait(1)
    stopper.start()
    release.set()
    starter.join(1)
    stopper.join(1)
    assert not starter.is_alive() and not stopper.is_alive()
    assert not plugin._active and plugin._backend.client is None
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


def test_publish_exception_drops_connection(fake_mqtt):
    client, _ = fake_mqtt
    client.publish.side_effect = RuntimeError("send exception")
    plugin = ext.FaceLightPlugin({}, "", None, None)
    plugin.start()
    assert not plugin.dispatch("blink", {"r": 255})["ok"]
    assert plugin._backend.client is None
    assert not plugin._info()["running"]
    plugin.stop()


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


def test_sdk_backend_complete_card_path(sdk_helper):
    helper, log = sdk_helper
    plugin = ext.FaceLightPlugin({"backend": "sdk", "sdk_executable": str(helper), "sdk_exclusive": True}, "", None, None)
    start = plugin.start()
    assert start["ok"], start
    process = plugin._backend.process
    try:
        colors = [[i, 10, 255 - i] for i in range(12)]
        assert plugin.dispatch("set_leds", {"colors": colors})["delivery"] == "sdk_udp_socket_sent"
        assert [int(v) for v in log.read_text().splitlines()[0].split()] == [v for c in colors for v in c]
        assert plugin.dispatch("set_led", {"index": 11, "r": 255})["ok"]
        assert plugin.dispatch("chase", {"g": 20, "duration_s": 0.1, "period_s": 0.2})["ok"]
        wait_until(lambda: not plugin._info()["running"])
        assert plugin._info()["capabilities"]["official_sdk_integrated"]
        assert not plugin._info()["hardware_verified"]
    finally:
        assert plugin.stop()["ok"]
    assert process.poll() is not None and plugin._backend.process is None
    assert [int(v) for v in log.read_text().splitlines()[-1].split()] == [0] * 36


def test_sdk_exclusive_guard_and_missing_executable(sdk_helper):
    helper, _ = sdk_helper
    plugin = ext.FaceLightPlugin({"backend": "sdk", "sdk_executable": str(helper)}, "", None, None)
    result = plugin.start()
    assert not result["ok"] and "sdk_exclusive" in result["message"]
    assert plugin._backend.process is None
    plugin = ext.FaceLightPlugin({"backend": "sdk", "sdk_executable": "/missing/face-light-helper", "sdk_exclusive": True}, "", None, None)
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
    plugin = ext.FaceLightPlugin({"backend": "sdk", "sdk_executable": str(helper), "sdk_exclusive": True}, "", None, None)
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
    plugin = ext.FaceLightPlugin({"backend": "sdk", "sdk_executable": str(helper), "sdk_exclusive": True}, "", None, None)
    assert not plugin.start()["ok"]
    assert plugin._backend.process is None


def test_canvas_config_replay_preserves_effect(light):
    light.dispatch('chase', {'g': 60, 'duration_s': 2})
    worker = light._thread
    assert light.dispatch('config', {'backend': 'simulated'})['changed'] is False
    assert light._thread is worker and worker.is_alive()


def test_canvas_backend_config_requires_restart(light, sdk_helper):
    helper, _ = sdk_helper
    light.dispatch('chase', {'g': 60, 'duration_s': 2})
    worker = light._thread
    result = light.dispatch('config', {'backend': 'sdk', 'sdk_executable': str(helper), 'sdk_exclusive': True})
    assert result['ok'] and result['needs_start']
    assert not worker.is_alive() and not light._active
    assert light.dispatch('start', {})['ok']
    assert light.dispatch('set_led', {'index': 7, 'r': 10})['ok']
    assert light._info()['backend'] == 'sdk'


@pytest.mark.parametrize('config', [{'backend': 'invalid'}, {'mqtt_port': True}, {'mqtt_port': 0},
                                    {'sdk_exclusive': 'true'}, {'sdk_dir': ''}, {'sdk_executable': None}])
def test_bad_canvas_config_preserves_backend(light, config):
    backend = light._backend
    assert light.dispatch('config', config)['code'] == 'INVALID_ARGUMENT'
    assert light._backend is backend and light._active


def test_missing_sdk_runtime_launcher_returns_clear_error():
    launcher = GO1 / 'deploy/face_light/run_sdk.sh'
    plugin = ext.FaceLightPlugin({'backend': 'sdk', 'sdk_executable': str(launcher),
                                 'sdk_dir': '/missing/official-sdk', 'sdk_exclusive': True}, '', None, None)
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
    plugin = ext.FaceLightPlugin({'backend': 'sdk', 'sdk_executable': str(helper), 'sdk_exclusive': True}, '', None, None)
    assert plugin.start()['ok']
    process = plugin._backend.process
    assert not plugin.dispatch('set_color', {'r': 1})['ok']
    assert process.poll() is not None
    plugin.stop()
