"""Bumi bundle plugin wiring, without ROS, SDK modules, or robot hardware."""

from __future__ import annotations

import sys
import json
import http.client
import threading
import types
from concurrent.futures import ThreadPoolExecutor
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest


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
        self.started = False
        self.stopped = False

    def attach_servo(self, servo):
        self.servo = servo

    def is_moving(self):
        return False

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


class FakeFoodSegmentationPlugin:
    PREFIX = "food_segmentation"

    def __init__(self, plugin_config, namespace, executor):
        self.plugin_config = plugin_config
        self.namespace = namespace
        self.executor = executor
        self.started = False
        self.stopped = False

    def get_tool(self):
        return {"name": "food_segmentation", "type": "processor"}

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def dispatch(self, action, _args):
        if action in ("start", "stop", "info"):
            return {"state": action}
        return None


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
    fake_food = types.ModuleType("food_segmentation")
    fake_food.FoodSegmentationPlugin = FakeFoodSegmentationPlugin
    monkeypatch.setitem(sys.modules, "food_segmentation", fake_food)
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


def test_food_segmentation_registration_coexists_with_existing_plugins(bundle_module):
    config = _config()
    config["plugins"]["food_segmentation"] = {
        "enabled": True, "model_path": "/models/yolo11n-seg.pt"}
    executor = object()
    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", executor, FakeHighCtrl(), media_ctrl=None)

    assert [tool["name"] for tool in bundle.get_all_tools()] == [
        "loco", "loco_servo", "food_segmentation"]
    food = bundle._plugins[-1]
    assert food.PREFIX == "food_segmentation"
    assert food.plugin_config["model_path"] == "/models/yolo11n-seg.pt"
    assert food.executor is executor
    bundle.start_all()
    assert food.started is True
    assert bundle.dispatch(
        "food_segmentation", {"action": "info"}) == {"state": "info"}
    bundle.stop_all()
    assert food.stopped is True


def test_food_segmentation_disabled_is_not_registered(bundle_module):
    config = _config(loco=False, loco_servo=False)
    config["plugins"]["food_segmentation"] = {"enabled": False}
    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", object(), high_ctrl=None, media_ctrl=None)
    assert bundle.get_all_tools() == []
    assert bundle._plugins == []


def test_food_segmentation_real_mcp_http_lifecycle_and_concurrency(bundle_module):
    config = _config(loco=False, loco_servo=False)
    config["plugins"]["food_segmentation"] = {
        "enabled": True, "model_path": "/models/yolo11n-seg.pt"}
    bundle = bundle_module.BumiDeviceBundle(
        config, "bumi_test", object(), high_ctrl=None, media_ctrl=None)
    bundle_module._bundle = bundle
    server = ThreadingHTTPServer(("127.0.0.1", 0), bundle_module.make_handler())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def rpc(method, params=None, request_id=1):
        connection = http.client.HTTPConnection(
            "127.0.0.1", server.server_address[1], timeout=2)
        body = json.dumps({
            "jsonrpc": "2.0", "id": request_id,
            "method": method, "params": params or {},
        })
        connection.request("POST", "/mcp", body, {
            "Content-Type": "application/json"})
        response = connection.getresponse()
        payload = json.loads(response.read())
        connection.close()
        assert response.status == 200
        return payload

    try:
        assert rpc("initialize")["result"]["protocolVersion"] == "2024-11-05"
        tools = rpc("tools/list")["result"]["tools"]
        assert tools == [{"name": "food_segmentation", "type": "processor"}]

        for action in ("info", "start", "start", "stop", "stop"):
            response = rpc("tools/call", {
                "name": "food_segmentation", "arguments": {"action": action}})
            content = json.loads(response["result"]["content"][0]["text"])
            assert content == {"state": action}

        unknown_tool = rpc("tools/call", {
            "name": "missing", "arguments": {"action": "info"}})
        assert unknown_tool["error"]["code"] == -32601
        unknown_action = rpc("tools/call", {
            "name": "food_segmentation", "arguments": {"action": "bad"}})
        assert unknown_action["error"]["code"] == -32601

        with ThreadPoolExecutor(max_workers=8) as pool:
            responses = list(pool.map(
                lambda index: rpc("tools/call", {
                    "name": "food_segmentation",
                    "arguments": {"action": "info"}}, index + 10),
                range(16),
            ))
        assert all("result" in response for response in responses)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2)

