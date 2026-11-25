"""Bumi live_stream card: contract, validation, and a real ffmpeg RTMP push."""

import importlib.util
import json
import shutil
import socket
import subprocess
import sys
import threading
import time
import types
from pathlib import Path

import pytest
import yaml

from test_mic_card import _load_device

BUMI = Path(__file__).resolve().parents[1]
HAVE_FFMPEG = shutil.which("ffmpeg") and shutil.which("ffprobe")


def _load():
    spec = importlib.util.spec_from_file_location("bumi_live_stream_test", BUMI / "live_stream.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Executor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, node):
        self.nodes.remove(node)


class _FakeInputs:
    """Stands in for the ROS node: feeds a JPEG at 15 fps and PCM in 20 ms chunks."""

    jpeg = b""

    def __init__(self, topics, on_jpeg, on_pcm, *, camera=True, mic=True):
        self.image_topic = "/robot/camera/color" if camera else ""
        self.audio_topic = "/robot/mic/audio" if mic else ""
        self._stop = threading.Event()
        self.node = types.SimpleNamespace(destroy_node=self._stop.set)
        self.on_jpeg, self.on_pcm = on_jpeg, on_pcm
        if camera or mic:
            threading.Thread(target=self._feed, daemon=True).start()

    def _feed(self):
        tick = 0
        while not self._stop.wait(0.02):
            tick += 1
            if self.audio_topic:
                self.on_pcm(b"\x10\x00" * 320, "pcm_16k_16bit_mono")
            if self.image_topic and tick % 3 == 0:
                self.on_jpeg(self.jpeg)


def _plugin(module, **inputs):
    executor = _Executor()
    plugin = module.LiveStreamPlugin(
        {"fps": 15}, "robot", executor,
        input_node=lambda *a: _FakeInputs(*a, **inputs))
    return plugin, executor


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def jpeg(tmp_path_factory):
    if not HAVE_FFMPEG:
        pytest.skip("ffmpeg not installed")
    path = tmp_path_factory.mktemp("frame") / "f.jpg"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-f", "lavfi", "-i", "testsrc=size=640x480",
                    "-frames:v", "1", str(path)], check=True)
    _FakeInputs.jpeg = path.read_bytes()
    return _FakeInputs.jpeg


def test_tool_keeps_the_address_a_setting_and_takes_camera_and_mic():
    tool = _load().LiveStreamPlugin({}, "robot", _Executor()).get_tool()
    field = tool["configSchema"]["properties"]["rtmp_url"]
    assert field["x-sensitive"] is True and field.get("format") != "password"
    # Agent Core turns every x-action-params entry into an LLM tool.
    assert set(tool["inputSchema"]["x-action-params"]) == {"go_live", "end_live", "info"}
    assert {p["format"] for p in tool["topic_in"]} == {"image/jpeg", "audio/pcm-16k"}


def test_bundle_loads_the_card_and_manifest_lists_it(monkeypatch):
    device, _ = _load_device(monkeypatch)
    monkeypatch.setitem(sys.modules, "device", device)
    monkeypatch.setitem(sys.modules, "live_stream", _load())
    monkeypatch.setitem(sys.modules, "rclpy.executors", types.ModuleType("rclpy.executors"))
    spec = importlib.util.spec_from_file_location("bumi_live_bundle_test", BUMI / "main.py")
    main = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(main)
    cfg = yaml.safe_load((BUMI / "config.yaml").read_text())
    bundle = main.BumiDeviceBundle({"plugins": {"live_stream": cfg["plugins"]["live_stream"]}},
                                   "robot", _Executor(), None, None)
    assert [t["name"] for t in bundle.get_all_tools()] == ["live_stream"]
    assert bundle.dispatch("live_stream", {"action": "info"})["state"] == "idle"
    manifest = yaml.safe_load((BUMI / "driver.yaml").read_text())
    assert {"name": "live_stream", "type": "actuator"} in manifest["cards"]


@pytest.mark.parametrize("url", ["http://example.com/live/k", "rtmp://example.com", "live/k", 42])
def test_config_rejects_anything_but_a_full_rtmp_address(url):
    plugin = _load().LiveStreamPlugin({}, "robot", _Executor())
    assert plugin.dispatch("config", {"rtmp_url": url})["state"] == "error"
    assert plugin.dispatch("go_live", {})["error"].startswith("请先在卡片设置里填写推流地址")


def test_go_live_needs_a_connected_camera():
    plugin, _ = _plugin(_load(), camera=False)
    plugin.dispatch("config", {"rtmp_url": "rtmps://push.example.com/live/key"})
    plugin.dispatch("start", {"input_topics": ["/robot/mic/audio"]})
    assert "没有连接摄像头" in plugin.dispatch("go_live", {})["error"]


def test_stop_removes_the_input_node():
    plugin, executor = _plugin(_load())
    started = plugin.dispatch("start", {"input_topics": ["/robot/camera/color", "/robot/mic/audio"]})
    assert started["state"] == "ready" and len(started["topic_in"]) == 2
    assert len(executor.nodes) == 1
    assert plugin.dispatch("stop", {}) == {"state": "idle"}
    assert executor.nodes == []


def test_go_live_pushes_video_and_audio_over_rtmp(jpeg, tmp_path):
    port = _free_port()
    out = tmp_path / "received.flv"
    server = subprocess.Popen(
        ["ffmpeg", "-loglevel", "error", "-listen", "1", "-i", f"rtmp://127.0.0.1:{port}/live/key",
         "-c", "copy", str(out)], stdin=subprocess.DEVNULL)
    plugin, _ = _plugin(_load())
    try:
        time.sleep(0.5)
        assert plugin.dispatch("config", {"rtmp_url": f"rtmp://127.0.0.1:{port}/live/key"}) \
            == {"state": "configured"}
        plugin.dispatch("start", {"input_topics": ["/robot/camera/color", "/robot/mic/audio"]})
        assert plugin.dispatch("go_live", {})["state"] == "live"
        time.sleep(3)
        info = plugin.dispatch("info", {})
        assert info["state"] == "live" and info["duration_s"] >= 5
        assert plugin.dispatch("end_live", {}) == {"state": "idle"}
        server.wait(timeout=10)
    finally:
        plugin.dispatch("stop", {})
        server.kill()
    probe = json.loads(subprocess.run(
        ["ffprobe", "-v", "error", "-show_streams", "-show_format", "-of", "json", str(out)],
        capture_output=True, check=True).stdout)
    codecs = {s["codec_type"]: s["codec_name"] for s in probe["streams"]}
    assert codecs == {"video": "h264", "audio": "aac"}
    assert float(probe["format"]["duration"]) > 4


def test_rejected_address_reports_the_error_without_the_key(jpeg):
    plugin, _ = _plugin(_load())
    plugin.dispatch("config", {"rtmp_url": f"rtmp://127.0.0.1:{_free_port()}/live/secret-key"})
    plugin.dispatch("start", {"input_topics": ["/robot/camera/color", "/robot/mic/audio"]})
    result = plugin.dispatch("go_live", {})
    plugin.dispatch("stop", {})
    assert result["state"] == "error"
    assert "推流失败" in result["error"] and "secret-key" not in result["error"]


@pytest.mark.parametrize("next_action", ["stop", "start"])
def test_pending_start_is_cancelled_by_newer_action(next_action):
    entered, release = threading.Event(), threading.Event()
    executor, created = _Executor(), []

    class Inputs:
        image_topic, audio_topic = "/robot/camera/color", "/robot/mic/audio"

        def __init__(self, topics, on_jpeg, on_pcm):
            self.node = types.SimpleNamespace(destroyed=False)
            self.node.destroy_node = lambda: setattr(self.node, "destroyed", True)
            created.append(self.node)
            if len(created) == 1:
                entered.set()
                assert release.wait(timeout=5)

    plugin = _load().LiveStreamPlugin({}, "robot", executor, input_node=Inputs)
    result = {}
    thread = threading.Thread(
        target=lambda: result.setdefault("old", plugin.dispatch("start", {"input_topics": ["/old"]})))
    thread.start()
    try:
        assert entered.wait(timeout=5)
        args = {"input_topics": ["/new"]} if next_action == "start" else {}
        expected = "ready" if next_action == "start" else "idle"
        assert plugin.dispatch(next_action, args)["state"] == expected
    finally:
        release.set()
        thread.join(timeout=5)

    assert not thread.is_alive()
    assert result["old"]["state"] == "superseded"
    assert created[0].destroyed
    assert executor.nodes == (created[1:] if next_action == "start" else [])
    assert plugin.dispatch("stop", {}) == {"state": "idle"}
    assert executor.nodes == []
