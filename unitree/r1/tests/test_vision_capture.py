"""R1 vision_capture contract, without rclpy, ROS or ffmpeg.

What is pinned here is the part a real robot cannot tell you cheaply: the MCP
schema agent-core reads, the "never return a cached frame" rule, single-flight
recording, and the terminal completion that releases the ACP barrier.

Run:  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest unitree/r1/tests/test_vision_capture.py -q
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import threading
import time
import types

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "vision_capture.py"


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((msg_type, topic, callback, qos))
        return object()

    def destroy_node(self):
        self.subscriptions.clear()

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None,
                                     error=lambda *a: None, debug=lambda *a: None)


class FakeExecutor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, node):
        self.nodes.remove(node)


def install_stubs():
    rclpy = types.ModuleType("rclpy")
    node = types.ModuleType("rclpy.node")
    node.Node = FakeNode
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    sensor = types.ModuleType("sensor_msgs")
    messages = types.ModuleType("sensor_msgs.msg")
    messages.CompressedImage = type("CompressedImage", (), {})
    sensor.msg = messages
    sys.modules.update({
        "rclpy": rclpy,
        "rclpy.node": node,
        "rclpy.qos": qos,
        "sensor_msgs": sensor,
        "sensor_msgs.msg": messages,
    })


install_stubs()
spec = importlib.util.spec_from_file_location("r1_vision_capture", MODULE)
vision_capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vision_capture)

ACTIONS = {"start", "stop", "info", "list_cameras", "capture_photo", "record_video"}


def jpeg(width=1280, height=720, payload=b"pixels"):
    sof = (b"\xff\xc0" + (17).to_bytes(2, "big") + b"\x08"
           + height.to_bytes(2, "big") + width.to_bytes(2, "big")
           + b"\x03" + b"\x00" * 9)
    return b"\xff\xd8" + sof + payload + b"\xff\xd9"


def make_plugin(tmp_path, **config):
    config.setdefault("output_dir", str(tmp_path))
    completions = []
    plugin = vision_capture.VisionCapturePlugin(
        config, "r1", FakeExecutor(),
        completion_sender=lambda *args: completions.append(args))
    return plugin, completions


def deliver(plugin, camera="main", data=None):
    plugin._on_frame(camera, types.SimpleNamespace(data=data or jpeg()))


@pytest.fixture(autouse=True)
def fast_timeouts(monkeypatch):
    monkeypatch.setattr(vision_capture, "FIRST_FRAME_TIMEOUT_S", 0.25)
    monkeypatch.setattr(vision_capture, "MAX_FRAME_AGE_S", 0.25)
    # ffmpeg is not needed: every recording test injects an encoder, but the
    # admission check still asks whether the binaries exist.
    monkeypatch.setattr(vision_capture.shutil, "which", lambda name: "/usr/bin/" + name)


def wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


# ── schema ───────────────────────────────────────────────────────────────────

def test_tool_is_an_actuator_with_the_shared_action_set(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    tool = plugin.get_tool()
    assert tool["name"] == "vision_capture"
    assert tool["type"] == "actuator"
    assert tool["multiInstance"] is False
    assert tool["description"].strip()
    assert set(tool["inputSchema"]["properties"]["action"]["enum"]) == ACTIONS


def test_action_params_cover_every_action(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    schema = plugin.get_tool()["inputSchema"]
    documented = set(schema["x-action-params"])
    assert documented == set(schema["properties"]["action"]["enum"])
    assert schema["x-action-params"]["capture_photo"]["params"] == ["camera"]
    assert set(schema["x-action-params"]["record_video"]["params"]) == {"camera", "duration_s"}


def test_completion_names_only_real_actions_and_matches_the_cap(tmp_path):
    plugin, _ = make_plugin(tmp_path, max_duration_s=7)
    schema = plugin.get_tool()["inputSchema"]
    completion = schema["x-completion"]
    assert completion["actions"] == ["record_video"]
    assert set(completion["actions"]) <= set(schema["properties"]["action"]["enum"])
    assert completion["timeout"] == 7 + 15
    assert schema["properties"]["duration_s"]["maximum"] == 7


def test_extensions_live_inside_input_schema(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    tool = plugin.get_tool()
    assert "x-completion" not in tool
    assert "x-completion" in tool["inputSchema"]


# ── lifecycle ────────────────────────────────────────────────────────────────

def test_lifecycle_verbs_report_the_states_the_canvas_expects(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    assert plugin.dispatch("start", {}) == {"state": "ready"}
    assert plugin.dispatch("stop", {}) == {"state": "idle"}


def test_subscribes_to_the_three_rgb_ports_only_while_started(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    assert plugin._node is None
    plugin.dispatch("start", {})
    topics = {topic for _, topic, _, _ in plugin._node.subscriptions}
    assert topics == {"/r1/camera/main", "/r1/camera/left", "/r1/camera/right"}
    plugin.dispatch("stop", {})
    assert plugin._node is None


def test_stop_forgets_frames_so_a_restart_cannot_serve_an_old_picture(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    plugin.dispatch("start", {})
    deliver(plugin)
    assert plugin.dispatch("info", {})["cameras"][0]["frames"] == 1
    plugin.dispatch("stop", {})
    plugin.dispatch("start", {})
    assert all(c["frames"] == 0 for c in plugin.dispatch("info", {})["cameras"])


# ── photo ────────────────────────────────────────────────────────────────────

def test_photo_never_returns_a_frame_that_predates_the_request(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    deliver(plugin, data=jpeg(payload=b"old"))

    result = plugin.dispatch("capture_photo", {"camera": "main"})

    assert result["ok"] is False
    assert result["code"] == "CAPTURE_FAILED"
    assert "No fresh frame" in result["message"]
    assert list(Path(tmp_path).rglob("*.jpg")) == []


def test_photo_saves_the_next_frame_without_leaving_a_partial(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    deliver(plugin, data=jpeg(payload=b"old"))
    timer = threading.Timer(0.05, lambda: deliver(plugin, data=jpeg(payload=b"new")))
    timer.start()

    result = plugin.dispatch("capture_photo", {"camera": "main"})
    timer.cancel()

    assert result["ok"] is True
    assert result["media_type"] == "photo"
    assert result["camera"] == "main"
    assert (result["width"], result["height"]) == (1280, 720)
    path = Path(result["file_path"])
    assert path.read_bytes() == jpeg(payload=b"new")
    assert list(path.parent.glob("*.part")) == []


def test_photo_rejects_a_camera_the_robot_does_not_have(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    result = plugin.dispatch("capture_photo", {"camera": "depth"})
    assert result["ok"] is False
    assert "camera must be one of" in result["message"]


def test_photo_carries_an_advisory_quality_verdict(tmp_path, monkeypatch):
    plugin, _ = make_plugin(tmp_path)
    monkeypatch.setattr(vision_capture, "image_quality",
                        lambda data: vision_capture.quality_verdict(120.0, 300.0))
    timer = threading.Timer(0.05, lambda: deliver(plugin, data=jpeg(payload=b"new")))
    timer.start()
    result = plugin.dispatch("capture_photo", {})
    timer.cancel()
    assert result["quality"]["ok"] is True
    assert plugin.dispatch("info", {})["last_capture"]["file_path"] == result["file_path"]


def test_quality_verdict_names_the_fix_not_just_the_failure():
    dark = vision_capture.quality_verdict(10.0, 400.0)
    blurred = vision_capture.quality_verdict(120.0, 5.0)
    good = vision_capture.quality_verdict(120.0, 400.0)
    assert dark["ok"] is False and "偏暗" in dark["hint"]
    assert blurred["ok"] is False and "模糊" in blurred["hint"]
    assert good == {"brightness": 120.0, "sharpness": 400.0, "ok": True, "hint": ""}


def test_quality_is_absent_rather_than_fatal_when_opencv_is_missing(tmp_path, monkeypatch):
    plugin, _ = make_plugin(tmp_path)
    monkeypatch.setattr(vision_capture, "image_quality", lambda data: None)
    timer = threading.Timer(0.05, lambda: deliver(plugin))
    timer.start()
    result = plugin.dispatch("capture_photo", {})
    timer.cancel()
    assert result["ok"] is True
    assert result["quality"] is None


def test_prune_removes_only_evidence_older_than_the_retention_window(tmp_path):
    import os
    plugin, _ = make_plugin(tmp_path, retention_days=7)
    photos = Path(tmp_path) / "photos"
    videos = Path(tmp_path) / "videos"
    photos.mkdir()
    videos.mkdir()
    old = photos / "old.jpg"
    fresh = photos / "fresh.jpg"
    old_video = videos / "old.mp4"
    for path in (old, fresh, old_video):
        path.write_bytes(b"x")
    ancient = time.time() - 8 * 86400
    os.utime(old, (ancient, ancient))
    os.utime(old_video, (ancient, ancient))

    plugin._prune()

    assert not old.exists()
    assert not old_video.exists()
    assert fresh.exists()


def test_retention_zero_disables_pruning(tmp_path):
    import os
    plugin, _ = make_plugin(tmp_path, retention_days=0)
    photos = Path(tmp_path) / "photos"
    photos.mkdir()
    old = photos / "old.jpg"
    old.write_bytes(b"x")
    ancient = time.time() - 30 * 86400
    os.utime(old, (ancient, ancient))
    plugin._prune()
    assert old.exists()


def test_jpeg_size_reads_the_sof_marker(tmp_path):
    assert vision_capture.jpeg_size(jpeg(544, 448)) == (544, 448)
    assert vision_capture.jpeg_size(b"not a jpeg") is None


# ── video ────────────────────────────────────────────────────────────────────

def fake_encoder(recorded, wait_for_cancel=False):
    def encode(active, cancel):
        path = Path(active["path"])
        path.write_bytes(b"mp4")
        if wait_for_cancel:
            cancel.wait(3.0)
            if cancel.is_set():
                raise RuntimeError("Video recording was cancelled")
        time.sleep(0.02)
        recorded.append(active)
        return {"ok": True, "media_type": "video", "file_path": str(path),
                "recorded_duration_s": active["duration_s"], "frames": 10,
                "captured_at": "2026-10-27T00:00:00+08:00"}
    return encode


def test_record_returns_immediately_and_reports_completion_once(tmp_path):
    plugin, completions = make_plugin(tmp_path)
    plugin._encoder = fake_encoder([])

    result = plugin.dispatch("record_video", {"duration_s": 2, "camera": "left"})

    assert result["ok"] is True
    assert result["state"] == "recording"
    assert result["duration_s"] == 2
    assert result["camera"] == "left"
    assert result["action_id"].startswith("vision_capture_record_video_")
    assert Path(result["file_path"]).suffix == ".mp4"

    assert wait_until(lambda: completions)
    action_id, status, payload = completions[0]
    assert action_id == result["action_id"]
    assert status == "completed"
    assert payload["ok"] is True
    info = plugin.dispatch("info", {})
    assert info["active_recording"] is None
    assert info["last_recording"]["status"] == "completed"


def test_only_one_recording_can_run(tmp_path):
    plugin, completions = make_plugin(tmp_path)
    plugin._encoder = fake_encoder([], wait_for_cancel=True)

    first = plugin.dispatch("record_video", {"duration_s": 3})
    second = plugin.dispatch("record_video", {"duration_s": 3})

    assert first["ok"] is True
    assert second["ok"] is False
    assert second["code"] == "RECORD_BUSY"
    plugin.dispatch("stop", {})
    assert wait_until(lambda: completions)


def test_stop_cancels_the_recording_and_removes_the_partial_file(tmp_path):
    plugin, completions = make_plugin(tmp_path)
    plugin._encoder = fake_encoder([], wait_for_cancel=True)

    started = plugin.dispatch("record_video", {"duration_s": 3})
    path = Path(started["file_path"])
    assert wait_until(path.exists)

    stopped = plugin.dispatch("stop", {})

    assert stopped["state"] == "stopping"
    assert wait_until(lambda: completions)
    _, status, payload = completions[0]
    assert status == "cancelled"
    assert payload["code"] == "RECORD_CANCELLED"
    assert not path.exists()
    assert plugin.dispatch("info", {})["last_recording"]["status"] == "cancelled"


def test_duration_is_clamped_to_the_configured_cap(tmp_path):
    plugin, completions = make_plugin(tmp_path, max_duration_s=5)
    plugin._encoder = fake_encoder([])

    result = plugin.dispatch("record_video", {"duration_s": 999})

    assert result["duration_s"] == 5
    assert wait_until(lambda: completions)


def test_recording_without_ffmpeg_fails_before_claiming_the_camera(tmp_path, monkeypatch):
    plugin, completions = make_plugin(tmp_path)
    monkeypatch.setattr(vision_capture.shutil, "which", lambda name: None)
    result = plugin.dispatch("record_video", {})
    assert result["ok"] is False
    assert "ffmpeg" in result["message"]
    assert plugin.dispatch("info", {})["active_recording"] is None
    assert completions == []


# ── status ───────────────────────────────────────────────────────────────────

def test_info_reports_frame_freshness_per_camera(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    deliver(plugin, "main")
    deliver(plugin, "main")
    info = plugin.dispatch("info", {})
    main = next(c for c in info["cameras"] if c["camera"] == "main")
    left = next(c for c in info["cameras"] if c["camera"] == "left")
    assert main["available"] is True
    assert main["frames"] == 2
    assert left["available"] is False
    assert info["output_dir"] == str(tmp_path)


def test_list_cameras_names_the_ports_the_canvas_wires(tmp_path):
    plugin, _ = make_plugin(tmp_path)
    listed = plugin.dispatch("list_cameras", {})
    assert {c["camera"]: c["topic"] for c in listed["cameras"]} == {
        "main": "/r1/camera/main",
        "left": "/r1/camera/left",
        "right": "/r1/camera/right",
    }
