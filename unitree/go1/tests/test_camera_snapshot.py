"""Go1 snapshot card: direct Nano capture and persistent JPEG output."""

import importlib
import json
import queue
import socket
import struct
import threading
import urllib.request
from contextlib import nullcontext
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest


@pytest.fixture
def completions(monkeypatch):
    received = queue.Queue()

    def post(request, **kwargs):
        assert request.full_url == "http://127.0.0.1:15678/api/acp/complete"
        assert request.get_method() == "POST"
        assert kwargs["timeout"] <= 5
        received.put(json.loads(request.data))
        return nullcontext()

    monkeypatch.setenv("AGENT_CORE_URL", "http://127.0.0.1:15678/")
    monkeypatch.setattr(urllib.request, "urlopen", post)
    return received


def completed_capture(card, completions, position="front"):
    accepted = card.dispatch("capture_photo", {"position": position})
    assert accepted["ok"] is True
    assert accepted["state"] == "capturing"
    assert accepted["action_id"]
    terminal = completions.get(timeout=3)
    assert terminal["action_id"] == accepted["action_id"]
    assert terminal["tool"] == "camera_snapshot"
    assert terminal["status"] == ("completed" if terminal["result"]["ok"] else "error")
    assert completions.empty()
    assert card.dispatch("info", {})["last_capture"]["result"] == terminal["result"]
    return terminal["result"]


def test_snapshot_connects_to_selected_camera_and_saves_jpeg(tmp_path, completions):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    jpeg = b"\xff\xd8photo\xff\xd9"
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    server.settimeout(2)
    port = server.getsockname()[1]

    def send_frame():
        try:
            connection, _ = server.accept()
            with connection:
                frame = struct.pack(">I", len(jpeg)) + jpeg
                connection.sendall(frame[:3])
                connection.sendall(frame[3:])
        finally:
            server.close()

    sender = threading.Thread(target=send_frame, daemon=True)
    sender.start()
    try:
        card = snapshot.CameraSnapshotPlugin({
            "output_dir": str(tmp_path),
            "positions": {"left": {"board_ip": "127.0.0.1", "image_port": port}},
        })
        result = completed_capture(card, completions, "left")
    finally:
        sender.join(timeout=2)

    assert result["ok"] is True
    assert result["position"] == "left"
    assert Path(result["file_path"]).read_bytes() == jpeg
    assert not sender.is_alive()


def test_snapshot_rejects_invalid_camera_without_creating_file(tmp_path):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})
    assert card.dispatch("capture_photo", {"position": "invalid"})["code"] == "INVALID_ARGUMENT"
    assert list(tmp_path.iterdir()) == []


def test_snapshot_rejects_incomplete_camera_frame(tmp_path, completions):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def send_incomplete_frame():
        try:
            connection, _ = server.accept()
            with connection:
                connection.sendall(struct.pack(">I", 10) + b"short")
        finally:
            server.close()

    sender = threading.Thread(target=send_incomplete_frame, daemon=True)
    sender.start()
    card = snapshot.CameraSnapshotPlugin({
        "output_dir": str(tmp_path),
        "positions": {"front": {"board_ip": "127.0.0.1", "image_port": port}},
    })
    result = completed_capture(card, completions)
    sender.join(timeout=2)
    assert result["code"] == "CAMERA_UNAVAILABLE"
    assert list(tmp_path.iterdir()) == []


def test_snapshot_does_not_leave_a_partial_photo_when_publish_fails(tmp_path, monkeypatch, completions):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    monkeypatch.setattr(snapshot.CameraSnapshotPlugin, "_capture_jpeg",
                        lambda self, position: b"\xff\xd8photo\xff\xd9")

    def fail_publish(self, target):
        raise OSError("storage interrupted")

    monkeypatch.setattr(Path, "replace", fail_publish)
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})
    assert completed_capture(card, completions)["code"] == "SAVE_FAILED"
    assert list(tmp_path.iterdir()) == []


def test_go1_manifest_lists_snapshot_card():
    driver = (Path(__file__).resolve().parents[1] / "driver.yaml").read_text(encoding="utf-8")
    assert "name: camera_snapshot" in driver


def test_go1_bundle_exposes_snapshot_tool_without_rgb(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    bundle = main.Go1Bundle({"plugins": {
        "camera_snapshot": {"enabled": True},
    }}, "test_go1", None, None)
    tools = {tool["name"]: tool for tool in bundle.get_all_tools()}
    assert tools["camera_snapshot"]["type"] == "actuator"
    assert bundle.dispatch("camera_snapshot", {"action": "info"})["state"] == "ready"


def test_snapshot_declares_completion_only_for_capture():
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    schema = snapshot.CameraSnapshotPlugin({}).get_tool()["inputSchema"]
    assert schema["x-completion"]["actions"] == ["capture_photo"]
    assert schema["x-completion"]["timeout"] >= 33
    assert schema["x-resource"] == "camera"


@pytest.mark.parametrize("error, code", [
    (TimeoutError("Nano timed out"), "CAMERA_UNAVAILABLE"),
    (RuntimeError("worker failed"), "CAPTURE_FAILED"),
])
def test_snapshot_failure_completes_and_releases_position(tmp_path, monkeypatch, completions, error, code):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})

    def fail(position):
        raise error

    monkeypatch.setattr(card, "_capture_jpeg", fail)
    assert completed_capture(card, completions)["code"] == code
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    assert completed_capture(card, completions)["ok"] is True


def test_capture_returns_before_network_finishes_and_blocks_same_position(tmp_path, monkeypatch, completions):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})
    entered, release, returned = threading.Event(), threading.Event(), threading.Event()
    results = []

    def capture(position):
        entered.set()
        assert release.wait(3)
        return b"\xff\xd8photo\xff\xd9"

    def call():
        results.append(card.dispatch("capture_photo", {}))
        returned.set()

    monkeypatch.setattr(card, "_capture_jpeg", capture)
    caller = threading.Thread(target=call)
    caller.start()
    try:
        assert entered.wait(1)
        assert returned.wait(1), "dispatch waited for the Nano"
        assert results[0]["action_id"]
        stopped = card.dispatch("stop", {})
        assert stopped["state"] == "idle"
        assert stopped["capture_active"] is True
        assert card.dispatch("info", {})["state"] == "capturing"
        busy = card.dispatch("capture_photo", {})
        assert busy["code"] == "RESOURCE_BUSY"
        assert "action_id" not in busy
        assert completions.empty()
    finally:
        release.set()
        caller.join(3)
    terminal = completions.get(timeout=3)
    assert terminal["action_id"] == results[0]["action_id"]
    assert terminal["status"] == "completed"


@pytest.mark.parametrize("camera_type", ["rgb", "depth", "pointcloud"])
def test_snapshot_rejects_active_same_position_stream(tmp_path, monkeypatch, completions, camera_type):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    camera = importlib.import_module("unitree.go1.camera")
    monkeypatch.setattr(camera, "_HAS_ROS2", False)
    plugin = camera.Plugin({}, "test", None, None, camera_type)
    plugin._node = object()
    release = threading.Event()
    monkeypatch.setattr(plugin._stream_cls, "_loop", lambda *args: release.wait(3))
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    assert plugin.dispatch("start", {"position": "left"})["ok"]
    try:
        busy = card.dispatch("capture_photo", {"position": "left"})
        assert busy["code"] == "RESOURCE_BUSY"
        assert "action_id" not in busy
        assert completions.empty()
        assert completed_capture(card, completions, "right")["ok"]
        # stop 返回时接收线程可能还在退出；关闭连接前仍须保留占用。
        plugin.dispatch("stop", {})
        assert card.dispatch("capture_photo", {"position": "left"})["code"] == "RESOURCE_BUSY"
    finally:
        release.set()
        plugin._streams["default"]._thread.join(3)
        plugin.stop()
    assert completed_capture(card, completions, "left")["ok"]


def test_stream_cannot_start_during_snapshot(tmp_path, monkeypatch, completions):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    camera = importlib.import_module("unitree.go1.camera")
    monkeypatch.setattr(camera, "_HAS_ROS2", False)
    plugin = camera.make_camera_rgb({}, "test", None, None)
    plugin._node = object()
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})
    release = threading.Event()

    def capture(position):
        assert release.wait(3)
        return b"\xff\xd8photo\xff\xd9"

    monkeypatch.setattr(card, "_capture_jpeg", capture)
    accepted = card.dispatch("capture_photo", {"position": "front"})
    try:
        assert accepted["action_id"]
        assert plugin.dispatch("start", {"position": "front"})["code"] == "RESOURCE_BUSY"
        assert plugin._streams == {}
    finally:
        release.set()
    assert completions.get(timeout=3)["status"] == "completed"


@pytest.mark.parametrize("source", ["camera_rgb", "camera_depth"])
def test_bundle_snapshot_inherits_camera_positions_even_when_stream_disabled(monkeypatch, source):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    bundle = main.Go1Bundle({"plugins": {
        source: {"enabled": False, "positions": {"left": {"board_ip": "10.0.0.8", "image_port": 9923}}},
        "camera_snapshot": {"enabled": True, "positions": {"left": {"image_port": 9924}}},
    }}, "test_go1", None, None)
    card = next(p for p in bundle._plugins if p.get_tool()["name"] == "camera_snapshot")
    assert card._endpoints["left"] == ("10.0.0.8", 9924)


def test_snapshot_and_rgb_have_identical_default_endpoints():
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    camera = importlib.import_module("unitree.go1.camera")
    card = snapshot.CameraSnapshotPlugin({})
    rgb = camera.make_camera_rgb({}, "test", None, None)
    assert card._endpoints == {p: (cfg["board_ip"], cfg["image_port"]) for p, cfg in rgb._positions.items()}


@pytest.mark.parametrize("jpeg, status", [(b"\xff\xd8photo\xff\xd9", "completed"), (b"bad", "error")])
def test_completion_reaches_http_endpoint(tmp_path, monkeypatch, jpeg, status):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    received = queue.Queue()

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            received.put((self.path, json.loads(self.rfile.read(int(self.headers["Content-Length"])))))
            self.send_response(200)
            self.end_headers()

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    serving = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05})
    serving.start()
    monkeypatch.setenv("AGENT_CORE_URL", f"http://127.0.0.1:{server.server_port}")
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: jpeg)
    try:
        accepted = card.dispatch("capture_photo", {})
        path, terminal = received.get(timeout=3)
        assert path == "/api/acp/complete"
        assert terminal["action_id"] == accepted["action_id"]
        assert terminal["status"] == status
        assert terminal["result"]["ok"] == (status == "completed")
    finally:
        server.shutdown()
        serving.join(3)
        server.server_close()


def test_failed_completion_delivery_keeps_result_and_releases_camera(tmp_path, monkeypatch, caplog):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    finished = threading.Event()
    notify = card._notify_complete
    attempts = []

    def unreachable(*args, **kwargs):
        attempts.append(1)
        raise OSError("Core offline")

    def notify_and_finish(*args):
        try:
            notify(*args)
        finally:
            finished.set()

    monkeypatch.setattr(urllib.request, "urlopen", unreachable)
    monkeypatch.setattr(snapshot.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(card, "_notify_complete", notify_and_finish)
    accepted = card.dispatch("capture_photo", {})
    assert finished.wait(3)
    info = card.dispatch("info", {})
    assert info["state"] == "ready"
    assert info["last_capture"]["action_id"] == accepted["action_id"]
    assert info["last_capture"]["result"]["ok"]
    assert "completion delivery failed" in caplog.text
    assert len(attempts) == 3
    assert "front" not in snapshot.camera._SNAPSHOT_POSITIONS


def test_transient_completion_delivery_failure_retries_successfully(tmp_path, monkeypatch, completions):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    successful_post = urllib.request.urlopen
    attempts = []
    delays = []

    def intermittent(request, **kwargs):
        attempts.append(json.loads(request.data))
        if len(attempts) == 1:
            raise OSError("Core temporarily offline")
        return successful_post(request, **kwargs)

    monkeypatch.setattr(urllib.request, "urlopen", intermittent)
    monkeypatch.setattr(snapshot.time, "sleep", delays.append)
    accepted = card.dispatch("capture_photo", {})
    terminal = completions.get(timeout=3)
    assert len(attempts) == 2
    assert attempts[0] == attempts[1] == terminal
    assert delays == [0.5]
    assert terminal["action_id"] == accepted["action_id"]
    assert terminal["status"] == "completed"


def test_worker_start_failure_does_not_leave_occupancy(tmp_path, monkeypatch, completions):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})

    def fail_start(self):
        raise RuntimeError("cannot create worker")

    with monkeypatch.context() as patch:
        patch.setattr(threading.Thread, "start", fail_start)
        result = card.dispatch("capture_photo", {})
    assert result["code"] == "CAPTURE_FAILED"
    assert "action_id" not in result
    assert completions.empty()
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    assert completed_capture(card, completions)["ok"]


def test_slow_fragments_cannot_reset_frame_deadline(monkeypatch):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    times = iter([0.0, 0.4, 1.1])
    timeouts = []

    class Connection:
        def settimeout(self, value):
            timeouts.append(value)

        def recv(self, size):
            return b"x"

    monkeypatch.setattr(snapshot.time, "monotonic", lambda: next(times))
    with pytest.raises(TimeoutError, match="deadline"):
        snapshot.CameraSnapshotPlugin._receive_exact(Connection(), 3, 1.0)
    assert timeouts == [1.0, 0.6]


def test_simultaneous_snapshots_reserve_position_atomically(tmp_path, monkeypatch, completions):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    cards = [snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)}) for _ in range(2)]
    start = threading.Barrier(3)
    release = threading.Event()
    results = queue.Queue()

    def capture(self, position):
        assert release.wait(3)
        return b"\xff\xd8photo\xff\xd9"

    def call(card):
        start.wait(3)
        results.put(card.dispatch("capture_photo", {}))

    monkeypatch.setattr(snapshot.CameraSnapshotPlugin, "_capture_jpeg", capture)
    callers = [threading.Thread(target=call, args=(card,)) for card in cards]
    for caller in callers:
        caller.start()
    try:
        start.wait(3)
        responses = [results.get(timeout=2) for _ in cards]
        assert sum(result["ok"] for result in responses) == 1
        assert next(result for result in responses if not result["ok"])["code"] == "RESOURCE_BUSY"
    finally:
        release.set()
        for caller in callers:
            caller.join(3)
    assert completions.get(timeout=3)["status"] == "completed"
    assert completions.empty()


def test_bundle_stream_and_snapshot_share_occupancy_and_reject_busy_hot_switch(tmp_path, monkeypatch, completions):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    bundle = main.Go1Bundle({"plugins": {
        "camera_rgb": {"enabled": True},
        "camera_snapshot": {"enabled": True, "output_dir": str(tmp_path)},
    }}, "test_go1", None, None)
    plugins = {p.get_tool()["name"]: p for p in bundle._plugins}
    rgb, snapshot = plugins["camera_rgb"], plugins["camera_snapshot"]
    camera = importlib.import_module("camera")
    monkeypatch.setattr(camera, "_HAS_ROS2", False)
    rgb._node = object()
    stream_release, capture_release = threading.Event(), threading.Event()
    monkeypatch.setattr(rgb._stream_cls, "_loop", lambda *args: stream_release.wait(3))

    def capture(position):
        assert capture_release.wait(3)
        return b"\xff\xd8photo\xff\xd9"

    monkeypatch.setattr(snapshot, "_capture_jpeg", capture)
    bundle.dispatch("camera_rgb", {"action": "config", "position": "left"})
    assert bundle.dispatch("camera_rgb", {"action": "start"})["ok"]
    stream = rgb._streams["default"]
    try:
        assert bundle.dispatch("camera_snapshot", {"action": "capture_photo", "position": "left"})["code"] == "RESOURCE_BUSY"
        accepted = bundle.dispatch("camera_snapshot", {"action": "capture_photo", "position": "right"})
        assert accepted["action_id"]
        result = bundle.dispatch("camera_rgb", {"action": "config", "position": "right"})
        assert result["code"] == "RESOURCE_BUSY"
        assert rgb._cfg["default"]["position"] == "left"
        assert rgb._streams["default"] is stream
        assert stream._run
    finally:
        capture_release.set()
        stream_release.set()
        stream._thread.join(3)
        bundle.stop_all()
    assert completions.get(timeout=3)["status"] == "completed"


def test_stream_cards_reject_occupied_position_before_opening_receiver(monkeypatch):
    camera = importlib.import_module("unitree.go1.camera")
    monkeypatch.setattr(camera, "_HAS_ROS2", False)
    rgb = camera.make_camera_rgb({}, "test", None, None)
    depth = camera.make_camera_depth({}, "test", None, None)
    rgb._node = depth._node = object()
    release = threading.Event()
    monkeypatch.setattr(rgb._stream_cls, "_loop", lambda *args: release.wait(3))
    monkeypatch.setattr(depth._stream_cls, "_loop", lambda *args: release.wait(3))
    assert rgb.dispatch("start", {"position": "front"})["ok"]
    try:
        busy = depth.dispatch("start", {"position": "front"})
        assert busy["code"] == "RESOURCE_BUSY"
        assert depth._streams == {}
        assert depth.dispatch("start", {"position": "left"})["ok"]
        second = depth._streams["default"]
        busy = depth.dispatch("config", {"position": "front"})
        assert busy["code"] == "RESOURCE_BUSY"
        assert depth._streams["default"] is second
        assert second._run and second.position == "left"
        assert depth.dispatch("start", {"instance_id": "another", "position": "left"})["code"] == "RESOURCE_BUSY"
        assert "another" not in depth._streams
    finally:
        rgb.stop()
        depth.stop()
        release.set()
        rgb._streams["default"]._thread.join(3)
        if "default" in depth._streams:
            depth._streams["default"]._thread.join(3)


def test_stream_hot_switch_keeps_previous_position_when_target_is_busy(monkeypatch):
    camera = importlib.import_module("unitree.go1.camera")
    monkeypatch.setattr(camera, "_HAS_ROS2", False)
    rgb = camera.make_camera_rgb({}, "test", None, None)
    rgb._node = object()
    release = threading.Event()
    monkeypatch.setattr(rgb._stream_cls, "_loop", lambda *args: release.wait(3))
    assert rgb.dispatch("config", {"instance_id": "first", "position": "front"})["ok"]
    assert rgb.dispatch("config", {"instance_id": "second", "position": "left"})["ok"]
    assert rgb.dispatch("start", {"instance_id": "first"})["ok"]
    assert rgb.dispatch("start", {"instance_id": "second"})["ok"]
    second = rgb._streams["second"]
    try:
        assert rgb.dispatch("start", {"instance_id": "second"})["ok"]
        assert rgb._streams["second"] is second
        assert rgb.dispatch("config", {"instance_id": "second", "position": "front"})["code"] == "RESOURCE_BUSY"
        assert rgb._cfg["second"]["position"] == "left"
        assert rgb._streams["second"] is second
        assert second._run and second.position == "left"
    finally:
        rgb.stop()
        release.set()
        for stream in rgb._streams.values():
            stream._thread.join(3)


def test_simultaneous_stream_starts_admit_only_one_position(monkeypatch):
    camera = importlib.import_module("unitree.go1.camera")
    monkeypatch.setattr(camera, "_HAS_ROS2", False)
    plugins = [camera.make_camera_rgb({}, "test", None, None),
               camera.make_camera_depth({}, "test", None, None)]
    release = threading.Event()
    start = threading.Barrier(3)
    results = queue.Queue()
    for plugin in plugins:
        plugin._node = object()
        monkeypatch.setattr(plugin._stream_cls, "_loop", lambda *args: release.wait(3))

    def call(plugin):
        start.wait(3)
        results.put(plugin.dispatch("start", {"position": "front"}))

    callers = [threading.Thread(target=call, args=(plugin,)) for plugin in plugins]
    for caller in callers:
        caller.start()
    try:
        start.wait(3)
        responses = [results.get(timeout=2) for _ in plugins]
        assert sum(result["ok"] for result in responses) == 1
        assert next(result for result in responses if not result["ok"])["code"] == "RESOURCE_BUSY"
    finally:
        for plugin in plugins:
            plugin.stop()
        release.set()
        for caller in callers:
            caller.join(3)
        for plugin in plugins:
            for stream in plugin._streams.values():
                stream._thread.join(3)
