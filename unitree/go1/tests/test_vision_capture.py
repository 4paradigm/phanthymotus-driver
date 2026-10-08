"""Go1 snapshot card: direct Nano capture and persistent JPEG output."""

import importlib
import json
import queue
import shutil
import socket
import struct
import subprocess
import threading
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest


def completed_capture(card, position="front"):
    result = card.dispatch("capture_photo", {"position": position})
    assert result["state"] == ("completed" if result["ok"] else "error")
    assert result["action_id"]
    assert card.dispatch("info", {})["last_capture"]["result"]["ok"] == result["ok"]
    return result


@pytest.mark.parametrize("action", ["capture_photo", "record_video"])
def test_media_result_waits_until_file_is_saved(tmp_path, monkeypatch, action):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(snapshot.shutil, "which", lambda command: "/usr/bin/ffmpeg")
    entered, release = threading.Event(), threading.Event()
    response = queue.Queue()

    def save(position, path, *args):
        entered.set()
        assert release.wait(3)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"saved")
        return {"ok": True, "media_type": "video" if args else "photo",
                "file_path": str(path), "filename": path.name}

    monkeypatch.setattr(card, "_record_and_save" if action == "record_video" else "_capture_and_save", save)
    caller = threading.Thread(target=lambda: response.put(card.dispatch(action, {})))
    caller.start()
    try:
        assert entered.wait(1)
        assert response.empty()
    finally:
        release.set()
        caller.join(3)
    result = response.get_nowait()
    assert result["state"] == "completed"
    assert Path(result["file_path"]).read_bytes() == b"saved"
    assert "x-completion" not in card.get_tool()["inputSchema"]


def test_mcp_result_waits_until_photo_is_saved(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    bundle = main.Go1Bundle({"plugins": {
        "vision_capture": {"enabled": True, "output_dir": str(tmp_path)},
    }}, "test_go1", None, None)
    card = next(plugin for plugin in bundle._plugins if plugin.get_tool()["name"] == "vision_capture")
    entered, release = threading.Event(), threading.Event()
    response = queue.Queue()

    def capture(position):
        entered.set()
        assert release.wait(3)
        return b"\xff\xd8photo\xff\xd9"

    monkeypatch.setattr(card, "_capture_jpeg", capture)
    monkeypatch.setattr(main, "_bundle", bundle)
    server = ThreadingHTTPServer(("127.0.0.1", 0), main.make_handler())
    serving = threading.Thread(target=server.serve_forever)
    serving.start()

    def invoke():
        request = urllib.request.Request(
            f"http://127.0.0.1:{server.server_port}/mcp",
            data=json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                             "params": {"name": "vision_capture", "arguments": {"action": "capture_photo"}}}).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(request, timeout=5) as reply:
            response.put(json.loads(reply.read()))

    caller = threading.Thread(target=invoke)
    caller.start()
    try:
        assert entered.wait(1)
        assert response.empty()
    finally:
        release.set()
        caller.join(3)
        server.shutdown()
        serving.join(3)
        server.server_close()
    result = json.loads(response.get_nowait()["result"]["content"][0]["text"])
    assert result["state"] == "completed"
    assert Path(result["file_path"]).read_bytes() == b"\xff\xd8photo\xff\xd9"


def test_snapshot_connects_to_selected_camera_and_saves_jpeg(tmp_path):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
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
        card = snapshot.VisionCapturePlugin({
            "output_dir": str(tmp_path),
            "positions": {"left": {"board_ip": "127.0.0.1", "image_port": port}},
        })
        result = completed_capture(card, "left")
    finally:
        sender.join(timeout=2)

    assert result["ok"] is True
    assert result["position"] == "left"
    assert Path(result["file_path"]).read_bytes() == jpeg
    assert not sender.is_alive()


def test_capture_result_waits_for_photo_to_be_saved(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    entered, release = threading.Event(), threading.Event()

    def capture(position):
        entered.set()
        assert release.wait(3)
        return b"\xff\xd8photo\xff\xd9"

    monkeypatch.setattr(card, "_capture_jpeg", capture)
    response = queue.Queue()
    caller = threading.Thread(target=lambda: response.put(card.dispatch("capture_photo", {"position": "front"})))
    caller.start()
    try:
        assert entered.wait(1)
        assert response.empty()
    finally:
        release.set()
        caller.join(3)
    result = response.get_nowait()
    path = Path(result["file_path"])
    assert path.parent == tmp_path / "photos"
    assert path.name.startswith("front_") and path.suffix == ".jpg"
    assert path.exists()


def test_shutdown_drains_active_photo_before_returning(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()

    def capture(position):
        entered.set()
        assert release.wait(3)
        return b"\xff\xd8photo\xff\xd9"

    monkeypatch.setattr(card, "_capture_jpeg", capture)
    response = queue.Queue()
    caller = threading.Thread(target=lambda: response.put(card.dispatch("capture_photo", {})))
    caller.start()
    assert entered.wait(1)
    shutdown = threading.Thread(target=lambda: (card.shutdown(), stopped.set()))
    shutdown.start()
    try:
        assert not stopped.wait(0.1), "shutdown returned before the accepted photo completed"
        assert card.dispatch("capture_photo", {})["code"] == "SHUTTING_DOWN"
    finally:
        release.set()
        shutdown.join(3)
        caller.join(3)
    assert stopped.is_set()
    result = response.get_nowait()
    assert result["state"] == "completed"
    assert Path(result["file_path"]).exists()


def test_failed_capture_returns_error_without_creating_file(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"invalid")
    result = card.dispatch("capture_photo", {})
    assert result["ok"] is False
    assert result["state"] == "error"
    assert list(tmp_path.rglob("*.jpg")) == []


def test_snapshot_rejects_invalid_camera_without_creating_file(tmp_path):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    assert card.dispatch("capture_photo", {"position": "invalid"})["code"] == "INVALID_ARGUMENT"
    assert list(tmp_path.iterdir()) == []


def test_snapshot_rejects_incomplete_camera_frame(tmp_path):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
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
    card = snapshot.VisionCapturePlugin({
        "output_dir": str(tmp_path),
        "positions": {"front": {"board_ip": "127.0.0.1", "image_port": port}},
    })
    result = completed_capture(card)
    sender.join(timeout=2)
    assert result["code"] == "CAMERA_UNAVAILABLE"
    assert list(tmp_path.iterdir()) == []


def test_snapshot_does_not_leave_a_partial_photo_when_publish_fails(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    monkeypatch.setattr(snapshot.VisionCapturePlugin, "_capture_jpeg",
                        lambda self, position: b"\xff\xd8photo\xff\xd9")

    def fail_publish(self, target):
        raise OSError("storage interrupted")

    monkeypatch.setattr(Path, "replace", fail_publish)
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    assert completed_capture(card)["code"] == "SAVE_FAILED"
    assert list((tmp_path / "photos").iterdir()) == []


def test_go1_manifest_lists_snapshot_card():
    go1_dir = Path(__file__).resolve().parents[1]
    driver = (go1_dir / "driver.yaml").read_text(encoding="utf-8")
    assert "name: vision_capture" in driver
    config = (go1_dir / "config.yaml").read_text(encoding="utf-8")
    service = (go1_dir / "deploy/service.yml").read_text(encoding="utf-8")
    assert 'output_dir: "/opt/phanthy-motus/data/vision_capture"' in config
    assert "/opt/phanthy-motus/data:/opt/phanthy-motus/data" in service


def test_go1_bundle_exposes_snapshot_tool_without_rgb(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    bundle = main.Go1Bundle({"plugins": {
        "vision_capture": {"enabled": True},
    }}, "test_go1", None, None)
    tools = {tool["name"]: tool for tool in bundle.get_all_tools()}
    assert tools["vision_capture"]["type"] == "actuator"
    assert bundle.dispatch("vision_capture", {"action": "info"})["state"] == "ready"


def test_vision_capture_uses_go1_plugin_factory():
    module = importlib.import_module("unitree.go1.vision_capture")
    card = module.make_vision_capture({}, "test", None, None)
    assert card.get_tool()["name"] == "vision_capture"


def test_named_photo_list_and_delete(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    accepted = card.dispatch("capture_photo", {"image_name": "hall_01"})
    assert accepted["state"] == "completed"
    assert Path(accepted["file_path"]).name == "hall_01.jpg"
    listing = card.dispatch("list", {})
    assert listing["files"][0]["filename"] == "hall_01.jpg"
    assert listing["files"][0]["size"] > 0
    assert card.dispatch("delete", {"name": "../hall_01.jpg"})["ok"] is False
    assert card.dispatch("delete", {"name": "hall_01.jpg"})["state"] == "deleted"
    assert not Path(accepted["file_path"]).exists()


def test_list_hides_unpublished_video_temp_file(tmp_path):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    photos = tmp_path / "photos"
    videos = tmp_path / "videos"
    photos.mkdir()
    videos.mkdir()
    (photos / "photo.jpg").write_bytes(b"\xff\xd8photo\xff\xd9")
    (videos / "video.mp4").write_bytes(b"published")
    temporary = videos / ".front_20260930_074445_842992_30c508ee.tmp.mp4"
    temporary.touch()

    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    names = {item["filename"] for item in card.dispatch("list", {})["files"]}

    assert names == {"photo.jpg", "video.mp4"}
    assert temporary.exists()


def test_named_capture_rejects_bad_names_and_existing_file(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    for name in ("../escape", "a/b", "a.jpg", " "):
        assert card.dispatch("capture_photo", {"image_name": name})["code"] == "INVALID_ARGUMENT"
    accepted = card.dispatch("capture_photo", {"image_name": "same"})
    assert card.dispatch("capture_photo", {"image_name": "same"})["code"] == "FILE_EXISTS"
    assert Path(accepted["file_path"]).exists()


def test_snapshot_returns_final_result_without_acp_completion():
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({})
    assert card.PREFIX == card.get_tool()["name"] == "vision_capture"
    schema = card.get_tool()["inputSchema"]
    assert "x-completion" not in schema
    assert "start_recording" not in schema["properties"]["action"]["enum"]
    assert "stop_recording" not in schema["properties"]["action"]["enum"]
    assert "x-resource" not in schema


def test_video_waits_for_save_and_releases_position(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    entered, release = threading.Event(), threading.Event()

    def record(position, path, duration, cancel):
        assert position == "left" and duration == 2
        entered.set()
        assert release.wait(3)
        path.parent.mkdir(parents=True)
        path.write_bytes(b"video")
        return {"ok": True, "media_type": "video", "file_path": str(path)}

    monkeypatch.setattr(snapshot.shutil, "which", lambda command: "/usr/bin/ffmpeg")
    monkeypatch.setattr(card, "_record_and_save", record)
    response = queue.Queue()
    caller = threading.Thread(target=lambda: response.put(card.dispatch("record_video", {"position": "left", "duration_s": 2})))
    caller.start()
    try:
        assert entered.wait(1)
        assert response.empty()
        assert card.dispatch("capture_photo", {"position": "left"})["code"] == "RESOURCE_BUSY"
    finally:
        release.set()
        caller.join(3)
    result = response.get_nowait()
    assert result["state"] == "completed"
    assert Path(result["file_path"]).parent == tmp_path / "videos"
    assert card.dispatch("info", {})["last_recording"]["status"] == "completed"


def test_video_failure_and_cancel_return_final_result(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(snapshot.shutil, "which", lambda command: "/usr/bin/ffmpeg")
    monkeypatch.setattr(card, "_record_and_save", lambda *args: {"ok": False, "code": "RECORD_FAILED"})
    result = card.dispatch("record_video", {})
    assert result["state"] == "error"
    entered = threading.Event()

    def cancel_record(position, path, duration, cancel):
        entered.set()
        assert cancel.wait(3)
        return {"ok": False, "code": "RECORD_CANCELLED"}

    monkeypatch.setattr(card, "_record_and_save", cancel_record)
    response = queue.Queue()
    caller = threading.Thread(target=lambda: response.put(card.dispatch("record_video", {})))
    caller.start()
    assert entered.wait(1)
    assert card.stop()["state"] == "idle"
    caller.join(3)
    result = response.get_nowait()
    assert result["code"] == "RECORD_CANCELLED"
    assert result["state"] == "cancelled"
    assert card.dispatch("info", {})["last_recording"]["status"] == "cancelled"
    assert list(tmp_path.rglob("*.mp4")) == []


def test_video_rejects_invalid_duration_and_missing_encoder(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    for value in (0, 31, 1.5, True):
        assert card.dispatch("record_video", {"duration_s": value})["code"] == "INVALID_ARGUMENT"
    monkeypatch.setattr(snapshot.shutil, "which", lambda command: None)
    assert card.dispatch("record_video", {})["code"] == "ENCODER_UNAVAILABLE"


def test_video_rejects_active_same_position_stream(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    camera = importlib.import_module("unitree.go1.camera")
    monkeypatch.setattr(camera, "_HAS_ROS2", False)
    monkeypatch.setattr(snapshot.shutil, "which", lambda command: "/usr/bin/ffmpeg")
    stream = camera.make_camera_rgb({}, "test", None, None)
    stream._node = object()
    release = threading.Event()
    monkeypatch.setattr(stream._stream_cls, "_loop", lambda *args: release.wait(3))
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    assert stream.dispatch("start", {"position": "front"})["ok"]
    try:
        busy = card.dispatch("record_video", {"position": "front"})
        assert busy["code"] == "RESOURCE_BUSY"
        assert "action_id" not in busy
        assert list(tmp_path.iterdir()) == []
    finally:
        release.set()
        stream._streams["default"]._thread.join(3)
        stream.stop()


@pytest.mark.parametrize("frame_interval", [0.05, 0.25])
def test_record_video_from_nano_frames_creates_playable_mp4(tmp_path, frame_interval):
    if not shutil.which("ffmpeg") or not shutil.which("ffprobe"):
        pytest.skip("ffmpeg and ffprobe required for encoder integration test")
    Image = pytest.importorskip("PIL.Image")
    ImageDraw = pytest.importorskip("PIL.ImageDraw")
    ImageChops = pytest.importorskip("PIL.ImageChops")
    ImageStat = pytest.importorskip("PIL.ImageStat")
    import io
    import time
    from datetime import datetime

    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def send_frames():
        try:
            connection, _ = server.accept()
            with connection:
                for index in range(40):
                    frame = Image.new("RGB", (64, 64), "red")
                    x = index * 4 % 48
                    ImageDraw.Draw(frame).rectangle((x, 24, x + 15, 39), fill="white")
                    image = io.BytesIO()
                    frame.save(image, format="JPEG")
                    jpeg = image.getvalue()
                    packet = struct.pack(">I", len(jpeg)) + jpeg
                    try:
                        connection.sendall(packet)
                    except OSError:
                        break
                    time.sleep(frame_interval)
        finally:
            server.close()

    sender = threading.Thread(target=send_frames, daemon=True)
    sender.start()
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({
        "output_dir": str(tmp_path),
        "positions": {"front": {"board_ip": "127.0.0.1", "image_port": port}},
    })
    result = card.dispatch("record_video", {"duration_s": 1, "video_name": "test_clip"})
    sender.join(3)
    assert result["state"] == "completed", result
    started = datetime.fromisoformat(result["recording_started_at"])
    ended = datetime.fromisoformat(result["recording_ended_at"])
    ready = datetime.fromisoformat(result["file_ready_at"])
    assert 0.8 <= (ended - started).total_seconds() <= 1.3
    assert ready >= ended
    path = Path(result["file_path"])
    assert path.is_file() and path.stat().st_size > 0
    assert path.name == "test_clip.mp4"
    assert list(tmp_path.rglob("*.mjpeg")) == []
    assert card.dispatch("list", {})["files"][0]["filename"] == path.name
    probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0",
                            "-show_entries", "stream=codec_name,width,height:format=duration", "-of", "json", str(path)],
                           capture_output=True, text=True, check=True)
    metadata = json.loads(probe.stdout)
    assert metadata["streams"][0]["codec_name"] == "h264"
    assert 0.8 <= float(metadata["format"]["duration"]) <= 1.2
    def video_frame(seconds):
        frame = subprocess.run(["ffmpeg", "-v", "error", "-ss", str(seconds), "-i", str(path),
                                "-frames:v", "1", "-f", "image2pipe", "-vcodec", "png", "-"],
                               capture_output=True, check=True)
        return Image.open(io.BytesIO(frame.stdout)).convert("RGB")

    # 模拟 Nano 中移动的物体：成片必须保留变化，不能只重复首帧补足时长。
    difference = ImageChops.difference(video_frame(0), video_frame(0.8))
    assert max(ImageStat.Stat(difference).mean) > 5
    assert card.dispatch("delete", {"name": path.name})["ok"] is True
    assert not path.exists()


def test_slow_encoder_does_not_starve_camera_capture(tmp_path, monkeypatch):
    import io
    import time

    snapshot = importlib.import_module("unitree.go1.vision_capture")
    monkeypatch.setattr(snapshot.shutil, "which", lambda command: "/usr/bin/ffmpeg")

    class SlowInput(io.BytesIO):
        def write(self, data):
            time.sleep(0.12)
            return len(data)

    class SlowEncoder:
        def __init__(self, args, **kwargs):
            self.path = Path(args[-1])
            self.stdin = SlowInput()
            self.stderr = io.BytesIO()
            self.returncode = None

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            self.path.write_bytes(b"mp4")
            self.returncode = 0
            return 0

        def terminate(self):
            self.returncode = -15

    monkeypatch.setattr(snapshot.subprocess, "Popen", SlowEncoder)
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def send_frames():
        try:
            connection, _ = server.accept()
            with connection:
                connection.settimeout(2)
                for index in range(40):
                    jpeg = b"\xff\xd8" + bytes([index]) * 32768 + b"\xff\xd9"
                    try:
                        connection.sendall(struct.pack(">I", len(jpeg)) + jpeg)
                    except OSError:
                        break
                    time.sleep(0.05)
        finally:
            server.close()

    sender = threading.Thread(target=send_frames, daemon=True)
    sender.start()
    card = snapshot.VisionCapturePlugin({
        "output_dir": str(tmp_path),
        "positions": {"front": {"board_ip": "127.0.0.1", "image_port": port}},
    })
    result = card.dispatch("record_video", {"duration_s": 1})
    sender.join(3)
    assert result["state"] == "completed", result
    assert result["source_frames"] >= 12


def test_record_video_drains_encoder_errors_while_feeding_frames(tmp_path, monkeypatch):
    """Continuous encoder errors must not block JPEG writes or completion."""
    import sys
    import time

    encoder = tmp_path / "encoder.py"
    encoder.write_text(
        "import pathlib, sys\n"
        "while sys.stdin.buffer.read(4096):\n"
        "    sys.stderr.buffer.write(b'x' * 8192)\n"
        "    sys.stderr.buffer.flush()\n"
        "pathlib.Path(sys.argv[1]).write_bytes(b'mp4')\n",
        encoding="utf-8",
    )
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    real_popen = subprocess.Popen
    processes = []

    def launch(args, **kwargs):
        process = real_popen([sys.executable, str(encoder), args[-1]], **kwargs)
        processes.append(process)
        return process

    monkeypatch.setattr(snapshot.subprocess, "Popen", launch)
    monkeypatch.setattr(snapshot.shutil, "which", lambda command: str(encoder))
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def send_frames():
        try:
            connection, _ = server.accept()
            with connection:
                connection.settimeout(2)
                frame = b"\xff\xd8" + b"f" * 131072 + b"\xff\xd9"
                packet = struct.pack(">I", len(frame)) + frame
                for _ in range(100):
                    try:
                        connection.sendall(packet)
                    except OSError:
                        break
                    time.sleep(0.02)
        finally:
            server.close()

    sender = threading.Thread(target=send_frames, daemon=True)
    sender.start()
    card = snapshot.VisionCapturePlugin({
        "output_dir": str(tmp_path),
        "positions": {"front": {"board_ip": "127.0.0.1", "image_port": port}},
    })
    try:
        result = card.dispatch("record_video", {"duration_s": 1})
        assert result["state"] == "completed", result
        assert Path(result["file_path"]).read_bytes() == b"mp4"
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
        sender.join(3)


@pytest.mark.parametrize("pause_launch", [False, True])
def test_stop_unblocks_encoder_finalization_and_reports_cancel(tmp_path, monkeypatch, pause_launch):
    import sys
    import time

    encoder = tmp_path / "stalled_encoder.py"
    encoder.write_text("import time\ntime.sleep(30)\n", encoding="utf-8")
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    real_popen = subprocess.Popen
    processes = []
    launched = threading.Event()
    release_launch = threading.Event()

    def launch(args, **kwargs):
        assert kwargs["stdout"] == subprocess.DEVNULL
        process = real_popen([sys.executable, str(encoder)], **kwargs)
        processes.append(process)
        launched.set()
        if pause_launch:
            assert release_launch.wait(4)
        return process

    monkeypatch.setattr(snapshot.subprocess, "Popen", launch)
    monkeypatch.setattr(snapshot.shutil, "which", lambda command: str(encoder))
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def send_frames():
        try:
            connection, _ = server.accept()
            with connection:
                frame = b"\xff\xd8" + b"f" * 131072 + b"\xff\xd9"
                packet = struct.pack(">I", len(frame)) + frame
                for _ in range(40):
                    try:
                        connection.sendall(packet)
                    except OSError:
                        break
                    time.sleep(0.05)
        finally:
            server.close()

    sender = threading.Thread(target=send_frames, daemon=True)
    sender.start()
    card = snapshot.VisionCapturePlugin({
        "output_dir": str(tmp_path),
        "positions": {"front": {"board_ip": "127.0.0.1", "image_port": port}},
    })
    stopper = None
    responses = queue.Queue()
    caller = threading.Thread(target=lambda: responses.put(card.dispatch("record_video", {"duration_s": 1})))
    try:
        caller.start()
        assert launched.wait(3)
        time.sleep(0.1)
        joining = threading.Event()
        original_join = card._recording_thread.join

        def join_recording(*args, **kwargs):
            joining.set()
            return original_join(*args, **kwargs)

        monkeypatch.setattr(card._recording_thread, "join", join_recording)
        stopper = threading.Thread(target=card.stop, daemon=True)
        stopper.start()
        if pause_launch:
            assert joining.wait(2)
            release_launch.set()
        result = responses.get(timeout=4)
        assert result["code"] == "RECORD_CANCELLED"
        assert result["state"] == "cancelled"
        assert card.dispatch("info", {})["last_recording"]["status"] == "cancelled"
        assert list(tmp_path.rglob("*.mp4")) == []
        assert list(tmp_path.rglob("*.mjpeg")) == []
        assert all(process.poll() is not None for process in processes)
        stopper.join(3)
        assert not stopper.is_alive()
    finally:
        release_launch.set()
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
        if stopper is not None:
            stopper.join(3)
        caller.join(3)
        sender.join(3)


def test_stop_before_mp4_publish_reports_cancel_without_file(tmp_path, monkeypatch):
    import io
    import time

    snapshot = importlib.import_module("unitree.go1.vision_capture")
    monkeypatch.setattr(snapshot.shutil, "which", lambda command: "/usr/bin/ffmpeg")

    class Encoder:
        stdin = None

        def __init__(self, args, **kwargs):
            self.path = Path(args[-1])
            self.stderr = io.BytesIO()
            self.returncode = 0

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            self.path.write_bytes(b"mp4")
            return 0

    monkeypatch.setattr(snapshot.subprocess, "Popen", Encoder)
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def send_frames():
        try:
            connection, _ = server.accept()
            with connection:
                packet = struct.pack(">I", 8) + b"\xff\xd8data\xff\xd9"
                for _ in range(40):
                    try:
                        connection.sendall(packet)
                    except OSError:
                        break
                    time.sleep(0.05)
        finally:
            server.close()

    sender = threading.Thread(target=send_frames, daemon=True)
    sender.start()
    card = snapshot.VisionCapturePlugin({
        "output_dir": str(tmp_path),
        "positions": {"front": {"board_ip": "127.0.0.1", "image_port": port}},
    })
    syncing = threading.Event()
    release_sync = threading.Event()
    real_fsync = snapshot.os.fsync

    def pause_sync(fd):
        syncing.set()
        assert release_sync.wait(4)
        return real_fsync(fd)

    monkeypatch.setattr(snapshot.os, "fsync", pause_sync)
    responses = queue.Queue()
    caller = threading.Thread(target=lambda: responses.put(card.dispatch("record_video", {"duration_s": 1})))
    try:
        caller.start()
        assert syncing.wait(3)
        stopper = threading.Thread(target=card.stop, daemon=True)
        stopper.start()
        assert card._recording.wait(2)
        release_sync.set()
        result = responses.get(timeout=3)
        assert result["state"] == "cancelled"
        assert result["code"] == "RECORD_CANCELLED"
        assert list(tmp_path.rglob("*.mp4")) == []
        stopper.join(3)
        assert not stopper.is_alive()
    finally:
        release_sync.set()
        caller.join(3)
        sender.join(3)


@pytest.mark.parametrize("error, code", [
    (TimeoutError("Nano timed out"), "CAMERA_UNAVAILABLE"),
    (RuntimeError("worker failed"), "CAPTURE_FAILED"),
])
def test_snapshot_failure_completes_and_releases_position(tmp_path, monkeypatch, error, code):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})

    def fail(position):
        raise error

    monkeypatch.setattr(card, "_capture_jpeg", fail)
    assert completed_capture(card)["code"] == code
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    assert completed_capture(card)["ok"] is True


def test_capture_waits_for_network_and_blocks_same_position(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
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
        assert not returned.wait(0.1), "dispatch returned before the Nano finished"
        stopped = card.dispatch("stop", {})
        assert stopped["state"] == "idle"
        assert stopped["capture_active"] is True
        assert card.dispatch("info", {})["state"] == "capturing"
        busy = card.dispatch("capture_photo", {})
        assert busy["code"] == "RESOURCE_BUSY"
        assert "action_id" not in busy
    finally:
        release.set()
        caller.join(3)
    assert returned.is_set()
    assert results[0]["state"] == "completed"


@pytest.mark.parametrize("camera_type", ["rgb", "depth", "pointcloud"])
def test_snapshot_rejects_active_same_position_stream(tmp_path, monkeypatch, camera_type):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    camera = importlib.import_module("unitree.go1.camera")
    monkeypatch.setattr(camera, "_HAS_ROS2", False)
    plugin = camera.Plugin({}, "test", None, None, camera_type)
    plugin._node = object()
    release = threading.Event()
    monkeypatch.setattr(plugin._stream_cls, "_loop", lambda *args: release.wait(3))
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    assert plugin.dispatch("start", {"position": "left"})["ok"]
    try:
        busy = card.dispatch("capture_photo", {"position": "left"})
        assert busy["code"] == "RESOURCE_BUSY"
        assert "action_id" not in busy
        assert completed_capture(card, "right")["ok"]
        # stop 返回时接收线程可能还在退出；关闭连接前仍须保留占用。
        plugin.dispatch("stop", {})
        assert card.dispatch("capture_photo", {"position": "left"})["code"] == "RESOURCE_BUSY"
    finally:
        release.set()
        plugin._streams["default"]._thread.join(3)
        plugin.stop()
    assert completed_capture(card, "left")["ok"]


def test_stream_cannot_start_during_snapshot(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    camera = importlib.import_module("unitree.go1.camera")
    monkeypatch.setattr(camera, "_HAS_ROS2", False)
    plugin = camera.make_camera_rgb({}, "test", None, None)
    plugin._node = object()
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    release = threading.Event()

    def capture(position):
        assert release.wait(3)
        return b"\xff\xd8photo\xff\xd9"

    monkeypatch.setattr(card, "_capture_jpeg", capture)
    responses = queue.Queue()
    caller = threading.Thread(target=lambda: responses.put(card.dispatch("capture_photo", {"position": "front"})))
    caller.start()
    try:
        assert card.dispatch("info", {})["state"] == "capturing"
        assert plugin.dispatch("start", {"position": "front"})["code"] == "RESOURCE_BUSY"
        assert plugin._streams == {}
    finally:
        release.set()
        caller.join(3)
    assert responses.get_nowait()["state"] == "completed"


@pytest.mark.parametrize("source", ["camera_rgb", "camera_depth"])
def test_bundle_snapshot_inherits_camera_positions_even_when_stream_disabled(monkeypatch, source):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    bundle = main.Go1Bundle({"plugins": {
        source: {"enabled": False, "positions": {"left": {"board_ip": "10.0.0.8", "image_port": 9923}}},
        "vision_capture": {"enabled": True, "positions": {"left": {"image_port": 9924}}},
    }}, "test_go1", None, None)
    card = next(p for p in bundle._plugins if p.get_tool()["name"] == "vision_capture")
    assert card._endpoints["left"] == ("10.0.0.8", 9924)


def test_snapshot_and_rgb_have_identical_default_endpoints():
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    camera = importlib.import_module("unitree.go1.camera")
    card = snapshot.VisionCapturePlugin({})
    rgb = camera.make_camera_rgb({}, "test", None, None)
    assert card._endpoints == {p: (cfg["board_ip"], cfg["image_port"]) for p, cfg in rgb._positions.items()}


@pytest.mark.parametrize("action", ["capture_photo", "record_video"])
def test_final_result_needs_no_core_notification(tmp_path, monkeypatch, action):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(snapshot.shutil, "which", lambda command: "/usr/bin/ffmpeg")
    monkeypatch.setattr(urllib.request, "urlopen", lambda *args, **kwargs: pytest.fail("unexpected Core request"))
    if action == "capture_photo":
        monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    else:
        def save(position, path, duration, cancel):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"video")
            return {"ok": True, "media_type": "video", "file_path": str(path), "filename": path.name}
        monkeypatch.setattr(card, "_record_and_save", save)
    result = card.dispatch(action, {})
    assert result["state"] == "completed"
    assert Path(result["file_path"]).exists()


@pytest.mark.parametrize("failure_point", ["construct", "start"])
@pytest.mark.parametrize("action", ["capture_photo", "record_video"])
def test_worker_failure_does_not_leave_occupancy(tmp_path, monkeypatch,
                                                 failure_point, action):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    card = snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)})
    monkeypatch.setattr(snapshot.shutil, "which", lambda command: "/usr/bin/ffmpeg")

    def fail_worker(*args, **kwargs):
        raise RuntimeError("cannot create worker")

    with monkeypatch.context() as patch:
        if failure_point == "construct":
            patch.setattr(threading, "Thread", fail_worker)
        else:
            patch.setattr(threading.Thread, "start", fail_worker)
        result = card.dispatch(action, {})
    assert result["code"] == ("CAPTURE_FAILED" if action == "capture_photo" else "RECORD_FAILED")
    assert "action_id" not in result
    assert card.dispatch("info", {})["state"] == "ready"
    monkeypatch.setattr(card, "_capture_jpeg", lambda position: b"\xff\xd8photo\xff\xd9")
    assert completed_capture(card)["ok"]


def test_slow_fragments_cannot_reset_frame_deadline(monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    times = iter([0.0, 0.4, 1.1])
    timeouts = []

    class Connection:
        def settimeout(self, value):
            timeouts.append(value)

        def recv(self, size):
            return b"x"

    monkeypatch.setattr(snapshot.time, "monotonic", lambda: next(times))
    with pytest.raises(TimeoutError, match="deadline"):
        snapshot.VisionCapturePlugin._receive_exact(Connection(), 3, 1.0)
    assert timeouts == [1.0, 0.6]


def test_simultaneous_snapshots_reserve_position_atomically(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.vision_capture")
    cards = [snapshot.VisionCapturePlugin({"output_dir": str(tmp_path)}) for _ in range(2)]
    start = threading.Barrier(3)
    release = threading.Event()
    results = queue.Queue()

    def capture(self, position):
        assert release.wait(3)
        return b"\xff\xd8photo\xff\xd9"

    def call(card):
        start.wait(3)
        results.put(card.dispatch("capture_photo", {}))

    monkeypatch.setattr(snapshot.VisionCapturePlugin, "_capture_jpeg", capture)
    callers = [threading.Thread(target=call, args=(card,)) for card in cards]
    for caller in callers:
        caller.start()
    try:
        start.wait(3)
        busy = results.get(timeout=2)
        assert busy["code"] == "RESOURCE_BUSY"
    finally:
        release.set()
        for caller in callers:
            caller.join(3)
    assert results.get_nowait()["state"] == "completed"


def test_bundle_stream_and_snapshot_share_occupancy_and_reject_busy_hot_switch(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    bundle = main.Go1Bundle({"plugins": {
        "camera_rgb": {"enabled": True},
        "vision_capture": {"enabled": True, "output_dir": str(tmp_path)},
    }}, "test_go1", None, None)
    plugins = {p.get_tool()["name"]: p for p in bundle._plugins}
    rgb, snapshot = plugins["camera_rgb"], plugins["vision_capture"]
    camera = importlib.import_module("camera")
    monkeypatch.setattr(camera, "_HAS_ROS2", False)
    rgb._node = object()
    stream_release, capture_release, capture_entered = threading.Event(), threading.Event(), threading.Event()
    monkeypatch.setattr(rgb._stream_cls, "_loop", lambda *args: stream_release.wait(3))

    def capture(position):
        capture_entered.set()
        assert capture_release.wait(3)
        return b"\xff\xd8photo\xff\xd9"

    monkeypatch.setattr(snapshot, "_capture_jpeg", capture)
    bundle.dispatch("camera_rgb", {"action": "config", "position": "left"})
    assert bundle.dispatch("camera_rgb", {"action": "start"})["ok"]
    stream = rgb._streams["default"]
    responses = queue.Queue()
    caller = threading.Thread(target=lambda: responses.put(bundle.dispatch("vision_capture", {"action": "capture_photo", "position": "right"})))
    try:
        assert bundle.dispatch("vision_capture", {"action": "capture_photo", "position": "left"})["code"] == "RESOURCE_BUSY"
        caller.start()
        assert capture_entered.wait(1)
        result = bundle.dispatch("camera_rgb", {"action": "config", "position": "right"})
        assert result["code"] == "RESOURCE_BUSY"
        assert rgb._cfg["default"]["position"] == "left"
        assert rgb._streams["default"] is stream
        assert stream._run
    finally:
        capture_release.set()
        caller.join(3)
        stream_release.set()
        stream._thread.join(3)
        bundle.stop_all()
    assert responses.get_nowait()["state"] == "completed"


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
