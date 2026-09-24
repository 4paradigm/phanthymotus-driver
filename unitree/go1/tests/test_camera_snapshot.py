"""Go1 snapshot card: direct Nano capture and persistent JPEG output."""

import importlib
import socket
import struct
import threading
from pathlib import Path


def test_snapshot_connects_to_selected_camera_and_saves_jpeg(tmp_path):
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
        result = card.dispatch("capture_photo", {"position": "left"})
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


def test_snapshot_rejects_incomplete_camera_frame(tmp_path):
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
    result = card.dispatch("capture_photo", {"position": "front"})
    sender.join(timeout=2)
    assert result["code"] == "CAMERA_UNAVAILABLE"
    assert list(tmp_path.iterdir()) == []


def test_snapshot_does_not_leave_a_partial_photo_when_publish_fails(tmp_path, monkeypatch):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")
    monkeypatch.setattr(snapshot.CameraSnapshotPlugin, "_capture_jpeg",
                        lambda self, position: b"\xff\xd8photo\xff\xd9")

    def fail_publish(self, target):
        raise OSError("storage interrupted")

    monkeypatch.setattr(Path, "replace", fail_publish)
    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)})
    assert card.dispatch("capture_photo", {"position": "front"})["code"] == "SAVE_FAILED"
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
