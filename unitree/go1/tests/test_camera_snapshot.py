"""Go1 snapshot card: fresh RGB frames and persistent JPEG output."""

import importlib
import threading
from pathlib import Path


def test_rgb_stream_waits_for_a_new_frame():
    camera = importlib.import_module("unitree.go1.camera")
    stream = camera._RgbStream(None, "/test/rgb")
    stream._run = True
    sequence = stream.frame_sequence()
    timer = threading.Timer(0.02, stream._note_frame, args=(b"\xff\xd8new\xff\xd9",))
    timer.start()
    try:
        assert stream.wait_for_frame(sequence, timeout_s=1) == b"\xff\xd8new\xff\xd9"
    finally:
        timer.join()
    assert stream.wait_for_frame(stream.frame_sequence(), timeout_s=0.02) is None


def test_snapshot_saves_selected_position_jpeg(tmp_path):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")

    class RunningStream:
        def frame_sequence(self):
            return 4

        def wait_for_frame(self, sequence, timeout_s):
            assert sequence == 4
            return b"\xff\xd8photo\xff\xd9"

    class RgbCard:
        def running_stream(self, position):
            assert position == "left"
            return RunningStream()

    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)}, RgbCard())
    result = card.dispatch("capture_photo", {"position": "left"})
    assert result["ok"] is True
    assert result["position"] == "left"
    assert Path(result["file_path"]).read_bytes() == b"\xff\xd8photo\xff\xd9"


def test_snapshot_rejects_unavailable_or_stalled_camera(tmp_path):
    snapshot = importlib.import_module("unitree.go1.camera_snapshot")

    class RgbCard:
        def running_stream(self, position):
            return None

    card = snapshot.CameraSnapshotPlugin({"output_dir": str(tmp_path)}, RgbCard())
    assert card.dispatch("capture_photo", {"position": "front"})["ok"] is False
    assert card.dispatch("capture_photo", {"position": "invalid"})["code"] == "INVALID_ARGUMENT"
    assert list(tmp_path.rglob("*.jpg")) == []


def test_go1_manifest_lists_snapshot_card():
    driver = (Path(__file__).resolve().parents[1] / "driver.yaml").read_text(encoding="utf-8")
    assert "name: camera_snapshot" in driver


def test_go1_bundle_exposes_snapshot_tool(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    bundle = main.Go1Bundle({"plugins": {
        "camera_rgb": {"enabled": True},
        "camera_snapshot": {"enabled": True},
    }}, "test_go1", None, None)
    tools = {tool["name"]: tool for tool in bundle.get_all_tools()}
    assert tools["camera_snapshot"]["type"] == "actuator"
    assert bundle.dispatch("camera_snapshot", {"action": "info"})["state"] == "ready"
