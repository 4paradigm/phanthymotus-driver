"""Exercise photo capture with synthetic RGB frames and a temporary data mount."""

import importlib.util
from pathlib import Path
from queue import Queue
import threading
from types import SimpleNamespace

import pytest
import yaml

DRIVER = Path(__file__).resolve().parent
ROOT = DRIVER.parents[1]


def _load(name):
    spec = importlib.util.spec_from_file_location(f"panther_capture_{name}", DRIVER / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def capture(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT))
    monkeypatch.syspath_prepend(str(DRIVER))
    cfg = yaml.safe_load((DRIVER / "config.yaml").read_text(encoding="utf-8"))
    cfg["plugins"] = {"vision_capture": {**cfg["plugins"]["vision_capture"], "output_dir": str(tmp_path)}}
    runtime = object.__new__(_load("camera").PantherCameraRuntime)
    runtime._lock = threading.RLock()
    runtime._photo_ready = threading.Condition(runtime._lock)
    runtime._photo_waiting = False
    runtime._photo_frame = None
    runtime._active = set()
    runtime._closed = False
    runtime._config = {}
    runtime._frames = {"rgb": None}
    runtime._last_frame_at = {"rgb": 0.0}
    runtime._node = SimpleNamespace(count_publishers=lambda _: 0)
    device = object.__new__(_load("device").PantherDevice)
    device._cfg, device._camera, device._node = cfg, runtime, runtime._node
    from common.vendor_runtime import DriverBundle

    return DriverBundle(_load("plugins").build_plugins(cfg, "test", device)), runtime


def test_capture_saves_next_rgb_frame_without_consuming_preview(capture, tmp_path):
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    bundle, runtime = capture
    photo, = bundle.get_all_tools()
    assert photo["name"] == "vision_capture"
    assert "capture_photo" in photo["inputSchema"]["properties"]["action"]["enum"]

    def frame(sec, color):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id="rgb", stamp=SimpleNamespace(sec=sec, nanosec=0)),
            encoding="rgb8", width=16, height=16, step=48, data=bytes(color) * 256,
        )

    runtime._active.add("rgb")
    runtime._on_rgb(frame(10, (0, 0, 255)))
    waiting = threading.Event()
    runtime._node.count_publishers = lambda _: waiting.set() or 1
    result = Queue()
    worker = threading.Thread(target=lambda: result.put(bundle.dispatch("vision_capture", {"action": "capture_photo"})))
    worker.start()
    try:
        assert waiting.wait(timeout=1.0)
        new_frame = frame(11, (255, 0, 0))
        runtime._on_rgb(new_frame)
        output = result.get(timeout=3.0)
        assert output["ok"], output
        assert output["stamp"] == {"sec": 11, "nanosec": 0}
        path = Path(output["file_path"])
        assert path.parent == tmp_path / "photos"
        image = cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
        assert image[8, 8, 2] > 240 and image[8, 8, 0] < 10
        assert runtime._frames["rgb"] is new_frame
        assert list(path.parent.iterdir()) == [path]
    finally:
        worker.join(timeout=4.0)


def test_capture_reports_missing_publisher_and_does_not_write_a_file(capture, tmp_path):
    bundle, _runtime = capture
    assert bundle.dispatch("vision_capture", {"action": "start"})["state"] == "waiting_for_camera"
    result = bundle.dispatch("vision_capture", {"action": "capture_photo"})
    assert result["code"] == "CAPTURE_FAILED"
    assert "start camera_color first" in result["message"]
    assert not (tmp_path / "photos").exists()


def test_publisher_without_fresh_frames_times_out_and_releases_capture_slot(capture):
    _, runtime = capture
    runtime._node.count_publishers = lambda _: 1
    for _ in range(2):
        with pytest.raises(TimeoutError, match="no new RGB frame"):
            runtime.capture_jpeg(timeout_s=0.01)
