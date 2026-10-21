import time
import threading
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import types
from pathlib import Path

import pytest

from test_mic_card import _load_device


@pytest.fixture
def plugin(monkeypatch, tmp_path):
    module, _ = _load_device(monkeypatch)
    return module.VisionCapturePlugin({"output_dir": str(tmp_path / "captures")}, None)


def put(plugin, relative, content=b"saved media"):
    path = plugin._output_dir / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return path


def test_declares_read_only_listing_without_changing_capture_contract(plugin):
    schema = plugin.get_tool()["inputSchema"]
    assert schema["x-file-download"] == {"protocol": "motus.files/1", "action": "list_files"}
    assert set(schema["properties"]["action"]["enum"]) == {
        "start", "stop", "info", "capture_photo", "record_video", "list_files"}
    assert schema["x-completion"] == {"actions": ["record_video"], "timeout": 45}
    assert plugin.dispatch("list_files", {}) == {"ok": True, "files": []}
    assert not plugin._output_dir.exists()


def test_history_excludes_deleted_empty_and_active_files_without_camera(plugin):
    photo = put(plugin, "photos/IMG_old.jpg")
    video = put(plugin, "videos/video_complete.mp4", b"completed video")
    active = put(plugin, "videos/video_active.mp4", b"partial video")
    put(plugin, "photos/empty.jpg", b"")
    put(plugin, "photos/notes.txt")
    plugin._active_recording = {"path": str(active)}
    files = plugin.dispatch("list_files", {})["files"]
    assert {f["name"] for f in files} == {photo.name, video.name}
    assert next(f for f in files if f["name"] == photo.name) == {
        "name": photo.name, "path": str(photo), "size": 11, "mime_type": "image/jpeg"}
    assert next(f for f in files if f["name"] == video.name)["mime_type"] == "video/mp4"
    photo.unlink()
    assert [f["name"] for f in plugin.dispatch("list_files", {})["files"]] == [video.name]


def test_recording_started_during_enumeration_is_excluded(plugin, monkeypatch):
    active = put(plugin, "videos/video_active.mp4")
    real_stat = Path.stat

    def stat(path, *args, **kwargs):
        result = real_stat(path, *args, **kwargs)
        if path == active:
            with plugin._recording_lock:
                plugin._active_recording = {"path": str(active)}
        return result

    monkeypatch.setattr(Path, "stat", stat)
    assert plugin.dispatch("list_files", {}) == {"ok": True, "files": []}


def test_concurrent_deletion_is_skipped(plugin, monkeypatch):
    gone = put(plugin, "photos/gone.jpg")
    keep = put(plugin, "photos/keep.jpg")
    real_stat = Path.stat

    def stat(path, *args, **kwargs):
        if path == gone:
            gone.unlink(missing_ok=True)
            raise FileNotFoundError(str(gone))
        return real_stat(path, *args, **kwargs)

    monkeypatch.setattr(Path, "stat", stat)
    assert [f["name"] for f in plugin.dispatch("list_files", {})["files"]] == [keep.name]


def test_original_photo_capture_still_saves_the_same_result(plugin):
    jpeg = b"\xff\xd8test-photo\xff\xd9"
    plugin._worker = types.SimpleNamespace(
        is_running=lambda: True,
        wait_for_color_frame=lambda *args: ({"data": jpeg, "timestamp_ms": time.time() * 1000}, 1))
    result = plugin.dispatch("capture_photo", {})
    assert result["ok"] and result["media_type"] == "photo"
    assert Path(result["file_path"]).read_bytes() == jpeg
    assert [f["path"] for f in plugin.dispatch("list_files", {})["files"]] == [result["file_path"]]
    assert plugin.dispatch("record_video", {"duration_s": 31})["code"] == "INVALID_DURATION"
    assert plugin.dispatch("stop", {})["state"] == "idle"


def test_directory_read_failure_is_not_empty_history(plugin, monkeypatch):
    put(plugin, "photos/saved.jpg")
    real_iterdir = Path.iterdir

    def iterdir(path):
        if path == plugin._output_dir / "photos":
            raise PermissionError("history directory unreadable")
        return real_iterdir(path)

    monkeypatch.setattr(Path, "iterdir", iterdir)
    result = plugin.dispatch("list_files", {})
    assert result["ok"] is False
    assert result["code"] == "LIST_FILES_FAILED"
    assert "unreadable" in result["message"]


def test_photo_is_listed_only_after_writing_finishes(plugin, monkeypatch):
    jpeg = b"complete-photo"
    plugin._worker = types.SimpleNamespace(
        is_running=lambda: True,
        wait_for_color_frame=lambda *args: ({"data": jpeg, "timestamp_ms": time.time() * 1000}, 1))
    partial_written, finish_write, listing_started = threading.Event(), threading.Event(), threading.Event()
    real_open = Path.open

    @contextmanager
    def slow_open(path, *args, **kwargs):
        with real_open(path, *args, **kwargs) as output:
            def write(data):
                output.write(data[:3])
                output.flush()
                partial_written.set()
                assert finish_write.wait(3)
                output.write(data[3:])
            yield types.SimpleNamespace(write=write)

    monkeypatch.setattr(Path, "open", slow_open)

    def listing():
        listing_started.set()
        return plugin.dispatch("list_files", {})

    with ThreadPoolExecutor(max_workers=2) as executor:
        capture = executor.submit(plugin.dispatch, "capture_photo", {})
        try:
            assert partial_written.wait(3)
            history = executor.submit(listing)
            assert listing_started.wait(3)
            time.sleep(0.05)
            assert not history.done(), "in-progress JPG must not be offered for download"
        finally:
            finish_write.set()
        result = capture.result(timeout=3)
        assert result["ok"]
        files = history.result(timeout=3)["files"]
        assert [file["size"] for file in files] == [len(jpeg)]


def test_failed_photo_close_does_not_leave_downloadable_partial_file(plugin, monkeypatch):
    plugin._worker = types.SimpleNamespace(
        is_running=lambda: True,
        wait_for_color_frame=lambda *args: ({"data": b"partial", "timestamp_ms": time.time() * 1000}, 1))
    real_open = Path.open

    @contextmanager
    def failed_close(path, *args, **kwargs):
        with real_open(path, *args, **kwargs) as output:
            yield output
        raise OSError("flush failed: disk full")

    monkeypatch.setattr(Path, "open", failed_close)
    assert plugin.dispatch("capture_photo", {})["code"] == "CAPTURE_FAILED"
    assert plugin.dispatch("list_files", {}) == {"ok": True, "files": []}


def test_photo_name_collision_keeps_existing_file(plugin, monkeypatch):
    plugin._worker = types.SimpleNamespace(
        is_running=lambda: True,
        wait_for_color_frame=lambda *args: ({"data": b"new photo", "timestamp_ms": time.time() * 1000}, 1))
    existing = put(plugin, "photos/IMG_fixed.jpg", b"existing photo")
    monkeypatch.setitem(plugin._capture_photo.__func__.__globals__, "datetime", types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(strftime=lambda fmt: "fixed")))
    assert plugin.dispatch("capture_photo", {})["code"] == "CAPTURE_FAILED"
    assert existing.read_bytes() == b"existing photo"
