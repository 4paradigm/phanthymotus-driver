"""Offline media management checks; no ROS or robot devices are used."""

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import errno
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from test_av_recording import load_plugin


class VisionCaptureFilesTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        # macOS /var is itself a link; use a genuine directory hierarchy.
        self.root = Path(self.temp.name).resolve() / "media"
        self.p = load_plugin()({}, "g1", None, None)
        self.p._output_dir = self.root

    def put(self, kind, name, content=b"saved", timestamp=1_700_000_000):
        directory = self.root / ("photos" if kind == "photo" else "videos")
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / name
        path.write_bytes(content)
        os.utime(path, (timestamp, timestamp))
        return path

    def call(self, action, **args):
        return self.p.dispatch(action, args)

    def test_schema_preserves_existing_actions_and_exposes_file_parameters(self):
        schema = self.p.get_tool()["inputSchema"]
        self.assertTrue({"capture_photo", "record_video", "stop", "list", "delete"}
                        <= set(schema["properties"]["action"]["enum"]))
        self.assertTrue(schema["properties"]["include_audio"]["default"])
        self.assertEqual(["media_type", "filename"], schema["x-action-params"]["delete"]["params"])

    def test_list_empty_missing_directory_does_not_create_files(self):
        self.assertEqual([], self.call("list")["files"])
        self.assertFalse(self.root.exists())

    def test_list_metadata_sort_type_filter_and_limit(self):
        old = self.put("photo", "old.jpg", timestamp=1_700_000_000)
        new = self.put("video", "new.mp4", timestamp=1_700_000_002)
        self.put("photo", "other.JPG", timestamp=1_700_000_001)
        result = self.call("list", limit=2)
        self.assertTrue(result["ok"])
        self.assertTrue(result["truncated"])
        self.assertEqual(["new.mp4", "other.JPG"], [f["filename"] for f in result["files"]])
        item = result["files"][0]
        self.assertEqual("video", item["media_type"])
        self.assertEqual("video/mp4", item["mime"])
        self.assertEqual(new.stat().st_size, item["size"])
        self.assertEqual(str(new), item["file_path"])
        self.assertIsNotNone(datetime.fromisoformat(item["modified_at"]).tzinfo)
        self.assertNotIn("_mtime_ns", item)
        photos = self.call("list", media_type="photo")
        self.assertEqual({old.name, "other.JPG"}, {f["filename"] for f in photos["files"]})
        self.assertFalse(photos["truncated"])

    def test_invalid_filter_and_limits(self):
        self.assertEqual("INVALID_MEDIA_TYPE", self.call("list", media_type="../videos")["code"])
        for limit in [0, 201, True, "10", 1.5, None]:
            with self.subTest(limit=limit):
                self.assertEqual("INVALID_LIMIT", self.call("list", limit=limit)["code"])

    def test_list_excludes_temp_empty_directories_and_links(self):
        photo = self.put("photo", "good.jpg")
        self.put("photo", ".partial.jpg")
        self.put("photo", "empty.jpg", b"")
        self.put("photo", "other.jpg.part")
        (photo.parent / "dir.jpg").mkdir()
        (photo.parent / "link.jpg").symlink_to(photo)
        if hasattr(os, "mkfifo"):
            os.mkfifo(photo.parent / "pipe.jpg")
        self.assertEqual(["good.jpg"], [f["filename"] for f in self.call("list")["files"]])

    def test_delete_only_requested_file_and_repeated_delete_is_explicit(self):
        target = self.put("photo", "one.jpg")
        keep = self.put("video", "one.mp4")
        result = self.call("delete", media_type="photo", filename=target.name)
        self.assertTrue(result["ok"])
        self.assertEqual("deleted", result["state"])
        self.assertFalse(target.exists())
        self.assertTrue(keep.exists())
        self.assertEqual("FILE_NOT_FOUND", self.call("delete", media_type="photo", filename=target.name)["code"])

    def test_delete_rejects_paths_wrong_extension_and_missing_type(self):
        keep = self.put("photo", "safe.jpg")
        for name in ["../safe.jpg", str(keep), "sub/safe.jpg", r"..\safe.jpg",
                     ".hidden.jpg", "x.mp4", None, 1, "x\x00.jpg", "a"*210+".jpg"]:
            with self.subTest(name=name):
                self.assertEqual("INVALID_FILENAME", self.call("delete", media_type="photo", filename=name)["code"])
        for kind in [None, "all", "photos", "../photos"]:
            self.assertEqual("INVALID_MEDIA_TYPE", self.call("delete", media_type=kind, filename="safe.jpg")["code"])
        self.assertTrue(keep.exists())

    def test_delete_rejects_file_link_and_directory(self):
        target = self.put("photo", "safe.jpg")
        link = target.parent / "link.jpg"
        link.symlink_to(target)
        (target.parent / "dir.jpg").mkdir()
        self.assertEqual("UNSAFE_PATH", self.call("delete", media_type="photo", filename=link.name)["code"])
        self.assertEqual("NOT_A_FILE", self.call("delete", media_type="photo", filename="dir.jpg")["code"])
        self.assertTrue(link.is_symlink())
        self.assertTrue(target.exists())

    def test_directory_or_root_symlinks_are_rejected(self):
        outside = Path(self.temp.name).resolve() / "outside"
        outside.mkdir(); (outside / "keep.jpg").write_bytes(b"keep")
        self.root.mkdir()
        (self.root / "photos").symlink_to(outside, target_is_directory=True)
        self.assertEqual("UNSAFE_PATH", self.call("list")["code"])
        self.assertEqual("UNSAFE_PATH", self.call("delete", media_type="photo", filename="keep.jpg")["code"])
        self.p._output_dir = self.root / "photos" / "nested"
        self.assertEqual("UNSAFE_PATH", self.call("list")["code"])
        self.assertEqual(b"keep", (outside / "keep.jpg").read_bytes())

    def test_open_directory_fd_survives_symlink_swap_without_escaping(self):
        target = self.put("photo", "same.jpg")
        outside = Path(self.temp.name).resolve() / "outside"
        outside.mkdir(); keep = outside / "same.jpg"; keep.write_bytes(b"outside")
        open_dir = self.p._open_media_directory
        def swap(kind):
            fd = open_dir(kind)
            target.parent.rename(self.root / "original-photos")
            target.parent.symlink_to(outside, target_is_directory=True)
            return fd
        self.p._open_media_directory = swap
        self.assertTrue(self.call("delete", media_type="photo", filename=target.name)["ok"])
        self.assertEqual(b"outside", keep.read_bytes())
        self.assertFalse((self.root / "original-photos" / target.name).exists())

    def test_recording_and_encoding_files_are_hidden_and_protected(self):
        self.put("video", "old.mp4")
        active = self.put("video", "active.mp4")
        for state in ["recording", "encoding", "stopping"]:
            self.p._active_recording = {"state": state, "path": str(active), "finished": False}
            result = self.call("list", media_type="video")
            self.assertEqual(["old.mp4"], [f["filename"] for f in result["files"]])
            self.assertEqual("MEDIA_BUSY", self.call("delete", media_type="video", filename=active.name)["code"])
        self.p._active_recording = {"state": "recording", "path": None}
        self.assertEqual("MEDIA_BUSY", self.call("delete", media_type="video", filename="old.mp4")["code"])
        self.p._active_recording = None
        self.assertTrue(self.call("delete", media_type="video", filename=active.name)["ok"])

    def test_permission_errors_are_reported_and_do_not_delete(self):
        target = self.put("photo", "keep.jpg")
        with patch.object(os, "unlink", side_effect=PermissionError(errno.EACCES, "permission denied")):
            self.assertEqual("PERMISSION_DENIED", self.call("delete", media_type="photo", filename=target.name)["code"])
        with patch.object(self.p, "_open_media_directory", side_effect=PermissionError(errno.EACCES, "permission denied")):
            self.assertEqual("PERMISSION_DENIED", self.call("list")["code"])
        self.assertTrue(target.exists())

    def test_concurrent_deletion_has_one_success_and_one_not_found(self):
        self.put("photo", "one.jpg")
        barrier = threading.Barrier(2)
        def delete():
            barrier.wait(timeout=2)
            return self.call("delete", media_type="photo", filename="one.jpg")
        with ThreadPoolExecutor(max_workers=2) as executor:
            results = list(executor.map(lambda _: delete(), range(2)))
        self.assertEqual(1, sum(r["ok"] for r in results))
        self.assertEqual(["FILE_NOT_FOUND"], [r["code"] for r in results if not r["ok"]])

    def test_external_removal_during_list_is_skipped(self):
        self.put("photo", "vanish.jpg")
        original = os.stat
        def stat(path, *args, **kwargs):
            if path == "vanish.jpg": raise FileNotFoundError(path)
            return original(path, *args, **kwargs)
        with patch.object(os, "stat", side_effect=stat):
            self.assertEqual([], self.call("list")["files"])

    def test_delete_waits_for_photo_write_to_finish(self):
        entered = threading.Event(); release = threading.Event(); deleted = threading.Event()
        self.p._camera = object()
        self.p._frame = lambda: ({"data": b"jpeg", "received_monotonic": 0}, 1)
        original = self.p._reserve_media_file
        def reserve(kind, filename):
            path, handle = original(kind, filename)
            class Writer:
                def __enter__(self): return self
                def __exit__(self, *args): handle.close()
                def write(self, data):
                    entered.set()
                    if not release.wait(3): raise TimeoutError("test writer was not released")
                    return handle.write(data)
            return path, Writer()
        with patch.object(self.p, "_reserve_media_file", reserve), ThreadPoolExecutor(max_workers=2) as executor:
            photo = executor.submit(self.p._capture_photo)
            try:
                self.assertTrue(entered.wait(2))
                target = next((self.root / "photos").iterdir())
                def remove():
                    result = self.call("delete", media_type="photo", filename=target.name)
                    deleted.set(); return result
                deletion = executor.submit(remove)
                self.assertFalse(deleted.wait(.05))
            finally:
                release.set()
            self.assertTrue(photo.result(timeout=2)["ok"])
            self.assertTrue(deletion.result(timeout=2)["ok"])

    def test_photo_name_collision_never_removes_existing_file(self):
        self.p._camera = object()
        self.p._frame = lambda: ({"data": b"jpeg", "received_monotonic": 0}, 1)
        keep = self.put("photo", "same.jpg", b"existing")
        result = self.p._capture_photo({"image_name": "same"})
        self.assertFalse(result["ok"])
        self.assertEqual("FILE_EXISTS", result["code"])
        self.assertEqual(b"existing", keep.read_bytes())


if __name__ == "__main__":
    unittest.main()
