"""Local atomic clip directories; no upload, arbitrary path reads, or deletion."""
from __future__ import annotations
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import threading
from uuid import uuid4

from telemetry_recorder_report import render_report


def metadata(doc):
    return {key: doc[key] for key in (
        "schema", "clip_id", "label", "trigger_utc", "requested_pre_s",
        "requested_post_s", "post_window_elapsed", "interrupted", "truncated",
        "alignment", "quality", "limitations", "receiver_totals_at_export")}


class ClipStore:
    MAX_JSON_BYTES = 16_000_000

    def __init__(self, directory, max_disk_bytes=64_000_000, max_clips=100):
        if not isinstance(directory, str) or not directory.strip():
            raise ValueError("output_dir is required")
        self.root = Path(directory).expanduser().resolve()
        if type(max_disk_bytes) is not int or not 1024 <= max_disk_bytes <= 1_000_000_000:
            raise ValueError("max_disk_bytes must be an integer in [1024, 1000000000]")
        if type(max_clips) is not int or not 1 <= max_clips <= 200:
            raise ValueError("max_clips must be an integer in [1, 200]")
        self.max_disk_bytes, self.max_clips = max_disk_bytes, max_clips
        self._lock = threading.Lock()

    def prepare(self):
        self.root.mkdir(parents=True, exist_ok=True)
        # Fail at startup rather than say "ready" with an unwritable directory.
        # Avoid Windows delete-on-close semantics: a denied temporary-file open
        # can make tempfile retry thousands of times in restricted workspaces.
        probe_path = self.root / (".write-probe-" + uuid4().hex)
        created = False
        try:
            with probe_path.open("xb") as probe:
                created = True
                probe.write(b"probe")
                probe.flush()
        finally:
            if created:
                probe_path.unlink()

    def _entries(self):
        entries = list(self.root.iterdir())
        if len(entries) > 400:
            raise ValueError("output directory has too many entries; use a dedicated directory")
        return entries

    def _usage(self):
        size, count = 0, 0
        for entry in self._entries():
            if entry.is_symlink() or not entry.is_dir():
                raise ValueError("output directory must contain only recorder clip directories")
            if not (re.fullmatch(r"[0-9a-f]{32}", entry.name) or entry.name.startswith(".pending-")):
                raise ValueError("unrecognized entry in output directory")
            children = list(entry.iterdir())
            if len(children) > 4:
                raise ValueError("unrecognized clip directory contents")
            for child in children:
                if child.is_symlink() or not child.is_file():
                    raise ValueError("clip directory contains non-regular files")
                size += child.stat().st_size
            count += not entry.name.startswith(".pending-")
        return size, count

    def write(self, doc):
        key = doc["clip_id"]
        if not re.fullmatch(r"[0-9a-f]{32}", key):
            raise ValueError("invalid clip id")
        raw = json.dumps(doc, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        report = render_report(doc).encode("utf-8")
        if len(raw) > self.MAX_JSON_BYTES:
            raise ValueError("encoded clip exceeds maximum file size")
        with self._lock:
            used, count = self._usage()
            if count >= self.max_clips or used + len(raw) + len(report) > self.max_disk_bytes:
                raise OSError("clip storage quota reached; operator must archive records")
            temp = Path(tempfile.mkdtemp(prefix=".pending-", dir=self.root))
            try:
                for name, data in (("clip.json", raw), ("report.html", report)):
                    with (temp / name).open("xb") as handle:
                        handle.write(data)
                        handle.flush()
                        os.fsync(handle.fileno())
                os.rename(temp, self.root / key)
            finally:
                # Only our own newly created temporary directory is removed.
                if temp.exists():
                    shutil.rmtree(temp)
        return {**metadata(doc), "status": "saved",
                "json_path": str(self.root / key / "clip.json"),
                "report_path": str(self.root / key / "report.html")}

    def get(self, key):
        if not isinstance(key, str) or not re.fullmatch(r"[0-9a-f]{32}", key):
            raise ValueError("invalid clip id")
        folder = self.root / key
        path = folder / "clip.json"
        if folder.is_symlink() or path.is_symlink() or path.stat().st_size > self.MAX_JSON_BYTES:
            raise ValueError("invalid clip file")
        with path.open(encoding="utf-8") as handle:
            doc = json.load(handle)
        if doc.get("schema") != "g1.telemetry_clip/1" or doc.get("clip_id") != key:
            raise ValueError("clip identity mismatch")
        return {**metadata(doc), "status": "saved", "json_path": str(path),
                "report_path": str(folder / "report.html")}

    def list(self, limit=10):
        if type(limit) is not int or not 1 <= limit <= 20:
            raise ValueError("limit must be an integer in [1, 20]")
        with self._lock:
            # Bounded files; use mtime without reading every multi-MB payload.
            dirs = [p for p in self._entries() if re.fullmatch(r"[0-9a-f]{32}", p.name)
                    and not p.is_symlink() and p.is_dir()]
            dirs.sort(key=lambda p: p.stat().st_mtime_ns, reverse=True)
            results = []
            for folder in dirs[:limit]:
                try:
                    results.append(self.get(folder.name))
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    results.append({"clip_id": folder.name, "status": "unreadable", "error": str(exc)[:300]})
            return results
