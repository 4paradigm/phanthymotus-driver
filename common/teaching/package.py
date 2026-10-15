"""Bounded action packages and temporary transfers; never executable content.

Only the robot adapter can validate/rebind a trajectory. Import creates a draft,
not an approved skill. No archive paths are extracted into the filesystem.
"""
import base64
import hashlib
import io
import json
from pathlib import Path
import secrets
import tempfile
import time
import zipfile

SCHEMA = "motus.teaching.package/1"
MAX_ARCHIVE = 16 * 1024 * 1024
MAX_RECORDING = 64 * 1024 * 1024
CHUNK = 24 * 1024


def unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("package_duplicate_field")
        result[key] = value
    return result


def validate_manifest(manifest):
    if not isinstance(manifest, dict) or set(manifest) != {"schema", "skill", "compatibility", "recording"}:
        raise ValueError("invalid_skill_package_manifest")
    if manifest["schema"] != SCHEMA or not isinstance(manifest["compatibility"], dict):
        raise ValueError("unsupported_skill_package")
    meta = manifest["skill"]
    limits = {"name": 100, "description": 1000, "conditions": 1000, "phrase": 200}
    if not isinstance(meta, dict) or set(meta) != set(limits):
        raise ValueError("invalid_package_skill")
    if any(not isinstance(meta[k], str) or not meta[k].strip() or len(meta[k]) > n for k, n in limits.items()):
        raise ValueError("invalid_package_skill")
    rec = manifest["recording"]
    if (not isinstance(rec, dict) or set(rec) != {"path", "bytes", "sha256", "schema"}
            or rec["path"] != "trajectory.jsonl" or type(rec["bytes"]) is not int
            or not 1 <= rec["bytes"] <= MAX_RECORDING
            or not isinstance(rec["sha256"], str) or len(rec["sha256"]) != 64
            or any(c not in "0123456789abcdef" for c in rec["sha256"])
            or not isinstance(rec["schema"], str) or not rec["schema"]):
        raise ValueError("invalid_package_recording")
    if len(json.dumps(manifest, ensure_ascii=False).encode()) > 16384:
        raise ValueError("package_manifest_too_large")


def pack(metadata, compatibility, recording, recording_schema):
    manifest = {"schema": SCHEMA, "skill": metadata, "compatibility": compatibility,
                "recording": {"path": "trajectory.jsonl", "bytes": len(recording),
                    "sha256": hashlib.sha256(recording).hexdigest(), "schema": recording_schema}}
    validate_manifest(manifest)
    out = io.BytesIO()
    with zipfile.ZipFile(out, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("manifest.json", json.dumps(manifest, ensure_ascii=False, allow_nan=False))
        archive.writestr("trajectory.jsonl", recording)
    if out.tell() > MAX_ARCHIVE:
        raise ValueError("skill_package_too_large")
    return manifest, out.getvalue()


def unpack(data):
    if not 1 <= len(data) <= MAX_ARCHIVE:
        raise ValueError("skill_package_too_large")
    try:
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            if sorted(archive.namelist()) != ["manifest.json", "trajectory.jsonl"]:
                raise ValueError("unexpected_package_files")
            for info in archive.infolist():
                limit = 16384 if info.filename == "manifest.json" else MAX_RECORDING
                if info.file_size > limit or info.flag_bits & 1 or info.compress_type not in (0, 8):
                    raise ValueError("invalid_package_file")
            manifest = json.loads(archive.read("manifest.json"), object_pairs_hook=unique)
            validate_manifest(manifest)
            rec = archive.getinfo("trajectory.jsonl")
            if rec.file_size != manifest["recording"]["bytes"]:
                raise ValueError("package_size_mismatch")
            recording = archive.read(rec)
            if hashlib.sha256(recording).hexdigest() != manifest["recording"]["sha256"]:
                raise ValueError("package_checksum_mismatch")
            return manifest, recording
    except (zipfile.BadZipFile, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid_skill_package") from exc


class Transfers:
    """One upload and one download per card, bounded by size and lifetime."""
    def __init__(self):
        self.root = tempfile.TemporaryDirectory(prefix="motus-teaching-")
        self.items = {}

    def _purge(self, mode=None):
        for ident, item in list(self.items.items()):
            if item["expires"] <= time.monotonic() or item["mode"] == mode:
                item["path"].unlink(missing_ok=True)
                del self.items[ident]

    def begin(self, size, digest, mode, data=None):
        if type(size) is not int or not 1 <= size <= MAX_ARCHIVE:
            raise ValueError("invalid_package_size")
        if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            raise ValueError("invalid_package_digest")
        self._purge(mode)
        ident = secrets.token_hex(16)
        path = Path(self.root.name) / ident
        path.touch(mode=0o600)
        if data is not None:
            path.write_bytes(data)
        self.items[ident] = {"path": path, "size": size, "sha256": digest,
                             "mode": mode, "expires": time.monotonic()+1800}
        return {"transfer_id": ident, "bytes": size, "sha256": digest, "chunk_bytes": CHUNK}

    def get(self, ident, mode):
        self._purge()
        if ident not in self.items or self.items[ident]["mode"] != mode:
            raise ValueError("package_transfer_expired")
        return self.items[ident]

    def read(self, ident, offset):
        item = self.get(ident, "download")
        if type(offset) is not int or not 0 <= offset < item["size"]:
            raise ValueError("invalid_package_offset")
        with item["path"].open("rb") as stream:
            stream.seek(offset)
            data = stream.read(CHUNK)
        return {"data": base64.b64encode(data).decode(), "next_offset": offset+len(data)}

    def write(self, ident, offset, encoded):
        item = self.get(ident, "upload")
        if not isinstance(encoded, str) or len(encoded) > CHUNK*4//3:
            raise ValueError("package_chunk_too_large")
        try:
            data = base64.b64decode(encoded, validate=True)
        except ValueError as exc:
            raise ValueError("invalid_package_chunk") from exc
        if (not data or type(offset) is not int or offset != item["path"].stat().st_size
                or offset+len(data) > item["size"]):
            raise ValueError("invalid_package_offset")
        with item["path"].open("ab") as stream:
            stream.write(data)
        return {"next_offset": offset+len(data)}

    def finish(self, ident):
        item = self.get(ident, "upload")
        data = item["path"].read_bytes()
        if len(data) != item["size"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise ValueError("package_transfer_incomplete_or_changed")
        item["path"].unlink()
        del self.items[ident]
        return unpack(data)
