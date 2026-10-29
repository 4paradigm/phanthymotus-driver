"""Bounded action packages; transport belongs to the driver's file intake.

Only the robot adapter can validate/rebind a trajectory. Import creates a draft,
not an approved skill. No archive paths are extracted into the filesystem.
"""
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import zipfile

SCHEMA = "motus.teaching.package/1"
MAX_ARCHIVE = 16 * 1024 * 1024
MAX_RECORDING = 64 * 1024 * 1024
SKILL_FIELDS = {"name": 100, "description": 1000, "conditions": 1000, "phrase": 200}
RECORDING_FIELDS = {"path", "bytes", "sha256", "schema"}


def unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("package_duplicate_field")
        result[key] = value
    return result


def validate_manifest(manifest):
    if not isinstance(manifest, dict) or not {"schema", "skill", "compatibility", "recording"} <= manifest.keys():
        raise ValueError("invalid_skill_package_manifest")
    if manifest["schema"] != SCHEMA or not isinstance(manifest["compatibility"], dict):
        raise ValueError("unsupported_skill_package")
    meta = manifest["skill"]
    if not isinstance(meta, dict) or not SKILL_FIELDS.keys() <= meta.keys():
        raise ValueError("invalid_package_skill")
    if any(not isinstance(meta[k], str) or not meta[k].strip() or len(meta[k]) > n for k, n in SKILL_FIELDS.items()):
        raise ValueError("invalid_package_skill")
    rec = manifest["recording"]
    if (not isinstance(rec, dict) or not RECORDING_FIELDS <= rec.keys()
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
            # Unknown manifest fields may be added without changing the wire
            # version. Do not forward them as authoring/save arguments.
            manifest = {"schema": manifest["schema"], "compatibility": manifest["compatibility"],
                        "skill": {k: manifest["skill"][k] for k in SKILL_FIELDS},
                        "recording": {k: manifest["recording"][k] for k in RECORDING_FIELDS}}
            return manifest, recording
    except (zipfile.BadZipFile, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid_skill_package") from exc


def read_package(path):
    """Read the receiving container's path returned by POST /file/upload."""
    if not isinstance(path, str) or not Path(path).is_absolute():
        raise ValueError("请选择动作包上传，并使用设备返回的绝对路径")
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, "rb") as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or not 1 <= info.st_size <= MAX_ARCHIVE:
            raise ValueError("invalid_skill_package_file")
        return unpack(stream.read(MAX_ARCHIVE + 1))
