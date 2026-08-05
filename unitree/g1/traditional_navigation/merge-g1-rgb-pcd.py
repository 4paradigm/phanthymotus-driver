#!/usr/bin/env python3
"""Merge atomic FAST-LIVO2 RGB checkpoint PCDs after an unclean shutdown."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import sys
from typing import BinaryIO


SCHEMA_VERSION = 1
POINT_STRUCT = struct.Struct("<fffI")
TIMESTAMP_PCD = re.compile(r"^[0-9]+\.[0-9]+\.pcd$")
EXPECTED_FIELDS = ["x", "y", "z", "rgb"]
EXPECTED_SIZE = ["4", "4", "4", "4"]
EXPECTED_TYPE = ["F", "F", "F", "U"]
EXPECTED_COUNT = ["1", "1", "1", "1"]


class RecoveryError(ValueError):
    """Raised when a checkpoint or recovered artifact is unsafe to consume."""


def _read_header(handle: BinaryIO, path: Path) -> tuple[dict[str, list[str]], int]:
    fields: dict[str, list[str]] = {}
    while True:
        line = handle.readline()
        if not line:
            raise RecoveryError(f"missing DATA binary header: {path}")
        try:
            text = line.decode("ascii").strip()
        except UnicodeDecodeError as exc:
            raise RecoveryError(f"non-ASCII PCD header: {path}") from exc
        if not text or text.startswith("#"):
            continue
        parts = text.split()
        fields[parts[0]] = parts[1:]
        if parts[0] == "DATA":
            break
    return fields, handle.tell()


def _one_int(fields: dict[str, list[str]], name: str, path: Path) -> int:
    values = fields.get(name)
    if values is None or len(values) != 1:
        raise RecoveryError(f"invalid {name} header: {path}")
    try:
        value = int(values[0])
    except ValueError as exc:
        raise RecoveryError(f"non-integer {name} header: {path}") from exc
    if value < 1:
        raise RecoveryError(f"{name} must be positive: {path}")
    return value


def inspect_rgb_pcd(path: Path) -> dict[str, int]:
    with path.open("rb") as handle:
        fields, payload_offset = _read_header(handle, path)
    expected = {
        "FIELDS": EXPECTED_FIELDS,
        "SIZE": EXPECTED_SIZE,
        "TYPE": EXPECTED_TYPE,
        "COUNT": EXPECTED_COUNT,
        "HEIGHT": ["1"],
        "DATA": ["binary"],
    }
    for name, values in expected.items():
        if fields.get(name) != values:
            raise RecoveryError(
                f"unexpected {name} header in {path}: {fields.get(name)!r}"
            )
    width = _one_int(fields, "WIDTH", path)
    points = _one_int(fields, "POINTS", path)
    if width != points:
        raise RecoveryError(f"WIDTH/POINTS mismatch in {path}")
    expected_size = payload_offset + points * POINT_STRUCT.size
    actual_size = path.stat().st_size
    if actual_size != expected_size:
        raise RecoveryError(
            f"payload size mismatch in {path}: expected={expected_size} actual={actual_size}"
        )
    return {
        "points": points,
        "payload_offset": payload_offset,
        "bytes": actual_size,
    }


def _output_header(points: int) -> bytes:
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F U\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {points}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points}\n"
        "DATA binary\n"
    ).encode("ascii")


def _atomic_json(path: Path, payload: dict[str, object]) -> None:
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    try:
        with temporary.open("x", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _is_nonempty_all_zero(path: Path) -> bool:
    if path.stat().st_size < 1:
        return False
    with path.open("rb") as handle:
        while chunk := handle.read(1024 * 1024):
            if any(chunk):
                return False
    return True


def merge_checkpoints(
    input_dir: Path,
    output: Path,
    manifest: Path,
    reason: str,
    skip_zero_filled_checkpoints: bool = False,
) -> dict[str, object]:
    input_dir = input_dir.resolve()
    output = output.resolve()
    manifest = manifest.resolve()
    if not input_dir.is_dir():
        raise RecoveryError(f"input directory does not exist: {input_dir}")
    if output.parent != input_dir or manifest.parent != input_dir:
        raise RecoveryError("output and manifest must stay inside the input directory")
    sources = sorted(
        (
            path
            for path in input_dir.iterdir()
            if path.is_file() and TIMESTAMP_PCD.fullmatch(path.name)
        ),
        key=lambda path: tuple(int(part) for part in path.stem.split(".")),
    )
    if not sources:
        raise RecoveryError(f"no timestamp RGB checkpoint PCDs in {input_dir}")

    inspected: list[tuple[Path, dict[str, int]]] = []
    skipped: list[dict[str, object]] = []
    for path in sources:
        try:
            inspected.append((path, inspect_rgb_pcd(path)))
        except RecoveryError:
            if not skip_zero_filled_checkpoints or not _is_nonempty_all_zero(path):
                raise
            skipped.append(
                {
                    "name": path.name,
                    "bytes": path.stat().st_size,
                    "reason": "all_zero_after_unclean_shutdown",
                }
            )
    if not inspected:
        raise RecoveryError("no valid RGB checkpoint PCDs remain after recovery filtering")

    total_points = sum(int(info["points"]) for _, info in inspected)
    source_bytes = sum(int(info["bytes"]) for _, info in inspected)
    header = _output_header(total_points)
    temporary = output.with_name(f".{output.name}.tmp.{os.getpid()}")
    digest = hashlib.sha256()
    bounds = {
        "x": [math.inf, -math.inf],
        "y": [math.inf, -math.inf],
        "z": [math.inf, -math.inf],
    }
    nonzero_rgb_points = 0
    written_points = 0

    try:
        with temporary.open("xb") as destination:
            destination.write(header)
            digest.update(header)
            for path, info in inspected:
                with path.open("rb") as source:
                    source.seek(int(info["payload_offset"]))
                    remaining = int(info["points"]) * POINT_STRUCT.size
                    while remaining:
                        chunk = source.read(min(1024 * 1024, remaining))
                        if not chunk:
                            raise RecoveryError(f"short payload read: {path}")
                        if len(chunk) % POINT_STRUCT.size:
                            raise RecoveryError(f"unaligned payload chunk: {path}")
                        for x, y, z, rgb in POINT_STRUCT.iter_unpack(chunk):
                            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                                raise RecoveryError(f"non-finite point in {path}")
                            bounds["x"][0] = min(bounds["x"][0], x)
                            bounds["x"][1] = max(bounds["x"][1], x)
                            bounds["y"][0] = min(bounds["y"][0], y)
                            bounds["y"][1] = max(bounds["y"][1], y)
                            bounds["z"][0] = min(bounds["z"][0], z)
                            bounds["z"][1] = max(bounds["z"][1], z)
                            nonzero_rgb_points += int(rgb != 0)
                        destination.write(chunk)
                        digest.update(chunk)
                        written_points += len(chunk) // POINT_STRUCT.size
                        remaining -= len(chunk)
            destination.flush()
            os.fsync(destination.fileno())
        if written_points != total_points:
            raise RecoveryError(
                f"point count changed during merge: expected={total_points} actual={written_points}"
            )
        os.replace(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)

    result: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "status": "recovered_after_unclean_shutdown",
        "reason": reason,
        "source_directory": str(input_dir),
        "candidate_source_count": len(sources),
        "sources": [
            {
                "name": path.name,
                "points": int(info["points"]),
                "bytes": int(info["bytes"]),
            }
            for path, info in inspected
        ],
        "source_count": len(inspected),
        "source_bytes": source_bytes,
        "skipped_source_count": len(skipped),
        "skipped_source_bytes": sum(int(item["bytes"]) for item in skipped),
        "skipped_sources": skipped,
        "output": output.name,
        "output_bytes": output.stat().st_size,
        "output_points": total_points,
        "output_sha256": f"sha256:{digest.hexdigest()}",
        "nonzero_rgb_points": nonzero_rgb_points,
        "bounds_m": {
            axis: [round(values[0], 6), round(values[1], 6)]
            for axis, values in bounds.items()
        },
        "clean_shutdown": False,
        "unsaved_tail_recovered": False,
    }
    _atomic_json(manifest, result)
    return result


def validate_recovery(output: Path, manifest: Path) -> dict[str, object]:
    with manifest.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise RecoveryError("unsupported recovery manifest schema")
    if payload.get("status") != "recovered_after_unclean_shutdown":
        raise RecoveryError("manifest is not an unclean-shutdown recovery")
    if payload.get("clean_shutdown") is not False:
        raise RecoveryError("manifest incorrectly claims a clean shutdown")
    skipped = payload.get("skipped_sources")
    if not isinstance(skipped, list):
        raise RecoveryError("manifest skipped source list is missing")
    if payload.get("skipped_source_count") != len(skipped):
        raise RecoveryError("manifest skipped source count mismatch")
    for item in skipped:
        if (
            not isinstance(item, dict)
            or not isinstance(item.get("name"), str)
            or TIMESTAMP_PCD.fullmatch(item["name"]) is None
            or not isinstance(item.get("bytes"), int)
            or item["bytes"] < 1
            or item.get("reason") != "all_zero_after_unclean_shutdown"
        ):
            raise RecoveryError("manifest contains an invalid skipped source")
    if payload.get("output") != output.name:
        raise RecoveryError("manifest output name mismatch")
    info = inspect_rgb_pcd(output)
    if payload.get("output_points") != info["points"]:
        raise RecoveryError("manifest output point count mismatch")
    digest = hashlib.sha256()
    with output.open("rb") as handle:
        while chunk := handle.read(1024 * 1024):
            digest.update(chunk)
    if payload.get("output_sha256") != f"sha256:{digest.hexdigest()}":
        raise RecoveryError("recovered PCD checksum mismatch")
    return payload


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument(
        "--reason",
        default="fast_livo2_exit_255_after_host_reboot",
    )
    parser.add_argument("--skip-zero-filled-checkpoints", action="store_true")
    parser.add_argument("--validate", action="store_true")
    return parser


def main() -> int:
    args = _parser().parse_args()
    try:
        if args.validate:
            result = validate_recovery(args.output, args.manifest)
            action = "validate"
        else:
            if args.input_dir is None:
                raise RecoveryError("--input-dir is required unless --validate is used")
            result = merge_checkpoints(
                args.input_dir,
                args.output,
                args.manifest,
                args.reason,
                args.skip_zero_filled_checkpoints,
            )
            action = "merge"
    except (OSError, RecoveryError, json.JSONDecodeError) as exc:
        print(f"rgb_pcd_recovery=FAIL error={exc}", file=sys.stderr)
        return 1
    bounds = result["bounds_m"]
    print(
        "rgb_pcd_recovery=PASS"
        f" action={action}"
        f" sources={result['source_count']}"
        f" skipped_zero_filled={result['skipped_source_count']}"
        f" points={result['output_points']}"
        f" bytes={result['output_bytes']}"
        f" nonzero_rgb={result['nonzero_rgb_points']}"
        f" x_m={bounds['x']}"
        f" y_m={bounds['y']}"
        f" z_m={bounds['z']}"
        f" clean_shutdown={str(result['clean_shutdown']).lower()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
