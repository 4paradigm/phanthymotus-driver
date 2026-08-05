#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest


NAV_DIR = Path(__file__).resolve().parent.parent
MODULE_PATH = NAV_DIR / "merge-g1-rgb-pcd.py"
SPEC = importlib.util.spec_from_file_location("merge_g1_rgb_pcd", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_rgb_pcd(path: Path, points: list[tuple[float, float, float, int]]) -> None:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F U\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    with path.open("wb") as handle:
        handle.write(header)
        for point in points:
            handle.write(struct.pack("<fffI", *point))


class RgbPcdRecoveryTests(unittest.TestCase):
    def test_merges_timestamp_checkpoints_and_writes_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_rgb_pcd(
                root / "1785871908.100000.pcd",
                [(0.0, 1.0, 2.0, 0x00112233), (2.0, 3.0, 4.0, 0x00445566)],
            )
            write_rgb_pcd(
                root / "1785871909.200000.pcd",
                [(-1.0, 5.0, 0.5, 0x00778899)],
            )
            write_rgb_pcd(root / "all_rgb_points.recovered.pcd", [(99.0, 99.0, 99.0, 1)])
            output = root / "all_rgb_points.recovered.pcd"
            manifest = root / "rgb-recovery-manifest.json"

            result = MODULE.merge_checkpoints(root, output, manifest, "test_reboot")

            self.assertEqual(result["source_count"], 2)
            self.assertEqual(result["output_points"], 3)
            self.assertEqual(result["nonzero_rgb_points"], 3)
            self.assertEqual(result["bounds_m"]["x"], [-1.0, 2.0])
            self.assertFalse(result["clean_shutdown"])
            self.assertFalse(result["unsaved_tail_recovered"])
            self.assertEqual(MODULE.inspect_rgb_pcd(output)["points"], 3)
            validated = MODULE.validate_recovery(output, manifest)
            self.assertEqual(validated["output_sha256"], result["output_sha256"])
            with manifest.open(encoding="utf-8") as handle:
                self.assertEqual(json.load(handle)["reason"], "test_reboot")

    def test_explicitly_skips_only_nonempty_all_zero_checkpoints(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_rgb_pcd(
                root / "1785871908.100000.pcd",
                [(0.0, 1.0, 2.0, 0x00112233)],
            )
            zero_filled = root / "1785871909.200000.pcd"
            zero_filled.write_bytes(bytes(4096))
            output = root / "all_rgb_points.recovered.pcd"
            manifest = root / "rgb-recovery-manifest.json"

            with self.assertRaisesRegex(MODULE.RecoveryError, "missing DATA"):
                MODULE.merge_checkpoints(root, output, manifest, "test_reboot")

            result = MODULE.merge_checkpoints(
                root,
                output,
                manifest,
                "test_reboot",
                skip_zero_filled_checkpoints=True,
            )

            self.assertEqual(result["candidate_source_count"], 2)
            self.assertEqual(result["source_count"], 1)
            self.assertEqual(result["skipped_source_count"], 1)
            self.assertEqual(result["skipped_source_bytes"], 4096)
            self.assertEqual(
                result["skipped_sources"],
                [
                    {
                        "name": zero_filled.name,
                        "bytes": 4096,
                        "reason": "all_zero_after_unclean_shutdown",
                    }
                ],
            )
            MODULE.validate_recovery(output, manifest)

    def test_rejects_truncated_checkpoint_without_replacing_output(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "1785871908.100000.pcd"
            write_rgb_pcd(source, [(0.0, 1.0, 2.0, 0x00112233)])
            source.write_bytes(source.read_bytes()[:-1])
            output = root / "all_rgb_points.recovered.pcd"
            output.write_bytes(b"preserve-me")

            with self.assertRaisesRegex(MODULE.RecoveryError, "payload size mismatch"):
                MODULE.merge_checkpoints(
                    root,
                    output,
                    root / "rgb-recovery-manifest.json",
                    "test_reboot",
                )

            self.assertEqual(output.read_bytes(), b"preserve-me")

    def test_rejects_non_rgb_checkpoint(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "1785871908.100000.pcd"
            write_rgb_pcd(source, [(0.0, 1.0, 2.0, 0x00112233)])
            source.write_bytes(source.read_bytes().replace(b"FIELDS x y z rgb", b"FIELDS x y z foo"))

            with self.assertRaisesRegex(MODULE.RecoveryError, "unexpected FIELDS"):
                MODULE.merge_checkpoints(
                    root,
                    root / "all_rgb_points.recovered.pcd",
                    root / "rgb-recovery-manifest.json",
                    "test_reboot",
                    skip_zero_filled_checkpoints=True,
                )


if __name__ == "__main__":
    unittest.main()
