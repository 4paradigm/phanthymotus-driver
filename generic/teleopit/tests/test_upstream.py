"""Pinned source/assets and safe operator-only setup contracts (no network)."""

from __future__ import annotations

import hashlib
import io
import json
import subprocess
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))

import setup_teleopit as setup
import upstream


class SafeExtractionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def archive(self, entries):
        archive = self.root / "assets.tar.gz"
        with tarfile.open(archive, "w:gz") as tar:
            for name, kind, data in entries:
                entry = tarfile.TarInfo(name)
                entry.type = kind
                entry.size = len(data) if kind == tarfile.REGTYPE else 0
                if kind in (tarfile.SYMTYPE, tarfile.LNKTYPE):
                    entry.linkname = "../outside"
                tar.addfile(entry, io.BytesIO(data))
        return archive

    def test_regular_files_extract_with_parent_directories(self):
        archive = self.archive([("./unitree_g1/model.xml", tarfile.REGTYPE, b"model")])
        setup.safe_extract(archive, self.root / "output")
        self.assertEqual((self.root / "output/unitree_g1/model.xml").read_bytes(), b"model")

    def test_rejects_traversal_absolute_windows_paths_links_and_devices(self):
        entries = [
            ("../outside", tarfile.REGTYPE), ("/outside", tarfile.REGTYPE),
            ("x/../../outside", tarfile.REGTYPE), ("C:/outside", tarfile.REGTYPE),
            ("x\\..\\outside", tarfile.REGTYPE), ("safe", tarfile.SYMTYPE),
            ("safe", tarfile.LNKTYPE), ("safe", tarfile.CHRTYPE),
            ("safe", tarfile.FIFOTYPE),
        ]
        for index, (name, kind) in enumerate(entries):
            with self.subTest(name=name, kind=kind):
                archive = self.archive([(name, kind, b"bad")])
                with self.assertRaises(ValueError):
                    setup.safe_extract(archive, self.root / f"out{index}")
        self.assertFalse((self.root / "outside").exists())

    def test_rejects_duplicate_members_and_expansion_limit_before_writing(self):
        archive = self.archive([("a", tarfile.REGTYPE, b"one"),
                                ("a", tarfile.REGTYPE, b"two")])
        with self.assertRaises(ValueError):
            setup.safe_extract(archive, self.root / "duplicates")
        self.assertFalse((self.root / "duplicates/a").exists())
        archive = self.archive([("a", tarfile.REGTYPE, b"long")])
        with self.assertRaises(ValueError):
            setup.safe_extract(archive, self.root / "limit", max_bytes=3)
        self.assertFalse((self.root / "limit/a").exists())

    def test_existing_different_assets_preserved_and_identical_install_is_idempotent(self):
        archive = self.archive([("model.xml", tarfile.REGTYPE, b"model")])
        asset = upstream.Asset("robots", "a.tar.gz", "assets/robots",
                               upstream.sha256_file(archive), archive.stat().st_size, True)
        first = setup.install_asset(asset, archive, self.root)
        self.assertEqual(first, setup.install_asset(asset, archive, self.root))
        installed = self.root / "assets/robots/model.xml"
        installed.write_text("operator data")
        with self.assertRaisesRegex(ValueError, "未覆盖"):
            setup.install_asset(asset, archive, self.root)
        self.assertEqual(installed.read_text(), "operator data")

    def test_download_requires_pinned_digest_and_cache_is_not_overwritten(self):
        data = b"test-model"
        asset = upstream.Asset("policy", "test.onnx", "ckpt/test.onnx",
                               hashlib.sha256(data).hexdigest(), len(data))
        cache = self.root / "cache"
        with patch("urllib.request.urlopen", return_value=io.BytesIO(data)) as request:
            target = setup.download_asset(asset, cache)
        self.assertIn(upstream.ASSET_REVISION, request.call_args.args[0].full_url)
        with patch("urllib.request.urlopen") as request:
            self.assertEqual(setup.download_asset(asset, cache), target)
            request.assert_not_called()
        target.write_bytes(b"operator")
        with patch("urllib.request.urlopen") as request, self.assertRaises(ValueError):
            setup.download_asset(asset, cache)
        request.assert_not_called()
        self.assertEqual(target.read_bytes(), b"operator")

    def test_failed_download_leaves_no_published_asset(self):
        asset = upstream.Asset("policy", "test.onnx", "test.onnx", "0" * 64, 3)
        with patch("urllib.request.urlopen", return_value=io.BytesIO(b"bad")):
            with self.assertRaises(ValueError):
                setup.download_asset(asset, self.root / "cache")
        self.assertEqual(list((self.root / "cache").iterdir()), [])

    def test_rejects_existing_symlink_in_asset_parent(self):
        source = self.root / "test-model"
        source.write_bytes(b"model")
        asset = upstream.Asset("policy", "model.onnx", "ckpt/model.onnx",
                               upstream.sha256_file(source), source.stat().st_size)
        outside = self.root / "operator-files"
        outside.mkdir()
        (self.root / "ckpt").symlink_to(outside, target_is_directory=True)
        with self.assertRaises(ValueError):
            setup.install_asset(asset, source, self.root)
        self.assertEqual(list(outside.iterdir()), [])


class InstallationInspectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.git("init", "--quiet")
        self.git("config", "user.email", "teleopit-test@example.invalid")
        self.git("config", "user.name", "Teleopit test")
        files = ["teleopit/configs/default.yaml",
                 "teleopit/retargeting/gmr/ik_configs/bvh_lafan1_to_g1.json",
                 "teleopit/retargeting/gmr/ik_configs/pico_bridge_to_g1.json"]
        for filename in files:
            path = self.root / filename
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("fixture")
        self.git("add", "teleopit")
        self.git("commit", "--quiet", "-m", "fixture")
        self.revision = self.git("rev-parse", "HEAD")
        self.addCleanup(patch.stopall)
        patch.object(upstream, "SOURCE_REVISION", self.revision).start()
        for filename in (upstream.DEFAULT_POLICY, upstream.DEFAULT_BVH, upstream.ROBOT_XML):
            path = self.root / filename
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("fixture")
        policy = upstream.Asset("policy", "policy.onnx", upstream.DEFAULT_POLICY,
                                hashlib.sha256(b"fixture").hexdigest(), 7)
        patch.object(upstream, "ASSETS", (policy,)).start()

    def git(self, *args):
        return subprocess.run(["git", "-C", str(self.root), *args],
                              capture_output=True, text=True, check=True).stdout.strip()

    def test_reports_ready_paths_and_checks_policy_without_importing_simulator(self):
        before = set(sys.modules)
        report = upstream.inspect_installation(self.root)
        self.assertTrue(report["ready"], report["errors"])
        self.assertTrue(report["policy_verified"])
        self.assertEqual(report["paths"]["bvh"], str((self.root / upstream.DEFAULT_BVH).resolve()))
        self.assertFalse(report["receipt_present"])
        self.assertNotIn("mujoco", set(sys.modules) - before)

    def test_wrong_revision_and_modified_tracked_source_rejected(self):
        with patch.object(upstream, "SOURCE_REVISION", "0" * 40):
            self.assertFalse(upstream.inspect_installation(self.root)["ready"])
        (self.root / "teleopit/configs/default.yaml").write_text("modified")
        report = upstream.inspect_installation(self.root)
        self.assertFalse(report["ready"])
        self.assertTrue(any("本地修改" in error for error in report["errors"]))
        with self.assertRaises(ValueError):
            setup.ensure_checkout(self.root)
        self.assertEqual((self.root / "teleopit/configs/default.yaml").read_text(), "modified")

    def test_missing_bvh_does_not_block_pico_but_missing_policy_does(self):
        (self.root / upstream.DEFAULT_BVH).unlink()
        self.assertFalse(upstream.inspect_installation(self.root)["ready"])
        self.assertTrue(upstream.inspect_installation(self.root, source="pico")["ready"])
        (self.root / upstream.DEFAULT_POLICY).write_text("tampered")
        self.assertFalse(upstream.inspect_installation(self.root, source="pico")["ready"])

    def test_stale_or_nonobject_receipt_is_reported_without_false_provenance(self):
        for receipt in ({"asset_revision": "stale"}, []):
            with self.subTest(receipt=receipt):
                (self.root / upstream.ASSET_RECEIPT).write_text(json.dumps(receipt))
                report = upstream.inspect_installation(self.root)
                self.assertFalse(report["receipt_present"])
                self.assertTrue(any("凭据" in warning for warning in report["warnings"]))

    def test_receipt_detects_modified_canonical_robot_xml(self):
        receipt = {"source_revision": self.revision, "asset_revision": upstream.ASSET_REVISION,
                   "assets": {"robots": {"files": {"unitree_g1/g1_29dof.xml":
                              hashlib.sha256(b"fixture").hexdigest()}}}}
        (self.root / upstream.ASSET_RECEIPT).write_text(json.dumps(receipt))
        self.assertTrue(upstream.inspect_installation(self.root, source="pico")["ready"])
        (self.root / upstream.ROBOT_XML).write_text("modified xml")
        report = upstream.inspect_installation(self.root, source="pico")
        self.assertFalse(report["ready"])
        self.assertTrue(any("资源已改变" in error for error in report["errors"]))

    def test_check_cli_is_read_only_and_refuses_missing_installation(self):
        with (patch.object(setup, "ensure_checkout") as checkout,
              patch.object(setup, "download_asset") as download,
              patch("builtins.print")):
            result = setup.main(["--root", str(self.root / "missing"), "--check"])
        self.assertEqual(result, 1)
        checkout.assert_not_called()
        download.assert_not_called()
        self.assertFalse((self.root / "missing").exists())


if __name__ == "__main__":
    unittest.main()
