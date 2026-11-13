import copy
from contextlib import closing
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import unittest

import install_skill
import verify


ROOT = Path(__file__).parent
SKILL = json.loads((ROOT / "skill.json").read_text(encoding="utf-8"))


class SkillTests(unittest.TestCase):
    def test_install_preserves_other_config_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.db"
            with closing(sqlite3.connect(str(path))) as conn:
                conn.execute("CREATE TABLE config(key TEXT PRIMARY KEY,value TEXT NOT NULL)")
                for key, value in (("core", {"configured": True}), ("services", {"mcp": []}),
                                   ("skills", {"extra": 7, "installed": [{"slug": "existing", "active": False}]})):
                    conn.execute("INSERT INTO config VALUES (?,?)", (key, json.dumps(value)))
                conn.commit()
            first = install_skill.install(path, SKILL)
            self.assertTrue(Path(first["backup"]).is_file())
            self.assertTrue(install_skill.install(path, SKILL)["unchanged"])
            with closing(sqlite3.connect(str(path))) as conn:
                values = {k: json.loads(v) for k, v in conn.execute("SELECT key,value FROM config")}
            self.assertEqual(values["skills"]["installed"][0], {"slug": "existing", "active": False})
            self.assertEqual(values["skills"]["extra"], 7)
            self.assertEqual(values["core"], {"configured": True})
            installed = values["skills"]["installed"][1]
            self.assertEqual(installed["instruction"], SKILL["instruction"])
            self.assertTrue(installed["active"])
            changed = copy.deepcopy(SKILL)
            changed["instruction"] += "\nChanged"
            with self.assertRaises(ValueError):
                install_skill.install(path, changed)

    def test_ssh_stdin_bundle_installs_in_an_existing_database(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.db"
            with closing(sqlite3.connect(str(path))) as conn:
                conn.execute("CREATE TABLE config(key TEXT PRIMARY KEY,value TEXT NOT NULL)")
                conn.executemany("INSERT INTO config VALUES (?,?)", [("core", "{}"), ("services", "{}")])
                conn.commit()
            result = subprocess.run([sys.executable, "-X", "utf8", "-", "--db", str(path)],
                                    input=install_skill.bundle_source(SKILL), text=True,
                                    encoding="utf-8", capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(json.loads(result.stdout)["ok"])

    def test_missing_database_is_not_created(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.db"
            with self.assertRaises(FileNotFoundError):
                install_skill.install(path, SKILL)
            self.assertFalse(path.exists())

    def test_outer_success_is_not_business_success(self):
        with self.assertRaises(ValueError):
            verify.unwrap({"code": 200, "data": [{"type": "text", "text": '{"ok":false,"error":"no frame"}'}]})

    def test_stale_unknown_and_invalid_frames_fail(self):
        for age in (None, True, -1, 2.01, float("nan"), "0.1"):
            with self.subTest(age=age), self.assertRaises(ValueError):
                verify.fresh_age(age)

    def test_failed_capture_never_claims_success(self):
        class FakeAPI:
            def json(self, path):
                return {"code": 200, "data": [{"id": "test", "tools": [{"name": "vision_capture", "inputSchema": {"properties": {"action": {"enum": ["info", "capture_photo"]}}}}]}]}

            def call(self, mcp, action):
                if action == "info":
                    return {"ok": True, "latest_frame_age_s": 0.01}
                return {"ok": True, "media_type": "photo", "frame_age_s": 0.01}

        with tempfile.TemporaryDirectory() as directory:
            report, path = verify.run(FakeAPI(), "Station A", directory)
            self.assertEqual(report["status"], "failed")
            self.assertNotIn("local_file_path", report)
            self.assertTrue(path.is_file())

    def test_instruction_file_matches_native_json(self):
        self.assertEqual(SKILL["instruction"], (ROOT / "instruction.md").read_text(encoding="utf-8").strip())


if __name__ == "__main__":
    unittest.main(verbosity=2)
