"""Temporary SQLite tests for the local Bumi guard skill installer."""

from __future__ import annotations

import json
import pathlib
import sqlite3
import tempfile
import unittest

from noetix.bumi.deploy import install_bumi_guard_skill as installer


SKILL = {
    "slug": "noetix-bumi-guard",
    "name": "Bumi visitor greeting",
    "description": "Guide the Bumi guard pipeline",
    "oneLiner": "Greet visitors with the local whitelist",
    "instruction": "Use the Bumi camera, auto_guard, TTS, and speaker cards.",
    "category": "robotics",
    "version": "1.0.0",
    "requiredTools": ["mcp__bumi__auto_guard"],
    "author": {"name": "Noetix"},
    "active": False,
}


def make_db(path: pathlib.Path, skills=None):
    conn = sqlite3.connect(path)
    conn.execute("CREATE TABLE config (key TEXT PRIMARY KEY, value TEXT NOT NULL)")
    conn.execute("CREATE TABLE chat_sessions (id TEXT PRIMARY KEY)")
    conn.execute("CREATE TABLE chat_messages (id INTEGER PRIMARY KEY)")
    conn.execute("INSERT INTO config VALUES (?, ?)",
                 ("core", json.dumps({"configured": True})))
    conn.execute("INSERT INTO config VALUES (?, ?)",
                 ("services", json.dumps({"mcp": []})))
    if skills is not None:
        conn.execute("INSERT INTO config VALUES (?, ?)", ("skills", json.dumps(skills)))
    conn.commit()
    conn.close()


def read_skills(path: pathlib.Path):
    with sqlite3.connect(path) as conn:
        row = conn.execute("SELECT value FROM config WHERE key='skills'").fetchone()
    return json.loads(row[0]) if row else None


class SkillInstallTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = pathlib.Path(self.temp.name)
        self.db = self.root / "data.db"
        self.skill_file = self.root / "bumi_guard_skill.json"
        self.skill_file.write_text(json.dumps(SKILL), encoding="utf-8")

    def test_repository_skill_manifest_is_valid_and_inactive(self):
        manifest = pathlib.Path(__file__).resolve().parents[1] / "deploy/bumi_guard_skill.json"
        skill = installer.load_skill(manifest)
        self.assertEqual(skill["slug"], "bumi-guard")
        self.assertFalse(skill["active"])
        self.assertIn("guard_status", skill["requiredTools"])

    def test_dry_run_does_not_write_and_apply_installs_once(self):
        make_db(self.db, {"installed": [], "other_setting": 7})
        before = read_skills(self.db)
        preview = installer.install(self.db, self.skill_file)
        self.assertEqual(preview["mode"], "dry-run")
        self.assertEqual(preview["action"], "install")
        self.assertEqual(read_skills(self.db), before)

        applied = installer.install(self.db, self.skill_file, apply=True)
        self.assertEqual(applied["action"], "install")
        result = read_skills(self.db)
        self.assertEqual(result["other_setting"], 7)
        self.assertEqual(len(result["installed"]), 1)
        self.assertEqual(result["installed"][0]["slug"], SKILL["slug"])
        self.assertEqual(result["installed"][0]["author"], "Noetix")
        self.assertFalse(result["installed"][0]["active"])
        self.assertTrue(result["installed"][0]["installedAt"])

        again = installer.install(self.db, self.skill_file, apply=True)
        self.assertEqual(again["action"], "unchanged")
        self.assertEqual(read_skills(self.db), result)

    def test_missing_skills_row_is_inserted_only_on_apply(self):
        make_db(self.db)
        self.assertIsNone(read_skills(self.db))
        installer.install(self.db, self.skill_file)
        self.assertIsNone(read_skills(self.db))
        installer.install(self.db, self.skill_file, apply=True)
        self.assertEqual(read_skills(self.db)["installed"][0]["slug"], SKILL["slug"])

    def test_operator_activation_survives_rerun_and_replacement(self):
        make_db(self.db, {"installed": []})
        installer.install(self.db, self.skill_file, apply=True)
        current = read_skills(self.db)
        current["installed"][0]["active"] = True
        with sqlite3.connect(self.db) as conn:
            conn.execute("UPDATE config SET value=? WHERE key='skills'", (json.dumps(current),))

        self.assertEqual(installer.install(self.db, self.skill_file)["action"], "unchanged")
        self.assertEqual(installer.install(self.db, self.skill_file, apply=True)["action"],
                         "unchanged")
        changed = dict(SKILL)
        changed["instruction"] = "Updated instructions for the Bumi guard."
        self.skill_file.write_text(json.dumps(changed), encoding="utf-8")
        installer.install(self.db, self.skill_file, apply=True, replace=True)
        target = read_skills(self.db)["installed"][0]
        self.assertTrue(target["active"])
        self.assertEqual(target["instruction"], changed["instruction"])

    def test_conflicting_slug_requires_replace_and_keeps_other_skills(self):
        unrelated = {"slug": "other-skill", "name": "Keep me", "active": True}
        existing = {**SKILL, "name": "Old Bumi version", "author": "Noetix",
                    "active": False, "installedAt": "2025-01-02T00:00:00"}
        make_db(self.db, {"installed": [unrelated, existing], "custom": {"keep": True}})
        before = read_skills(self.db)
        with self.assertRaisesRegex(installer.InstallError, "--replace"):
            installer.install(self.db, self.skill_file, apply=True)
        self.assertEqual(read_skills(self.db), before)

        preview = installer.install(self.db, self.skill_file, replace=True)
        self.assertEqual(preview["action"], "replace")
        self.assertEqual(read_skills(self.db), before)
        installer.install(self.db, self.skill_file, apply=True, replace=True)
        result = read_skills(self.db)
        self.assertEqual(result["installed"][0], unrelated)
        self.assertEqual(result["custom"], {"keep": True})
        target = result["installed"][1]
        self.assertEqual(target["name"], SKILL["name"])
        self.assertEqual(target["installedAt"], existing["installedAt"])
        self.assertFalse(target["active"], "preserve an operator's active setting")
        self.assertEqual(installer.install(self.db, self.skill_file, apply=True)["action"],
                         "unchanged")

    def test_refuses_missing_and_unrelated_databases_without_creating_config(self):
        with self.assertRaisesRegex(installer.InstallError, "does not exist"):
            installer.install(self.db, self.skill_file, apply=True)
        self.assertFalse(self.db.exists())
        with sqlite3.connect(self.db) as conn:
            conn.execute("CREATE TABLE other_data (value TEXT)")
        with self.assertRaisesRegex(installer.InstallError, "not an Agent Core"):
            installer.install(self.db, self.skill_file, apply=True)
        with sqlite3.connect(self.db) as conn:
            tables = {r[0] for r in conn.execute(
                "SELECT name FROM sqlite_master WHERE type='table'")}
        self.assertEqual(tables, {"other_data"})

    def test_validation_failure_leaves_database_untouched(self):
        make_db(self.db, {"installed": []})
        self.skill_file.write_text('{"slug": "noetix-bumi-guard", "name": "missing content"}',
                                   encoding="utf-8")
        before = read_skills(self.db)
        with self.assertRaisesRegex(installer.InstallError, "oneLiner"):
            installer.install(self.db, self.skill_file, apply=True)
        self.assertEqual(read_skills(self.db), before)

    def test_slug_and_required_tools_follow_agent_core_manifest_rules(self):
        make_db(self.db, {"installed": []})
        malformed = dict(SKILL)
        malformed["slug"] = "Bumi Guard"
        self.skill_file.write_text(json.dumps(malformed), encoding="utf-8")
        with self.assertRaisesRegex(installer.InstallError, r"\[a-z0-9-\]"):
            installer.install(self.db, self.skill_file)
        malformed["slug"] = SKILL["slug"]
        malformed["requiredTools"] = ["mcp__bumi__auto_guard", "mcp__bumi__auto_guard"]
        self.skill_file.write_text(json.dumps(malformed), encoding="utf-8")
        with self.assertRaisesRegex(installer.InstallError, "unique nonempty strings"):
            installer.install(self.db, self.skill_file)

    def test_sqlite_error_rolls_back_transaction(self):
        make_db(self.db, {"installed": []})
        with sqlite3.connect(self.db) as conn:
            conn.execute("CREATE TRIGGER forbid_skill_write BEFORE UPDATE ON config "
                         "WHEN NEW.key='skills' BEGIN SELECT RAISE(ABORT, 'locked'); END")
        before = read_skills(self.db)
        with self.assertRaisesRegex(installer.InstallError, "locked"):
            installer.install(self.db, self.skill_file, apply=True)
        self.assertEqual(read_skills(self.db), before)


if __name__ == "__main__":
    unittest.main()
