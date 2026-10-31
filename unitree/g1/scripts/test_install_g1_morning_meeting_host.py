"""Installer checks; these tests do not connect to or command a robot."""
from contextlib import closing
import json
import sqlite3
import tempfile
import unittest
from pathlib import Path

from install_g1_morning_meeting_host import install, new_skill


class MorningMeetingHostInstallerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.db = Path(self.temp.name) / 'data.db'
        with closing(sqlite3.connect(self.db)) as conn:
            conn.execute('CREATE TABLE config(key TEXT PRIMARY KEY, value TEXT)')
            conn.commit()

    def read_skills(self):
        with closing(sqlite3.connect(self.db)) as conn:
            row = conn.execute("SELECT value FROM config WHERE key='skills'").fetchone()
        return json.loads(row[0])

    def test_definition_is_scoped_to_voice_and_arm(self):
        self.assertEqual(new_skill['requiredTools'], ['tts', 'arm'])
        self.assertIn('不得使用麦克风、ASR、相机、OCR', new_skill['instruction'])
        self.assertIn('不得行走、转向', new_skill['instruction'])
        self.assertIn('蹲起', new_skill['instruction'])

    def test_install_creates_disabled_skill_and_backup(self):
        result = install(self.db)
        saved = self.read_skills()
        self.assertEqual(result['slug'], 'g1-morning-meeting-host')
        self.assertFalse(result['active'])
        self.assertTrue(Path(result['backup']).is_file())
        self.assertEqual(len(saved['installed']), 1)
        self.assertFalse(saved['installed'][0]['active'])

    def test_reinstall_preserves_other_entries_and_active_state(self):
        other = {'slug': 'other', 'active': True, 'instruction': 'keep'}
        with closing(sqlite3.connect(self.db)) as conn:
            conn.execute("INSERT INTO config VALUES('skills',?)", (json.dumps({
                'installed': [other, {'slug': new_skill['slug'], 'active': True,
                                      'installedAt': 'original'}], 'extra': 1}),))
            conn.commit()
        install(self.db, backup=False)
        install(self.db, backup=False)
        saved = self.read_skills()
        self.assertEqual(saved['extra'], 1)
        self.assertEqual(saved['installed'][0], other)
        matching = [x for x in saved['installed'] if x['slug'] == new_skill['slug']]
        self.assertEqual(len(matching), 1)
        self.assertTrue(matching[0]['active'])
        self.assertEqual(matching[0]['installedAt'], 'original')

    def test_malformed_config_is_not_overwritten(self):
        original = {'installed': {}}
        with closing(sqlite3.connect(self.db)) as conn:
            conn.execute("INSERT INTO config VALUES('skills',?)", (json.dumps(original),))
            conn.commit()
        with self.assertRaises(ValueError):
            install(self.db, backup=False)
        self.assertEqual(self.read_skills(), original)

    def test_missing_database_is_not_created(self):
        missing = self.db.with_name('missing.db')
        with self.assertRaises(FileNotFoundError):
            install(missing)
        self.assertFalse(missing.exists())


if __name__ == '__main__':
    unittest.main()
