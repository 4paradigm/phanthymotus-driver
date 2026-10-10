"""SQLite installer regression tests; no robot I/O or model validation."""
from contextlib import contextmanager
import json
import sqlite3
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from install_g1_shift_handover import install, new_skill

@contextmanager
def db_connect(path):
    conn = sqlite3.connect(path)
    try:
        with conn:
            yield conn
    finally:
        conn.close()

class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.db = Path(self.tmp.name) / 'data.db'
        with db_connect(self.db) as conn:
            conn.execute('CREATE TABLE config(key TEXT PRIMARY KEY, value TEXT)')

    def seed(self, value):
        with db_connect(self.db) as conn:
            conn.execute("INSERT INTO config VALUES('skills',?)", (json.dumps(value),))

    def read(self):
        with db_connect(self.db) as conn:
            return json.loads(conn.execute("SELECT value FROM config WHERE key='skills'").fetchone()[0])

    def test_missing_key_is_created_disabled(self):
        result = install(self.db)
        self.assertFalse(result['active'])
        self.assertEqual(self.read()['installed'][0]['version'], '1.1.3')
        self.assertTrue(Path(result['backup']).is_file())

    def test_reinstall_preserves_other_data_and_disabled_state(self):
        other = {'slug': 'other', 'active': True, 'instruction': 'unchanged'}
        self.seed({'installed': [other, {'slug':'g1-shift-handover', 'active':False, 'installedAt':'old', 'custom':42}], 'extra':1})
        install(self.db, backup=False)
        install(self.db, backup=False)
        cfg = self.read()
        self.assertEqual(cfg['installed'][0], other)
        self.assertEqual(cfg['extra'], 1)
        sentry = cfg['installed'][1]
        self.assertFalse(sentry['active'])
        self.assertEqual(sentry['custom'], 42)
        self.assertEqual(sentry['installedAt'], 'old')
        self.assertEqual(len(cfg['installed']), 2)

    def test_backup_contains_original(self):
        original = {'installed': [{'slug':'g1-shift-handover','active':True,'instruction':'old'}]}
        self.seed(original)
        result = install(self.db)
        with db_connect(result['backup']) as conn:
            value = json.loads(conn.execute("SELECT value FROM config WHERE key='skills'").fetchone()[0])
        self.assertEqual(value, original)
        self.assertTrue(self.read()['installed'][0]['active'])

    def test_invalid_config_is_not_overwritten(self):
        for cfg in ({'installed':{}}, {'installed':[None]}, [], {'installed':[{}]}):
            with self.subTest(cfg=cfg):
                with db_connect(self.db) as conn:
                    conn.execute("INSERT OR REPLACE INTO config VALUES('skills',?)", (json.dumps(cfg),))
                with self.assertRaises(ValueError):
                    install(self.db, backup=False)
                self.assertEqual(self.read(), cfg)

    def test_missing_database_is_not_created(self):
        missing = self.db.with_name('missing.db')
        with self.assertRaises(FileNotFoundError):
            install(missing)
        self.assertFalse(missing.exists())

    def test_syntax_error_rolls_back(self):
        with db_connect(self.db) as conn:
            conn.execute("INSERT INTO config VALUES('skills','not-json')")
        with self.assertRaises(ValueError):
            install(self.db, backup=False)
        with db_connect(self.db) as conn:
            self.assertEqual(conn.execute("SELECT value FROM config WHERE key='skills'").fetchone()[0], 'not-json')


if __name__ == '__main__':
    unittest.main()
