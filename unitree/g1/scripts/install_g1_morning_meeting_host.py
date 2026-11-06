#!/usr/bin/env python3
"""Install the G1 morning meeting host Skill into an Agent Core database."""
from contextlib import closing
import argparse
import copy
import datetime
import json
import sqlite3
import uuid
from pathlib import Path


SKILL_PATH = Path(__file__).with_name('g1_morning_meeting_host.json')
new_skill = json.loads(SKILL_PATH.read_text(encoding='utf-8'))


def install(db_path, *, backup=True):
    path = Path(db_path).resolve(strict=True)
    conn = sqlite3.connect(path.as_uri() + '?mode=rw', uri=True, timeout=10)
    backup_path = None
    try:
        row = conn.execute("SELECT value FROM config WHERE key='skills'").fetchone()
        if backup:
            suffix = '.g1-morning-meeting-host-' + uuid.uuid4().hex[:12] + '.bak'
            backup_path = path.with_name(path.name + suffix)
            with closing(sqlite3.connect(backup_path)) as target:
                conn.backup(target)
        conn.execute('BEGIN IMMEDIATE')
        cfg = json.loads(row[0]) if row else {'installed': []}
        if not isinstance(cfg, dict) or not isinstance(cfg.get('installed'), list):
            raise ValueError('Malformed skills config; refusing to overwrite')
        if not all(isinstance(skill, dict) and isinstance(skill.get('slug'), str)
                   for skill in cfg['installed']):
            raise ValueError('Malformed installed entry; refusing to overwrite')

        previous = next((skill for skill in cfg['installed']
                         if skill['slug'] == new_skill['slug']), {})
        skill = {**previous, **copy.deepcopy(new_skill)}
        skill['active'] = previous.get('active') is True
        skill['installedAt'] = previous.get('installedAt') or datetime.datetime.now(
            datetime.timezone.utc).isoformat()
        cfg['installed'] = [item for item in cfg['installed']
                            if item['slug'] != skill['slug']] + [skill]
        value = json.dumps(cfg, ensure_ascii=False, allow_nan=False)
        if row:
            conn.execute("UPDATE config SET value=? WHERE key='skills'", (value,))
        else:
            conn.execute("INSERT INTO config(key,value) VALUES('skills',?)", (value,))
        conn.commit()
        result = json.loads(conn.execute(
            "SELECT value FROM config WHERE key='skills'").fetchone()[0])
        if result != cfg:
            raise RuntimeError('Read-back verification failed')
        return {'slug': skill['slug'], 'version': skill['version'],
                'active': skill['active'],
                'backup': str(backup_path) if backup_path else None}
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--db', default='/opt/phanthy-motus/data/data.db')
    args = parser.parse_args()
    result = install(args.db)
    print(json.dumps(result, ensure_ascii=False))
    print('A fresh install stays disabled; verify the Skill and canvas before activation.')


if __name__ == '__main__':
    main()
