#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Remove the g1-award-ceremony skill from the agent-core skills DB."""
import sqlite3
import json
import shutil
import time

SLUG = "g1-award-ceremony"
DB_PATH = "/opt/phanthy-motus/data/data.db"

conn = sqlite3.connect(DB_PATH)
row = conn.execute("SELECT value FROM config WHERE key='skills'").fetchone()
if not row:
    conn.close()
    raise SystemExit("ERROR: config key='skills' not found in " + DB_PATH)

skills = json.loads(row[0])
backup = DB_PATH + ".bak-remove-" + SLUG + "-" + time.strftime("%Y%m%d-%H%M%S")
shutil.copy2(DB_PATH, backup)

before = len(skills.get("installed", []))
skills["installed"] = [s for s in skills.get("installed", []) if s.get("slug") != SLUG]
after = len(skills["installed"])

if before == after:
    print('WARNING: skill "%s" not found, nothing to remove' % SLUG)
else:
    conn.execute(
        "UPDATE config SET value=? WHERE key='skills'",
        (json.dumps(skills, ensure_ascii=False),),
    )
    conn.commit()
    print("backup     :", backup)
    print('OK: removed "%s" (%d -> %d)' % (SLUG, before, after))

conn.close()
