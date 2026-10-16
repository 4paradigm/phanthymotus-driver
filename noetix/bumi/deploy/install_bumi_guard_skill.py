#!/usr/bin/env python3
"""Install the sibling Bumi guard skill into Agent Core's local SQLite DB.

The default is a read-only dry run.  --apply changes only config['skills'] in
one transaction.  It does not restart Agent Core or publish to Resource Center.
"""

from __future__ import annotations

import argparse
import datetime
import json
import pathlib
import re
import sqlite3
import sys


DEFAULT_DB = pathlib.Path("/opt/phanthy-motus/data/data.db")
SKILL_FILE = pathlib.Path(__file__).with_name("bumi_guard_skill.json")
CONTENT_FIELDS = (
    "name", "description", "icon", "oneLiner", "instruction", "category",
    "version", "requiredTools", "configSchema", "author",
)


class InstallError(Exception):
    """A condition that must be resolved before the skill can be installed."""


def load_skill(path: pathlib.Path = SKILL_FILE) -> dict:
    """Read and validate a local skill JSON file, without editing it."""
    try:
        with path.open("r", encoding="utf-8") as stream:
            source = json.load(stream)
    except (OSError, ValueError) as error:
        raise InstallError(f"Cannot read skill JSON {path}: {error}") from error
    if not isinstance(source, dict):
        raise InstallError("Skill JSON must be an object")
    for field in ("slug", "name", "oneLiner", "instruction"):
        if not isinstance(source.get(field), str) or not source[field].strip():
            raise InstallError(f"Skill JSON needs a nonempty string {field!r}")
    slug = source["slug"].strip()
    if not re.fullmatch(r"[a-z0-9-]+", slug) or slug != source["slug"]:
        raise InstallError("Skill slug must match [a-z0-9-]+")
    if "active" in source and not isinstance(source["active"], bool):
        raise InstallError("Skill active must be a boolean")
    required_tools = source.get("requiredTools", [])
    if (not isinstance(required_tools, list)
            or any(not isinstance(tool, str) or not tool.strip() or tool != tool.strip()
                   for tool in required_tools)
            or len(required_tools) != len(set(required_tools))):
        raise InstallError("Skill requiredTools must contain unique nonempty strings")
    schema = source.get("configSchema")
    if schema is not None and not isinstance(schema, dict):
        raise InstallError("Skill configSchema must be an object or null")
    author = source.get("author", "")
    if isinstance(author, dict):
        author = author.get("name", "")
    if not isinstance(author, str):
        raise InstallError("Skill author must be a string or an object with name")
    values = {
        "slug": slug,
        "name": source["name"],
        "description": source.get("description", ""),
        "icon": source.get("icon"),
        "oneLiner": source["oneLiner"],
        "instruction": source["instruction"],
        "category": source.get("category", ""),
        "version": source.get("version", "1.0.0"),
        "requiredTools": required_tools,
        "configSchema": schema,
        "author": author,
    }
    for field in ("description", "oneLiner", "instruction", "category", "version"):
        if not isinstance(values[field], str):
            raise InstallError(f"Skill {field} must be a string")
    if values["icon"] is not None and not isinstance(values["icon"], str):
        raise InstallError("Skill icon must be a string or null")
    if "active" in source:
        values["active"] = source["active"]
    return values


def _connect(db_path: pathlib.Path, *, apply: bool) -> sqlite3.Connection:
    path = db_path.expanduser().resolve()
    if not path.is_file():
        raise InstallError(f"Agent Core DB does not exist: {path}")
    mode = "rw" if apply else "ro"
    try:
        conn = sqlite3.connect(path.as_uri() + f"?mode={mode}", uri=True, timeout=10)
        if not apply:
            conn.execute("PRAGMA query_only=ON")
        return conn
    except sqlite3.Error as error:
        raise InstallError(f"Cannot open Agent Core DB {path}: {error}") from error


def _validate_agent_core_db(conn: sqlite3.Connection) -> None:
    """Fail closed rather than creating an Agent Core config in an unrelated DB."""
    tables = {row[0] for row in conn.execute(
        "SELECT name FROM sqlite_master WHERE type='table'"
    )}
    if not {"config", "chat_sessions", "chat_messages"} <= tables:
        raise InstallError("DB is not an Agent Core database (required tables missing)")
    columns = {row[1]: row for row in conn.execute("PRAGMA table_info(config)")}
    if not {"key", "value"} <= set(columns) or columns["key"][5] != 1:
        raise InstallError("DB has an incompatible Agent Core config table")
    identity = dict(conn.execute(
        "SELECT key, value FROM config WHERE key IN ('core', 'services')"
    ).fetchall())
    if set(identity) != {"core", "services"}:
        raise InstallError("DB is not an initialized Agent Core database")
    try:
        core = json.loads(identity["core"])
        services = json.loads(identity["services"])
    except ValueError as error:
        raise InstallError("Agent Core identity rows contain invalid JSON") from error
    if (not isinstance(core, dict) or "configured" not in core
            or not isinstance(services, dict) or not isinstance(services.get("mcp"), list)):
        raise InstallError("DB does not contain Agent Core configuration")


def _read_skills(conn: sqlite3.Connection) -> tuple[dict, bool]:
    row = conn.execute("SELECT value FROM config WHERE key='skills'").fetchone()
    if row is None:
        return {"installed": []}, False
    try:
        config = json.loads(row[0])
    except ValueError as error:
        raise InstallError("Agent Core skills row contains invalid JSON") from error
    if not isinstance(config, dict) or not isinstance(config.get("installed"), list):
        raise InstallError("Agent Core skills row has an invalid installed list")
    skills = config["installed"]
    if any(not isinstance(item, dict) or not isinstance(item.get("slug"), str)
           for item in skills):
        raise InstallError("Agent Core installed skills contain an invalid record")
    return config, True


def _plan(config: dict, skill: dict, *, replace: bool) -> tuple[dict, str]:
    installed = config["installed"]
    existing = [item for item in installed if item["slug"] == skill["slug"]]
    if len(existing) > 1:
        raise InstallError(f"Multiple installed skills already use slug {skill['slug']!r}")
    updated = dict(config)
    updated["installed"] = list(installed)
    if not existing:
        record = dict(skill)
        record.setdefault("active", True)
        record["installedAt"] = datetime.datetime.now().isoformat()
        updated["installed"].append(record)
        return updated, "install"

    old = existing[0]
    same_content = all(old.get(field) == skill[field] for field in CONTENT_FIELDS)
    # The manifest's active flag is an installation default.  Once installed,
    # Agent Core's UI owns it; a later operator activation is not a conflict.
    if same_content:
        return updated, "unchanged"
    if not replace:
        raise InstallError(
            f"Skill slug {skill['slug']!r} already has different content; "
            "use --replace to update that record"
        )
    record = dict(old)
    record.update({key: value for key, value in skill.items() if key != "active"})
    record.setdefault("installedAt", datetime.datetime.now().isoformat())
    index = installed.index(old)
    updated["installed"][index] = record
    return updated, "replace"


def install(db_path: pathlib.Path, skill_path: pathlib.Path = SKILL_FILE,
            *, apply: bool = False, replace: bool = False) -> dict:
    """Preview or transactionally write only the target skill record."""
    skill = load_skill(skill_path)
    conn = _connect(db_path, apply=apply)
    try:
        if apply:
            conn.execute("BEGIN IMMEDIATE")
        _validate_agent_core_db(conn)
        config, existed = _read_skills(conn)
        updated, action = _plan(config, skill, replace=replace)
        if apply and action != "unchanged":
            encoded = json.dumps(updated, ensure_ascii=False)
            if existed:
                conn.execute("UPDATE config SET value=? WHERE key='skills'", (encoded,))
            else:
                conn.execute("INSERT INTO config (key, value) VALUES ('skills', ?)", (encoded,))
        if apply:
            conn.commit()
        return {
            "mode": "applied" if apply else "dry-run",
            "action": action,
            "slug": skill["slug"],
            "installed_count": len(updated["installed"]),
            "db": str(db_path.expanduser().resolve()),
        }
    except (sqlite3.Error, InstallError) as error:
        if apply:
            conn.rollback()
        if isinstance(error, InstallError):
            raise
        raise InstallError(f"Agent Core DB operation failed: {error}") from error
    finally:
        conn.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", type=pathlib.Path, default=DEFAULT_DB,
                        help="Agent Core SQLite path (default: %(default)s)")
    parser.add_argument("--apply", action="store_true",
                        help="Write the skill; default is a read-only preview")
    parser.add_argument("--replace", action="store_true",
                        help="Allow updating a different skill with the same slug")
    args = parser.parse_args(argv)
    try:
        result = install(args.db, apply=args.apply, replace=args.replace)
    except InstallError as error:
        print(f"Skill installation refused: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
