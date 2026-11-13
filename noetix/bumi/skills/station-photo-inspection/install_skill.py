"""Install one native PhanthyMotus skill into an existing ConfigDB.

Run on the robot with --auto (Docker), or --db /actual/data.db.
No service restart is needed: ConfigDB reads the skills row on each request.
"""
import argparse
import copy
from contextlib import closing
import datetime as dt
import json
from pathlib import Path, PurePosixPath
import re
import sqlite3
import subprocess
import sys
import uuid


def validate_skill(skill):
    for key in ("slug", "name", "oneLiner", "instruction", "version"):
        if not isinstance(skill.get(key), str) or not skill[key].strip():
            raise ValueError("Missing skill field: " + key)
    if not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", skill["slug"]):
        raise ValueError("Invalid skill slug")
    if "vision_capture" not in skill.get("requiredTools", []):
        raise ValueError("vision_capture dependency is required")


def install(db_path, skill, replace=False):
    validate_skill(skill)
    db_path = Path(db_path).resolve(strict=True)
    if not db_path.is_file():
        raise ValueError("Expected an existing database file")
    now = dt.datetime.now(dt.timezone(dt.timedelta(hours=8)))
    with closing(sqlite3.connect(db_path.as_uri() + "?mode=rw", uri=True, timeout=15)) as conn:
        conn.execute("BEGIN IMMEDIATE")
        keys = {row[0] for row in conn.execute("SELECT key FROM config")}
        if not {"core", "services"}.issubset(keys):
            raise ValueError("This is not the expected PhanthyMotus ConfigDB")
        row = conn.execute("SELECT value FROM config WHERE key=?", ("skills",)).fetchone()
        old_text = row[0] if row else None
        cfg = json.loads(old_text) if old_text is not None else {"installed": []}
        if not isinstance(cfg, dict) or not isinstance(cfg.get("installed"), list):
            raise ValueError("Invalid existing skills configuration; no changes made")
        items = cfg["installed"]
        if any(not isinstance(item, dict) for item in items):
            raise ValueError("Invalid existing skill entry; no changes made")
        matches = [i for i, item in enumerate(items) if item.get("slug") == skill["slug"]]
        if len(matches) > 1:
            raise ValueError("Duplicate slug already exists; no changes made")
        if matches:
            old = items[matches[0]]
            if all(old.get(k) == v for k, v in skill.items()) and old.get("active") is True:
                return {"ok": True, "unchanged": True, "slug": skill["slug"], "db": str(db_path)}
            if not replace:
                raise ValueError("A different version exists; review it, then use --replace")
        backup = db_path.with_name(db_path.name + ".skills-backup-" + now.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:8] + ".json")
        with backup.open("x", encoding="utf-8") as handle:
            json.dump({"key": "skills", "previous_value": old_text}, handle, ensure_ascii=False, indent=2)
        new = copy.deepcopy(skill)
        new.update(active=True, installedAt=now.isoformat())
        if matches:
            new["installedAt"] = items[matches[0]].get("installedAt", new["installedAt"])
            items[matches[0]] = new
        else:
            items.append(new)
        conn.execute("INSERT OR REPLACE INTO config(key,value) VALUES (?,?)",
                     ("skills", json.dumps(cfg, ensure_ascii=False)))
        conn.commit()
    return {"ok": True, "slug": skill["slug"], "name": skill["name"], "active": True,
            "db": str(db_path), "backup": str(backup)}


def bundle_source(skill):
    source = globals().get("_INSTALLER_SOURCE") or Path(__file__).read_text(encoding="utf-8")
    return "_INSTALLER_SOURCE = " + repr(source) + "\n_BUNDLED_SKILL = " + repr(skill) + "\n" + source


def docker_install(skill, container=None, replace=False):
    ids = subprocess.run(["docker", "ps", "-q"], capture_output=True, text=True, timeout=15)
    if ids.returncode:
        raise RuntimeError("Cannot query Docker. Run with Docker permission, or specify --db inside Agent Core.")
    if not ids.stdout.split():
        raise RuntimeError("No running containers; use --db with the actual Agent Core ConfigDB")
    inspect = subprocess.run(["docker", "inspect", *ids.stdout.split()], capture_output=True, text=True, timeout=15, check=True)
    candidates = []
    for item in json.loads(inspect.stdout):
        cfg = item.get("Config", {})
        label = (item.get("Name", "") + " " + cfg.get("Image", "")).lower()
        exposed = cfg.get("ExposedPorts") or {}
        if container:
            matched = item["Id"].startswith(container) or item.get("Name", "").lstrip("/") == container
        else:
            matched = "15678/tcp" in exposed or any(s in label for s in ("agent-core", "agent_core", "agentcore", "motus-core"))
        if matched:
            candidates.append(item)
    if len(candidates) != 1:
        raise RuntimeError("Expected one Agent Core container, found %s. Specify --container NAME." % len(candidates))
    selected = candidates[0]
    cfg = selected["Config"]
    env = dict(value.split("=", 1) for value in cfg.get("Env", []) if "=" in value)
    workdir = cfg.get("WorkingDir") or "/work"
    db = env.get("DB_PATH") or "./resource/data.db"
    if not PurePosixPath(db).is_absolute():
        db = str(PurePosixPath(workdir) / db)
    for executable in (str(PurePosixPath(workdir) / ".venv/bin/python"), "python3"):
        probe = subprocess.run(["docker", "exec", selected["Id"], executable, "--version"], capture_output=True, timeout=10)
        if probe.returncode == 0:
            break
    else:
        raise RuntimeError("No Python interpreter found in the Agent Core container")
    command = ["docker", "exec", "-i", selected["Id"], executable, "-X", "utf8", "-", "--db", db]
    if replace:
        command.append("--replace")
    result = subprocess.run(command, input=bundle_source(skill), text=True, encoding="utf-8", capture_output=True, timeout=45)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "Container installation failed")
    print(result.stdout.strip())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--db", help="Existing ConfigDB; never creates a new database")
    target.add_argument("--auto", action="store_true", help="Find the running Agent Core Docker container")
    parser.add_argument("--container", help="Explicit Docker name or ID, with --auto")
    parser.add_argument("--skill-file", type=Path, default=Path(__file__).with_name("skill.json"))
    parser.add_argument("--replace", action="store_true", help="Replace only this slug, after backing it up")
    args = parser.parse_args()
    try:
        skill = globals().get("_BUNDLED_SKILL")
        if skill is None:
            skill = json.loads(args.skill_file.read_text(encoding="utf-8-sig"))
        validate_skill(skill)
        if args.auto:
            docker_install(skill, args.container, args.replace)
        else:
            print(json.dumps(install(args.db, skill, args.replace), ensure_ascii=False, indent=2))
    except (ValueError, OSError, RuntimeError, sqlite3.Error, subprocess.SubprocessError) as error:
        print("Installation failed: " + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
