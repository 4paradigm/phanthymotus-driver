"""Build and install a local native Skill through the current app's import API."""
import argparse
import asyncio
import getpass
import datetime
import hashlib
import json
import os
from pathlib import Path
import sys
import time

from person_wave import Client, SLUG, read_token

ROOT = Path(__file__).resolve().parent
FILES = ("person_wave.py", "README.md")


def build():
    manifest = []
    for name in FILES:
        content = (ROOT / name).read_bytes()
        manifest.append({"path": name, "bytes": len(content), "sha256": hashlib.sha256(content).hexdigest()})
    skill = {
        "slug": SLUG,
        "name": "看到人招手",
        "oneLiner": "门口持续迎宾：机器人看到人后挥手并说“您好，欢迎光临！”，不要求对方招手。遇到开始看到人招手、开始迎宾、停止迎宾或查询迎宾状态，先调用 activate_skill(slug=bumi-person-wave)，再按指令运行随附程序。开始命令不是人体检测证据，不直接调用 wave 或 tts。",
        "description": "Bumi 固定站立的正门迎宾场景。复用 vop、motion_state、semantic_action.wave 和 tts/speaker；连续看到人约1秒后挥手并说“您好，欢迎光临！”。同一批人停留只迎宾一次，离开3秒后重新等待，两次尝试至少30秒。不统计进出或识别人名。",
        "version": "1.1.0",
        "category": "interaction",
        "requiredTools": ["vop", "motion_state", "semantic_action", "tts", "speaker"],
        "instruction": (ROOT / "instruction.md").read_text(encoding="utf-8").strip(),
        "files": manifest,
    }
    if len(json.dumps(skill, ensure_ascii=False).encode("utf-8")) > 32768:
        raise ValueError("Native definition exceeds 32 KiB")
    (ROOT / "skill.json").write_text(json.dumps(skill, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return skill


async def store(client, skill, files_root):
    result = await client.json("POST", "/api/skills/import", skill)
    if result.get("code") != 200:
        raise RuntimeError("Skill import failed")
    for entry in skill["files"]:
        path = "/api/skills/files/" + SLUG + "/" + entry["path"]
        async with client.session.put(client.base + path, data=(files_root / entry["path"]).read_bytes(),
                                      headers={"Content-Type": "application/octet-stream"}, allow_redirects=False) as response:
            if response.status != 200 or (await response.json()).get("code") != 200:
                raise RuntimeError("Failed to store skill file: " + entry["path"])


async def install(skill, token, replace=False):
    async with Client("https://127.0.0.1:15678", token) as client:
        previous = (await client.json("GET", "/api/skills/" + SLUG)).get("data", {})
        backup = None
        if previous and replace and any(previous.get(k) != v for k, v in skill.items()):
            # The app's import API intentionally refuses conflicting manifests.
            # Use its uninstall/import lifecycle, after backing up and verifying quiescence.
            state_result = await client.json("POST", "/api/file/read", {
                "path": "/work/resource/skill_files/" + SLUG + "/runtime/state.json"})
            state = json.loads(state_result.get("data", {}).get("content", "{}"))
            if not state or state.get("running") is not False or state.get("wave_may_still_be_finishing"):
                raise RuntimeError("Stop greeting and confirm the previous action has ended before replacing files")
            if time.time() - state.get("last_wave_wall", 0) < 30:
                raise RuntimeError("Wait for the previous greeting's 30-second cooldown before replacing files")
            backup = ROOT / "backups" / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            backup.mkdir(parents=True, exist_ok=False)
            previous_definition = {k: v for k, v in previous.items()
                                   if k in set(skill) | {"compatibility", "attachment", "origin"}}
            (backup / "skill.json").write_text(json.dumps(previous_definition, ensure_ascii=False, indent=2), encoding="utf-8")
            (backup / "runtime-state.json").write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
            for entry in previous.get("files", []):
                async with client.session.get(client.base + "/api/file/download", params={
                    "path": "/work/resource/skill_files/" + SLUG + "/" + entry["path"]}, allow_redirects=False) as response:
                    if response.status != 200:
                        raise RuntimeError("Failed to back up existing skill file")
                    content = await response.read()
                if len(content) != entry["bytes"] or hashlib.sha256(content).hexdigest() != entry["sha256"]:
                    raise RuntimeError("Existing skill file does not match its manifest")
                dest = backup / entry["path"]
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(content)
            await client.json("POST", "/api/skills/deactivate", {"slug": SLUG})
            await client.json("POST", "/api/skills/uninstall", {"slug": SLUG})
        try:
            await store(client, skill, ROOT)
        except Exception:
            if backup:
                await client.json("POST", "/api/skills/uninstall", {"slug": SLUG})
                await store(client, previous_definition, backup)
                if not previous.get("active"):
                    await client.json("POST", "/api/skills/deactivate", {"slug": SLUG})
            raise
        installed = (await client.json("GET", "/api/skills/" + SLUG)).get("data", {})
        stored = (await client.json("GET", "/api/skills/files/" + SLUG)).get("data", [])
        if installed.get("instruction") != skill["instruction"] or installed.get("active") is not True:
            raise RuntimeError("Installed instruction or active state does not match")
        if len(stored) != len(skill["files"]) or not all(x.get("stored") is True for x in stored):
            raise RuntimeError("One or more declared files are missing")
        return {"ok": True, "slug": SLUG, "name": skill["name"], "active": True,
                "stored_files": [x["path"] for x in stored], "backup": str(backup) if backup else None,
                "greeting_started": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--replace", action="store_true", help="Back up and replace an installed, stopped skill")
    args = parser.parse_args()
    try:
        skill = build()
        if args.build_only:
            print(json.dumps({"ok": True, "definition": str(ROOT / "skill.json"), "files": skill["files"]}, ensure_ascii=False, indent=2))
            return 0
        try:
            token = read_token()
        except RuntimeError:
            token = getpass.getpass("Dashboard Access Token: ")
        result = asyncio.run(install(skill, token, args.replace))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
