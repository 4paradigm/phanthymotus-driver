"""Check this skill against the real project's event/skills.py without ROS or a live DB."""
import argparse
import asyncio
import importlib.util
import json
from pathlib import Path
import sys
import types


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    args = parser.parse_args()
    skill = json.loads(Path(__file__).with_name("skill.json").read_text(encoding="utf-8"))
    source = args.repo / "agent-core/src/event/skills.py"
    sandbox_config = types.ModuleType("config")
    sandbox_config.main = {"skills": {"installed": [dict(skill, active=True)]}}
    sys.modules["config"] = sandbox_config
    spec = importlib.util.spec_from_file_location("native_skill_contract", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if [s["slug"] for s in module.visible_skills()] != [skill["slug"]]:
        raise RuntimeError("Native runtime does not list the installed skill")
    if module.get_active_skills():
        raise RuntimeError("Full instruction should be lazy-loaded")
    asyncio.run(module.Tools().activate_skill(skill["slug"]))
    loaded = module.get_active_skills()
    if len(loaded) != 1 or loaded[0]["instruction"] != skill["instruction"]:
        raise RuntimeError("Native activate_skill did not load the exact instruction")
    module.active_skills.discard(skill["slug"])
    if module.visible_skills() or module.get_active_skills():
        raise RuntimeError("Native deactivation did not hide the skill")
    print("PASS: native visibility, lazy activation, exact instruction, deactivation")


if __name__ == "__main__":
    main()
