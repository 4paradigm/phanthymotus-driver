"""Validate the shipped definition against actual Core validators and schemas."""
import ast
import json
from pathlib import Path
import re
import sys
import types

import pytest

from noetix.bumi.tests import test_loco_destination as harness
from common.traffic_light.plugin import TrafficLightPlugin
from test_traffic_light import load_bumi

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT.parent / "phanthymotus" / "agent-core" / "src"
SKILL = ROOT / "docs" / "skills" / "bumi-smart-traffic-police.json"


def core_functions(path, names, namespace):
    tree = ast.parse(path.read_text())
    chosen = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    assert {node.name for node in chosen} == set(names)
    exec(compile(ast.Module(body=chosen, type_ignores=[]), str(path), "exec"), namespace)
    return namespace


def test_skill_imports_with_the_real_core_validator_without_opening_its_database():
    validator = core_functions(CORE / "api" / "skills.py", ["validate_local_skill"],
                               {"json": json, "re": re,
                                "fastapi": types.SimpleNamespace(HTTPException=ValueError)})["validate_local_skill"]
    definition = json.loads(SKILL.read_text())
    result = validator(definition)
    assert result["slug"] == "bumi-smart-traffic-police"
    assert result["version"] == "1.0.0" and result["name"] == "智能交警"
    slugs = [json.loads(path.read_text()).get("slug") for path in (ROOT/"docs"/"skills").glob("*.json")]
    assert len(slugs) == len(set(slugs))


def test_required_tools_match_actual_plugin_descriptors(monkeypatch):
    module, _ = harness._load_device(monkeypatch)
    monkeypatch.setitem(sys.modules, "device", module)
    main = load_bumi(monkeypatch)
    bundle = main.BumiDeviceBundle({"plugins": {"loco": {"enabled": True},
        "traffic_light": {"enabled": True}}}, "test", None, object(), None)
    descriptors = bundle.get_all_tools()
    by_name = {tool["name"]: tool for tool in descriptors}
    definition = json.loads(SKILL.read_text())
    assert set(definition["requiredTools"]) == {"traffic_light", "loco_relative_move", "traffic_police_gesture"}
    assert all(name in by_name for name in definition["requiredTools"])
    actions = by_name["loco_relative_move"]["inputSchema"]["x-action-params"]
    assert {"observation_begin", "observation_rotate", "observation_status", "observation_end", "stop_move"} <= actions.keys()


def test_observation_actions_are_visible_through_real_core_processor_filter():
    namespace = {"_SYSTEM_ACTIONS": {"start", "stop", "info", "config"},
                 "PARALLEL_PARAM": "concurrent", "_PARALLEL_PARAM_DESC": "test"}
    core_functions(CORE / "mcp_client.py", ["_to_openai_schema", "present_to_llm", "with_parallel_param"], namespace)
    tool = TrafficLightPlugin({}, "", None).get_tools()[0]
    schemas = namespace["_to_openai_schema"]("perception", tool)
    visible = [namespace["present_to_llm"](schema, {"type": "processor"}, schema["name"].split("__")[-1]) for schema in schemas]
    assert all(visible)
    assert {s["name"] for s in visible} == {"mcp__perception__traffic_light__"+a
                                            for a in ("observe", "finish_observation", "latest")}


def test_gesture_actions_are_visible_with_explicit_execute_and_safe_cancel(monkeypatch):
    module, _ = harness._load_device(monkeypatch)
    tool = next(t for t in module.LocoPlugin({}, "test", None, object()).get_tools()
                if t["name"] == "traffic_police_gesture")
    namespace = {"_SYSTEM_ACTIONS": {"start", "stop", "info", "config"},
                 "PARALLEL_PARAM": "concurrent", "_PARALLEL_PARAM_DESC": "test"}
    core_functions(CORE / "mcp_client.py", ["_to_openai_schema", "present_to_llm", "with_parallel_param"], namespace)
    schemas = namespace["_to_openai_schema"]("bumi", tool)
    for schema in schemas:
        action = schema["name"].split("__")[-1]
        assert namespace["present_to_llm"](schema, {"type": "actuator"}, action)
        if action in ("stop", "go"):
            assert schema["parameters"]["required"] == ["execute"]
            assert schema["parameters"]["properties"]["execute"]["const"] is True
    instruction = json.loads(SKILL.read_text())["instruction"]
    assert "任何红灯" in instruction and "任何绿灯" in instruction
    assert "symbol为unknown" in instruction and "人工" in instruction
    assert "observation_end" in instruction and "traffic_police_gesture.cancel" in instruction
    assert "不伪造completed" in instruction
    # P1-2 splits Card readiness from the current playback state. Synchronise
    # only those field checks; traffic decisions and cleanup stay unchanged.
    assert "traffic_police_gesture.info的state为ready、gesture_state为idle、busy=false" in instruction
    assert "再读info确认state=ready、gesture_state=idle且busy=false" in instruction
    assert "traffic_police_gesture.info为idle" not in instruction


@pytest.mark.parametrize("required", [
    "observation_begin", "observation_rotate", "observation_status", "observation_end", "stop_move",
    "finish_observation", "stop_confirmed", "released", "request_id", "session_id", "action_id",
    "frame_stamp", "observation_after_ns", "unknown", "bulbs", "120秒", "最多3次", "30度", "10度",
    "用户中断", "超时", "禁止巡逻", "身体平移", "不能", "置信度", "稳定性",
])
def test_instruction_contains_full_observe_rotate_cleanup_and_reporting_contract(required):
    instruction = json.loads(SKILL.read_text())["instruction"]
    assert required in instruction
