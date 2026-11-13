"""Real Core handshake/rollback/removed-card code, fake SDK and memory MCP.

Read and compile selected functions from the sibling Core checkout, as the
Skill contract tests do. Never import its service, open its DB, or start a
Driver server. The startup decision and final rollback guard are Core source,
not a test reimplementation of its readiness rules.
"""
import ast
import asyncio
from copy import deepcopy
import json
from pathlib import Path
import sys
import types

import pytest

from test_bumi_traffic_police_gesture import robot, wait_until  # noqa: F401 - shared fake SDK fixture
from test_traffic_light import load_bumi, rpc_for

ROOT = Path(__file__).resolve().parents[1]
CORE_CONFIG = ROOT.parent / "phanthymotus/agent-core/src/api/config.py"
CARD = {"id": "gesture-card", "toolName": "traffic_police_gesture", "mcpId": "bumi"}


def load_core_contract(namespace):
    tree = ast.parse(CORE_CONFIG.read_text())
    functions = {node.name: node for node in tree.body
                 if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}
    project = functions["_start_project_run"]
    nested = {node.name: node for node in project.body
              if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}
    module_names = ["payload_of", "tool_state_of", "start_outcome", "stop_rules",
                    "is_declined", "is_pending", "_bound_inputs", "_control_topics",
                    "_dropped_inputs", "_stop_cards", "stop_removed_cards"]
    nested_names = ["_resolve_and_register", "_try_resolve", "_topic_clash", "_start_and_resolve"]
    # Copy the actual project's final if-errors/rollback + success tail verbatim.
    guard_index = next(i for i, node in enumerate(project.body)
                       if isinstance(node, ast.If) and isinstance(node.test, ast.Name)
                       and node.test.id == "errors")
    finish = ast.AsyncFunctionDef(
        name="finish_start", args=ast.arguments(posonlyargs=[], args=[], kwonlyargs=[],
                                               kw_defaults=[], defaults=[]),
        body=deepcopy(project.body[guard_index:]), decorator_list=[])
    constant_names = {"_STOP_FAILED_STATES", "_STOP_PENDING_STATES", "_STOP_POLL_INTERVAL_S",
                      "STOP_CARD_TIMEOUT_S"}
    constants = [node for node in tree.body if isinstance(node, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id in constant_names for t in node.targets)]
    assert len(constants) == len(constant_names)
    selected = [*constants, *(functions[name] for name in module_names),
                *(nested[name] for name in nested_names), finish]
    module = ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[]))
    exec(compile(module, str(CORE_CONFIG), "exec"), namespace)
    return namespace


@pytest.fixture
def core(monkeypatch, robot):
    monkeypatch.setitem(sys.modules, "device", robot.module)
    main = load_bumi(monkeypatch)
    bundle = main.BumiDeviceBundle({"plugins": {"loco": {"enabled": True},
        "traffic_police_gesture": {"stop_recording_id": None, "go_recording_id": None,
                                   "playback_timeout_s": 5}}}, "core_test", None, robot.ctrl, None)
    plugin = bundle._plugins[0]
    plugin._observation.stop_timeout_s = .025
    monkeypatch.setattr(plugin, "_send_edge_and_wait", robot.edge)
    rpc = rpc_for(main, bundle)
    tools = rpc("tools/list")["result"]["tools"]
    assert any(tool["name"] == CARD["toolName"] for tool in tools)
    calls, events, rollbacks = [], [], []
    info_override = {}

    async def transport(mcp_id, request, **kwargs):
        assert mcp_id == CARD["mcpId"] and request.tool == CARD["toolName"]
        args = dict(request.arguments)
        response = rpc("tools/call", {"name": request.tool, "arguments": args})
        assert "error" not in response
        content = response["result"]["content"]
        payload = json.loads(content[0]["text"])
        if args["action"] == "info":
            payload.update(info_override)
            content = [{"type": "text", "text": json.dumps(payload)}]
        calls.append((dict(request.arguments), payload))
        return {"code": 200, "data": content}

    async def push(event):
        events.append(event)

    async def rollback(**kwargs):
        rollbacks.append(kwargs)

    async def owned_wait(run, awaitable):
        return await awaitable

    class Run:
        pending = {}
        ended = False

        async def wait(self, awaitable):
            return await awaitable

        def check(self):
            pass

        def track(self, task):
            task.cancel()
            pytest.fail("A ready gesture Card must not enter the loading poll")

    run = Run()
    namespace = load_core_contract({"run": run, "_asyncio": asyncio,
        "errors": [], "resolved_info": {}, "resolved_topics": {}, "cards": [dict(CARD)],
        "INFO_TIMEOUT_S": 10, "MCPCallRequest": types.SimpleNamespace,
        "mcp_call_tool": transport, "push_event": push, "_do_stop_project": rollback,
        "_invalidate_start_run": lambda *args, **kwargs: None,
        "config": types.SimpleNamespace(main={}), "_wait_start_owned": owned_wait,
        "find_tool": lambda mcp_id, name: next(t for t in tools if t["name"] == name)})
    # _stop_cards has its own imports. Replace only these I/O boundaries.
    api = types.ModuleType("api")
    api.__path__ = []
    manage = types.ModuleType("api.mcp_manage")
    manage.mcp_call_tool, manage.MCPCallRequest = transport, types.SimpleNamespace
    stream = types.ModuleType("api.motus_stream")
    stream.push_event = push
    for name, module in [("api", api), ("api.mcp_manage", manage), ("api.motus_stream", stream)]:
        monkeypatch.setitem(sys.modules, name, module)
    yield types.SimpleNamespace(ns=namespace, plugin=plugin, rpc=rpc, calls=calls, events=events,
        rollbacks=rollbacks, override=info_override, robot=robot)
    robot.ctrl.mode, robot.ctrl.fault, robot.ctrl.speed = 2, 0, 0.
    plugin._traffic_gesture.shutdown()
    delivery_worker = plugin._traffic_gesture._delivery.worker
    if delivery_worker:
        delivery_worker.join(1)


def start_project(core):
    async def start():
        await core.ns["_start_and_resolve"](dict(CARD))
        return await core.ns["finish_start"]()
    return asyncio.run(start())


def business(core, action):
    result = core.rpc("tools/call", {"name": CARD["toolName"], "arguments": {
        "action": action, "execute": True, "instance_id": CARD["id"]}})
    return json.loads(result["result"]["content"][0]["text"])


@pytest.mark.parametrize("configured", [False, True])
def test_real_core_start_info_and_project_success_never_roll_back(core, configured):
    if configured:
        core.plugin._traffic_gesture.config.update(stop_recording_id=101, go_recording_id=202)
    assert start_project(core) is True
    assert [args for args, _ in core.calls] == [
        {"action": "start", "instance_id": CARD["id"]},
        {"action": "info", "instance_id": CARD["id"]}]
    for _, payload in core.calls:
        assert payload["state"] == "ready" and payload["gesture_state"] == "idle"
        assert payload["started"] and not payload["busy"]
        assert payload["stop_configured"] is configured and payload["go_configured"] is configured
    assert core.ns["resolved_info"][CARD["id"]]["state"] == "ready"
    assert core.ns["errors"] == [] and not core.rollbacks
    assert core.ns["config"].main["core"]["project_running"] is True
    assert core.events[-2:] == [
        {"type": "project_start_done", "payload": {"has_error": False}},
        {"type": "project_state", "payload": {"running": True}}]
    assert not core.robot.events and not core.robot.callbacks and core.plugin._body_action is None
    if not configured:
        for action in ("stop", "go"):
            result = business(core, action)
            assert result["reason"] == "gesture_not_configured"
            assert result["missing_config"] == [action+"_recording_id"]
        assert not core.robot.events


def test_legacy_idle_info_is_rejected_by_the_same_real_core_and_rolls_back(core):
    # A negative control ensures the test would catch the original P1-2 bug.
    core.override["state"] = "idle"
    assert start_project(core) is False
    assert core.calls[0][1]["state"] == "ready" and core.calls[1][1]["state"] == "idle"
    assert core.ns["errors"] == [CARD["toolName"]]
    assert core.rollbacks == [{"expected_run": core.ns["run"]}]
    assert not core.ns["config"].main.get("core", {}).get("project_running", False)
    assert not core.robot.events


@pytest.mark.parametrize("cleanup", ["project_stop", "removed_card"])
@pytest.mark.parametrize("playing", [False, True])
def test_real_core_cleanup_disarms_without_playing_a_stop_gesture(core, cleanup, playing):
    assert start_project(core) is True
    if playing:
        core.plugin._traffic_gesture.config["go_recording_id"] = 202
        assert business(core, "go")["state"] == "running"
    prior_events = list(core.robot.events)
    if cleanup == "project_stop":
        results = asyncio.run(core.ns["_stop_cards"]([dict(CARD)]))
        assert len(results) == 1 and results[0]["ok"]
    else:
        assert asyncio.run(core.ns["stop_removed_cards"]([dict(CARD)], [])) == 1
    assert core.calls[-1][0] == {"action": "stop", "instance_id": CARD["id"]}
    assert core.robot.events == prior_events + ([("WALK", 0)] if playing else [])
    info = core.plugin._traffic_gesture.info()
    assert info["state"] == "idle" and not info["started"]
    assert info["gesture_state"] == "idle" and not info["busy"]
    assert core.plugin._body_action is None
    if playing:
        wait_until(lambda: bool(core.robot.callbacks))
    assert len(core.robot.callbacks) == int(playing)


def test_locked_playback_is_a_real_core_start_error_not_ready(core, monkeypatch):
    assert start_project(core) is True
    core.plugin._traffic_gesture.config["stop_recording_id"] = 101
    assert business(core, "stop")["state"] == "running"
    monkeypatch.setattr(core.plugin._observation, "_settled", lambda: (False, "unconfirmed"))
    result = core.plugin._traffic_gesture.cancel()
    assert result["state"] == "locked"
    core.calls.clear()
    assert start_project(core) is False
    assert all(payload["state"] == "error" and payload["gesture_state"] == "locked"
               for _, payload in core.calls)
    assert core.ns["errors"] == [CARD["toolName"]] and core.rollbacks
    assert core.plugin._body_action == "traffic_police_gesture.stop"


@pytest.mark.parametrize("state,phase,outcome", [
    ("ready", "start", "ready"), ("ready", "info", "error"),
    ("running", "start", "ready"), ("running", "info", "ready"),
    ("idle", "start", "cancelled"), ("idle", "info", "cancelled"),
    ("cancelled", "info", "cancelled"), ("stopped", "start", "error"),
    ("loading", "start", "waiting"), ("starting", "info", "waiting"),
    ("stopping", "info", "waiting"), ("error", "start", "error"),
    (None, "start", "error"), ("unknown", "info", "error"),
])
def test_real_core_state_contract(state, phase, outcome):
    namespace = load_core_contract({})
    assert namespace["start_outcome"](state, phase=phase) == outcome
