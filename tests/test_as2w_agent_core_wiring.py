"""Actual Agent Core canvas wiring and LLM schema conversion, without hardware.

Check out 4paradigm/phanthymotus at PLATFORM_REVISION and install that Core's
test dependencies (FastAPI, aiohttp, openai, jsonschema, python-multipart)
into a test environment:

  PHANTHYMOTUS_CHECKOUT=/path/to/phanthymotus \
    python -m pytest tests/test_as2w_agent_core_wiring.py -q

The real Core config/startup/schema functions and the real dry-run AS2W card
execute here. SQLite is redirected to a temporary database; only the external
MCP/ROS/channel/dashboard boundaries are faked. This does not verify browser
rendering, real DDS discovery, or hardware motion. No production dependencies
or deployed database are modified.
"""
from __future__ import annotations

import asyncio
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import types

import pytest


ROOT = Path(__file__).resolve().parents[1]
PLATFORM_REVISION = "b42effb65b395d0860e36f85ba40f6e10129e55a"
SOURCE_TOPIC = "/actucore/navi/control"


def _load(monkeypatch, name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, name, module)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def project(monkeypatch, tmp_path):
    checkout = os.environ.get("PHANTHYMOTUS_CHECKOUT")
    if not checkout:
        pytest.skip("set PHANTHYMOTUS_CHECKOUT for pinned cross-repository validation")
    checkout = Path(checkout).resolve()
    revision = subprocess.check_output(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
    assert revision == PLATFORM_REVISION, "audit the Core before updating the pin"
    assert subprocess.run(
        ["git", "-C", str(checkout), "diff", "--quiet", "HEAD", "--", "agent-core/src"],
        check=False).returncode == 0, "pinned Core source must have no local changes"
    # An explicitly requested cross-repository run must fail, not silently skip,
    # if the required Core dependencies are missing.
    for dependency in ("fastapi", "aiohttp", "openai", "jsonschema", "python_multipart"):
        __import__(dependency)

    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("DB_PATH", str(tmp_path / "agent-core.db"))
    core_src = checkout / "agent-core/src"
    monkeypatch.syspath_prepend(str(ROOT))
    config = _load(monkeypatch, "config", core_src / "config.py")
    _load(monkeypatch, "event_bus", core_src / "event_bus.py")
    core = _load(monkeypatch, "_as2w_test_core_config", core_src / "api/config.py")
    mcp_client = _load(monkeypatch, "_as2w_test_mcp_client", core_src / "mcp_client.py")

    def install(name, **attributes):
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
        return module

    subscriptions, nodes, calls, events = [], [], [], []

    class Node:
        def __init__(self, name):
            self.name, self.destroyed = name, False

        def create_subscription(self, message_type, topic, callback, qos):
            sub = types.SimpleNamespace(message_type=message_type, topic=topic,
                                        callback=callback, qos=qos)
            subscriptions.append(sub)
            return sub

        def destroy_node(self):
            self.destroyed = True

    class SDK:
        max_call_seconds = .35
        calls = []

        def __getattr__(self, name):
            if name in {"Move", "StopMove", "GetState", "acquire_control", "release_control"}:
                self.calls.append(name)
                raise AssertionError("dry_run must never call SDK: " + name)
            raise AttributeError(name)

    install("rclpy")
    install("rclpy.node", Node=Node)
    install("std_msgs")
    install("std_msgs.msg", String=type("String", (), {}))
    _load(monkeypatch, "as2w_control", ROOT / "unitree/as2w/as2w_control.py")
    servo = _load(monkeypatch, "_as2w_core_wiring_servo", ROOT / "unitree/as2w/loco_servo.py")
    sdk = SDK()
    executor = types.SimpleNamespace(add_node=nodes.append, remove_node=nodes.remove)
    receiver = servo.LocoServoPlugin({}, "", executor, sdk)
    receiver._controller._threaded = False
    starts, source_topics = {}, [{"topic": SOURCE_TOPIC, "format": "control/velocity"}]

    # Replace only the network-facing API. Core itself constructs these requests
    # and the actual AS2W dispatch implementation receives their unmodified args.
    async def call(mcp_id, req, timeout_s=None):
        args = dict(req.arguments)
        action = args["action"]
        calls.append((req.tool, action, args))
        if action == "info":
            assert timeout_s is not None
        if action == "start":
            starts[req.tool] = args
        if req.tool == "loco_servo":
            result = receiver.dispatch(action, args)
        else:
            assert req.tool == "control_source"
            result = {"state": "idle" if action == "stop" else "running",
                      "topic_out": list(source_topics)}
        return {"code": 200, "data": result}

    async def push(event):
        events.append(event)

    async def register(*args):
        pass

    install("api.mcp_manage", mcp_call_tool=call, MCPCallRequest=types.SimpleNamespace)
    install("api.motus_stream", push_event=push)
    install("api.inspection", register_topic_internal=register)
    install("channel.manager",
            manager=types.SimpleNamespace(sync_from_canvas=lambda: None, _adapters={}),
            _get_channel_configs=lambda: [])

    def layout(connected=True):
        # Deliberately reverse source/receiver ordering: Core must resolve the
        # live source output before supplying the receiver's wiring argument.
        cards = [
            {"id": "servo", "toolName": "loco_servo", "mcpId": "as2w"},
            {"id": "source", "toolName": "control_source", "mcpId": "source"},
        ]
        connections = ([{"fromCardId": "source", "toCardId": "servo",
                         "fromPortIdx": "0", "toPortIdx": "0", "fromTopic": ""}]
                       if connected else [])
        config.main["canvas_layout"] = {"cards": cards, "connections": connections}

    yield types.SimpleNamespace(core=core, config=config, receiver=receiver, sdk=sdk,
                                schemas=mcp_client._to_openai_schema, layout=layout,
                                calls=calls, starts=starts, events=events, nodes=nodes,
                                subscriptions=subscriptions, source_topics=source_topics)
    receiver.stop()


def test_actual_core_exposes_only_explicit_runtime_llm_actions(project):
    tool = project.receiver.get_tool()
    schemas = project.schemas("as2w", tool)
    assert {schema["name"] for schema in schemas} == {
        "mcp__as2w__loco_servo__pause", "mcp__as2w__loco_servo__resume",
        "mcp__as2w__loco_servo__reset_fault",
    }
    assert all(schema["parameters"]["properties"] == {} for schema in schemas)
    assert all(schema["parameters"]["required"] == [] for schema in schemas)
    assert "start" in tool["inputSchema"]["properties"]["action"]["enum"]
    assert "input_topic" in tool["inputSchema"]["properties"]
    assert project.sdk.calls == []


def test_actual_core_canvas_start_supplies_topic_without_llm_start_schema(project):
    project.layout()
    assert asyncio.run(project.core._do_start_project()) is True
    assert project.starts["loco_servo"]["input_topic"] == SOURCE_TOPIC
    assert project.subscriptions[0].topic == SOURCE_TOPIC
    info = project.receiver.dispatch("info", {})
    assert info["connected"] and info["dry_run"] and info["state"] == "running"
    assert project.config.main["core"]["project_running"]
    assert project.sdk.calls == []
    node = project.nodes[0]
    asyncio.run(project.core._do_stop_project())
    assert node.destroyed and not project.nodes
    assert not project.receiver.dispatch("info", {})["connected"]


def test_actual_core_unwired_start_fails_and_rolls_back(project):
    project.layout(connected=False)
    assert asyncio.run(project.core._do_start_project()) is False
    assert "input_topic" not in project.starts["loco_servo"]
    assert not project.subscriptions
    errors = [e["payload"] for e in project.events if e["type"] == "project_start_item"
              and e["payload"]["status"] == "error"]
    assert any("input_topic is required" in e["message"] for e in errors)
    assert not project.config.main["core"]["project_running"]
    assert project.sdk.calls == []


def test_actual_core_unresolved_connection_never_starts_receiver(project):
    project.source_topics.clear()
    project.layout()
    assert asyncio.run(project.core._do_start_project()) is False
    assert "loco_servo" not in project.starts
    assert not project.subscriptions
    assert project.sdk.calls == []
