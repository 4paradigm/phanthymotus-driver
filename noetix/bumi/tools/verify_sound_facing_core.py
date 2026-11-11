#!/usr/bin/env python3
"""Isolated real Core/API/Agent integration; fake robot and scripted LLM only.

Requires a disposable Core checkout, ROS Humble + generated audio_msgs, and
loopback-only DDS. Never runs bundle.main or imports either vendor SDK module.
No endpoints, access checks, MCP/ACP handlers or dispatchers are replaced.
Core's service/Docker/network bootstrap is deliberately not started; its
normal ROS bridge is started separately in the isolated domain.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import hashlib
import importlib.abc
import importlib.util
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import socket
import subprocess
import sys
import threading
import time
import types
from http.server import ThreadingHTTPServer


BUMI = Path(__file__).resolve().parents[1]


class NoVendorSDK(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in {"highcontrol_py", "mediacontrol_py"}:
            raise AssertionError("Offline integration forbids vendor SDK imports")


class FakeController:
    """A fake yaw plant. All velocity writes are recorded, never forwarded."""

    def __init__(self):
        self.lock = threading.Lock()
        self.yaw = 0.0
        self.mode = 2
        self.advance = True
        self.commands = []

    def get_mode(self):
        return self.mode

    def get_imu_data(self):
        with self.lock:
            half = math.radians(self.yaw) / 2
            return types.SimpleNamespace(ori=[0, 0, math.sin(half), math.cos(half)],
                                         angular_vel=[0, 0, 0])

    def publish_cmd(self, x, y, z, action_cmd, index=0):
        assert x == y == 0, "This integration must never translate"
        with self.lock:
            self.commands.append((x, y, z))
            if self.advance:
                self.yaw += z * 3


class SyntheticSource:
    def __init__(self):
        self.healthy = True
        self.angle = None

    def dispatch(self, action, args):
        assert action == "info"
        return {"state": "running" if self.healthy else "unavailable",
                "calibrated_directions": ["front", "right"] if self.healthy else []}


async def verify(core_root: Path, output: Path):
    sys.dont_write_bytecode = True
    if not (core_root / "src/start.py").is_file():
        raise ValueError("--core must identify an existing agent-core directory")
    if os.environ.get("ROS_DOMAIN_ID") in (None, "0", "42"):
        raise ValueError("An isolated ROS_DOMAIN_ID is required")
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
        raise ValueError("Use rmw_fastrtps_cpp and the loopback-only fixture")
    profile = Path(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]).read_text()
    if "127.0.0.1" not in profile or "<useBuiltinTransports>false" not in profile:
        raise ValueError("Loopback whitelist and disabled built-in transports are required")
    output.mkdir(parents=True, exist_ok=False)
    runtime = output / "runtime"
    runtime.mkdir()
    (runtime / "web").symlink_to(core_root / "web", target_is_directory=True)
    token = secrets.token_urlsafe(32)
    env_file = runtime / ".env"
    env_file.write_text("ACCESS_TOKEN=" + token + "\n")
    env_file.chmod(0o600)
    os.environ["DB_PATH"] = str(runtime / "data.db")
    os.environ["NO_PROXY"] = os.environ["no_proxy"] = "127.0.0.1,localhost"
    os.chdir(runtime)
    sys.meta_path.insert(0, NoVendorSDK())
    sys.path.insert(0, str(BUMI))
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    import device
    from sound_facing import CONVENTION
    from sound_facing_card import SoundFacingPlugin
    from sound_facing_authority import canonical
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

    # Test-only operator role. The Driver receives only this public key;
    # production signing happens on the separate operator workstation.
    operator_key = Ed25519PrivateKey.generate()
    public_pin = runtime / "synthetic-operator.pub.pem"
    public_pin.write_bytes(operator_key.public_key().public_bytes(
        serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo))
    public_pin.chmod(0o400)
    os.environ["BUMI_SOUND_FACING_OPERATOR_PUBLIC_KEY"] = str(public_pin)

    spec = importlib.util.spec_from_file_location("sound_facing_bundle", BUMI / "main.py")
    bundle_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bundle_module)
    device._get_control_cmd = lambda name: name  # fake SDK enum, no SDK import
    plant, source = FakeController(), SyntheticSource()
    sys.path.insert(0, str(core_root / "src"))
    import config
    config.main["core"] = {"configured": True, "auto_start": False, "main_loop_enable": False}
    config.main["services"] = {"mcp": [], "resource_center": {"url": "http://127.0.0.1:1"}}
    config.main["peer_settings"] = {"enabled": False}
    event_config = config.main.get("event", {})
    config.main["event"] = {**event_config, "llm": {**event_config.get("llm", {}),
                                                   "auto_narration": False, "max_rounds": 2}}
    shutil.copytree(core_root / "resource/memory", runtime / "resource/memory", dirs_exist_ok=True)
    import start
    import auth
    import client
    import mcp_client
    import uvicorn
    import httpx
    from event.llm import Event, _build_system_tools
    from event.desktop import DesktopTools
    import event

    import ros2_bridge
    ros2_bridge.start(asyncio.get_running_loop())
    executor = MultiThreadedExecutor(num_threads=3)
    node = Node("sound_facing_core_test_source")
    executor.add_node(node)
    namespace = "sound_facing_core_test"
    publisher = node.create_publisher(String, f"/{namespace}/sound_direction", qos_profile_sensor_data)
    states = []
    node.create_subscription(String, f"/{namespace}/bumi_sound_facing/state",
                             lambda msg: states.append(json.loads(msg.data)), 1)
    loco = device.LocoPlugin({}, namespace, executor, plant)
    card = SoundFacingPlugin({}, namespace, executor, loco, source_plugin=source)
    bundle = bundle_module.BumiDeviceBundle({"plugins": {}}, namespace, executor, None, None)
    bundle._plugins = [loco, card]
    bundle_module._bundle = bundle
    driver_http = ThreadingHTTPServer(("127.0.0.1", 0), bundle_module.make_handler())
    driver_thread = threading.Thread(target=driver_http.serve_forever, daemon=True)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    halt = threading.Event()

    def publish():
        while not halt.wait(0.25):
            end = time.time_ns() // 1000
            angle = source.angle
            payload = {"state": "no_event"} if angle is None else {
                "state": "fresh", "angle_deg": angle, "angle_convention": CONVENTION,
                "audio_window_start_us": end - 200_000, "audio_window_end_us": end,
                "timestamp_ms": end // 1000}
            publisher.publish(String(data=json.dumps(payload)))

    producer = threading.Thread(target=publish, daemon=True)
    # Use normal token authentication with an isolated token file, never an
    # auth bypass or the machine's /opt/phanthy-motus/.env.
    auth._ENV_PATH = env_file
    auth._ENV_PATH_DEV = env_file
    auth.init()
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    os.environ["AGENT_CORE_URL"] = f"http://127.0.0.1:{port}"
    server = uvicorn.Server(uvicorn.Config(start.app, log_level="warning", lifespan="off"))
    server_task = asyncio.create_task(server.serve(sockets=[sock]))
    cases = []
    observations = []

    def passed(name):
        cases.append(name)
        print("[software-check] " + name, flush=True)

    async def wait(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            await asyncio.sleep(0.02)
        raise AssertionError(card._engine.snapshot())

    try:
        driver_thread.start()
        spin_thread.start()
        producer.start()
        card.start()
        await wait(lambda: server.started)
        base = f"http://127.0.0.1:{port}"
        async with httpx.AsyncClient(base_url=base, trust_env=False, timeout=15) as http:
            assert (await http.get("/api/mcp")).status_code == 401
            http.headers["Authorization"] = "Bearer " + token
            registered = (await http.post("/api/mcp", json={
                "name": "isolated-bumi-sound-facing", "category": "driver",
                "url": f"http://127.0.0.1:{driver_http.server_port}/mcp"})).json()
            mid = registered["data"]["id"]
            await http.post(f"/api/mcp/{mid}/ping")
            await wait(lambda: mid in mcp_client.registry and mcp_client.registry[mid].get("online"))
            info = mcp_client.registry[mid]
            assert "bumi_sound_facing" in info["tools"]
            passed("real_core_registration_and_discovery")

            async def canvas(action, **args):
                response = await http.post(f"/api/mcp/{mid}/call", json={
                    "tool": "bumi_sound_facing", "arguments": {"action": action, **args}})
                result = response.json()
                assert result["code"] == 200, result
                return json.loads(result["data"][0]["text"])

            async def operator_approve(valid_for_s=120):
                prepared = await canvas("prepare_authorization")
                assert "error" not in prepared, prepared
                payload = {k: v for k, v in prepared["authorization"]["request"].items()
                           if k != "max_valid_for_s"}
                payload["valid_for_s"] = valid_for_s
                proof = json.dumps({"payload": payload, "signature": base64.b64encode(
                    operator_key.sign(canonical(payload))).decode("ascii")})
                approved = await canvas("authorize_follow", authorization=proof)
                assert "error" not in approved and not approved["enabled"], approved
                return proof

            assert not (await canvas("start"))["enabled"] and not plant.commands
            assert not (await canvas("info"))["enabled"]
            passed("start_and_registration_do_not_enable")
            full = lambda action: f"mcp__{mid}__bumi_sound_facing__{action}"
            for action in ("enable_follow", "renew_follow", "disable_follow"):
                schema = info["schemas"][full(action)]
                assert schema["parameters"]["type"] == "object"
            assert info["schemas"][full("renew_follow")]["parameters"]["required"] == ["session_id"]
            passed("actual_core_canvas_and_agent_schema_conversion")

            editor = secrets.token_hex(8)
            assert (await http.post("/api/canvas/claim-edit", json={"session_id": editor})).status_code == 200
            layout = {"session_id": editor, "cards": [
                {"id": "core", "mcpId": "agentcore", "toolName": "decision_core"},
                {"id": "facing", "mcpId": mid, "toolName": "bumi_sound_facing"}],
                "execConnections": [{"fromCardId": "core", "toCardId": "facing",
                                      "toMcpId": mid, "toToolName": "bumi_sound_facing"}]}
            agent = Event()
            agent._desktop_tools = DesktopTools()
            agent._sys_tools = _build_system_tools([("finish", event.finish.__call__)])
            assert agent._get_bound_tool_schemas() == []
            saved = await http.post("/api/canvas/layout", json=layout)
            assert saved.status_code == 200, saved.text
            assert full("enable_follow") in {s["name"] for s in agent._get_bound_tool_schemas()}
            passed("real_canvas_binding_controls_agent_tool_visibility")

            async def agent_turn(action, flags=None):
                async def scripted_llm(**kwargs):
                    return {"content": "", "tool_calls": [
                        {"id": "facing", "type": "function", "function": {
                            "name": full(action), "arguments": "{}"}},
                        {"id": "done", "type": "function", "function": {
                            "name": "finish", "arguments": "{}"}}]}
                original = client.call
                client.call = scripted_llm  # deterministic model output only
                agent._current_turn = []
                try:
                    await agent._one_turn({"source": "offline_source", "text": action,
                                           "ts": time.time(), **(flags or {})}, asyncio.Event())
                finally:
                    client.call = original

            await agent_turn("enable_follow")
            assert not card._engine.session_id and not plant.commands
            assert any("operator_authorization_required" in str(m) for m in agent._current_turn)
            passed("ordinary_agent_without_operator_signature_is_rejected")
            assert (await canvas("authorize_follow", authorization='{"authorized":true}'))["error"]
            passed("ordinary_mcp_authorization_forgery_is_rejected")
            await operator_approve()
            await agent_turn("enable_follow")
            assert card._engine.session_id, agent._current_turn
            assert not mcp_client.get_pending_actions()
            passed("real_main_agent_dispatch_enables_with_independent_operator_signature")
            session = card._engine.session_id
            renewed = json.loads(await mcp_client.call_tool(full("renew_follow"), {
                "session_id": session, "_trace_id": "offline-renew"}))
            assert renewed["enabled"] and renewed["session_id"] == session
            wrong = await canvas("renew_follow", session_id="retired-session")
            assert wrong["error"] == "session_not_active_or_mismatched"
            assert (await canvas("renew_follow", session_id=session))["session_id"] == session
            passed("standard_mcp_and_canvas_renew_require_current_session")
            for target in (35, -35, 90):
                plant.advance = target != 35
                source.angle = target
                await wait(lambda: card._engine.state == "turning")
                if target == 35:
                    internal_id = card._engine.action_id
                    for status in ("completed", "error", "completed"):
                        response = await http.post("/api/acp/complete", json={
                            "action_id": internal_id, "tool": "loco_relative_move",
                            "status": status, "result": {"state": "done"}})
                        assert response.status_code == 200
                    assert card._engine.action_id == internal_id and card._engine.turns == 0
                    assert not mcp_client.get_pending_actions()
                    plant.advance = True
                    passed("early_duplicate_network_acp_cannot_finish_internal_turn")
                source.angle = None
                await wait(lambda: card._engine.state == "settling")
                await wait(lambda: card._engine.state in ("observing", "paused"))
            assert card._engine.turns == 3 and not mcp_client.get_pending_actions()
            passed("real_ros_continuous_three_turns_no_orphan_acp")
            await agent_turn("disable_follow")
            await wait(lambda: not card._engine.reserved)
            before = len(plant.commands)
            source.angle = -60
            await asyncio.sleep(1)
            assert len(plant.commands) == before
            passed("agent_disable_stops_future_turns")

            for flag in ("_viewer_channel_event", "_bot_channel_event"):
                await agent_turn("enable_follow", {flag: True})
                assert not card._engine.session_id
                tool_results = [m["content"] for m in agent._current_turn if m.get("role") == "tool"]
                assert any("tool-restricted" in value for value in tool_results), tool_results
                passed("existing_agent_access_control_" + flag)

            await operator_approve()
            assert (await canvas("enable_follow"))["enabled"]
            await wait(lambda: card._engine.state == "turning")
            assert not (await canvas("disable_follow"))["enabled"]
            await wait(lambda: not card._engine.reserved)
            assert plant.commands[-1] == (0, 0, 0)
            before = len(plant.commands)
            await asyncio.sleep(1)
            assert len(plant.commands) == before
            source.angle = None
            passed("canvas_enable_and_cancel_running_owned_turn")

            # Finite public Loco actions retain the real Core ACP contract.
            public_rotate = f"mcp__{mid}__loco_relative_move__rotate"
            action = json.loads(await mcp_client.call_tool(public_rotate, {"angle_deg": 25}))
            aid = action["action_id"]
            assert aid in mcp_client.get_pending_actions()
            completion = await mcp_client.sync([aid], timeout=10)
            assert completion["status"] == "completed", completion
            assert completion["results"][aid]["status"] == "completed"
            assert aid not in mcp_client.get_pending_actions()
            passed("real_core_public_rotation_pending_and_http_acp_terminal")
            for status in ("completed", "error", "cancelled"):
                r = await http.post("/api/acp/complete", json={
                    "action_id": aid, "status": status, "result": {"state": "late"}})
                assert r.status_code == 200
            assert not mcp_client.get_pending_actions() and not card._engine.session_id
            passed("duplicate_and_late_http_acp_cannot_restart_session")

            failed = json.loads(await mcp_client.call_tool(public_rotate, {"angle_deg": 150}))
            plant.mode = 26  # fake protection fault; never sent to a device
            failed_result = await mcp_client.sync([failed["action_id"]], timeout=10)
            assert failed_result["status"] == "completed", failed_result
            assert failed_result["results"][failed["action_id"]]["status"] == "error"
            plant.mode = 2
            passed("real_core_error_acp_preserves_failed_outcome")

            timed = json.loads(await mcp_client.call_tool(public_rotate, {"angle_deg": 150}))
            timed_id = timed["action_id"]
            timeout_result = await mcp_client.sync([timed_id], timeout=0.01)
            assert timeout_result["status"] == "timeout" and timed_id in timeout_result["pending"]
            assert timed_id in mcp_client.get_pending_actions()
            await mcp_client.call_tool(f"mcp__{mid}__loco_relative_move__stop_move", {})
            cancelled = await mcp_client.sync([timed_id], timeout=10)
            assert cancelled["results"][timed_id]["status"] == "cancelled", cancelled
            assert not mcp_client.get_pending_actions()
            passed("core_wait_timeout_then_late_cancelled_acp_retires_pending")

            await canvas("config", session_timeout_s=1)
            await operator_approve()
            assert (await canvas("enable_follow"))["enabled"]
            await wait(lambda: card._engine.reason == "session_expired")
            await wait(lambda: not card._engine.reserved)
            before = len(plant.commands)
            source.angle = 60
            await asyncio.sleep(1)
            assert len(plant.commands) == before
            passed("unmanaged_session_expires_without_automatic_retry")
            stopped = await canvas("stop")
            assert not stopped["running"] and not stopped["enabled"]
            assert card._node is None and not card._worker.is_alive()
            passed("real_core_stop_cleans_resources")
            observations.append({"state_messages": len(states), "fake_velocity_commands": len(plant.commands),
                                 "completed_facing_turns": card._engine.turns})
    finally:
        halt.set()
        if producer.is_alive():
            producer.join(2)
        card.stop()
        driver_http.shutdown()
        driver_http.server_close()
        driver_thread.join(2)
        server.should_exit = True
        await asyncio.wait_for(server_task, 5)
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown(timeout_sec=3)
        spin_thread.join(3)
        bridge_thread = ros2_bridge._thread
        await asyncio.to_thread(ros2_bridge.stop)
        if bridge_thread:
            bridge_thread.join(2)
            assert not bridge_thread.is_alive()
        mcp_client.stop_sse()
    receipt = {"cases_passed": len(cases), "cases": cases, "observations": observations,
               "core_head": subprocess.check_output(["git", "-C", str(core_root), "rev-parse", "HEAD"], text=True).strip(),
               "core_app": "unmodified start.app HTTP/API and Agent dispatcher",
               "core_bootstrap": "lifespan off; actual ROS bridge started; service, Docker, network and peer bootstrap not run",
               "llm": "scripted responses; actual Event._one_turn and access checks",
               "controller": "FakeController; vendor imports forbidden", "hardware_motion": False,
               "authorization": "real Ed25519 verification; synthetic test operator, not a real human ceremony",
               "source_sha256": {name: hashlib.sha256((BUMI / name).read_bytes()).hexdigest()
                                 for name in ("sound_facing.py", "sound_facing_card.py", "sound_facing_authority.py", "device.py",
                                              "tools/verify_sound_facing_core.py")}}
    (output / "receipt.json").write_text(json.dumps(receipt, indent=2))
    print(json.dumps(receipt))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core", required=True, type=Path, help="Disposable agent-core source directory")
    parser.add_argument("--output", required=True, type=Path, help="New task-owned directory")
    args = parser.parse_args()
    asyncio.run(verify(args.core.resolve(), args.output.resolve()))


if __name__ == "__main__":
    main()
