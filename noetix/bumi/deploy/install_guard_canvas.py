#!/usr/bin/env python3
"""Add the Bumi automatic greeting route to an Agent Core canvas.

The default is a read-only dry run.  --apply claims the canvas edit lock,
rechecks the saved layout and project state, and adds only missing cards/edges.
It never starts the project or changes tool configuration.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import ssl
import sys
import urllib.error
import urllib.request
import uuid


BUMI_SERVER = "bumi-device-bundle"
PERCEPTION_SERVER = "perception-bundle"
CARD_IDS = {
    "camera": "bumi-guard-camera",
    "auto_guard": "bumi-guard-auto-guard",
    "tts": "bumi-guard-tts",
    "speaker": "bumi-guard-speaker",
}
EDGE_IDS = (
    "conn-bumi-guard-camera-auto-guard",
    "conn-bumi-guard-auto-guard-tts",
    "conn-bumi-guard-tts-speaker",
)


class InstallError(Exception):
    """A condition that must be resolved before the canvas can be edited."""


class AgentCoreClient:
    def __init__(self, base_url: str, token: str = "", ca_file: str = "") -> None:
        if not base_url.startswith(("http://", "https://")):
            raise InstallError("--base-url must start with http:// or https://")
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.ca_file = ca_file

    def request(self, method: str, path: str, body: dict | None = None) -> dict:
        headers = {"Accept": "application/json"}
        if self.token:
            headers["Authorization"] = f"Bearer {self.token}"
        data = None
        if body is not None:
            headers["Content-Type"] = "application/json"
            data = json.dumps(body).encode("utf-8")
        request = urllib.request.Request(
            self.base_url + path, data=data, headers=headers, method=method,
        )
        try:
            context = None
            if self.base_url.startswith("https://"):
                # Agent Core uses its persistent self-signed certificate with
                # CN=phanthy-motus and no localhost SAN. Pin that certificate
                # by file; skip hostname matching but never skip signature check.
                if not self.ca_file:
                    raise InstallError("HTTPS requires --ca-file for Agent Core's certificate")
                context = ssl.create_default_context(cafile=self.ca_file)
                context.check_hostname = False
            with urllib.request.urlopen(request, timeout=15, context=context) as response:
                result = json.load(response)
        except urllib.error.HTTPError as error:
            detail = error.read(4096).decode("utf-8", errors="replace")
            raise InstallError(f"{method} {path}: HTTP {error.code}: {detail}") from error
        except (urllib.error.URLError, TimeoutError, ValueError, OSError) as error:
            raise InstallError(f"{method} {path}: {error}") from error
        if not isinstance(result, dict):
            raise InstallError(f"{method} {path}: expected a JSON object")
        if result.get("code", 200) != 200:
            raise InstallError(
                f"{method} {path}: {result.get('message') or result.get('error') or result}"
            )
        return result

    def get(self, path: str) -> dict:
        return self.request("GET", path)

    def post(self, path: str, body: dict) -> dict:
        return self.request("POST", path, body)


def _registered_device(mcps: list, server_name: str) -> dict:
    matches = [m for m in mcps if isinstance(m, dict) and m.get("server_name") == server_name]
    if len(matches) != 1 or not matches[0].get("id"):
        raise InstallError(f"Expected exactly one registered {server_name} MCP; found {len(matches)}")
    return matches[0]


def _tool(mcp: dict, name: str) -> dict:
    matches = [t for t in mcp.get("tools", []) if isinstance(t, dict) and t.get("name") == name]
    if len(matches) != 1:
        raise InstallError(f"{mcp.get('server_name')}: expected tool {name!r} exactly once")
    return matches[0]


def _port(tool: dict, side: str, fmt: str, *, topic_required: bool = False) -> dict:
    ports = tool.get(side)
    if not isinstance(ports, list) or len(ports) != 1 or not isinstance(ports[0], dict):
        raise InstallError(f"{tool['name']}.{side}: expected exactly one port")
    port = ports[0]
    if port.get("format") != fmt:
        raise InstallError(f"{tool['name']}.{side}: expected {fmt}, got {port.get('format')!r}")
    if topic_required and not port.get("topic"):
        raise InstallError(f"{tool['name']}.{side}: missing runtime topic")
    return port


def _current_state(client: AgentCoreClient) -> None:
    state = client.get("/api/config/project-running")
    if not isinstance(state.get("running"), bool):
        raise InstallError("Agent Core did not report a project running state")
    if state["running"]:
        raise InstallError("Stop the Agent Core project before changing its canvas")


def _layout(client: AgentCoreClient) -> dict:
    result = client.get("/api/canvas/layout")
    data = result.get("data")
    if not isinstance(data, dict):
        raise InstallError("Agent Core returned an invalid canvas layout")
    for key in ("cards", "connections", "execConnections"):
        if not isinstance(data.get(key, []), list):
            raise InstallError(f"Canvas {key} must be a list")
    if not isinstance(data.get("transform", {}), dict):
        raise InstallError("Canvas transform must be an object")
    return data


def _existing_card(cards: list, mcp_id: str, tool_name: str, stable_id: str) -> dict | None:
    by_id = [c for c in cards if c.get("id") == stable_id]
    if by_id and (by_id[0].get("mcpId"), by_id[0].get("toolName")) != (mcp_id, tool_name):
        raise InstallError(f"Card id {stable_id} belongs to a different tool")
    if tool_name == "tts":
        # TTS supports multiple instances; use only our dedicated instance.
        return by_id[0] if by_id else None
    matches = [c for c in cards if (c.get("mcpId"), c.get("toolName")) == (mcp_id, tool_name)]
    if len(matches) > 1:
        raise InstallError(f"Multiple {tool_name} cards already exist on the canvas")
    if by_id and matches and by_id[0] is not matches[0]:
        raise InstallError(f"Conflicting {tool_name} cards on the canvas")
    return matches[0] if matches else None


def _verify_saved_port(card: dict, key: str, fmt: str, tool_name: str,
                       wanted_topic: str | None = None) -> None:
    ports = card.get(key) or []
    if ports and not isinstance(ports, list):
        raise InstallError(f"{tool_name} card has invalid {key}")
    if ports:
        if not isinstance(ports[0], dict) or ports[0].get("format") != fmt:
            raise InstallError(f"Existing {tool_name} card has an incompatible {key} format")
        saved = ports[0].get("topic")
        if wanted_topic and saved and saved != wanted_topic:
            raise InstallError(
                f"Existing {tool_name} card has stale {key} topic {saved!r}; "
                f"expected {wanted_topic!r}"
            )


def _check_incoming(connections: list, card_id: str, wanted_source: str) -> None:
    for edge in connections:
        if edge.get("toCardId") == card_id and (
            edge.get("fromCardId") != wanted_source
            or str(edge.get("fromPortIdx", "0")) != "0"
            or str(edge.get("toPortIdx", "0")) != "0"
        ):
            raise InstallError(f"Card {card_id} already has a different input route")


def make_plan(layout: dict, mcps: list) -> dict:
    """Return the merged layout and a concise review summary, without mutating inputs."""
    bumi = _registered_device(mcps, BUMI_SERVER)
    perception = _registered_device(mcps, PERCEPTION_SERVER)
    camera = _tool(bumi, "camera")
    guard = _tool(bumi, "auto_guard")
    tts = _tool(perception, "tts")
    speaker = _tool(bumi, "speaker")

    camera_port = _port(camera, "topic_out", "image/jpeg", topic_required=True)
    _port(guard, "topic_in", "image/jpeg")
    guard_port = _port(guard, "topic_out", "data/json", topic_required=True)
    _port(tts, "topic_in", "data/json")
    _port(tts, "topic_out", "audio/pcm-16k")
    _port(speaker, "topic_in", "audio/pcm-16k")

    camera_topic = camera_port["topic"]
    guard_topic = guard_port["topic"]
    if (not camera_topic.endswith("/camera/color")
            or not guard_topic.endswith("/auto_guard/text")
            or camera_topic.removesuffix("/camera/color")
               != guard_topic.removesuffix("/auto_guard/text")):
        raise InstallError("Bumi camera and auto_guard report inconsistent ROS namespaces")
    tts_topic = guard_topic + "/tts"

    merged = copy.deepcopy(layout)
    cards = merged.setdefault("cards", [])
    connections = merged.setdefault("connections", [])
    merged.setdefault("execConnections", [])
    merged.setdefault("transform", {})
    if not isinstance(cards, list) or not isinstance(connections, list):
        raise InstallError("Canvas cards and connections must be lists")
    ids = [c.get("id") for c in cards if isinstance(c, dict)]
    if len(ids) != len(cards) or len(ids) != len(set(ids)) or any(not i for i in ids):
        raise InstallError("Canvas contains invalid or duplicate card ids")
    if any(not isinstance(edge, dict) for edge in connections):
        raise InstallError("Canvas contains an invalid connection")

    specs = (
        ("camera", bumi, camera, [], [camera_port]),
        ("auto_guard", bumi, guard, guard["topic_in"], [guard_port]),
        ("tts", perception, tts, tts["topic_in"],
         [{"topic": tts_topic, "format": "audio/pcm-16k"}]),
        ("speaker", bumi, speaker, speaker["topic_in"], []),
    )
    role_ids = {}
    added_cards = []
    expected_inputs = {"camera": None, "auto_guard": ("image/jpeg", camera_topic),
                       "tts": ("data/json", guard_topic),
                       "speaker": ("audio/pcm-16k", tts_topic)}
    expected_outputs = {"camera": ("image/jpeg", camera_topic),
                        "auto_guard": ("data/json", guard_topic),
                        "tts": ("audio/pcm-16k", tts_topic), "speaker": None}
    right_edge = max((c.get("x", 0) for c in cards if isinstance(c.get("x", 0), (int, float))),
                     default=-260) + 340
    row_y = min((c.get("y", 80) for c in cards if isinstance(c.get("y", 80), (int, float))),
                default=80)
    for index, (role, mcp, tool, topic_in, topic_out) in enumerate(specs):
        existing = _existing_card(cards, mcp["id"], role, CARD_IDS[role])
        if existing:
            input_spec = expected_inputs[role]
            output_spec = expected_outputs[role]
            if input_spec:
                _verify_saved_port(existing, "topicIn", input_spec[0], role,
                                   wanted_topic=input_spec[1])
            if output_spec:
                _verify_saved_port(existing, "topicOut", output_spec[0], role,
                                   wanted_topic=output_spec[1])
            role_ids[role] = existing["id"]
            continue
        card = {
            "id": CARD_IDS[role],
            "mcpId": mcp["id"],
            "toolName": role,
            "driverName": mcp.get("server_name") or mcp.get("name") or mcp["id"],
            "x": right_edge + 340 * index,
            "y": row_y,
            "topicIn": copy.deepcopy(topic_in),
            "topicOut": copy.deepcopy(topic_out),
        }
        cards.append(card)
        role_ids[role] = card["id"]
        added_cards.append(role)

    routes = (
        ("camera", "auto_guard", "image/jpeg", camera_topic),
        ("auto_guard", "tts", "data/json", guard_topic),
        ("tts", "speaker", "audio/pcm-16k", tts_topic),
    )
    # A previously wired guard or dedicated TTS could make the same message
    # reach two speech paths.  Leave that graph for the operator to resolve.
    for edge in connections:
        target_card = next((c for c in cards if c["id"] == edge.get("toCardId")), None)
        if (edge.get("fromCardId") == role_ids["auto_guard"]
                and target_card and target_card.get("toolName") == "tts"
                and target_card["id"] != role_ids["tts"]):
            raise InstallError("auto_guard is already connected to another TTS card")
        if (edge.get("fromCardId") == role_ids["tts"]
                and target_card and target_card.get("toolName") == "speaker"
                and target_card["id"] != role_ids["speaker"]):
            raise InstallError("Dedicated guard TTS is already connected to another speaker")
    added_edges = []
    for index, (source, target, fmt, topic) in enumerate(routes):
        source_id, target_id = role_ids[source], role_ids[target]
        _check_incoming(connections, target_id, source_id)
        matches = [edge for edge in connections if edge.get("fromCardId") == source_id
                   and edge.get("toCardId") == target_id]
        if len(matches) > 1:
            raise InstallError(f"Duplicate {source} -> {target} routes already exist")
        if matches:
            edge = matches[0]
            if (str(edge.get("fromPortIdx", "0")) != "0"
                    or str(edge.get("toPortIdx", "0")) != "0"
                    or edge.get("format") != fmt
                    or edge.get("fromTopic") != topic):
                raise InstallError(f"Existing {source} -> {target} route does not match live topics")
            continue
        if any(edge.get("id") == EDGE_IDS[index] for edge in connections):
            raise InstallError(f"Connection id {EDGE_IDS[index]} is already in use")
        connections.append({
            "id": EDGE_IDS[index],
            "fromCardId": source_id,
            "fromPortIdx": "0",
            "toCardId": target_id,
            "toPortIdx": "0",
            "format": fmt,
            "fromTopic": topic,
        })
        added_edges.append(f"{source} -> {target}")

    return {
        "layout": merged,
        "added_cards": added_cards,
        "added_edges": added_edges,
        "topics": {"camera": camera_topic, "text": guard_topic, "audio": tts_topic},
        "card_ids": role_ids,
    }


def install(client: AgentCoreClient, *, apply: bool) -> dict:
    _current_state(client)
    mcps_response = client.get("/api/mcp")
    mcps = mcps_response.get("data")
    if not isinstance(mcps, list):
        raise InstallError("Agent Core returned an invalid MCP registry")

    session_id = "bumi-guard-installer-" + uuid.uuid4().hex
    claimed = False
    try:
        if apply:
            client.post("/api/canvas/claim-edit", {"session_id": session_id})
            claimed = True
            _current_state(client)
        current = _layout(client)
        plan = make_plan(current, mcps)
        if apply and (plan["added_cards"] or plan["added_edges"]):
            _current_state(client)
            body = {key: plan["layout"][key] for key in
                    ("cards", "connections", "execConnections", "transform")}
            body["session_id"] = session_id
            client.post("/api/canvas/layout", body)
        return plan
    finally:
        if claimed:
            client.post("/api/canvas/release-edit", {"session_id": session_id})


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="https://127.0.0.1:15678",
                        help="Agent Core address (default: %(default)s)")
    parser.add_argument("--ca-file", default="/opt/phanthy-motus/data/certs/cert.pem",
                        help="Local Agent Core certificate to trust (default: %(default)s)")
    parser.add_argument("--token", default=os.environ.get("AGENT_CORE_TOKEN", ""),
                        help="Agent Core bearer token (or AGENT_CORE_TOKEN environment variable)")
    parser.add_argument("--apply", action="store_true",
                        help="Save the additive canvas change; default is a read-only dry run")
    args = parser.parse_args(argv)
    try:
        plan = install(AgentCoreClient(args.base_url, args.token, args.ca_file), apply=args.apply)
    except InstallError as error:
        print(f"Canvas installation refused: {error}", file=sys.stderr)
        return 1
    print(json.dumps({
        "mode": "applied" if args.apply else "dry-run",
        "added_cards": plan["added_cards"],
        "added_edges": plan["added_edges"],
        "card_ids": plan["card_ids"],
        "topics": plan["topics"],
        "total_cards": len(plan["layout"]["cards"]),
        "total_connections": len(plan["layout"]["connections"]),
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
