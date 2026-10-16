#!/usr/bin/env python3
"""Opt-in, status-only Agent Core canvas binding for the Bumi guard.

The default is a read-only preview. --apply edits the stopped canvas but never
starts a project, changes another card, or grants another tool to decision_core.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
import uuid

try:
    from . import install_guard_canvas as canvas
except ImportError:
    # Direct execution from /deploy, where the script directory is on sys.path.
    import install_guard_canvas as canvas


STATUS_CARD_ID = "bumi-guard-status"
STATUS_EDGE_ID = "exec-bumi-guard-status"


def make_plan(layout: dict, mcps: list) -> dict:
    """Add at most one guard_status card and one executor connection."""
    merged = copy.deepcopy(layout)
    cards = merged.setdefault("cards", [])
    topic_edges = merged.setdefault("connections", [])
    exec_edges = merged.setdefault("execConnections", [])
    transform = merged.setdefault("transform", {})
    if not all(isinstance(value, list) for value in (cards, topic_edges, exec_edges)):
        raise canvas.InstallError("Canvas cards and both connection lists must be lists")
    if not isinstance(transform, dict):
        raise canvas.InstallError("Canvas transform must be an object")
    if not all(isinstance(item, dict) for item in cards + topic_edges + exec_edges):
        raise canvas.InstallError("Canvas contains a malformed card or connection")
    card_ids = [item.get("id") for item in cards]
    if any(not ident for ident in card_ids) or len(card_ids) != len(set(card_ids)):
        raise canvas.InstallError("Canvas contains invalid or duplicate card IDs")

    # Bind only an existing decision core. This grants model access to the
    # read-only tool; it does not authenticate an operator. Never create a
    # controller here: Agent Core starts every card on the next project start.
    cores = [card for card in cards if card.get("mcpId") == "agentcore"
             and card.get("toolName") == "decision_core"]
    if len(cores) != 1:
        raise canvas.InstallError("Expected exactly one existing decision_core card")
    core_id = cores[0]["id"]

    bumi = canvas._registered_device(mcps, canvas.BUMI_SERVER)
    status_tool = canvas._tool(bumi, "guard_status")
    if status_tool.get("type") != "actuator":
        raise canvas.InstallError("guard_status must be advertised as an actuator")
    if status_tool.get("topic_in") or status_tool.get("topic_out"):
        raise canvas.InstallError("guard_status must not have ROS topic ports")
    schema = status_tool.get("inputSchema") or {}
    properties = schema.get("properties") or {}
    action = properties.get("action") or {}
    if (action.get("enum") != ["status"]
            or set(properties) != {"action"}
            or set(schema.get("required") or []) != {"action"}):
        raise canvas.InstallError(
            "guard_status must advertise only the read-only status action"
        )
    if schema.get("x-action-params"):
        raise canvas.InstallError("guard_status must use a plain status action schema")

    existing = canvas._existing_card(cards, bumi["id"], "guard_status", STATUS_CARD_ID)
    added_cards = []
    if existing is None:
        x = max((card.get("x", 0) for card in cards
                 if isinstance(card.get("x", 0), (int, float))), default=-260) + 340
        y = min((card.get("y", 80) for card in cards
                 if isinstance(card.get("y", 80), (int, float))), default=80)
        existing = {
            "id": STATUS_CARD_ID,
            "mcpId": bumi["id"],
            "toolName": "guard_status",
            "driverName": bumi.get("server_name") or bumi.get("name") or bumi["id"],
            "x": x,
            "y": y,
            "topicIn": [],
            "topicOut": [],
        }
        cards.append(existing)
        added_cards.append("guard_status")
    elif existing.get("topicIn") or existing.get("topicOut"):
        raise canvas.InstallError("Existing guard_status card has unexpected topic ports")
    status_id = existing["id"]

    matches = [edge for edge in exec_edges if edge.get("fromCardId") == core_id
               and edge.get("toCardId") == status_id]
    if len(matches) > 1:
        raise canvas.InstallError("Duplicate decision_core to guard_status bindings")
    for edge in exec_edges:
        if (edge.get("toCardId") == status_id and edge.get("fromCardId") != core_id):
            raise canvas.InstallError("guard_status is bound to another controller")
        if (edge.get("fromCardId") == core_id
                and edge.get("toMcpId") == bumi["id"]
                and edge.get("toToolName") == "guard_status"
                and edge.get("toCardId") != status_id):
            raise canvas.InstallError("decision_core already binds another guard_status card")

    added_edges = []
    if matches:
        edge = matches[0]
        if (edge.get("toToolName"), edge.get("toMcpId")) != ("guard_status", bumi["id"]):
            raise canvas.InstallError("Existing guard_status binding has stale MCP metadata")
    else:
        edge_ids = [edge.get("id") for edge in topic_edges + exec_edges]
        if STATUS_EDGE_ID in edge_ids:
            raise canvas.InstallError(f"Connection ID {STATUS_EDGE_ID} is already in use")
        exec_edges.append({
            "id": STATUS_EDGE_ID,
            "fromCardId": core_id,
            "toCardId": status_id,
            "toToolName": "guard_status",
            "toMcpId": bumi["id"],
        })
        added_edges.append("decision_core -> guard_status")
    return {
        "layout": merged,
        "added_cards": added_cards,
        "added_edges": added_edges,
        "card_ids": {"decision_core": core_id, "guard_status": status_id},
    }


def install(client: canvas.AgentCoreClient, *, apply: bool) -> dict:
    # This stops layout mutation while the project runs.  The executor edge is
    # an authorization grant; --apply therefore requires an explicit operator.
    canvas._current_state(client)
    mcps = client.get("/api/mcp").get("data")
    if not isinstance(mcps, list):
        raise canvas.InstallError("Agent Core returned an invalid MCP registry")
    session_id = "bumi-guard-status-installer-" + uuid.uuid4().hex
    claimed = False
    try:
        if apply:
            client.post("/api/canvas/claim-edit", {"session_id": session_id})
            claimed = True
            canvas._current_state(client)
        plan = make_plan(canvas._layout(client), mcps)
        if apply and (plan["added_cards"] or plan["added_edges"]):
            canvas._current_state(client)
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
    parser.add_argument("--base-url", default="https://127.0.0.1:15678")
    parser.add_argument("--ca-file", default="/opt/phanthy-motus/data/certs/cert.pem")
    parser.add_argument("--token", default=os.environ.get("AGENT_CORE_TOKEN", ""))
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args(argv)
    try:
        plan = install(canvas.AgentCoreClient(args.base_url, args.token, args.ca_file),
                       apply=args.apply)
    except canvas.InstallError as error:
        print(f"Status binding refused: {error}", file=sys.stderr)
        return 1
    print(json.dumps({
        "mode": "applied" if args.apply else "dry-run",
        "added_cards": plan["added_cards"],
        "added_edges": plan["added_edges"],
        "card_ids": plan["card_ids"],
        "total_cards": len(plan["layout"]["cards"]),
        "total_exec_connections": len(plan["layout"]["execConnections"]),
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
