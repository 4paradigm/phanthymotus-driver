"""Hardware-free contract tests for the optional read-only Bumi status binding."""

from __future__ import annotations

import copy
import unittest

from noetix.bumi.deploy import install_guard_status_binding as binder

from noetix.bumi.tests.test_install_guard_canvas import FakeClient, empty_layout, registry


def prepared():
    layout = empty_layout()
    layout["cards"].append({
        "id": "existing-decision-core", "mcpId": "agentcore",
        "toolName": "decision_core", "x": 10, "y": 20,
    })
    layout["cards"].append({
        "id": "unrelated", "mcpId": "another-device", "toolName": "another-tool",
        "x": 50, "y": 60,
    })
    layout["connections"].append({
        "id": "unrelated-topic", "fromCardId": "unrelated",
        "toCardId": "existing-decision-core", "format": "data/json",
    })
    mcps = registry()
    mcps[0]["tools"].append({
        "name": "guard_status", "type": "actuator",
        "topic_in": [], "topic_out": [],
        "inputSchema": {
            "type": "object",
            "properties": {"action": {"type": "string", "enum": ["status"]}},
            "required": ["action"],
        },
    })
    return layout, mcps


class StatusBindingPlanTests(unittest.TestCase):
    def test_adds_only_status_card_and_exact_exec_edge(self):
        layout, mcps = prepared()
        before = copy.deepcopy(layout)
        plan = binder.make_plan(layout, mcps)
        self.assertEqual(layout, before)
        self.assertEqual(plan["added_cards"], ["guard_status"])
        self.assertEqual(plan["added_edges"], ["decision_core -> guard_status"])
        self.assertEqual(len(plan["layout"]["cards"]), len(layout["cards"]) + 1)
        self.assertEqual(plan["layout"]["cards"][:2], layout["cards"])
        self.assertEqual(plan["layout"]["connections"], layout["connections"])
        self.assertEqual(plan["layout"]["execConnections"], [{
            "id": binder.STATUS_EDGE_ID,
            "fromCardId": "existing-decision-core",
            "toCardId": binder.STATUS_CARD_ID,
            "toToolName": "guard_status",
            "toMcpId": "mcp-bumi-local",
        }])

    def test_repeat_is_idempotent(self):
        layout, mcps = prepared()
        first = binder.make_plan(layout, mcps)
        second = binder.make_plan(first["layout"], mcps)
        self.assertEqual(second["added_cards"], [])
        self.assertEqual(second["added_edges"], [])
        self.assertEqual(second["layout"], first["layout"])

    def test_legacy_layout_without_optional_lists(self):
        layout, mcps = prepared()
        legacy = {"cards": layout["cards"][:1]}
        plan = binder.make_plan(legacy, mcps)
        self.assertEqual(plan["layout"]["connections"], [])
        self.assertEqual(plan["layout"]["transform"], {})
        self.assertEqual(len(plan["layout"]["execConnections"]), 1)

    def test_requires_existing_unique_decision_core(self):
        layout, mcps = prepared()
        layout["cards"] = [c for c in layout["cards"]
                           if c["toolName"] != "decision_core"]
        with self.assertRaisesRegex(binder.canvas.InstallError, "one existing decision_core"):
            binder.make_plan(layout, mcps)
        layout["cards"].extend([
            {"id": "core-a", "mcpId": "agentcore", "toolName": "decision_core"},
            {"id": "core-b", "mcpId": "agentcore", "toolName": "decision_core"},
        ])
        with self.assertRaisesRegex(binder.canvas.InstallError, "one existing decision_core"):
            binder.make_plan(layout, mcps)

    def test_rejects_public_mutating_action(self):
        layout, mcps = prepared()
        schema = mcps[0]["tools"][-1]["inputSchema"]
        schema["properties"]["action"]["enum"].append("start")
        with self.assertRaisesRegex(binder.canvas.InstallError, "read-only status"):
            binder.make_plan(layout, mcps)

    def test_rejects_split_status_schema(self):
        layout, mcps = prepared()
        mcps[0]["tools"][-1]["inputSchema"]["x-action-params"] = {
            "status": {"params": []},
        }
        with self.assertRaisesRegex(binder.canvas.InstallError, "plain status"):
            binder.make_plan(layout, mcps)


class StatusBindingWorkflowTests(unittest.TestCase):
    def test_dry_run_never_posts(self):
        layout, mcps = prepared()
        client = FakeClient(layout, mcps)
        binder.install(client, apply=False)
        self.assertFalse(any(call[0] == "POST" for call in client.calls))

    def test_apply_claims_saves_and_releases_without_starting(self):
        layout, mcps = prepared()
        client = FakeClient(layout, mcps)
        binder.install(client, apply=True)
        posts = [call[1] for call in client.calls if call[0] == "POST"]
        self.assertEqual(posts, [
            "/api/canvas/claim-edit", "/api/canvas/layout",
            "/api/canvas/release-edit",
        ])
        self.assertEqual(len(client.layout["cards"]), len(layout["cards"]) + 1)

    def test_refuses_running_project_before_claim(self):
        layout, mcps = prepared()
        client = FakeClient(layout, mcps, running=True)
        with self.assertRaisesRegex(binder.canvas.InstallError, "Stop the Agent Core project"):
            binder.install(client, apply=True)
        self.assertEqual(client.calls, [("GET", "/api/config/project-running")])


if __name__ == "__main__":
    unittest.main()
