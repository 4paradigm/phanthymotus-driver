"""Hardware-free checks for the additive Bumi canvas installer."""

from __future__ import annotations

import copy
import io
import json
import unittest
from unittest import mock

from noetix.bumi.deploy import install_guard_canvas as installer


def registry(namespace="bumi_01"):
    base = f"/{namespace}"
    return [
        {
            "id": "mcp-bumi-local",
            "server_name": installer.BUMI_SERVER,
            "tools": [
                {"name": "camera", "topic_out": [
                    {"topic": base + "/camera/color", "format": "image/jpeg"}]},
                {"name": "auto_guard", "topic_in": [{"format": "image/jpeg"}],
                 "topic_out": [{"topic": base + "/auto_guard/text", "format": "data/json"}]},
                {"name": "speaker", "topic_in": [{"format": "audio/pcm-16k",
                                                   "message_type": "audio_msgs/msg/AudioChunk"}]},
            ],
        },
        {
            "id": "mcp-perception-local",
            "server_name": installer.PERCEPTION_SERVER,
            "tools": [{"name": "tts", "topic_in": [{"format": "data/json"}],
                       "topic_out": [{"format": "audio/pcm-16k"}]}],
        },
    ]


def empty_layout():
    return {"cards": [], "connections": [], "execConnections": [],
            "transform": {"zoom": 0.75, "tx": 34, "ty": 18}}


class FakeClient:
    def __init__(self, layout=None, mcps=None, running=False):
        self.layout = copy.deepcopy(layout if layout is not None else empty_layout())
        self.mcps = mcps if mcps is not None else registry()
        self.running = running
        self.calls = []

    def get(self, path):
        self.calls.append(("GET", path))
        if path == "/api/config/project-running":
            return {"running": self.running}
        if path == "/api/mcp":
            return {"code": 200, "data": copy.deepcopy(self.mcps)}
        if path == "/api/canvas/layout":
            return {"code": 200, "data": copy.deepcopy(self.layout)}
        raise AssertionError(path)

    def post(self, path, body):
        self.calls.append(("POST", path, copy.deepcopy(body)))
        if path == "/api/canvas/layout":
            self.layout = copy.deepcopy(body)
            self.layout.pop("session_id")
        return {"code": 200}


class CanvasPlanTests(unittest.TestCase):
    def test_four_card_route_uses_live_namespace_and_does_not_mutate_input(self):
        original = empty_layout()
        plan = installer.make_plan(original, registry("visitor_bot"))
        self.assertEqual(original, empty_layout())
        self.assertEqual(plan["added_cards"], ["camera", "auto_guard", "tts", "speaker"])
        self.assertEqual([c["format"] for c in plan["layout"]["connections"]],
                         ["image/jpeg", "data/json", "audio/pcm-16k"])
        self.assertEqual([c["fromTopic"] for c in plan["layout"]["connections"]],
                         ["/visitor_bot/camera/color", "/visitor_bot/auto_guard/text",
                          "/visitor_bot/auto_guard/text/tts"])
        self.assertEqual(plan["card_ids"]["tts"], "bumi-guard-tts")
        self.assertEqual(plan["layout"]["transform"], original["transform"])

    def test_repeat_install_is_idempotent(self):
        first = installer.make_plan(empty_layout(), registry())
        second = installer.make_plan(first["layout"], registry())
        self.assertEqual(second["added_cards"], [])
        self.assertEqual(second["added_edges"], [])
        self.assertEqual(second["layout"], first["layout"])

    def test_reuses_existing_camera_and_speaker_and_preserves_other_canvas_data(self):
        layout = empty_layout()
        layout["cards"] = [
            {"id": "old-camera", "mcpId": "mcp-bumi-local", "toolName": "camera",
             "x": 11, "y": 22, "topicOut": [{"topic": "/bumi_01/camera/color",
                                              "format": "image/jpeg"}]},
            {"id": "old-speaker", "mcpId": "mcp-bumi-local", "toolName": "speaker",
             "x": 44, "y": 55},
            {"id": "other-card", "mcpId": "unrelated", "toolName": "other", "x": 77, "y": 88},
        ]
        layout["connections"] = [{"id": "other-edge", "fromCardId": "old-camera",
                                  "toCardId": "other-card", "format": "image/jpeg"}]
        layout["execConnections"] = [{"id": "other-exec", "fromCardId": "other-card",
                                      "toCardId": "old-speaker"}]
        plan = installer.make_plan(layout, registry())
        self.assertEqual(plan["card_ids"]["camera"], "old-camera")
        self.assertEqual(plan["card_ids"]["speaker"], "old-speaker")
        self.assertEqual(plan["added_cards"], ["auto_guard", "tts"])
        self.assertEqual(plan["layout"]["cards"][:3], layout["cards"])
        self.assertEqual(plan["layout"]["connections"][0], layout["connections"][0])
        self.assertEqual(plan["layout"]["execConnections"], layout["execConnections"])

    def test_refuses_to_rewire_an_existing_speaker(self):
        layout = empty_layout()
        layout["cards"] = [{"id": "speaker-old", "mcpId": "mcp-bumi-local",
                            "toolName": "speaker"}]
        layout["connections"] = [{"id": "old-route", "fromCardId": "old-tts",
                                  "toCardId": "speaker-old", "fromPortIdx": "0",
                                  "toPortIdx": "0", "format": "audio/pcm-16k",
                                  "fromTopic": "/old/tts"}]
        with self.assertRaisesRegex(installer.InstallError, "different input route"):
            installer.make_plan(layout, registry())

    def test_refuses_stale_existing_topic_and_mismatched_port_format(self):
        layout = empty_layout()
        layout["cards"] = [{"id": "old-camera", "mcpId": "mcp-bumi-local",
                            "toolName": "camera", "topicOut": [
                                {"topic": "/old/camera/color", "format": "image/jpeg"}]}]
        with self.assertRaisesRegex(installer.InstallError, "stale topic"):
            installer.make_plan(layout, registry())
        mcps = registry()
        mcps[0]["tools"][1]["topic_in"][0]["format"] = "data/json"
        with self.assertRaisesRegex(installer.InstallError, "expected image/jpeg"):
            installer.make_plan(empty_layout(), mcps)

    def test_refuses_existing_guard_route_to_another_tts(self):
        layout = empty_layout()
        layout["cards"] = [
            {"id": "guard-old", "mcpId": "mcp-bumi-local", "toolName": "auto_guard"},
            {"id": "tts-old", "mcpId": "mcp-perception-local", "toolName": "tts"},
        ]
        layout["connections"] = [{"id": "old-tts-route", "fromCardId": "guard-old",
                                  "toCardId": "tts-old"}]
        with self.assertRaisesRegex(installer.InstallError, "another TTS"):
            installer.make_plan(layout, registry())


class InstallWorkflowTests(unittest.TestCase):
    def test_dry_run_never_claims_lock_or_writes_layout(self):
        client = FakeClient()
        installer.install(client, apply=False)
        self.assertFalse(any(call[0] == "POST" for call in client.calls))

    def test_apply_claims_lock_saves_additive_layout_and_releases_lock(self):
        client = FakeClient()
        plan = installer.install(client, apply=True)
        posts = [call for call in client.calls if call[0] == "POST"]
        self.assertEqual([call[1] for call in posts],
                         ["/api/canvas/claim-edit", "/api/canvas/layout",
                          "/api/canvas/release-edit"])
        self.assertEqual(posts[0][2]["session_id"], posts[1][2]["session_id"])
        self.assertEqual(posts[1][2]["session_id"], posts[2][2]["session_id"])
        self.assertEqual(client.layout, plan["layout"])

    def test_apply_releases_lock_when_layout_is_conflicting(self):
        layout = empty_layout()
        layout["cards"] = [{"id": "speaker-old", "mcpId": "mcp-bumi-local",
                            "toolName": "speaker"}]
        layout["connections"] = [{"id": "old-route", "fromCardId": "old-tts",
                                  "toCardId": "speaker-old"}]
        client = FakeClient(layout)
        with self.assertRaises(installer.InstallError):
            installer.install(client, apply=True)
        posts = [call[1] for call in client.calls if call[0] == "POST"]
        self.assertEqual(posts, ["/api/canvas/claim-edit", "/api/canvas/release-edit"])

    def test_running_project_is_refused_before_claim(self):
        client = FakeClient(running=True)
        with self.assertRaisesRegex(installer.InstallError, "Stop the Agent Core project"):
            installer.install(client, apply=True)
        self.assertEqual(client.calls, [("GET", "/api/config/project-running")])

    def test_bearer_token_is_sent_without_exposing_it_in_output(self):
        client = installer.AgentCoreClient("http://127.0.0.1:15678", "secret-token")
        response = mock.MagicMock()
        response.__enter__.return_value = io.BytesIO(json.dumps({"code": 200}).encode())
        with mock.patch.object(installer.urllib.request, "urlopen", return_value=response) as urlopen:
            self.assertEqual(client.get("/api/mcp"), {"code": 200})
        request = urlopen.call_args.args[0]
        self.assertEqual(request.get_header("Authorization"), "Bearer secret-token")

    def test_https_pins_local_agent_core_certificate(self):
        client = installer.AgentCoreClient(
            "https://127.0.0.1:15678", ca_file="/opt/phanthy-motus/data/certs/cert.pem",
        )
        response = mock.MagicMock()
        response.__enter__.return_value = io.BytesIO(json.dumps({"code": 200}).encode())
        with mock.patch.object(installer.ssl, "create_default_context") as create_context, \
                mock.patch.object(installer.urllib.request, "urlopen", return_value=response) as urlopen:
            client.get("/api/mcp")
        create_context.assert_called_once_with(
            cafile="/opt/phanthy-motus/data/certs/cert.pem",
        )
        self.assertIs(urlopen.call_args.kwargs["context"], create_context.return_value)
        self.assertFalse(create_context.return_value.check_hostname)


if __name__ == "__main__":
    unittest.main()
