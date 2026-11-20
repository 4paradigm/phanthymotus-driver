"""Offline regression tests. All Webhook and ACP calls are mocked."""
import base64
from contextlib import ExitStack
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "fanshi_group_send_under_test",
    ROOT / "x-humanoid/tianyi2.0/fanshi_group_send.py")
card = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(card)
GOOD = (200, b'{"errcode":0,"errmsg":"ok","data":{"fail":{}}}')
BAD_GROUP = (200, b'{"errcode":0,"data":{"fail":{"123456":40000}}}')
PNG = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jQioAAAAASUVORK5CYII=")


class SplitSendTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.photo = Path(self.tmp.name) / "photo.png"
        self.photo.write_bytes(PNG)
        self.plugin = card.FanshiGroupSendPlugin({}, "test")
        self.plugin._image_roots = (Path(self.tmp.name),)
        self.config = {"webhook_url": "https://example.invalid/hook?token=TEST_ONLY",
                       "group_id": "123456"}
        self._patches = ExitStack()
        self.addCleanup(self._patches.close)
        self.post = self._patches.enter_context(patch.object(card, "_post_json", return_value=GOOD))
        self.complete = self._patches.enter_context(patch.object(card, "_complete"))
        self.printed = self._patches.enter_context(patch.object(card, "print", create=True))
        self.plugin.dispatch("config", self.config)
        self.plugin.dispatch("start", {})

    def send(self, **args):
        return self.plugin.dispatch("send", {"text": "test notice", **args})

    def finish(self, queued):
        self.assertEqual(queued["status"], "queued", queued)
        self.plugin._worker_thread.join(3)
        self.assertFalse(self.plugin._worker_thread.is_alive())
        return self.plugin.dispatch("result", {"action_id": queued["action_id"]})

    def pair(self, **args):
        return self.send(image_path=str(self.photo), **args)

    def types(self):
        return [[item["type"] for item in call.args[1]["message"]["body"]]
                for call in self.post.call_args_list]

    def test_text_only_keeps_one_request(self):
        result = self.finish(self.send())
        self.assertEqual(self.types(), [["TEXT"]])
        self.assertEqual(result["status"], "accepted")
        self.assertEqual(result["request_count"], 1)

    def test_image_only_keeps_one_request_and_original_bytes(self):
        result = self.finish(self.pair(text=""))
        self.assertEqual(result["status"], "accepted")
        self.assertEqual(self.types(), [["IMAGE"]])
        image = self.post.call_args.args[1]["message"]["body"][0]["content"]
        self.assertEqual(base64.b64decode(image), PNG)

    def test_pair_uses_two_requests_one_action_and_one_callback(self):
        queued = self.pair(group_id=999, webhook_url="https://other.invalid")
        result = self.finish(queued)
        self.assertEqual(self.types(), [["TEXT"], ["IMAGE"]])
        self.assertEqual(result["status"], "accepted")
        self.assertEqual(result["accepted_parts"], 2)
        self.assertEqual(result["total_parts"], 2)
        self.assertEqual(result["action_id"], queued["action_id"])
        for call in self.post.call_args_list:
            self.assertEqual(call.args[0], self.config["webhook_url"])
            self.assertEqual(call.args[1]["message"]["header"]["toid"], [123456])
        self.complete.assert_called_once_with(queued["action_id"], result)

    def test_pair_mentions_member_only_once_with_text(self):
        self.finish(self.pair(at_userid="manager"))
        self.assertEqual(self.types(), [["TEXT", "AT"], ["IMAGE"]])
        at = self.post.call_args_list[0].args[1]["message"]["body"][-1]
        self.assertEqual(at, {"type": "AT", "atuserids": ["manager"], "atall": False})

    def test_image_only_can_keep_its_mention(self):
        self.finish(self.pair(text="", at_userid="manager"))
        self.assertEqual(self.types(), [["IMAGE", "AT"]])

    def test_invalid_image_prevents_even_text_submission(self):
        self.photo.write_bytes(b"not an image")
        self.assertEqual(self.pair()["status"], "failed")
        self.post.assert_not_called()

    def test_oversized_image_is_rejected_before_text(self):
        self.photo.write_bytes(b"\x89PNG\r\n\x1a\n" + b"0" * 750000)
        self.assertEqual(self.pair()["status"], "failed")
        self.post.assert_not_called()

    def test_first_failure_stops_image_and_retains_group_code(self):
        self.post.return_value = BAD_GROUP
        result = self.finish(self.pair())
        self.assertEqual(result["status"], "failed")
        self.assertEqual(self.post.call_count, 1)
        self.assertEqual(result["parts"][0]["failed_groups"], {"123456": 40000})
        self.assertEqual(result["parts"][1]["status"], "not_sent")
        self.assertFalse(result["parts"][1]["attempted"])

    def test_first_unknown_stops_image_without_retry(self):
        self.post.side_effect = TimeoutError("TEST_ONLY")
        result = self.finish(self.pair())
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(self.post.call_count, 1)
        self.assertEqual(result["parts"][1]["status"], "not_sent")
        self.assertNotIn("TEST_ONLY", json.dumps(result))

    def test_second_failure_reports_partial_without_retry(self):
        self.post.side_effect = [GOOD, BAD_GROUP]
        result = self.finish(self.pair())
        self.assertEqual(result["status"], "partial")
        self.assertFalse(result["ok"])
        self.assertEqual(result["accepted_parts"], 1)
        self.assertEqual([p["status"] for p in result["parts"]], ["accepted", "failed"])
        self.assertEqual(result["parts"][1]["failed_groups"], {"123456": 40000})
        self.assertEqual(self.post.call_count, 2)

    def test_second_unknown_preserves_text_success_without_retry(self):
        self.post.side_effect = [GOOD, TimeoutError("TEST_ONLY")]
        result = self.finish(self.pair())
        self.assertEqual(result["status"], "unknown")
        self.assertEqual(result["parts"][0]["status"], "accepted")
        self.assertEqual(result["parts"][1]["status"], "unknown")
        self.assertEqual(result["accepted_parts"], 1)
        self.assertEqual(self.post.call_count, 2)
        self.assertNotIn("TEST_ONLY", json.dumps(result))

    def test_stop_between_parts_prevents_image(self):
        def stop_after_text(*args):
            self.plugin.stop()
            return GOOD
        self.post.side_effect = stop_after_text
        result = self.finish(self.pair())
        self.assertEqual(result["status"], "partial")
        self.assertEqual(result["parts"][1]["status"], "cancelled")
        self.assertFalse(result["parts"][1]["attempted"])
        self.assertEqual(self.post.call_count, 1)

    def test_config_change_between_parts_prevents_new_target_submission(self):
        def change_target(*args):
            self.plugin.dispatch("config", {**self.config, "group_id": "789"})
            self.plugin.dispatch("start", {})
            return GOOD
        self.post.side_effect = change_target
        result = self.finish(self.pair())
        self.assertEqual(result["status"], "partial")
        self.assertEqual(self.post.call_count, 1)
        self.assertFalse(result["parts"][1]["attempted"])

    def test_stop_before_worker_keeps_prior_private_test_contract(self):
        with patch.object(card.threading.Thread, "start"):
            queued = self.send()
        generation = self.plugin._generation
        self.plugin.stop()
        self.plugin._worker(queued["action_id"], self.plugin._settings, generation, {})
        self.post.assert_not_called()
        result = self.plugin.dispatch("result", {"action_id": queued["action_id"]})
        self.assertEqual(result["status"], "cancelled")

    def test_busy_is_held_until_both_parts_finish(self):
        entered, release = threading.Event(), threading.Event()
        def wait_for_image(url, payload):
            if payload["message"]["body"][0]["type"] == "IMAGE":
                entered.set()
                if not release.wait(3):
                    raise TimeoutError()
            return GOOD
        self.post.side_effect = wait_for_image
        queued = self.pair()
        try:
            self.assertTrue(entered.wait(2))
            progress = self.plugin.dispatch("result", {"action_id": queued["action_id"]})
            self.assertEqual(progress["status"], "sending")
            self.assertEqual(progress["parts"][0]["status"], "accepted")
            self.assertEqual(self.send()["status"], "failed")
        finally:
            release.set()
        self.assertEqual(self.finish(queued)["status"], "accepted")
        self.assertEqual(self.post.call_count, 2)

    def test_pair_reserves_two_rate_slots(self):
        self.plugin._recent.extend([time.monotonic()] * 28)
        self.assertEqual(self.finish(self.pair())["status"], "accepted")
        self.assertEqual(len(self.plugin._recent), 30)
        self.assertEqual(self.send()["status"], "failed")
        self.assertEqual(self.post.call_count, 2)

    def test_pair_rejected_at_29_without_partial_send(self):
        self.plugin._recent.extend([time.monotonic()] * 29)
        self.assertEqual(self.pair()["status"], "failed")
        self.post.assert_not_called()
        self.assertEqual(self.finish(self.send())["status"], "accepted")

    def test_expired_rate_slots_are_released(self):
        self.plugin._recent.extend([time.monotonic() - 61] * 30)
        self.assertEqual(self.finish(self.pair())["status"], "accepted")
        self.assertEqual(len(self.plugin._recent), 2)

    def test_callback_failure_preserves_queryable_result(self):
        self.complete.side_effect = ValueError("TEST_ONLY")
        result = self.finish(self.pair())
        self.assertEqual(result["status"], "accepted")
        self.assertEqual(self.post.call_count, 2)
        self.assertNotIn("TEST_ONLY", str(self.printed.call_args_list))

    def test_results_are_deep_copied_and_contain_no_request_payload(self):
        queued = self.pair()
        result = self.finish(queued)
        result["parts"][0]["status"] = "changed by caller"
        stored = self.plugin.dispatch("result", {"action_id": queued["action_id"]})
        self.assertEqual(stored["parts"][0]["status"], "accepted")
        encoded = json.dumps(stored)
        self.assertNotIn("TEST_ONLY", encoded)
        self.assertNotIn(base64.b64encode(PNG).decode(), encoded)
        self.assertNotIn("test notice", encoded)

    def test_interpret_zero_does_not_hide_target_failure(self):
        result = card._interpret(*BAD_GROUP)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["platform_code"], 0)
        self.assertEqual(result["failed_groups"], {"123456": 40000})
        self.assertTrue(result["has_group_failures"])

    def test_interpret_does_not_echo_arbitrary_failure_text(self):
        result = card._interpret(200, json.dumps({"errcode": 0, "data": {"fail": {
            "123456": "https://example.invalid/?token=TEST_ONLY",
            "https://example.invalid/?token=TEST_ONLY": 1}}}).encode())
        self.assertNotIn("TEST_ONLY", json.dumps(result))
        self.assertEqual(result["failed_groups"], {"123456": "detail_unavailable"})

    def test_response_variants_and_unknown_cases(self):
        for raw, expected in [
            (b'{"errorcode":0}', "accepted"),
            (b'{"errcode":0,"errorcode":-1}', "unknown"),
            (b'{"errcode":false}', "unknown"),
            (b'{"errcode":-1}', "failed"),
            (b'<html/>', "unknown"),
        ]:
            with self.subTest(raw=raw):
                self.assertEqual(card._interpret(200, raw)["status"], expected)

    def test_schema_keeps_tool_name_and_calling_parameters(self):
        tool = self.plugin.get_tool()
        self.assertEqual(tool["name"], "fanshi_group_send")
        self.assertEqual(tool["inputSchema"]["properties"]["action"]["enum"], ["send", "result"])
        self.assertEqual(tool["inputSchema"]["properties"]["image_path"]["uploadTo"], "mcp")
        self.assertEqual(tool["inputSchema"]["x-completion"]["actions"], ["send"])
        self.assertNotIn("webhook_url", tool["inputSchema"]["properties"])


if __name__ == "__main__":
    unittest.main()
