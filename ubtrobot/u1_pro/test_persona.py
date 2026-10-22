"""Tests for the U1 Pro persona card (routing, serial playback, safety)."""

from __future__ import annotations

import time
import unittest
from unittest import mock

import test_device  # noqa: F401  installs ROS/vendor stubs before importing device
from test_device import FakeNodes

import device
from persona import PersonaRouter


def _finish(nodes: FakeNodes, success: bool = True) -> None:
    """Emit the vendor playback result for the most recent queued request."""
    uuid = f"adapter-{len(nodes.string_calls)}"
    nodes.playback_listener({"phase": "result", "uuid": uuid,
                             "state_name": "COMPLETED" if success else "FAILED"})


def _wait(predicate, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class PersonaRouterTests(unittest.TestCase):
    def setUp(self):
        self.router = PersonaRouter()

    def route(self, text, current=None):
        return self.router.analyze(text, current)

    def test_acceptance_samples_route_to_expected_persona(self):
        cases = {
            "我最近很失落。": "qingjie",
            "我心情不好，想找人聊聊。": "anning",
            "帮帮我，明天汇报怎么办？": "lanzong",
            "我想听一首李清照的宋词。": "qingzhao",
        }
        for text, pid in cases.items():
            with self.subTest(text=text):
                self.assertEqual(self.route(text)["persona"], pid)

    def test_safety_beats_any_persona_keyword(self):
        result = self.route("我好失败，不想活了")
        self.assertEqual(result["intent"], "safety")
        self.assertIsNone(result["persona"])

    def test_negated_keyword_does_not_trigger(self):
        self.assertEqual(self.route("我不焦虑，就是随便问问")["intent"], "none")
        self.assertEqual(self.route("我没失落")["intent"], "none")

    def test_explicit_request_overrides_keywords(self):
        result = self.route("我很焦虑，切换到清照")
        self.assertEqual(result["persona"], "qingzhao")
        self.assertTrue(result["explicit"])
        self.assertEqual(self.route("岚总模式")["persona"], "lanzong")

    def test_more_hits_win_and_current_persona_wins_ties(self):
        self.assertEqual(self.route("我很焦虑压力大睡不着，汇报怎么办")["persona"], "anning")
        self.assertEqual(self.route("我很焦虑，怎么办", current="anning")["persona"], "anning")
        self.assertEqual(self.route("我很焦虑，怎么办")["persona"], "lanzong")

    def test_stop_phrases(self):
        self.assertEqual(self.route("停一下")["intent"], "stop")
        self.assertEqual(self.route("别说了")["intent"], "stop")
        self.assertNotEqual(self.route("我停不下来，脑子一直在转，很焦虑")["intent"], "stop")

    def test_personas_are_configurable(self):
        router = PersonaRouter({"personas": [
            {"id": "guide", "name": "小导", "announce": "现在由小导为你讲解。", "keywords": ["展品"]}]})
        self.assertEqual(router.analyze("这个展品是什么", None)["persona"], "guide")
        with self.assertRaises(ValueError):
            PersonaRouter({"personas": [{"id": "x"}]})


class PersonaCardTests(unittest.TestCase):
    def setUp(self):
        self.nodes = FakeNodes()
        self.notify = mock.patch.object(device, "_acp_notify").start()
        self.addCleanup(mock.patch.stopall)
        self.audio = device.AudioPlugin(self.nodes)
        self.audio.start()
        self.expression = device.ExpressionPlugin(self.audio)
        self.card = device.build_persona_plugin(self.audio, self.expression, self.nodes,
                                                {"poll_interval": 0.005, "busy_timeout": 1.0,
                                                 "playback_timeout": 1.0})
        self.card.start()
        self.addCleanup(self.card.stop)

    def spoken(self):
        return [p.get("text") for name, p in self.nodes.string_calls if name == "play_text"]

    def test_tool_is_declared_and_registered(self):
        tool = self.card.get_tool()
        self.assertEqual(tool["name"], "persona")
        params = tool["inputSchema"]["x-action-params"]
        self.assertEqual(params["route"]["params"], ["text"])
        self.assertIn("smile", tool["inputSchema"]["properties"]["expression"]["enum"])

    def test_announcement_is_spoken_before_reply_and_serially(self):
        result = self.card.dispatch("route", {"text": "帮帮我，明天汇报怎么办？"})
        self.assertTrue(result["switched"])
        self.assertEqual(result["persona"]["id"], "lanzong")
        self.card.dispatch("say", {"text": "告诉我汇报对象和时长。", "action_id": "reply-1"})
        self.assertTrue(_wait(lambda: len(self.spoken()) == 1))
        time.sleep(0.05)
        self.assertEqual(self.spoken(), ["现在由岚总在线处理。"])  # reply waits for announcement
        _finish(self.nodes)
        self.assertTrue(_wait(lambda: len(self.spoken()) == 2))
        self.assertEqual(self.spoken()[1], "告诉我汇报对象和时长。")
        _finish(self.nodes)
        self.assertTrue(_wait(lambda: any(c.args[:2] == ("reply-1", "completed")
                                          for c in self.notify.call_args_list)))

    def test_same_persona_is_not_reannounced_and_duplicates_are_ignored(self):
        self.card.dispatch("route", {"text": "我好沮丧"})
        self.assertTrue(_wait(lambda: len(self.spoken()) == 1))
        _finish(self.nodes)
        again = self.card.dispatch("route", {"text": "我好沮丧"})
        self.assertTrue(again.get("deduplicated"))
        other = self.card.dispatch("route", {"text": "还是很失落"})
        self.assertFalse(other["switched"])
        time.sleep(0.05)
        self.assertEqual(len(self.spoken()), 1)

    def test_safety_interrupts_and_skips_persona_announcement(self):
        self.card.dispatch("route", {"text": "我好沮丧"})
        self.card.dispatch("say", {"text": "我在这里。", "action_id": "reply-x"})
        self.assertTrue(_wait(lambda: len(self.spoken()) == 1))
        result = self.card.dispatch("route", {"text": "我好失败，不想活了"})
        self.assertEqual(result["intent"], "safety")
        self.assertGreaterEqual(self.nodes.interrupts, 1)
        self.assertTrue(_wait(lambda: len(self.spoken()) == 2))
        self.assertIn("安全", self.spoken()[1])
        self.assertTrue(any(c.args[:2] == ("reply-x", "cancelled") for c in self.notify.call_args_list))
        _finish(self.nodes)
        # Keywords no longer pull the visitor back into a persona.
        follow = self.card.dispatch("route", {"text": "我就是觉得自己很失败"})
        self.assertEqual(follow["intent"], "none")
        self.assertEqual(self.card.mode, "safety")

    def test_stop_clears_queue_and_acknowledges_once(self):
        self.card.dispatch("route", {"text": "我想听宋词"})
        self.card.dispatch("say", {"text": "昨夜雨疏风骤。"})
        self.assertTrue(_wait(lambda: len(self.spoken()) == 1))
        result = self.card.dispatch("route", {"text": "停一下"})
        self.assertEqual(result["intent"], "stop")
        self.assertTrue(_wait(lambda: len(self.spoken()) == 2))
        self.assertEqual(self.spoken()[1], "好的，我先停下。")
        _finish(self.nodes)
        time.sleep(0.05)
        self.assertEqual(len(self.spoken()), 2)

    def test_expression_runs_before_reply_and_invalid_one_is_dropped(self):
        self.card.dispatch("set", {"persona": "qingzhao"})
        self.assertTrue(_wait(lambda: len(self.nodes.string_calls) == 1))
        _finish(self.nodes)
        result = self.card.dispatch("say", {"text": "知否知否。", "expression": "smile"})
        self.assertEqual(result["expression"], "smile")
        self.assertTrue(_wait(lambda: len(self.nodes.string_calls) == 2))
        self.assertEqual(self.nodes.string_calls[1], ("play_action", mock.ANY))
        _finish(self.nodes)
        self.assertTrue(_wait(lambda: len(self.nodes.string_calls) == 3))
        _finish(self.nodes)
        bad = self.card.dispatch("say", {"text": "再来一句。", "expression": "angry"})
        self.assertIsNone(bad["expression"])

    def test_busy_channel_times_out_without_claiming_success(self):
        self.audio._active = {"action_id": "external", "vendor_uuid": "x", "kind": "play_action",
                              "tool_name": "expression"}
        self.card.dispatch("say", {"text": "你好", "action_id": "blocked"})
        self.assertTrue(_wait(lambda: any(c.args[:2] == ("blocked", "error")
                                          for c in self.notify.call_args_list), timeout=3))
        self.assertEqual(self.spoken(), [])

    def test_asr_topic_messages_are_routed(self):
        subscribed = {}
        card = device.build_persona_plugin(self.audio, self.expression, mock.Mock(
            subscribe_core_text=lambda topic, cb: subscribed.setdefault(topic, cb),
            unsubscribe_core=lambda sub: None), {"poll_interval": 0.005})
        self.addCleanup(card.stop)
        self.assertEqual(card.start("/test/asr/text")["input_topic"], "/test/asr/text")
        subscribed["/test/asr/text"]('{"text": "我心情不好", "is_final": false}')
        self.assertIsNone(card.current)
        subscribed["/test/asr/text"]('{"text": "我心情不好", "confidence": 0.9}')
        self.assertEqual(card.current, "anning")


if __name__ == "__main__":
    unittest.main()
