import json
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock


class _FakeNodeBase:
    def __init__(self, name):
        self.name = name

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscription = (message_type, topic, callback, qos)
        return self.subscription

    def destroy_node(self):
        pass


class _QoSProfile:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


rclpy = types.ModuleType("rclpy")
rclpy_node = types.ModuleType("rclpy.node")
rclpy_qos = types.ModuleType("rclpy.qos")
rclpy_node.Node = _FakeNodeBase
rclpy_qos.QoSProfile = _QoSProfile
rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE="volatile")
rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT="best_effort")
rclpy.node = rclpy_node
rclpy.qos = rclpy_qos
sys.modules.setdefault("rclpy", rclpy)
sys.modules.setdefault("rclpy.node", rclpy_node)
sys.modules.setdefault("rclpy.qos", rclpy_qos)

std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")
std_msgs_msg.String = type("String", (), {})
std_msgs.msg = std_msgs_msg
sys.modules.setdefault("std_msgs", std_msgs)
sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

_BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_BASE))

import meeting_facilitator as mf


class _FakeExecutor:
    def __init__(self):
        self.added = []
        self.removed = []

    def add_node(self, node):
        self.added.append(node)

    def remove_node(self, node):
        self.removed.append(node)


def _dependencies():
    dependencies = {
        "tts": MagicMock(),
        "led": MagicMock(),
        "arm": MagicMock(),
    }
    dependencies["tts"].dispatch.return_value = {"ret": 0}
    dependencies["led"].dispatch.return_value = {"state": "idle"}
    dependencies["arm"].dispatch.return_value = {"ret": 0}
    dependencies["asr"] = MagicMock()
    dependencies["asr"].get_tool.return_value = {
        "topic_out": [{"topic": "/robot/asr/text", "format": "data/json"}]
    }
    return dependencies


class MeetingControllerTests(unittest.TestCase):
    def test_start_pause_resume_and_next_speaker(self):
        controller = mf.MeetingController()
        self.assertEqual(controller.start(["甲", "乙"], 30, 10), "甲")
        controller.pause(20)
        self.assertEqual(controller.status(25)["remaining_s"], 20)
        controller.resume(40)
        self.assertEqual(controller.status(45)["remaining_s"], 15)
        self.assertEqual(controller.next_speaker(50), "乙")
        self.assertEqual(controller.status(50)["remaining_s"], 30)
        self.assertIsNone(controller.next_speaker(80))

    def test_transcript_and_action_items(self):
        controller = mf.MeetingController(transcript_limit=2, action_item_limit=1)
        controller.start(["甲"], 30, 0)
        controller.add_transcript("第一句", 1)
        controller.add_transcript("第二句", 2)
        controller.add_transcript("第三句", 3)
        controller.add_action_item("完成方案", "甲")
        self.assertEqual(
            [item["text"] for item in controller.transcript],
            ["第二句", "第三句"],
        )
        self.assertEqual(
            controller.status(5)["action_items"],
            [{"description": "完成方案", "owner": "甲"}],
        )
        with self.assertRaisesRegex(ValueError, "action item limit reached"):
            controller.add_action_item("第二项")

    def test_rejects_invalid_start(self):
        controller = mf.MeetingController()
        with self.assertRaises(ValueError):
            controller.start([], 30, 0)
        with self.assertRaises(ValueError):
            controller.start([None], 30, 0)
        with self.assertRaises(ValueError):
            controller.start(["甲"], 0, 0)
        with self.assertRaises(ValueError):
            controller.start(["甲"], float("nan"), 0)


class MeetingAsrNodeTests(unittest.TestCase):
    def test_parses_valid_text(self):
        captured = []
        node = mf.MeetingAsrNode("/robot/asr/text", captured.append)
        message = types.SimpleNamespace(data=json.dumps({"text": "形成结论"}))
        node._handle_message(message)
        self.assertEqual(captured, ["形成结论"])

    def test_ignores_invalid_messages(self):
        captured = []
        node = mf.MeetingAsrNode("/robot/asr/text", captured.append)
        for data in ("not-json", "{}", '{"text": 1}', '{"text": "  "}'):
            node._handle_message(types.SimpleNamespace(data=data))
        self.assertEqual(captured, [])


class PluginTests(unittest.TestCase):
    def setUp(self):
        self.executor = _FakeExecutor()
        self.dependencies = _dependencies()
        self.plugin = mf.make_plugin(
            {
                "auto_start": True,
                "speaker_duration_s": 30,
                "warning_before_s": 5,
                "led_min_hold_s": 0,
            },
            "robot",
            self.executor,
            self.dependencies,
        )
        self.plugin.start()

    def tearDown(self):
        self.plugin.teardown()

    def _wait_for(self, predicate, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail("condition was not reached")

    def test_full_meeting_flow(self):
        result = self.plugin.dispatch(
            "start_meeting",
            {"participants": ["甲", "乙"], "duration_s": 30},
        )
        self.assertTrue(result["active"])
        self.assertEqual(result["current_speaker"], "甲")
        self._wait_for(lambda: self.dependencies["tts"].dispatch.call_count >= 1)
        self.dependencies["tts"].dispatch.assert_any_call(
            "speak", {"text": "会议开始，请甲发言", "voice": 0}
        )

        self.plugin._node._handle_message(
            types.SimpleNamespace(data=json.dumps({"text": "本周完成联调"}))
        )
        result = self.plugin.dispatch(
            "add_action_item",
            {"description": "完成联调", "owner": "甲"},
        )
        self.assertEqual(result["transcript_count"], 1)

        result = self.plugin.dispatch("next_speaker", {})
        self.assertEqual(result["current_speaker"], "乙")
        result = self.plugin.dispatch("end_meeting", {})
        self.assertFalse(result["active"])
        self.assertEqual(result["transcript"][0]["text"], "本周完成联调")
        self.assertEqual(
            result["action_items"],
            [{"description": "完成联调", "owner": "甲"}],
        )

    def test_start_accepts_json_string_participants(self):
        result = self.plugin.dispatch(
            "start_meeting",
            {"participants": '["张三", "李四"]', "duration_s": 30},
        )
        self.assertEqual(result["participants"], ["张三", "李四"])
        self.assertEqual(result["current_speaker"], "张三")

    def test_start_accepts_comma_separated_participants(self):
        for participants in ("张三，李四", "张三, 李四"):
            result = self.plugin.dispatch(
                "start_meeting",
                {"participants": participants, "duration_s": 30},
            )
            self.assertEqual(result["participants"], ["张三", "李四"])
            self.plugin.dispatch("end_meeting", {})

    def test_start_rejects_invalid_participant_text(self):
        for participants in ("", '"张三"', 1, '["张三",]', '["张三", "李四"'):
            result = self.plugin.dispatch(
                "start_meeting",
                {"participants": participants, "duration_s": 30},
            )
            self.assertIn("error", result)

    def test_schema_accepts_array_and_string_participants(self):
        participants_schema = self.plugin.get_tool()["inputSchema"]["properties"][
            "participants"
        ]
        self.assertEqual(
            [variant["type"] for variant in participants_schema["anyOf"]],
            ["array", "string"],
        )

    def test_pause_and_resume_preserve_remaining_time(self):
        self.plugin.dispatch(
            "start_meeting", {"participants": ["甲"], "duration_s": 30}
        )
        paused = self.plugin.dispatch("pause", {})
        self.assertTrue(paused["paused"])
        resumed = self.plugin.dispatch("resume", {})
        self.assertFalse(resumed["paused"])
        self.assertGreater(resumed["remaining_s"], 29)

    def test_stop_resets_state_and_led(self):
        self.plugin.dispatch(
            "start_meeting", {"participants": ["甲"], "duration_s": 30}
        )
        result = self.plugin.dispatch("stop", {})
        self.assertEqual(result, {"state": "idle"})
        self.assertFalse(self.plugin.dispatch("status", {})["active"])
        self.dependencies["led"].dispatch.assert_any_call(
            "state", {"state": "idle"}
        )

    def test_info_declares_asr_topic(self):
        self.assertEqual(
            self.plugin.dispatch("info", {}),
            {"topic_in": [{"topic": "/robot/asr/text", "format": "data/json"}]},
        )

    def test_timer_emits_warning_and_timeout_once(self):
        self.plugin.dispatch(
            "start_meeting", {"participants": ["甲"], "duration_s": 0.15}
        )
        self._wait_for(
            lambda: any(
                call.args[1].get("text", "").startswith("还剩")
                for call in self.dependencies["tts"].dispatch.call_args_list
            )
        )
        self._wait_for(
            lambda: any(
                call.args[1].get("text") == "甲发言时间已到"
                for call in self.dependencies["tts"].dispatch.call_args_list
            )
        )
        timeout_calls = [
            call
            for call in self.dependencies["tts"].dispatch.call_args_list
            if call.args[1].get("text") == "甲发言时间已到"
        ]
        self.assertEqual(len(timeout_calls), 1)

    def test_announcements_are_ordered(self):
        spoken = []
        self.dependencies["tts"].dispatch.side_effect = (
            lambda action, args: spoken.append(args["text"]) or {"ret": 0}
        )
        self.plugin.dispatch(
            "start_meeting", {"participants": ["甲", "乙"], "duration_s": 30}
        )
        self.plugin.dispatch("next_speaker", {})
        self.plugin.dispatch("pause", {})
        self._wait_for(lambda: len(spoken) == 3)
        self.assertEqual(
            spoken,
            ["会议开始，请甲发言", "下面请乙发言", "会议计时已暂停"],
        )

    def test_stop_discards_queued_announcements(self):
        first_started = threading.Event()
        release_first = threading.Event()

        def speak(action, args):
            if args["text"] == "会议开始，请甲发言":
                first_started.set()
                release_first.wait(1)
            return {"ret": 0}

        self.dependencies["tts"].dispatch.side_effect = speak
        self.plugin.dispatch(
            "start_meeting", {"participants": ["甲", "乙"], "duration_s": 30}
        )
        self.assertTrue(first_started.wait(1))
        self.plugin.dispatch("next_speaker", {})
        self.plugin.dispatch("stop", {})
        release_first.set()
        time.sleep(0.05)
        spoken = [
            call.args[1]["text"]
            for call in self.dependencies["tts"].dispatch.call_args_list
        ]
        self.assertNotIn("下面请乙发言", spoken)

    def test_teardown_stops_workers_and_removes_node(self):
        self.plugin.teardown()
        self.assertFalse(self.plugin._timer_thread.is_alive())
        self.assertFalse(self.plugin._announcement_thread.is_alive())
        self.assertEqual(self.executor.removed, [self.plugin._node])
        self.assertEqual(
            self.plugin.dispatch("start", {}),
            {"error": "meeting facilitator has been torn down"},
        )


if __name__ == "__main__":
    unittest.main()
