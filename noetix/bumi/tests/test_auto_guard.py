"""Hardware-free contract checks for the automatic Bumi guard card."""

import json
import io
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from noetix.bumi.auto_guard import AutoGuardPlugin
from noetix.bumi.guard_policy import GuardPolicy


class FakeMessage:
    def __init__(self):
        self.data = ""


class FakePublisher:
    def __init__(self):
        self.subscribers = 0
        self.messages = []
        self.failures = 0
        self.attempts = 0

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.attempts += 1
        if self.failures:
            self.failures -= 1
            raise RuntimeError("temporary publisher failure")
        self.messages.append(json.loads(message.data))


class FakeCamera:
    def __init__(self):
        self.running = True
        self.sequence = 0
        self.time_offset_s = 0.0

    def is_running(self):
        return self.running

    def wait_for_color_frame(self, after_sequence=None, timeout_s=1.0):
        self.sequence += 1
        return {
            "frame_sequence": self.sequence,
            "received_monotonic": time.monotonic() + self.time_offset_s,
            "data": b"jpeg",
        }, self.sequence


class FakeWhitelist:
    def __init__(self):
        self.ready = True

    def is_ready(self):
        return self.ready

    def recognize_jpeg(self, jpeg):
        assert jpeg == b"jpeg"
        return {"result": "matched", "person_id": "person_1"}


class FakeSpeaker:
    def __init__(self):
        self.topic = None

    def is_subscribed_to(self, topic):
        return self.topic == topic


class AutoGuardTests(unittest.TestCase):
    def setUp(self):
        self.publisher = FakePublisher()
        self.camera = FakeCamera()
        self.whitelist = FakeWhitelist()
        self.speaker = FakeSpeaker()
        self.tts = {"ready": False}
        self.guard = AutoGuardPlugin(
            {"sample_interval_s": 0.1}, "ubuntu", None,
            self.camera, self.whitelist, self.speaker,
            publisher=self.publisher, message_type=FakeMessage,
            tts_probe=lambda: self.tts["ready"],
        )

    def tearDown(self):
        self.guard.stop()

    def test_schema_declares_processor_lifecycle_and_topics(self):
        tool = self.guard.get_tool()
        self.assertEqual(tool["type"], "processor")
        self.assertEqual(tool["inputSchema"]["properties"]["action"]["enum"],
                         ["start", "stop", "info"])
        self.assertEqual(tool["topic_in"], [{"format": "image/jpeg"}])
        self.assertEqual(tool["topic_out"], [{
            "topic": "/ubuntu/auto_guard/text", "format": "data/json",
        }])
        self.assertIsNone(self.guard.dispatch("unknown", {}))

    def test_image_config_and_marketplace_ship_the_guard(self):
        driver = Path(__file__).resolve().parents[1]
        dockerfile = (driver / "Dockerfile").read_text()
        self.assertIn("COPY guard_policy.py /work/guard_policy.py", dockerfile)
        self.assertIn("COPY auto_guard.py /work/auto_guard.py", dockerfile)
        self.assertIn("auto_guard:", (driver / "config.yaml").read_text())
        self.assertIn("tts_instance_id: bumi-guard-tts", (driver / "config.yaml").read_text())
        self.assertIn("{ name: auto_guard,        type: processor }",
                      (driver / "driver.yaml").read_text())

    def test_bundle_start_does_not_arm_guard(self):
        self.guard.start()
        self.assertEqual(self.guard.dispatch("info", {})["state"], "idle")
        self.assertEqual(self.camera.sequence, 0)

    def test_rejects_wrong_input_and_missing_enrollment(self):
        self.assertEqual(self.guard.dispatch("start", {"input_topic": "/other"})["state"], "error")
        self.whitelist.ready = False
        self.assertEqual(self.guard.dispatch("start", {
            "input_topic": "/ubuntu/camera/color",
        })["state"], "error")

    def test_slow_sampling_still_allows_three_frame_confirmation(self):
        slow = AutoGuardPlugin(
            {"sample_interval_s": 5.0}, "ubuntu", None,
            self.camera, self.whitelist, self.speaker,
            publisher=self.publisher, message_type=FakeMessage,
            tts_probe=lambda: True,
        )
        policy = GuardPolicy(**slow._policy_kwargs)
        self.assertIsNone(policy.observe(
            {"status": "unknown"}, frame_ts=1.0, now=1.0,
        ))
        self.assertIsNone(policy.observe(
            {"status": "unknown"}, frame_ts=6.0, now=6.0,
        ))
        self.assertIsNotNone(policy.observe(
            {"status": "unknown"}, frame_ts=11.0, now=11.0,
        ))

    def test_speaks_only_after_tts_and_speaker_are_connected(self):
        result = self.guard.dispatch("start", {"input_topic": "/ubuntu/camera/color"})
        self.assertEqual(result["state"], "running")
        time.sleep(0.2)
        self.assertEqual(self.publisher.messages, [])
        self.publisher.subscribers = 1
        time.sleep(0.2)
        self.assertEqual(self.publisher.messages, [])
        self.speaker.topic = "/ubuntu/auto_guard/text/tts"
        time.sleep(0.2)
        self.assertEqual(self.publisher.messages, [])
        self.tts["ready"] = True
        deadline = time.monotonic() + 3
        while not self.publisher.messages and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual(self.publisher.messages, [{"text": "欢迎来到范式智能。"}])
        time.sleep(0.4)
        self.assertEqual(len(self.publisher.messages), 1)
        self.assertEqual(self.guard.dispatch("info", {})["last_event"],
                         "known_visitor_arrived")

    def test_stop_prevents_later_greeting(self):
        self.guard.dispatch("start", {"input_topic": "/ubuntu/camera/color"})
        self.assertEqual(self.guard.dispatch("stop", {})["state"], "idle")
        self.publisher.subscribers = 1
        self.speaker.topic = "/ubuntu/auto_guard/text/tts"
        self.tts["ready"] = True
        time.sleep(0.25)
        self.assertEqual(self.publisher.messages, [])

    def test_tts_probe_checks_exact_instance_and_topic(self):
        info = {"state": "running", "topic_in": [
            {"topic": "/ubuntu/auto_guard/text", "format": "data/json"},
        ]}
        rpc = {"result": {"content": [{"text": json.dumps(info)}]}}

        def response_for(value):
            response = mock.MagicMock()
            response.__enter__.return_value = io.BytesIO(json.dumps(value).encode())
            return response

        with mock.patch("urllib.request.urlopen", return_value=response_for(rpc)) as urlopen:
            self.assertTrue(self.guard._perception_tts_running())
        request = urlopen.call_args.args[0]
        payload = json.loads(request.data)
        self.assertEqual(payload["params"]["arguments"]["instance_id"], "bumi-guard-tts")
        self.assertEqual(payload["params"]["arguments"]["input_topic"],
                         "/ubuntu/auto_guard/text")
        info["topic_in"][0]["topic"] = "/other/text"
        with mock.patch("urllib.request.urlopen", return_value=response_for({
            "result": {"content": [{"text": json.dumps(info)}]},
        })):
            self.assertFalse(self.guard._perception_tts_running())

    def test_reconnect_does_not_regreet_same_visible_visitor(self):
        self.publisher.subscribers = 1
        self.speaker.topic = "/ubuntu/auto_guard/text/tts"
        self.tts["ready"] = True
        self.guard.dispatch("start", {"input_topic": "/ubuntu/camera/color"})
        deadline = time.monotonic() + 3
        while not self.publisher.messages and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual(len(self.publisher.messages), 1)
        self.tts["ready"] = False
        time.sleep(0.2)
        self.tts["ready"] = True
        time.sleep(0.6)
        self.assertEqual(len(self.publisher.messages), 1)

    def test_transient_publish_failure_retries_same_fresh_visitor_once(self):
        self.publisher.subscribers = 1
        self.publisher.failures = 1
        self.speaker.topic = "/ubuntu/auto_guard/text/tts"
        self.tts["ready"] = True
        self.guard.dispatch("start", {"input_topic": "/ubuntu/camera/color"})
        deadline = time.monotonic() + 4
        while not self.publisher.messages and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual(self.publisher.attempts, 2)
        self.assertEqual(self.publisher.messages, [{"text": "欢迎来到范式智能。"}])
        time.sleep(0.3)
        self.assertEqual(len(self.publisher.messages), 1)

    def test_tts_outage_reconfirms_visible_visitor_after_pending_expires(self):
        self.publisher.subscribers = 1
        self.speaker.topic = "/ubuntu/auto_guard/text/tts"
        calls = 0
        recovery_at = None

        def outage_probe():
            nonlocal calls, recovery_at
            calls += 1
            if calls == 4:
                recovery_at = time.monotonic() + 2.3
            return recovery_at is None or time.monotonic() >= recovery_at

        self.guard._tts_probe = outage_probe
        self.guard.dispatch("start", {"input_topic": "/ubuntu/camera/color"})
        deadline = time.monotonic() + 6
        while not self.publisher.messages and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertIsNotNone(recovery_at)
        self.assertEqual(self.publisher.messages, [{"text": "欢迎来到范式智能。"}])
        self.assertGreaterEqual(self.camera.sequence, 6)

    def test_pending_greeting_reconfirms_after_stale_frame(self):
        self.publisher.subscribers = 1
        self.publisher.failures = 1
        self.speaker.topic = "/ubuntu/auto_guard/text/tts"
        self.tts["ready"] = True
        self.guard.dispatch("start", {"input_topic": "/ubuntu/camera/color"})
        deadline = time.monotonic() + 3
        while self.publisher.attempts == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(self.publisher.attempts, 1)
        self.camera.time_offset_s = -5.0
        while self.camera.sequence < 4 and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertGreaterEqual(self.camera.sequence, 4)
        self.assertEqual(self.publisher.messages, [])
        self.camera.time_offset_s = 0.0
        while not self.publisher.messages and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertGreaterEqual(self.camera.sequence, 7)
        self.assertEqual(self.publisher.messages, [{"text": "欢迎来到范式智能。"}])

    def test_stop_during_final_tts_probe_does_not_publish(self):
        self.publisher.subscribers = 1
        self.speaker.topic = "/ubuntu/auto_guard/text/tts"
        entered = threading.Event()
        release = threading.Event()
        calls = 0

        def blocking_probe():
            nonlocal calls
            calls += 1
            if calls == 4:
                entered.set()
                release.wait(2)
            return True

        self.guard._tts_probe = blocking_probe
        self.guard.dispatch("start", {"input_topic": "/ubuntu/camera/color"})
        self.assertTrue(entered.wait(2))
        stopper = threading.Thread(target=self.guard.stop)
        stopper.start()
        deadline = time.monotonic() + 2
        while not self.guard._stop_event.is_set() and time.monotonic() < deadline:
            time.sleep(0.01)
        release.set()
        stopper.join(3)
        self.assertFalse(stopper.is_alive())
        self.assertEqual(self.publisher.messages, [])

    def test_concurrent_stop_then_start_keeps_new_worker_tracked(self):
        self.publisher.subscribers = 1
        self.speaker.topic = "/ubuntu/auto_guard/text/tts"
        entered = threading.Event()
        release = threading.Event()
        calls = 0

        def blocking_first_probe():
            nonlocal calls
            calls += 1
            if calls == 1:
                entered.set()
                release.wait(2)
            return True

        self.guard._tts_probe = blocking_first_probe
        self.guard.dispatch("start", {"input_topic": "/ubuntu/camera/color"})
        self.assertTrue(entered.wait(2))
        stopper = threading.Thread(target=self.guard.stop)
        stopper.start()
        deadline = time.monotonic() + 2
        while not self.guard._stop_event.is_set() and time.monotonic() < deadline:
            time.sleep(0.01)
        starter = threading.Thread(
            target=lambda: self.guard.dispatch("start", {
                "input_topic": "/ubuntu/camera/color",
            }),
        )
        starter.start()
        release.set()
        stopper.join(3)
        starter.join(3)
        self.assertFalse(stopper.is_alive())
        self.assertFalse(starter.is_alive())
        self.assertIsNotNone(self.guard._thread)
        self.assertTrue(self.guard._thread.is_alive())
        self.assertFalse(self.guard._stop_event.is_set())

    def test_recognition_error_is_visible(self):
        self.publisher.subscribers = 1
        self.speaker.topic = "/ubuntu/auto_guard/text/tts"
        self.tts["ready"] = True
        self.whitelist.recognize_jpeg = lambda jpeg: {
            "state": "error", "error": "model failed",
        }
        self.guard.dispatch("start", {"input_topic": "/ubuntu/camera/color"})
        deadline = time.monotonic() + 2
        while self.guard.dispatch("info", {})["last_error"] is None and time.monotonic() < deadline:
            time.sleep(0.05)
        info = self.guard.dispatch("info", {})
        self.assertEqual(info["last_error"], "model failed")
        self.assertEqual(info["state"], "error")
        self.assertEqual(self.publisher.messages, [])


if __name__ == "__main__":
    unittest.main()
