import asyncio
import hashlib
import json
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

import person_wave as workflow


class GateTests(unittest.TestCase):
    def test_packaged_instruction_and_attachment_hashes_match_current_sources(self):
        root = Path(__file__).parent
        skill = json.loads((root / "skill.json").read_text(encoding="utf-8"))
        self.assertEqual(skill["instruction"], (root / "instruction.md").read_text(encoding="utf-8").strip())
        self.assertEqual({item["path"] for item in skill["files"]}, {"person_wave.py", "README.md"})
        for item in skill["files"]:
            content = (root / item["path"]).read_bytes()
            self.assertEqual(item["bytes"], len(content))
            self.assertEqual(item["sha256"], hashlib.sha256(content).hexdigest())

    def test_stable_person_triggers_once_and_staying_does_not_repeat(self):
        gate = workflow.PresenceGate()
        self.assertFalse(gate.update(True, 0))
        self.assertFalse(gate.update(True, 0.5))
        self.assertTrue(gate.update(True, 1))
        gate.attempted(1)
        for now in range(2, 70):
            self.assertFalse(gate.update(True, now))

    def test_short_detection_and_brief_absence_do_not_rearm(self):
        gate = workflow.PresenceGate()
        self.assertFalse(gate.update(True, 0))
        self.assertFalse(gate.update(False, 0.5))
        self.assertFalse(gate.update(True, 1))
        self.assertTrue(gate.update(True, 2))
        gate.attempted(2)
        gate.update(False, 3)
        gate.update(False, 4)
        self.assertFalse(gate.update(True, 5))

    def test_return_during_cooldown_is_suppressed_without_delayed_wave(self):
        gate = workflow.PresenceGate()
        gate.update(True, 0)
        self.assertTrue(gate.update(True, 1))
        gate.attempted(1)
        for now in (2, 3, 4, 5):
            gate.update(False, now)
        gate.update(True, 6)
        self.assertFalse(gate.update(True, 7))
        for now in range(8, 40):
            self.assertFalse(gate.update(True, now))
        for now in range(40, 44):
            gate.update(False, now)
        self.assertFalse(gate.update(True, 44))
        self.assertTrue(gate.update(True, 45))

    def test_stream_gap_does_not_confirm_presence_or_clear_latch(self):
        gate = workflow.PresenceGate()
        gate.update(True, 0)
        self.assertFalse(gate.update(True, 10))
        self.assertTrue(gate.update(True, 11))
        gate.attempted(11)
        self.assertFalse(gate.update(True, 100))

    def test_duplicate_and_reversed_observation_times_do_not_trigger(self):
        gate = workflow.PresenceGate()
        gate.update(True, 1)
        self.assertFalse(gate.update(True, 1))
        self.assertFalse(gate.update(True, 0))
        self.assertTrue(gate.update(True, 2))

    def test_invalid_configuration_rejected(self):
        for duration in (0, -1, float("nan"), True):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                workflow.PresenceGate(hold=duration)


class ObservationTests(unittest.TestCase):
    def test_only_confident_person_counts_as_visible(self):
        payload = {"timestamp": 100, "objects": [{"name": "chair", "confidence": 0.9}, {"name": "person", "confidence": 0.49}]}
        self.assertFalse(workflow.person_visible(payload, 100))
        payload["objects"][1]["confidence"] = 0.5
        self.assertTrue(workflow.person_visible(payload, 100))
        self.assertFalse(workflow.person_visible({"timestamp": 100, "objects": []}, 100))

    def test_unknown_stale_and_future_observations_are_not_absence(self):
        for payload in ({}, {"timestamp": 96, "objects": []}, {"timestamp": 102, "objects": []},
                        {"timestamp": 100, "objects": [{"name": "person", "confidence": True}]}):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                workflow.person_visible(payload, 100)

    def test_motion_state_uses_confirmed_faults_not_undocumented_raw_codes(self):
        state = {"fresh": True, "workmode": {"code": 2, "protection": False},
                 "activity": "stationary", "motor_faults": [], "joint_states": [{"error": 1, "fault": False}]}
        self.assertTrue(workflow.motion_ready(state, 0.1))
        for key, value in (("fresh", False), ("activity", "moving"), ("motor_faults", [1])):
            modified = dict(state, **{key: value})
            self.assertFalse(workflow.motion_ready(modified, 0.1))
        self.assertFalse(workflow.motion_ready(state, 3))
        self.assertFalse(workflow.motion_ready(dict(state, workmode={"code": 8, "protection": False}), 0.1))

    def test_gateway_success_is_not_business_success(self):
        with self.assertRaises(RuntimeError):
            workflow.unpack({"code": 200, "data": [{"type": "text", "text": '{"state":"error"}'}]})


class IntegrationTests(unittest.TestCase):
    def test_live_workflow_sends_one_wave_and_observes_auto_completion(self):
        class Fake:
            token = "test-token"
            def __init__(self):
                self.calls = []
                self.wave_at = None
            async def discover(self):
                return {"robot": "r", "motion": "/motion", "detector_info": {"state": "running", "topic_out": [{"topic": "/people", "format": "data/json"}]}}
            async def conditions(self, robot):
                return None
            async def voice_info(self, robot):
                return {"ready": True}
            async def speak_greeting(self, robot):
                self.calls.append(("tts", {"action": "speak", "text": workflow.GREETING}))
                return {"status": "queued", "action_id": "welcome-test"}
            async def reader(self, topic, frames, stopped):
                while not stopped.is_set():
                    now = workflow.time.monotonic()
                    if topic == "/people":
                        payload = {"timestamp": workflow.time.time(), "objects": [{"name": "person", "confidence": 0.9}]}
                    else:
                        waving = self.wave_at is not None and now - self.wave_at < 0.4
                        payload = {"fresh": True, "workmode": {"code": 8 if waving else 2, "protection": False},
                                   "activity": "moving" if waving else "stationary", "motor_faults": []}
                    frames[topic] = (now, payload)
                    await asyncio.sleep(0.02)
            async def call(self, robot, tool, args):
                self.calls.append((tool, args))
                self.wave_at = workflow.time.monotonic()
                return {"command_sent": True, "confirmed_started": True, "workmode": 8}
        async def scenario(client):
            loop = asyncio.get_running_loop()
            with patch.object(loop, "add_signal_handler", return_value=None):
                return await workflow.monitor(client, 1.9, True)
        with tempfile.TemporaryDirectory() as directory, patch.object(workflow, "RUNTIME", Path(directory)):
            client = Fake()
            result = asyncio.run(scenario(client))
            self.assertEqual(client.calls, [("semantic_action", {"action": "wave"}),
                                           ("tts", {"action": "speak", "text": workflow.GREETING})])
            self.assertTrue(result["wave_was_called"])
            self.assertTrue(result["speech_was_queued"])
            self.assertFalse(workflow.read_state()["running"])
            self.assertFalse(workflow.read_state()["wave_may_still_be_finishing"])

    def test_unconfirmed_action_stops_without_retry_and_redacts_token(self):
        class Fake:
            token = "private-test-token"
            calls = 0
            async def discover(self):
                return {"robot": "r", "motion": "/motion", "detector_info": {"state": "running", "topic_out": [{"topic": "/people", "format": "data/json"}]}}
            async def conditions(self, robot):
                return None
            async def voice_info(self, robot):
                return {"ready": True}
            async def reader(self, topic, frames, stopped):
                while not stopped.is_set():
                    payload = ({"timestamp": workflow.time.time(), "objects": [{"name": "person", "confidence": 0.9}]}
                               if topic == "/people" else {"fresh": True, "workmode": {"code": 2, "protection": False}, "activity": "stationary", "motor_faults": []})
                    frames[topic] = (workflow.time.monotonic(), payload)
                    await asyncio.sleep(0.02)
            async def call(self, *args):
                self.calls += 1
                raise TimeoutError("query?token=private-test-token")
        async def scenario(client):
            loop = asyncio.get_running_loop()
            with patch.object(loop, "add_signal_handler", return_value=None):
                return await workflow.monitor(client, 2, True)
        with tempfile.TemporaryDirectory() as directory, patch.object(workflow, "RUNTIME", Path(directory)):
            client = Fake()
            with self.assertRaises(TimeoutError):
                asyncio.run(scenario(client))
            self.assertEqual(client.calls, 1)
            state = workflow.read_state()
            self.assertEqual(state["phase"], "error")
            self.assertNotIn(client.token, json.dumps(state))

    def test_live_mode_requires_project_running_skill_enabled_and_exec_binding(self):
        class Fake(workflow.Client):
            def __init__(self, running=True, enabled=True, bound=True):
                self.running, self.enabled, self.bound = running, enabled, bound
            async def json(self, method, path, body=None):
                if path.endswith("project-running"):
                    return {"running": self.running}
                if path.startswith("/api/skills/"):
                    return {"code": 200, "data": {"active": self.enabled}}
                return {"data": {"cards": [{"id": "core", "toolName": "decision_core"}],
                        "execConnections": [{"fromCardId": "core", "toMcpId": "robot", "toToolName": "semantic_action"}] if self.bound else []}}
        asyncio.run(Fake().conditions("robot"))
        for kwargs in ({"running": False}, {"enabled": False}, {"bound": False}):
            with self.subTest(kwargs=kwargs), self.assertRaises(RuntimeError):
                asyncio.run(Fake(**kwargs).conditions("robot"))

    def test_observe_never_sends_robot_action(self):
        class Fake:
            token = "test-token"
            called = False
            async def discover(self):
                return {"robot": "r", "motion": "/motion", "detector_info": {"state": "running", "topic_out": [{"topic": "/people", "format": "data/json"}]}}
            async def reader(self, topic, frames, stopped):
                while not stopped.is_set():
                    if topic == "/people":
                        payload = {"timestamp": workflow.time.time(), "objects": [{"name": "person", "confidence": 0.9}]}
                    else:
                        payload = {"fresh": True, "workmode": {"code": 2, "protection": False}, "activity": "stationary", "motor_faults": []}
                    frames[topic] = (workflow.time.monotonic(), payload)
                    await asyncio.sleep(0.02)
            async def call(self, *args):
                self.called = True
                raise AssertionError("Observe must not call wave")
        client = Fake()
        result = asyncio.run(workflow.monitor(client, 1.2, False))
        self.assertTrue(result["person_seen"])
        self.assertFalse(result["wave_was_called"])
        self.assertFalse(result["speech_was_queued"])
        self.assertFalse(client.called)


class VoiceTests(unittest.TestCase):
    def test_audio_route_accepts_direct_or_mixed_speech_and_rejects_disconnected_audio(self):
        tts = {"state": "running", "topic_out": [{"format": "audio/pcm-16k", "topic": "/voice"}]}
        direct = {"state": "running", "topic_in": [{"format": "audio/pcm-16k", "topic": "/voice"}]}
        mixed = {"state": "running", "topic_in": [{"format": "audio/pcm-16k", "topic": "/mixed"}]}
        mixer = {"state": "running", "topic_in": [{"format": "audio/pcm-16k", "topic": "/voice"}],
                 "topic_out": [{"format": "audio/pcm-16k", "topic": "/mixed"}]}
        self.assertTrue(workflow.voice_route_ready(tts, direct))
        self.assertTrue(workflow.voice_route_ready(tts, dict(direct, state="playing")))
        self.assertTrue(workflow.voice_route_ready(tts, mixed, mixer))
        self.assertFalse(workflow.voice_route_ready(tts, mixed))
        self.assertFalse(workflow.voice_route_ready(tts, dict(direct, state="idle")))

    def test_welcome_is_submitted_once_and_uncertain_speech_is_not_retried(self):
        class Fake(workflow.Client):
            def __init__(self, result=None, ready=True, fail=False):
                self.calls, self.result, self.ready, self.fail = [], result, ready, fail
            async def voice_info(self, robot):
                return {"provider": "speech", "ready": self.ready}
            async def call(self, provider, tool, arguments):
                self.calls.append((provider, tool, arguments))
                if self.fail:
                    raise TimeoutError("Speech timeout")
                return self.result
        valid = Fake({"status": "queued", "action_id": "greeting-1"})
        self.assertEqual(asyncio.run(valid.speak_greeting("robot"))["action_id"], "greeting-1")
        self.assertEqual(valid.calls, [("speech", "tts", {"action": "speak", "text": workflow.GREETING})])
        for client in (Fake({"state": "loading"}), Fake({"status": "queued"}), Fake(fail=True)):
            with self.subTest(client=client), self.assertRaises((RuntimeError, TimeoutError)):
                asyncio.run(client.speak_greeting("robot"))
            self.assertEqual(len(client.calls), 1)
        disconnected = Fake(ready=False)
        with self.assertRaises(RuntimeError):
            asyncio.run(disconnected.speak_greeting("robot"))
        self.assertEqual(disconnected.calls, [])

    def test_prepare_starts_idle_audio_without_speaking_and_preserves_busy_routes(self):
        class Fake(workflow.Client):
            def __init__(self, busy=False):
                self.calls, self.busy = [], busy
            async def voice_info(self, robot):
                speaker_started = any(t == "speaker" for _, t, _ in self.calls)
                return {"provider": "speech", "tts": {"state": "running", "topic_out": [{"format": "audio/pcm-16k", "topic": "/voice"}]},
                        "speaker": {"state": "running" if self.busy or speaker_started else "idle"},
                        "ready": speaker_started}
            async def call(self, provider, tool, arguments):
                self.calls.append((provider, tool, arguments))
                return {"state": "running"}
        idle = Fake()
        self.assertTrue(asyncio.run(idle.prepare_voice("robot"))["ready"])
        self.assertEqual(idle.calls, [("robot", "speaker", {"action": "start", "input_topic": "/voice"})])
        busy = Fake(busy=True)
        with self.assertRaises(RuntimeError):
            asyncio.run(busy.prepare_voice("robot"))
        self.assertEqual(busy.calls, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
