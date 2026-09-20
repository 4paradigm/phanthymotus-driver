"""Contract tests for the Agent-facing U1 Pro cards without ROS installed."""

from __future__ import annotations

import sys
import types
import unittest
from unittest import mock


def _install_stubs():
    common = types.ModuleType("rclpy")
    node = types.ModuleType("rclpy.node")
    node.Node = object
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)
    common.node, common.qos = node, qos
    sys.modules.update({"rclpy": common, "rclpy.node": node, "rclpy.qos": qos})

    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = type("String", (), {})
    std_msg.Header = type("Header", (), {})
    std.msg = std_msg
    sys.modules.update({"std_msgs": std, "std_msgs.msg": std_msg})

    def message(name):
        return type(name, (), {"__init__": lambda self: None})

    audio = types.ModuleType("audio_msgs")
    audio_msg = types.ModuleType("audio_msgs.msg")
    for name in ("AudioChunk", "AudioInData", "AudioOutData"):
        setattr(audio_msg, name, message(name))
    audio_srv = types.ModuleType("audio_msgs.srv")
    for name in ("EnableAudioIn", "SetAudioVolume"):
        setattr(audio_srv, name, type(name, (), {"Request": message("Request")}))
    audio.msg, audio.srv = audio_msg, audio_srv
    sys.modules.update({"audio_msgs": audio, "audio_msgs.msg": audio_msg, "audio_msgs.srv": audio_srv})

    robo = types.ModuleType("robo_sdk")
    robo_srv = types.ModuleType("robo_sdk.srv")
    robo_srv.StringCall = type("StringCall", (), {"Request": type("Request", (), {"__init__": lambda self: setattr(self, "params", "")})})
    robo.srv = robo_srv
    sys.modules.update({"robo_sdk": robo, "robo_sdk.srv": robo_srv})
    std_srv = types.ModuleType("std_srvs.srv")
    std_srv.Trigger = type("Trigger", (), {"Request": message("Request")})
    sys.modules.update({"std_srvs.srv": std_srv})


_install_stubs()

from device import MIC_TOPIC, PLAYBACK_TOPIC, SPEAKER_TOPIC  # noqa: E402


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeAudioChunk:
    def __init__(self):
        self.format = ""
        self.data = []


class FakeAudioOutData:
    def __init__(self):
        self.uuid = ""
        self.data = types.SimpleNamespace(data=[])


class FakeNodes:
    def __init__(self):
        self.namespace = "test"
        self.mic_topic = "/test/mic/audio"
        self.mic_enabled = []
        self.event_enabled = []
        self.interrupts = 0
        self.string_calls = []
        self.playback_listener = None
        self.mic_publisher = FakePublisher()

    def set_mic_enabled(self, enabled):
        self.mic_enabled.append(enabled)
        return {"code": 0}

    def set_event_enabled(self, name, enabled):
        self.event_enabled.append((name, enabled))

    def call(self, name, request):
        if name == "interrupt":
            self.interrupts += 1
            return {"error_code": 0}
        raise AssertionError(f"unexpected service: {name}")

    def string_call(self, name, params):
        self.string_calls.append((name, params))
        return {"ok": True, "code": "OK", "data": {}}

    def trigger_call(self, name):
        self.interrupts += name == "interrupt"
        return {"success": True}

    def add_playback_listener(self, listener):
        self.playback_listener = listener

    def close_speaker_subscription(self):
        pass


class U1CardContractTests(unittest.TestCase):
    def test_stream_topics_match_robot_contract(self):
        import device

        self.assertEqual(MIC_TOPIC, "/audio/sense/audio_data_to_asr")
        self.assertEqual(SPEAKER_TOPIC, "/sys/device/audio_out/raw")
        self.assertEqual(PLAYBACK_TOPIC, "/robo/media/subscribe/playback_state")
        self.assertEqual(device.EVENT_TOPICS["doa_event"], "/robo/audio/subscribe/doa_event")

    def test_event_bridge_keeps_sdk_string_payloads(self):
        import device

        message = types.SimpleNamespace(data='{"code":"EVENT","data":{"azimuth":12.5}}')
        self.assertEqual(device._event_json(message), message.data)
        self.assertEqual(device._event_data(message.data), {"azimuth": 12.5})

    def test_agent_facing_plugins_exist(self):
        import device

        self.assertTrue(hasattr(device, "MicPlugin"))
        self.assertTrue(hasattr(device, "SpeakerPlugin"))
        self.assertTrue(hasattr(device, "AudioPlugin"))
        self.assertNotIn("AuthPlugin", vars(device))

    def test_nodes_are_registered_with_the_matching_domain_executors(self):
        import device

        class FakeExecutor:
            def __init__(self):
                self.nodes = []

            def add_node(self, node):
                self.nodes.append(node)

            def remove_node(self, node):
                self.nodes.remove(node)

        class FakeRos:
            ctx_robot = object()
            ctx_core = object()

            def __init__(self):
                self.executor_robot = FakeExecutor()
                self.executor_core = FakeExecutor()

        class FakeNode:
            def __init__(self, name, **kwargs):
                self.name = name
                self.clients = {}

            def create_publisher(self, *args, **kwargs):
                return types.SimpleNamespace(publish=lambda message: None)

            def create_subscription(self, *args, **kwargs):
                self.subscriptions = getattr(self, "subscriptions", [])
                self.subscriptions.append(args)
                return types.SimpleNamespace()

            def create_client(self, srv_type, name):
                client = types.SimpleNamespace(srv_name=name)
                self.clients[name] = client
                return client

            def destroy_node(self):
                pass

            def destroy_subscription(self, subscription):
                pass

        original_node = sys.modules["rclpy.node"].Node
        sys.modules["rclpy.node"].Node = FakeNode
        try:
            ros = FakeRos()
            nodes = device.U1Nodes({}, "test", ros)
            self.assertEqual(ros.executor_robot.nodes, [nodes.robot])
            self.assertEqual(ros.executor_core.nodes, [nodes.core])
            self.assertEqual(len(nodes.robot.subscriptions), 3)
            self.assertTrue(all(subscription[0] is sys.modules["std_msgs.msg"].String for subscription in nodes.robot.subscriptions[:2]))
            self.assertEqual(nodes.robot.clients["/robo/audio/call/play_action"].srv_name, "/robo/audio/call/play_action")
            self.assertEqual(nodes.robot.clients["/robo/auth/call/authorize"].srv_name, "/robo/auth/call/authorize")
            nodes.close()
            self.assertEqual(ros.executor_robot.nodes, [])
            self.assertEqual(ros.executor_core.nodes, [])
        finally:
            sys.modules["rclpy.node"].Node = original_node

    def test_event_start_stop_controls_forwarding(self):
        import device

        nodes = FakeNodes()
        plugin = device.EventPlugin(nodes, "doa_event", "test event")
        plugin.start()
        plugin.stop()
        self.assertEqual(nodes.event_enabled, [("doa_event", True), ("doa_event", False)])
        self.assertFalse(plugin.running)

    def test_disabled_event_callback_does_not_publish(self):
        import device

        publisher = FakePublisher()
        nodes = types.SimpleNamespace(String=lambda: types.SimpleNamespace(data=""), _event_forwarding={}, _event_publishers={"event": publisher})
        callback = device.U1Nodes._event_callback(nodes, "event")
        callback(types.SimpleNamespace(value=1))
        self.assertEqual(publisher.messages, [])
        nodes._event_forwarding["event"] = True
        callback(types.SimpleNamespace(value=1))
        self.assertEqual(len(publisher.messages), 1)

    def test_mic_stop_disables_audio_forwarding(self):
        import device

        nodes = FakeNodes()
        plugin = device.MicPlugin(nodes)
        plugin.start()
        plugin.stop()
        self.assertEqual(nodes.mic_enabled, [True, False])
        self.assertFalse(plugin.running)

    def test_mic_start_failure_returns_error_state(self):
        import device

        nodes = FakeNodes()
        nodes.set_mic_enabled = mock.Mock(side_effect=RuntimeError("service unavailable"))
        plugin = device.MicPlugin(nodes)
        result = plugin.dispatch("start", {})
        self.assertEqual(result["state"], "error")
        self.assertFalse(plugin.running)
        self.assertFalse(plugin._enable_requested)

    def test_unknown_actions_return_none(self):
        import device

        nodes = FakeNodes()
        self.assertIsNone(device.MicPlugin(nodes).dispatch("unknown", {}))
        self.assertIsNone(device.SpeakerPlugin(nodes).dispatch("unknown", {}))
        self.assertIsNone(device.AudioPlugin(nodes).dispatch("unknown", {}))
        self.assertEqual(device.SpeakerPlugin(nodes).dispatch("start", {}), {"state": "ready"})

    def test_mic_callback_drops_audio_after_stop(self):
        import device

        publisher = FakePublisher()
        nodes = types.SimpleNamespace(
            _mic_forwarding=True,
            AudioChunk=FakeAudioChunk,
            _mic_publisher=publisher,
        )
        message = types.SimpleNamespace(
            sample_rate=16000,
            channels=1,
            data=types.SimpleNamespace(data=[1, 2, 3]),
        )
        device.U1Nodes._mic_callback(nodes, message)
        nodes._mic_forwarding = False
        device.U1Nodes._mic_callback(nodes, message)
        self.assertEqual(len(publisher.messages), 1)

    def test_lifecycle_initializes_robot_defaults(self):
        import device

        nodes = mock.Mock()
        lifecycle = device._LifecyclePlugin(nodes)
        lifecycle.start()
        nodes.initialize_robot.assert_called_once_with()

    def test_audio_stop_interrupts_vendor_playback(self):
        import device

        nodes = FakeNodes()
        plugin = device.AudioPlugin(nodes)
        self.assertEqual(plugin.dispatch("start", {}), {"state": "ready"})
        plugin.dispatch("play_action", {"motion_id": "A029"})
        with mock.patch.object(device, "_acp_notify"):
            result = plugin.dispatch("stop", {})
        self.assertEqual(nodes.interrupts, 1)
        self.assertEqual(result["state"], "idle")
        self.assertFalse(plugin.running)

    def test_audio_idle_stop_is_stable(self):
        import device

        plugin = device.AudioPlugin(FakeNodes())
        self.assertEqual(plugin.dispatch("stop", {}), {"state": "idle"})

    def test_audio_schema_has_lifecycle_and_completion_contract(self):
        import device

        schema = device.AudioPlugin(FakeNodes()).get_tool()["inputSchema"]
        self.assertEqual(schema["properties"]["action"]["enum"], ["start", "list_actions", "play_action", "play_text", "stop", "info"])
        self.assertEqual(schema["x-completion"]["actions"], ["play_action", "play_text"])

    def test_audio_uses_documented_string_call_payloads(self):
        import device

        nodes = FakeNodes()
        plugin = device.AudioPlugin(nodes)
        plugin.start()
        action = plugin.dispatch("play_action", {"motion_id": "A029", "action_id": "request-1"})
        self.assertEqual(action["state"], "queued")
        self.assertEqual(nodes.string_calls[0][0], "play_action")
        self.assertEqual(nodes.string_calls[0][1]["action"], "A029")
        vendor_uuid = nodes.string_calls[0][1]["uuid"]
        self.assertTrue(vendor_uuid)
        self.assertNotEqual(vendor_uuid, "request-1")
        self.assertEqual(plugin._active["vendor_uuid"], vendor_uuid)

    def test_playback_result_completes_only_matching_active_action(self):
        import device

        nodes = FakeNodes()
        plugin = device.AudioPlugin(nodes)
        plugin.start()
        plugin.dispatch("play_text", {"text": "hello", "action_id": "request-2"})
        vendor_uuid = plugin._active["vendor_uuid"]
        with mock.patch.object(device, "_acp_notify") as notify:
            plugin._on_playback_state({"uuid": "old", "phase": "result", "success": True, "state_name": "COMPLETED"})
            notify.assert_not_called()
            plugin._on_playback_state({"uuid": vendor_uuid, "phase": "feedback", "success": True, "state_name": "COMPLETED"})
            notify.assert_not_called()
            plugin._on_playback_state({"code": "EVENT", "data": {"uuid": vendor_uuid, "phase": "result", "success": True, "state": "COMPLETED", "message": "x" * 1000, "unexpected": "drop"}})
            notify.assert_called_once()
            self.assertEqual(notify.call_args.args[1], "completed")
            playback = notify.call_args.args[2]["playback"]
            self.assertEqual(playback["message"], "x" * 512)
            self.assertNotIn("unexpected", playback)

    def test_speaker_callback_drops_audio_after_stop(self):
        import device

        publisher = FakePublisher()
        nodes = types.SimpleNamespace(
            _speaker_forwarding=True,
            AudioOutData=FakeAudioOutData,
            _speaker_uuid="u1-test",
            _speaker_publisher=publisher,
        )
        message = types.SimpleNamespace(data=[1, 2, 3])
        device.U1Nodes._speaker_callback(nodes, message)
        nodes._speaker_forwarding = False
        device.U1Nodes._speaker_callback(nodes, message)
        self.assertEqual(len(publisher.messages), 1)


if __name__ == "__main__":
    unittest.main()
