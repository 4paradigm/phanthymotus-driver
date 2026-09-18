"""Contract tests for the Agent-facing U1 Pro cards without ROS installed."""

from __future__ import annotations

import sys
import types
import unittest


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
    for name in ("AudioChunk", "AudioInData", "AudioOutData", "DoaEvent", "FinishList", "MainWakeupWord", "WakeupEvent", "WakeupState"):
        setattr(audio_msg, name, message(name))
    audio_srv = types.ModuleType("audio_msgs.srv")
    for name in ("EnableAudioIn", "SetAudioVolume"):
        setattr(audio_srv, name, type(name, (), {"Request": message("Request")}))
    audio.msg, audio.srv = audio_msg, audio_srv
    sys.modules.update({"audio_msgs": audio, "audio_msgs.msg": audio_msg, "audio_msgs.srv": audio_srv})

    for package, names in {
        "coze_msgs.srv": ("InterruptActionAudio", "PlayResources"),
        "uworld_action_msgs.srv": ("GetMotionInfoList", "PlayMotion"),
    }.items():
        module = types.ModuleType(package)
        for name in names:
            setattr(module, name, type(name, (), {"Request": message("Request")}))
        sys.modules[package] = module


_install_stubs()

from device import MIC_TOPIC, SPEAKER_TOPIC  # noqa: E402


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

    def close_speaker_subscription(self):
        pass


class U1CardContractTests(unittest.TestCase):
    def test_stream_topics_match_robot_contract(self):
        self.assertEqual(MIC_TOPIC, "/audio/sense/audio_data_to_asr")
        self.assertEqual(SPEAKER_TOPIC, "/sys/device/audio_out/raw")

    def test_agent_facing_plugins_exist(self):
        import device

        self.assertTrue(hasattr(device, "MicPlugin"))
        self.assertTrue(hasattr(device, "SpeakerPlugin"))
        self.assertTrue(hasattr(device, "AudioPlugin"))

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

            def create_publisher(self, *args, **kwargs):
                return types.SimpleNamespace(publish=lambda message: None)

            def create_subscription(self, *args, **kwargs):
                return types.SimpleNamespace()

            def create_client(self, srv_type, name):
                return types.SimpleNamespace(srv_name=name)

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
            nodes.close()
            self.assertEqual(ros.executor_robot.nodes, [])
            self.assertEqual(ros.executor_core.nodes, [])
        finally:
            sys.modules["rclpy.node"].Node = original_node

    def test_event_start_stop_controls_forwarding(self):
        import device

        nodes = FakeNodes()
        plugin = device.EventPlugin(nodes, "wakeup_event", "test event")
        plugin.start()
        plugin.stop()
        self.assertEqual(nodes.event_enabled, [("wakeup_event", True), ("wakeup_event", False)])
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

    def test_audio_stop_interrupts_vendor_playback(self):
        import device

        nodes = FakeNodes()
        plugin = device.AudioPlugin(nodes)
        plugin.start()
        plugin.stop()
        self.assertEqual(nodes.interrupts, 1)
        self.assertFalse(plugin.running)

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
