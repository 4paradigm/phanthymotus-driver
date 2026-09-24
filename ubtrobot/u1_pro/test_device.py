"""Contract tests for the Agent-facing U1 Pro cards without ROS installed."""

from __future__ import annotations

import sys
import io
import json
import os
import threading
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from contextlib import redirect_stdout
from unittest import mock


def _install_stubs():
    common = types.ModuleType("rclpy")
    node = types.ModuleType("rclpy.node")
    node.Node = object
    context = types.ModuleType("rclpy.context")
    context.Context = type("Context", (), {})

    class Executor:
        def __init__(self, context=None):
            self.nodes = []

        def add_node(self, value):
            self.nodes.append(value)

        def remove_node(self, value):
            self.nodes.remove(value)

        def spin_once(self, timeout_sec=None):
            pass

        def shutdown(self):
            pass

    executors = types.ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = Executor
    common.init = lambda **kwargs: None
    common.ok = lambda **kwargs: False
    common.shutdown = lambda **kwargs: None
    common.executors = executors
    common.context = context
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)
    common.node, common.qos = node, qos
    sys.modules.update({"rclpy": common, "rclpy.node": node, "rclpy.qos": qos,
                        "rclpy.context": context, "rclpy.executors": executors})

    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = type("String", (), {})
    std_msg.Header = type("Header", (), {
        "__init__": lambda self: (
            setattr(self, "stamp", types.SimpleNamespace(sec=0, nanosec=0)),
            setattr(self, "frame_id", ""),
        )[-1]
    })
    std_msg.UInt8 = type("UInt8", (), {})
    std.msg = std_msg
    sys.modules.update({"std_msgs": std, "std_msgs.msg": std_msg})

    sensor = types.ModuleType("sensor_msgs")
    sensor_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msg.CompressedImage = type("CompressedImage", (), {
        "__init__": lambda self: setattr(self, "header", types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id=""))})
    sensor.msg = sensor_msg
    sys.modules.update({"sensor_msgs": sensor, "sensor_msgs.msg": sensor_msg})

    shm = types.ModuleType("shm_msgs")
    shm_msg = types.ModuleType("shm_msgs.msg")
    shm_msg.Image6m = type("Image6m", (), {})
    shm.msg = shm_msg
    sys.modules.update({"shm_msgs": shm, "shm_msgs.msg": shm_msg})

    def message(name):
        return type(name, (), {"__init__": lambda self: None})

    audio = types.ModuleType("audio_msgs")
    audio_msg = types.ModuleType("audio_msgs.msg")
    for name in ("AudioChunk", "AudioInData", "AudioOutData", "AudioInfo"):
        setattr(audio_msg, name, message(name))
    audio_srv = types.ModuleType("audio_msgs.srv")
    for name in ("EnableAudioIn", "EnableAudioOut", "SetAudioVolume"):
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
        self.header = types.SimpleNamespace(frame_id="input", stamp=types.SimpleNamespace(sec=12, nanosec=34))
        self.format = ""
        self.data = []


class FakeAudioOutData:
    def __init__(self):
        self.header = None
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

    def wait_for_mic_frame(self, timeout):
        return True

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
    def test_audio_device_domain_matches_vendor_adapter_runtime(self):
        import yaml

        config = yaml.safe_load(Path(__file__).with_name("config.yaml").read_text())
        self.assertEqual(config["ros"]["robot_domain_id"], 20)
        self.assertEqual(config["ros"]["audio_device_domain_id"], 2)

    def test_cyclonedds_config_overrides_inherited_uri(self):
        import common.vendor_runtime as runtime

        configured = "<CycloneDDS><Domain><Tracing><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>"
        with mock.patch.dict(os.environ, {"CYCLONEDDS_URI": "<invalid/>"}, clear=True):
            runtime.configure_cyclonedds({"ros": {
                "robot_interface": "lo",
                "cyclonedds_uri": configured,
            }})
            self.assertEqual(os.environ["CYCLONEDDS_URI"], configured)

    def test_cyclonedds_uses_generated_uri_when_config_uri_is_absent(self):
        import common.vendor_runtime as runtime

        with mock.patch.dict(os.environ, {}, clear=True):
            interface = runtime.configure_cyclonedds({"ros": {"robot_interface": "lo"}})
            self.assertEqual(interface, "lo")
            uri = os.environ["CYCLONEDDS_URI"]
            root = ET.fromstring(uri)
            general = root.find("./Domain/General")
            self.assertIsNotNone(general)
            self.assertEqual(general.findtext("AllowMulticast"), "false")
            self.assertEqual(general.find("./Interfaces/NetworkInterface").attrib["name"], "lo")
            self.assertIsNotNone(root.find("./Domain/Tracing/OutputFile"))

    def test_deployment_shares_vendor_runtime_ipc(self):
        service = Path(__file__).with_name("deploy") / "service.yml"
        text = service.read_text()
        self.assertIn("/tmp/robo/ipc:/tmp/robo/ipc", text)
        self.assertIn("/dev/shm:/dev/shm", text)

    def test_stream_topics_match_robot_contract(self):
        import device

        self.assertEqual(SPEAKER_TOPIC, "/sys/device/audio_out/raw")
        self.assertEqual(MIC_TOPIC, "/sys/device/audio_in/raw")
        self.assertEqual(PLAYBACK_TOPIC, "/robo/media/subscribe/playback_state")
        self.assertEqual(device.EVENT_TOPICS["doa_event"], "/robo/audio/subscribe/doa_event")

    def test_u1_cyclonedds_config_matches_official_sdk_runtime(self):
        config = Path(__file__).with_name("config.yaml").read_text(encoding="utf-8")
        self.assertIn("robot_domain_id: 20", config)
        self.assertIn('robot_interface: "lo"', config)
        self.assertIn("NetworkInterface name='lo'", config)
        self.assertIn("AllowMulticast>false", config)
        self.assertIn("MaxAutoParticipantIndex>200", config)

    def test_mic_uses_verified_adapter_raw_topic(self):
        import device

        nodes = object.__new__(device.U1Nodes)
        class EnableRequest:
            pass
        nodes.EnableAudioIn = types.SimpleNamespace(Request=EnableRequest)
        nodes.call = mock.Mock(return_value=types.SimpleNamespace(code=0, message=""))
        nodes._audio_header = lambda: types.SimpleNamespace()
        nodes._mic_forwarding = False
        nodes._mic_frames = 0
        nodes._mic_frame_event = threading.Event()
        result = nodes.set_mic_enabled(True)
        self.assertTrue(nodes._mic_forwarding)
        self.assertTrue(nodes.call.call_args.args[1].enable)
        self.assertEqual(result["source_topic"], MIC_TOPIC)
        nodes.set_mic_enabled(False)
        self.assertFalse(nodes._mic_forwarding)

    def test_mic_enable_service_failure_is_reported(self):
        import device

        nodes = object.__new__(device.U1Nodes)
        nodes.EnableAudioIn = types.SimpleNamespace(Request=type("Request", (), {}))
        nodes.call = mock.Mock(return_value=types.SimpleNamespace(code=7, message="device busy"))
        nodes._mic_forwarding = True
        with self.assertRaisesRegex(RuntimeError, "code 7"):
            nodes.set_mic_enabled(True)
        self.assertFalse(nodes._mic_forwarding)

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

    def test_build_plugins_follow_plugin_contract(self):
        import device

        nodes = types.SimpleNamespace(
            namespace="test",
            mic_topic="/test/mic/audio",
            add_playback_listener=lambda listener: None,
            initialize_robot=mock.Mock(),
        )
        with mock.patch.object(device, "U1Nodes", return_value=nodes):
            plugins = device.build_plugins({}, "test", object())
        nodes.initialize_robot.assert_called_once_with()

        prefixes = [plugin.PREFIX for plugin in plugins]
        self.assertEqual(prefixes, [
            "lifecycle", "mic", "speaker", "tts", "expression", "head", "system_controls",
            "camera_left", "camera_right", "doa_event",
        ])
        self.assertEqual(len(prefixes), len(set(prefixes)))
        for plugin in plugins:
            self.assertTrue(plugin.PREFIX)
            self.assertTrue(callable(plugin.start))
            self.assertTrue(callable(plugin.stop))
            self.assertTrue(callable(plugin.dispatch))
            result = plugin.dispatch("unknown", {})
            self.assertTrue(result is None or isinstance(result, dict))
            if hasattr(plugin, "get_tool"):
                self.assertEqual(plugin.get_tool()["name"], plugin.PREFIX)

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
        original_context = sys.modules["rclpy.context"].Context
        original_init = sys.modules["rclpy"].init
        original_ok = sys.modules["rclpy"].ok
        original_shutdown = sys.modules["rclpy"].shutdown
        sys.modules["rclpy.node"].Node = FakeNode
        class FakeContext:
            pass

        initialized_domains = []
        sys.modules["rclpy.context"].Context = FakeContext
        sys.modules["rclpy"].init = lambda context, domain_id: initialized_domains.append((context, domain_id))
        sys.modules["rclpy"].ok = lambda context: False
        sys.modules["rclpy"].shutdown = lambda context: None
        try:
            ros = FakeRos()
            nodes = device.U1Nodes({}, "test", ros)
            self.assertEqual(ros.executor_robot.nodes, [nodes.robot])
            self.assertEqual(ros.executor_core.nodes, [nodes.core])
            self.assertEqual(len(nodes.robot.subscriptions), 2)
            self.assertEqual(len(getattr(nodes.audio_device, "subscriptions", [])), 2)
            self.assertEqual(initialized_domains[0][1], 2)
            self.assertEqual(nodes.audio_device.clients["/sys/device/audio_out/set_volume"].srv_name,
                             "/sys/device/audio_out/set_volume")
            self.assertIn("/sys/device/audio_in/raw", [sub[1] for sub in nodes.audio_device.subscriptions])
            self.assertEqual(nodes.robot.clients["/robo/audio/call/play_action"].srv_name, "/robo/audio/call/play_action")
            self.assertEqual(nodes.robot.clients["/robo/auth/call/authorize"].srv_name, "/robo/auth/call/authorize")
            self.assertEqual(nodes.robot.clients["/robo/system/call/set_vision_enabled"].srv_name,
                             "/robo/system/call/set_vision_enabled")
            self.assertEqual(nodes.robot.clients["/robo/system/call/get_vision_enabled"].srv_name,
                             "/robo/system/call/get_vision_enabled")
            nodes.close()
            self.assertEqual(ros.executor_robot.nodes, [])
            self.assertEqual(ros.executor_core.nodes, [])
        finally:
            sys.modules["rclpy.node"].Node = original_node
            sys.modules["rclpy.context"].Context = original_context
            sys.modules["rclpy"].init = original_init
            sys.modules["rclpy"].ok = original_ok
            sys.modules["rclpy"].shutdown = original_shutdown

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
        nodes.set_mic_enabled = mock.Mock(side_effect=[RuntimeError("service unavailable"), {"code": 0}])
        plugin = device.MicPlugin(nodes)
        result = plugin.dispatch("start", {})
        self.assertEqual(result["state"], "error")
        self.assertFalse(plugin.running)
        self.assertFalse(plugin._enable_requested)
        self.assertEqual(nodes.set_mic_enabled.call_args_list[1].args, (False,))

    def test_mic_no_frame_timeout_disables_device(self):
        import device

        nodes = FakeNodes()
        nodes.wait_for_mic_frame = mock.Mock(return_value=False)
        plugin = device.MicPlugin(nodes)
        result = plugin.start()
        self.assertEqual(result["state"], "error")
        self.assertIn("no PCM frames received", result["message"])
        self.assertEqual(nodes.mic_enabled, [True, False])
        self.assertFalse(plugin._enable_requested)
        self.assertFalse(plugin.running)

    def test_mic_timeout_reports_disable_failure_and_keeps_cleanup_pending(self):
        import device

        nodes = FakeNodes()
        nodes.wait_for_mic_frame = mock.Mock(return_value=False)
        nodes.set_mic_enabled = mock.Mock(side_effect=[{"code": 0}, RuntimeError("disable service timeout")])
        plugin = device.MicPlugin(nodes)
        result = plugin.start()
        self.assertEqual(result["state"], "error")
        self.assertIn("disable failed: disable service timeout", result["message"])
        self.assertTrue(plugin._enable_requested)
        self.assertFalse(plugin.running)

    def test_unknown_actions_return_none(self):
        import device

        nodes = FakeNodes()
        self.assertIsNone(device.MicPlugin(nodes).dispatch("unknown", {}))
        self.assertIsNone(device.SpeakerPlugin(nodes).dispatch("unknown", {}))
        self.assertIsNone(device.AudioPlugin(nodes).dispatch("unknown", {}))
        self.assertEqual(device.SpeakerPlugin(nodes).dispatch("start", {})["state"], "waiting_for_input")

    def test_camera_contract_and_expression_contract(self):
        import device

        nodes = FakeNodes()
        audio = device.AudioPlugin(nodes)
        camera = device.EyeCameraPlugin(nodes, "left")
        camera_tool = camera.get_tool()
        self.assertEqual(camera_tool["name"], "camera_left")
        self.assertEqual(camera_tool["topic_out"], [{"topic": "/test/camera/left", "format": "image/jpeg"}])
        self.assertEqual(camera.source_topic, "/sensor/camera/left_eye/color/raw")
        expression = device.ExpressionPlugin(audio)
        expression_tool = expression.get_tool()
        self.assertEqual(expression_tool["name"], "expression")
        self.assertIn("play", expression_tool["inputSchema"]["properties"]["action"]["enum"])
        with self.assertRaises(ValueError):
            expression.dispatch("play", {"name": "not-an-expression"})

    def test_camera_converts_vendor_header_before_publishing(self):
        import device

        publisher = FakePublisher()
        class Header:
            def __init__(self):
                self.stamp = types.SimpleNamespace(sec=0, nanosec=0)
                self.frame_id = ""

        class CompressedImage:
            def __init__(self):
                self.header = Header()
                self.format = ""
                self.data = []

        nodes = types.SimpleNamespace(
            namespace="test",
            CompressedImage=CompressedImage,
            Image6m=types.SimpleNamespace,
            core=types.SimpleNamespace(create_publisher=lambda *args: publisher),
            robot=types.SimpleNamespace(create_subscription=lambda *args: None),
            _sensor_qos=None,
        )
        camera = device.EyeCameraPlugin(nodes, "left")
        camera._publisher = publisher
        frame = types.SimpleNamespace(
            width=1,
            height=1,
            step=3,
            encoding="rgb8",
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=7, nanosec=8),
                frame_id="left-camera",
            ),
            data=[255, 0, 0],
        )
        camera._on_frame(frame)
        self.assertEqual(len(publisher.messages), 1)
        header = publisher.messages[0].header
        self.assertEqual(header.stamp.sec, 7)
        self.assertEqual(header.stamp.nanosec, 8)
        self.assertEqual(header.frame_id, "left-camera")
        self.assertTrue(publisher.messages[0].data)

    def test_expression_excludes_songs_from_dynamic_action_list(self):
        import device

        audio = device.AudioPlugin(FakeNodes())
        expression = device.ExpressionPlugin(audio)
        audio.nodes.string_call = mock.Mock(return_value={"data": {"motion_info_list": [
            {"motion_id": "A001", "motion_name": "眨眼"},
            {"motion_id": "A101", "motion_name": "song"},
        ]}})
        result = expression.dispatch("list_actions", {})
        self.assertEqual(result["actions"], [{"name": "blink", "label": "眨眼"}])
        with self.assertRaisesRegex(ValueError, "not available"):
            expression.dispatch("play", {"name": "smile"})

    def test_expression_parses_nested_json_motion_list(self):
        import device

        response = {"data": json.dumps({"motion_info_list": [
            {"motion_id": "A019", "motion_name": "撒娇"},
            {"motion_id": "A101", "motion_name": "歌曲_1"},
        ]}, ensure_ascii=False)}
        self.assertEqual(device.ExpressionPlugin._expression_actions(response)["actions"],
                         [{"name": "affectionate", "label": "撒娇"}])

    def test_vendor_string_call_message_envelope_decodes_motion_list(self):
        import device

        response = types.SimpleNamespace(
            success=True,
            message=json.dumps({"data": [{"motion_id": "A019", "motion_name": "撒娇"},
                                         {"motion_id": "A014", "motion_name": "点头"}]}),
        )
        decoded = device._decode_vendor_result(response)
        self.assertEqual(device.ExpressionPlugin._expression_actions(decoded)["actions"], [
            {"name": "nod", "label": "点头"},
            {"name": "affectionate", "label": "撒娇"},
        ])

    def test_vendor_string_call_failure_envelope_is_preserved(self):
        import device

        self.assertEqual(device._decode_vendor_result(types.SimpleNamespace(
            success=False, message='{"code":"FAILED"}')),
            {"code": "FAILED", "ok": False})

    def test_authorization_logs_do_not_include_vendor_response(self):
        import device

        nodes = types.SimpleNamespace(
            config={"auth": {key: "secret-value" for key in ("appid", "api_key", "api_secret", "device_id", "license")}},
            trigger_call=mock.Mock(return_value={"code": "UNAUTHORIZED", "data": {"authorized": False}}),
            string_call=mock.Mock(side_effect=[
                {"ok": True, "code": "OK", "data": {"authorized": True, "token": "do-not-log", "license": "private"}},
            ]),
            set_system_enabled=mock.Mock(return_value={"enabled": False}),
        )
        output = io.StringIO()
        with redirect_stdout(output):
            device.U1Nodes.initialize_robot(nodes)
        text = output.getvalue()
        self.assertNotIn("secret-value", text)
        self.assertNotIn("do-not-log", text)
        self.assertIn("authorization request completed", text)
        self.assertIn("authorization request completed", text)

    def test_existing_vendor_authorization_skips_credential_submission(self):
        import device

        nodes = types.SimpleNamespace(
            config={},
            trigger_call=mock.Mock(return_value={"code": "OK", "data": {"authorized": True}}),
            call=mock.Mock(),
            string_call=mock.Mock(return_value={"ok": True, "code": "OK", "data": {"authorized": True}}),
            set_system_enabled=mock.Mock(return_value={"enabled": False}),
        )
        device.U1Nodes.initialize_robot(nodes)
        nodes.trigger_call.assert_called_once_with("auth_state")
        nodes.set_system_enabled.assert_not_called()

    def test_unauthorized_vendor_state_performs_authorization(self):
        import device

        nodes = types.SimpleNamespace(
            config={"auth": {key: "present" for key in ("appid", "api_key", "api_secret", "device_id", "license")}},
            trigger_call=mock.Mock(return_value={"code": "UNAUTHORIZED", "data": {"authorized": False}}),
            string_call=mock.Mock(side_effect=[
                {"ok": True, "code": "OK", "data": {"authorized": True}},
            ]),
            call=mock.Mock(),
            set_system_enabled=mock.Mock(return_value={"enabled": False}),
        )
        device.U1Nodes.initialize_robot(nodes)
        self.assertEqual(nodes.string_call.call_args_list[0].args,
                         ("authorize", mock.ANY))
        nodes.set_system_enabled.assert_not_called()

    def test_authorization_loads_secret_file_and_license(self):
        import device

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "robo.license").write_text('{"license":"test"}', encoding="utf-8")
            (root / "robo_auth.json").write_text(json.dumps({
                "appid": "app",
                "api_key": "key",
                "api_secret": "secret",
                "device_id": "device",
                "license_file": "robo.license",
            }), encoding="utf-8")
            nodes = object.__new__(device.U1Nodes)
            nodes.config = {}
            nodes.string_call = mock.Mock(return_value={"ok": True, "code": "OK", "data": {"authorized": True}})
            nodes.set_system_enabled = mock.Mock(return_value={"enabled": False})
            with mock.patch.dict(os.environ, {"U1_PRO_AUTH_FILE": str(root / "robo_auth.json")}, clear=False):
                nodes.initialize_robot()
            payload = nodes.string_call.call_args_list[0].args[1]
            self.assertEqual(payload["appid"], "app")
            self.assertEqual(payload["license"], '{"license":"test"}')

    def test_authorization_fails_closed_for_missing_credentials(self):
        import device

        nodes = object.__new__(device.U1Nodes)
        nodes.config = {}
        nodes.string_call = mock.Mock()
        with mock.patch.dict(os.environ, {}, clear=True), self.assertRaisesRegex(RuntimeError, "missing fields"):
            nodes.initialize_robot()
        nodes.string_call.assert_not_called()

    def test_authorization_fails_closed_for_malformed_auth_file(self):
        import device

        with tempfile.TemporaryDirectory() as directory:
            auth_path = Path(directory) / "robo_auth.json"
            auth_path.write_text("not json", encoding="utf-8")
            nodes = object.__new__(device.U1Nodes)
            nodes.config = {}
            nodes.string_call = mock.Mock()
            with mock.patch.dict(os.environ, {"U1_PRO_AUTH_FILE": str(auth_path)}, clear=True):
                with self.assertRaisesRegex(RuntimeError, "missing fields"):
                    nodes.initialize_robot()
            nodes.string_call.assert_not_called()

    def test_authorization_fails_closed_for_rejected_vendor_response(self):
        import device

        nodes = types.SimpleNamespace(
            config={"auth": {key: "present" for key in ("appid", "api_key", "api_secret", "device_id", "license")}},
            string_call=mock.Mock(return_value={"ok": False, "code": "AUTHORIZE_FAILED", "data": {}}),
        )
        with self.assertRaisesRegex(RuntimeError, "authorization was rejected"):
            device.U1Nodes.initialize_robot(nodes)

    def test_authorization_failure_does_not_disable_wakeup_or_expose_plugins(self):
        import device

        def fail_authorization():
            raise RuntimeError("U1 Pro authorization was rejected")

        nodes = types.SimpleNamespace(
            config={"auth": {key: "present" for key in ("appid", "api_key", "api_secret", "device_id", "license")}},
            namespace="test",
            mic_topic="/test/mic/audio",
            add_playback_listener=lambda listener: None,
            close=mock.Mock(),
            string_call=mock.Mock(return_value={"ok": False, "code": "AUTHORIZE_FAILED", "data": {}}),
            trigger_call=mock.Mock(return_value={"code": "UNAUTHORIZED", "data": {"authorized": False}}),
            initialize_robot=fail_authorization,
        )
        ros = mock.Mock()
        with mock.patch.object(device, "U1Nodes", return_value=nodes):
            with self.assertRaisesRegex(RuntimeError, "authorization was rejected"):
                device.build_plugins({}, "test", ros)
        nodes.string_call.assert_not_called()
        nodes.close.assert_called_once_with()
        ros.shutdown.assert_called_once_with()

    def test_acp_error_log_escapes_action_id(self):
        import device

        with mock.patch.object(device.urllib.request, "urlopen", side_effect=RuntimeError("transport details")):
            output = io.StringIO()
            with redirect_stdout(output):
                device._acp_notify("request\nforged", "error", {})
        self.assertIn(r"action_id=request\nforged", output.getvalue())
        self.assertNotIn("transport details", output.getvalue())

    def test_video_frame_conversion_strips_step_padding(self):
        import device

        metadata = {"width": 2, "height": 2, "step": 8, "encoding": "rgb8"}
        payload = bytes((255, 0, 0, 0, 255, 0, 99, 99,
                         0, 0, 255, 255, 255, 255, 88, 88))
        jpeg = device._jpeg_from_frame(payload, metadata)
        self.assertTrue(jpeg.startswith(b"\xff\xd8\xff"))

    def test_video_frame_conversion_supports_yuy2_encoding(self):
        import device

        metadata = {"width": 2, "height": 2, "step": 4, "encoding": "yuv422_yuy2"}
        payload = bytes((100, 128, 150, 128, 80, 128, 120, 128))
        jpeg = device._jpeg_from_frame(payload, metadata)
        self.assertTrue(jpeg.startswith(b"\xff\xd8\xff"))

    def test_mic_converts_adapter_raw_message_and_preserves_header(self):
        import device

        publisher = FakePublisher()
        nodes = types.SimpleNamespace(
            _mic_forwarding=True,
            _mic_frames=0,
            _mic_frame_event=threading.Event(),
            AudioChunk=FakeAudioChunk,
            _mic_publisher=publisher,
        )
        message = types.SimpleNamespace(
            sample_rate=16000, channels=1, sample_format="s16_le",
            header=types.SimpleNamespace(frame_id="mic"),
            data=types.SimpleNamespace(data=[1, 2, 3]))
        device.U1Nodes._mic_callback(nodes, message)
        self.assertEqual(len(publisher.messages), 1)
        self.assertIs(publisher.messages[0].header, message.header)
        self.assertEqual(publisher.messages[0].data, [1, 2, 3])

    def test_mic_drops_unsupported_sample_format(self):
        import device

        publisher = FakePublisher()
        nodes = types.SimpleNamespace(
            _mic_forwarding=True,
            _mic_frames=0,
            _mic_frame_event=threading.Event(),
            AudioChunk=FakeAudioChunk,
            _mic_publisher=publisher,
        )
        message = types.SimpleNamespace(
            sample_rate=16000, channels=1, sample_format="float32",
            header=types.SimpleNamespace(), data=types.SimpleNamespace(data=[1, 2, 3]))
        device.U1Nodes._mic_callback(nodes, message)
        self.assertEqual(publisher.messages, [])

    def test_expression_maps_readable_action_name_to_vendor_id(self):
        import device

        nodes = FakeNodes()
        nodes.string_call = mock.Mock(side_effect=[
            {"data": {"motion_info_list": [{"motion_id": "A007", "motion_name": "笑"}]}},
            {"accepted": True},
        ])
        expression = device.ExpressionPlugin(device.AudioPlugin(nodes))
        with mock.patch.object(device, "_acp_notify"):
            result = expression.dispatch("play", {"name": "smile", "action_id": "expr-1"})
        self.assertEqual(result["state"], "queued")
        self.assertEqual(nodes.string_call.call_args_list[0].args, ("motion_list", {}))
        self.assertEqual(nodes.string_call.call_args_list[1].args[0], "play_action")
        self.assertEqual(nodes.string_call.call_args_list[1].args[1]["action"], "A007")

    def test_lifecycle_start_does_not_repeat_robot_initialization(self):
        import device

        nodes = mock.Mock()
        lifecycle = device._LifecyclePlugin(nodes)
        lifecycle.start()
        nodes.initialize_robot.assert_not_called()

    def test_tts_stop_interrupts_vendor_playback(self):
        import device

        nodes = FakeNodes()
        plugin = device.AudioPlugin(nodes)
        self.assertEqual(plugin.dispatch("start", {}), {"state": "ready"})
        plugin.dispatch("speak", {"text": "hello"})
        with mock.patch.object(device, "_acp_notify"):
            result = plugin.dispatch("stop", {})
        self.assertEqual(nodes.interrupts, 1)
        self.assertEqual(result["state"], "idle")
        self.assertFalse(plugin.running)

    def test_tts_idle_stop_is_stable(self):
        import device

        plugin = device.AudioPlugin(FakeNodes())
        self.assertEqual(plugin.dispatch("stop", {}), {"state": "idle"})

    def test_head_stop_cancellation_keeps_head_tool_name(self):
        import device

        nodes = FakeNodes()
        nodes.string_call = mock.Mock(return_value={"data": {"motion_info_list": [
            {"motion_id": "A014", "motion_name": "点头"},
        ]}})
        head = device.HeadPlugin(device.AudioPlugin(nodes))
        head.dispatch("play", {"name": "nod", "action_id": "head-stop-1"})
        with mock.patch.object(device, "_acp_notify") as notify:
            head.stop()
        self.assertEqual(notify.call_args.args[0], "head-stop-1")
        self.assertEqual(notify.call_args.args[1], "cancelled")
        self.assertEqual(notify.call_args.args[3], "head")

    def test_tts_interrupt_is_an_explicit_alias_for_stop(self):
        import device

        nodes = FakeNodes()
        plugin = device.AudioPlugin(nodes)
        plugin.dispatch("speak", {"text": "hello"})
        with mock.patch.object(device, "_acp_notify"):
            result = plugin.dispatch("interrupt", {})
        self.assertEqual(result["state"], "idle")
        self.assertEqual(nodes.interrupts, 1)

    def test_tts_schema_has_completion_contract(self):
        import device

        tool = device.AudioPlugin(FakeNodes()).get_tool()
        schema = tool["inputSchema"]
        self.assertEqual(tool["name"], "tts")
        self.assertEqual(schema["properties"]["action"]["enum"], ["speak", "set_volume", "get_volume", "interrupt", "stop", "info"])
        self.assertEqual(schema["x-completion"]["actions"], ["speak"])

    def test_tts_exposes_shared_speaker_volume_controls(self):
        import device

        actions = device.AudioPlugin(FakeNodes()).get_tool()["inputSchema"]["properties"]["action"]["enum"]
        self.assertIn("set_volume", actions)
        self.assertIn("get_volume", actions)

    def test_head_card_uses_readable_preset_names(self):
        import device

        nodes = FakeNodes()
        nodes.string_call = mock.Mock(return_value={"data": {"motion_info_list": [
            {"motion_id": "A014", "motion_name": "点头"},
            {"motion_id": "A101", "motion_name": "歌曲_1"},
        ]}})
        head = device.HeadPlugin(device.AudioPlugin(nodes))
        self.assertEqual(head.dispatch("list_actions", {})["actions"], [{"name": "nod", "label": "点头"}])
        head_schema = head.get_tool()["inputSchema"]
        self.assertEqual(head_schema["x-completion"]["actions"], ["play"])
        self.assertEqual(head_schema["x-completion"]["timeout"], 120)
        head.dispatch("play", {"name": "nod", "action_id": "head-1"})
        vendor_uuid = head.audio._active["vendor_uuid"]
        with mock.patch.object(device, "_acp_notify") as notify:
            head.audio._on_playback_state({"uuid": vendor_uuid, "phase": "result", "success": True, "state_name": "COMPLETED"})
        self.assertEqual(notify.call_args.args[0], "head-1")
        self.assertEqual(notify.call_args.args[3], "head")
        with self.assertRaisesRegex(ValueError, "not available"):
            head.dispatch("play", {"name": "shake"})

    def test_system_switch_cards_use_documented_vendor_services(self):
        import device

        nodes = mock.Mock()
        nodes.string_call.return_value = {"ok": True}
        nodes.trigger_call.side_effect = [
            {"code": "OK", "data": {"enabled": False}},
            {"code": "OK", "data": {"enabled": False}},
        ]
        nodes.get_system_enabled.return_value = {"code": "OK", "data": {"enabled": False}}
        wakeup = device._SystemSwitchPlugin(nodes, "wakeup_control", "wakeup_enabled", "wakeup_enabled_state", "wakeup")
        vision = device._SystemSwitchPlugin(nodes, "visual_follow_control", "vision_enabled", "vision_enabled_state", "vision")
        self.assertEqual(device.U1Nodes.set_system_enabled(nodes, "wakeup_enabled", False), {
            "ok": True, "requested": False, "enabled": False,
            "state": {"code": "OK", "data": {"enabled": False}},
        })
        self.assertEqual(vision.dispatch("status", {}), {"code": "OK", "data": {"enabled": False}})
        self.assertEqual(wakeup.dispatch("start", {}), {"state": "ready"})
        self.assertEqual(wakeup.dispatch("stop", {}), {"state": "idle"})
        controls = device.SystemControlsPlugin(nodes)
        all_state = controls.dispatch("status", {"control": "all"})
        self.assertEqual(set(all_state), {"wakeup", "wakeup_followup", "visual_behavior"})
        nodes.string_call.assert_called_once_with("wakeup_enabled", {"enabled": False})
        nodes.get_system_enabled.assert_has_calls([
            mock.call("wakeup_enabled_state"), mock.call("vision_enabled_state"),
        ])

    def test_speaker_enables_device_output_before_forwarding(self):
        import device

        nodes = object.__new__(device.U1Nodes)
        nodes.EnableAudioOut = types.SimpleNamespace(Request=type("Request", (), {"ADD": 0}))
        nodes.AudioInfo = type("AudioInfo", (), {})
        nodes._audio_header = lambda: types.SimpleNamespace()
        nodes._speaker_subscription = None
        nodes._speaker_uuid = ""
        nodes._speaker_frames = 0
        nodes._audio_qos = object()
        nodes._speaker_forwarding = False
        nodes.call = mock.Mock(return_value=types.SimpleNamespace(code=0, message=""))
        nodes.core = types.SimpleNamespace(create_subscription=mock.Mock(return_value="subscription"))
        nodes.AudioChunk = object
        result = nodes.connect_speaker("/tts/audio")
        self.assertTrue(nodes.call.call_args.args[1].enable)
        info = nodes.call.call_args.args[1].info
        self.assertEqual((info.channels, info.sample_rate, info.sample_format), (1, 16000, "S16LE"))
        self.assertEqual((nodes.call.call_args.args[1].mode, nodes.call.call_args.args[1].gain), (0, 0.0))
        self.assertEqual(result["input_topic"], "/tts/audio")
        self.assertTrue(nodes._speaker_forwarding)

    def test_tts_uses_documented_play_text_payload(self):
        import device

        nodes = FakeNodes()
        plugin = device.AudioPlugin(nodes)
        plugin.start()
        action = plugin.dispatch("speak", {"text": "hello", "action_id": "request-1"})
        self.assertEqual(action["state"], "queued")
        self.assertEqual(nodes.string_calls[0][0], "play_text")
        self.assertEqual(nodes.string_calls[0][1]["text"], "hello")
        vendor_uuid = nodes.string_calls[0][1]["uuid"]
        self.assertTrue(vendor_uuid)
        self.assertNotEqual(vendor_uuid, "request-1")
        self.assertEqual(plugin._active["vendor_uuid"], vendor_uuid)

    def test_playback_result_completes_only_matching_active_action(self):
        import device

        nodes = FakeNodes()
        plugin = device.AudioPlugin(nodes)
        plugin.start()
        plugin.dispatch("speak", {"text": "hello", "action_id": "request-2"})
        vendor_uuid = plugin._active["vendor_uuid"]
        with mock.patch.object(device, "_acp_notify") as notify:
            plugin._on_playback_state({"uuid": "old", "phase": "result", "success": True, "state_name": "COMPLETED"})
            notify.assert_not_called()
            plugin._on_playback_state({"uuid": vendor_uuid, "phase": "feedback", "success": True, "state_name": "COMPLETED"})
            notify.assert_not_called()
            plugin._on_playback_state({"code": "EVENT", "data": {"uuid": vendor_uuid, "phase": "result", "success": True, "state": "COMPLETED", "message": "x" * 1000, "unexpected": "drop"}})
            notify.assert_called_once()
            self.assertEqual(notify.call_args.args[1], "completed")
            self.assertEqual(notify.call_args.args[3], "tts")
            playback = notify.call_args.args[2]["playback"]
            self.assertEqual(playback["message"], "x" * 512)
            self.assertNotIn("unexpected", playback)

    def test_speaker_callback_drops_audio_after_stop(self):
        import device

        publisher = FakePublisher()
        nodes = types.SimpleNamespace(
            _speaker_forwarding=True,
            _speaker_frames=0,
            AudioOutData=FakeAudioOutData,
            _speaker_uuid="u1-test",
            _speaker_publisher=publisher,
        )
        message = types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id="input", stamp=types.SimpleNamespace(sec=12, nanosec=34)),
            format=device.AUDIO_FORMAT,
            data=[1, 2, 3],
        )
        device.U1Nodes._speaker_callback(nodes, message)
        nodes._speaker_forwarding = False
        device.U1Nodes._speaker_callback(nodes, message)
        self.assertEqual(len(publisher.messages), 1)
        self.assertIs(publisher.messages[0].header, message.header)
        self.assertEqual(publisher.messages[0].data.data, [1, 2, 3])

    def test_speaker_callback_rejects_non_pcm_input(self):
        import device

        publisher = FakePublisher()
        nodes = types.SimpleNamespace(
            _speaker_forwarding=True,
            AudioOutData=FakeAudioOutData,
            _speaker_uuid="u1-test",
            _speaker_publisher=publisher,
        )
        message = types.SimpleNamespace(
            header=types.SimpleNamespace(),
            format="audio/opus",
            data=[1, 2, 3],
        )
        device.U1Nodes._speaker_callback(nodes, message)
        self.assertEqual(publisher.messages, [])


if __name__ == "__main__":
    unittest.main()
