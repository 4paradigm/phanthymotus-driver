"""No-hardware contract tests for the As2W driver cards.

Run with: python3 -m unittest unitree/as2w/test_driver.py
"""
import importlib.util
import struct
import sys
import types
import unittest
from unittest.mock import patch
from pathlib import Path


ROOT = Path(__file__).parent


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _install_device_stubs():
    yaml_module = types.ModuleType("yaml")
    yaml_module.safe_load = lambda _value: {}
    sys.modules.setdefault("yaml", yaml_module)
    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.String = type("String", (), {})
    std_msgs.UInt8MultiArray = type("UInt8MultiArray", (), {})
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std_msgs
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    class Node:
        def __init__(self, *_args, **_kwargs): pass
    rclpy_node.Node = Node
    rclpy.node = rclpy_node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.MultiThreadedExecutor = type("MultiThreadedExecutor", (), {})
    rclpy.executors = rclpy_executors
    sys.modules["rclpy.executors"] = rclpy_executors
    qos = types.ModuleType("rclpy.qos")
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.QoSProfile = lambda **kwargs: kwargs
    sys.modules["rclpy.qos"] = qos
    audio_msgs = types.ModuleType("audio_msgs.msg")
    audio_msgs.AudioChunk = type("AudioChunk", (), {})
    sys.modules["audio_msgs"] = types.ModuleType("audio_msgs")
    sys.modules["audio_msgs.msg"] = audio_msgs
    sensor_msgs = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.CompressedImage = type("CompressedImage", (), {})
    sys.modules["sensor_msgs"] = types.ModuleType("sensor_msgs")
    sys.modules["sensor_msgs.msg"] = sensor_msgs
    for name in ("unitree_sdk2py", "unitree_sdk2py.core", "unitree_sdk2py.idl",
                 "unitree_sdk2py.idl.unitree_go", "unitree_sdk2py.idl.unitree_go.msg",
                 "unitree_sdk2py.idl.sensor_msgs", "unitree_sdk2py.idl.sensor_msgs.msg",
                 "unitree_sdk2py.idl.std_msgs", "unitree_sdk2py.idl.std_msgs.msg",
                 "unitree_sdk2py.idl.unitree_hg", "unitree_sdk2py.idl.unitree_hg.msg"):
        sys.modules.setdefault(name, types.ModuleType(name))
    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelSubscriber = type("ChannelSubscriber", (), {})
    channel.ChannelFactoryInitialize = lambda *_args, **_kwargs: None
    sys.modules["unitree_sdk2py.core.channel"] = channel
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_go.msg.dds_")
    dds.SportModeState_ = type("SportModeState_", (), {})
    sys.modules["unitree_sdk2py.idl.unitree_go.msg.dds_"] = dds
    sensor_dds = types.ModuleType("unitree_sdk2py.idl.sensor_msgs.msg.dds_")
    sensor_dds.PointCloud2_ = type("PointCloud2_", (), {})
    sys.modules["unitree_sdk2py.idl.sensor_msgs.msg.dds_"] = sensor_dds
    std_dds = types.ModuleType("unitree_sdk2py.idl.std_msgs.msg.dds_")
    std_dds.String_ = type("String_", (), {})
    sys.modules["unitree_sdk2py.idl.std_msgs.msg.dds_"] = std_dds
    hg_dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    hg_dds.LowState_ = type("LowState_", (), {})
    hg_dds.BmsState_ = type("BmsState_", (), {})
    sys.modules["unitree_sdk2py.idl.unitree_hg.msg.dds_"] = hg_dds
    rpc_proxy = types.ModuleType("rpc_proxy")
    rpc_proxy.RpcProxy = type("RpcProxy", (), {})
    sys.modules["rpc_proxy"] = rpc_proxy


class _Proxy:
    def __init__(self):
        self.moves = []
        self.stops = 0
        self.state = "AI_STAND_UP"
        self.balance_stands = 0

    def GetState(self):
        return 0, {"fsm_name": self.state}

    def Move(self, *args):
        self.moves.append(args)
        return 0

    def StopMove(self):
        self.stops += 1
        self.state = "AI_STAND_UP"
        return 0

    def BalanceStand(self):
        self.balance_stands += 1
        self.state = "AI_BALANCE_STAND"
        return 0

    def Damp(self):
        return 0

    def RecoveryStand(self):
        self.state = "AI_RECOVERY_STAND"
        return 0


class TestDriverContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _install_device_stubs()
        cls.device = _load("as2w_device_under_test", ROOT / "device.py")
        cls.multimedia = _load("as2w_multimedia_under_test", ROOT / "multimedia.py")
        cls.spatial = _load("as2w_spatial_under_test", ROOT / "controlled_spatial.py")
        cls.mapping = _load("as2w_slam_mapping_under_test", ROOT / "slam_mapping.py")
        cls.main = _load("as2w_main_under_test", ROOT / "main.py")

    @staticmethod
    def _interface(name, ipv4, *, up=True, wireless=False, virtual=False):
        return {"name": name, "ipv4": ipv4, "up": up,
                "wireless": wireless, "virtual": virtual}

    def test_interface_resolver_prefers_cli_over_environment_and_config(self):
        with patch.object(self.main.sys, "argv", ["main.py", "eno9"]), \
                patch.dict(self.main.os.environ, {"NETWORK_INTERFACE": "eno8"}, clear=True), \
                patch.object(self.main, "_network_interfaces", side_effect=AssertionError("must not scan")):
            self.assertEqual("eno9", self.main.resolve_robot_interface({"robot_interface": "eno7"}))

    def test_interface_resolver_uses_environment_override(self):
        with patch.object(self.main.sys, "argv", ["main.py"]), \
                patch.dict(self.main.os.environ, {"NETWORK_INTERFACE": "eno8"}, clear=True), \
                patch.object(self.main, "_network_interfaces", side_effect=AssertionError("must not scan")):
            self.assertEqual("eno8", self.main.resolve_robot_interface({"robot_interface": "eno7"}))

    def test_interface_resolver_uses_explicit_config(self):
        with patch.object(self.main.sys, "argv", ["main.py"]), \
                patch.dict(self.main.os.environ, {}, clear=True), \
                patch.object(self.main, "_network_interfaces", side_effect=AssertionError("must not scan")):
            self.assertEqual("eno7", self.main.resolve_robot_interface({"robot_interface": "eno7"}))

    def test_interface_resolver_auto_selects_unique_unitree_wired_interface(self):
        interfaces = [
            self._interface("wlP1p1s0", "10.100.128.225", wireless=True),
            self._interface("eno1", "192.168.123.222"),
        ]
        with patch.object(self.main.sys, "argv", ["main.py"]), \
                patch.dict(self.main.os.environ, {}, clear=True), \
                patch.object(self.main, "_network_interfaces", return_value=interfaces):
            self.assertEqual("eno1", self.main.resolve_robot_interface({"robot_interface": "auto"}))

    def test_interface_resolver_never_selects_wifi_or_office_network(self):
        interfaces = [
            self._interface("wlan0", "192.168.123.50", wireless=True),
            self._interface("eno1", "10.100.128.225"),
        ]
        with patch.object(self.main.sys, "argv", ["main.py"]), \
                patch.dict(self.main.os.environ, {"NETWORK_INTERFACE": "auto"}, clear=True), \
                patch.object(self.main, "_network_interfaces", return_value=interfaces):
            self.assertIsNone(self.main.resolve_robot_interface({"robot_interface": "eno7"}))

    def test_interface_resolver_rejects_down_and_virtual_candidates(self):
        interfaces = [
            self._interface("eno1", "192.168.123.222", up=False),
            self._interface("docker0", "192.168.123.1", virtual=True),
            self._interface("veth123", "192.168.123.2", virtual=True),
        ]
        with patch.object(self.main.sys, "argv", ["main.py", "auto"]), \
                patch.dict(self.main.os.environ, {"NETWORK_INTERFACE": "eno8"}, clear=True), \
                patch.object(self.main, "_network_interfaces", return_value=interfaces):
            self.assertIsNone(self.main.resolve_robot_interface({"robot_interface": "eno7"}))

    def test_empty_positional_interface_uses_environment_override(self):
        with patch.object(self.main.sys, "argv", ["main.py", ""]), \
                patch.dict(self.main.os.environ, {"NETWORK_INTERFACE": "eno8"}, clear=True), \
                patch.object(self.main, "_network_interfaces", side_effect=AssertionError("must not scan")):
            self.assertEqual("eno8", self.main.resolve_robot_interface(
                {"robot_interface": "eno7"}))

    def test_interface_resolver_requires_override_for_multiple_candidates(self):
        interfaces = [
            self._interface("eno1", "192.168.123.222"),
            self._interface("enp2s0", "192.168.123.223"),
        ]
        with patch.object(self.main.sys, "argv", ["main.py"]), \
                patch.dict(self.main.os.environ, {}, clear=True), \
                patch.object(self.main, "_network_interfaces", return_value=interfaces):
            self.assertIsNone(self.main.resolve_robot_interface({"robot_interface": "auto"}))

    def test_interface_resolver_degrades_when_interface_scan_fails(self):
        with patch.object(self.main.sys, "argv", ["main.py"]), \
                patch.dict(self.main.os.environ, {}, clear=True), \
                patch.object(self.main, "_network_interfaces", side_effect=OSError("ioctl unavailable")):
            self.assertIsNone(self.main.resolve_robot_interface({"robot_interface": "auto"}))

    def test_deployment_does_not_hardcode_robot_interface(self):
        service = (ROOT / "deploy" / "service.yml").read_text()
        dockerfile = (ROOT / "Dockerfile").read_text()
        config = (ROOT / "config.yaml").read_text()
        self.assertNotIn("NETWORK_INTERFACE=", service)
        self.assertIn("robot_interface: auto", config)
        self.assertIn("exec python3 /work/main.py", dockerfile)
        self.assertNotIn("${NETWORK_INTERFACE", dockerfile)

    def test_card_stop_cancels_continuous_move(self):
        proxy = _Proxy()
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        with patch.object(self.device, "_acp_notify") as notify:
            result = plugin.dispatch("move", {"vx": 0.1, "vy": 0, "vyaw": 0, "duration": -1})
            self.assertEqual("running", result["status"])
            stopped = plugin.dispatch("stop", {})
            for _ in range(50):
                if notify.called:
                    break
                __import__("time").sleep(0.01)
        self.assertEqual("idle", stopped["state"])
        self.assertGreaterEqual(proxy.stops, 1)
        self.assertIsNone(plugin._stop)

    def test_loco_moves_directly_from_as2_ai_stand_up(self):
        proxy = _Proxy()
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        result = plugin.dispatch("move", {"vx": 0.2, "vy": 0, "vyaw": 0})
        self.assertEqual(0, result["ret"])
        self.assertEqual("AI_STAND_UP", result["current_state"])
        self.assertEqual(0, proxy.balance_stands)
        self.assertEqual([(0.2, 0, 0)], proxy.moves)

    def test_loco_retries_move_after_stand_up_rejects_first_velocity(self):
        class _NeedsBalanceProxy(_Proxy):
            def Move(self, *args):
                self.moves.append(args)
                return -1 if len(self.moves) == 1 else 0

        proxy = _NeedsBalanceProxy()
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        with patch.object(self.device, "_acp_notify") as notify:
            result = plugin.dispatch("move", {
                "vx": 0.2, "vy": 0, "vyaw": 0, "duration": 0.02})
            self.assertTrue(result["accepted"])
            self.assertEqual("balance_stand", result["transition"])
            action_id = result["action_id"]
            for _ in range(100):
                if any(call.args[0] == action_id for call in notify.call_args_list):
                    break
                __import__("time").sleep(0.01)
        self.assertEqual(1, proxy.balance_stands)
        self.assertTrue(any(
            call.args[0] == action_id and call.args[1] == "completed"
            for call in notify.call_args_list))
        plugin.stop()

    def test_stop_move_returns_before_blocking_rpc_and_completes_via_acp(self):
        entered = __import__("threading").Event()
        release = __import__("threading").Event()

        class _BlockingStopProxy(_Proxy):
            def StopMove(self):
                entered.set()
                release.wait(1)
                return super().StopMove()

        proxy = _BlockingStopProxy()
        generation = [0]
        def motion_snapshot():
            generation[0] += 1
            return {"velocity": [0, 0, 0], "yaw_speed": 0,
                    "received_monotonic": __import__("time").monotonic(),
                    "timestamp": __import__("time").time(),
                    "generation": generation[0], "stream_id": "blocking-stop-test"}
        plugin = self.device.LocoPlugin({}, "test", None, proxy,
                                       motion_snapshot=motion_snapshot)
        with patch.object(self.device, "_acp_notify") as notify:
            started = __import__("time").monotonic()
            result = plugin.dispatch("stop_move", {})
            elapsed = __import__("time").monotonic() - started
            self.assertTrue(result["accepted"])
            self.assertEqual("stopping", result["status"])
            self.assertLess(elapsed, 0.2)
            self.assertTrue(entered.wait(1))
            release.set()
            for _ in range(100):
                if notify.called:
                    break
                __import__("time").sleep(0.01)
        self.assertTrue(notify.called)
        self.assertEqual("completed", notify.call_args.args[1])

    def test_loco_recognizes_as2_ai_walking_and_down_states(self):
        proxy = _Proxy()
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        proxy.state = "AI_FREE_WALK"
        walking = plugin.dispatch("stand_down", {})
        self.assertFalse(walking["accepted"])
        self.assertIn("stop_move", walking["suggested_actions"])
        proxy.state = "AI_STAND_DOWN"
        down = plugin.dispatch("move", {"vx": 0.2, "vy": 0, "vyaw": 0})
        self.assertEqual(-1, down["ret"])
        self.assertIn("stand_up", down["suggested_actions"])

    def test_recovery_stand_accepts_as2_ai_fall_state(self):
        proxy = _Proxy()
        proxy.state = "AI_FALL"
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        with patch.object(self.device, "_acp_notify") as notify:
            result = plugin.dispatch("recovery_stand", {})
            for _ in range(50):
                if notify.called:
                    break
                __import__("time").sleep(0.01)
        self.assertTrue(result["accepted"])
        self.assertEqual(0, result["ret"])

    def test_timed_move_reports_acp_completion(self):
        proxy = _Proxy()
        plugin = self.device.LocoPlugin({}, "test", None, proxy)
        with patch.object(self.device, "_acp_notify") as notify:
            result = plugin.dispatch("move", {
                "vx": 0.2, "vy": 0, "vyaw": 0, "duration": 0.02})
            action_id = result["action_id"]
            for _ in range(100):
                if any(call.args[0] == action_id for call in notify.call_args_list):
                    break
                __import__("time").sleep(0.01)
        matching = [call for call in notify.call_args_list
                    if call.args[0] == action_id]
        self.assertTrue(matching)
        self.assertEqual("completed", matching[-1].args[1])
        self.assertEqual(0, matching[-1].args[2]["ret"])

    def test_state_callbacks_keep_only_the_newest_sample(self):
        node = self.device._StateNode.__new__(self.device._StateNode)
        node._latest_lock = __import__("threading").Lock()
        node._latest_low = None
        node._low_generation = 0
        node._publisher_thread = object()
        first = types.SimpleNamespace(marker="first")
        second = types.SimpleNamespace(marker="second")
        with patch.object(node, "_publish_low") as publish:
            node._on_low(first)
            node._on_low(second)
        self.assertIs(second, node._latest_low)
        self.assertEqual(2, node._low_generation)
        publish.assert_not_called()

    def test_led_clamps_color_and_uses_isolated_audio_rpc(self):
        calls = []
        proxy = types.SimpleNamespace(
            Audio_LedControl=lambda *args: calls.append(args) or 0)
        plugin = self.device.LedPlugin({}, "test", None, proxy)
        result = plugin.dispatch(
            "set_color", {"red": 300, "green": -2, "blue": 18})
        self.assertEqual([255, 0, 18], result["color"])
        self.assertIn((255, 0, 18), calls)
        plugin.stop()

    def test_rpc_channel_ignores_late_timed_out_results(self):
        rpc = _load("as2w_rpc_under_test", ROOT / "rpc_proxy.py")
        channel = rpc._RpcChannel.__new__(rpc._RpcChannel)
        channel._startup_error = None
        channel._lock = __import__("threading").Lock()
        channel._timeout = 0.05
        channel._next_request_id = 1
        channel._last_error = {}
        import queue
        channel._commands = queue.Queue()
        channel._results = queue.Queue()
        channel._results.put({"request_id": 1, "result": "late"})
        channel._results.put({"request_id": 2, "result": "current"})
        self.assertEqual("current", channel.call("Ping"))

    def test_special_actions_are_schema_marked_and_confirmed(self):
        proxy = _Proxy()
        plugin = self.device.SpecialMotionPlugin({}, "test", None, proxy)
        schema = plugin.get_tool()["inputSchema"]
        self.assertEqual("special_motion", plugin.get_tool()["name"])
        self.assertTrue(schema["x-is-dangerous"])
        self.assertEqual(
            {"front_flip", "back_flip", "handstand", "biped_stand"},
            set(schema["x-completion"]["actions"]),
        )
        self.assertEqual(45, schema["x-completion"]["timeout"])
        self.assertIn("confirm", schema["x-action-params"]["front_flip"]["params"])
        self.assertIn("error", plugin.dispatch("front_flip", {}))

    def test_special_motion_refuses_non_standing_state(self):
        proxy = _Proxy()
        proxy.state = "AI_STAND_DOWN"
        plugin = self.device.SpecialMotionPlugin({}, "test", None, proxy)
        result = plugin.dispatch("front_flip", {"confirm": True})
        self.assertEqual(-1, result["ret"])
        self.assertFalse(result["accepted"])
        self.assertIn("standing", result["reason"])

    def test_special_motion_requires_literal_boolean_confirmation(self):
        proxy = types.SimpleNamespace(
            FrontFlip=lambda: 0,
            GetState=lambda: (0, {"fsm_name": "AI_BALANCE_STAND"}))
        plugin = self.device.SpecialMotionPlugin({}, "test", None, proxy)

        for confirm in (None, False, "false", "true", 0, 1, {}, []):
            with self.subTest(confirm=confirm):
                result = plugin.dispatch("front_flip", {"confirm": confirm})
                self.assertFalse(result["ok"])
                self.assertEqual("INVALID_ARGUMENT", result["code"])

        completed = __import__("threading").Event()
        with patch.object(
                self.device, "_acp_notify",
                side_effect=lambda *_args, **_kwargs: completed.set()):
            result = plugin.dispatch("front_flip", {"confirm": True})
            self.assertTrue(result["accepted"])
            self.assertTrue(completed.wait(1))

    def test_special_motion_stop_exits_sustained_posture(self):
        calls = []
        proxy = types.SimpleNamespace(
            HandStand=lambda flag: calls.append(("handstand", flag)) or 0,
            BipedStand=lambda flag: calls.append(("biped_stand", flag)) or 0,
            GetState=lambda: (0, {"fsm_name": "AI_BALANCE_STAND"}),
        )
        plugin = self.device.SpecialMotionPlugin({}, "test", None, proxy)
        completed = __import__("threading").Event()
        with patch.object(self.device, "_acp_notify", side_effect=lambda *_args, **_kwargs: completed.set()):
            result = plugin.dispatch("handstand", {"confirm": True, "enter": True})
            self.assertTrue(result["action_id"].startswith("as2w_special_motion_"))
            self.assertTrue(completed.wait(1))
        self.assertEqual("handstand", plugin._active_posture)
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
        self.assertEqual([("handstand", 1), ("handstand", 0)], calls)

    def test_special_motion_stop_cancels_inflight_posture_enter(self):
        for action, method_name in (
                ("handstand", "HandStand"),
                ("biped_stand", "BipedStand")):
            with self.subTest(action=action):
                entered = __import__("threading").Event()
                release = __import__("threading").Event()
                exited = __import__("threading").Event()
                notified = []
                calls = []

                def posture(flag):
                    calls.append(flag)
                    if flag:
                        entered.set()
                        release.wait(1)
                    else:
                        exited.set()
                    return 0

                proxy = types.SimpleNamespace(
                    **{method_name: posture},
                    GetState=lambda: (0, {"fsm_name": "AI_BALANCE_STAND"}))
                plugin = self.device.SpecialMotionPlugin(
                    {}, "test", None, proxy)
                with patch.object(
                        self.device, "_acp_notify",
                        side_effect=lambda *args, **kwargs:
                        notified.append((args, kwargs))):
                    result = plugin.dispatch(
                        action, {"confirm": True, "enter": True})
                    self.assertTrue(entered.wait(1))

                    stopped = plugin.dispatch("stop", {})
                    self.assertEqual("idle", stopped["state"])
                    self.assertEqual([1], calls)

                    release.set()
                    self.assertTrue(exited.wait(1))
                    for _ in range(100):
                        if notified:
                            break
                        __import__("time").sleep(.01)

                self.assertEqual([1, 0], calls)
                self.assertIsNone(plugin._active_posture)
                self.assertEqual(1, len(notified))
                args, kwargs = notified[0]
                self.assertEqual(result["action_id"], args[0])
                self.assertEqual("cancelled", args[1])
                self.assertTrue(args[2]["cancelled"])
                self.assertEqual(0, args[2]["cancel_ret"])
                self.assertEqual("special_motion", kwargs["tool"])

    def test_special_motion_returns_immediately_and_reports_acp_completion(self):
        entered = __import__("threading").Event()
        release = __import__("threading").Event()
        notified = []

        def front_flip():
            entered.set()
            release.wait(1)
            return 0

        proxy = types.SimpleNamespace(
            FrontFlip=front_flip,
            GetState=lambda: (0, {"fsm_name": "AI_BALANCE_STAND"}))
        plugin = self.device.SpecialMotionPlugin({}, "test", None, proxy)
        with patch.object(
                self.device, "_acp_notify",
                side_effect=lambda *args, **kwargs: notified.append((args, kwargs))):
            result = plugin.dispatch("front_flip", {"confirm": True})
            self.assertTrue(result["accepted"])
            self.assertEqual("running", result["status"])
            self.assertTrue(result["action_id"].startswith("as2w_special_motion_"))
            self.assertTrue(entered.wait(1))
            self.assertEqual([], notified)
            self.assertIn("error", plugin.dispatch("back_flip", {"confirm": True}))
            release.set()
            for _ in range(100):
                if notified:
                    break
                __import__("time").sleep(.01)

        self.assertEqual(1, len(notified))
        args, kwargs = notified[0]
        self.assertEqual(result["action_id"], args[0])
        self.assertEqual("completed", args[1])
        self.assertEqual(0, args[2]["ret"])
        self.assertEqual("special_motion", kwargs["tool"])

    def test_special_motion_exception_reports_error_and_releases_slot(self):
        notified = []
        completed = __import__("threading").Event()

        def fail():
            raise RuntimeError("motion failed")

        proxy = types.SimpleNamespace(
            FrontFlip=fail,
            GetState=lambda: (0, {"fsm_name": "AI_BALANCE_STAND"}))
        plugin = self.device.SpecialMotionPlugin({}, "test", None, proxy)
        with patch.object(
                self.device, "_acp_notify",
                side_effect=lambda *args, **kwargs: (notified.append((args, kwargs)), completed.set())):
            first = plugin.dispatch("front_flip", {"confirm": True})
            self.assertTrue(completed.wait(1))
            self.assertEqual(first["action_id"], notified[0][0][0])
            self.assertEqual("error", notified[0][0][1])
            self.assertIn("RuntimeError", notified[0][0][2]["error"])

            proxy.FrontFlip = lambda: 0
            completed.clear()
            second = plugin.dispatch("front_flip", {"confirm": True})
            self.assertTrue(second["accepted"])
            self.assertNotEqual(first["action_id"], second["action_id"])
            self.assertTrue(completed.wait(1))
            self.assertEqual("completed", notified[1][0][1])

    def test_multimedia_card_contracts_match_verified_hardware(self):
        mic = self.multimedia.MicPlugin.__new__(self.multimedia.MicPlugin)
        mic._topic = "/test/mic/audio"
        speaker = self.multimedia.SpeakerPlugin.__new__(self.multimedia.SpeakerPlugin)
        camera = self.multimedia.CameraPlugin.__new__(self.multimedia.CameraPlugin)
        camera._topic = "/test/camera/front"

        self.assertEqual("audio/pcm-16k", mic.get_tool()["topic_out"][0]["format"])
        self.assertEqual("audio/pcm-16k", speaker.get_tool()["topic_in"][0]["format"])
        self.assertEqual("camera", camera.get_tool()["name"])
        self.assertEqual("image/jpeg", camera.get_tool()["topic_out"][0]["format"])

    def test_degraded_bundle_omits_microphone_card(self):
        created = []

        class FakeMicPlugin:
            def __init__(self, *_args):
                created.append("mic")

        device = types.ModuleType("device")
        device.StatePlugin = device.LocoPlugin = device.SpecialMotionPlugin = object
        device.LedPlugin = object
        multimedia = types.ModuleType("multimedia")
        multimedia.CameraPlugin = multimedia.SpeakerPlugin = object
        multimedia.MicPlugin = FakeMicPlugin
        lidar = types.ModuleType("lidar")
        lidar.LidarPlugin = object
        spatial = types.ModuleType("controlled_spatial")
        spatial.ControlledSpatialPlugin = object
        mapping = types.ModuleType("slam_mapping")
        mapping.SlamMappingPlugin = object
        config = {"plugins": {
            "state": {"enabled": False},
            "loco": {"enabled": False},
            "special_motion": {"enabled": False},
            "mic": {"enabled": True},
            "speaker": {"enabled": False},
            "led": {"enabled": False},
            "camera": {"enabled": False},
            "lidar": {"enabled": False},
            "controlled_spatial": {"enabled": False},
            "slam_mapping": {"enabled": False},
        }}

        modules = {
            "device": device,
            "multimedia": multimedia,
            "lidar": lidar,
            "controlled_spatial": spatial,
            "slam_mapping": mapping,
        }
        with patch.dict(sys.modules, modules):
            degraded = self.main.Bundle(
                config, "test", object(), object(), None, dds_ready=False)
            ready = self.main.Bundle(
                config, "test", object(), object(), "eth0", dds_ready=True)

        self.assertEqual([], degraded.plugins)
        self.assertEqual(["mic"], created)
        self.assertEqual(1, len(ready.plugins))

    def test_camera_start_returns_running_with_async_readiness(self):
        plugin = self.multimedia.CameraPlugin.__new__(self.multimedia.CameraPlugin)
        plugin._topic = "/test/camera/front"
        node = types.SimpleNamespace(state="idle")

        def start_capture():
            node.state = "starting"

        node.start_capture = start_capture
        node.status = lambda: {
            "state": node.state,
            "frames": 0,
            "last_frame_ago_ms": -1,
            "last_error": "",
        }
        plugin._node = node

        result = plugin.dispatch("start", {})

        self.assertEqual("running", result["state"])
        self.assertEqual("starting", result["readiness"])
        self.assertEqual("starting", plugin.dispatch("info", {})["state"])

    def test_camera_start_preserves_backend_failure(self):
        plugin = self.multimedia.CameraPlugin.__new__(self.multimedia.CameraPlugin)
        plugin._topic = "/test/camera/front"
        plugin._node = types.SimpleNamespace(
            start_capture=lambda: {
                "ok": False, "state": "error", "error": "videohub unavailable"},
            status=lambda: {
                "ok": False,
                "state": "error",
                "frames": 0,
                "last_frame_ago_ms": -1,
                "last_error": "videohub unavailable",
            },
        )

        result = plugin.dispatch("start", {})

        self.assertEqual("error", result["state"])
        self.assertEqual("error", result["readiness"])
        self.assertEqual("videohub unavailable", result["last_error"])

    def test_camera_canvas_stop_keeps_state_stream_running(self):
        plugin = self.multimedia.CameraPlugin.__new__(self.multimedia.CameraPlugin)
        plugin._topic = "/test/camera/front"
        stopped = []
        plugin._node = types.SimpleNamespace(
            status=lambda: {
                "ok": True,
                "state": "running",
                "frames": 42,
                "last_frame_ago_ms": 10,
                "last_error": "",
            },
            stop_capture=lambda: stopped.append(True),
        )

        result = plugin.dispatch("stop", {})

        self.assertEqual([], stopped)
        self.assertEqual("idle", result["state"])
        self.assertEqual("running", result["stream_state"])
        self.assertEqual(42, result["frames"])

        plugin.stop()
        self.assertEqual([True], stopped)

    def test_camera_repeated_start_reuses_live_backend(self):
        calls = []
        backend = types.SimpleNamespace(
            is_available=lambda: True,
            status=lambda: {
                "ok": True, "state": "running", "frames": 12,
                "last_frame_ago_ms": 5, "last_error": ""},
            call=lambda *_args: calls.append(_args),
        )
        node = self.multimedia._CameraNode(
            "/test/camera/front", "eth0", 10, 1, 2)
        node._backend = backend

        result = node.start_capture()

        self.assertEqual([], calls)
        self.assertEqual(12, result["frames"])

    def test_zero_pcm_has_no_variation(self):
        self.assertFalse(self.multimedia._pcm_has_variation(b"\x00" * 1024))

    def test_constant_nonzero_pcm_has_no_variation(self):
        pcm = struct.pack("<512h", *([123] * 512))
        self.assertFalse(self.multimedia._pcm_has_variation(pcm))

    def test_pcm_with_a_different_sample_has_variation(self):
        pcm = struct.pack("<512h", *([0] * 511 + [1]))
        self.assertTrue(self.multimedia._pcm_has_variation(pcm))

    def test_mic_start_fails_when_received_pcm_stays_flat(self):
        plugin = self.multimedia.MicPlugin.__new__(self.multimedia.MicPlugin)
        plugin._node = types.SimpleNamespace(
            packet_count=1,
            varying_chunk_count=0,
            state="waiting",
            last_error="",
        )
        now = [0.0]

        def monotonic():
            now[0] += 0.1
            return now[0]

        with patch.object(self.multimedia.time, "monotonic", side_effect=monotonic), \
                patch.object(self.multimedia.time, "sleep", return_value=None):
            state, message = plugin._self_check()

        self.assertEqual("error", state)
        self.assertEqual("error", plugin._node.state)
        self.assertEqual(
            "收到静音数据。请同时按下 L1+L2，将语音状态切换为唤醒模式，"
            "然后重新开启智能控制。",
            message,
        )

    def test_mic_start_without_multicast_returns_wakeup_hint(self):
        plugin = self.multimedia.MicPlugin.__new__(self.multimedia.MicPlugin)
        plugin._node = types.SimpleNamespace(
            packet_count=0,
            varying_chunk_count=0,
            state="waiting",
            last_error="",
        )
        now = [0.0]

        def monotonic():
            now[0] += 0.1
            return now[0]

        with patch.object(self.multimedia.time, "monotonic", side_effect=monotonic), \
                patch.object(self.multimedia.time, "sleep", return_value=None):
            state, message = plugin._self_check()

        self.assertEqual("error", state)
        self.assertEqual("error", plugin._node.state)
        self.assertEqual(
            "未收到麦克风组播数据。请同时按下 L1+L2，"
            "将语音状态切换为唤醒模式，"
            "然后重新开启智能控制。",
            message,
        )

    def test_speaker_info_returns_authoritative_input_topic(self):
        plugin = self.multimedia.SpeakerPlugin.__new__(self.multimedia.SpeakerPlugin)
        plugin._node = types.SimpleNamespace(
            state="ready",
            topic="/current/audio",
            _backend=types.SimpleNamespace(
                is_available=lambda: True,
                status=lambda: {"ok": True, "queue_drops": 0},
            ),
        )
        inferred = plugin.dispatch("info", {"input_topic": "/wired/audio"})
        self.assertEqual(
            [{"topic": "/wired/audio", "format": "audio/pcm-16k"}],
            inferred["topic_in"],
        )
        current = plugin.dispatch("info", {})
        self.assertEqual("/current/audio", current["topic_in"][0]["topic"])

        plugin._node.topic = ""
        unwired = plugin.dispatch("info", {})
        self.assertEqual([{"format": "audio/pcm-16k"}], unwired["topic_in"])

    def test_speaker_lifecycle_recreates_backend_after_stop(self):
        created = []

        class FakeBackend:
            def __init__(self, interface, block_bytes, startup_prefill_bytes,
                         rebuffer_prefill_bytes, max_lead_s):
                self.interface = interface
                self.block_bytes = block_bytes
                self.startup_prefill_bytes = startup_prefill_bytes
                self.rebuffer_prefill_bytes = rebuffer_prefill_bytes
                self.max_lead_s = max_lead_s
                self.error = ""
                self.alive = True
                created.append(self)
            def is_available(self): return self.alive
            def close(self): self.alive = False
            def call(self, *_args, **_kwargs): return {"ok": True}
            def put(self, _pcm): pass

        executor = types.SimpleNamespace(add_node=lambda _node: None)
        with patch.object(self.multimedia, "_SpeakerBackend", FakeBackend):
            plugin = self.multimedia.SpeakerPlugin(
                {"block_ms": 300, "startup_prefill_ms": 300,
                 "rebuffer_prefill_ms": 500, "max_lead_ms": 240},
                "test", executor, "eth0")
            self.assertIsNone(plugin._node._backend)
            self.assertTrue(plugin.start()["ok"])
            first = plugin._node._backend
            self.assertEqual(9600, first.block_bytes)
            self.assertEqual(9600, first.startup_prefill_bytes)
            self.assertEqual(16000, first.rebuffer_prefill_bytes)
            self.assertEqual(.24, first.max_lead_s)
            self.assertEqual("ready", plugin._node.state)
            plugin.stop()
            self.assertIsNone(plugin._node._backend)
            self.assertEqual("idle", plugin._node.state)
            self.assertTrue(plugin.start()["ok"])
            second = plugin._node._backend
            self.assertIsNot(first, second)
            self.assertFalse(first.alive)
            self.assertTrue(second.alive)
            self.assertEqual(2, len(created))

    def test_speaker_lifecycle_start_without_topic_returns_ready(self):
        created = []

        class FakeBackend:
            def __init__(self, *_args):
                self.error = ""
                self.alive = True
                created.append(self)
            def is_available(self): return self.alive
            def close(self): self.alive = False
            def call(self, *_args, **_kwargs): return {"ok": True}

        executor = types.SimpleNamespace(add_node=lambda _node: None)
        with patch.object(self.multimedia, "_SpeakerBackend", FakeBackend):
            plugin = self.multimedia.SpeakerPlugin({}, "test", executor, "eth0")
            self.assertEqual(
                ["input_topic"],
                plugin.get_tool()["inputSchema"]["x-action-params"]["start"]["params"],
            )
            result = plugin.dispatch("start", {})

        self.assertEqual(1, len(created))
        self.assertIs(plugin._node._backend, created[0])
        self.assertEqual({"ok": True, "state": "ready", "topic": ""}, result)

    def test_docker_image_builds_and_validates_owned_audio_msgs(self):
        dockerfile = (ROOT / "Dockerfile").read_text()
        audio_message = (ROOT / "vendor/audio_msgs/msg/AudioChunk.msg").read_text()
        self.assertIn("COPY vendor/audio_msgs/", dockerfile)
        self.assertIn("colcon build --packages-select audio_msgs", dockerfile)
        self.assertIn("test -f /as2w_ws/install/setup.bash", dockerfile)
        self.assertNotIn("/ros_ws/install/setup.bash", dockerfile)
        self.assertIn("from audio_msgs.msg import AudioChunk", dockerfile)
        self.assertEqual(
            ["std_msgs/Header header", "string format", "uint8[] data"],
            audio_message.splitlines(),
        )

    def test_camera_worker_is_pinned_to_verified_videohub_client(self):
        source = (ROOT / "multimedia.py").read_text()
        self.assertIn("unitree_sdk2py.go2.video.video_client", source)
        self.assertNotIn("unitree_sdk2py.b2.front_video.front_video_client", source)

    def test_camera_worker_publishes_only_valid_jpeg(self):
        channel = sys.modules["unitree_sdk2py.core.channel"]
        channel.ChannelFactoryInitialize = lambda *_: None
        video_pkg = types.ModuleType("unitree_sdk2py.go2.video")
        video_client = types.ModuleType("unitree_sdk2py.go2.video.video_client")

        class FakeVideoClient:
            def SetTimeout(self, _timeout): pass
            def Init(self): pass
            def GetImageSample(self): return 0, list(b"\xff\xd8frame\xff\xd9")

        video_client.VideoClient = FakeVideoClient
        sys.modules["unitree_sdk2py.go2.video"] = video_pkg
        sys.modules["unitree_sdk2py.go2.video.video_client"] = video_client
        frames, statuses = __import__("queue").Queue(2), __import__("queue").Queue(4)
        stopped = __import__("threading").Event()
        thread = __import__("threading").Thread(
            target=self.multimedia._camera_worker,
            args=(frames, statuses, stopped, "eth0", 10, 1, .1), daemon=True)
        thread.start()
        self.assertEqual(b"\xff\xd8frame\xff\xd9", frames.get(timeout=1))
        stopped.set()
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())

    def test_camera_process_builds_and_publishes_frames_locally(self):
        channel = sys.modules["unitree_sdk2py.core.channel"]
        channel.ChannelFactoryInitialize = lambda *_: None
        video_pkg = types.ModuleType("unitree_sdk2py.go2.video")
        video_client = types.ModuleType("unitree_sdk2py.go2.video.video_client")

        class FakeVideoClient:
            def SetTimeout(self, _timeout): pass
            def Init(self): pass
            def GetImageSample(self): return 0, list(b"\xff\xd8frame\xff\xd9")

        video_client.VideoClient = FakeVideoClient
        sys.modules["unitree_sdk2py.go2.video"] = video_pkg
        sys.modules["unitree_sdk2py.go2.video.video_client"] = video_client
        published = []

        class FakeMessage:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=None)
                self.format = ""
                self.data = None

        class FakeNode:
            def __init__(self, _name): pass
            def create_publisher(self, *_args):
                return types.SimpleNamespace(publish=published.append)
            def get_clock(self):
                stamp = types.SimpleNamespace(to_msg=lambda: "stamp")
                return types.SimpleNamespace(now=lambda: stamp)
            def destroy_node(self): pass

        q = __import__("queue")
        control, results = q.Queue(), q.Queue()
        rclpy = sys.modules["rclpy"]
        with patch.object(self.multimedia, "Node", FakeNode), \
                patch.object(self.multimedia, "CompressedImage", FakeMessage), \
                patch.object(rclpy, "init", create=True), \
                patch.object(rclpy, "shutdown", create=True):
            thread = __import__("threading").Thread(
                target=self.multimedia._camera_process,
                args=(control, results, "/test/camera", "eth0", 10, 1, .1),
                daemon=True,
            )
            thread.start()
            self.assertTrue(results.get(timeout=1)["ok"])
            control.put(("start", "start", None))
            self.assertTrue(results.get(timeout=1)["ok"])
            status = None
            for _ in range(100):
                control.put(("status", "status", None))
                status = results.get(timeout=1)
                if status["frames"]:
                    break
                __import__("time").sleep(.01)
            self.assertGreaterEqual(status["frames"], 1)
            self.assertEqual("running", status["state"])
            self.assertEqual("jpeg", published[0].format)
            self.assertEqual(b"\xff\xd8frame\xff\xd9", bytes(published[0].data))
            control.put(("close", "close", None))
            self.assertTrue(results.get(timeout=1)["ok"])
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive())

    def test_speaker_worker_uses_a2_voice_service(self):
        channel = sys.modules["unitree_sdk2py.core.channel"]
        channel.ChannelFactoryInitialize = lambda *_: None
        for name in ("unitree_sdk2py.a2", "unitree_sdk2py.a2.audio"):
            sys.modules[name] = types.ModuleType(name)
        audio_client = types.ModuleType("unitree_sdk2py.a2.audio.audio_client")

        class FakeAudioClient:
            instance = None
            def __init__(self):
                self.played = []
                FakeAudioClient.instance = self
            def SetTimeout(self, _timeout): pass
            def Init(self): pass
            def PlayStop(self, _app): return 0
            def PlayStream(self, _app, _stream, pcm):
                self.played.append(pcm)
                return 0, None
            def GetVolume(self): return 0, {"volume": 100}
            def SetVolume(self, _volume): return 0

        audio_client.AudioClient = FakeAudioClient
        sys.modules["unitree_sdk2py.a2.audio.audio_client"] = audio_client
        q = __import__("queue")
        control, results, pcm = q.Queue(), q.Queue(), q.Queue()
        thread = __import__("threading").Thread(
            target=self.multimedia._speaker_worker,
            args=(control, results, pcm, "eth0", 9600, 9600, 16000, .24),
            daemon=True)
        thread.start()
        self.assertTrue(results.get(timeout=1)["ok"])
        pcm.put(b"\x00" * 6400)
        # Playback waits for the configured 300ms startup prefill.
        __import__("time").sleep(.05)
        self.assertEqual([], FakeAudioClient.instance.played)
        pcm.put(b"\x00" * 3200)
        for _ in range(100):
            if len(FakeAudioClient.instance.played) == 1:
                break
            __import__("time").sleep(.01)
        self.assertEqual(
            [b"\x00" * 9600],
            FakeAudioClient.instance.played,
        )
        control.put(("status", "status", None))
        status = results.get(timeout=1)
        self.assertEqual(1, status["play_calls"])
        self.assertEqual(0, status["play_errors"])
        self.assertEqual(9600, status["played_bytes"])
        self.assertEqual(0, status["buffered_bytes"])
        self.assertEqual(9600, status["block_bytes"])
        self.assertEqual(9600, status["startup_prefill_bytes"])
        self.assertEqual(16000, status["rebuffer_prefill_bytes"])
        self.assertEqual(6400, status["first_input_bytes"])
        self.assertGreaterEqual(status["first_input_to_play_ms"], 0)
        self.assertGreaterEqual(status["first_play_rpc_ms"], 0)
        control.put(("volume", "get_volume", None))
        self.assertEqual((0, {"volume": 100}), results.get(timeout=1)["result"])
        control.put(("close", "close", None))
        self.assertTrue(results.get(timeout=1)["ok"])
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())

    def test_speaker_rebuffers_after_underflow_but_continues_across_tts_eof(self):
        channel = sys.modules["unitree_sdk2py.core.channel"]
        channel.ChannelFactoryInitialize = lambda *_: None
        for name in ("unitree_sdk2py.a2", "unitree_sdk2py.a2.audio"):
            sys.modules[name] = types.ModuleType(name)
        audio_client = types.ModuleType("unitree_sdk2py.a2.audio.audio_client")

        class FakeAudioClient:
            instance = None
            def __init__(self):
                self.played = []
                FakeAudioClient.instance = self
            def SetTimeout(self, _timeout): pass
            def Init(self): pass
            def PlayStop(self, _app): return 0
            def PlayStream(self, _app, _stream, pcm):
                self.played.append(pcm)
                return 0, None
            def GetVolume(self): return 0, {"volume": 100}
            def SetVolume(self, _volume): return 0

        audio_client.AudioClient = FakeAudioClient
        sys.modules["unitree_sdk2py.a2.audio.audio_client"] = audio_client
        q = __import__("queue")
        control, results, pcm = q.Queue(), q.Queue(), q.Queue()

        def status():
            control.put(("status", "status", None))
            return results.get(timeout=1)

        with patch.object(self.multimedia, "_SPEAKER_EMPTY_POLL_S", .01), \
                patch.object(self.multimedia, "_SPEAKER_FLUSH_AFTER_IDLE", 1000), \
                patch.object(self.multimedia, "_SPEAKER_PREFILL_FALLBACK_IDLE", 1000):
            thread = __import__("threading").Thread(
                target=self.multimedia._speaker_worker,
                args=(control, results, pcm, "eth0", 320, 640, 960, 0),
                daemon=True)
            thread.start()
            self.assertTrue(results.get(timeout=1)["ok"])

            pcm.put(b"\x00" * 640)
            current = None
            for _ in range(100):
                current = status()
                if current["underflows"]:
                    break
                __import__("time").sleep(.01)
            self.assertEqual(1, current["underflows"])
            self.assertFalse(current["draining"])
            self.assertEqual(960, current["prefill_target_bytes"])

            # One block after starvation must wait for a complete rebuffer.
            played_before = current["play_calls"]
            pcm.put(b"\x00" * 320)
            __import__("time").sleep(.03)
            current = status()
            self.assertEqual(played_before, current["play_calls"])
            self.assertEqual(320, current["buffered_bytes"])

            # Completing the full prefill resumes smoothly.  TTS may already
            # have queued its next internally split segment, so put the EOF and
            # continuation together: it must play without another fixed
            # continuation prefill.
            pcm.put(b"\x00" * 640)
            pcm.put(self.multimedia._AUDIO_EOF_MAGIC)
            pcm.put(b"\x00" * 320)
            pcm.put(self.multimedia._AUDIO_EOF_MAGIC)
            for _ in range(100):
                current = status()
                if (current["rebuffer_count"] and current["eof_count"] == 2
                        and current["continuation_resumes"]):
                    break
                __import__("time").sleep(.01)
            self.assertEqual(1, current["rebuffer_count"])
            self.assertEqual(1, current["continuation_resumes"])
            # With zero allowed playback lead in this accelerated test, the
            # second EOF can immediately expire its deadline and enter the
            # configured recovery buffer again.
            self.assertEqual(960, current["prefill_target_bytes"])
            self.assertGreaterEqual(current["last_rebuffer_wait_ms"], 0)

            control.put(("close", "close", None))
            self.assertTrue(results.get(timeout=1)["ok"])
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive())

    def test_speaker_worker_reports_playstream_errors_and_flushes_eof(self):
        channel = sys.modules["unitree_sdk2py.core.channel"]
        channel.ChannelFactoryInitialize = lambda *_: None
        for name in ("unitree_sdk2py.a2", "unitree_sdk2py.a2.audio"):
            sys.modules[name] = types.ModuleType(name)
        audio_client = types.ModuleType("unitree_sdk2py.a2.audio.audio_client")

        class FailingAudioClient:
            def SetTimeout(self, _timeout): pass
            def Init(self): pass
            def PlayStop(self, _app): return 0
            def PlayStream(self, _app, _stream, _pcm): return 7, None
            def GetVolume(self): return 0, {"volume": 100}
            def SetVolume(self, _volume): return 0

        audio_client.AudioClient = FailingAudioClient
        sys.modules["unitree_sdk2py.a2.audio.audio_client"] = audio_client
        q = __import__("queue")
        control, results, pcm = q.Queue(), q.Queue(), q.Queue()
        thread = __import__("threading").Thread(
            target=self.multimedia._speaker_worker,
            args=(control, results, pcm, "eth0", 9600, 22400, 16000, .24),
            daemon=True)
        thread.start()
        self.assertTrue(results.get(timeout=1)["ok"])
        pcm.put(b"\x01" * 3200)
        pcm.put(self.multimedia._AUDIO_EOF_MAGIC)
        status = None
        for _ in range(100):
            control.put(("status", "status", None))
            status = results.get(timeout=1)
            if status["play_calls"]:
                break
            __import__("time").sleep(.01)
        self.assertEqual(1, status["play_calls"])
        self.assertEqual(1, status["play_errors"])
        self.assertEqual(1, status["eof_count"])
        self.assertEqual(3200, status["attempted_bytes"])
        self.assertEqual(0, status["played_bytes"])
        self.assertFalse(status["draining"])
        self.assertIn("7", status["last_play_error"])
        control.put(("close", "close", None))
        self.assertTrue(results.get(timeout=1)["ok"])
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())

    def test_speaker_deadline_reanchors_after_slow_rpc(self):
        advance = self.multimedia._next_speaker_deadline
        self.assertAlmostEqual(1.3, advance(None, 1.0, 1.02, .3))
        self.assertAlmostEqual(5.0, advance(1.3, 1.4, 5.0, .3))

    def test_speaker_queue_overflow_is_counted(self):
        stats_lock = __import__("threading").Lock()
        stats = {
            "received_chunks": 0,
            "received_bytes": 0,
            "queue_drops": 0,
            "last_input_ts": 0.0,
            "max_input_gap_ms": 0.0,
        }

        class FullOnceQueue:
            def __init__(self):
                self.puts = 0
                self.items = [b"old"]
            def put_nowait(self, value):
                self.puts += 1
                if self.puts == 1:
                    raise __import__("queue").Full
                self.items.append(value)
            def get_nowait(self):
                return self.items.pop(0)

        pcm = FullOnceQueue()
        self.multimedia._put_speaker_pcm(pcm, b"new", stats, stats_lock)

        self.assertEqual(1, stats["queue_drops"])
        self.assertEqual(1, stats["received_chunks"])
        self.assertEqual(3, stats["received_bytes"])
        self.assertEqual([b"new"], pcm.items)

    def test_camera_and_speaker_data_paths_are_isolated_from_main_executor(self):
        import inspect

        speaker_process = inspect.getsource(self.multimedia._speaker_process)
        speaker_backend = inspect.getsource(self.multimedia._SpeakerBackend)
        camera_process = inspect.getsource(self.multimedia._camera_process)

        self.assertIn("create_subscription", speaker_process)
        self.assertIn("_put_speaker_pcm", speaker_process)
        self.assertNotIn("self._pcm", speaker_backend)
        self.assertIn("queue.Queue(maxsize=1)", camera_process)
        self.assertIn('array("B", frame)', camera_process)
        self.assertIn("publisher.publish(message)", camera_process)

    def test_camera_plugin_does_not_register_with_main_executor(self):
        added = []
        executor = types.SimpleNamespace(add_node=added.append)

        plugin = self.multimedia.CameraPlugin(
            {"backend": "videohub", "fps": 10}, "test", executor, "eth0")

        self.assertEqual([], added)
        self.assertEqual("/test/camera/front", plugin._topic)

    def test_mic_waiting_state_explains_voice_assistant_precondition(self):
        node = self.multimedia._MicNode.__new__(self.multimedia._MicNode)
        node.state = "waiting"
        node._started_at = self.multimedia.time.monotonic() - 10
        node.startup_grace_s = 3
        node.last_error = ""
        node.packet_count = 0
        node.last_packet_ts = 0
        result = node.status()
        self.assertEqual("waiting", result["state"])
        self.assertIn("voice assistant", result["message"])

    def test_navigation_declares_completion(self):
        plugin = self.spatial.ControlledSpatialPlugin.__new__(self.spatial.ControlledSpatialPlugin)
        schema = plugin.get_tool()["inputSchema"]
        self.assertIn("navigate_to", schema["x-completion"]["actions"])
        self.assertEqual(180, schema["x-completion"]["timeout"])

    def test_navigation_returns_action_id_without_waiting_for_arrival(self):
        plugin = self.spatial.ControlledSpatialPlugin.__new__(self.spatial.ControlledSpatialPlugin)
        plugin._client = types.SimpleNamespace(call=lambda *_: {"code": 0, "response": "{}"})
        plugin._nav_done = self.spatial.threading.Event()
        plugin._nav_result = None
        plugin._nav_action_id = None
        plugin._nav_lock = self.spatial.threading.Lock()
        with patch.object(self.spatial.threading, "Thread") as thread:
            result = plugin.dispatch("navigate_to", {"x": 1, "y": 2})
        self.assertEqual("navigating", result["status"])
        self.assertTrue(result["action_id"].startswith("as2w_nav_"))
        thread.assert_called_once()

    def test_navigation_buffers_task_result_arriving_during_rpc(self):
        plugin = self.spatial.ControlledSpatialPlugin.__new__(self.spatial.ControlledSpatialPlugin)
        plugin._nav_done = self.spatial.threading.Event()
        plugin._nav_result = None
        plugin._nav_action_id = None
        plugin._nav_lock = self.spatial.threading.Lock()
        def call(action, data):
            plugin._on_slam_key_info(types.SimpleNamespace(data='{"type":"task_result","errorCode":0,"data":{"is_arrived":true}}'))
            return {"code": 0, "response": "{}"}
        plugin._client = types.SimpleNamespace(call=call)
        with patch.object(self.spatial.threading, "Thread") as thread:
            result = plugin.dispatch("navigate_to", {"x": 1, "y": 2})
        self.assertTrue(result["action_id"].startswith("as2w_nav_"))
        self.assertTrue(plugin._nav_done.is_set())
        self.assertTrue(plugin._nav_result["data"]["is_arrived"])
        thread.assert_called_once()

    def test_model_resource_is_textual_urdf(self):
        urdf = (ROOT / "resource" / "as2w.urdf").read_text()
        self.assertIn('<robot name="As2W">', urdf)
        self.assertNotIn("meshes/", urdf)
        for name in ("FL_foot", "FR_foot", "RL_foot", "RR_foot"):
            self.assertIn(f'<joint name="{name}_joint" type="continuous">', urdf)

    def test_state_sensor_info_includes_topic(self):
        plugin = self.device.StatePlugin.__new__(self.device.StatePlugin)
        plugin._namespace = "test"
        for name in ("imu", "joints", "joint_state", "battery", "loco_state"):
            result = plugin.dispatch(name, {})
            self.assertEqual("running", result["state"])
            self.assertTrue(result["topic_out"][0]["topic"].startswith("/test/"))

    def test_lowstate_extra_motor_slots_are_ignored(self):
        node = self.device._StateNode.__new__(self.device._StateNode)
        published = []
        node.imu = node.joints = node.joint_state = node.battery = types.SimpleNamespace(
            publish=lambda message: published.append(message.data))
        motors = [types.SimpleNamespace(q=float(i), dq=0, tau_est=0, temperature=0) for i in range(20)]
        imu = types.SimpleNamespace(quaternion=[], gyroscope=[], accelerometer=[], rpy=[])
        node._on_low(types.SimpleNamespace(imu_state=imu, motor_state=motors, bms_state=None))
        self.assertEqual(3, len(published))
        joint_state = __import__("json").loads(published[1])
        self.assertEqual(12, len([key for key in joint_state if key.endswith("_q")]))
        self.assertIn("FR_hip_joint_q", joint_state)

    def test_joints_payload_keeps_skeleton_contract(self):
        node = self.device._StateNode.__new__(self.device._StateNode)
        published = []
        node.imu = node.joint_state = node.battery = types.SimpleNamespace(publish=lambda message: None)
        node.joints = types.SimpleNamespace(publish=lambda message: published.append(message.data))
        motors = [types.SimpleNamespace(q=float(i), dq=0, tau_est=0, temperature=[0, 0]) for i in range(16)]
        imu = types.SimpleNamespace(quaternion=[1, 0, 0, 0], gyroscope=[], accelerometer=[], rpy=[])
        node._on_low(types.SimpleNamespace(imu_state=imu, motor_state=motors))
        payload = __import__("json").loads(published[0])
        self.assertEqual({"joints", "imu_quat"}, set(payload))
        self.assertEqual(12, len(payload["joints"]))
        self.assertEqual([1, 0, 0, 0], payload["imu_quat"])
        self.assertEqual({"idx", "name", "q", "dq", "tau", "temperature"},
                         set(payload["joints"][0]))

    def test_battery_current_is_explicitly_exposed_in_ma_and_a(self):
        node = self.device._StateNode.__new__(self.device._StateNode)
        published = []
        node.battery = types.SimpleNamespace(publish=lambda message: published.append(message.data))
        node._on_bms(types.SimpleNamespace(soc=87, current=325, cycle=4, temperature=[]))
        payload = __import__("json").loads(published[0])
        self.assertEqual(325, payload["current_ma"])
        self.assertNotIn("current", payload)
        self.assertNotIn("current_a", payload)

    def test_loco_state_does_not_duplicate_imu(self):
        node = self.device._StateNode.__new__(self.device._StateNode)
        published = []
        node.loco = types.SimpleNamespace(publish=lambda message: published.append(message.data))
        node._on_sport(types.SimpleNamespace(mode=2, velocity=[1, 2, 3], position=[4, 5, 6], body_height=0.2,
                                              imu_state=types.SimpleNamespace(rpy=[7, 8, 9])))
        self.assertNotIn("imu_rpy_0", __import__("json").loads(published[0]))

    def test_loco_uses_presets_and_acp_completion(self):
        plugin = self.device.LocoPlugin({}, "test", None, _Proxy())
        schema = plugin.get_tool()["inputSchema"]
        self.assertEqual(["slow", "normal", "fast"], schema["properties"]["speed_preset"]["enum"])
        self.assertIn("stop_move", schema["x-completion"]["actions"])
        self.assertIn("stand_up", schema["x-completion"]["actions"])
        self.assertNotIn("switch_gait", schema["properties"]["action"]["enum"])

    def test_state_stop_then_start_recreates_shared_node(self):
        plugin = self.device.StatePlugin.__new__(self.device.StatePlugin)
        old_state = types.SimpleNamespace(close=lambda: setattr(plugin, "closed", True))
        plugin._namespace = "test"
        plugin._executor = object()
        plugin._state = old_state
        plugin.closed = False
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
        self.assertTrue(plugin.closed)
        self.assertIsNone(plugin._state)
        replacement = object()
        with patch.object(self.device, "_StateNode", return_value=replacement) as node:
            self.assertEqual("running", plugin.dispatch("start", {})["state"])
        node.assert_called_once_with("test", plugin._executor)
        self.assertIs(replacement, plugin._state)

    def test_lidar_stop_then_start_recreates_node(self):
        lidar = _load("as2w_lidar_lifecycle_test", ROOT / "lidar.py")
        plugin = lidar.LidarPlugin.__new__(lidar.LidarPlugin)
        plugin.topic = "/test/lidar/cloud"
        plugin._executor = object()
        plugin._config = {"source_topics": ["rt/test"]}
        plugin.node = types.SimpleNamespace(close=lambda: setattr(plugin, "closed", True))
        plugin.closed = False
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
        self.assertTrue(plugin.closed)
        self.assertIsNone(plugin.node)
        replacement = object()
        with patch.object(lidar, "_LidarNode", return_value=replacement) as node:
            self.assertEqual("running", plugin.dispatch("start", {})["state"])
        node.assert_called_once_with("/test/lidar/cloud", plugin._executor, ["rt/test"])
        self.assertIs(replacement, plugin.node)

    def test_mcp_supports_sse_and_never_falls_back_to_wifi(self):
        source = (ROOT / "main.py").read_text()
        self.assertIn('parsed.path != "/mcp/sse"', source)
        self.assertIn('"/mcp/messages"', source)
        self.assertIn("must never silently bind to the office Wi-Fi", source)

    def test_lidar_uses_direct_sensor_topics_not_conditional_slam_clouds(self):
        source = (ROOT / "lidar.py").read_text()
        self.assertIn('"rt/utlidar/cloud_deskewed"', source)
        self.assertIn('"rt/utlidar/cloud"', source)
        self.assertNotIn('"rt/unitree/slam_mapping/points"', source)

    def test_lidar_render_budget_is_bounded(self):
        lidar = _load("as2w_lidar_budget_test", ROOT / "lidar.py")
        self.assertEqual(2000, lidar._MAX_RENDER_POINTS)

    def test_lidar_process_and_optional_backend_are_shipped(self):
        config = (ROOT / "config.yaml").read_text()
        dockerfile = (ROOT / "Dockerfile").read_text()
        worker = (ROOT / "sensor_worker.py").read_text()
        self.assertIn("process: true", config)
        self.assertIn("max_render_points: 12000", config)
        self.assertIn("lidar_backend.py", dockerfile)
        self.assertIn("sensor_worker.py", dockerfile)
        self.assertIn("def run_lidar", worker)
        self.assertNotIn("def run_camera", worker)

    def test_lidar_backend_falls_back_when_cupy_is_unavailable(self):
        backend = _load("as2w_lidar_backend_test", ROOT / "lidar_backend.py")
        with patch.object(backend.importlib.util, "find_spec", return_value=None):
            instance = backend.PointCloudBackend()
        self.assertEqual("cpu", instance.kind)
        self.assertIn("not installed", instance.reason)

    def test_slam_mapping_card_uses_shared_mapping_contract(self):
        plugin = self.mapping.SlamMappingPlugin.__new__(self.mapping.SlamMappingPlugin)
        plugin._topic = "/test/spatial/mapping"
        tool = plugin.get_tool()
        self.assertEqual("slam_mapping", tool["name"])
        self.assertEqual("sensor", tool["type"])
        self.assertEqual(
            [{"topic": "/test/spatial/mapping", "format": "sensor/mapping"}],
            tool["topic_out"],
        )

    def test_slam_mapping_decodes_offset_and_big_endian_xyz(self):
        raw = b"HEAD" + struct.pack(">fff", 1.25, -2.5, 3.75) + b"TAIL"
        points = self.mapping._SlamMappingNode._decode_points(
            raw, 20, 1, {"x": 4, "y": 8, "z": 12}, True, 100)
        self.assertEqual(1, len(points))
        self.assertAlmostEqual(1.25, points[0][0])
        self.assertAlmostEqual(-2.5, points[0][1])
        self.assertAlmostEqual(3.75, points[0][2])

    def test_slam_mapping_packet_matches_g1_go2_wire_format(self):
        payload = self.mapping._SlamMappingNode._build_payload(
            [(1.0, 2.0, 3.0)], {"x": 4.0, "y": 5.0, "yaw": 0.25})
        x, y, yaw, flags, count = struct.unpack_from("<fffBI", payload, 0)
        self.assertEqual((4.0, 5.0), (x, y))
        self.assertAlmostEqual(-0.25, yaw)
        self.assertEqual(0x03, flags)
        self.assertEqual(1, count)
        self.assertEqual((1.0, 2.0, 3.0), struct.unpack_from("<fff", payload, 17))

    def test_lidar_normalizes_pointcloud_fields_for_renderer(self):
        import struct
        node = self.device  # keep the test module's SDK stubs loaded
        del node
        lidar = _load("as2w_lidar_under_test", ROOT / "lidar.py")
        raw = b"\x00\x00\x00\x00" + struct.pack("<fff", 1.0, 2.0, 3.0) + b"\x00\x00\x00\x00"
        normalized = lidar._LidarNode._to_xyz(raw, 20, 1,
                                               {"x": 4, "y": 8, "z": 12}, False)
        x, y, z = struct.unpack("<fff", normalized)
        self.assertAlmostEqual(1.308, x, places=2)
        self.assertAlmostEqual(2.879, y, places=2)
        self.assertAlmostEqual(-2.0, z, places=3)

    def test_lidar_normalizes_big_endian_xyz(self):
        import struct
        lidar = _load("as2w_lidar_endian_test", ROOT / "lidar.py")
        raw = struct.pack(">fff", 1.0, -2.0, 3.0)
        normalized = lidar._LidarNode._to_xyz(raw, 12, 1,
                                               {"x": 0, "y": 4, "z": 8}, True)
        x, y, z = struct.unpack("<fff", normalized)
        self.assertAlmostEqual(1.308, x, places=2)
        self.assertAlmostEqual(2.879, y, places=2)
        self.assertAlmostEqual(2.0, z, places=3)

    def test_lidar_applies_as2w_jt128_mount_rotation(self):
        import struct
        lidar = _load("as2w_lidar_mount_test", ROOT / "lidar.py")
        raw = struct.pack("<fff", 1.0, 0.0, 0.0)
        normalized = lidar._LidarNode._to_xyz(raw, 12, 1,
                                               {"x": 0, "y": 4, "z": 8}, False)
        x, y, z = struct.unpack("<fff", normalized)
        self.assertAlmostEqual(0.9945, x, places=3)
        self.assertAlmostEqual(-0.1045, y, places=3)
        self.assertAlmostEqual(0.0, z, places=3)


if __name__ == "__main__":
    unittest.main()
