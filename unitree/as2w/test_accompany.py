"""Contract tests for the As2W ``accompany`` supervisor card.

Run from the repository root:

    python3 -m unittest unitree/as2w/test_accompany.py

The suite is deliberately independent of ``test_driver.py``: it loads only
``accompany.py`` plus the two stubs it needs, so it runs on a workstation
without ROS, without a Unitree network, and without the Linux-only ``fcntl``
import that ``multimedia.py`` pulls in.

These tests exist mainly to pin one thing down: the vendor exposes no accompany
enter/exit call, so the card must never regain an action that claims to switch
the mode. If someone later adds ``start_accompany``/``stop_accompany`` to the
enum, ``test_tool_contract_has_no_control_actions`` fails on purpose.
"""
import importlib.util
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).parent


def _stubs():
    """Install just enough of ROS for the module to import on any platform."""
    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.String = type("String", (), {})
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs"))
    sys.modules["std_msgs.msg"] = std_msgs
    qos = types.ModuleType("rclpy.qos")
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.QoSProfile = lambda **kwargs: kwargs
    sys.modules.setdefault("rclpy", types.ModuleType("rclpy"))
    sys.modules["rclpy.qos"] = qos
    node = types.ModuleType("rclpy.node")
    node.Node = type("Node", (), {"__init__": lambda self, *a, **k: None})
    sys.modules["rclpy.node"] = node
    executors = types.ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = type("MultiThreadedExecutor", (), {})
    sys.modules["rclpy.executors"] = executors
    # main.py imports the vendor channel factory at module scope.
    for name in ("unitree_sdk2py", "unitree_sdk2py.core"):
        sys.modules.setdefault(name, types.ModuleType(name))
    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda *_a, **_k: None
    channel.ChannelSubscriber = type("ChannelSubscriber", (), {})
    sys.modules["unitree_sdk2py.core.channel"] = channel


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class _FakeState:
    """Stands in for StatePlugin.remote_snapshot()."""

    def __init__(self, snapshot=None, raises=False):
        self.snapshot = snapshot or {}
        self.raises = raises

    def remote_snapshot(self):
        if self.raises:
            raise RuntimeError("state card stopped")
        return dict(self.snapshot)


class _FakeProxy:
    """Stands in for RpcProxy. Only GetState is used by this card."""

    def __init__(self, fsm="AI_BALANCE_STAND", code=0, raises=False):
        self.fsm = fsm
        self.code = code
        self.raises = raises
        self.calls = 0

    def GetState(self):
        self.calls += 1
        if self.raises:
            raise RuntimeError("rpc timeout")
        return self.code, {"fsm_name": self.fsm}


def _remote(available=True, fresh=True, axes=None, buttons=None, active=False):
    axes = axes if axes is not None else {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}
    return {
        "available": available,
        "fresh": fresh,
        "control_level": "LOWLEVEL",
        "buttons": buttons or {},
        "axes": axes,
        "active": active,
        "timestamp_ms": 1700000000000,
    }


class AccompanyContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _stubs()
        cls.module = _load("as2w_accompany_under_test", ROOT / "accompany.py")
        cls.Plugin = cls.module.AccompanyPlugin

    def make(self, config=None, snapshot=None, proxy=None, state_raises=False):
        plugin = self.Plugin(config or {}, "test", None,
                             proxy=proxy,
                             state_plugin=_FakeState(snapshot or _remote(),
                                                     raises=state_raises))
        return plugin

    # ---------------------------------------------------------- card contract

    def test_tool_contract_is_a_read_only_sensor(self):
        tool = self.make().get_tool()
        self.assertEqual("accompany", tool["name"])
        self.assertEqual("sensor", tool["type"])
        self.assertFalse(tool["multiInstance"])

    def test_tool_contract_declares_the_sensor_stream(self):
        """Agent Core attaches renderers from tools/list, so topic_out must be there."""
        tool = self.make().get_tool()
        self.assertEqual([{"topic": "/test/state/accompany", "format": "data/json"}],
                         tool["topic_out"])

    def test_info_and_tool_agree_on_the_stream(self):
        """The stream must not drift between the tool list and the info action."""
        plugin = self.make()
        self.assertEqual(plugin.get_tool()["topic_out"],
                         plugin.dispatch("info", {})["topic_out"])

    def test_tool_contract_has_no_control_actions(self):
        """The vendor has no enter/exit call, so no action may pretend to have one."""
        tool = self.make().get_tool()
        actions = tool["inputSchema"]["properties"]["action"]["enum"]
        self.assertEqual(["info", "preflight", "status", "guide"], actions)
        for forbidden in ("start_accompany", "stop_accompany", "enter", "exit",
                          "enable", "disable", "toggle"):
            self.assertNotIn(forbidden, actions)

    def test_description_states_the_remote_must_be_powered_on(self):
        text = self.make().get_tool()["description"]
        self.assertIn("MUST be powered on", text)
        self.assertIn("2.4 m", text)
        self.assertIn("2 m/s", text)
        self.assertIn("solid purple", text)

    def test_description_states_joystick_deflection_exits_the_mode(self):
        text = self.make().get_tool()["description"]
        self.assertIn("joystick", text.lower())
        self.assertIn("exits", text)

    def test_description_admits_it_cannot_switch_or_observe_the_keys(self):
        text = self.make().get_tool()["description"]
        self.assertIn("no enter/exit call", text)
        self.assertIn("cannot read the M or", text)

    def test_description_warns_about_app_exclusivity(self):
        text = self.make().get_tool()["description"]
        self.assertIn("Unitree Go App cannot both control", text)

    # ---------------------------------------------------------------- status

    def test_missing_remote_reports_unknown_and_points_at_the_power_key(self):
        plugin = self.make(snapshot=_remote(available=False, fresh=False))
        result = plugin.dispatch("status", {})
        self.assertEqual("unknown", result["mode"])
        self.assertEqual("none", result["confidence"])
        self.assertIn("电源键", result["note"])
        self.assertFalse(result["remote"]["available"])

    def test_stale_remote_is_not_treated_as_absent(self):
        plugin = self.make(snapshot=_remote(available=True, fresh=False))
        result = plugin.dispatch("status", {})
        self.assertEqual("unknown", result["mode"])
        self.assertIn("0.5 s", result["note"])
        self.assertTrue(result["remote"]["available"])
        self.assertFalse(result["remote"]["fresh"])

    def test_empty_fsm_names_never_claims_a_confirmed_mode(self):
        """Default config has no known state name; silence must not become certainty."""
        plugin = self.make(config={})
        result = plugin.dispatch("status", {})
        self.assertEqual("none", result["confidence"])
        self.assertEqual("none", result["mode_source"])
        self.assertIn("fsm_names", result["note"])

    def test_known_fsm_name_confirms_accompany_mode(self):
        plugin = self.make(config={"fsm_names": ["ai_accompany"]},
                           proxy=_FakeProxy(fsm="AI_ACCOMPANY"))
        plugin._poll_fsm()
        result = plugin.dispatch("status", {})
        self.assertEqual("accompanying", result["mode"])
        self.assertEqual("confirmed", result["confidence"])
        self.assertEqual("fsm", result["mode_source"])

    def test_known_fsm_name_list_is_case_insensitive(self):
        plugin = self.make(config={"fsm_names": ["AI_Accompany"]},
                           proxy=_FakeProxy(fsm="ai_accompany"))
        plugin._poll_fsm()
        self.assertEqual("accompanying", plugin.dispatch("status", {})["mode"])

    def test_unlisted_fsm_name_confirms_the_robot_is_not_accompanying(self):
        plugin = self.make(config={"fsm_names": ["ai_accompany"]},
                           proxy=_FakeProxy(fsm="AI_BALANCE_STAND"))
        plugin._poll_fsm()
        result = plugin.dispatch("status", {})
        self.assertEqual("not_accompanying", result["mode"])
        self.assertEqual("confirmed", result["confidence"])

    def test_joystick_input_while_accompanying_becomes_exit_evidence(self):
        plugin = self.make(config={"fsm_names": ["ai_accompany"]},
                           proxy=_FakeProxy(fsm="AI_ACCOMPANY"))
        plugin._poll_fsm()
        plugin._observe(_remote(axes={"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}))
        plugin._observe(_remote(axes={"lx": 0.62, "ly": 0.0, "rx": 0.0, "ry": 0.0}))
        result = plugin.dispatch("status", {})
        self.assertIsNotNone(result["exit_evidence"])
        self.assertEqual("confirmed", result["confidence"])
        self.assertIn("指示灯", result["exit_evidence"]["detail"])

    def test_stale_confirmed_fsm_expires_to_unknown(self):
        """A dead GetState link must not keep claiming confirmed forever.

        Regression for the third review round: after a confirmed accompany
        poll, a lost link plus an unobservable P-key exit used to leave the
        card reporting ``accompanying`` indefinitely.
        """
        plugin = self.make(config={"fsm_names": ["ai_accompany"]},
                           proxy=_FakeProxy(fsm="AI_ACCOMPANY"))
        plugin._poll_fsm()
        self.assertEqual("accompanying", plugin.dispatch("status", {})["mode"])

        with plugin._lock:
            plugin._fsm_at_ms -= int(plugin._fsm_period * 3000) + 1000

        result = plugin.dispatch("status", {})
        self.assertEqual("unknown", result["mode"])
        self.assertEqual("stale", result["confidence"])
        self.assertEqual("fsm_stale", result["mode_source"])
        self.assertIn("过期", result["note"])
        self.assertIsNone(result["exit_evidence"],
                         "stale state must not produce exit evidence")

    def test_a_fresh_poll_after_expiry_restores_confirmed(self):
        plugin = self.make(config={"fsm_names": ["ai_accompany"]},
                           proxy=_FakeProxy(fsm="AI_ACCOMPANY"))
        plugin._poll_fsm()
        with plugin._lock:
            plugin._fsm_at_ms -= int(plugin._fsm_period * 3000) + 1000
        self.assertEqual("stale", plugin.dispatch("status", {})["confidence"])

        plugin._poll_fsm()
        result = plugin.dispatch("status", {})
        self.assertEqual("accompanying", result["mode"])
        self.assertEqual("confirmed", result["confidence"])

    def test_input_outside_accompany_does_not_become_exit_evidence(self):
        """Guards the noisy case: a stick nudge while idle is not an exit."""
        plugin = self.make()
        plugin._observe(_remote(axes={"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}))
        plugin._observe(_remote(axes={"lx": 0.7, "ly": 0.0, "rx": 0.0, "ry": 0.0}))
        result = plugin.dispatch("status", {})
        self.assertIsNone(result["exit_evidence"])
        self.assertIsNotNone(result["last_operator_input"])

    def test_deadzone_does_not_trip_on_stick_drift(self):
        plugin = self.make()
        plugin._observe(_remote(axes={"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}))
        plugin._observe(_remote(axes={"lx": 0.10, "ly": 0.0, "rx": 0.0, "ry": 0.0}))
        self.assertIsNone(plugin.dispatch("status", {})["last_operator_input"])

    def test_stopped_state_card_degrades_instead_of_raising(self):
        plugin = self.make(state_raises=True)
        result = plugin.dispatch("status", {})
        self.assertEqual("unknown", result["mode"])
        self.assertFalse(result["remote"]["available"])

    def test_status_reports_the_indicator_as_the_only_confirmation(self):
        plugin = self.make()
        result = plugin.dispatch("status", {})
        self.assertEqual("purple", result["indicator"]["color"])
        self.assertEqual("伴随模式开启", result["indicator"]["meaning"])

    # ------------------------------------------------------------- preflight

    def test_preflight_blocks_when_the_remote_is_off(self):
        plugin = self.make(snapshot=_remote(available=False, fresh=False))
        result = plugin.dispatch("preflight", {})
        self.assertFalse(result["ready"])
        self.assertIn("remote_powered", result["blocking"])

    def test_preflight_blocks_when_the_remote_is_already_being_worked(self):
        plugin = self.make(snapshot=_remote(axes={"lx": 0.8, "ly": 0.0,
                                                  "rx": 0.0, "ry": 0.0}))
        result = plugin.dispatch("preflight", {})
        self.assertFalse(result["ready"])
        self.assertIn("remote_idle", result["blocking"])

    def test_preflight_blocks_when_the_dog_is_lying_down(self):
        plugin = self.make(config={"fsm_names": ["ai_accompany"]},
                           proxy=_FakeProxy(fsm="AI_STAND_DOWN"))
        plugin._poll_fsm()
        result = plugin.dispatch("preflight", {})
        self.assertFalse(result["ready"])
        self.assertIn("standing", result["blocking"])

    def test_preflight_marks_geometry_checks_as_human_only(self):
        plugin = self.make()
        result = plugin.dispatch("preflight", {})
        for check_id in ("wear_position", "distance", "environment"):
            self.assertIn(check_id, result["needs_human_confirmation"],
                          f"{check_id} must stay a human check, not a sensor claim")
        self.assertIn("没有任何传感器", result["note"])

    def test_preflight_passes_when_only_human_checks_remain(self):
        plugin = self.make(snapshot=_remote())
        result = plugin.dispatch("preflight", {})
        self.assertTrue(result["ready"])
        self.assertEqual([], result["blocking"])
        self.assertIn("M 键", result["next_step"])

    def test_preflight_distance_check_never_reports_a_measured_value(self):
        """There is no range source; a fabricated number would be worse than None."""
        plugin = self.make()
        checks = {item["id"]: item for item in plugin.dispatch("preflight", {})["checks"]}
        self.assertIsNone(checks["distance"]["ok"])
        self.assertIn("无测距源", checks["distance"]["evidence"])

    # ----------------------------------------------------------------- guide

    def test_guide_lists_all_five_documented_exit_triggers(self):
        guide = self.make().dispatch("guide", {})
        ids = [item["id"] for item in guide["exit"]["triggers"]]
        self.assertEqual(["p_key", "joystick", "tilt", "second_remote", "power_off"], ids)

    def test_guide_marks_which_triggers_the_card_can_see(self):
        guide = self.make().dispatch("guide", {})
        observable = {item["id"]: item["observable"]
                      for item in guide["exit"]["triggers"]}
        self.assertFalse(observable["p_key"])
        self.assertFalse(observable["power_off"])
        self.assertTrue(observable["joystick"])

    def test_guide_enter_step_names_the_m_key_and_marks_it_unobservable(self):
        guide = self.make().dispatch("guide", {})
        self.assertIn("M", guide["enter"]["key"])
        self.assertFalse(guide["enter"]["key_observable"])

    def test_guide_carries_the_documented_limits(self):
        guide = self.make().dispatch("guide", {})
        self.assertEqual(2.0, guide["safety"]["max_speed_mps"])
        self.assertEqual(2.4, guide["safety"]["max_distance_m"])
        self.assertEqual(40.0, guide["safety"]["max_tilt_deg"])
        self.assertEqual([0.2, 0.5], guide["safety"]["best_height_m"])

    def test_guide_cautions_cover_app_exclusivity_and_steps(self):
        cautions = " ".join(self.make().dispatch("guide", {})["cautions"])
        self.assertIn("App", cautions)
        self.assertIn("台阶", cautions)
        self.assertIn("2 m/s", cautions)

    def test_guide_cites_the_manual(self):
        guide = self.make().dispatch("guide", {})
        self.assertIn("使用手册", guide["source"])
        self.assertIn("19-21", guide["source"])
        self.assertIn("FAQ 4", guide["source"])

    # ------------------------------------------------------------------ misc

    def test_info_declares_the_card_cannot_enter_or_exit(self):
        result = self.make().dispatch("info", {})
        self.assertEqual("idle", result["state"],
                         "info must not claim running before start()")
        self.assertFalse(result["capability"]["can_enter"])
        self.assertFalse(result["capability"]["can_exit"])
        self.assertIn("没有伴随模式接口", result["capability"]["reason"])
        self.assertEqual("/test/state/accompany", result["topic_out"][0]["topic"])

    def test_unknown_action_is_rejected(self):
        self.assertIsNone(self.make().dispatch("enter", {}))
        self.assertIsNone(self.make().dispatch("dance", {}))

    def test_stop_is_idempotent_without_a_thread(self):
        plugin = self.make()
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])

    def test_proxy_failure_does_not_break_status(self):
        plugin = self.make(config={"fsm_names": ["ai_accompany"]},
                           proxy=_FakeProxy(raises=True))
        plugin._poll_fsm()
        result = plugin.dispatch("status", {})
        self.assertIsNone(result["fsm_name"])
        self.assertEqual("unknown", result["mode"])


class _StubPlugin:
    """Accepts any constructor signature and ignores attach_* wiring."""

    def __init__(self, *_args, **_kwargs):
        pass

    def __getattr__(self, name):
        if name.startswith("attach_") or name in ("is_moving", "is_running"):
            return lambda *_args, **_kwargs: None
        raise AttributeError(name)

    def get_tool(self):
        return {"name": "stub", "type": "sensor"}

    def get_tools(self):
        return [self.get_tool()]

    def dispatch(self, *_args, **_kwargs):
        return None

    def start(self):
        pass

    def stop(self):
        pass


class BundleWiringTests(unittest.TestCase):
    """The card is registered through main.py, which also copies files by name."""

    def test_bundle_registers_accompany_card_when_enabled(self):
        from unittest.mock import patch

        sys.path.insert(0, str(ROOT))
        stubs = {
            "yaml": types.ModuleType("yaml"),
            "rpc_proxy": types.ModuleType("rpc_proxy"),
            "device": types.ModuleType("device"),
            "multimedia": types.ModuleType("multimedia"),
            "lidar": types.ModuleType("lidar"),
            "controlled_spatial": types.ModuleType("controlled_spatial"),
            "slam_mapping": types.ModuleType("slam_mapping"),
            "loco_servo": types.ModuleType("loco_servo"),
            "vision_capture": types.ModuleType("vision_capture"),
        }
        stubs["yaml"].safe_load = lambda _value: {}
        stubs["rpc_proxy"].RpcProxy = object
        for name in ("device", "multimedia", "lidar", "controlled_spatial",
                     "slam_mapping", "loco_servo", "vision_capture"):
            for attr in ("StatePlugin", "LocoPlugin", "SpecialMotionPlugin",
                         "LedPlugin", "CameraPlugin", "MicPlugin", "SpeakerPlugin",
                         "LidarPlugin", "ControlledSpatialPlugin", "SlamMappingPlugin",
                         "LocoServoPlugin", "VisionCapturePlugin"):
                setattr(stubs[name], attr, _StubPlugin)
        # main.py imports the card by module name, so register it under that name.
        stubs["accompany"] = _load("accompany", ROOT / "accompany.py")

        with patch.dict(sys.modules, stubs):
            main = _load("as2w_main_for_accompany", ROOT / "main.py")
            bundle = main.Bundle(
                {"plugins": {"accompany": {"enabled": True}}},
                "test", object(), object(), None, dds_ready=True)
            names = [tool["name"] for tool in bundle.tools()]
            self.assertIn("accompany", names)

    def test_bundle_skips_accompany_when_disabled_or_degraded(self):
        from unittest.mock import patch

        sys.path.insert(0, str(ROOT))
        stubs = {
            "yaml": types.ModuleType("yaml"),
            "rpc_proxy": types.ModuleType("rpc_proxy"),
            "device": types.ModuleType("device"),
            "multimedia": types.ModuleType("multimedia"),
            "lidar": types.ModuleType("lidar"),
            "controlled_spatial": types.ModuleType("controlled_spatial"),
            "slam_mapping": types.ModuleType("slam_mapping"),
            "loco_servo": types.ModuleType("loco_servo"),
            "vision_capture": types.ModuleType("vision_capture"),
        }
        stubs["yaml"].safe_load = lambda _value: {}
        stubs["rpc_proxy"].RpcProxy = object
        for name in ("device", "multimedia", "lidar", "controlled_spatial",
                     "slam_mapping", "loco_servo", "vision_capture"):
            for attr in ("StatePlugin", "LocoPlugin", "SpecialMotionPlugin",
                         "LedPlugin", "CameraPlugin", "MicPlugin", "SpeakerPlugin",
                         "LidarPlugin", "ControlledSpatialPlugin", "SlamMappingPlugin",
                         "LocoServoPlugin", "VisionCapturePlugin"):
                setattr(stubs[name], attr, _StubPlugin)
        stubs["accompany"] = _load("accompany", ROOT / "accompany.py")

        with patch.dict(sys.modules, stubs):
            main = _load("as2w_main_for_accompany_disabled", ROOT / "main.py")
            disabled = main.Bundle(
                {"plugins": {"accompany": {"enabled": False}}},
                "test", object(), object(), None, dds_ready=True)
            degraded = main.Bundle(
                {"plugins": {"accompany": {"enabled": True}}},
                "test", object(), object(), None, dds_ready=False)
            self.assertNotIn("accompany", [t["name"] for t in disabled.tools()])
            self.assertNotIn("accompany", [t["name"] for t in degraded.tools()])

    def test_dockerfile_copies_the_new_module(self):
        """A module that is not copied never reaches the image."""
        text = (ROOT / "Dockerfile").read_text(encoding="utf-8")
        self.assertIn("accompany.py", text)


class _FakeNode:
    """Minimal rclpy node: records publishers and whether it was destroyed."""

    created = []

    def __init__(self, name):
        self.name = name
        self.destroyed = False
        self.publishers = []
        _FakeNode.created.append(self)

    def create_publisher(self, msg_type, topic, qos):
        publisher = types.SimpleNamespace(msg_type=msg_type, topic=topic, qos=qos)
        self.publishers.append(publisher)
        return publisher

    def destroy_node(self):
        self.destroyed = True


class _Executor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class NodeLifecycleTests(unittest.TestCase):
    """A stopped card must not leave a node or a DDS writer behind."""

    @classmethod
    def setUpClass(cls):
        _stubs()
        cls.module = _load("as2w_accompany_node_test", ROOT / "accompany.py")
        cls.Plugin = cls.module.AccompanyPlugin

    def setUp(self):
        self._saved = sys.modules.get("rclpy.node")
        fake = types.ModuleType("rclpy.node")
        fake.Node = _FakeNode
        sys.modules["rclpy.node"] = fake
        _FakeNode.created.clear()

    def tearDown(self):
        if self._saved is None:
            sys.modules.pop("rclpy.node", None)
        else:
            sys.modules["rclpy.node"] = self._saved

    def make(self, executor):
        return self.Plugin({}, "test", executor, proxy=None,
                           state_plugin=_FakeState(_remote()))

    def test_node_is_retained_and_joined_to_the_executor(self):
        executor = _Executor()
        plugin = self.make(executor)
        publisher = plugin._publisher()
        self.assertIsNotNone(publisher)
        self.assertEqual("/test/state/accompany", publisher.topic)
        node = _FakeNode.created[-1]
        self.assertIs(plugin._node, node)
        self.assertEqual([node], executor.nodes,
                         "the node must join the shared executor")

    def test_stop_destroys_the_node_it_created(self):
        plugin = self.make(_Executor())
        plugin._publisher()
        node = _FakeNode.created[-1]
        plugin.stop()
        self.assertTrue(node.destroyed, "stop() must release the node")
        self.assertIsNone(plugin._node)
        self.assertIsNone(plugin._pub)

    def test_restart_builds_a_fresh_node(self):
        plugin = self.make(_Executor())
        plugin._publisher()
        first = _FakeNode.created[-1]
        plugin.stop()
        plugin._publisher()
        second = _FakeNode.created[-1]
        self.assertIsNot(first, second)
        self.assertFalse(second.destroyed)

    def test_stop_without_a_node_is_safe(self):
        plugin = self.make(_Executor())
        plugin.stop()
        self.assertIsNone(plugin._node)
        self.assertIsNone(plugin._pub)


if __name__ == "__main__":
    unittest.main()