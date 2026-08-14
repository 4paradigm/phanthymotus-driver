from __future__ import annotations

import sys
import types
import unittest


def _install_driver_import_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_qos = types.ModuleType("rclpy.qos")

    class Node:
        pass

    class QoSProfile:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    class EnumValue:
        BEST_EFFORT = "best_effort"
        RELIABLE = "reliable"
        KEEP_LAST = "keep_last"
        VOLATILE = "volatile"

    rclpy_node.Node = Node
    rclpy_qos.QoSProfile = QoSProfile
    rclpy_qos.ReliabilityPolicy = EnumValue
    rclpy_qos.HistoryPolicy = EnumValue
    rclpy_qos.DurabilityPolicy = EnumValue

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Header = type("Header", (), {})
    std_msgs_msg.String = type("String", (), {})

    audio_msgs = types.ModuleType("audio_msgs")
    audio_msgs_msg = types.ModuleType("audio_msgs.msg")
    audio_msgs_msg.AudioChunk = type("AudioChunk", (), {})

    unitree_sdk = types.ModuleType("unitree_sdk2py")
    unitree_g1 = types.ModuleType("unitree_sdk2py.g1")
    unitree_audio = types.ModuleType("unitree_sdk2py.g1.audio")
    unitree_audio_client = types.ModuleType("unitree_sdk2py.g1.audio.g1_audio_client")
    unitree_audio_client.AudioClient = type("AudioClient", (), {})

    pointcloud_utils = types.ModuleType("pointcloud_utils")
    pointcloud_utils.gravity_align_inplace = lambda *args, **kwargs: None
    numpy = types.ModuleType("numpy")

    modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "audio_msgs": audio_msgs,
        "audio_msgs.msg": audio_msgs_msg,
        "unitree_sdk2py": unitree_sdk,
        "unitree_sdk2py.g1": unitree_g1,
        "unitree_sdk2py.g1.audio": unitree_audio,
        "unitree_sdk2py.g1.audio.g1_audio_client": unitree_audio_client,
        "pointcloud_utils": pointcloud_utils,
        "numpy": numpy,
    }
    for name, module in modules.items():
        sys.modules.setdefault(name, module)


_install_driver_import_stubs()

from device import LocoPlugin  # noqa: E402
from velocity_proposal import resolve_optional_expected_nav_id  # noqa: E402


class FakeClient:
    def __init__(self):
        self.stop_count = 0
        self.stop_ret = 0
        self.stop_error = None
        self.fsm_ids = []

    def StopMove(self):
        self.stop_count += 1
        if self.stop_error is not None:
            raise self.stop_error
        return self.stop_ret

    def SetFsmId(self, fsm_id):
        self.fsm_ids.append(fsm_id)
        return 0


class FakeSmartMotion:
    def __init__(self):
        self.bound_topic = ""
        self.expected_nav_id = ""
        self.last_reason = None
        self.proposal_execution = {
            "received": 0,
            "accepted": 0,
            "rejected": 0,
            "applied": 0,
            "last_rejection_reason": None,
            "last_set_velocity_ret": None,
        }
        self.bind_result = None
        self.unbind_reasons = []
        self.unbind_result = {
            "state": "idle",
            "connected": False,
            "stop_confirmed": True,
        }

    def bind_velocity_proposal(self, topic, expected_nav_id):
        if self.bind_result is not None:
            return dict(self.bind_result)
        expected_nav_id = resolve_optional_expected_nav_id(
            {"expected_nav_id": expected_nav_id}
        )
        self.bound_topic = topic
        self.expected_nav_id = expected_nav_id or ""
        awaiting_nav_id = expected_nav_id is None
        return {
            "state": "connected",
            "connected": True,
            "armed": not awaiting_nav_id,
            "awaiting_nav_id": awaiting_nav_id,
            "nav_id_binding_mode": (
                "first_valid_proposal"
                if awaiting_nav_id
                else "control_plane"
            ),
            "topic": topic,
            "expected_nav_id": expected_nav_id,
            "active_nav_id": expected_nav_id,
        }

    def unbind_velocity_proposal(self, reason):
        self.unbind_reasons.append(reason)
        self.last_reason = reason
        result = dict(self.unbind_result)
        if not result.get("connected"):
            self.bound_topic = ""
            self.expected_nav_id = ""
        return result

    def get_velocity_proposal_status(self):
        return {
            "connected": bool(self.bound_topic),
            "armed": bool(self.bound_topic and self.expected_nav_id),
            "awaiting_nav_id": bool(
                self.bound_topic and not self.expected_nav_id
            ),
            "topic": self.bound_topic or None,
            "expected_nav_id": self.expected_nav_id or None,
            "active_nav_id": self.expected_nav_id or None,
            "nav_id_binding_mode": (
                "control_plane"
                if self.expected_nav_id
                else "first_valid_proposal" if self.bound_topic else None
            ),
            "last_reason": self.last_reason,
            "proposal_execution": dict(self.proposal_execution),
        }


class LocoTopicLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.client = FakeClient()
        self.smart_motion = FakeSmartMotion()
        self.plugin = LocoPlugin(
            {}, "ubuntu", executor=None, loco_client=self.client,
            smart_motion=self.smart_motion,
        )
        self.topic = "/ubuntu/navigation/nav2/velocity_proposal"
        self.nav_id = "nav-001"

    def test_tool_exposes_velocity_proposal_input(self):
        self.assertEqual(self.plugin.STOP_PRIORITY, 0)
        tool = self.plugin.get_tools()[0]
        port = tool["topic_in"]
        self.assertEqual(len(port), 1)
        self.assertEqual(port[0]["port"], "velocity_proposal")
        self.assertEqual(port[0]["topic"], self.topic)
        self.assertEqual(
            tool["inputSchema"]["properties"]["action"]["enum"],
            [
                "move", "stop_move", "set_stand_height", "get_fsm_id",
                "get_fsm_mode", "get_balance_mode", "get_swing_height",
                "get_stand_height", "get_phase", "wave_hand", "shake_hand",
            ],
        )

    def test_start_info_stop_owns_real_subscription_lifecycle(self):
        started = self.plugin.dispatch(
            "start",
            {"input_topic": self.topic, "expected_nav_id": self.nav_id},
        )
        self.assertTrue(started["connected"])
        self.assertEqual(started["state"], "ready")
        self.assertEqual(self.smart_motion.bound_topic, self.topic)
        self.assertEqual(self.smart_motion.expected_nav_id, self.nav_id)

        info = self.plugin.dispatch("info", {"_tool_name": "loco"})
        self.assertTrue(info["connected"])
        self.assertTrue(info["armed"])

        stopped = self.plugin.dispatch("stop", {})
        self.assertFalse(stopped["connected"])
        self.assertTrue(stopped["stop_confirmed"])
        self.assertEqual(self.smart_motion.unbind_reasons, ["canvas_stop"])

    def test_info_retains_proposal_execution_diagnostics_after_unbind(self):
        self.smart_motion.proposal_execution.update({
            "received": 3,
            "accepted": 2,
            "rejected": 1,
            "applied": 1,
            "last_rejection_reason": "set_velocity_failed",
            "last_set_velocity_ret": 3104,
        })
        self.plugin.dispatch(
            "start",
            {"input_topic": self.topic, "expected_nav_id": self.nav_id},
        )

        self.plugin.dispatch("stop", {})
        info = self.plugin.dispatch("info", {"_tool_name": "loco"})

        self.assertEqual(info["last_reason"], "canvas_stop")
        self.assertEqual(
            info["proposal_execution"],
            self.smart_motion.proposal_execution,
        )

    def test_start_accepts_exactly_one_input_topics_entry(self):
        started = self.plugin.dispatch(
            "start",
            {"input_topics": [self.topic], "expected_nav_id": self.nav_id},
        )
        self.assertTrue(started["connected"])
        self.assertEqual(self.smart_motion.bound_topic, self.topic)

    def test_start_without_nav_id_is_ready_for_legacy_core(self):
        result = self.plugin.dispatch("start", {"input_topic": self.topic})

        self.assertEqual(result["state"], "ready")
        self.assertTrue(result["connected"])
        self.assertFalse(result["armed"])
        self.assertTrue(result["awaiting_nav_id"])
        self.assertEqual(
            result["nav_id_binding_mode"],
            "first_valid_proposal",
        )
        self.assertEqual(self.smart_motion.bound_topic, self.topic)
        self.assertEqual(self.smart_motion.expected_nav_id, "")
        self.assertEqual(self.client.stop_count, 0)

    def test_fake_smart_motion_matches_real_optional_nav_id_contract(self):
        empty = self.smart_motion.bind_velocity_proposal(self.topic, "")

        self.assertTrue(empty["connected"])
        self.assertFalse(empty["armed"])
        self.assertTrue(empty["awaiting_nav_id"])
        with self.assertRaises(ValueError):
            self.smart_motion.bind_velocity_proposal(self.topic, 123)

    def test_start_rejects_invalid_explicit_nav_id(self):
        result = self.plugin.dispatch(
            "start",
            {"input_topic": self.topic, "expected_nav_id": 123},
        )

        self.assertEqual(result["state"], "error")
        self.assertFalse(result["connected"])
        self.assertEqual(result["error"], "invalid_expected_nav_id")
        self.assertEqual(self.client.stop_count, 1)

    def test_start_preserves_stop_confirmation_diagnostics_on_failure(self):
        diagnostics = {
            "stop_move_ret": 0,
            "stop_move_error": None,
            "confirmation_started_monotonic": 123.0,
            "last_odometry_monotonic": 123.4,
            "last_odometry_age_ms": 100,
            "last_odometry_velocity": {"x": 0.04, "y": 0.0, "yaw": 0.0},
            "odometry_callback_count": 42,
            "odometry_callbacks_since_confirmation": 1,
            "confirmation_timed_out": True,
        }
        self.smart_motion.bind_result = {
            "error": "StopMove/odometry stop was not confirmed before proposal bind",
            "connected": False,
            "armed": False,
            "stop_confirmed": False,
            "stop_move_ret": 0,
            "stop_move_error": None,
            "stop_confirmation": diagnostics,
        }

        result = self.plugin.dispatch(
            "start",
            {"input_topic": self.topic, "expected_nav_id": self.nav_id},
        )

        self.assertEqual(result["state"], "error")
        self.assertFalse(result["connected"])
        self.assertEqual(result["stop_move_ret"], 0)
        self.assertEqual(result["stop_confirmation"], diagnostics)
        self.assertEqual(result["fallback_stop_ret"], 0)
        self.assertIsNone(result["fallback_stop_error"])
        self.assertEqual(self.client.stop_count, 1)

    def test_start_preserves_child_diagnostics_when_fallback_stop_raises(self):
        diagnostics = {
            "stop_move_ret": 3104,
            "stop_move_error": None,
            "confirmation_timed_out": False,
        }
        self.smart_motion.bind_result = {
            "error": "StopMove/odometry stop was not confirmed before proposal bind",
            "connected": False,
            "armed": False,
            "stop_confirmed": False,
            "stop_move_ret": 3104,
            "stop_move_error": None,
            "stop_confirmation": diagnostics,
        }
        self.client.stop_error = RuntimeError("fallback unavailable")

        result = self.plugin.dispatch(
            "start",
            {"input_topic": self.topic, "expected_nav_id": self.nav_id},
        )

        self.assertEqual(result["stop_confirmation"], diagnostics)
        self.assertIsNone(result["fallback_stop_ret"])
        self.assertEqual(result["fallback_stop_error"], "fallback unavailable")
        self.assertEqual(self.client.stop_count, 1)

    def test_other_tools_do_not_bind_or_unbind_loco_topic(self):
        self.smart_motion.bound_topic = self.topic

        started = self.plugin.dispatch(
            "start", {"_tool_name": "motion_events"}
        )
        stopped = self.plugin.dispatch(
            "stop", {"_tool_name": "switch_mode"}
        )

        self.assertEqual(started, {"state": "running"})
        self.assertEqual(stopped, {"state": "idle"})
        self.assertEqual(self.smart_motion.bound_topic, self.topic)
        self.assertEqual(self.smart_motion.unbind_reasons, [])
        self.assertEqual(self.client.stop_count, 0)

    def test_wrong_topic_fails_closed_without_binding(self):
        self.smart_motion.bound_topic = self.topic
        result = self.plugin.dispatch(
            "start",
            {
                "input_topic": "/ubuntu/navigation/nav2/cmd_vel_shadow",
                "expected_nav_id": self.nav_id,
            },
        )
        self.assertEqual(result["state"], "error")
        self.assertFalse(result["connected"])
        self.assertEqual(self.client.stop_count, 1)
        self.assertEqual(self.smart_motion.bound_topic, "")
        self.assertEqual(self.smart_motion.unbind_reasons, ["proposal_bind_rejected"])

    def test_missing_safety_harness_cannot_execute_topic(self):
        plugin = LocoPlugin({}, "ubuntu", None, self.client, smart_motion=None)
        result = plugin.dispatch(
            "start",
            {"input_topic": self.topic, "expected_nav_id": self.nav_id},
        )
        self.assertEqual(result["state"], "error")
        self.assertFalse(result["connected"])
        self.assertEqual(self.client.stop_count, 1)

    def test_unconfirmed_stop_falls_back_and_blocks_mode_switch(self):
        self.smart_motion.bound_topic = self.topic
        self.smart_motion.unbind_result = {
            "state": "error",
            "connected": True,
            "stop_confirmed": False,
            "error": "fresh zero-odometry stop was not confirmed",
        }

        result = self.plugin.dispatch("switch_mode_expert", {"fsm_id": 1})

        self.assertEqual(result["state"], "error")
        self.assertEqual(self.client.stop_count, 1)
        self.assertEqual(self.client.fsm_ids, [])


if __name__ == "__main__":
    unittest.main()
