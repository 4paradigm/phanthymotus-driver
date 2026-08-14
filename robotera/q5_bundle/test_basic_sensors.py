"""Offline contract checks for the Q5 read-only cards and motion guard."""

from __future__ import annotations

import unittest
import time
import re
import math
import threading
from pathlib import Path

import base_drive
import battery
import control_contract
import diagnostics
import estop
import hand_control
import hand_gesture
import head_control
import hand_state
import joints
import joints_state
import q5_sdk_client
import robot_ready
import simple_action
import arm_control
import system_health


FRESH_JOINT_SNAPSHOT = {
    "fresh": True,
    "age_ms": 12,
    "received_at_ms": 1000,
    "message_timestamp_ms": 999,
    "joint_names": ["hip_joint", "left_hand_index_joint1", "right_hand_index_joint1"],
    "joints": {"hip_joint": 0.1, "left_hand_index_joint1": 0.2, "right_hand_index_joint1": 0.3},
    "velocities": {"hip_joint": 0.0},
    "efforts": {"hip_joint": 1.0},
}


class _Client:
    def snapshot(self):
        return dict(FRESH_JOINT_SNAPSHOT)

    def sensor_snapshot(self, name):
        return {"available": True, "fresh": True, "state": 4} if name == "robot_status" else {}

    def get_lifecycle_state(self):
        return "active"


class _LifecycleFuture:
    def result(self):
        state = type("State", (), {"label": "active"})()
        return type("Response", (), {"current_state": state})()

    def add_done_callback(self, callback):
        callback(self)


class _LifecycleService:
    def service_is_ready(self):
        return True

    def call_async(self, request):
        return _LifecycleFuture()


class Q5BasicSensorTests(unittest.TestCase):
    def test_dockerfile_copies_every_runtime_python_module(self):
        package_dir = Path(__file__).parent
        dockerfile = (package_dir / "Dockerfile").read_text(encoding="utf-8")
        copied_modules = set(
            re.findall(r"^COPY ([A-Za-z0-9_]+\.py) /work/", dockerfile, flags=re.MULTILINE)
        )
        runtime_modules = {
            path.name for path in package_dir.glob("*.py") if not path.name.startswith("test_")
        }

        self.assertSetEqual(copied_modules, runtime_modules)

    def test_deploy_manifest_mounts_q5_interfaces_and_dds_config(self):
        package_dir = Path(__file__).parent
        dockerfile = (package_dir / "Dockerfile").read_text(encoding="utf-8")
        service = (package_dir / "deploy" / "service.yml").read_text(encoding="utf-8")

        self.assertIn("COPY deploy/ /deploy/", dockerfile)
        self.assertIn("network_mode: host", service)
        self.assertIn("/home/nvidia/teleop_client/install:/opt/teleop_client/install:ro", service)
        self.assertIn("/home/nvidia/cyclonedds-orin.xml:/etc/cyclonedds/q5.xml:ro", service)
        self.assertIn("Q5_CYCLONEDDS_URI=file:///etc/cyclonedds/q5.xml", service)

    def test_joint_state_groups_confirmed_hand_names(self):
        data = joints_state.build(FRESH_JOINT_SNAPSHOT)
        self.assertTrue(data["fresh"])
        self.assertEqual(data["joint_count"], 3)
        self.assertEqual(data["groups"]["body"]["joint_names"], ["hip_joint"])
        self.assertEqual(data["groups"]["left_hand"]["joint_names"], ["left_hand_index_joint1"])
        self.assertEqual(data["groups"]["right_hand"]["joint_names"], ["right_hand_index_joint1"])

    def test_joint_cards_preserve_received_but_stale_data_as_unavailable_for_live_use(self):
        stale = dict(FRESH_JOINT_SNAPSHOT, available=True, fresh=False, stale=True, age_ms=5001)
        skeleton = joints.build(stale)
        state = joints_state.build(stale)
        hands = hand_state.build(stale)
        self.assertTrue(skeleton["available"])
        self.assertFalse(skeleton["fresh"])
        self.assertEqual(skeleton["message"], "关节状态消息已过期")
        self.assertTrue(state["available"])
        self.assertEqual(state["message"], "关节状态消息已过期")
        self.assertTrue(hands["available"])
        self.assertEqual(hands["message"], "手部关节状态消息已过期")

    def test_robot_ready_uses_q5_robot_status_as_its_primary_state(self):
        data = robot_ready.build(
            FRESH_JOINT_SNAPSHOT,
            lifecycle_state="active",
            robot_status={"available": True, "fresh": True, "age_ms": 10,
                          "state": 3, "message": "Current state: READY"},
        )
        self.assertTrue(data["available"])
        self.assertTrue(data["ready"])
        self.assertTrue(data["motion_ready"])
        self.assertEqual(data["ready_scope"], "robot_status")
        self.assertTrue(data["motion_manager_lifecycle"]["active"])
        self.assertEqual(data["robot_status"]["state"], 3)
        self.assertEqual(data["robot_status"]["state_label"], "READY")
        self.assertEqual(data["robot_state"], "READY")
        self.assertNotIn("control_authority", data)
        self.assertNotIn("sdk_is_ready", data)

    def test_robot_ready_treats_active_as_a_q5_control_ready_state(self):
        data = robot_ready.build(
            FRESH_JOINT_SNAPSHOT,
            lifecycle_state="active",
            robot_status={"available": True, "fresh": True, "age_ms": 10,
                          "state": 4, "message": "Current state: ACTIVE"},
        )
        self.assertTrue(data["ready"])
        self.assertTrue(data["robot_status"]["ready"])
        self.assertEqual(data["robot_state"], "ACTIVE")
        self.assertTrue(data["motion_manager_lifecycle"]["active"])

    def test_battery_keeps_only_practical_full_charge_readings(self):
        data = battery.build({
            "available": True, "fresh": True, "age_ms": 10, "voltage": 67.2300033569336,
            "temperature": 35, "current": 0, "charge": 15, "capacity": 0,
            "design_capacity": 15, "percentage": 100, "power_supply_status": 0,
            "power_supply_health": 0, "power_supply_technology": 0, "present": False,
        })
        self.assertTrue(data["available"])
        self.assertEqual(data["voltage_v"], 67.2300033569336)
        self.assertEqual(data["percentage"], 100.0)
        self.assertEqual(data["level"], "full")
        self.assertEqual(data["message"], "电量已满")
        self.assertNotIn("charging_state", data)
        self.assertNotIn("vendor_report", data)

    def test_base_drive_requires_a_live_ros_publisher_not_confirmation(self):
        plugin = base_drive.Plugin({}, "test", None, _Client())
        start = plugin.dispatch("start", {})
        move = plugin.dispatch("move", {"linear_x": 0.1, "angular_z": 0.0, "duration_s": 1.0})
        self.assertEqual(start["state"], "ready")
        self.assertEqual(move["code"], "ROS_UNAVAILABLE")

    def test_base_drive_declares_reliable_qos_for_the_verified_q5_controller(self):
        with open(base_drive.__file__, encoding="utf-8") as source:
            self.assertIn("reliability=ReliabilityPolicy.RELIABLE", source.read())

    def test_joint_state_subscription_accepts_vendor_best_effort_stream(self):
        with open(q5_sdk_client.__file__, encoding="utf-8") as source:
            text = source.read()
        self.assertIn("reliability=ReliabilityPolicy.BEST_EFFORT", text)
        self.assertIn("high-rate /joint_states publisher", text)

    def test_optional_degree_joint_feedback_is_normalized_to_radians(self):
        client = q5_sdk_client.Q5SdkClient("degrees")
        stamp = type("Stamp", (), {"sec": 0, "nanosec": 0})()
        header = type("Header", (), {"stamp": stamp, "frame_id": ""})()
        message = type("JointState", (), {
            "name": ["hip_joint", "left_hand_index_joint1"],
            "position": [-14.0, -9.0],
            "velocity": [10.0, 0.0],
            "effort": [1.0, 2.0],
            "header": header,
        })()
        client._on_joint_state(message)
        snapshot = client.snapshot()
        self.assertAlmostEqual(snapshot["joints"]["hip_joint"], math.radians(-14.0))
        self.assertAlmostEqual(snapshot["velocities"]["hip_joint"], math.radians(10.0))
        self.assertEqual(snapshot["position_unit"], "rad")
        self.assertEqual(snapshot["source_position_unit"], "deg")

    def test_default_joint_feedback_uses_the_ros_radian_contract(self):
        client = q5_sdk_client.Q5SdkClient()
        stamp = type("Stamp", (), {"sec": 0, "nanosec": 0})()
        header = type("Header", (), {"stamp": stamp, "frame_id": ""})()
        message = type("JointState", (), {
            "name": ["hip_joint"], "position": [-0.2], "velocity": [0.1],
            "effort": [1.0], "header": header,
        })()
        client._on_joint_state(message)
        snapshot = client.snapshot()
        self.assertEqual(snapshot["joints"]["hip_joint"], -0.2)
        self.assertEqual(snapshot["velocities"]["hip_joint"], 0.1)
        self.assertEqual(snapshot["source_position_unit"], "rad")

    def test_base_drive_direction_actions_normalize_to_guarded_velocity(self):
        plugin = base_drive.Plugin({}, "test", None, _Client())
        forward = plugin._directional_args("forward", {"speed_mps": 0.1, "duration_s": 0.5})
        right = plugin._directional_args("turn_right", {"turn_speed_radps": 0.2, "duration_s": 0.5})
        self.assertEqual((forward["linear_x"], forward["angular_z"]), (0.1, 0.0))
        self.assertEqual((right["linear_x"], right["angular_z"]), (0.0, -0.2))

    def test_lifecycle_poll_reads_active_without_side_effects(self):
        client = q5_sdk_client.Q5SdkClient()
        client._lifecycle_client = _LifecycleService()
        client._lifecycle_request_type = object
        client._refresh_lifecycle_state()
        self.assertEqual(client.get_lifecycle_state(), "active")
        self.assertEqual(client._lifecycle_source, "/motion_manager/get_state")

    def test_physical_controls_require_a_fresh_q5_ready_or_active_fsm(self):
        class FsmClient:
            def sensor_snapshot(self, name):
                if name != "robot_status":
                    raise AssertionError(name)
                return {"available": True, "fresh": True, "state": 4}

        allowed, status = control_contract.q5_is_control_ready(FsmClient())
        self.assertTrue(allowed)
        self.assertEqual(status["state_label"], "ACTIVE")

        class ReadyClient:
            def sensor_snapshot(self, name):
                if name != "robot_status":
                    raise AssertionError(name)
                return {"available": True, "fresh": True, "state": 3}

        allowed, status = control_contract.q5_is_control_ready(ReadyClient())
        self.assertTrue(allowed)
        self.assertEqual(status["state_label"], "READY")
        self.assertEqual(status["control_ready_states"], [3, 4])

        class InitClient:
            def sensor_snapshot(self, name):
                return {"available": True, "fresh": True, "state": 0}

        allowed, status = control_contract.q5_is_control_ready(InitClient())
        self.assertFalse(allowed)
        self.assertEqual(status["state_label"], "INIT")

    def test_hand_control_does_not_reject_a_fresh_ready_fsm(self):
        class ReadyClient(_Client):
            def sensor_snapshot(self, name):
                if name == "robot_status":
                    return {"available": True, "fresh": True, "state": 3}
                return {}

            def snapshot(self):
                snapshot = dict(FRESH_JOINT_SNAPSHOT)
                snapshot["joints"] = {name: 0.0 for name in hand_control.HAND_JOINTS}
                return snapshot

        plugin = hand_control.Plugin({}, "test", None, ReadyClient())
        plugin._router.status = lambda: {"ros_publisher_available": True}
        result = plugin._allowed({})
        self.assertNotEqual(result.get("code"), "Q5_FSM_NOT_READY")
        self.assertTrue(result["q5_fsm"]["state"] == 3)

    def test_verified_status_cards_preserve_source_and_freshness(self):
        now = int(time.time() * 1000)
        data = system_health.build({
            "motion": {"payload": {"status": 1}, "received_at_ms": now},
            "robot": {"payload": {"state": 4, "message": "Current state: ACTIVE"}, "received_at_ms": now},
            "temperature": {"payload": {"body": 35.0}, "received_at_ms": now},
            "faults": {"payload": {"faults": []}, "received_at_ms": now},
        }, now_ms=now)
        self.assertTrue(data["motion"]["fresh"])
        self.assertTrue(data["robot"]["fresh"])
        self.assertEqual(data["robot"]["state_label"], "ACTIVE")
        self.assertTrue(data["summary"]["robot_active"])
        self.assertTrue(data["summary"]["all_required_sources_fresh"])
        self.assertEqual(data["message"], "必需健康遥测正常")

    def test_event_driven_motion_does_not_make_required_health_stale(self):
        now = int(time.time() * 1000)
        data = system_health.build({
            "robot": {"payload": {"state": 4}, "received_at_ms": now},
            "temperature": {"payload": {"name": ["neck/motor"], "temperature": [42.0]},
                            "received_at_ms": now},
        }, publisher_counts={"motion": 1, "robot": 1, "temperature": 1, "faults": 1}, now_ms=now)
        self.assertEqual(data["motion"]["report_state"], "awaiting_event")
        self.assertTrue(data["summary"]["all_required_sources_fresh"])
        self.assertEqual(data["temperature_summary"]["maximum_celsius"], 42.0)

    def test_system_health_accepts_q5_motion_state_field(self):
        plugin = system_health.Plugin.__new__(system_health.Plugin)
        plugin._snapshots = {}
        message = type("MotionStatus", (), {"state": 4, "msg": "Current state: ACTIVE"})()
        plugin._on_motion(message)
        self.assertEqual(plugin._snapshots["motion"]["payload"], {
            "state": 4, "status": 4, "message": "Current state: ACTIVE",
        })

    def test_generic_sensor_cards_report_missing_messages(self):
        health = system_health.build({}, now_ms=int(time.time() * 1000))
        self.assertEqual(health["summary"]["available_sources"], [])
        diagnostics_data = diagnostics.build(None, None)
        self.assertFalse(diagnostics_data["available"])
        self.assertFalse(diagnostics_data["fresh"])
        self.assertEqual(diagnostics_data["source_state"], "unknown")

    def test_control_cards_do_not_require_confirmation(self):
        for module, action, args in (
            (simple_action, "run", {"action_name": "lift_up"}),
            (arm_control, "left_elbow_pitch_joint", {"left_elbow_pitch_rad": -0.5}),
            (hand_control, "set", {"targets": [{"joint_name": hand_control.HAND_JOINTS[0], "position_rad": 0.1}]}),
            (hand_gesture, "light_grip", {}),
            (head_control, "neck_yaw", {"neck_yaw_rad": 0.1}),
        ):
            plugin = module.Plugin({}, "test", None, _Client())
            result = plugin.dispatch(action, args)
            expected_code = "ARM_CONTROL_DISABLED" if module is arm_control else "ROS_UNAVAILABLE"
            self.assertEqual(result["code"], expected_code, module.CARD)

    def test_control_schemas_expose_action_specific_frontend_forms(self):
        cards = (base_drive, simple_action, arm_control, hand_control, hand_gesture, head_control)
        for module in cards:
            schema = module.Plugin({}, "test", None, _Client()).get_tool()["inputSchema"]
            self.assertIn("x-action-params", schema, module.CARD)
            self.assertNotIn("x-is-dangerous", schema, module.CARD)
            self.assertNotIn("confirm", schema["properties"], module.CARD)
            self.assertEqual(set(schema["properties"]["action"]["enum"]),
                             set(schema["x-action-params"]), module.CARD)
            for action, definition in schema["x-action-params"].items():
                self.assertNotIn("confirm", definition["params"], module.CARD)
                self.assertIn(action, {item["const"] for item in schema["properties"]["action"]["oneOf"]}, module.CARD)
                for parameter in definition["params"]:
                    self.assertIn(parameter, schema["properties"], module.CARD)
        hand_schema = hand_control.Plugin({}, "test", None, _Client()).get_tool()["inputSchema"]
        self.assertEqual(hand_schema["properties"]["targets"]["x-widget"], "json")
        self.assertNotIn("confirm", hand_schema["x-action-params"]["set"]["params"])
        self.assertNotIn("duration_s", hand_schema["properties"])
        self.assertTrue({"open_hand", "close_hand", "set_hand", "set_finger", "cancel"}.issubset(
            hand_schema["x-action-params"]))
        self.assertIn("thumb", hand_schema["properties"]["finger"]["enum"])
        self.assertIn("rotation_rad", hand_schema["properties"])
        gesture_schema = hand_gesture.Plugin({}, "test", None, _Client()).get_tool()["inputSchema"]
        self.assertNotIn("gesture", gesture_schema["properties"])
        self.assertNotIn("open", gesture_schema["properties"]["action"]["enum"])
        for module in (simple_action, arm_control, hand_control, hand_gesture, head_control):
            schema = module.Plugin({}, "test", None, _Client()).get_tool()["inputSchema"]
            self.assertNotIn("duration_s", schema["properties"], module.CARD)
        base_schema = base_drive.Plugin({}, "test", None, _Client()).get_tool()["inputSchema"]
        self.assertIn("duration_s", base_schema["properties"])

    def test_bundle_preserves_direct_control_arguments(self):
        unchanged = control_contract.prepare_call_args(
            {"inputSchema": {"x-is-dangerous": True}}, {"action": "move"},
        )
        normal = control_contract.prepare_call_args({"inputSchema": {}}, {"action": "info"})
        self.assertEqual(unchanged, {"action": "move"})
        self.assertEqual(normal, {"action": "info"})

    def test_hand_control_builds_whole_hand_and_single_finger_targets(self):
        snapshot = dict(FRESH_JOINT_SNAPSHOT)
        snapshot["joints"] = {name: 0.1 for name in hand_control.HAND_JOINTS}

        class HandClient:
            def snapshot(self):
                return dict(snapshot)

        plugin = hand_control.Plugin({}, "test", None, HandClient())
        finger = plugin._profile_targets("set_finger", {
            "side": "left", "finger": "index", "curl_rad": 0.2,
        })
        self.assertEqual(set(finger["targets"]), {"left_hand_index_joint1"})
        self.assertEqual(set(finger["targets"].values()), {0.2})
        thumb = plugin._profile_targets("set_finger", {
            "side": "left", "finger": "thumb", "curl_rad": 0.2, "rotation_rad": 0.3,
        })
        self.assertEqual(thumb["targets"], {
            "left_hand_thumb_bend_joint": 0.2, "left_hand_thumb_rota_joint1": 0.3,
        })
        closing = plugin._profile_targets("close_hand", {"side": "right"})
        self.assertEqual(len(closing["targets"]), 6)
        self.assertTrue(all(abs(value - 1.0) < 1e-9 for value in closing["targets"].values()))

    def test_arm_and_head_share_one_body_command_router(self):
        client = _Client()
        arm = arm_control.Plugin({"hardware_enable": True}, "test", None, client)
        head = head_control.Plugin({}, "test", None, client)
        self.assertIs(arm._router, head._router)
        self.assertEqual(arm._router.status()["topic"], "/wr1_controller/commands")

    def test_arm_control_is_hard_disabled_without_explicit_hardware_enable(self):
        plugin = arm_control.Plugin({"enabled": True, "hardware_enable": False}, "test", None, _Client())
        self.assertIsNone(plugin._router)
        self.assertEqual(plugin.dispatch("start", {})["state"], "disabled")
        result = plugin.dispatch("left_elbow_pitch_joint", {"left_elbow_pitch_rad": -0.5})
        self.assertEqual(result["code"], "ARM_CONTROL_DISABLED")

    def test_head_completion_holds_target_not_delayed_feedback(self):
        plugin = head_control.Plugin({"hold_repetitions": 1, "publish_rate_hz": 1000}, "test", None, _Client())
        published = []
        plugin._router.publish = lambda positions: published.append(dict(positions)) or True
        plugin._run(threading.Event(), "neck_yaw_joint", 0.0, 0.05, 0.0)
        self.assertEqual(published[-1], {"neck_yaw_joint": 0.05})

    def test_arm_completion_holds_target_not_delayed_feedback(self):
        plugin = arm_control.Plugin({"hardware_enable": True, "hold_repetitions": 1, "publish_rate_hz": 1000,
                                     "settle_timeout_s": 0.001}, "test", None, _Client())
        published = []
        plugin._router.publish = lambda positions: published.append(dict(positions)) or True
        plugin._run_move(threading.Event(), "left_elbow_pitch_joint", 0.0, 0.03, 0.0)
        self.assertEqual(published[-1], {"left_elbow_pitch_joint": 0.03})

    def test_absolute_arm_and_head_targets_use_urdf_limits(self):
        class ActiveClient(_Client):
            def snapshot(self):
                snapshot = dict(FRESH_JOINT_SNAPSHOT)
                snapshot["joints"] = {
                    **snapshot["joints"],
                    "left_elbow_pitch_joint": -0.5,
                    "neck_pitch_joint": 0.0,
                }
                return snapshot

            def sensor_snapshot(self, name):
                if name != "robot_status":
                    return {}
                return {"available": True, "fresh": True, "state": 4}

        client = ActiveClient()
        arm = arm_control.Plugin({"hardware_enable": True}, "test", None, client)
        head = head_control.Plugin({}, "test", None, client)
        arm._router.status = lambda: {"ros_publisher_available": True}
        head._router.status = lambda: {"ros_publisher_available": True}
        arm._router.acquire = lambda owner: True
        head._router.acquire = lambda owner: True

        arm_command = arm._validate_move("left_elbow_pitch_joint", -1.0)
        self.assertEqual(arm_command["code"], "TARGET_DELTA_EXCEEDED")
        arm_command = arm._validate_move("left_elbow_pitch_joint", -0.54)
        self.assertEqual(arm_command[2], -0.54)
        arm_rejected = arm._validate_move("left_elbow_pitch_joint", 0.10)
        self.assertEqual(arm_rejected["code"], "LIMIT_EXCEEDED")

        head._allowed = lambda args, name: {"ros_publisher_available": True}
        rejected = head.dispatch("neck_pitch", {"neck_pitch_rad": 0.71})
        self.assertEqual(rejected["code"], "LIMIT_EXCEEDED")
        schema = head.get_tool()["inputSchema"]
        self.assertIn("neck_pitch_rad", schema["properties"])
        self.assertNotIn("delta_rad", schema["properties"])
        self.assertEqual(schema["properties"]["neck_pitch_rad"]["maximum"], 0.7)
        self.assertEqual(schema["properties"]["action"]["enum"], ["start", "neck_yaw", "neck_pitch", "cancel", "info"])
        self.assertNotIn("target_position_rad", schema["properties"])
        arm_schema = arm.get_tool()["inputSchema"]
        arm_limit = arm_schema["properties"]["left_elbow_pitch_rad"]
        self.assertIn("范围[", arm_limit["description"])
        head_limit = schema["properties"]["neck_pitch_rad"]
        self.assertIn("范围[-0.26,0.70]rad", head_limit["description"])

    def test_arm_refuses_external_body_command_publishers(self):
        class ActiveClient(_Client):
            def snapshot(self):
                return {
                    **FRESH_JOINT_SNAPSHOT,
                    "joints": {"left_elbow_pitch_joint": -0.5, "neck_yaw_joint": 0.0},
                }

            def sensor_snapshot(self, name):
                return {"available": True, "fresh": True, "state": 3} if name == "robot_status" else {}

        client = ActiveClient()
        arm = arm_control.Plugin({"hardware_enable": True}, "test", None, client)
        head = head_control.Plugin({}, "test", None, client)
        conflict = {"ros_publisher_available": True, "other_publishers": [{"node_name": "mpc_policy_node"}]}
        arm._router.status = lambda: dict(conflict)
        head._router.status = lambda: dict(conflict)
        arm_command = arm._validate_move("left_elbow_pitch_joint", -1.0)
        self.assertEqual(arm_command["code"], "BODY_COMMAND_CONFLICT")
        head_status = head._allowed({}, "neck_yaw_joint")
        self.assertEqual(head_status["other_publishers"], conflict["other_publishers"])

    def test_default_control_set_uses_direct_interfaces(self):
        config = Path("config.yaml").read_text(encoding="utf-8")
        self.assertIn("simple_action:\n    # Vendor action service", config)
        self.assertIn("    enabled: false\n  arm_control:", config)
        for card in ("base_drive", "arm_control", "hand_control", "hand_gesture", "head_control"):
            match = re.search(rf"^  {card}:$(.*?)(?=^  \w|\Z)", config, re.MULTILINE | re.DOTALL)
            self.assertIsNotNone(match, card)
            self.assertIn("enabled: true", match.group(1), card)

    def test_xhand_lite_preset_has_the_verified_joint_layout(self):
        self.assertIn("light_grip", hand_gesture.PRESETS)
        self.assertEqual(set(hand_gesture.PRESETS["closed_fist"]["left"].values()), {1.0})
        self.assertTrue({"open_hand", "victory", "thumbs_up", "ok_sign", "three", "rock"}.issubset(hand_gesture.PRESETS))
        self.assertEqual(len(hand_control.HAND_JOINTS), 12)

    def test_hand_state_uses_the_verified_xhand_lite_joint_layout(self):
        positions = {name: 0.1 for name in hand_state.LEFT_JOINTS + hand_state.RIGHT_JOINTS}
        data = hand_state.build({"fresh": True, "joints": positions, "velocities": {}, "efforts": {}})
        self.assertTrue(data["left"]["complete"])
        self.assertTrue(data["right"]["complete"])

    def test_diagnostics_and_estop_state_cards_preserve_safe_state(self):
        now = int(time.time() * 1000)
        diagnostic_waiting = diagnostics.build(None, None, publisher_count=1)
        emergency = estop.build(7, "E_STOP", now)
        normal = estop.build(4, "ACTIVE", now)
        self.assertFalse(diagnostic_waiting["available"])
        self.assertTrue(diagnostic_waiting["publisher_connected"])
        self.assertEqual(diagnostic_waiting["source_state"], "awaiting_diagnostic_event")
        self.assertTrue(emergency["emergency_stop"])
        self.assertFalse(normal["emergency_stop"])
        self.assertIsNone(normal["message"])
        self.assertNotIn("physical_estop_state", normal)

    def test_diagnostics_do_not_hide_stale_payloads_as_waiting(self):
        diagnostic = diagnostics.build({"status": []}, 1, publisher_count=1)
        self.assertTrue(diagnostic["available"])
        self.assertFalse(diagnostic["fresh"])
        self.assertEqual(diagnostic["source_state"], "stale")
        self.assertEqual(diagnostic["message"], "诊断消息已过期")


if __name__ == "__main__":
    unittest.main()
