"""Unit tests plus opt-in DDS integration on domain 93 (never robot domain 0)."""
import os
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from preset_postures import MOTIONS, PRESETS, PresetController, PresetPosturesCard
from preset_robot import (RobotTransport, upright_quaternion, select_posture, reconcile_action,
                          FULL_IMU, TORSO_ORIENTATION, MC_STATE)


class FakeRobot:
    def __init__(self, mode=1002, status=100, ready=True, upright=True):
        self.mode, self.status, self.ready = mode, status, ready
        self.is_upright = upright
        self.calls = []
        self.last_response = {"code": 0, "state": 1, "reason": 0, "task_id": 123}
        self.cancelled = False

    def check(self):
        if self.cancelled:
            raise RuntimeError("cancelled")

    def query(self):
        self.check()
        return {"value": self.mode, "status": self.status, "switch_ready": self.ready,
                "source": "sdk_rpc", "description": "mode"}

    def services(self):
        return {"GetMcAction": True}

    def upright(self):
        return self.is_upright

    def posture(self):
        return {"upright": self.is_upright, "source": TORSO_ORIENTATION, "age_s": 0}

    def pause(self, _):
        self.check()

    def set_mode(self, mode):
        self.calls.append(("mode", mode))
        self.mode = mode
        return self.last_response

    def preset(self, number, interrupt):
        self.calls.append(("preset", number, interrupt))
        return self.last_response

    def motion(self, name):
        self.calls.append(("motion", name))
        return self.last_response


class PresetTests(unittest.TestCase):
    def test_deployment_context_contains_every_docker_copy_source(self):
        import importlib.util
        from types import SimpleNamespace
        path = Path(__file__).resolve().parents[1] / "deploy/q1.py"
        spec = importlib.util.spec_from_file_location("q1_deploy", path)
        deploy = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(deploy)

        def inspect_context(*args):
            context = Path(args[-1])
            for line in (context / "Dockerfile").read_text().splitlines():
                if line.startswith("COPY "):
                    for source in line.split()[1:-1]:
                        self.assertTrue((context / source).exists(), source)

        with patch.object(deploy, "command", side_effect=inspect_context) as build:
            deploy.build(SimpleNamespace(base_image="test-base", image="test-image"))
            build.assert_called_once()

    def test_factory_catalog_and_bumi_card_contract(self):
        card = PresetPosturesCard("test", None, 93)
        definition = card.get_tool()
        self.assertEqual((definition["name"], definition["type"], definition["multiInstance"]),
                         ("preset_postures", "actuator", False))
        schema = definition["inputSchema"]
        self.assertEqual(len(PRESETS), 16)
        self.assertEqual(len(MOTIONS), 16)
        self.assertEqual(set(schema["properties"]["action"]["enum"]),
                         set(PRESETS) | set(MOTIONS) | {"reset", "status"})
        self.assertTrue(all(entry["params"] == [] for entry in schema["x-action-params"].values()))
        self.assertNotIn("x-completion", schema)  # No fabricated physical completion.
        self.assertIsNone(card.dispatch("invented_motion", {}))

    def test_all_documented_presets_prepare_wbc_then_send_once(self):
        for action, (number, _) in PRESETS.items():
            with self.subTest(action=action):
                robot = FakeRobot()
                result = PresetController(robot).execute(action)
                self.assertEqual(robot.calls, [("mode", 1005), ("preset", number, False)])
                self.assertEqual(result["state"], "accepted")
                self.assertFalse(result["execution_confirmed"])

    def test_all_documented_whole_body_motions_prepare_walk(self):
        for action, (name, _) in MOTIONS.items():
            with self.subTest(action=action):
                robot = FakeRobot(1005)
                result = PresetController(robot).execute(action)
                self.assertEqual(robot.calls, [("mode", 1002), ("motion", name)])
                self.assertEqual(result["task_id"], 123)

    def test_no_commands_from_protection_or_nonstanding_modes(self):
        for mode in (1, 2, 201, 1001, 1003, 1007, 1009, 1010, 1014):
            robot = FakeRobot(mode)
            self.assertEqual(PresetController(robot).execute("wave")["state"], "rejected")
            self.assertEqual(robot.calls, [])

    def test_stale_pose_transition_or_not_ready_refuses(self):
        for robot in (FakeRobot(upright=False), FakeRobot(status=200), FakeRobot(ready=False)):
            self.assertEqual(PresetController(robot).execute("wave")["state"], "rejected")
            self.assertEqual(robot.calls, [])

    def test_info_reports_not_ready_without_commanding_hardware(self):
        for robot in (FakeRobot(ready=False), FakeRobot(upright=False), FakeRobot(2)):
            result = PresetController(robot).execute("info")
            self.assertEqual(result["state"], "not_ready")
            self.assertEqual(robot.calls, [])

    def test_missing_pose_is_unknown_not_nonstanding(self):
        result = PresetController(FakeRobot(upright=None)).execute("info")
        self.assertIsNone(result["standing"])
        self.assertEqual(result["state"], "not_ready")
        self.assertIn("数据缺失", result["message"])

    def test_high_level_pose_fallback_and_freshness(self):
        self.assertEqual(select_posture({TORSO_ORIENTATION: (10, True)}, 10.1)["upright"], True)
        self.assertIsNone(select_posture({TORSO_ORIENTATION: (9, True)}, 10)["upright"])
        self.assertIsNone(select_posture({TORSO_ORIENTATION: (11, True)}, 10)["upright"])
        samples = {FULL_IMU: (10, None), TORSO_ORIENTATION: (10, True)}
        self.assertEqual(select_posture(samples, 10)["source"], TORSO_ORIENTATION)
        samples[FULL_IMU] = (10, False)
        self.assertFalse(select_posture(samples, 10)["upright"])

    def test_rpc_default_flag_requires_fresh_matching_native_state(self):
        rpc = FakeRobot(ready=False).query()
        state = {"action": FakeRobot().query(), "is_fallen": False,
                 "is_lifted": False, "is_moving": False}
        result = reconcile_action(rpc, (10, state), 10.1)
        self.assertTrue(result["switch_ready"])
        self.assertFalse(result["rpc_switch_ready"])
        self.assertEqual(result["switch_ready_source"], MC_STATE)
        for sample in (None, (9, state), (11, state)):
            self.assertFalse(reconcile_action(rpc, sample, 10)["switch_ready"])
        for key, value in (("value", 1005), ("status", 200)):
            different = {**state, "action": {**state["action"], key: value}}
            self.assertFalse(reconcile_action(rpc, (10, different), 10)["switch_ready"])
        state["action"]["switch_ready"] = False
        self.assertFalse(reconcile_action({**rpc, "switch_ready": True}, (10, state), 10)["switch_ready"])

    def test_native_fall_lift_or_motion_prevents_command(self):
        for key in ("is_fallen", "is_lifted", "is_moving"):
            robot = FakeRobot()
            original = robot.query
            robot.query = lambda: {**original(), key: True}
            self.assertEqual(PresetController(robot).execute("wave")["state"], "rejected")
            self.assertEqual(robot.calls, [])

    def test_only_readonly_rpc_timeout_is_retried(self):
        from types import SimpleNamespace as NS
        transport = RobotTransport.__new__(RobotTransport)
        transport.types = {"GetMcAction": NS(Request=lambda: NS(request=NS(header=NS(stamp=None))))}
        transport.node = Mock()
        transport.pause = Mock()
        transport.common = None
        response = NS(header=NS(code=0), info=NS(current_action=NS(value=1002),
                      status=NS(value=100), switch_ready=False, source="", action_desc="walk"))
        transport.rpc = Mock(side_effect=[TimeoutError("first reply lost"), response])
        self.assertEqual(transport.query()["value"], 1002)
        self.assertEqual(transport.rpc.call_count, 2)
        transport.rpc = Mock(side_effect=TimeoutError("motion reply lost"))
        with self.assertRaises(TimeoutError):
            transport._send("SetMcPresetMotion", object())
        transport.rpc.assert_called_once()

    def test_rpc_error_or_cancellation_is_not_retried(self):
        from types import SimpleNamespace as NS
        transport = RobotTransport.__new__(RobotTransport)
        transport.types = {"GetMcAction": NS(Request=lambda: NS(request=NS(header=NS(stamp=None))))}
        transport.node = Mock()
        transport.rpc = Mock(side_effect=RuntimeError("cancelled"))
        with self.assertRaises(RuntimeError):
            transport.query()
        transport.rpc.assert_called_once()

    def test_core_info_polling_is_immediate_and_never_starts_dds(self):
        card = PresetPosturesCard("test", None, 93)
        with patch("preset_postures.multiprocessing.get_context") as context:
            for _ in range(20):
                result = card.dispatch("info", {})
                self.assertFalse(result["status_fresh"])
                self.assertIsNone(result["status_snapshot"])
            context.assert_not_called()

    def test_header_error_rejects_even_with_running_state(self):
        robot = FakeRobot(1005)
        robot.last_response.update(code=1, state=400, reason=15)
        result = PresetController(robot).execute("wave")
        self.assertEqual(result["state"], "rejected")
        self.assertFalse(result["accepted"])
        self.assertEqual(len(robot.calls), 1)

    def test_running_ack_is_not_completion(self):
        robot = FakeRobot(1005)
        robot.last_response["state"] = 400
        result = PresetController(robot).execute("wave")
        self.assertTrue(result["accepted"])
        self.assertFalse(result["execution_confirmed"])

    def test_cancel_during_mode_change_prevents_motion(self):
        robot = FakeRobot()
        original = robot.set_mode

        def cancel_after_mode(mode):
            reply = original(mode)
            robot.cancelled = True
            return reply

        robot.set_mode = cancel_after_mode
        result = PresetController(robot).execute("wave")
        self.assertEqual(robot.calls, [("mode", 1005)])
        self.assertFalse(result["motion_request_sent"])

    def test_external_mode_change_prevents_motion(self):
        robot = FakeRobot()
        robot.pause = lambda _: setattr(robot, "mode", 2)
        result = PresetController(robot).execute("wave")
        self.assertEqual(robot.calls, [("mode", 1005)])
        self.assertEqual(result["state"], "unknown")

    def test_unconfirmed_mode_does_not_send_motion(self):
        robot = FakeRobot()
        robot.set_mode = lambda mode: robot.last_response
        with patch("preset_postures.time.monotonic", side_effect=[0, 0, 7]):
            result = PresetController(robot).execute("wave")
        self.assertFalse(result["motion_request_sent"])
        self.assertEqual(robot.calls, [])

    def test_lost_motion_response_never_retries(self):
        robot = FakeRobot(1005)

        def lost_response(*args):
            robot.calls.append(args)
            raise TimeoutError("lost")

        robot.preset = lost_response
        result = PresetController(robot).execute("wave")
        self.assertEqual(len(robot.calls), 1)
        self.assertEqual(result["state"], "unknown")
        self.assertIsNone(result["accepted"])

    def test_reset_and_readonly_status(self):
        robot = FakeRobot(1005)
        PresetController(robot).execute("info")
        self.assertEqual(robot.calls, [])
        PresetController(robot).execute("reset")
        self.assertEqual(robot.calls, [("preset", 0, True)])
        robot = FakeRobot(1003)
        PresetController(robot).execute("reset")
        self.assertEqual(robot.calls, [("mode", 1002)])
        robot = FakeRobot(2)
        self.assertEqual(PresetController(robot).execute("reset")["state"], "rejected")
        self.assertEqual(robot.calls, [])

    def test_lifecycle_and_busy_never_spawn_or_queue_commands(self):
        card = PresetPosturesCard("test", None, 93)
        with patch("preset_postures.multiprocessing.get_context") as context:
            card.start()
            card.dispatch("start", {})
            card.dispatch("stop", {})
            card.stop()
            with card._call_lock:
                self.assertEqual(card.dispatch("wave", {})["state"], "busy")
            context.assert_not_called()

    def test_quaternion_validation(self):
        from types import SimpleNamespace as NS
        def imu(w, x, y, z, covariance=0):
            return NS(orientation=NS(w=w, x=x, y=y, z=z), orientation_covariance=[covariance])
        self.assertTrue(upright_quaternion(imu(2, 0, 0, 0)))
        for sample in (imu(0, 0, 0, 0), imu(1, 0, 0, 0, -1),
                       imu(1, 1, 0, 0), imu(float("nan"), 0, 0, 0)):
            self.assertFalse(upright_quaternion(sample))


@unittest.skipUnless(os.environ.get("Q1_RUN_DDS_TESTS") == "1", "requires Q1 ARM64 image")
class DdsPresetTests(unittest.TestCase):
    def test_real_sdk_roundtrip_on_isolated_domain(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
        from nav_msgs.msg import Odometry
        from aimdk_msgs.msg import McCommonState
        from aimdk_msgs.srv import GetMcAction, SetMcAction, SetMcPresetMotion, SetMcMotion

        # Fixed non-production domain: test actuator services cannot reach the robot.
        rclpy.init(domain_id=93)
        node = rclpy.create_node("q1_fake_motion_server", start_parameter_services=False)
        mode, calls = [1002], []

        def query(req, response):
            response.header.code = 0
            response.info.current_action.value = mode[0]
            response.info.status.value = 100
            response.info.switch_ready = False  # Firmware RPC omits this field.
            return response

        def command(kind):
            def callback(req, response):
                self.assertEqual(req.source, "motus_q1_preset_postures")
                if kind == "SetMcAction":
                    mode[0] = req.command.action.value
                    calls.append((kind, mode[0]))
                elif kind == "SetMcPresetMotion":
                    calls.append((kind, req.motion.value, req.interrupt, req.ani_path))
                else:
                    calls.append((kind, req.motion, req.type, req.operation, req.model, req.resource))
                response.response.header.code = 0
                response.response.state.value = 1
                response.response.task_id = 99
                return response
            return callback

        def create_services():
            startup.cancel()
            for typ in (GetMcAction, SetMcAction, SetMcPresetMotion, SetMcMotion):
                node.create_service(typ, "/aimdk_5Fmsgs/srv/" + typ.__name__,
                                    query if typ is GetMcAction else command(typ.__name__))
        # Discovery slower than the old 3-second limit.
        startup = node.create_timer(4, create_services)
        pub = node.create_publisher(Odometry, TORSO_ORIENTATION, qos_profile_sensor_data)
        sample = Odometry()
        sample.pose.pose.orientation.w = 1.
        state_pub = node.create_publisher(McCommonState, MC_STATE, qos_profile_sensor_data)
        def publish():
            state = McCommonState()
            state.action_info.current_action.value = mode[0]
            state.action_info.status.value = 100
            state.action_info.switch_ready = True
            state_pub.publish(state)
            pub.publish(sample)
        node.create_timer(.05, publish)
        stopping = threading.Event()

        def spin():
            while not stopping.is_set():
                rclpy.spin_once(node, timeout_sec=.05)

        thread = threading.Thread(target=spin)
        thread.start()
        card = PresetPosturesCard("test", None, 93)
        try:
            card.start()
            first = card.dispatch("wave", {})
            self.assertEqual(first["state"], "initializing")
            self.assertFalse(first["motion_request_sent"])
            self.assertFalse(first["mode_request_sent"])
            def wait_for_status():
                deadline = time.monotonic() + 25
                while True:
                    result = card.dispatch("status", {})
                    if result["state"] != "initializing":
                        return result
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.2)
            self.assertEqual(wait_for_status()["state"], "ready")
            self.assertEqual(calls, [])  # The cold-start wave was never queued.
            worker_pid = None
            for action in ("status", "wave", "reset", "boxing"):
                result = card.dispatch(action, {})
                self.assertEqual(result["state"], "ready" if action == "status" else "accepted", result)
                pid = card._worker[0].pid
                if worker_pid is not None:
                    self.assertEqual(worker_pid, pid)
                worker_pid = pid
                if action == "status":
                    self.assertEqual(result["posture"]["source"], TORSO_ORIENTATION)
                    self.assertTrue(result["current_action"]["switch_ready"])
                    self.assertFalse(result["current_action"]["rpc_switch_ready"])
                    cached = card.dispatch("info", {})
                    self.assertTrue(cached["status_fresh"])
                    self.assertEqual(cached["status_snapshot"]["current_action"]["value"], 1002)
            self.assertEqual(calls, [("SetMcAction", 1005),
                                    ("SetMcPresetMotion", 3004, False, ""),
                                    ("SetMcPresetMotion", 0, True, ""),
                                    ("SetMcAction", 1002),
                                    ("SetMcMotion", "BOXING", 1, 2, "", "")])
            card.dispatch("stop", {})
            self.assertEqual(card.dispatch("wave", {})["state"], "idle")
            self.assertEqual(len(calls), 5)
            card.start()
            started = time.monotonic()
            self.assertEqual(card.dispatch("status", {})["state"], "ready")
            self.assertLess(time.monotonic() - started, 2)
            self.assertEqual(card._worker[0].pid, worker_pid)
            card._worker[0].terminate()
            card._worker[0].join(timeout=2)
            self.assertEqual(wait_for_status()["state"], "ready")
            self.assertNotEqual(card._worker[0].pid, worker_pid)
            self.assertEqual(len(calls), 5)  # Recovery never replays an action.
        finally:
            card.close()
            stopping.set()
            thread.join(timeout=3)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
