"""Unit tests plus opt-in DDS integration on domain 93 (never robot domain 0)."""
import os
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from preset_postures import MOTIONS, PRESETS, PresetController, PresetPosturesCard
from preset_robot import upright_quaternion


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
        from aimdk_msgs.srv import GetMcAction, SetMcAction, SetMcPresetMotion, SetMcMotion

        # Fixed non-production domain: test actuator services cannot reach the robot.
        rclpy.init(domain_id=93)
        node = rclpy.create_node("q1_fake_motion_server", start_parameter_services=False)
        mode, calls = [1002], []

        def query(req, response):
            response.header.code = 0
            response.info.current_action.value = mode[0]
            response.info.status.value = 100
            response.info.switch_ready = True
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

        for typ in (GetMcAction, SetMcAction, SetMcPresetMotion, SetMcMotion):
            node.create_service(typ, "/aimdk_5Fmsgs/srv/" + typ.__name__,
                                query if typ is GetMcAction else command(typ.__name__))
        pub = node.create_publisher(Imu, "/aima/hal/imu/state", qos_profile_sensor_data)
        sample = Imu()
        sample.orientation.w = 1.
        node.create_timer(.05, lambda: pub.publish(sample))
        stopping = threading.Event()

        def spin():
            while not stopping.is_set():
                rclpy.spin_once(node, timeout_sec=.05)

        thread = threading.Thread(target=spin)
        thread.start()
        card = PresetPosturesCard("test", None, 93)
        try:
            card.start()
            for action in ("status", "wave", "reset", "boxing"):
                result = card.dispatch(action, {})
                self.assertEqual(result["state"], "ready" if action == "status" else "accepted", result)
                if action == "status":
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
        finally:
            card.stop()
            stopping.set()
            thread.join(timeout=3)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
