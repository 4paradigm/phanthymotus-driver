"""Offline tests only. No ROS imports, graph discovery, or robot connection."""
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location("head_probe", Path(__file__).with_name("head_probe.py"))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def contract():
    # Synthetic contract exercises gates, NOT evidence about the real robot.
    c = {k: "offline fixture" for k in (
        "evidence", "reference_frame", "zero_definition", "positive_direction",
        "revolute_velocity_unit", "revolute_acceleration_unit", "prismatic_velocity_unit",
        "prismatic_acceleration_unit", "zero_zones_meaning")}
    c.update(mode="relative", angle_unit="degree", waist_disabled_supported=True,
             min_degree=-5, max_degree=5, revolute_setup_max_vel=.1,
             revolute_setup_max_acc=.1, prismatic_setup_max_vel=.01, prismatic_setup_max_acc=.04)
    return c


class ProbeTests(unittest.TestCase):
    def test_default_is_readonly_without_ros(self):
        self.assertIsNone(probe.plan(probe.parser().parse_args([])))

    def test_stray_motion_values_rejected(self):
        for arg in ("--degree", "--max-vel", "--prismatic-max-acc"):
            with self.assertRaises(ValueError):
                probe.plan(probe.parser().parse_args([arg, "1"]))

    def args(self, path):
        return probe.parser().parse_args([
            "--send-once", "--contract", str(path), "--exclusive-neck-confirmed",
            "--confirm-no-cancel", "--degree", "2", "--max-vel", ".05",
            "--max-acc", ".05", "--prismatic-max-vel", ".005", "--prismatic-max-acc", ".02"])

    def test_motion_gates_and_limits(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "contract.json"
            path.write_text(json.dumps(contract()))
            args = self.args(path)
            req = probe.plan(args)
            self.assertFalse(req["with_waist"])
            self.assertGreater(req["prismatic_setup"]["max_vel"], 0)
            for key, value in (("degree", 0), ("degree", 6), ("degree", float("nan")),
                               ("max_vel", 0), ("max_acc", float("inf")),
                               ("prismatic_max_vel", 0), ("exclusive_neck_confirmed", False),
                               ("confirm_no_cancel", False), ("degree", True)):
                args = self.args(path)
                setattr(args, key, value)
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    probe.plan(args)
            c = contract()
            c["zero_definition"] = "UNCONFIRMED"
            path.write_text(json.dumps(c))
            with self.assertRaises(ValueError):
                probe.plan(self.args(path))

    def test_feedback_freshness_and_content(self):
        data = {"joint_names": ["neck_base_joint"], "positions": [0],
                "actual": {"positions": [0]}, "desired": {"positions": [0]}}
        latest = {topic: (10, data) for topic in (probe.STATE, probe.JOINTS)}
        self.assertTrue(probe.ready(latest, 10.2))
        self.assertFalse(probe.ready(latest, 11))
        self.assertFalse(probe.ready({probe.STATE: (10, data)}, 10.2))
        data["actual"]["positions"] = [float("nan")]
        self.assertFalse(probe.ready(latest, 10.2))

    def runtime(self, request=None, fail=False, completed=False, response_error=False):
        now = [0.0]
        node = Mock()
        callbacks = {}
        node.create_subscription.side_effect = lambda typ, topic, cb, qos: callbacks.update({topic: cb})
        node.get_publishers_info_by_topic.return_value = []
        node.get_service_names_and_types.return_value = []
        client = node.create_client.return_value
        future = client.call_async.return_value
        future.done.return_value = completed
        future.result.return_value = NS(success=True, status=0, message="ok\n\x1b" + "x" * 1000)
        if response_error:
            future.result.side_effect = RuntimeError("response failed")
        if fail:
            client.call_async.side_effect = RuntimeError("transport uncertain")
        client.wait_for_service.return_value = True
        p = NS(positions=[0], velocities=[], accelerations=[], effort=[])
        neck = NS(joint_names=["neck_base_joint"], actual=p, desired=p, error=p)
        def spin(*a, **kw):
            now[0] += .1
            callbacks[probe.STATE](neck)
            callbacks[probe.JOINTS](NS(name=["neck_base_joint"], position=[0], velocity=[0]))
        ros = NS(init=Mock(), create_node=lambda name: node, spin_once=spin, shutdown=Mock())
        mods = {"rclpy": ros, "rclpy.qos": NS(qos_profile_sensor_data=object()),
                "rclpy.utilities": NS(get_rmw_implementation_identifier=lambda: "rmw_fastrtps_cpp"),
                "sensor_msgs.msg": NS(JointState=object),
                "trajectory_msgs.msg": NS(JointTrajectory=object),
                "control_msgs.msg": NS(JointTrajectoryControllerState=object),
                "uworld_motion_msgs.srv": NS(SoundSource=NS(Request=lambda: NS(
                    revolute_setup=NS(), prismatic_setup=NS())))}
        with patch.dict(sys.modules, mods), patch.object(probe.time, "monotonic", side_effect=lambda: now[0]), patch("builtins.print") as printed:
            if fail or response_error:
                with self.assertRaises(RuntimeError):
                    probe.run(NS(seconds=8), request)
            else:
                result = probe.run(NS(seconds=8), request)
                self.assertEqual(result, 2 if request and not completed else 0)
        events = [json.loads(c.args[0]) for c in printed.call_args_list]
        if completed and not response_error:
            receipt = next(e for e in events if e["event"] == "receipt")
            self.assertLessEqual(len(receipt["message"]), 256)
            self.assertNotIn("\n", receipt["message"])
            self.assertNotIn("\x1b", receipt["message"])
            self.assertFalse(receipt["arrival_confirmed"])
        if fail or response_error:
            error = next(e for e in events if e["event"] == "error")
            self.assertTrue(error["request_attempted"])
            self.assertTrue(error["outcome_unknown"])
        node.destroy_node.assert_called_once()
        ros.shutdown.assert_called_once()
        return node, client

    def test_default_never_creates_motion_client(self):
        node, _ = self.runtime()
        node.create_client.assert_not_called()

    def test_timeout_sends_exactly_once_and_cleans_up(self):
        req = {"degree": 2, "revolute_setup": {}, "prismatic_setup": {}}
        node, client = self.runtime(req)
        client.call_async.assert_called_once()
        node.destroy_client.assert_called_once_with(client)

    def test_transport_exception_never_retries(self):
        _, client = self.runtime({"degree": 2, "revolute_setup": {}, "prismatic_setup": {}}, fail=True)
        client.call_async.assert_called_once()

    def test_receipt_is_bounded_not_arrival(self):
        self.runtime({"degree": 2, "revolute_setup": {}, "prismatic_setup": {}}, completed=True)

    def test_response_exception_is_structured_and_cleaned_up(self):
        self.runtime({"degree": 2, "revolute_setup": {}, "prismatic_setup": {}}, completed=True, response_error=True)

    def test_untrusted_collection_and_strings_are_bounded(self):
        data = probe.log_value({"joint_names": ["a\n\x1b" * 300] * 500})
        self.assertEqual(len(data["joint_names"]), 16)
        self.assertTrue(all(len(s) <= 256 and "\x1b" not in s and "\n" not in s for s in data["joint_names"]))


if __name__ == "__main__":
    unittest.main()
