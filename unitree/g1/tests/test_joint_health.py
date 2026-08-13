import pathlib
import sys
import unittest


G1_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(G1_DIR))

from joint_health import G1_MOTOR_NAMES, JointHealthAnalyzer, JointHealthPlugin


def motor(temp=40, torque=0.0, q=0.0, dq=0.0):
    return {
        "temperature": [temp, temp - 1],
        "tau_est": torque,
        "q": q,
        "dq": dq,
        "motorstate": 0,
    }


def motors(**overrides):
    result = [motor() for _ in G1_MOTOR_NAMES]
    for idx, values in overrides.items():
        result[int(idx)] = motor(**values)
    return result


class JointHealthAnalyzerTests(unittest.TestCase):
    def test_waits_for_first_lowstate_sample(self):
        snapshot = JointHealthAnalyzer().snapshot(now=10.0)
        self.assertEqual("waiting", snapshot["state"])
        self.assertEqual("no_lowstate_data", snapshot["reason"])

    def test_healthy_snapshot_has_named_motors(self):
        snapshot = JointHealthAnalyzer().update(motors(), now=10.0)
        self.assertEqual("normal", snapshot["health_level"])
        self.assertEqual(35, snapshot["summary"]["healthy"])
        self.assertEqual("left_knee_joint", snapshot["motors"][3]["joint"])

    def test_temperature_warning_and_critical(self):
        analyzer = JointHealthAnalyzer()
        snapshot = analyzer.update(
            motors(**{"3": {"temp": 75}, "9": {"temp": 90}}), now=10.0
        )
        by_joint = {alert["joint"]: alert for alert in snapshot["alerts"]}
        self.assertEqual("warning", by_joint["left_knee_joint"]["level"])
        self.assertEqual("critical", by_joint["right_knee_joint"]["level"])
        self.assertEqual("critical", snapshot["health_level"])

    def test_temperature_rise_requires_five_seconds_of_history(self):
        analyzer = JointHealthAnalyzer({
            "warning_temp_rise_c_per_min": 10,
            "critical_temp_rise_c_per_min": 20,
        })
        analyzer.update(motors(), now=0.0)
        snapshot = analyzer.update(motors(**{"3": {"temp": 41.5}}), now=6.0)
        alerts = [a for a in snapshot["alerts"] if a["joint"] == "left_knee_joint"]
        self.assertEqual("warning", alerts[0]["level"])
        self.assertEqual("temperature_rising_fast", alerts[0]["reason"])

    def test_torque_must_be_sustained(self):
        analyzer = JointHealthAnalyzer({"torque_consecutive_samples": 3})
        analyzer.update(motors(**{"3": {"torque": 50}}), now=1.0)
        analyzer.update(motors(**{"3": {"torque": 50}}), now=1.1)
        snapshot = analyzer.update(motors(**{"3": {"torque": 50}}), now=1.2)
        alerts = [a for a in snapshot["alerts"] if a["joint"] == "left_knee_joint"]
        self.assertEqual("sustained_high_torque", alerts[0]["reason"])
        self.assertEqual("warning", alerts[0]["level"])

    def test_stale_lowstate_is_critical(self):
        analyzer = JointHealthAnalyzer({"stale_timeout_sec": 0.5})
        analyzer.update(motors(), now=1.0)
        snapshot = analyzer.snapshot(now=2.0)
        self.assertEqual("stale", snapshot["state"])
        self.assertEqual("critical", snapshot["health_level"])
        self.assertEqual("lowstate_timeout", snapshot["alerts"][0]["reason"])

    def test_reset_baseline_clears_rate_and_torque_history(self):
        analyzer = JointHealthAnalyzer({"torque_consecutive_samples": 2})
        analyzer.update(motors(), now=0.0)
        analyzer.update(motors(**{"3": {"temp": 45, "torque": 50}}), now=10.0)
        snapshot = analyzer.reset_baseline(now=10.0)
        knee = snapshot["motors"][3]
        self.assertEqual(0, knee["over_torque_samples"])
        self.assertEqual(0.0, knee["temperature_rise_c_per_min"])


class JointHealthPluginContractTests(unittest.TestCase):
    def setUp(self):
        self.plugin = JointHealthPlugin.__new__(JointHealthPlugin)
        self.plugin._topic = "/test_robot/health/joints"
        self.plugin._analyzer = JointHealthAnalyzer()

    def test_tool_declares_sensor_topic_and_actions(self):
        tool = self.plugin.get_tool()
        self.assertEqual("joint_health", tool["name"])
        self.assertEqual("sensor", tool["type"])
        self.assertEqual("data/json", tool["topic_out"][0]["format"])
        self.assertEqual(
            ["snapshot", "list_alerts", "reset_baseline"],
            tool["inputSchema"]["properties"]["action"]["enum"],
        )

    def test_info_returns_authoritative_topic(self):
        result = self.plugin.dispatch("info", {})
        self.assertEqual("waiting", result["state"])
        self.assertEqual(
            "/test_robot/health/joints", result["topic_out"][0]["topic"]
        )


if __name__ == "__main__":
    unittest.main()
