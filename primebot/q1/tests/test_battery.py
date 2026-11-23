"""Run inside the Q1 driver image on isolated DDS domains."""
import json
import os
import sys
import time
import unittest
from pathlib import Path

import yaml
from aimdk_msgs.msg import Bms
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.vendor_runtime import DualDomainROS2, DriverBundle
from battery import BatteryCard


class BatteryTest(unittest.TestCase):
    def test_dashboard_deployment(self):
        service = yaml.safe_load(Path("/deploy/service.yml").read_text())["primebot-q1"]
        profile = "/opt/phanthy-motus/dds-local.xml"
        self.assertIn(f"{profile}:{profile}:ro", service["volumes"])
        self.assertEqual(service["environment"]["FASTRTPS_DEFAULT_PROFILES_FILE"], profile)
        self.assertEqual(os.environ.get("RCUTILS_COLORIZED_OUTPUT"), "0")

    def test_robot_to_canvas(self):
        ros = DualDomainROS2(87, 88)
        vendor = Node("test_q1", context=ros.ctx_robot)
        canvas = Node("test_canvas", context=ros.ctx_core)
        ros.executor_robot.add_node(vendor)
        ros.executor_core.add_node(canvas)
        pub = vendor.create_publisher(Bms, "/aima/hal/bms/state", qos_profile_sensor_data)

        frames = []
        canvas.create_subscription(String, "/test_q1/battery",
                                   lambda msg: frames.append(json.loads(msg.data)), 5)
        card = BatteryCard("test_q1", ros, 87)
        ros.start_spin()
        bundle = DriverBundle([card])

        def call(action):
            return bundle.dispatch("battery", {"action": action})

        try:
            bundle.start_all()
            definition, = bundle.get_all_tools()
            self.assertEqual((definition["name"], definition["type"]), ("battery", "sensor"))
            self.assertEqual(set(definition["inputSchema"]["properties"]["action"]["enum"]),
                             {"info", "start", "stop"})
            self.assertEqual(definition["topic_out"], [{"topic": "/test_q1/battery", "format": "data/json"}])
            deadline = time.monotonic() + 5
            while not frames and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertTrue(frames, "No frames on cold start")
            self.assertEqual(frames[-1]["battery"], {"state": "waiting"})
            sample = Bms(remaining_capacity_percentage=73, voltage=51450, current=-260,
                         power=13377, temperature=30.25, remaining_capacity=3390,
                         cycle_count=2, cycle_total_capacity=8625, status=4)
            deadline = time.monotonic() + 14
            while time.monotonic() < deadline:
                pub.publish(sample)
                if sum(frame["battery"]["state"] == "live" for frame in frames) >= 3:
                    break
                time.sleep(0.1)
            self.assertGreaterEqual(sum(frame["battery"]["state"] == "live" for frame in frames), 3)
            data = frames[-1]
            self.assertEqual(set(data), {"battery"})
            expected = {"percentage": 73, "voltage_v": 51.45, "current_a": -0.26,
                        "power_w": 13.377, "temperature_c": 30.25,
                        "remaining_capacity_mah": 3390, "cycle_count": 2, "cycle_total_capacity_mah": 8625, "status_bits": 4}
            self.assertEqual({k: data["battery"][k] for k in expected}, expected)
            self.assertEqual(call("stop")["state"], "idle")
            time.sleep(0.6)
            stopped_count = len(frames)
            time.sleep(5.2)
            self.assertEqual(len(frames), stopped_count)
            self.assertEqual(call("info")["data"]["battery"]["state"], "stale")
            call("start")
            deadline = time.monotonic() + 3
            while len(frames) == stopped_count and time.monotonic() < deadline:
                time.sleep(0.1)
            self.assertGreater(len(frames), stopped_count)
            self.assertEqual(frames[-1]["battery"]["state"], "stale")
        finally:
            bundle.stop_all()
            vendor.destroy_node()
            canvas.destroy_node()
            ros.shutdown()
            for thread in ros._threads:
                thread.join()


if __name__ == "__main__":
    unittest.main()
