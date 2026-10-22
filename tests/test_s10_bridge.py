import importlib.util
import sys
import time
import unittest
import subprocess
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deep_robotics/s10_pro"))
from bridge import ReaderBridge, reader_environment


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.bridge = ReaderBridge({}, __file__, process_factory=self.no_process, clock=lambda: self.now)

    @staticmethod
    def no_process(*args, **kwargs):
        raise RuntimeError("offline test")

    def tearDown(self):
        self.bridge.close()

    def test_vendor_child_does_not_inherit_platform_dds(self):
        env = reader_environment({}, {"FASTRTPS_DEFAULT_PROFILES_FILE": "/platform.xml",
                "FASTDDS_DEFAULT_PROFILES_FILE": "/platform.xml", "ROS_LOCALHOST_ONLY": "1",
                "ROS_DOMAIN_ID": "42", "RMW_IMPLEMENTATION": "other"})
        self.assertNotIn("FASTRTPS_DEFAULT_PROFILES_FILE", env)
        self.assertNotIn("FASTDDS_DEFAULT_PROFILES_FILE", env)
        self.assertNotIn("ROS_LOCALHOST_ONLY", env)
        self.assertEqual(env["ROS_DOMAIN_ID"], "0")

    def test_two_owners_release_independently_and_samples_expire(self):
        self.bridge.acquire("state", ["battery"], {"stale_after_s": 3})
        self.bridge.acquire("imu", ["imu"])
        self.assertTrue(self.bridge._accept_event({"key": "battery", "data": {"level":59},
                          "received_monotonic": 100., "received_at_ms":123}))
        self.now = 104.
        self.assertFalse(self.bridge.snapshot("battery")["fresh"])
        self.bridge.release("state")
        self.assertEqual(self.bridge.status()["active_sources"], ["imu"])
        self.assertFalse(self.bridge._accept_event({"key": "battery", "data": {},
                          "received_monotonic": 100.}))
        self.assertIsNone(self.bridge.snapshot("battery")["data"])

    def test_rejects_old_backlog_and_out_of_order_data(self):
        self.bridge.acquire("imu", ["imu"])
        self.assertFalse(self.bridge._accept_event({"key":"imu","data":{},"received_monotonic":80.}))
        self.now = 101.
        self.assertTrue(self.bridge._accept_event({"key":"imu","data":{"n":1},"received_monotonic":101.}))
        self.assertFalse(self.bridge._accept_event({"key":"imu","data":{"n":0},"received_monotonic":100.}))
        self.assertEqual(self.bridge.snapshot("imu")["data"], {"n":1})

    def test_reactivation_drops_cached_data(self):
        self.bridge.acquire("imu", ["imu"])
        self.bridge._accept_event({"key":"imu","data":{},"received_monotonic":100.})
        self.bridge.release("imu")
        self.now = 101.
        self.bridge.acquire("imu", ["imu"])
        self.assertFalse(self.bridge._accept_event({"key":"imu","data":{},"received_monotonic":100.5}))
        self.assertIsNone(self.bridge.snapshot("imu")["data"])


if __name__ == "__main__":
    unittest.main()


class SubprocessLifecycleTests(unittest.TestCase):
    def test_reader_restarts_and_last_release_reaps_real_child(self):
        child_program = '''import json,sys,time
for line in sys.stdin:
    control=json.loads(line)
    for key in control['active']:
        print(json.dumps({'key':key,'data':{'source':key},'received_monotonic':time.monotonic(),
                          'received_at_ms':int(time.time()*1000)}),flush=True)
'''
        children = []

        def factory(command, **kwargs):
            child = subprocess.Popen([sys.executable, "-u", "-c", child_program], **kwargs)
            children.append(child)
            return child

        def wait_for(predicate):
            deadline = time.monotonic() + 5
            while not predicate():
                if time.monotonic() > deadline:
                    self.fail("reader lifecycle condition timed out")
                time.sleep(0.01)

        bridge = ReaderBridge({}, __file__, process_factory=factory)
        try:
            bridge.acquire("imu", ["imu"])
            bridge.acquire("state", ["battery"])
            wait_for(lambda: bridge.snapshot("imu")["fresh"] and bridge.snapshot("battery")["fresh"])
            bridge.release("imu")
            wait_for(lambda: bridge.status()["active_sources"] == ["battery"])
            self.assertTrue(bridge.status()["reader_running"])
            self.assertIsNone(bridge.snapshot("imu")["data"])
            children[0].kill()
            wait_for(lambda: len(children) >= 2 and bridge.status()["reader_running"])
            wait_for(lambda: bridge.snapshot("battery")["sequence"] > 2)
            bridge.release("state")
            wait_for(lambda: not bridge.status()["reader_running"])
            wait_for(lambda: all(child.poll() is not None for child in children))
        finally:
            bridge.close()
        self.assertTrue(all(child.poll() is not None for child in children))
