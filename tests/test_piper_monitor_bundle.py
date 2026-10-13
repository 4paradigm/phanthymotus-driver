"""Deployment adapter must preserve control ownership and avoid control calls."""
from dataclasses import dataclass
from types import SimpleNamespace
import math
import sys
from pathlib import Path
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agilex.piper.monitor_bundle import add_monitor, read_passive_snapshot


@dataclass
class Grip:
    width_mm: float = 20
    effort_nm: float = 0
    enabled: bool = False
    feedback_time: float = 0


class BundleTests(unittest.TestCase):
    def runtime(self):
        now = time.time()
        state = SimpleNamespace(feedback_time=now, joints_deg=[1,2,3,4,5,6], enabled=[True]*6,
                                control_mode="CAN_CTRL(0x1)", arm_status="NORMAL(0x0)", error_code=0, motor_faults=[])
        driver = SimpleNamespace(get_state=lambda: state, get_gripper=lambda: Grip(feedback_time=now),
                                 arm=SimpleNamespace(GetSDKJointLimitParam=lambda joint: (-math.pi,math.pi)))
        def forbidden(*args, **kwargs):
            self.fail("Snapshot attempted a control/connection operation")
        return SimpleNamespace(config={"name":"机械臂 2", "joint_limits_deg":[[-10,10]]*6},
                               topic="/piper2/piper/rgb", driver=driver, closed=False,
                               connection_lock=threading.Lock(), gate=threading.Lock(), active_card=None,
                               motion_enabled=False, last_actions={}, connected=forbidden,
                               status=forbidden, read_state=forbidden,
                               get_tools=lambda:[{"name":"piper_status"}])

    def test_existing_safe_provider_is_used_verbatim(self):
        r=self.runtime(); data={"fresh":True}
        r.read_monitor_snapshot=lambda: data
        self.assertIs(read_passive_snapshot(r),data)

    def test_legacy_feedback_and_limits_are_passive(self):
        r=self.runtime(); r.gate.acquire()
        data=read_passive_snapshot(r)
        self.assertTrue(data["fresh"])
        self.assertTrue(data["busy"])
        self.assertEqual(data["joint_limits_deg"],[[-10,10]]*6)
        self.assertEqual(data["gripper"]["width_mm"],20)
        self.assertTrue(r.gate.locked())
        self.assertFalse(r.connection_lock.locked())

    def test_unconnected_and_connection_change_do_not_connect_or_wait(self):
        r=self.runtime(); r.driver=None
        self.assertFalse(read_passive_snapshot(r)["connected"])
        r.connection_lock.acquire(); start=time.monotonic()
        self.assertFalse(read_passive_snapshot(r)["fresh"])
        self.assertLess(time.monotonic()-start,.1)

    def test_stale_feedback_never_becomes_fresh(self):
        r=self.runtime(); r.driver.get_state().feedback_time-=1
        data=read_passive_snapshot(r)
        self.assertFalse(data["fresh"])
        self.assertIsNone(data["joints_deg"])

    def test_registration_preserves_runtime_and_isolates_topics(self):
        a,b=self.runtime(),self.runtime(); a.topic="/agilex/piper/rgb"
        pa,pb=add_monitor([a],a.config,None),add_monitor([b],b.config,None)
        self.assertIs(pa[0],a); self.assertIs(pb[0],b)
        self.assertEqual(pa[1].info()["topic_out"][0]["topic"],"/agilex/piper/state/joints")
        self.assertEqual(pb[1].info()["topic_out"][0]["topic"],"/piper2/piper/state/joints")
        with self.assertRaises(ValueError): add_monitor(pa,a.config,None)


if __name__=="__main__": unittest.main()
