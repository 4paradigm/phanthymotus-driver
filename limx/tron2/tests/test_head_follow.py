"""Pure VOP association and bounded visual-servo policy contracts."""
import copy
import math
import unittest
from head_follow.client import Error, pose
from head_follow.engine import Policy, parse_frame, VOP_TOPIC
from head_follow.servo import Limiter


def packet(t=1000., people=((0.,0.),)):
    return {'active':True,'input_topic':VOP_TOPIC,'received_at':t,
        'data':{'timestamp':t,'latency_ms':0,'count':len(people),
            'objects':[{'name':'person','confidence':.9,'position':list(p)} for p in people]}}


class FollowPolicyTests(unittest.TestCase):
    def setUp(self):self.policy=Policy();self.t=1000.
    def tick(self,people=((0.,0.),),angles=(0.,0.),dt=.3):
        self.t+=dt
        return self.policy.update(packet(self.t,people),self.t,self.t,angles)
    def test_two_distinct_frames_are_required_before_tracking(self):
        self.assertEqual(self.tick(((.8,0.),)),[0.,0.])
        self.assertLess(self.tick(((.8,0.),))[1],0)
    def test_center_deadband_does_not_create_a_motion(self):
        self.tick();self.assertEqual(self.tick(),[0.,0.])
    def test_axis_sign_and_velocity_limits(self):
        self.tick(((.9,.8),));v=self.tick(((.9,.8),))
        self.assertEqual(v,[-8.,-12.])
    def test_ambiguous_people_do_not_confirm_loss_or_switch(self):
        for _ in range(12):self.tick(((-.2,0.),(.2,0.)),angles=(0.,10.))
        self.assertFalse(self.policy.homing);self.assertIsNone(self.policy.target)
        self.assertEqual(self.policy.misses,0)
    def test_short_occlusion_holds_without_home(self):
        self.tick();self.tick();self.tick((),angles=(0.,10.))
        self.assertFalse(self.policy.homing)
    def test_valid_loss_over_time_returns_home_without_identity_jump(self):
        self.tick();self.tick()
        for _ in range(8):v=self.tick((),angles=(0.,10.))
        self.assertTrue(self.policy.homing);self.assertLess(v[1],0)
    def test_duplicate_frames_cannot_confirm_absence(self):
        self.tick();self.tick()
        frame=packet(self.t+.1,())
        for _ in range(10):self.policy.update(frame,self.t+.2,self.t+10,(0.,10.))
        self.assertLess(self.policy.misses,3);self.assertFalse(self.policy.homing)
    def test_failure_and_stale_input_never_mean_no_person(self):
        for bad in ({'active':False},packet(10.,())):
            self.assertIsNone(self.policy.update(bad,1000.,1000.,(0.,10.)))
            self.assertFalse(self.policy.homing)
    def test_clock_regression_is_a_control_error(self):
        self.tick()
        with self.assertRaises(Error):self.policy.update(packet(self.t-.1),self.t,self.t,(0.,0.))
    def test_wrong_topic_invalid_values_and_nonpeople(self):
        p=packet();p['input_topic']='/other'
        with self.assertRaises(Error):parse_frame(p,1000.)
        for value in (float('nan'),float('inf'),2.):
            p=packet(people=((value,0.),))
            with self.assertRaises(Error):parse_frame(p,1000.)
        p=packet();p['data']['objects'][0]['name']='basket'
        self.assertEqual(parse_frame(p,1000.)['points'],[])
    def test_limits_and_body_center_do_not_invent_boxes(self):
        p=packet(people=((1.,-1.),))
        self.assertEqual(parse_frame(p,1000.)['points'],[[1.,0.]])
    def test_limiter_bounds_velocity_acceleration_state_and_displacement(self):
        limiter=Limiter([0.,0.]);last=limiter.position[:]
        for i in range(300):
            target=limiter.step([100.,-100.],.02)
            self.assertLessEqual(max(map(abs,limiter.velocity)),12.)
            self.assertLessEqual(max(map(abs,limiter.acceleration)),32.)
            self.assertLessEqual(max(abs(a-b) for a,b in zip(target,last)),12.*.02+1e-10)
            self.assertTrue(-55<=target[0]<=30 and -45<=target[1]<=45)
            last=target
    def test_large_new_error_is_a_ramp_not_an_instant_jump(self):
        limiter=Limiter([0.,0.]);target=limiter.step([8.,12.],.02)
        self.assertLess(max(map(abs,target)),.01)
        for _ in range(50):limiter.step([8.,12.],.02)
        before=limiter.velocity[:];limiter.step([-8.,-12.],.02)
        self.assertLess(max(abs(a-b) for a,b in zip(before,limiter.velocity)),32.*.02+1e-9)
    def test_no_catchup_after_a_long_loop_gap(self):
        limiter=Limiter([0.,0.])
        for dt in (0.,1.,float('nan')):
            with self.assertRaises(Error):limiter.step([1.,1.],dt)


if __name__=='__main__':unittest.main()
