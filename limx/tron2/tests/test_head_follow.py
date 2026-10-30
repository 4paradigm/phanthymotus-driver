import copy
import math
import unittest
from head_follow.client import Error, pose, Client
from head_follow.engine import Engine, parse_frame, associate


class Fake:
    def __init__(self):
        self.t = 1000.
        self.seq = 1
        self.boxes = [[.4, .3, .6, .9]]
        self.q = [0.]*16
        self.motion = None
        self.calls = []
        self.bad_input = False
        self.uncertain = False
        self.arm_busy = False

    def robot(self):
        return {'version':'0.14.0', 'ready':True, 'robot_connected':True,
                'samples':{'joint_state':{'available':True,'reception_age_seconds':.1,
                    'data':{'q':self.q[:], 'dq':[0.]*16,'robot_timestamp':self.t}}},
                'head_motion':copy.deepcopy(self.motion),'arm_motion':{'state':'moving'} if self.arm_busy else None}

    def detections(self):
        return {'active':True,'input_topic':'/limx_tron2/camera/objects','received_at':self.t,
                'data':{'available':not self.bad_input,'timestamp':self.t,
                'objects':[{'name':'person','confidence':.9,'position':[
                    b[0]+b[2]-1, b[1]+b[3]-.2-1]} for b in self.boxes]}}

    def move(self, pitch, yaw, duration):
        self.calls.append((pitch,yaw,duration))
        if self.uncertain:
            raise TimeoutError('unknown send')
        self.motion = {'action_id':'head-%s'%len(self.calls),'state':'moving'}
        return {'action_id':self.motion['action_id'],'motion_sent':True}

    def arrive(self):
        pitch,yaw,_ = self.calls[-1]
        self.q[14:]=[-math.radians(pitch),math.radians(yaw)]
        self.motion.update(status='completed',state='completed',arrival_verified=True,
            motion_may_continue=False,callback_delivered=True,can_accept_motion=True,
            verification='post_duration_advancing_head_feedback',verified_samples=3)


class FollowTests(unittest.TestCase):
    def setUp(self):
        self.f=Fake();self.e=Engine(self.f,lambda:self.f.t,lambda:self.f.t)

    def tick(self,n=1):
        for _ in range(n):
            self.f.t+=.3;self.f.seq+=1;self.e.step()

    def start(self):
        self.e.call({'action':'follow'})

    def lock(self):
        self.start();self.tick(5)
        self.assertIsNotNone(self.e.target)

    def test_project_lifecycle_and_info_do_not_move(self):
        for a in ('start','info','stop'):
            self.e.call({'action':a});self.tick(10)
        self.assertEqual(self.f.calls,[])

    def test_off_center_person_acquired(self):
        self.f.boxes=[[.75,.3,.95,.9]];self.start();self.tick(10)
        self.assertIsNotNone(self.e.target);self.assertEqual(len(self.f.calls),1)

    def test_central_person_two_frames_and_deadband(self):
        self.lock();self.assertEqual(self.f.calls,[])
        self.assertEqual(self.e.state,'tracking')

    def test_correct_yaw_and_pitch_sign_full_target(self):
        self.lock();self.f.boxes=[[.5,.45,.7,.95]];self.tick()
        p,y,t=self.f.calls[-1]
        self.assertLess(y,0);self.assertLess(p,0)
        self.assertAlmostEqual(p,-2.5);self.assertAlmostEqual(y,-3.);self.assertGreaterEqual(t,.6)

    def test_verified_arrival_does_not_repeat_stationary_wait(self):
        self.lock();self.f.boxes=[[.5,.3,.7,.9]];self.tick();self.f.arrive()
        self.tick();self.assertIsNotNone(self.e.certified_pose)
        self.tick();self.assertEqual(len(self.f.calls),2)

    def test_no_stationary_proof_retains_extra_window(self):
        self.lock();self.f.boxes=[[.5,.3,.7,.9]];self.tick();self.f.arrive()
        self.f.motion.pop('verification')
        self.tick(2);self.assertEqual(len(self.f.calls),1)
        self.tick(3);self.assertEqual(len(self.f.calls),2)

    def test_initial_admission_still_requires_stable_window(self):
        self.start();self.tick(2);self.assertEqual(self.f.calls,[]);self.assertIsNone(self.e.target)

    def test_large_error_not_split_into_four_degree_steps(self):
        self.start()
        from head_follow.client import pose
        p=pose(self.f.robot())
        self.e.dispatch(p,12.,-15.,'follow')
        pitch,yaw,duration=self.f.calls[-1]
        self.assertEqual((pitch,yaw),(12.,-15.))
        self.assertAlmostEqual(duration,2.5)
        self.assertAlmostEqual(max(abs(pitch),abs(yaw))/duration,6.)

    def test_large_home_move_keeps_original_speed_and_limits(self):
        self.f.q[14]=-math.radians(-50.)
        self.start()
        from head_follow.client import pose
        self.e.dispatch(pose(self.f.robot()),0.,0.,'return_home')
        self.assertEqual(self.f.calls[-1][:2],(0.,0.))
        self.assertAlmostEqual(self.f.calls[-1][2],50./6.)

    def test_no_overlapping_head_requests(self):
        self.lock();self.f.boxes=[[.5,.3,.7,.9]];self.tick(15)
        self.assertEqual(len(self.f.calls),1)

    def test_completed_action_then_only_fresh_frame(self):
        self.lock();self.f.boxes=[[.5,.3,.7,.9]];self.tick();self.f.arrive()
        self.tick();self.assertEqual(len(self.f.calls),1)
        self.tick(5);self.assertEqual(len(self.f.calls),2)

    def test_no_person_initially_returns_home(self):
        self.f.q[15]=math.radians(10);self.f.boxes=[];self.start();self.tick(10)
        self.assertTrue(self.e.returning);self.assertEqual(self.f.calls[-1][:2],(0.,0.))
        self.f.arrive();self.tick(5)
        self.assertIsNone(self.e.target);self.assertFalse(self.e.returning)
        self.tick(20);self.assertEqual(len(self.f.calls),1)

    def test_initial_empty_frames_hold_during_grace(self):
        self.f.q[15]=math.radians(10);self.f.boxes=[];self.start();self.tick(5)
        self.assertEqual(self.f.calls,[])

    def test_initial_input_failure_never_recenters(self):
        self.f.q[15]=math.radians(10);self.start();self.f.bad_input=True;self.tick(20)
        self.assertEqual(self.f.calls,[])

    def test_person_found_interrupts_initial_empty_counter(self):
        self.f.q[15]=math.radians(10);self.f.boxes=[];self.start();self.tick(5)
        self.f.boxes=[[.4,.3,.6,.9]];self.tick(3)
        self.assertIsNotNone(self.e.target);self.assertFalse(self.e.returning)
        self.assertEqual(self.f.calls,[])

    def test_pause_removed_but_lifecycle_stop_still_works(self):
        from head_follow.client import Error
        self.start()
        with self.assertRaises(Error):self.e.call({'action':'pause'})
        self.e.call({'action':'stop'});self.assertFalse(self.e.active)

    def test_short_occlusion_does_not_home(self):
        self.lock();self.f.boxes=[];self.tick(4)
        self.assertEqual(self.f.calls,[]);self.assertEqual(self.e.state,'occluded')

    def test_absent_person_returns_home(self):
        self.f.q[15]=math.radians(10);self.lock();self.f.boxes=[];self.tick(8)
        self.assertTrue(self.e.returning);self.assertGreaterEqual(len(self.f.calls),1)
        self.assertAlmostEqual(self.f.calls[0][1],0)

    def test_full_return_home_and_reacquire(self):
        self.f.q[15]=math.radians(10);self.lock();self.f.boxes=[];self.tick(8)
        for _ in range(30):
            if self.e.pending:self.f.arrive()
            self.tick()
        self.assertFalse(self.e.returning);self.assertIsNone(self.e.target)
        self.assertLessEqual(abs(math.degrees(self.f.q[15])),.8)
        self.f.boxes=[[.4,.3,.6,.9]];self.tick(5)
        self.assertIsNotNone(self.e.target)

    def test_camera_failure_never_means_person_gone(self):
        self.f.q[15]=math.radians(10);self.lock();self.f.bad_input=True;self.tick(30)
        self.assertEqual(self.f.calls,[]);self.assertFalse(self.e.returning)

    def test_ambiguous_crossing_does_not_switch(self):
        self.lock();old=self.e.target[:]
        self.f.boxes=[[.39,.3,.59,.9],[.41,.3,.61,.9]];self.tick()
        self.assertEqual(self.e.target,old);self.assertEqual(self.f.calls,[])

    def test_do_not_jump_to_distant_person(self):
        self.lock();self.f.boxes=[[.8,.3,1.,.9]];self.tick()
        self.assertEqual(self.e.state,'occluded');self.assertEqual(self.f.calls,[])

    def test_stop_pending_allows_only_already_accepted_segment(self):
        self.lock();self.f.boxes=[[.5,.3,.7,.9]];self.tick();self.e.call({'action':'stop'})
        self.f.arrive();self.tick(10)
        self.assertEqual(len(self.f.calls),1);self.assertFalse(self.e.active)

    def test_unknown_send_no_retry_no_home(self):
        self.lock();self.f.uncertain=True;self.f.boxes=[[.5,.3,.7,.9]];self.tick(20)
        self.assertEqual(len(self.f.calls),1);self.assertTrue(self.e.unknown_send)
        with self.assertRaises(Error):self.e.call({'action':'follow'})

    def test_external_head_command_stops_follow(self):
        self.lock();self.f.motion={'action_id':'external','status':'completed','arrival_verified':True,
            'motion_may_continue':False,'callback_delivered':True,'can_accept_motion':True}
        self.tick();self.assertEqual(self.e.state,'fault');self.assertEqual(self.f.calls,[])

    def test_arm_busy_stops_follow(self):
        self.lock();self.f.arm_busy=True;self.tick()
        self.assertEqual(self.e.state,'fault');self.assertEqual(self.f.calls,[])

    def test_arm_displaced_stops_follow(self):
        self.lock();self.f.q[0]=.1;self.tick()
        self.assertEqual(self.e.state,'fault')

    def test_stale_wrong_camera_and_invalid_vop_centers(self):
        h=self.f.detections();h['data']['timestamp']-=3
        with self.assertRaises(Error):parse_frame(h,self.f.t)
        h=self.f.detections();h['input_topic']='/limx_tron2/camera_left'
        with self.assertRaises(Error):parse_frame(h,self.f.t)
        h=self.f.detections();h['data']['objects'][0]['position'][0]=float('nan')
        with self.assertRaises(Error):parse_frame(h,self.f.t)

    def test_replayed_vop_results_cannot_confirm_loss(self):
        self.lock();h=self.f.detections();h['data']['objects']=[]
        self.f.detections=lambda:h
        self.tick(15);self.assertFalse(self.e.returning);self.assertEqual(self.f.calls,[])

    def test_driver_failure_not_recovered_by_tracker(self):
        self.lock();self.f.boxes=[[.5,.3,.7,.9]];self.tick()
        self.f.motion.update(status='error',state='blocked_requires_operator_check')
        self.tick(10);self.assertEqual(len(self.f.calls),1);self.assertFalse(self.e.active)

    def test_joint_feedback_invalid_blocks_enable(self):
        self.f.q[0]=float('nan')
        with self.assertRaises(Error):self.start()
        self.assertEqual(self.f.calls,[])

    def test_only_validated_in_process_head_callback(self):
        calls=[]
        def move(pitch,yaw,duration):
            calls.append((pitch,yaw,duration))
            return {'motion_sent':True,'action_id':'one'}
        c=Client(self.f.robot,self.f.detections,move)
        c.move(2,-2,.7)
        self.assertEqual(calls,[(2,-2,.7)])
        c.move(0,0,9.)
        self.assertEqual(calls[-1],(0,0,9.))
        with self.assertRaises(Error):c.move(0,0,15.1)
        with self.assertRaises(Error):c.move(60,0,1)
        self.assertEqual(len(calls),2)


class VopContractTests(unittest.TestCase):
    def packet(self):
        return {'active':True,'input_topic':'/limx_tron2/camera/objects','received_at':1000.,
                'data':{'timestamp':1000.,'objects':[{'name':'person','position':[.6,-.4],'confidence':.9}]}}

    def test_real_vop_point_contract_without_invented_image_geometry(self):
        p=self.packet();f=parse_frame(p,1000.)
        self.assertEqual(f['points'],[[.8,.3]])
        self.assertEqual(f['time_basis'],'vop_publication')
        p['data'].update(count=1,latency_ms=200)
        self.assertEqual(parse_frame(p,1000.)['received'],999.8)

    def test_malformed_or_expired_result_is_not_empty_detection(self):
        for key,value in [('count',2),('count',True),('timestamp',True),('timestamp',998.),
                          ('latency_ms',1600),('latency_ms',float('nan')),('objects',None)]:
            p=self.packet();p['data'][key]=value
            with self.subTest(key=key,value=value),self.assertRaises(Error):parse_frame(p,1000.)
        p=self.packet();p['received_at']=998.
        with self.assertRaises(Error):parse_frame(p,1000.)

    def test_named_feedback_is_reordered_before_head_mapping(self):
        from skeleton import JOINT_NAMES
        h=Fake().robot();d=h['samples']['joint_state']['data']
        d['names']=list(reversed(JOINT_NAMES));q=[0.]*16;q[14:]=[-.2,.1]
        d['q']=list(reversed(q));d['dq']=[0.]*16
        p=pose(h);self.assertEqual(p['q'],q);self.assertAlmostEqual(p['pitch'],math.degrees(.2))

if __name__=='__main__':unittest.main(verbosity=2)
