"""Automatic original-home preparation and retained fault ownership, offline."""
import asyncio
import copy
import hashlib
import math
from unittest.mock import AsyncMock, patch
import unittest

import test_pose as fixture
import completion
from vendor import MoveError

HOME=fixture.HOME

class HomePreparationTests(unittest.IsolatedAsyncioTestCase):
    asyncSetUp=fixture.PoseTests.asyncSetUp
    asyncTearDown=fixture.PoseTests.asyncTearDown
    accept_feedback=fixture.PoseTests.accept_feedback
    fake_send=fixture.PoseTests.fake_send
    fast_arrival=fixture.PoseTests.fast_arrival
    fake_gripper_state=fixture.PoseTests.fake_gripper_state
    fake_gripper_send=fixture.PoseTests.fake_gripper_send

    def away(self,head=False):
        q=list(HOME);q[2]=math.radians(6.36);q[9]=math.radians(-6.31)
        self.accept_feedback(q+([.2,.3] if head else [0,0]))
        return q

    def test_away_plan_screens_original_home_then_head_then_gesture_and_preserves_file(self):
        before=self.file.read_bytes();start=self.away(head=True)
        for preset in ('salute','wave','welcome'):
            plan=self.adapter.pose.plan({'pose':preset})
            self.assertEqual([f['kind'] for f in plan['frames'][:3]],['home','head','trajectory'])
            home=plan['frames'][0]
            self.assertEqual(home['joint'],HOME);self.assertAlmostEqual(home['time'],16/1.2)
            self.assertEqual(plan['start_q'],start);self.assertEqual(plan['frames'][2]['path'].start,HOME)
            self.assertTrue(plan['home_preparation_required'])
            self.assertFalse(plan['collision_checked']);self.assertEqual(plan['home_policy'],'verify_then_reset_if_needed')
            self.assertEqual(plan['pose_sha256'],hashlib.sha256(before).hexdigest())
            if preset!='welcome':self.assertEqual(plan['waypoints'][0]['joint'][:7],HOME[:7])
        self.assertEqual(before,self.file.read_bytes());self.reader.send_movej.assert_not_awaited()

    def test_already_home_plan_skips_reset_and_keeps_gesture_speed_scales_vs_live092(self):
        for preset,scale in (('salute',1.44),('wave',1.44),('welcome',1.2)):
            plan=self.adapter.pose.plan({'pose':preset})
            self.assertFalse(plan['home_preparation_required'])
            self.assertEqual(plan['frames'][0]['kind'],'trajectory')
            self.assertEqual(plan['motion_scale'],scale)
        self.assertEqual(self.adapter.pose.info()['home_policy'],'verify_then_reset_if_needed')

    async def test_old_tracking_failure_blocks_before_planning_without_auto_reset(self):
        self.away()
        motion=self.adapter.motion
        motion.active={'action_id':'old-wave','action':'pose','state':'blocked_requires_operator_check'}
        with patch.object(self.adapter.pose,'plan') as plan:
            with self.assertRaises(MoveError):await self.adapter.pose.execute({'pose':'wave'})
            plan.assert_not_called()
        self.reader.send_movej.assert_not_awaited();self.reader.send_gripper.assert_not_awaited()
        self.adapter.pose.guard.assert_not_awaited()
        motion.active=None

    async def test_invalid_pose_away_from_home_never_queries_modes_or_resets(self):
        self.away()
        with self.assertRaises(ValueError):await self.adapter.pose.execute({'pose':'other'})
        self.reader.send_movej.assert_not_awaited();self.adapter.pose.guard.assert_not_awaited()
        self.assertIsNone(self.adapter.motion.active)

    async def test_unsafe_home_path_is_rejected_before_any_command(self):
        self.away()
        with patch.object(self.adapter.pose.geometry,'validate_path',side_effect=ValueError('unsafe home segment')) as screen:
            with self.assertRaisesRegex(ValueError,'unsafe home'):await self.adapter.pose.execute({'pose':'salute'})
            self.assertEqual(screen.call_args.args[1][0]['joint'],HOME)
        self.reader.send_movej.assert_not_awaited();self.assertIsNone(self.adapter.motion.active)

    async def test_home_stage_cannot_be_moved_shortened_or_target_modified(self):
        self.away();plan=self.adapter.pose.plan({'pose':'salute'})
        variants=[]
        changed=copy.deepcopy(plan);changed['frames'][0]['time']=1;variants.append(changed)
        changed=copy.deepcopy(plan);changed['frames'][0]['joint'][7]+=.1;variants.append(changed)
        changed=copy.deepcopy(plan);changed['frames']=changed['frames'][1:];variants.append(changed)
        changed=copy.deepcopy(plan);changed['frames'].append(changed['frames'].pop(0));variants.append(changed)
        for changed in variants:
            with self.assertRaises(ValueError):await self.adapter.motion.dispatch_sequence(changed,self.adapter.pose.guard)
        self.reader.send_movej.assert_not_awaited();self.assertIsNone(self.adapter.motion.active)

    async def test_home_unknown_ack_or_target_miss_never_centers_head_or_streams_or_grips(self):
        for fault in ('ack','arrival'):
            self.adapter.motion.active=None;self.away(head=True)
            plan=self.adapter.pose.plan({'pose':'wave'})
            self.reader.send_moveh=AsyncMock();self.reader.send_pose_servoj=AsyncMock()
            if fault=='ack':self.reader.send_movej.side_effect=MoveError('unknown ACK',True)
            else:
                self.reader.send_movej.side_effect=lambda q,duration:{'status':'accepted','motion_sent':True}
                self.adapter.motion.wait_for_arrival=AsyncMock(side_effect=ValueError('target miss'))
            await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
            await self.adapter.motion.task
            self.assertEqual(self.adapter.motion.last['status'],'error')
            self.assertTrue(self.adapter.motion.last['motion_may_continue'])
            self.assertIsNotNone(self.adapter.motion.active)
            self.reader.send_moveh.assert_not_awaited();self.reader.send_pose_servoj.assert_not_awaited()
            self.reader.send_gripper.assert_not_awaited()

    async def test_verified_home_precedes_head_and_gesture_under_one_parent_acp(self):
        self.away(head=True);events=[]
        plan=self.adapter.pose.plan({'pose':'salute'})
        async def home(q,duration):
            events.append('home');self.assertEqual(q,HOME);self.assertAlmostEqual(duration,16/1.2)
            self.accept_feedback(q+self.position[14:]);return {'status':'accepted','motion_sent':True}
        async def home_arrival(job):
            events.append('home_verified');self.accept_feedback()
            return {'arrival_verified':True}
        async def head(q,duration):
            events.append('head');self.assertEqual(self.position[:14],HOME)
            self.accept_feedback(HOME+q);return {'status':'accepted','motion_sent':True}
        async def gesture(owner,job,frame,admitted):
            events.append('gesture');self.assertEqual(self.position,HOME+[0,0])
            self.assertEqual(frame['path'].start,HOME)
            admitted({'motion_sent':True});self.accept_feedback(frame['joint']+[0,0])
        self.reader.send_movej.side_effect=home
        self.adapter.motion.wait_for_arrival=home_arrival
        self.adapter.head.send=AsyncMock(side_effect=head)
        self.adapter.head.wait_for_arrival=AsyncMock()
        with patch('trajectory.run',side_effect=gesture):
            response=await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
            await self.adapter.motion.task
        self.assertEqual(events,['home','home_verified','head','gesture'])
        last=self.adapter.motion.last
        self.assertEqual(last['status'],'completed');self.assertTrue(last['preparation_home_verified'])
        self.assertEqual(response['action_id'],last['action_id'])
        self.assertEqual(self.adapter._report_completion.await_count,1)
        self.assertIsNone(self.adapter.motion.active)

    async def test_stop_while_home_in_flight_verifies_it_then_sends_no_head_or_gesture(self):
        self.away(head=True);plan=self.adapter.pose.plan({'pose':'salute'})
        self.reader.send_moveh=AsyncMock();self.reader.send_pose_servoj=AsyncMock()
        async def home(q,duration):
            self.accept_feedback(q+self.position[14:]);self.adapter.pose.stop()
            return {'status':'accepted','motion_sent':True}
        self.reader.send_movej.side_effect=home
        # Head was not yet centered; cancellation is a stationary home boundary.
        await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
        await self.adapter.motion.task
        self.assertEqual(self.arrivals,[1]);self.reader.send_moveh.assert_not_awaited()
        self.reader.send_pose_servoj.assert_not_awaited()

    async def test_real_start_check_requires_advancing_stable_feedback_before_home_or_gesture(self):
        self.start_verifier.stop()
        for prepare in (False,True):
            self.adapter.motion.active=None
            if prepare:self.away()
            else:self.accept_feedback(HOME+[0,0])
            plan=self.adapter.pose.plan({'pose':'salute'})
            task=asyncio.create_task(self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard))
            await asyncio.sleep(.03)
            self.reader.send_movej.assert_not_awaited()
            # Cancel during a read-only start window: no owner or ACP retained.
            self.adapter.pose.stop()
            with self.assertRaises(MoveError):await task
            self.assertIsNone(self.adapter.motion.active)
            self.adapter._report_completion.assert_not_awaited()

    async def test_frozen_stale_or_drifting_start_never_issues_reset(self):
        self.start_verifier.stop()
        for fault in ('frozen','stale','drift'):
            self.away();plan=self.adapter.pose.plan({'pose':'salute'})
            async def update():
                while True:
                    if fault=='frozen':self.accept_feedback(advance=False)
                    elif fault=='stale':self.reader.joints_at-=1
                    else:
                        q=self.position.copy();q[7]+=.1;self.accept_feedback(q)
                    await asyncio.sleep(.01)
            updater=asyncio.create_task(update())
            try:
                with patch('completion.POSE_PREPARATION_TIMEOUT',.1):
                    with self.assertRaises(MoveError):await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
            finally:
                updater.cancel();await asyncio.gather(updater,return_exceptions=True)
            self.reader.send_movej.assert_not_awaited();self.assertIsNone(self.adapter.motion.active)

    async def test_real_already_home_check_observes_three_advancing_samples_then_no_reset(self):
        self.start_verifier.stop();observations=[]
        async def update():
            while True:
                self.accept_feedback();observations.append(self.stamp);await asyncio.sleep(.01)
        updater=asyncio.create_task(update());plan=self.adapter.pose.plan({'pose':'salute'})
        async def gesture(owner,job,frame,admitted):
            self.assertGreaterEqual(len(observations),3)
            admitted({'motion_sent':True});self.accept_feedback(frame['joint']+[0,0])
        try:
            with patch('completion.STABILITY_SECONDS',.03),patch('trajectory.run',side_effect=gesture):
                await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
                await self.adapter.motion.task
        finally:
            updater.cancel();await asyncio.gather(updater,return_exceptions=True)
        self.reader.send_movej.assert_not_awaited()
        self.assertTrue(self.adapter.motion.last['preparation_home_verified'])
        self.assertEqual(self.adapter.motion.last['status'],'completed')
