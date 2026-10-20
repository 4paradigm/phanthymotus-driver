"""Offline TRON2 geometry and shared-owner sequence/ACP contracts; no hardware."""
import asyncio
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import AsyncMock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from aiohttp import web, ClientSession
from test_sensors import settings,info,joints
from device import Adapter
from vendor import RobotReader,MoveError
from state import joint_state
from reset import ARM_NAMES
import completion
import pose

HOME=list(map(math.radians,pose.NOMINAL_HOME))

class PoseTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.file=Path(self.tmp.name)/'home.json'
        self.file.write_text(json.dumps({'schema':'tron2-arms-reset-pose-v1','scope':'arms_only','unit':'rad',
            'joint_names':ARM_NAMES,'target_q':HOME,'head_policy':'unchanged','gripper_policy':'unchanged',
            'accid':'TEST_TRON2A','firmware':'test-fw'}))
        self.reader=RobotReader(settings());packet=info();packet['data']['sw_version']='test-fw'
        self.reader.accept_info(packet);self.reader.ws=types.SimpleNamespace(closed=False)
        self.stamp=100;self.position=HOME+[0,0];self.accept_feedback()
        self.adapter=Adapter(lambda *a:None,settings(enabled_cards=['joint_state','arm_control','move_head','gripper','pose'],home_file=str(self.file)),reader=self.reader)
        self.adapter._report_completion=AsyncMock(return_value=True)
        self.adapter.pose.guard=AsyncMock(return_value=None)
        self.reader.head_control_status=AsyncMock(return_value={'vr_head_follow':False})
        self.reader.send_movej=AsyncMock(side_effect=self.fake_send)
        self.waiter=patch.object(self.adapter.motion,'wait_for_arrival',new=self.fast_arrival);self.waiter.start();self.addCleanup(self.waiter.stop)
        self.frames=[];self.arrivals=[];self.fail_at=None;self.block=None
        self.gripper_opening=0;self.gripper_stamp=1000;self.claw_writes=[]
        self.reader.read_gripper=AsyncMock(side_effect=self.fake_gripper_state)
        self.reader.send_gripper=AsyncMock(side_effect=self.fake_gripper_send)
        self.gripper_timing=patch.multiple('gripper',POLL_SECONDS=.001,STABILITY_SECONDS=.005)
        self.gripper_timing.start();self.addCleanup(self.gripper_timing.stop)

    async def asyncTearDown(self):
        await self.adapter.motion.close();await self.adapter.head.close()
        for o in self.adapter.grippers.values():await o.close()

    def accept_feedback(self,q=None,dq=None,advance=True):
        if q is not None:self.position=list(q)
        if advance:self.stamp+=1
        self.reader.joints=joint_state(joints(q=self.position,dq=[0]*16 if dq is None else dq,timestamp=self.stamp))
        self.reader.joints_at=time.monotonic();self.reader.joints_received=time.time()
        self.reader.info_at=time.monotonic()

    async def fake_send(self,q,duration):
        self.frames.append(list(q))
        if self.fail_at==len(self.frames):raise MoveError('unknown stage ACK',True)
        self.accept_feedback(list(q)+[0,0])
        return {'status':'accepted','motion_sent':True}

    async def fast_arrival(self,job):
        if self.block:await self.block.wait()
        self.arrivals.append(job['stage']);self.accept_feedback()
        return {'arrival_verified':True,'verified_samples':3}

    async def fake_gripper_state(self):
        self.gripper_stamp+=1;self.accept_feedback()
        return {'result':'success','timestamp':self.gripper_stamp,
            'left_opening':0,'left_speed':0,'left_force':0,
            'right_opening':self.gripper_opening,'right_speed':0,'right_force':0}

    async def fake_gripper_send(self,payload):
        self.claw_writes.append(dict(payload));self.gripper_opening=payload['right_opening']
        return {'result':'success'}

    def compact_plan(self,name='wave',side=None):
        # Internal single-home verifier fixture; reset is no longer public.
        plan=self.adapter.pose.plan({'pose':pose.LABELS['welcome' if name=='reset' else name]});plan['frames']=plan['waypoints']
        if name=='reset':plan['frames']=plan['frames'][-1:]
        for f in plan['frames']:
            if f.get('kind')!='gripper':f.update(time=.1,hold=0)
        return plan

    def test_public_schema_has_only_pose_and_actionless_acp(self):
        t=pose.tool();s=t['inputSchema']
        self.assertEqual(t['name'],'pose');self.assertEqual(set(s['properties']),{'pose'})
        self.assertEqual(s['x-completion'],{'timeout':480});self.assertEqual(s['x-resource'],['arm_l','arm_r','head'])
        self.assertEqual(list(s['properties']['pose']['enum']),['salute','wave','welcome'])

    def test_fixed_sides_speed_fit_vendor_bounds_and_sampled_model_exclusions(self):
        digest=hashlib.sha256(self.file.read_bytes()).hexdigest()
        for name in pose.LABELS:
            self.accept_feedback()
            p=self.adapter.pose.plan({'pose':pose.LABELS[name]})
            self.assertEqual(p['side'],'right' if name in ('salute','wave') else 'both')
            self.assertEqual(p['planned_deg_per_second'],12)
            self.assertEqual(p['waypoints'][-1]['joint'],HOME)
            self.assertEqual(p['pose_sha256'],digest);self.assertFalse(p['collision_checked'])
            self.assertLess(sum(f['time']+f['hold']+10 for f in p['waypoints'])+130,480)
            if name!='reset':self.assertGreaterEqual(p['minimum_head_reference_distance_m'],.2)
            for f in p['waypoints']:
                completion.validate_movej(f['joint'],max(1,f['time']))
                if name in ('salute','wave'):self.assertEqual(f['joint'][:7],HOME[:7])
        self.assertEqual(hashlib.sha256(self.file.read_bytes()).hexdigest(),digest)

    def test_shoulders_move_first_then_elbows_and_return_in_reverse_order(self):
        for name in ('salute','wave','welcome'):
            self.accept_feedback()
            p=self.adapter.pose.plan({'pose':pose.LABELS[name]});frames=p['waypoints']
            self.assertEqual([f['label'] for f in frames[:2]],['shoulder',name])
            for offset in ([7] if name!='welcome' else [0,7]):
                self.assertEqual(frames[0]['joint'][offset+3:offset+7],HOME[offset+3:offset+7])
                self.assertNotEqual(frames[0]['joint'][offset:offset+3],HOME[offset:offset+3])
                self.assertEqual(frames[1]['joint'][offset:offset+3],frames[0]['joint'][offset:offset+3])
                self.assertLess(frames[1]['joint'][offset+3],-1.)
                self.assertEqual(frames[-2]['label'],'extend')
                self.assertEqual(frames[-2]['joint'][offset+3:offset+7],HOME[offset+3:offset+7])
                self.assertEqual(frames[-2]['joint'][offset:offset+3],frames[-3]['joint'][offset:offset+3])
            self.assertLess(p['planned_motion_seconds'],55)

    def test_right_only_preserves_actual_left_and_first_shoulder_preserves_actual_elbow(self):
        actual=list(HOME);actual[3]-=.005;actual[10]-=.005
        self.accept_feedback(actual+[0,0])
        for name in ('salute','wave'):
            plan=self.adapter.pose.plan({'pose':pose.LABELS[name]})
            self.assertTrue(all(f['joint'][:7]==actual[:7] for f in plan['waypoints']))
            self.assertEqual(plan['waypoints'][0]['joint'][10:14],actual[10:14])
            self.assertEqual(plan['waypoints'][-1]['joint'][7:],HOME[7:])
            self.assertEqual(plan['home_q'],HOME)

    def test_wave_rotates_joint9_three_times_then_claw_once_before_return(self):
        p=self.adapter.pose.plan({'pose':'wave'});frames=p['waypoints']
        swings=[f for f in frames if f['label'] in ('wave_out','wave_in')]
        self.assertEqual([f['label'] for f in swings],['wave_out','wave_in']*3)
        yaws=[math.degrees(f['joint'][9]) for f in swings]
        self.assertAlmostEqual(max(yaws)-min(yaws),40)
        self.assertTrue(all(abs(math.degrees(f['joint'][8])+25)<1e-6 for f in swings))
        grip=[f for f in frames if f.get('kind')=='gripper']
        self.assertEqual([f['gripper']['opening_mm'] for f in grip],[85,0])
        self.assertEqual(p['gripper_policy'],'right_open_close_once_ends_closed')
        self.assertTrue(all(f['joint']==swings[-1]['joint'] for f in grip))
        self.assertEqual([f['label'] for f in frames[-2:]],['extend','home'])
        self.assertEqual(len(frames),12);self.assertTrue(all(f['hold']==0 for f in frames))

    def test_salute_more_lateral_and_presentation_holds_one_second(self):
        for name in ('salute','welcome'):
            plan=self.adapter.pose.plan({'pose':pose.LABELS[name]})
            displayed=next(f for f in plan['waypoints'] if f['label']==name)
            self.assertEqual(displayed['hold'],1)
            if name=='salute':self.assertAlmostEqual(math.degrees(displayed['joint'][8]),-60)

    def test_faster_wave_only_changes_swing_durations_not_lift_return_or_targets(self):
        plan=self.adapter.pose.plan({'pose':'wave'})
        normal=plan['planned_motion_seconds']
        with patch('pose.WAVE_DEG_PER_SECOND',12):
            old=self.adapter.pose.plan({'pose':'wave'})
        for current,prior in zip(plan['waypoints'],old['waypoints']):
            self.assertEqual(current['joint'],prior['joint'])
            self.assertEqual(current.get('gripper'),prior.get('gripper'))
            if current['label'] in ('wave_out','wave_in'):
                self.assertLess(current['time'],prior['time'])
            else:self.assertEqual(current,prior)
        self.assertGreater(old['planned_motion_seconds']-normal,7)
        self.assertEqual(plan['wave_planned_deg_per_second'],24)

    async def test_pose_feedback_rate_lease_released_after_success_error_cancel_and_report_failure(self):
        for outcome in ('success','error','cancel','callback_failure'):
            self.adapter.motion.active=None;self.frames.clear();self.fail_at=None
            self.accept_feedback(HOME+[0,0]);plan=self.compact_plan('reset')
            if outcome=='cancel':plan['frames'].append(dict(plan['frames'][0]))
            entered=asyncio.Event();release=asyncio.Event()
            async def arrival(job):
                entered.set();await release.wait()
                if outcome=='error':raise ValueError('feedback failed')
                return {'arrival_verified':True}
            self.adapter._report_completion.return_value=outcome!='callback_failure'
            with patch.object(self.adapter.motion,'wait_for_arrival',side_effect=arrival):
                await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
                await asyncio.wait_for(entered.wait(),1)
                self.assertEqual(self.reader.joint_poll_interval(),.1)
                if outcome=='cancel':self.adapter.pose.stop()
                release.set();await self.adapter.motion.task
            self.assertEqual(self.reader.joint_poll_interval(),.5)
            self.assertEqual(self.reader._pose_feedback_leases,set())
            self.assertEqual(self.adapter.motion.last['status'],
                             'error' if outcome=='error' else 'cancelled' if outcome=='cancel' else 'completed')
            if outcome in ('error','callback_failure'):self.assertIsNotNone(self.adapter.motion.active)

    async def test_pose_read_only_polling_wakes_single_reader_and_restores_idle_cadence(self):
        times=[];connections=[]
        async def robot(request):
            ws=web.WebSocketResponse();await ws.prepare(request);connections.append(ws)
            await ws.send_json(info())
            async for msg in ws:
                body=json.loads(msg.data);self.assertEqual(body['title'],'request_get_joint_state')
                self.assertEqual(body['data'],{});times.append(time.monotonic())
                await ws.send_json({'title':'response_get_joint_state','accid':body['accid'],
                    'guid':body['guid'],'data':joints(timestamp=len(times))})
            return ws
        app=web.Application();app.router.add_get('/ws',robot)
        runner=web.AppRunner(app,shutdown_timeout=.2);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        port=site._server.sockets[0].getsockname()[1]
        reader=RobotReader(settings(url='ws://127.0.0.1:'+str(port)+'/ws'))
        try:
            async with ClientSession(trust_env=False) as session:
                task=asyncio.create_task(reader.run(session))
                try:
                    for _ in range(100):
                        if reader.sample('joint_state')['available']:break
                        await asyncio.sleep(.01)
                    await asyncio.sleep(.03);before=len(times);started=time.monotonic()
                    with reader.pose_feedback():
                        # The first fast request wakes the existing .5s wait.
                        await asyncio.sleep(.36)
                        self.assertGreaterEqual(len(times)-before,4)
                        self.assertLess(times[before]-started,.15)
                        with reader.pose_feedback():self.assertEqual(reader.joint_poll_interval(),.1)
                        self.assertEqual(reader.joint_poll_interval(),.1)
                    await asyncio.sleep(.05);idle=len(times)
                    await asyncio.sleep(.25);self.assertEqual(len(times),idle)
                    self.assertEqual(reader.joint_poll_interval(),.5)
                    self.assertEqual(len(connections),1)
                    self.assertEqual(reader.settings['poll_interval_seconds'],.5)
                finally:
                    task.cancel();await asyncio.gather(task,return_exceptions=True)
        finally:await runner.cleanup()

    async def test_full_mixed_sequence_timeout_budget_rejected_before_motion(self):
        plan=self.compact_plan()
        for frame in plan['frames']:
            if frame.get('kind','arm')=='arm':frame['time']=60
        with self.assertRaisesRegex(ValueError,'budget'):
            await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
        self.reader.send_movej.assert_not_awaited();self.reader.send_gripper.assert_not_awaited()
        self.assertIsNone(self.adapter.motion.active)

    async def test_gripper_unknown_ack_cancels_return_and_preserves_parent_owner(self):
        async def uncertain(data):
            self.claw_writes.append(data)
            raise MoveError('unknown claw ACK',True)
        self.reader.send_gripper.side_effect=uncertain
        received=await self.adapter.motion.dispatch_sequence(self.compact_plan(),self.adapter.pose.guard)
        await self.adapter.motion.task
        self.assertEqual(len(self.frames),8);self.assertEqual(len(self.claw_writes),1)
        self.assertEqual(self.adapter.motion.last['status'],'error')
        self.assertEqual(self.adapter.motion.last['gripper_step_error']['parent_action_id'],received['action_id'])
        self.assertTrue(self.adapter.motion.last['motion_may_continue'])
        self.assertIsNotNone(self.adapter.motion.active)
        self.assertEqual(self.adapter._report_completion.await_count,1)

    async def test_gripper_target_miss_and_nonzero_speed_never_continue_to_close_or_home(self):
        import gripper
        for kind in ('miss','speed'):
            self.claw_writes.clear();self.frames.clear();self.gripper_opening=0
            self.accept_feedback(HOME+[0,0])
            async def send(data):
                self.claw_writes.append(data)
                if kind=='speed':self.gripper_opening=data['right_opening']
                return {'result':'success'}
            async def state():
                result=await self.fake_gripper_state()
                if kind=='speed' and self.claw_writes:result['right_speed']=1
                return result
            self.reader.send_gripper.side_effect=send;self.reader.read_gripper.side_effect=state
            with patch.object(gripper,'TIMEOUT',.03):
                await self.adapter.motion.dispatch_sequence(self.compact_plan(),self.adapter.pose.guard)
                await self.adapter.motion.task
            self.assertEqual(len(self.frames),8);self.assertEqual(len(self.claw_writes),1)
            self.assertEqual(self.adapter.motion.last['status'],'error')
            # Only clear offline fixture owners between the two independent cases.
            self.adapter.motion.active=None;self.adapter.grippers['gripper_right'].pose_step=None

    async def test_stop_during_gripper_ack_verifies_current_step_but_no_more_commands(self):
        entered=asyncio.Event();release=asyncio.Event()
        async def send(data):
            self.claw_writes.append(data);self.gripper_opening=data['right_opening']
            entered.set();await release.wait();return {'result':'success'}
        self.reader.send_gripper.side_effect=send
        await self.adapter.motion.dispatch_sequence(self.compact_plan(),self.adapter.pose.guard)
        await asyncio.wait_for(entered.wait(),2)
        self.assertTrue(self.adapter.pose.stop()['cancel_requested']);release.set()
        await self.adapter.motion.task
        self.assertEqual(self.adapter.motion.last['status'],'cancelled')
        self.assertEqual(len(self.frames),8);self.assertEqual(len(self.claw_writes),1)
        self.assertEqual(len(self.adapter.motion.last['gripper_steps']),1)
        self.assertIsNone(self.adapter.motion.active)

    async def test_stop_during_child_pre_read_prevents_any_gripper_write(self):
        entered=asyncio.Event();release=asyncio.Event()
        async def state():
            entered.set();await release.wait();return await self.fake_gripper_state()
        self.reader.read_gripper.side_effect=state
        await self.adapter.motion.dispatch_sequence(self.compact_plan(),self.adapter.pose.guard)
        await asyncio.wait_for(entered.wait(),2);self.adapter.pose.stop();release.set()
        await self.adapter.motion.task
        self.reader.send_gripper.assert_not_awaited()
        self.assertEqual(self.adapter.motion.last['status'],'cancelled');self.assertEqual(len(self.frames),8)

    async def test_other_tools_remain_blocked_while_owned_gripper_moves(self):
        entered=asyncio.Event();release=asyncio.Event()
        async def send(data):
            self.claw_writes.append(data);self.gripper_opening=data['right_opening']
            entered.set();await release.wait();return {'result':'success'}
        self.reader.send_gripper.side_effect=send
        await self.adapter.motion.dispatch_sequence(self.compact_plan(),self.adapter.pose.guard)
        await asyncio.wait_for(entered.wait(),2)
        self.assertEqual(self.adapter.motion.summary()['stage_kind'],'gripper')
        self.assertFalse(self.adapter.grippers['gripper_right'].info()['can_accept_motion'])
        for owner in self.adapter.grippers.values():
            with self.assertRaises(MoveError):await owner.dispatch('open',{'opening_mm':20,'speed':10,'force':10})
        with self.assertRaises(MoveError):await self.adapter.motion.dispatch('move',HOME,8,{})
        with self.assertRaises(MoveError):await self.adapter.head.dispatch('move',[0,0],5,{})
        self.adapter.pose.stop();release.set();await self.adapter.motion.task
        self.assertEqual(len(self.claw_writes),1)

    async def test_arm_drift_during_gripper_verification_cancels_remaining_stages(self):
        async def state():
            result=await self.fake_gripper_state()
            if self.claw_writes:
                changed=list(self.position);changed[10]+=.1;self.accept_feedback(changed)
            return result
        self.reader.read_gripper.side_effect=state
        await self.adapter.motion.dispatch_sequence(self.compact_plan(),self.adapter.pose.guard)
        await self.adapter.motion.task
        self.assertEqual(len(self.frames),8);self.assertEqual(len(self.claw_writes),1)
        self.assertEqual(self.adapter.motion.last['status'],'error');self.assertIsNotNone(self.adapter.motion.active)

    async def test_orphaned_uncertain_child_cannot_be_bypassed_by_public_tools(self):
        owner=self.adapter.grippers['gripper_right']
        owner.pose_step={'parent_action_id':'old','state':'blocked_requires_operator_check'}
        with self.assertRaises(MoveError):await owner.dispatch('open',{'opening_mm':20,'speed':10,'force':10})
        with self.assertRaises(MoveError):await owner.recover({})
        with self.assertRaises(MoveError):await self.adapter.motion.dispatch('move',HOME,8,{})
        with self.assertRaises(MoveError):await self.adapter.head.dispatch('move',[0,0],5,{})
        self.reader.send_gripper.assert_not_awaited();self.reader.send_movej.assert_not_awaited()
        owner.pose_step=None

    async def test_private_gripper_cannot_run_without_exact_live_parent(self):
        plan=self.compact_plan();frame=next(f for f in plan['frames'] if f.get('kind')=='gripper')
        with self.assertRaises(MoveError):
            await self.adapter.grippers['gripper_right'].execute_pose_step(self.adapter.motion,{},frame,lambda:None)
        self.reader.send_gripper.assert_not_awaited();self.reader.read_gripper.assert_not_awaited()
        frame['gripper']['side']='left'
        with self.assertRaises(ValueError):await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
        self.reader.send_movej.assert_not_awaited()

    async def test_failed_wave_admission_gripper_read_is_readonly_and_has_no_motion_owner(self):
        self.reader.read_gripper.side_effect=MoveError('vendor fault',False)
        with self.assertRaises(MoveError):await self.adapter.pose.execute({'pose':'wave'})
        self.reader.send_movej.assert_not_awaited();self.reader.send_gripper.assert_not_awaited()
        self.assertIsNone(self.adapter.motion.active)

    async def test_invalid_arguments_do_not_query_modes_or_create_owner(self):
        for args in ({},{'pose':'reset'},{'pose':'敬礼'},{'pose':'挥手'},{'pose':'欢迎'},{'pose':'other'},{'pose':True},{'pose':'wave','side':'both'}, {'pose':'wave','side':'right'}, {'pose':'wave','speed':'slow'},
                     {'pose':'wave','side':'LEFT'},{'pose':'salute','speed':True},
                     {'pose':'wave','action':'move'},{'pose':'wave','joint':[0]*14}):
            with self.assertRaises((ValueError,TypeError)):await self.adapter.pose.execute(args)
        self.adapter.pose.guard.assert_not_awaited();self.reader.send_movej.assert_not_awaited()
        self.assertIsNone(self.adapter.motion.active)

    def test_changed_home_or_start_or_head_is_rejected_without_rewriting(self):
        original=self.file.read_bytes()
        for changes in ({'head':math.radians(70)},{'arm':.1},{'moving':1}):
            q=HOME+[changes.get('head',0),0]
            q[3]+=changes.get('arm',0)
            self.accept_feedback(q,dq=[changes.get('moving',0)]*16)
            with self.assertRaises(ValueError):self.adapter.pose.plan({'pose':'salute'})
        self.accept_feedback(HOME+[0,0]);data=json.loads(original);data['target_q'][2]=0
        self.file.write_text(json.dumps(data))
        with self.assertRaises(ValueError):self.adapter.pose.plan({'pose':'wave'})
        self.assertEqual(json.loads(self.file.read_text()),data)
        self.reader.send_movej.assert_not_awaited()

    async def test_complete_sequence_reports_once_only_after_every_arrival_and_home(self):
        plan=self.compact_plan();gate=asyncio.Event();self.block=gate
        received=await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
        self.assertTrue(received['motion_sent']);self.assertFalse(received['arrival_verified'])
        self.adapter._report_completion.assert_not_awaited()
        with self.assertRaises(MoveError):await self.adapter.motion.dispatch('move',HOME,8,{})
        with self.assertRaises(MoveError):await self.adapter.head.dispatch('move',[0,0],5,{})
        with self.assertRaises(MoveError):await self.adapter.grippers['gripper_right'].dispatch('open',{'opening_mm':10,'speed':10,'force':10})
        gate.set();await self.adapter.motion.task
        callback=self.adapter._report_completion.await_args.args
        self.assertEqual(callback[:3],('pose',received['action_id'],'completed'))
        arm_stages=[i for i,f in enumerate(plan['frames'],1) if f.get('kind')!='gripper']
        self.assertEqual(len(self.frames),len(arm_stages));self.assertEqual(self.arrivals,arm_stages)
        self.assertEqual([v['right_opening'] for v in self.claw_writes],[94,0])
        self.assertEqual(len(callback[3]['gripper_steps']),2)
        self.assertEqual(self.frames[-1],HOME);self.assertTrue(callback[3]['at_home']);self.assertIsNone(self.adapter.motion.active)
        self.assertEqual(self.adapter._report_completion.await_count,1)

    async def test_later_unknown_ack_sends_no_more_frames_and_keeps_owner(self):
        self.fail_at=2;plan=self.compact_plan('salute')
        received=await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
        await self.adapter.motion.task
        self.assertEqual(len(self.frames),2);self.assertEqual(self.arrivals,[1])
        last=self.adapter.motion.last;self.assertEqual(last['status'],'error');self.assertFalse(last['arrival_verified'])
        self.assertEqual(last['completed_stages'],1);self.assertTrue(last['motion_may_continue'])
        self.assertEqual(self.adapter.motion.active['state'],'blocked_requires_operator_check')
        self.assertEqual(self.adapter._report_completion.await_args.args[:3],('pose',received['action_id'],'error'))

    async def test_first_unknown_ack_has_tracking_id_but_no_false_arrival_or_resend(self):
        self.fail_at=1;received=await self.adapter.motion.dispatch_sequence(self.compact_plan(),self.adapter.pose.guard)
        await self.adapter.motion.task
        self.assertIn('action_id',received);self.assertFalse(received['arrival_verified'])
        self.assertEqual(len(self.frames),1);self.assertEqual(self.arrivals,[])
        self.assertEqual(self.adapter.motion.last['status'],'error');self.assertIsNotNone(self.adapter.motion.active)

    async def test_explicit_first_rejection_has_no_sticky_owner_or_callback(self):
        self.reader.send_movej.side_effect=MoveError('rejected',True,'vendor_rejected')
        with self.assertRaises(MoveError):await self.adapter.motion.dispatch_sequence(self.compact_plan(),self.adapter.pose.guard)
        self.assertIsNone(self.adapter.motion.active);self.adapter._report_completion.assert_not_awaited()

    async def test_stop_cancels_future_frames_after_current_arrival_without_return_motion(self):
        gate=asyncio.Event();self.block=gate
        await self.adapter.motion.dispatch_sequence(self.compact_plan(),self.adapter.pose.guard)
        self.assertTrue(self.adapter.pose.stop()['cancel_requested']);gate.set()
        await self.adapter.motion.task
        self.assertEqual(len(self.frames),1);self.assertEqual(self.adapter.motion.last['status'],'cancelled')
        self.assertFalse(self.adapter.motion.last['motion_may_continue']);self.assertIsNone(self.adapter.motion.active)
        self.assertNotEqual(self.frames[-1],HOME)

    async def test_callback_failure_keeps_shared_owner_even_after_home(self):
        self.adapter._report_completion.return_value=False
        await self.adapter.motion.dispatch_sequence(self.compact_plan('reset'),self.adapter.pose.guard)
        await self.adapter.motion.task
        self.assertTrue(self.adapter.motion.last['at_home']);self.assertFalse(self.adapter.motion.last['callback_delivered'])
        self.assertIsNotNone(self.adapter.motion.active)

    async def test_feedback_mismatch_between_stages_prevents_next_send(self):
        calls=0
        async def guard():
            nonlocal calls
            calls+=1
            if calls==2:self.accept_feedback(HOME+[0,0])
        await self.adapter.motion.dispatch_sequence(self.compact_plan(),guard)
        await self.adapter.motion.task
        self.assertEqual(len(self.frames),1);self.assertEqual(self.adapter.motion.last['status'],'error')

    async def test_sequence_obeys_real_post_duration_advancing_timestamp_verification(self):
        plan=self.compact_plan('reset');self.waiter.stop()
        async def pump():
            while True:self.accept_feedback();await asyncio.sleep(.015)
        task=asyncio.create_task(pump())
        try:
            with patch('completion.STABILITY_SECONDS',.03),patch('completion.ARRIVAL_GRACE',.2):
                await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
                await self.adapter.motion.task
            self.assertEqual(self.adapter.motion.last['status'],'completed')
        finally:task.cancel();await asyncio.gather(task,return_exceptions=True)

    async def test_post_ack_window_overlaps_duration_but_never_completes_early(self):
        plan=self.compact_plan('reset');plan['frames'][0]['time']=.3;self.waiter.stop()
        async def pump():
            while True:self.accept_feedback();await asyncio.sleep(.01)
        task=asyncio.create_task(pump())
        try:
            with patch('completion.STABILITY_SECONDS',.2),patch('completion.ARRIVAL_GRACE',.4):
                started=time.monotonic()
                await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
                await asyncio.sleep(.22)
                self.assertFalse(self.adapter.motion.task.done())
                await self.adapter.motion.task
                elapsed=time.monotonic()-started
            self.assertGreaterEqual(elapsed,.3);self.assertLess(elapsed,.46)
            self.assertEqual(self.adapter.motion.last['status'],'completed')
        finally:task.cancel();await asyncio.gather(task,return_exceptions=True)

    async def test_late_arrival_still_requires_full_stable_feedback_window(self):
        plan=self.compact_plan('reset');plan['frames'][0]['time']=.15;self.waiter.stop()
        self.reader.send_movej.side_effect=None
        self.reader.send_movej.return_value={'status':'accepted','motion_sent':True}
        off=list(HOME);off[3]-=.1
        self.accept_feedback(off+[0,0])
        # Exercise the verifier directly, keeping the plan's target and connection.
        job={'duration':.15,'target':HOME,'ws':self.reader.ws,'serial':self.reader.serial,'sequence':[{}]}
        async def pump():
            started=time.monotonic()
            while True:
                self.accept_feedback((HOME if time.monotonic()-started>=.15 else off)+[0,0])
                await asyncio.sleep(.01)
        task=asyncio.create_task(pump())
        try:
            with patch('completion.STABILITY_SECONDS',.15),patch('completion.ARRIVAL_GRACE',.4):
                started=time.monotonic();result=await self.adapter.motion.wait_for_arrival(job)
            self.assertGreaterEqual(time.monotonic()-started,.3);self.assertTrue(result['arrival_verified'])
        finally:task.cancel();await asyncio.gather(task,return_exceptions=True)

    def test_readiness_sensor_sampling_has_no_undefined_discovery_dependency(self):
        import check
        with patch.object(check,'rpc',return_value={'content':[{'text':json.dumps({'sample':{'available':True}})}]}) as rpc:
            self.assertEqual(check.read_samples(),{'joint_state':{'available':True}})
        self.assertEqual(rpc.call_args.args[1]['arguments']['action'],'info')

    async def test_frozen_vendor_timestamp_cannot_complete_sequence(self):
        plan=self.compact_plan('reset');self.waiter.stop()
        async def pump():
            while True:self.accept_feedback(advance=False);await asyncio.sleep(.015)
        task=asyncio.create_task(pump())
        try:
            with patch('completion.STABILITY_SECONDS',.03),patch('completion.ARRIVAL_GRACE',.15):
                await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
                await self.adapter.motion.task
            self.assertEqual(self.adapter.motion.last['status'],'error');self.assertIsNotNone(self.adapter.motion.active)
        finally:task.cancel();await asyncio.gather(task,return_exceptions=True)

    async def test_actual_mcp_actionless_pose_routes_to_shared_owner_and_pose_callback(self):
        plan=self.compact_plan('salute','right')
        app=web.Application();app.router.add_post('/mcp',self.adapter.rpc)
        runner=web.AppRunner(app,shutdown_timeout=.2);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        url='http://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1])+'/mcp'
        try:
            async with ClientSession() as session:
                with patch.object(self.adapter.pose,'plan',return_value=plan):
                    async with session.post(url,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                            'params':{'name':'pose','arguments':{'pose':'salute','_trace_id':'trace'}}}) as response:
                        body=(await response.json())['result'];self.assertFalse(body['isError'],body)
                await self.adapter.motion.task
                self.assertEqual(self.adapter._report_completion.await_args.args[0],'pose')
                async with session.post(url,json={'jsonrpc':'2.0','id':2,'method':'tools/call',
                        'params':{'name':'pose','arguments':{'pose':'salute','side':'both'}}}) as response:
                    body=(await response.json())['result'];self.assertTrue(body['isError'])
        finally:await runner.cleanup()

    async def test_new_head_deviation_between_frames_cancels_remaining_sends(self):
        calls=0
        async def guard():
            nonlocal calls
            calls+=1
            if calls==2:self.accept_feedback(self.position[:14]+[math.radians(15),0])
        await self.adapter.motion.dispatch_sequence(self.compact_plan(),guard)
        await self.adapter.motion.task
        self.assertEqual(len(self.frames),1);self.assertEqual(self.adapter.motion.last['status'],'error')

    async def test_guard_checks_local_teleop_and_native_follow_without_changing_either(self):
        state={'active':True,'output_active':True};requests=[]
        async def health(request):requests.append(request.method);return web.json_response(state)
        app=web.Application();app.router.add_get('/health',health)
        runner=web.AppRunner(app,shutdown_timeout=.2);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        url='http://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1])+'/health'
        try:
            with patch('pose.TELEOP_HEALTH_URL',url):
                for value in ({'active':True,'output_active':False},{'active':False,'output_active':True},{'active':False},[]):
                    state=value
                    with self.assertRaises(ValueError):await pose.Pose.guard(self.adapter.pose)
                self.reader.head_control_status.assert_not_awaited()
                state={'active':False,'output_active':False}
                await pose.Pose.guard(self.adapter.pose)
                self.reader.head_control_status.assert_awaited_once()
                self.reader.head_control_status.side_effect=MoveError('VR follow enabled',False)
                with self.assertRaises(MoveError):await pose.Pose.guard(self.adapter.pose)
            self.assertEqual(requests,['GET']*6);self.reader.send_movej.assert_not_awaited()
        finally:await runner.cleanup()

    async def test_caller_cancellation_does_not_cancel_or_resend_admitted_sequence(self):
        # Hold the ACK, then cancel the waiting RPC; the shared owner still owes one ACP result.
        gate=asyncio.Event();entered=asyncio.Event();original=self.reader.send_movej.side_effect
        async def send(q,duration):entered.set();await gate.wait();return await original(q,duration)
        self.reader.send_movej.side_effect=send
        caller=asyncio.create_task(self.adapter.motion.dispatch_sequence(self.compact_plan('reset'),self.adapter.pose.guard))
        await entered.wait();caller.cancel();await asyncio.gather(caller,return_exceptions=True)
        gate.set();await self.adapter.motion.task
        self.assertEqual(len(self.frames),1);self.assertEqual(self.adapter._report_completion.await_count,1)
        self.assertEqual(self.adapter.motion.last['status'],'completed')

    async def test_real_websocket_mcp_and_acp_round_trip_for_actionless_pose(self):
        import os
        feedback=HOME+[0,0];stamp=100;writes=[];callbacks=[];claw=0;grip_writes=[];events=[]
        async def ws_handler(request):
            nonlocal stamp,feedback,claw
            ws=web.WebSocketResponse();await ws.prepare(request)
            packet=info();packet['data']['sw_version']='test-fw';await ws.send_json(packet)
            async for message in ws:
                body=json.loads(message.data);title=body['title']
                if title=='request_get_joint_state':
                    stamp+=1;data=joints(q=feedback,timestamp=stamp);await ws.send_json(packet)
                elif title=='request_movej':
                    writes.append(body['data']);events.append('arm');feedback=list(body['data']['joint'])+[0,0];data={'result':'success'}
                elif title=='request_get_limx_2fclaw_state':
                    stamp+=1;data={'result':'success','timestamp':stamp,'left_opening':0,'left_speed':0,'left_force':0,
                        'right_opening':claw,'right_speed':0,'right_force':0}
                elif title=='request_set_limx_2fclaw_cmd':
                    grip_writes.append(body['data']);events.append('claw');claw=body['data']['right_opening'];data={'result':'success'}
                else:raise AssertionError(title)
                await ws.send_json({'accid':body['accid'],'guid':body['guid'],
                    'title':title.replace('request_','response_'),'data':data})
            return ws
        async def head_info(request):return web.json_response({'enable_vr_follow_head':False,
            'enable_vr_follow_head_valid':True,'working_mode':'developer_mode','working_mode_valid':True})
        async def health(request):return web.json_response({'active':False,'output_active':False})
        async def complete(request):
            data=await request.json();callbacks.append(data)
            return web.json_response({'ok':True,'action_id':data['action_id']})
        app=web.Application();app.router.add_get('/ws',ws_handler);app.router.add_get('/get_robot_info',head_info)
        app.router.add_get('/teleop-health',health);app.router.add_post('/api/acp/complete',complete)
        runner=web.AppRunner(app,shutdown_timeout=.2);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start();port=site._server.sockets[0].getsockname()[1]
        root='http://127.0.0.1:'+str(port)
        reader=RobotReader(settings(url='ws://127.0.0.1:'+str(port)+'/ws',poll_interval_seconds=.5,response_timeout_seconds=1))
        async with ClientSession() as session:
            task=asyncio.create_task(reader.run(session));adapter=None;rpc_runner=None
            try:
                for _ in range(100):
                    if reader.sample('joint_state')['available']:break
                    await asyncio.sleep(.01)
                adapter=Adapter(lambda *a:None,settings(enabled_cards=['joint_state','pose'],home_file=str(self.file)),reader=reader,session=session)
                original=adapter.pose.plan
                def short(args):
                    plan=original(args);plan['frames']=plan['waypoints']
                    for f in plan['frames']:
                        if f.get('kind')!='gripper':f.update(time=.1,hold=0)
                    return plan
                rpc_app=web.Application();rpc_app.router.add_post('/mcp',adapter.rpc)
                rpc_runner=web.AppRunner(rpc_app,shutdown_timeout=.2);await rpc_runner.setup()
                rpc_site=web.TCPSite(rpc_runner,'127.0.0.1',0);await rpc_site.start()
                endpoint='http://127.0.0.1:'+str(rpc_site._server.sockets[0].getsockname()[1])+'/mcp'
                with patch('vendor.HEAD_API_PORT',port),patch('pose.TELEOP_HEALTH_URL',root+'/teleop-health'), \
                     patch.dict(os.environ,AGENT_CORE_URL=root),patch('completion.STABILITY_SECONDS',.03), \
                     patch('completion.ARRIVAL_GRACE',.5),patch('vendor.POSE_JOINT_POLL_SECONDS',.02), \
                     patch.object(adapter.pose,'plan',side_effect=short):
                    async with session.post(endpoint,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                        'params':{'name':'pose','arguments':{'pose':'wave','_trace_id':'real-wire'}}}) as response:
                        result=(await response.json())['result'];self.assertFalse(result['isError'],result)
                        accepted=json.loads(result['content'][0]['text'])
                    await asyncio.wait_for(asyncio.shield(adapter.motion.task),6)
                self.assertEqual(len(writes),10);self.assertEqual(set(writes[0]),{'joint','time'})
                self.assertEqual(events,['arm']*8+['claw']*2+['arm']*2)
                self.assertEqual([v['right_opening'] for v in grip_writes],[94,0])
                self.assertTrue(all(set(v)=={'right_opening','right_speed','right_force'} for v in grip_writes))
                self.assertEqual(len(writes[0]['joint']),14);self.assertEqual(len(callbacks),1)
                self.assertEqual(callbacks[0]['action_id'],accepted['action_id']);self.assertEqual(callbacks[0]['tool'],'pose')
                self.assertEqual(callbacks[0]['status'],'completed');self.assertTrue(callbacks[0]['result']['at_home'])
                self.assertEqual(reader.joint_poll_interval(),.5)
            finally:
                if adapter:await adapter.motion.close();await adapter.head.close()
                if rpc_runner:await rpc_runner.cleanup()
                await session.close()
                for _ in range(3):
                    task.cancel()
                    done,_pending=await asyncio.wait([task],timeout=.1)
                    if done:break
                self.assertTrue(task.done())
                await asyncio.gather(task,return_exceptions=True)
        await runner.cleanup()

    async def test_stop_during_admission_guard_prevents_late_first_command(self):
        entered=asyncio.Event();gate=asyncio.Event()
        async def guard():entered.set();await gate.wait()
        self.adapter.pose.guard.side_effect=guard
        task=asyncio.create_task(self.adapter.pose.execute({'pose':'salute'}))
        await entered.wait();self.assertTrue(self.adapter.pose.stop()['cancel_requested']);gate.set()
        with self.assertRaises(MoveError):await task
        self.assertIsNone(self.adapter.motion.active);self.reader.send_movej.assert_not_awaited()

    async def test_stop_during_background_planning_prevents_late_first_command(self):
        import threading
        entered=threading.Event();gate=threading.Event();plan=self.compact_plan('reset')
        def slow(args):entered.set();gate.wait(2);return plan
        with patch.object(self.adapter.pose,'plan',side_effect=slow):
            task=asyncio.create_task(self.adapter.pose.execute({'pose':'welcome'}))
            for _ in range(100):
                if entered.is_set():break
                await asyncio.sleep(.005)
            self.assertTrue(entered.is_set());self.assertTrue(self.adapter.pose.stop()['cancel_requested']);gate.set()
            with self.assertRaises(MoveError):await task
        self.assertIsNone(self.adapter.motion.active);self.reader.send_movej.assert_not_awaited()

    async def test_lifecycle_ignores_declared_inactive_form_values_without_motion(self):
        for action in ('info','start','stop'):
            result=self.adapter.call('pose',{'action':action,'pose':'salute'})
            self.assertFalse(result['motion_sent'])
            if action in ('start','stop'):
                self.assertEqual(result['state'],'ready' if action=='start' else 'idle')
                self.assertTrue(result['lifecycle_only'])
        with self.assertRaises(ValueError):self.adapter.call('pose',{'action':'start','joint':[0]*14})
        self.reader.send_movej.assert_not_awaited();self.assertIsNone(self.adapter.motion.active)

    async def test_reconnection_during_admission_does_not_send_old_request_on_new_socket(self):
        async def changed():self.reader.ws=types.SimpleNamespace(closed=False)
        self.adapter.pose.guard.side_effect=changed
        with self.assertRaises(MoveError):await self.adapter.pose.execute({'pose':'welcome'})
        self.reader.send_movej.assert_not_awaited();self.assertIsNone(self.adapter.motion.active)

    async def test_reconnection_during_planning_does_not_replay_pending_request(self):
        plan=self.compact_plan('reset')
        def changed(args):self.reader.ws=types.SimpleNamespace(closed=False);return plan
        with patch.object(self.adapter.pose,'plan',side_effect=changed):
            with self.assertRaises(MoveError):await self.adapter.pose.execute({'pose':'welcome'})
        self.reader.send_movej.assert_not_awaited();self.assertIsNone(self.adapter.motion.active)
