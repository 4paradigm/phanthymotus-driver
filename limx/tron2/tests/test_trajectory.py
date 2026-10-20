"""Offline continuous path, centering, ownership, faults and terminal ACP tests."""
import asyncio
import math
import json
import os
import time
import unittest
from unittest.mock import AsyncMock, patch
from aiohttp import web, ClientSession

import test_pose as fixture
HOME = fixture.HOME
from trajectory import Path, blocks
from vendor import MoveError
import pose
import trajectory


class TrajectoryTests(unittest.IsolatedAsyncioTestCase):
    asyncSetUp = fixture.PoseTests.asyncSetUp
    asyncTearDown = fixture.PoseTests.asyncTearDown
    accept_feedback = fixture.PoseTests.accept_feedback
    fake_send = fixture.PoseTests.fake_send
    fast_arrival = fixture.PoseTests.fast_arrival
    fake_gripper_state = fixture.PoseTests.fake_gripper_state
    fake_gripper_send = fixture.PoseTests.fake_gripper_send

    def short_plan(self, head=False, grip=False):
        current=list(HOME)
        target=list(current);target[7]-=.02
        path=Path(current,[{'joint':target,'time':.1,'hold':0,'label':'raise'},
                                 {'joint':current,'time':.1,'hold':0,'label':'home'}])
        frames=[{'kind':'trajectory','path':path,'joint':current,'time':path.duration,'hold':0,'label':'test_trajectory'}]
        if grip:
            frames += [{'kind':'gripper','label':'gripper_'+action,'joint':current,'time':0,'hold':0,
                        'gripper':{'side':'right','opening_mm':value,'speed':10,'force':10}}
                       for action,value in (('open',85),('close',0))]
        if head:
            frames.insert(0,{'kind':'head','head_joint':[0.,0.],'joint':current,'time':.1,'hold':0,'label':'center_head'})
        return {'frames':frames,'start_q':current,'home_q':current,'start_head_q':self.position[14:],
            'motion_tool':'pose','preset':'wave' if grip else 'salute','side':'right',
            'execution':'time_indexed_quintic_servoj','pose_sha256':'test'}

    async def execute_short(self, plan, servo=None):
        self.servo_writes=[]
        async def follow(q,token):
            self.servo_writes.append((time.monotonic(),q));self.accept_feedback(q)
        self.reader.send_pose_servoj=AsyncMock(side_effect=servo or follow)
        with patch('trajectory.STABILITY_SECONDS',.03),patch('trajectory.OUTPUT_HZ',100),patch('trajectory.MAX_OUTPUT_GAP',.2):
            reply=await self.adapter.motion.dispatch_sequence(plan,self.adapter.pose.guard)
            await self.adapter.motion.task
        return reply

    def test_english_catalog_and_all_real_paths_are_prevalidated(self):
        self.assertEqual(pose.tool()['inputSchema']['properties']['pose']['enum'],['salute','wave','welcome'])
        for name in pose.LABELS:
            plan=self.adapter.pose.plan({'pose':name})
            trajectories=[f for f in plan['frames'] if f.get('kind')=='trajectory']
            self.assertEqual(len(trajectories),2 if name=='wave' else 1)
            for frame in trajectories:
                path=frame['path'];self.assertEqual(path.at(0),path.start)
                self.assertEqual(path.at(path.duration),path.target)
                for i in range(1,100):
                    q=path.at(path.duration*i/100)
                    self.adapter.motion.validate(q,.1)
                self.assertLess(path.duration,60)
            self.assertFalse(plan['collision_checked'])

    def test_disjoint_shoulder_elbow_ramps_overlap_without_joint_overshoot(self):
        shoulder=list(HOME);shoulder[7]-=.1
        elbow=list(shoulder);elbow[10]-=.1
        path=Path(HOME,[{'joint':shoulder,'time':1,'hold':0,'label':'shoulder'},
                             {'joint':elbow,'time':1,'hold':0,'label':'elbow'}])
        self.assertEqual(path.ramps[0][0],0)
        self.assertEqual(path.ramps[1][0],.75)
        q=path.at(.85)
        self.assertLess(q[7],HOME[7]);self.assertGreater(q[7],shoulder[7])
        self.assertLess(q[10],HOME[10]);self.assertGreater(q[10],elbow[10])
        dt=1e-4
        for t in (0.,path.duration):
            self.assertLess(max(abs(a-b)/dt for a,b in zip(path.at(t+dt),path.at(t-dt))),1e-4)

    def test_gripper_barrier_splits_continuous_output(self):
        p=self.adapter.pose.plan({'pose':'wave'})
        self.assertEqual([f['kind'] for f in p['frames']],['trajectory','gripper','gripper','trajectory'])
        self.assertEqual([f['gripper']['opening_mm'] for f in p['frames'] if f['kind']=='gripper'],[85,0])

    def test_all_three_presets_are_twenty_percent_faster_than092_with_unchanged_holds_and_targets(self):
        for name in pose.LABELS:
            fast=self.adapter.pose.plan({'pose':name})
            with patch.dict(pose.MOTION_SCALES,salute=1.2,wave=1.2,welcome=1.):
                baseline=self.adapter.pose.plan({'pose':name})
            scale=1.2 if name=='welcome' else 1.44
            for current,prior in zip(fast['frames'],baseline['frames']):
                if current['kind']!='trajectory':
                    self.assertEqual(current,prior)
                    continue
                a,b=current['path'],prior['path']
                self.assertEqual(a.start,b.start);self.assertEqual(a.target,b.target)
                for ramp,old in zip(a.ramps,b.ramps):
                    self.assertAlmostEqual(ramp[1]*1.2,old[1])
                    self.assertEqual(ramp[2:],old[2:])
                self.assertEqual([h[1:] for h in a.holds],[h[1:] for h in b.holds])
                # Ramp maxima remain analytically bounded, including yaw swings.
                for _,duration,delta,_ in a.ramps:
                    travel=max(abs(math.degrees(v)) for v in delta)
                    self.assertLessEqual(10/math.sqrt(3)*travel/duration**2,60*scale**2+1e-8)
            self.assertEqual(fast['motion_scale'],scale)
            self.assertLess(fast['planned_motion_seconds'],baseline['planned_motion_seconds'])
            info=self.adapter.pose.info()
            self.assertEqual(info['preset_peak_acceleration_deg_per_second_squared'][name],round(60*scale**2,3))

    def test_unbounded_or_nonfinite_motion_scale_is_rejected(self):
        frame={'joint':list(HOME),'time':1,'hold':0,'label':'home'}
        for scale in (True,False,0,.9,1.440001,math.nan,math.inf,'1.44',None):
            with self.assertRaises(ValueError):Path(HOME,[frame],scale)

    def test_head_not_centered_is_planned_before_arms_without_any_write(self):
        self.accept_feedback(HOME+[math.radians(30),math.radians(40)])
        p=self.adapter.pose.plan({'pose':'salute'})
        self.assertEqual(p['frames'][0]['kind'],'head');self.assertEqual(p['frames'][0]['head_joint'],[0,0])
        self.reader.send_movej.assert_not_awaited()
        self.reader.send_gripper.assert_not_awaited()

    async def test_stream_has_no_positive_ack_and_parent_completes_once_at_home(self):
        reply=await self.execute_short(self.short_plan())
        self.assertEqual(reply['status'],'submitted');self.assertFalse(reply['vendor_acknowledged'])
        self.assertGreater(len(self.servo_writes),15)
        self.assertTrue(all(q[14:]==[0,0] for _,q in self.servo_writes))
        self.reader.send_movej.assert_not_awaited()
        last=self.adapter.motion.last
        self.assertEqual(last['status'],'completed');self.assertTrue(last['at_home'])
        self.assertFalse(last['vendor_acknowledged'])
        self.assertIsNone(self.reader._pose_servo);self.assertIsNone(self.adapter.motion.active)
        self.assertEqual(self.adapter._report_completion.await_count,1)
        diag=last['trajectory_diagnostics'][0]
        self.assertEqual(diag['samples_sent'],len(self.servo_writes))
        self.assertGreater(diag['average_output_hz'],0)
        self.assertGreater(diag['max_output_gap_seconds'],0)
        self.assertGreaterEqual(diag['max_tracking_error_deg'],0)

    async def test_center_ack_and_arrival_precede_first_servo_and_no_child_acp(self):
        self.accept_feedback(HOME+[.2,.3]);events=[]
        async def center(q,duration):
            events.append('center_send');self.accept_feedback(HOME+[0,0])
            return {'status':'accepted','motion_sent':True}
        async def arrival(job):events.append('center_verified');return {'arrival_verified':True}
        async def follow(q,token):events.append('servo');self.accept_feedback(q)
        self.reader.send_moveh=AsyncMock(side_effect=center)
        with patch.object(self.adapter.head,'wait_for_arrival',side_effect=arrival):
            await self.execute_short(self.short_plan(head=True),follow)
        self.assertEqual(events[:3],['center_send','center_verified','servo'])
        self.assertTrue(self.adapter.motion.last['head_center_verified'])
        self.assertEqual(self.adapter._report_completion.await_count,1)
        self.assertEqual(self.adapter._report_completion.await_args.args[0],'pose')

    async def test_head_target_miss_or_unknown_ack_blocks_every_arm_and_gripper_write(self):
        for fault in ('target','ack'):
            self.adapter.motion.active=None;self.accept_feedback(HOME+[.2,.3])
            self.adapter._report_completion.reset_mock()
            self.reader.send_moveh=AsyncMock(return_value={'status':'accepted','motion_sent':True})
            async def failure(job):raise MoveError('head target miss',True,'target_not_reached')
            if fault=='ack':self.reader.send_moveh.side_effect=MoveError('unknown ACK',True)
            with patch.object(self.adapter.head,'wait_for_arrival',side_effect=failure):
                await self.execute_short(self.short_plan(head=True,grip=True))
            self.reader.send_pose_servoj.assert_not_awaited();self.reader.send_gripper.assert_not_awaited()
            self.assertEqual(self.adapter.motion.last['status'],'error')
            self.assertIsNotNone(self.adapter.motion.active)

    async def test_servo_fault_keeps_owner_and_does_not_continue_to_gripper(self):
        async def fault(q,token):
            self.accept_feedback(q)
            token['error']='Vendor ServoJ failure notification'
        # Use the actual transport validator to consume the injected notification.
        original=self.reader.send_pose_servoj
        calls=0
        async def send(q,token):
            nonlocal calls
            calls+=1
            if calls==1:await fault(q,token)
            else:await original(q,token)
        await self.execute_short(self.short_plan(grip=True),send)
        self.assertEqual(calls,2);self.assertIsNotNone(self.adapter.motion.active)
        self.reader.send_gripper.assert_not_awaited()
        self.assertIsNone(self.reader._pose_servo)

    async def test_frozen_timestamp_and_tracking_fault_never_finish_successfully(self):
        for kind in ('frozen','tracking'):
            self.adapter.motion.active=None;self.accept_feedback(HOME+[0,0])
            async def bad(q,token):
                if kind=='frozen':self.accept_feedback(advance=False)
                else:
                    bad_q=list(q);bad_q[7]+=.3;self.accept_feedback(bad_q)
            with patch('trajectory.MAX_FEEDBACK_AGE',.06):
                await self.execute_short(self.short_plan(grip=True),bad)
            self.assertEqual(self.adapter.motion.last['status'],'error')
            self.reader.send_gripper.assert_not_awaited()
            self.assertIsNotNone(self.adapter.motion.active)
            summary=self.adapter.motion.summary()
            self.assertEqual(summary['action_id'],self.adapter.motion.last['action_id'])
            self.assertEqual(summary['error_code'],self.adapter.motion.last['error_code'])
            self.assertEqual(summary['state'],'blocked_requires_operator_check')
            self.assertTrue(summary['motion_may_continue'])
            self.assertTrue(summary['trajectory_diagnostics'])
            if kind=='tracking':
                diag=summary['trajectory_diagnostics'][0]
                self.assertEqual(diag['worst_tracking_joint_index'],7)
                self.assertGreater(diag['max_tracking_error_deg'],8)
                self.assertAlmostEqual(abs(diag['worst_tracking_actual_deg']-diag['worst_tracking_reference_deg']),diag['max_tracking_error_deg'])

    async def test_active_summary_never_borrows_previous_task_error(self):
        motion=self.adapter.motion
        motion.last={'action_id':'old','error':'old failure','error_code':'old_code'}
        motion.active={'action_id':'current','action':'move','state':'moving'}
        self.assertEqual(motion.summary(),motion.active)
        motion.active=None

    async def test_stop_stops_future_stream_writes_and_retains_uncertain_owner(self):
        writes=[]
        async def send(q,token):
            writes.append(q);self.accept_feedback(q)
            self.adapter.pose.stop()
        await self.execute_short(self.short_plan(grip=True),send)
        self.assertEqual(len(writes),1);self.reader.send_gripper.assert_not_awaited()
        self.assertTrue(self.adapter.motion.last['motion_may_continue'])
        self.assertIsNotNone(self.adapter.motion.active)

    async def test_invalid_english_request_cannot_center_head_or_create_owner(self):
        self.accept_feedback(HOME+[.2,.3]);self.reader.send_moveh=AsyncMock()
        with self.assertRaises(ValueError):await self.adapter.pose.execute({'pose':'invalid'})
        self.reader.send_moveh.assert_not_awaited();self.adapter.pose.guard.assert_not_awaited()
        self.assertIsNone(self.adapter.motion.active)

    async def test_first_unknown_servo_send_returns_tracking_id_and_retains_owner(self):
        async def uncertain(q,token):raise MoveError('unknown socket submission',None,'trajectory_send_unknown')
        reply=await self.execute_short(self.short_plan(grip=True),uncertain)
        self.assertIn('action_id',reply);self.assertIsNone(reply['motion_sent'])
        self.assertFalse(reply['arrival_verified']);self.assertIsNotNone(self.adapter.motion.active)
        self.reader.send_gripper.assert_not_awaited()

    async def test_unverified_presentation_hold_cannot_be_hidden_by_final_home(self):
        plan=self.short_plan()
        goal=list(HOME);goal[7]-=.02
        path=Path(HOME,[{'joint':goal,'time':.1,'hold':.15,'label':'salute'},
                       {'joint':list(HOME),'time':.1,'hold':0,'label':'home'}])
        plan['frames'][0].update(path=path,joint=path.target,time=path.duration)
        async def not_at_pose(q,token):
            altered=list(q);altered[7]+=.04;self.accept_feedback(altered)
        await self.execute_short(plan,not_at_pose)
        self.assertEqual(self.adapter.motion.last['status'],'error')
        self.assertIn('hold',self.adapter.motion.last['error'])
        self.assertIsNotNone(self.adapter.motion.active)

    async def test_control_source_failure_and_output_stall_cancel_remaining_steps(self):
        for fault in ('source','stall'):
            self.adapter.motion.active=None;self.accept_feedback(HOME+[0,0])
            async def send(q,token):
                self.accept_feedback(q)
                if fault=='stall':await asyncio.sleep(.07)
            self.adapter.pose.guard=AsyncMock(side_effect=ValueError('teleop became active') if fault=='source' else None)
            with patch('trajectory.CONTROL_GUARD_INTERVAL',.02):
                # dispatch_sequence itself runs the same guard before the block.
                calls=0
                async def guard():
                    nonlocal calls
                    calls+=1
                    if calls>1 and fault=='source':raise ValueError('teleop became active')
                plan=self.short_plan(grip=True)
                self.reader.send_pose_servoj=AsyncMock(side_effect=send)
                with patch('trajectory.STABILITY_SECONDS',.03),patch('trajectory.OUTPUT_HZ',100):
                    await self.adapter.motion.dispatch_sequence(plan,guard)
                    await self.adapter.motion.task
            self.assertEqual(self.adapter.motion.last['status'],'error')
            self.reader.send_gripper.assert_not_awaited()
            self.assertIsNotNone(self.adapter.motion.active)

    async def test_servo_transport_rejects_unowned_wrong_shape_and_head_commands(self):
        with self.assertRaises(MoveError):await self.reader.send_pose_servoj(HOME+[0,0],{})
        with self.reader.pose_servo({}) as token:
            for q in (HOME,HOME+[.1,0],HOME+[0,0,0]):
                with self.assertRaises(MoveError):await self.reader.send_pose_servoj(q,token)
            with self.assertRaises(MoveError):await fixture.RobotReader.send_movej(self.reader,HOME,1)

    async def test_real_websocket_home_then_head_then_servo_without_ack_and_parent_acp(self):
        from device import Adapter
        from test_sensors import settings, info, joints
        events=[];callbacks=[];start=list(HOME);start[7]-=.03
        feedback=start+[.2,.3];stamp=1000;claw=0
        async def robot(request):
            nonlocal feedback,stamp,claw
            ws=web.WebSocketResponse();await ws.prepare(request)
            packet=info();packet['data']['sw_version']='test-fw';await ws.send_json(packet)
            async for message in ws:
                body=json.loads(message.data);title=body['title']
                if title=='request_servoj':
                    events.append('servo');feedback=list(body['data']['q'])
                    self.assertEqual(len(feedback),16);self.assertEqual(feedback[14:],[0,0])
                    continue  # Deliberately no positive response exists.
                if title=='request_movej':
                    events.append('home');self.assertEqual(body['data']['joint'],HOME)
                    self.assertEqual(body['data']['time'],8)
                    feedback[:14]=body['data']['joint'];data={'result':'success'}
                elif title=='request_moveh':
                    events.append('head');feedback[14:]=body['data']['joint'];data={'result':'success'}
                elif title=='request_get_joint_state':
                    stamp+=1;data=joints(q=feedback,timestamp=stamp)
                    await ws.send_json(packet)
                elif title=='request_get_limx_2fclaw_state':
                    stamp+=1;data={'result':'success','timestamp':stamp,'left_opening':0,
                        'left_speed':0,'left_force':0,'right_opening':claw,'right_speed':0,'right_force':0}
                elif title=='request_set_limx_2fclaw_cmd':
                    events.append('claw');claw=body['data']['right_opening'];data={'result':'success'}
                else:raise AssertionError(title)
                await ws.send_json({'accid':body['accid'],'guid':body['guid'],
                    'title':title.replace('request_','response_'),'data':data})
            return ws
        async def mode(request):return web.json_response({'enable_vr_follow_head':False,
            'enable_vr_follow_head_valid':True,'working_mode':'developer_mode','working_mode_valid':True})
        async def inactive(request):return web.json_response({'active':False,'output_active':False})
        async def complete(request):
            data=await request.json();callbacks.append(data)
            return web.json_response({'ok':True,'action_id':data['action_id']})
        app=web.Application();app.router.add_get('/ws',robot);app.router.add_get('/get_robot_info',mode)
        app.router.add_get('/teleop',inactive);app.router.add_post('/api/acp/complete',complete)
        runner=web.AppRunner(app,shutdown_timeout=.2);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        port=site._server.sockets[0].getsockname()[1];url='http://127.0.0.1:'+str(port)
        reader=fixture.RobotReader(settings(url='ws://127.0.0.1:'+str(port)+'/ws',poll_interval_seconds=.02))
        adapter=None
        async with ClientSession(trust_env=False) as session:
            task=asyncio.create_task(reader.run(session))
            try:
                for _ in range(100):
                    if reader.sample('joint_state')['available']:break
                    await asyncio.sleep(.01)
                adapter=Adapter(lambda *a:None,settings(home_file=str(self.file),enabled_cards=['pose']),reader,session)
                plan=self.short_plan(head=True,grip=True);plan['start_head_q']=[.2,.3]
                plan.update(start_q=start,home_preparation_required=True,home_policy='verify_then_reset_if_needed')
                plan['frames'].insert(0,{'kind':'home','label':'prepare_home','joint':list(HOME),'time':8,'hold':0})
                rpc=web.Application();rpc.router.add_post('/mcp',adapter.rpc)
                rpc_runner=web.AppRunner(rpc,shutdown_timeout=.2);await rpc_runner.setup()
                rpc_site=web.TCPSite(rpc_runner,'127.0.0.1',0);await rpc_site.start()
                endpoint='http://127.0.0.1:'+str(rpc_site._server.sockets[0].getsockname()[1])+'/mcp'
                try:
                    with patch('vendor.HEAD_API_PORT',port),patch('pose.TELEOP_HEALTH_URL',url+'/teleop'), \
                         patch.dict(os.environ,AGENT_CORE_URL=url),patch('completion.STABILITY_SECONDS',.03), \
                         patch('trajectory.STABILITY_SECONDS',.03),patch('trajectory.OUTPUT_HZ',100), \
                         patch('trajectory.MAX_OUTPUT_GAP',.2),patch.object(adapter.pose,'plan',return_value=plan):
                        # The plan checksum is used by every source guard.
                        plan['pose_sha256']=adapter.reset.load()[2]
                        async with session.post(endpoint,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                            'params':{'name':'pose','arguments':{'pose':'wave'}}}) as response:
                            result=(await response.json())['result'];self.assertFalse(result['isError'],result)
                        await asyncio.wait_for(asyncio.shield(adapter.motion.task),12)
                    self.assertEqual(events[:2],['home','head']);self.assertGreater(events.count('servo'),15)
                    self.assertTrue(callbacks[0]['result']['preparation_home_verified'])
                    self.assertEqual(events.count('claw'),2)
                    self.assertEqual(len(callbacks),1);self.assertEqual(callbacks[0]['status'],'completed')
                    self.assertTrue(callbacks[0]['result']['head_center_verified'])
                    self.assertTrue(callbacks[0]['result']['at_home'])
                finally:await rpc_runner.cleanup()
            finally:
                if adapter:await adapter.motion.close();await adapter.head.close()
                await session.close()
                for _ in range(3):
                    task.cancel()
                    done,_pending=await asyncio.wait([task],timeout=.1)
                    if done:break
                self.assertTrue(task.done())
                await asyncio.gather(task,return_exceptions=True)
        await runner.cleanup()

    async def test_complete_readonly_checker_accepts_current_version_and_english_catalog(self):
        import check
        from device import VERSION
        self.assertEqual(check.VERSION,VERSION)
        self.adapter.registered_id='offline-registered'
        self.reader.send_moveh=AsyncMock()
        self.reader.send_pose_servoj=AsyncMock()
        app=web.Application();app.router.add_get('/health',self.adapter.health)
        app.router.add_post('/mcp',self.adapter.rpc)
        runner=web.AppRunner(app,shutdown_timeout=.2);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        url='http://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1])
        async def feedback():
            while True:
                self.accept_feedback();await asyncio.sleep(.02)
        pump=asyncio.create_task(feedback())
        try:
            with patch.object(check,'BASE',url):
                await asyncio.wait_for(asyncio.to_thread(check.run_check,3,False),6)
            self.reader.send_movej.assert_not_awaited();self.reader.send_moveh.assert_not_awaited()
            self.reader.send_pose_servoj.assert_not_awaited();self.reader.send_gripper.assert_not_awaited()
            self.assertIsNone(self.adapter.motion.active);self.assertIsNone(self.adapter.head.active)
        finally:
            pump.cancel();await asyncio.gather(pump,return_exceptions=True)
            await runner.cleanup()
