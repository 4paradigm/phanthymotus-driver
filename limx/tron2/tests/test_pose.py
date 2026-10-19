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
        self.shrink=patch('pose.SPEEDS',{'slow':4.,'normal':6.});self.shrink.start();self.addCleanup(self.shrink.stop)

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

    def compact_plan(self,name='wave',side='left'):
        plan=self.adapter.pose.plan({'pose':name,'side':side})
        for f in plan['frames']:f.update(time=.1,hold=0)
        return plan

    def test_public_schema_has_only_pose_side_speed_and_actionless_acp(self):
        t=pose.tool();s=t['inputSchema']
        self.assertEqual(t['name'],'pose');self.assertEqual(set(s['properties']),{'pose','side','speed'})
        self.assertEqual(s['x-completion'],{'timeout':300});self.assertEqual(s['x-resource'],['arm_l','arm_r','head'])
        self.assertEqual(list(s['properties']['pose']['enum']),['salute','wave','welcome','reset'])

    def test_all_preset_sides_fit_vendor_bounds_and_sampled_model_exclusions(self):
        digest=hashlib.sha256(self.file.read_bytes()).hexdigest()
        for name,sides in [('salute',['left','right']),('wave',['left','right']),('welcome',['both']),('reset',['both'])]:
            for side in sides:
                for speed in ('slow','normal'):
                    p=self.adapter.pose.plan({'pose':name,'side':side,'speed':speed})
                    self.assertEqual(p['frames'][-1]['joint'],HOME)
                    self.assertEqual(p['pose_sha256'],digest);self.assertFalse(p['collision_checked'])
                    self.assertLess(sum(f['time']+f['hold']+10 for f in p['frames'])+10,300)
                    if name!='reset':self.assertGreaterEqual(p['minimum_head_reference_distance_m'],.2)
                    for f in p['frames']:
                        completion.validate_movej(f['joint'],f['time'])
                        if name!='reset' and side!='both':
                            inactive=slice(7,14) if side=='left' else slice(0,7)
                            self.assertEqual(f['joint'][inactive],HOME[inactive])
        self.assertEqual(hashlib.sha256(self.file.read_bytes()).hexdigest(),digest)

    async def test_invalid_arguments_do_not_query_modes_or_create_owner(self):
        for args in ({},{'pose':'other'},{'pose':True},{'pose':'wave','side':'both'},
                     {'pose':'wave','side':'LEFT'},{'pose':'salute','speed':True},
                     {'pose':'wave','action':'move'},{'pose':'wave','joint':[0]*14}):
            with self.assertRaises((ValueError,TypeError)):await self.adapter.pose.execute(args)
        self.adapter.pose.guard.assert_not_awaited();self.reader.send_movej.assert_not_awaited()
        self.assertIsNone(self.adapter.motion.active)

    def test_changed_home_or_start_or_head_is_rejected_without_rewriting(self):
        original=self.file.read_bytes()
        for changes in ({'head':math.radians(11)},{'arm':.1},{'moving':1}):
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
        self.assertEqual(len(self.frames),len(plan['frames']));self.assertEqual(self.arrivals,list(range(1,len(self.frames)+1)))
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
                            'params':{'name':'pose','arguments':{'pose':'salute','side':'right','speed':'slow','_trace_id':'trace'}}}) as response:
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
        feedback=HOME+[0,0];stamp=100;writes=[];callbacks=[]
        async def ws_handler(request):
            nonlocal stamp,feedback
            ws=web.WebSocketResponse();await ws.prepare(request)
            packet=info();packet['data']['sw_version']='test-fw';await ws.send_json(packet)
            async for message in ws:
                body=json.loads(message.data);title=body['title']
                if title=='request_get_joint_state':
                    stamp+=1;data=joints(q=feedback,timestamp=stamp)
                elif title=='request_movej':
                    writes.append(body['data']);feedback=list(body['data']['joint'])+[0,0];data={'result':'success'}
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
        reader=RobotReader(settings(url='ws://127.0.0.1:'+str(port)+'/ws',poll_interval_seconds=.01,response_timeout_seconds=.3))
        async with ClientSession() as session:
            task=asyncio.create_task(reader.run(session));adapter=None;rpc_runner=None
            try:
                for _ in range(100):
                    if reader.sample('joint_state')['available']:break
                    await asyncio.sleep(.01)
                adapter=Adapter(lambda *a:None,settings(enabled_cards=['joint_state','pose'],home_file=str(self.file)),reader=reader,session=session)
                original=adapter.pose.plan
                def short(args):
                    plan=original(args)
                    for f in plan['frames']:f.update(time=.1,hold=0)
                    return plan
                rpc_app=web.Application();rpc_app.router.add_post('/mcp',adapter.rpc)
                rpc_runner=web.AppRunner(rpc_app,shutdown_timeout=.2);await rpc_runner.setup()
                rpc_site=web.TCPSite(rpc_runner,'127.0.0.1',0);await rpc_site.start()
                endpoint='http://127.0.0.1:'+str(rpc_site._server.sockets[0].getsockname()[1])+'/mcp'
                with patch('vendor.HEAD_API_PORT',port),patch('pose.TELEOP_HEALTH_URL',root+'/teleop-health'), \
                     patch.dict(os.environ,AGENT_CORE_URL=root),patch('completion.STABILITY_SECONDS',.03), \
                     patch('completion.ARRIVAL_GRACE',.5),patch.object(adapter.pose,'plan',side_effect=short):
                    async with session.post(endpoint,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                        'params':{'name':'pose','arguments':{'pose':'reset','_trace_id':'real-wire'}}}) as response:
                        result=(await response.json())['result'];self.assertFalse(result['isError'],result)
                        accepted=json.loads(result['content'][0]['text'])
                    await asyncio.wait_for(asyncio.shield(adapter.motion.task),3)
                self.assertEqual(len(writes),1);self.assertEqual(set(writes[0]),{'joint','time'})
                self.assertEqual(len(writes[0]['joint']),14);self.assertEqual(len(callbacks),1)
                self.assertEqual(callbacks[0]['action_id'],accepted['action_id']);self.assertEqual(callbacks[0]['tool'],'pose')
                self.assertEqual(callbacks[0]['status'],'completed');self.assertTrue(callbacks[0]['result']['at_home'])
            finally:
                if adapter:await adapter.motion.close();await adapter.head.close()
                if rpc_runner:await rpc_runner.cleanup()
                await session.close();task.cancel();await asyncio.wait_for(asyncio.gather(task,return_exceptions=True),2)
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
            task=asyncio.create_task(self.adapter.pose.execute({'pose':'reset'}))
            for _ in range(100):
                if entered.is_set():break
                await asyncio.sleep(.005)
            self.assertTrue(entered.is_set());self.assertTrue(self.adapter.pose.stop()['cancel_requested']);gate.set()
            with self.assertRaises(MoveError):await task
        self.assertIsNone(self.adapter.motion.active);self.reader.send_movej.assert_not_awaited()

    async def test_lifecycle_ignores_declared_inactive_form_values_without_motion(self):
        for action in ('info','start','stop'):
            result=self.adapter.call('pose',{'action':action,'pose':'salute','side':'right','speed':'slow'})
            self.assertFalse(result['motion_sent'])
        with self.assertRaises(ValueError):self.adapter.call('pose',{'action':'start','joint':[0]*14})
        self.reader.send_movej.assert_not_awaited();self.assertIsNone(self.adapter.motion.active)
