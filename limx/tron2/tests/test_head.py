"""Head limits, exclusive ownership, and real loopback MoveH/ACP; no hardware."""
import asyncio
import json
import math
import time
import types
import unittest
from unittest.mock import AsyncMock, patch

import aiohttp
from aiohttp import web
from test_sensors import settings, info, joints
from head import HeadCompletion, arguments, tool, card_degrees, COORDINATE_CONVENTION
from device import Adapter
from skeleton import JOINT_NAMES
from state import joint_state
from vendor import MoveError, RobotReader, validate_moveh, HEAD_RAD_LIMITS


class HeadValidationTests(unittest.TestCase):
    def test_manual_degrees_convert_to_only_two_radians(self):
        for pitch,yaw in ((-60,-88),(45,88),(0,0),(12,-20)):
            target,duration=arguments({'pitch_deg':pitch,'yaw_deg':yaw,'time':5})
            self.assertEqual(target,[math.radians(-pitch),math.radians(yaw)])
            for actual,expected in zip(card_degrees(target),(pitch,yaw)):
                self.assertAlmostEqual(actual,expected)
            self.assertEqual(duration,5)
        props=tool()['inputSchema']
        self.assertEqual(set(props['properties']),{'pitch_deg','yaw_deg','time'})
        self.assertEqual(props['properties']['pitch_deg']['minimum'],-60)
        self.assertEqual(props['properties']['pitch_deg']['maximum'],45)
        self.assertEqual(props['properties']['yaw_deg']['minimum'],-88)
        self.assertEqual(props['properties']['yaw_deg']['maximum'],88)
        self.assertEqual(props['required'],['pitch_deg','yaw_deg'])
        self.assertNotIn('action',props['properties'])
        self.assertNotIn('x-action-params',props)
        self.assertEqual(props['x-completion'],{'timeout':120})

    def test_customer_endpoints_preserve_native_physical_limits_without_clamping(self):
        for pitch,yaw in ((45.001,0),(-60.001,0),(0,90.001),(0,-90.001),(45,90.001)):
            with self.subTest(pitch=pitch,yaw=yaw),self.assertRaisesRegex(ValueError,'头部角度范围'):
                arguments({'pitch_deg':pitch,'yaw_deg':yaw,'time':5})
            with self.assertRaises(ValueError):validate_moveh([math.radians(-pitch),math.radians(yaw)],5)
        for pitch,yaw in ((-60,-88),(45,88),(5,0),(0,-88),(0,88),(-12,20)):
            target,_=arguments({'pitch_deg':pitch,'yaw_deg':yaw,'time':5})
            self.assertEqual(target,[math.radians(-pitch),math.radians(yaw)])
            self.assertTrue(all(low<=v<=high for v,(low,high) in zip(target,HEAD_RAD_LIMITS)))

    def test_yaw_margin_rejects_old_endpoints_without_clamping_or_pitch_change(self):
        for yaw in (-90,-89,-88.001,88.001,89,90):
            with self.subTest(yaw=yaw):
                with self.assertRaises(ValueError):arguments({'pitch_deg':0,'yaw_deg':yaw})
                with self.assertRaises(MoveError) as caught:validate_moveh([0,math.radians(yaw)],5)
                self.assertFalse(caught.exception.motion_sent)
                self.assertEqual(caught.exception.code,'head_yaw_out_of_range')
        for pitch in (-60,45):
            for yaw in (-88,88):
                target,_=arguments({'pitch_deg':pitch,'yaw_deg':yaw})
                self.assertEqual(target,[math.radians(-pitch),math.radians(yaw)])
        # Feedback plausibility still follows the mechanical range, not command margin.
        self.assertEqual(HEAD_RAD_LIMITS[1],(-math.pi/2,math.pi/2))

    def test_missing_bad_and_out_of_range_values_never_validate(self):
        for key in ('pitch_deg','yaw_deg'):
            for value in (None,True,'1',float('nan'),float('inf'),10**1000,-100,100):
                with self.subTest(key=key,value=type(value).__name__),self.assertRaises(ValueError):
                    arguments({**{'pitch_deg':0,'yaw_deg':0},key:value})
        for args in ({'pitch_deg':0},{'yaw_deg':0},{}):
            with self.assertRaises(ValueError):arguments(args)
        for duration in (True,0,61,float('nan'),'5'):
            with self.assertRaises(ValueError):arguments({'pitch_deg':0,'yaw_deg':0,'time':duration})
        for target in ([0]*14,[0],[-math.pi/4-.001,0],[0,math.pi/2+.001]):
            with self.assertRaises(ValueError):validate_moveh(target,5)


class HeadCompletionTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.reader=RobotReader(settings());self.reader.accept_info(info())
        self.reader.ws=types.SimpleNamespace(closed=False)
        self.reader.send_moveh=AsyncMock(return_value={'status':'accepted','motion_sent':True})
        self.reader.head_control_status=AsyncMock(return_value={'vr_head_follow':False})
        self.reader.send_movej=AsyncMock(return_value={'status':'accepted','motion_sent':True})
        self.report=AsyncMock(return_value=True)
        self.owner=HeadCompletion(self.reader,self.report)
        self.tick=100;self.feedback([0]*16)
        self.patches=[patch('completion.ARRIVAL_GRACE',.22),patch('completion.STABILITY_SECONDS',.04),patch('head.RECOVERY_TIMEOUT',.25)]
        for p in self.patches:p.start()
        self.pump_task=None

    def feedback(self,q,dq=None,names=None,advance=True):
        if advance:self.tick+=1
        self.reader.joints=joint_state(joints(q=q,dq=[0]*16 if dq is None else dq,
            names=[] if names is None else names,timestamp=self.tick))
        self.reader.joints_at=time.monotonic();self.reader.joints_received=time.time()

    def pump(self,q,dq=None,advance=True,names=None):
        async def loop():
            while True:
                self.feedback(q,dq,names,advance);await asyncio.sleep(.015)
        self.pump_task=asyncio.create_task(loop())

    async def asyncTearDown(self):
        if self.pump_task:
            self.pump_task.cancel();await asyncio.gather(self.pump_task,return_exceptions=True)
        await self.owner.close()
        for p in reversed(self.patches):p.stop()

    async def dispatch(self):
        return await self.owner.dispatch('move',[.1,-.1],.1,{'target_pitch_deg':math.degrees(.1),'target_yaw_deg':math.degrees(-.1)})

    async def test_ack_not_arrival_then_fresh_head_feedback_completes_and_releases(self):
        accepted=await self.dispatch()
        self.assertTrue(accepted['action_id'].startswith('tron2_head_'))
        self.assertFalse(accepted['arrival_verified']);self.report.assert_not_called()
        self.pump([.7]*14+[.1,-.1])  # Unrelated arm positions do not affect head arrival.
        await self.owner.task
        self.reader.send_moveh.assert_awaited_once_with([.1,-.1],.1)
        self.reader.send_movej.assert_not_called()
        self.assertEqual(self.owner.last['status'],'completed')
        self.assertTrue(self.owner.last['head_target_verified'])
        self.assertEqual(self.owner.last['joint_count'],2)
        self.assertTrue(self.owner.last['callback_delivered']);self.assertIsNone(self.owner.active)

    async def test_invalid_target_never_owns_head_and_valid_next_request_is_admitted(self):
        for target,duration in (([0,math.pi/2],10),([-math.pi/4-.01,0],10),([0,0],float('nan')),([0,0],0)):
            with self.assertRaises(ValueError):await self.owner.dispatch('move',target,duration,{})
            self.assertIsNone(self.owner.active);self.assertIsNone(self.owner.task)
            self.assertIsNone(self.owner.last)
        self.reader.send_moveh.assert_not_called();self.report.assert_not_called()
        await self.dispatch();self.pump([0]*14+[.1,-.1]);await self.owner.task
        self.assertEqual(self.owner.last['status'],'completed')

    async def test_moving_or_out_of_range_feedback_rejects_without_lock_and_can_retry(self):
        for q,dq,code in (([0]*14+[0,0],[0]*14+[.2,0],'head_not_stationary'),
                          ([0]*14+[2,0],[0]*16,'head_feedback_out_of_range')):
            self.feedback(q,dq)
            with self.assertRaises(MoveError) as caught:await self.dispatch()
            self.assertEqual(caught.exception.code,code);self.assertFalse(caught.exception.motion_sent)
            self.assertIsNone(self.owner.active);self.assertIsNone(self.owner.task)
            self.reader.send_moveh.assert_not_called()
        self.feedback([0]*16)
        await self.dispatch();self.pump([0]*14+[.1,-.1]);await self.owner.task
        self.assertEqual(self.owner.last['status'],'completed')

    async def test_feedback_change_between_admission_and_send_does_not_latch(self):
        original=self.owner.validate_admission_feedback;calls=[]
        def check():
            calls.append(1)
            if len(calls)==2:self.feedback([0]*16,[0]*14+[.2,0])
            original()
        with patch.object(self.owner,'validate_admission_feedback',side_effect=check):
            with self.assertRaises(MoveError):await self.dispatch()
        self.reader.send_moveh.assert_not_called();self.report.assert_not_called()
        self.assertIsNone(self.owner.active)

    async def blocked(self):
        q=[0]*16
        await self.dispatch();self.pump(q);await self.owner.task
        self.assertEqual(self.owner.last['error_code'],'target_not_reached')
        return q

    async def test_new_valid_goal_recovers_stationary_target_miss_and_preserves_error(self):
        q=await self.blocked();old=self.owner.last['action_id']
        admitted=await self.owner.dispatch('move',[.02,0],.1,{})
        previous=admitted['previous_action_error']
        self.assertEqual(previous['action_id'],old);self.assertEqual(previous['status'],'error')
        self.assertFalse(previous['arrival_verified']);self.assertFalse(previous['recovery']['motion_sent'])
        self.assertGreaterEqual(previous['recovery']['verified_samples'],3)
        q[14:]=[.02,0];await self.owner.task
        self.assertEqual(self.owner.last['status'],'completed')
        self.assertNotEqual(self.owner.last['action_id'],old)
        self.assertEqual(self.reader.send_moveh.await_count,2)

    async def test_unknown_ack_callback_or_connection_never_auto_recovers(self):
        await self.blocked();original=dict(self.owner.last);ws=self.reader.ws
        for change in ('ack','callback','connection','peer'):
            self.owner.last=dict(original);self.reader.ws=ws;self.owner.peers=[]
            if change=='ack':self.owner.last['vendor_acknowledged']=False
            elif change=='callback':self.owner.last['callback_delivered']=False
            elif change=='connection':self.reader.ws=types.SimpleNamespace(closed=False)
            else:self.owner.peers=[types.SimpleNamespace(closed=False,active={'state':'moving'})]
            with self.assertRaises(MoveError):await self.owner.dispatch('move',[0,0],.1,{})
            self.assertIsNotNone(self.owner.active);self.assertFalse(self.owner.recovering)
        self.reader.send_moveh.assert_awaited_once()

    async def test_recovery_requires_advancing_frames_and_stationary_arms_too(self):
        await self.blocked();self.pump_task.cancel();await asyncio.gather(self.pump_task,return_exceptions=True)
        self.feedback([0]*16)
        with self.assertRaises(MoveError) as caught:await self.owner.dispatch('move',[0,0],.1,{})
        self.assertEqual(caught.exception.code,'head_recovery_timeout')
        self.feedback([0]*16,[.2]+[0]*15)
        with self.assertRaises(MoveError) as caught:await self.owner.dispatch('move',[0,0],.1,{})
        self.assertEqual(caught.exception.code,'head_recovery_not_stationary')
        self.reader.send_moveh.assert_awaited_once();self.assertIsNotNone(self.owner.active)

    async def test_concurrent_recovery_and_stop_cannot_clear_or_send(self):
        await self.blocked();entered=asyncio.Event();release=asyncio.Event()
        async def check():entered.set();await release.wait()
        self.reader.head_control_status=check
        pending=asyncio.create_task(self.owner.dispatch('move',[0,0],.1,{}))
        await entered.wait()
        with self.assertRaises(MoveError) as caught:await self.owner.dispatch('move',[0,0],.1,{})
        self.assertEqual(caught.exception.code,'head_admission_busy')
        await self.owner.close();release.set()
        with self.assertRaises(MoveError):await pending
        self.reader.send_moveh.assert_awaited_once();self.assertIsNotNone(self.owner.active)

    async def test_duplicate_vendor_timestamp_cannot_complete(self):
        await self.dispatch();self.pump([0]*14+[.1,-.1],advance=False)
        await self.owner.task
        self.assertEqual(self.owner.last['status'],'error');self.assertIsNotNone(self.owner.active)

    async def test_head_not_moving_keeps_real_error_and_targets_in_health_and_next_rejection(self):
        await self.dispatch();self.pump([0]*16);await self.owner.task
        summary=self.owner.summary()
        self.assertEqual(summary['status'],'error')
        self.assertEqual(summary['error_code'],'target_not_reached')
        self.assertTrue(summary['vendor_acknowledged'])
        self.assertTrue(summary['callback_delivered'])
        self.assertFalse(summary['arrival_verified'])
        self.assertEqual(summary['state'],'blocked_requires_operator_check')
        self.assertAlmostEqual(summary['target_pitch_deg'],-math.degrees(.1))
        self.assertEqual(summary['current_pitch_deg'],0)
        with self.assertRaises(ValueError):await self.owner.dispatch('move',[5,0],1,{})
        self.assertEqual(self.owner.active['action_id'],summary['action_id'])
        self.reader.send_moveh.assert_awaited_once()
        self.reader.joints_at-=100
        self.assertFalse(self.owner.summary()['feedback_available'])

    async def test_failed_task_reports_customer_signs_without_changing_raw_joint_feedback(self):
        await self.dispatch();self.pump([0]*14+[.05,-.02]);await self.owner.task
        summary=self.owner.summary()
        self.assertEqual(summary['status'],'error')
        self.assertAlmostEqual(summary['target_pitch_deg'],-math.degrees(.1))
        self.assertAlmostEqual(summary['current_pitch_deg'],-math.degrees(.05))
        self.assertAlmostEqual(summary['current_yaw_deg'],math.degrees(-.02))
        self.assertEqual(summary['target_vendor_rad'],[.1,-.1])
        self.assertEqual(summary['current_vendor_rad'],[.05,-.02])
        self.assertEqual(self.reader.sample('joint_state')['data']['q'][14:],[.05,-.02])
        self.assertIn('俯仰-5.73°',summary['error'])
        self.assertIn('俯仰-2.86°',summary['error'])

    async def test_moving_velocity_or_wrong_target_cannot_complete(self):
        await self.dispatch();self.pump([0]*14+[.1,-.1],dq=[0]*14+[1,0])
        await self.owner.task
        self.assertFalse(self.owner.last['arrival_verified'])

    async def test_named_feedback_maps_head_correctly(self):
        names=list(reversed(JOINT_NAMES))
        self.feedback(list(reversed([0]*14+[.1,-.1])),names=names)
        self.assertEqual(self.owner.feedback()[1],[.1,-.1])

    async def test_missing_velocity_stale_feedback_and_bad_names_reject_before_send(self):
        for change in ('missing_dq','stale','bad_names'):
            self.feedback([0]*16)
            if change=='missing_dq':self.reader.joints['dq']=[]
            elif change=='stale':self.reader.joints_at-=100
            else:self.reader.joints['names']=['j'+str(i) for i in range(16)]
            with self.assertRaises(ValueError):await self.dispatch()
        self.reader.send_moveh.assert_not_called()

    async def test_unknown_ack_is_not_retried_or_declared_complete_by_position(self):
        self.reader.send_moveh.side_effect=MoveError('unknown ACK',True)
        result=await self.dispatch();self.pump([0]*14+[.1,-.1]);await self.owner.task
        self.assertEqual(result['status'],'ack_unverified')
        self.assertEqual(self.owner.last['status'],'error')
        self.reader.send_moveh.assert_awaited_once();self.assertIsNotNone(self.owner.active)

    async def test_vendor_rejection_does_not_latch_or_report_success(self):
        self.reader.send_moveh.side_effect=MoveError('reject',True,'vendor_rejected')
        with self.assertRaises(MoveError):await self.dispatch()
        self.assertIsNone(self.owner.active);self.report.assert_not_called()

    async def test_callback_failure_or_reconnect_retains_owner(self):
        self.report.return_value=False
        await self.dispatch();self.pump([0]*14+[.1,-.1]);await self.owner.task
        self.assertEqual(self.owner.last['status'],'completed')
        self.assertFalse(self.owner.last['callback_delivered']);self.assertIsNotNone(self.owner.active)

    async def test_connection_replacement_cannot_confirm(self):
        await self.dispatch();self.reader.ws=types.SimpleNamespace(closed=False)
        self.pump([0]*14+[.1,-.1]);await self.owner.task
        self.assertFalse(self.owner.last['arrival_verified'])

    async def test_head_arm_and_grippers_are_mutually_exclusive(self):
        adapter=Adapter(lambda *a:None,settings(enabled_cards=['joint_state','arm_control','move_head','gripper_left','gripper_right']),reader=self.reader)
        adapter.head.report=self.report
        await adapter.head.dispatch('move',[.1,-.1],.1,{})
        try:
            with self.assertRaises(MoveError):await adapter.motion.dispatch('move',[0]*14,.1,{})
            for owner in adapter.grippers.values():
                with self.assertRaises(MoveError):await owner.dispatch('open',{'opening_mm':20,'speed':10,'force':10})
            self.reader.send_movej.assert_not_called()
            await adapter.head.close()
            adapter.head.closed=False;adapter.head.active=None
            adapter.motion.active={'state':'moving'}
            with self.assertRaises(MoveError):await adapter.head.dispatch('move',[.1,-.1],.1,{})
            adapter.motion.active=None
            for gripper in adapter.grippers.values():
                gripper.recovering=True
                with self.assertRaises(MoveError):await adapter.head.dispatch('move',[.1,-.1],.1,{})
                gripper.recovering=False
        finally:
            await adapter.head.close()


class HeadWireTests(unittest.IsolatedAsyncioTestCase):
    async def test_real_mcp_websocket_and_acp_use_head_only_and_exact_correlation(self):
        requests=[];ack_mode=['success'];stamp=[100];head_q=[0.0,0.0];completions=[];vr_follow=[False]
        async def robot(request):
            ws=web.WebSocketResponse();await ws.prepare(request);await ws.send_json(info())
            async for msg in ws:
                body=json.loads(msg.data);requests.append(body)
                if body['title']=='request_get_joint_state':
                    stamp[0]+=1
                    await ws.send_json({'title':'response_get_joint_state','accid':'TEST_TRON2A','guid':body['guid'],
                        'data':joints(q=[0]*14+head_q,timestamp=stamp[0])})
                elif body['title']=='request_moveh':
                    ack={'title':'response_moveh','accid':'TEST_TRON2A','guid':body['guid'],'data':{'result':'success'}}
                    await ws.send_json({**ack,'guid':'wrong'})
                    await ws.send_json({**ack,'accid':'wrong'})
                    await ws.send_json({**ack,'title':'response_movej'})
                    if ack_mode[0]=='success':
                        head_q[:]=body['data']['joint'];await ws.send_json(ack)
                    elif ack_mode[0]=='reject':
                        await ws.send_json({**ack,'data':{'result':'fail_motor'}})
            return ws
        async def complete(request):
            payload=await request.json();completions.append(payload)
            return web.json_response({'ok':True,'action_id':payload['action_id']})
        async def control_status(request):
            return web.json_response({'enable_vr_follow_head':vr_follow[0],'enable_vr_follow_head_valid':True,
                                      'working_mode':'developer_mode','working_mode_valid':True})
        app=web.Application();app.router.add_get('/robot',robot);app.router.add_post('/api/acp/complete',complete)
        app.router.add_get('/get_robot_info',control_status)
        runner=web.AppRunner(app,shutdown_timeout=.2);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        port=site._server.sockets[0].getsockname()[1]
        api_patch=patch('vendor.HEAD_API_PORT',port);api_patch.start()
        async with aiohttp.ClientSession(trust_env=False,timeout=aiohttp.ClientTimeout(total=5)) as session:
            reader=RobotReader(settings(url='ws://127.0.0.1:'+str(port)+'/robot',poll_interval_seconds=.01,response_timeout_seconds=.3))
            adapter=Adapter(lambda *a:None,settings(enabled_cards=['joint_state','move_head']),reader=reader,session=session)
            app2=web.Application();app2.router.add_post('/mcp',adapter.rpc)
            run2=web.AppRunner(app2,shutdown_timeout=.2);await run2.setup();site2=web.TCPSite(run2,'127.0.0.1',0);await site2.start()
            task=asyncio.create_task(reader.run(session))
            try:
                for _ in range(100):
                    if reader.sample('joint_state')['available']:break
                    await asyncio.sleep(.01)
                self.assertFalse(any(p['title']=='request_moveh' for p in requests))
                for action in ('info','start','stop'):
                    self.assertFalse(adapter.call('move_head',{'action':action})['motion_sent'])
                # Real MCP admission: invalid input never sends WS, creates an ACP task,
                # or acquires ownership. The following valid call must still complete.
                bad_inputs=[{'pitch_deg':0,'yaw_deg':90,'time':10},
                    {'pitch_deg':0,'yaw_deg':-90,'time':10},
                    {'pitch_deg':46,'yaw_deg':0,'time':10},
                    {'pitch_deg':'5','yaw_deg':0}, {'pitch_deg':True,'yaw_deg':0},
                    {'pitch_deg':0}, {'pitch_deg':0,'yaw_deg':0,'time':0},
                    {'pitch_deg':0,'yaw_deg':0,'unexpected':1}]
                for bad in bad_inputs:
                    async with session.post('http://127.0.0.1:'+str(site2._server.sockets[0].getsockname()[1])+'/mcp',json={
                        'jsonrpc':'2.0','id':6,'method':'tools/call','params':{'name':'move_head','arguments':bad}}) as response:
                        invalid=(await response.json())['result'];self.assertTrue(invalid['isError'])
                        rejection=json.loads(invalid['content'][0]['text'])
                        self.assertFalse(rejection['motion_sent']);self.assertNotIn('action_id',rejection)
                    self.assertIsNone(adapter.head.active);self.assertIsNone(adapter.head.task)
                    self.assertEqual(completions,[])
                self.assertFalse(any(p['title']=='request_moveh' for p in requests))
                with patch.dict('os.environ',{'AGENT_CORE_URL':'http://127.0.0.1:'+str(port)}),patch('completion.STABILITY_SECONDS',.05):
                    async with session.post('http://127.0.0.1:'+str(site2._server.sockets[0].getsockname()[1])+'/mcp',json={
                        'jsonrpc':'2.0','id':7,'method':'tools/call','params':{'name':'move_head','arguments':{
                            'pitch_deg':10,'yaw_deg':12,'time':.1,'_trace_id':'trace','instance_id':'card'}}}) as response:
                        result=(await response.json())['result'];self.assertFalse(result['isError'])
                        admitted=json.loads(result['content'][0]['text']);self.assertFalse(admitted['arrival_verified'])
                    await asyncio.wait_for(adapter.head.task,2)
                writes=[p for p in requests if p['title']!='request_get_joint_state']
                self.assertEqual(len(writes),1);self.assertEqual(writes[0]['title'],'request_moveh')
                self.assertEqual(writes[0]['data'],{'joint':[math.radians(-10),math.radians(12)],'time':.1})
                self.assertNotIn('_trace_id',writes[0]);self.assertNotIn('instance_id',writes[0])
                self.assertEqual(completions[0]['tool'],'move_head');self.assertEqual(completions[0]['status'],'completed')
                self.assertEqual(completions[0]['action_id'],admitted['action_id'])
                self.assertTrue(completions[0]['result']['head_target_verified'])
                self.assertEqual(completions[0]['result']['coordinate_convention'],COORDINATE_CONVENTION)
                self.assertEqual(completions[0]['result']['target_pitch_deg'],10)
                self.assertEqual(completions[0]['result']['reported_pitch_deg'],10)
                self.assertAlmostEqual(completions[0]['result']['reported_yaw_deg'],12)
                feedback=adapter.call('move_head',{'action':'info'})
                self.assertEqual(feedback['pitch_deg'],10)
                self.assertEqual(feedback['vendor_joint_rad'],[math.radians(-10),math.radians(12)])
                self.assertIsNone(adapter.head.active)
                vr_follow[0]=True
                before=len([p for p in requests if p['title']=='request_moveh'])
                with self.assertRaises(MoveError) as caught:
                    await adapter.head.dispatch('move',[0,0],1,{})
                self.assertEqual(caught.exception.code,'head_vr_follow_enabled')
                self.assertFalse(caught.exception.motion_sent)
                self.assertIsNone(adapter.head.active)
                self.assertEqual(before,len([p for p in requests if p['title']=='request_moveh']))
                vr_follow[0]=False
                ack_mode[0]='reject'
                with self.assertRaisesRegex(MoveError,'rejected'):await reader.send_moveh([0,0],1)
                ack_mode[0]='timeout'
                with self.assertRaisesRegex(MoveError,'unconfirmed'):await reader.send_moveh([0,0],1)
                self.assertIsNone(reader.head_pending)
            finally:
                await adapter.head.close()
                if reader.ws is not None:await reader.ws.close()
                task.cancel();await asyncio.wait_for(asyncio.gather(task,return_exceptions=True),2)
                await run2.cleanup()
                api_patch.stop()
        await runner.cleanup()

    async def test_unverified_follow_state_blocks_without_any_motor_write(self):
        mode=[{'enable_vr_follow_head':True,'enable_vr_follow_head_valid':True,
               'working_mode':'developer_mode','working_mode_valid':True}]
        async def status(request):return web.json_response(mode[0])
        app=web.Application();app.router.add_get('/get_robot_info',status)
        runner=web.AppRunner(app);await runner.setup();site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        reader=RobotReader(settings());reader.accept_info(info())
        reader.ws=types.SimpleNamespace(closed=False,send_json=AsyncMock())
        try:
            with patch('vendor.HEAD_API_PORT',site._server.sockets[0].getsockname()[1]):
                for state,code in ((mode[0],'head_vr_follow_enabled'),
                                   ({},'head_control_status_unavailable'),
                                   ({**mode[0],'enable_vr_follow_head':False,'working_mode':'controller_mode'},'vendor_not_ready')):
                    mode[0]=state
                    with self.assertRaises(MoveError) as caught:await reader.send_moveh([0,0],1)
                    self.assertEqual(caught.exception.code,code)
                    self.assertFalse(caught.exception.motion_sent)
            reader.ws.send_json.assert_not_called()
        finally:await runner.cleanup()

    async def test_head_mode_query_handles_chunked_body_and_rejects_redirects_or_oversize(self):
        variant=['chunked']
        async def status(request):
            if variant[0]=='redirect':raise web.HTTPFound('/unexpected')
            if variant[0]=='large':return web.Response(body=b' '*131073)
            response=web.StreamResponse();await response.prepare(request)
            raw=json.dumps({'enable_vr_follow_head':False,'enable_vr_follow_head_valid':True,
                            'working_mode':'developer_mode','working_mode_valid':True}).encode()
            await response.write(raw[:20]);await asyncio.sleep(.01)
            await response.write(raw[20:]);await response.write_eof();return response
        app=web.Application();app.router.add_get('/get_robot_info',status)
        runner=web.AppRunner(app);await runner.setup();site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        reader=RobotReader(settings())
        try:
            with patch('vendor.HEAD_API_PORT',site._server.sockets[0].getsockname()[1]):
                self.assertEqual(await reader.head_control_status(),{'vr_head_follow':False,'source':'fresh_vendor_get_robot_info'})
                for value in ('redirect','large'):
                    variant[0]=value
                    with self.assertRaises(MoveError):await reader.head_control_status()
        finally:await runner.cleanup()

    async def test_transport_preconditions_never_send_or_toggle_modes(self):
        reader=RobotReader(settings());reader.accept_info(info())
        reader.ws=types.SimpleNamespace(closed=False,send_json=AsyncMock())
        for key,value in (('working_mode','idle'),('collision_enable',False),('compliant_enable',True)):
            reader.accept_info(info());reader.info[key]=value
            with self.assertRaises(MoveError):await reader.send_moveh([0,0],1)
        reader.accept_info(info());reader.info_at-=100
        with self.assertRaises(MoveError):await reader.send_moveh([0,0],1)
        reader.ws.send_json.assert_not_called()
