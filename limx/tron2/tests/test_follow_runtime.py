"""Wired VOP, explicit pause/reset, ownership and session ACP without hardware."""
import asyncio
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock
import aiohttp
from aiohttp import web
from device import Adapter
from state import joint_state
from test_sensors import settings, info, joints
from head_follow.runtime import Runtime
from head_follow.engine import VOP_TOPIC
from head_follow.client import Error


class Publisher:
    def __init__(self):self.callback=None;self.unbound=[]
    def publish(self,*args):pass
    def subscribe_json(self,topic,callback):self.callback=callback;return topic
    def unsubscribe(self,sub):self.unbound.append(sub);self.callback=None


class RuntimeTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.pub=Publisher();self.q=[0.]*16;self.sent=[];self.people=[[0.,0.]];self.publish=True
        self.adapter=Adapter(self.pub.publish,settings(enabled_cards=['joint_state','camera','move_head','arm_control','gripper','head_visual_follow']))
        self.reader=self.adapter.reader
        self.reader.ws=SimpleNamespace(closed=False,send_json=self.wire)
        self.reader.head_control_status=AsyncMock(return_value={'vr_head_follow':False})
        self.reader.send_movej=AsyncMock();self.reader.send_gripper=AsyncMock()
        self.adapter._report_completion=AsyncMock(return_value=True)
        self.feedback();self.runtime=Runtime(self.adapter,self.pub);self.adapter.follow_runtime=self.runtime
        self.pump=asyncio.create_task(self.pumping());self.runner=None
    def feedback(self):
        self.reader.accept_info(info())
        self.reader.joints=joint_state(joints(q=self.q[:],dq=[0.]*16,timestamp=time.monotonic()))
        self.reader.joints_at=time.monotonic();self.reader.joints_received=time.time()
    def frame(self):
        return json.dumps({'timestamp':time.time(),'latency_ms':0,'count':len(self.people),
            'objects':[{'name':'person','confidence':.9,'position':p} for p in self.people]})
    async def pumping(self):
        while True:
            self.feedback()
            if self.publish and self.pub.callback:self.pub.callback(self.frame())
            await asyncio.sleep(.03)
    async def wire(self,data):
        self.sent.append(data)
        self.assertEqual(data['title'],'request_moveh')
        self.assertEqual(len(data['data']['joint']),2)
        self.q[14:]=data['data']['joint'];self.feedback()
        self.reader.head_pending[1].set_result({'result':'success'})
    async def asyncTearDown(self):
        await self.runtime.close();await self.adapter.head.close();await self.adapter.motion.close()
        self.pump.cancel();await asyncio.gather(self.pump,return_exceptions=True)
        if self.runner:await self.runner.cleanup()
        self.reader.send_movej.assert_not_awaited();self.reader.send_gripper.assert_not_awaited()
    async def enable(self):
        return await self.runtime.call({'action':'follow','input_topic':VOP_TOPIC})
    async def await_job(self,job,timeout=8):
        await asyncio.wait_for(asyncio.shield(job['_task']),timeout)
    async def wait_sends(self,n=1):
        end=time.monotonic()+3
        while len(self.sent)<n and time.monotonic()<end:await asyncio.sleep(.03)
        self.assertGreaterEqual(len(self.sent),n,self.runtime.info())
    async def test_card_has_vop_port_and_two_public_modes_with_suboptions(self):
        t=next(t for t in self.adapter.tools() if t['name']=='head_visual_follow');s=t['inputSchema']
        self.assertEqual(t['topic_in'][0]['topic'],VOP_TOPIC)
        self.assertEqual(t['topic_in'][0]['format'],'data/json')
        self.assertEqual([v for v in s['properties']['action']['enum'] if v not in {'start','stop','info','config'}],['follow','not_follow'])
        self.assertEqual(s['properties']['non_follow_action']['enum'],['pause','reset'])
        self.assertEqual(s['x-action-params']['not_follow']['params'],['non_follow_action'])
        self.assertEqual(s['x-completion'],{'background':True,'timeout':0,'silent_completion':True})
    async def test_info_is_authoritative_without_subscribing_or_arming(self):
        r=await self.runtime.call({'action':'info'})
        self.assertEqual(r['topic_in'],[{'topic':VOP_TOPIC,'format':'data/json'}])
        self.assertFalse(r['active']);self.assertFalse(r['input_connected']);self.assertEqual(self.sent,[])
        self.assertNotIn('action_id',r)
    async def test_unwired_follow_is_rejected(self):
        with self.assertRaisesRegex(Error,'连接'):await self.runtime.call({'action':'follow'})
        self.assertEqual(self.sent,[])
    async def test_wrong_input_or_multiple_sources_are_rejected(self):
        for args in ({'input_topic':'/limx_tron2/camera_right/objects'},{'input_topics':[VOP_TOPIC,'/other']},{'input_topics':'invalid'}):
            with self.assertRaises(Error):await self.runtime.call({'action':'follow',**args})
        self.assertEqual(self.sent,[])
    async def test_lifecycle_start_only_binds_even_with_stale_reset_form_value(self):
        self.runtime.launch()
        r=await self.runtime.call({'action':'start','input_topic':VOP_TOPIC,'non_follow_action':'reset'})
        self.assertTrue(r['input_connected']);self.assertFalse(r['active']);self.assertIsNone(self.adapter.head.active)
        self.assertEqual(self.sent,[])
    async def test_duplicate_follow_reuses_session_and_head_owner(self):
        a=await self.enable();b=await self.enable()
        self.assertEqual(a['action_id'],b['action_id']);self.assertTrue(a['completion_pending'])
        self.assertIs(self.adapter.head.active,self.runtime.follow['_stream'].job)
    async def test_follow_stream_uses_short_two_axis_updates_without_arrival_dwells(self):
        self.people=[[.8,0.]];await self.enable();await self.wait_sends(4)
        self.assertTrue(all(s['data']['time']==.25 for s in self.sent))
        y=[s['data']['joint'][1] for s in self.sent]
        self.assertTrue(all(v<0 for v in y));self.assertLess(y[-1],y[0])
        self.assertEqual(self.adapter._report_completion.await_count,0)
    async def test_pause_stops_updates_and_reports_only_after_verified_settle(self):
        self.people=[[.8,0.]];a=await self.enable();await self.wait_sends(3)
        pause=await self.runtime.call({'action':'not_follow','non_follow_action':'pause'})
        n=len(self.sent);await self.await_job(self.runtime.control)
        self.assertEqual(len(self.sent),n);self.assertIsNone(self.adapter.head.active)
        calls=self.adapter._report_completion.await_args_list
        self.assertEqual(calls[0].args[1:3],(a['action_id'],'cancelled'))
        self.assertTrue(calls[0].args[3]['physical_stop_verified'])
        self.assertEqual(calls[1].args[1:3],(pause['action_id'],'completed'))
        self.assertFalse(calls[1].args[3]['motion_may_continue'])
    async def test_reset_waits_for_follow_then_uses_native_arrival_verifier(self):
        self.people=[[.8,0.]];await self.enable();await self.wait_sends(3)
        reset=await self.runtime.call({'action':'not_follow','non_follow_action':'reset'})
        await self.await_job(self.runtime.control)
        self.assertEqual(self.sent[-1]['data']['joint'],[0.,0.])
        self.assertGreaterEqual(self.sent[-1]['data']['time'],3.)
        last=self.adapter._report_completion.await_args_list[-1].args
        self.assertEqual(last[1:3],(reset['action_id'],'completed'))
        self.assertTrue(last[3]['reset_verified']);self.assertFalse(self.runtime.info()['active'])
        self.assertTrue(last[3]['head_motion']['arrival_verified'])
    async def test_pause_and_reset_do_not_require_vop_input(self):
        await self.runtime.call({'action':'not_follow','non_follow_action':'pause'});await self.await_job(self.runtime.control)
        await self.runtime.call({'action':'not_follow','non_follow_action':'reset'});await self.await_job(self.runtime.control)
        self.assertEqual(self.sent[-1]['data']['joint'],[0.,0.]);self.assertFalse(self.runtime.bound)
    async def test_reset_cancelled_during_vendor_probe_never_sends(self):
        async def mode():
            await self.runtime.call({'action':'stop'})
            return {'vr_head_follow':False}
        self.reader.head_control_status=AsyncMock(side_effect=mode)
        await self.runtime.call({'action':'not_follow','non_follow_action':'reset'})
        await self.await_job(self.runtime.control)
        self.assertEqual(self.sent,[])
        self.assertFalse(self.runtime.control['result']['reset_verified'])
        self.assertIsNone(self.adapter.head.active)
    async def test_pause_during_stream_mode_probe_is_definitely_no_send(self):
        self.people=[[.8,0.]]
        calls=0
        async def mode():
            nonlocal calls
            calls+=1
            if calls>1:await self.runtime.call({'action':'not_follow','non_follow_action':'pause'})
            return {'vr_head_follow':False}
        self.reader.head_control_status=AsyncMock(side_effect=mode)
        await self.enable();await self.await_job(self.runtime.follow)
        await self.await_job(self.runtime.control)
        self.assertEqual(self.sent,[]);self.assertFalse(self.runtime.info()['send_result_unknown'])
        self.assertIsNone(self.adapter.head.active)
    async def test_pause_remains_available_with_bad_stale_topic_metadata(self):
        await self.enable()
        await self.runtime.call({'action':'not_follow','non_follow_action':'pause','input_topic':'/wrong/stale'})
        await self.await_job(self.runtime.control)
        self.assertIsNone(self.adapter.head.active);self.assertEqual(self.sent,[])
    async def test_arm_change_faults_stream_and_blocks_automatic_reset(self):
        await self.enable();self.q[0]=.03;self.feedback()
        await self.await_job(self.runtime.follow)
        self.assertEqual(self.runtime.follow['status'],'error');self.assertEqual(self.sent,[])
        await self.runtime.call({'action':'not_follow','non_follow_action':'reset'});await self.await_job(self.runtime.control)
        self.assertEqual(self.sent,[]);self.assertIsNotNone(self.adapter.head.active)
    async def test_manual_arm_and_head_actions_cannot_overlap_tracking_owner(self):
        await self.enable()
        with self.assertRaises(ValueError):await self.adapter.execute_tool('move_head',{'action':'move','pitch_deg':0.,'yaw_deg':5.,'time':1.})
        with self.assertRaises(ValueError):await self.adapter.execute_tool('arm_control',{'action':'move','joint':[0.]*14,'time':1.})
        self.assertEqual(self.sent,[])
    async def test_callback_failure_retains_owner_and_blocks_reenable(self):
        await self.enable();self.adapter._report_completion.return_value=False
        await self.runtime.call({'action':'stop'});await self.await_job(self.runtime.follow)
        self.assertEqual(self.runtime.follow['state'],'blocked_reporting');self.assertIsNotNone(self.adapter.head.active)
        with self.assertRaises(Error):await self.enable()
    async def test_unknown_ack_is_not_retried_or_auto_reset(self):
        async def no_ack(data):self.sent.append(data)
        self.reader.ws.send_json=no_ack;self.reader.settings['response_timeout_seconds']=.1
        self.people=[[.8,0.]];await self.enable();await self.await_job(self.runtime.follow)
        self.assertEqual(len(self.sent),1);self.assertTrue(self.runtime.info()['send_result_unknown'])
        await self.runtime.call({'action':'not_follow','non_follow_action':'reset'});await self.await_job(self.runtime.control)
        self.assertEqual(len(self.sent),1)
    async def test_invalid_input_never_becomes_no_person_or_home(self):
        self.people=[[.8,0.]];await self.enable();await self.wait_sends(3)
        self.publish=False;self.pub.callback('invalid-json')
        await asyncio.sleep(.25);n=len(self.sent);await asyncio.sleep(.25)
        self.assertEqual(len(self.sent),n);self.assertEqual(self.runtime.policy.state,'input_wait')
        self.assertFalse(any(s['data']['joint']==[0.,0.] for s in self.sent))
    async def test_stop_during_input_preparation_cancels_enable(self):
        self.publish=False
        task=asyncio.create_task(self.enable());await asyncio.sleep(.06)
        await self.runtime.call({'action':'stop'})
        with self.assertRaises(Error):await task
        self.assertFalse(self.runtime.info()['active']);self.assertEqual(self.sent,[])
    async def test_previous_subscription_cannot_rearm_after_disconnect(self):
        await self.runtime.call({'action':'start','input_topic':VOP_TOPIC});old=self.pub.callback
        await self.runtime.call({'action':'stop'});self.publish=False
        await self.runtime.call({'action':'start','input_topic':VOP_TOPIC});old(self.frame())
        self.assertFalse(self.runtime.detections()['active']);self.assertEqual(self.sent,[])
    async def test_real_mcp_info_reports_the_same_resolved_input_port(self):
        app=web.Application();app.router.add_post('/mcp',self.adapter.rpc)
        self.runner=web.AppRunner(app);await self.runner.setup();site=web.TCPSite(self.runner,'127.0.0.1',0);await site.start()
        async with aiohttp.ClientSession() as session:
            async with session.post('http://127.0.0.1:%d/mcp'%site._server.sockets[0].getsockname()[1],json={
                'jsonrpc':'2.0','id':1,'method':'tools/call','params':{'name':'head_visual_follow','arguments':{'action':'info'}}}) as r:body=await r.json()
        self.assertFalse(body['result']['isError'])
        data=json.loads(body['result']['content'][0]['text'])
        self.assertEqual(data['topic_in'][0]['topic'],VOP_TOPIC);self.assertFalse(data['active']);self.assertEqual(self.sent,[])


class PackagingTests(unittest.TestCase):
    def test_scope_does_not_add_dependencies_or_a_service(self):
        root=Path(__file__).resolve().parents[1]
        self.assertEqual((root/'requirements.txt').read_bytes(),b'aiohttp>=3.10.11,<4\nPyYAML>=6.0.2,<7\n')
        self.assertIn('head_follow/servo.py',(root/'Dockerfile').read_text())
        self.assertFalse((root/'head_follow/Dockerfile').exists())
        self.assertFalse((root/'yolo_detect').exists())

if __name__=='__main__':unittest.main()
