"""VOP integration on the original TRON2 MCP/MoveH path; no hardware."""
import asyncio
import copy
import json
import math
from pathlib import Path
import sys
import time
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock, patch
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
        self.pub=Publisher()
        self.adapter=Adapter(self.pub.publish,settings(enabled_cards=['joint_state','camera','move_head','head_visual_follow']))
        self.reader=self.adapter.reader;self.reader.accept_info(info())
        self.reader.ws=SimpleNamespace(closed=False,send_json=AsyncMock())
        self.reader.head_control_status=AsyncMock(return_value={'vr_head_follow':False})
        self.reader.send_movej=AsyncMock();self.reader.send_gripper=AsyncMock()
        self.adapter.head.report=AsyncMock(return_value=True)
        self.adapter._report_completion=AsyncMock(return_value=True)
        self.q=[0.]*16;self.feedback()
        self.runtime=Runtime(self.adapter,self.pub);self.adapter.follow_runtime=self.runtime
        self.runner=None

    def feedback(self):
        self.reader.accept_info(info())
        self.reader.joints=joint_state(joints(q=self.q[:],dq=[0.]*16,timestamp=time.monotonic()))
        self.reader.joints_at=time.monotonic();self.reader.joints_received=time.time()

    async def asyncTearDown(self):
        await self.runtime.close();await self.adapter.head.close();await self.adapter.motion.close()
        if self.runner:await self.runner.cleanup()

    async def start(self):
        return await self.adapter.execute_tool('head_visual_follow',{'action':'start'})

    def packet(self,people=((.6,0.),)):
        return json.dumps({'timestamp':time.time(),'latency_ms':0,'objects':[
            {'name':'person','confidence':.9,'position':list(xy)} for xy in people]})

    async def enable(self):
        await self.start();self.pub.callback(self.packet())
        await self.adapter.execute_tool('head_visual_follow',{'action':'follow'})

    def no_sends(self):
        self.reader.ws.send_json.assert_not_awaited()
        self.reader.send_movej.assert_not_awaited();self.reader.send_gripper.assert_not_awaited()

    async def test_one_visible_follow_and_original_manual_schema_same_driver(self):
        tools={t['name']:t for t in self.adapter.tools()}
        self.assertIn('head_visual_follow',tools);self.assertNotIn('yolo_detect',tools)
        self.assertEqual(set(tools['move_head']['inputSchema']['properties']),{'pitch_deg','yaw_deg','time'})
        self.assertNotIn('topic_in',tools['move_head'])
        schema=tools['head_visual_follow']['inputSchema']
        self.assertEqual(set(schema['properties']),{'action'})
        actions=schema['properties']['action']['enum']
        self.assertEqual([a for a in actions if a not in {'start','stop','info','config'}],['follow'])
        self.assertEqual(schema['properties']['action']['default'],'follow')
        self.assertEqual(schema['required'],[])
        self.assertEqual(schema['x-action-params'],{'follow':{'params':[],'description':'寻找画面人物并跟随'}})
        self.assertEqual(schema['x-completion'],{'background':True,'timeout':0,'silent_completion':True})
        self.assertNotIn('actions',schema['x-completion'])
        self.assertEqual(tools['head_visual_follow']['topic_in'],[])
        self.assertEqual(tools['head_visual_follow']['topic_out'],[])
        self.no_sends()

    async def test_lifecycle_launch_info_and_health_never_arm_or_deadlock(self):
        self.runtime.launch();await self.start()
        for action in ('info','stop'):
            result=await asyncio.wait_for(self.adapter.execute_tool('head_visual_follow',{'action':action}),2)
            self.assertFalse(result['active'])
        await asyncio.wait_for(self.adapter.health(None),2)
        self.no_sends()

    async def test_source_must_be_head_vop_and_exactly_one(self):
        for args in ({'input_topic':'/limx_tron2/camera_left/objects'},
                     {'input_topics':[VOP_TOPIC,'/other']},{'input_topics':'bad'}):
            with self.assertRaises(Error):await self.runtime.call({'action':'start',**args})
        self.assertIsNone(self.pub.callback);self.no_sends()
        with patch('head_follow.runtime.PREPARE_SECONDS',.05),self.assertRaises(Error):
            await self.runtime.call({'action':'follow'})

    async def test_delayed_old_callback_and_invalid_payload_cannot_rearm(self):
        await self.start();old=self.pub.callback
        old(self.packet());self.assertTrue(self.runtime.detections()['active'])
        await self.runtime.call({'action':'stop'});await self.start()
        old(self.packet());self.assertFalse(self.runtime.detections()['active'])
        self.pub.callback('bad-json')
        with self.assertRaises(Error):await self.runtime.call({'action':'follow'})
        self.assertFalse(self.runtime.engine.active);self.no_sends()

    async def test_internal_move_uses_original_validation_and_axis_conversion(self):
        await self.enable()
        self.adapter.head.dispatch=AsyncMock(return_value={'motion_sent':True,'action_id':'one'})
        r=await asyncio.to_thread(self.runtime.move,2.,-6.,1.)
        self.assertEqual(r['action_id'],'one')
        action,target,duration,details=self.adapter.head.dispatch.call_args.args
        self.assertEqual((action,duration),('move',1.))
        self.assertEqual(target,[-math.radians(2),-math.radians(6)])
        self.assertIn('head_follow_guard',details)
        self.no_sends()

    async def guarded_send(self,mode):
        await self.enable();self.reader.head_control_status=AsyncMock(side_effect=mode)
        with self.assertRaises(ValueError):await asyncio.to_thread(self.runtime.move,0.,-2.,.6)
        self.no_sends()

    async def test_stop_during_mode_query_blocks_native_send(self):
        async def mode():
            await self.runtime.call({'action':'stop'})
            return {'vr_head_follow':False}
        await self.guarded_send(mode)

    async def test_expiry_during_mode_query_blocks_native_send(self):
        async def mode():
            self.runtime.send_guard['expires']-=2
            return {'vr_head_follow':False}
        await self.guarded_send(mode)

    async def test_reference_change_during_mode_query_blocks_native_send(self):
        async def mode():
            self.q[0]=.1;self.feedback()
            return {'vr_head_follow':False}
        await self.guarded_send(mode)

    async def test_arm_reference_changed_before_dispatch_blocks_send(self):
        await self.enable();self.q[0]=.1;self.feedback()
        with self.assertRaisesRegex(Error,'参考姿态'):
            await asyncio.to_thread(self.runtime.move,0.,-2.,.6)
        self.no_sends()

    async def test_eventloop_cannot_block_itself(self):
        with self.assertRaisesRegex(Error,'阻塞'):self.runtime.robot()
        self.no_sends()

    async def test_info_authoritatively_has_no_canvas_ports_or_binding(self):
        result=await self.adapter.execute_tool('head_visual_follow',{'action':'info','instance_id':'canvas','_trace_id':'inference'})
        self.assertEqual(result['topic_in'],[])
        self.assertEqual(result['topic_out'],[])
        self.assertEqual(result['detection_topic'],VOP_TOPIC)
        self.assertEqual(self.runtime.info()['topic_in'],result['topic_in'])
        self.assertFalse(self.runtime.bound);self.assertFalse(result['active'])
        self.assertIsNone(self.pub.callback);self.no_sends()

    async def test_manual_follow_subscribes_fixed_vop_without_start_or_wire(self):
        task=asyncio.create_task(self.adapter.execute_tool('head_visual_follow',{'action':'follow'}))
        await asyncio.sleep(.05)
        self.assertTrue(self.runtime.bound);self.assertIsNotNone(self.pub.callback)
        self.pub.callback(self.packet())
        result=await asyncio.wait_for(task,2)
        self.assertTrue(result['active']);self.assertEqual(result['topic_in'],[])
        self.assertEqual(result['detection_topic'],VOP_TOPIC);self.no_sends()

    async def test_stop_while_waiting_for_vop_cancels_enable(self):
        task=asyncio.create_task(self.adapter.execute_tool('head_visual_follow',{'action':'follow'}))
        await asyncio.sleep(.05);old=self.pub.callback
        await self.runtime.call({'action':'stop'});old(self.packet())
        with self.assertRaises(Error):await task
        self.assertFalse(self.runtime.engine.active);self.no_sends()

    async def test_real_loopback_mcp_exposes_follow_and_uses_lifecycle(self):
        app=web.Application();app.router.add_post('/mcp',self.adapter.rpc);app.router.add_get('/health',self.adapter.health)
        self.runner=web.AppRunner(app);await self.runner.setup()
        site=web.TCPSite(self.runner,'127.0.0.1',0);await site.start()
        url='http://127.0.0.1:%d'%site._server.sockets[0].getsockname()[1]
        async with aiohttp.ClientSession() as session:
            async def rpc(method,params):
                async with session.post(url+'/mcp',json={'jsonrpc':'2.0','id':1,'method':method,'params':params}) as r:return await r.json()
            r=await rpc('tools/list',{})
            self.assertIn('head_visual_follow',{t['name'] for t in r['result']['tools']})
            follow=next(t for t in r['result']['tools'] if t['name']=='head_visual_follow')
            self.assertEqual(follow['topic_in'],[])
            self.assertEqual(follow['topic_out'],[])
            r=await rpc('tools/call',{'name':'head_visual_follow','arguments':{'action':'info','instance_id':'canvas-inference'}})
            self.assertFalse(r['result']['isError'])
            resolved=json.loads(r['result']['content'][0]['text'])
            self.assertEqual(resolved['topic_in'],[])
            self.assertEqual(resolved['detection_topic'],VOP_TOPIC)
            self.assertFalse(self.runtime.bound);self.assertIsNone(self.pub.callback)
            r=await rpc('tools/call',{'name':'head_visual_follow','arguments':{'action':'start'}})
            self.assertFalse(r['result']['isError']);self.assertTrue(self.runtime.bound)
            async with session.get(url+'/health') as r:h=await r.json()
            self.assertFalse(h['head_visual_follow']['active'])
        self.no_sends()

    async def test_execute_without_action_and_repeated_execute_share_one_session(self):
        await self.start();self.pub.callback(self.packet())
        first,second=await asyncio.gather(self.runtime.call({}),self.runtime.call({}))
        self.assertTrue(first['action_id'].startswith('tron2_follow_'))
        self.assertEqual(first['action_id'],second['action_id'])
        self.assertTrue(first['completion_pending']);self.assertEqual(first['completion_protocol'],'acp')
        self.adapter._report_completion.assert_not_awaited();self.no_sends()

    async def test_lifecycle_diagnostics_do_not_return_top_level_session_action_id(self):
        await self.enable()
        for action in ('info','start','stop'):
            result=await self.runtime.call({'action':action})
            self.assertNotIn('action_id',result)
            self.assertIn('action_id',result['session'])
        await self.runtime.session_task
        self.assertEqual(self.adapter._report_completion.await_count,1)
        tool,aid,status,result=self.adapter._report_completion.call_args.args
        self.assertEqual((tool,status),('head_visual_follow','cancelled'))
        self.assertFalse(result['motion_sent']);self.assertFalse(result['motion_may_continue'])
        self.assertEqual(self.runtime.info()['session']['action_id'],aid)
        self.no_sends()

    async def test_stop_waits_for_already_accepted_segment_then_reports_once(self):
        await self.enable()
        self.runtime.engine.pending={'action_id':'accepted-head'}
        await self.runtime.call({'action':'stop'})
        await asyncio.sleep(.2);self.adapter._report_completion.assert_not_awaited()
        self.runtime.engine.pending=None
        await self.runtime.session_task
        self.assertEqual(self.adapter._report_completion.call_args.args[2],'cancelled')
        await self.runtime.close()
        self.assertEqual(self.adapter._report_completion.await_count,1)
        self.no_sends()

    async def test_fault_reports_error_and_does_not_claim_physical_stop_or_home(self):
        await self.enable();self.runtime.engine.fail(Error('feedback lost'))
        await self.runtime.session_task
        _,_,status,result=self.adapter._report_completion.call_args.args
        self.assertEqual(status,'error');self.assertTrue(result['motion_may_continue'])
        self.assertFalse(result['automatic_home_sent_by_completion']);self.no_sends()

    async def test_callback_failure_blocks_new_session_without_resending(self):
        await self.enable();self.adapter._report_completion.return_value=False
        await self.runtime.call({'action':'stop'});await self.runtime.session_task
        self.assertEqual(self.runtime.info()['session']['state'],'blocked_reporting')
        self.pub.callback=None
        with self.assertRaisesRegex(Error,'回调未确认'):await self.runtime.call({})
        self.assertEqual(self.adapter._report_completion.await_count,1);self.no_sends()

    async def test_settled_session_restarts_with_new_unique_id(self):
        await self.enable();first=self.runtime.session['action_id']
        await self.runtime.call({'action':'stop'});await self.runtime.session_task
        await self.start();self.pub.callback(self.packet());result=await self.runtime.call({})
        self.assertNotEqual(result['action_id'],first)
        self.assertTrue(result['active']);self.no_sends()

    async def test_home_and_missing_person_do_not_settle_live_session(self):
        await self.enable()
        self.runtime.engine.state='waiting_person';self.runtime.engine.event('home_verified')
        await asyncio.sleep(.2)
        self.assertEqual(self.runtime.info()['session']['state'],'running')
        self.adapter._report_completion.assert_not_awaited();self.no_sends()

    async def test_shutdown_preserves_truth_about_inflight_motion(self):
        await self.enable();self.runtime.engine.pending={'action_id':'accepted-head'}
        await self.runtime.close()
        _,_,status,result=self.adapter._report_completion.call_args.args
        self.assertEqual((status,result['reason']),('cancelled','driver_shutdown'))
        self.assertTrue(result['motion_may_continue'])
        self.assertEqual(self.adapter._report_completion.await_count,1);self.no_sends()

    async def test_cancelled_enable_cannot_leave_untracked_follow_active(self):
        await self.start();self.pub.callback(self.packet())
        entered,release=threading.Event(),threading.Event()
        original=self.runtime.engine.call
        def delayed(args):
            if args.get('action')=='follow':
                entered.set();release.wait(2)
            return original(args)
        with patch.object(self.runtime.engine,'call',side_effect=delayed):
            task=asyncio.create_task(self.runtime.call({}))
            await asyncio.to_thread(entered.wait,1)
            self.assertTrue(entered.is_set());task.cancel();release.set()
            with self.assertRaises(asyncio.CancelledError):await task
        self.assertFalse(self.runtime.engine.active)
        self.assertIsNone(self.runtime.session);self.no_sends()

class PackagingTests(unittest.TestCase):
    def test_no_custom_detector_generic_control_or_extra_service(self):
        root=Path(__file__).resolve().parents[1]
        for name in ('yolo_detect','head_stream.py','vision.py'):
            self.assertFalse((root/name).exists())
        self.assertEqual((root/'requirements.txt').read_bytes(),b'aiohttp>=3.10.11,<4\nPyYAML>=6.0.2,<7\n')
        for name in ('device.py','main.py','Dockerfile','driver.yaml','config.yaml','head_follow/runtime.py'):
            s=(root/name).read_text()
            for forbidden in ('172.28.4.23','15792','15793','actucore','control/joint'):
                self.assertNotIn(forbidden,s)
        self.assertFalse((root/'head_follow/Dockerfile').exists())

    def test_ros_input_uses_original_context_best_effort_and_shutdown_order(self):
        import types
        from unittest.mock import MagicMock
        from ros2 import JsonPublisher
        log=[];node=MagicMock();executor=MagicMock()
        executor.spin.side_effect=lambda:log.append('spin')
        executor.shutdown.side_effect=lambda:log.append('executor_shutdown')
        node.destroy_node.side_effect=lambda:log.append('destroy_node')
        qos=types.ModuleType('rclpy.qos');qos.QoSProfile=lambda **kw:SimpleNamespace(**kw)
        qos.ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort')
        qos.HistoryPolicy=SimpleNamespace(KEEP_LAST='keep_last');qos.DurabilityPolicy=SimpleNamespace(VOLATILE='volatile')
        executors=types.ModuleType('rclpy.executors');executors.SingleThreadedExecutor=Mock(return_value=executor)
        p=JsonPublisher.__new__(JsonPublisher);p.node=node;p.executor=p.spin_thread=None;p.message_type=object
        p.rclpy=SimpleNamespace(ok=lambda:True,shutdown=lambda:log.append('rclpy_shutdown'))
        received=[]
        with patch.dict(sys.modules,{'rclpy.qos':qos,'rclpy.executors':executors}):
            sub=p.subscribe_json(VOP_TOPIC,received.append)
            callback=node.create_subscription.call_args.args[2];profile=node.create_subscription.call_args.args[3]
            callback(SimpleNamespace(data='{}'));self.assertEqual(received,['{}'])
            self.assertEqual(profile.reliability,'best_effort');self.assertEqual(profile.depth,1)
            p.unsubscribe(sub);p.close()
        executor.add_node.assert_called_once_with(node)
        self.assertLess(log.index('executor_shutdown'),log.index('destroy_node'))
        self.assertLess(log.index('destroy_node'),log.index('rclpy_shutdown'))

if __name__=='__main__':unittest.main()
