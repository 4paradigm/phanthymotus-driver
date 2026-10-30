"""Offline generic-head adapter tests: exact native guards, no real motors."""
import asyncio
import copy
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from device import Adapter
from head_stream import HeadStream, descriptor, STATE_TOPIC
from state import joint_state
from test_sensors import settings, info, joints
from vendor import MoveError

class Publisher:
    def __init__(self):self.callback=None;self.sent=[];self.unbound=[]
    def publish(self,*args):self.sent.append(args)
    def subscribe_json(self,topic,callback):self.callback=callback;return topic
    def unsubscribe(self,sub):self.unbound.append(sub);self.callback=None

class Tests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.pub=Publisher();self.adapter=Adapter(self.pub.publish,settings(enabled_cards=['joint_state','move_head']))
        self.reader=self.adapter.reader;self.reader.accept_info(info())
        self.reader.ws=SimpleNamespace(closed=False,send_json=AsyncMock())
        self.feedback()
        self.stream=HeadStream(self.adapter,self.pub);self.adapter.head_stream=self.stream
        self.adapter.report_head_completion=AsyncMock(return_value=True)
        self.adapter.head.report=AsyncMock(return_value=True)
        self.reader.head_control_status=AsyncMock(return_value={'vr_head_follow':False})
        self.reader.send_movej=AsyncMock();self.reader.send_gripper=AsyncMock()
    def feedback(self,head=None,age=0):
        q=[0.]*14+([0.,0.] if head is None else list(head))
        self.reader.joints=joint_state(joints(q=q,dq=[0.]*16,timestamp=time.monotonic()))
        self.reader.joints_at=time.monotonic()-age;self.reader.joints_received=time.time()-age
    async def asyncTearDown(self):
        await self.stream.close();await self.adapter.head.close();await self.adapter.motion.close()
        if self.stream.task:await asyncio.gather(self.stream.task,return_exceptions=True)
    async def start(self):return await self.stream.call({'action':'start','input_topic':'/actucore/headfollow/cmd'})
    def message(self,**updates):
        now=int(time.time()*1000)
        s=self.stream.state()
        out={'schema':'motus.control/1','seq':1,'stamp_ms':now,'obs_stamp_ms':now-10,'ttl_ms':500,
             'source':'test-headfollow','priority':50,'mode':'joint_position','dof':2,'values':[.01,-.02],
             'duration_s':1.,'head_session':self.stream.session,
             'reference':{'sample_stamp':s['sample_stamp'],'values':list(s['values']),'motion_id':s['motion_id']}}
        out.update(updates);return out
    async def drain(self):
        if self.stream.task:await asyncio.wait_for(asyncio.shield(self.stream.task),2)
    def no_sends(self):
        self.reader.ws.send_json.assert_not_awaited();self.reader.send_movej.assert_not_awaited();self.reader.send_gripper.assert_not_awaited()
    async def test_no_yolo_or_follow_card_and_original_manual_head_fields(self):
        names={t['name'] for t in self.adapter.tools()}
        self.assertNotIn('yolo_detect',names);self.assertNotIn('head_visual_follow',names)
        tool=next(t for t in self.adapter.tools() if t['name']=='move_head')
        self.assertEqual(set(tool['inputSchema']['properties']),{'pitch_deg','yaw_deg','time'})
        self.assertEqual(tool['topic_in'][0]['format'],'control/joint')
        self.assertEqual(tool['topic_out'][0]['topic'],STATE_TOPIC)
        self.no_sends()
    async def test_descriptor_available_offline_without_motion(self):
        self.reader.ws=None;self.reader.connected=False
        r=self.adapter.call('move_head',{'action':'info'})
        self.assertEqual(r['control_interface'],descriptor());self.assertFalse(r['available'])
    async def test_start_stop_info_never_move_and_inputs_are_explicit(self):
        self.assertFalse((await self.start())['motion_sent'])
        self.assertTrue(self.stream.running)
        old=self.pub.callback;await self.stream.call({'action':'stop'})
        old(json.dumps(self.message()));await asyncio.sleep(0)
        self.assertIsNone(self.stream.task);self.no_sends()
    async def test_invalid_multiple_or_non_topic_inputs_fail_before_subscription(self):
        for args in ({'input_topics':['/a','/b']},{'input_topic':'http://robot'},{'input_topic':'/x;rm'}):
            with self.assertRaises(ValueError):await self.stream.call({'action':'start',**args})
        self.assertIsNone(self.pub.callback);self.no_sends()
    async def test_fresh_target_uses_original_head_dispatch_and_native_conversion(self):
        await self.start();m=self.message()
        self.adapter.head.dispatch=AsyncMock(return_value={'action_id':'a1','motion_sent':True})
        self.stream.offer(m,self.stream.epoch);await self.drain()
        args=self.adapter.head.dispatch.await_args.args
        self.assertEqual(args[0],'move');self.assertEqual(args[1],m['values']);self.assertEqual(args[2],1.)
        self.assertEqual(args[3]['head_stream'],{'epoch':self.stream.epoch,'seq':1,'source':m['source']})
        self.assertEqual(self.stream.request['outcome'],'accepted');self.no_sends()
    async def test_stale_future_wrong_mode_bad_bounds_nan_and_speed_rejected(self):
        await self.start()
        patches=[{'stamp_ms':0},{'stamp_ms':int(time.time()*1000)+1000},{'obs_stamp_ms':0},
                 {'ttl_ms':600},{'head_session':'other'},{'mode':'twist'},{'dof':14},
                 {'reference':['invalid']},{'values':[10,0]},{'values':[float('nan'),0]},{'values':[.5,0],'duration_s':.6}]
        for updates in patches:
            self.stream.offer(self.message(**updates),self.stream.epoch)
            await asyncio.sleep(0)
            self.assertIsNone(self.stream.task)
        self.no_sends()
    async def test_changed_reference_or_other_head_owner_rejected(self):
        await self.start();m=self.message();self.feedback([.1,0])
        self.stream.offer(m,self.stream.epoch);self.assertIsNone(self.stream.task)
        self.no_sends()
    async def test_stale_feedback_never_admits(self):
        await self.start();m=self.message();self.feedback(age=2)
        self.stream.offer(m,self.stream.epoch);self.assertIsNone(self.stream.task);self.no_sends()
    async def test_busy_and_blocked_owner_do_not_trigger_automatic_recovery(self):
        await self.start();self.adapter.head.active={'action_id':'old','action':'move','state':'blocked_requires_operator_check',
            'target':[0.,0.],'duration':1,'details':{}}
        m=self.message();self.stream.offer(m,self.stream.epoch)
        self.assertIsNone(self.stream.task);self.no_sends();self.adapter.head.active=None
    async def test_duplicate_accepted_seq_is_not_resent_or_changed_to_error(self):
        await self.start();self.adapter.head.dispatch=AsyncMock(return_value={'motion_sent':True,'action_id':'one'})
        m=self.message();self.stream.offer(m,self.stream.epoch);await self.drain()
        before=copy.deepcopy(self.stream.request);self.stream.offer(m,self.stream.epoch);await asyncio.sleep(0)
        self.assertEqual(self.stream.request,before);self.assertEqual(self.adapter.head.dispatch.await_count,1)
    async def test_other_source_cannot_take_over_owned_stream(self):
        await self.start();self.adapter.head.dispatch=AsyncMock(return_value={'motion_sent':True,'action_id':'one'})
        self.stream.offer(self.message(),self.stream.epoch);await self.drain()
        self.stream.offer(self.message(source='other',seq=2),self.stream.epoch)
        self.assertEqual(self.adapter.head.dispatch.await_count,1)
    async def test_stop_between_offer_and_task_run_blocks_send(self):
        await self.start();m=self.message();self.stream.offer(m,self.stream.epoch)
        await self.stream.call({'action':'stop'});await self.drain()
        self.assertEqual(self.stream.request['outcome'],'failed');self.no_sends()
    async def test_expiry_after_readonly_mode_probe_blocks_native_wire_send(self):
        await self.start();m=self.message()
        async def mode():
            self.stream.message['stamp_ms']-=1000
            return {'vr_head_follow':False}
        self.reader.head_control_status=AsyncMock(side_effect=mode)
        self.stream.offer(m,self.stream.epoch);await self.drain()
        self.assertEqual(self.stream.request['outcome'],'failed');self.no_sends()
    async def test_stop_during_readonly_mode_probe_blocks_native_wire_send(self):
        await self.start();m=self.message()
        async def mode():
            await self.stream.call({'action':'stop'})
            return {'vr_head_follow':False}
        self.reader.head_control_status=AsyncMock(side_effect=mode)
        self.stream.offer(m,self.stream.epoch);await self.drain()
        self.assertEqual(self.stream.request['outcome'],'failed');self.no_sends()
    async def test_native_one_send_arrival_callback_and_feedback_completion(self):
        await self.start();m=self.message()
        async def wire(payload):
            self.assertEqual(payload['title'],'request_moveh')
            self.feedback(payload['data']['joint'])
            self.reader.head_pending[1].set_result({'result':'success'})
        self.reader.ws.send_json=AsyncMock(side_effect=wire)
        self.adapter.head.wait_for_arrival=AsyncMock(return_value={'arrival_verified':True,
            'verification':'post_duration_advancing_head_feedback','verified_samples':3})
        self.stream.offer(m,self.stream.epoch);await self.drain()
        await asyncio.wait_for(self.adapter.head.task,2)
        self.reader.ws.send_json.assert_awaited_once()
        s=self.stream.state();self.assertEqual(s['request']['outcome'],'completed');self.assertTrue(s['completion_verified'])
        self.reader.send_movej.assert_not_awaited();self.reader.send_gripper.assert_not_awaited()
    async def test_published_head_state_is_unavailable_when_feedback_stales(self):
        self.stream.publish_state();self.assertEqual(self.pub.sent[-1][0],STATE_TOPIC)
        self.feedback(age=2);self.stream.publish_state();self.assertFalse(self.pub.sent[-1][1]['available'])

class PackagingTests(unittest.TestCase):
    def test_no_detector_dependency_or_private_gpu_endpoint_in_driver(self):
        root=Path(__file__).resolve().parents[1]
        self.assertFalse((root/'yolo_detect').exists());self.assertFalse((root/'head_follow').exists())
        req=(root/'requirements.txt').read_text().lower();self.assertNotIn('pillow',req)
        for name in ('device.py','main.py','Dockerfile','driver.yaml','config.yaml','head_stream.py'):
            s=(root/name).read_text();self.assertNotIn('172.28.4.23',s);self.assertNotIn('15792',s);self.assertNotIn('15793',s)
        self.assertIn('head_stream.py',(root/'Dockerfile').read_text())

    def test_ros_input_uses_original_context_best_effort_and_shutdown_order(self):
        import types
        from unittest.mock import MagicMock
        from ros2 import JsonPublisher
        log=[]
        node=MagicMock()
        executor=MagicMock()
        executor.spin.side_effect=lambda:log.append('spin')
        executor.shutdown.side_effect=lambda:log.append('executor_shutdown')
        node.destroy_node.side_effect=lambda:log.append('destroy_node')
        qos=types.ModuleType('rclpy.qos')
        qos.QoSProfile=lambda **kw:SimpleNamespace(**kw)
        qos.ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort')
        qos.HistoryPolicy=SimpleNamespace(KEEP_LAST='keep_last')
        qos.DurabilityPolicy=SimpleNamespace(VOLATILE='volatile')
        executors=types.ModuleType('rclpy.executors')
        executors.SingleThreadedExecutor=Mock(return_value=executor)
        publisher=JsonPublisher.__new__(JsonPublisher)
        publisher.node=node;publisher.executor=publisher.spin_thread=None
        publisher.message_type=object
        publisher.rclpy=SimpleNamespace(ok=lambda:True,shutdown=lambda:log.append('rclpy_shutdown'))
        received=[]
        with patch.dict(sys.modules,{'rclpy.qos':qos,'rclpy.executors':executors}):
            sub=publisher.subscribe_json('/actucore/headfollow/cmd',received.append)
            callback=node.create_subscription.call_args.args[2]
            profile=node.create_subscription.call_args.args[3]
            callback(SimpleNamespace(data='{"schema":"test"}'))
            self.assertEqual(received,['{"schema":"test"}'])
            self.assertEqual(profile.reliability,'best_effort');self.assertEqual(profile.depth,1)
            publisher.unsubscribe(sub);publisher.close()
        executor.add_node.assert_called_once_with(node)
        self.assertLess(log.index('executor_shutdown'),log.index('destroy_node'))
        self.assertLess(log.index('destroy_node'),log.index('rclpy_shutdown'))

if __name__=='__main__':unittest.main()
