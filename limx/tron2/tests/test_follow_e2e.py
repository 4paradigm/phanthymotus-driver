"""Synthetic VOP -> in-driver follow -> original native arrival/ACP verifier."""
import asyncio,json,time,unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock
from test_sensors import settings,info,joints
from device import Adapter
from state import joint_state
from head_follow.runtime import Runtime
from head_follow.engine import VOP_TOPIC
class Bus:
 def publish(self,*args):pass
 def subscribe_json(self,topic,cb):self.callback=cb;return topic
 def unsubscribe(self,sub):self.callback=None
async def exercise():
 bus=Bus();adapter=Adapter(bus.publish,settings(enabled_cards=['joint_state','camera','move_head','head_visual_follow']))
 reader=adapter.reader;reader.accept_info(info());q=[0.]*16;sent=[]
 def feedback():
  reader.accept_info(info());reader.joints=joint_state(joints(q=q[:],dq=[0.]*16,timestamp=time.monotonic()))
  reader.joints_at=time.monotonic();reader.joints_received=time.time()
 async def wire(data):
  assert data['title']=='request_moveh'
  sent.append(data);q[14:]=data['data']['joint'];feedback()
  reader.head_pending[1].set_result({'result':'success'})
 reader.ws=SimpleNamespace(closed=False,send_json=wire)
 reader.head_control_status=AsyncMock(return_value={'vr_head_follow':False})
 reader.send_movej=AsyncMock();reader.send_gripper=AsyncMock();adapter.head.report=AsyncMock(return_value=True)
 feedback();runtime=Runtime(adapter,bus);adapter.follow_runtime=runtime
 await runtime.call({'action':'start'})
 def detection(people):
  bus.callback(json.dumps({'timestamp':time.time(),'latency_ms':0,'count':len(people),
   'objects':[{'name':'person','confidence':.95,'position':xy} for xy in people]}))
 detection([[.8,0.]])
 await runtime.call({'action':'follow'});runtime.launch()
 started=time.monotonic();stage='follow';completed=[]
 try:
  while time.monotonic()-started<18:
   feedback();s=await asyncio.to_thread(runtime.info)
   if s['state']=='fault':raise AssertionError(s)
   if stage=='follow' and len(sent)==1 and s['head_motion'] is None:
    completed.append(adapter.head.summary());stage='home'
   elif stage=='home' and len(sent)==2 and any(e['event']=='home_verified' for e in s['events']):
    completed.append(adapter.head.summary());break
   detection([[.8,0.]] if stage=='follow' else [])
   await asyncio.sleep(.075)
  assert len(sent)==2 and len(completed)==2,(len(sent),s,adapter.head.summary())
  assert all(s['status']=='completed' and s['arrival_verified'] and s['callback_delivered'] and
   s['verification']=='post_duration_advancing_head_feedback' and s['verified_samples']>=3 for s in completed),completed
  assert sent[0]['data']['joint'][1]<0 and sent[1]['data']['joint']==[0.,0.]
  assert reader.send_movej.await_count==reader.send_gripper.await_count==0
  return {'head_sends':len(sent),'verified_arrivals':len(completed),'arm_or_gripper_sends':0}
 finally:
  await runtime.close();await adapter.head.close();await adapter.motion.close()
class NativeFollowEndToEndTests(unittest.IsolatedAsyncioTestCase):
 async def test_vop_follow_and_home_reuse_native_advancing_feedback_and_acp(self):
  result=await exercise()
  self.assertEqual(result,{'head_sends':2,'verified_arrivals':2,'arm_or_gripper_sends':0})

if __name__=='__main__':unittest.main()
