"""Real local MCP/ACP HTTP, synthetic VOP and guarded native head transport."""
import asyncio
import json
import math
import os
import time
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch
import aiohttp
from aiohttp import web
from device import Adapter
from state import joint_state
from test_sensors import settings, info, joints
from head_follow.runtime import Runtime
from head_follow.engine import VOP_TOPIC


class Bus:
    def publish(self,*args):pass
    def subscribe_json(self,topic,callback):self.callback=callback;return topic
    def unsubscribe(self,sub):self.callback=None


class NativeFollowEndToEndTests(unittest.IsolatedAsyncioTestCase):
    async def test_wired_stream_pause_reset_and_parent_acp_over_real_http(self):
        bus=Bus();q=[0.]*16;sent=[];callbacks=[];pending=set();people=[[.8,0.]]
        async def complete(request):
            data=await request.json();self.assertIn(data['action_id'],pending)
            callbacks.append(data)
            return web.json_response({'ok':True,'action_id':data['action_id']})
        acp_app=web.Application();acp_app.router.add_post('/api/acp/complete',complete)
        acp=web.AppRunner(acp_app);await acp.setup();site=web.TCPSite(acp,'127.0.0.1',0);await site.start()
        core_url='http://127.0.0.1:%s'%site._server.sockets[0].getsockname()[1]
        pump=None;runtime=None;adapter=None;mcp=None
        try:
            async with aiohttp.ClientSession(trust_env=False) as session:
                adapter=Adapter(bus.publish,settings(enabled_cards=['joint_state','camera','move_head','head_visual_follow']),session=session)
                reader=adapter.reader
                def feedback():
                    reader.accept_info(info());reader.joints=joint_state(joints(q=q[:],dq=[0.]*16,timestamp=time.monotonic()))
                    reader.joints_at=time.monotonic();reader.joints_received=time.time()
                async def wire(data):
                    self.assertEqual(data['title'],'request_moveh');self.assertEqual(len(data['data']['joint']),2)
                    pending.add(adapter.head.active['action_id']);sent.append((time.monotonic(),data))
                    q[14:]=data['data']['joint'];feedback();reader.head_pending[1].set_result({'result':'success'})
                reader.ws=SimpleNamespace(closed=False,send_json=wire)
                reader.head_control_status=AsyncMock(return_value={'vr_head_follow':False})
                reader.send_movej=AsyncMock();reader.send_gripper=AsyncMock()
                feedback();runtime=Runtime(adapter,bus);adapter.follow_runtime=runtime
                async def publish():
                    while True:
                        feedback()
                        if getattr(bus,'callback',None):
                            bus.callback(json.dumps({'timestamp':time.time(),'latency_ms':0,'count':len(people),
                                'objects':[{'name':'person','confidence':.9,'position':v} for v in people]}))
                        await asyncio.sleep(.025)
                pump=asyncio.create_task(publish())
                app=web.Application();app.router.add_post('/mcp',adapter.rpc)
                mcp=web.AppRunner(app);await mcp.setup();rpcsite=web.TCPSite(mcp,'127.0.0.1',0);await rpcsite.start()
                url='http://127.0.0.1:%s/mcp'%rpcsite._server.sockets[0].getsockname()[1]
                async def rpc(args):
                    async with session.post(url,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                            'params':{'name':'head_visual_follow','arguments':args}}) as response:body=await response.json()
                    self.assertFalse(body['result']['isError'],body)
                    result=json.loads(body['result']['content'][0]['text'])
                    if result.get('action_id'):pending.add(result['action_id'])
                    return result
                with patch.dict(os.environ,AGENT_CORE_URL=core_url):
                    follow=await rpc({'action':'follow','input_topic':VOP_TOPIC})
                    self.assertTrue(follow['completion_pending']);self.assertEqual(callbacks,[])
                    started=time.monotonic()
                    while len(sent)<5 and time.monotonic()-started<3:await asyncio.sleep(.03)
                    self.assertGreaterEqual(len(sent),5,runtime.info())
                    # Move the same measured person smoothly through the image.
                    for x in (.6,.4,.2,0.,-.2,-.4,-.6,-.8):
                        people[:]=[[x,0.]];await asyncio.sleep(.15)
                    await asyncio.sleep(.9)
                    yaw=[math.degrees(s['data']['joint'][1]) for _,s in sent]
                    self.assertTrue(any(b<a for a,b in zip(yaw,yaw[1:])),yaw)
                    self.assertTrue(any(b>a for a,b in zip(yaw,yaw[1:])),yaw)
                    self.assertEqual(callbacks,[])  # ACKs are not whole-session completion.
                    for (ta,a),(tb,b) in zip(sent,sent[1:]):
                        self.assertGreaterEqual(tb-ta,.095)
                        step=abs(math.degrees(b['data']['joint'][1]-a['data']['joint'][1]))
                        self.assertLessEqual(step,12*(tb-ta)+.03)
                    # Only distinct valid missing-person observations may home.
                    people[:]=[];end=time.monotonic()+5
                    while time.monotonic()<end and not any(e['event']=='home_center_observed' for e in runtime.policy.events):await asyncio.sleep(.05)
                    self.assertTrue(any(e['event']=='home_center_observed' for e in runtime.policy.events),runtime.info())
                    self.assertEqual(runtime.follow['action_id'],follow['action_id']);self.assertEqual(callbacks,[])
                    pause=await rpc({'action':'not_follow','non_follow_action':'pause'})
                    count=len(sent);await asyncio.wait_for(asyncio.shield(runtime.control['_task']),3)
                    self.assertEqual(len(sent),count);self.assertIsNone(adapter.head.active)
                    self.assertEqual([(c['action_id'],c['status']) for c in callbacks],
                        [(follow['action_id'],'cancelled'),(pause['action_id'],'completed')])
                    self.assertTrue(callbacks[0]['result']['physical_stop_verified'])
                    reset=await rpc({'action':'not_follow','non_follow_action':'reset'})
                    await asyncio.wait_for(asyncio.shield(runtime.control['_task']),6)
                    outer=callbacks[-1]
                    self.assertEqual((outer['action_id'],outer['status']),(reset['action_id'],'completed'))
                    self.assertTrue(outer['result']['reset_verified'])
                    self.assertTrue(outer['result']['head_motion']['arrival_verified'])
                    self.assertGreaterEqual(outer['result']['head_motion']['verified_samples'],3)
                    self.assertEqual(sent[-1][1]['data']['joint'],[0.,0.])
                    self.assertFalse(runtime.info()['active'])
                    reader.send_movej.assert_not_awaited();reader.send_gripper.assert_not_awaited()
                    await runtime.close();await adapter.head.close();await adapter.motion.close()
        finally:
            if pump:pump.cancel();await asyncio.gather(pump,return_exceptions=True)
            if mcp:await mcp.cleanup()
            await acp.cleanup()

if __name__=='__main__':unittest.main()
