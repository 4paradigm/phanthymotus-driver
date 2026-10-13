"""Bounded feedback completion, exclusive arm ownership, and actual ACP HTTP."""
import asyncio
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import types
import unittest
from unittest.mock import AsyncMock, patch
import aiohttp
from aiohttp import web
from test_sensors import settings, info, joints
from device import Adapter
from vendor import RobotReader, MoveError
from state import joint_state
from skeleton import JOINT_NAMES
from completion import ArmCompletion, arm_feedback

class CompletionTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.reader=RobotReader(settings()); self.reader.accept_info(info())
        self.reader.ws=types.SimpleNamespace(closed=False)
        self.feedback([0]*16)
        self.reader.send_movej=AsyncMock(return_value={'status':'accepted','motion_sent':True})
        self.report=AsyncMock(return_value=True)
        self.motion=ArmCompletion(self.reader,self.report)
        self.patches=[patch('completion.ARRIVAL_GRACE',.35),patch('completion.STABILITY_SECONDS',.05)]
        for p in self.patches: p.start()
        self.pump_task=None
    async def asyncTearDown(self):
        if self.pump_task:
            self.pump_task.cancel(); await asyncio.gather(self.pump_task,return_exceptions=True)
        await self.motion.close()
        for p in reversed(self.patches): p.stop()
    def feedback(self,q,dq=None,names=None):
        self.reader.joints=joint_state(joints(q=q,dq=[0]*16 if dq is None else dq,names=[] if names is None else names))
        self.reader.joints_at=time.monotonic(); self.reader.joints_received=time.time()
    def pump(self,q,dq=None):
        async def run():
            while True: self.feedback(q,dq); await asyncio.sleep(.02)
        self.pump_task=asyncio.create_task(run())
    async def dispatch(self): return await self.motion.dispatch('move',[.1]*14,.1,{})

    async def test_ack_is_not_completion_then_distinct_stable_feedback_completes(self):
        accepted=await self.dispatch()
        self.assertTrue(accepted['action_id'].startswith('tron2_arm_'))
        self.assertFalse(accepted['arrival_verified'])
        self.report.assert_not_called()
        with self.assertRaisesRegex(MoveError,'busy'): await self.dispatch()
        self.reader.send_movej.assert_awaited_once()
        self.pump([.1]*14+[0,0])
        await self.motion.task
        args=self.report.await_args.args
        self.assertEqual(args[0],accepted['action_id']); self.assertEqual(args[1],'completed')
        self.assertTrue(args[2]['arrival_verified']); self.assertGreaterEqual(args[2]['verified_samples'],3)
        self.assertIsNone(self.motion.active)
        second=await self.dispatch()
        self.assertNotEqual(second['action_id'],accepted['action_id'])
        await self.motion.task

    async def test_cached_target_or_off_target_cannot_complete_and_latches_owner(self):
        self.feedback([.1]*14+[0,0])
        await self.dispatch(); await self.motion.task
        self.assertEqual(self.report.await_args.args[1],'error')
        self.assertFalse(self.motion.last['arrival_verified'])
        with self.assertRaisesRegex(MoveError,'busy'): await self.dispatch()
        self.reader.send_movej.assert_awaited_once()

    async def test_fast_feedback_at_target_never_claims_stationary_arrival(self):
        await self.dispatch(); self.pump([.1]*14+[0,0],dq=[1]*16)
        await self.motion.task
        self.assertEqual(self.report.await_args.args[1],'error')

    async def test_disconnect_and_reconnect_to_same_serial_cannot_complete(self):
        accepted=await self.dispatch()
        self.reader.disconnect('test'); self.reader.accept_info(info())
        self.reader.ws=types.SimpleNamespace(closed=False)
        self.feedback([.1]*14+[0,0]); await self.motion.task
        self.assertEqual(self.report.await_args.args[:2],(accepted['action_id'],'error'))
        self.assertIsNotNone(self.motion.active)

    async def test_rejected_ack_has_no_action_id_callback_or_latched_owner(self):
        self.reader.send_movej.side_effect=MoveError('rejected',True,'vendor_rejected')
        with self.assertRaisesRegex(MoveError,'rejected'): await self.dispatch()
        self.report.assert_not_called(); self.assertIsNone(self.motion.active)

    async def test_unknown_ack_returns_tracking_id_without_resending(self):
        self.reader.send_movej.side_effect=MoveError('ACK timeout',True)
        accepted=await self.dispatch(); self.assertEqual(accepted['status'],'ack_unverified')
        self.pump([.1]*14+[0,0]); await self.motion.task
        self.reader.send_movej.assert_awaited_once()
        self.assertEqual(self.report.await_args.args[:2],(accepted['action_id'],'completed'))

    async def test_callback_failure_preserves_local_owner_and_shutdown_reports_error(self):
        self.report.return_value=False
        await self.dispatch(); self.pump([.1]*14+[0,0]); await self.motion.task
        self.assertFalse(self.motion.last['callback_delivered'])
        with self.assertRaises(MoveError): await self.dispatch()
        self.motion=ArmCompletion(self.reader,self.report)
        self.report.return_value=True
        accepted=await self.dispatch(); await self.motion.close()
        self.assertEqual(self.report.await_args.args[:2],(accepted['action_id'],'error'))
        self.assertFalse(self.report.await_args.args[2]['arrival_verified'])

    async def test_cancelled_rpc_does_not_cancel_physical_send_or_ownership(self):
        gate=asyncio.Event()
        async def send(*args): await gate.wait(); return {'status':'accepted','motion_sent':True}
        self.reader.send_movej.side_effect=send
        caller=asyncio.create_task(self.dispatch()); await asyncio.sleep(.02)
        caller.cancel(); await asyncio.gather(caller,return_exceptions=True)
        self.assertIsNotNone(self.motion.active)
        gate.set(); self.pump([.1]*14+[0,0]); await self.motion.task
        self.reader.send_movej.assert_awaited_once(); self.report.assert_awaited_once()

    async def test_named_feedback_is_reordered_before_verification(self):
        q=[i/100 for i in range(16)]
        self.feedback(q,names=list(reversed(JOINT_NAMES)))
        _,actual,_=arm_feedback(self.reader)
        self.assertEqual(actual,list(reversed(q))[:14])
        self.reader.joints_at-=10
        with self.assertRaisesRegex(ValueError,'Fresh'): await self.dispatch()
        self.reader.send_movej.assert_not_called()

class CallbackTests(unittest.IsolatedAsyncioTestCase):
    async def test_real_rpc_admission_then_real_http_callback_matches_registered_action(self):
        received=[]; terminal=asyncio.Event(); pending=set()
        async def callback(request):
            self.assertNotIn('Authorization',request.headers)
            body=await request.json(); received.append(body)
            self.assertIn(body['action_id'],pending)
            terminal.set(); return web.json_response({'ok':True,'action_id':body['action_id']})
        app=web.Application(); app.router.add_post('/api/acp/complete',callback)
        runner=web.AppRunner(app); await runner.setup()
        server=await asyncio.get_running_loop().create_server(runner.server,'127.0.0.1',0)
        async with aiohttp.ClientSession(trust_env=False) as session:
            adapter=Adapter(lambda *args:None,settings(enabled_cards=['joint_state','arm_control']),session=session)
            r=adapter.reader; r.accept_info(info()); r.ws=types.SimpleNamespace(closed=False)
            r.joints=joint_state(joints(q=[0]*16)); r.joints_at=time.monotonic(); r.joints_received=time.time()
            r.send_movej=AsyncMock(return_value={'status':'accepted','motion_sent':True})
            mcp=web.Application(); mcp.router.add_post('/mcp',adapter.rpc)
            rpc_runner=web.AppRunner(mcp); await rpc_runner.setup()
            rpc_server=await asyncio.get_running_loop().create_server(rpc_runner.server,'127.0.0.1',0)
            url='http://127.0.0.1:'+str(server.sockets[0].getsockname()[1])
            async def pump():
                while True:
                    r.joints=joint_state(joints(q=[.1]*14+[0,0]));r.joints_at=time.monotonic();r.joints_received=time.time()
                    await asyncio.sleep(.02)
            try:
                with patch.dict(os.environ,AGENT_CORE_URL=url),patch('completion.STABILITY_SECONDS',.05):
                    async with session.post('http://127.0.0.1:'+str(rpc_server.sockets[0].getsockname()[1])+'/mcp',json={
                        'jsonrpc':'2.0','id':1,'method':'tools/call','params':{'name':'arm_control','arguments':{'action':'move','joint':[.1]*14,'time':.1,'_trace_id':'test'}}}) as reply:
                        result=await reply.json()
                    admission=json.loads(result['result']['content'][0]['text'])
                    self.assertFalse(result['result']['isError']);pending.add(admission['action_id'])
                    self.assertEqual(received,[])
                    pump_task=asyncio.create_task(pump())
                    try: await asyncio.wait_for(terminal.wait(),2);await adapter.motion.task
                    finally: pump_task.cancel();await asyncio.gather(pump_task,return_exceptions=True)
                    self.assertEqual(received[0]['status'],'completed')
                    self.assertEqual(received[0]['tool'],'arm_control')
                    self.assertTrue(received[0]['result']['arrival_verified'])
                    self.assertNotIn('_trace_id',received[0]);r.send_movej.assert_awaited_once()
            finally:
                await adapter.motion.close();rpc_server.close();await rpc_server.wait_closed();await rpc_runner.cleanup()
        server.close();await server.wait_closed();await runner.cleanup()

    async def test_callback_retries_identical_payload_preserves_ssl_and_blocks_redirects(self):
        context=object();calls=[]
        class Response:
            status=503
            async def __aenter__(self): return self
            async def __aexit__(self,*args): pass
            async def json(self): return {}
        class Session:
            def post(self,url,**kwargs): calls.append((url,kwargs));return Response()
        adapter=Adapter(lambda *args:None,settings(),session=Session());adapter.core_ssl=context
        with patch('device.asyncio.sleep',new_callable=AsyncMock):
            self.assertFalse(await adapter.report_completion('id','error',{'arrival_verified':False}))
        self.assertEqual(len(calls),3)
        for _,kwargs in calls:
            self.assertIs(kwargs['ssl'],context);self.assertFalse(kwargs['allow_redirects'])
            self.assertEqual(kwargs['timeout'].total,3)
            self.assertEqual(kwargs['json'],calls[0][1]['json'])
            self.assertNotIn('headers',kwargs)

    async def test_completion_schema_and_actuator_only_health_not_falsely_ready(self):
        adapter=Adapter(lambda *args:None,settings(enabled_cards=['arm_control']))
        tool=adapter.tools()[0]['inputSchema']
        self.assertEqual(tool['x-completion'],{'actions':['move','pose','reset'],'timeout':120})
        self.assertEqual(tool['x-resource'],['arm_l','arm_r'])
        self.assertEqual((await adapter.health(None)).status,503)
        adapter.reader.accept_info(info())
        self.assertEqual((await adapter.health(None)).status,503)

class ImagePackageTests(unittest.TestCase):
    def test_apt_only_installs_missing_base_packages(self):
        driver=Path(__file__).resolve().parents[1]
        text=(driver/'Dockerfile').read_text()
        script=text.split('RUN set -eu;',1)[1].split('\nCOPY requirements',1)[0].replace('\\\n','\n')
        for missing,expected in [('',[]),('ca-certificates',['ca-certificates']),('python3-pip ca-certificates',['python3-pip','ca-certificates'])]:
            with self.subTest(missing=missing),tempfile.TemporaryDirectory() as temp:
                root=Path(temp);log=root/'calls'
                for name,body in {
                    'dpkg-query':'case " $TEST_MISSING " in *" $3 "*) exit 1;; *) printf "install ok installed";; esac',
                    'apt-get':'printf "%s\\n" "$*" >> "$TEST_LOG"',
                    'rm':'exit 0'}.items():
                    p=root/name;p.write_text('#!/bin/sh\n'+body+'\n');p.chmod(0o700)
                result=subprocess.run(['/bin/sh','-c','set -eu;'+script],env={**os.environ,'PATH':str(root)+':/usr/bin:/bin','TEST_MISSING':missing,'TEST_LOG':str(log)},capture_output=True,text=True)
                self.assertEqual(result.returncode,0,result.stderr)
                calls=log.read_text().splitlines() if log.exists() else []
                self.assertEqual(calls,[] if not expected else ['update','install -y --no-install-recommends '+' '.join(expected)])

if __name__=='__main__': unittest.main()
