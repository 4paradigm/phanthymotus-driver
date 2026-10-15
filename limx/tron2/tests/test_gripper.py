"""Real loopback WebSocket/MCP tests; no robot or DDS is contacted."""
import asyncio
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import AsyncMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import aiohttp
from aiohttp import web
from device import Adapter
import gripper
from vendor import RobotReader, MoveError
from test_sensors import info, joints, settings


def sample(opening=40, **updates):
    out = {'result': 'success', 'timestamp': 123456,
           'left_opening': opening, 'left_speed': 30, 'left_force': 30,
           'right_opening': 60, 'right_speed': 30, 'right_force': 30}
    out.update(updates)
    return out


class ValidationTests(unittest.TestCase):
    def test_each_action_only_controls_explicit_side(self):
        for side in ('left', 'right'):
            for action, expected in [('open', 100), ('close', 0), ('set_opening', 43)]:
                args = {'side': side, 'speed': 30, 'force': 25}
                if action == 'set_opening': args['opening'] = 43
                self.assertEqual(gripper.command(action, args),
                                 {side+'_opening': expected, side+'_speed': 30, side+'_force': 25})

    def test_bad_values_and_cross_side_payload_never_pass_validation(self):
        base = {'side': 'left', 'opening': 40, 'speed': 30, 'force': 20}
        for key in ('opening', 'speed', 'force'):
            for value in (True, None, '50', float('nan'), float('inf'), -1, 101, 10**1000):
                with self.subTest(key=key, value=type(value).__name__), self.assertRaises(ValueError):
                    gripper.command('set_opening', {**base, key: value})
        for extra in ({'right_opening': 20}, {'serial': 'other'}, {'command': 'request_movej'}):
            with self.assertRaises(ValueError): gripper.command('set_opening', {**base, **extra})
        for args in ({}, {**base, 'side': 'both'}, {**base, 'speed': 0}):
            with self.assertRaises(ValueError): gripper.command('set_opening', args)

    def test_fault_and_missing_state_are_not_success(self):
        for data in (sample(result='fail_motor'), {}, sample(left_opening=True), sample(right_force=None)):
            with self.assertRaises(ValueError): gripper.state_data(data)
        out = gripper.state_data(sample())
        self.assertFalse(out['grasp_verified']);self.assertFalse(out['measurement_time_verified'])
        self.assertNotIn('force_newton', out)

    def test_customer_schema_and_no_motion_first_action(self):
        tool = gripper.tool({})
        schema = tool['inputSchema']
        self.assertEqual(schema['properties']['action']['enum'][0], 'get_state')
        self.assertEqual(schema['x-action-params']['get_state']['params'], [])
        self.assertEqual(schema['x-completion']['actions'], list(gripper.MOTION_ACTIONS))
        self.assertEqual(schema['x-resource'], ['arm_l', 'arm_r'])
        self.assertNotIn('url', schema['properties'])
        self.assertNotIn('ACP', tool['description'])


class CompletionTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.reader = RobotReader(settings())
        self.reader.accept_info(info());self.reader.ws = object()
        self.reader.read_gripper = AsyncMock(return_value=sample(100))
        self.tick=123456
        async def fresh():
            self.tick+=1
            return {**self.reader.read_gripper.return_value,'timestamp':self.tick}
        self.reader.read_gripper.side_effect=fresh
        self.reader.send_gripper = AsyncMock(return_value={'result': 'success'})
        self.report = AsyncMock(return_value=True)
        self.controller = gripper.Gripper(self.reader, self.report)
        self.fast = patch.multiple('gripper', POLL_SECONDS=.005, STABILITY_SECONDS=.015, TIMEOUT=.15)
        self.fast.start()

    async def asyncTearDown(self):
        await self.controller.close();self.fast.stop()

    async def dispatch(self):
        return await self.controller.dispatch('open', {'side':'left','speed':30,'force':20})

    async def test_ack_does_not_complete_until_post_ack_fresh_queries(self):
        received = await self.dispatch()
        self.assertFalse(received['target_opening_verified'])
        await self.controller.task
        result = self.report.call_args.args[2]
        self.assertTrue(result['target_opening_verified']);self.assertFalse(result['grasp_verified'])
        self.assertGreaterEqual(self.reader.read_gripper.await_count, 4)
        self.assertEqual(self.reader.send_gripper.await_count, 1)
        self.assertIsNone(self.controller.active)

    async def test_fault_before_dispatch_prevents_send(self):
        self.reader.read_gripper.return_value = sample(result='fail_motor')
        with self.assertRaises(ValueError): await self.dispatch()
        self.reader.send_gripper.assert_not_awaited();self.report.assert_not_awaited()

    async def test_motor_rejection_is_not_false_completion(self):
        self.reader.send_gripper.side_effect = MoveError('fault', True, 'vendor_rejected')
        with self.assertRaises(MoveError): await self.dispatch()
        self.assertIsNone(self.controller.active);self.report.assert_not_awaited()

    async def test_unknown_ack_never_retried_or_completed_from_position(self):
        self.reader.send_gripper.side_effect = MoveError('ack timeout', True)
        result = await self.dispatch();await self.controller.task
        self.assertEqual(result['state'], 'result_unconfirmed')
        self.assertEqual(self.reader.send_gripper.await_count, 1)
        self.assertEqual(self.report.call_args.args[1], 'error')
        self.assertEqual(self.reader.read_gripper.await_count, 1)
        with self.assertRaises(MoveError): await self.dispatch()

    async def test_target_not_reached_cannot_claim_object_grasped(self):
        self.reader.read_gripper.return_value = sample(60)
        await self.dispatch();await self.controller.task
        self.assertEqual(self.report.call_args.args[1], 'error')
        self.assertFalse(self.controller.last['target_opening_verified'])
        self.assertFalse(self.controller.last['grasp_verified'])
        self.assertEqual(self.controller.active['state'], 'blocked_requires_operator_check')

    async def test_callback_failure_keeps_local_owner(self):
        self.report.return_value = False
        await self.dispatch();await self.controller.task
        with self.assertRaises(MoveError): await self.dispatch()

    async def test_concurrent_call_and_lifecycle_do_not_add_motion(self):
        await self.dispatch()
        with self.assertRaises(MoveError): await self.dispatch()
        await self.controller.task
        adapter = Adapter(lambda *a:None, settings(enabled_cards=['gripper']), reader=self.reader)
        for action in ('start','stop','info'):
            self.assertFalse(adapter.call('gripper', {'action':action})['motion_sent'])
        self.assertEqual(self.reader.send_gripper.await_count, 1)

    async def test_query_cache_does_not_survive_connection_change(self):
        await self.controller.get_state();self.assertTrue(self.controller.info()['available'])
        self.reader.ws = object()
        self.assertFalse(self.controller.info()['available'])
        self.assertIsNone(self.controller.info()['data'])

    async def test_cached_vendor_timestamps_cannot_complete_motion(self):
        self.reader.read_gripper.side_effect=None
        await self.dispatch();await self.controller.task
        self.assertFalse(self.controller.last['target_opening_verified'])
        self.assertEqual(self.report.call_args.args[1],'error')

    async def test_shutdown_during_send_reports_unknown_and_never_resends(self):
        entered=asyncio.Event()
        async def pending_send(data):
            entered.set();await asyncio.Event().wait()
        self.reader.send_gripper.side_effect=pending_send
        waiter=asyncio.create_task(self.dispatch())
        await entered.wait();await self.controller.close()
        result=await waiter
        self.assertIsNone(result['motion_sent'])
        self.assertEqual(self.reader.send_gripper.await_count,1)
        self.assertFalse(self.controller.last['target_opening_verified'])

    async def test_connection_change_during_motion_blocks_completion(self):
        await self.dispatch();self.reader.ws = object()
        await self.controller.task
        self.assertEqual(self.report.call_args.args[1], 'error')
        self.assertFalse(self.controller.last['target_opening_verified'])

    async def blocked_at_94(self):
        self.reader.read_gripper.return_value = sample(94,left_speed=0,right_speed=0)
        await self.dispatch();await self.controller.task
        self.assertEqual(self.controller.info()['state'],'blocked_requires_operator_check')
        return {'action':'recover','side':'left','confirm_stopped':True}

    async def test_state_query_exposes_failure_without_unlocking_94_as_100(self):
        await self.blocked_at_94()
        state=await self.controller.get_state()
        self.assertEqual(state['state'],'blocked_requires_operator_check')
        self.assertFalse(state['can_accept_motion'])
        self.assertEqual(state['motion']['target_opening'],100)
        self.assertEqual(state['motion']['reported_opening'],94)
        self.assertIn('94',state['motion']['error'])
        self.assertFalse(state['motion']['target_opening_verified'])

    async def test_operator_recovery_preserves_failed_result_and_never_resends_motion(self):
        args=await self.blocked_at_94()
        with patch.multiple('gripper',RECOVERY_TIMEOUT=.15,RECOVERY_STABILITY_SECONDS=.015):
            result=await self.controller.recover(args)
        self.assertTrue(result['recovered']);self.assertTrue(result['can_accept_motion'])
        self.assertFalse(result['motion_sent']);self.assertFalse(result['recovering'])
        self.assertEqual(self.controller.last['status'],'error')
        self.assertFalse(self.controller.last['target_opening_verified'])
        self.assertEqual(self.controller.last['operator_recovery']['observed_opening'],94)
        self.reader.send_gripper.assert_awaited_once()
        self.assertIsNone(self.controller.active)
        self.reader.read_gripper.return_value=sample(0,left_speed=0,right_speed=0)
        await self.controller.dispatch('close',{'side':'left','speed':10,'force':10})
        self.assertNotIn('error',self.controller.info()['motion'])
        await self.controller.task
        self.assertEqual(self.reader.send_gripper.await_count,2)
        self.assertEqual(self.controller.last['status'],'completed')

    async def test_recovery_requires_literal_confirmation_and_original_side(self):
        args=await self.blocked_at_94();count=self.reader.read_gripper.await_count
        for update in ({'confirm_stopped':False},{'confirm_stopped':'true'},{'side':'right'},{'force':10}):
            with self.assertRaises(ValueError):await self.controller.recover({**args,**update})
        self.assertEqual(self.reader.read_gripper.await_count,count)
        self.assertIsNotNone(self.controller.active)

    async def test_recovery_rejects_moving_or_cached_or_missing_timestamp_feedback(self):
        args=await self.blocked_at_94()
        for updates in ({'left_speed':1},{'right_speed':1},{'timestamp':None},{'result':'fail_motor'}):
            self.reader.read_gripper.return_value=sample(94,left_speed=0,right_speed=0)
            self.reader.read_gripper.return_value.update(updates)
            if 'timestamp' in updates:self.reader.read_gripper.side_effect=None
            with patch.multiple('gripper',RECOVERY_TIMEOUT=.04,RECOVERY_STABILITY_SECONDS=.015):
                with self.assertRaises(ValueError):await self.controller.recover(args)
            self.assertIsNotNone(self.controller.active);self.assertFalse(self.controller.recovering)
        self.reader.read_gripper.side_effect=None
        self.reader.read_gripper.return_value=sample(94,left_speed=0,right_speed=0)
        with patch.multiple('gripper',RECOVERY_TIMEOUT=.04,RECOVERY_STABILITY_SECONDS=.015):
            with self.assertRaises(ValueError):await self.controller.recover(args)
        self.reader.send_gripper.assert_awaited_once()

    async def test_recovery_rejects_cumulative_opening_drift(self):
        args=await self.blocked_at_94();opening=60
        async def drift():
            nonlocal opening
            opening+=1;self.tick+=1
            return sample(opening,left_speed=0,right_speed=0,timestamp=self.tick)
        self.reader.read_gripper.side_effect=drift
        with patch.multiple('gripper',RECOVERY_TIMEOUT=.04,RECOVERY_STABILITY_SECONDS=.015):
            with self.assertRaises(ValueError):await self.controller.recover(args)
        self.assertIsNotNone(self.controller.active)

    async def test_recovery_retains_lock_if_original_callback_still_fails(self):
        self.report.return_value=False;args=await self.blocked_at_94()
        with patch.multiple('gripper',RECOVERY_TIMEOUT=.15,RECOVERY_STABILITY_SECONDS=.015):
            with self.assertRaises(ValueError):await self.controller.recover(args)
            self.assertIsNotNone(self.controller.active)
            self.report.return_value=True
            result=await self.controller.recover(args)
        self.assertTrue(result['recovered'])
        self.assertEqual(self.report.call_args.args[1],'error')
        self.reader.send_gripper.assert_awaited_once()

    async def test_recovery_stops_if_connection_changes(self):
        args=await self.blocked_at_94()
        async def changed():
            self.reader.ws=object()
            return sample(94,left_speed=0,right_speed=0,timestamp=999999)
        self.reader.read_gripper.side_effect=changed
        with self.assertRaises(ValueError):await self.controller.recover(args)
        self.assertIsNotNone(self.controller.active)

    async def test_recovery_callback_exception_keeps_lock_and_query_truthful(self):
        self.report.return_value=False;args=await self.blocked_at_94()
        self.report.side_effect=OSError('callback offline')
        with patch.multiple('gripper',RECOVERY_TIMEOUT=.15,RECOVERY_STABILITY_SECONDS=.015):
            with self.assertRaises(ValueError):await self.controller.recover(args)
        self.assertIsNotNone(self.controller.active)
        self.assertFalse(self.controller.recovering)

    async def test_recovery_cannot_clear_active_arm_or_ongoing_gripper(self):
        args=await self.blocked_at_94()
        from types import SimpleNamespace
        self.controller.peer=SimpleNamespace(closed=False,active={'state':'moving'})
        with self.assertRaises(ValueError):await self.controller.recover(args)
        self.controller.peer=None;self.controller.active['state']='moving'
        with self.assertRaises(ValueError):await self.controller.recover(args)
        self.assertIsNotNone(self.controller.active)


class TransportTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.requests=[];self.opening=40;self.bad_reply=False;self.mode='developer_mode';self.cap=None
        async def vendor(request):
            ws=web.WebSocketResponse();await ws.prepare(request)
            packet=info();packet['data']['working_mode']=self.mode
            await ws.send_json(packet)
            async for msg in ws:
                data=json.loads(msg.data);self.requests.append(data)
                title=data['title'];payload=None
                if title=='request_get_joint_state': payload=joints()
                if title=='request_get_limx_2fclaw_state':
                    payload=sample(self.opening,timestamp=len(self.requests))
                    if self.cap is not None:payload.update(left_speed=0,right_speed=0)
                if title=='request_set_limx_2fclaw_cmd':
                    self.opening=data['data'].get('left_opening',self.opening)
                    if self.cap is not None:self.opening=min(self.opening,self.cap)
                    payload={'result':'success'}
                if title=='request_movej': payload={'result':'success'}
                if payload is not None:
                    if '2fclaw' in title and self.bad_reply:
                        await ws.send_json({'title':title.replace('request_','response_'), 'accid':'OTHER','guid':data['guid'],'data':payload})
                        await ws.send_json({'title':title.replace('request_','response_'), 'accid':data['accid'],'guid':'unrelated','data':payload})
                    else:
                        await ws.send_json({'title':title.replace('request_','response_'), 'accid':data['accid'],'guid':data['guid'],'data':payload})
            return ws
        app=web.Application();app.router.add_get('/',vendor)
        self.runner=web.AppRunner(app);await self.runner.setup()
        site=web.TCPSite(self.runner,'127.0.0.1',0);await site.start()
        self.session=aiohttp.ClientSession()
        self.reader=RobotReader(settings(url='ws://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1]),
                                        poll_interval_seconds=.01,response_timeout_seconds=.2))
        self.poll=asyncio.create_task(self.reader.run(self.session))
        for _ in range(100):
            if self.reader.connected:break
            await asyncio.sleep(.005)
        self.assertTrue(self.reader.connected)

    async def asyncTearDown(self):
        self.poll.cancel();await asyncio.gather(self.poll,return_exceptions=True)
        await self.session.close();await self.runner.cleanup()

    async def test_query_and_write_use_exact_guid_title_identity_and_one_side(self):
        state=await self.reader.read_gripper();self.assertEqual(state['left_opening'],40)
        payload=gripper.command('open',{'side':'left','speed':30,'force':20})
        await self.reader.send_gripper(payload)
        self.assertEqual((await self.reader.read_gripper())['left_opening'],100)
        writes=[r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']
        self.assertEqual(len(writes),1);self.assertEqual(writes[0]['data'],payload)
        grip=[r for r in self.requests if '2fclaw' in r['title']]
        self.assertEqual(len({r['guid'] for r in grip}),3)

    async def test_wrong_identity_or_guid_does_not_admit_motor_command(self):
        self.bad_reply=True
        adapter=Adapter(lambda *a:None,settings(enabled_cards=['gripper']),reader=self.reader)
        with self.assertRaises(MoveError):
            await adapter.gripper.dispatch('open',{'side':'left','speed':30,'force':20})
        self.assertFalse(any(r['title']=='request_set_limx_2fclaw_cmd' for r in self.requests))

    async def test_stale_mode_and_invalid_side_refuse_before_send(self):
        payload={'left_opening':100,'left_speed':30,'left_force':20}
        self.reader.info_at-=10
        with self.assertRaises(MoveError): await self.reader.send_gripper(payload)
        with self.assertRaises(MoveError): await self.reader.send_gripper({**payload,'right_opening':0})
        self.assertFalse(any(r['title']=='request_set_limx_2fclaw_cmd' for r in self.requests))

    async def test_private_gripper_transport_cannot_send_arbitrary_commands(self):
        with self.assertRaises(MoveError):
            await self.reader._gripper_request('request_movej',{},True)
        self.assertFalse(any(r['title']=='request_movej' for r in self.requests))

    async def test_actual_mcp_http_path_and_gripper_callback_tool_name(self):
        adapter=Adapter(lambda *a:None,settings(enabled_cards=['gripper']),reader=self.reader)
        adapter._report_completion=AsyncMock(return_value=True)
        app=web.Application();app.router.add_post('/mcp',adapter.rpc)
        runner=web.AppRunner(app);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        endpoint='http://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1])+'/mcp'
        async def call(action,**args):
            async with self.session.post(endpoint,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                    'params':{'name':'gripper','arguments':{'action':action,**args}}}) as resp:
                return (await resp.json())['result']
        try:
            before=len(self.requests)
            for action in ('start','stop','info'):
                result=await call(action,_trace_id='test',instance_id='customer-card')
                self.assertFalse(result['isError'])
            self.assertFalse(any('2fclaw' in r['title'] for r in self.requests[before:]))
            result=await call('get_state');self.assertFalse(result['isError'])
            invalid=await call('get_state',side='left')
            self.assertTrue(invalid['isError'])
            self.assertTrue(json.loads(invalid['content'][0]['text'])['read_only'])
            self.assertFalse(any(r['title']=='request_set_limx_2fclaw_cmd' for r in self.requests))
            with patch.multiple('gripper',POLL_SECONDS=.01,STABILITY_SECONDS=.025):
                result=await call('open',side='left',speed=30,force=20)
                self.assertFalse(result['isError']);await adapter.gripper.task
            self.assertEqual(adapter._report_completion.call_args.args[0],'gripper')
            self.assertEqual(adapter._report_completion.call_args.args[2],'completed')
            self.assertEqual(len([r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']),1)
        finally:
            await adapter.gripper.close();await runner.cleanup()

    async def test_failed_94_feedback_and_recovery_through_actual_mcp(self):
        self.cap=94
        adapter=Adapter(lambda *a:None,settings(enabled_cards=['gripper']),reader=self.reader)
        adapter._report_completion=AsyncMock(return_value=True)
        app=web.Application();app.router.add_post('/mcp',adapter.rpc)
        runner=web.AppRunner(app);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        endpoint='http://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1])+'/mcp'
        async def call(action,**args):
            async with self.session.post(endpoint,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                'params':{'name':'gripper','arguments':{'action':action,**args}}}) as response:
                body=(await response.json())['result']
                self.assertFalse(body['isError'],body)
                return json.loads(body['content'][0]['text'])
        try:
            with patch.multiple('gripper',TIMEOUT=.12,POLL_SECONDS=.01,RECOVERY_TIMEOUT=.2,RECOVERY_STABILITY_SECONDS=.025):
                await call('open',side='left',speed=10,force=10)
                await adapter.gripper.task
                state=await call('get_state')
                self.assertEqual(state['motion']['reported_opening'],94)
                self.assertFalse(state['can_accept_motion'])
                recovered=await call('recover',side='left',confirm_stopped=True)
                self.assertTrue(recovered['recovered']);self.assertTrue(recovered['can_accept_motion'])
                self.assertEqual(recovered['motion']['status'],'error')
                self.assertFalse(recovered['motion']['target_opening_verified'])
            self.assertEqual(len([r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']),1)
        finally:
            await adapter.gripper.close();await runner.cleanup()

    async def shared_http_app(self):
        adapter=Adapter(lambda *a:None,settings(enabled_cards=['joint_state','arm_control','gripper']),reader=self.reader)
        adapter._report_completion=AsyncMock(return_value=True)
        app=web.Application();app.router.add_post('/mcp',adapter.rpc)
        runner=web.AppRunner(app);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        endpoint='http://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1])+'/mcp'
        async def call(name,args):
            async with self.session.post(endpoint,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                    'params':{'name':name,'arguments':args}}) as resp:
                return (await resp.json())['result']
        return adapter,runner,call

    async def test_pending_arm_blocks_gripper_through_actual_http(self):
        adapter,runner,call=await self.shared_http_app()
        try:
            for _ in range(100):
                if self.reader.sample('joint_state')['available']:break
                await asyncio.sleep(.005)
            result=await call('arm_control',{'action':'move','joint':[.2]*14,'time':.1})
            self.assertFalse(result['isError'])
            result=await call('gripper',{'action':'open','side':'left','speed':30,'force':20})
            self.assertTrue(result['isError'])
            self.assertFalse(any(r['title']=='request_set_limx_2fclaw_cmd' for r in self.requests))
            schemas={t['name']:t['inputSchema'] for t in adapter.tools()}
            self.assertEqual(schemas['gripper']['x-resource'],schemas['arm_control']['x-resource'])
        finally:
            await adapter.motion.close();await adapter.gripper.close();await runner.cleanup()

    async def test_pending_gripper_blocks_arm_through_actual_http(self):
        adapter,runner,call=await self.shared_http_app()
        try:
            result=await call('gripper',{'action':'open','side':'left','speed':30,'force':20})
            self.assertFalse(result['isError'])
            result=await call('arm_control',{'action':'move','joint':[.2]*14,'time':.1})
            self.assertTrue(result['isError'])
            self.assertFalse(any(r['title']=='request_movej' for r in self.requests))
        finally:
            await adapter.motion.close();await adapter.gripper.close();await runner.cleanup()

if __name__=='__main__':unittest.main()
