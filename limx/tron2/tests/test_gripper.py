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
    def test_each_action_only_controls_fixed_side(self):
        for side in ('left', 'right'):
            for action, expected in [('open', 43), ('close', 0)]:
                args = {'opening':43, 'speed':30, 'force':25}
                self.assertEqual(gripper.command(action,args,side),
                    {side+'_opening':expected,side+'_speed':30,side+'_force':25})

    def test_bad_values_and_cross_side_payload_never_pass_validation(self):
        base={'opening':40,'speed':30,'force':20}
        for key in base:
            for value in (True,None,'50',float('nan'),float('inf'),-1,101,10**1000):
                with self.subTest(key=key,value=type(value).__name__),self.assertRaises(ValueError):
                    gripper.command('open',{**base,key:value},'left')
        for extra in ({'side':'right'},{'confirm_stopped':True},{'right_opening':20},{'serial':'other'},{'command':'request_movej'}):
            with self.assertRaises(ValueError):gripper.command('open',{**base,**extra},'left')
        for args in ({},{'speed':10,'force':10},{**base,'speed':0}):
            with self.assertRaises(ValueError):gripper.command('open',args,'left')

    def test_fault_and_missing_selected_side_are_not_success(self):
        for data in (sample(result='fail_motor'),{},sample(left_opening=True)):
            with self.assertRaises(ValueError):gripper.state_data(data,'left')
        out=gripper.state_data(sample(right_force=None),'left')
        self.assertNotIn('right',out)
        self.assertFalse(out['grasp_verified']);self.assertFalse(out['measurement_time_verified'])

    def test_two_customer_schemas_fixed_side_and_explicit_open_target(self):
        for side,resource in [('left','arm_l'),('right','arm_r')]:
            tool=gripper.tool({},side);schema=tool['inputSchema']
            self.assertEqual(tool['name'],'gripper_'+side)
            self.assertEqual(schema['properties']['action']['enum'],['open','close','recover'])
            self.assertEqual(schema['x-action-params']['recover']['params'],[])
            self.assertEqual(schema['x-action-params']['open']['params'],['opening','speed','force'])
            self.assertEqual(schema['x-resource'],[resource])
            self.assertNotIn('side',schema['properties']);self.assertNotIn('confirm_stopped',schema['properties'])
            self.assertNotIn('default',schema['properties']['opening'])
            self.assertEqual(schema['x-completion']['actions'],['open','close'])
        for old in ('get_state','set_opening'):
            with self.assertRaises(ValueError):gripper.command(old,{'opening':40,'speed':10,'force':10},'left')

    def test_hidden_declared_fields_are_filtered_but_unknown_keys_rejected(self):
        residual={'action':'close','speed':10,'force':10,'opening':94}
        self.assertEqual(gripper.command('close',residual,'right'),{'right_opening':0,'right_speed':10,'right_force':10})
        self.assertEqual(gripper.normalize_arguments('info',{**residual,'action':'info'}),{'action':'info'})
        self.assertEqual(gripper.recovery_arguments({**residual,'action':'recover'}),{'action':'recover'})
        for action in ('info','close','recover'):
            with self.assertRaises(ValueError):gripper.normalize_arguments(action,{**residual,'action':action,'right_opening':50})


class CompletionTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.reader = RobotReader(settings())
        self.reader.accept_info(info());self.reader.ws = object()
        self.reader.read_gripper = AsyncMock(return_value=sample(100))
        self.tick=123456
        async def fresh():
            self.tick+=1
            state={**self.reader.read_gripper.return_value,'timestamp':self.tick}
            if self.reader.read_gripper.await_count==1:state['left_opening']=40
            return state
        self.reader.read_gripper.side_effect=fresh
        self.reader.send_gripper = AsyncMock(return_value={'result': 'success'})
        self.report = AsyncMock(return_value=True)
        self.controller = gripper.Gripper(self.reader, self.report, 'left')
        self.fast = patch.multiple('gripper', POLL_SECONDS=.005, STABILITY_SECONDS=.015, TIMEOUT=.15)
        self.fast.start()

    async def asyncTearDown(self):
        await self.controller.close();self.fast.stop()

    async def dispatch(self):
        return await self.controller.dispatch('open', {'opening':100, 'speed':30, 'force':20})

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
        second = await self.dispatch()
        self.assertTrue(second['no_op']);self.assertFalse(second['motion_sent'])
        await self.controller.task
        adapter = Adapter(lambda *a:None, settings(enabled_cards=['gripper_left']), reader=self.reader)
        for action in ('start','stop','info'):
            self.assertFalse(adapter.call('gripper_left', {'action':action})['motion_sent'])
        self.assertEqual(self.reader.send_gripper.await_count, 1)

    async def test_query_cache_does_not_survive_connection_change(self):
        await self.controller.read_state();self.assertTrue(self.controller.info()['available'])
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
        return {'action':'recover'}

    async def test_state_query_exposes_failure_without_unlocking_94_as_100(self):
        await self.blocked_at_94()
        state=await self.controller.read_state()
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
        await self.controller.dispatch('close',{'speed':10, 'force':10})
        self.assertNotIn('error',self.controller.info()['motion'])
        await self.controller.task
        self.assertEqual(self.reader.send_gripper.await_count,1)
        self.assertTrue(self.controller.last['no_op'])
        self.assertEqual(self.controller.last['status'],'completed')

    async def test_recovery_rejects_removed_routing_and_confirmation_fields(self):
        args=await self.blocked_at_94();count=self.reader.read_gripper.await_count
        for update in ({'confirm_stopped':True},{'side':'right'},{'unknown':10}):
            with self.assertRaises(ValueError):await self.controller.recover({**args,**update})
        self.assertEqual(self.reader.read_gripper.await_count,count)
        self.assertIsNotNone(self.controller.active)

    async def test_recovery_rejects_moving_or_cached_or_missing_timestamp_feedback(self):
        args=await self.blocked_at_94()
        for updates in ({'left_speed':1},{'timestamp':None},{'result':'fail_motor'}):
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

    async def test_repeated_close_at_zero_is_verified_without_any_motor_write(self):
        await self.controller.read_state()
        self.reader.read_gripper.return_value=sample(0,left_speed=0,right_opening=0,right_speed=0)
        for _ in range(2):
            result=await self.controller.dispatch('close',{'speed':10, 'force':10, 'opening':94})
            self.assertFalse(result['motion_sent']);self.assertTrue(result['no_op'])
            await self.controller.task
            self.assertEqual(self.controller.last['status'],'completed')
            self.assertEqual(self.controller.last['verification'],'already_at_target_fresh_feedback')
            self.assertIsNone(self.controller.active)
        self.reader.send_gripper.assert_not_awaited()

    async def test_failed_noop_verification_does_not_create_uncertain_motor_lock(self):
        self.reader.read_gripper.side_effect=None
        self.reader.read_gripper.return_value=sample(0,left_speed=0,right_opening=0,right_speed=0)
        await self.controller.dispatch('close',{'speed':10, 'force':10})
        await self.controller.task
        self.assertEqual(self.controller.last['status'],'error')
        self.assertFalse(self.controller.last['target_opening_verified'])
        self.assertFalse(self.controller.last['motion_sent'])
        self.assertFalse(self.controller.last['motion_may_continue'])
        self.assertIsNone(self.controller.active)
        self.reader.send_gripper.assert_not_awaited()

    async def test_opposite_side_cannot_be_selected_on_fixed_controller(self):
        await self.blocked_at_94()
        with self.assertRaises(ValueError):
            await self.controller.dispatch('close',{'side':'right','speed':10,'force':10})
        self.reader.send_gripper.assert_awaited_once()
    async def test_close_handover_requires_known_ack_and_original_session(self):
        self.reader.send_gripper.side_effect=MoveError('ack timeout',True)
        await self.dispatch();await self.controller.task
        count=self.reader.read_gripper.await_count
        with self.assertRaises(MoveError):await self.controller.dispatch('close',{'speed':10,'force':10})
        self.assertEqual(self.reader.read_gripper.await_count,count)
        self.reader.send_gripper.assert_awaited_once()

    async def test_close_handover_cannot_follow_reconnection(self):
        await self.blocked_at_94();self.reader.ws=object()
        with self.assertRaises(ValueError):await self.controller.dispatch('close',{'speed':10,'force':10})
        self.assertIsNotNone(self.controller.active);self.reader.send_gripper.assert_awaited_once()

    async def test_close_handover_rejects_stale_moving_fault_and_callback_failure(self):
        await self.blocked_at_94()
        for updates in ({'left_speed':1},{'timestamp':None},{'result':'fail_motor'}):
            self.reader.read_gripper.return_value=sample(94,left_speed=0,right_speed=0,**{})
            self.reader.read_gripper.return_value.update(updates)
            if 'timestamp' in updates:self.reader.read_gripper.side_effect=None
            with patch.multiple('gripper',RECOVERY_TIMEOUT=.04,RECOVERY_STABILITY_SECONDS=.015):
                with self.assertRaises(ValueError):await self.controller.dispatch('close',{'speed':10,'force':10})
            self.assertIsNotNone(self.controller.active)
            self.assertFalse(self.controller.recovering);self.assertIsNone(self.controller.handover_cancel)
        self.reader.send_gripper.assert_awaited_once()

    async def test_close_handover_retains_original_owner_if_failure_callback_not_delivered(self):
        self.report.return_value=False;await self.blocked_at_94()
        with patch.multiple('gripper',RECOVERY_TIMEOUT=.15,RECOVERY_STABILITY_SECONDS=.015):
            with self.assertRaises(ValueError):await self.controller.dispatch('close',{'speed':10,'force':10})
        self.assertIsNotNone(self.controller.active);self.reader.send_gripper.assert_awaited_once()
        self.assertEqual(self.controller.last['status'],'error')
        self.assertNotIn('close_handover',self.controller.last)
    async def test_close_handover_rechecks_stationarity_after_delayed_error_callback(self):
        self.report.return_value=False;await self.blocked_at_94()
        async def callback(*args):
            self.reader.read_gripper.return_value.update(left_speed=1)
            return True
        self.report.side_effect=callback
        with patch.multiple('gripper',RECOVERY_TIMEOUT=.15,RECOVERY_STABILITY_SECONDS=.015):
            with self.assertRaises(ValueError):await self.controller.dispatch('close',{'speed':10,'force':10})
        self.assertIsNotNone(self.controller.active)
        self.assertTrue(self.controller.last['callback_delivered'])
        self.assertNotIn('close_handover',self.controller.last)
        self.reader.send_gripper.assert_awaited_once()



class TransportTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.requests=[];self.opening=40;self.right_opening=60;self.bad_reply=False;self.mode='developer_mode';self.cap=None;self.stationary=False
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
                    payload['right_opening']=self.right_opening
                    if self.cap is not None or self.stationary:payload.update(left_speed=0,right_speed=0)
                if title=='request_set_limx_2fclaw_cmd':
                    self.opening=data['data'].get('left_opening',self.opening)
                    self.right_opening=data['data'].get('right_opening',self.right_opening)
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
        # Close loopback sockets before cancellation: Python3.9 wait_for can
        # otherwise race a just-received frame and leave the polling task alive.
        await self.session.close()
        self.poll.cancel()
        await asyncio.wait_for(asyncio.gather(self.poll,return_exceptions=True),2)
        await self.runner.cleanup()

    async def test_query_and_write_use_exact_guid_title_identity_and_one_side(self):
        state=await self.reader.read_gripper();self.assertEqual(state['left_opening'],40)
        payload=gripper.command('open',{'opening':100,'speed':30,'force':20},'left')
        await self.reader.send_gripper(payload)
        self.assertEqual((await self.reader.read_gripper())['left_opening'],100)
        writes=[r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']
        self.assertEqual(len(writes),1);self.assertEqual(writes[0]['data'],payload)
        grip=[r for r in self.requests if '2fclaw' in r['title']]
        self.assertEqual(len({r['guid'] for r in grip}),3)

    async def test_wrong_identity_or_guid_does_not_admit_motor_command(self):
        self.bad_reply=True
        adapter=Adapter(lambda *a:None,settings(enabled_cards=['gripper_left']),reader=self.reader)
        with self.assertRaises(MoveError):
            await adapter.grippers['gripper_left'].dispatch('open',{'opening':100, 'speed':30, 'force':20})
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
        adapter=Adapter(lambda *a:None,settings(enabled_cards=['gripper_left']),reader=self.reader)
        adapter._report_completion=AsyncMock(return_value=True)
        app=web.Application();app.router.add_post('/mcp',adapter.rpc)
        runner=web.AppRunner(app);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        endpoint='http://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1])+'/mcp'
        async def call(action,**args):
            async with self.session.post(endpoint,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                    'params':{'name':'gripper_left','arguments':{'action':action,**args}}}) as resp:
                return (await resp.json())['result']
        try:
            before=len(self.requests)
            for action in ('start','stop','info'):
                result=await call(action,_trace_id='test',instance_id='customer-card')
                self.assertFalse(result['isError'])
            self.assertFalse(any(r['title']=='request_set_limx_2fclaw_cmd' for r in self.requests[before:]))
            result=await call('info');self.assertFalse(result['isError'])
            removed=await call('get_state');self.assertTrue(removed['isError'])
            inactive=await call('info',speed=10,force=10)
            self.assertFalse(inactive['isError'])
            self.assertTrue(json.loads(inactive['content'][0]['text'])['read_only'])
            self.assertFalse(any(r['title']=='request_set_limx_2fclaw_cmd' for r in self.requests))
            with patch.multiple('gripper',POLL_SECONDS=.01,STABILITY_SECONDS=.025):
                result=await call('open',opening=100,speed=30,force=20)
                self.assertFalse(result['isError']);await adapter.grippers['gripper_left'].task
            self.assertEqual(adapter._report_completion.call_args.args[0],'gripper_left')
            self.assertEqual(adapter._report_completion.call_args.args[2],'completed')
            self.assertEqual(len([r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']),1)
        finally:
            await adapter.grippers['gripper_left'].close();await adapter.grippers['gripper_right'].close();await runner.cleanup()

    async def test_failed_94_feedback_and_recovery_through_actual_mcp(self):
        self.cap=94
        adapter=Adapter(lambda *a:None,settings(enabled_cards=['gripper_left']),reader=self.reader)
        adapter._report_completion=AsyncMock(return_value=True)
        app=web.Application();app.router.add_post('/mcp',adapter.rpc)
        runner=web.AppRunner(app);await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0);await site.start()
        endpoint='http://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1])+'/mcp'
        async def call(action,**args):
            async with self.session.post(endpoint,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                'params':{'name':'gripper_left','arguments':{'action':action,**args}}}) as response:
                body=(await response.json())['result']
                self.assertFalse(body['isError'],body)
                return json.loads(body['content'][0]['text'])
        try:
            with patch.multiple('gripper',TIMEOUT=.12,POLL_SECONDS=.01,RECOVERY_TIMEOUT=.2,RECOVERY_STABILITY_SECONDS=.025):
                await call('open',opening=100,speed=10,force=10)
                await adapter.grippers['gripper_left'].task
                state=await call('info')
                self.assertEqual(state['motion']['reported_opening'],94)
                self.assertFalse(state['can_accept_motion'])
                recovered=await call('recover')
                self.assertTrue(recovered['recovered']);self.assertTrue(recovered['can_accept_motion'])
                self.assertEqual(recovered['motion']['status'],'error')
                self.assertFalse(recovered['motion']['target_opening_verified'])
            self.assertEqual(len([r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']),1)
        finally:
            await adapter.grippers['gripper_left'].close();await adapter.grippers['gripper_right'].close();await runner.cleanup()

    async def shared_http_app(self):
        adapter=Adapter(lambda *a:None,settings(enabled_cards=['joint_state','arm_control','gripper_left','gripper_right']),reader=self.reader)
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

    async def test_dashboard_action_switch_and_closed_right_close_over_actual_http(self):
        self.stationary=True;self.right_opening=0
        adapter,runner,call=await self.shared_http_app()
        old_fields={'opening':35, 'speed':10, 'force':10}
        try:
            with patch.multiple('gripper',POLL_SECONDS=.01,STABILITY_SECONDS=.025):
                state=await call('gripper_left',{'action':'info',**old_fields})
                self.assertFalse(state['isError'])
                moved=await call('gripper_left',{'action':'open','opening':100,**old_fields})
                self.assertFalse(moved['isError']);await adapter.grippers['gripper_left'].task
                closed=await call('gripper_right',{'action':'close', **old_fields})
                self.assertFalse(closed['isError'])
                self.assertTrue(json.loads(closed['content'][0]['text'])['no_op'])
                await adapter.grippers['gripper_right'].task
                recovered=await call('gripper_right',{'action':'recover', **old_fields})
                self.assertFalse(recovered['isError'])
            writes=[r['data'] for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']
            self.assertEqual(writes,[{'left_opening':35,'left_speed':10,'left_force':10}])
            self.assertTrue(adapter.grippers['gripper_left'].info()['can_accept_motion'])
        finally:
            await adapter.motion.close();await adapter.grippers['gripper_left'].close();await adapter.grippers['gripper_right'].close();await runner.cleanup()

    async def test_pending_arm_blocks_gripper_through_actual_http(self):
        adapter,runner,call=await self.shared_http_app()
        try:
            for _ in range(100):
                if self.reader.sample('joint_state')['available']:break
                await asyncio.sleep(.005)
            result=await call('arm_control',{'action':'move','joint':[.2]*14,'time':.1})
            self.assertFalse(result['isError'])
            result=await call('gripper_left',{'action':'open','opening':100, 'speed':30, 'force':20})
            self.assertTrue(result['isError'])
            self.assertFalse(any(r['title']=='request_set_limx_2fclaw_cmd' for r in self.requests))
            schemas={t['name']:t['inputSchema'] for t in adapter.tools()}
            self.assertEqual(schemas['gripper_left']['x-resource'],['arm_l'])
        finally:
            await adapter.motion.close();await adapter.grippers['gripper_left'].close();await adapter.grippers['gripper_right'].close();await runner.cleanup()

    async def test_pending_gripper_blocks_arm_through_actual_http(self):
        adapter,runner,call=await self.shared_http_app()
        try:
            result=await call('gripper_left',{'action':'open','opening':100, 'speed':30, 'force':20})
            self.assertFalse(result['isError'])
            result=await call('arm_control',{'action':'move','joint':[.2]*14,'time':.1})
            self.assertTrue(result['isError'])
            self.assertFalse(any(r['title']=='request_movej' for r in self.requests))
        finally:
            await adapter.motion.close();await adapter.grippers['gripper_left'].close();await adapter.grippers['gripper_right'].close();await runner.cleanup()

    async def test_left_failure_does_not_block_right_open_or_right_recovery(self):
        self.cap=94
        adapter,runner,call=await self.shared_http_app()
        try:
            with patch.multiple('gripper',TIMEOUT=.15,POLL_SECONDS=.005,STABILITY_SECONDS=.015,
                                RECOVERY_TIMEOUT=.15,RECOVERY_STABILITY_SECONDS=.015):
                left=await call('gripper_left',{'action':'open','opening':100,'speed':10,'force':10})
                self.assertFalse(left['isError']);await adapter.grippers['gripper_left'].task
                self.assertEqual(adapter.grippers['gripper_left'].active['state'],'blocked_requires_operator_check')
                right=await call('gripper_right',{'action':'open','opening':70,'speed':10,'force':10})
                self.assertFalse(right['isError']);await adapter.grippers['gripper_right'].task
                self.assertEqual(adapter.grippers['gripper_right'].last['status'],'completed')
                noop=await call('gripper_right',{'action':'recover'})
                self.assertFalse(noop['isError'])
                self.assertIsNotNone(adapter.grippers['gripper_left'].active)
                # Recovery of LEFT accepts stationary left feedback even when the other side reports motion.
                self.cap=None;self.stationary=False
                original=self.reader.read_gripper
                async def moving_right():
                    data=await original();data.update(left_speed=0,right_speed=30);return data
                self.reader.read_gripper=moving_right
                recovered=await call('gripper_left',{'action':'recover','opening':100,'speed':10,'force':10})
                self.assertFalse(recovered['isError'])
                self.assertTrue(json.loads(recovered['content'][0]['text'])['recovered'])
                self.assertEqual(adapter.grippers['gripper_left'].last['status'],'error')
            writes=[r['data'] for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']
            self.assertEqual(writes,[{'left_opening':100,'left_speed':10,'left_force':10},
                                     {'right_opening':70,'right_speed':10,'right_force':10}])
        finally:
            for owner in adapter.grippers.values():await owner.close()
            await runner.cleanup()

    async def test_concurrent_left_right_open_have_separate_owners_and_callbacks(self):
        adapter,runner,call=await self.shared_http_app()
        try:
            with patch.multiple('gripper',POLL_SECONDS=.005,STABILITY_SECONDS=.015):
                responses=await asyncio.gather(*[call('gripper_'+side,{'action':'open','opening':target,'speed':10,'force':10})
                                                for side,target in [('left',55),('right',75)]])
                self.assertTrue(all(not r['isError'] for r in responses))
                await asyncio.gather(*(o.task for o in adapter.grippers.values()))
            self.assertTrue(all(o.last['status']=='completed' and o.active is None for o in adapter.grippers.values()))
            self.assertEqual({c.args[0] for c in adapter._report_completion.call_args_list},
                             {'gripper_left','gripper_right'})
            for side in ('left','right'):
                result=await call('gripper_'+side,{'action':'open','speed':10,'force':10})
                self.assertTrue(result['isError'])
                result=await call('gripper_'+side,{'action':'open','opening':50,'side':'right','speed':10,'force':10})
                self.assertTrue(result['isError'])
            self.assertEqual(len([r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']),2)
        finally:
            for owner in adapter.grippers.values():await owner.close()
            await runner.cleanup()

    async def test_right_failure_blocks_arm_but_does_not_block_left(self):
        adapter,runner,call=await self.shared_http_app()
        try:
            right=adapter.grippers['gripper_right']
            right.active={'state':'blocked_requires_operator_check','action_id':'right-failed'}
            arm=await call('arm_control',{'action':'move','joint':[.2]*14,'time':.1})
            self.assertTrue(arm['isError'])
            with patch.multiple('gripper',POLL_SECONDS=.005,STABILITY_SECONDS=.015):
                left=await call('gripper_left',{'action':'open','opening':50,'speed':10,'force':10})
                self.assertFalse(left['isError']);await adapter.grippers['gripper_left'].task
            self.assertEqual(adapter.grippers['gripper_left'].last['status'],'completed')
            self.assertFalse(any(r['title']=='request_movej' for r in self.requests))
            right.active=None
        finally:
            for owner in adapter.grippers.values():await owner.close()
            await runner.cleanup()

    async def wait_until_queued(self, owner):
        for _ in range(100):
            if owner.waiting:return
            await asyncio.sleep(.001)
        self.fail('Expected exactly one queued operation')

    async def test_user_open_left_open_right_close_right_waits_for_actual_completion(self):
        adapter,runner,call=await self.shared_http_app();gate=asyncio.Event()
        owner=adapter.grippers['gripper_right'];original=owner.verify_target
        async def delayed(job):
            await gate.wait();return await original(job)
        owner.verify_target=delayed
        queued=None
        try:
            with patch.multiple('gripper',TIMEOUT=.2,POLL_SECONDS=.005,STABILITY_SECONDS=.015):
                for side in ('left','right'):
                    result=await call('gripper_'+side,{'action':'open','opening':70,'speed':10,'force':10})
                    self.assertFalse(result['isError'])
                prior=owner.task
                queued=asyncio.create_task(call('gripper_right',{'action':'close','speed':10,'force':10}))
                await self.wait_until_queued(owner)
                self.assertFalse(queued.done());self.assertFalse(owner.info()['can_accept_motion'])
                extra=await call('gripper_right',{'action':'open','opening':60,'speed':10,'force':10})
                self.assertTrue(extra['isError']);self.assertIn('已有一条',json.loads(extra['content'][0]['text'])['error'])
                writes=[r['data'] for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']
                self.assertEqual(writes,[{'left_opening':70,'left_speed':10,'left_force':10},
                                         {'right_opening':70,'right_speed':10,'right_force':10}])
                gate.set();result=await queued
                self.assertFalse(result['isError']);self.assertTrue(prior.done())
                await owner.task
                self.assertEqual(owner.last['status'],'completed');self.assertEqual(owner.last['action'],'close')
                self.assertIsNone(owner.waiting)
                self.assertEqual(len([r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']),3)
        finally:
            gate.set()
            if queued:await asyncio.gather(queued,return_exceptions=True)
            for o in adapter.grippers.values():await o.close()
            await runner.cleanup()

    async def test_queued_open_after_failed_open_is_not_sent_and_both_recover_order_is_independent(self):
        self.cap=94
        adapter,runner,call=await self.shared_http_app()
        original=self.reader.read_gripper
        async def bounded_positions():
            data=await original();data['right_opening']=88;return data
        self.reader.read_gripper=bounded_positions
        try:
            with patch.multiple('gripper',TIMEOUT=.12,POLL_SECONDS=.005,STABILITY_SECONDS=.015,
                                RECOVERY_TIMEOUT=.2,RECOVERY_STABILITY_SECONDS=.015):
                for side in ('left','right'):
                    result=await call('gripper_'+side,{'action':'open','opening':100,'speed':10,'force':10})
                    self.assertFalse(result['isError'])
                result=await call('gripper_right',{'action':'open','opening':70,'speed':10,'force':10})
                self.assertTrue(result['isError'])
                details=json.loads(result['content'][0]['text'])
                self.assertIn('目标100',details['error']);self.assertIn('88',details['error'])
                self.assertFalse(details['motion_sent']);self.assertEqual(details['gripper']['state'],'blocked_requires_operator_check')
                self.assertEqual(len([r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']),2)
                for side in ('left','right'):
                    result=await call('gripper_'+side,{'action':'recover'})
                    self.assertFalse(result['isError'])
                    self.assertTrue(json.loads(result['content'][0]['text'])['recovered'])
                self.assertTrue(all(o.info()['can_accept_motion'] for o in adapter.grippers.values()))
                self.reader.read_gripper=original;self.cap=None;self.stationary=True
                result=await call('gripper_left',{'action':'close','speed':10,'force':10})
                self.assertFalse(result['isError']);await adapter.grippers['gripper_left'].task
                self.assertEqual(adapter.grippers['gripper_left'].last['status'],'completed')
        finally:
            for o in adapter.grippers.values():await o.close()
            await runner.cleanup()

    async def test_stop_cancels_queued_close_without_stopping_or_cancelling_active_open(self):
        adapter,runner,call=await self.shared_http_app();gate=asyncio.Event()
        owner=adapter.grippers['gripper_left'];original=owner.verify_target
        async def delayed(job):
            await gate.wait();return await original(job)
        owner.verify_target=delayed;queued=None
        try:
            with patch.multiple('gripper',TIMEOUT=.2,POLL_SECONDS=.005,STABILITY_SECONDS=.015):
                await call('gripper_left',{'action':'open','opening':70,'speed':10,'force':10})
                original_task=owner.task
                queued=asyncio.create_task(call('gripper_left',{'action':'close','speed':10,'force':10}))
                await self.wait_until_queued(owner)
                stopped=await call('gripper_left',{'action':'stop'})
                self.assertFalse(stopped['isError'])
                result=await queued;self.assertTrue(result['isError'])
                self.assertIn('已取消',json.loads(result['content'][0]['text'])['error'])
                self.assertFalse(original_task.done());self.assertIsNone(owner.waiting)
                gate.set();await original_task
                self.assertEqual(owner.last['status'],'completed')
                self.assertEqual(len([r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']),1)
        finally:
            gate.set()
            if queued:await asyncio.gather(queued,return_exceptions=True)
            for o in adapter.grippers.values():await o.close()
            await runner.cleanup()

    async def test_queued_timeout_and_cancellation_do_not_cancel_the_physical_owner(self):
        adapter,runner,call=await self.shared_http_app();gate=asyncio.Event()
        owner=adapter.grippers['gripper_left'];original=owner.verify_target
        async def delayed(job):
            await gate.wait();return await original(job)
        owner.verify_target=delayed
        try:
            with patch.multiple('gripper',TIMEOUT=.2,POLL_SECONDS=.005,STABILITY_SECONDS=.015,PREVIOUS_TIMEOUT=.02):
                await owner.dispatch('open',{'opening':70,'speed':10,'force':10})
                physical=owner.task
                with self.assertRaises(MoveError):await owner.dispatch('close',{'speed':10,'force':10})
                self.assertFalse(physical.done());self.assertIsNone(owner.waiting)
                queued=asyncio.create_task(owner.dispatch('close',{'speed':10,'force':10}))
                await self.wait_until_queued(owner);queued.cancel()
                with self.assertRaises(asyncio.CancelledError):await queued
                self.assertFalse(physical.done());self.assertIsNone(owner.waiting)
                gate.set();await physical
                self.assertEqual(owner.last['status'],'completed')
                self.assertEqual(len([r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']),1)
        finally:
            gate.set()
            for o in adapter.grippers.values():await o.close()
            await runner.cleanup()

    async def test_close_waits_for_same_side_recover_to_finish(self):
        self.cap=94
        adapter,runner,call=await self.shared_http_app();gate=asyncio.Event();entered=asyncio.Event()
        owner=adapter.grippers['gripper_left'];original=self.reader.read_gripper
        async def held_feedback():
            if owner.recovering:
                entered.set();await gate.wait()
            return await original()
        recovering=queued=None
        try:
            with patch.multiple('gripper',TIMEOUT=.12,POLL_SECONDS=.005,STABILITY_SECONDS=.015,
                                RECOVERY_TIMEOUT=.2,RECOVERY_STABILITY_SECONDS=.015):
                await owner.dispatch('open',{'opening':100,'speed':10,'force':10});await owner.task
                self.reader.read_gripper=held_feedback
                recovering=asyncio.create_task(call('gripper_left',{'action':'recover'}))
                await asyncio.wait_for(entered.wait(),1)
                queued=asyncio.create_task(call('gripper_left',{'action':'close','speed':10,'force':10}))
                await self.wait_until_queued(owner)
                # Other side recovery cannot undo left-side waiting or recovery.
                right=await call('gripper_right',{'action':'recover'})
                self.assertFalse(right['isError']);self.assertFalse(queued.done())
                gate.set();self.cap=None;self.stationary=True
                result=await recovering;self.assertFalse(result['isError'])
                result=await queued;self.assertFalse(result['isError']);await owner.task
                self.assertEqual(owner.last['status'],'completed');self.assertEqual(owner.last['action'],'close')
        finally:
            gate.set()
            for task in (recovering,queued):
                if task:await asyncio.gather(task,return_exceptions=True)
            for o in adapter.grippers.values():await o.close()
            await runner.cleanup()

    async def test_user_open100_left_open_right_then_left_close_checks_stationary_and_keeps_error(self):
        self.cap=94
        adapter,runner,call=await self.shared_http_app()
        try:
            with patch.multiple('gripper',TIMEOUT=.12,POLL_SECONDS=.005,STABILITY_SECONDS=.015,
                                RECOVERY_TIMEOUT=.2,RECOVERY_STABILITY_SECONDS=.015):
                for side,target in [('left',100),('right',70)]:
                    result=await call('gripper_'+side,{'action':'open','opening':target,'speed':10,'force':10})
                    self.assertFalse(result['isError'])
                result=await call('gripper_left',{'action':'close','speed':10,'force':10})
                self.assertFalse(result['isError'],result)
                admitted=json.loads(result['content'][0]['text'])
                prior=admitted['previous_action_error']
                self.assertEqual(prior['status'],'error');self.assertEqual(prior['error_code'],'target_not_reached')
                self.assertEqual(prior['target_opening'],100);self.assertEqual(prior['reported_opening'],94)
                self.assertFalse(prior['target_opening_verified']);self.assertTrue(prior['callback_delivered'])
                self.assertTrue(prior['close_handover']['reported_stationary_verified'])
                self.assertGreaterEqual(prior['close_handover']['verified_samples'],3)
                await adapter.grippers['gripper_left'].task
                self.assertEqual(adapter.grippers['gripper_left'].last['status'],'completed')
                self.assertEqual(adapter.grippers['gripper_left'].last['reported_opening'],0)
                self.assertEqual(adapter.grippers['gripper_left'].last['previous_action_error'],prior)
            writes=[r['data'] for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']
            self.assertEqual(writes,[{'left_opening':100,'left_speed':10,'left_force':10},
                {'right_opening':70,'right_speed':10,'right_force':10},
                {'left_opening':0,'left_speed':10,'left_force':10}])
            left=[c.args for c in adapter._report_completion.call_args_list if c.args[0]=='gripper_left']
            self.assertEqual([c[2] for c in left],['error','completed'])
        finally:
            for o in adapter.grippers.values():await o.close()
            await runner.cleanup()

    async def test_stop_cancels_close_handover_check_before_any_close_write(self):
        self.cap=94
        adapter,runner,call=await self.shared_http_app();gate=asyncio.Event();entered=asyncio.Event()
        owner=adapter.grippers['gripper_left'];original=self.reader.read_gripper
        async def held_feedback():
            if owner.handover_cancel:
                entered.set();await gate.wait()
            return await original()
        closing=None
        try:
            with patch.multiple('gripper',TIMEOUT=.12,POLL_SECONDS=.005,STABILITY_SECONDS=.015,
                                RECOVERY_TIMEOUT=.2,RECOVERY_STABILITY_SECONDS=.015):
                await owner.dispatch('open',{'opening':100,'speed':10,'force':10});await owner.task
                self.reader.read_gripper=held_feedback
                closing=asyncio.create_task(call('gripper_left',{'action':'close','speed':10,'force':10}))
                await asyncio.wait_for(entered.wait(),1)
                extra=await call('gripper_left',{'action':'close','speed':10,'force':10})
                self.assertTrue(extra['isError'])
                self.assertEqual(owner.info()['waiting_action'],'close')
                stopped=await call('gripper_left',{'action':'stop'});self.assertFalse(stopped['isError'])
                gate.set();result=await closing;self.assertTrue(result['isError'])
                self.assertIsNotNone(owner.active);self.assertIsNone(owner.handover_cancel)
                self.assertEqual(len([r for r in self.requests if r['title']=='request_set_limx_2fclaw_cmd']),1)
        finally:
            gate.set()
            if closing:await asyncio.gather(closing,return_exceptions=True)
            for o in adapter.grippers.values():await o.close()
            await runner.cleanup()

if __name__=='__main__':unittest.main()
