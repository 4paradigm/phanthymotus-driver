"""Offline protocol/interlock tests; no physical robot acceptance is claimed."""
import asyncio
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import AsyncMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
if importlib.util.find_spec('aiohttp') is None:
    fake = types.ModuleType('aiohttp')
    fake.ClientError = type('ClientError', (Exception,), {})
    fake.WSMsgType = types.SimpleNamespace(TEXT=1, CLOSE=2, CLOSED=3, ERROR=4)
    fake.web = types.SimpleNamespace(json_response=lambda data, status=200:
        types.SimpleNamespace(status=status, text=json.dumps(data)),
        Response=lambda status: types.SimpleNamespace(status=status))
    sys.modules['aiohttp'] = fake

import aiohttp
import pose
pose.SERIAL = 'TEST_TRON2A'
import motion
from device import Adapter


def snapshot(stamp=100):
    return {'accid': pose.SERIAL, 'connected': True, 'fresh': True,
            'info_age_seconds': .1, 'joint_age_seconds': .1,
            'robot': {'motor': 'OK', 'imu': 'OK', 'sw_version': 'test',
                      'working_mode_valid': True, 'working_mode': 'test-confirmed-mode'},
            'joint_state': {'result': 'success', 'q': [.1]*16, 'dq': [0]*16,
                            'names': [], 'robot_timestamp': stamp}}


class FakeRobot:
    state = snapshot()
    teach = {'active': False}
    calls = []
    uncertain = False
    started = None
    finish = None
    def __init__(self, session): pass
    async def __aenter__(self): return self
    async def __aexit__(self, *args): pass
    async def status(self):
        self.calls.append('status')
        return self.state, self.teach
    async def ready_state(self):
        return await self.status()
    async def move_and_verify(self, home, config, before_send):
        motion.preflight(home, self.state, self.teach, config)
        await before_send(self.state)
        self.calls.append('move')
        if self.started:
            self.started.set()
            await self.finish.wait()
        if self.uncertain:
            raise motion.MotionFailure('timeout after possible delivery', True)
        return {'motion_sent': True, 'status': 'arrived', 'at_home': True}


class InterlockTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        self.path, self.cfg, self.latch = root/'home.json', root/'cfg.json', root/'latch.json'
        pose.save_pose(self.path, pose.make_pose([snapshot(100), snapshot(102), snapshot(104)]))
        self.adapter = Adapter(None, self.path, self.cfg, self.latch, FakeRobot)
        self.config = {'commissioned': True, 'pose_sha256': self.adapter.load_pose()[1],
                      'high_level_confirmed_by_operator': True,
                      'expected_working_mode': 'test-confirmed-mode',
                      'teach_state_pointer': '/active', 'teach_inactive_value': False}
        self.cfg.write_text(json.dumps(self.config))
        FakeRobot.state, FakeRobot.teach, FakeRobot.calls = snapshot(), {'active': False}, []
        FakeRobot.uncertain, FakeRobot.started, FakeRobot.finish = False, None, None

    async def enable(self):
        return await self.adapter.call({'action': 'start', 'instance_id': 'canvas-reset'})

    async def test_disabled_or_uncommissioned_cannot_execute(self):
        with self.assertRaisesRegex(ValueError, 'Enable intelligent control'):
            await self.adapter.call({'action': 'execute'})
        self.config['commissioned'] = False
        self.cfg.write_text(json.dumps(self.config))
        with self.assertRaises(ValueError):
            await self.enable()
        self.assertFalse(self.adapter.platform_active)
        self.assertEqual(FakeRobot.calls, [])

    async def test_platform_start_allows_repeated_explicit_calls_without_local_arm(self):
        before = self.path.read_bytes()
        result = await self.enable()
        self.assertTrue(result['platform_active'])
        self.assertFalse(result['motion_sent'])
        self.assertEqual(FakeRobot.calls, [])
        for _ in range(2):
            self.assertTrue((await self.adapter.call({'action': 'execute'}))['at_home'])
        self.assertEqual(FakeRobot.calls.count('move'), 2)
        self.assertEqual(before, self.path.read_bytes())
        self.assertFalse(self.latch.exists())

    async def test_stop_disables_future_calls_without_physical_stop_command(self):
        await self.enable()
        result = await self.adapter.call({'action': 'stop', 'instance_id': 'canvas-reset'})
        self.assertFalse(result['platform_active'])
        self.assertIn('does NOT stop', result['note'])
        with self.assertRaises(ValueError):
            await self.adapter.call({'action': 'execute'})
        self.assertEqual(FakeRobot.calls, [])

    async def test_canvas_metadata_does_not_allow_custom_targets(self):
        for action in ('start', 'info', 'execute', 'stop'):
            with self.subTest(action=action), self.assertRaises(ValueError):
                await self.adapter.call({'action': action, 'instance_id': 'card', 'target_q': [0]*14})
        for instance in (None, 123, {}, 'x'*129, 'card\nforged'):
            with self.subTest(instance=instance), self.assertRaises(ValueError):
                await self.adapter.call({'action': 'start', 'instance_id': instance})
        with self.assertRaises(ValueError):
            await self.adapter.call({'action': 'arm'})
        self.assertEqual(FakeRobot.calls, [])

    async def test_active_teach_wrong_mode_and_invalid_state_still_refuse_motion(self):
        await self.enable()
        for teach in [{'active': True}, {'result': 'success'}, {'active': 'false'}]:
            FakeRobot.teach = teach
            with self.subTest(teach=teach), self.assertRaises(motion.MotionFailure) as caught:
                await self.adapter.call({'action': 'execute'})
            self.assertFalse(caught.exception.attempted)
        FakeRobot.teach = {'active': False}
        for field, value in [('working_mode', 'controller_mode'), ('sw_version', 'changed')]:
            FakeRobot.state = snapshot()
            FakeRobot.state['robot'][field] = value
            with self.assertRaises(motion.MotionFailure) as caught:
                await self.adapter.call({'action': 'execute'})
            self.assertFalse(caught.exception.attempted)
        FakeRobot.state = snapshot()
        FakeRobot.state['joint_age_seconds'] = 4
        with self.assertRaises(motion.MotionFailure):
            await self.adapter.call({'action': 'execute'})
        self.assertNotIn('move', FakeRobot.calls)

    async def test_changed_target_is_not_silently_recommissioned(self):
        await self.enable()
        home = json.loads(self.path.read_text())
        home['target_q'][0] += .01
        self.path.write_text(json.dumps(home))
        with self.assertRaises(ValueError):
            await self.adapter.call({'action': 'execute'})
        self.assertNotIn('move', FakeRobot.calls)

    async def test_stop_then_restart_during_preflight_invalidates_pending_send(self):
        await self.enable()
        async def interrupted(home, config, before_send):
            await self.adapter.call({'action': 'stop'})
            await self.enable()
            await before_send(snapshot())
            self.fail('Old pending request must not survive stop/restart')
        with patch.object(FakeRobot, 'move_and_verify', side_effect=interrupted):
            with self.assertRaises(motion.MotionFailure) as caught:
                await self.adapter.call({'action': 'execute'})
        self.assertFalse(caught.exception.attempted)
        self.assertFalse(self.latch.exists())
        self.assertNotIn('move', FakeRobot.calls)

    async def test_shutdown_before_send_refuses_motion_and_start(self):
        await self.enable()
        async def interrupted(home, config, before_send):
            self.adapter.stopping = True
            await before_send(snapshot())
            self.fail('Shutdown must reject send')
        with patch.object(FakeRobot, 'move_and_verify', side_effect=interrupted):
            with self.assertRaises(motion.MotionFailure) as caught:
                await self.adapter.call({'action': 'execute'})
        self.assertFalse(caught.exception.attempted)
        with self.assertRaises(ValueError):
            await self.enable()
        self.assertFalse(self.latch.exists())
        self.assertNotIn('move', FakeRobot.calls)

    async def test_target_change_during_preflight_is_rejected_before_send(self):
        await self.enable()
        async def changed(home, config, before_send):
            newer = json.loads(self.path.read_text())
            newer['target_q'][0] += .01
            self.path.write_text(json.dumps(newer))
            await before_send(snapshot())
            self.fail('Changed target must not be sent')
        with patch.object(FakeRobot, 'move_and_verify', side_effect=changed):
            with self.assertRaises(motion.MotionFailure) as caught:
                await self.adapter.call({'action': 'execute'})
        self.assertFalse(caught.exception.attempted)
        self.assertFalse(self.latch.exists())

    async def test_uncertain_result_survives_restart_and_reports_unknown_send(self):
        await self.enable()
        FakeRobot.uncertain = True
        class Request:
            async def json(self):
                return {'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call',
                        'params': {'name': 'reset_pose', 'arguments': {'action': 'execute'}}}
        response = await self.adapter.rpc(Request())
        result = json.loads(response.text)['result']
        self.assertTrue(result['isError'])
        self.assertEqual(json.loads(result['content'][0]['text'])['motion_sent'], 'unknown')
        self.assertTrue(self.latch.exists())
        restarted = Adapter(None, self.path, self.cfg, self.latch, FakeRobot)
        self.assertFalse(restarted.platform_active)
        await restarted.call({'action': 'start'})
        with self.assertRaisesRegex(ValueError, 'unconfirmed'):
            await restarted.call({'action': 'execute'})
        self.assertEqual(FakeRobot.calls.count('move'), 1)

    async def test_duplicate_calls_and_http_cancellation_do_not_resend(self):
        await self.enable()
        FakeRobot.started, FakeRobot.finish = asyncio.Event(), asyncio.Event()
        caller = asyncio.create_task(self.adapter.call({'action': 'execute'}))
        await FakeRobot.started.wait()
        with self.assertRaises(ValueError):
            await self.adapter.call({'action': 'execute'})
        caller.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await caller
        self.assertFalse(self.adapter.active_task.done())
        # Disabling the platform prevents new motion, but is not a physical stop.
        await self.adapter.call({'action': 'stop'})
        FakeRobot.finish.set()
        await self.adapter.active_task
        self.assertEqual(FakeRobot.calls.count('move'), 1)

    async def test_restart_does_not_restore_platform_enabled_state(self):
        await self.enable()
        restarted = Adapter(None, self.path, self.cfg, self.latch, FakeRobot)
        self.assertFalse(restarted.platform_active)
        with self.assertRaises(ValueError):
            await restarted.call({'action': 'execute'})
        self.assertNotIn('move', FakeRobot.calls)


class TransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_request_matches_guid_and_serial_and_status_is_read_only(self):
        requests, queue = [], []
        class WS:
            async def send_json(self, data):
                requests.append(data)
                base = {'title': 'response_drag_teach_manage', 'accid': pose.SERIAL,
                        'data': {'result': 'success', 'active': False}}
                queue.extend([{**base, 'guid': 'wrong'}, {**base, 'guid': data['guid']}])
            async def receive(self):
                return types.SimpleNamespace(type=aiohttp.WSMsgType.TEXT,
                                             data=json.dumps(queue.pop(0)))
        client = motion.RobotClient(None)
        client.ws = WS()
        result = await client.request('request_drag_teach_manage', {'cmd': 'status', 'arg': ''})
        self.assertFalse(result['active'])
        self.assertEqual(len(requests), 1)
        self.assertEqual(requests[0]['data'], {'cmd': 'status', 'arg': ''})
        with self.assertRaises(ValueError):
            await client.request('request_drag_teach_manage', {'cmd': 'exit', 'arg': ''})
        self.assertEqual(len(requests), 1)

    async def test_wrong_serial_notify_refuses_handshake(self):
        class WS:
            async def receive(self):
                return types.SimpleNamespace(type=aiohttp.WSMsgType.TEXT,
                    data=json.dumps({'title': 'notify_robot_info', 'accid': 'other', 'data': {}}))
        client = motion.RobotClient(None)
        client.ws = WS()
        with self.assertRaises(ValueError):
            await client.receive(time.monotonic() + 1)

    async def test_movej_payload_and_three_feedback_samples_after_duration(self):
        clock = [1000.]
        home = pose.make_pose([snapshot(100), snapshot(102), snapshot(104)])
        config = {'expected_working_mode': 'test-confirmed-mode',
                  'teach_state_pointer': '/active', 'teach_inactive_value': False}
        sent, commit = [], []
        class Client(motion.RobotClient):
            def __init__(self):
                super().__init__(None)
                self.robot, self.info_at = snapshot()['robot'], clock[0]
                self.stamp = 100
            async def request(self, title, data, timeout=4):
                sent.append((title, data))
                if title == 'request_movej':
                    clock[0] += 8
                return {'result': 'success', 'active': False}
            async def joints(self):
                self.stamp += 1
                clock[0] += .3
                self.info_at = clock[0]
                return {**snapshot(self.stamp)['joint_state']}
        async def before(current): commit.append(current)
        with patch('motion.time.monotonic', side_effect=lambda: clock[0]), \
                patch('motion.asyncio.sleep', new=AsyncMock()):
            result = await Client().move_and_verify(home, config, before)
        moves = [data for title,data in sent if title == 'request_movej']
        self.assertEqual(moves, [{'time': 8, 'joint': home['target_q']}])
        self.assertEqual(len(commit), 1)
        self.assertTrue(result['at_home'])
        self.assertEqual(result['stable_samples'], 3)
        self.assertFalse(result['automatic_collision_check'])

    async def test_ack_timeout_is_an_uncertain_delivery_not_a_retry(self):
        home = pose.make_pose([snapshot(100), snapshot(102), snapshot(104)])
        config = {'expected_working_mode': 'test-confirmed-mode',
                  'teach_state_pointer': '/active', 'teach_inactive_value': False}
        sent = []
        class Client(motion.RobotClient):
            async def status(self): return snapshot(), {'active': False}
            async def ready_state(self): return await self.status()
            async def request(self, title, data, timeout=4):
                sent.append(title)
                raise TimeoutError('ack lost')
        async def before(current): pass
        with self.assertRaises(motion.MotionFailure) as caught:
            await Client(None).move_and_verify(home, config, before)
        self.assertTrue(caught.exception.attempted)
        self.assertEqual(sent, ['request_movej'])

    async def test_success_ack_is_not_arrival_if_feedback_never_returns_to_home(self):
        home = pose.make_pose([snapshot(100), snapshot(102), snapshot(104)])
        config = {'expected_working_mode': 'test-confirmed-mode',
                  'teach_state_pointer': '/active', 'teach_inactive_value': False}
        clock, sent = [1000.], []
        class Client(motion.RobotClient):
            def __init__(self):
                super().__init__(None)
                self.robot, self.info_at = snapshot()['robot'], clock[0]
                self.stamp = 100
            async def ready_state(self): return snapshot(), {'active': False}
            async def request(self, title, data, timeout=4):
                sent.append(title)
                return {'result': 'success'}
            async def joints(self):
                self.stamp += 1
                clock[0] += 1
                self.info_at = clock[0]
                data = snapshot(self.stamp)['joint_state']
                data['q'][0] += .04
                return data
        async def before(current): pass
        with patch('motion.time.monotonic', side_effect=lambda: clock[0]), \
                patch('motion.asyncio.sleep', new=AsyncMock()), \
                self.assertRaises(motion.MotionFailure) as caught:
            await Client().move_and_verify(home, config, before)
        self.assertTrue(caught.exception.attempted)
        self.assertEqual(sent.count('request_movej'), 1)

    async def test_stationarity_rejects_pose_drift_even_when_dq_reports_zero(self):
        class Client(motion.RobotClient):
            async def status(self): return snapshot(), {'active': False}
            async def joints(self):
                return snapshot(101)['joint_state']
            def snapshot(self, joints):
                result = snapshot(101)
                result['joint_state']['q'][0] += .02
                return result
        with patch('motion.asyncio.sleep', new=AsyncMock()), self.assertRaises(ValueError):
            await Client(None).ready_state()

    def test_success_receipt_is_not_mode_confirmation(self):
        for pointer in ('/result', '/success', '/cmd', None):
            with self.subTest(pointer=pointer), self.assertRaises(ValueError):
                motion.inactive_value({'result': 'success', 'success': True, 'cmd': 'status'},
                    {'teach_state_pointer': pointer, 'teach_inactive_value': False})

    def test_observed_developer_and_hand_guiding_messages_are_distinct(self):
        cfg = {'teach_state_pointer': '/message#state',
               'teach_inactive_value': 'ST_DEVED', 'expected_working_mode': 'developer_mode'}
        good = {'result': 'success', 'success': True, 'cmd': 'status',
                'message': 'state=ST_DEVED;sub=;rec=0'}
        motion.inactive_value(good, cfg)
        with self.assertRaises(ValueError):
            motion.inactive_value({**good, 'message': 'state=ST_DRAG_TEACH;sub=;rec=0'}, cfg)
        with self.assertRaises(ValueError):
            motion.inactive_value(good, {**cfg, 'expected_working_mode': 'controller_mode'})

    def test_unknown_duplicate_recording_or_partial_messages_are_rejected(self):
        cfg = {'teach_state_pointer': '/message#state',
               'teach_inactive_value': 'ST_DEVED', 'expected_working_mode': 'developer_mode'}
        base = {'result': 'success', 'success': True, 'cmd': 'status'}
        for msg in ['state=ST_DEVED;sub=record;rec=0', 'state=ST_DEVED;sub=;rec=1',
                    'state=ST_DEVED;state=ST_DRAG_TEACH;sub=;rec=0',
                    'state=ST_UNKNOWN;sub=;rec=0', 'state=ST_DEVED',
                    'state=ST_DEVED;sub=;rec=0;extra=1']:
            with self.subTest(message=msg), self.assertRaises(ValueError):
                motion.inactive_value({**base, 'message': msg}, cfg)

    async def test_thirteen_degree_nearby_pose_uses_eight_second_minimum(self):
        current = snapshot()
        home = pose.make_pose([snapshot(100), snapshot(102), snapshot(104)])
        current['joint_state']['q'][0] -= .232
        result = motion.preflight(home, current, {'active': False},
            {'expected_working_mode': 'test-confirmed-mode', 'teach_state_pointer': '/active',
             'teach_inactive_value': False})
        self.assertAlmostEqual(result['max_joint_change_deg'], 13.292620847, places=5)
        self.assertEqual(result['duration_seconds'], 8)


    async def test_forty_five_degree_reset_uses_nineteen_seconds_and_waits_for_feedback(self):
        samples = [snapshot(100), snapshot(102), snapshot(104)]
        for sample in samples:
            sample['joint_state']['q'][5] = 0.02649974822998047
        home = pose.make_pose(samples)
        current = snapshot()
        current['joint_state']['q'][5] = -0.7661002278327942
        config = {'expected_working_mode': 'test-confirmed-mode',
                  'teach_state_pointer': '/active', 'teach_inactive_value': False}
        plan = motion.preflight(home, current, {'active': False}, config)
        self.assertAlmostEqual(plan['max_joint_change_deg'], 45.412633470567066)
        self.assertEqual(plan['duration_seconds'], 19)
        clock, sent = [1000.], []
        class Client(motion.RobotClient):
            def __init__(self):
                super().__init__(None)
                self.robot, self.info_at = current['robot'], clock[0]
                self.stamp = 104
                self.feedback_count = 0
            async def ready_state(self):
                return current, {'active': False}
            async def request(self, title, data, timeout=4):
                sent.append((title, data, timeout))
                return {'result': 'success', 'active': False}
            async def joints(self):
                self.feedback_count += 1
                self.stamp += 1
                clock[0] += .5
                self.info_at = clock[0]
                state = snapshot(self.stamp)['joint_state']
                state['q'][:14] = home['target_q']
                return state
        async def before(current): pass
        client = Client()
        with patch('motion.time.monotonic', side_effect=lambda: clock[0]), \
                patch('motion.asyncio.sleep', new=AsyncMock()):
            result = await client.move_and_verify(home, config, before)
        moves = [(data, timeout) for title, data, timeout in sent if title == 'request_movej']
        self.assertEqual(moves, [({'time': 19, 'joint': home['target_q']}, 23)])
        self.assertTrue(result['at_home'])
        self.assertEqual(result['duration_seconds'], 19)
        self.assertEqual(result['stable_samples'], 3)
        self.assertGreaterEqual(clock[0] - 1000, 19)
        self.assertGreaterEqual(client.feedback_count, 40)

    async def test_large_reset_still_refuses_outside_vendor_joint_limits(self):
        current = snapshot()
        current['joint_state']['q'][5] = -1.0
        home = pose.make_pose([snapshot(100), snapshot(102), snapshot(104)])
        with self.assertRaisesRegex(ValueError, 'Outside SDK limits'):
            motion.preflight(home, current, {'active': False},
                {'expected_working_mode': 'test-confirmed-mode',
                 'teach_state_pointer': '/active', 'teach_inactive_value': False})


if __name__ == '__main__':
    unittest.main()
