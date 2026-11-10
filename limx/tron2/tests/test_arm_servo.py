"""Real ControlSink + fake DDS/WebSocket; no robot motion or ROS required."""
import asyncio
from contextlib import contextmanager
import json
import math
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock

from arm_servo import ArmServoPlugin, Limiter, build_descriptor, MAX_VELOCITY, MAX_ACCELERATION
from common.control import parse_descriptor
from reset import ARM_NAMES
from skeleton import JOINT_NAMES
from state import configuration
from vendor import RobotReader, MoveError


def q0():
    return [0.] * 14 + [.12, -.2]


class Reader:
    def __init__(self):
        self.configuration = configuration('DACH_TRON2A_TEST')
        self.move_pending = self.head_pending = self._pose_servo = None
        self.sent = []
        self.q = q0()
        self.frozen = False
        self.age = .001
        self.stamp = 1.
        self.lease = 0
        self.connected = True

    def sample(self, name):
        base = {'accid': 'DACH_TRON2A_TEST', 'available': True, 'connected': True,
            'sample_status': 'recently_received', 'received_at': time.time(),
            'reception_age_seconds': self.age, 'measurement_time_verified': False}
        if name == 'robot_status':
            base['data'] = {'working_mode': 'developer_mode', 'working_mode_valid': True,
                'collision_enable_valid': True, 'collision_enable': True,
                'compliant_enable_valid': True, 'compliant_enable': False, 'motor': 'OK'}
        else:
            if not self.frozen:
                self.stamp += .001
            base['data'] = {'q': list(self.q), 'dq': [0.] * 16, 'tau': [0.] * 16,
                'names': list(JOINT_NAMES), 'robot_timestamp': self.stamp}
        return base

    @contextmanager
    def pose_feedback(self):
        self.lease += 1
        try:
            yield
        finally:
            self.lease -= 1

    @contextmanager
    def pose_servo(self, job):
        if self._pose_servo is not None:
            raise MoveError('busy', False)
        token = {'job': job, 'error': None}
        self._pose_servo = token
        try:
            yield token
        finally:
            self._pose_servo = None

    async def send_arm_servoj(self, q, token):
        assert self._pose_servo is token
        self.sent.append(list(q))
        self.q = list(q)


class Bus:
    def subscribe_json(self, topic, callback):
        self.callback = callback
        return object()

    def unsubscribe(self, subscription):
        self.removed = True


def plugin(dry=False):
    a = SimpleNamespace(reader=Reader(), motion=SimpleNamespace(active=None),
        head=SimpleNamespace(active=None), follow_runtime=None)
    p = ArmServoPlugin(a, {'dry_run': dry})
    p.publisher = Bus()
    return p


def message(seq=1, ttl=150, offset=.01):
    q = q0()
    q[0] = offset
    return {'schema': 'motus.control/1', 'seq': seq, 'stamp_ms': int(time.time()*1000),
        'obs_stamp_ms': int(time.time()*1000), 'ttl_ms': ttl, 'source': 'test',
        'priority': 50, 'mode': 'joint_position', 'dof': 16, 'values': q}


async def feed(p, msg):
    p.publisher.callback(json.dumps(msg))
    await asyncio.sleep(.015)


class ContractTests(unittest.TestCase):
    def test_descriptor_and_card_follow_r1_structure(self):
        d = parse_descriptor(build_descriptor())
        self.assertEqual(d.dof, 16)
        self.assertEqual(list(d.joint_names), JOINT_NAMES)
        t = plugin().get_tool()
        self.assertFalse(t['multiInstance'])
        self.assertEqual(t['topic_in'][0]['format'], 'control/joint')
        self.assertEqual(set(t['inputSchema']['x-action-params']), {'pause', 'resume'})
        self.assertIsNone(build_descriptor()['force_torque'])

    def test_first_target_and_reversal_slew_are_bounded(self):
        limiter = Limiter([0.] * 16)
        previous, velocity = 0., 0.
        for n in range(600):
            target = [.2 if n < 300 else -.2] * 16
            q = limiter.step(target, 1/300)
            v = (q[0]-previous)*300
            self.assertLessEqual(abs(v), MAX_VELOCITY+1e-8)
            self.assertLessEqual(abs(v-velocity), MAX_ACCELERATION/300+1e-8)
            previous, velocity = q[0], v
        with self.assertRaises(ValueError):
            limiter.step([0.]*16, .1)
        with self.assertRaises(ValueError):
            Limiter([99.]*16)


class RuntimeTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.p = plugin()

    async def asyncTearDown(self):
        await self.p.close()

    async def start(self):
        return await self.p.call({'action': 'start', 'input_topic': '/test/control'})

    async def test_start_stop_inert_and_restartable(self):
        with self.assertRaises(ValueError):
            await self.p.call({'action': 'start'})
        self.assertEqual((await self.start())['state'], 'running')
        await asyncio.sleep(.025)
        self.assertEqual(self.p.reader.sent, [])
        self.assertEqual((await self.p.call({'action': 'stop'}))['state'], 'idle')
        await self.start()
        self.assertEqual(self.p.reader.sent, [])

    async def test_valid_stream_controls_head_and_cancels_on_pause(self):
        await self.start()
        m = message();m['values'][14] += .01;m['values'][15] -= .01
        await feed(self.p, m)
        self.assertGreater(len(self.p.reader.sent), 0)
        self.assertGreater(self.p.reader.sent[-1][14], q0()[14])
        self.assertLess(self.p.reader.sent[-1][15], q0()[15])
        self.assertLess(self.p.reader.sent[0][0], .001)
        await self.p.preempt()
        count = len(self.p.reader.sent)
        await feed(self.p, message(2))
        self.assertEqual(len(self.p.reader.sent), count)
        self.assertIsNone(self.p.reader._pose_servo)
        self.assertEqual(self.p.reader.lease, 0)
        await self.p.call({'action': 'resume'})
        await feed(self.p, message(3))
        self.assertGreater(len(self.p.reader.sent), count)

    async def test_bad_contract_expiry_and_sequence_never_replay(self):
        await self.start()
        for patch in ({'dof': 10}, {'mode': 'twist'}, {'stamp_ms': 1}, {'gripper': .5}, {'values': [99.]*14}):
            m = message();m.update(patch)
            await feed(self.p, m)
        self.assertEqual(self.p.reader.sent, [])
        await feed(self.p, message(10))
        self.assertGreater(len(self.p.reader.sent), 0)
        await feed(self.p, message(9))
        self.assertEqual(self.p.last_outcome['verdict'], 'dropped')
        await asyncio.sleep(.22)
        n = len(self.p.reader.sent)
        await asyncio.sleep(.04)
        self.assertEqual(len(self.p.reader.sent), n)
        self.assertIsNone(self.p.reader._pose_servo)

    async def test_constructed_under_lower_configuration_never_sends(self):
        self.p.reader.configuration = configuration('WF_TRON2A_TEST')
        await self.start()  # Wiring only, not a motor operation.
        await feed(self.p, message())
        self.assertEqual(self.p.reader.sent, [])
        self.assertEqual(self.p.last_outcome['verdict'], 'rejected')

    async def test_stale_feedback_and_owner_conflict(self):
        self.p.adapter.motion.active = {'state': 'moving'}
        with self.assertRaises(ValueError):
            await self.start()
        self.p.adapter.motion.active = None
        await self.start()
        self.p.reader.age = 1.
        await feed(self.p, message())
        self.assertEqual(self.p.reader.sent, [])

    async def test_dry_run_receives_but_never_sends(self):
        await self.p.close()
        self.p = plugin(True)
        await self.start()
        await feed(self.p, message())
        self.assertEqual(self.p.applied, 1)
        self.assertEqual(self.p.reader.sent, [])

    async def test_vendor_failure_stops_without_retry(self):
        await self.start()
        await feed(self.p, message())
        self.p.reader._pose_servo['error'] = 'vendor failed'
        await asyncio.sleep(.02)
        self.assertTrue(self.p.paused)
        count = len(self.p.reader.sent)
        await feed(self.p, message(2))
        self.assertEqual(len(self.p.reader.sent), count)
        self.assertIsNone(self.p.reader._pose_servo)

    async def test_stop_waits_for_inflight_writer(self):
        await self.start()
        begun = asyncio.Event()
        async def blocked(q, token):
            begun.set()
            await asyncio.sleep(10)
        self.p.reader.send_arm_servoj = blocked
        self.p.publisher.callback(json.dumps(message()))
        await asyncio.wait_for(begun.wait(), 1)
        await self.p.preempt()
        self.assertIsNone(self.p.reader._pose_servo)
        self.assertEqual(self.p.reader.lease, 0)

    async def test_config_dry_run_pauses_and_old_subscription_cannot_restart(self):
        await self.start()
        old = self.p.publisher.callback
        await feed(self.p, message())
        await self.p.call({'action': 'config', 'dry_run': True})
        self.assertTrue(self.p.paused)
        self.assertIsNone(self.p.reader._pose_servo)
        await self.p.call({'action': 'stop'})
        await self.start()
        before = self.p.applied
        old(json.dumps(message(999)))
        await asyncio.sleep(.01)
        self.assertEqual(self.p.applied, before)


class AdapterTests(unittest.IsolatedAsyncioTestCase):
    async def test_real_adapter_discovery_and_manual_action_handover(self):
        from device import Adapter
        a = Adapter(lambda *args: None, {'enabled_cards': ['loco_servo_arm', 'arm_control', 'joints'],
            'poll_interval_seconds': .5, 'response_timeout_seconds': 5,
            'stale_after_seconds': 5, 'serial': ''})
        self.assertIn('loco_servo_arm', {t['name'] for t in a.tools()})
        self.assertEqual((await a.execute_tool('loco_servo_arm', {'action': 'info'}))['state'], 'idle')
        a.arm_servo.preempt = AsyncMock()
        a.motion.dispatch = AsyncMock(return_value={'status': 'accepted'})
        result = await a.execute_tool('arm_control', {'action': 'move', 'joint': [0.]*14, 'time': 1})
        self.assertEqual(result['status'], 'accepted')
        a.arm_servo.preempt.assert_awaited_once()
        a.arm_servo.preempt.reset_mock()
        a.enabled.append('head_visual_follow')
        a.follow_runtime = SimpleNamespace(call=AsyncMock(return_value={'state': 'paused'}))
        await a.execute_tool('head_visual_follow', {'action': 'start'})
        a.arm_servo.preempt.assert_not_awaited()
        await a.arm_servo.close()


class VendorTests(unittest.IsolatedAsyncioTestCase):
    async def test_real_packet_preserves_head_requires_owner_and_lower_refused(self):
        r = RobotReader({'serial': '', 'poll_interval_seconds': .5,
            'stale_after_seconds': 5, 'response_timeout_seconds': 5})
        r.configuration = configuration('DACH_TRON2A_TEST')
        r.connected = True;r.serial = 'DACH_TRON2A_TEST'
        r.ws = SimpleNamespace(closed=False, send_json=AsyncMock())
        r.sample = Reader().sample
        with r.pose_servo({}) as token:
            await r.send_arm_servoj(q0(), token)
            packet = r.ws.send_json.call_args.args[0]
            self.assertEqual(packet['title'], 'request_servoj')
            self.assertEqual(packet['data'], {'q': q0(), 'filter_ratio': 1.0})
            r.configuration = configuration('WF_TRON2A_TEST')
            with self.assertRaises(MoveError):
                await r.send_arm_servoj(q0(), token)
        self.assertEqual(r.ws.send_json.await_count, 1)


if __name__ == '__main__':
    unittest.main()
