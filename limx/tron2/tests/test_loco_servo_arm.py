"""Real ControlSink + fake DDS/WebSocket; no robot motion or ROS required."""
import asyncio
from contextlib import contextmanager
import json
import math
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock

from arm_servo import ArmServoPlugin, Limiter, build_descriptor, MAX_VELOCITY, MAX_ACCELERATION, CONTROL_JOINT_NAMES
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
        self.gripper_sent = []
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

    async def send_arm_grippers(self, openings, token):
        assert self._pose_servo is token
        self.gripper_sent.append(tuple(openings))
        return {'result': 'success'}


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
        'priority': 50, 'mode': 'joint_position', 'dof': 18,
        'values': q[:7] + [.2] + q[7:14] + [.8] + q[14:]}


async def feed(p, msg):
    p.publisher.callback(json.dumps(msg))
    await asyncio.sleep(.015)


class ContractTests(unittest.TestCase):
    def test_descriptor_and_card_follow_r1_structure(self):
        d = parse_descriptor(build_descriptor())
        self.assertEqual(d.dof, 18)
        self.assertEqual(list(d.joint_names), list(CONTROL_JOINT_NAMES))
        t = plugin().get_tool()
        self.assertFalse(t['multiInstance'])
        self.assertEqual(t['topic_in'][0]['format'], 'control/joint')
        self.assertEqual(set(t['inputSchema']['x-action-params']), {'pause', 'resume', 'config'})
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
        self.assertEqual((await self.start())['state'], 'ready')
        await asyncio.sleep(.025)
        self.assertEqual(self.p.reader.sent, [])
        self.assertEqual((await self.p.call({'action': 'stop'}))['state'], 'idle')
        await self.start()
        self.assertEqual(self.p.reader.sent, [])

    async def test_valid_stream_controls_head_and_cancels_on_pause(self):
        await self.start()
        m = message();m['values'][16] += .01;m['values'][17] -= .01
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

    async def test_official_layout_routes_both_grippers_and_pause_stops_both(self):
        await self.start()
        await feed(self.p, message())
        self.assertEqual(self.p.reader.gripper_sent, [(.2, .8)])
        self.assertEqual(len(self.p.reader.sent[-1]), 16)
        await self.p.call({'action': 'pause'})
        counts = (len(self.p.reader.sent), len(self.p.reader.gripper_sent))
        await feed(self.p, message(2))
        self.assertEqual(counts, (len(self.p.reader.sent), len(self.p.reader.gripper_sent)))
        self.assertIsNone(self.p.gripper_task)

    async def test_gripper_ack_wait_never_blocks_joint_loop_and_is_joined(self):
        await self.start()
        begun = asyncio.Event()
        async def delayed(openings, token):
            begun.set()
            await asyncio.sleep(10)
        self.p.reader.send_arm_grippers = delayed
        await feed(self.p, message())
        await asyncio.wait_for(begun.wait(), 1)
        before = len(self.p.reader.sent)
        await asyncio.sleep(.025)
        self.assertGreater(len(self.p.reader.sent), before)
        await self.p.preempt()
        self.assertIsNone(self.p.gripper_task)
        self.assertIsNone(self.p.reader._pose_servo)

    async def test_invalid_gripper_range_and_legacy_16_rejected_before_any_output(self):
        await self.start()
        for value in (-.1, 1.1, True, float('nan')):
            m = message(); m['values'][7] = value
            await feed(self.p, m)
        m = message(); m.update(dof=16, values=q0())
        await feed(self.p, m)
        self.assertEqual(self.p.reader.sent, [])
        self.assertEqual(self.p.reader.gripper_sent, [])

    async def test_gripper_failure_pauses_entire_card_without_retry(self):
        await self.start()
        self.p.reader.send_arm_grippers = AsyncMock(side_effect=MoveError('vendor rejected', True))
        await feed(self.p, message())
        self.assertTrue(self.p.paused)
        count = len(self.p.reader.sent)
        await feed(self.p, message(2))
        self.assertEqual(len(self.p.reader.sent), count)
        self.p.reader.send_arm_grippers.assert_awaited_once()

    async def test_gripper_ack_backpressure_keeps_only_latest_target(self):
        await self.start()
        release = asyncio.Event()
        sent = []
        async def delayed(openings, token):
            sent.append(tuple(openings))
            await release.wait()
        self.p.reader.send_arm_grippers = delayed
        await feed(self.p, message())
        m = message(2); m['values'][7], m['values'][15] = .21, .79
        await feed(self.p, m)
        m = message(3); m['values'][7], m['values'][15] = .22, .78
        await feed(self.p, m)
        self.assertEqual(sent, [(.2, .8)])
        release.set()
        deadline = time.monotonic() + .15
        while len(sent) < 2 and time.monotonic() < deadline:
            await asyncio.sleep(.005)
        self.assertEqual(sent, [(.2, .8), (.22, .78)])

    async def test_gripper_dry_run_is_inert_and_busy_owner_blocks_start(self):
        self.p.dry_run = True
        await self.start()
        await feed(self.p, message())
        self.assertEqual(self.p.reader.sent, [])
        self.assertEqual(self.p.reader.gripper_sent, [])
        await self.p.close()
        self.p.adapter.grippers = {'left': SimpleNamespace(active={}, recovering=True, pose_step=None, waiting=None)}
        with self.assertRaises(ValueError):
            await self.start()


class AdapterTests(unittest.IsolatedAsyncioTestCase):
    async def test_public_mcp_lifecycle_and_declared_config(self):
        from device import Adapter
        a = Adapter(lambda *args: None, {'enabled_cards': ['loco_servo_arm'],
            'poll_interval_seconds': .5, 'response_timeout_seconds': 5,
            'stale_after_seconds': 5, 'serial': ''})
        a.arm_servo.publisher = Bus()
        async def rpc(method, params):
            request = SimpleNamespace(json=AsyncMock(return_value={
                'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params}))
            response = await a.rpc(request)
            return json.loads(response.body)['result']
        async def call(**args):
            result = await rpc('tools/call', {'name': 'loco_servo_arm', 'arguments': args})
            return result['isError'], json.loads(result['content'][0]['text'])
        try:
            tools = (await rpc('tools/list', {}))['tools']
            card = next(t for t in tools if t['name'] == 'loco_servo_arm')
            schema = card['inputSchema']
            self.assertIn('config', schema['properties']['action']['enum'])
            self.assertEqual(schema['properties']['dry_run']['type'], 'boolean')
            self.assertEqual(schema['x-action-params']['config']['params'], ['dry_run'])
            error, result = await call(action='start', input_topic='/test/public_servo')
            self.assertFalse(error); self.assertEqual(result['state'], 'ready')
            error, result = await call(action='info')
            self.assertFalse(error); self.assertEqual(result['state'], 'ready')
            for value in (None, 'true', 1):
                error, result = await call(action='config', dry_run=value)
                self.assertTrue(error)
                self.assertFalse(a.arm_servo.dry_run)
            error, result = await call(action='config', dry_run=True)
            self.assertFalse(error); self.assertTrue(result['dry_run'])
            self.assertTrue(result['paused']); self.assertEqual(result['state'], 'ready')
            error, result = await call(action='start', input_topic='/test/public_servo')
            self.assertFalse(error); self.assertEqual(result['state'], 'ready')
            self.assertTrue(result['paused'])  # Repeated start never resumes a paused stream.
            error, result = await call(action='stop')
            self.assertFalse(error); self.assertEqual(result['state'], 'idle')
            self.assertFalse(result['paused'])
            self.assertEqual(a.reader.gripper_pending, {})
        finally:
            await a.arm_servo.close()

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
        a.enabled.append('gripper')
        a.grippers['gripper_left'].dispatch = AsyncMock(return_value={'status': 'accepted'})
        await a.execute_tool('gripper', {'side': 'left', 'opening_mm': 20, 'speed': 50, 'force': 50})
        a.arm_servo.preempt.assert_awaited_once()
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

    async def test_paired_gripper_transport_requires_same_owner_and_exact_normalized_units(self):
        card, socket = real_transport_plugin()
        r = card.reader
        with r.pose_servo({'motion_tool': 'test'}) as token:
            await r.send_arm_grippers((0., 1.), token)
            self.assertEqual(socket.packets[-1]['data'], {'left_opening': 0, 'right_opening': 100,
                'left_speed': 100, 'right_speed': 100, 'left_force': 50, 'right_force': 50})
            for values in ((-1., 0.), (0., 101.), (True, 0.), (0.,)):
                with self.assertRaises(MoveError):
                    await r.send_arm_grippers(values, token)
            r.configuration = configuration('WF_TRON2A_TEST')
            with self.assertRaises(MoveError):
                await r.send_arm_grippers((.1, .2), token)
        with self.assertRaises(MoveError):
            await r.send_arm_grippers((.1, .2), token)
        self.assertEqual(len(socket.packets), 1)



"""Synthetic commands -> real card/sink/interpolator/transport -> captured WS.

No policy, no robot socket. The optional DDS test uses a private ROS context
inside a network-isolated container, never the live robot namespace.
"""
import asyncio
import json
import math
import os
import time
from types import SimpleNamespace
import unittest

from arm_servo import ArmServoPlugin, SERVO_LOWER, SERVO_UPPER
from state import configuration
from vendor import RobotReader



class CapturedSocket:
    closed = False

    def __init__(self, feedback):
        self.feedback = feedback
        self.packets = []

    async def send_json(self, packet):
        # Deep-copy the actual encoded request and simulate matching encoder feedback.
        saved = json.loads(json.dumps(packet, allow_nan=False))
        self.packets.append(saved)
        if saved['title'] == 'request_servoj':
            self.feedback.q = list(saved['data']['q'])
        else:
            expected, future = self.reader.gripper_pending[saved['guid']]
            assert expected == 'response_set_limx_2fclaw_cmd'
            future.set_result({'result': 'success'})


def real_transport_plugin():
    feedback = Reader()
    reader = RobotReader({'serial': '', 'poll_interval_seconds': .5,
        'stale_after_seconds': 5, 'response_timeout_seconds': 5})
    reader.connected = True
    reader.serial = 'DACH_TRON2A_TEST'
    reader.configuration = configuration(reader.serial)
    reader.ws = CapturedSocket(feedback)
    reader.ws.reader = reader
    reader.sample = feedback.sample
    adapter = SimpleNamespace(reader=reader, motion=SimpleNamespace(active=None),
        head=SimpleNamespace(active=None), follow_runtime=None)
    card = ArmServoPlugin(adapter)
    card.publisher = Bus()
    return card, reader.ws


def target(seq=1):
    m = message(seq=seq, ttl=180)
    for index, delta in ((0, .01), (8, -.01), (16, .01), (17, -.01)):
        m['values'][index] += delta
    return m


class PacketPathTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.card, self.socket = real_transport_plugin()
        await self.card.call({'action': 'start', 'input_topic': '/test/servo_packets'})

    async def asyncTearDown(self):
        await self.card.close()

    async def publish(self, value):
        self.card.publisher.callback(json.dumps(value))
        for _ in range(20):
            await asyncio.sleep(.005)
            if self.socket.packets:
                return

    async def test_receive_to_real_vendor_packet_and_joint_order(self):
        await self.publish(target())
        self.assertGreater(len(self.socket.packets), 0)
        await asyncio.sleep(.015)
        # Let an in-flight send finish before comparing submission counters.
        self.card._clear()
        await asyncio.wait_for(asyncio.shield(self.card.output_task), 1)
        grippers = [p for p in self.socket.packets if p['title'] == 'request_set_limx_2fclaw_cmd']
        self.assertEqual(len(grippers), 1)
        self.assertEqual(grippers[0]['data'], {'left_opening': 20, 'right_opening': 80,
            'left_speed': 100, 'right_speed': 100, 'left_force': 50, 'right_force': 50})
        servo_packets = [p for p in self.socket.packets if p['title'] == 'request_servoj']
        for packet in servo_packets:
            self.assertEqual(set(packet), {'title', 'accid', 'guid', 'timestamp', 'data'})
            self.assertEqual(packet['title'], 'request_servoj')
            self.assertEqual(packet['accid'], 'DACH_TRON2A_TEST')
            self.assertIsInstance(packet['timestamp'], int)
            self.assertTrue(packet['guid'])
            self.assertEqual(set(packet['data']), {'q', 'filter_ratio'})
            self.assertEqual(packet['data']['filter_ratio'], 1.0)
            q = packet['data']['q']
            self.assertEqual(len(q), 16)
            self.assertTrue(all(math.isfinite(v) and lo <= v <= hi
                for v, lo, hi in zip(q, SERVO_LOWER, SERVO_UPPER)))
            self.assertGreater(q[0], q0()[0])
            self.assertLess(q[7], q0()[7])
            self.assertGreater(q[14], q0()[14])
            self.assertLess(q[15], q0()[15])
            for i in set(range(16)) - {0, 7, 14, 15}:
                self.assertAlmostEqual(q[i], q0()[i], places=12)
        self.assertEqual(self.card.info()['commands_submitted'], len(servo_packets))
        self.assertEqual(self.card.info()['gripper_commands_acknowledged'], 1)
        self.assertFalse(self.card.info()['vendor_acknowledged'])
        self.assertFalse(self.card.info()['arrival_verified'])

    async def test_rejected_input_cannot_reach_real_sender(self):
        for patch in ({'mode': 'twist'}, {'dof': 14}, {'stamp_ms': 1},
                      {'values': [99.] * 16}, {'values': [float('nan')] * 16}):
            m = target();m.update(patch)
            self.card.publisher.callback(json.dumps(m))
            await asyncio.sleep(.01)
            self.assertEqual(self.socket.packets, [])
        self.assertEqual(self.card.commands, 0)

    async def test_expiry_and_pause_cancel_actual_packet_output(self):
        await self.publish(target())
        self.assertGreater(len(self.socket.packets), 0)
        await asyncio.sleep(.25)
        count = len(self.socket.packets)
        await asyncio.sleep(.025)
        self.assertEqual(len(self.socket.packets), count)
        self.assertIsNone(self.card.reader._pose_servo)
        await self.card.call({'action': 'pause'})
        self.card.publisher.callback(json.dumps(target(2)))
        await asyncio.sleep(.02)
        self.assertEqual(len(self.socket.packets), count)


@unittest.skipUnless(os.environ.get('TRON2_TEST_PRIVATE_DDS') == '1',
                     'requires explicitly isolated ROS/DDS test container')
class PrivateDdsPacketTests(unittest.IsolatedAsyncioTestCase):
    async def test_real_dds_to_captured_vendor_output(self):
        from ros2 import JsonPublisher
        # Separate network and IPC namespaces are mandatory in the runner.
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '213')
        topic = '/tron2_private_test/servo_command'
        card, socket = real_transport_plugin()
        bus = JsonPublisher([topic])
        card.publisher = bus
        try:
            await card.call({'action': 'start', 'input_topic': topic})
            deadline = time.monotonic() + 4
            while bus.publishers[topic].get_subscription_count() < 1 and time.monotonic() < deadline:
                await asyncio.sleep(.02)
            self.assertGreater(bus.publishers[topic].get_subscription_count(), 0)
            for seq in range(1, 9):
                bus.publish(topic, target(seq))
                await asyncio.sleep(.04)
            self.assertGreater(card.applied, 0)
            self.assertGreater(len(socket.packets), 0)
            q = [p for p in socket.packets if p['title'] == 'request_servoj'][-1]['data']['q']
            self.assertEqual(len(q), 16)
            self.assertGreater(q[0], q0()[0]);self.assertLess(q[7], q0()[7])
            self.assertGreater(q[14], q0()[14]);self.assertLess(q[15], q0()[15])
            await card.call({'action': 'pause'})
            count = len(socket.packets)
            bus.publish(topic, target(9))
            await asyncio.sleep(.04)
            self.assertEqual(len(socket.packets), count)
            print('DDS_PACKET_PROOF '+json.dumps({'received_targets': card.applied,
                'captured_servoj_packets': count, 'dof': len(q),
                'joint_directions_verified': ['left_arm', 'right_arm', 'head_pitch', 'head_yaw'],
                'robot_network_opened': False, 'pause_prevents_output': True}))
        finally:
            await card.close()
            bus.close()


if __name__ == '__main__':
    unittest.main()
