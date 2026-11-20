"""Exercise EEF discovery and arbitration through the driver's public dispatcher."""
import asyncio
import io
import json
import time
import unittest
from unittest.mock import AsyncMock, Mock, patch

import check
from common.control import parse_descriptor
from device import Adapter, VERSION
from vendor import RobotReader


class EefWiringTests(unittest.IsolatedAsyncioTestCase):
    def adapter(self, *cards):
        settings = {'enabled_cards': list(cards), 'serial': '', 'stale_after_seconds': 5}
        return Adapter(Mock(), settings)

    async def test_sensor_and_servo_can_be_enabled_separately(self):
        a = self.adapter('eef_state')
        self.assertEqual([t['name'] for t in a.tools()], ['eef_state'])
        info = await a.execute_tool('eef_state', {'action': 'info'})
        self.assertEqual(info['state'], 'idle')
        self.assertEqual(info['topic_out'][0]['topic'], '/limx_tron2/state/eef')
        self.assertFalse(info['available'])
        self.assertIsNone(a.servo_eef)
        self.assertEqual((await a.execute_tool('eef_state', {'action': 'stop'}))['state'], 'idle')

        a = self.adapter('servo_eef')
        self.assertEqual([t['name'] for t in a.tools()], ['servo_eef'])
        self.assertIs(a.servo_eef.state, a.eef_state)
        port = a.tools()[0]['topic_in'][0]
        self.assertEqual((port['topic'], port['format']), ('/limx_tron2/control/eef', 'control/eef'))
        interface = (await a.execute_tool('servo_eef', {'action': 'info'}))['control_interface']
        self.assertEqual(interface['control_interface'], 'motus.control/1')
        descriptor = parse_descriptor(interface)
        self.assertEqual((descriptor.dof, descriptor.frame), (16, 'base_Link'))
        self.assertEqual([(g.offset, g.count, g.mode) for g in descriptor.groups],
                         [(0, 7, 'eef_pose'), (7, 1, 'joint_position'),
                          (8, 7, 'eef_pose'), (15, 1, 'joint_position')])
        self.assertGreater(descriptor.max_obs_age_ms, 0)
        self.assertGreater(descriptor.watchdog_ms, 0)
        with self.assertRaises(ValueError):
            await a.execute_tool('servo_eef', {'action': 'start', 'dry_run': True})
        self.assertFalse(a.servo_eef.running)

    async def test_streams_handover_by_pause_and_explicit_commands_preempt(self):
        a = self.adapter('servo_eef', 'loco_servo_arm', 'arm_control')
        a.servo_eef.publisher = a.arm_servo.publisher = Mock()
        try:
            await a.execute_tool('servo_eef', {'action': 'start'})
            joint_start = {'action': 'start', 'input_topic': '/test/joint'}
            with self.assertRaisesRegex(ValueError, 'other arm servo'):
                await a.execute_tool('loco_servo_arm', joint_start)
            await a.execute_tool('servo_eef', {'action': 'pause'})
            await a.execute_tool('loco_servo_arm', joint_start)
            with self.assertRaisesRegex(ValueError, 'other arm servo'):
                await a.execute_tool('servo_eef', {'action': 'resume'})
            await a.execute_tool('loco_servo_arm', {'action': 'stop'})
            await a.execute_tool('servo_eef', {'action': 'resume'})
            a.motion.dispatch = AsyncMock(return_value={'action_id': 'test'})
            await a.execute_tool('arm_control', {'action': 'move', 'joint': [0.] * 14, 'time': 6})
            self.assertTrue(a.servo_eef.paused)
            a.motion.dispatch.assert_awaited_once()
        finally:
            await a.servo_eef.close()
            await a.arm_servo.close()

    async def test_native_sample_anchors_single_move_and_other_side_holds(self):
        reader = RobotReader({'serial': 'DACH_TRON2A_TEST'})
        reader.connected, reader.ws = True, Mock(closed=False)
        reader.configuration = {'mode': 'upper_limb', 'state': 'identified'}
        poses = {'result': 'success', 'left_position': [.48, .2, -.38], 'left_quat': [1., 0., 0., 0.],
                 'right_position': [.41, -.21, -.37], 'right_quat': [.8, 0., -.6, 0.]}
        reader.read_move_pose = AsyncMock(side_effect=lambda: dict(poses, timestamp=time.time_ns() // 1_000_000))
        reader.read_gripper = AsyncMock(side_effect=lambda: {'result': 'success',
            'timestamp': time.time_ns() // 1_000_000, 'left_opening': 40, 'right_opening': 47})
        reader.sample = Mock(return_value={'available': True, 'data': {'working_mode': 'developer_mode',
            'working_mode_valid': True, 'motor': 'OK', 'collision_enable_valid': True, 'collision_enable': True,
            'compliant_enable_valid': True, 'compliant_enable': False}})
        reader.send_move_p = AsyncMock(return_value={'status': 'accepted'})
        reader.send_gripper = AsyncMock()
        frames = asyncio.Queue()
        a = Adapter(lambda topic, packet: frames.put_nowait((topic, packet)),
                    {'enabled_cards': ['eef_state', 'servo_eef']}, reader=reader)
        bus = a.servo_eef.publisher = Mock()
        sensor = asyncio.create_task(a.eef_state.run())
        try:
            await a.execute_tool('eef_state', {'action': 'start'})
            await a.execute_tool('servo_eef', {'action': 'start', 'input_topic': '/test/control/eef'})
            self.assertEqual(bus.subscribe_json.call_args.args[0], '/test/control/eef')
            callback = bus.subscribe_json.call_args.args[1]
            topic, state = await asyncio.wait_for(frames.get(), 1.)
            self.assertEqual(topic, '/limx_tron2/state/eef')
            target = state['eef'].copy()
            target[0] -= .002
            for seq in (1, 2):
                callback(json.dumps({'schema': 'motus.control/1', 'source': 'native-eef-test',
                    'seq': seq, 'ttl_ms': 200, 'stamp_ms': time.time_ns() // 1_000_000,
                    'obs_stamp_ms': state['stamp_ms'], 'mode': 'eef_pose', 'dof': 16, 'values': target}))
                await asyncio.sleep(.03)
            reader.send_move_p.assert_awaited_once()
            left, right = reader.send_move_p.await_args.args[:2]
            self.assertEqual(list(left), target[:7])
            self.assertEqual(list(right), state['eef'][8:15])
            reader.send_gripper.assert_not_awaited()
        finally:
            await a.servo_eef.close()
            sensor.cancel()
            await asyncio.gather(sensor, return_exceptions=True)

    async def test_readonly_checker_for_standalone_cards(self):
        for name in ('eef_state', 'servo_eef'):
            with self.subTest(card=name):
                adapter = self.adapter(name)
                info = await adapter.execute_tool(name, {'action': 'info'})
                reply = {'content': [{'type': 'text', 'text': json.dumps(info)}]}
                health = io.BytesIO(json.dumps({'version': VERSION, 'registered_mcp_id': 'test'}).encode())
                with patch.object(check.OPENER, 'open', return_value=health), \
                        patch.object(check, 'rpc', side_effect=[{'tools': adapter.tools()}, reply]) as rpc:
                    check.run_check()
                rpc.assert_called_with('tools/call', {'name': name, 'arguments': {'action': 'info'}})
