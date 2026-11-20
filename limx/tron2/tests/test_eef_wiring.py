"""Exercise EEF discovery and arbitration through the driver's public dispatcher."""
import io
import json
import unittest
from unittest.mock import AsyncMock, Mock, patch

import check
from common.control import parse_descriptor
from device import Adapter, VERSION


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
