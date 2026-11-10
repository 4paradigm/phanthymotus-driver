"""Manufacturer mode transitions with synthetic identities and fake transport."""
import asyncio
import json
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

from stand import Stand, startup_match
from sit_down import SitDown
from vendor import MoveError


def robot(mode='DAMPING'):
    status = {'status': mode, 'imu': 'OK', 'motor': 'OK',
        'working_mode': 'developer_mode', 'working_mode_valid': True,
        'collision_enable': True, 'collision_enable_valid': True,
        'compliant_enable': False, 'compliant_enable_valid': True,
        'sw_version': 'TEST_FIRMWARE'}
    q = [0.] * 10
    q[3] = q[8] = -2.6
    def sample(name):
        data = dict(status) if name == 'robot_status' else {
            'joint_count': 10, 'q': list(q), 'dq': [0.] * 10}
        return {'available': True, 'received_at': time.time(), 'data': data}
    r = SimpleNamespace(serial='WF_TRON2A_TEST', configuration={'mode': 'wheel_foot'},
        connected=True, ws=SimpleNamespace(closed=False), sample=sample,
        move_pending=None, head_pending=None, _pose_servo=None)
    r.ws.send_json = AsyncMock()
    return r, status


class LowerControlTests(unittest.IsolatedAsyncioTestCase):
    async def test_stand_prepare_then_balance_without_velocity(self):
        r, status = robot()
        report = AsyncMock(return_value=True)
        owner = Stand(r, report)
        owner.POLL_SECONDS = .001
        async def send(packet):
            command = packet['title']
            status['status'] = 'STAND' if command == 'request_stand_mode' else 'WALK'
            owner.receive({**packet, 'title': command.replace('request_', 'response_'),
                'data': {'result': 'success'}})
        r.ws.send_json.side_effect = send
        owner.dispatch()
        await asyncio.wait_for(owner.task, 1)
        self.assertEqual(owner.state, 'completed')
        self.assertEqual([c.args[0]['title'] for c in r.ws.send_json.call_args_list],
                         ['request_stand_mode', 'request_walk_mode'])
        self.assertEqual(report.call_args.args[2], 'completed')

    async def test_sit_works_for_generic_wheel_unit_and_brakes_first(self):
        r, status = robot('WALK')
        owner = SitDown(r, AsyncMock(return_value=True))
        owner.STOP_SECONDS, owner.BRAKE_INTERVAL = .005, .001
        async def send(packet):
            if packet['title'] == 'request_sitdown':
                status['status'] = 'DAMPING'
                owner.receive({**packet, 'title': 'response_sitdown',
                    'data': {'result': 'success'}})
        r.ws.send_json.side_effect = send
        with patch.dict(os.environ, {}, clear=False):
            owner.dispatch()
            await asyncio.wait_for(owner.task, 1)
        packets = [c.args[0] for c in r.ws.send_json.call_args_list]
        self.assertEqual(owner.state, 'completed')
        self.assertGreater(len(packets), 1)
        self.assertEqual(packets[-1]['title'], 'request_sitdown')
        self.assertTrue(all(p['title'] == 'request_twist' and
            p['data'] == {'x': 0., 'y': 0., 'z': 0.} for p in packets[:-1]))

    async def test_already_folded_is_verification_only(self):
        r, _ = robot('SIT')
        owner = SitDown(r, AsyncMock(return_value=True))
        owner.dispatch()
        await asyncio.wait_for(owner.task, 1)
        self.assertEqual(owner.state, 'completed')
        r.ws.send_json.assert_not_called()

    def test_unknown_startup_requires_an_external_profile(self):
        r, _ = robot('UNKNOW')
        owner = Stand(r, AsyncMock())
        with patch.dict(os.environ, {'TRON2_STARTUP_PROFILE': ''}):
            self.assertFalse(startup_match(r)['matched'])
            with self.assertRaises(MoveError):
                owner.dispatch()
        r.ws.send_json.assert_not_called()
        profile = {'name': 'synthetic_test', 'accid': r.serial, 'mode': 'wheel_foot',
            'sw_version': 'TEST_FIRMWARE', 'working_mode': 'developer_mode',
            'status': 'UNKNOW', 'folded_statuses': ['UNKNOW'],
            'stationary_dq_limit_rad_s': .1, 'pose_indices': [3, 8],
            'pose_ranges': {'3': [-2.7, -2.5], '8': [-2.7, -2.5]}}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'profile.json'
            path.write_text(json.dumps(profile))
            with patch.dict(os.environ, {'TRON2_STARTUP_PROFILE': str(path)}):
                self.assertTrue(startup_match(r)['matched'])
                r.serial = 'WF_TRON2A_OTHER'
                self.assertFalse(startup_match(r)['matched'])


if __name__ == '__main__':
    unittest.main()
