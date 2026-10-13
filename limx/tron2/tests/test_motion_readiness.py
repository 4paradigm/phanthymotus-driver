"""Reject unknown/incompatible vendor state before any physical command."""
import types
import unittest
from unittest.mock import AsyncMock
from vendor import RobotReader,MoveError,motion_readiness
from test_sensors import settings,info


class MotionReadinessTests(unittest.IsolatedAsyncioTestCase):
    def reader(self,**updates):
        r=RobotReader(settings());body=info();body['data'].update(updates)
        r.accept_info(body);r.ws=types.SimpleNamespace(closed=False,send_json=AsyncMock())
        return r

    async def test_each_invalid_missing_and_nonboolean_vendor_flag_never_sends(self):
        conditions=[{'working_mode':value} for value in (None,'teleop_mode','low_level_mode','unknown',1)]
        for key in ('working_mode_valid','collision_enable_valid','compliant_enable_valid','collision_enable'):
            conditions.extend({key:value} for value in (False,None,0,1,'true','false'))
        conditions.extend({'compliant_enable':value} for value in (True,None,0,1,'false'))
        for updates in conditions:
            with self.subTest(updates=updates):
                r=self.reader(**updates)
                with self.assertRaises(MoveError) as exc:await r.send_movej([.1]*14,8)
                self.assertFalse(exc.exception.motion_sent)
                self.assertEqual(exc.exception.code,'vendor_not_ready')
                r.ws.send_json.assert_not_called();self.assertIsNone(r.move_pending)
        for key in ('working_mode','working_mode_valid','collision_enable_valid','compliant_enable_valid','collision_enable','compliant_enable'):
            r=self.reader();del r.info[key]
            with self.assertRaises(MoveError):await r.send_movej([.1]*14,8)
            r.ws.send_json.assert_not_called()

    async def test_policy_accepts_commissioned_state_and_rejects_drag_without_requiring_drag_on(self):
        ready=info()['data']
        self.assertTrue(motion_readiness(ready)['ready'])
        drag={**ready,'compliant_enable':True}
        self.assertFalse(motion_readiness(drag)['ready'])
        self.assertTrue(ready['collision_enable']);self.assertFalse(ready['compliant_enable'])

    async def test_validity_does_not_replace_actual_enabled_state(self):
        r=self.reader(collision_enable=False,collision_enable_valid=True)
        with self.assertRaises(MoveError):await r.send_movej([.1]*14,8)
        r.ws.send_json.assert_not_called()
        self.assertFalse(motion_readiness({})['ready'])

if __name__=='__main__':unittest.main()
