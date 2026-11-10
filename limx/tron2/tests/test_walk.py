"""No hardware: timing, four signed velocities, stop and ownership behavior."""
import asyncio
import ast
import math
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock
from vendor import MoveError
from walk import Walk,duration,DIRECTIONS

class WalkTests(unittest.IsolatedAsyncioTestCase):
    def make(self,mode='WALK'):
        data={'status':mode,'working_mode':'developer_mode','working_mode_valid':True,
            'imu':'OK','motor':'OK','collision_enable':True,'collision_enable_valid':True,
            'compliant_enable':False,'compliant_enable_valid':True}
        r=SimpleNamespace(connected=True,serial='WF_TRON2A_TEST',configuration={'mode':'wheel_foot'},
            ws=SimpleNamespace(closed=False,send_json=AsyncMock()),move_pending=None,head_pending=None,
            _pose_servo=None,sample=lambda n:{'available':True,'data':data})
        report=AsyncMock(return_value=True);w=Walk(r,report);w.STOP_SECONDS=.01
        return w,r,report,data

    async def test_all_four_directions_stop_with_zero_and_report(self):
        expected={'forward':(.2,0.,0.),'backward':(-.2,0.,0.),'turn_left':(0.,0.,.3),'turn_right':(0.,0.,-.3)}
        for action,values in expected.items():
            w,r,report,d=self.make();started=time.monotonic();w.dispatch(action,.1);await w.task
            packets=[c.args[0] for c in r.ws.send_json.call_args_list]
            self.assertEqual(packets[0]['data'],dict(zip(('x','y','z'),values)))
            self.assertEqual(packets[-1]['data'],{'x':0.,'y':0.,'z':0.})
            self.assertTrue(all(p['title']=='request_twist' for p in packets))
            self.assertGreaterEqual(time.monotonic()-started,.1)
            self.assertEqual(w.state,'completed');self.assertEqual(report.call_args.args[2],'completed')
            self.assertTrue(w.result['zero_velocity_sent']);self.assertFalse(w.result['physical_stop_verified'])

    async def test_stop_early_and_duplicate_block(self):
        w,r,report,d=self.make();w.dispatch('forward',5)
        while not r.ws.send_json.await_count:await asyncio.sleep(0)
        with self.assertRaises(MoveError):w.dispatch('backward',1)
        w.stop();await w.task
        self.assertEqual(w.state,'stopped');self.assertTrue(w.result['stopped_early'])
        self.assertEqual(r.ws.send_json.call_args.args[0]['data'],{'x':0.,'y':0.,'z':0.})

    async def test_send_failure_attempts_zero_never_replays_nonzero(self):
        w,r,report,d=self.make();r.ws.send_json.side_effect=[asyncio.TimeoutError(),None]
        w.STOP_SECONDS=0;w.dispatch('forward',.1);await w.task
        self.assertEqual(w.state,'unconfirmed')
        calls=r.ws.send_json.call_args_list;self.assertEqual(len(calls),2)
        self.assertEqual(calls[1].args[0]['data'],{'x':0.,'y':0.,'z':0.})
        with self.assertRaises(MoveError):w.dispatch('forward',1)

    async def test_disconnect_does_not_replay_on_replacement_connection(self):
        w,r,report,d=self.make();other=SimpleNamespace(closed=False,send_json=AsyncMock())
        async def send(payload):r.ws=other;w.disconnect()
        r.ws.send_json.side_effect=send;w.dispatch('forward',1);await w.task
        other.send_json.assert_not_called();self.assertEqual(w.state,'unconfirmed')

    async def test_vendor_failure_stops_and_is_not_success(self):
        w,r,report,d=self.make()
        async def send(p):
            if p['data']['x']:
                w.receive({'accid':r.serial,'title':'notify_twist','data':{'result':'fail_motor'}})
        r.ws.send_json.side_effect=send;w.dispatch('forward',1);await w.task
        self.assertEqual(w.state,'unconfirmed');self.assertEqual(report.call_args.args[2],'error')
        self.assertEqual(r.ws.send_json.call_args.args[0]['data'],{'x':0.,'y':0.,'z':0.})

    async def test_not_walk_and_peer_busy_rejected_before_send(self):
        for mode in ('STAND','SIT','DAMPING','UNKNOW'):
            w,r,_,_=self.make(mode)
            with self.assertRaises(MoveError):w.dispatch('forward',1)
            r.ws.send_json.assert_not_called()
        w,r,_,_=self.make();r.stand=SimpleNamespace(task=SimpleNamespace(done=lambda:False))
        with self.assertRaises(MoveError):w.dispatch('forward',1)
        r.ws.send_json.assert_not_called()

    async def test_invalid_time_and_direction(self):
        for value in (True,None,'2',0,-1,61,math.nan,math.inf,10**400):
            with self.subTest(value=value),self.assertRaises(ValueError):duration(value)
        w,r,_,_=self.make()
        with self.assertRaises(ValueError):w.dispatch('arbitrary',1)
        r.ws.send_json.assert_not_called()

    async def test_form_declaration_and_packaging(self):
        root=Path(__file__).resolve().parents[1]
        tree=ast.parse((root/'device.py').read_text())
        node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_loco_tool')
        ns={'metadata_properties':lambda:{}}
        exec(compile(ast.Module(body=[node],type_ignores=[]),'card','exec'),ns)
        card=ns['_loco_tool'](None)
        self.assertEqual(card['name'], 'loco_leg')
        self.assertEqual(set(card['inputSchema']['x-action-params']), {'stand_up', 'sit_down', 'move', 'stop_move'})
        self.assertEqual(card['inputSchema']['properties']['duration']['maximum'],60)
        if (root/'Dockerfile').exists():
            self.assertIn('walk.py',(root/'Dockerfile').read_text())
        else:
            self.assertTrue((root/'walk.py').is_file())
        self.assertTrue(any(n+': {enabled: true}' in (root/'config.yaml').read_text() for n in ('walk','loco','loco_leg')))
        self.assertEqual(Walk.PERIOD,1/35)
