"""Local fake Adapter + real URDF: no ROS, robot, Core edits or network."""
import ast
import asyncio
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,AsyncMock
import xml.etree.ElementTree as ET
from skeleton import model,joints_stream,skeleton_stream,MODELS
from state import configuration,battery_stream
from walk import form_number

ROOT=Path(__file__).resolve().parents[1]

def adapter():
    tree=ast.parse((ROOT/'device.py').read_text())
    source=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Adapter')
    names={'_loco_tool','tools','call','execute_tool','snapshot','skeleton_configuration','publish_loop','_report_lower_completion','_finish_lower_tasks','close_lower'}
    methods=[n for n in source.body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name in names]
    metadata=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='metadata_properties')
    description=next(n.value for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='DESCRIPTIONS' for t in n.targets))
    ns={'asyncio':asyncio,'time':__import__('time'),'model':model,'form_number':form_number,'Path':Path,'os':__import__('os'),
        'CAMERA_SOURCES':{},'GRIPPER_SIDES':{},'DIRECTIONS':{},'PREVIEW_TOPIC':'/limx_tron2/joint_state/preview',
        'DESCRIPTIONS':ast.literal_eval(description),'battery_stream':battery_stream,'joints_stream':joints_stream,'skeleton_stream':skeleton_stream,
        'SENSOR_TOPICS':{'battery':'/limx_tron2/state/battery','joints':'/limx_tron2/state/joints'}}
    cls=ast.ClassDef(name='Adapter',bases=[],keywords=[],body=methods,decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[metadata,cls],type_ignores=[])),'driver_methods','exec'),ns)
    a=ns['Adapter']();a._lower_stopping=False;a._lower_closed=False;a._lower_stop_task=None;a.enabled=['loco_leg','joints','battery']
    a.reader=SimpleNamespace(configuration=configuration('WF_TRON2A_TEST'),info_at=1)
    a.stand=SimpleNamespace(dispatch=Mock(return_value={'action_id':'up'}),info=lambda:{'state':'idle'})
    a.sit_down=SimpleNamespace(dispatch=Mock(return_value={'action_id':'down'}),info=lambda:{'state':'idle'})
    a.walk=SimpleNamespace(dispatch_velocity=Mock(return_value={'action_id':'move'}),stop_move=AsyncMock(return_value={'state':'stopped'}),
        info=lambda:{'state':'idle'},stop=lambda:{'state':'idle','motion_sent':False})
    a._report_completion=AsyncMock(return_value=True)
    return a

def sample(serial,count):
    return {'accid':serial,'connected':True,'available':True,'sample_status':'recently_received',
        'received_at':1.,'reception_age_seconds':0.,'measurement_time_verified':False,
        'data':{'q':[i/100 for i in range(count)],'dq':[0.]*count,'tau':[0.]*count,'names':[]}}


class NativeContractTests(unittest.IsolatedAsyncioTestCase):
    def test_loco_public_schema_removed_vy_and_renamed_sit(self):
        c=adapter()._loco_tool();s=c['inputSchema']
        self.assertNotIn('vy',s['properties']);self.assertEqual(s['x-action-params']['move']['params'],['vx','vyaw','duration'])
        self.assertIn('sit_down',s['properties']['action']['enum']);self.assertNotIn('stand_down',s['properties']['action']['enum'])
        self.assertIn('sit_down',s['x-completion']['actions']);self.assertNotIn('stand_down',s['x-completion']['actions'])
        for name in ('vx','vyaw','duration'):
            self.assertIn('default',s['properties'][name]);self.assertTrue(s['properties'][name]['description'])
        self.assertIn('anyOf',s['properties']['duration'])
    async def test_sit_alias_and_completion_action_names(self):
        a=adapter()
        await a.execute_tool('loco_leg',{'action':'sit_down'});await a.execute_tool('loco_leg',{'action':'stand_down'})
        self.assertEqual(a.sit_down.dispatch.call_count,2)
        await a._report_lower_completion('sit_down','id','completed',{})
        a._report_completion.assert_awaited_with('loco_leg','id','completed',{'action':'sit_down'})

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

    def test_unknown_startup_supports_automatic_and_explicit_profiles(self):
        r, _ = robot('UNKNOW')
        owner = Stand(r, AsyncMock())
        with patch.dict(os.environ, {'TRON2_STARTUP_PROFILE': ''}):
            self.assertTrue(startup_match(r)['matched'])
            self.assertEqual(r._startup_calibration['accid'], r.serial)
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

    async def test_continuous_move_runs_until_explicit_stop_without_acp_timeout(self):
        w,r,report,_ = self.make()
        result = w.dispatch_velocity(.1, 0., 0., -1)
        self.assertTrue(result['continuous'])
        self.assertNotIn('action_id', result)
        await asyncio.sleep(.12)
        self.assertFalse(w.task.done())
        await w.stop_move()
        await asyncio.wait_for(w.task, 1)
        self.assertTrue(w.result['zero_velocity_sent'])
        self.assertEqual(r.ws.send_json.call_args.args[0]['data'], {'x':0.,'y':0.,'z':0.})
        report.assert_not_awaited()

    async def test_calibrated_22_5_degrees_maps_to_vendor_1_5_and_stops(self):
        w,r,report,_=self.make();packets=[]
        async def capture(packet):packets.append((time.monotonic(),packet))
        r.ws.send_json.side_effect=capture
        w.dispatch_velocity(0.,0.,22.5,2.)
        await asyncio.wait_for(w.task,4)
        moving=[(at,p) for at,p in packets if p['data']['z']!=0]
        zeros=[(at,p) for at,p in packets if p['data']['z']==0]
        self.assertGreater(len(moving),50)
        self.assertTrue(all(abs(p['data']['z']-1.5)<1e-12 for _,p in moving))
        first_zero=zeros[0][0]
        self.assertTrue(all(at<first_zero for at,_ in moving))
        self.assertLess(abs(first_zero-moving[0][0]-2.),.08)
        self.assertAlmostEqual(w.result['vendor_z_command'],1.5)
        self.assertFalse(w.result['angle_control'])

    async def test_invalid_time_and_direction(self):
        for value in (True,None,'2',0,-2,61,math.nan,math.inf,10**400):
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


class CardLifecycleTests(unittest.IsolatedAsyncioTestCase):
    def make_card(self, mode='DAMPING'):
        a = adapter()
        r, status = robot(mode)
        a.reader = r
        a.stand = Stand(r, AsyncMock(return_value=True))
        a.sit_down = SitDown(r, AsyncMock(return_value=True))
        a.walk = Walk(r, AsyncMock(return_value=True))
        a.walk.STOP_SECONDS = .01
        return a, r

    async def test_card_stop_joins_stand_and_sit_without_later_commands(self):
        for action, mode, owner_name in [('stand_up', 'DAMPING', 'stand'),
                                        ('sit_down', 'STAND', 'sit_down'),
                                        ('sit_down', 'WALK', 'sit_down')]:
            a, r = self.make_card(mode)
            await a.execute_tool('loco_leg', {'action': action})
            while not r.ws.send_json.await_count:
                await asyncio.sleep(0)
            result = await a.execute_tool('loco_leg', {'action': 'stop'})
            count = r.ws.send_json.await_count
            await asyncio.sleep(.02)
            self.assertEqual(count, r.ws.send_json.await_count)
            self.assertTrue(getattr(a, owner_name).task.done())
            self.assertFalse(result['physical_stop_verified'])
            self.assertNotEqual(getattr(a, owner_name).state, 'completed')

    async def test_stop_before_task_first_runs_and_shutdown_blocks_new_action(self):
        a, r = self.make_card()
        await a.execute_tool('loco_leg', {'action': 'stand_up'})
        await a.close_lower(shutting_down=True)
        self.assertTrue(a.stand.task.done())
        with self.assertRaises(ValueError):
            await a.execute_tool('loco_leg', {'action': 'stand_up'})

    async def test_walk_stop_joins_zero_cleanup(self):
        a, r = self.make_card('WALK')
        await a.execute_tool('loco_leg', {'action': 'move', 'vx': .1, 'duration': 5})
        while not r.ws.send_json.await_count:
            await asyncio.sleep(0)
        await asyncio.gather(a.execute_tool('loco_leg', {'action': 'stop'}),
                             a.execute_tool('loco_leg', {'action': 'stop'}))
        self.assertTrue(a.walk.task.done())
        self.assertEqual(r.ws.send_json.call_args.args[0]['data'], {'x': 0., 'y': 0., 'z': 0.})
        count = r.ws.send_json.await_count
        await asyncio.sleep(.02)
        self.assertEqual(count, r.ws.send_json.await_count)

    async def test_stop_still_validates_card_and_arguments(self):
        a, _ = self.make_card()
        for args in (None, {'action': 'stop', 'unexpected': 1}):
            with self.assertRaises(ValueError):
                await a.execute_tool('loco_leg', args)
        a.enabled.remove('loco_leg')
        with self.assertRaises(ValueError):
            await a.execute_tool('loco_leg', {'action': 'stop'})


class BuiltinStartupProfileTests(unittest.TestCase):
    def test_automatic_profile_uses_current_identity_and_broad_folded_knees(self):
        from stand import STARTUP_PROFILES, startup_match
        r, status = robot('UNKNOW')
        r.serial = 'WF_TRON2A_379'
        profile = STARTUP_PROFILES['wheel_foot']
        status['sw_version'] = 'robot-tron2-r-2.7.16-test'
        original_sample = r.sample
        q = [sum(profile['pose_ranges'][str(i)])/2 if i in profile['pose_indices'] else 0. for i in range(10)]
        def read(name):
            sample = original_sample(name)
            if name == 'joint_state': sample['data']['q'] = list(q)
            return sample
        r.sample = read
        with patch.dict(os.environ, {'TRON2_STARTUP_PROFILE': ''}):
            self.assertTrue(startup_match(r)['matched'])
            self.assertEqual(r._startup_calibration['sw_version'], status['sw_version'])
            r.serial = 'WF_TRON2A_OTHER'
            r._startup_calibration = None  # A new connection / different unit.
            status['sw_version'] = 'robot-tron2-r-2.7.17-test'
            q[0], q[1], q[5], q[6] = .4, .6, 2.8, -.6
            self.assertTrue(startup_match(r)['matched'])
            q[3] = 0.
            self.assertFalse(startup_match(r)['matched'])
            r.serial = 'WF_TRON2A_OTHER'
            self.assertFalse(startup_match(r)['matched'])

class YawCalibrationTests(unittest.TestCase):
    def test_signed_conversion_and_limits(self):
        from walk import Walk
        from unittest.mock import Mock
        w=object.__new__(Walk);w._dispatch=Mock(return_value={})
        for speed,z in ((22.5,1.5),(15.,1.),(-15.,-1.),(-22.5,-1.5),(0.,0.)):
            w.dispatch_velocity(0,0,speed,2)
            self.assertEqual(w._dispatch.call_args.args[2][2],z)
        for speed in (22.6,-22.6,45.):
            with self.assertRaises(ValueError):w.dispatch_velocity(0,0,speed,2)
