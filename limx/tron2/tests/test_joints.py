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
    names={'_loco_tool','tools','call','execute_tool','snapshot','skeleton_configuration','publish_loop','_report_lower_completion'}
    methods=[n for n in source.body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name in names]
    metadata=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='metadata_properties')
    description=next(n.value for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='DESCRIPTIONS' for t in n.targets))
    ns={'asyncio':asyncio,'time':__import__('time'),'model':model,'form_number':form_number,'Path':Path,'os':__import__('os'),
        'CAMERA_SOURCES':{},'GRIPPER_SIDES':{},'DIRECTIONS':{},'PREVIEW_TOPIC':'/limx_tron2/joint_state/preview',
        'DESCRIPTIONS':ast.literal_eval(description),'battery_stream':battery_stream,'joints_stream':joints_stream,'skeleton_stream':skeleton_stream,
        'SENSOR_TOPICS':{'battery':'/limx_tron2/state/battery','joints':'/limx_tron2/state/joints'}}
    cls=ast.ClassDef(name='Adapter',bases=[],keywords=[],body=methods,decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[metadata,cls],type_ignores=[])),'driver_methods','exec'),ns)
    a=ns['Adapter']();a._lower_stopping=False;a.enabled=['loco_leg','joints','battery']
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
    def test_each_model_joint_name_matches_urdf_and_no_inline_model(self):
        for serial,count in [('DACH_TRON2A_TEST',16),('WF_TRON2A_TEST',10),('SF_TRON2A_TEST',10)]:
            profile=configuration(serial);m=model(profile);packet=joints_stream(sample(serial,count),profile)
            moving={j.get('name') for j in ET.fromstring(m['urdf']).findall('joint') if j.get('type')!='fixed'}
            self.assertEqual(set(packet),{'joints'})
            self.assertEqual({j['name'] for j in packet['joints']},moving)
            self.assertEqual(len(packet['joints']),count)
            self.assertTrue(all(set(j)=={'idx','name','q','dq','tau'} for j in packet['joints']))
            if count==10:self.assertTrue(all('head' not in n and 'shoulder' not in n for n in moving))
    def test_offline_and_bad_names_do_not_fabricate_joints(self):
        raw=sample('WF_TRON2A_TEST',10);profile=configuration(raw['accid'])
        raw['available']=False;self.assertEqual(joints_stream(raw,profile),{'joints':[]})
        raw['available']=True;raw['data']['names']=['wrong']*10
        self.assertEqual(joints_stream(raw,profile),{'joints':[]})
    def test_public_model_is_plain_urdf_like_other_drivers(self):
        a=adapter();self.assertEqual(set(a.call('model',{})),{'urdf'})
        self.assertEqual(a.call('model',{})['urdf'],model(a.reader.configuration)['urdf'])
        c=next(t for t in a.tools() if t['name']=='joints')
        self.assertEqual(c['topic_out'][0]['format'],'sensor/skeleton')
    async def test_move_always_uses_zero_lateral_and_legacy_zero_is_compatible(self):
        a=adapter()
        await a.execute_tool('loco_leg',{'action':'move','vx':'-.2','vyaw':'20','duration':'2'})
        a.walk.dispatch_velocity.assert_called_once_with(-.2,0.,20.,2.)
        await a.execute_tool('loco_leg',{'action':'move','vy':'0'})
        with self.assertRaises(ValueError):await a.execute_tool('loco_leg',{'action':'move','vy':.1})
        for value in (.01,math.nan,math.inf,True,-1):
            with self.assertRaises(ValueError):a.call('loco_leg',{'action':'move','duration':value})
    async def test_native_publish_does_not_send_diagnostic_urdf_in_stream(self):
        a=adapter();raw=sample('WF_TRON2A_TEST',10);emitted={}
        a.snapshot=lambda:{'joints':raw,'battery':{'available':True,'data':{'percentage':30,'current':-1880,'cycle':31}}}
        def publish(topic,data):
            emitted[topic]=data
            if len(emitted)==2:raise asyncio.CancelledError
        a.publish=publish
        with self.assertRaises(asyncio.CancelledError):await a.publish_loop()
        self.assertEqual(set(emitted['/limx_tron2/state/joints']),{'joints'})
        self.assertEqual(emitted['/limx_tron2/state/battery'],{'soc':30,'current':-1880,'cycle':31})

class StateInputTests(unittest.IsolatedAsyncioTestCase):
    def reader(self):
        from vendor import RobotReader
        return RobotReader({'serial': '', 'poll_interval_seconds': .5,
            'response_timeout_seconds': 5, 'stale_after_seconds': 5})
    def status(self, accid='DACH_TRON2A_TEST', **updates):
        return {'accid': accid, 'data': {'imu': 'OK', 'motor': 'OK',
            'working_mode': 'developer_mode', 'working_mode_valid': True,
            'collision_enable': True, 'collision_enable_valid': True,
            'compliant_enable': False, 'compliant_enable_valid': True, **updates}}
    async def test_identification_waits_for_feedback_and_is_fixed_for_process(self):
        from main import identify_configuration
        r = self.reader()
        task = asyncio.create_task(identify_configuration(r))
        await asyncio.sleep(.005)
        self.assertFalse(task.done())
        r.accept_info(self.status('WF_TRON2A_TEST'))
        await asyncio.wait_for(task, .2)
        self.assertEqual(r.configuration['mode'], 'wheel_foot')
        original = dict(r.configuration)
        r.disconnect('test')
        await identify_configuration(r)
        self.assertEqual(r.configuration, original)
        self.assertEqual(configuration('UNKNOWN_TRON2A_TEST')['mode'], 'unknown')
        self.assertFalse(configuration('SF_TRON2A_TEST')['field_verified'])
    def test_new_joint_name_keeps_follow_compatible_and_battery_optional(self):
        import time
        from device import Adapter
        from state import joint_state
        from head_follow.client import pose
        settings = {'serial': '', 'poll_interval_seconds': .5,
            'response_timeout_seconds': 5, 'stale_after_seconds': 5,
            'enabled_cards': ['joints', 'battery']}
        a = Adapter(lambda *args: None, settings)
        a.reader.accept_info(self.status())  # No battery: optional field stays unavailable.
        q = [0.] * 14 + [.12, -.2]
        a.reader.joints = joint_state({'result': 'success', 'timestamp': 1,
            'names': [], 'q': q, 'dq': [0.] * 16, 'tau': [0.] * 16})
        a.reader.joints_at = time.monotonic()
        a.reader.joints_received = time.time()
        health = a.health_data()
        self.assertTrue(health['ready'])
        self.assertFalse(health['samples']['battery']['available'])
        self.assertEqual(pose(health)['q'], q)
