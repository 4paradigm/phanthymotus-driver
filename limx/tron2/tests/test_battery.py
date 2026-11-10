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
    def test_battery_notes_explain_ranges_and_unknown_units(self):
        text=next(t for t in adapter().tools() if t['name']=='battery')['description']
        for phrase in ('0～100%','有符号','尚未确认','非负整数','null'):self.assertIn(phrase,text)

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
    def test_numeric_strings_and_invalid_battery_do_not_poison_robot_status(self):
        for value in ('NaN', True, -1, 101, None):
            r = self.reader();r.accept_info(self.status(battery=value))
            self.assertFalse(r.sample('battery_state')['available'])
            self.assertTrue(r.sample('robot_status')['available'])
        r = self.reader()
        r.accept_info(self.status(battery='42', bms_current='-1800',
            bms_current_valid=True, bms_charging_cycle='5'))
        self.assertEqual(battery_stream(r.sample('battery_state')),
            {'soc': 42, 'current': -1800, 'cycle': 5})
        r.disconnect('test')
        self.assertEqual(battery_stream(r.sample('battery_state')),
            {'soc': None, 'current': None, 'cycle': None})
