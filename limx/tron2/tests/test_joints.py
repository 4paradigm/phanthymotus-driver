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
    names={'_loco_tool','tools','call','execute_tool','snapshot','skeleton_configuration','_skeleton_display','publish_loop','_report_lower_completion'}
    methods=[n for n in source.body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name in names]
    metadata=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='metadata_properties')
    description=next(n.value for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='DESCRIPTIONS' for t in n.targets))
    ns={'asyncio':asyncio,'time':__import__('time'),'math':math,'model':model,'form_number':form_number,'Path':Path,'os':__import__('os'),
        'CAMERA_SOURCES':{},'GRIPPER_SIDES':{},'DIRECTIONS':{},'PREVIEW_TOPIC':'/limx_tron2/joint_state/preview',
        'DESCRIPTIONS':ast.literal_eval(description),'battery_stream':battery_stream,'joints_stream':joints_stream,'skeleton_stream':skeleton_stream,
        'SENSOR_TOPICS':{'battery':'/limx_tron2/state/battery','joints':'/limx_tron2/state/joints'}}
    cls=ast.ClassDef(name='Adapter',bases=[],keywords=[],body=methods,decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[metadata,cls],type_ignores=[])),'driver_methods','exec'),ns)
    a=ns['Adapter']();a._lower_stopping=False;a.enabled=['loco_leg','joints','battery']
    a.reader=SimpleNamespace(configuration=configuration('WF_TRON2A_TEST'),info_at=1,
        sample=lambda name:{'available':False,'received_at':None,'data':None})
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
        for value in (.01,math.nan,math.inf,True,-2):
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
    async def test_identification_waits_for_feedback_and_is_fixed_for_connection(self):
        from main import identify_configuration
        r = self.reader()
        task = asyncio.create_task(identify_configuration(r))
        await asyncio.sleep(.005)
        self.assertFalse(task.done())
        r.accept_info(self.status('WF_TRON2A_TEST'))
        await asyncio.wait_for(task, .2)
        self.assertEqual(r.configuration['mode'], 'wheel_foot')
        r.disconnect('test')
        self.assertEqual(r.configuration['state'], 'pending')
        r.accept_info(self.status('DACH_TRON2A_TEST'))
        await identify_configuration(r)
        self.assertEqual(r.configuration['mode'], 'upper_limb')
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


class ReconnectionIdentityTests(unittest.TestCase):
    def test_auto_identity_relearns_after_disconnect_and_battery_is_preserved(self):
        from vendor import RobotReader
        r = RobotReader({'serial': '', 'stale_after_seconds': 5})
        def notify(accid):
            return {'accid': accid, 'title': 'notify_robot_info', 'data': {
                'battery': '94', 'bms_current': '-1740', 'bms_current_valid': True,
                'bms_charging_cycle': '32', 'imu': 'OK', 'motor': 'OK'}}
        r.accept_info(notify('DACH_TRON2A_TEST'))
        self.assertEqual(r.configuration['mode'], 'upper_limb')
        with self.assertRaises(ValueError):
            r.accept_info(notify('WF_TRON2A_TEST'))
        r.disconnect('robot reboot')
        self.assertIsNone(r.serial)
        self.assertEqual(r.configuration['state'], 'pending')
        r.accept_info(notify('WF_TRON2A_TEST'))
        self.assertEqual(r.configuration['mode'], 'wheel_foot')
        self.assertEqual(battery_stream(r.sample('battery_state')), {'soc': 94, 'current': -1740, 'cycle': 32, 'bms_current_valid': True})
        self.assertEqual(model(r.configuration)['model_id'], 'WF_TRON2A')
        self.assertEqual(len(joints_stream(sample(r.serial, 10), r.configuration)['joints']), 10)

    def test_explicit_identity_lock_survives_disconnect(self):
        from vendor import RobotReader
        r = RobotReader({'serial': 'DACH_TRON2A_TEST', 'stale_after_seconds': 5})
        r.disconnect('test')
        with self.assertRaises(ValueError):
            r.accept_info({'accid': 'WF_TRON2A_TEST', 'data': {}})


class DisplayFrameTests(unittest.TestCase):
    REFERENCE=[2.6029,-.2599,.2821,-2.6215,.48,2.579,.2611,.0374,-2.6208,-.61]

    def rotation(self, urdf):
        from skeleton import _axis_rotation, _multiply
        root=ET.fromstring(urdf)
        roll,pitch,yaw=map(float,root.find(
            "joint[@name='display_world_Joint']/origin").get('rpy').split())
        rotation=((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))
        for axis,angle in (((0,0,1),yaw),((0,1,0),pitch),((1,0,0),roll)):
            rotation=_multiply(rotation,_axis_rotation(axis,angle))
        return rotation

    def test_folded_reference_keeps_head_up_and_support_legs_below(self):
        from skeleton import lower_leg_segments, _dot
        for mode in ('wheel_foot', 'sole_foot'):
            profile={'mode': mode, 'display_reference_q': self.REFERENCE}
            rendered=model(profile);root=ET.fromstring(rendered['urdf'])
            rotation=self.rotation(rendered['urdf'])
            # Native +Z is the torso/head direction; the support legs point down.
            self.assertGreater(_dot(rotation[2],(0.,0.,1.)),.9)
            segments=lower_leg_segments(rendered['urdf'],mode,tuple(self.REFERENCE))
            total=tuple(sum(f[i]-k[i] for k, f in segments) for i in range(3))
            self.assertLess(_dot(rotation[2],total),0.)
            self.assertEqual(rendered['model_revision'],
                model({'mode':mode})['model_revision'])
            # Changing live q never recalculates the captured display frame.
            serial='WF_TRON2A_TEST' if mode=='wheel_foot' else 'SF_TRON2A_TEST'
            raw=sample(serial,10);raw['data']['q']=list(self.REFERENCE)
            self.assertEqual([j['q'] for j in joints_stream(raw,profile)['joints']],
                self.REFERENCE)
            raw['data']['q']=[0.]*10
            joints_stream(raw,profile)
            self.assertEqual(model(profile)['model_revision'],rendered['model_revision'])
            self.assertEqual(len([j for j in root.findall('joint') if j.get('type')!='fixed']),10)

    def test_horizontal_legs_do_not_tip_the_head_sideways(self):
        q=[1.1,-.1,.17,-2.6,0.,1.15,.12,-.15,-2.59,0.]
        root=ET.fromstring(model({'mode':'wheel_foot','display_reference_q':q})['urdf'])
        rpy=[float(v) for v in root.find(
            "joint[@name='display_world_Joint']/origin").get('rpy').split()]
        self.assertTrue(all(abs(v)<1e-12 for v in rpy))

    def test_unidentified_reference_does_not_use_head_up_rotation(self):
        root=ET.fromstring(model({'mode':'wheel_foot'})['urdf'])
        self.assertEqual(root.find("joint[@name='display_world_Joint']/origin").get('rpy'),'0 0 0')

class DisplayLifecycleTests(unittest.IsolatedAsyncioTestCase):
    FOLDED=[2.6029,-.2599,.2821,-2.6215,.48,2.579,.2611,.0374,-2.6208,-.61]

    def make(self, status='SIT', q=None):
        from device import Adapter
        from vendor import RobotReader
        from state import joint_state
        import time
        settings={'serial':'','enabled_cards':['joints','loco_leg'],'stale_after_seconds':5,
                  'poll_interval_seconds':.5,'response_timeout_seconds':.2}
        reader=RobotReader(settings)
        reader.accept_info({'accid':'WF_TRON2A_TEST','data':{'sw_version':'test-version',
            'status':status,'imu':'OK','motor':'OK','working_mode':'developer_mode',
            'working_mode_valid':True,'collision_enable':True,'collision_enable_valid':True,
            'compliant_enable':False,'compliant_enable_valid':True}})
        q=list(q or self.FOLDED)
        reader.joints=joint_state({'result':'success','timestamp':1.,'q':q,
            'dq':[0.]*10,'tau':[0.]*10,'names':[]})
        reader.joints_at=time.monotonic();reader.joints_received=time.time()
        return reader, Adapter(lambda *args:None,settings,reader=reader)

    async def test_startup_captures_one_frame_and_holds_it(self):
        import time
        reader,adapter=self.make()
        first=adapter.skeleton_configuration()
        self.assertEqual(first['display_frame'],'captured')
        self.assertEqual(first['display_reference_q'],self.FOLDED)
        rendered=model(first)
        reader.joints['q']=[0.]*10;reader.joints_at=time.monotonic()
        reader.joints_received=time.time()+10
        second=adapter.skeleton_configuration()
        self.assertEqual(second['display_reference_q'],self.FOLDED)
        self.assertEqual(model(second)['model_revision'],rendered['model_revision'])

    async def test_model_request_waits_for_the_first_lower_body_frame(self):
        import time
        from state import joint_state
        reader,adapter=self.make()
        reader.joints_at=reader.joints_received=None
        async def feed():
            await asyncio.sleep(.01)
            reader.joints=joint_state({'result':'success','timestamp':1.,
                'q':self.FOLDED,'dq':[0.]*10,'tau':[0.]*10,'names':[]})
            reader.joints_at=time.monotonic();reader.joints_received=time.time()
        feeder=asyncio.create_task(feed())
        rendered=await adapter.execute_tool('model',{})
        await feeder
        self.assertEqual(adapter.skeleton_configuration()['display_frame'],'captured')
        self.assertEqual(rendered['urdf'],model(adapter.skeleton_configuration())['urdf'])
        self.assertEqual(rendered['urdf'],model({'mode':'wheel_foot'})['urdf'])

    async def test_stand_request_switches_to_live_native_frame(self):
        import time
        reader,adapter=self.make()
        adapter.skeleton_configuration()
        async def send(packet):
            reader.info['status']='STAND' if packet['title']=='request_stand_mode' else 'WALK'
            reader.info_at=time.monotonic();reader.info_received=time.time()
        reader.ws=SimpleNamespace(closed=False,send_json=AsyncMock(side_effect=send))
        adapter.stand.POLL_SECONDS=.001
        adapter.stand.dispatch()
        await asyncio.wait_for(adapter.stand.task,1)
        profile=adapter.skeleton_configuration()
        self.assertEqual(profile['display_frame'],'live')
        self.assertIsNone(profile['display_reference_q'])
        self.assertEqual(model(profile)['urdf'],model({'mode':'wheel_foot'})['urdf'])

    async def test_sit_completion_recaptures_fresh_folded_frame_once(self):
        import time
        reader,adapter=self.make(status='WALK')
        reader._skeleton_display_frame={'state':'live','reference_q':None,
            'source':'stand_up_requested'}
        async def send(packet):
            if packet['title']=='request_sitdown':
                reader.info['status']='DAMPING'
                reader.info_at=time.monotonic();reader.info_received=time.time()
        reader.ws=SimpleNamespace(closed=False,send_json=AsyncMock(side_effect=send))
        adapter.sit_down.STOP_SECONDS,adapter.sit_down.BRAKE_INTERVAL=.005,.001
        adapter.sit_down.POLL_SECONDS=.001
        adapter.sit_down.dispatch()
        await asyncio.wait_for(adapter.sit_down.task,1)
        self.assertEqual(reader._skeleton_display_frame['source'],'sit_down_completed')
        self.assertEqual(adapter.skeleton_configuration()['display_frame'],'pending')
        folded=[2.5,-.2,.2,-2.3,.3,2.5,.2,.03,-2.3,-.4]
        from state import joint_state
        reader.joints=joint_state({'result':'success','timestamp':2.,'q':folded,
            'dq':[0.]*10,'tau':[0.]*10,'names':[]})
        reader.joints_at=time.monotonic();reader.joints_received=time.time()
        captured=adapter.skeleton_configuration()
        self.assertEqual(captured['display_frame'],'captured')
        self.assertEqual(captured['display_reference_q'],folded)
        rendered=model(captured)
        reader.joints['q']=[0.]*10;reader.joints_received=time.time()+10
        self.assertEqual(adapter.skeleton_configuration()['display_reference_q'],folded)
        self.assertEqual(model(adapter.skeleton_configuration())['model_revision'],
            rendered['model_revision'])


class RealImuTests(unittest.IsolatedAsyncioTestCase):
    def reader(self):
        from vendor import RobotReader
        import time
        r=RobotReader({'serial':'','poll_interval_seconds':.1,
            'response_timeout_seconds':5,'stale_after_seconds':5})
        r.accept_info({'accid':'WF_TRON2A_TEST','data':{'imu':'OK'}})
        r.joints=sample(r.serial,10)['data']
        r.joints_at=time.monotonic();r.joints_received=time.time()
        return r

    async def test_real_notification_to_public_joints(self):
        import aiohttp,time
        r=self.reader();ws=SimpleNamespace(send_json=AsyncMock())
        await r.enable_imu(ws)
        request=ws.send_json.call_args.args[0]
        self.assertEqual(request['title'],'request_enable_imu')
        self.assertEqual(request['data'],{'enable':True})
        q=[.670594,.143134,-.715425,.134103]
        ws.receive=AsyncMock(return_value=SimpleNamespace(type=aiohttp.WSMsgType.TEXT,
            data=__import__('json').dumps({'title':'notify_imu','accid':r.serial,'data':{'quat':q}})))
        await r.receive(ws,time.monotonic()+1)
        packet=joints_stream(r.sample('joint_state'),r.configuration)
        self.assertEqual(set(packet),{'joints','imu_quat'})
        self.assertAlmostEqual(sum(v*v for v in packet['imu_quat']),1)
        self.assertEqual([j['q'] for j in packet['joints']],r.joints['q'])
        # Native URDF remains constant: do not rotate both URDF and IMU.
        self.assertEqual(model({**r.configuration,'display_reference_q':[2.6,0,0,-2.6,0]*2})['urdf'],model(r.configuration)['urdf'])
        r.imu_at-=2
        self.assertNotIn('imu_quat',joints_stream(r.sample('joint_state'),r.configuration))
        r.disconnect('test');self.assertIsNone(r.imu)

    def test_invalid_or_foreign_imu_is_never_published(self):
        r=self.reader()
        r.accept_imu({'accid':'OTHER','data':{'quat':[1,0,0,0]}})
        self.assertIsNone(r.imu)
        for q in ([],[0,0,0,0],[True,0,0,0],[float('nan'),0,0,1],[5,0,0,0]):
            r.accept_imu({'accid':r.serial,'data':{'quat':q}})
            self.assertNotIn('imu_quat',joints_stream(r.sample('joint_state'),r.configuration))

    async def test_upper_limb_does_not_enable_undocumented_lower_imu_api(self):
        r=self.reader();r.configuration={'mode':'upper_limb'}
        ws=SimpleNamespace(send_json=AsyncMock())
        await r.enable_imu(ws);ws.send_json.assert_not_called()
