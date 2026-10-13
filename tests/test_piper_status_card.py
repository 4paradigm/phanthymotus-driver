from copy import deepcopy
from types import SimpleNamespace
import sys
from pathlib import Path
import threading
import time
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agilex.piper.status_card import PiperStatusCard, split_status_card
from agilex.piper.monitor import PiperMonitorPlugin
from test_piper_monitor import Transport
from common.vendor_runtime import DriverBundle


class StatusCardTests(unittest.TestCase):
    def runtime(self):
        data={"device_name":"机械臂 1","connected":True,"fresh":True,"feedback_time":time.time(),
              "joints_deg":[1,2,3,4,5,6],"motion_enabled":False}
        connects=[]
        r=SimpleNamespace(config={"name":"机械臂 1"},topic='/agilex/piper/rgb',closed=False,connection_lock=threading.RLock(),
                          connected=lambda:connects.append('same-runtime'),read_monitor_snapshot=lambda:deepcopy(data))
        r.get_tools=lambda:[{"name":"arm_control","type":"actuator","inputSchema":{
            "properties":{"action":{"enum":["read","info","halt","move_joint"],"default":"read",
            "oneOf":[{"const":x,"title":x} for x in ['read','info','halt','move_joint']]}},
            "x-action-params":{"read":{},"halt":{}}}}]
        r.dispatch=lambda action,args:{"action":action}
        return r, connects

    def test_manual_read_uses_shared_connection_and_passive_snapshot(self):
        r,connects=self.runtime(); plugin=PiperStatusCard(r)
        result=DriverBundle([plugin]).dispatch('piper_status',{'action':'read'})
        self.assertEqual(result['parameters']['J1 角度（°）'],1)
        self.assertEqual(result['parameters']['动作许可'],'关闭')
        self.assertEqual(len(result['skeleton']['joints']),6)
        self.assertEqual(result['skeleton']['position_unit'],'rad')
        self.assertEqual(connects,['same-runtime'])

    def test_lifecycle_does_not_touch_shared_connection(self):
        r,connects=self.runtime(); plugin=PiperStatusCard(r)
        plugin.stop(); plugin.start(); plugin.info()
        self.assertEqual(connects,[])
        self.assertFalse(r.closed)
        with self.assertRaises(ValueError):plugin.dispatch('piper_status',{'action':'enable'})

    def test_split_removes_read_and_keeps_control_dispatch(self):
        r,connects=self.runtime(); original=deepcopy(r.get_tools())
        plugins=split_status_card([r]); definitions=r.get_tools()
        self.assertNotIn('read',definitions[0]['inputSchema']['properties']['action']['enum'])
        self.assertEqual(definitions[0]['inputSchema']['properties']['action']['enum'][0],'select_action')
        self.assertEqual(original[0]['inputSchema']['properties']['action']['default'],'read')
        bundle=DriverBundle(plugins)
        self.assertFalse(bundle.dispatch('arm_control',{'action':'select_action'})['executed'])
        self.assertEqual(connects,[])
        with self.assertRaises(ValueError):bundle.dispatch('arm_control',{'action':'read'})
        self.assertEqual(bundle.dispatch('arm_control',{'action':'halt'}),{'action':'halt'})
        self.assertEqual(bundle.dispatch('piper_status',{'action':'read'})['parameters']['设备名称'],'机械臂 1')

    def test_sensor_registration_reuses_sampler_and_model_without_duplicates(self):
        r,connects=self.runtime()
        monitor=PiperMonitorPlugin(r.read_monitor_snapshot,namespace='agilex/piper',device_name='机械臂 1')
        plugins=split_status_card([r,monitor])
        self.assertNotIn(monitor,plugins)
        self.assertIs(plugins[-1].monitor,monitor)
        bundle=DriverBundle(plugins)
        tools={t['name']:t for t in bundle.get_all_tools()}
        self.assertNotIn('piper_monitor',tools)
        self.assertEqual(tools['piper_status']['type'],'sensor')
        self.assertEqual(tools['piper_status']['topic_out'],[
            {'topic':'/agilex/piper/state/joints','format':'sensor/skeleton'}])
        self.assertIn('<robot',bundle.dispatch('model',{})['urdf'])
        self.assertEqual(bundle.dispatch('piper_status',{'action':'info'})['state'],'idle')
        bundle.dispatch('piper_status',{'action':'start'})
        self.assertEqual(bundle.dispatch('piper_status',{'action':'stop'})['state'],'idle')
        self.assertEqual(connects,[])

    def test_continuous_pose_updates_and_stop_release_without_hardware_commands(self):
        r,connects=self.runtime(); transports=[]
        def factory(*args):
            transport=Transport(*args);transports.append(transport);return transport
        monitor=PiperMonitorPlugin(r.read_monitor_snapshot,namespace='agilex/piper',device_name='机械臂 1',
                                  ros2=object(),publisher_factory=factory)
        plugin=PiperStatusCard(r,monitor)
        plugin.start();plugin.start()
        self.assertEqual(len(transports),1)
        transports[0].callback()
        self.assertEqual([fmt for fmt,_ in transports[0].messages],['data/json','sensor/skeleton'])
        plugin.stop();plugin.stop()
        self.assertEqual(transports[0].closed,1)
        self.assertEqual(connects,[])
        self.assertFalse(r.closed)


if __name__=='__main__':unittest.main()
