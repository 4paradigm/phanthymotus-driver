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
                          driver=None,gate=threading.Lock(),connected_event=threading.Event(),
                          connected=lambda:connects.append('same-runtime'),read_monitor_snapshot=lambda:deepcopy(data))
        def connected():
            connects.append('same-runtime'); r.driver=object();r.connected_event.set()
            return r.driver
        r.connected=connected
        r.get_tools=lambda:[{"name":"arm_control","type":"actuator","inputSchema":{
            "properties":{"action":{"enum":["read","info","halt","move_joint"],"default":"read",
            "oneOf":[{"const":x,"title":x} for x in ['read','info','halt','move_joint']]}},
            "x-action-params":{"read":{},"halt":{}}}}]
        r.dispatch=lambda action,args:{"action":action}
        r.dispatch_sync=r.dispatch
        r.require_motion_permission=lambda:None
        r.worker_context=threading.local()
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
        self.assertEqual(tools['piper_status']['type'],'resource')
        self.assertEqual(tools['piper_status']['topic_out'],[
            {'topic':'/agilex/piper/state/joints','format':'sensor/skeleton'}])
        self.assertIn('<robot',bundle.dispatch('model',{})['urdf'])
        self.assertEqual(bundle.dispatch('piper_status',{'action':'info'})['state'],'idle')
        bundle.dispatch('piper_status',{'action':'start'})
        self.assertTrue(r.connected_event.wait(1))
        self.assertEqual(bundle.dispatch('piper_status',{'action':'stop'})['state'],'idle')
        self.assertEqual(connects,['same-runtime'])

    def test_teaching_actions_are_removed_and_rejected_without_hardware(self):
        r,connects=self.runtime();plugins=split_status_card([r])
        bundle=DriverBundle(plugins)
        control=next(t for t in bundle.get_all_tools() if t['name']=='arm_control')
        for action in ('enter_teaching','exit_teaching'):
            self.assertNotIn(action,control['inputSchema']['properties']['action']['enum'])
            self.assertNotIn(action,control['inputSchema'].get('x-action-params',{}))
            with self.assertRaises(ValueError):bundle.dispatch('arm_control',{'action':action})
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

    def test_detector_has_no_read_or_execute_form_and_initializes_shared_listener_once(self):
        r,connects=self.runtime();plugin=PiperStatusCard(r,auto_connect=True)
        self.addCleanup(plugin.stop)
        definition=plugin.get_tools()[0]
        self.assertEqual(set(definition['inputSchema']['properties']['action']['enum']),{'start','stop','info'})
        self.assertNotIn('x-manual-execution',definition['inputSchema'])
        plugin.start();self.assertTrue(r.connected_event.wait(1));plugin.start()
        self.assertEqual(connects,['same-runtime'])
        plugin.stop();self.assertFalse(plugin._connection_thread.is_alive())

    def test_auto_connection_does_not_wait_for_action_gate_or_take_motion_permission(self):
        r,connects=self.runtime();r.gate.acquire()
        self.addCleanup(r.gate.release)
        plugin=PiperStatusCard(r,auto_connect=True)
        plugin.start();plugin.stop()
        self.assertEqual(connects,[])
        self.assertFalse(plugin._connection_thread.is_alive())

    def test_auto_connection_failure_is_visible_and_restart_can_recover(self):
        r,connects=self.runtime();good=r.connected;failed=threading.Event()
        def fail():
            failed.set();raise RuntimeError('CAN unavailable')
        r.connected=fail;plugin=PiperStatusCard(r,auto_connect=True)
        self.addCleanup(plugin.stop)
        plugin.start();self.assertTrue(failed.wait(1));plugin.stop()
        self.assertEqual(plugin.info()['connection_error'],'CAN unavailable')
        r.connected=good;plugin.start();self.assertTrue(r.connected_event.wait(1));plugin.stop()
        self.assertIsNone(plugin.info()['connection_error'])


if __name__=='__main__':unittest.main()
