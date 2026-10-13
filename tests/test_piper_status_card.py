from copy import deepcopy
from types import SimpleNamespace
import sys
from pathlib import Path
import threading
import time
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agilex.piper.status_card import PiperStatusCard, split_status_card
from common.vendor_runtime import DriverBundle


class StatusCardTests(unittest.TestCase):
    def runtime(self):
        data={"device_name":"机械臂 1","connected":True,"fresh":True,"feedback_time":time.time(),
              "joints_deg":[1,2,3,4,5,6],"motion_enabled":False}
        connects=[]
        r=SimpleNamespace(config={"name":"机械臂 1"},closed=False,connection_lock=threading.RLock(),
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
        self.assertEqual(result['J1 角度（°）'],1)
        self.assertEqual(result['动作许可'],'关闭')
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
        self.assertEqual(bundle.dispatch('piper_status',{'action':'read'})['设备名称'],'机械臂 1')


if __name__=='__main__':unittest.main()
