"""MPC source selection leaves joystick emergency-stop topics untouched."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('input_gate',Path(__file__).resolve().parents[1]/'host/upper_input_gate.py')
gate_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(gate_module)


class InputGateTests(unittest.TestCase):
    def setUp(self):
        self.gate=gate_module.UpperInputGate('master')
        self.calls=[]
        self.links={}
        self.pubs={'/cmd_vel':['/rosbridge_websocket','/mobile_manipulator_joy_command_node','/external'],
            '/cmd_vel_world':['/mobile_manipulator_joy_command_node']}
        def query(uri,method,*args):
            self.calls.append((uri,method,args))
            if method=='getSystemState':return [list(self.pubs.items()),[(t,list(gate_module.CONSUMERS)) for t in gate_module.TOPICS],[]]
            if method=='lookupNode':return 'uri:'+args[0]
            if method=='publisherUpdate':self.links[(uri,args[0])]=list(args[1]);return 1
            if method=='getBusInfo':return [[0,peer,'i','TCPROS',topic,True] for (node,topic),peers in self.links.items() if node==uri for peer in peers]
            raise AssertionError(method)
        self.gate.query=query

    def test_card_only_receives_driver_velocity_and_remote_gets_joystick_publishers(self):
        self.gate.select('card')
        for consumer in gate_module.CONSUMERS:
            self.assertEqual(self.links[('uri:'+consumer,'/cmd_vel')],['uri:/rosbridge_websocket'])
            self.assertEqual(self.links[('uri:'+consumer,'/cmd_vel_world')],[])
        self.gate.select('remote')
        for consumer in gate_module.CONSUMERS:
            self.assertEqual(self.links[('uri:'+consumer,'/cmd_vel')],['uri:/mobile_manipulator_joy_command_node'])
        writes=[args[0] for _,method,args in self.calls if method=='publisherUpdate']
        self.assertEqual(set(writes),{'/cmd_vel','/cmd_vel_world'})
        self.assertNotIn('/joy',writes)
        self.assertNotIn('/stop_robot',writes)

    def test_hold_disconnects_velocity_only_and_shutdown_restores_master_publishers(self):
        self.gate.select('hold')
        self.assertTrue(all(peers==[] for peers in self.links.values()))
        self.gate.restore()
        self.assertEqual(set(self.links[('uri:/nodelet_manager','/cmd_vel')]),
            {'uri:/rosbridge_websocket','uri:/mobile_manipulator_joy_command_node','uri:/external'})

    def test_missing_driver_publisher_blocks_card_handoff(self):
        self.pubs['/cmd_vel'].remove('/rosbridge_websocket')
        with self.assertRaisesRegex(RuntimeError,'card_publisher_missing'):
            self.gate.select('card')
        self.assertFalse(any(method=='publisherUpdate' for _,method,_ in self.calls))
