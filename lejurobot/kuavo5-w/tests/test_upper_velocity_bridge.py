"""Two-master forwarding is lease/state gated; MPC remains the producer."""
import importlib.util
from pathlib import Path
import threading
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import types
import unittest
from unittest.mock import patch, Mock

from mapping_keyboard import UpperVelocityBridge

spec=importlib.util.spec_from_file_location('upper_bridge_remote',Path(__file__).resolve().parents[1]/'host/upper_velocity_bridge.py')
remote=importlib.util.module_from_spec(spec);spec.loader.exec_module(remote)
# The actual SSH payload prepends the keyboard module's standard imports.
import math
remote.math=math


class BridgeGateTests(unittest.TestCase):
    def state(self):
        return types.SimpleNamespace(manual_mode=False,mapping=False,soft_estop=False,
            estop=False,bumper=False,driver_error=False,driver_enable=True)

    def test_only_fresh_authorized_healthy_auto_velocity_is_forwarded(self):
        state=self.state();sample=(10.,True,[.1,0.,0.,0.,0.,.2])
        self.assertEqual(remote.bridge_values(sample,(state,10.),True,10.1),sample[2])
        for cause in ('lease','sample_old','feedback_old','disabled','manual_mode','mapping','soft_estop','estop','bumper','driver_error','driver_enable'):
            with self.subTest(cause=cause):
                state=self.state();candidate=sample;stamp=10.;allowed=True
                if cause=='lease':allowed=False
                elif cause=='sample_old':candidate=(9.,True,sample[2])
                elif cause=='feedback_old':stamp=9.
                elif cause=='disabled':candidate=(10.,False,sample[2])
                else:setattr(state,cause,cause!='driver_enable')
                self.assertEqual(remote.bridge_values(candidate,(state,stamp),allowed,10.1),[0.]*6)

    def test_invalid_mpc_velocity_cannot_become_direct_chassis_command(self):
        for values in ([float('nan'),0,0,0,0,0],[.9,0,0,0,0,0],[0,0,.1,0,0,0],[0,0,0,0,0,1.1]):
            with self.assertRaisesRegex(RuntimeError,'upper_bridge_invalid_velocity'):
                remote.bridge_values((10.,True,values),(self.state(),10.),True,10.1)

    def test_driver_ownership_excludes_navigation_mapping_preparation_and_stop(self):
        state=types.SimpleNamespace(motion_blocked=False,read=lambda key:(True,10.))
        bundle=types.SimpleNamespace(tracker=types.SimpleNamespace(lock=threading.RLock(),
            _active={'base':('id','base_drive',threading.Event())}),_closing=False,_transition=None,
            state=state,_stop_epoch=2,ros1=types.SimpleNamespace(connection=(True,1)),
            _upper_route_key=(2,(True,1)),spatial=types.SimpleNamespace(mapping_active=lambda:False))
        bridge=UpperVelocityBridge(bundle)
        self.assertTrue(bridge.allowed())
        for owner in ('controlled_spatial','mapping_keyboard'):
            bundle.tracker._active['base']=('id',owner,threading.Event())
            self.assertFalse(bridge.allowed())
        bundle.tracker._active['base']=('id','base_servo',threading.Event())
        self.assertTrue(bridge.allowed())
        bundle._transition={};self.assertFalse(bridge.allowed())
        bundle._transition=None;state.motion_blocked=True;self.assertFalse(bridge.allowed())
        state.motion_blocked=False;bundle._stop_epoch+=1;self.assertFalse(bridge.allowed())

    def test_stream_failure_invalidates_previously_cached_route(self):
        bundle=types.SimpleNamespace(tracker=types.SimpleNamespace(lock=threading.RLock()),
            spatial=types.SimpleNamespace(client=object()),_upper_route_key='previous')
        bridge=UpperVelocityBridge(bundle)
        with patch('mapping_keyboard.FactoryKeyboardStream',side_effect=ConnectionError('no link')):
            bridge._run()
        self.assertIsNone(bundle._upper_route_key)
        self.assertIsNotNone(bridge.error)
        self.assertTrue(bridge.ready.is_set())

    def test_missing_stop_confirmation_persists_a_fence_instead_of_claiming_idle(self):
        for confirmed in (False,True):
            with self.subTest(confirmed=confirmed):
                bundle=types.SimpleNamespace(tracker=types.SimpleNamespace(lock=threading.RLock()),
                    state=types.SimpleNamespace(motion_blocked=False),
                    spatial=types.SimpleNamespace(client=object(),_preparation=None,_persist=Mock()),
                    _upper_route_key=None)
                bridge=UpperVelocityBridge(bundle)
                replies=iter([{'state':'held','nonce':'n'}, {'state':'ready','nonce':'next','seq':0},
                    {'state':'stopped','stationary':confirmed,'bridge_zero_sent':True}])
                def read(timeout):
                    reply=next(replies)
                    if reply.get('state')=='ready':bridge.stop_event.set()
                    return reply
                stream=types.SimpleNamespace(read=read,send=lambda value:None,stop=lambda:None,close=lambda:None)
                with patch('mapping_keyboard.FactoryKeyboardStream',return_value=stream), \
                        patch.object(bridge,'allowed',return_value=False):
                    bridge._run()
                if confirmed:
                    self.assertIsNone(bundle.spatial._preparation)
                    self.assertFalse(bundle.state.motion_blocked)
                else:
                    self.assertTrue(bundle.state.motion_blocked)
                    self.assertEqual(bundle.spatial._preparation['stage'],'upper_bridge_stop_unconfirmed')
                    bundle.spatial._persist.assert_called_once()
