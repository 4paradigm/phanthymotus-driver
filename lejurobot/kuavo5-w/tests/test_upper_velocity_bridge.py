"""Two-master forwarding is lease/state gated; MPC remains the producer."""
import importlib.util
from pathlib import Path
import threading
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import types
import unittest
from unittest.mock import patch, Mock

from mapping_keyboard import UpperVelocityBridge, stream_source

spec=importlib.util.spec_from_file_location('upper_bridge_remote',Path(__file__).resolve().parents[1]/'host/upper_velocity_bridge.py')
remote=importlib.util.module_from_spec(spec);spec.loader.exec_module(remote)
# The actual SSH payload prepends the keyboard module's standard imports.
import math
remote.math=math


class BridgeGateTests(unittest.TestCase):
    def test_ready_wait_checks_current_control_and_actual_bridge_ack(self):
        b = types.SimpleNamespace()
        bridge = UpperVelocityBridge(b)
        bridge.thread = Mock()
        bridge.thread.is_alive.return_value = True
        bridge.remote_state = 'blocked'
        check = Mock()
        def progress(_):bridge.remote_state = 'ready'
        with patch.object(bridge.closed, 'wait', side_effect=progress):
            bridge.wait_remote_ready(check)
        self.assertGreaterEqual(check.call_count, 2)
        bridge.remote_state = 'ready'
        with self.assertRaisesRegex(RuntimeError, 'operator_stop'):
            bridge.wait_remote_ready(Mock(side_effect=RuntimeError('operator_stop')))
        bridge.error = 'disconnected'
        with self.assertRaisesRegex(RuntimeError, 'bridge_failed'):
            bridge.wait_remote_ready(lambda:None)

    def test_actual_assembled_payload_defines_every_bridge_dependency(self):
        navigation = (Path(__file__).resolve().parents[1]/'host/navigation_rpc.py').read_text()
        # An embedded main guard, including a different quote style, must not
        # truncate any of the protocol helpers in the outer payload.
        for text in (navigation, navigation.replace("'__main__'", '"__main__"')):
            namespace = {'__name__': 'assembled_payload_test'}
            exec(compile(stream_source(text, bridge=True), 'bridge_payload.py', 'exec'), namespace)
            self.assertEqual(namespace['NAVIGATION_SOURCE'], text)
            for name in ('receive', 'dispatch_proxy', 'run_bridge', 'upper_observer', 'UpperInputGate'):
                self.assertTrue(callable(namespace.get(name)), name)

    def test_bridge_keeps_exclusive_lock_after_creating_shared_ownership_state(self):
        import tempfile
        import fcntl
        with tempfile.TemporaryDirectory() as directory:
            lock_path = str(Path(directory)/'bridge.lock')
            builtin_open = open
            def redirect(path, *args, **kwargs):
                return builtin_open(lock_path if path == '/tmp/motus-upper-velocity-bridge.lock' else path,
                                    *args, **kwargs)
            def start():
                with builtin_open(lock_path, 'a') as other:
                    with self.assertRaises(BlockingIOError):
                        fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)
                raise RuntimeError('test_stop_before_ros')
            context = Mock()
            context.Process.return_value.start.side_effect = start
            with patch('builtins.open', side_effect=redirect), \
                    patch.object(remote.multiprocessing, 'get_context', return_value=context):
                with self.assertRaisesRegex(RuntimeError, 'test_stop_before_ros'):
                    remote.run_bridge({})

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
                        patch.object(bridge,'allowed',return_value=False), patch.object(bridge,'owner',return_value='hold'):
                    bridge._run()
                if confirmed:
                    self.assertIsNone(bundle.spatial._preparation)
                    self.assertFalse(bundle.state.motion_blocked)
                else:
                    self.assertTrue(bundle.state.motion_blocked)
                    self.assertEqual(bundle.spatial._preparation['stage'],'upper_bridge_stop_unconfirmed')
                    bundle.spatial._persist.assert_called_once()

    def remote_bundle(self):
        enabled=[True]
        spatial=types.SimpleNamespace(mapping_active=lambda:False,unresolved=lambda:False)
        tracker=types.SimpleNamespace(lock=threading.RLock(),_active={},finish=lambda *args:None)
        bundle=types.SimpleNamespace(tracker=tracker,_closing=False,_transition=None,
            state=types.SimpleNamespace(motion_blocked=False,confirmed_enable=lambda:enabled[0],
                read=lambda key:(True,10.)),_stop_epoch=3,ros1=types.SimpleNamespace(connection=(True,1)),
            _upper_route_key=(3,(True,1)),spatial=spatial)
        return bundle,enabled

    def test_idle_upper_route_returns_remote_but_all_base_owners_and_stops_block_it(self):
        bundle,enabled=self.remote_bundle();bridge=UpperVelocityBridge(bundle)
        self.assertEqual(bridge.owner(),'remote')
        for tool in ('base_drive','base_servo','controlled_spatial','mapping_keyboard'):
            bundle.tracker._active={'base':('id',tool,None)}
            self.assertEqual(bridge.owner(),'card' if tool in ('base_drive','base_servo') else 'hold')
        bundle.tracker._active={}
        bundle.state.motion_blocked=True;self.assertEqual(bridge.owner(),'hold')
        bundle.state.motion_blocked=False;enabled[0]=False;self.assertEqual(bridge.owner(),'hold')
        enabled[0]=True;bundle.spatial.mapping_active=lambda:True;self.assertEqual(bridge.owner(),'hold')

    def test_boot_zero_input_opens_card_gate_without_forwarding_motion(self):
        bundle, _ = self.remote_bundle()
        bridge = UpperVelocityBridge(bundle)
        bundle.state.motion_blocked = True
        bundle._upper_route_key = None
        bundle._transition = {'tool': 'boot_remote_ready'}
        self.assertEqual(bridge.owner(), 'hold')
        bundle._transition['boot_zero_input'] = True
        self.assertEqual(bridge.owner(), 'card')
        self.assertFalse(bridge.allowed())
        bundle._transition = {'tool': 'controlled_spatial', 'boot_zero_input': True}
        self.assertEqual(bridge.owner(), 'hold')

    def test_input_handoff_requires_new_ack_and_checks_cancellation(self):
        bundle, _ = self.remote_bundle()
        bridge = UpperVelocityBridge(bundle)
        bridge.thread = Mock()
        bridge.thread.is_alive.return_value = True
        bridge.input_owner = 'card'
        with self.assertRaisesRegex(RuntimeError, 'input_owner_unconfirmed'):
            bridge.wait_input_owner('card', lambda: None, timeout=.01)
        def acknowledge():
            bridge.input_ack_count += 1
        bridge.wait_input_owner('card', acknowledge)
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            bridge.wait_input_owner('card', Mock(side_effect=RuntimeError('cancelled')))

    def test_completed_native_navigation_requests_handoff_without_resuming_a_paused_body(self):
        bundle,enabled=self.remote_bundle();bridge=UpperVelocityBridge(bundle)
        bundle.start_motion=Mock(return_value=('id',None))
        bundle.tracker.finish=Mock()
        bundle._upper_route_key=None
        enabled[0]=False
        bridge.request_return('controlled_spatial')
        bridge.consider_return()
        bundle.start_motion.assert_not_called()
        # A later explicit operator enable permits a guarded handoff.
        enabled[0]=True
        bridge.consider_return()
        bridge.return_thread.join(1)
        self.assertEqual(bundle.start_motion.call_args.args[:2],('base','remote_control'))
        self.assertEqual(bundle.start_motion.call_args.kwargs['request_epoch'],3)
        bundle.tracker.finish.assert_called_once()

    def test_stop_epoch_revokes_a_pending_automatic_remote_return(self):
        bundle,_=self.remote_bundle();bridge=UpperVelocityBridge(bundle)
        bundle.start_motion=Mock()
        bridge.request_return('controlled_spatial')
        bundle._stop_epoch+=1
        bridge.consider_return()
        self.assertFalse(bridge.return_requested)
        bundle.start_motion.assert_not_called()

    def test_normal_upper_completion_uses_existing_route_instead_of_repreparing(self):
        bundle,_=self.remote_bundle();bridge=UpperVelocityBridge(bundle)
        bundle.start_motion=Mock()
        bridge.request_return('base_drive')
        bridge.consider_return()
        bundle.start_motion.assert_not_called()
        self.assertEqual(bridge.owner(),'remote')
        self.assertFalse(bridge.return_requested)
