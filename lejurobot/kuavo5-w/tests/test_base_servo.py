"""ROS2 navi stream -> guarded ROS1 base, tested without motors or ROS runtime."""
import json
import sys
import math
import time
import threading
import types
import unittest
from unittest.mock import patch
import test_driver as fixtures
from test_driver import driver, wait_until
from base_servo import build_descriptor
from common.control import parse_descriptor


class BaseServoTests(unittest.TestCase):
    def setUp(self):
        fixtures.DriverContractTests.setUp(self)
        self.node.create_subscription = lambda kind, topic, callback, depth: callback
        self.node.destroy_subscription = lambda subscription: None
        self.bundle.spatial.start_chassis = lambda: {'state': 'running'}
        self.bundle.spatial.startup_status = lambda: {'state': 'running'}
        self.servo = self.bundle.base_servo
        self.seq = 0

    tearDown = fixtures.DriverContractTests.tearDown
    call = fixtures.DriverContractTests.call

    def start(self):
        with patch.dict(sys.modules, {'std_msgs.msg': fixtures.std}):
            return self.call('base_servo', 'start', input_topic='/actucore/navi/cmd')

    def send(self, **changes):
        self.seq += 1
        now = int(time.time()*1000)
        message = {'schema': 'motus.control/1', 'mode': 'twist', 'dof': 6,
                   'source': 'test_navi', 'seq': self.seq, 'stamp_ms': now,
                   'obs_stamp_ms': now, 'ttl_ms': 300, 'priority': 50,
                   'values': [.1, 0., 0., 0., 0., .1]}
        message.update(changes)
        self.servo._receive(types.SimpleNamespace(data=json.dumps(message)))

    def nonzero(self):
        return [p for p in self.ros.published if p[0] == '/cmd_vel'
                and any(abs(v)>1e-9 for d in p[2].values() for v in d.values())]

    def test_descriptor_and_lifecycle_do_not_move_without_input(self):
        descriptor = parse_descriptor(build_descriptor())
        self.assertEqual(descriptor.mode, 'twist')
        self.assertEqual(self.call('base_servo', 'start')['state'], 'error')
        result = self.start()
        self.assertEqual(result['state'], 'running')
        self.assertEqual(result['control_interface']['units']['angular'], 'rad/s')
        time.sleep(.08)
        self.assertFalse(self.ros.published)
        self.assertFalse(self.ros.sent_services)
        self.assertEqual(self.call('base_servo','stop')['state'], 'idle')
        self.assertEqual(self.start()['state'], 'running')

    def test_stream_relay_watchdog_and_recovery(self):
        self.start();self.send()
        self.assertTrue(wait_until(lambda:self.servo._applied == 1))
        self.assertTrue(self.nonzero())
        self.assertTrue(wait_until(lambda:self.servo._lease is None))
        self.assertEqual(self.ros.published[-1][2]['linear']['x'], 0.)
        self.assertFalse(self.callbacks)  # stream sessions have no pending ACP
        self.send()
        self.assertTrue(wait_until(lambda:self.servo._applied == 2))

    def test_stale_wrong_mode_pinned_axes_and_replayed_frames_are_rejected(self):
        self.start()
        for changes in ({'stamp_ms': 1}, {'obs_stamp_ms': 1}, {'mode': 'joint'},
                        {'values': [0.,0.,.1,0.,0.,0.]}, {'values': [float('nan')]*6}):
            self.send(**changes);time.sleep(.06)
        self.assertFalse(self.nonzero())
        self.send(seq=100)
        self.assertTrue(wait_until(lambda:self.servo._applied == 1))
        self.send(seq=100)
        time.sleep(.08)
        self.assertEqual(self.servo._applied,1)

    def test_pause_is_latched_and_global_stop_does_not_auto_resume(self):
        self.start();self.send()
        self.assertTrue(wait_until(lambda:self.nonzero()))
        self.assertEqual(self.call('base_servo','pause')['state'], 'paused')
        count = len(self.nonzero());self.send();time.sleep(.1)
        self.assertEqual(len(self.nonzero()),count)
        self.call('base_servo','resume');self.send()
        self.assertTrue(wait_until(lambda:len(self.nonzero())>count))
        self.bundle.pause('test_global_stop')
        count=len(self.nonzero());self.send();time.sleep(.1)
        self.assertEqual(len(self.nonzero()),count)
        self.assertEqual(self.servo.info()['state'],'paused')

    def test_explicit_base_command_preempts_stream(self):
        self.start();self.send()
        self.assertTrue(wait_until(lambda:self.nonzero()))
        result=self.call('base_drive','move',vx=.1,duration=.2)
        self.assertEqual(result['state'],'executing')
        self.assertEqual(self.servo.info()['state'],'paused')
        self.send();time.sleep(.06)
        self.assertNotEqual(self.bundle.tracker._active['base'][1],'base_servo')

    def test_refuses_conflicting_native_task(self):
        with patch.object(self.bundle.spatial,'unresolved',return_value=True):
            self.assertEqual(self.start()['state'],'error')
        self.assertFalse(self.ros.published)

    def test_slow_preparation_never_executes_expired_target(self):
        original=self.bundle.start_motion
        def delayed(*args,**kwargs):
            time.sleep(.35)
            return original(*args,**kwargs)
        with patch.object(self.bundle,'start_motion',side_effect=delayed):
            self.start();self.send()
            self.assertTrue(wait_until(lambda:self.servo._last is not None))
        self.assertFalse(self.nonzero())
        self.send()
        self.assertTrue(wait_until(lambda:self.nonzero()))

    def test_initial_partial_send_failure_blocks_and_pauses(self):
        original=self.ros.publish
        failed=[]
        def send(topic,*args):
            original(topic,*args)
            if topic=='/cmd_vel' and not failed:
                failed.append(True)
                raise ConnectionError('partial delivery')
        self.ros.publish=send
        self.start();self.send()
        self.assertTrue(wait_until(lambda:self.bundle.state.motion_send_uncertain))
        self.assertTrue(wait_until(lambda:self.servo.info()['state']=='paused'))
        self.assertTrue(self.bundle.state.motion_blocked)
        self.assertFalse(self.nonzero())

    def test_pending_chassis_check_and_old_topic_callback_cannot_drive(self):
        with patch.object(self.bundle.spatial,'startup_status',return_value={'state':'loading'}):
            self.assertEqual(self.start()['state'],'loading')
            self.send();time.sleep(.1)
            self.assertFalse(self.nonzero())
        old = self.servo._subscription
        self.call('base_servo','stop')
        self.start()
        now=int(time.time()*1000)
        old(types.SimpleNamespace(data=json.dumps({'schema':'motus.control/1',
            'mode':'twist','dof':6,'source':'old_topic','seq':1,'stamp_ms':now,
            'obs_stamp_ms':now,'ttl_ms':300,'values':[.1,0,0,0,0,0]})))
        time.sleep(.1)
        self.assertFalse(self.nonzero())

    def test_mode_feedback_loss_stops_stream_and_requires_resume(self):
        self.start();self.send()
        self.assertTrue(wait_until(lambda:self.nonzero()))
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode',0)
        self.assertTrue(wait_until(lambda:self.servo.info()['state']=='paused'))
        count=len(self.nonzero());self.send();time.sleep(.1)
        self.assertEqual(len(self.nonzero()),count)

    def test_reconnect_invalidates_owned_stream_even_when_startup_becomes_idle(self):
        self.start();self.send()
        self.assertTrue(wait_until(lambda:self.nonzero()))
        with patch.object(self.bundle.spatial,'startup_status',return_value={'state':'idle'}):
            self.ros.connection=(True,2)
            self.assertTrue(wait_until(lambda:self.servo.info()['state']=='paused'))
        count=len(self.nonzero());self.send();time.sleep(.1)
        self.assertEqual(len(self.nonzero()),count)

    def test_zero_send_failure_on_explicit_pause_blocks_new_motion(self):
        self.start();self.send()
        self.assertTrue(wait_until(lambda:self.nonzero()))
        original=self.ros.publish
        def failure(topic,*args):
            if topic=='/cmd_vel':
                raise ConnectionError('failed stop')
            original(topic,*args)
        self.ros.publish=failure
        self.assertEqual(self.call('base_servo','pause')['state'],'error')
        self.assertTrue(self.bundle.state.motion_send_uncertain)
        self.assertTrue(self.bundle.state.motion_blocked)
        self.ros.publish=original

    def test_pause_does_not_confirm_stop_ahead_of_a_late_enable(self):
        self.ros.emit('/enable_control_state',False)
        self.bundle.state.verify_enable()
        entered, release=threading.Event(),threading.Event()
        original=self.ros.wait_service
        def delayed(pending,timeout,cancelled=None):
            if pending[0]=='/enable_control' and pending[2]['data'] is True:
                entered.set();release.wait(2)
                response=original(pending,timeout,None)
                if cancelled.is_set():
                    raise InterruptedError('late enable resolved after cancellation')
                return response
            return original(pending,timeout,cancelled)
        with patch.object(self.ros,'wait_service',side_effect=delayed):
            self.start();self.send()
            self.assertTrue(entered.wait(1))
            try:
                result=self.call('base_servo','pause')
                self.assertEqual(result['state'],'error')
                self.assertTrue(self.bundle.state.remote_resume_unresolved)
            finally:
                release.set()
            self.assertTrue(wait_until(lambda:self.ros.latched.get('/enable_control_state') is True))
            self.assertTrue(wait_until(lambda:self.bundle._transition is None))
        self.assertEqual(self.bundle.pause('confirm_stop_after_late_enable')['state'], 'paused')
        self.assertFalse(self.ros.latched['/enable_control_state'])
        self.assertFalse(self.nonzero())
        self.assertEqual(self.servo.info()['state'],'paused')

    def test_removed_absolute_action_and_new_profile_limits(self):
        schema=next(t for t in self.bundle.get_all_tools() if t['name']=='base_drive')['inputSchema']
        self.assertNotIn('move_to',schema['properties']['action']['enum'])
        self.assertEqual(self.call('base_drive','move_to',x=0,y=0,yaw=0)['state'],'error')
        self.assertEqual(driver.SPEED_FRACTIONS,{'low':.3,'medium':.6,'high':1.})
        self.assertAlmostEqual(driver.LINEAR_LIMIT,.8)
        self.assertAlmostEqual(driver.ANGULAR_LIMIT,1.)
        self.assertEqual(self.call('base_drive','move',vx=.8,vy=.8,duration=1)['state'],'error')
