"""ROS2 navi stream -> guarded ROS1 base, tested without motors or ROS runtime."""
import json
import sys
import math
import time
import threading
import tempfile
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
        self.assertEqual(result['state'], 'ready')
        self.assertEqual(result['stream_state'], 'running')
        self.assertEqual(self.start()['state'], 'ready')
        self.assertEqual(result['control_interface']['units']['angular'], 'rad/s')
        time.sleep(.08)
        self.assertFalse(self.ros.published)
        self.assertFalse(self.ros.sent_services)
        self.assertEqual(self.call('base_servo','stop')['state'], 'idle')
        self.assertEqual(self.start()['state'], 'ready')

    def test_stream_relay_watchdog_and_recovery(self):
        self.start();self.send()
        self.assertTrue(wait_until(lambda:self.servo._applied == 1))
        self.assertTrue(self.nonzero())
        self.assertTrue(wait_until(lambda:self.servo._lease is None))
        self.assertEqual(self.ros.published[-1][2]['linear']['x'], 0.)
        self.assertFalse(self.callbacks)  # stream sessions have no pending ACP
        self.send()
        self.assertTrue(wait_until(lambda:self.servo._applied == 2))

    def test_stop_confirms_idle_without_replaying_a_historical_stream_error(self):
        self.start()
        self.servo._error = 'control-enable verification is missing or stale'
        callback = self.servo._subscription
        for _ in range(2):
            self.assertEqual(self.call('base_servo', 'stop'), {'state': 'idle'})
        import test_main
        http = test_main.McpTests()
        with patch.object(test_main.main, '_bundle', self.bundle):
            status, response = http.request(http.rpc('base_servo', {'action': 'stop'}))
        self.assertEqual(status, 200)
        self.assertFalse(response['result'].get('isError', False))
        self.assertEqual(json.loads(response['result']['content'][0]['text']), {'state': 'idle'})
        info = self.servo.info()
        self.assertEqual(info['state'], 'idle')
        self.assertEqual(info['input'], '')
        self.assertEqual(info['error'], 'control-enable verification is missing or stale')
        self.assertIsNone(self.servo._subscription)
        self.assertTrue(self.servo._paused)
        self.assertIsNone(self.servo._latest)
        callback(types.SimpleNamespace(data=json.dumps({'values': [.1]*6})))
        self.assertIsNone(self.servo._latest)
        self.assertFalse(self.ros.published)

    def test_stop_preserves_a_current_zero_send_failure(self):
        self.start(); self.send()
        self.assertTrue(wait_until(lambda: self.nonzero()))
        original = self.ros.publish
        def failure(topic, *args):
            if topic == '/cmd_vel':
                raise ConnectionError('failed stop')
            return original(topic, *args)
        try:
            with patch.object(self.ros, 'publish', side_effect=failure):
                result = self.call('base_servo', 'stop')
                self.assertEqual(result['state'], 'error')
                self.assertIn('zero velocity send failed', result['error'])
                self.assertTrue(self.bundle.state.motion_send_uncertain)
                self.assertTrue(self.bundle.state.motion_blocked)
        finally:
            self.call('base_servo', 'stop')

    def test_start_and_pause_do_not_fail_mcp_because_of_a_previous_stream_error(self):
        import test_main
        http = test_main.McpTests()
        self.start()
        self.servo._error = 'previous stream verification failure'
        with patch.dict(sys.modules, {'std_msgs.msg': fixtures.std}), \
                patch.object(test_main.main, '_bundle', self.bundle):
            for action in ('start', 'pause', 'stop', 'start', 'pause'):
                _, response = http.request(http.rpc('base_servo', {
                    'action': action, 'input_topic': '/actucore/navi/cmd'}))
                result = json.loads(response['result']['content'][0]['text'])
                self.assertFalse(response['result'].get('isError', False), (action, result))
                self.assertEqual(result['state'], {'start': 'ready', 'pause': 'paused', 'stop': 'idle'}[action])
                if action != 'stop':
                    self.assertEqual(result['last_stream_error'], 'previous stream verification failure')
                self.assertEqual(self.servo.info()['error'], 'previous stream verification failure')
        self.assertFalse(self.ros.published)
        self.assertFalse(self.ros.sent_services)
        self.assertEqual(self.call('base_servo', 'resume')['state'], 'running')
        self.assertIsNone(self.servo.info()['error'])

    def test_stop_does_not_confirm_idle_when_subscription_cleanup_raises(self):
        self.start()
        with patch.object(self.node, 'destroy_subscription', side_effect=RuntimeError('cleanup failed')):
            result = self.call('base_servo', 'stop')
        self.assertEqual(result['state'], 'error')
        self.assertIn('cleanup failed', result['error'])
        self.assertIsNotNone(self.servo._subscription)
        self.assertTrue(self.servo._paused)
        self.assertEqual(self.call('base_servo', 'stop'), {'state': 'idle'})

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

    def test_explicit_native_navigation_preempts_stream_and_keeps_it_paused(self):
        import tempfile
        from controlled_spatial import SpatialStore
        from test_controlled_spatial import Client
        with tempfile.TemporaryDirectory() as folder:
            self.bundle.spatial.client = client = Client()
            self.bundle.spatial.store = SpatialStore(folder+'/db')
            client.check = {'success': True, 'status': 1, 'arrived': False}
            self.start(); self.send()
            self.assertTrue(wait_until(lambda: self.nonzero()))
            result = self.call('controlled_spatial', 'navigate_to_pose', x=1, y=2, yaw=.3)
            self.assertEqual(result['state'], 'executing')
            self.assertTrue(wait_until(lambda: (self.bundle.spatial._task or {}).get('task_id')))
            self.assertEqual(self.servo.info()['state'], 'paused')
            before = self.servo._applied
            self.send(); time.sleep(.06)
            self.assertEqual(self.servo._applied, before)
            self.assertEqual(self.bundle.tracker._active['base'][1], 'controlled_spatial')
            self.assertIs(self.bundle.state.read('upper_velocity_enabled')[0], False)
            self.assertEqual(self.call('controlled_spatial', 'stop_nav')['state'], 'paused')

    def test_starting_servo_during_base_drive_only_connects_a_paused_input(self):
        result = self.call('base_drive', 'move', vx=.1, duration=1)
        started = self.start()
        self.assertEqual(started['state'], 'ready')
        self.assertEqual(started['stream_state'], 'paused')
        self.send(); time.sleep(.06)
        self.assertTrue(self.bundle.tracker.current('base', result['action_id']))
        self.assertEqual(self.servo._applied, 0)

    def test_start_connects_paused_without_overriding_an_unresolved_native_task(self):
        with patch.object(self.bundle.spatial,'unresolved',return_value=True):
            result = self.start()
            self.assertEqual(result['state'], 'ready')
            self.assertEqual(result['stream_state'], 'paused')
            self.send()
            time.sleep(.06)
            self.assertEqual(self.call('base_servo', 'resume')['state'], 'error')
        self.assertFalse(self.ros.published)
        self.assertFalse(self.ros.sent_services)

    def test_start_and_resume_join_live_shared_chassis_preparation(self):
        from controlled_spatial import ControlledSpatialPlugin, SpatialStore
        from test_controlled_spatial import Client
        spatial = self.bundle.spatial
        with tempfile.TemporaryDirectory() as folder:
            spatial.store = SpatialStore(folder + '/db')
            spatial.client = client = Client()
            original = client.native
            for resume in (False, True):
                with self.subTest(resume=resume):
                    if resume:
                        self.call('base_servo', 'pause')
                    entered, release = threading.Event(), threading.Event()
                    def prepare(operation, **args):
                        if operation == 'prepare':
                            entered.set()
                            release.wait(2)
                        return original(operation, **args)
                    spatial._startup_key = None
                    with patch.object(client, 'native', side_effect=prepare), \
                            patch.object(spatial, 'start_chassis', side_effect=lambda: ControlledSpatialPlugin.start_chassis(spatial)), \
                            patch.object(spatial, 'startup_status', side_effect=lambda: ControlledSpatialPlugin.startup_status(spatial)):
                        spatial.start_chassis()
                        try:
                            self.assertTrue(entered.wait(1))
                            self.assertEqual(self.bundle._transition['action'], 'prepare_chassis')
                            self.assertTrue(spatial.unresolved())
                            result = self.call('base_servo', 'resume') if resume else self.start()
                            self.assertEqual(result['state'], 'loading' if resume else 'ready')
                            self.assertEqual(self.servo.info()['state'], 'loading')
                            self.assertFalse(self.servo._paused)
                            self.assertIsNone(self.servo._lease)
                            self.assertFalse(self.ros.published)
                            self.assertFalse(self.ros.sent_services)
                        finally:
                            release.set()
                            spatial._startup_thread.join(1)
                        self.assertFalse(spatial._startup_thread.is_alive())
                        self.assertEqual(self.servo.info()['state'], 'running')
            self.assertEqual([op for op, _ in client.calls].count('prepare'), 2)
            spatial.store.close()

    def test_start_cannot_join_orphan_preparation_or_other_control_transition(self):
        spatial = self.bundle.spatial
        spatial._restored = True
        spatial._preparation = {'delivery_uncertain': True}
        started = self.start()
        self.assertEqual(started['state'], 'ready')
        self.assertEqual(started['stream_state'], 'paused')
        self.assertEqual(self.call('base_servo', 'resume')['state'], 'error')
        spatial._preparation = None
        self.call('base_servo', 'stop')
        with self.bundle.tracker.lock:
            token = self.bundle._new_transition('resume')
        try:
            self.assertEqual(self.start()['state'], 'error')
            self.assertIsNone(self.servo._subscription)
        finally:
            self.bundle._end_control(token)
        self.assertFalse(self.ros.published)
        self.assertFalse(self.ros.sent_services)

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
            result = self.start()
            self.assertEqual(result['state'], 'ready')
            self.assertEqual(result['stream_state'], 'loading')
            self.assertEqual(result['startup']['state'], 'loading')
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
