"""Offline fault injection only: no robot access and no proof of physical stop."""
import importlib.util
import json
import multiprocessing
from pathlib import Path
import queue
import sys
import threading
import time
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'unitree/go1'))
import loco_control
import sdk_proxy
import go1_sdk_client


class FakeClient:
    available = True

    def __init__(self):
        self.control_epoch = 0
        self.seq = 1
        self.velocity = [0., 0.]
        self.yaw = 0.
        self.mode = 1
        self.height = .3
        self.fresh = True
        self.frozen = False
        self.stop_works = True
        self.behavior = 'normal'
        self.calls = []
        self.owner = None
        self.send_errors = 0
        self.sent_age = .01
        self.rpy = [0., 0., 0.]

    def snapshot(self):
        if not self.frozen:
            self.seq += 1
        return dict(fresh=self.fresh, telemetry_age_sec=.01 if self.fresh else 2,
                    sample_seq=self.seq, velocity=list(self.velocity), yaw_speed=self.yaw,
                    imu={'rpy_rad': self.rpy}, body_height=self.height, mode=self.mode,
                    gait=1, last_send_age_sec=self.sent_age, send_error_count=self.send_errors)

    def request_stop(self):
        self.calls.append('stop')
        self.control_epoch += 1
        if self.stop_works:
            self.velocity = [0., 0.]
            self.yaw = 0.

    def claim_control(self, owner, epoch):
        if epoch != self.control_epoch:
            raise sdk_proxy.SdkError('CANCELLED', 'stale command')
        self.owner = owner

    def release_control(self, owner):
        if self.owner == owner:
            self.owner = None

    def control_move(self, owner, epoch, vx, vy, vyaw, until):
        if epoch != self.control_epoch:
            raise sdk_proxy.SdkError('CANCELLED', 'stale command')
        self.calls.append('move')
        if self.behavior == 'rpc_error':
            raise sdk_proxy.SdkError('SDK_TIMEOUT', 'injected timeout')
        if self.behavior == 'send_error':
            self.send_errors += 1
        if self.behavior == 'stale':
            self.fresh = False
        if self.behavior == 'normal':
            self.velocity, self.yaw = [vx, vy], vyaw
        elif self.behavior == 'partial':
            self.velocity, self.yaw = [vx, 0.], 0.
        elif self.behavior == 'reverse':
            self.velocity, self.yaw = [-vx, -vy], -vyaw

    def control_posture(self, owner, epoch, mode):
        self.calls.append('posture')
        if self.behavior == 'normal':
            self.mode = 1 if mode in (6, 8) else mode
            self.height = .1 if mode == 5 else .3


class LocoTests(unittest.TestCase):
    def setUp(self):
        self.client = FakeClient()
        self.card = loco_control.make_loco_confirmed({'control_enabled': True}, '', None, self.client)
        self.card._poll = .003
        self.card._settle = .01
        # Real-thread tests need scheduler headroom on loaded CI/developer hosts.
        # Keep production grace; do not turn a 25 ms scheduling pause into a
        # synthetic hardware failure. Fault cases still have bounded deadlines.
        self.card._grace = .3
        self.card._stop_timeout = .5
        self.card._posture_timeout = .5
        self.card._notify = mock.Mock()

    def move(self, **args):
        return self.card.dispatch('move', dict(vx=.1, duration=.2, **args))

    def finished(self, aid):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            result = self.card.dispatch('status', {'action_id': aid})
            if result['status'] != 'accepted':
                return result
            time.sleep(.003)
        self.fail('background job did not finish')

    def test_normal_move_requires_motion_and_confirmed_stop(self):
        result = self.move()
        self.assertTrue(result['ok'], result)
        self.assertEqual(result['status'], 'completed')
        self.assertTrue(result['stop']['stop_confirmed'])
        self.assertNotIn('action_id', result)
        self.assertEqual(self.card.dispatch('status', {})['command_id'], result['command_id'])

    def test_no_motion_returns_error(self):
        self.client.behavior = 'none'
        result = self.move()
        self.assertFalse(result['ok'])
        self.assertEqual(result['code'], 'MOTION_NOT_OBSERVED')
        self.assertTrue(result['stop']['stop_confirmed'])

    def test_rejected_claim_does_not_stop_existing_card(self):
        self.client.velocity = [.1, 0.]
        self.client.owner = 'original-card'
        with mock.patch.object(self.client, 'claim_control', side_effect=
                               sdk_proxy.SdkError('RESOURCE_BUSY', 'existing movement')):
            result = self.move()
        self.assertEqual(result['code'], 'RESOURCE_BUSY')
        self.assertEqual(self.client.calls, [])
        self.assertEqual(self.client.velocity, [.1, 0.])
        self.assertEqual(self.client.owner, 'original-card')
        self.assertEqual(self.client.control_epoch, 0)

    def test_unknown_claim_outcome_still_requests_stop(self):
        with mock.patch.object(self.client, 'claim_control', side_effect=
                               sdk_proxy.SdkError('SDK_TIMEOUT', 'unknown outcome')):
            result = self.move()
        self.assertEqual(result['code'], 'SDK_TIMEOUT')
        self.assertIn('stop', self.client.calls)
        self.assertTrue(result['stop']['stop_confirmed'])

    def test_all_requested_axes_must_match(self):
        self.client.behavior = 'partial'
        self.assertEqual(self.move(vy=.1, vyaw=10)['code'], 'MOTION_NOT_OBSERVED')

    def test_reverse_motion_is_not_success(self):
        self.client.behavior = 'reverse'
        self.assertEqual(self.move()['code'], 'MOTION_NOT_OBSERVED')

    def test_rpc_error_not_swallowed(self):
        self.client.behavior = 'rpc_error'
        result = self.move()
        self.assertEqual(result['code'], 'SDK_TIMEOUT')
        self.assertIn('stop', self.client.calls)

    def test_send_failure_not_success(self):
        self.client.behavior = 'send_error'
        self.assertEqual(self.move()['code'], 'UDP_SEND_FAILED')

    def test_stale_during_move_requests_stop(self):
        self.client.behavior = 'stale'
        result = self.move()
        self.assertFalse(result['ok'])
        self.assertEqual(result['code'], 'STOP_UNCONFIRMED')
        self.assertEqual(result['cause_code'], 'TELEMETRY_STALE')
        self.assertIn('stop', self.client.calls)

    def test_preflight_failures_do_not_move(self):
        for attr, value, code in [('available', False, 'STUB_MODE'),
                                  ('fresh', False, 'TELEMETRY_STALE'),
                                  ('sent_age', 2., 'UDP_UNAVAILABLE'),
                                  ('mode', 0, 'PRECONDITION_FAILED'),
                                  ('rpy', [1., 0., 0.], 'PRECONDITION_FAILED')]:
            with self.subTest(attr=attr):
                old = getattr(self.client, attr)
                setattr(self.client, attr, value)
                self.assertEqual(self.move()['code'], code)
                setattr(self.client, attr, old)
                self.assertNotIn('move', self.client.calls)

    def test_invalid_input_does_not_write(self):
        for args in [dict(vx=float('nan')), dict(vx=True), dict(vx=2),
                     dict(vy=.7), dict(vyaw=float('inf')), dict(duration=-1),
                     dict(vx=.01), dict(vyaw=1)]:
            with self.subTest(args=args):
                self.assertEqual(self.card.dispatch('move', args)['code'], 'INVALID_ARGUMENT')
        self.assertEqual(self.client.calls, [])

    def test_stop_move_and_lifecycle_schema(self):
        result = self.card.dispatch('stop_move', {})
        self.assertTrue(result['stop_confirmed'])
        schema = self.card.get_tool()['inputSchema']
        self.assertEqual(schema['x-hooks']['on_interrupt_motion']['action'], 'stop_move')
        for action in ('start', 'stop', 'stop_move'):
            self.assertIn(action, schema['properties']['action']['enum'])
            self.assertIn(action, schema['x-action-params'])
            self.assertNotIn(action, schema['x-completion']['actions'])

    def test_lifecycle_start_is_local_with_control_disabled_or_enabled(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                card = loco_control.make_loco_confirmed({'control_enabled': enabled}, '', None, self.client)
                card._closed = True
                self.client.available = False
                with mock.patch.object(card, '_preflight', side_effect=AssertionError('hardware read')):
                    self.assertEqual(card.dispatch('start', {}), {'state': 'ready'})
                    self.assertEqual(card.dispatch('start', {}), {'state': 'ready'})
                self.assertFalse(card._closed)
                self.assertEqual(card._control_enabled, enabled)
                self.assertEqual(self.client.calls, [])
                self.assertEqual(self.client.control_epoch, 0)

    def test_disabled_lifecycle_stop_is_local_and_start_does_not_enable_control(self):
        card = loco_control.make_loco_confirmed({}, '', None, self.client)
        with mock.patch.object(card, '_confirm_stop', side_effect=AssertionError('hardware read')):
            self.assertEqual(card.dispatch('stop', {}), {'state': 'idle'})
            self.assertEqual(card.dispatch('stop', {}), {'state': 'idle'})
        self.assertTrue(card._closed)
        self.assertEqual(card.dispatch('start', {}), {'state': 'ready'})
        self.assertEqual(card.dispatch('move', {'vx': .1})['code'], 'CONTROL_DISABLED')
        self.assertEqual(card.dispatch('stop_move', {})['code'], 'CONTROL_DISABLED')
        self.assertEqual(self.client.calls, [])

    def test_lifecycle_idle_does_not_claim_physical_stop(self):
        self.client.stop_works = False
        self.client.velocity = [.2, 0.]
        self.client.fresh = False
        with mock.patch.object(self.card, '_confirm_stop', side_effect=AssertionError('must not wait')):
            self.assertEqual(self.card.dispatch('stop', {}), {'state': 'idle'})
        self.assertEqual(self.client.calls, ['stop'])
        self.assertEqual(self.client.velocity, [.2, 0.])
        self.assertTrue(self.card._closed)
        self.assertEqual(self.move()['code'], 'CARD_STOPPED')
        # Physical-stop check is still available while the lifecycle is idle.
        self.assertEqual(self.card.dispatch('stop_move', {})['code'], 'STOP_UNCONFIRMED')

    def test_lifecycle_restart_does_not_rearm_or_resurrect_old_job(self):
        entered, release = threading.Event(), threading.Event()

        def monitor(job):
            entered.set()
            if not release.wait(2):
                raise AssertionError('test did not release monitor')
            self.card._check_cancel(job)

        with mock.patch.object(self.card, '_monitor', side_effect=monitor):
            accepted = self.card.dispatch('move', dict(vx=.1, duration=3))
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.card.dispatch('stop', {}), {'state': 'idle'})
                stopped_epoch = self.client.control_epoch
                self.assertEqual(self.card.dispatch('start', {}), {'state': 'ready'})
                self.assertEqual(self.client.control_epoch, stopped_epoch)
                self.assertTrue(self.card._active['cancel'].is_set())
                self.assertEqual(self.move()['code'], 'RESOURCE_BUSY')
            finally:
                release.set()
            self.assertEqual(self.finished(accepted['action_id'])['status'], 'cancelled')
            self.assertNotIn('move', self.client.calls)
        self.assertTrue(self.move()['ok'])

    def test_lifecycle_stop_failure_is_not_silently_reported_as_success(self):
        with mock.patch.object(self.client, 'request_stop', side_effect=
                               sdk_proxy.SdkError('SDK_UNAVAILABLE', 'offline')):
            result = self.card.dispatch('stop', {})
        self.assertEqual(result['state'], 'idle')
        self.assertFalse(result['ok'])
        self.assertFalse(result['stop_confirmed'])
        self.assertEqual(result['code'], 'SDK_UNAVAILABLE')
        self.assertTrue(self.card._closed)
        self.assertEqual(self.move()['code'], 'CARD_STOPPED')

    def test_zero_velocity_uses_physical_stop_without_closing_lifecycle(self):
        result = self.card.dispatch('move', {'vx': 0})
        self.assertTrue(result['stop_confirmed'])
        self.assertEqual(result['action'], 'stop_move')
        self.assertFalse(self.card._closed)

    def test_image_uses_mounted_dds_profile_without_transport_override(self):
        root = Path(loco_control.__file__).parent
        self.assertNotIn('FASTDDS_BUILTIN_TRANSPORTS', (root / 'Dockerfile').read_text())
        fragment = (root / 'deploy/service.yml').read_text()
        self.assertNotIn('FASTDDS_BUILTIN_TRANSPORTS', fragment)
        self.assertIn('/opt/phanthy-motus/dds-local.xml:/opt/phanthy-motus/dds-local.xml:ro', fragment)
        self.assertIn('FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml', fragment)

    def test_stop_unavailable_still_latches(self):
        self.client.fresh = False
        self.assertEqual(self.card.dispatch('stop_move', {})['code'], 'STOP_UNCONFIRMED')
        self.assertEqual(self.client.calls[0], 'stop')

    def test_stop_still_moving_not_confirmed(self):
        self.client.stop_works = False
        self.client.velocity = [.2, 0.]
        self.assertFalse(self.card.dispatch('stop_move', {})['stop_confirmed'])

    def test_replayed_sample_cannot_confirm_stop(self):
        self.client.frozen = True
        self.assertFalse(self.card.dispatch('stop_move', {})['stop_confirmed'])

    def test_async_cancel_and_no_old_command_resurrection(self):
        accepted = self.card.dispatch('move', dict(vx=.1, duration=3))
        self.assertEqual(accepted['status'], 'accepted')
        self.assertFalse(accepted['executed'])
        self.assertEqual(self.move()['code'], 'RESOURCE_BUSY')
        self.assertTrue(self.card.dispatch('stop_move', {})['stop_confirmed'])
        result = self.finished(accepted['action_id'])
        self.assertEqual(result['status'], 'cancelled')
        after_stop = self.client.calls[self.client.calls.index('stop') + 1:]
        self.assertNotIn('move', after_stop)
        self.assertIsNone(self.client.owner)
        self.assertTrue(self.move()['ok'])  # explicit new action rearms

    def test_stop_during_preflight_cannot_be_cleared_by_old_request(self):
        original = self.client.snapshot
        def interrupted_snapshot():
            snap = original()
            self.client.request_stop()
            return snap
        self.client.snapshot = interrupted_snapshot
        self.assertEqual(self.move()['code'], 'CANCELLED')
        self.assertNotIn('move', self.client.calls)

    def test_postures_and_explicit_danger_confirmation(self):
        for action in loco_control.POSTURES:
            with self.subTest(action=action):
                if action in ('damp', 'recovery_stand'):
                    self.assertEqual(self.card.dispatch(action, {})['code'], 'PRECONDITION_FAILED')
                accepted = self.card.dispatch(action, {'confirm': True})
                result = self.finished(accepted['action_id'])
                self.assertEqual(result['status'], 'completed')
                self.assertTrue(result['holding_control'])
                self.assertEqual(self.client.owner, accepted['action_id'])
                self.assertTrue(self.card.dispatch('stop_move', {})['stop_confirmed'])
        self.assertEqual(self.card._notify.call_count, 5)

    def hold_posture(self):
        accepted = self.card.dispatch('balance_stand', {})
        self.assertEqual(self.finished(accepted['action_id'])['status'], 'completed')
        return accepted['action_id']

    def test_held_posture_blocks_new_job_until_explicit_stop(self):
        aid = self.hold_posture()
        self.assertEqual(self.move()['code'], 'RESOURCE_BUSY')
        self.assertEqual(self.card.dispatch('stand_down', {})['code'], 'RESOURCE_BUSY')
        self.assertEqual(self.card.dispatch('start', {}), {'state': 'ready'})
        self.assertEqual(self.client.owner, aid)
        self.assertTrue(self.card.dispatch('stop_move', {})['stop_confirmed'])
        self.assertIsNone(self.client.owner)
        self.assertFalse(self.card.dispatch('status', {'action_id': aid})['holding_control'])
        self.assertTrue(self.move()['ok'])

    def test_lifecycle_stop_releases_held_posture_only_after_latching(self):
        self.hold_posture()
        original = self.client.release_control
        def release(owner):
            self.assertEqual(self.client.calls[-1], 'stop')
            original(owner)
        with mock.patch.object(self.client, 'release_control', side_effect=release):
            self.assertEqual(self.card.dispatch('stop', {}), {'state': 'idle'})
        self.assertIsNone(self.card._held)
        self.assertIsNone(self.client.owner)
        self.assertEqual(self.card.dispatch('start', {}), {'state': 'ready'})
        self.assertTrue(self.move()['ok'])

    def test_failed_held_release_is_retained_for_stop_retry(self):
        aid = self.hold_posture()
        with mock.patch.object(self.client, 'release_control', side_effect=
                               sdk_proxy.SdkError('SDK_TIMEOUT', 'unknown release')):
            result = self.card.dispatch('stop', {})
        self.assertEqual(result['code'], 'SDK_TIMEOUT')
        self.assertEqual(self.card._held['id'], aid)
        self.card.start()
        self.assertEqual(self.move()['code'], 'RESOURCE_BUSY')
        self.assertTrue(self.card.dispatch('stop_move', {})['stop_confirmed'])
        self.assertIsNone(self.card._held)

    def test_failed_stop_does_not_release_held_posture_or_leak_stopping(self):
        aid = self.hold_posture()
        with mock.patch.object(self.client, 'request_stop', side_effect=RuntimeError('stop failed')):
            with self.assertRaisesRegex(RuntimeError, 'stop failed'):
                self.card.dispatch('stop_move', {})
        self.assertEqual(self.client.owner, aid)
        self.assertEqual(self.card._stopping, 0)
        self.assertTrue(self.card.dispatch('stop_move', {})['stop_confirmed'])

    def test_stop_racing_posture_completion_does_not_leave_held_owner(self):
        entered, resume = threading.Event(), threading.Event()
        def monitor(job):
            self.client.control_posture(job['id'], job['epoch'], 1)
            entered.set()
            self.assertTrue(resume.wait(2))
            return {}
        with mock.patch.object(self.card, '_monitor', side_effect=monitor):
            accepted = self.card.dispatch('balance_stand', {})
            try:
                self.assertTrue(entered.wait(1))
                self.card.stop()
            finally:
                resume.set()
            self.assertEqual(self.finished(accepted['action_id'])['status'], 'cancelled')
        self.assertIsNone(self.card._held)
        self.assertIsNone(self.client.owner)

    def test_completed_posture_rejects_real_legacy_card_via_worker_arbitration(self):
        import controllers
        with mock.patch.object(go1_sdk_client.Go1HighSdkClient, '_init_sdk'):
            sdk = go1_sdk_client.Go1HighSdkClient()
        sdk.available = True
        sdk._control_owner = None
        sdk.snapshot = self.client.snapshot
        proxy = sdk_proxy.SdkProxy.__new__(sdk_proxy.SdkProxy)
        proxy._epoch = multiprocessing.Value('Q', 0)
        proxy._stop_signal = sdk._stop_signal
        def call(cmd, args=None, kwargs=None, timeout=1., owner=None, epoch=None):
            return sdk_proxy._execute(sdk, dict(cmd=cmd, args=args or [], kwargs=kwargs or {},
                owner=owner, epoch=proxy.control_epoch if epoch is None else epoch,
                deadline=time.monotonic()+timeout), proxy._stop_signal, proxy._epoch)
        proxy._call = call
        proxy.available = True
        self.card._client = proxy
        aid = self.hold_posture()
        self.assertEqual(sdk._control_owner, aid)
        original_pose = dict(sdk._posture)
        old = controllers.make_loco({}, '', None, proxy)
        for action, args in [('stand_down', {}), ('move', {'vx': .1}), ('stop', {})]:
            with self.subTest(action=action):
                with self.assertRaises(sdk_proxy.SdkError) as error:
                    old.dispatch(action, args)
                self.assertEqual(error.exception.code, 'RESOURCE_BUSY')
        self.assertEqual(sdk._posture, original_pose)
        self.assertTrue(self.card.dispatch('stop_move', {})['stop_confirmed'])
        self.assertIsNone(sdk._control_owner)
        with self.assertRaises(sdk_proxy.SdkError) as error:
            old.dispatch('stand_down', {})
        self.assertEqual(error.exception.code, 'STOP_LATCHED')
        sdk._cmd = types.SimpleNamespace()
        sdk._compose_cmd()
        self.assertIsNone(sdk._posture)
        # A new explicit confirmed action, not lifecycle start, rearms the SDK.
        self.hold_posture()
        self.assertIsNotNone(sdk._control_owner)

    def test_posture_not_reached_errors_and_stops(self):
        self.client.behavior = 'none'
        accepted = self.card.dispatch('stand_down', {})
        result = self.finished(accepted['action_id'])
        self.assertEqual(result['code'], 'POSTURE_UNCONFIRMED')
        self.assertIn('stop', self.client.calls)

    def test_unknown_action_follows_bundle_convention(self):
        self.assertIsNone(self.card.dispatch('not_an_action', {}))

    def test_completion_delivery_error_retained_without_blind_retries(self):
        job = {'id': 'test', 'result': {'status': 'completed'}}
        with mock.patch('loco_control.urllib.request.urlopen', side_effect=TimeoutError('unknown delivery')) as post:
            loco_control.ConfirmedLocoPlugin._notify(self.card, job)
        self.assertEqual(post.call_count, 1)
        self.assertEqual(job['result']['completion_delivery'], 'failed')

    def test_completion_payload_and_delivery_status(self):
        job = {'id': 'test', 'result': {'status': 'cancelled', 'ok': False}}
        response = mock.MagicMock()
        response.__enter__.return_value.read.return_value = b'{"ok":true,"matched":true}'
        with mock.patch('loco_control.urllib.request.urlopen', return_value=response) as post:
            loco_control.ConfirmedLocoPlugin._notify(self.card, job)
        payload = json.loads(post.call_args.args[0].data)
        self.assertEqual(payload['status'], 'cancelled')
        self.assertEqual(payload['action_id'], 'test')
        self.assertEqual(payload['tool'], 'loco_confirmed')
        self.assertEqual(job['result']['completion_delivery'], 'delivered')

    def test_bundle_preserves_original_and_adds_separate_card(self):
        path = Path(loco_control.__file__).with_name('main.py')
        spec = importlib.util.spec_from_file_location('go1_loco_test_main', path)
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {'yaml': types.ModuleType('yaml')}):
            spec.loader.exec_module(module)
        bundle = module.Go1Bundle({'plugins': {'loco': {'enabled': True},
                                  'loco_confirmed': {'enabled': True}}}, 'go1', None, self.client)
        tools = {t['name']: t for t in bundle.get_all_tools()}
        self.assertEqual(set(tools), {'loco', 'loco_confirmed'})
        self.assertNotIn('stop_move', tools['loco']['inputSchema']['properties']['action']['enum'])
        self.assertNotIn('x-hooks', tools['loco_confirmed']['inputSchema'])
        self.assertEqual(bundle.dispatch('loco_confirmed', {'action': 'start'}), {'state': 'ready'})
        self.assertEqual(bundle.dispatch('loco_confirmed', {'action': 'stop'}), {'state': 'idle'})
        self.assertEqual(bundle.dispatch('loco_confirmed', {'action': 'stop_move'})['code'], 'CONTROL_DISABLED')
        self.assertEqual(bundle.dispatch('loco', {'action': 'start'}), {'state': 'ready'})
        self.assertEqual(self.client.calls, [])

    def test_new_card_disabled_by_default_without_affecting_original(self):
        card = loco_control.make_loco_confirmed({}, '', None, self.client)
        for action in ['move', 'stop_move'] + list(loco_control.POSTURES):
            self.assertEqual(card.dispatch(action, {'vx': .1})['code'], 'CONTROL_DISABLED')
        card.start()
        card.stop()
        self.assertEqual(self.client.calls, [])
        self.assertFalse(card.dispatch('status', {})['control_enabled'])


class ProxyTests(unittest.TestCase):
    def setUp(self):
        self.proxy = sdk_proxy.SdkProxy.__new__(sdk_proxy.SdkProxy)
        self.proxy._epoch = multiprocessing.Value('Q', 0)
        self.proxy._stop_signal = threading.Event()
        self.proxy._lock = threading.Lock()
        self.proxy._stopped = False
        self.proxy._proc = types.SimpleNamespace(is_alive=lambda: True)
        self.proxy._cmd_q = queue.Queue()
        self.proxy._result_q = queue.Queue()

    def test_stop_does_not_wait_for_rpc_lock(self):
        with self.proxy._lock:
            result = self.proxy.request_stop()
        self.assertFalse(result['stop_confirmed'])
        self.assertTrue(self.proxy._stop_signal.is_set())
        self.assertEqual(self.proxy.control_epoch, 1)

    def test_late_response_not_used_for_next_call(self):
        def worker():
            req = self.proxy._cmd_q.get(timeout=1)
            self.proxy._result_q.put({'id': 'old-request', 'result': 'wrong'})
            self.proxy._result_q.put({'id': req['id'], 'result': 'correct'})
        thread = threading.Thread(target=worker)
        thread.start()
        self.assertEqual(self.proxy._call('snapshot'), 'correct')
        thread.join()

    def test_rpc_error_propagates(self):
        def worker():
            req = self.proxy._cmd_q.get(timeout=1)
            self.proxy._result_q.put({'id': req['id'], 'error': 'failure', 'code': 'TEST_ERROR'})
        thread = threading.Thread(target=worker)
        thread.start()
        with self.assertRaises(sdk_proxy.SdkError) as error:
            self.proxy._call('move')
        self.assertEqual(error.exception.code, 'TEST_ERROR')
        thread.join()

    def test_worker_rejects_expired_and_pre_stop_requests(self):
        client = mock.Mock(available=True)
        req = dict(cmd='move', deadline=time.monotonic() - 1, epoch=0)
        with self.assertRaises(sdk_proxy.SdkError) as error:
            sdk_proxy._execute(client, req, self.proxy._stop_signal, self.proxy._epoch)
        self.assertEqual(error.exception.code, 'SDK_TIMEOUT')
        req['deadline'] = time.monotonic() + 1
        self.proxy.request_stop()
        with self.assertRaises(sdk_proxy.SdkError) as error:
            sdk_proxy._execute(client, req, self.proxy._stop_signal, self.proxy._epoch)
        self.assertEqual(error.exception.code, 'CANCELLED')
        client.move.assert_not_called()

    def test_owner_and_latch_prevent_other_card_overwrite(self):
        client = mock.Mock(available=True)
        client._control_owner = 'loco'
        req = dict(cmd='move', deadline=time.monotonic() + 1, epoch=0, owner=None)
        with self.assertRaises(sdk_proxy.SdkError) as error:
            sdk_proxy._execute(client, req, self.proxy._stop_signal, self.proxy._epoch)
        self.assertEqual(error.exception.code, 'RESOURCE_BUSY')
        self.proxy._stop_signal.set()
        with self.assertRaises(sdk_proxy.SdkError) as error:
            sdk_proxy._execute(client, req, self.proxy._stop_signal, self.proxy._epoch)
        self.assertEqual(error.exception.code, 'STOP_LATCHED')

    def test_new_card_cannot_claim_while_old_card_is_moving(self):
        client = mock.Mock(available=True)
        client._control_owner = None
        client.snapshot.return_value = {'commanded_motion': {'active': True}}
        req = dict(cmd='claim_control', deadline=time.monotonic() + 1, epoch=0, owner='new-card')
        with self.assertRaises(sdk_proxy.SdkError) as error:
            sdk_proxy._execute(client, req, self.proxy._stop_signal, self.proxy._epoch)
        self.assertEqual(error.exception.code, 'RESOURCE_BUSY')
        client.stop_move.assert_not_called()


class SdkTests(unittest.TestCase):
    def setUp(self):
        with mock.patch.object(go1_sdk_client.Go1HighSdkClient, '_init_sdk'):
            self.client = go1_sdk_client.Go1HighSdkClient()
        self.client.available = True
        self.client._cmd = types.SimpleNamespace()

    def test_stop_latch_repeatedly_clears_all_targets(self):
        self.client.move(.1, .1, .2, 1)
        self.client._stop_signal.set()
        for _ in range(3):
            self.client._compose_cmd()
            self.assertEqual(self.client._cmd.velocity, [0., 0.])
            self.assertEqual(self.client._cmd.yawSpeed, 0.)
            self.assertEqual(self.client._cmd.mode, 0)
            self.assertIsNone(self.client._move_cmd)
            self.assertIsNone(self.client._posture)

    def test_gait_and_absolute_deadline(self):
        self.client._desired_gait = 2
        self.client._cmd.bodyHeight = -.1
        self.client._cmd.euler = [.2, .2, .2]
        self.client.move(.1, 0., 0., 1, until=time.monotonic() + .01)
        self.client._compose_cmd()
        self.assertEqual(self.client._cmd.gaitType, 1)
        self.assertEqual(self.client._cmd.bodyHeight, 0.)
        self.assertEqual(self.client._cmd.euler, [0., 0., 0.])
        time.sleep(.02)
        self.client._compose_cmd()
        self.assertEqual(self.client._cmd.velocity, [0., 0.])

    def test_only_new_error_free_packets_refresh_state(self):
        stats = types.SimpleNamespace(RecvCount=1, RecvCRCError=0, FlagError=0)
        self.client._udp = types.SimpleNamespace(udpState=stats)
        self.client._parse_state = mock.Mock()
        self.client._accept_received_state()
        self.client._accept_received_state()
        self.assertEqual(self.client._parse_state.call_count, 1)
        stats.RecvCount, stats.RecvCRCError = 2, 1
        self.client._accept_received_state()
        self.assertEqual(self.client._parse_state.call_count, 1)
        stats.RecvCount = 3
        self.client._accept_received_state()
        self.assertEqual(self.client._parse_state.call_count, 2)

    def test_aged_snapshot_becomes_stale(self):
        self.client._snapshot = {'fresh': True}
        self.client._snapshot_received_at = time.monotonic() - 1
        self.assertFalse(self.client.snapshot()['fresh'])

    def test_receipt_timestamp_survives_snapshot_reads(self):
        self.client._parse_state(types.SimpleNamespace())
        first = self.client.snapshot()
        self.assertTrue(first['fresh'])
        self.assertEqual(first['received_monotonic_s'], self.client._snapshot_received_at)
        self.assertEqual(self.client.snapshot()['received_monotonic_s'], first['received_monotonic_s'])

    def test_send_positive_bytes_succeeds_zero_and_negative_fail(self):
        self.client._udp = mock.Mock()
        self.client._udp.SetSend.return_value = 0
        self.client._udp.Send.return_value = 129
        self.client._send_cmd()
        for value in (0, -1):
            self.client._udp.Send.return_value = value
            with self.assertRaises(RuntimeError):
                self.client._send_cmd()


if __name__ == '__main__':
    unittest.main()
