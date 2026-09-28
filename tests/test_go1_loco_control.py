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

    def test_stop_alias_and_interrupt_schema(self):
        for action in ('stop', 'stop_move'):
            result = self.card.dispatch(action, {})
            self.assertTrue(result['stop_confirmed'])
        schema = self.card.get_tool()['inputSchema']
        self.assertEqual(schema['x-hooks']['on_interrupt_motion']['action'], 'stop_move')
        self.assertIn('stop_move', schema['properties']['action']['enum'])

    def test_stop_unavailable_still_latches(self):
        self.client.fresh = False
        self.assertEqual(self.card.dispatch('stop', {})['code'], 'STOP_UNCONFIRMED')
        self.assertEqual(self.client.calls[0], 'stop')

    def test_stop_still_moving_not_confirmed(self):
        self.client.stop_works = False
        self.client.velocity = [.2, 0.]
        self.assertFalse(self.card.dispatch('stop', {})['stop_confirmed'])

    def test_replayed_sample_cannot_confirm_stop(self):
        self.client.frozen = True
        self.assertFalse(self.card.dispatch('stop', {})['stop_confirmed'])

    def test_async_cancel_and_no_old_command_resurrection(self):
        accepted = self.card.dispatch('move', dict(vx=.1, duration=3))
        self.assertEqual(accepted['status'], 'accepted')
        self.assertFalse(accepted['executed'])
        self.assertEqual(self.move()['code'], 'RESOURCE_BUSY')
        self.assertTrue(self.card.dispatch('stop', {})['stop_confirmed'])
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
        self.assertEqual(self.card._notify.call_count, 5)

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
        self.assertEqual(bundle.dispatch('loco_confirmed', {'action': 'stop_move'})['code'], 'CONTROL_DISABLED')
        self.assertEqual(bundle.dispatch('loco', {'action': 'start'}), {'state': 'ready'})
        self.assertEqual(self.client.calls, [])

    def test_new_card_disabled_by_default_without_affecting_original(self):
        card = loco_control.make_loco_confirmed({}, '', None, self.client)
        for action in ['move', 'stop', 'stop_move'] + list(loco_control.POSTURES):
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
