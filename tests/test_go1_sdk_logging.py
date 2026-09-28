"""Offline logging regressions; fake SDK only, including a real spawn child."""
import multiprocessing
from pathlib import Path
import sys
import time
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'unitree/go1'))
import go1_sdk_client
import sdk_proxy


def _spawn_worker_probe(cmd_q, result_q, stop_signal, epoch):
    # Replace the binding before entering the production worker. No socket or
    # real robot_interface import is possible, even on an SDK-equipped host.
    from common import logsafe
    original_install = logsafe.install
    installs = []

    def install(**kwargs):
        installs.append(kwargs)
        original_install(**kwargs)

    class FakeUDP:
        def __init__(self, *args):
            result_q.put({'probe': True, 'installs': installs[:],
                          'stdout_safe': isinstance(sys.stdout, logsafe.LineAtomicStream),
                          'stderr_safe': isinstance(sys.stderr, logsafe.LineAtomicStream)})
            self.udpState = types.SimpleNamespace(RecvCount=0, RecvCRCError=0,
                                                  FlagError=0, RecvLoseError=0)

        def InitCmdData(self, cmd):
            pass

        def Recv(self):
            self.udpState.RecvCount += 1

        def GetRecv(self, state):
            state.mode = 'malformed'

        def SetSend(self, cmd):
            return 0

        def Send(self):
            return 0  # Disconnect: exercise real error counting + throttling.

    binding = types.SimpleNamespace(UDP=FakeUDP, HighCmd=types.SimpleNamespace,
                                    HighState=types.SimpleNamespace)
    with mock.patch.dict(sys.modules, {'robot_interface': binding}), \
            mock.patch.object(logsafe, 'install', side_effect=install):
        sdk_proxy._sdk_worker(cmd_q, result_q, '', 'unused', 0, 0, stop_signal, epoch)


class SdkLoggingTests(unittest.TestCase):
    def setUp(self):
        with mock.patch.object(go1_sdk_client.Go1HighSdkClient, '_init_sdk'):
            self.client = go1_sdk_client.Go1HighSdkClient()

    def test_continuous_failure_is_sampled_and_recovery_is_reported(self):
        with mock.patch.object(go1_sdk_client.time, 'monotonic', return_value=100.) as clock, \
                mock.patch('builtins.print') as output:
            for _ in range(2500):
                self.client._log_health('loop', RuntimeError('disconnected'))
            self.assertEqual(output.call_count, 1)
            clock.return_value = 105.
            self.client._log_health('loop', RuntimeError('still disconnected'))
            self.assertEqual(output.call_count, 2)
            self.assertIn('2501 since recovery', output.call_args.args[0])
            clock.return_value = 106.
            self.client._log_health('loop')
            self.assertEqual(output.call_count, 2)
            clock.return_value = 110.
            self.client._log_health('loop')
            self.assertIn('recovered after 2501 errors', output.call_args.args[0])
            self.client._log_health('loop')
            self.assertEqual(output.call_count, 3)

    def test_flapping_and_different_messages_cannot_bypass_rate_limit(self):
        with mock.patch.object(go1_sdk_client.time, 'monotonic', return_value=100.) as clock, \
                mock.patch('builtins.print') as output:
            for idx in range(2000):
                clock.return_value = 100. + idx * .002
                self.client._log_health('loop', RuntimeError(str(idx)))
                self.client._log_health('loop')
            self.assertEqual(output.call_count, 1)
            clock.return_value = 105.
            self.client._log_health('loop')
            self.assertEqual(output.call_count, 2)
            self.client._log_health('loop', RuntimeError('new outage'))
            self.assertEqual(output.call_count, 2)

    def test_loop_failure_keeps_diagnostics_and_stop_latch(self):
        self.client._udp = mock.Mock()
        self.client._udp.SetSend.return_value = 0
        self.client._udp.Send.return_value = 0
        self.client._control_owner = 'test-owner'
        self.client._running = True
        ticks = 0

        def tick(_period):
            nonlocal ticks
            ticks += 1
            if ticks == 1000:
                self.client._running = False

        with mock.patch.object(self.client, '_accept_received_state'), \
                mock.patch.object(self.client, '_compose_cmd'), \
                mock.patch.object(go1_sdk_client.time, 'monotonic', return_value=100.), \
                mock.patch.object(go1_sdk_client.time, 'sleep', side_effect=tick), \
                mock.patch('builtins.print') as output:
            self.client._loop()
        self.assertEqual(output.call_count, 1)
        self.assertEqual(self.client.diagnostics()['send_error'], 1000)
        self.assertEqual(self.client.diagnostics()['total_count'], 1000)
        self.assertEqual(self.client.diagnostics()['send_count'], 0)
        self.assertFalse(self.client.diagnostics()['accessible'])
        self.assertTrue(self.client._stop_signal.is_set())
        self.assertEqual(self.client._last_send_at, 0.)

    def test_parse_errors_are_sampled_without_refreshing_telemetry(self):
        with mock.patch.object(go1_sdk_client.time, 'monotonic', return_value=100.) as clock, \
                mock.patch('builtins.print') as output:
            self.client._parse_state(types.SimpleNamespace())
            for _ in range(1000):
                self.client._parse_state(types.SimpleNamespace(mode='bad'))
            self.assertEqual(output.call_count, 1)
            clock.return_value = 106.
            self.client._parse_state(types.SimpleNamespace(mode='bad'))
            self.assertEqual(output.call_count, 2)
            self.assertEqual(self.client._snapshot_received_at, 100.)
            self.assertFalse(self.client.snapshot()['fresh'])
            # Independent rate limit: parse failures cannot hide loop errors.
            self.client._log_health('loop', RuntimeError('send failed'))
            self.assertEqual(output.call_count, 3)
            clock.return_value = 111.
            self.client._parse_state(types.SimpleNamespace())
            self.assertIn('parse_state recovered after 1001 errors', output.call_args.args[0])
            self.assertTrue(self.client.snapshot()['fresh'])

    def test_real_spawn_worker_installs_logsafe_before_sdk_initialization(self):
        ctx = multiprocessing.get_context('spawn')
        cmd_q, result_q = ctx.Queue(), ctx.Queue()
        stop_signal, epoch = ctx.Event(), ctx.Value('Q', 0)
        proc = ctx.Process(target=_spawn_worker_probe,
                           args=(cmd_q, result_q, stop_signal, epoch))
        proc.start()
        try:
            probe = result_q.get(timeout=10)
            self.assertTrue(probe['probe'])
            self.assertEqual(probe['installs'], [{'check_fd': False}])
            self.assertTrue(probe['stdout_safe'])
            self.assertTrue(probe['stderr_safe'])
            self.assertTrue(result_q.get(timeout=5)['available'])
            deadline = time.monotonic() + 5
            while True:
                cmd_q.put({'id': 'diag', 'cmd': 'diagnostics', 'deadline': deadline})
                result = result_q.get(timeout=5)['result']
                if result['send_error'] >= 10:
                    break
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.02)
            self.assertEqual(result['send_count'], 0)
            self.assertFalse(result['accessible'])
            cmd_q.put(None)
            proc.join(timeout=5)
            self.assertEqual(proc.exitcode, 0)
            self.assertTrue(stop_signal.is_set())
        finally:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=5)
            cmd_q.close()
            result_q.close()


if __name__ == '__main__':
    unittest.main()
