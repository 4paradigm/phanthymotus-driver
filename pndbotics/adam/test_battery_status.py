"""Battery mapping, disconnect/timeout handling and receiver lifecycle tests."""
import json
import queue
import time
import unittest
from unittest import mock

import websocket
from battery_status import BatteryStatusReceiver, parse_sample


SAMPLE = dict(capacity=91, percentage=100, voltage=4535, current=683,
              cycle_count=13, mos_temp_dc=53.7, t1_temp_dc=51.0,
              t2_temp_dc=51.6, pstatus="Normal")


class FakeSocket:
    def __init__(self):
        self.messages = queue.Queue()
        self.closed = False

    def settimeout(self, timeout):
        pass

    def recv(self):
        try:
            item = self.messages.get(timeout=.02)
        except queue.Empty:
            raise websocket.WebSocketTimeoutException()
        if isinstance(item, Exception):
            raise item
        return item

    def abort(self):
        self.messages.put(ConnectionError("aborted"))

    def close(self, **kwargs):
        self.closed = True


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError("receiver condition timed out")


class MappingTests(unittest.TestCase):
    def test_vendor_capacity_not_percentage_and_extended_fields(self):
        self.assertEqual(parse_sample(json.dumps(SAMPLE)), {
            "percentage": 91, "cycle_count": 13, "mos_temperature_c": 53.7,
            "t1_temperature_c": 51.0, "t2_temperature_c": 51.6,
            "protection_status": "Normal"})

    def test_invalid_capacity_is_not_a_sample(self):
        for capacity in (True, None, "91", -1, 101, float('nan'), float('inf')):
            with self.subTest(capacity=capacity), self.assertRaises(ValueError):
                parse_sample(json.dumps(dict(SAMPLE, capacity=capacity)))

    def test_optional_invalid_fields_are_null_not_invented(self):
        d = parse_sample(json.dumps(dict(SAMPLE, mos_temp_dc=float('nan'),
                                         cycle_count=1.5, pstatus=42)))
        self.assertIsNone(d['mos_temperature_c'])
        self.assertIsNone(d['cycle_count'])
        self.assertIsNone(d['protection_status'])

    def test_missing_optional_fields_do_not_reuse_old_fields(self):
        d = parse_sample('{"capacity": 90}')
        self.assertIsNone(d['t1_temperature_c'])
        self.assertIsNone(d['cycle_count'])

    def test_invalid_json_and_non_objects(self):
        for raw in ('{', '[]', 'null', '{"percentage":100}'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                parse_sample(raw)


class CacheTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.receiver = BatteryStatusReceiver('http://localhost:8626',
            monotonic=lambda:self.now, wall_time=lambda:1000, stale_after_sec=10)
        self.receiver._connected = True

    def test_fresh_stale_and_recovery(self):
        r = self.receiver
        r._accept(json.dumps(SAMPLE))
        self.assertEqual(r.snapshot()['percentage'], 91)
        self.now += 10.1
        d = r.snapshot()
        self.assertFalse(d['pac_fresh'])
        self.assertFalse(d['percentage_available'])
        self.assertIsNone(d['percentage'])
        self.assertIsNone(d['mos_temperature_c'])
        self.assertEqual(d['pac_last_received_at_ms'], 1000000)
        r._accept(json.dumps(dict(SAMPLE, capacity=90)))
        self.assertTrue(r.snapshot()['pac_fresh'])
        self.assertEqual(r.snapshot()['percentage'], 90)

    def test_disconnected_even_recent_data_is_unavailable(self):
        self.receiver._accept(json.dumps(SAMPLE))
        self.receiver._connected = False
        self.assertFalse(self.receiver.snapshot()['percentage_available'])
        self.assertIsNone(self.receiver.snapshot()['percentage'])

    def test_malformed_does_not_refresh_timestamp(self):
        r = self.receiver
        r._accept(json.dumps(SAMPLE))
        self.now += 11
        with self.assertRaises(ValueError): r._accept('{')
        self.assertFalse(r.snapshot()['pac_fresh'])

    def test_waiting_does_not_claim_zero_soc(self):
        self.assertIsNone(self.receiver.snapshot()['percentage'])
        self.assertFalse(self.receiver.snapshot()['pac_fresh'])


class WorkerTests(unittest.TestCase):
    def test_reconnect_malformed_and_idempotent_lifecycle(self):
        sockets = [FakeSocket(), FakeSocket()]
        connector = mock.Mock(side_effect=sockets)
        r = BatteryStatusReceiver('http://localhost:8626', connector=connector, reconnect_sec=.01)
        try:
            r.start(); r.start()
            sockets[0].messages.put(json.dumps(SAMPLE))
            wait_for(lambda:r.snapshot()['pac_fresh'])
            self.assertEqual(connector.call_count, 1)
            sockets[0].messages.put('{')
            wait_for(lambda:r.snapshot()['pac_last_error'] is not None)
            sockets[0].messages.put(ConnectionError('test disconnect'))
            wait_for(lambda:connector.call_count == 2)
            self.assertFalse(r.snapshot()['percentage_available'])
            sockets[1].messages.put(json.dumps(dict(SAMPLE, capacity=89)))
            wait_for(lambda:r.snapshot()['percentage'] == 89)
            self.assertIsNone(r.snapshot()['pac_last_error'])
        finally:
            r.stop(); r.stop()
        self.assertTrue(all(s.closed for s in sockets))
        self.assertFalse(r.snapshot()['pac_connected'])
        self.assertIsNone(r._thread)

    def test_silent_initial_connection_reconnects(self):
        sockets = [FakeSocket(), FakeSocket()]
        r = BatteryStatusReceiver('http://localhost:8626', connector=mock.Mock(side_effect=sockets),
                                  stale_after_sec=.05, reconnect_sec=.01)
        try:
            r.start()
            wait_for(lambda:sockets[0].closed)
            self.assertIsNone(r.snapshot()['percentage'])
        finally: r.stop()

    def test_connection_failure_retains_diagnostic_and_can_stop(self):
        r = BatteryStatusReceiver('http://localhost:8626',
                                  connector=mock.Mock(side_effect=OSError('offline')))
        try:
            r.start()
            wait_for(lambda:r.snapshot()['pac_last_error'] == 'offline')
            self.assertFalse(r.snapshot()['pac_fresh'])
        finally: r.stop()


if __name__ == '__main__': unittest.main()
