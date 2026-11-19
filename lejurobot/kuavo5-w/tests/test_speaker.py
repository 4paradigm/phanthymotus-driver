import base64
import importlib.util
import json
import struct
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from uuid import uuid4
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import speaker
spec = importlib.util.spec_from_file_location('speaker_relay', ROOT / 'host/speaker_relay.py')
relay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay)


def until(predicate):
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.005)
    return False


class Bridge:
    def __init__(self, backend=None):
        self.backend = backend or types.SimpleNamespace(volume=30, request_id=None)
        self.connection = (True, 1)
        self.motion_ready = True
        self.published = []
        self.calls = []
        self.session = None
        self.closed = False
        self.fail_stop = False

    @property
    def volume(self):
        return self.backend.volume

    @volume.setter
    def volume(self, value):
        self.backend.volume = value

    @property
    def volume_request_id(self):
        return self.backend.request_id

    @volume_request_id.setter
    def volume_request_id(self, value):
        self.backend.request_id = value

    def start(self):
        pass

    def close(self):
        self.closed = True

    def advertise(self, *args):
        pass

    def publish(self, topic, kind, message):
        packet = json.loads(message['data'])
        if topic.endswith('/volume'):
            self.volume = packet['volume']
            self.volume_request_id = packet['request_id']
        else:
            self.published.append(packet)

    def call_service(self, path, kind, args, timeout):
        action = path.rsplit('/', 1)[1]
        self.calls.append(action)
        if action == 'start':
            self.session = uuid4().hex
        if action == 'stop':
            if self.fail_stop:
                raise ConnectionError('test stop failure')
            self.session = None
        return {'success': True, 'message': json.dumps({'session': self.session, 'volume_percent': self.volume, 'volume_request_id': self.volume_request_id})}


class SpeakerTests(unittest.TestCase):
    def setUp(self):
        qos = types.SimpleNamespace(QoSProfile=lambda **kw: kw,
            ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1),
            HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),
            DurabilityPolicy=types.SimpleNamespace(VOLATILE=1))
        self.modules = patch.dict(sys.modules, {'audio_msgs.msg': types.SimpleNamespace(AudioChunk=object),
                                               'rclpy.qos': qos})
        self.modules.start()
        self.node = Mock()
        self.bridges = []
        self.backend = types.SimpleNamespace(volume=30, request_id=None)
        def factory():
            bridge = Bridge(self.backend)
            self.bridges.append(bridge)
            return bridge
        self.player = speaker.Speaker(self.node, factory)

    def tearDown(self):
        self.player.close()
        self.modules.stop()

    def start(self):
        result = self.player.dispatch('start', {'input_topic': '/test/audio'})
        self.assertEqual(result['state'], 'ready', result)
        return self.bridges[-1]

    def chunk(self, pcm=b'\0\0' * 160, fmt='audio/pcm-16k', session=None):
        self.player._chunk(types.SimpleNamespace(data=pcm, format=fmt), session or self.player.session)

    def test_schema_matches_stream_contract_without_interrupt(self):
        tool = self.player.tool()
        self.assertEqual(tool['topic_in'], [{'format': 'audio/pcm-16k'}])
        self.assertNotIn('x-hooks', tool['inputSchema'])
        self.assertNotIn('interrupt', tool['inputSchema']['properties']['action']['enum'])
        self.assertNotIn('x-completion', tool['inputSchema'])

    def test_stream_start_stop_and_idempotent_start(self):
        bridge = self.start()
        self.start()
        self.assertEqual(len(self.bridges), 1)
        self.chunk()
        self.assertTrue(until(lambda: any(p['pcm'] for p in bridge.published)))
        self.assertTrue(all(p['session'] == bridge.session for p in bridge.published))
        self.assertEqual(self.player.dispatch('stop', {})['state'], 'idle')
        self.assertTrue(bridge.closed)
        self.assertEqual(bridge.calls[-1], 'stop')
        self.node.destroy_subscription.assert_called_once()

    def test_invalid_topic_does_not_create_bridge(self):
        for topic in ('', 'relative', '/bad//topic', '/bad;topic'):
            self.assertEqual(self.player.dispatch('start', {'input_topic': topic})['state'], 'error')
        self.assertFalse(self.bridges)

    def test_interrupt_is_not_supported_or_hooked(self):
        self.assertEqual(self.player.dispatch('interrupt', {})['state'], 'error')
        self.assertFalse(self.bridges)

    def test_eof_is_ignored_and_restarted_sessions_reject_old_callbacks(self):
        bridge = self.start()
        old_callback = self.node.create_subscription.call_args.args[2]
        self.player.dispatch('stop', {})
        new = self.start()
        old_callback(types.SimpleNamespace(data=b'00', format='audio/pcm-16k'))
        self.chunk(speaker.EOF)
        self.assertTrue(self.player.buffer.empty())
        self.chunk()
        self.assertTrue(until(lambda: any(p['pcm'] for p in new.published)))
        self.assertTrue(bridge.closed)
        self.assertFalse(any(base64.b64decode(p['pcm']) == speaker.EOF for p in new.published))

    def test_volume_get_set_without_starting_playback(self):
        self.assertEqual(self.player.dispatch('get_volume', {})['volume'], 30)
        result = self.player.dispatch('set_volume', {'volume': 63})
        self.assertEqual(result, {'state': 'idle', 'volume': 63, 'volume_scope': 'this_pcm_stream'})
        self.assertTrue(self.bridges[-1].closed)
        self.assertIsNone(self.bridges[-1].session)

    def test_active_volume_change_and_invalid_values_preserve_session(self):
        bridge = self.start()
        session = bridge.session
        self.assertEqual(self.player.dispatch('set_volume', {'volume': 0})['volume'], 0)
        for value in (True, -1, 101, 1.5, '30'):
            self.assertEqual(self.player.dispatch('set_volume', {'volume': value})['state'], 'error')
        self.assertEqual(bridge.session, session)
        self.assertFalse(bridge.closed)
        self.assertEqual(self.player.dispatch('get_volume', {})['volume'], 0)

    def test_volume_connection_does_not_block_live_pcm(self):
        bridge = self.start()
        entered, release = threading.Event(), threading.Event()
        original = self.player._connect
        results = []
        def connect():
            entered.set()
            if not release.wait(2.):
                raise TimeoutError('test volume connection blocked')
            return original()
        with patch.object(self.player, '_connect', side_effect=connect):
            thread = threading.Thread(target=lambda: results.append(
                self.player.dispatch('set_volume', {'volume': 37})))
            thread.start()
            try:
                self.assertTrue(entered.wait(1.))
                self.chunk()
                self.assertTrue(until(lambda: any(p['pcm'] for p in bridge.published)))
                self.assertFalse(bridge.closed)
            finally:
                release.set()
                thread.join(2.)
        self.assertFalse(thread.is_alive())
        self.assertEqual(results[0]['volume'], 37)
        self.assertIs(self.player.client, bridge)

    def test_volume_requires_matching_remote_acknowledgement(self):
        clock = [0.]
        def now():
            clock[0] += .5
            return clock[0]
        with patch.object(speaker, 'time', types.SimpleNamespace(monotonic=now, sleep=lambda _: None)), \
             patch.object(self.player, '_call', return_value={'volume_percent': 90, 'volume_request_id': 'wrong'}):
            result = self.player.dispatch('set_volume', {'volume': 30})
        self.assertEqual(result['state'], 'error')
        self.assertIn('acknowledgement', result['error'])
        self.assertTrue(self.bridges[-1].closed)

    def test_volume_does_not_publish_before_ros_publisher_connections_are_ready(self):
        bridge = Bridge()
        bridge.motion_ready = False
        self.player.factory = lambda: bridge
        clock = [0.]
        def now():
            clock[0] += .5
            return clock[0]
        with patch.object(speaker, 'time', types.SimpleNamespace(monotonic=now, sleep=lambda _: None)), \
             patch.object(bridge, 'publish') as publish:
            result = self.player.dispatch('set_volume', {'volume': 30})
        self.assertEqual(result['state'], 'error')
        publish.assert_not_called()

    def test_bad_format_or_length_stops_audio(self):
        for data, fmt in ((b'0', 'audio/pcm-16k'), (b'00', 'mp3'), (b'00' * 16001, 'audio/pcm-16k')):
            self.start()
            self.chunk(data, fmt)
            self.assertTrue(self.player.stop_event.is_set())
            self.assertTrue(until(lambda: self.bridges[-1].session is None))
            self.player.close()

    def test_core_pcm_format_is_accepted(self):
        bridge = self.start()
        self.chunk(fmt='pcm_16k_16bit_mono')
        self.assertTrue(until(lambda: any(p['pcm'] for p in bridge.published)))

    def test_reconnect_fails_session_without_replay(self):
        bridge = self.start()
        bridge.connection = (True, 2)
        self.assertTrue(until(self.player.stop_event.is_set))
        self.assertEqual(self.player.dispatch('info', {})['state'], 'error')
        self.assertEqual(len(self.bridges), 1)

    def test_failed_stop_is_reported_as_unconfirmed(self):
        bridge = self.start()
        bridge.fail_stop = True
        result = self.player.dispatch('stop', {})
        self.assertEqual(result['state'], 'error')
        self.assertIn('unconfirmed', result['error'])
        self.assertTrue(bridge.closed)

    def test_buffer_overflow_cancels_instead_of_unbounded_growth(self):
        self.player.session = 'test'
        for i in range(101):
            self.chunk(b'00' * 1600)
        self.assertTrue(self.player.stop_event.is_set())
        self.assertLessEqual(self.player.buffer.qsize(), 100)

    def test_callback_does_not_wait_for_lifecycle_lock(self):
        self.player.session = 'test'
        with self.player.lock:
            thread = threading.Thread(target=lambda: self.chunk())
            thread.start()
            thread.join(timeout=.2)
            self.assertFalse(thread.is_alive())
        self.assertTrue(self.player.buffer.empty())

    def test_failed_subscription_stops_remote_session(self):
        self.node.create_subscription.side_effect = RuntimeError('test subscription failure')
        result = self.player.dispatch('start', {'input_topic': '/test/audio'})
        self.assertEqual(result['state'], 'error')
        self.assertIsNone(self.bridges[-1].session)
        self.assertTrue(self.bridges[-1].closed)

    def test_service_registration_race_retries_only_definite_absence(self):
        original = Bridge.call_service
        attempts = []
        def call(bridge, path, *args, **kwargs):
            if path.endswith('/start'):
                attempts.append(path)
                if len(attempts) == 1:
                    raise RuntimeError('Service /motus/kuavo5w/speaker/start does not exist')
            return original(bridge, path, *args, **kwargs)
        with patch.object(Bridge, 'call_service', call):
            self.start()
        self.assertEqual(len(attempts), 2)

    def test_uncertain_start_timeout_is_never_replayed(self):
        with patch.object(Bridge, 'call_service', side_effect=TimeoutError('service timed out')) as call:
            result = self.player.dispatch('start', {'input_topic': '/test/audio'})
        self.assertEqual(result['state'], 'error')
        self.assertEqual(call.call_count, 1)

    def test_missing_helper_repair_can_register_after_old_short_deadline(self):
        original = Bridge.call_service
        clock = [0.]
        def call(bridge, path, *args, **kwargs):
            if path.endswith('/start') and clock[0] < 3.5:
                raise RuntimeError('Service /motus/kuavo5w/speaker/start does not exist')
            return original(bridge, path, *args, **kwargs)
        def sleep(seconds):
            clock[0] += seconds
        with patch.object(speaker, 'time', types.SimpleNamespace(
                monotonic=lambda: clock[0], sleep=sleep)), \
             patch.object(Bridge, 'call_service', call):
            self.start()
        self.assertGreaterEqual(clock[0], 3.5)

    def test_chassis_watchdog_failure_does_not_report_ready(self):
        bridge = self.start()
        bridge.session = None
        self.assertTrue(until(self.player.stop_event.is_set))
        self.assertEqual(self.player.dispatch('info', {})['state'], 'error')


class SinkTests(unittest.TestCase):
    def setUp(self):
        self.sink = relay.PcmSink()
        self.sink.session = 'test'
        self.sink.last_seen = time.monotonic()

    def packet(self, pcm=b'\0\0', seq=0, session='test'):
        return json.dumps({'session': session, 'seq': seq, 'pcm': base64.b64encode(pcm).decode()})

    def test_volume_control_changes_only_pcm_gain_and_acknowledges_request(self):
        for volume, expected in ((0, (0, 0)), (50, (250, -250)), (100, (2000, -2000))):
            request_id = uuid4().hex
            self.sink.receive_volume(json.dumps({'volume': volume, 'request_id': request_id}))
            self.assertEqual(self.sink.status()['volume_request_id'], request_id)
            self.assertEqual(self.sink.status()['volume_percent'], volume)
            self.assertEqual(struct.unpack('<hh', self.sink._gain(struct.pack('<hh', 2000, -2000))), expected)

    def test_invalid_volume_messages_cannot_mutate_gain(self):
        for volume in (True, '50', -1, 101, .5, None):
            self.sink.receive_volume(json.dumps({'volume': volume, 'request_id': uuid4().hex}))
        for raw in ('[]', '{}', 'invalid', 'x' * 201):
            self.sink.receive_volume(raw)
        self.assertEqual(self.sink.volume, 30)
        self.assertIsNone(self.sink.volume_request_id)

    def test_old_session_and_duplicate_packets_do_not_extend_lease(self):
        self.sink.receive(self.packet())
        seen = self.sink.last_seen
        self.sink.receive(self.packet())
        self.sink.receive(self.packet(seq=10, session='old'))
        self.assertEqual(self.sink.last_seen, seen)
        self.assertEqual(self.sink.queued_bytes, 2)

    def test_malformed_packet_cannot_renew_lease(self):
        seen = self.sink.last_seen
        for raw in ('{}', '[]', 'not json', self.packet(b'0'), self.packet(b'00' * 1601),
                    json.dumps({'session': 'test', 'seq': 0, 'pcm': '!'})):
            self.sink.receive(raw)
        self.assertEqual(self.sink.last_seen, seen)
        self.assertEqual(self.sink.queued_bytes, 0)

    def test_input_loss_clears_and_invalidates_session(self):
        self.sink.receive(self.packet())
        self.sink.last_seen -= 2
        self.sink.tick()
        self.assertIsNone(self.sink.session)
        self.assertEqual(self.sink.queued_bytes, 0)
        self.assertIn('lease expired', self.sink.error)

    def test_playback_uses_verified_sink_and_per_stream_volume(self):
        process = Mock()
        process.poll.return_value = None
        process.stdin.fileno.return_value = 23
        self.sink.receive(self.packet())
        with patch.object(relay.subprocess, 'Popen', return_value=process) as launch, \
             patch.object(relay.os, 'set_blocking'), patch.object(relay.os, 'write', return_value=2):
            self.sink.tick()
        command = launch.call_args.args[0]
        self.assertIn('--device=' + relay.SINK, command)
        self.assertIn('--rate=16000', command)
        self.assertIn('--channels=1', command)
        self.assertEqual(self.sink.bytes_written, 2)
        self.sink.stop()
        process.terminate.assert_called_once()

    def test_backpressure_does_not_block_stop_or_grow_queue(self):
        process = Mock()
        process.poll.return_value = None
        self.sink.process = process
        self.sink.receive(self.packet())
        with patch.object(relay.os, 'write', side_effect=BlockingIOError):
            self.sink.tick()
        self.assertEqual(self.sink.queued_bytes, 2)
        self.sink.stop()
        self.assertEqual(self.sink.queued_bytes, 0)
        self.assertIsNone(self.sink.session)

    def test_start_checks_sink_and_replaces_session(self):
        original = self.sink.session
        with patch.object(relay.shutil, 'which', return_value='/usr/bin/tool'), \
             patch.object(relay.subprocess, 'run', return_value=types.SimpleNamespace(stdout='0\t' + relay.SINK)):
            result = self.sink.start()
        self.assertNotEqual(result['session'], original)
        self.assertEqual(len(result['session']), 32)
        with patch.object(relay.shutil, 'which', return_value='/usr/bin/tool'), \
             patch.object(relay.subprocess, 'run', return_value=types.SimpleNamespace(stdout='0\tauto_null')):
            with self.assertRaisesRegex(RuntimeError, 'unavailable'):
                self.sink.start()
        self.assertIsNone(self.sink.session)

    def test_player_exit_is_an_error(self):
        self.sink.process = Mock()
        self.sink.process.poll.return_value = 1
        self.sink.tick()
        self.assertIsNone(self.sink.session)
        self.assertIn('paplay exited', self.sink.error)

    def test_queue_overflow_stops_and_invalidates_the_session(self):
        for sequence in range(11):
            self.sink.receive(self.packet(b'00' * 1600, seq=sequence))
        self.assertIsNone(self.sink.session)
        self.assertEqual(self.sink.queued_bytes, 0)
        self.assertIn('overflow', self.sink.error)


if __name__ == '__main__':
    unittest.main()


class RegistrationWatchTests(unittest.TestCase):
    def test_master_restart_missing_entries_restarts_process_even_when_service_is_active(self):
        now=[0.]
        watch=relay.RegistrationWatch(lambda:now[0])
        master=Mock()
        master.getSystemState.return_value=[[],[],[]]
        watch.check(master,'/speaker')
        master.getSystemState.assert_not_called()
        for t in (10.,12.):
            now[0]=t;watch.check(master,'/speaker')
        now[0]=14.
        with self.assertRaisesRegex(RuntimeError,'registration lost'):
            watch.check(master,'/speaker')

    def test_recovery_resets_failures_and_other_owner_does_not_count(self):
        now=[10.]
        watch=relay.RegistrationWatch(lambda:0.)
        master=Mock()
        master.getSystemState.side_effect=ConnectionError('master unavailable')
        watch.clock=lambda:now[0]
        watch.check(master,'/speaker')
        master.getSystemState.side_effect=None
        master.getSystemState.return_value=[[],[],[(relay.ROOT+'/'+n,['/speaker']) for n in ('start','stop','status')]]
        now[0]=12.;watch.clock=lambda:now[0];watch.check(master,'/speaker')
        self.assertEqual(watch.failures,0)
        master.getSystemState.return_value=[[],[],[(relay.ROOT+'/'+n,['/other'])for n in ('start','stop','status')]]
        for t in (14.,16.):now[0]=t;watch.check(master,'/speaker')
        now[0]=18.
        with self.assertRaises(RuntimeError):watch.check(master,'/speaker')
