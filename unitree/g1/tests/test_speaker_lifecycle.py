import importlib.util
import json
import queue
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock


G1_DIR = Path(__file__).resolve().parents[1]


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class _Publisher:
    def __init__(self, topic, qos):
        self.topic = topic
        self.qos = qos
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FailingPublisher(_Publisher):
    def __init__(self, topic, qos, fail_attempts):
        super().__init__(topic, qos)
        self.fail_attempts = set(fail_attempts)
        self.attempts = 0

    def publish(self, msg):
        self.attempts += 1
        if self.attempts in self.fail_attempts:
            raise RuntimeError('injected publish failure')
        super().publish(msg)


class _GatePublisher(_Publisher):
    def __init__(self, topic, qos):
        super().__init__(topic, qos)
        self.attempts = 0
        self.retry_entered = threading.Event()
        self.retry_release = threading.Event()

    def publish(self, msg):
        self.attempts += 1
        if self.attempts == 1:
            raise RuntimeError('injected first publish failure')
        if self.attempts == 2:
            self.retry_entered.set()
            self.retry_release.wait(timeout=1)
        super().publish(msg)


class _Timer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _Node:
    def __init__(self, name):
        self.name = name
        self.publishers = []
        self.timers = []
        self._logger = _Logger()

    def create_publisher(self, _msg_type, topic, _qos):
        publisher = _Publisher(topic, _qos)
        self.publishers.append(publisher)
        return publisher

    def destroy_publisher(self, publisher):
        if publisher in self.publishers:
            self.publishers.remove(publisher)

    def create_subscription(self, _msg_type, topic, callback, _qos):
        return types.SimpleNamespace(topic=topic, callback=callback)

    def destroy_subscription(self, _subscription):
        pass

    def create_timer(self, _period, callback):
        timer = _Timer(callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        if timer in self.timers:
            self.timers.remove(timer)

    def get_logger(self):
        return self._logger


class _Header:
    def __init__(self):
        self.frame_id = ''
        self.stamp = None


class _String:
    def __init__(self):
        self.data = ''


class _AudioChunk:
    def __init__(self):
        self.header = _Header()
        self.format = 'audio/pcm-16k'
        self.data = []


class _QoSProfile:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


def _package(name):
    module = types.ModuleType(name)
    module.__path__ = []
    return module


def _load_device_module():
    rclpy = _package('rclpy')
    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = _Node
    rclpy_qos = types.ModuleType('rclpy.qos')
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(
        BEST_EFFORT='best_effort', RELIABLE='reliable',
    )
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST='keep_last')
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(
        VOLATILE='volatile', TRANSIENT_LOCAL='transient_local',
    )

    std_msgs = _package('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.Header = _Header
    std_msgs_msg.String = _String
    audio_msgs = _package('audio_msgs')
    audio_msgs_msg = types.ModuleType('audio_msgs.msg')
    audio_msgs_msg.AudioChunk = _AudioChunk

    sdk = _package('unitree_sdk2py')
    sdk_g1 = _package('unitree_sdk2py.g1')
    sdk_audio = _package('unitree_sdk2py.g1.audio')
    sdk_client = types.ModuleType('unitree_sdk2py.g1.audio.g1_audio_client')
    sdk_client.AudioClient = object
    pointcloud = types.ModuleType('pointcloud_utils')
    pointcloud.gravity_align_inplace = lambda *args, **kwargs: None
    numpy = types.ModuleType('numpy')

    stubs = {
        'rclpy': rclpy,
        'rclpy.node': rclpy_node,
        'rclpy.qos': rclpy_qos,
        'std_msgs': std_msgs,
        'std_msgs.msg': std_msgs_msg,
        'audio_msgs': audio_msgs,
        'audio_msgs.msg': audio_msgs_msg,
        'unitree_sdk2py': sdk,
        'unitree_sdk2py.g1': sdk_g1,
        'unitree_sdk2py.g1.audio': sdk_audio,
        'unitree_sdk2py.g1.audio.g1_audio_client': sdk_client,
        'pointcloud_utils': pointcloud,
        'numpy': numpy,
    }
    spec = importlib.util.spec_from_file_location(
        'g1_device_under_test', G1_DIR / 'device.py',
    )
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


DEVICE = _load_device_module()


class _Clock:
    def __init__(self):
        self.value = 100.0

    def monotonic(self):
        return self.value

    def wall_time(self):
        return 1_780_000_000.0 + self.value

    def wait(self, seconds):
        self.value += seconds
        return False


class _AudioClient:
    def __init__(self):
        self.stream_calls = []
        self.stop_calls = []
        self.stream_send_timeouts = []
        self.stream_finish_timeouts = []
        self.stop_send_timeouts = []
        self.stop_finish_timeouts = []
        self.stream_code = 0
        self.stop_code = 0
        self.block_stream = False
        self.stream_entered = threading.Event()
        self.stream_release = threading.Event()

    def PlayStream(self, app_name, stream_id, pcm):
        self.stream_calls.append((app_name, stream_id, bytes(pcm)))
        self.stream_entered.set()
        if self.block_stream:
            self.stream_release.wait(timeout=1)
        return self.stream_code, {}

    def PlayStreamStart(self, app_name, stream_id, pcm, send_timeout=None):
        self.stream_calls.append((app_name, stream_id, bytes(pcm)))
        self.stream_send_timeouts.append(send_timeout)
        self.stream_entered.set()
        return 0, object()

    def PlayStreamFinish(self, _pending, timeout=None):
        self.stream_finish_timeouts.append(timeout)
        if self.block_stream:
            self.stream_release.wait(timeout=1)
        return self.stream_code, {}

    def PlayStop(self, app_name):
        self.stop_calls.append(app_name)
        self.stream_release.set()
        return self.stop_code

    def PlayStopStart(self, app_name, send_timeout=None):
        self.stop_calls.append(app_name)
        self.stop_send_timeouts.append(send_timeout)
        self.stream_release.set()
        return 0, object()

    def PlayStopFinish(self, _pending, timeout=None):
        self.stop_finish_timeouts.append(timeout)
        return self.stop_code


class _PlaybackStateMonitor:
    def __init__(self, result=None, *, available=True, ready=None):
        self.result = result or {
            'state': 'completed',
            'reason': '',
            'playing_seq': 1,
            'idle_seq': 2,
        }
        self.available = available
        self.ready = available if ready is None else ready
        self.sequence = 0
        self.submissions = []
        self.wait_calls = []
        self.forgotten = []
        self.ready_waits = []

    def checkpoint(self):
        checkpoint = self.sequence
        self.sequence += 1
        return checkpoint

    def note_submission(self, key, checkpoint):
        self.submissions.append((key, checkpoint))

    def wait_for_completion(self, key, *, timeout, cancelled):
        self.wait_calls.append((key, timeout))
        if cancelled():
            return {'state': 'interrupted', 'reason': 'interrupted'}
        return dict(self.result)

    def wait_until_ready(self, *, timeout, cancelled=None):
        self.ready_waits.append(timeout)
        if cancelled is not None and cancelled():
            return {'state': 'interrupted', 'reason': 'interrupted'}
        if not self.available:
            return {'state': 'error', 'reason': 'injected_unavailable'}
        if not self.ready:
            return {
                'state': 'timeout',
                'reason': 'play_state_publisher_timeout',
            }
        return {'state': 'ready', 'reason': ''}

    def forget(self, key):
        self.forgotten.append(key)

    def status(self):
        return {
            'available': self.available,
            'ready': self.ready,
            'matched_publishers': 1 if self.ready else 0,
            'topic': 'rt/audio_msg',
            'current_state': 0,
            'event_seq': self.sequence,
            'last_event_ts': 123.0,
            'connect_error': '' if self.available else 'injected_unavailable',
        }


class _BlockingPlaybackStateMonitor(_PlaybackStateMonitor):
    def __init__(self):
        super().__init__()
        self.wait_entered = threading.Event()
        self.release_wait = threading.Event()

    def wait_for_completion(self, key, *, timeout, cancelled):
        self.wait_calls.append((key, timeout))
        self.wait_entered.set()
        while not self.release_wait.wait(timeout=0.005):
            if cancelled():
                return {'state': 'interrupted', 'reason': 'interrupted'}
        if cancelled():
            return {'state': 'interrupted', 'reason': 'interrupted'}
        return dict(self.result)


class _BlockingReadyPlaybackStateMonitor(_PlaybackStateMonitor):
    def __init__(self):
        super().__init__(ready=False)
        self.ready_wait_entered = threading.Event()
        self.ready_wait_release = threading.Event()

    def wait_until_ready(self, *, timeout, cancelled=None):
        self.ready_waits.append(timeout)
        self.ready_wait_entered.set()
        while not self.ready_wait_release.wait(timeout=0.005):
            if cancelled is not None and cancelled():
                return {'state': 'interrupted', 'reason': 'interrupted'}
        if cancelled is not None and cancelled():
            return {'state': 'interrupted', 'reason': 'interrupted'}
        return {'state': 'ready', 'reason': ''}


class _StopFirstSdkLock:
    """Force interrupt's PlayStop to linearize before a pending PlayStream."""

    def __init__(self):
        self.play_waiting = threading.Event()
        self.allow_play = threading.Event()

    def __enter__(self):
        if threading.current_thread().name == 'late-play':
            self.play_waiting.set()
            self.allow_play.wait(timeout=1)
        return self

    def __exit__(self, *_args):
        if threading.current_thread().name != 'late-play':
            self.allow_play.set()


class _BlockingGetQueue(queue.Queue):
    def __init__(self):
        super().__init__()
        self.item_popped = threading.Event()
        self.release_item = threading.Event()
        self._blocked_once = False

    def get(self, block=True, timeout=None):
        item = super().get(block=block, timeout=timeout)
        if (block and not self._blocked_once
                and threading.current_thread() is not threading.main_thread()):
            self._blocked_once = True
            self.item_popped.set()
            self.release_item.wait(timeout=1)
        return item


class _EmptyRaceQueue(queue.Queue):
    def __init__(self):
        super().__init__()
        self.empty_observed = threading.Event()
        self.release_empty = threading.Event()
        self._armed = False

    def arm(self):
        self._armed = True

    def empty(self):
        result = super().empty()
        if (self._armed and result
                and threading.current_thread() is not threading.main_thread()):
            self._armed = False
            self.empty_observed.set()
            self.release_empty.wait(timeout=1)
        return result


class _GetWaitingQueue(queue.Queue):
    def __init__(self):
        super().__init__()
        self.get_waiting = threading.Event()

    def get(self, block=True, timeout=None):
        if block and self.empty():
            self.get_waiting.set()
        return super().get(block=block, timeout=timeout)


class _PreGetGateQueue(queue.Queue):
    def __init__(self):
        super().__init__()
        self.before_get = threading.Event()
        self.release_get = threading.Event()
        self._gated_once = False

    def get(self, block=True, timeout=None):
        if (block and not self._gated_once
                and threading.current_thread() is not threading.main_thread()):
            self._gated_once = True
            self.before_get.set()
            self.release_get.wait(timeout=1)
        return super().get(block=block, timeout=timeout)


class _BlockingStopClient(_AudioClient):
    def __init__(self):
        super().__init__()
        self.stop_entered = threading.Event()
        self.stop_release = threading.Event()
        self.block_stop = False

    def PlayStop(self, app_name):
        self.stop_calls.append(app_name)
        self.stop_entered.set()
        if self.block_stop:
            self.stop_release.wait(timeout=1)
        return self.stop_code

    def PlayStopStart(self, app_name, send_timeout=None):
        self.stop_calls.append(app_name)
        self.stop_send_timeouts.append(send_timeout)
        self.stream_release.set()
        self.stop_entered.set()
        return 0, object()

    def PlayStopFinish(self, _pending, timeout=None):
        if self.block_stop:
            self.stop_release.wait(timeout=1)
        return self.stop_code


class _DelayedFinishClient(_AudioClient):
    def __init__(self):
        super().__init__()
        self.finish_entered = threading.Event()
        self.finish_release = threading.Event()
        self.finish_calls = 0

    def PlayStreamFinish(self, _pending, timeout=None):
        self.finish_calls += 1
        if self.finish_calls == 1:
            self.finish_entered.set()
            self.finish_release.wait(timeout=2)
        return self.stream_code, {}


class _SequenceStreamClient(_AudioClient):
    def __init__(self, stream_codes):
        super().__init__()
        self.stream_codes = list(stream_codes)

    def PlayStreamFinish(self, _pending, timeout=None):
        if self.stream_codes:
            return self.stream_codes.pop(0), {}
        return 0, {}


class _StaleErrorClient(_AudioClient):
    def __init__(self):
        super().__init__()
        self.first_finish_entered = threading.Event()
        self.first_finish_release = threading.Event()
        self.finish_calls = 0

    def PlayStreamFinish(self, _pending, timeout=None):
        self.finish_calls += 1
        if self.finish_calls == 1:
            self.first_finish_entered.set()
            self.first_finish_release.wait(timeout=2)
            return 17, {}
        return 0, {}


class _StaleExceptionClient(_AudioClient):
    def __init__(self):
        super().__init__()
        self.first_finish_entered = threading.Event()
        self.first_finish_release = threading.Event()
        self.finish_calls = 0

    def PlayStreamFinish(self, _pending, timeout=None):
        self.finish_calls += 1
        if self.finish_calls == 1:
            self.first_finish_entered.set()
            self.first_finish_release.wait(timeout=2)
            raise RuntimeError('stale response failure')
        return 0, {}


def _chunk(utterance_id, pcm):
    msg = _AudioChunk()
    msg.header.frame_id = utterance_id
    msg.data = list(pcm)
    return msg


def _receipts(node):
    return [json.loads(msg.data) for msg in node._receipt_pub.messages]


class SpeakerLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.clock = _Clock()
        self.client = _AudioClient()
        self.node = DEVICE._SpeakerNode(
            self.client,
            monotonic=self.clock.monotonic,
            wall_time=self.clock.wall_time,
            waiter=self.clock.wait,
        )
        self.client.stop_calls.clear()  # Ignore stale-session cleanup in __init__.
        self.client.stop_send_timeouts.clear()
        self.client.stop_finish_timeouts.clear()
        self.client.stream_release.clear()
        self.node.start_play('/perception/tts')

    def _eof(self, utterance_id):
        return _chunk(utterance_id, self.node.AUDIO_EOF_MAGIC)

    def _join_drain(self, node=None):
        node = node or self.node
        thread = node._drain_thread
        self.assertIsNotNone(thread)
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive(), 'speaker drain thread did not finish')

    def _wait_for(self, predicate, message, timeout=1):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.001)
        self.fail(message)

    def test_eof_forces_short_utterance_drain(self):
        pcm = b'\x11\x22' * 1600
        self.node._on_chunk(_chunk('utt:short', pcm))
        self.assertEqual(self.client.stream_calls, [])

        self.node._on_chunk(self._eof('utt:short'))
        self._join_drain()

        self.assertEqual([call[2] for call in self.client.stream_calls], [pcm])
        self.assertNotIn(self.node.AUDIO_EOF_MAGIC,
                         [call[2] for call in self.client.stream_calls])
        self.assertEqual(self.node.state, 'ready')

    def test_completion_emitted_once_after_driver_drained(self):
        pcm = b'\x01\x02' * 1600
        self.node._on_chunk(_chunk('utt:one', pcm))
        self.node._on_chunk(self._eof('utt:one'))
        self._join_drain()

        events = [r for r in _receipts(self.node) if r['utterance_id'] == 'utt:one']
        self.assertEqual([r['state'] for r in events], ['started', 'completed'])
        terminal = events[-1]
        self.assertEqual(terminal['session_id'], self.node._session_id)
        self.assertEqual(terminal['completion_basis'], 'driver_drained_estimated')
        self.assertEqual(terminal['audio_bytes'], len(pcm))
        self.assertGreaterEqual(self.clock.value, 100.0 + len(pcm) / 32000)
        self.assertEqual(
            self.client.stream_send_timeouts, [self.node.RPC_SEND_TIMEOUT_SEC],
        )
        self.assertEqual(
            self.client.stream_finish_timeouts, [self.node.STREAM_RPC_TIMEOUT_SEC],
        )
        self.assertEqual(self.node._generation_ids[self.node._generation], set())
        self.assertEqual(self.node.interrupt()['cancelled_utterance_ids'], [])
        self.assertLessEqual(
            self.client.stop_send_timeouts[-1], self.node.RPC_SEND_TIMEOUT_SEC,
        )
        self.assertLessEqual(
            self.client.stop_finish_timeouts[-1], self.node.CONTROL_RPC_TIMEOUT_SEC,
        )

    def test_hardware_completion_waits_for_g1_play_state(self):
        monitor = _PlaybackStateMonitor()
        client = _AudioClient()
        node = DEVICE._SpeakerNode(
            client,
            monotonic=self.clock.monotonic,
            wall_time=self.clock.wall_time,
            waiter=self.clock.wait,
            playback_state_monitor=monitor,
            completion_mode='hardware_state',
        )
        client.stop_calls.clear()
        node.start_play('/perception/hardware-state')
        pcm = b'\x10\x20' * 1600

        node._on_chunk(_chunk('utt:hardware', pcm))
        node._on_chunk(_chunk('utt:hardware', node.AUDIO_EOF_MAGIC))
        self._join_drain(node)

        terminal = [
            receipt for receipt in _receipts(node)
            if receipt['utterance_id'] == 'utt:hardware'
            and receipt['state'] == 'completed'
        ][0]
        generation = node._generation
        self.assertEqual(
            monitor.submissions,
            [((generation, 'utt:hardware'), 0)],
        )
        self.assertEqual(
            monitor.wait_calls,
            [((generation, 'utt:hardware'), node._play_state_timeout_sec)],
        )
        self.assertEqual(terminal['completion_basis'], 'g1_play_state_observed')
        self.assertIn((generation, 'utt:hardware'), monitor.forgotten)

    def test_hardware_idle_gate_advances_only_after_stream_response(self):
        monitor = _PlaybackStateMonitor()
        client = _DelayedFinishClient()
        node = DEVICE._SpeakerNode(
            client,
            monotonic=self.clock.monotonic,
            wall_time=self.clock.wall_time,
            waiter=self.clock.wait,
            playback_state_monitor=monitor,
            completion_mode='hardware_state',
        )
        client.stop_calls.clear()
        node.start_play('/perception/hardware-response-boundary')

        node._on_chunk(_chunk(
            'utt:hardware-response-boundary', b'R' * 3200,
        ))
        node._on_chunk(_chunk(
            'utt:hardware-response-boundary', node.AUDIO_EOF_MAGIC,
        ))
        self.assertTrue(client.finish_entered.wait(timeout=1))

        self.assertEqual(monitor.submissions, [])
        client.finish_release.set()
        self._join_drain(node)

        generation = node._generation
        self.assertEqual(
            monitor.submissions,
            [((generation, 'utt:hardware-response-boundary'), 0)],
        )
        self.assertTrue(any(
            receipt['utterance_id'] == 'utt:hardware-response-boundary'
            and receipt['state'] == 'completed'
            for receipt in _receipts(node)
        ))

    def test_hardware_state_timeout_is_error_and_stops_player(self):
        monitor = _PlaybackStateMonitor({
            'state': 'timeout',
            'reason': 'play_state_idle_timeout',
        })
        client = _AudioClient()
        node = DEVICE._SpeakerNode(
            client,
            monotonic=self.clock.monotonic,
            wall_time=self.clock.wall_time,
            waiter=self.clock.wait,
            playback_state_monitor=monitor,
            completion_mode='hardware_state',
        )
        client.stop_calls.clear()
        node.start_play('/perception/hardware-timeout')

        node._on_chunk(_chunk('utt:hardware-timeout', b'T' * 3200))
        node._on_chunk(_chunk(
            'utt:hardware-timeout', node.AUDIO_EOF_MAGIC,
        ))
        self._join_drain(node)

        terminal = [
            receipt for receipt in _receipts(node)
            if receipt['utterance_id'] == 'utt:hardware-timeout'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual([item['state'] for item in terminal], ['error'])
        self.assertEqual(terminal[0]['reason'], 'play_state_idle_timeout')
        self.assertEqual(
            terminal[0]['completion_basis'], 'g1_play_state_observed',
        )
        self.assertEqual(client.stop_calls, [DEVICE.APP_NAME])

    def test_hardware_wait_interrupted_cannot_emit_completed(self):
        monitor = _BlockingPlaybackStateMonitor()
        client = _AudioClient()
        node = DEVICE._SpeakerNode(
            client,
            playback_state_monitor=monitor,
            completion_mode='hardware_state',
        )
        client.stop_calls.clear()
        node.start_play('/perception/hardware-interrupt')
        node._on_chunk(_chunk('utt:hardware-interrupt', b'I' * 3200))
        node._on_chunk(_chunk(
            'utt:hardware-interrupt', node.AUDIO_EOF_MAGIC,
        ))
        self.assertTrue(monitor.wait_entered.wait(timeout=1))

        result = node.interrupt()
        monitor.release_wait.set()
        thread = node._drain_thread
        if thread is not None:
            thread.join(timeout=1)

        terminals = [
            receipt['state'] for receipt in _receipts(node)
            if receipt['utterance_id'] == 'utt:hardware-interrupt'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(result['cancelled_utterance_ids'], [
            'utt:hardware-interrupt',
        ])
        self.assertEqual(terminals, ['cancelled'])

    def test_stale_stream_response_cannot_recreate_hardware_observation(self):
        monitor = _PlaybackStateMonitor()
        client = _DelayedFinishClient()
        node = DEVICE._SpeakerNode(
            client,
            playback_state_monitor=monitor,
            completion_mode='hardware_state',
        )
        client.stop_calls.clear()
        node.start_play('/perception/hardware-stale-response')
        node._on_chunk(_chunk(
            'utt:hardware-stale-response', b'S' * 3200,
        ))
        node._on_chunk(_chunk(
            'utt:hardware-stale-response', node.AUDIO_EOF_MAGIC,
        ))
        self.assertTrue(client.finish_entered.wait(timeout=1))

        node.interrupt()
        client.finish_release.set()
        thread = node._drain_thread
        if thread is not None:
            thread.join(timeout=1)

        self.assertEqual(monitor.submissions, [])
        terminals = [
            receipt['state'] for receipt in _receipts(node)
            if receipt['utterance_id'] == 'utt:hardware-stale-response'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(terminals, ['cancelled'])

    def test_hardware_mode_rejects_lossy_pause(self):
        monitor = _PlaybackStateMonitor()
        node = DEVICE._SpeakerNode(
            _AudioClient(),
            playback_state_monitor=monitor,
            completion_mode='hardware_state',
        )
        node.start_play('/perception/hardware-pause')
        node._on_chunk(_chunk('utt:hardware-pause', b'P' * 3200))

        result = node.pause()

        self.assertEqual(
            result['error'], 'pause_not_supported_with_hardware_state',
        )
        self.assertEqual(node.state, 'playing')
        node.interrupt()

    def test_smart_motion_does_not_advertise_unsupported_hardware_pause(self):
        executor = types.SimpleNamespace(add_node=lambda _node: None)
        speaker = DEVICE.SpeakerPlugin(
            {'completion_mode': 'hardware_state'},
            '', executor, _AudioClient(),
            playback_state_monitor=_PlaybackStateMonitor(),
        )
        smart_motion = DEVICE.SmartMotionPlugin(
            {}, '', executor, speaker_plugin=speaker,
        )

        actions = smart_motion.get_tool()['inputSchema']['properties'][
            'action'
        ]['enum']

        self.assertNotIn('pause_speak', actions)
        self.assertNotIn('resume_speak', actions)
        self.assertIn('interrupt_speak', actions)

    def test_playback_grace_is_applied_once_per_utterance(self):
        self.node.PREFILL = 100
        for _ in range(6):
            self.node._on_chunk(_chunk('utt:multi-block', b'M' * 3200))
        self.node._on_chunk(self._eof('utt:multi-block'))
        self._join_drain()

        self.assertEqual([len(call[2]) for call in self.client.stream_calls],
                         [9600, 9600])
        self.assertAlmostEqual(self.clock.value, 100.65, places=6)

    def test_receipt_qos_is_reliable_and_transient_local(self):
        qos = self.node._receipt_pub.qos.kwargs

        self.assertEqual(qos['reliability'], 'reliable')
        self.assertEqual(qos['durability'], 'transient_local')
        self.assertEqual(qos['depth'], 50)

    def test_terminal_publish_failure_is_retried_from_outbox(self):
        publisher = _FailingPublisher(
            self.node._receipt_topic, self.node._receipt_pub.qos,
            fail_attempts={2},
        )
        self.node._receipt_pub = publisher
        self.node._on_chunk(_chunk('utt:retry', b'R' * 3200))
        self.node._on_chunk(self._eof('utt:retry'))
        self._join_drain()

        self.assertEqual(
            [receipt['state'] for receipt in _receipts(self.node)], ['started'],
        )
        self.assertEqual(len(self.node._pending_receipts), 1)
        self.assertIsNotNone(self.node._receipt_retry_timer)

        self.node._retry_pending_receipts()

        self.assertEqual(
            [receipt['state'] for receipt in _receipts(self.node)],
            ['started', 'completed'],
        )
        self.assertEqual(self.node._pending_receipts, {})
        self.assertEqual(self.node._receipt_publish_errors, {})
        self.assertIsNone(self.node._receipt_retry_timer)

    def test_terminal_supersedes_failed_started_receipt(self):
        publisher = _FailingPublisher(
            self.node._receipt_topic, self.node._receipt_pub.qos,
            fail_attempts={1},
        )
        self.node._receipt_pub = publisher
        self.node._on_chunk(_chunk('utt:no-late-start', b'S' * 3200))
        self.node._on_chunk(self._eof('utt:no-late-start'))
        self._join_drain()

        self.assertEqual(
            [receipt['state'] for receipt in _receipts(self.node)], ['completed'],
        )

    def test_concurrent_retry_cannot_publish_started_after_terminal(self):
        publisher = _GatePublisher(
            self.node._receipt_topic, self.node._receipt_pub.qos,
        )
        self.node._receipt_pub = publisher
        self.node._emit_receipt('utt:monotonic', 'started')
        retry_thread = threading.Thread(
            target=self.node._retry_pending_receipts,
        )
        retry_thread.start()
        self.assertTrue(publisher.retry_entered.wait(timeout=1))
        def emit_terminal_while_holding_state_lock():
            # Reproduce the drain's historical outer-lock call pattern. Receipt
            # publication must never form state-lock/publish-lock ABBA.
            with self.node._lock:
                self.node._emit_receipt(
                    'utt:monotonic', 'completed',
                    completion_basis='driver_drained_estimated',
                )

        terminal_thread = threading.Thread(
            target=emit_terminal_while_holding_state_lock,
        )
        terminal_thread.start()
        time.sleep(0.02)
        publisher.retry_release.set()
        retry_thread.join(timeout=1)
        terminal_thread.join(timeout=1)

        self.assertFalse(retry_thread.is_alive())
        self.assertFalse(terminal_thread.is_alive())
        states = [receipt['state'] for receipt in _receipts(self.node)]
        self.assertIn(states, (['started', 'completed'], ['completed']))
        self.assertEqual(states[-1], 'completed')

    def test_returning_to_topic_restarts_matching_receipt_outbox(self):
        publisher = _FailingPublisher(
            self.node._receipt_topic, self.node._receipt_pub.qos,
            fail_attempts={2},
        )
        self.node._receipt_pub = publisher
        self.node._on_chunk(_chunk('utt:topic-a', b'A'))
        self.node._on_chunk(self._eof('utt:topic-a'))
        self._join_drain()
        self.assertEqual(len(self.node._pending_receipts), 1)

        self.assertEqual(self.node.stop_play()['state'], 'idle')
        self.node.start_play('/perception/topic-b')
        self.node._retry_pending_receipts()
        self.assertIsNone(self.node._receipt_retry_timer)
        self.assertEqual(len(self.node._pending_receipts), 1)

        self.assertEqual(self.node.stop_play()['state'], 'idle')
        self.node.start_play('/perception/tts')

        self.assertIsNotNone(self.node._receipt_retry_timer)
        self.node._retry_pending_receipts()
        self.assertEqual(self.node._pending_receipts, {})
        terminals = [
            receipt for receipt in _receipts(self.node)
            if receipt['utterance_id'] == 'utt:topic-a'
            and receipt['state'] == 'completed'
        ]
        self.assertEqual(len(terminals), 1)
        self.node._retry_pending_receipts()
        self.assertEqual(
            [receipt['state'] for receipt in _receipts(self.node)], ['completed'],
        )

    def test_stop_keeps_transient_receipt_publisher_for_late_subscriber(self):
        publisher = self.node._receipt_pub
        receipt_topic = self.node._receipt_topic
        self.node._on_chunk(_chunk('utt:retained', b'T' * 3200))
        self.node._on_chunk(self._eof('utt:retained'))
        self._join_drain()

        result = self.node.stop_play()

        self.assertEqual(result['state'], 'idle')
        self.assertIs(self.node._receipt_pub, publisher)
        self.assertEqual(self.node._receipt_topic, receipt_topic)
        self.assertIn(publisher, self.node.publishers)

    def test_short_stream_flush_debounces_until_idle(self):
        self.node._on_chunk(_chunk('utt:debounce', b'A'))
        first_timer = self.node._flush_timer
        self.clock.value += 0.1
        self.node._on_chunk(_chunk('utt:debounce', b'B'))
        second_timer = self.node._flush_timer

        self.assertTrue(first_timer.cancelled)
        self.assertIsNot(first_timer, second_timer)
        # A cancelled callback may already be queued; it must not cancel the
        # replacement timer.
        first_timer.callback()
        self.assertIs(self.node._flush_timer, second_timer)
        self.assertFalse(second_timer.cancelled)

        # If a timer is delivered early, re-arm for the remaining idle window.
        second_timer.callback()
        third_timer = self.node._flush_timer
        self.assertIsNotNone(third_timer)
        self.assertIsNot(third_timer, second_timer)
        self.clock.value += 0.16
        third_timer.callback()
        self._join_drain()

        self.assertEqual([call[2] for call in self.client.stream_calls], [b'AB'])
        terminal = [
            receipt for receipt in _receipts(self.node)
            if receipt['utterance_id'] == 'utt:debounce'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual([receipt['reason'] for receipt in terminal], ['missing_eof'])

    def test_stale_flush_callback_cannot_cancel_new_session_timer(self):
        self.node._on_chunk(_chunk('utt:old-timer', b'A'))
        stale_timer = self.node._flush_timer
        self.assertEqual(self.node.stop_play()['state'], 'idle')
        self.node.start_play('/perception/tts')
        self.node._on_chunk(_chunk('utt:new-timer', b'B'))
        current_timer = self.node._flush_timer

        stale_timer.callback()

        self.assertIs(self.node._flush_timer, current_timer)
        self.assertFalse(current_timer.cancelled)
        self.clock.value += 0.2
        current_timer.callback()
        self._join_drain()
        self.assertEqual([call[2] for call in self.client.stream_calls], [b'B'])

    def test_interrupt_drops_old_tail_and_next_utterance_survives(self):
        self.client.block_stream = True
        pcm_a = b'A' * 3200
        for _ in range(3):
            self.node._on_chunk(_chunk('utt:a', pcm_a))
        self.assertTrue(self.client.stream_entered.wait(timeout=1))

        result = self.node.interrupt()
        self.assertEqual(result['play_stop_code'], 0)
        self.assertEqual(result['cancelled_utterance_ids'], ['utt:a'])

        # Late frames and EOF from A must not leak into the next session.
        self.node._on_chunk(_chunk('utt:a', b'LATE'))
        self.node._on_chunk(self._eof('utt:a'))

        self.client.block_stream = False
        self.client.stream_entered.clear()
        pcm_b = b'B' * 3200
        self.node._on_chunk(_chunk('utt:b', pcm_b))
        self.node._on_chunk(self._eof('utt:b'))
        self._join_drain()

        streamed = [call[2] for call in self.client.stream_calls]
        self.assertNotIn(b'LATE', streamed)
        self.assertEqual(streamed[-1], pcm_b)
        terminals = {
            r['utterance_id']: r['state']
            for r in _receipts(self.node)
            if r['state'] in ('completed', 'cancelled', 'error')
        }
        self.assertEqual(terminals['utt:a'], 'cancelled')
        self.assertEqual(terminals['utt:b'], 'completed')

    def test_interrupt_linearizes_stop_before_pending_stream(self):
        sdk_lock = _StopFirstSdkLock()
        self.node._sdk_lock = sdk_lock
        generation = self.node._generation
        with self.node._lock:
            self.node._generation_ids[generation].add('utt:race')
        result_holder = {}

        def late_play():
            result_holder['value'] = self.node._play_merged(
                b'RACE', 1, 'utt:race', generation,
            )

        play_thread = threading.Thread(target=late_play, name='late-play')
        play_thread.start()
        self.assertTrue(sdk_lock.play_waiting.wait(timeout=1))

        result = self.node.interrupt()
        play_thread.join(timeout=1)

        self.assertFalse(play_thread.is_alive())
        self.assertEqual(result['play_stop_code'], 0)
        self.assertEqual(result['cancelled_utterance_ids'], ['utt:race'])
        self.assertEqual(result_holder['value'], (False, 'interrupted'))
        self.assertEqual(self.client.stream_calls, [])

    def test_interrupt_does_not_wait_for_stalled_stream_response(self):
        client = _DelayedFinishClient()
        node = DEVICE._SpeakerNode(client)
        client.stop_calls.clear()
        node.start_play('/perception/stalled-response')
        for _ in range(3):
            node._on_chunk(_chunk('utt:stalled', b'S' * 3200))
        self.assertTrue(client.finish_entered.wait(timeout=1))
        old_drain = node._drain_thread

        started = time.monotonic()
        result = node.interrupt()
        elapsed = time.monotonic() - started

        self.assertLess(elapsed, 0.25)
        self.assertEqual(result['state'], 'ready')
        self.assertEqual(result['play_stop_code'], 0)
        self.assertEqual(result['cancelled_utterance_ids'], ['utt:stalled'])
        client.finish_release.set()
        old_drain.join(timeout=1)
        self.assertFalse(old_drain.is_alive())

    def test_stale_stream_error_cannot_stop_new_generation(self):
        client = _StaleErrorClient()
        node = DEVICE._SpeakerNode(
            client,
            monotonic=self.clock.monotonic,
            wall_time=self.clock.wall_time,
            waiter=self.clock.wait,
        )
        client.stop_calls.clear()
        node.start_play('/perception/stale-error')
        for _ in range(3):
            node._on_chunk(_chunk('utt:a', b'A' * 3200))
        self.assertTrue(client.first_finish_entered.wait(timeout=1))
        old_drain = node._drain_thread
        self.assertEqual(node.interrupt()['state'], 'ready')
        stop_count_after_interrupt = len(client.stop_calls)

        node._on_chunk(_chunk('utt:b', b'B' * 3200))
        node._on_chunk(_chunk('utt:b', node.AUDIO_EOF_MAGIC))
        self._wait_for(
            lambda: len(client.stream_calls) == 2,
            'new generation did not submit before stale response release',
        )
        client.first_finish_release.set()
        old_drain.join(timeout=1)
        self.assertFalse(old_drain.is_alive())
        self._wait_for(
            lambda: node._drain_thread is None,
            'new-generation drain did not finish',
        )

        self.assertEqual(len(client.stop_calls), stop_count_after_interrupt)
        terminals = {
            receipt['utterance_id']: receipt['state']
            for receipt in _receipts(node)
            if receipt['state'] in ('completed', 'cancelled', 'error')
        }
        self.assertEqual(terminals['utt:a'], 'cancelled')
        self.assertEqual(terminals['utt:b'], 'completed')

    def test_interrupt_receipts_item_already_popped_from_queue(self):
        handoff_queue = _BlockingGetQueue()
        self.node._buf = handoff_queue
        for _ in range(3):
            self.node._on_chunk(_chunk('utt:handoff', b'H' * 3200))
        self.assertTrue(handoff_queue.item_popped.wait(timeout=1))
        interrupted_generation = self.node._generation
        result_holder = {}

        interrupt_thread = threading.Thread(
            target=lambda: result_holder.setdefault('value', self.node.interrupt()),
        )
        interrupt_thread.start()
        self._wait_for(
            lambda: self.node._generation != interrupted_generation,
            'interrupt did not advance the speaker generation',
        )
        handoff_queue.release_item.set()
        interrupt_thread.join(timeout=1)

        self.assertFalse(interrupt_thread.is_alive())
        self.assertEqual(
            result_holder['value']['cancelled_utterance_ids'], ['utt:handoff'],
        )
        terminals = [
            receipt for receipt in _receipts(self.node)
            if receipt['utterance_id'] == 'utt:handoff'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual([receipt['state'] for receipt in terminals], ['cancelled'])

    def test_interrupt_does_not_let_old_drain_consume_new_generation(self):
        client = _BlockingStopClient()
        node = DEVICE._SpeakerNode(
            client,
            monotonic=self.clock.monotonic,
            wall_time=self.clock.wall_time,
            waiter=self.clock.wait,
        )
        client.stop_calls.clear()
        client.stop_entered.clear()
        node.start_play('/perception/generation-race')
        waiting_queue = _GetWaitingQueue()
        node._buf = waiting_queue
        node._start_drain()
        self.assertTrue(waiting_queue.get_waiting.wait(timeout=1))

        client.block_stop = True
        result_holder = {}
        interrupt_thread = threading.Thread(
            target=lambda: result_holder.setdefault('value', node.interrupt()),
        )
        interrupt_thread.start()
        self.assertTrue(client.stop_entered.wait(timeout=1))

        pcm = b'NEW-GENERATION'
        node._on_chunk(_chunk('utt:new-generation', pcm))
        node._on_chunk(_chunk('utt:new-generation', node.AUDIO_EOF_MAGIC))
        client.stop_release.set()
        interrupt_thread.join(timeout=1)
        self.assertFalse(interrupt_thread.is_alive())
        self.assertEqual(result_holder['value']['play_stop_code'], 0)
        self._wait_for(
            lambda: any(
                receipt['utterance_id'] == 'utt:new-generation'
                and receipt['state'] == 'completed'
                for receipt in _receipts(node)
            ),
            'new-generation utterance was consumed by the old drain',
        )
        self._wait_for(
            lambda: node._drain_thread is None,
            'new-generation drain thread did not finish',
        )

        self.assertEqual([call[2] for call in client.stream_calls], [pcm])

    def test_interrupt_before_queue_get_detaches_old_buffer(self):
        gated_buffer = _PreGetGateQueue()
        self.node._buf = gated_buffer
        self.node._start_drain()
        self.assertTrue(gated_buffer.before_get.wait(timeout=1))
        old_drain = self.node._drain_thread

        result = self.node.interrupt()
        pcm = b'NEW-AFTER-PRE-GET'
        self.node._on_chunk(_chunk('utt:after-pre-get', pcm))
        self.node._on_chunk(self._eof('utt:after-pre-get'))
        gated_buffer.release_get.set()

        old_drain.join(timeout=1)
        self.assertFalse(old_drain.is_alive())
        self._wait_for(
            lambda: any(
                receipt['utterance_id'] == 'utt:after-pre-get'
                and receipt['state'] == 'completed'
                for receipt in _receipts(self.node)
            ),
            'old pre-get drain consumed the new generation buffer',
        )
        self._wait_for(
            lambda: self.node._drain_thread is None,
            'new-generation drain did not finish',
        )

        self.assertEqual(result['state'], 'ready')
        self.assertEqual([call[2] for call in self.client.stream_calls], [pcm])

    def test_back_to_back_utterances_have_one_terminal_each(self):
        for utterance_id, byte in [('utt:a', b'A'), ('utt:b', b'B')]:
            self.node._on_chunk(_chunk(utterance_id, byte * 3200))
            self.node._on_chunk(self._eof(utterance_id))
            self._join_drain()

        terminals = [
            (r['utterance_id'], r['state'])
            for r in _receipts(self.node)
            if r['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(terminals, [('utt:a', 'completed'), ('utt:b', 'completed')])

    def test_same_utterance_id_can_be_reused_after_restart(self):
        self.node._on_chunk(_chunk('utt:1', b'FIRST'))
        self.node._on_chunk(self._eof('utt:1'))
        self._join_drain()
        first_session = self.node._session_id
        self.assertEqual(self.node.stop_play()['state'], 'idle')
        self.node.start_play('/perception/tts')
        second_session = self.node._session_id

        self.node._on_chunk(_chunk('utt:1', b'SECOND'))
        self.node._on_chunk(self._eof('utt:1'))
        self._join_drain()

        terminals = [
            receipt for receipt in _receipts(self.node)
            if receipt['utterance_id'] == 'utt:1'
            and receipt['state'] == 'completed'
        ]
        self.assertEqual(len(terminals), 2)
        self.assertNotEqual(first_session, second_session)
        self.assertEqual(
            {receipt['session_id'] for receipt in terminals},
            {first_session, second_session},
        )
        self.assertEqual(
            [call[2] for call in self.client.stream_calls], [b'FIRST', b'SECOND'],
        )

    def test_eof_tail_race_restarts_drain_for_next_utterance(self):
        race_queue = _EmptyRaceQueue()
        self.node._buf = race_queue
        self.node._on_chunk(_chunk('utt:a', b'A' * 3200))
        race_queue.arm()
        self.node._on_chunk(self._eof('utt:a'))
        self.assertTrue(race_queue.empty_observed.wait(timeout=1))

        self.node._on_chunk(_chunk('utt:b', b'B' * 3200))
        self.node._on_chunk(self._eof('utt:b'))
        race_queue.release_empty.set()
        self._wait_for(
            lambda: any(
                receipt['utterance_id'] == 'utt:b'
                and receipt['state'] == 'completed'
                for receipt in _receipts(self.node)
            ),
            'next utterance was stranded at the previous EOF boundary',
        )
        self._wait_for(
            lambda: self.node._drain_thread is None,
            'replacement drain thread did not finish',
        )

        terminals = [
            (receipt['utterance_id'], receipt['state'])
            for receipt in _receipts(self.node)
            if receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(terminals, [('utt:a', 'completed'), ('utt:b', 'completed')])

    def test_sdk_error_is_error_not_completed(self):
        self.client.stream_code = 17
        self.node._on_chunk(_chunk('utt:error', b'E' * 3200))
        self.node._on_chunk(self._eof('utt:error'))
        self._join_drain()

        terminals = [
            r for r in _receipts(self.node)
            if r['utterance_id'] == 'utt:error'
            and r['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(len(terminals), 1)
        self.assertEqual(terminals[0]['state'], 'error')
        self.assertEqual(terminals[0]['reason'], 'play_stream_failed:17')

    def test_stream_error_drops_remaining_blocks(self):
        client = _SequenceStreamClient([17, 0])
        node = DEVICE._SpeakerNode(
            client,
            monotonic=self.clock.monotonic,
            wall_time=self.clock.wall_time,
            waiter=self.clock.wait,
        )
        client.stop_calls.clear()
        node.start_play('/perception/stream-error')
        node.PREFILL = 100
        for _ in range(6):
            node._on_chunk(_chunk('utt:partial-failure', b'X' * 3200))
        node._on_chunk(_chunk(
            'utt:partial-failure', node.AUDIO_EOF_MAGIC,
        ))
        self._join_drain(node)

        self.assertEqual(len(client.stream_calls), 1)
        terminals = [
            receipt for receipt in _receipts(node)
            if receipt['utterance_id'] == 'utt:partial-failure'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(len(terminals), 1)
        self.assertEqual(terminals[0]['state'], 'error')
        self.assertEqual(terminals[0]['reason'], 'play_stream_failed:17')

    def test_play_stop_failure_fails_closed(self):
        self.client.stop_code = 9
        self.node._on_chunk(_chunk('utt:stop-failure', b'F' * 3200))

        result = self.node.interrupt()
        self.node._on_chunk(_chunk('utt:must-drop', b'DROP'))
        self.node._on_chunk(self._eof('utt:must-drop'))

        self.assertEqual(result['state'], 'error')
        self.assertEqual(result['play_stop_code'], 9)
        self.assertEqual(result['error'], 'play_stop_failed:9')
        self.assertFalse(self.node._accept_chunks)
        self.assertEqual(self.client.stream_calls, [])
        terminal = [
            receipt for receipt in _receipts(self.node)
            if receipt['utterance_id'] == 'utt:stop-failure'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(len(terminal), 1)
        self.assertEqual(terminal[0]['state'], 'error')
        self.assertEqual(terminal[0]['reason'], 'play_stop_failed:9')

    def test_second_interrupt_cannot_fake_recovery_from_error(self):
        self.client.stop_code = 9
        self.node._on_chunk(_chunk('utt:failed-stop', b'F'))
        self.assertEqual(self.node.interrupt()['state'], 'error')
        stop_call_count = len(self.client.stop_calls)
        self.client.stop_code = 0

        retry = self.node.interrupt()

        self.assertEqual(retry['state'], 'error')
        self.assertEqual(retry['error'], 'speaker requires stop/start recovery')
        self.assertEqual(len(self.client.stop_calls), stop_call_count)
        self.assertFalse(self.node._accept_chunks)

        self.assertEqual(self.node.stop_play()['state'], 'idle')
        self.node.start_play('/perception/recovered')
        self.node._on_chunk(_chunk('utt:recovered', b'OK'))
        self.node._on_chunk(_chunk('utt:recovered', self.node.AUDIO_EOF_MAGIC))
        self._join_drain()
        self.assertEqual(self.client.stream_calls[-1][2], b'OK')

    def test_pause_stop_failure_errors_pending_utterance(self):
        self.client.stop_code = 13
        self.node._on_chunk(_chunk('utt:pause-failure', b'F' * 3200))

        result = self.node.pause()

        self.assertEqual(result['state'], 'error')
        self.assertEqual(result['play_stop_code'], 13)
        self.assertEqual(result['error'], 'play_stop_failed:13')
        self.assertFalse(self.node._accept_chunks)
        self.assertTrue(self.node._buf.empty())
        terminal = [
            receipt for receipt in _receipts(self.node)
            if receipt['utterance_id'] == 'utt:pause-failure'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(len(terminal), 1)
        self.assertEqual(terminal[0]['state'], 'error')
        self.assertEqual(terminal[0]['reason'], 'play_stop_failed:13')

    def test_resume_without_pending_audio_returns_ready(self):
        self.assertEqual(self.node.pause()['state'], 'paused')

        self.assertEqual(self.node.resume()['state'], 'ready')

    def test_duplicate_eof_does_not_duplicate_terminal_receipt(self):
        self.node._on_chunk(_chunk('utt:once', b'O' * 3200))
        self.node._on_chunk(self._eof('utt:once'))
        self._join_drain()
        self.node._on_chunk(self._eof('utt:once'))
        self._join_drain()

        terminals = [
            r for r in _receipts(self.node)
            if r['utterance_id'] == 'utt:once'
            and r['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(len(terminals), 1)
        self.assertEqual(terminals[0]['state'], 'completed')

    def test_correlated_stream_without_eof_fails_closed(self):
        self.node._on_chunk(_chunk('utt:no-eof', b'N' * 3200))
        self.clock.value += 0.2
        self.node.timers[-1].callback()
        self._join_drain()

        terminals = [
            r for r in _receipts(self.node)
            if r['utterance_id'] == 'utt:no-eof'
            and r['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(len(terminals), 1)
        self.assertEqual(terminals[0]['state'], 'error')
        self.assertEqual(terminals[0]['reason'], 'missing_eof')

    def test_legacy_mute_watchdog_accepts_new_utterance_after_gap(self):
        self.node._on_chunk(_chunk('', b'OLD' * 1000))
        result = self.node.interrupt()
        self.assertEqual(result['cancelled_utterance_ids'], ['legacy:1'])

        self.node._on_chunk(_chunk('', b'LATE'))
        self.clock.value += self.node.LEGACY_NEW_UTTERANCE_GAP_SEC + 0.01
        pcm = b'NEW' * 1000
        self.node._on_chunk(_chunk('', pcm))
        self.node._on_chunk(self._eof(''))
        self._join_drain()

        self.assertEqual([call[2] for call in self.client.stream_calls], [pcm])
        terminals = {
            r['utterance_id']: r['state']
            for r in _receipts(self.node)
            if r['state'] in ('completed', 'cancelled', 'error')
        }
        self.assertEqual(terminals['legacy:1'], 'cancelled')
        self.assertEqual(terminals['legacy:2'], 'completed')

    def test_legacy_idle_completion_resets_id_after_lost_eof(self):
        self.node._on_chunk(_chunk('', b'FIRST'))
        self.clock.value += 0.2
        self.node.timers[-1].callback()
        self._join_drain()

        self.node._on_chunk(_chunk('', b'SECOND'))
        self.node._on_chunk(self._eof(''))
        self._join_drain()

        terminals = [
            (receipt['utterance_id'], receipt['state'])
            for receipt in _receipts(self.node)
            if receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(
            terminals, [('legacy:1', 'completed'), ('legacy:2', 'completed')],
        )
        self.assertEqual(
            [call[2] for call in self.client.stream_calls], [b'FIRST', b'SECOND'],
        )

    def test_non_protocol_frame_id_is_treated_as_legacy(self):
        self.node._on_chunk(_chunk('request-123', b'L' * 3200))
        self.node._on_chunk(self._eof('request-123'))
        self._join_drain()

        utterance_ids = {receipt['utterance_id'] for receipt in _receipts(self.node)}
        self.assertEqual(utterance_ids, {'legacy:1'})

    def test_stop_drops_callback_racing_with_sdk_stop(self):
        client = _BlockingStopClient()
        node = DEVICE._SpeakerNode(
            client,
            monotonic=self.clock.monotonic,
            wall_time=self.clock.wall_time,
            waiter=self.clock.wait,
        )
        client.stop_calls.clear()
        client.stop_entered.clear()
        node.start_play('/perception/old')
        client.block_stop = True
        result_holder = {}
        stop_thread = threading.Thread(
            target=lambda: result_holder.setdefault('value', node.stop_play()),
        )
        stop_thread.start()
        self.assertTrue(client.stop_entered.wait(timeout=1))

        node._on_chunk(_chunk('utt:stale', b'STALE'))
        node._on_chunk(_chunk('utt:stale', node.AUDIO_EOF_MAGIC))
        client.stop_release.set()
        stop_thread.join(timeout=1)

        self.assertFalse(stop_thread.is_alive())
        self.assertEqual(result_holder['value']['state'], 'idle')
        self.assertTrue(node._buf.empty())

        client.block_stop = False
        node.start_play('/perception/new')
        node._on_chunk(_chunk('utt:new', b'NEW'))
        node._on_chunk(_chunk('utt:new', node.AUDIO_EOF_MAGIC))
        self._join_drain(node)

        self.assertEqual([call[2] for call in client.stream_calls], [b'NEW'])

    def test_stop_and_start_are_lifecycle_serialized(self):
        client = _BlockingStopClient()
        node = DEVICE._SpeakerNode(client)
        client.stop_calls.clear()
        client.stop_entered.clear()
        node.start_play('/perception/old')
        client.block_stop = True
        stop_result = {}
        start_result = {}
        stop_thread = threading.Thread(
            target=lambda: stop_result.setdefault('value', node.stop_play()),
        )
        stop_thread.start()
        self.assertTrue(client.stop_entered.wait(timeout=1))
        start_thread = threading.Thread(
            target=lambda: start_result.setdefault(
                'value', node.start_play('/perception/new'),
            ),
        )
        start_thread.start()
        time.sleep(0.05)

        self.assertTrue(start_thread.is_alive())
        client.stop_release.set()
        stop_thread.join(timeout=1)
        start_thread.join(timeout=1)

        self.assertFalse(stop_thread.is_alive())
        self.assertFalse(start_thread.is_alive())
        self.assertEqual(stop_result['value']['state'], 'idle')
        self.assertEqual(start_result['value'], '/perception/new')
        self.assertEqual(node.state, 'ready')
        self.assertEqual(node._topic, '/perception/new')
        self.assertEqual(node._sub.topic, '/perception/new')
        self.assertTrue(node._accept_chunks)
        self.assertIsNotNone(node._receipt_pub)

    def test_pause_does_not_complete_until_resumed(self):
        client = _AudioClient()
        node = DEVICE._SpeakerNode(client)
        client.stop_calls.clear()
        client.stream_entered.clear()
        node.start_play('/perception/pause')
        pcm = b'P' * 9600
        node._on_chunk(_chunk('utt:pause', pcm))
        node._on_chunk(_chunk('utt:pause', node.AUDIO_EOF_MAGIC))
        self.assertTrue(client.stream_entered.wait(timeout=1))

        pause_result = node.pause()
        time.sleep(0.08)

        self.assertEqual(pause_result['state'], 'paused')
        self.assertFalse(any(
            receipt['utterance_id'] == 'utt:pause'
            and receipt['state'] == 'completed'
            for receipt in _receipts(node)
        ))

        self.assertEqual(node.resume()['state'], 'playing')
        self._join_drain(node)
        terminals = [
            receipt['state'] for receipt in _receipts(node)
            if receipt['utterance_id'] == 'utt:pause'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(terminals, ['completed'])
        self.assertEqual([call[2] for call in client.stream_calls], [pcm, pcm])

    def test_pause_epoch_replays_block_when_finish_arrives_after_resume(self):
        client = _DelayedFinishClient()
        node = DEVICE._SpeakerNode(client)
        client.stop_calls.clear()
        node.start_play('/perception/pause-epoch')
        pcm = b'E' * 9600
        node._on_chunk(_chunk('utt:pause-epoch', pcm))
        node._on_chunk(_chunk('utt:pause-epoch', node.AUDIO_EOF_MAGIC))
        self.assertTrue(client.finish_entered.wait(timeout=1))

        self.assertEqual(node.pause()['state'], 'paused')
        self.assertEqual(node.resume()['state'], 'playing')
        self.assertEqual(len(client.stream_calls), 1)
        client.finish_release.set()
        self._wait_for(
            lambda: len(client.stream_calls) == 2,
            'paused in-flight block was not replayed after resume',
        )
        self._wait_for(
            lambda: node._drain_thread is None,
            'pause-epoch drain did not finish',
            timeout=2,
        )

        terminals = [
            receipt['state'] for receipt in _receipts(node)
            if receipt['utterance_id'] == 'utt:pause-epoch'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(terminals, ['completed'])
        self.assertEqual([call[2] for call in client.stream_calls], [pcm, pcm])

    def test_pause_epoch_ignores_cancelled_stream_error_response(self):
        client = _StaleErrorClient()
        node = DEVICE._SpeakerNode(client)
        client.stop_calls.clear()
        node.start_play('/perception/pause-error-response')
        pcm = b'C' * 9600
        node._on_chunk(_chunk('utt:pause-cancelled-rpc', pcm))
        node._on_chunk(_chunk(
            'utt:pause-cancelled-rpc', node.AUDIO_EOF_MAGIC,
        ))
        self.assertTrue(client.first_finish_entered.wait(timeout=1))

        self.assertEqual(node.pause()['state'], 'paused')
        self.assertEqual(node.resume()['state'], 'playing')
        client.first_finish_release.set()
        self._wait_for(
            lambda: len(client.stream_calls) == 2,
            'cancelled stream response was not replayed after resume',
        )
        self._wait_for(
            lambda: node._drain_thread is None,
            'pause error-response drain did not finish',
            timeout=2,
        )

        terminals = [
            receipt['state'] for receipt in _receipts(node)
            if receipt['utterance_id'] == 'utt:pause-cancelled-rpc'
            and receipt['state'] in ('completed', 'cancelled', 'error')
        ]
        self.assertEqual(terminals, ['completed'])
        self.assertEqual(len(client.stop_calls), 1)

    def test_interrupt_invalidates_remaining_startup_sound(self):
        client = _DelayedFinishClient()
        executor = types.SimpleNamespace(add_node=lambda _node: None)
        plugin = DEVICE.SpeakerPlugin({}, '', executor, client)
        client.stop_calls.clear()
        plugin._node._monotonic = self.clock.monotonic
        plugin._node._waiter = self.clock.wait
        result_holder = {}
        start_thread = threading.Thread(
            target=lambda: result_holder.setdefault(
                'value', plugin.dispatch(
                    'start', {'input_topic': '/perception/startup'},
                ),
            ),
        )
        start_thread.start()
        self.assertTrue(client.finish_entered.wait(timeout=1))

        interrupt_result = plugin._node.interrupt()
        stream_count_at_interrupt = len(client.stream_calls)
        client.finish_release.set()
        start_thread.join(timeout=1)

        self.assertFalse(start_thread.is_alive())
        self.assertEqual(interrupt_result['state'], 'idle')
        self.assertEqual(stream_count_at_interrupt, 1)
        self.assertEqual(len(client.stream_calls), 1)
        self.assertEqual(result_holder['value']['error'], 'startup_interrupted')
        self.assertIsNone(plugin._node._sub)
        self.assertIsNone(plugin._node._topic)

    def test_stale_startup_exception_cannot_stop_new_generation(self):
        client = _StaleExceptionClient()
        executor = types.SimpleNamespace(add_node=lambda _node: None)
        plugin = DEVICE.SpeakerPlugin({}, '', executor, client)
        client.stop_calls.clear()
        plugin._node._monotonic = self.clock.monotonic
        plugin._node._waiter = self.clock.wait
        result_holder = {}
        start_thread = threading.Thread(
            target=lambda: result_holder.setdefault(
                'value', plugin.dispatch(
                    'start', {'input_topic': '/perception/stale-startup'},
                ),
            ),
        )
        start_thread.start()
        self.assertTrue(client.first_finish_entered.wait(timeout=1))

        self.assertEqual(plugin._node.interrupt()['state'], 'idle')
        stop_count_after_interrupt = len(client.stop_calls)
        plugin._node.start_play('/perception/new-session')
        pcm = b'NEW-SESSION'
        plugin._node._on_chunk(_chunk('utt:new-session', pcm))
        plugin._node._on_chunk(_chunk(
            'utt:new-session', plugin._node.AUDIO_EOF_MAGIC,
        ))
        self._wait_for(
            lambda: any(
                receipt['utterance_id'] == 'utt:new-session'
                and receipt['state'] == 'completed'
                for receipt in _receipts(plugin._node)
            ),
            'new session did not complete while startup response was stale',
        )

        client.first_finish_release.set()
        start_thread.join(timeout=1)

        self.assertFalse(start_thread.is_alive())
        self.assertEqual(result_holder['value']['error'], 'startup_interrupted')
        self.assertEqual(len(client.stop_calls), stop_count_after_interrupt)
        self.assertEqual([call[2] for call in client.stream_calls], [
            mock.ANY, pcm,
        ])

    def test_info_advertises_receipt_capability(self):
        executor = types.SimpleNamespace(add_node=lambda _node: None)
        plugin = DEVICE.SpeakerPlugin({}, '', executor, _AudioClient())
        plugin._node.start_play('/perception/tts')

        info = plugin.dispatch('info', {})

        self.assertEqual(info['speech_protocol'], 'audiochunk-frameid-v1')
        self.assertEqual(info['receipt_topic'],
                         '/perception/tts/speaker_receipts')
        self.assertEqual(info['completion_basis'], 'driver_drained_estimated')
        self.assertEqual(info['session_id'], plugin._node._session_id)
        self.assertEqual(info['receipt_recovery'],
                         'transient_local_then_info_poll')
        self.assertEqual(info['topic_out'][0]['qos']['reliability'], 'reliable')

    def test_native_tts_speak_can_be_hidden_for_strict_speaker_ownership(self):
        plugin = DEVICE.NativeTtsPlugin(
            {
                'speak_enabled': False,
                'speak_disabled_reason': 'strict_speaker_owns_body_audio',
            },
            '', None, _AudioClient(),
        )

        actions = plugin.get_tool()['inputSchema']['properties']['action'][
            'enum'
        ]
        result = plugin.dispatch('speak', {'text': 'must not play'})

        self.assertNotIn('speak', actions)
        self.assertIn('get_volume', actions)
        self.assertEqual(result, {
            'state': 'error',
            'error': 'strict_speaker_owns_body_audio',
        })

    def test_hardware_info_advertises_g1_play_state_evidence(self):
        executor = types.SimpleNamespace(add_node=lambda _node: None)
        monitor = _PlaybackStateMonitor()
        plugin = DEVICE.SpeakerPlugin(
            {
                'completion_mode': 'hardware_state',
                'play_state_timeout_sec': 1.5,
            },
            '', executor, _AudioClient(),
            playback_state_monitor=monitor,
        )
        plugin._node.start_play('/perception/hardware-info')

        info = plugin.dispatch('info', {})

        self.assertEqual(info['completion_mode'], 'hardware_state')
        self.assertEqual(info['completion_basis'], 'g1_play_state_observed')
        self.assertTrue(info['play_state']['available'])
        self.assertEqual(info['play_state']['topic'], 'rt/audio_msg')
        self.assertEqual(plugin._node._play_state_timeout_sec, 1.5)

    def test_hardware_start_fails_before_audio_when_monitor_unavailable(self):
        executor = types.SimpleNamespace(add_node=lambda _node: None)
        monitor = _PlaybackStateMonitor(available=False)
        client = _AudioClient()
        plugin = DEVICE.SpeakerPlugin(
            {'completion_mode': 'hardware_state'},
            '', executor, client,
            playback_state_monitor=monitor,
        )
        client.stop_calls.clear()

        result = plugin.dispatch(
            'start', {'input_topic': '/perception/unavailable'},
        )

        self.assertEqual(result['state'], 'error')
        self.assertEqual(result['error'], 'play_state_monitor_unavailable')
        self.assertEqual(client.stream_calls, [])

    def test_hardware_start_waits_for_matched_state_publisher(self):
        executor = types.SimpleNamespace(add_node=lambda _node: None)
        monitor = _PlaybackStateMonitor(ready=False)
        client = _AudioClient()
        plugin = DEVICE.SpeakerPlugin(
            {
                'completion_mode': 'hardware_state',
                'play_state_discovery_timeout_sec': 1.25,
            },
            '', executor, client,
            playback_state_monitor=monitor,
        )
        client.stop_calls.clear()

        result = plugin.dispatch(
            'start', {'input_topic': '/perception/unmatched-state'},
        )

        self.assertEqual(result['state'], 'error')
        self.assertEqual(result['error'], 'play_state_publisher_timeout')
        self.assertEqual(monitor.ready_waits, [1.25])
        self.assertEqual(client.stream_calls, [])

    def test_stop_during_discovery_cannot_resurrect_stale_start(self):
        executor = types.SimpleNamespace(add_node=lambda _node: None)
        monitor = _BlockingReadyPlaybackStateMonitor()
        client = _AudioClient()
        plugin = DEVICE.SpeakerPlugin(
            {'completion_mode': 'hardware_state'},
            '', executor, client,
            playback_state_monitor=monitor,
        )
        client.stop_calls.clear()
        result_holder = {}
        start_thread = threading.Thread(
            target=lambda: result_holder.setdefault(
                'value', plugin.dispatch(
                    'start', {'input_topic': '/perception/stale-discovery'},
                ),
            ),
        )
        start_thread.start()
        self.assertTrue(monitor.ready_wait_entered.wait(timeout=1))

        self.assertEqual(plugin._node.stop_play()['state'], 'idle')
        monitor.ready_wait_release.set()
        start_thread.join(timeout=1)

        self.assertFalse(start_thread.is_alive())
        self.assertEqual(result_holder['value']['error'], 'interrupted')
        self.assertEqual(client.stream_calls, [])
        self.assertIsNone(plugin._node._sub)
        self.assertIsNone(plugin._node._topic)

    def test_hardware_startup_sound_waits_for_g1_play_state(self):
        executor = types.SimpleNamespace(add_node=lambda _node: None)
        monitor = _PlaybackStateMonitor()
        client = _AudioClient()
        plugin = DEVICE.SpeakerPlugin(
            {'completion_mode': 'hardware_state'},
            '', executor, client,
            playback_state_monitor=monitor,
        )
        client.stop_calls.clear()
        plugin._node._monotonic = self.clock.monotonic
        plugin._node._waiter = self.clock.wait

        result = plugin.dispatch(
            'start', {'input_topic': '/perception/hardware-startup'},
        )

        startup_key = monitor.wait_calls[0][0]
        self.assertEqual(result['state'], 'ready')
        self.assertEqual(result['completion_basis'], 'g1_play_state_observed')
        self.assertTrue(client.stream_calls)
        self.assertEqual(startup_key[1], f'startup:{startup_key[0]}')
        self.assertEqual(
            {key for key, _checkpoint in monitor.submissions},
            {startup_key},
        )
        self.assertEqual(
            monitor.wait_calls,
            [(startup_key, plugin._node._play_state_timeout_sec)],
        )
        self.assertIn(startup_key, monitor.forgotten)

    def test_hardware_startup_state_timeout_fails_before_subscription(self):
        executor = types.SimpleNamespace(add_node=lambda _node: None)
        monitor = _PlaybackStateMonitor({
            'state': 'timeout',
            'reason': 'play_state_idle_timeout',
        })
        client = _AudioClient()
        plugin = DEVICE.SpeakerPlugin(
            {'completion_mode': 'hardware_state'},
            '', executor, client,
            playback_state_monitor=monitor,
        )
        client.stop_calls.clear()
        plugin._node._monotonic = self.clock.monotonic
        plugin._node._waiter = self.clock.wait

        result = plugin.dispatch(
            'start', {'input_topic': '/perception/hardware-startup-timeout'},
        )

        self.assertEqual(
            result['error'], 'startup_play_state_idle_timeout',
        )
        self.assertIsNone(plugin._node._sub)
        self.assertIsNone(plugin._node._topic)
        self.assertTrue(client.stream_calls)
        self.assertEqual(client.stop_calls, [DEVICE.APP_NAME, DEVICE.APP_NAME])


class AudioClientWrapperTests(unittest.TestCase):
    def _load_client(self):
        class _Client:
            def __init__(self, *_args, **_kwargs):
                pass

        packages = {
            'unitree_sdk2py': _package('unitree_sdk2py'),
            'unitree_sdk2py.g1': _package('unitree_sdk2py.g1'),
            'unitree_sdk2py.g1.audio': _package('unitree_sdk2py.g1.audio'),
        }
        rpc = types.ModuleType('unitree_sdk2py.rpc.client')
        rpc.Client = _Client
        packages['unitree_sdk2py.rpc'] = _package('unitree_sdk2py.rpc')
        packages['unitree_sdk2py.rpc.client'] = rpc
        api = types.ModuleType('unitree_sdk2py.g1.audio.g1_audio_api')
        api.AUDIO_SERVICE_NAME = 'audio'
        api.AUDIO_API_VERSION = '1.0'
        api.ROBOT_API_ID_AUDIO_TTS = 1001
        api.ROBOT_API_ID_AUDIO_ASR = 1002
        api.ROBOT_API_ID_AUDIO_START_PLAY = 1003
        api.ROBOT_API_ID_AUDIO_STOP_PLAY = 1004
        api.ROBOT_API_ID_AUDIO_GET_VOLUME = 1005
        api.ROBOT_API_ID_AUDIO_SET_VOLUME = 1006
        api.ROBOT_API_ID_AUDIO_SET_RGB_LED = 1007
        packages['unitree_sdk2py.g1.audio.g1_audio_api'] = api

        name = 'unitree_sdk2py.g1.audio.g1_audio_client_under_test'
        spec = importlib.util.spec_from_file_location(
            name, G1_DIR / 'unitree_sdk2py/g1/audio/g1_audio_client.py',
        )
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, packages):
            spec.loader.exec_module(module)
        return module.AudioClient()

    def test_play_stop_returns_sdk_code(self):
        client = self._load_client()
        client._Call = lambda *_args, **_kwargs: (23, {'ignored': True})

        self.assertEqual(client.PlayStop('g1_speaker'), 23)

    def test_native_tts_uses_monotonic_request_indices(self):
        client = self._load_client()
        parameters = []

        def call(_api_id, parameter):
            parameters.append(json.loads(parameter))
            return 0, {}

        client._Call = call

        self.assertEqual(client.TtsMaker('first', 0), 0)
        self.assertEqual(client.TtsMaker('second', 1), 0)
        self.assertEqual([parameter['index'] for parameter in parameters], [1, 2])

    def test_async_audio_wrappers_forward_send_and_finish_timeouts(self):
        client = self._load_client()
        calls = []
        client._CallRequestWithParamAndBinStart = (
            lambda *args: calls.append(('stream_start', args)) or (0, 'stream')
        )
        client._CallRequestWithParamAndBinFinish = (
            lambda *args: calls.append(('stream_finish', args)) or (0, {})
        )
        client._CallStart = (
            lambda *args: calls.append(('stop_start', args)) or (0, 'stop')
        )
        client._CallFinish = (
            lambda *args: calls.append(('stop_finish', args)) or (0, {})
        )

        self.assertEqual(
            client.PlayStreamStart('speaker', '0', b'PCM', send_timeout=0.025),
            (0, 'stream'),
        )
        self.assertEqual(client.PlayStreamFinish('stream', timeout=0.5), (0, {}))
        self.assertEqual(
            client.PlayStopStart('speaker', send_timeout=0.03), (0, 'stop'),
        )
        self.assertEqual(client.PlayStopFinish('stop', timeout=0.4), 0)

        self.assertEqual(calls[0][1][-1], 0.025)
        self.assertEqual(calls[1][1], ('stream', 0.5))
        self.assertEqual(calls[2][1][-1], 0.03)
        self.assertEqual(calls[3][1], ('stop', 0.4))


class ClientForwardingTests(unittest.TestCase):
    def _load_client(self):
        class _ClientBase:
            def __init__(self, _service_name):
                self.base_calls = []

            def _CallStartBase(self, *args):
                self.base_calls.append(('parameter', args))
                return 0, 'parameter-pending'

            def _CallRequestWithParamAndBinStartBase(self, *args):
                self.base_calls.append(('binary', args))
                return 0, 'binary-pending'

        packages = {
            'unitree_sdk2py': _package('unitree_sdk2py'),
            'unitree_sdk2py.rpc': _package('unitree_sdk2py.rpc'),
        }
        client_base = types.ModuleType('unitree_sdk2py.rpc.client_base')
        client_base.ClientBase = _ClientBase
        packages[client_base.__name__] = client_base
        lease_client = types.ModuleType('unitree_sdk2py.rpc.lease_client')
        lease_client.LeaseClient = object
        packages[lease_client.__name__] = lease_client
        internal = types.ModuleType('unitree_sdk2py.rpc.internal')
        internal.RPC_INTERNAL_API_ID_MAX = 100
        internal.RPC_ERR_CLIENT_API_NOT_REG = -1
        packages[internal.__name__] = internal

        name = 'unitree_sdk2py.rpc.client_under_test'
        spec = importlib.util.spec_from_file_location(
            name, G1_DIR / 'unitree_sdk2py/rpc/client.py',
        )
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, packages):
            spec.loader.exec_module(module)
        return module.Client('audio')

    def test_client_forwards_optional_send_timeout_to_both_start_paths(self):
        client = self._load_client()
        client._RegistApi(1003, 7)
        client._RegistApi(1004, 9)

        self.assertEqual(
            client._CallStart(1004, '{}', send_timeout=0.03),
            (0, 'parameter-pending'),
        )
        self.assertEqual(
            client._CallRequestWithParamAndBinStart(
                1003, '{}', [1, 2], send_timeout=0.025,
            ),
            (0, 'binary-pending'),
        )

        self.assertEqual(
            client.base_calls,
            [
                ('parameter', (1004, '{}', 9, 0, 0.03)),
                ('binary', (1003, '{}', [1, 2], 7, 0, 0.025)),
            ],
        )


class _RpcFutureResult:
    FUTURE_SUCC = 0
    FUTUTE_ERR_TIMEOUT = 1


class _RpcFuture:
    def __init__(self, result):
        self.result = result
        self.timeouts = []

    def GetResult(self, timeout):
        self.timeouts.append(timeout)
        return self.result


class _RpcClientStub:
    def __init__(self, _service_name):
        self.future = None
        self.requests = []
        self.removed = []

    def Init(self):
        pass

    def SendRequest(self, request, timeout):
        self.requests.append((request, timeout))
        return self.future

    def RemoveFuture(self, request_id):
        self.removed.append(request_id)


class ClientBaseAsyncTests(unittest.TestCase):
    SEND_ERROR = -101
    TIMEOUT_ERROR = -102
    API_MISMATCH_ERROR = -103

    def _load_client_base(self):
        class _Identity:
            def __init__(self, request_id, api_id):
                self.id = request_id
                self.api_id = api_id

        class _Lease:
            def __init__(self, lease_id):
                self.id = lease_id

        class _Policy:
            def __init__(self, priority, no_reply):
                self.priority = priority
                self.no_reply = no_reply

        class _Header:
            def __init__(self, identity, lease, policy):
                self.identity = identity
                self.lease = lease
                self.policy = policy

        class _Request:
            def __init__(self, header, parameter, binary):
                self.header = header
                self.parameter = parameter
                self.binary = binary

        packages = {
            'unitree_sdk2py': _package('unitree_sdk2py'),
            'unitree_sdk2py.rpc': _package('unitree_sdk2py.rpc'),
            'unitree_sdk2py.idl': _package('unitree_sdk2py.idl'),
            'unitree_sdk2py.idl.unitree_api': _package(
                'unitree_sdk2py.idl.unitree_api',
            ),
            'unitree_sdk2py.idl.unitree_api.msg': _package(
                'unitree_sdk2py.idl.unitree_api.msg',
            ),
            'unitree_sdk2py.utils': _package('unitree_sdk2py.utils'),
        }
        dds = types.ModuleType('unitree_sdk2py.idl.unitree_api.msg.dds_')
        dds.Request_ = _Request
        dds.RequestHeader_ = _Header
        dds.RequestLease_ = _Lease
        dds.RequestIdentity_ = _Identity
        dds.RequestPolicy_ = _Policy
        packages[dds.__name__] = dds
        future = types.ModuleType('unitree_sdk2py.utils.future')
        future.FutureResult = _RpcFutureResult
        packages[future.__name__] = future
        client_stub = types.ModuleType('unitree_sdk2py.rpc.client_stub')
        client_stub.ClientStub = _RpcClientStub
        packages[client_stub.__name__] = client_stub
        internal = types.ModuleType('unitree_sdk2py.rpc.internal')
        internal.RPC_ERR_CLIENT_SEND = self.SEND_ERROR
        internal.RPC_ERR_CLIENT_API_TIMEOUT = self.TIMEOUT_ERROR
        internal.RPC_ERR_UNKNOWN = -104
        internal.RPC_ERR_CLIENT_API_NOT_MATCH = self.API_MISMATCH_ERROR
        packages[internal.__name__] = internal

        name = 'unitree_sdk2py.rpc.client_base_under_test'
        spec = importlib.util.spec_from_file_location(
            name, G1_DIR / 'unitree_sdk2py/rpc/client_base.py',
        )
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, packages):
            spec.loader.exec_module(module)
        client = module.ClientBase('audio')
        stub = client._ClientBase__stub
        return client, stub

    @staticmethod
    def _response(api_id, status=0, data='ok'):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(
                identity=types.SimpleNamespace(api_id=api_id),
                status=types.SimpleNamespace(code=status),
            ),
            data=data,
        )

    def test_binary_rpc_start_returns_before_finish_waits(self):
        client, stub = self._load_client_base()
        client.SetTimeout(10.0)
        future = _RpcFuture(types.SimpleNamespace(
            code=_RpcFutureResult.FUTURE_SUCC,
            value=self._response(1003, status=7, data='accepted'),
        ))
        stub.future = future

        code, pending = client._CallRequestWithParamAndBinStartBase(
            1003, '{"app_name":"g1_speaker"}', [1, 2, 3],
            send_timeout=0.025,
        )

        self.assertEqual(code, 0)
        self.assertEqual(future.timeouts, [])
        self.assertEqual(stub.requests[0][1], 0.025)
        self.assertEqual(stub.requests[0][0].binary, [1, 2, 3])
        self.assertEqual(
            client._CallRequestWithParamAndBinFinishBase(pending),
            (7, 'accepted'),
        )
        self.assertEqual(future.timeouts, [10.0])

    def test_parameter_rpc_start_send_timeout_is_optional_and_compatible(self):
        client, stub = self._load_client_base()
        client.SetTimeout(10.0)
        stub.future = _RpcFuture(types.SimpleNamespace(
            code=_RpcFutureResult.FUTURE_SUCC,
            value=self._response(1004),
        ))

        self.assertEqual(client._CallStartBase(
            1004, '{}', send_timeout=0.04,
        )[0], 0)
        self.assertEqual(stub.requests[-1][1], 0.04)
        self.assertEqual(client._CallStartBase(1004, '{}')[0], 0)
        self.assertEqual(stub.requests[-1][1], 10.0)

    def test_control_rpc_finish_uses_short_timeout_and_removes_future(self):
        client, stub = self._load_client_base()
        future = _RpcFuture(types.SimpleNamespace(
            code=_RpcFutureResult.FUTUTE_ERR_TIMEOUT,
            value=None,
        ))
        stub.future = future
        code, pending = client._CallStartBase(1004, '{}')
        request_id = pending[1]

        result = client._CallFinishBase(pending, timeout=0.25)

        self.assertEqual(code, 0)
        self.assertEqual(result, (self.TIMEOUT_ERROR, None))
        self.assertEqual(future.timeouts, [0.25])
        self.assertEqual(stub.removed, [request_id])

    def test_binary_finish_timeout_removes_future(self):
        client, stub = self._load_client_base()
        future = _RpcFuture(types.SimpleNamespace(
            code=_RpcFutureResult.FUTUTE_ERR_TIMEOUT,
            value=None,
        ))
        stub.future = future
        code, pending = client._CallRequestWithParamAndBinStartBase(
            1003, '{}', [], send_timeout=0.02,
        )
        request_id = pending[1]

        result = client._CallRequestWithParamAndBinFinishBase(
            pending, timeout=0.5,
        )

        self.assertEqual(code, 0)
        self.assertEqual(result, (self.TIMEOUT_ERROR, None))
        self.assertEqual(future.timeouts, [0.5])
        self.assertEqual(stub.removed, [request_id])

    def test_async_finish_rejects_mismatched_response_api(self):
        client, stub = self._load_client_base()
        stub.future = _RpcFuture(types.SimpleNamespace(
            code=_RpcFutureResult.FUTURE_SUCC,
            value=self._response(9999),
        ))
        code, pending = client._CallStartBase(1004, '{}')

        self.assertEqual(code, 0)
        self.assertEqual(
            client._CallFinishBase(pending),
            (self.API_MISMATCH_ERROR, None),
        )

    def test_async_start_propagates_send_failure(self):
        client, _stub = self._load_client_base()

        self.assertEqual(
            client._CallRequestWithParamAndBinStartBase(1003, '{}', []),
            (self.SEND_ERROR, None),
        )


if __name__ == '__main__':
    unittest.main()
