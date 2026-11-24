"""Speaker volume must attenuate its own playback without changing mic PCM."""
import queue
import struct
import sys
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audio import AudioPlugin, FORMAT, _speaker_pcm, _worker

CONFIG = {'server': 'auto', 'uid': 1001, 'gid': 1002, 'source': 'mic', 'sink': 'speaker'}


class PlaybackLifecycleTest(unittest.TestCase):
    def test_overflow_reaps_playback_before_next_command(self):
        from common import logsafe

        commands, replies = queue.Queue(), queue.Queue()
        commands.put(('start', {'input_topic': '/test/audio'}))
        node, playback = Mock(), Mock()
        playback.poll.return_value = None

        def overflow(*args, **kwargs):
            callback = node.create_subscription.call_args.args[2]
            callback(SimpleNamespace(format=FORMAT, data=bytes(257 * 4096)))
            playback.terminate.assert_not_called()
            node.destroy_subscription.assert_not_called()
            commands.put(('info', {}))

        def close(*args, **kwargs):
            playback.terminate.assert_called_once()
            playback.wait.assert_called_once_with(timeout=2)
            commands.put(('close', {}))

        rclpy = ModuleType('rclpy')
        rclpy.init, rclpy.shutdown = Mock(), Mock()
        spins = iter((lambda *args, **kwargs: None, overflow, close))
        rclpy.spin_once = lambda *args, **kwargs: next(spins)(*args, **kwargs)
        modules = {
            'rclpy': rclpy,
            'rclpy.node': SimpleNamespace(Node=Mock(return_value=node)),
            'rclpy.qos': SimpleNamespace(QoSProfile=Mock(), ReliabilityPolicy=Mock(), HistoryPolicy=Mock()),
            'audio_msgs.msg': SimpleNamespace(AudioChunk=object()),
        }
        with patch.dict(sys.modules, modules), patch.dict('os.environ'), \
             patch.object(logsafe, 'install'), patch('audio._resolve_pulse', return_value={'server': 'test'}), \
             patch('audio._pulse_user', return_value={}), patch('audio.time.sleep'), \
             patch('audio.subprocess.Popen', return_value=playback), patch('audio.threading.Thread'):
            _worker('speaker', '', {'server': 'test', 'device': 'test'}, commands, replies)

        self.assertEqual(replies.get_nowait()['state'], 'running')
        info = replies.get_nowait()
        self.assertEqual(info['state'], 'error')
        self.assertIn('speaker buffer full', info['error'])
        self.assertEqual(info['dropped'], 1)
        self.assertEqual(info['queued_chunks'], 0)


class VolumeTest(unittest.TestCase):
    def test_pcm_zero_unity_and_signed_attenuation(self):
        samples = (-32768, -32767, -101, -1, 0, 1, 101, 32767)
        raw = struct.pack('<8h', *samples)
        self.assertIs(_speaker_pcm(raw, 100), raw)
        self.assertEqual(_speaker_pcm(raw, 0), bytes(16))
        self.assertEqual(struct.unpack('<8h', _speaker_pcm(raw, 50)),
                         (-16384, -16383, -50, 0, 0, 0, 50, 16383))
        self.assertEqual(struct.unpack('<8h', raw), samples)
        self.assertEqual(len(_speaker_pcm(raw, 37)), len(raw))

    def test_rejects_bad_values_without_changing_saved_volume(self):
        plugin = AudioPlugin('speaker', 'test', CONFIG)
        self.assertEqual(plugin.dispatch('set_volume', {'volume': 60})['volume'], 60)
        for value in (None, True, False, -1, 101, '50', 50.0, float('nan')):
            with self.subTest(value=value):
                self.assertEqual(plugin.dispatch('set_volume', {'volume': value})['state'], 'error')
                self.assertEqual(plugin.dispatch('get_volume', {})['volume'], 60)

    def test_idle_volume_never_starts_hardware_and_is_used_by_next_worker(self):
        plugin = AudioPlugin('speaker', 'test', CONFIG)
        self.assertEqual(plugin.dispatch('get_volume', {}),
                         {'state': 'idle', 'volume': 100, 'volume_scope': 'stream'})
        self.assertEqual(plugin.dispatch('set_volume', {'volume': 0})['volume'], 0)
        self.assertEqual(plugin.config['volume'], 0)
        self.assertIsNone(plugin._process)
        self.assertEqual(plugin.dispatch('stop', {})['volume'], 0)

    def test_active_volume_reaches_worker_and_survives_worker_exit(self):
        plugin = AudioPlugin('speaker', 'test', CONFIG)
        plugin._process = SimpleNamespace(is_alive=lambda: True)
        plugin._commands = queue.Queue()
        plugin._replies = queue.Queue()
        plugin._replies.put({'state': 'running', 'volume': 35, 'volume_scope': 'stream'})
        self.assertEqual(plugin.dispatch('set_volume', {'volume': 35})['volume'], 35)
        self.assertEqual(plugin._commands.get_nowait(), ('set_volume', {'volume': 35}))
        plugin._process = None
        self.assertEqual(plugin.dispatch('get_volume', {})['volume'], 35)
        self.assertEqual(plugin.config['volume'], 35)

    def test_mic_does_not_expose_or_accept_volume(self):
        mic = AudioPlugin('mic', 'test', CONFIG)
        self.assertEqual(mic.get_tool()['inputSchema']['properties']['action']['enum'], ['start', 'stop', 'info'])
        self.assertNotIn('volume', mic.get_tool()['inputSchema']['properties'])
        self.assertIn('error', mic.dispatch('set_volume', {'volume': 50}))
        self.assertNotIn('volume', mic.config)
        speaker = AudioPlugin('speaker', 'test', CONFIG).get_tool()['inputSchema']
        self.assertEqual(speaker['x-action-params']['set_volume']['params'], ['volume'])
        self.assertEqual(speaker['properties']['volume']['maximum'], 100)


if __name__ == '__main__':
    unittest.main()
