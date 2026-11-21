"""Speaker volume must attenuate its own playback without changing mic PCM."""
import queue
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audio import AudioPlugin, _speaker_pcm

CONFIG = {'server': 'auto', 'uid': 1001, 'gid': 1002, 'source': 'mic', 'sink': 'speaker'}


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
