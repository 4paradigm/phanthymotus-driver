import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('motion_music_rpc', Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
rpc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rpc)


class MotionMusicTests(unittest.TestCase):
    def test_only_movement_track_changes_backup_restore_and_external_edit_protected(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            for name in ('1_flower.mp3', '2_alarm.mp3', '7_standby.mp3'):
                (directory / name).write_bytes(name.encode())
            def encode(command, **kwargs):
                Path(command[-1]).write_bytes(b'synthetic silent audio fixture')
            with patch('shutil.which', return_value='/usr/bin/ffmpeg'), patch('subprocess.run', side_effect=encode) as run:
                result = rpc.configure_motion_music(False, directory)
                self.assertEqual(result['state'], 'disabled')
                self.assertTrue(result['other_audio_unchanged'])
                self.assertFalse(rpc.configure_motion_music(False, directory)['changed'])
                self.assertEqual(run.call_count, 1)
                for name in ('2_alarm.mp3', '7_standby.mp3'):
                    self.assertEqual((directory / name).read_bytes(), name.encode())
                self.assertEqual(rpc.configure_motion_music(True, directory)['state'], 'enabled')
                self.assertEqual((directory / '1_flower.mp3').read_bytes(), b'1_flower.mp3')
                (directory / '1_flower.mp3').write_bytes(b'operator changed song')
                self.assertEqual(rpc.configure_motion_music(False, directory)['state'], 'blocked')
                self.assertEqual((directory / '1_flower.mp3').read_bytes(), b'operator changed song')

    def test_missing_tool_or_unknown_layout_does_not_modify_audio(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            self.assertEqual(rpc.configure_motion_music(False, directory)['state'], 'unsupported')
            track = directory / '1_flower.mp3';track.write_bytes(b'original')
            with patch('shutil.which', return_value=None):
                self.assertEqual(rpc.configure_motion_music(False, directory)['state'], 'unsupported')
            self.assertEqual(track.read_bytes(), b'original')
            self.assertFalse((directory / '.motus-motion-music').exists())


class StandbyVoiceTests(unittest.TestCase):
    def test_independent_backup_restore_preserves_movement_alarm_and_tts(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            names = ('1_flower.mp3', '2_alarm.mp3', '7_standby.mp3', 'tts.wav')
            for name in names:
                (directory / name).write_bytes(name.encode())
            def encode(command, **kwargs):
                Path(command[-1]).write_bytes(b'silent fixture')
            with patch('shutil.which', return_value='/usr/bin/ffmpeg'), patch('subprocess.run', side_effect=encode):
                self.assertEqual(rpc.configure_motion_music(False, directory)['state'], 'disabled')
                result = rpc.configure_standby_voice(False, directory)
                self.assertEqual(result['scope'], 'factory_id7_standby_track_only')
                self.assertFalse(rpc.configure_standby_voice(False, directory)['changed'])
                self.assertEqual((directory / '2_alarm.mp3').read_bytes(), b'2_alarm.mp3')
                self.assertEqual((directory / 'tts.wav').read_bytes(), b'tts.wav')
                rpc.configure_standby_voice(True, directory)
                self.assertEqual((directory / '7_standby.mp3').read_bytes(), b'7_standby.mp3')
                self.assertEqual((directory / '1_flower.mp3').read_bytes(), b'silent fixture')
                (directory / '7_standby.mp3').write_bytes(b'operator edit')
                self.assertEqual(rpc.configure_standby_voice(False, directory)['state'], 'blocked')
                self.assertEqual((directory / '7_standby.mp3').read_bytes(), b'operator edit')
