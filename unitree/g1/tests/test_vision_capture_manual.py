"""Manual-save control, naming, ownership, and real recorder integration."""
import io
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch
import wave

from test_av_recording import av, load_plugin, pcm, burst_capture_events


def run_events(events, directory, duration=5, stop_at=None, include_audio=True, cancel=False):
    class Clock:
        now = 0.
        def monotonic(self): return self.now
    clock = Clock(); stop = threading.Event(); cancelled = threading.Event(); state = {}
    if cancel: cancelled.set()
    events = list(events)
    class Buffer:
        def get(self, timeout):
            next_time = events[0][1] if events else float('inf')
            if stop_at is not None and not stop.is_set() and stop_at <= min(clock.now+timeout, next_time):
                clock.now = max(clock.now, stop_at)
                state['stop_at_monotonic'] = stop_at
                stop.set(); return None
            if events and next_time <= clock.now+timeout:
                source, received, kind, data = events.pop(0)
                clock.now = max(clock.now, received)
                return kind, 10_000_000_000+round(source*1e9), data, received, received-source
            clock.now += timeout
            return None
    with patch.object(av.time, 'monotonic', clock.monotonic):
        result = av.collect(Buffer(), directory, duration, cancelled,
                            stop_event=stop, stop_state=state, include_audio=include_audio)
    return result, clock.now


class ManualControlTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()
        camera = types.SimpleNamespace(is_running=lambda: True, _color_topic='/g1/camera/rgb')
        self.p = load_plugin()({}, 'g1', Mock(), camera)
        self.p._output_dir = self.root
        self.p._frame = lambda *args: ({'data': b'jpeg', 'received_monotonic': time.monotonic()}, 1)
        self.notifications = []
        self.p._record_video_async.__globals__['_vision_acp_notify'] = lambda *args: self.notifications.append(args)

    def start(self, **args):
        with patch.object(shutil, 'which', return_value='ffmpeg'), patch.object(threading, 'Thread'):
            return self.p.dispatch('start_recording', args)

    def test_manual_start_is_immediate_and_not_an_acp_barrier(self):
        r = self.start(video_name='session')
        self.assertTrue(r['ok']); self.assertEqual('recording', r['state'])
        self.assertNotIn('action_id', r); self.assertIn('recording_id', r)
        self.assertTrue(r['include_audio']); self.assertEqual('session.mp4', r['filename'])
        self.assertEqual(30, self.p._active_recording['duration_s'])
        schema = self.p.get_tool()['inputSchema']
        self.assertNotIn('start_recording', schema['x-completion']['actions'])
        self.assertIn('stop_recording', schema['x-completion']['actions'])
        for action in ['start_recording', 'record_video']:
            self.assertNotIn('include_audio', schema['x-action-params'][action]['params'])
        self.assertEqual([], self.p._list_media({})['files'])

    def test_stop_save_is_idempotent_and_distinct_from_cancel(self):
        self.start(video_name='session')
        active = self.p._active_recording
        r1 = self.p._stop_recording(); r2 = self.p._stop_recording()
        self.assertEqual(r1['action_id'], r2['action_id'])
        self.assertTrue(active['save_event'].is_set()); self.assertFalse(active['cancel_event'].is_set())
        self.assertEqual('MEDIA_BUSY', self.p._delete_media({'media_type':'video','filename':'session.mp4'})['code'])
        self.p.stop()
        self.assertTrue(active['cancel_event'].is_set())
        self.assertEqual('RECORD_CANCELLED', self.p._stop_recording()['code'])

    def test_stop_without_start_returns_idle(self):
        self.assertEqual('idle', self.p._stop_recording()['state'])
        self.assertNotIn('action_id', self.p._stop_recording())

    def test_duplicate_start_does_not_reserve_second_file(self):
        self.start(video_name='first')
        self.assertEqual('RECORD_IN_PROGRESS', self.start(video_name='second')['code'])
        self.assertFalse((self.root/'videos/second.mp4').exists())

    def test_custom_photo_metadata_and_collision(self):
        r = self.p.dispatch('capture_photo', {'image_name':'sample.jpg'})
        self.assertTrue(r['ok']); self.assertEqual('sample.jpg', r['filename'])
        self.assertEqual(r['file_path'], r['path']); self.assertEqual('image/jpeg', r['mime'])
        self.assertEqual(4, r['size'])
        self.assertEqual('FILE_EXISTS', self.p._capture_photo({'image_name':'sample'})['code'])
        self.assertEqual(b'jpeg', Path(r['path']).read_bytes())
        self.assertTrue(self.p._delete_media({'media_type':'photo','filename':r['filename']})['ok'])

    def test_video_collision_never_overwrites_existing_data(self):
        (self.root/'videos').mkdir(); keep=self.root/'videos/session.mp4';keep.write_bytes(b'existing')
        self.assertEqual('FILE_EXISTS', self.start(video_name='session.mp4')['code'])
        self.assertEqual(b'existing',keep.read_bytes());self.assertIsNone(self.p._active_recording)

    def test_invalid_names_do_not_create_files(self):
        for value in ['../escape', '/tmp/escape', r'a\b', '.hidden', 'a b', 'x'*101, 42, 'a\x00b']:
            with self.subTest(value=value):
                self.assertEqual('INVALID_FILENAME', self.start(video_name=value)['code'])
                self.assertEqual('INVALID_FILENAME', self.p._capture_photo({'image_name':value})['code'])
        self.assertEqual([], list(self.root.iterdir()))

    def test_new_duration_overrides_stored_legacy_field(self):
        with patch.object(shutil,'which',return_value='ffmpeg'),patch.object(threading,'Thread'):
            r=self.p._start_video_recording({'duration':10,'duration_s':5,'include_audio':False,'video_name':'old-api'})
        self.assertTrue(r['ok']); self.assertEqual(10,self.p._active_recording['duration_s'])
        self.assertFalse(self.p._active_recording['include_audio'])

    def test_thread_launch_failure_cleans_reserved_file(self):
        with patch.object(shutil,'which',return_value='ffmpeg'),patch.object(threading,'Thread') as thread:
            thread.return_value.start.side_effect=RuntimeError('cannot launch')
            r=self.p._start_video_recording({'video_name':'failed'})
        self.assertFalse(r['ok']);self.assertIsNone(self.p._active_recording)
        self.assertEqual([],list((self.root/'videos').iterdir()))

    def test_completed_save_is_listed_notified_and_deletable(self):
        self.start(video_name='saved')
        active=self.p._active_recording;stop=self.p._stop_recording()
        def record(a):
            Path(a['path']).write_bytes(b'encoded')
            return {'ok':True,'file_path':a['path'],'recorded_duration_s':10,'include_audio':True}
        self.p._record_video=record
        self.p._record_video_async(active)
        result=self.p._last_recording['result']
        self.assertEqual('recorded',result['state']); self.assertEqual('saved.mp4',result['filename'])
        self.assertEqual({active['action_id'],stop['action_id']},{r[0] for r in self.notifications})
        self.assertTrue(all(r[1]=='completed' for r in self.notifications))
        self.assertEqual(['saved.mp4'],[f['filename'] for f in self.p._list_media({})['files']])
        self.assertTrue(self.p._delete_media({'media_type':'video','filename':'saved.mp4'})['ok'])

    def test_cancel_and_worker_error_release_reservation(self):
        for code in ['RECORD_CANCELLED','AUDIO_UNAVAILABLE']:
            self.p._active_recording=None;self.start(video_name=code)
            active=self.p._active_recording
            if code=='RECORD_CANCELLED':active['cancel_event'].set()
            self.p._record_video=lambda a:{'ok':False,'code':code,'message':'test failure'}
            self.p._record_video_async(active)
            self.assertIsNone(self.p._active_recording)
            self.assertFalse(Path(active['path']).exists())


class ManualTimelineTests(unittest.TestCase):
    def run_capture(self, **kwargs):
        with tempfile.TemporaryDirectory() as d:
            result, elapsed=run_events(burst_capture_events(b'jpeg'),Path(d),**kwargs)
            with wave.open(str(Path(d)/'audio.wav'),'rb') as wav:
                frames=wav.getnframes()
            return result,elapsed,frames

    def test_manual_stop_uses_actual_time_and_drains_final_pcm_burst(self):
        (frames, info),elapsed,samples=self.run_capture(stop_at=1.15)
        self.assertEqual([0,.07,.37,.83],[t for t,_ in frames])
        self.assertAlmostEqual(1.15,info['recorded_duration_s'])
        self.assertEqual(18400,info['audio_samples']); self.assertEqual(18400,samples)
        self.assertEqual('stop_recording',info['ended_by'])
        self.assertEqual(0,info['audio_gap_s']); self.assertLess(elapsed,1.65)

    def test_maximum_duration_auto_saves(self):
        (frames,info),elapsed,_=self.run_capture(duration=1)
        self.assertEqual('duration_limit',info['ended_by']);self.assertEqual(1,info['recorded_duration_s'])
        self.assertTrue(all(t<1 for t,_ in frames));self.assertLessEqual(elapsed,1.5)

    def test_manual_silent_mode_does_not_require_mic(self):
        events=burst_capture_events(b'jpeg',with_audio=False)
        with tempfile.TemporaryDirectory() as d:
            (frames,info),elapsed=run_events(events,Path(d),stop_at=1.15,include_audio=False)
        self.assertEqual(4,len(frames));self.assertEqual(1.15,info['recorded_duration_s'])
        self.assertNotIn('audio_samples',info);self.assertLess(elapsed,1.21)

    def test_stop_before_first_frame_is_explicit_failure(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(av.RecordingError) as e:
                run_events([],Path(d),stop_at=.01)
        self.assertEqual('RECORD_TOO_SHORT',e.exception.code)

    def test_cancel_wins_over_save(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(av.RecordingError) as e:
                run_events(burst_capture_events(b'jpeg'),Path(d),stop_at=0,cancel=True)
        self.assertEqual('RECORD_CANCELLED',e.exception.code)

    def test_spool_limit_remains_enforced(self):
        with tempfile.TemporaryDirectory() as d,patch.object(av,'MAX_SPOOL_BYTES',1):
            with self.assertRaises(av.RecordingError) as e:
                run_events(burst_capture_events(b'jpeg'),Path(d),stop_at=1.15)
        self.assertEqual('RECORD_SPOOL_LIMIT',e.exception.code)


@unittest.skipUnless(shutil.which('ffmpeg'),'FFmpeg unavailable')
class ManualEncodingTests(unittest.TestCase):
    def test_real_vfr_aac_manual_stop_and_silent_compatibility(self):
        for sound in [True,False]:
            with self.subTest(sound=sound), tempfile.TemporaryDirectory() as d:
                root=Path(d).resolve();jpeg=root/'source.jpg'
                subprocess.run(['ffmpeg','-v','error','-f','lavfi','-i','color=c=blue:s=64x48','-frames:v','1',str(jpeg)],check=True)
                probe=subprocess.run(['ffmpeg','-v','error','-f','image2','-ts_from_file','2','-i',str(jpeg),'-frames:v','1','-f','null','-'],capture_output=True)
                if b'nanosecond file timestamps unavailable' in probe.stderr:self.skipTest('FFmpeg build lacks nanosecond input')
                self.assertEqual(0,probe.returncode,probe.stderr)
                p=load_plugin()({},'g1',Mock(),types.SimpleNamespace(_color_topic='/g1/camera/rgb'))
                p._output_dir=root
                path,reservation=p._reserve_media_file('video','named.mp4');reservation.close()
                active={'duration_s':5,'cancel_event':threading.Event(),'save_event':threading.Event(),
                        'include_audio':sound,'reserved_path':str(path),'path':str(path)}
                events=burst_capture_events(jpeg.read_bytes(),with_audio=sound)
                collect=av.collect
                def capture(buffer,directory,duration,cancel,**kwargs):
                    with patch.object(av,'collect',collect):
                        return run_events(events,directory,duration,stop_at=1.15,include_audio=sound)[0]
                node=types.SimpleNamespace(destroy_node=Mock())
                with patch.object(av,'make_input_node',return_value=node),patch.object(av,'collect',side_effect=capture):
                    result=av.record_av(p,active)
                self.assertTrue(result['ok'],result);self.assertAlmostEqual(1.15,result['recorded_duration_s'])
                self.assertEqual(str(path),result['file_path']);self.assertIsNone(active['process'])
                node.destroy_node.assert_called_once()
                decoded=subprocess.run(['ffmpeg','-hide_banner','-i',str(path),'-vf','showinfo','-enc_time_base:v','1:1000000','-f','null','-'],capture_output=True,text=True,check=True)
                self.assertIn('Video: h264',decoded.stderr)
                self.assertEqual(sound,'Audio: aac' in decoded.stderr)
                self.assertIn('pts_time:0.83',decoded.stderr.replace('pts_time: ','pts_time:'))
                self.assertEqual([path],list((root/'videos').iterdir()))
