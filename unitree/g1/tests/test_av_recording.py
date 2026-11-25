"""Offline A/V timing, failure isolation, and real FFmpeg muxing checks."""

from array import array
import ast
from datetime import datetime
import importlib.util
import io
import math
import os
from pathlib import Path
import re
import select
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch
from uuid import uuid4
import wave


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("g1_av_under_test", ROOT / "av_recording.py")
av = importlib.util.module_from_spec(spec)
spec.loader.exec_module(av)


def pcm(count, value=1000):
    data = array("h", [value] * count)
    if sys.byteorder != "little":
        data.byteswap()
    return data.tobytes()


def load_plugin():
    tree = ast.parse((ROOT / "device.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "VisionCapturePlugin")
    ns = dict(globals(), _VISION_FIRST_FRAME_TIMEOUT_S=5.0,
              _VISION_MAX_FRAME_AGE_S=3.0)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), "device.py", "exec"), ns)
    return ns["VisionCapturePlugin"]


def burst_capture_events(jpeg, with_audio=True):
    events = [(t, t+.025, "video", jpeg) for t in
              [0., .07, .37, .83, 1.4, 1.9, 2.6, 3., 3.8, 4.7]]
    end = 0.
    if with_audio:
        # Some arrivals combine two usual UDP payloads into a 320 ms burst.
        for count in [5, 5] + [10] * 15:
            end += count * .032
            for i in range(count):
                stamp = end - (count-1-i) * .0004
                events.append((stamp, stamp+.005, "audio", pcm(512)))
    return sorted(events, key=lambda e: e[1])


def collect_events(events, directory, duration=5):
    class Clock:
        now = 0.
        def monotonic(self): return self.now
    clock = Clock()
    events = list(events)
    class Buffer:
        def get(self, timeout):
            if events and events[0][1] <= clock.now+timeout:
                source, received, kind, data = events.pop(0)
                clock.now = max(clock.now, received)
                return kind, 10_000_000_000+round(source*1e9), data, received, received-source
            clock.now += timeout
            return None
    with patch.object(av.time, "monotonic", clock.monotonic):
        return av.collect(Buffer(), directory, duration, threading.Event())


class TimelineTests(unittest.TestCase):
    def timeline(self, duration=1):
        self.raw = io.BytesIO()
        self.wav = wave.open(self.raw, "wb")
        return av.AudioTimeline(self.wav, 10_000_000_000, duration)

    def tearDown(self):
        if hasattr(self, "wav"):
            self.wav.close()

    def test_sample_clock_trims_start_and_places_chunk_by_header_end(self):
        t = self.timeline()
        t.add(10_100_000_000, pcm(3200, 123))  # [-.1, .1], clipped to [0, .1]
        t.add(10_300_000_000, pcm(1600, 456))  # [.2, .3], 100 ms gap
        self.assertEqual(4800, t.position)
        self.assertEqual(1600, t.padded)
        self.wav.close()
        self.raw.seek(0)
        with wave.open(self.raw, "rb") as f:
            data = f.readframes(4800)
        self.assertEqual(pcm(1600, 123) + pcm(1600, 0) + pcm(1600, 456), data)

    def test_overlapping_packet_and_end_are_trimmed(self):
        t = self.timeline(.1)
        t.add(10_080_000_000, pcm(1280))
        t.add(10_112_000_000, pcm(640))
        self.assertEqual(1600, t.finish()["audio_samples"])
        self.assertEqual(128, t.trimmed)

    def test_long_audio_gap_fails(self):
        t = self.timeline()
        t.add(10_032_000_000, pcm(512))
        with self.assertRaisesRegex(av.RecordingError, "gap exceeded"):
            t.add(10_400_000_000, pcm(512))

    def test_missing_audio_fails(self):
        with self.assertRaisesRegex(av.RecordingError, "No microphone"):
            self.timeline().finish()

    def test_audio_stops_early_fails(self):
        t = self.timeline()
        t.add(10_100_000_000, pcm(1600))
        with self.assertRaisesRegex(av.RecordingError, "gap exceeded"):
            t.finish()

    def test_all_zero_audio_is_not_reported_as_success(self):
        t = self.timeline(.1)
        t.add(10_100_000_000, pcm(1600, 0))
        with self.assertRaisesRegex(av.RecordingError, "Only zero PCM"):
            t.finish()

    def test_duplicate_or_backwards_audio_stamp_fails(self):
        t = self.timeline()
        t.add(10_032_000_000, pcm(512))
        with self.assertRaises(av.RecordingError):
            t.add(10_032_000_000, pcm(512))

    def test_malformed_audio_fails(self):
        t = self.timeline()
        with self.assertRaises(av.RecordingError):
            t.add(10_032_000_000, b"x")


class BurstTests(unittest.TestCase):
    def make_timeline(self, duration):
        self.raw = io.BytesIO()
        self.wav = wave.open(self.raw, "wb")
        self.addCleanup(self.wav.close)
        return av.AudioBurstAssembler(av.AudioTimeline(self.wav, 10_000_000_000, duration))

    def burst(self, t, end, count=5, value=1000):
        for i in range(count):
            # Observed G1 pattern: five 32 ms chunks published about .4 ms
            # apart, followed by about 160 ms until the next UDP payload.
            stamp = 10_000_000_000 + round((end - (count - 1 - i) * .0004) * 1e9)
            t.add(stamp, pcm(512, value))

    def test_g1_batched_publication_preserves_every_pcm_sample(self):
        t = self.make_timeline(1.6)
        for i in range(10):
            self.burst(t, (i + 1) * .16, value=1000+i)
        info = t.finish()
        self.assertEqual(0, info["audio_gap_s"])
        self.assertEqual(0, info["audio_overlap_trim_s"])
        self.wav.close(); self.raw.seek(0)
        with wave.open(self.raw, "rb") as f:
            self.assertEqual(b"".join(pcm(2560, 1000+i) for i in range(10)), f.readframes(25600))

    def test_host_scheduling_jitter_does_not_drop_or_pad_samples(self):
        t = self.make_timeline(1.6)
        for i in range(10):
            jitter = [0, .006, -.004, .010, -.007][i % 5]
            self.burst(t, (i + 1) * .16 + jitter)
        info = t.finish()
        self.assertEqual(0, info["audio_gap_s"])
        self.assertEqual(0, info["audio_overlap_trim_s"])
        self.assertGreater(info["audio_adjusted_bursts"], 0)
        self.assertLessEqual(info["audio_max_clock_adjust_s"], .020)

    def test_a_missing_32ms_chunk_is_still_visible(self):
        t = self.make_timeline(.48)
        self.burst(t, .16)
        self.burst(t, .32, count=4)
        self.burst(t, .48)
        self.assertAlmostEqual(.032, t.finish()["audio_gap_s"])

    def test_real_long_interruption_still_fails(self):
        t = self.make_timeline(1)
        self.burst(t, .16)
        self.burst(t, .80)
        with self.assertRaisesRegex(av.RecordingError, "gap exceeded"):
            t.finish()

    def test_repeated_short_losses_still_fail(self):
        t = self.make_timeline(1.6)
        for i in range(10):
            self.burst(t, (i + 1) * .16, count=4)
        with self.assertRaisesRegex(av.RecordingError, "10%"):
            t.finish()

    def test_duplicate_stamp_is_not_smoothed_away(self):
        t = self.make_timeline(1)
        t.add(10_032_000_000, pcm(512))
        with self.assertRaises(av.RecordingError):
            t.add(10_032_000_000, pcm(512))


class InputTests(unittest.TestCase):
    def test_pending_320ms_burst_is_not_a_microphone_interruption(self):
        with tempfile.TemporaryDirectory() as d:
            frames, info = collect_events(burst_capture_events(b"jpeg"), Path(d))
        self.assertEqual(10, len(frames))
        self.assertEqual(5*av.RATE, info["audio_samples"])
        self.assertLess(info["audio_gap_s"], .25)

    def test_real_audio_stop_is_detected_even_with_pending_pcm(self):
        events = [e for e in burst_capture_events(b"jpeg") if e[2] != "audio" or e[0] <= .32]
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(av.RecordingError) as error:
                collect_events(events, Path(d))
        self.assertEqual("AUDIO_INTERRUPTED", error.exception.code)
        self.assertIn("input idle", str(error.exception))

    def test_video_traffic_does_not_mask_missing_microphone(self):
        events = [(i*.001, i*.001, "video", b"jpeg") for i in range(600)]
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(av.RecordingError) as error:
                collect_events(events, Path(d))
        self.assertEqual("AUDIO_UNAVAILABLE", error.exception.code)

    def test_queue_overflow_is_explicit_not_a_silent_audio_drop(self):
        b = av.InputBuffer()
        with patch.object(av, "MAX_QUEUE_BYTES", 8):
            b.put("audio", time.time_ns(), b"12345678")
            b.put("audio", time.time_ns(), b"xx")
        with self.assertRaisesRegex(av.RecordingError, "buffer exceeded"):
            b.get(0)
        b.close()
        self.assertFalse(b.items)
        self.assertEqual(0, b.bytes)

    def test_invalid_source_clock_has_no_arrival_time_fallback(self):
        b = av.InputBuffer()
        b.put("video", 0, b"jpeg")
        with self.assertRaisesRegex(av.RecordingError, "ROS wall-clock"):
            b.get(0)

    def test_closed_queue_ignores_late_callbacks(self):
        b = av.InputBuffer()
        b.close()
        b.put("audio", time.time_ns(), b"xx")
        self.assertIsNone(b.get(0))

    def test_collect_missing_microphone_fails_promptly(self):
        class Clock:
            now = 0.0
            def monotonic(self):
                return self.now
        clock = Clock()
        class Buffer:
            first = True
            def get(self, timeout):
                if self.first:
                    self.first = False
                    return ("video", 10_000_000_000, b"jpeg", 0., 0.)
                clock.now += timeout
                return None
        with tempfile.TemporaryDirectory() as d, patch.object(av.time, "monotonic", clock.monotonic):
            with self.assertRaises(av.RecordingError) as raised:
                av.collect(Buffer(), Path(d), 30, threading.Event())
        self.assertEqual("AUDIO_UNAVAILABLE", raised.exception.code)
        self.assertLess(clock.now, 1)

    def test_collect_cancel_before_first_frame(self):
        cancel = threading.Event()
        cancel.set()
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(av.RecordingError) as raised:
                av.collect(av.InputBuffer(), Path(d), 1, cancel)
        self.assertEqual("RECORD_CANCELLED", raised.exception.code)

    def test_collect_uses_source_pts_and_preroll_audio(self):
        class Clock:
            now = 0.
            def monotonic(self):
                return self.now
        clock = Clock()
        origin = 10_000_000_000
        # The first audio packet arrives before the first JPEG callback, but
        # covers [0, .032] on the shared source clock.
        items = [("audio", origin + 32_000_000, pcm(512), .04, .008),
                 ("video", origin, b"jpeg", .04, .04)]
        items += [("audio", origin + i * 32_000_000, pcm(512), i * .032, 0.)
                  for i in range(2, 32)]
        items += [("video", origin + 900_000_000, b"jpeg", .995, .095)]
        class Buffer:
            def get(self, timeout):
                if items:
                    item = items.pop(0)
                    clock.now = max(clock.now, item[3])
                    return item
                clock.now += timeout
                return None
        with tempfile.TemporaryDirectory() as d, patch.object(av.time, "monotonic", clock.monotonic):
            frames, info = av.collect(Buffer(), Path(d), 1, threading.Event())
            self.assertEqual([0., .9], [t for t, _ in frames])
            self.assertEqual(16000, info["audio_samples"])
            self.assertAlmostEqual(.008, info["audio_gap_s"])
            self.assertEqual(2, len(list(Path(d).glob("*.jpg"))))

    def test_too_much_cumulative_audio_loss_fails(self):
        raw = io.BytesIO()
        with wave.open(raw, "wb") as wav:
            t = av.AudioTimeline(wav, 0, 1)
            for i in range(1, 11):
                t.add(i * 100_000_000, pcm(1280))  # 80 ms / 100 ms
            with self.assertRaisesRegex(av.RecordingError, "10%"):
                t.finish()

    def test_collect_g1_bursts_with_irregular_camera_frames(self):
        class Clock:
            now = 0.
            def monotonic(self): return self.now
        clock = Clock()
        events = [(0., "video", b"jpeg"), (.07, "video", b"jpeg"),
                  (.37, "video", b"jpeg"), (.83, "video", b"jpeg")]
        for batch in range(1, 7):
            for i in range(5):
                events.append((batch * .16 - (4-i) * .0004, "audio", pcm(512)))
        events.sort(key=lambda e: e[0])
        class Buffer:
            def get(self, timeout):
                if events and events[0][0] <= clock.now + timeout:
                    stamp, kind, data = events.pop(0)
                    clock.now = stamp
                    return kind, 10_000_000_000+round(stamp*1e9), data, stamp, 0.
                clock.now += timeout
                return None
        with tempfile.TemporaryDirectory() as d, patch.object(av.time, "monotonic", clock.monotonic):
            frames, info = av.collect(Buffer(), Path(d), 1, threading.Event())
            self.assertEqual([0., .07, .37, .83], [t for t, _ in frames])
            self.assertEqual(16000, info["audio_samples"])
            self.assertAlmostEqual(.04, info["audio_gap_s"])


class PluginTests(unittest.TestCase):
    def make_plugin(self):
        p = load_plugin()({}, "g1", object(), types.SimpleNamespace(
            is_running=lambda: True, _color_topic="/g1/camera/rgb"))
        tmp = tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup)
        p._output_dir = Path(tmp.name).resolve()
        return p

    def test_default_and_canvas_audio_parameter(self):
        p = self.make_plugin()
        schema = p.get_tool()["inputSchema"]
        self.assertIs(True, schema["properties"]["include_audio"]["default"])
        self.assertNotIn("include_audio", schema["x-action-params"]["record_video"]["params"])

    def test_non_boolean_audio_option_rejected(self):
        p = self.make_plugin()
        for value in [1, "false", None]:
            self.assertEqual("INVALID_AUDIO_OPTION", p._start_video_recording(
                {"duration_s": 1, "include_audio": value})["code"])

    def test_explicit_false_preserves_silent_option(self):
        p = self.make_plugin()
        with patch.object(shutil, "which", return_value="ffmpeg"), patch.object(threading, "Thread"):
            self.assertTrue(p._start_video_recording({"duration_s": 1, "include_audio": False})["ok"])
        self.assertIs(False, p._active_recording["include_audio"])

    def test_audio_option_routes_to_helper(self):
        p = self.make_plugin()
        helper = types.SimpleNamespace(record_av=lambda plugin, active: {"ok": True, "include_audio": True})
        with patch.dict(sys.modules, av_recording=helper):
            self.assertTrue(p._record_video({"include_audio": True})["include_audio"])

    def test_capture_error_destroys_only_recording_subscriber(self):
        p = self.make_plugin()
        node = types.SimpleNamespace(destroy_node=unittest.mock.Mock())
        executor = types.SimpleNamespace(add_node=unittest.mock.Mock(), remove_node=unittest.mock.Mock())
        p._executor = executor
        active = {"duration_s": 1, "cancel_event": threading.Event()}
        with tempfile.TemporaryDirectory() as d:
            p._output_dir = Path(d)
            with patch.object(av, "make_input_node", return_value=node), patch.object(av, "collect", side_effect=av.RecordingError("AUDIO_UNAVAILABLE", "no audio")):
                result = av.record_av(p, active)
            self.assertEqual("AUDIO_UNAVAILABLE", result["code"])
            self.assertEqual([], list((Path(d) / "videos").iterdir()))
        executor.remove_node.assert_called_once_with(node)
        node.destroy_node.assert_called_once()

    def test_encoding_failure_or_cancel_reaps_process_and_removes_partial(self):
        for mode in ("failure", "cancel", "timeout"):
            cancelled = mode == "cancel"
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as d:
                p = self.make_plugin()
                p._output_dir = Path(d)
                node = types.SimpleNamespace(destroy_node=unittest.mock.Mock())
                p._executor = types.SimpleNamespace(add_node=unittest.mock.Mock(), remove_node=unittest.mock.Mock())
                cancel = threading.Event()
                active = {"duration_s": 1, "cancel_event": cancel}
                process = types.SimpleNamespace(stdin=None, returncode=1,
                                                poll=lambda: 1 if mode == "failure" else None,
                                                terminate=unittest.mock.Mock(), wait=unittest.mock.Mock())
                def launch(*args, **kwargs):
                    Path(args[0][-1]).write_bytes(b"partial mp4")
                    if cancelled:
                        cancel.set()
                    return process
                with patch.object(av, "make_input_node", return_value=node), patch.object(
                        av, "collect", return_value=([(0., "000000.jpg")], {})), patch.object(
                        av.subprocess, "Popen", side_effect=launch), patch.object(
                        av, "prepare_frames", return_value=Path(d) / "%06d.jpg"), patch.object(
                        av, "ENCODE_TIMEOUT_S", 0 if mode == "timeout" else 30):
                    result = av.record_av(p, active)
                self.assertEqual({"cancel": "RECORD_CANCELLED", "timeout": "ENCODE_TIMEOUT",
                                  "failure": "AV_ENCODE_FAILED"}[mode], result["code"])
                self.assertIsNone(active["process"])
                self.assertEqual([], list((Path(d) / "videos").iterdir()))
                node.destroy_node.assert_called_once()
                if mode != "failure":
                    process.terminate.assert_called_once()
                    process.wait.assert_called_once()


@unittest.skipUnless(shutil.which("ffmpeg"), "FFmpeg not available locally")
class EncodingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name).resolve()
        self.jpeg = self.directory / "000000.jpg"
        subprocess.run(["ffmpeg", "-nostdin", "-v", "error", "-f", "lavfi", "-i",
                        "color=c=red:s=64x48", "-frames:v", "1", str(self.jpeg)], check=True)

    def encode(self, times, duration):
        self.require_nanosecond_input()
        for i in range(1, len(times)):
            (self.directory / ("%06d.jpg" % i)).write_bytes(self.jpeg.read_bytes())
        frames = [(t, "%06d.jpg" % i) for i, t in enumerate(times)]
        manifest = av.prepare_frames(self.directory, frames, duration)
        audio = self.directory / "audio.wav"
        # Impulse at exactly .25 s makes sample alignment observable after AAC.
        values = array("h", [int(1000 * math.sin(i * .15)) for i in range(round(duration * av.RATE))])
        values[4000:4016] = array("h", [30000] * 16)
        if sys.byteorder != "little":
            values.byteswap()
        with wave.open(str(audio), "wb") as f:
            f.setparams((1, 2, av.RATE, 0, "NONE", "not compressed"))
            f.writeframes(values.tobytes())
        output = self.directory / "output.mp4"
        subprocess.run(av.encoder_command(manifest, audio, output, duration), check=True)
        return output

    def require_nanosecond_input(self):
        probe = subprocess.run(["ffmpeg", "-nostdin", "-v", "error", "-f", "image2",
                                "-ts_from_file", "2", "-i", str(self.jpeg),
                                "-frames:v", "1", "-f", "null", "-"], capture_output=True)
        if b"nanosecond file timestamps unavailable" in probe.stderr:
            self.skipTest("This FFmpeg build lacks POSIX nanosecond timestamps; run on G1/Linux")
        self.assertEqual(0, probe.returncode, probe.stderr.decode("utf-8", "replace"))

    def test_vfr_keeps_irregular_frame_pts_and_audible_aac(self):
        output = self.encode([0, .08, .31, .9], 1.2)
        r = subprocess.run(["ffmpeg", "-nostdin", "-i", str(output), "-map", "0:v:0",
                            "-vf", "showinfo", "-f", "null", "-"], capture_output=True, text=True, check=True)
        pts = [float(v) for v in re.findall(r"\bpts_time:\s*([0-9.]+)", r.stderr)]
        self.assertEqual(5, len(pts))  # Four real frames + one final repeat.
        for actual, expected in zip(pts, [0, .08, .31, .9, 1.199]):
            self.assertAlmostEqual(expected, actual, places=3)
        self.assertRegex(r.stderr, r"Audio: aac")
        decoded = subprocess.run(["ffmpeg", "-nostdin", "-v", "error", "-i", str(output),
                                  "-map", "0:a:0", "-f", "s16le", "-acodec", "pcm_s16le", "-"],
                                 capture_output=True, check=True).stdout
        samples = array("h", decoded)
        if sys.byteorder != "little":
            samples.byteswap()
        peak = max(range(len(samples)), key=lambda i: abs(samples[i]))
        self.assertLess(abs(peak / av.RATE - .25), .02)
        self.assertGreaterEqual(len(samples), round(1.19 * av.RATE))

    def test_one_frame_recording_still_has_video_and_audio(self):
        output = self.encode([0], .5)
        r = subprocess.run(["ffmpeg", "-nostdin", "-i", str(output), "-f", "null", "-"],
                           capture_output=True, text=True, check=True)
        self.assertRegex(r.stderr, r"Video: h264")
        self.assertRegex(r.stderr, r"Audio: aac")
        self.assertIn("Duration: 00:00:00.50", r.stderr)

    def test_original_photo_and_silent_recording_still_work(self):
        p = load_plugin()({}, "g1", object(), None)
        p._output_dir = self.directory
        p._camera = object()
        p._frame = lambda *args: ({"data": self.jpeg.read_bytes(),
                                  "received_monotonic": time.monotonic()}, 1)
        photo = p._capture_photo()
        self.assertTrue(photo["ok"], photo)
        self.assertEqual(self.jpeg.read_bytes(), Path(photo["file_path"]).read_bytes())
        result = p._record_video({"duration_s": 1, "cancel_event": threading.Event()})
        self.assertTrue(result["ok"], result)
        info = subprocess.run(["ffmpeg", "-nostdin", "-i", result["file_path"],
                               "-f", "null", "-"], capture_output=True, text=True, check=True)
        self.assertIn("Video: h264", info.stderr)
        self.assertNotIn("Audio:", info.stderr)

    def test_record_av_success_releases_subscriber_before_encoding(self):
        self.require_nanosecond_input()
        node = types.SimpleNamespace(destroy_node=unittest.mock.Mock())
        executor = types.SimpleNamespace(add_node=unittest.mock.Mock(), remove_node=unittest.mock.Mock())
        p = load_plugin()({}, "g1", executor, types.SimpleNamespace(_color_topic="/g1/camera/rgb"))
        p._output_dir = self.directory
        active = {"duration_s": 1, "cancel_event": threading.Event()}
        jpeg = self.jpeg.read_bytes()
        def collect(buffer, directory, duration, cancel):
            (directory / "000000.jpg").write_bytes(jpeg)
            with wave.open(str(directory / "audio.wav"), "wb") as wav:
                t = av.AudioTimeline(wav, 0, 1)
                t.add(1_000_000_000, pcm(16000))
                info = t.finish()
            return [(0., "000000.jpg")], info
        command = av.encoder_command
        def checked_command(*args):
            node.destroy_node.assert_called_once()
            executor.remove_node.assert_called_once_with(node)
            self.assertEqual("encoding", active["state"])
            return command(*args)
        with patch.object(av, "make_input_node", return_value=node), patch.object(
                av, "collect", side_effect=collect), patch.object(av, "encoder_command", side_effect=checked_command):
            result = av.record_av(p, active)
        self.assertTrue(result["ok"], result)
        self.assertTrue(Path(result["file_path"]).is_file())
        self.assertIsNone(active["process"])
        self.assertEqual([Path(result["file_path"])], list((self.directory / "videos").iterdir()))

    def test_full_capture_and_encoding_with_320ms_bursts(self):
        self.require_nanosecond_input()
        node = types.SimpleNamespace(destroy_node=unittest.mock.Mock())
        executor = types.SimpleNamespace(add_node=unittest.mock.Mock(), remove_node=unittest.mock.Mock())
        p = load_plugin()({}, "g1", executor, types.SimpleNamespace(_color_topic="/g1/camera/rgb"))
        p._output_dir = self.directory
        active = {"duration_s": 5, "cancel_event": threading.Event()}
        events = burst_capture_events(self.jpeg.read_bytes())
        real_collect = av.collect
        def capture(buffer, directory, duration, cancel):
            # Exercise the real collect state machine with synthetic events,
            # not a mocked successful capture result. Restore real time before
            # FFmpeg's process timeout/cancellation loop is entered.
            with patch.object(av, "collect", real_collect):
                return collect_events(events, directory, duration)
        with patch.object(av, "make_input_node", return_value=node), patch.object(av, "collect", side_effect=capture):
            result = av.record_av(p, active)
        self.assertTrue(result["ok"], result)
        self.assertEqual(10, result["frames"])
        self.assertEqual(80000, result["audio_samples"])
        decoded = subprocess.run(["ffmpeg", "-nostdin", "-i", result["file_path"],
                                  "-f", "null", "-"], capture_output=True, text=True, check=True)
        self.assertIn("Audio: aac", decoded.stderr)
        self.assertIn("Video: h264", decoded.stderr)
        # AAC's final codec frame may pad the container duration slightly.
        seconds = float(re.search(r"Duration: 00:00:([0-9.]+)", decoded.stderr).group(1))
        self.assertAlmostEqual(5., seconds, delta=.08)


if __name__ == "__main__":
    unittest.main()
