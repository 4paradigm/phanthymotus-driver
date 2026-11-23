"""Bumi-local timestamped JPEG/PCM recording.

Audio timeline, burst handling and bounded buffering adapted from
unitree/g1/av_recording.py. This is private to Bumi, not shared infrastructure.
No robot SDK or model dependency. Video is sampled
onto an explicit CFR clock: dropped camera frames are repeated, never played
faster. Host receive timestamps provide software, not hardware, A/V sync.
"""
from __future__ import annotations

import shutil
import bisect

from array import array
from collections import deque
from datetime import datetime
from pathlib import Path
import os
import subprocess
import sys
import tempfile
import threading
import time
import wave


RATE = 16000
MAX_GAP_S = 0.25
ENCODE_TIMEOUT_S = 30
MAX_QUEUE_BYTES = 16 * 1024 * 1024
MAX_SPOOL_BYTES = 256 * 1024 * 1024


class RecordingError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class InputBuffer:
    """Callbacks copy bytes into a bounded queue; no file I/O or encoding."""

    def __init__(self):
        self.condition = threading.Condition()
        self.items = deque()
        self.bytes = 0
        self.error = None
        self.closed = False

    def put(self, kind, stamp_ns, data):
        received = time.monotonic()
        age = (time.time_ns() - stamp_ns) / 1e9
        with self.condition:
            if self.closed or self.error:
                return
            if stamp_ns <= 0 or not -0.1 <= age <= 2.0:
                self.error = RecordingError(
                    "INVALID_MEDIA_TIMESTAMP", "Media must use a fresh host ROS wall-clock stamp")
            elif kind == "audio" and (not data or len(data) % 2 or len(data) > RATE * 2):
                self.error = RecordingError(
                    "INVALID_AUDIO", "Expected bounded mono 16 kHz s16le PCM")
            elif len(self.items) >= 128 or self.bytes + len(data) > MAX_QUEUE_BYTES:
                self.error = RecordingError(
                    "RECORD_BUFFER_OVERFLOW", "Recording input buffer exceeded its limit")
            else:
                self.items.append((kind, stamp_ns, data, received, age))
                self.bytes += len(data)
            self.condition.notify_all()

    def get(self, timeout):
        with self.condition:
            if not self.items and not self.error and not self.closed:
                self.condition.wait(max(0, timeout))
            if self.error:
                raise self.error
            if not self.items:
                return None
            item = self.items.popleft()
            self.bytes -= len(item[2])
            return item

    def close(self):
        with self.condition:
            self.closed = True
            self.items.clear()
            self.bytes = 0
            self.condition.notify_all()


class AudioTimeline:
    """Place PCM on the video clock, trimming overlap and bounding gap fill."""

    def __init__(self, wav, origin_ns, duration_s):
        self.wav = wav
        self.origin_ns = origin_ns
        self.limit = round(duration_s * RATE)
        self.position = 0
        self.padded = 0
        self.trimmed = 0
        self.received = 0
        self.last_stamp = None
        self.has_signal = False
        wav.setparams((1, 2, RATE, 0, "NONE", "not compressed"))

    def _pad(self, count):
        if count > round(MAX_GAP_S * RATE):
            raise RecordingError("AUDIO_INTERRUPTED", "Microphone PCM gap exceeded 250 ms")
        self.wav.writeframesraw(b"\0\0" * count)
        self.position += count
        self.padded += count

    def add(self, stamp_ns, pcm):
        if not pcm or len(pcm) % 2 or len(pcm) > RATE * 2:
            raise RecordingError("INVALID_AUDIO", "Expected bounded mono 16 kHz s16le PCM")
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            raise RecordingError("INVALID_MEDIA_TIMESTAMP", "PCM timestamps moved backwards or repeated")
        self.last_stamp = stamp_ns
        samples = len(pcm) // 2
        start = round((stamp_ns - self.origin_ns) * RATE / 1e9) - samples
        end = min(start + samples, self.limit)
        if end <= 0 or start >= self.limit:
            return
        start_clip = max(start, 0)
        if start_clip > self.position:
            self._pad(start_clip - self.position)
        overlap = max(0, self.position - start_clip)
        if overlap > round(MAX_GAP_S * RATE):
            raise RecordingError("INVALID_MEDIA_TIMESTAMP", "PCM overlap exceeded 250 ms")
        begin = max(self.position, start_clip)
        if begin >= end:
            return
        chunk = pcm[(begin - start) * 2:(end - start) * 2]
        values = array("h", chunk)
        if sys.byteorder != "little":
            values.byteswap()
        self.has_signal = self.has_signal or any(values)
        self.wav.writeframesraw(chunk)
        self.received += len(chunk) // 2
        self.trimmed += overlap
        self.position = end

    def finish(self):
        if not self.received:
            raise RecordingError("AUDIO_UNAVAILABLE", "No microphone PCM covered the recording")
        self._pad(self.limit - self.position)
        if self.padded > self.limit * 0.10:
            raise RecordingError("AUDIO_INTERRUPTED", "More than 10% of audio samples were missing")
        if not self.has_signal:
            raise RecordingError("AUDIO_NO_SIGNAL", "Only zero PCM received; check microphone mute/capture")
        return {"audio_samples": self.position, "audio_gap_s": self.padded / RATE,
                "audio_overlap_trim_s": self.trimmed / RATE}


class AudioBurstAssembler:
    """Recover sample timing from G1's burst-published PCM chunks.

    A UDP packet contains several 32 ms PCM chunks. The Mic publisher stamps
    each chunk inside its split loop, so only the *last* stamp of a burst can
    approximate its sample end. Keep the PCM contiguous within that burst,
    and tolerate small host-clock scheduling jitter between complete bursts.
    Larger discontinuities still go through AudioTimeline's loss checks.
    """

    def __init__(self, timeline):
        self.timeline = timeline
        self.pending = bytearray()
        self.last_stamp = None
        self.adjusted_bursts = 0
        self.max_adjust_s = 0.

    @property
    def position(self):
        return self.timeline.position

    @property
    def received(self):
        return self.timeline.received

    @property
    def pending_duration_s(self):
        return len(self.pending) / (RATE * 2)

    def add(self, stamp_ns, pcm):
        if not pcm or len(pcm) % 2 or len(pcm) > RATE * 2:
            raise RecordingError("INVALID_AUDIO", "Expected bounded mono 16 kHz s16le PCM")
        if self.last_stamp is not None:
            delta = (stamp_ns - self.last_stamp) / 1e9
            if delta <= 0:
                raise RecordingError("INVALID_MEDIA_TIMESTAMP", "PCM timestamps moved backwards or repeated")
            if delta > 0.016:
                self.flush()
        if len(self.pending) + len(pcm) > RATE * 2:
            raise RecordingError("INVALID_AUDIO", "PCM publication burst exceeded one second")
        self.pending.extend(pcm)
        self.last_stamp = stamp_ns

    def flush(self):
        if not self.pending:
            return
        t = self.timeline
        samples = len(self.pending) // 2
        stamp = self.last_stamp
        start = round((stamp - t.origin_ns) * RATE / 1e9) - samples
        adjustment = start - t.position
        # Below a full 32 ms input chunk: do not hide a lost standard packet.
        # Preserve the sample clock instead of inserting/trimming on each tiny
        # scheduling fluctuation. The first burst still uses its source stamp.
        if t.received and abs(adjustment) <= round(.020 * RATE):
            stamp = t.origin_ns + round((t.position + samples) * 1e9 / RATE)
            if adjustment:
                self.adjusted_bursts += 1
                self.max_adjust_s = max(self.max_adjust_s, abs(adjustment) / RATE)
        t.add(stamp, bytes(self.pending))
        self.pending.clear()

    def finish(self):
        self.flush()
        return {**self.timeline.finish(),
                "audio_timestamp_mode": "burst_end_and_pcm_sample_clock",
                "audio_adjusted_bursts": self.adjusted_bursts,
                "audio_max_clock_adjust_s": self.max_adjust_s}


MAX_VIDEO_GAP_S = 1.0


def make_input_node(video_topic, audio_topic, buffer, include_audio):
    """Subscribe to existing producers; callbacks never encode or write files."""
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import CompressedImage
    from audio_msgs.msg import AudioChunk

    node = Node("av_capture_%s" % time.time_ns())

    def receive(kind, msg):
        if kind == "audio" and msg.format != "audio/pcm-16k":
            return
        data = bytes(msg.data)
        if kind == "video" and not (data.startswith(b"\xff\xd8") and data.endswith(b"\xff\xd9")):
            return
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        buffer.put(kind, stamp, data)

    inputs = [("video", CompressedImage, video_topic, 8)]
    if include_audio:
        inputs.append(("audio", AudioChunk, audio_topic, 64))
    try:
        for kind, msg_type, topic, depth in inputs:
            qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
            node.create_subscription(msg_type, topic,
                                     lambda msg, k=kind: receive(k, msg), qos)
    except Exception:
        node.destroy_node()
        raise
    return node


def collect(buffer, directory, duration_s, cancel, include_audio=True):
    """Use source time for media and monotonic time for bounded waits."""
    frames, pre_audio = [], deque(maxlen=64)
    origin = origin_mono = timeline = None
    last_audio_received = None
    last_video_mono = time.monotonic()
    deadline = last_video_mono + 5.0
    total_bytes = 0
    max_video_gap = 0.0
    with wave.open(str(directory / "audio.wav"), "wb") as wav:
        wav.setparams((1, 2, RATE, 0, "NONE", "not compressed"))
        while True:
            if cancel.is_set():
                raise RecordingError("RECORD_CANCELLED", "Video recording was cancelled")
            now = time.monotonic()
            if now >= deadline:
                break
            if include_audio and timeline is not None:
                since = last_audio_received if last_audio_received is not None else origin_mono
                budget = MAX_GAP_S + 0.1 + timeline.pending_duration_s
                if now - since > budget:
                    code = "AUDIO_INTERRUPTED" if last_audio_received is not None else "AUDIO_UNAVAILABLE"
                    raise RecordingError(code, "No fresh microphone PCM during recording")
            if frames and now - last_video_mono > MAX_VIDEO_GAP_S:
                raise RecordingError("VIDEO_INTERRUPTED", "Camera input stopped during recording")
            item = buffer.get(min(0.05, deadline - now))
            if item is None:
                continue
            kind, stamp_ns, data, received, age = item
            if kind == "audio":
                if not include_audio:
                    continue
                last_audio_received = received
            if origin is None:
                if kind == "audio":
                    pre_audio.append((stamp_ns, data))
                    continue
                origin = stamp_ns
                origin_mono = received - age
                deadline = origin_mono + duration_s
                if include_audio:
                    timeline = AudioBurstAssembler(AudioTimeline(wav, origin, duration_s))
                    for stamp, pcm in pre_audio:
                        timeline.add(stamp, pcm)
                    pre_audio.clear()
            relative = (stamp_ns - origin) / 1e9
            if abs(relative - (received - origin_mono)) > 0.5:
                raise RecordingError("INVALID_MEDIA_TIMESTAMP", "Media clock jumped or delivery exceeded 500 ms")
            if kind == "audio":
                timeline.add(stamp_ns, data)
                continue
            if relative < 0 or relative >= duration_s:
                continue
            if frames and relative <= frames[-1][0]:
                raise RecordingError("INVALID_MEDIA_TIMESTAMP", "Video timestamps moved backwards or repeated")
            if frames:
                gap = relative - frames[-1][0]
                if gap > MAX_VIDEO_GAP_S:
                    raise RecordingError("VIDEO_INTERRUPTED", "Camera frame gap exceeded one second")
                max_video_gap = max(max_video_gap, gap)
            total_bytes += len(data)
            if total_bytes > MAX_SPOOL_BYTES or len(frames) >= 1000:
                raise RecordingError("RECORD_SPOOL_LIMIT", "Recording temporary storage limit exceeded")
            name = "%06d.jpg" % len(frames)
            (directory / name).write_bytes(data)
            frames.append((relative, name))
            last_video_mono = received
        if not frames:
            raise RecordingError("VIDEO_UNAVAILABLE", "No timestamped RGB frame arrived within 5 seconds")
        if time.monotonic() - last_video_mono > MAX_VIDEO_GAP_S:
            raise RecordingError("VIDEO_INTERRUPTED", "Camera stopped before recording ended")
        info = timeline.finish() if include_audio else {}
    return frames, {**info, "max_video_frame_gap_s": round(max_video_gap, 6)}


def prepare_frames(directory, frames, duration_s, fps, cancel):
    """Select the latest source frame for every output tick, using cheap links.

    This avoids a fixed-rate MJPEG pipe accelerating a recording when fewer
    source frames arrive. No future frame is moved earlier. At 15 fps the
    additional quantization is <67 ms. Works with FFmpeg 4.4 and on Windows.
    """
    if not frames or frames[0][0] != 0:
        raise RecordingError("INVALID_MEDIA_TIMESTAMP", "The first frame must define video time zero")
    times = [round(t * 1e9) for t, _ in frames]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise RecordingError("INVALID_MEDIA_TIMESTAMP", "Video timestamps must increase")
    timed = directory / "timed"
    timed.mkdir()
    count = round(duration_s * fps)
    used = set()
    copied_bytes = 0
    for index in range(count):
        if cancel.is_set():
            raise RecordingError("RECORD_CANCELLED", "Video recording was cancelled")
        source_index = bisect.bisect_right(times, round(index * 1e9 / fps)) - 1
        used.add(source_index)
        source = directory / frames[source_index][1]
        target = timed / ("%06d.jpg" % index)
        try:
            os.link(source, target)
        except OSError:
            copied_bytes += source.stat().st_size
            if copied_bytes > MAX_SPOOL_BYTES:
                raise RecordingError("RECORD_SPOOL_LIMIT", "Timed frame copies exceeded the storage limit")
            shutil.copyfile(source, target)
    return timed / "%06d.jpg", {"encoded_frames": count,
                                "repeated_frames": count - len(used),
                                "video_fps": fps, "video_timing": "source_timestamps_to_cfr"}


def encoder_command(images, audio, output, duration_s, fps):
    command = ["ffmpeg", "-nostdin", "-y", "-loglevel", "error", "-framerate", str(fps),
               "-f", "image2", "-start_number", "0", "-i", str(images)]
    if audio is not None:
        command += ["-i", str(audio), "-map", "0:v:0", "-map", "1:a:0", "-c:a", "aac", "-b:a", "64k"]
    else:
        command += ["-an"]
    return command + ["-c:v", "libx264", "-preset", "ultrafast", "-threads", "2",
                      "-pix_fmt", "yuv420p", "-t", str(duration_s), "-movflags", "+faststart", str(output)]


def record(plugin, active):
    """Capture, release subscribers, then encode and commit one owned MP4."""
    buffer = InputBuffer()
    node = process = path = None
    complete = False
    include_audio = active["include_audio"]
    try:
        directory = plugin._output_dir / "videos"
        directory.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".av-", dir=str(directory)) as tmp:
            tmp = Path(tmp)
            try:
                node = make_input_node(plugin._video_topic, plugin._audio_topic, buffer, include_audio)
                plugin._executor.add_node(node)
                frames, audio_info = collect(buffer, tmp, active["duration_s"],
                                             active["cancel_event"], include_audio)
            finally:
                buffer.close()
                if node is not None:
                    try:
                        plugin._executor.remove_node(node)
                    finally:
                        node.destroy_node()
            with plugin._recording_lock:
                if active["cancel_event"].is_set():
                    raise RecordingError("RECORD_CANCELLED", "Video recording was cancelled")
                active["state"] = "encoding"
            images, video_info = prepare_frames(tmp, frames, active["duration_s"],
                                                 plugin._fps, active["cancel_event"])
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = directory / ("video_%s.mp4" % stamp)
            with path.open("xb"):
                pass
            with tempfile.TemporaryFile() as errors:
                try:
                    with plugin._recording_lock:
                        active["path"] = str(path)
                        if active["cancel_event"].is_set():
                            raise RecordingError("RECORD_CANCELLED", "Video recording was cancelled")
                        process = subprocess.Popen(encoder_command(
                            images, tmp / "audio.wav" if include_audio else None,
                            path, active["duration_s"], plugin._fps),
                            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=errors)
                        active["process"] = process
                    encode_deadline = time.monotonic() + ENCODE_TIMEOUT_S
                    while True:
                        if active["cancel_event"].wait(0.05):
                            raise RecordingError("RECORD_CANCELLED", "Video recording was cancelled")
                        if time.monotonic() >= encode_deadline:
                            raise RecordingError("ENCODE_TIMEOUT", "FFmpeg exceeded the 30-second encode budget")
                        if process.poll() is not None:
                            if process.returncode != 0 or not path.stat().st_size:
                                errors.seek(0)
                                raise RecordingError("AV_ENCODE_FAILED", errors.read(4096).decode("utf-8", "replace"))
                            break
                finally:
                    # Reap FFmpeg BEFORE spool/error-log context managers close.
                    # In particular Windows cannot remove files it still reads.
                    plugin._terminate_encoder(process)
                    process = None
                    with plugin._recording_lock:
                        active["process"] = None
        if active["cancel_event"].is_set():
            raise RecordingError("RECORD_CANCELLED", "Video recording was cancelled")
        complete = True
        return {"ok": True, "media_type": "video", "file_path": str(path),
                "include_audio": include_audio, "audio_codec": "aac" if include_audio else None,
                "timestamp_source": "host_camera_publish_and_mic_capture_end",
                "synchronization": "software", "recorded_duration_s": video_info["encoded_frames"] / plugin._fps,
                "frames": len(frames), "captured_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                **audio_info, **video_info}
    except Exception as exc:
        return {"ok": False, "code": getattr(exc, "code", "AV_RECORD_FAILED"), "message": str(exc)}
    finally:
        plugin._terminate_encoder(process)
        with plugin._recording_lock:
            active["process"] = None
        if not complete and path is not None:
            try:
                path.unlink(missing_ok=True)
            except OSError as exc:
                return {"ok": False, "code": "CLEANUP_FAILED", "message": str(exc)}
