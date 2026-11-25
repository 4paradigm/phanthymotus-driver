"""Record Bumi's timestamped JPEG and PCM streams on one media clock."""
from __future__ import annotations

from collections import deque
from contextlib import nullcontext
from datetime import datetime
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import wave


RATE = 16000
MAX_GAP_S = 0.25
MAX_VIDEO_GAP_NS = 1_000_000_000
NS_PER_SECOND = 1_000_000_000
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
                self.error = RecordingError("INVALID_AUDIO", "Expected bounded mono 16 kHz s16le PCM")
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
    """Place complete Bumi PCM blocks on the video clock."""

    def __init__(self, wav, duration_s):
        self.wav = wav
        self.limit = round(duration_s * RATE)
        self.position = 0
        self.padded = 0
        self.trimmed = 0
        self.received = 0
        self.last_stamp = None
        self.has_signal = False
        self.adjusted_chunks = 0
        self.max_adjust_samples = 0
        wav.setparams((1, 2, RATE, 0, "NONE", "not compressed"))

    def _pad(self, count):
        if count > round(MAX_GAP_S * RATE):
            raise RecordingError("AUDIO_INTERRUPTED", "Microphone PCM gap exceeded 250 ms")
        self.wav.writeframesraw(b"\0\0" * count)
        self.position += count
        self.padded += count

    def add(self, stamp_ns, pcm):
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            raise RecordingError("INVALID_MEDIA_TIMESTAMP", "PCM timestamps moved backwards or repeated")
        self.last_stamp = stamp_ns
        samples = len(pcm) // 2
        start = round(stamp_ns * RATE / NS_PER_SECOND) - samples
        # Smooth host scheduling jitter below the mic's minimum 32 ms block.
        adjustment = abs(start - self.position)
        if self.received and 0 < adjustment <= round(0.020 * RATE):
            start = self.position
            self.adjusted_chunks += 1
            self.max_adjust_samples = max(self.max_adjust_samples, adjustment)
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
        self.has_signal = self.has_signal or any(chunk)
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
                "audio_overlap_trim_s": self.trimmed / RATE,
                "audio_timestamp_mode": "chunk_end_and_pcm_sample_clock",
                "audio_adjusted_chunks": self.adjusted_chunks,
                "audio_max_clock_adjust_s": self.max_adjust_samples / RATE}


class VideoTimeline:
    """Write the most recent JPEG at each output frame tick."""

    def __init__(self, stream, fps, duration_s):
        self.stream = stream
        self.fps = fps
        self.limit = duration_s * fps
        self.source_frames = self.encoded_frames = self.used_frames = self.size = 0
        self.last_stamp = self.frame = None
        self.max_gap_ns = 0

    def _write_until(self, end):
        count = end - self.encoded_frames
        if not count:
            return
        self.size += count * len(self.frame)
        if self.size > MAX_SPOOL_BYTES:
            raise RecordingError("RECORD_SPOOL_LIMIT", "Recording temporary storage limit exceeded")
        for _ in range(count):
            self.stream.write(self.frame)
        self.encoded_frames = end
        self.used_frames += 1

    def add(self, stamp_ns, jpeg):
        if self.last_stamp is not None:
            gap = stamp_ns - self.last_stamp
            if gap <= 0:
                raise RecordingError("INVALID_MEDIA_TIMESTAMP", "Video timestamps moved backwards or repeated")
            if gap > MAX_VIDEO_GAP_NS:
                raise RecordingError("VIDEO_INTERRUPTED", "Camera frame gap exceeded one second")
            self.max_gap_ns = max(self.max_gap_ns, gap)
            # Compare ticks at the same nanosecond precision as ROS stamps.
            tick = stamp_ns * self.fps // NS_PER_SECOND
            if round(tick * NS_PER_SECOND / self.fps) < stamp_ns:
                tick += 1
            self._write_until(tick)
        self.frame, self.last_stamp = jpeg, stamp_ns
        self.source_frames += 1

    def finish(self):
        if not self.source_frames:
            raise RecordingError("VIDEO_UNAVAILABLE", "No timestamped RGB frame arrived within 5 seconds")
        self._write_until(self.limit)
        return {"frames": self.source_frames, "encoded_frames": self.encoded_frames,
                "repeated_frames": self.encoded_frames - self.used_frames, "video_fps": self.fps,
                "max_video_frame_gap_s": round(self.max_gap_ns / NS_PER_SECOND, 6),
                "video_timing": "source_timestamps_to_cfr"}


def make_input_node(video_topic, audio_topic, buffer):
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
    if audio_topic is not None:
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


def collect(buffer, directory, duration_s, cancel, include_audio, fps):
    """Capture both streams relative to the first camera frame."""
    pre_audio = deque(maxlen=64)
    origin = origin_mono = last_audio = last_video = None
    deadline = time.monotonic() + 5.0
    with (directory / "video.mjpeg").open("wb") as stream, (
        wave.open(str(directory / "audio.wav"), "wb") if include_audio else nullcontext()
    ) as wav:
        video = VideoTimeline(stream, fps, duration_s)
        audio = AudioTimeline(wav, duration_s) if include_audio else None
        while (now := time.monotonic()) < deadline:
            check_cancelled(cancel)
            if origin is not None:
                if now - last_video > MAX_VIDEO_GAP_NS / NS_PER_SECOND:
                    raise RecordingError("VIDEO_INTERRUPTED", "Camera input stopped during recording")
                if include_audio and now - (last_audio if last_audio is not None else origin_mono) > MAX_GAP_S + 0.1:
                    code = "AUDIO_INTERRUPTED" if last_audio is not None else "AUDIO_UNAVAILABLE"
                    raise RecordingError(code, "No fresh microphone PCM during recording")
            item = buffer.get(min(0.05, deadline - now))
            if item is None:
                continue
            kind, stamp_ns, data, received, age = item
            if kind == "audio":
                if audio is None:
                    continue
                last_audio = received
                if origin is None:
                    pre_audio.append((stamp_ns, data))
                    continue
            if origin is None:
                origin, origin_mono = stamp_ns, received - age
                deadline = origin_mono + duration_s
                for stamp, pcm in pre_audio:
                    audio.add(stamp - origin, pcm)
                pre_audio.clear()
            relative_ns = stamp_ns - origin
            if abs(relative_ns / NS_PER_SECOND - (received - origin_mono)) > 0.5:
                raise RecordingError("INVALID_MEDIA_TIMESTAMP", "Media clock jumped or delivery exceeded 500 ms")
            if kind == "audio":
                audio.add(relative_ns, data)
            elif 0 <= relative_ns < duration_s * NS_PER_SECOND:
                video.add(relative_ns, data)
                last_video = received
        check_cancelled(cancel)
        if last_video is not None and deadline - last_video > MAX_VIDEO_GAP_NS / NS_PER_SECOND:
            raise RecordingError("VIDEO_INTERRUPTED", "Camera stopped before recording ended")
        return {**video.finish(), **(audio.finish() if audio is not None else {})}


def check_cancelled(cancel):
    if cancel.is_set():
        raise RecordingError("RECORD_CANCELLED", "Video recording was cancelled")


def encode(directory, include_audio, fps, cancel):
    output = directory / "video.mp4"
    command = ["ffmpeg", "-nostdin", "-loglevel", "error", "-framerate", str(fps),
               "-f", "mjpeg", "-i", str(directory / "video.mjpeg")]
    if include_audio:
        command += ["-i", str(directory / "audio.wav"), "-map", "0:v:0", "-map", "1:a:0",
                    "-c:a", "aac", "-b:a", "64k"]
    command += ["-c:v", "libx264", "-preset", "ultrafast", "-threads", "2",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output)]
    check_cancelled(cancel)
    with tempfile.TemporaryFile() as errors:
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=errors)
        deadline = time.monotonic() + ENCODE_TIMEOUT_S
        try:
            while process.poll() is None:
                cancel.wait(0.05)
                check_cancelled(cancel)
                if time.monotonic() >= deadline:
                    raise RecordingError("ENCODE_TIMEOUT", "FFmpeg exceeded the 30-second encode budget")
            if process.returncode != 0 or not output.is_file() or not output.stat().st_size:
                errors.seek(0)
                raise RecordingError("AV_ENCODE_FAILED", errors.read(4096).decode("utf-8", "replace"))
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
    return output


def record(*, executor, video_topic, audio_topic, directory, duration_s, fps, cancel, on_encoding):
    """Capture, release subscribers and publish the encoded file atomically."""
    include_audio = audio_topic is not None
    check_cancelled(cancel)
    directory.mkdir(parents=True, exist_ok=True)
    buffer = InputBuffer()
    with tempfile.TemporaryDirectory(prefix=".av-", dir=directory) as name:
        tmp = Path(name)
        node = make_input_node(video_topic, audio_topic, buffer)
        try:
            executor.add_node(node)
            try:
                info = collect(buffer, tmp, duration_s, cancel, include_audio, fps)
            finally:
                buffer.close()
                executor.remove_node(node)
        finally:
            node.destroy_node()
        check_cancelled(cancel)
        on_encoding()
        output = encode(tmp, include_audio, fps, cancel)
        check_cancelled(cancel)
        path = directory / datetime.now().strftime("video_%Y%m%d_%H%M%S_%f.mp4")
        os.link(output, path)
    return {"ok": True, "media_type": "video", "file_path": str(path),
            "include_audio": include_audio, "audio_codec": "aac" if include_audio else None,
            "timestamp_source": "host_camera_publish_and_mic_capture_end",
            "synchronization": "software", "recorded_duration_s": info["encoded_frames"] / fps,
            "captured_at": datetime.now().astimezone().isoformat(timespec="seconds"), **info}
