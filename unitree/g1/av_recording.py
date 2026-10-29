"""Bounded, timestamped JPEG/PCM recording. No SDK or model dependencies.

G1 publishers stamp JPEGs on receipt and PCM chunks in a burst publication loop.
These are host ROS timestamps, NOT hardware exposure/sample timestamps. Audio
sample count supplies timing within each 16 kHz mono s16le publication burst.
"""

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


def prepare_frames(directory, frames, duration_s):
    """Expose source PTS through image2's nanosecond file timestamps.

    FFmpeg 4.4 lacks concat's per-file demuxer options. image2 ts_from_file=2
    is supported there and avoids the concat JPEG default 40 ms timebase.
    Files live only in the private spool; no shared camera files are touched.
    """
    end_tick = round(duration_s * 1000) - 1
    previous = -1
    for t, name in frames:
        tick = round(t * 1000)
        if tick <= previous or tick >= end_tick:
            raise RecordingError("INVALID_MEDIA_TIMESTAMP", "Video frames need distinct millisecond timestamps")
        stamp = 10_000_000_000 + tick * 1_000_000
        os.utime(directory / name, ns=(stamp, stamp))
        previous = tick
    # A single cheap repeat closes the final frame's presentation interval.
    last = directory / ("%06d.jpg" % len(frames))
    last.write_bytes((directory / frames[-1][1]).read_bytes())
    stamp = 10_000_000_000 + end_tick * 1_000_000
    os.utime(last, ns=(stamp, stamp))
    return directory / "%06d.jpg"


def encoder_command(images, audio, output, duration_s):
    return ["ffmpeg", "-nostdin", "-y", "-loglevel", "error",
            "-f", "image2", "-ts_from_file", "2", "-start_number", "0", "-i", str(images),
            "-i", str(audio), "-map", "0:v:0", "-map", "1:a:0",
            "-vsync", "vfr", "-enc_time_base:v", "1:1000",
            "-c:v", "libx264", "-preset", "ultrafast", "-threads", "2",
            # Set nominal codec metadata without imposing output frame cadence.
            "-x264-params", "fps=30/1:force-cfr=0",
            "-pix_fmt", "yuv420p", "-video_track_timescale", "1000000",
            "-c:a", "aac", "-b:a", "64k", "-t", str(duration_s),
            "-movflags", "+faststart", str(output)]


def make_input_node(video_topic, audio_topic, buffer):
    # Lazy imports keep timeline/encoding tests ROS- and hardware-free.
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import CompressedImage
    from audio_msgs.msg import AudioChunk

    node = Node("g1_av_capture")

    def receive(kind, msg):
        data = bytes(msg.data)
        if kind == "audio" and msg.format != "audio/pcm-16k":
            return
        if kind == "video" and not (data.startswith(b"\xff\xd8") and data.endswith(b"\xff\xd9")):
            return
        buffer.put(kind, int(msg.header.stamp.sec) * 1000000000
                   + int(msg.header.stamp.nanosec), data)

    try:
        for kind, msg_type, topic, depth in [
                ("video", CompressedImage, video_topic, 4),
                ("audio", AudioChunk, audio_topic, 64)]:
            qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
            node.create_subscription(msg_type, topic,
                                     lambda msg, k=kind: receive(k, msg), qos)
    except Exception:
        node.destroy_node()
        raise
    return node


def collect(buffer, directory, duration_s, cancel):
    """Spool JPEGs/PCM on a worker; all memory/disk use is bounded."""
    frames = []
    pre_audio = deque(maxlen=64)
    origin = origin_mono = timeline = None
    last_audio_received = None
    last_video_mono = time.monotonic()
    deadline = last_video_mono + 5.0
    total_bytes = 0
    with wave.open(str(directory / "audio.wav"), "wb") as wav:
        # A failure before the first video frame must still close a valid WAV.
        wav.setparams((1, 2, RATE, 0, "NONE", "not compressed"))
        while True:
            if cancel.is_set():
                raise RecordingError("RECORD_CANCELLED", "Video recording was cancelled")
            now = time.monotonic()
            if now >= deadline:
                break
            if timeline is not None:
                # A buffered burst has arrived even though it has not yet been
                # committed to the WAV. Measuring freshness from WAV position
                # incorrectly adds a full burst of latency (or two for batched
                # UDP delivery). Watch the actual ingress clock instead.
                since = last_audio_received if last_audio_received is not None else origin_mono
                idle_s = now - since
                budget_s = MAX_GAP_S + 0.1 + timeline.pending_duration_s
                if idle_s > budget_s:
                    code = "AUDIO_INTERRUPTED" if last_audio_received is not None else "AUDIO_UNAVAILABLE"
                    raise RecordingError(code, "No fresh microphone PCM during recording "
                                         "(input idle %.3fs, budget %.3fs, buffered %.3fs)" %
                                         (idle_s, budget_s, timeline.pending_duration_s))
            if frames and now - last_video_mono > 3.0:
                raise RecordingError("VIDEO_INTERRUPTED", "No camera frame received for 3 seconds")
            item = buffer.get(min(0.05, deadline - now))
            if item is None:
                continue
            kind, stamp_ns, data, received, age = item
            if kind == "audio":
                last_audio_received = received
            if origin is None:
                if kind == "audio":
                    pre_audio.append((stamp_ns, data))
                    continue
                origin = stamp_ns
                origin_mono = received - age
                deadline = origin_mono + duration_s
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
            if relative < 0 or relative >= duration_s - 0.001:
                continue
            if frames and relative <= frames[-1][0]:
                raise RecordingError("INVALID_MEDIA_TIMESTAMP", "Video timestamps moved backwards or repeated")
            total_bytes += len(data)
            if total_bytes > MAX_SPOOL_BYTES or len(frames) >= 1000:
                raise RecordingError("RECORD_SPOOL_LIMIT", "Recording temporary storage limit exceeded")
            name = "%06d.jpg" % len(frames)
            (directory / name).write_bytes(data)
            frames.append((relative, name))
            last_video_mono = received
        if not frames:
            raise RecordingError("VIDEO_UNAVAILABLE", "No timestamped RGB frame arrived within 5 seconds")
        if time.monotonic() - last_video_mono > 3.0:
            raise RecordingError("VIDEO_INTERRUPTED", "Camera stopped before recording ended")
        audio_info = timeline.finish()
    return frames, audio_info


def record_av(plugin, active):
    """Same H.264 encoder as silent recording, with VFR input and AAC muxing."""
    buffer = InputBuffer()
    node = process = path = None
    complete = False
    try:
        directory = plugin._output_dir / "videos"
        directory.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".av-", dir=str(directory)) as tmp:
            tmp = Path(tmp)
            try:
                node = make_input_node(plugin._camera._color_topic, plugin._audio_topic, buffer)
                plugin._executor.add_node(node)
                frames, audio_info = collect(buffer, tmp, active["duration_s"], active["cancel_event"])
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
            images = prepare_frames(tmp, frames, active["duration_s"])
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = directory / ("video_%s.mp4" % stamp)
            with path.open("xb"):
                pass
            with tempfile.TemporaryFile() as errors:
                with plugin._recording_lock:
                    active["path"] = str(path)
                    if active["cancel_event"].is_set():
                        raise RecordingError("RECORD_CANCELLED", "Video recording was cancelled")
                    process = subprocess.Popen(
                        encoder_command(images, tmp / "audio.wav", path, active["duration_s"]),
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
        complete = True
        return {"ok": True, "media_type": "video", "file_path": str(path),
                "include_audio": True, "video_timing": "vfr", "audio_codec": "aac",
                "timestamp_source": "ros_publish_header_pcm_burst_end",
                "recorded_duration_s": active["duration_s"], "frames": len(frames),
                "captured_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                **audio_info}
    except Exception as exc:
        return {"ok": False, "code": getattr(exc, "code", "AV_RECORD_FAILED"), "message": str(exc)}
    finally:
        plugin._terminate_encoder(process)
        with plugin._recording_lock:
            active["process"] = None
        if not complete and path is not None:
            plugin._remove_partial(path)
