"""Bounded queue, lifecycle, failure propagation and secret-free subprocess."""
import os
import subprocess
import time
import threading
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from live_stream.worker import AudioBuffer, LiveStreamWorker


class Camera:
    def __init__(self, running=True):
        self.running = running

    def is_running(self):
        return self.running


class IdleWorker(LiveStreamWorker):
    def _run(self, url, cancel):
        cancel.wait(timeout=5)


def test_audio_queue_drops_oldest_and_rejects_invalid_samples():
    buf = AudioBuffer(capacity=2)
    assert buf.push(b"\x00\x01", "audio/pcm-16k")
    assert buf.push(b"\x02\x03", "pcm_16k_16bit_mono")
    assert buf.push(b"\x04\x05", "audio/pcm-16k")
    assert buf.dropped == 1
    assert buf.queue.get_nowait() == b"\x02\x03"
    assert not buf.push(b"\x01", "audio/pcm-16k")
    assert not buf.push(b"\x00\x00", "audio/mp3")
    assert buf.fresh()
    buf.clear()
    assert buf.queue.empty()


def test_worker_preflight_and_idempotent_stop():
    audio = AudioBuffer()
    assert LiveStreamWorker(Camera(False), audio).start("unused")["error_code"] == "CAMERA_UNAVAILABLE"
    worker = LiveStreamWorker(Camera(True), audio)
    assert worker.start("unused")["error_code"] == "MIC_UNAVAILABLE"
    audio.push(b"\x00\x00", "audio/pcm-16k")
    worker = IdleWorker(Camera(True), audio)
    assert worker.start("unused")["state"] == "starting"
    assert worker.start("unused")["state"] == "starting"
    assert worker.stop()["state"] == "idle"
    assert worker.stop()["state"] == "idle"


def test_encoder_command_uses_one_mjpeg_and_one_pcm_input():
    audio = AudioBuffer()
    audio.push(b"\x00\x00", "audio/pcm-16k")
    called = {}

    def reject_before_stream(cmd, **kwargs):
        called["cmd"] = cmd
        raise OSError("simulated FFmpeg startup failure")

    class FakePublisher:
        def __init__(self, url):
            pass
        def connect(self):
            pass
        def close(self):
            pass

    worker = LiveStreamWorker(Camera(), audio, popen=reject_before_stream,
                              publisher_factory=FakePublisher)
    worker._status = "starting"  # _run is normally invoked by start().
    worker._run("rtmp://tx.direct.huya.com/huyalive/private", threading.Event())
    cmd = called["cmd"]
    assert cmd.count("-i") == 2
    assert "mjpeg" in cmd
    assert "s16le" in cmd
    assert "libx264" in cmd
    assert "aac" in cmd
    assert cmd[-1] == "pipe:1"
    assert not any("private" in str(item) for item in cmd)
    assert "stdout" not in str(worker.info())
    assert worker.info()["state"] == "error"
    assert worker.info()["error_code"] == "ENCODER_START_FAILED"
    assert "private" not in str(worker.info())

class FailingCamera(Camera):
    def wait_for_color_frame(self, **kwargs):
        raise TimeoutError("simulated camera disconnect")


class FakePublisher:
    def __init__(self, url):
        self.url = url
        self.connected = False
        self.closed = False

    def connect(self):
        self.connected = True

    def send(self, record):
        pass

    def close(self):
        self.closed = True


def test_video_writer_failure_is_error_not_normal_idle(monkeypatch):
    # Windows select cannot poll pipes; emulate an idle readable set. The
    # failure itself originates in the camera writer, not this poll.
    import live_stream.worker as module
    monkeypatch.setattr(module.select, "select",
                        lambda readable, writable, errors, timeout: ([], [], []))
    audio = AudioBuffer()
    audio.push(b"\x00\x00", "pcm_16k_16bit_mono")

    spawned = []
    publishers = []

    def spawn_encoder(cmd, **options):
        assert cmd[-1] == "pipe:1"
        assert not any("private-key" in str(arg) for arg in cmd)
        proc = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(20)"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        spawned.append(proc)
        return proc

    def publisher_factory(url):
        pub = FakePublisher(url)
        publishers.append(pub)
        return pub

    worker = LiveStreamWorker(FailingCamera(), audio, popen=spawn_encoder,
                              publisher_factory=publisher_factory)
    assert worker.start("rtmp://tx.direct.huya.com/huyalive/private-key")["state"] == "starting"
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline and worker.info()["state"] in ("starting", "streaming"):
        time.sleep(0.025)
    assert worker.info()["state"] == "error"
    assert worker.info()["error_code"] == "VIDEO_PIPE_FAILED"
    assert "private-key" not in str(worker.info())
    assert publishers[0].closed
    assert spawned[0].poll() is not None
    assert worker.stop()["state"] == "idle"

def test_audio_writer_failure_is_error_not_idle(monkeypatch):
    import queue
    import live_stream.worker as module
    monkeypatch.setattr(module.select, "select",
                        lambda readable, writable, errors, timeout: ([], [], []))

    class DisconnectedAudio:
        def __init__(self):
            self.calls = 0
            self.dropped = 0
            self.queue = queue.Queue()

        def fresh(self, max_age=4):
            self.calls += 1
            return self.calls == 1

        def clear(self):
            pass

    class CameraFrames(Camera):
        def wait_for_color_frame(self, **kwargs):
            return {"data": b"\xff\xd8\xff\xd9"}, 1

    def spawn(cmd, **options):
        return subprocess.Popen(
            [sys.executable, "-c", "import time;time.sleep(20)"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    worker = LiveStreamWorker(CameraFrames(), DisconnectedAudio(),
                              popen=spawn, publisher_factory=FakePublisher)
    assert worker.start("rtmp://tx.direct.huya.com/huyalive/test")["state"] == "starting"
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline and worker.info()["state"] in ("starting", "streaming"):
        time.sleep(0.025)
    assert worker.info()["state"] == "error"
    assert worker.info()["error_code"] == "AUDIO_PIPE_FAILED"
    worker.stop()
