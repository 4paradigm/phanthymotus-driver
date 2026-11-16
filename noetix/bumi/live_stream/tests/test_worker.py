"""Bounded queue, preflight, lifecycle and FFmpeg argument contract."""
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

    worker = LiveStreamWorker(Camera(), audio, popen=reject_before_stream)
    worker._status = "starting"  # _run is normally invoked by start().
    worker._run("rtmp://tx.direct.huya.com/huyalive/private", threading.Event())
    cmd = called["cmd"]
    assert cmd.count("-i") == 2
    assert "mjpeg" in cmd
    assert "s16le" in cmd
    assert "libx264" in cmd
    assert "aac" in cmd
    assert worker.info()["state"] == "error"
    assert worker.info()["error_code"] == "ENCODER_START_FAILED"
    assert "private" not in str(worker.info())
