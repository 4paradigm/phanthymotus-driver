"""R1 vision capture: photo persistence and audible video verification."""

import importlib
import json
import math
import struct
import subprocess
import threading
import wave
from pathlib import Path

import pytest


def _capture_module():
    return importlib.import_module("unitree.r1.vision_capture")


def test_pcm_timeline_preserves_audible_samples_and_gaps(tmp_path):
    capture = _capture_module()
    tone = b"".join(struct.pack("<h", int(12000 * math.sin(2 * math.pi * 440 * n / 16000)))
                    for n in range(1600))
    path = tmp_path / "sound.wav"
    capture.write_audio_timeline(path, [(10.1, tone), (10.3, tone)], 10.0, 0.5)
    with wave.open(str(path), "rb") as audio:
        samples = audio.readframes(audio.getnframes())
        assert audio.getframerate() == 16000
        assert audio.getnchannels() == 1
        assert audio.getnframes() == 8000
    assert any(samples)
    assert samples[4000:6000] == bytes(2000)


def test_silent_or_missing_audio_is_rejected(tmp_path):
    capture = _capture_module()
    with pytest.raises(ValueError, match="audio"):
        capture.write_audio_timeline(tmp_path / "missing.wav", [], 10.0, 1)
    with pytest.raises(ValueError, match="audio"):
        capture.write_audio_timeline(tmp_path / "flat.wav", [(10.1, bytes(3200))], 10.0, 1)
    with pytest.raises(ValueError, match="audio"):
        capture.write_audio_timeline(
            tmp_path / "flat_nonzero.wav", [(10.1, struct.pack("<h", 120) * 1600)],
            10.0, 1)


def test_encoded_video_contains_video_and_audio(tmp_path):
    capture = _capture_module()
    ffmpeg = capture.require_encoder()
    jpeg = subprocess.check_output([
        ffmpeg, "-v", "error", "-f", "lavfi", "-i", "color=c=blue:s=64x64",
        "-frames:v", "1", "-f", "image2pipe", "-vcodec", "mjpeg", "pipe:1",
    ])
    frames = tmp_path / "frames.mjpeg"
    frames.write_bytes(jpeg * 5)
    tone = b"".join(struct.pack("<h", int(12000 * math.sin(2 * math.pi * 440 * n / 16000)))
                    for n in range(16000))
    audio = tmp_path / "audio.wav"
    capture.write_audio_timeline(audio, [(1.0, tone)], 0.0, 1.0)
    output = tmp_path / "result.mp4"
    capture.encode_video(frames, audio, output, fps=5, duration_s=1)
    metadata = json.loads(subprocess.check_output([
        "ffprobe", "-v", "error", "-show_streams", "-of", "json", str(output),
    ]))
    assert {s["codec_type"] for s in metadata["streams"]} == {"video", "audio"}
    assert output.stat().st_size > 1000


def test_photo_saves_next_camera_frame(tmp_path):
    capture = _capture_module()
    plugin = object.__new__(capture.VisionCapturePlugin)
    plugin._condition = threading.Condition()
    plugin._frame = None
    plugin._frame_sequence = 0
    plugin._output_dir = tmp_path
    jpeg = b"\xff\xd8example\xff\xd9"
    timer = threading.Timer(0.05, plugin._on_frame, args=(
        type("Image", (), {"format": "jpeg", "data": jpeg})(),))
    timer.start()
    try:
        result = plugin._photo()
    finally:
        timer.join()
    assert result["ok"] is True
    assert Path(result["file_path"]).read_bytes() == jpeg


def test_r1_manifest_lists_capture_card():
    text = (Path(__file__).resolve().parents[1] / "driver.yaml").read_text(encoding="utf-8")
    assert "name: vision_capture" in text
