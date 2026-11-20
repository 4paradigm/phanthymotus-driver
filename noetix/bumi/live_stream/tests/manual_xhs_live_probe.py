"""Explicit real Xiaohongshu RTMP probe with SYNTHETIC video/audio.

Manual test only; never imported by pytest. No Bumi camera or microphone.
Reads private full RTMP URL from a file and never includes it in logs or argv.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from live_stream import LiveStreamPlugin
from live_stream.config import validate_stream_url
from live_stream.worker import AudioBuffer, LiveStreamWorker


class MemoryStore:
    def __init__(self):
        self.url = None

    def read(self):
        return self.url

    def save(self, url):
        self.url = url


class SyntheticCamera:
    def __init__(self):
        self.sequence = 0
        self.jpeg = subprocess.check_output([
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "lavfi", "-i", "testsrc=size=320x180:rate=15",
            "-frames:v", "1", "-c:v", "mjpeg", "-f", "image2pipe", "pipe:1",
        ], timeout=12)

    def is_running(self):
        return True

    def wait_for_color_frame(self, **kwargs):
        self.sequence += 1
        return {"data": self.jpeg}, self.sequence


def feed_audio(buffer: AudioBuffer, stop: threading.Event):
    packet = struct.pack("<640h", *(
        int(3500 * math.sin(2 * math.pi * 500 * n / 16000))
        for n in range(640)))
    while not stop.is_set():
        buffer.push(packet, "audio/pcm-16k")
        stop.wait(0.04)


def run(path: Path, seconds: int):
    if os.environ.get("ALLOW_XHS_LIVE_TEST") != "1":
        raise RuntimeError("Set ALLOW_XHS_LIVE_TEST=1 to permit a real external stream")
    url = path.read_text(encoding="utf-8").strip()
    platform, url = validate_stream_url(url)
    if platform != "xiaohongshu":
        raise RuntimeError("Only the Xiaohongshu stream may be tested here")
    audio = AudioBuffer()
    stopped = threading.Event()
    source = threading.Thread(target=feed_audio, args=(audio, stopped), daemon=True)
    source.start()
    worker = LiveStreamWorker(SyntheticCamera(), audio, fps=15, bitrate_kbps=350)
    # This standalone, explicitly opt-in MANUAL probe runs only synthetic
    # media in this process. Inject a test-only verifier because it does not
    # contact the authenticated Agent Core. Production Driver uses its normal
    # signed-approval verifier and remains fail-closed.
    card = LiveStreamPlugin({}, "sim", None, None,
                            store=MemoryStore(), audio=audio, worker=worker,
                            approval_verifier=lambda *_: True)
    streamed = False
    state = "not_started"
    error = None
    published_records = 0
    video_frames = 0
    audio_chunks = 0
    try:
        configured = card.dispatch("config", {"rtmp_url": url})
        if not configured.get("ok"):
            error = configured.get("error_code", "CONFIG_FAILED")
        else:
            result = card.dispatch("go_live", {})
            if result.get("state") != "starting":
                error = result.get("error_code", "START_FAILED")
            else:
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    status = card.dispatch("info", {})
                    state = status["state"]
                    error = status.get("error_code")
                    published_records = status["published_records"]
                    video_frames = status["video_frames"]
                    audio_chunks = status["audio_chunks"]
                    streamed = streamed or (
                        state == "streaming" and published_records > 5
                        and video_frames > 5 and audio_chunks > 5)
                    if state == "error":
                        break
                    time.sleep(0.2)
    finally:
        ended = card.dispatch("end_live", {})
        stopped.set()
        source.join(timeout=1)
        # No credentials in the output, subprocess argv, or tracked files.
        print(json.dumps({
            "platform": "xiaohongshu", "streamed": streamed,
            "last_state": state, "error_code": error,
            "video_frames": video_frames,
            "audio_chunks": audio_chunks,
            "published_records": published_records,
            "stop_state": ended.get("state"),
        }))
    return 0 if streamed and not error and ended.get("state") == "idle" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--url-file", type=Path, required=True)
    parser.add_argument("--seconds", type=int, default=10)
    args = parser.parse_args()
    if not 3 <= args.seconds <= 15:
        raise SystemExit("Only 3-15 seconds permitted")
    raise SystemExit(run(args.url_file, args.seconds))
