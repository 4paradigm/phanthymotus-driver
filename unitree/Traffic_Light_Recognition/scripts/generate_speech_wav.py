#!/usr/bin/env python3
"""Generate the traffic-light speech WAV assets (run once, on a dev machine).

Produces three 16 kHz mono S16LE WAVs under
``unitree/Traffic_Light_Recognition/resource/``:

    traffic_light_red.wav    — "红灯，请等待"
    traffic_light_yellow.wav — "黄灯，请注意"
    traffic_light_green.wav  — "绿灯，请通行"

using Microsoft Edge TTS (``edge-tts``). The card plays these at runtime; without
them it falls back to a per-color chime (see speech.py).

Requirements (dev machine only, not the robot image):

    pip install edge-tts
    ffmpeg   # to decode edge-tts' MP3 into PCM

Usage:

    python3 generate_speech_wav.py [--out-dir PATH]
"""

import argparse
import asyncio
import subprocess
import tempfile
from pathlib import Path

PHRASES = {
    "red": "红灯，请等待",
    "yellow": "黄灯，请注意",
    "green": "绿灯，请通行",
}


async def _tts_mp3(text: str) -> bytes:
    import edge_tts
    communicate = edge_tts.Communicate(text, "zh-CN-XiaoxiaoNeural")
    audio = bytearray()
    async for chunk in communicate.stream():
        if chunk["type"] == "audio":
            audio.extend(chunk["data"])
    return bytes(audio)


def _mp3_to_wav_pcm(mp3: bytes, out_path: Path) -> None:
    with tempfile.NamedTemporaryFile(suffix=".mp3") as tmp:
        tmp.write(mp3)
        tmp.flush()
        subprocess.run(
            ["ffmpeg", "-y", "-i", tmp.name,
             "-f", "wav", "-acodec", "pcm_s16le", "-ar", "16000", "-ac", "1",
             str(out_path)],
            check=True, capture_output=True,
        )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--out-dir",
        default=str(Path(__file__).resolve().parents[1] / "resource"),
    )
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for color, text in PHRASES.items():
        mp3 = asyncio.run(_tts_mp3(text))
        out = out_dir / f"traffic_light_{color}.wav"
        _mp3_to_wav_pcm(mp3, out)
        print(f"[traffic_light] wrote {out} ({out.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
