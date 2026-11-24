#!/usr/bin/env python3
"""Generate the fall-alarm speech WAV asset (run once, on a dev machine).

Produces ``noetix/bumi/fall_alarm/resource/fall_alarm.wav`` — 16 kHz mono
S16LE — from the alarm phrase using Microsoft Edge TTS (``edge-tts``). The
card plays this file at runtime; without it the card falls back to a chime.

Requirements (dev machine only, not the robot image):

    pip install edge-tts
    ffmpeg   # to decode edge-tts' MP3 into PCM

Usage:

    python3 generate_alarm_wav.py [--text "检测到有人摔倒，请注意安全"] [--out PATH]
"""

import argparse
import asyncio
import subprocess
import tempfile
from pathlib import Path


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
    ap.add_argument("--text", default="检测到有人摔倒，请注意安全")
    ap.add_argument(
        "--out",
        default=str(Path(__file__).resolve().parents[1] / "resource" / "fall_alarm.wav"),
    )
    args = ap.parse_args()

    mp3 = asyncio.run(_tts_mp3(args.text))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    _mp3_to_wav_pcm(mp3, out)
    print(f"[fall_alarm] wrote {out} ({out.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
