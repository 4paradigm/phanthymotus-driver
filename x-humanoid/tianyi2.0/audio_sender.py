#!/usr/bin/env python3
"""
audio_sender.py — TCP audio streaming server for remote microphones.

Captures audio from a local ALSA device (e.g. DJI Wireless Mic), converts
to S16_LE mono 16kHz PCM, and streams to all connected TCP clients.

Usage:
    python3 audio_sender.py [--port 9800] [--card 0]

Deploy on the machine where the USB mic is physically connected.
Clients (e.g. ext_mic plugin on another host) connect and receive raw PCM-16k.
"""

import argparse
import numpy as np
import os
import re
import select
import socket
import subprocess
import threading
import time


def _detect_card(card_idx: int) -> dict:
    """Detect audio format/rate for given ALSA card index."""
    info = {"rate": 48000, "channels": 1, "format": "S16_LE"}
    try:
        result = subprocess.run(
            ['arecord', '-D', f'hw:{card_idx},0', '--dump-hw-params', '-d', '1', '/dev/null'],
            capture_output=True, text=True, timeout=5,
        )
        output = result.stdout + result.stderr
        for line in output.splitlines():
            if line.strip().startswith('FORMAT:'):
                fmts = line.split('FORMAT:')[1].strip().split()
                if 'S24_3LE' in fmts:
                    info["format"] = "S24_3LE"
                else:
                    info["format"] = "S16_LE"
            elif 'CHANNELS:' in line:
                ch = line.split('CHANNELS:')[1].strip()
                if '2' in ch:
                    info["channels"] = 2
            elif 'RATE:' in line:
                rates = [int(x) for x in re.findall(r'\d+', line.split('RATE:')[1])]
                if rates:
                    info["rate"] = min(rates)
    except Exception as e:
        print(f"[audio_sender] hw probe failed: {e}")
    return info


def _capture_loop(card_idx: int, clients: list, clients_lock: threading.Lock):
    """Capture audio and broadcast PCM-16k to all connected clients."""
    import alsaaudio

    hw = _detect_card(card_idx)
    print(f"[audio_sender] device: format={hw['format']} channels={hw['channels']} rate={hw['rate']}")

    if hw["format"] == "S24_3LE" and hw["channels"] == 2:
        pcm = alsaaudio.PCM(
            type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NORMAL,
            rate=hw["rate"], channels=2, format=alsaaudio.PCM_FORMAT_S24_3LE,
            periodsize=1024, cardindex=card_idx,
        )
        is_s24_stereo = True
    else:
        pcm = alsaaudio.PCM(
            type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NORMAL,
            rate=hw["rate"], channels=1, format=alsaaudio.PCM_FORMAT_S16_LE,
            periodsize=1024, cardindex=card_idx,
        )
        is_s24_stereo = False

    native_rate = hw["rate"]
    target_rate = 16000
    pub_buf = bytearray()
    CHUNK_SIZE = 1024  # 512 int16 samples = 1024 bytes

    print(f"[audio_sender] capturing card {card_idx}, {native_rate}Hz → {target_rate}Hz")

    last_success = time.time()
    MAX_FAIL_DURATION = 10  # exit if no successful read for 10s

    while True:
        try:
            length, data = pcm.read()
        except Exception as e:
            print(f"[audio_sender] read error: {e}, reopening...")
            time.sleep(1)
            try:
                pcm.close()
            except Exception:
                pass
            try:
                if is_s24_stereo:
                    pcm = alsaaudio.PCM(
                        type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NORMAL,
                        rate=native_rate, channels=2, format=alsaaudio.PCM_FORMAT_S24_3LE,
                        periodsize=1024, cardindex=card_idx,
                    )
                else:
                    pcm = alsaaudio.PCM(
                        type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NORMAL,
                        rate=native_rate, channels=1, format=alsaaudio.PCM_FORMAT_S16_LE,
                        periodsize=1024, cardindex=card_idx,
                    )
            except Exception:
                time.sleep(3)
            # Watchdog: exit if stuck too long
            if time.time() - last_success > MAX_FAIL_DURATION:
                print(f"[audio_sender] FATAL: no successful read for {MAX_FAIL_DURATION}s, exiting", flush=True)
                os._exit(1)
            continue

        if length <= 0:
            if time.time() - last_success > MAX_FAIL_DURATION:
                print(f"[audio_sender] FATAL: no successful read for {MAX_FAIL_DURATION}s, exiting", flush=True)
                os._exit(1)
            continue

        last_success = time.time()

        # Convert S24_3LE stereo → S16_LE mono
        if is_s24_stereo:
            raw = np.frombuffer(data, dtype=np.uint8)
            n_frames = len(raw) // 6
            if n_frames == 0:
                continue
            raw = raw[:n_frames * 6].reshape(n_frames, 2, 3)
            left = raw[:, 0, 1:].copy().view(np.int16).flatten()
            right = raw[:, 1, 1:].copy().view(np.int16).flatten()
            samples = ((left.astype(np.int32) + right.astype(np.int32)) // 2).astype(np.int16)
        else:
            samples = np.frombuffer(data, dtype=np.int16)

        # Resample to 16kHz
        if native_rate != target_rate:
            n_out = int(len(samples) * target_rate / native_rate)
            if n_out <= 0:
                continue
            x_new = np.linspace(0, len(samples) - 1, n_out)
            samples = np.interp(x_new, np.arange(len(samples)), samples.astype(np.float32)).astype(np.int16)

        # Buffer and send in fixed chunks
        pub_buf += samples.tobytes()
        while len(pub_buf) >= CHUNK_SIZE:
            chunk = bytes(pub_buf[:CHUNK_SIZE])
            pub_buf = pub_buf[CHUNK_SIZE:]

            # Broadcast to all clients
            with clients_lock:
                dead = []
                for i, conn in enumerate(clients):
                    try:
                        conn.sendall(chunk)
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        dead.append(i)
                for i in reversed(dead):
                    try:
                        clients[i].close()
                    except Exception:
                        pass
                    clients.pop(i)
                    print(f"[audio_sender] client disconnected, {len(clients)} remaining")


def main():
    parser = argparse.ArgumentParser(description="TCP audio streaming server")
    parser.add_argument("--port", type=int, default=9800, help="TCP listen port")
    parser.add_argument("--card", type=int, default=0, help="ALSA card index")
    args = parser.parse_args()

    clients: list[socket.socket] = []
    clients_lock = threading.Lock()

    # Start capture thread
    capture_thread = threading.Thread(
        target=_capture_loop, args=(args.card, clients, clients_lock), daemon=True
    )
    capture_thread.start()

    # TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", args.port))
    server.listen(5)
    print(f"[audio_sender] listening on 0.0.0.0:{args.port}")

    try:
        while True:
            conn, addr = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with clients_lock:
                clients.append(conn)
            print(f"[audio_sender] client connected from {addr}, total={len(clients)}")
    except KeyboardInterrupt:
        print("[audio_sender] shutting down")
    finally:
        server.close()


if __name__ == "__main__":
    main()
