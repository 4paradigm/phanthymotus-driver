#!/usr/bin/env python3
"""
drivers/unitree/r1/ext_devices.py — External mic and camera plugins (multiInstance).

Enumerates system audio/video devices, excluding built-in mic
and RealSense cameras. Each external device can be started as an independent
tool instance on the canvas.
"""

import glob
import logging
import os
import re
import subprocess
import threading
import time
from typing import Any, Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from audio_msgs.msg import AudioChunk

log = logging.getLogger(__name__)

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)

JPEG_QUALITY = 80

# Preferred pixel format order when auto-selecting
_FOURCC_PRIORITY = ['MJPG', 'H264', 'YUYV']


# ── Device Enumeration ────────────────────────────────────────────────────────

def _enumerate_ext_mics() -> list[dict]:
    """List external USB microphone input devices via arecord -l, fallback to sounddevice."""
    devices = []

    # Primary: parse arecord -l (works reliably in Docker with /dev/snd mapped)
    try:
        output = subprocess.check_output(['arecord', '-l'], text=True, timeout=5, stderr=subprocess.DEVNULL)
        for line in output.splitlines():
            if not line.startswith('card '):
                continue
            # Format: "card N: NAME [DESC], device M: ..."
            name_lower = line.lower()
            if 'realsense' in name_lower or 'intel' in name_lower:
                continue
            # Skip NVIDIA APE internal devices
            if 'ape' in name_lower or 'tegra' in name_lower:
                continue
            try:
                card_part = line.split(':')[0]  # "card N"
                card_num = int(card_part.split()[1])
                device_part = line.split('device')[1].split(':')[0].strip()
                device_num = int(device_part)
                # Extract description between [ ]
                desc = line.split('[')[1].split(']')[0] if '[' in line else f"card{card_num}"
                alsa_id = f"hw:{card_num},{device_num}"
                devices.append({
                    "index": card_num,
                    "device_num": device_num,
                    "alsa_id": alsa_id,
                    "name": desc,
                })
            except (IndexError, ValueError):
                continue
    except Exception as e:
        log.debug(f"[ext_mic] arecord -l failed: {e}")

    if devices:
        return devices

    # Second try: pyalsaaudio — works when PortAudio doesn't enumerate USB audio (e.g. Jetson)
    try:
        import alsaaudio
        capture_pcms = set(alsaaudio.pcms(alsaaudio.PCM_CAPTURE))
        # Parse full names from /proc/asound/cards: "N [short]: driver - Full Name"
        full_names: dict[int, str] = {}
        try:
            with open('/proc/asound/cards') as f:
                for line in f:
                    m = re.match(r'\s*(\d+)\s+\[\S+\s*\]:\s*\S+\s+-\s+(.+)', line)
                    if m:
                        full_names[int(m.group(1))] = m.group(2).strip()
        except Exception:
            pass
        for idx, card_name in enumerate(alsaaudio.cards()):
            name_lower = card_name.lower()
            if 'realsense' in name_lower or 'intel' in name_lower:
                continue
            if 'ape' in name_lower or 'tegra' in name_lower or 'hda' in name_lower:
                continue
            alsa_id = f"hw:CARD={card_name},DEV=0"
            if alsa_id not in capture_pcms:
                continue
            display_name = full_names.get(idx, card_name)
            devices.append({
                "index": idx,
                "alsa_id": alsa_id,
                "name": display_name,
            })
        if devices:
            return devices
    except Exception as e:
        log.debug(f"[ext_mic] pyalsaaudio enumeration failed: {e}")

    # Fallback: sounddevice
    try:
        import sounddevice as sd
    except ImportError:
        log.warning("[ext_mic] sounddevice not installed and arecord unavailable")
        return []

    for i, dev in enumerate(sd.query_devices()):
        if dev['max_input_channels'] < 1:
            continue
        name = dev['name'].lower()
        if 'realsense' in name or 'intel' in name:
            continue
        devices.append({
            "index": i,
            "alsa_id": i,
            "name": dev['name'],
            "channels": dev['max_input_channels'],
            "sample_rate": int(dev['default_samplerate']),
        })
    return devices


def _parse_arecord_output(output: str) -> list[dict]:
    """Parse arecord -l output into device list, excluding internal devices."""
    devices = []
    EXCLUDE = ["realsense", "nvidia", "tegra", "hdmi", "ape"]
    for line in output.splitlines():
        m = re.match(r'card (\d+): (\w+) \[(.+?)\], device (\d+):', line)
        if m:
            card_idx, card_id, desc, dev_idx = m.groups()
            if any(x in desc.lower() or x in card_id.lower() for x in EXCLUDE):
                continue
            devices.append({
                "card": int(card_idx),
                "device": int(dev_idx),
                "name": desc,
                "alsa_id": f"hw:{card_idx},{dev_idx}",
            })
    return devices


def _parse_hw_params(output: str) -> dict:
    """Parse arecord --dump-hw-params output for format/rate/channels."""
    info = {"format": "S16_LE", "rate": 48000, "channels": 1}
    for line in output.splitlines():
        if line.strip().startswith('FORMAT:'):
            fmts = line.split('FORMAT:')[1].strip().split()
            if 'S24_3LE' in fmts:
                info["format"] = "S24_3LE"
            elif 'S16_LE' in fmts:
                info["format"] = "S16_LE"
        elif 'CHANNELS:' in line:
            ch = line.split('CHANNELS:')[1].strip()
            if '2' in ch:
                info["channels"] = 2
        elif 'RATE:' in line:
            rates = [int(x) for x in re.findall(r'\d+', line.split('RATE:')[1])]
            if rates:
                info["rate"] = min(rates)
    return info


def _enumerate_ext_cameras() -> list[dict]:
    """List external V4L2 video capture devices (excluding RealSense)."""
    devices = []
    for path in sorted(glob.glob('/dev/video*')):
        try:
            info = subprocess.check_output(
                ['v4l2-ctl', '-d', path, '--info'],
                text=True, timeout=2, stderr=subprocess.DEVNULL,
                env={**os.environ, 'LC_ALL': 'C'},
            )
        except Exception:
            continue

        # Exclude RealSense (Intel vendor)
        if 'RealSense' in info or 'Intel(R) RealSense' in info:
            continue
        # Only keep Video Capture devices (not metadata nodes)
        if 'Video Capture' not in info:
            continue

        name = "Unknown"
        for line in info.splitlines():
            if 'Card type' in line:
                name = line.split(':', 1)[-1].strip()
                break

        # Probe supported pixel formats and resolutions via v4l2-ctl --list-formats-ext.
        # If the probe succeeds but returns no formats, the node is not a real capture device
        # (e.g. secondary metadata interface) — skip it.
        formats: list[str] = []
        resolutions: list[str] = []
        fmt_probe_ok = False
        try:
            fmt_out = subprocess.check_output(
                ['v4l2-ctl', '-d', path, '--list-formats-ext'],
                text=True, timeout=2, stderr=subprocess.DEVNULL,
                env={**os.environ, 'LC_ALL': 'C'},
            )
            fmt_probe_ok = True
            formats = re.findall(r"'\s*([A-Z0-9]{4})\s*'", fmt_out)
            for line in fmt_out.splitlines():
                m = re.search(r'Size: Discrete (\d+x\d+)', line)
                if m and m.group(1) not in resolutions:
                    resolutions.append(m.group(1))
        except Exception:
            pass

        # Probe succeeded but no formats → secondary/metadata node, not usable for capture
        if fmt_probe_ok and not formats:
            continue

        devices.append({"path": path, "name": name, "formats": formats, "resolutions": resolutions})
    return devices


# ── ROS2 Nodes ────────────────────────────────────────────────────────────────

class _ExtMicNode(Node):
    """Captures audio from a system input device and publishes AudioChunk."""

    def __init__(self, device_index, device_name: str, namespace: str, instance_id: str, context=None):
        node_name = f"ext_mic_{instance_id.replace('-', '_')}"
        super().__init__(node_name, context=context)
        self._device_index = device_index  # alsa_id string (hw:CARD=...) or numeric index
        self._device_name = device_name
        self._instance_id = instance_id
        self._topic = f"/{namespace}/ext_mic/{instance_id.replace('-', '_')}/audio"
        self._pub = self.create_publisher(AudioChunk, self._topic, _LOW_LAT_QOS)
        self._stream = None
        self._alsa_pcm = None   # used when device_index is an ALSA card name string
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._alsa_native_rate: int = 16000
        self._alsa_rate_locked: bool = False
        self._alsa_probe_samples: int = 0
        self._alsa_probe_start: float = 0.0
        self.state = "idle"

    def _is_alsa_id(self) -> bool:
        return isinstance(self._device_index, str) and self._device_index.startswith("hw:")

    def start(self) -> dict:
        if self.state == "running":
            return self._status_dict()
        if self._is_alsa_id():
            self._start_alsaaudio()
        else:
            self._start_sounddevice()
        self.state = "running"
        log.info(f"[ext_mic] started device={self._device_name} ({self._device_index}) → {self._topic}")
        return self._status_dict()

    def _start_sounddevice(self):
        import sounddevice as sd
        self._stream = sd.InputStream(
            device=self._device_index,
            samplerate=16000, channels=1, dtype='int16',
            blocksize=512, callback=self._audio_cb,
        )
        self._stream.start()

    def _start_alsaaudio(self):
        import alsaaudio
        # alsa_id format: "hw:CARD=Pro,DEV=0" or "hw:N,M"
        if "CARD=" in self._device_index:
            card_part = self._device_index.split("hw:CARD=", 1)[1].split(",DEV=")[0]
            card_idx = alsaaudio.cards().index(card_part)
        else:
            # "hw:N,M" format — card number is N
            card_idx = int(self._device_index.split("hw:", 1)[1].split(",")[0])

        # Query device capabilities via arecord --dump-hw-params, then open
        # at the device's native rate and resample to 16kHz in software.
        hw_rate = 44100  # safe default
        try:
            hw_info = subprocess.run(
                ['arecord', '-D', f'hw:{card_idx},0', '--dump-hw-params', '-d', '1', '/dev/null'],
                capture_output=True, text=True, timeout=5,
            )
            # hw params are printed to stderr
            output = hw_info.stdout + hw_info.stderr
            for line in output.splitlines():
                if 'RATE:' in line:
                    rates = [int(x) for x in re.findall(r'\d+', line.split('RATE:')[1])]
                    if rates:
                        hw_rate = min(rates)  # prefer lowest supported rate
                    break
            print(f"[ext_mic] hw probe: rate={hw_rate}", flush=True)
        except Exception as e:
            print(f"[ext_mic] hw probe failed ({e}), using rate={hw_rate}", flush=True)

        self._alsa_native_fmt = "S16_LE_mono"
        self._alsa_pcm = alsaaudio.PCM(
            type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NORMAL,
            rate=hw_rate, channels=1, format=alsaaudio.PCM_FORMAT_S16_LE,
            periodsize=1024, cardindex=card_idx,
        )
        self._alsa_native_rate = hw_rate
        self._alsa_rate_locked = True
        print(f"[ext_mic] opened {self._device_name} as S16_LE mono {hw_rate}Hz", flush=True)

        self._alsa_probe_samples = 0
        self._alsa_probe_start = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._alsa_capture_loop, daemon=True)
        self._thread.start()

    def _alsa_capture_loop(self):
        first_read = True
        first_publish = True
        _pub_buf = bytearray()       # accumulate resampled bytes until we have a full 512-sample chunk
        _TARGET = 1024               # 512 int16 samples @ 16 kHz = 1024 bytes
        while self._running:
            length, data = self._alsa_pcm.read()
            if length <= 0:
                continue

            # Convert S24_3LE stereo to S16_LE mono if needed
            if self._alsa_native_fmt == "S24_3LE_stereo":
                raw = np.frombuffer(data, dtype=np.uint8)
                n_frames = len(raw) // 6  # 6 bytes per stereo frame (3 bytes × 2 channels)
                if n_frames == 0:
                    continue
                raw = raw[:n_frames * 6].reshape(n_frames, 2, 3)
                # Reconstruct 24-bit signed int, then >> 8 to 16-bit
                def _s24_to_s16_local(ch):
                    val = (ch[:, 0].astype(np.int32) | (ch[:, 1].astype(np.int32) << 8) | (ch[:, 2].astype(np.int32) << 16))
                    val[val >= 0x800000] -= 0x1000000
                    return (val >> 8).astype(np.int16)
                left = _s24_to_s16_local(raw[:, 0, :])
                right = _s24_to_s16_local(raw[:, 1, :])
                mono = ((left.astype(np.int32) + right.astype(np.int32)) // 2).astype(np.int16)
                data = mono.tobytes()
                length = n_frames

            # Start probe timer on first actual data (not before thread start,
            # to avoid counting ALSA init latency as part of elapsed time)
            if first_read:
                self._alsa_probe_start = time.monotonic()
                self._alsa_probe_samples = 0
                first_read = False
                print(f"[ext_mic] capture loop running, first read length={length} bytes={len(data)}", flush=True)

            # Phase 1: accumulate samples to measure actual hardware rate
            if not self._alsa_rate_locked:
                self._alsa_probe_samples += length
                elapsed = time.monotonic() - self._alsa_probe_start
                if elapsed >= 0.5:
                    measured = int(self._alsa_probe_samples / elapsed)
                    std_rates = [8000, 11025, 16000, 22050, 32000, 44100, 48000]
                    self._alsa_native_rate = min(std_rates, key=lambda r: abs(r - measured))
                    self._alsa_rate_locked = True
                    print(f"[ext_mic] detected native_rate={self._alsa_native_rate} (measured={measured})", flush=True)
                    log.info(f"[ext_mic] detected native_rate={self._alsa_native_rate} (measured={measured})")
                continue  # discard probe data, don't publish

            # Phase 2: resample to 16000 Hz if device delivers a different rate
            if self._alsa_native_rate != 16000:
                samples = np.frombuffer(data, dtype=np.int16).astype(np.float32)
                n_out = int(len(samples) * 16000 / self._alsa_native_rate)
                if n_out <= 0:
                    continue
                x_new = np.linspace(0, len(samples) - 1, n_out)
                data = np.interp(x_new, np.arange(len(samples)), samples).astype(np.int16).tobytes()
                # After downsampling the chunk may be too small for the VAD (< 512 samples).
                # Buffer until we have a full TARGET-byte chunk before publishing.
                _pub_buf += data
                while len(_pub_buf) >= _TARGET:
                    chunk = bytes(_pub_buf[:_TARGET])
                    _pub_buf = _pub_buf[_TARGET:]
                    msg = AudioChunk()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.format = "audio/pcm-16k"
                    msg.data = list(chunk)
                    self._pub.publish(msg)
                    if first_publish:
                        print(f"[ext_mic] first publish on {self._topic}, resample {self._alsa_native_rate}→16k", flush=True)
                        first_publish = False
                continue

            msg = AudioChunk()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "audio/pcm-16k"
            msg.data = list(data)
            self._pub.publish(msg)

    def stop(self) -> dict:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
            self._thread = None
        if self._alsa_pcm:
            self._alsa_pcm.close()
            self._alsa_pcm = None
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None
        self.state = "idle"
        return self._status_dict()

    def _audio_cb(self, indata, frames, time_info, status):
        if status:
            log.debug(f"[ext_mic] sounddevice status: {status}")
        msg = AudioChunk()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "audio/pcm-16k"
        msg.data = list(indata.tobytes())
        self._pub.publish(msg)

    def _status_dict(self) -> dict:
        return {
            "state": self.state,
            "device_name": self._device_name,
            "device_index": self._device_index,
            "topic_in": [],
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k", "desc": ""}],
        }


class _NetworkMicNode(Node):
    """Captures audio from a remote host via SSH + arecord and publishes AudioChunk."""

    def __init__(self, url: str, device_name: str, namespace: str, instance_id: str, context=None,
                 ssh_user: str = "", ssh_pass: str = "", ssh_script: str = "", ssh_card: str = "1"):
        node_name = f"ext_mic_{instance_id.replace('-', '_')}"
        super().__init__(node_name, context=context)
        self._url = url  # "ssh://user:pass@host/hw:card,dev" or "tcp://host:port"
        self._device_name = device_name
        self._instance_id = instance_id
        self._topic = f"/{namespace}/ext_mic/{instance_id.replace('-', '_')}/audio"
        self._pub = self.create_publisher(AudioChunk, self._topic, _LOW_LAT_QOS)
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self.state = "idle"
        # SSH management config for remote audio_sender health check / restart
        self._ssh_user = ssh_user
        self._ssh_pass = ssh_pass
        self._ssh_script = ssh_script
        self._ssh_card = ssh_card

    def start(self) -> dict:
        if self.state == "running":
            # Check if capture thread is actually alive
            if self._thread and self._thread.is_alive():
                return self._status_dict()
            # Thread died — reset and restart
            self._running = False
            self._thread = None
            self.state = "idle"
            log.warning(f"[ext_mic/net] thread was dead for {self._device_name}, restarting")

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self.state = "running"
        log.info(f"[ext_mic/net] started {self._device_name} ({self._url}) → {self._topic}")
        return self._status_dict()

    def _capture_loop(self):
        """Capture via SSH arecord or TCP socket depending on URL scheme."""
        if self._url.startswith("ssh://"):
            self._ssh_capture()
        else:
            self._tcp_capture()

    def _ssh_capture(self):
        """ssh://user:pass@host/hw:card,dev — remote arecord via SSH pipe."""
        # Parse: ssh://ubuntu:123@192.168.41.1/hw:1,0
        parts = self._url.replace("ssh://", "")
        user_pass, rest = parts.split("@", 1)
        host, alsa_dev = rest.split("/", 1)
        user, password = user_pass.split(":", 1) if ":" in user_pass else (user_pass, "")

        # Probe remote device capabilities
        rate = 48000
        fmt = "S16_LE"
        channels = 1
        try:
            probe_cmd = ["sshpass", "-p", password, "ssh", "-o", "StrictHostKeyChecking=no",
                         f"{user}@{host}",
                         f"arecord -D {alsa_dev} --dump-hw-params -d 1 /dev/null 2>&1"]
            probe = subprocess.run(probe_cmd, capture_output=True, text=True, timeout=10)
            output = probe.stdout + probe.stderr
            print(f"[ext_mic/ssh] probe output ({len(output)} chars): {output[:200]}", flush=True)
            for line in output.splitlines():
                if 'RATE:' in line:
                    rates = [int(x) for x in re.findall(r'\d+', line.split('RATE:')[1])]
                    if rates:
                        rate = min(rates)
                elif 'FORMAT:' in line:
                    fmts = line.split('FORMAT:')[1].strip().split()
                    if 'S24_3LE' in fmts and 'S16_LE' not in fmts:
                        fmt = "S24_3LE"
                    elif 'S16_LE' in fmts:
                        fmt = "S16_LE"
                elif 'CHANNELS:' in line:
                    ch = line.split('CHANNELS:')[1].strip()
                    if '2' in ch and '1' not in ch:
                        channels = 2
            print(f"[ext_mic/ssh] remote device: fmt={fmt} ch={channels} rate={rate}", flush=True)
        except Exception as e:
            print(f"[ext_mic/ssh] probe failed: {e}, using defaults", flush=True)

        target_rate = 16000
        CHUNK_SIZE = 1024  # 512 int16 samples
        is_s24_stereo = (fmt == "S24_3LE" and channels == 2)
        if fmt == "S24_3LE" and channels == 1:
            channels = 2  # S24_3LE usually paired with stereo

        while self._running:
            proc = None
            try:
                arecord_fmt = fmt
                cmd = [
                    "sshpass", "-p", password,
                    "ssh", "-o", "StrictHostKeyChecking=no", "-o", "ServerAliveInterval=10",
                    f"{user}@{host}",
                    f"arecord -D {alsa_dev} -f {arecord_fmt} -r {rate} -c {channels} -t raw -q -"
                ]
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                print(f"[ext_mic/ssh] connected to {user}@{host}, {alsa_dev} {arecord_fmt} {channels}ch {rate}Hz", flush=True)

                pub_buf = bytearray()
                raw_buf = bytearray()  # buffer for frame-alignment
                first_publish = True
                bytes_per_frame = (3 * channels) if fmt == "S24_3LE" else (2 * channels)

                while self._running:
                    data = proc.stdout.read(4096)
                    if not data:
                        break

                    raw_buf += data

                    # Process only complete frames
                    n_frames = len(raw_buf) // bytes_per_frame
                    if n_frames == 0:
                        continue
                    frame_bytes = n_frames * bytes_per_frame
                    chunk_data = bytes(raw_buf[:frame_bytes])
                    raw_buf = raw_buf[frame_bytes:]

                    # Convert S24_3LE → S16_LE mono
                    if fmt == "S24_3LE":
                        raw = np.frombuffer(chunk_data, dtype=np.uint8)

                        def _s24_to_s16(ch_bytes):
                            """Reconstruct 24-bit signed int, then >> 8 to 16-bit."""
                            val = (ch_bytes[:, 0].astype(np.int32) |
                                   (ch_bytes[:, 1].astype(np.int32) << 8) |
                                   (ch_bytes[:, 2].astype(np.int32) << 16))
                            val[val >= 0x800000] -= 0x1000000
                            return (val >> 8).astype(np.int16)

                        if channels == 2:
                            raw = raw.reshape(n_frames, 2, 3)
                            left = _s24_to_s16(raw[:, 0, :])
                            right = _s24_to_s16(raw[:, 1, :])
                            samples = ((left.astype(np.int32) + right.astype(np.int32)) // 2).astype(np.int16)
                        else:
                            raw = raw.reshape(n_frames, 3)
                            samples = _s24_to_s16(raw)
                    else:
                        samples = np.frombuffer(chunk_data, dtype=np.int16)
                        if channels == 2:
                            samples = samples.reshape(-1, 2).mean(axis=1).astype(np.int16)

                    # Resample to 16kHz
                    if rate != target_rate:
                        n_out = int(len(samples) * target_rate / rate)
                        if n_out <= 0:
                            continue
                        x_new = np.linspace(0, len(samples) - 1, n_out)
                        samples = np.interp(x_new, np.arange(len(samples)), samples.astype(np.float32)).astype(np.int16)

                    pub_buf += samples.tobytes()
                    while len(pub_buf) >= CHUNK_SIZE:
                        chunk = bytes(pub_buf[:CHUNK_SIZE])
                        pub_buf = pub_buf[CHUNK_SIZE:]
                        msg = AudioChunk()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.format = "audio/pcm-16k"
                        msg.data = list(chunk)
                        self._pub.publish(msg)
                        if first_publish:
                            print(f"[ext_mic/ssh] first publish on {self._topic}", flush=True)
                            first_publish = False
            except Exception as e:
                if self._running:
                    print(f"[ext_mic/ssh] error: {e}, reconnecting in 3s", flush=True)
            finally:
                if proc:
                    try:
                        proc.kill()
                        proc.wait(timeout=2)
                    except Exception:
                        pass
            if self._running:
                time.sleep(3)

    def _tcp_capture(self):
        """tcp://host:port[/format] — connect to raw audio TCP server.

        Default format: S24_3LE stereo 48kHz (DJI Wireless Mic).
        Append /pcm16k for pre-converted 16kHz mono streams.
        """
        import socket as _socket
        url_body = self._url.replace("tcp://", "")

        # Parse optional format suffix: tcp://host:port/pcm16k
        pre_converted = False
        if "/pcm16k" in url_body:
            url_body = url_body.replace("/pcm16k", "")
            pre_converted = True

        host, port = url_body.rsplit(":", 1)
        port = int(port)

        # Health check: ensure remote audio_sender is alive and streaming
        self._ensure_remote_ready(host, port)

        # S24_3LE stereo 48kHz params
        FRAME_SIZE = 6  # 3 bytes * 2 channels
        NATIVE_RATE = 48000
        TARGET_RATE = 16000
        PUB_CHUNK = 1024  # 512 int16 samples = 1024 bytes

        connect_failures = 0
        MAX_CONNECT_RETRIES = 3

        while self._running:
            sock = None
            try:
                sock = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((host, port))
                sock.settimeout(10.0)  # detect stale connection (no data for 10s)
                sock.setsockopt(_socket.IPPROTO_TCP, _socket.TCP_NODELAY, 1)
                connect_failures = 0  # reset on successful connect
                print(f"[ext_mic/tcp] connected to {host}:{port} (pre_converted={pre_converted})", flush=True)

                raw_buf = bytearray()
                pub_buf = bytearray()
                first_publish = True

                while self._running:
                    try:
                        data = sock.recv(8192)
                    except _socket.timeout:
                        print(f"[ext_mic/tcp] no data for 10s, reconnecting...", flush=True)
                        break
                    if not data:
                        break

                    if pre_converted:
                        # Already PCM-16k mono
                        pub_buf += data
                    else:
                        # S24_3LE stereo 48kHz → S16_LE mono 16kHz
                        raw_buf += data
                        n_frames = len(raw_buf) // FRAME_SIZE
                        if n_frames == 0:
                            continue
                        frame_bytes = n_frames * FRAME_SIZE
                        chunk = bytes(raw_buf[:frame_bytes])
                        raw_buf = raw_buf[frame_bytes:]

                        arr = np.frombuffer(chunk, dtype=np.uint8).reshape(n_frames, 2, 3)
                        # S24_3LE decode
                        def _s24(ch):
                            v = (ch[:, 0].astype(np.int32) | (ch[:, 1].astype(np.int32) << 8) | (ch[:, 2].astype(np.int32) << 16))
                            v[v >= 0x800000] -= 0x1000000
                            return (v >> 8).astype(np.int16)
                        left = _s24(arr[:, 0, :])
                        right = _s24(arr[:, 1, :])
                        mono = ((left.astype(np.int32) + right.astype(np.int32)) // 2).astype(np.int16)

                        # Resample 48k → 16k
                        n_out = int(len(mono) * TARGET_RATE / NATIVE_RATE)
                        if n_out > 0:
                            x_new = np.linspace(0, len(mono) - 1, n_out)
                            resampled = np.interp(x_new, np.arange(len(mono)), mono.astype(np.float32)).astype(np.int16)
                            pub_buf += resampled.tobytes()

                    # Publish in fixed chunks
                    while len(pub_buf) >= PUB_CHUNK:
                        chunk = bytes(pub_buf[:PUB_CHUNK])
                        pub_buf = pub_buf[PUB_CHUNK:]
                        msg = AudioChunk()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.format = "audio/pcm-16k"
                        msg.data = list(chunk)
                        self._pub.publish(msg)
                        if first_publish:
                            print(f"[ext_mic/tcp] first publish on {self._topic}", flush=True)
                            first_publish = False
            except Exception as e:
                if self._running:
                    connect_failures += 1
                    if connect_failures >= MAX_CONNECT_RETRIES:
                        print(f"[ext_mic/tcp] failed to connect to {host}:{port} after {MAX_CONNECT_RETRIES} attempts, giving up", flush=True)
                        self.state = "error"
                        return
                    print(f"[ext_mic/tcp] error: {e}, retrying ({connect_failures}/{MAX_CONNECT_RETRIES}) in 2s", flush=True)
            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass
            if self._running:
                time.sleep(2)
                self._ensure_remote_ready(host, port)

    def _ensure_remote_ready(self, host: str, port: int, force_restart: bool = False):
        """Check remote audio_sender health; restart via SSH if unhealthy."""
        import socket as _socket
        import subprocess as _subprocess

        # Step 1: probe — connect and try to read data within 3s
        if not force_restart:
            healthy = False
            try:
                sock = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
                sock.settimeout(3.0)
                sock.connect((host, port))
                sock.settimeout(3.0)
                data = sock.recv(1024)
                healthy = len(data) > 0
                sock.close()
            except Exception:
                pass

            if healthy:
                return

        # Step 2: unhealthy — SSH restart if config available
        if not self._ssh_user or not self._ssh_script:
            print(f"[ext_mic/tcp] remote unhealthy on {host}:{port} but no SSH config", flush=True)
            return

        print(f"[ext_mic/tcp] remote unhealthy, restarting via SSH...", flush=True)
        # Use SIGKILL to ensure immediate termination
        kill_cmd = f"pkill -9 -f '{self._ssh_script}' 2>/dev/null"
        ssh_base = [
            "sshpass", "-p", self._ssh_pass,
            "ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=5",
            f"{self._ssh_user}@{host}",
        ]
        try:
            _subprocess.run(ssh_base + [kill_cmd], capture_output=True, timeout=10)
        except Exception as e:
            print(f"[ext_mic/tcp] SSH kill failed: {e}", flush=True)

        # Wait for ALSA device to be fully released
        time.sleep(3)

        # Start fresh audio_sender
        start_cmd = (
            f"nohup python3 {self._ssh_script} --port {port} --card {self._ssh_card} "
            f"> /tmp/audio_sender.log 2>&1 &"
        )
        try:
            _subprocess.run(ssh_base + [start_cmd], capture_output=True, timeout=10)
        except Exception as e:
            print(f"[ext_mic/tcp] SSH start failed: {e}", flush=True)
            return

        # Step 3: wait for ready — poll TCP until data received (max ~10s)
        for i in range(5):
            time.sleep(2)
            try:
                sock = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
                sock.settimeout(3.0)
                sock.connect((host, port))
                sock.settimeout(3.0)
                data = sock.recv(1024)
                sock.close()
                if len(data) > 0:
                    print(f"[ext_mic/tcp] remote ready after restart ({(i+1)*2}s)", flush=True)
                    return
            except Exception:
                pass

        print(f"[ext_mic/tcp] remote still not ready after restart, proceeding anyway", flush=True)

    def stop(self) -> dict:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
            self._thread = None
        self.state = "idle"
        return self._status_dict()

    def _status_dict(self) -> dict:
        return {
            "state": self.state,
            "device_name": self._device_name,
            "device_index": self._url,
            "topic_in": [],
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k", "desc": ""}],
        }


class _ExtCameraNode:
    """Manages a subprocess that captures video from a V4L2 device and publishes JPEG."""

    def __init__(self, device_path: str, device_name: str, namespace: str, instance_id: str,
                 fps: int = 15, width: int = 1920, height: int = 1080,
                 pixel_format: str = "auto", available_formats: Optional[list] = None):
        self._device_path = device_path
        self._device_name = device_name
        self._instance_id = instance_id
        self._namespace = namespace
        self._topic = f"/{namespace}/ext_camera/{instance_id.replace('-', '_')}/rgb"
        self._fps = fps
        self._width = width
        self._height = height
        self._pixel_format = pixel_format
        self._available_formats: list = available_formats or []
        self._proc: Optional[Any] = None
        self.state = "idle"

    def start(self) -> dict:
        if self.state == "running":
            return self._status_dict()
        import multiprocessing as mp
        ctx = mp.get_context("spawn")
        self._proc = ctx.Process(
            target=_run_ext_camera_process,
            args=(self._device_path, self._namespace, self._instance_id,
                  self._fps, self._width, self._height,
                  self._pixel_format, self._available_formats),
            name=f"ext_camera_{self._instance_id}",
            daemon=True,
        )
        self._proc.start()
        self.state = "running"
        print(f"[ext_camera] subprocess started → pid={self._proc.pid} device={self._device_path} "
              f"({self._width}x{self._height}@{self._fps}) → {self._topic}", flush=True)
        return self._status_dict()

    def stop(self) -> dict:
        if self._proc is not None and self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=3.0)
            if self._proc.is_alive():
                self._proc.kill()
                self._proc.join(timeout=2.0)
        self._proc = None
        self.state = "idle"
        return self._status_dict()

    def _status_dict(self) -> dict:
        return {
            "state": self.state,
            "device_path": self._device_path,
            "device_name": self._device_name,
            "topic_in": [],
            "topic_out": [{"topic": self._topic, "format": "image/jpeg", "desc": ""}],
        }


def _run_ext_camera_process(device_path: str, namespace: str, instance_id: str,
                            fps: int, width: int, height: int,
                            pixel_format: str, available_formats: list) -> None:
    """Ext camera subprocess entry — independent GIL for full throughput."""
    import cv2
    import rclpy
    from rclpy.node import Node as _Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage as _CompressedImage

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )

    _FOURCC_PRIO = ['MJPG', 'H264', 'YUYV']
    _JPEG_Q = 80

    rclpy.init()
    node_name = f"ext_camera_{instance_id.replace('-', '_')}"
    node = _Node(node_name)
    topic = f"/{namespace}/ext_camera/{instance_id.replace('-', '_')}/rgb"
    pub = node.create_publisher(_CompressedImage, topic, _QOS)

    cap = cv2.VideoCapture(device_path)
    if not cap.isOpened():
        node.get_logger().error(f"[ext_camera] Cannot open device: {device_path}")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Set FOURCC
    fourcc = None
    if pixel_format == "auto":
        for f in _FOURCC_PRIO:
            if f in available_formats:
                fourcc = cv2.VideoWriter_fourcc(*f)
                break
    else:
        try:
            fourcc = cv2.VideoWriter_fourcc(*pixel_format)
        except Exception:
            pass
    if fourcc is not None:
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    actual = int(cap.get(cv2.CAP_PROP_FOURCC))
    actual_str = "".join([chr((actual >> 8 * i) & 0xFF) for i in range(4)])
    mjpg_passthrough = (actual_str == "MJPG")
    if mjpg_passthrough:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    node.get_logger().info(
        f"[ext_camera] capture started — {device_path} {width}x{height}@{fps} "
        f"fourcc={actual_str} passthrough={mjpg_passthrough}"
    )

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                import time
                time.sleep(0.1)
                continue
            if mjpg_passthrough:
                jpeg_bytes = frame.tobytes()
            else:
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, _JPEG_Q])
                jpeg_bytes = jpeg.tobytes()
            msg = _CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = jpeg_bytes
            pub.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()


# ── Plugins ───────────────────────────────────────────────────────────────────

TOOLS_EXT_MIC = [
    {
        "name": "ext_mic",
        "type": "sensor",
        "multiInstance": True,
        "description": "External USB microphone — captures audio and publishes PCM-16k",
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["start", "stop", "info"],
                    "description": "Action to perform",
                },
            },
            "required": ["action"],
        },
        "configSchema": {
            "type": "object",
            "properties": {
                "device_index": {
                    "type": "string",
                    "description": "音频设备",
                    "scope": "instance",
                },
                "device_name":  {"type": "string", "description": "设备名称", "scope": "instance"},
            },
        },
        "topic_in": [],
        "topic_out": [{"format": "audio/pcm-16k", "desc": "external mic audio"}],
    }
]

TOOLS_EXT_CAMERA = [
    {
        "name": "ext_camera",
        "type": "sensor",
        "multiInstance": True,
        "description": "External camera (action cam / USB cam) — captures JPEG video",
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["start", "stop", "info"],
                    "description": "Action to perform",
                },
            },
            "required": ["action"],
        },
        "configSchema": {
            "type": "object",
            "properties": {
                "device_path": {"type": "string", "description": "设备路径 (如 /dev/video2)", "scope": "instance"},
                "device_name": {"type": "string", "description": "设备名称", "scope": "instance"},
                "fps":         {"type": "integer", "description": "帧率", "default": 15, "scope": "instance"},
                "resolution":  {"type": "string", "description": "分辨率 (如 1920x1080)", "default": "1920x1080", "scope": "instance"},
            },
        },
        "topic_in": [],
        "topic_out": [{"format": "image/jpeg", "desc": "external camera JPEG stream"}],
    }
]


class ExtMicPlugin:
    PREFIX = "ext_mic"

    def __init__(self, plugin_cfg: dict, namespace: str, executor, remote_devices: list = None):
        self._namespace = namespace
        self._executor = executor
        self._nodes: dict[str, _ExtMicNode] = {}
        self._instance_configs: dict[str, dict] = {}  # instance_id → saved config params
        self._available_devices = _enumerate_ext_mics()
        # Add dynamically probed remote devices (from main.py _probe_remote_mics)
        if remote_devices:
            self._available_devices.extend(remote_devices)
        elif plugin_cfg.get("network_sources"):
            # Fallback: static config (backward compat if probe not called)
            for ns in plugin_cfg.get("network_sources", []):
                url = ns.get("url", f"tcp://{ns.get('ssh_host', '')}:{ns.get('port', 9800)}/pcm16k")
                self._available_devices.append({
                    "index": url,
                    "alsa_id": url,
                    "name": ns.get("name", ns.get("ssh_host", "Remote Mic")),
                    "network": True,
                })
        log.info(f"[ext_mic] found {len(self._available_devices)} external mic device(s)")
        for d in self._available_devices:
            log.info(f"  [{d['index']}] {d['name']}")

    def get_tools(self) -> list:
        # Build dynamic configSchema with enumerated devices
        device_options = [{"const": d.get("alsa_id", str(d["index"])), "title": d["name"]} for d in self._available_devices]
        tool = dict(TOOLS_EXT_MIC[0])
        tool["configSchema"] = {
            "type": "object",
            "properties": {
                "device_index": {
                    "type": "string",
                    "description": "Audio device",
                    "scope": "instance",
                    "oneOf": device_options if device_options else [{"const": "", "title": "No devices available"}],
                },
                "ssh_host": {
                    "type": "string",
                    "description": "Remote host address",
                    "default": "192.168.41.1",
                },
                "ssh_user": {
                    "type": "string",
                    "description": "SSH username",
                    "default": "ubuntu",
                },
                "ssh_pass": {
                    "type": "string",
                    "format": "password",
                    "description": "SSH password",
                    "default": "123",
                },
            },
        }
        return [tool]

    def start(self) -> None:
        pass  # Don't auto-start — wait for canvas to start instances

    def stop(self) -> None:
        for key in list(self._nodes.keys()):
            self._nodes[key].stop()
            self._executor.remove_node(self._nodes[key])
            del self._nodes[key]

    def _probe_remote(self, ssh_host: str, ssh_user: str, ssh_pass: str,
                      port: int, script: str) -> list[dict]:
        """SSH probe remote host: detect devices, deploy & start audio_sender, cache results."""
        import subprocess as _sp
        from pathlib import Path

        def _ssh(cmd, timeout=10):
            return _sp.run(
                ["sshpass", "-p", ssh_pass, "ssh",
                 "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=3",
                 f"{ssh_user}@{ssh_host}", cmd],
                capture_output=True, text=True, timeout=timeout)

        # Remove old remote devices for this host
        self._available_devices = [d for d in self._available_devices
                                   if not (d.get("network") and d.get("_ssh_host") == ssh_host)]

        # Probe devices
        try:
            result = _ssh("arecord -l")
            devices = _parse_arecord_output(result.stdout + result.stderr)
        except Exception as e:
            log.warning(f"[ext_mic/probe] {ssh_host}: SSH failed: {e}")
            return []

        if not devices:
            log.info(f"[ext_mic/probe] {ssh_host}: no capture devices")
            return []

        # Probe hw params for each device
        for dev in devices:
            try:
                hw = _ssh(f"arecord -D hw:{dev['card']},{dev['device']} --dump-hw-params -d 1 /dev/null 2>&1")
                dev.update(_parse_hw_params(hw.stdout + hw.stderr))
            except Exception:
                dev.setdefault("format", "S16_LE")
                dev.setdefault("rate", 16000)
                dev.setdefault("channels", 1)

        # Deploy audio_sender.py if missing
        try:
            check = _ssh(f"test -f {script} && echo EXISTS")
            if "EXISTS" not in (check.stdout or ""):
                local_src = str(Path(__file__).parent / "audio_sender.py")
                _sp.run(["sshpass", "-p", ssh_pass, "scp",
                         "-o", "StrictHostKeyChecking=no",
                         local_src, f"{ssh_user}@{ssh_host}:{script}"],
                        check=True, timeout=15)
                log.info(f"[ext_mic/probe] {ssh_host}: deployed audio_sender.py")
        except Exception as e:
            log.warning(f"[ext_mic/probe] {ssh_host}: deploy failed: {e}")

        # Start audio_sender if not running
        primary_card = devices[0]["card"]
        try:
            check = _ssh("pgrep -f audio_sender.py")
            if check.returncode != 0 or not check.stdout.strip():
                _ssh(f"nohup python3 {script} --port {port} --card {primary_card} "
                     f"> /tmp/audio_sender.log 2>&1 &")
                time.sleep(1)
                log.info(f"[ext_mic/probe] {ssh_host}: started audio_sender (card={primary_card}, port={port})")
        except Exception:
            pass

        # Cache results and return summary
        probed = []
        for dev in devices:
            fmt_desc = f"{dev.get('format', '?')}/{dev.get('rate', '?')}Hz/{dev.get('channels', '?')}ch"
            entry = {
                "index": f"tcp://{ssh_host}:{port}/pcm16k",
                "alsa_id": f"tcp://{ssh_host}:{port}/pcm16k",
                "name": f"{dev['name']} ({fmt_desc}) @ {ssh_host}",
                "network": True,
                "_ssh_host": ssh_host,
                "_ssh_user": ssh_user,
                "_ssh_pass": ssh_pass,
                "_ssh_card": str(dev["card"]),
                "_ssh_script": script,
                "_port": port,
            }
            self._available_devices.append(entry)
            probed.append({"name": entry["name"], "url": entry["alsa_id"]})

        log.info(f"[ext_mic/probe] {ssh_host}: found {len(devices)} device(s)")
        return probed

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get("instance_id", "")

        if action == "info":
            if instance_id and instance_id in self._nodes:
                return self._nodes[instance_id]._status_dict()
            # Infer topic from namespace + instance_id even before start
            inferred_topic = f"/{self._namespace}/ext_mic/{instance_id.replace('-', '_')}/audio" if instance_id else ""
            return {
                "state": "idle",
                "available_devices": self._available_devices,
                "active_instances": list(self._nodes.keys()),
                "topic_in": [],
                "topic_out": [{"topic": inferred_topic, "format": "audio/pcm-16k", "desc": "external mic audio"}],
            }

        elif action == "start":
            if not instance_id:
                raise ValueError("instance_id is required for multiInstance tool")
            device_id = args.get("device_index")  # alsa_id string like "hw:0,0" or integer index
            device_name = args.get("device_name", "")
            # Fallback to saved config from prior 'config' action
            if not device_id and instance_id in self._instance_configs:
                device_id = self._instance_configs[instance_id].get("device_index")
                device_name = device_name or self._instance_configs[instance_id].get("device_name", "")
            if not device_id:
                # Try to pick first available device
                if self._available_devices:
                    device_id = self._available_devices[0].get("alsa_id", self._available_devices[0]["index"])
                    device_name = self._available_devices[0]["name"]
                else:
                    raise ValueError("No external mic device available")
            # Auto-resolve device_name from available devices if not provided
            if not device_name:
                for d in self._available_devices:
                    if d.get("alsa_id") == device_id or str(d.get("index")) == str(device_id):
                        device_name = d["name"]
                        break
            # Try to convert to int for sounddevice numeric index, keep string for alsa_id
            try:
                device_id = int(device_id)
            except (ValueError, TypeError):
                pass  # keep as string (alsa_id like "hw:0,0" or "tcp://...")
            if instance_id not in self._nodes:
                if isinstance(device_id, str) and (device_id.startswith("tcp://") or device_id.startswith("ssh://")):
                    # Extract SSH params from available_devices for health check/restart
                    ssh_params = {}
                    for d in self._available_devices:
                        if d.get("alsa_id") == device_id or d.get("index") == device_id:
                            ssh_params = {
                                "ssh_user": d.get("_ssh_user", ""),
                                "ssh_pass": d.get("_ssh_pass", ""),
                                "ssh_script": d.get("_ssh_script", ""),
                                "ssh_card": d.get("_ssh_card", "1"),
                            }
                            break
                    node = _NetworkMicNode(device_id, device_name, self._namespace, instance_id,
                                           context=self._executor.context, **ssh_params)
                else:
                    node = _ExtMicNode(device_id, device_name, self._namespace, instance_id,
                                       context=self._executor.context)
                self._executor.add_node(node)
                self._nodes[instance_id] = node
            return self._nodes[instance_id].start()

        elif action == "stop":
            if instance_id and instance_id in self._nodes:
                result = self._nodes[instance_id].stop()
                self._executor.remove_node(self._nodes[instance_id])
                del self._nodes[instance_id]
                return result
            elif not instance_id:
                # Stop all
                for key in list(self._nodes.keys()):
                    self._nodes[key].stop()
                    self._executor.remove_node(self._nodes[key])
                    del self._nodes[key]
                return {"state": "idle"}
            return {"state": "idle"}

        elif action == "config":
            if instance_id:
                # Instance config (per-card device selection)
                self._instance_configs[instance_id] = {
                    k: v for k, v in args.items() if k not in ('action', 'instance_id')
                }
                return {"configured": True, "instance_id": instance_id}
            else:
                # Shared config (sidebar SSH settings) — trigger remote probe
                ssh_host = args.get("ssh_host", "").strip()
                if ssh_host:
                    probed = self._probe_remote(
                        ssh_host=ssh_host,
                        ssh_user=args.get("ssh_user", "ubuntu"),
                        ssh_pass=args.get("ssh_pass", "123"),
                        port=9800,
                        script="/home/ubuntu/audio_sender.py",
                    )
                    return {"configured": True, "remote_devices": probed}
                return {"configured": True}

        return None


# ---------------------------------------------------------------------------
# V4L2 control helpers
# ---------------------------------------------------------------------------

def _parse_v4l2_controls(device_path: str) -> list[dict]:
    """Run v4l2-ctl --list-ctrls-menus and parse into structured control defs."""
    try:
        out = subprocess.check_output(
            ['v4l2-ctl', '-d', device_path, '--list-ctrls-menus'],
            text=True, timeout=3, stderr=subprocess.DEVNULL,
            env={**os.environ, 'LC_ALL': 'C'},
        )
    except Exception:
        return []

    controls: list[dict] = []
    current: dict | None = None

    for line in out.splitlines():
        # Control line: "  brightness 0x00980900 (int) : min=0 max=100 ..."
        ctrl_m = re.match(r'^\s+(\w+)\s+0x[0-9a-f]+\s+\((\w+)\)\s*:\s*(.*)$', line)
        if ctrl_m:
            name, ctype, attrs = ctrl_m.group(1), ctrl_m.group(2), ctrl_m.group(3)
            current = {'name': name, 'type': ctype}
            for key in ('min', 'max', 'default', 'step', 'value'):
                m = re.search(rf'(?<!\w){key}=(-?\d+)', attrs)
                if m:
                    current[key] = int(m.group(1))
            flags_m = re.search(r'flags=(\S+)', attrs)
            if flags_m:
                current['flags'] = flags_m.group(1)
            controls.append(current)
            continue

        # Menu entry: "        0: Disabled"  (no hex address, digit-colon format)
        if current and current['type'] == 'menu':
            menu_m = re.match(r'^\s+(\d+):\s+(.+)$', line)
            if menu_m:
                current.setdefault('menu_options', []).append({
                    'value': int(menu_m.group(1)),
                    'label': menu_m.group(2).strip(),
                })

    return controls


def _ctrl_to_schema_prop(ctrl: dict) -> dict:
    """Convert a parsed V4L2 control dict to a JSON-Schema property dict."""
    ctype = ctrl['type']
    desc_parts = []

    if ctrl.get('flags') == 'inactive':
        desc_parts.append('自动模式开启时不可用')
    if ctrl.get('step', 1) > 1:
        desc_parts.append(f"步进 {ctrl['step']}")

    prop: dict = {'description': '、'.join(desc_parts) if desc_parts else ctrl['name'].replace('_', ' ')}

    if ctype == 'int':
        prop['type'] = 'integer'
        if 'min' in ctrl: prop['minimum'] = ctrl['min']
        if 'max' in ctrl: prop['maximum'] = ctrl['max']
        if 'default' in ctrl: prop['default'] = ctrl['default']
    elif ctype == 'bool':
        prop['type'] = 'boolean'
        if 'default' in ctrl: prop['default'] = bool(ctrl['default'])
    elif ctype == 'menu':
        prop['type'] = 'integer'
        options = ctrl.get('menu_options', [])
        if options:
            prop['oneOf'] = [{'const': o['value'], 'title': o['label']} for o in options]
        if 'default' in ctrl: prop['default'] = ctrl['default']
    else:
        prop['type'] = 'string'

    return prop


class ExtCameraPlugin:
    PREFIX = "ext_camera"

    def __init__(self, plugin_cfg: dict, namespace: str, executor):
        self._namespace = namespace
        self._executor = executor
        self._nodes: dict[str, _ExtCameraNode] = {}
        self._instance_configs: dict[str, dict] = {}
        self._available_devices = _enumerate_ext_cameras()
        log.info(f"[ext_camera] found {len(self._available_devices)} external camera device(s)")
        for d in self._available_devices:
            log.info(f"  {d['path']} — {d['name']}")
        # Parse V4L2 controls per device; merge into deduplicated dict (first device wins)
        self._device_controls: dict[str, list[dict]] = {}
        self._merged_controls: dict[str, dict] = {}
        for d in self._available_devices:
            ctrls = _parse_v4l2_controls(d['path'])
            if ctrls:
                self._device_controls[d['path']] = ctrls
                log.info(f"[ext_camera] {d['path']}: {len(ctrls)} controls discovered")
                for c in ctrls:
                    self._merged_controls.setdefault(c['name'], c)

    def get_tools(self) -> list:
        # Build dynamic configSchema with enumerated devices
        device_options = [{"const": d["path"], "title": f"{d['name']} ({d['path']})"} for d in self._available_devices]
        # Collect all unique formats across devices for pixel_format selector
        all_formats: list[str] = []
        for d in self._available_devices:
            for f in d.get("formats", []):
                if f not in all_formats:
                    all_formats.append(f)
        format_options = [{"const": "auto", "title": "自动"}] + [{"const": f, "title": f} for f in all_formats]
        # Collect all unique resolutions across devices
        all_resolutions: list[str] = []
        for d in self._available_devices:
            for r in d.get("resolutions", []):
                if r not in all_resolutions:
                    all_resolutions.append(r)
        resolution_options = [{"const": r, "title": r} for r in all_resolutions] or [{"const": "1920x1080", "title": "1920x1080"}]
        tool = dict(TOOLS_EXT_CAMERA[0])
        tool["configSchema"] = {
            "type": "object",
            "properties": {
                "device_path": {
                    "type": "string",
                    "description": "摄像头设备",
                    "scope": "instance",
                    "oneOf": device_options if device_options else [{"const": "", "title": "无可用设备"}],
                },
                "device_name": {"type": "string", "description": "设备名称", "scope": "instance"},
                "fps": {"type": "integer", "description": "帧率", "default": 15, "scope": "instance"},
                "resolution": {
                    "type": "string",
                    "description": "分辨率",
                    "default": "1920x1080",
                    "scope": "instance",
                    "oneOf": resolution_options,
                },
                "pixel_format": {
                    "type": "string",
                    "description": "像素格式",
                    "default": "auto",
                    "scope": "instance",
                    "oneOf": format_options,
                },
            },
        }
        # Expand action enum with flattened set_*/get_* actions for each V4L2 control
        if self._merged_controls:
            ctrl_action_entries = []
            for name, ctrl in self._merged_controls.items():
                min_v = ctrl.get('min', '')
                max_v = ctrl.get('max', '')
                range_str = f" [{min_v}, {max_v}]" if min_v != '' and max_v != '' else ""
                ctrl_action_entries.append({"const": f"set_{name}", "title": f"set_{name} — {name.replace('_', ' ')}{range_str}"})
                ctrl_action_entries.append({"const": f"get_{name}", "title": f"get_{name} — 读取 {name.replace('_', ' ')}"})
            input_schema = dict(tool["inputSchema"])
            input_props = dict(input_schema["properties"])
            input_props["action"] = {
                "type": "string",
                "description": "操作类型",
                "oneOf": [
                    {"const": "start", "title": "start"},
                    {"const": "stop",  "title": "stop"},
                    {"const": "info",  "title": "info"},
                ] + ctrl_action_entries,
            }
            input_props["value"] = {
                "type": "integer",
                "description": "设置目标值（仅 set_* 动作需要）",
            }
            input_schema["properties"] = input_props
            tool["inputSchema"] = input_schema
        return [tool]

    def start(self) -> None:
        pass  # Don't auto-start

    def stop(self) -> None:
        for key in list(self._nodes.keys()):
            self._nodes[key].stop()
            del self._nodes[key]

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get("instance_id", "")
        print(f"[ext_camera] dispatch: action={action!r} instance_id={instance_id!r} args_keys={list(args.keys())}", flush=True)

        if action == 'config':
            if instance_id:
                self._instance_configs[instance_id] = {k: v for k, v in args.items()
                                                        if k not in ('action', 'instance_id', '_tool_name')}
                print(f"[ext_camera] config cached for instance {instance_id}: {self._instance_configs[instance_id]}", flush=True)
            return {'ok': True}

        if action == "info":
            if instance_id and instance_id in self._nodes:
                return self._nodes[instance_id]._status_dict()
            # Infer topic from namespace + instance_id even before start
            inferred_topic = f"/{self._namespace}/ext_camera/{instance_id.replace('-', '_')}/rgb" if instance_id else ""
            return {
                "state": "idle",
                "available_devices": self._available_devices,
                "active_instances": list(self._nodes.keys()),
                "topic_in": [],
                "topic_out": [{"topic": inferred_topic, "format": "image/jpeg", "desc": "external camera JPEG"}],
            }

        elif action == "start":
            if not instance_id:
                raise ValueError("instance_id is required for multiInstance tool")
            # Merge cached config into args (config is sent before start)
            if instance_id in self._instance_configs:
                merged = {**self._instance_configs[instance_id], **{k: v for k, v in args.items() if k not in ('action', 'instance_id', '_tool_name')}}
                args.update(merged)
            device_path = args.get("device_path")
            device_name = args.get("device_name", "")
            if not device_path:
                if self._available_devices:
                    device_path = self._available_devices[0]["path"]
                    device_name = self._available_devices[0]["name"]
                else:
                    raise ValueError("No external camera device available")
            # Resolve available formats for the selected device
            available_formats: list[str] = []
            for d in self._available_devices:
                if d["path"] == device_path:
                    available_formats = d.get("formats", [])
                    break
            # Parse resolution
            resolution = args.get("resolution", "1920x1080")
            try:
                w, h = resolution.lower().split('x')
                width, height = int(w), int(h)
            except Exception:
                width, height = 1920, 1080
            fps = int(args.get("fps", 15))
            pixel_format = args.get("pixel_format", "auto")

            if instance_id not in self._nodes:
                node = _ExtCameraNode(device_path, device_name, self._namespace, instance_id,
                                      fps=fps, width=width, height=height,
                                      pixel_format=pixel_format, available_formats=available_formats)
                self._nodes[instance_id] = node
            return self._nodes[instance_id].start()

        elif action == "stop":
            if instance_id and instance_id in self._nodes:
                result = self._nodes[instance_id].stop()
                del self._nodes[instance_id]
                return result
            elif not instance_id:
                for key in list(self._nodes.keys()):
                    self._nodes[key].stop()
                    del self._nodes[key]
                return {"state": "idle"}
            return {"state": "idle"}

        elif action.startswith('set_'):
            ctrl_name = action[4:]
            device_path = self._resolve_device_path(instance_id, args)
            value = args.get('value')
            if value is None:
                raise ValueError(f"'value' is required for {action}")
            print(f"[ext_camera] set_ctrl: device={device_path} ctrl={ctrl_name} value={value}", flush=True)
            try:
                out = subprocess.check_output(
                    ['v4l2-ctl', '-d', device_path, f'--set-ctrl={ctrl_name}={value}'],
                    text=True, timeout=5, stderr=subprocess.PIPE,
                    env={**os.environ, 'LC_ALL': 'C'},
                )
                print(f"[ext_camera] set_ctrl ok: {out.strip()!r}", flush=True)
            except subprocess.CalledProcessError as e:
                print(f"[ext_camera] set_ctrl failed: {e.stderr.strip()}", flush=True)
                raise RuntimeError(f'v4l2-ctl set failed: {e.stderr.strip()}')
            return {'ok': True, 'ctrl': ctrl_name, 'value': value}

        elif action.startswith('get_'):
            ctrl_name = action[4:]
            device_path = self._resolve_device_path(instance_id, args)
            print(f"[ext_camera] get_ctrl: device={device_path} ctrl={ctrl_name}", flush=True)
            return self._ctrl_get_one(device_path, ctrl_name)

        return None

    def _resolve_device_path(self, instance_id: str, args: dict) -> str:
        if instance_id and instance_id in self._nodes:
            return self._nodes[instance_id]._device_path
        if instance_id and instance_id in self._instance_configs:
            dp = self._instance_configs[instance_id].get('device_path', '')
            if dp:
                print(f"[ext_camera] _resolve_device_path: using cached config for {instance_id} → {dp}", flush=True)
                return dp
        dp = args.get('device_path')
        if dp:
            return dp
        raise ValueError('device_path required (configure instance first or start an instance)')

    def _ctrl_get_one(self, device_path: str, ctrl_name: str) -> dict:
        try:
            out = subprocess.check_output(
                ['v4l2-ctl', '-d', device_path, f'--get-ctrl={ctrl_name}'],
                text=True, timeout=3, stderr=subprocess.DEVNULL,
                env={**os.environ, 'LC_ALL': 'C'},
            )
            m = re.search(r':\s*(-?\d+)', out)
            return {'ctrl': ctrl_name, 'value': int(m.group(1)) if m else None, 'raw': out.strip()}
        except Exception as e:
            return {'ctrl': ctrl_name, 'error': str(e)}
