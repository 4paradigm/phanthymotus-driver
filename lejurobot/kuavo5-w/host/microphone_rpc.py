"""Image-owned audio input probe/stream. Runs as the selected SSH user, no sudo."""
import json
import os
from pathlib import Path
import re
import select
import shutil
import struct
import subprocess
import sys
import time

BLOCK = 1024  # 32 ms, signed mono PCM16 at 16 kHz


def probe(asound=Path('/proc/asound')):
    devices = []
    if shutil.which('arecord') and (asound/'pcm').is_file():
        for line in (asound/'pcm').read_text().splitlines():
            match = re.match(r'(\d+)-(\d+): (.+?) :.*capture [1-9]', line)
            if not match:
                continue
            card, dev, description = match.groups()
            directory = asound / ('card'+str(int(card)))
            card_id = (directory/'id').read_text().strip()
            # APE exposes routing endpoints even with no physical mic connected.
            if card_id == 'APE' or not re.fullmatch(r'[A-Za-z0-9_-]+', card_id):
                continue
            devices.append({'id': 'alsa/plughw:CARD='+card_id+',DEV='+str(int(dev)),
                            'name': description, 'usb': (directory/'usbid').is_file(),
                            'acoustically_verified': False})
    if shutil.which('pactl') and shutil.which('parecord'):
        result = subprocess.run(['pactl', 'list', 'short', 'sources'], capture_output=True,
                                text=True, timeout=5)
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                fields = line.split()
                if len(fields) < 2:
                    continue
                name = fields[1]
                # Monitor sources are playback loopback, never microphones.
                if (name.startswith('alsa_input.') and not name.endswith('.monitor')
                        and re.fullmatch(r'[A-Za-z0-9_.:-]+', name)):
                    devices.append({'id': 'pulse/'+name, 'name': name,
                                    'usb': '.usb-' in name, 'acoustically_verified': False})
    return {'devices': devices, 'format': 'audio/pcm-16k'}


def capture_command(device):
    # Always re-enumerate, so a stale selection cannot open an arbitrary PCM/plugin.
    if device not in {entry['id'] for entry in probe()['devices']}:
        raise ValueError('capture_device_unavailable_refresh_devices')
    backend, name = device.split('/', 1)
    if backend == 'pulse':
        return ['parecord', '--raw', '--format=s16le', '--rate=16000', '--channels=1',
                '--latency-msec=40', '--device='+name]
    return ['arecord', '-q', '-D', name, '-t', 'raw', '-f', 'S16_LE', '-r', '16000', '-c', '1']


def stream(device, lease_s=10.):
    process = subprocess.Popen(capture_command(device), stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, bufsize=0)
    try:
        # A bounded initial line; the client then reads fixed-size binary frames.
        sys.stdout.buffer.write(b'{"ok":true,"format":"audio/pcm-16k"}\n')
        sys.stdout.buffer.flush()
        buffer = b''
        warmup_bytes = 32000  # Discard one second of analog input startup transients.
        lease = time.monotonic() + lease_s
        last_audio = time.monotonic()
        os.set_blocking(sys.stdin.fileno(), False)
        os.set_blocking(sys.stdout.fileno(), False)
        while process.poll() is None:
            now = time.monotonic()
            if now >= lease or now-last_audio > 5.:
                break
            ready, _, _ = select.select([sys.stdin.buffer, process.stdout], [], [], .2)
            if sys.stdin.buffer in ready:
                heartbeat = os.read(sys.stdin.fileno(), 256)
                if not heartbeat:
                    break  # Driver closed its SSH stdin: stop the recorder.
                lease = time.monotonic() + lease_s
            if process.stdout not in ready:
                continue
            data = os.read(process.stdout.fileno(), BLOCK)
            if not data:
                break
            last_audio = time.monotonic()
            if warmup_bytes:
                discarded = min(warmup_bytes, len(data))
                data = data[discarded:]
                warmup_bytes -= discarded
            buffer += data
            while len(buffer) >= BLOCK:
                chunk, buffer = buffer[:BLOCK], buffer[BLOCK:]
                packet = struct.pack('!dI', time.time(), len(chunk)) + chunk
                deadline = time.monotonic()+1.
                # Do not accumulate delayed audio across a blocked SSH connection.
                while packet:
                    remaining = deadline-time.monotonic()
                    if remaining <= 0 or not select.select([], [sys.stdout.buffer], [], remaining)[1]:
                        return
                    try:
                        count = os.write(sys.stdout.fileno(), packet)
                    except BlockingIOError:
                        continue
                    packet = packet[count:]
    finally:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        process.stdout.close()


def main():
    request = json.loads(sys.stdin.buffer.readline(16385))
    if request.get('operation') == 'probe':
        print(json.dumps({'ok': True, 'result': probe()}), flush=True)
    elif request.get('operation') == 'capture':
        stream(request.get('device'))
    else:
        raise ValueError('unsupported_microphone_operation')


if __name__ == '__main__':
    try:
        main()
    except Exception:
        # No raw device errors or SSH metadata in the audio transport.
        sys.exit(1)
