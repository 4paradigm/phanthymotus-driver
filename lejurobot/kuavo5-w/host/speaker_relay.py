#!/usr/bin/env python3
"""Chassis-side PCM sink. Standard ROS1 messages; no motor/controller access.

Each start allocates a new lease. Old sessions/sequence numbers never replay.
Only this service's paplay child is stopped; other robot prompts are untouched.
"""
import argparse
import base64
import json
import os
import shutil
import subprocess
import threading
import time
from collections import deque
from uuid import uuid4

ROOT = '/motus/kuavo5w/speaker'
SINK = 'alsa_output.pci-0000_00_1f.3.analog-stereo'


class PcmSink:
    def __init__(self, volume=30):
        self.volume = volume
        self.lock = threading.RLock()
        self.session = None
        self.process = None
        self.queue = deque()
        self.pending = b''
        self.queued_bytes = 0
        self.sequence = -1
        self.last_seen = 0.
        self.error = None
        self.bytes_written = 0

    def _stop(self):
        self.session = None
        self.queue.clear()
        self.pending = b''
        self.queued_bytes = 0
        process, self.process = self.process, None
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=.3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=.3)
            process.stdin.close()

    def stop(self):
        with self.lock:
            self._stop()
            return self.status()

    def start(self):
        with self.lock:
            self._stop()
            if not shutil.which('paplay') or not shutil.which('pactl'):
                raise RuntimeError('paplay/pactl are not installed on the chassis')
            sinks = subprocess.run(['pactl', 'list', 'short', 'sinks'], check=True,
                                   capture_output=True, text=True, timeout=1).stdout
            if not any(len(row.split()) > 1 and row.split()[1] == SINK
                       for row in sinks.splitlines()):
                raise RuntimeError('verified chassis analog sink is unavailable')
            self.session = uuid4().hex
            self.sequence = -1
            self.last_seen = time.monotonic()
            self.error = None
            self.bytes_written = 0
            return self.status()

    def status(self):
        with self.lock:
            return {'session': self.session, 'state': 'error' if self.error else
                    ('ready' if self.session else 'idle'), 'error': self.error,
                    'buffer_bytes': self.queued_bytes, 'bytes_written': self.bytes_written,
                    'volume_percent': self.volume, 'sink': SINK}

    def receive(self, raw):
        # Callback only validates/enqueues; subprocess writes run in tick().
        if not isinstance(raw, str) or len(raw) > 6000:
            return
        try:
            packet = json.loads(raw)
            with self.lock:
                if not self.session or packet.get('session') != self.session:
                    return
                sequence = packet.get('seq')
                if type(sequence) is not int or sequence <= self.sequence:
                    return
                encoded = packet.get('pcm', '')
                if not isinstance(encoded, str):
                    raise ValueError('invalid pcm')
                pcm = base64.b64decode(encoded, validate=True)
                if len(pcm) > 3200 or len(pcm) % 2:
                    raise ValueError('invalid PCM16k block')
                if self.queued_bytes + len(pcm) > 32000:
                    self.error = 'speaker queue overflow'
                    self._stop()
                    return
                self.sequence = sequence
                self.last_seen = time.monotonic()
                if pcm:
                    self.queue.append(pcm)
                    self.queued_bytes += len(pcm)
        except (ValueError, TypeError, AttributeError):
            # Malformed/oversized packets do not renew the playback lease.
            return

    def tick(self):
        with self.lock:
            if not self.session:
                return
            try:
                if time.monotonic() - self.last_seen > 1.5:
                    raise RuntimeError('speaker input lease expired')
                if self.process is not None and self.process.poll() is not None:
                    raise RuntimeError('paplay exited')
                if self.pending or self.queue:
                    if self.process is None:
                        self.process = subprocess.Popen([
                            'paplay', '--raw', '--format=s16le', '--rate=16000',
                            '--channels=1', '--latency-msec=60', '--device=' + SINK,
                            '--volume=' + str(round(65536 * self.volume / 100))],
                            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, bufsize=0)
                        os.set_blocking(self.process.stdin.fileno(), False)
                    if not self.pending:
                        self.pending = self.queue.popleft()
                    try:
                        count = os.write(self.process.stdin.fileno(), self.pending)
                    except BlockingIOError:
                        return
                    self.pending = self.pending[count:]
                    self.queued_bytes -= count
                    self.bytes_written += count
            except Exception as exc:
                self.error = type(exc).__name__ + ': ' + str(exc)[:160]
                self._stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--volume', type=int, default=30)
    args = parser.parse_args()
    if not 0 <= args.volume <= 100:
        parser.error('volume must be from 0 to 100')
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse
    rospy.init_node('motus_kuavo5w_speaker')
    sink = PcmSink(args.volume)

    def call(operation):
        try:
            return TriggerResponse(success=True, message=json.dumps(operation()))
        except Exception as exc:
            return TriggerResponse(success=False, message=type(exc).__name__)

    services = [rospy.Service(ROOT + '/' + name, Trigger,
                lambda request, fn=operation: call(fn)) for name, operation in
                (('start', sink.start), ('stop', sink.stop), ('status', sink.status))]
    sub = rospy.Subscriber(ROOT + '/pcm', String, lambda msg: sink.receive(msg.data), queue_size=10)
    rospy.on_shutdown(sink.stop)
    # Wall time watchdog remains live when ROS simulated time stops.
    while not rospy.is_shutdown():
        sink.tick()
        time.sleep(.01)


if __name__ == '__main__':
    main()
