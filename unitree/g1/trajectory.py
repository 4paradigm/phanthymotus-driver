"""G1 JSONL trajectories. Bounded writer pattern follows PiPER trajectory.py.

Only completed SDK references enter this file; a file is not motion permission.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import stat
import threading
import time
from uuid import uuid4

SCHEMA = 'motus.g1.trajectory/1'
MAX_BYTES, MAX_FRAMES, MAX_DURATION_NS = 64*1024*1024, 120000, 600_000_000_000
MAX_LINE = 32768
NAME = re.compile(r'[A-Za-z0-9_-]{1,96}\.jsonl')
UNITS = {'q': 'rad', 'dq': 'rad/s', 'time': 'ns'}


def encode(value):
    raw = (json.dumps(value, allow_nan=False, separators=(',', ':'))+'\n').encode()
    if len(raw) > MAX_LINE: raise ValueError('trajectory_line_too_large')
    return raw


def unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result: raise ValueError('trajectory_duplicate_field')
        result[key] = value
    return result


def vector(value):
    if not isinstance(value, list) or len(value) != 10 or any(
            type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError('trajectory_vector')


def validate_frame(frame, previous, limits):
    keys = {'type', 'kind', 't_ns', 'target_q', 'feedback_q', 'feedback_dq', 'feedback_ns',
            'source_sequence', 'mapping_epoch', 'session_id', 'sdk_sequence'}
    if not isinstance(frame, dict) or set(frame) != keys or frame['type'] != 'frame' or frame['kind'] not in ('target', 'hold'):
        raise ValueError('trajectory_frame')
    for key in ('t_ns', 'feedback_ns', 'source_sequence', 'mapping_epoch', 'sdk_sequence'):
        if type(frame[key]) is not int or frame[key] < 0: raise ValueError('trajectory_time_or_sequence')
    if not isinstance(frame['session_id'], str) or not frame['session_id']: raise ValueError('trajectory_session')
    if frame['t_ns'] > MAX_DURATION_NS or previous and frame['t_ns'] <= previous['t_ns']:
        raise ValueError('trajectory_time_order')
    for key in ('target_q', 'feedback_q', 'feedback_dq'): vector(frame[key])
    if any(not lo <= q <= hi for q, (lo, hi) in zip(frame['target_q'], limits)):
        raise ValueError('trajectory_joint_limit')
    if previous:
        if frame['mapping_epoch'] != previous['mapping_epoch']: raise ValueError('trajectory_mapping_changed')


def read_recording(directory, filename, limits):
    if not isinstance(filename, str) or not NAME.fullmatch(filename): raise ValueError('invalid_recording_filename')
    fd = os.open(Path(directory)/filename, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as stream:
        meta = os.fstat(stream.fileno())
        if not stat.S_ISREG(meta.st_mode) or meta.st_size > MAX_BYTES: raise ValueError('invalid_recording_file')
        digest, full, frames, header, footer, total = hashlib.sha256(), hashlib.sha256(), [], None, None, 0
        while raw := stream.readline(MAX_LINE+1):
            total += len(raw)
            if total > MAX_BYTES or len(raw) > MAX_LINE or not raw.endswith(b'\n'):
                raise ValueError('trajectory_truncated_or_oversized')
            full.update(raw)
            value = json.loads(raw, object_pairs_hook=unique)
            if not isinstance(value, dict) or footer is not None: raise ValueError('trajectory_order')
            if header is None:
                if (set(value) != {'type', 'schema', 'units', 'binding', 'joint_names'} or
                        value['type'] != 'header' or value['schema'] != SCHEMA or value['units'] != UNITS or
                        not isinstance(value['binding'], dict)):
                    raise ValueError('trajectory_header')
                header = value
            elif value.get('type') == 'complete':
                checksum = digest.copy()
                checksum.update(str(value.get('duration_ns')).encode())
                if (set(value) != {'type', 'frames', 'sha256', 'duration_ns'} or
                        type(value['frames']) is not int or value['frames'] != len(frames) or len(frames) < 2 or
                        value['sha256'] != checksum.hexdigest() or type(value['duration_ns']) is not int or
                        not frames[-1]['t_ns'] <= value['duration_ns'] <= MAX_DURATION_NS):
                    raise ValueError('trajectory_checksum_or_count')
                footer = value
                continue
            else:
                if len(frames) >= MAX_FRAMES: raise ValueError('trajectory_too_long')
                validate_frame(value, frames[-1] if frames else None, limits)
                frames.append(value)
            digest.update(raw)
        if footer is None: raise ValueError('recording_incomplete')
    return {'filename': filename, 'sha256': full.hexdigest(),
            'binding': header['binding'], 'joint_names': header['joint_names'], 'frames': frames,
            'duration_seconds': footer['duration_ns']/1e9, 'duration_ns': footer['duration_ns']}


class Recorder:
    def __init__(self, directory, binding, joint_names, limits, *, clock=time.monotonic_ns):
        self.clock, self.limits = clock, limits
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.filename = 'g1-'+uuid4().hex+'.jsonl'
        self.partial = self.directory/(self.filename+'.partial')
        self.header = {'type': 'header', 'schema': SCHEMA, 'binding': binding, 'joint_names': joint_names, 'units': UNITS}
        self.queue = queue.Queue(maxsize=256)
        self.lock = threading.RLock()
        self.stopping, self.ready = threading.Event(), threading.Event()
        self.error, self.complete, self.count = None, False, 0
        self._last_frame = None
        self.origin_ns, self.end_ns = clock(), None
        self.thread = threading.Thread(target=self._write, name='g1-recorder', daemon=True)
        self.thread.start()
        if not self.ready.wait(2): self.fail('recording_open_timeout')
        if self.error: raise RuntimeError(self.error)

    def fail(self, reason):
        with self.lock:
            self.error = self.error or str(reason)
            self.stopping.set()

    def append(self, frame):
        with self.lock:
            if self.stopping.is_set(): return
            try:
                value = dict(frame, type='frame', t_ns=frame['t_ns']-self.origin_ns)
                self.queue.put_nowait(encode(value))
                self._last_frame = dict(frame)
            except (queue.Full, ValueError) as exc:
                self.error = 'recording_queue_full' if isinstance(exc, queue.Full) else str(exc)
                self.stopping.set()

    def pause(self):
        with self.lock:
            frame = self._last_frame
            now = self.clock()
            if frame and frame['kind'] != 'hold' and now > frame['t_ns']:
                # Keep the last accepted reference and its original feedback timestamp.
                self.append(dict(frame, kind='hold', t_ns=now))

    def _write(self):
        try:
            fd = os.open(self.partial, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, 'wb') as stream:
                raw = encode(self.header)
                stream.write(raw); stream.flush(); self.ready.set()
                digest, total, previous = hashlib.sha256(raw), len(raw), None
                while not self.stopping.is_set() or not self.queue.empty():
                    if self.error: break
                    try: raw = self.queue.get(timeout=.05)
                    except queue.Empty: continue
                    value = json.loads(raw)
                    validate_frame(value, previous, self.limits)
                    total += len(raw)
                    if self.count >= MAX_FRAMES or total+MAX_LINE > MAX_BYTES: raise ValueError('recording_limit')
                    stream.write(raw); digest.update(raw); self.count += 1; previous = value
                if not self.error:
                    duration = (self.end_ns or self.clock())-self.origin_ns
                    if self.count < 2 or duration > MAX_DURATION_NS: raise ValueError('recording_duration_or_count')
                    digest.update(str(duration).encode())
                    stream.write(encode({'type': 'complete', 'frames': self.count,
                                         'sha256': digest.hexdigest(), 'duration_ns': duration}))
                    stream.flush(); os.fsync(stream.fileno())
            if not self.error:
                self.partial.rename(self.directory/self.filename)
                directory_fd = os.open(self.directory, os.O_RDONLY)
                try: os.fsync(directory_fd)
                finally: os.close(directory_fd)
                self.complete = True
        except Exception as exc:
            self.fail(str(exc))
        finally:
            self.ready.set()

    def stop(self):
        with self.lock:
            self.end_ns = self.end_ns or self.clock()
            self.stopping.set()
        self.thread.join(2)
        return self.status()

    def status(self):
        return {'filename': self.filename if self.complete else self.partial.name, 'frames': self.count,
                'complete': self.complete, 'error': self.error, 'started_monotonic_ns': self.origin_ns,
                'recording': not self.stopping.is_set() and self.thread.is_alive(),
                'stopping': self.stopping.is_set() and self.thread.is_alive()}


def list_recordings(directory):
    root = Path(directory)
    if not root.exists(): return []
    return [{'filename': p.name, 'bytes': p.stat().st_size, 'complete': not p.name.endswith('.partial')}
            for p in sorted(root.iterdir()) if not p.is_symlink() and p.is_file()
            and (NAME.fullmatch(p.name) or p.name.endswith('.jsonl.partial'))]
