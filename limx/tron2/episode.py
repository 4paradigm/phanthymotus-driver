"""TRON2 episodes: original JPEG video packets, independent clocks, atomic receipt."""
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import threading
import time
from uuid import uuid4

from camera import jpeg_size
from skeleton import JOINT_NAMES
from state import vector

SCHEMA = 'motus.tron2.episode/1'
TRAJECTORY_SCHEMA = 'motus.tron2.trajectory/1'
CAMERAS = ('head', 'left_wrist', 'right_wrist')
STATE_NAMES = JOINT_NAMES[:14] + ['left_gripper_opening', 'right_gripper_opening']
UNITS = ['rad'] * 14 + ['vendor_0_to_100'] * 2
MAX_TRAJECTORY = 64 * 1024 * 1024


def encode(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n').encode()


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def atomic_json(path, value):
    temp = path.with_suffix('.pending')
    with temp.open('xb') as stream:
        stream.write(encode(value)); stream.flush(); os.fsync(stream.fileno())
    os.replace(temp, path)
    fd = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def episode_path(root, ident):
    if not isinstance(ident, str) or not re.fullmatch('[a-f0-9]{32}', ident):
        raise ValueError('invalid_episode_id')
    path = Path(root) / ident
    if path.is_symlink():
        raise ValueError('episode_symlink')
    return path


def timestamp(value):
    if type(value) is not int or value < 0:
        raise ValueError('invalid_monotonic_timestamp')
    return value


def validate_state(frame):
    timestamp(frame['received_ns'])
    vector(frame['q'], 16, 'q')
    if frame.get('gripper') is not None:
        values = vector(frame['gripper'], 2, 'gripper')
        if any(not 0 <= v <= 100 for v in values):
            raise ValueError('invalid_gripper_range')
        timestamp(frame['gripper_received_ns'])
    target = frame.get('vendor_target')
    if target is not None:
        vector(target['q'], 16, 'vendor_target')
        timestamp(target['received_ns'])
    if type(frame['sequence']) is not int or frame['sequence'] < 0:
        raise ValueError('invalid_sequence')


class EpisodeWriter:
    def __init__(self, root, binding, task, outcome='unlabelled', notes='', capacity=256, reserve_bytes=256*1024*1024):
        if not isinstance(task, str) or not task.strip() or len(task) > 2000:
            raise ValueError('task_required_max_2000')
        if outcome not in ('success', 'failure', 'unlabelled') or not isinstance(notes, str) or len(notes) > 4000:
            raise ValueError('invalid_episode_annotation')
        if not binding.get('serial') or not binding.get('model_sha256'):
            raise ValueError('device_binding_required')
        self.ident = uuid4().hex
        self.path = episode_path(root, self.ident)
        self.path.mkdir(parents=True, mode=0o700)
        self.manifest = dict(schema=SCHEMA, id=self.ident, binding=binding, task=task, outcome=outcome, notes=notes,
            joint_names=JOINT_NAMES, state_names=STATE_NAMES, units=UNITS, clock='host_monotonic_ns',
            source_clock='vendor_opaque_unverified', measurement_time_verified=False,
            image_time='host_receive_not_exposure', hardware_synchronized=False,
            action_source='measured_future_state', replay_source='measured_state', complete=False,
            started_unix_ns=time.time_ns(), cameras={name:{'codec':'mjpeg_original_packets'} for name in CAMERAS})
        atomic_json(self.path/'manifest.json', self.manifest)
        self.queue = queue.Queue(capacity)
        self.reserve = reserve_bytes
        self.error = None
        self.accepting = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._write, name='tron2-episode-writer', daemon=True)
        self.thread.start()

    def put(self, kind, value):
        with self.lock:
            if not self.accepting:
                return False
            try:
                self.queue.put_nowait((kind, value))
                return True
            except queue.Full:
                self.error = 'writer_queue_overflow'
                self.accepting = False
                return False

    def fail(self, reason):
        with self.lock:
            self.error = self.error or str(reason)[:240]
            self.accepting = False

    def finish(self):
        with self.lock:
            self.accepting = False
        self.thread.join(15)
        if self.thread.is_alive():
            raise RuntimeError('writer_still_finishing')
        return self.manifest

    def _write(self):
        streams = {}
        counts = {k:0 for k in ('state', *CAMERAS)}
        bounds = {}
        previous = {}
        sequence = -1
        grippers = 0
        gaps = 0
        try:
            streams['state'] = (self.path/'trajectory.jsonl').open('xb')
            for name in CAMERAS:
                streams[name] = (self.path/(name+'.mjpeg')).open('xb')
                streams[name+'_index'] = (self.path/(name+'.jsonl')).open('xb')
            while self.accepting or not self.queue.empty():
                try:
                    kind, value = self.queue.get(timeout=.05)
                except queue.Empty:
                    continue
                if shutil.disk_usage(self.path).free < self.reserve:
                    raise OSError('disk_reserve_exhausted')
                stamp = timestamp(value['received_ns'])
                if kind in previous and stamp <= previous[kind]:
                    raise ValueError('nonadvancing_receive_clock')
                if kind == 'state':
                    validate_state(value)
                    if value['sequence'] <= sequence:
                        raise ValueError('nonadvancing_sequence')
                    if kind in previous and stamp-previous[kind] > 100_000_000:
                        gaps += 1
                    sequence = value['sequence']
                    grippers += value.get('gripper') is not None
                    streams[kind].write(encode(value))
                    if streams[kind].tell() > MAX_TRAJECTORY:
                        raise ValueError('trajectory_size_limit')
                elif kind in CAMERAS:
                    width, height = jpeg_size(value['jpeg'])
                    shape = [width, height]
                    meta = self.manifest['cameras'][kind]
                    if 'size' in meta and meta['size'] != shape:
                        raise ValueError('camera_dimensions_changed')
                    meta['size'] = shape
                    offset = streams[kind].tell()
                    streams[kind].write(value['jpeg'])
                    streams[kind+'_index'].write(encode(dict(received_ns=stamp, source_timestamp=value.get('source_timestamp'),
                        sequence=counts[kind], offset=offset, bytes=len(value['jpeg']))))
                else:
                    raise ValueError('unknown_recording_stream')
                counts[kind] += 1
                previous[kind] = stamp
                bounds.setdefault(kind, [stamp, stamp])[1] = stamp
                if max(previous.values())-min(x[0] for x in bounds.values()) > 1800_000_000_000:
                    raise ValueError('recording_duration_limit')
        except Exception as exc:
            self.fail(type(exc).__name__+': '+str(exc))
        finally:
            while not self.queue.empty():
                self.queue.get_nowait()
            for stream in streams.values():
                try:
                    stream.flush(); os.fsync(stream.fileno()); stream.close()
                except OSError as exc:
                    self.fail(str(exc))
            complete = self.error is None and counts['state'] >= 2
            training = complete and grippers == counts['state'] and all(counts[k] >= 2 for k in CAMERAS)
            self.manifest.update(complete=complete, training_complete=training,
                replay_complete=complete and grippers == counts['state'], error=self.error,
                quality=dict(counts=counts, gripper_samples=grippers, state_gaps_over_100ms=gaps,
                    observed_hz={k:(counts[k]-1)*1e9/(v[1]-v[0]) if v[1]>v[0] else 0 for k,v in bounds.items()}))
            try:
                files = {p.name:{'bytes':p.stat().st_size,'sha256':digest(p)} for p in self.path.iterdir()
                         if p.suffix in ('.jsonl','.mjpeg')}
                self.manifest['files'] = files
                atomic_json(self.path/'manifest.json', self.manifest)
                if complete:
                    atomic_json(self.path/'complete.json', {'schema':SCHEMA,'manifest_sha256':digest(self.path/'manifest.json')})
            except OSError as exc:
                self.error = 'completion_write_failed: '+str(exc)
                self.manifest.update(complete=False, training_complete=False, replay_complete=False, error=self.error)


def read_lines(path, limit=MAX_TRAJECTORY):
    if path.is_symlink() or path.stat().st_size > limit:
        raise ValueError('invalid_recording_file')
    with path.open('rb') as stream:
        for line in stream:
            if len(line) > 32768:
                raise ValueError('recording_line_too_large')
            yield json.loads(line)


def load_episode(root, ident, binding=None):
    path = episode_path(root, ident)
    for name in ('manifest.json','complete.json'):
        p = path/name
        if p.is_symlink() or p.stat().st_size > 65536:
            raise ValueError('invalid_episode_manifest')
    manifest = json.loads((path/'manifest.json').read_bytes())
    receipt = json.loads((path/'complete.json').read_bytes())
    if manifest.get('schema') != SCHEMA or manifest.get('id') != ident or manifest.get('complete') is not True \
            or receipt.get('manifest_sha256') != digest(path/'manifest.json'):
        raise ValueError('episode_incomplete_or_changed')
    if binding is not None and manifest['binding'] != binding:
        raise ValueError('episode_device_binding_mismatch')
    expected = {'trajectory.jsonl'} | {c+s for c in CAMERAS for s in ('.mjpeg','.jsonl')}
    if set(manifest['files']) != expected or manifest['joint_names'] != JOINT_NAMES or manifest['units'] != UNITS:
        raise ValueError('episode_layout_mismatch')
    for name, meta in manifest['files'].items():
        p = path/name
        if p.is_symlink() or not p.is_file() or p.stat().st_size != meta['bytes'] or digest(p) != meta['sha256']:
            raise ValueError('episode_checksum_mismatch: '+name)
    states = list(read_lines(path/'trajectory.jsonl'))
    previous = -1
    seq = -1
    for state in states:
        validate_state(state)
        if state['received_ns'] <= previous or state['sequence'] <= seq:
            raise ValueError('episode_state_order')
        previous, seq = state['received_ns'], state['sequence']
    if len(states) < 2:
        raise ValueError('episode_empty')
    return path, manifest, states
