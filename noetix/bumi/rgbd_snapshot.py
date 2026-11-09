"""On-demand RGB-D snapshots over an inherited local socket, never a camera opener.

No device-to-host clock mapping is introduced. Level B proves advancement from
a producer-side device-frame baseline, not exposure after the request.
"""
from __future__ import annotations

import base64
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import select
import shutil
import socket
import struct
import threading
import time
import uuid

try:
    from .visual_inputs import camera_metadata
except ImportError:
    from visual_inputs import camera_metadata

MAX_PACKET = 8 * 1024 * 1024


class SnapshotError(RuntimeError):
    def __init__(self, code, message):
        self.code = code
        super().__init__(message)


def check(cancel, deadline):
    if cancel.is_set():
        raise SnapshotError('CANCELLED', 'Snapshot cancelled')
    if time.monotonic() >= deadline:
        raise SnapshotError('TIMEOUT', 'Snapshot deadline exceeded')


def send_packet(sock, message):
    data = json.dumps(message, allow_nan=False).encode()
    if len(data) > MAX_PACKET:
        raise SnapshotError('INVALID_PACKET', 'Snapshot packet too large')
    sock.sendall(struct.pack('!I', len(data)) + data)


class Receiver:
    def __init__(self, sock):
        self.sock, self.buffer = sock, bytearray()

    def poll(self, timeout=0):
        if len(self.buffer) >= 4:
            size = struct.unpack('!I', self.buffer[:4])[0]
            if not 0 < size <= MAX_PACKET:
                raise ValueError('Invalid snapshot packet size')
            if len(self.buffer) >= size + 4:
                data = bytes(self.buffer[4:4+size])
                del self.buffer[:4+size]
                message = json.loads(data)
                if not isinstance(message, dict):
                    raise ValueError('Snapshot packet must be a JSON object')
                return message
        if select.select([self.sock], [], [], timeout)[0]:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise SnapshotError('CAMERA_DISCONNECTED', 'Camera request channel closed')
            self.buffer.extend(chunk)
        return None


def validate_pair(meta, baseline):
    """Only original source stream identities, never ROS frame_id or cache counters."""
    if not meta.get('epoch') or meta['epoch'] != baseline.get('epoch'):
        raise SnapshotError('CAMERA_RESTART', 'Camera session changed')
    if not meta.get('source') or meta['source'] != baseline.get('source'):
        raise SnapshotError('SOURCE_CHANGED', 'Camera source changed')
    scale = meta.get('depth_scale')
    if type(scale) not in (int, float) or not math.isfinite(scale) or not 0 < scale < .1:
        raise SnapshotError('INVALID_DEPTH_SCALE', 'Metric depth scale unavailable')
    for key in ('rgb', 'depth'):
        item, prior = meta[key], baseline[key]
        if (type(item['frame_number']) is not int or item['frame_number'] < 0
                or item['frame_number'] <= prior['frame_number']):
            raise SnapshotError('REPEATED_FRAME', 'Device frame did not advance')
        if (type(item['source_ms']) not in (int, float) or not math.isfinite(item['source_ms'])
                or item['source_ms'] <= prior['source_ms']):
            raise SnapshotError('CLOCK_RESET', 'Device timestamp did not advance')
        if item['domain'] != prior['domain']:
            raise SnapshotError('CLOCK_DOMAIN_CHANGED', 'Device clock domain changed')
    if meta['rgb']['domain'] != meta['depth']['domain']:
        raise SnapshotError('PAIR_DOMAIN_MISMATCH', 'RGB/depth domains differ')
    if meta['rgb']['domain'] not in ('hardware_clock', 'system_time', 'global_time'):
        raise SnapshotError('UNSUPPORTED_CLOCK', 'Unknown device clock domain')
    skew = abs(meta['rgb']['source_ms'] - meta['depth']['source_ms'])
    if skew > 20:
        raise SnapshotError('PAIR_SKEW', 'RGB/depth timestamp skew exceeds 20 ms')
    return skew


class SnapshotProducer:
    """Camera-loop hook. Metadata is observed even when no consumer is requesting.

    The sole pipeline owner calls poll() before wait_for_frames and observe()
    after alignment. One bounded sender thread handles serialization/socket I/O.
    There is no file I/O and no interaction with RawCameraPublisher.enabled.
    """
    def __init__(self, sock, source, depth_scale):
        if not source or not math.isfinite(depth_scale) or not 0 < depth_scale < .1:
            raise ValueError('Actual device identity and depth scale required')
        self.sock, self.source, self.depth_scale = sock, source, depth_scale
        self.sock.settimeout(.5)
        self.receiver = Receiver(sock)
        self.epoch, self.latest, self.pending = uuid.uuid4().hex, None, None
        self.clock_fault = None
        self.latest_wait_end_ns = None
        self.observed_pairs = 0
        self.outbox = queue.Queue(maxsize=1)
        self.closed = threading.Event()
        self.sender = threading.Thread(target=self._send, daemon=True, name='rgbd-snapshot-send')
        self.sender.start()

    def _send(self):
        while not self.closed.is_set():
            try:
                message = self.outbox.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                for key in ('rgb_bgr', 'depth_z16'):
                    if key in message:
                        message[key] = base64.b64encode(message[key]).decode('ascii')
                send_packet(self.sock, message)
            except Exception:
                self.close()

    def _reply(self, message):
        try:
            self.outbox.put_nowait(message)
        except queue.Full:
            self.close()  # A slow client cannot grow memory or block media.

    def poll(self):
        try:
            if self.pending and time.monotonic_ns() >= self.pending['deadline_monotonic_ns']:
                self.pending = None
            message = self.receiver.poll()
            if message and message.get('action') == 'cancel' and self.pending:
                if message.get('request_id') == self.pending['request_id']:
                    self.pending = None
            if message and message.get('action') == 'capture':
                if self.clock_fault:
                    self._reply({'request_id': message.get('request_id'),
                                 'error': self.clock_fault[0], 'message': self.clock_fault[1]})
                elif self.pending:
                    self._reply({'request_id': message.get('request_id'), 'error': 'BUSY'})
                else:
                    self.pending = {**message, 'baseline': deepcopy(self.latest),
                                    'producer_admitted_monotonic_ns': time.monotonic_ns()}
            if self.pending and time.monotonic_ns() >= self.pending['deadline_monotonic_ns']:
                self.pending = None  # Lease expires without touching VO demand.
        except Exception:
            self.close()

    def _log_clock_fault(self, meta, stream, host_delta_ms, wait_window_ns):
        # The fault is latched before logging, so there is at most one record
        # per producer epoch, even if the fault preceded the first request.
        wait_end_ns = wait_window_ns[1] if wait_window_ns is not None else None
        record = {
            'code': self.clock_fault[0], 'epoch': self.epoch, 'stream': stream,
            'pairs_before_fault': self.observed_pairs, 'request_pending': self.pending is not None,
            'host_monotonic_delta_ms': host_delta_ms,
            'host_wall_delta_ms': (meta['received_ns'] - self.latest['received_ns']) / 1e6,
            'wait_return_delta_ms': ((wait_end_ns - self.latest_wait_end_ns) / 1e6
                                     if wait_end_ns is not None and self.latest_wait_end_ns is not None
                                     else None),
            'wait_duration_ms': ((wait_end_ns - wait_window_ns[0]) / 1e6
                                 if wait_window_ns is not None else None),
            'post_wait_to_observe_ms': ((meta['received_monotonic_ns'] - wait_end_ns) / 1e6
                                       if wait_end_ns is not None else None),
        }
        for key in ('rgb', 'depth'):
            previous, current = self.latest[key], meta[key]
            delta_ms = current['source_ms'] - previous['source_ms']
            delta_ms = delta_ms if math.isfinite(delta_ms) else None
            record[key] = {
                'previous_domain': previous['domain'], 'current_domain': current['domain'],
                'previous_frame_number': previous['frame_number'],
                'current_frame_number': current['frame_number'],
                'device_delta_ms': delta_ms,
                'device_minus_host_ms': delta_ms - host_delta_ms if delta_ms is not None else None,
            }
        try:
            print('[rgbd_snapshot] clock_fault ' + json.dumps(record, allow_nan=False, separators=(',', ':')), flush=True)
        except (OSError, ValueError):
            pass  # Failure to log must not replace the clock error.

    def observe(self, source_frames, aligned_frames, *, wait_window_ns=None):
        if self.closed.is_set() or self.clock_fault:
            return
        try:
            rgb, depth = source_frames.get_color_frame(), source_frames.get_depth_frame()
            if not rgb or not depth:
                return
            meta = camera_metadata(rgb, depth, epoch=self.epoch,
                                   depth_scale=self.depth_scale, received_ns=time.time_ns())
            meta['source'] = self.source
            if self.latest:
                host_delta_ms = (meta['received_monotonic_ns'] - self.latest['received_monotonic_ns'])/1e6
                # A delayed old frame can otherwise look like progress after a reset.
                # Only restarting the pipeline establishes a new source session.
                for key in ('rgb', 'depth'):
                    current, previous = meta[key], self.latest[key]
                    delta_ms = current['source_ms'] - previous['source_ms']
                    if current['domain'] != previous['domain']:
                        self.clock_fault = ('CLOCK_DOMAIN_CHANGED', 'Device clock domain changed')
                    elif delta_ms < 0 or current['frame_number'] < previous['frame_number']:
                        self.clock_fault = ('CAMERA_RESTART', 'Frame identity or clock reset')
                    elif abs(delta_ms - host_delta_ms) > 1000:
                        self.clock_fault = ('CLOCK_DISCONTINUITY', 'Device/host elapsed times diverged by over 1 s')
                    if self.clock_fault:
                        self._log_clock_fault(meta, key, host_delta_ms, wait_window_ns)
                        raise SnapshotError(*self.clock_fault)
            self.latest = meta
            self.latest_wait_end_ns = wait_window_ns[1] if wait_window_ns is not None else None
            self.observed_pairs += 1
            request = self.pending
            if not request:
                return
            if time.monotonic_ns() >= request['deadline_monotonic_ns']:
                self.pending = None
                return
            if request['baseline'] is None:
                request['baseline'] = deepcopy(meta)
                return  # First observed pair is a baseline, never the snapshot.
            try:
                skew = validate_pair(meta, request['baseline'])
            except SnapshotError as exc:
                if exc.code == 'REPEATED_FRAME':
                    return
                raise
            color, aligned = aligned_frames.get_color_frame(), aligned_frames.get_depth_frame()
            if not color or not aligned:
                raise SnapshotError('INVALID_PAIR', 'Aligned pair missing')
            if (color.get_frame_number() != meta['rgb']['frame_number']
                    or aligned.get_frame_number() != meta['depth']['frame_number']):
                raise SnapshotError('INVALID_PAIR', 'Aligned pixels do not match source identities')
            import numpy as np
            bgr = np.asanyarray(color.get_data())
            z16 = np.asanyarray(aligned.get_data())
            intr = color.profile.as_video_stream_profile().intrinsics
            if (bgr.dtype != np.uint8 or z16.dtype != np.uint16
                    or bgr.shape != (intr.height, intr.width, 3)
                    or z16.shape != bgr.shape[:2] or bgr.size > 640*480*3):
                raise SnapshotError('INVALID_PIXELS', 'Invalid RGB-D shape or encoding')
            calibration = dict(width=intr.width, height=intr.height, fx=intr.fx, fy=intr.fy,
                               ppx=intr.ppx, ppy=intr.ppy, model=str(intr.model).rsplit('.', 1)[-1],
                               coeffs=list(intr.coeffs), source='active_realsense_color_profile',
                               depth_alignment='color_optical_plane')
            if (not all(math.isfinite(x) for x in (intr.fx, intr.fy, intr.ppx, intr.ppy, *intr.coeffs))
                    or intr.fx <= 0 or intr.fy <= 0):
                raise SnapshotError('INVALID_CALIBRATION', 'Invalid active intrinsics')
            self._reply(dict(request_id=request['request_id'], baseline=request['baseline'],
                             metadata=meta, calibration=calibration, pair_skew_ms=skew,
                             producer_admitted_monotonic_ns=request['producer_admitted_monotonic_ns'],
                             rgb_bgr=bgr.tobytes(), depth_z16=z16.astype('<u2', copy=False).tobytes()))
            self.pending = None
        except Exception as exc:
            if self.pending:
                self._reply({'request_id': self.pending['request_id'],
                             'error': getattr(exc, 'code', 'INVALID_PAIR'), 'message': str(exc)})
            self.pending = None

    def close(self):
        self.closed.set()
        try:
            self.sock.close()
        except OSError:
            pass


class SnapshotClient:
    def __init__(self, sock):
        self.sock, self.receiver = sock, Receiver(sock)
        self.sock.settimeout(.05)
        self.flight = threading.Lock()

    def capture(self, request_id, deadline, cancel):
        if not self.flight.acquire(blocking=False):
            raise SnapshotError('BUSY', 'Another acquisition is pending')
        outstanding = False
        try:
            boundary = time.monotonic_ns()
            check(cancel, deadline)
            send_packet(self.sock, {'action': 'capture', 'request_id': request_id,
                                   'deadline_monotonic_ns': int(deadline*1e9)})
            outstanding = True
            while True:
                check(cancel, deadline)
                message = self.receiver.poll(.02)
                if message is None or message.get('request_id') != request_id:
                    continue  # Never associate a late reply with a new request.
                outstanding = False
                if 'error' in message:
                    raise SnapshotError(message['error'], message.get('message', 'Producer rejected request'))
                skew = validate_pair(message['metadata'], message['baseline'])
                if message['producer_admitted_monotonic_ns'] < boundary:
                    raise SnapshotError('INVALID_BOUNDARY', 'Producer admission predates request')
                if message['metadata']['received_monotonic_ns'] < message['producer_admitted_monotonic_ns']:
                    raise SnapshotError('INVALID_BOUNDARY', 'Selected pair was observed before producer admission')
                message.update(request_admitted_monotonic_ns=boundary,
                               response_received_monotonic_ns=time.monotonic_ns(), pair_skew_ms=skew)
                for key in ('rgb_bgr', 'depth_z16'):
                    message[key] = base64.b64decode(message[key], validate=True)
                return message
        except (OSError, ValueError, KeyError) as exc:
            outstanding = False
            self.close()  # Partial stream writes/invalid framing cannot be safely reused.
            raise SnapshotError('CAMERA_CHANNEL_ERROR', str(exc)) from exc
        finally:
            try:
                if outstanding:
                    send_packet(self.sock, {'action': 'cancel', 'request_id': request_id})
            except OSError:
                self.close()  # A partial cancel packet also poisons the stream.
            finally:
                self.flight.release()

    def close(self):
        self.sock.close()


class SnapshotCapture:
    """Single-flight ACP action over the shared camera."""
    def __init__(self, camera, output_dir):
        self.camera, self.output_dir = camera, Path(output_dir)
        self.lock, self.active = threading.Lock(), None
        self.worker = None
        self.last_completion = None

    def cancel(self, *, wait=False):
        with self.lock:
            requested = self.active is not None and not self.active.is_set()
            if requested:
                self.active.set()
            worker = self.worker
        if wait and worker is not None:
            worker.join(timeout=6)
        return requested

    def info(self):
        with self.lock:
            return {'active': self.active is not None, 'guarantee_levels': ['A', 'B'],
                    'exposure_after_request_supported': False, 'single_flight': True,
                    'last_completion': deepcopy(self.last_completion)}

    def start(self, args, notify, response_sent=None):
        request_id = uuid.uuid4().hex
        if type(args.get('require_exposure_after_request', False)) is not bool:
            return {'ok': False, 'code': 'INVALID_ARGUMENT', 'request_id': request_id}
        if args.get('require_exposure_after_request', False):
            return {'ok': False, 'code': 'EXPOSURE_UNVERIFIABLE', 'request_id': request_id,
                    'guarantee_level': None, 'exposure_after_request_verified': False}
        timeout = args.get('timeout_s', 5.)
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or not .1 <= timeout <= 10:
            return {'ok': False, 'code': 'INVALID_TIMEOUT', 'request_id': request_id}
        cancel = threading.Event()
        with self.lock:
            if self.active is not None:
                return {'ok': False, 'code': 'BUSY', 'request_id': request_id}
            self.active = cancel
            deadline = time.monotonic() + timeout
            worker = threading.Thread(
                target=self._run, args=(request_id, deadline, cancel, notify, response_sent),
                name='bumi-rgbd-snapshot', daemon=True)
            try:
                worker.start()
            except RuntimeError as exc:
                self.active = None
                return {'ok': False, 'code': 'CAPTURE_RGBD_FAILED', 'request_id': request_id,
                        'message': str(exc)}
            self.worker = worker
        return {'ok': True, 'state': 'queued', 'action_id': request_id, 'request_id': request_id}

    def _settle(self, status, result):
        # Called with the lock held, including at the successful commit boundary.
        self.last_completion = {'action_id': result['request_id'], 'status': status, 'result': result}
        self.active = None

    def _run(self, request_id, deadline, cancel, notify, response_sent):
        try:
            result = self._capture(request_id, deadline, cancel)
        except Exception as exc:
            result = {'ok': False, 'request_id': request_id, 'code': 'CAPTURE_RGBD_FAILED',
                      'message': str(exc), 'exposure_after_request_verified': False}
        status = 'completed'
        if not result['ok']:
            with self.lock:
                cleanup_failed = bool(result.get('cleanup_errors'))
                if cancel.is_set() and not cleanup_failed:
                    result.update(code='CANCELLED', message='Snapshot cancelled')
                status = 'cancelled' if result['code'] == 'CANCELLED' and not cleanup_failed else 'error'
                self._settle(status, result)
        if response_sent is not None:
            # Core registers pending after parsing the MCP response. Its current
            # API has no registration acknowledgement or early-completion buffer.
            if not response_sent.wait(timeout=5):
                print(f'[Bumi ACP] MCP response write did not finish for {request_id}', flush=True)
        notify(request_id, status, result)

    def _capture(self, request_id, deadline, cancel):
        temporary = final = None
        owns_temporary = published = False
        try:
            if self.camera is None or not self.camera.is_running():
                raise SnapshotError('CAMERA_UNAVAILABLE', 'Shared camera is not running')
            packet = self.camera.capture_rgbd(request_id, deadline, cancel)
            check(cancel, deadline)
            if packet.get('request_id') != request_id:
                raise SnapshotError('REQUEST_MISMATCH', 'Response belongs to another request')
            import numpy as np
            cal = packet['calibration']
            height, width = cal['height'], cal['width']
            if (type(height) is not int or type(width) is not int
                    or height <= 0 or width <= 0 or height*width > 640*480):
                raise SnapshotError('INVALID_PIXELS', 'Invalid dimensions')
            rgb = np.frombuffer(packet.pop('rgb_bgr'), np.uint8).reshape(height, width, 3)
            depth = np.frombuffer(packet.pop('depth_z16'), '<u2').reshape(height, width)
            directory = self.output_dir / 'rgbd'
            directory.mkdir(parents=True, exist_ok=True)
            temporary, final = directory / ('.'+request_id+'.partial'), directory / request_id
            temporary.mkdir()
            owns_temporary = True
            # PPM preserves RGB without a codec dependency; NPY preserves raw Z16.
            (temporary/'rgb.ppm').write_bytes(f'P6\n{width} {height}\n255\n'.encode() + rgb[:, :, ::-1].tobytes())
            np.save(temporary/'depth.npy', depth, allow_pickle=False)
            (temporary/'calibration.json').write_text(json.dumps(cal, allow_nan=False), encoding='utf-8')
            packet.update(guarantee_level='B', received_after_request=True,
                          exposure_after_request_verified=False,
                          timestamp_semantics='SDK source timestamp; NOT exposure time',
                          verification=['producer_request_id', 'same_frameset', 'device_frame_advance',
                                        'same_epoch_source', 'domain_and_20ms_skew', 'aligned_pixel_identity'],
                          depth_encoding='uint16_z16', depth_unit_m=packet['metadata']['depth_scale'],
                          invalid_depth_value=0, saved_wall_time_ns=time.time_ns())
            packet['files'] = {name: {'bytes': (temporary/name).stat().st_size,
                                     'sha256': hashlib.sha256((temporary/name).read_bytes()).hexdigest()}
                               for name in ('rgb.ppm', 'depth.npy', 'calibration.json')}
            (temporary/'metadata.json').write_text(json.dumps(packet, allow_nan=False, indent=2), encoding='utf-8')
            check(cancel, deadline)
            temporary.rename(final)  # All-or-nothing directory visibility on one filesystem.
            published = True
            result = {'ok': True, 'request_id': request_id, 'guarantee_level': 'B',
                      'exposure_after_request_verified': False, 'directory': str(final),
                      'files': {n: str(final/n) for n in (*packet['files'], 'metadata.json')}}
            with self.lock:
                check(cancel, deadline)
                self._settle('completed', result)  # Commit wins atomically against stop.
            return result
        except Exception as exc:
            cleanup_errors = []
            for path, owned in ((temporary, owns_temporary), (final, published)):
                try:
                    if owned and path is not None and path.exists():
                        if path.resolve().parent != directory.resolve() or path.is_symlink():
                            raise RuntimeError('Refusing cleanup outside snapshot directory')
                        shutil.rmtree(path)  # Only this UUID request's newly created directories.
                except (OSError, RuntimeError) as cleanup_error:
                    cleanup_errors.append(str(cleanup_error))
            return {'ok': False, 'request_id': request_id, 'code': getattr(exc, 'code', 'CAPTURE_RGBD_FAILED'),
                    'message': str(exc), 'exposure_after_request_verified': False,
                    'cleanup_errors': cleanup_errors}
