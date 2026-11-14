"""Source-stamped planar chassis odometry; no ROS or motion-control dependency.

The diagnostic timestamp belongs to the chassis source event, not the TCP read.
Clock uncertainty bounds the software clock mapping, not sensor hardware delay.
"""
from __future__ import annotations

import copy
import http.client
import json
import math
import socket
import threading
import time
import uuid
from urllib.parse import urlsplit

TOPIC = 'base/deadreckon_with_gyro_yaw'
CLOCK_ID = 'host_realtime'
UNCERTAINTY_NS = 50_000_000
MAX_AGE_NS = 250_000_000


class ClockObservationRejected(ValueError):
    """A high-latency probe supplies no evidence and cannot refresh the mapping."""


class ClockMapping:
    """Freeze one bounded offset per segment; invalidate instead of slewing poses."""
    def __init__(self):
        self.offset = None
        self._bounds = None
        self._count = 0
        self._device = None
        self._observed = None
        self._wall_minus_mono = None

    def observe(self, device_ms, before_ns, after_ns, monotonic_ns):
        if (type(device_ms) is not int or device_ms <= 0 or
                after_ns < before_ns):
            raise ValueError('clock observation has invalid time')
        if self._device is not None and device_ms <= self._device:
            raise ValueError('chassis clock reset or did not advance')
        wall_minus_mono = before_ns - monotonic_ns
        if (self._wall_minus_mono is not None and
                abs(wall_minus_mono - self._wall_minus_mono) > 2_000_000):
            raise ValueError('host clock discontinuity')
        if after_ns - before_ns > 8_000_000:
            raise ClockObservationRejected('clock observation has excessive round trip')
        # The server clock is sampled during the request; allow 1 ms quantization.
        low = before_ns - device_ms * 1_000_000 - 1_000_000
        high = after_ns - device_ms * 1_000_000 + 1_000_000
        if self.offset is None:
            if self._bounds is not None:
                low, high = max(low, self._bounds[0]), min(high, self._bounds[1])
            if low > high:
                raise ValueError('clock observations disagree')
            self._bounds = low, high
            self._count += 1
            if self._count >= 5:
                self.offset = (low + high) // 2
        elif low > self.offset + UNCERTAINTY_NS // 2 or high < self.offset - UNCERTAINTY_NS // 2:
            raise ValueError('clock mapping drift exceeds segment budget')
        elif max(abs(self.offset - low), abs(self.offset - high)) > UNCERTAINTY_NS // 2:
            # A broad interval overlapping the budget proves neither validity
            # nor drift. Discard it without refreshing the last verified time.
            raise ClockObservationRejected('clock observation cannot verify segment budget')
        self._device, self._observed = device_ms, monotonic_ns
        self._wall_minus_mono = wall_minus_mono

    def map(self, source_ms, now_ns, monotonic_ns):
        if self.offset is None or self._observed is None:
            raise ValueError('clock is not synchronized')
        if monotonic_ns - self._observed > 2_000_000_000 or monotonic_ns < self._observed:
            raise ValueError('clock observation expired')
        if abs(now_ns - monotonic_ns - self._wall_minus_mono) > 2_000_000:
            raise ValueError('host clock discontinuity')
        stamp = source_ms * 1_000_000 + self.offset
        if stamp > now_ns + UNCERTAINTY_NS or now_ns - stamp > MAX_AGE_NS:
            raise ValueError('source measurement is future or stale')
        return stamp, {'method': 'chassis_clock_request_interval', 'version': '1',
                       'offset_ns': self.offset, 'uncertainty_ns': UNCERTAINTY_NS,
                       'verified': True}


class IncrementalOdometry:
    """Compose body-frame planar displacement increments in acquisition order.

    The first timestamp group establishes the origin; its pre-subscription
    displacement is discarded. Equal-stamped increments retain delivery order.
    An invalid batch retires the entire segment.
    """
    def __init__(self, clock):
        self.clock = clock
        self.segment = uuid.uuid4().hex
        self._last = None
        self._pending = []
        self._pose = (0., 0., 0.)
        self._state = 'waiting'
        self._reason = 'waiting for source increments'

    def status(self):
        return {'status': self._state, 'reason': self._reason,
                'segment_id': self.segment, 'source_topic': TOPIC}

    def lost(self, reason):
        self._state, self._reason = 'lost', str(reason)
        self._pending = []
        return {'schema': 'motus.odom/1', 'stamp_ms': None, 'frame': 'body',
                'units': {'linear': 'm/s', 'angular': 'rad/s', 'length': 'm'},
                'twist': [None] * 6, 'pose': None,
                'vendor': {'motus_slam': {**self.status(), 'clock_id': CLOCK_ID,
                                         'stamp_source': 'unknown'}}}

    def feed(self, messages, now_ns, monotonic_ns):
        try:
            if self._state == 'lost':
                raise ValueError('lost segment requires a new subscription')
            if not isinstance(messages, list):
                raise ValueError('diagnostic batch must be a list')
            if self._pending:
                self.clock.map(self._pending[0][0], now_ns, monotonic_ns)
            parsed = list(self._pending)
            previous = parsed[-1][0] if parsed else self._last
            for message in messages:
                stamp = message['timestamp']
                payload = message['payload']
                dx, dy = (payload['positionDifference'][axis] for axis in ('x', 'y'))
                dyaw = payload['angularDifference']
                if type(stamp) is not int or stamp <= 0:
                    raise ValueError('invalid source timestamp')
                if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                       not math.isfinite(v) for v in (dx, dy, dyaw)):
                    raise ValueError('invalid displacement increment')
                if previous is not None and not 0 <= stamp - previous <= 100:
                    raise ValueError('source increment reorder or gap')
                mapped, mapping = self.clock.map(stamp, now_ns, monotonic_ns)
                parsed.append((stamp, mapped, mapping, dx, dy, dyaw))
                previous = stamp
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.lost(str(exc))
            raise ValueError(str(exc)) from exc
        groups = []
        for event in parsed:
            if not groups or groups[-1][0][0] != event[0]:
                groups.append([])
            groups[-1].append(event)
        # Hold the newest timestamp until a later source event closes its group.
        # This also handles equal timestamps split across TCP reads. exact_once
        # subscription order is preserved; no increment is deduplicated.
        self._pending = groups.pop() if groups else []
        samples = []
        for group in groups:
            stamp, mapped, mapping = group[0][:3]
            x, y, yaw = self._pose
            twist = [None] * 6
            if self._last is not None:
                dt = (stamp - self._last) / 1000.
                origin_x, origin_y, origin_yaw = x, y, yaw
                total_yaw = 0.
                for _, _, _, dx, dy, dyaw in group:
                    x += math.cos(yaw) * dx - math.sin(yaw) * dy
                    y += math.sin(yaw) * dx + math.cos(yaw) * dy
                    yaw = math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw))
                    total_yaw += dyaw
                dx, dy = x - origin_x, y - origin_y
                vx = (math.cos(origin_yaw)*dx + math.sin(origin_yaw)*dy) / dt
                vy = (-math.sin(origin_yaw)*dx + math.cos(origin_yaw)*dy) / dt
                twist = [vx, vy, None, None, None, total_yaw / dt]
            self._last, self._pose = stamp, (x, y, yaw)
            self._state, self._reason = 'tracking', ''
            frame = self.segment + '_odom'
            measurement = {'schema': 'motus.slam_frame/1', 'stamp_ns': mapped,
                           'frame_id': frame, 'clock_id': CLOCK_ID,
                           'clock_epoch': self.segment, 'source_generation': self.segment,
                           'source_clock': 'slamware_uptime_ms', 'source_stamp': stamp,
                           'source_unit': 'ms', 'mapping': mapping}
            samples.append({'schema': 'motus.odom/1', 'stamp_ms': mapped // 1_000_000,
                            'frame': 'body',
                            'units': {'linear': 'm/s', 'angular': 'rad/s', 'length': 'm'},
                            'twist': twist,
                            'pose': {'frame_id': frame, 'child_frame_id': 'base_link',
                                     'position': [x, y, 0.],
                                     'quaternion': [0., 0., math.sin(yaw/2), math.cos(yaw/2)],
                                     'covariance': None},
                            'vendor': {'stamp_source': 'robot', 'source_stamp_ms': stamp,
                                       'motus_slam': {**self.status(), 'stamp_source': 'measurement',
                                                      'clock_id': CLOCK_ID, 'planar': True,
                                                      'pose_reference': 'chassis_odometry',
                                                      'synchronization': 'bounded_software',
                                                      'timestamp_semantics': 'diagnostic_event_group',
                                                      'source_events_in_group': len(group),
                                                      'measurement': measurement}}})
        return samples


# Only these read/subscribe commands are used; this module has no motion API.
class DiagnosticClient:
    def __init__(self, base_url):
        url = urlsplit(base_url)
        self.host, self.http_port = url.hostname, url.port or 80
        self.prefix = url.path.rstrip('/')
        self._socket = self._reader = self._http = None
        self._sequence = 0
        self._subscription = None

    def __enter__(self):
        self._socket = socket.create_connection((self.host, 1445), timeout=.5)
        self._socket.settimeout(.5)
        self._reader = self._socket.makefile('rb')
        self._http = http.client.HTTPConnection(self.host, self.http_port, timeout=.5)
        return self

    def _rpc(self, command, args):
        self._sequence += 1
        request = {'command': 'diagnosis', 'request_id': self._sequence,
                   'args': {'command': command, 'args': args}}
        self._socket.sendall((json.dumps(request) + '\n\r\n\r\n').encode())
        # Match the vendor SDK's framing, with a bounded batch/line size.
        for _ in range(8):
            line = self._reader.readline(2_000_000)
            if not line or len(line) >= 2_000_000:
                raise ConnectionError('diagnostic response closed or oversized')
            if line.strip():
                break
        else:
            raise ValueError('empty diagnostic response')
        response = json.loads(line)
        if not isinstance(response, dict) or response.get('request_id') != self._sequence:
            raise ValueError('diagnostic response ID mismatch')
        result = response.get('result', {})
        if not isinstance(result, dict):
            raise ValueError('invalid diagnostic result')
        if result.get('code') != 1 or result.get('result') != 'success':
            raise ValueError('diagnostic request failed: ' + str(result.get('error', result)))
        return result.get('response')

    def subscribe(self):
        response = self._rpc('topic/subscribe', {'topic': TOPIC, 'options': {
            'qos': 'exact_once', 'timeoutInSeconds': 3}})
        if not isinstance(response, dict):
            raise ValueError('invalid diagnostic subscription response')
        subscription = response.get('subscriptionId')
        if type(subscription) is not int or subscription < 0:
            raise ValueError('invalid diagnostic subscription')
        self._subscription = subscription

    def read(self):
        return self._rpc('subscription/read', {'subscriptionId': self._subscription,
                          'readMultipleMessages': True, 'timeoutInMs': 0})

    def observe(self, clock):
        before, mono = time.time_ns(), time.monotonic_ns()
        self._http.request('GET', self.prefix + '/api/platform/v1/timestamp')
        response = self._http.getresponse()
        payload = response.read(4096)
        after = time.time_ns()
        if response.status != 200:
            raise ValueError('chassis clock request failed')
        value = json.loads(payload)
        if isinstance(value, str) and value.isdigit():
            value = int(value)
        clock.observe(value, before, after, mono)

    def __exit__(self, *unused):
        try:
            if self._subscription is not None:
                self._rpc('subscription/unsubscribe', {'subscriptionId': self._subscription})
        except (OSError, ValueError, TypeError):
            pass  # The bounded server subscription expires after disconnection.
        finally:
            for resource in (self._reader, self._socket, self._http):
                if resource is not None:
                    resource.close()


class OdometrySource:
    """Independent observer: telemetry/control must not wait for SLAM timing."""
    def __init__(self, base_url, hz, publish):
        self._url, self._period, self._publish = base_url, 1. / hz, publish
        self._stop = threading.Event()
        self._thread = None
        self._lifecycle = threading.Lock()
        self._lock = threading.Lock()
        self._status = {'status': 'waiting', 'reason': 'source not started'}

    @property
    def running(self):
        return bool(self._thread and self._thread.is_alive() and not self._stop.is_set())

    def info(self):
        with self._lock:
            return copy.deepcopy(self._status)

    def start(self):
        with self._lifecycle:
            if self._thread and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name='tianyi-source-odom', daemon=True)
            self._thread.start()

    def stop(self):
        with self._lifecycle:
            self._stop.set()
            if self._thread:
                self._thread.join(timeout=2)

    def _emit(self, sample):
        if not self._stop.is_set():
            self._publish(sample)
            with self._lock:
                self._status = copy.deepcopy(sample['vendor']['motus_slam'])

    def _run(self):
        while not self._stop.is_set():
            clock = ClockMapping()
            integrator = IncrementalOdometry(clock)
            try:
                with DiagnosticClient(self._url) as client:
                    calibration_deadline = time.monotonic() + 2
                    while clock.offset is None:
                        if self._stop.is_set():
                            return
                        if time.monotonic() >= calibration_deadline:
                            raise ValueError('clock calibration timed out')
                        try:
                            client.observe(clock)
                        except ClockObservationRejected:
                            pass
                        if time.monotonic() >= calibration_deadline:
                            raise ValueError('clock calibration timed out')
                        if self._stop.wait(.02):
                            return
                    client.subscribe()
                    observed = received = next_publish = time.monotonic()
                    while not self._stop.is_set():
                        started = time.monotonic()
                        if started - observed >= .5:
                            try:
                                client.observe(clock)
                            except ClockObservationRejected:
                                # Retry at the next drain instead of repeatedly
                                # colliding with a periodic sensor callback.
                                # ClockMapping freshness is still unchanged.
                                pass
                            else:
                                observed = time.monotonic()
                        samples = integrator.feed(client.read(), time.time_ns(), time.monotonic_ns())
                        if samples:
                            # Integrate ALL deltas, publish the newest source time at card Hz.
                            received = time.monotonic()
                            if received >= next_publish:
                                self._emit(samples[-1])
                                next_publish += self._period
                                if next_publish <= received:
                                    next_publish = received + self._period
                        elif time.monotonic() - received > MAX_AGE_NS / 1e9:
                            raise ValueError('source increments stopped')
                        self._stop.wait(max(0., min(.05, self._period) - (time.monotonic() - started)))
            except (OSError, ValueError, TypeError, KeyError, http.client.HTTPException) as exc:
                self._emit(integrator.lost(str(exc)))
                self._stop.wait(.5)
