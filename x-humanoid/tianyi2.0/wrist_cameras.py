"""Repeatable wrist RGB/depth cards, with one isolated capture owner per device."""
from __future__ import annotations

import json
import logging
import multiprocessing as mp
import os
from pathlib import Path
import queue
import re
import threading
import time
import zlib

from wrist_camera_devices import discover_devices, select_profiles

WIDTH, HEIGHT, FPS = 640, 480, 15
STARTUP_SECONDS, STALE_SECONDS = 12.0, 5.0
ROUTE_TIMEOUT_SECONDS = 2.0
CHANNELS = ('rgb', 'depth')
FORMATS = {'rgb': 'image/jpeg', 'depth': 'image/depth-zlib'}
DDS_PROFILE = '/opt/phanthy-motus/dds-local.xml'
log = logging.getLogger(__name__)


def camera_topic(namespace, instance_id, channel, side=None):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,127}', instance_id) or channel not in CHANNELS:
        raise ValueError('Invalid camera instance_id or channel')
    if side not in (None, 'left', 'right'):
        raise ValueError('Invalid wrist side')
    suffix = f'{side}_{channel}' if side else channel
    return f"/{namespace.strip('/')}/wrist_camera/{instance_id.replace('-', '_')}/{suffix}"


def encode_depth_mm(depth, scale_m):
    """Headerless little-endian uint16 millimetres, without resizing or wrapping."""
    import numpy as np
    if (depth.shape != (HEIGHT, WIDTH) or depth.dtype != np.uint16
            or not np.isfinite(scale_m) or scale_m <= 0):
        raise ValueError('Invalid Z16 layout or depth scale')
    mm = np.rint(depth.astype(np.float64) * float(scale_m) * 1000.)
    mm[(mm < 1) | (mm > 65535)] = 0
    return zlib.compress(mm.astype('<u2').tobytes(), 1)


def _frame_intrinsics(frame):
    intr = frame.profile.as_video_stream_profile().get_intrinsics()
    return {'width': int(intr.width), 'height': int(intr.height),
            'K': [float(intr.fx), 0., float(intr.ppx), 0., float(intr.fy), float(intr.ppy), 0., 0., 1.],
            'D': [float(v) for v in intr.coeffs], 'sdk_distortion': str(intr.model)}


def _report(status_queue, status):
    # Copy mutable maps: Queue serialization runs asynchronously.
    snapshot = {key: dict(value) if isinstance(value, dict) else value for key, value in status.items()}
    try:
        status_queue.put_nowait(snapshot)
    except queue.Full:
        try:
            status_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            status_queue.put_nowait(snapshot)
        except queue.Full:
            pass


def _capture_worker(namespace, device, profiles, quality, routes, control, stop_event, status_queue):
    """SDK, encoding and ROS live here; route ACK serializes stop with publish."""
    from common import logsafe
    logsafe.install()
    os.environ.update(RCUTILS_COLORIZED_OUTPUT='0', ROS_DOMAIN_ID='42',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', FASTRTPS_DEFAULT_PROFILES_FILE=DDS_PROFILE)
    route_lock = threading.Lock()
    routes = dict(routes)
    pipe = cap = node = None
    started = ros_started = False
    pubs = {}
    status = {'error': None, 'last_frame': {}, 'frames': {}, 'intrinsics': {},
              'sdk_timestamp_ms': {}, 'depth_scale_m': None, 'device_name': device['name']}

    def listen():
        # Starts before SDK initialization, so adding a second card during
        # first-frame startup does not kill the shared camera owner.
        try:
            while not stop_event.is_set():
                if control.poll(.1):
                    revision, replacement = control.recv()
                    with route_lock:
                        routes.clear()
                        routes.update(replacement)
                        control.send(revision)
        except (EOFError, OSError):
            stop_event.set()

    listener = threading.Thread(target=listen, daemon=True)
    listener.start()
    try:
        if not os.path.isfile(DDS_PROFILE):
            raise RuntimeError('Missing loopback-only DDS profile; refusing camera publication')
        import cv2
        import numpy as np
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        if device['backend'] == 'realsense':
            import pyrealsense2 as rs
            config = rs.config()
            config.enable_device(device['serial'])
            for channel, p in profiles.items():
                config.enable_stream(rs.stream.color if channel == 'rgb' else rs.stream.depth,
                                     p['width'], p['height'], getattr(rs.format, p['format']), p['fps'])
            pipe = rs.pipeline()
            active = pipe.start(config)
            started = True
            if 'depth' in profiles:
                scale = float(active.get_device().first_depth_sensor().get_depth_scale())
                if not np.isfinite(scale) or scale <= 0:
                    raise RuntimeError('Camera returned invalid depth scale')
                status['depth_scale_m'] = scale
        else:
            p = profiles['rgb']
            cap = cv2.VideoCapture(device['path'], cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError('USB camera could not be opened')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*p['format']))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, p['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, p['height'])
            cap.set(cv2.CAP_PROP_FPS, p['fps'])
            actual = (round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            if actual != (p['width'], p['height']):
                raise RuntimeError(f'USB camera did not accept native resolution: {actual}')
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            if actual_fps > 0 and abs(actual_fps - p['fps']) > .5:
                raise RuntimeError(f'USB camera did not accept frame rate: {actual_fps}')

        rclpy.init(domain_id=42)
        ros_started = True
        node = rclpy.create_node(f'tianyi_wrist_capture_{os.getpid()}')
        last_report, last_input = 0., time.monotonic()
        while not stop_event.is_set():
            if pipe is not None:
                try:
                    frames = pipe.wait_for_frames(timeout_ms=1000)
                except RuntimeError:
                    if time.monotonic() - last_input >= STARTUP_SECONDS:
                        raise RuntimeError('No camera frames received within timeout')
                    continue
                last_input = time.monotonic()
                inputs = {'rgb': frames.get_color_frame()}
                if 'depth' in profiles:
                    inputs['depth'] = frames.get_depth_frame()
            else:
                ok, raw = cap.read()
                if not ok:
                    raise RuntimeError('USB camera read failed or device disconnected')
                inputs = {'rgb': raw}
            for channel, frame in inputs.items():
                if stop_event.is_set() or frame is None or (pipe is not None and not frame):
                    continue
                if pipe is not None:
                    if channel not in status['intrinsics']:
                        status['intrinsics'][channel] = _frame_intrinsics(frame)
                    raw = np.asanyarray(frame.get_data())
                    status['sdk_timestamp_ms'][channel] = float(frame.get_timestamp())
                else:
                    raw = frame
                p = profiles[channel]
                shape = (p['height'], p['width'], 3) if channel == 'rgb' else (p['height'], p['width'])
                if raw.shape != shape or raw.dtype != (np.uint8 if channel == 'rgb' else np.uint16):
                    raise RuntimeError(f'Unexpected {channel} frame layout')
                if channel == 'rgb':
                    bgr = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR) if pipe is not None and p['format'] == 'rgb8' else raw
                    ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
                    if not ok:
                        raise RuntimeError('Camera JPEG encoding failed')
                    payload, fmt = encoded.tobytes(), 'jpeg'
                else:
                    payload, fmt = encode_depth_mm(raw, scale), '16UC1; compressedDepth zlib'
                with route_lock:
                    if stop_event.is_set():
                        break
                    for instance_id, side in routes.items():
                        key = (instance_id, side, channel)
                        if key not in pubs:
                            pubs[key] = node.create_publisher(CompressedImage,
                                camera_topic(namespace, instance_id, channel, side), qos_profile_sensor_data)
                        msg = CompressedImage()
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.header.frame_id = f"{namespace}_wrist_{instance_id.replace('-', '_')}_{side}_{channel}_optical_frame"
                        msg.format, msg.data = fmt, payload
                        pubs[key].publish(msg)
                    for key in list(pubs):
                        if routes.get(key[0]) != key[1]:
                            node.destroy_publisher(pubs.pop(key))
                status['last_frame'][channel] = time.monotonic()
                status['frames'][channel] = status['frames'].get(channel, 0) + 1
            now = time.monotonic()
            if now - last_report >= .5:
                _report(status_queue, status)
                last_report = now
    except Exception as exc:
        status['error'] = str(exc)
        _report(status_queue, status)
    finally:
        stop_event.set()
        listener.join(timeout=.2)
        if started:
            try:
                pipe.stop()
            except Exception:
                pass
        if cap is not None:
            cap.release()
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if ros_started:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        control.close()


class WristCameraSession:
    """One physical device; every instance has its own topic pair and lifetime."""

    def __init__(self, namespace, device, profiles, jpeg_quality=85):
        self.namespace, self.device, self.profiles = namespace, device, profiles
        self.jpeg_quality = jpeg_quality
        self.routes, self._requested_at, self._status = {}, {}, {}
        self._ctx = mp.get_context('spawn')
        self._lock = threading.RLock()
        self._proc = self._control = self._queue = self._quit = None
        self._revision = 0

    def _drain(self):
        if self._queue is not None:
            while True:
                try:
                    self._status = self._queue.get_nowait()
                except queue.Empty:
                    break

    def healthy(self):
        self._drain()
        if self._proc is None or not self._proc.is_alive() or self._status.get('error'):
            return False
        now = time.monotonic()
        if now - min(self._requested_at.values(), default=now) < STARTUP_SECONDS:
            return True
        # Recover native calls that hang without exiting. A fresh sibling
        # still owns useful capture: its missing channel is reported by info.
        return any(now - last < STALE_SECONDS
                   for last in self._status.get('last_frame', {}).values())

    def _close(self):
        if self._proc is not None:
            self._quit.set()
            self._proc.join(timeout=.5)
            if self._proc.is_alive():
                self._proc.terminate()
                self._proc.join(timeout=.5)
            if self._proc.is_alive():
                self._proc.kill()
                self._proc.join(timeout=.5)
            self._proc.close()
        if self._control is not None:
            self._control.close()
        if self._queue is not None:
            self._queue.cancel_join_thread()
            self._queue.close()
        self._proc = self._control = self._queue = self._quit = None

    def _sync(self):
        self._revision += 1
        try:
            self._control.send((self._revision, dict(self.routes)))
            if not self._control.poll(ROUTE_TIMEOUT_SECONDS) or self._control.recv() != self._revision:
                raise RuntimeError('Camera route update timed out')
        except (OSError, EOFError, RuntimeError) as exc:
            self._close()
            self._status['error'] = str(exc)

    def add(self, instance_id, side):
        with self._lock:
            unchanged = self.routes.get(instance_id) == side
            self.routes[instance_id] = side
            if self.healthy():
                if not unchanged:
                    self._requested_at.setdefault(instance_id, time.monotonic())
                    self._sync()
                return
            self._close()
            self._status = {}
            self._requested_at = {key: time.monotonic() for key in self.routes}
            self._control, child_control = self._ctx.Pipe()
            self._queue, self._quit = self._ctx.Queue(maxsize=8), self._ctx.Event()
            self._proc = self._ctx.Process(target=_capture_worker,
                args=(self.namespace, self.device, self.profiles, self.jpeg_quality,
                      dict(self.routes), child_control, self._quit, self._queue), daemon=True)
            try:
                self._proc.start()
            finally:
                child_control.close()

    def remove(self, instance_id):
        with self._lock:
            self.routes.pop(instance_id, None)
            self._requested_at.pop(instance_id, None)
            if not self.routes:
                self._close()
            elif self.healthy():
                self._sync()

    def close(self):
        with self._lock:
            self._close()

    def info(self, instance_id, side):
        from common.camera_info import build
        with self._lock:
            self._drain()
            now = time.monotonic()
            requested = self._requested_at.get(instance_id, now)
            streams, calibration = {}, []
            for channel in CHANNELS:
                p = self.profiles.get(channel)
                if p is None:
                    streams[channel] = {'state': 'unavailable', 'error': None,
                                        'message': 'This device has no native depth stream'}
                    continue
                last = self._status.get('last_frame', {}).get(channel)
                fresh = last is not None and last >= requested and now - last < STALE_SECONDS
                error = self._status.get('error')
                if not error and (self._proc is None or not self._proc.is_alive()):
                    error = 'Camera capture worker exited'
                if not error and now - requested >= STARTUP_SECONDS and not fresh:
                    error = f'Camera not supplying fresh {channel} frames'
                state = 'error' if error else 'running' if fresh else 'loading'
                streams[channel] = {'state': state, 'error': error, 'fresh': fresh,
                    'width': p['width'], 'height': p['height'], 'fps': p['fps'],
                    'frames_published': self._status.get('frames', {}).get(channel, 0),
                    'last_frame_age_s': None if last is None else max(0., now - last),
                    'sdk_timestamp_ms': self._status.get('sdk_timestamp_ms', {}).get(channel)}
                intr = self._status.get('intrinsics', {}).get(channel)
                calibration.append(build(topic=camera_topic(self.namespace, instance_id, channel, side),
                    id=f"{self.device['backend']}/{self.device['name']}/{channel}", format=FORMATS[channel],
                    width=p['width'], height=p['height'], K=intr['K'] if intr else None,
                    D=intr['D'] if intr else None, distortion_model='unknown',
                    source='vendor-spec' if intr else 'unknown', pipeline=[self.device['backend']],
                    vendor={'sdk_distortion': intr['sdk_distortion']} if intr else {}))
            errors = [v['error'] for v in streams.values() if v.get('error')]
            state = 'error' if errors else 'loading' if any(v['state'] == 'loading' for v in streams.values()) else 'running'
            return {'state': state, 'error': '; '.join(errors) or None, 'side': side, 'streams': streams,
                    'message': 'Waiting for camera frames' if state == 'loading' else '',
                    'stamp_source': 'host_ros_clock', 'depth_aligned_to_rgb': False,
                    'depth_scale_m': self._status.get('depth_scale_m'), 'camera_info': calibration}


class WristCameraPlugin:
    PREFIX = 'wrist_camera'

    def __init__(self, plugin_cfg, namespace):
        self._namespace = namespace
        self._quality = max(1, min(100, int(plugin_cfg.get('jpeg_quality', 85))))
        self._state_file = str(plugin_cfg.get('state_file', '/opt/phanthy-motus/data/wrist-camera.json'))
        self._configs, self._sessions, self._bindings, self._errors = {}, {}, {}, {}
        self._devices, self._discovery_errors = [], []
        self._lock, self._quit, self._thread = threading.RLock(), threading.Event(), None
        try:
            saved = json.loads(Path(self._state_file).read_text())
            for key, value in saved.items():
                self._validate(key, value)
                self._configs[key] = {**value, 'enabled': bool(value.get('enabled', False))}
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            log.warning('Cannot restore wrist camera instances: %s', exc)

    def _validate(self, instance_id, values):
        camera_topic(self._namespace, instance_id, 'rgb')
        for key in self._configs:
            if key != instance_id and key.replace('-', '_') == instance_id.replace('-', '_'):
                raise ValueError('Camera instance topic conflicts with another instance')
        if values.get('side') not in ('left', 'right'):
            raise ValueError('Select left or right wrist')
        if values.get('fps', 'auto') not in ('auto', '15', '30', 15, 30):
            raise ValueError('fps must be auto, 15 or 30')
        if not re.fullmatch(r'auto|[1-9]\d*x[1-9]\d*', str(values.get('resolution', 'auto'))):
            raise ValueError('resolution must be auto or a native WIDTHxHEIGHT')
        if not isinstance(values.get('device_id', 'auto'), str) or not values.get('device_id', 'auto'):
            raise ValueError('Select a camera device')
        if set(values) - {'side', 'device_id', 'fps', 'resolution', 'enabled', 'selection'}:
            raise ValueError('Unknown wrist camera configuration field')

    def _save(self):
        path = Path(self._state_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        try:
            temporary.write_text(json.dumps(self._configs, indent=2))
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)

    def get_tools(self):
        with self._lock:
            options = [{'const': 'auto', 'title': 'Auto (only when one camera is available)'}]
            options += [{'const': d['id'], 'title': f"{d['name']} · {d['id']} · " +
                         ('RGB + Depth' if d['capabilities']['depth'] else 'RGB only')}
                        for d in self._devices]
            present = {v['const'] for v in options}
            for cfg in self._configs.values():
                if cfg['device_id'] not in present:
                    options.append({'const': cfg['device_id'], 'title': f"Disconnected: {cfg['device_id']}"})
                    present.add(cfg['device_id'])
            resolutions = {}
            for device in self._devices:
                sizes = {(p['width'], p['height']) for p in device['profiles'].get('rgb', [])}
                for width, height in sorted(sizes):
                    value = f'{width}x{height}'
                    try:
                        select_profiles(device, resolution=value)
                    except ValueError:
                        continue  # No compatible native depth profile/frame rate.
                    resolutions.setdefault(value, set()).add(device['name'])
            resolution_options = [{'const': 'auto', 'title': 'Auto (native RGB resolution)'}]
            resolution_options += [{'const': value, 'title': value + ' · ' + ' / '.join(sorted(names))}
                                   for value, names in resolutions.items()]
            for cfg in self._configs.values():
                value = cfg['resolution']
                if value != 'auto' and value not in resolutions:
                    resolution_options.append({'const': value, 'title': f'{value} (currently unavailable)'})
                    resolutions[value] = set()
            return [{'name': self.PREFIX, 'type': 'sensor', 'multiInstance': True,
                'description': 'Wrist camera: select a USB device and left/right wrist. RGB and native depth; RGB-only cameras have no depth.',
                'inputSchema': {'type': 'object', 'properties': {
                    'action': {'type': 'string', 'enum': ['start', 'stop', 'info']}}, 'required': ['action']},
                'configSchema': {'type': 'object', 'required': ['side'], 'properties': {
                    'side': {'type': 'string', 'title': 'Wrist side', 'enum': ['left', 'right'], 'scope': 'instance'},
                    'device_id': {'type': 'string', 'title': 'Camera device', 'oneOf': options, 'scope': 'instance', 'default': 'auto'},
                    'fps': {'type': 'string', 'title': 'Native frame rate', 'enum': ['auto', '15', '30'], 'default': 'auto', 'scope': 'instance'},
                    'resolution': {'type': 'string', 'title': 'RGB resolution', 'oneOf': resolution_options,
                                   'default': 'auto', 'scope': 'instance',
                                   'description': 'Native RGB sizes across detected cameras; selected device/FPS is validated on save. Depth stays 640x480.'}}},
                'topic_in': [], 'topic_out': [{'format': FORMATS[c], 'desc': c.upper()} for c in CHANNELS]}]

    def start(self):
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._quit.clear()
            self._thread = threading.Thread(target=self._monitor, name='wrist_camera_discovery', daemon=True)
            self._thread.start()

    def stop(self):
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout=10)
        with self._lock:
            for session in self._sessions.values():
                session.close()
            self._sessions.clear()
            self._bindings.clear()

    def _monitor(self):
        while not self._quit.is_set():
            try:
                self._refresh()
            except Exception:
                log.exception('Wrist camera discovery/reconnect failed')
            self._quit.wait(3)

    def _refresh(self):
        result = discover_devices()
        with self._lock:
            if self._quit.is_set():
                return
            self._discovery_errors = result['errors']
            # An SDK scan failure is not proof that a running device vanished.
            failed_backends = {'realsense' for error in result['errors'] if error.startswith('RealSense discovery:')}
            if any(error.startswith('UVC discovery') for error in result['errors']):
                failed_backends.add('uvc')
            if result['errors'] and not result['devices'] and not failed_backends:
                # Preserve cached identities on an inconclusive scan, but still
                # service crashed/stalled captures below.
                devices = self._devices
            else:
                devices = result['devices'] + [d for d in self._devices if d['backend'] in failed_backends]
            self._devices = list({d['id']: d for d in devices}.values())
            present = {d['id'] for d in self._devices}
            for device_id in list(self._sessions):
                session = self._sessions[device_id]
                if device_id not in present or not session.healthy():
                    session.close()
                    del self._sessions[device_id]
                    self._bindings = {key: value for key, value in self._bindings.items() if value != device_id}
            for key, cfg in self._configs.items():
                if cfg['enabled']:
                    try:
                        self._activate(key)
                    except (ValueError, RuntimeError, OSError) as exc:
                        self._errors[key] = str(exc)

    def _release(self, instance_id):
        device_id = self._bindings.pop(instance_id, None)
        if device_id in self._sessions:
            session = self._sessions[device_id]
            session.remove(instance_id)
            if not session.routes:
                session.close()
                del self._sessions[device_id]

    def _activate(self, instance_id):
        cfg = self._configs[instance_id]
        device_id = cfg['device_id']
        if device_id == 'auto':
            available = [d for d in self._devices if d['capabilities']['rgb']]
            if len(available) != 1:
                raise ValueError('Multiple cameras available; select a device' if available else 'No camera available')
            device_id = cfg['device_id'] = available[0]['id']
            self._save()
        device = next((d for d in self._devices if d['id'] == device_id), None)
        if device is None:
            raise ValueError(f'Selected camera disconnected or unsupported: {device_id}')
        profiles = select_profiles(device, cfg['fps'], cfg['resolution'])
        existing = self._sessions.get(device_id)
        if existing is not None and existing.profiles != profiles:
            raise ValueError('Camera settings conflict with an active instance; stop the other instances first')
        if self._bindings.get(instance_id) != device_id:
            self._release(instance_id)
        if existing is None:
            existing = self._sessions[device_id] = WristCameraSession(self._namespace, device, profiles, self._quality)
        existing.add(instance_id, cfg['side'])
        self._bindings[instance_id] = device_id
        self._errors.pop(instance_id, None)

    def _info(self, instance_id):
        cfg = self._configs.get(instance_id)
        state = 'unconfigured' if cfg is None else 'idle'
        result = {'state': state, 'error': None, 'side': cfg['side'] if cfg else None,
                  'device_id': cfg['device_id'] if cfg else None}
        if cfg and cfg['enabled']:
            device_id = self._bindings.get(instance_id)
            if instance_id in self._errors:
                # An old live route is not evidence that a requested side or
                # profile transition succeeded. Keep the failure visible.
                result.update(state='error', error=self._errors[instance_id])
            elif device_id in self._sessions:
                result.update(self._sessions[device_id].info(instance_id, cfg['side']))
            else:
                result.update(state='error', error='Waiting for selected camera')
        result.update(instance_id=instance_id, available_devices=self._devices,
                      discovery_errors=self._discovery_errors, topic_in=[],
                      topic_out=[{'topic': camera_topic(self._namespace, instance_id, c, cfg['side'] if cfg else None),
                                  'format': FORMATS[c], 'desc': c.upper()} for c in CHANNELS] if instance_id else [])
        if not instance_id:
            # Core heartbeats discover topics without a canvas instance_id.
            result['topic_out'] = [port for key, saved in self._configs.items() if saved['enabled']
                                   for port in self._info(key)['topic_out']]
        return result

    def dispatch(self, action, args):
        with self._lock:
            instance_id = args.get('instance_id', '')
            try:
                if action == 'info':
                    return self._info(instance_id)
                if action == 'stop':
                    for key in [instance_id] if instance_id else list(self._configs):
                        if key in self._configs:
                            self._configs[key]['enabled'] = False
                            self._release(key)
                            self._errors.pop(key, None)
                    self._save()
                    return self._info(instance_id)
                if action == 'config':
                    values = {k: v for k, v in args.items() if k not in ('instance_id', 'action', '_tool_name')}
                    self._validate(instance_id, values)
                    old = self._configs.get(instance_id, {})
                    selection = values.get('device_id', 'auto')
                    # Core replays the saved 'auto' before every start. Keep
                    # its resolved identity even when a second camera appears.
                    device_id = (old.get('device_id', 'auto')
                                 if selection == 'auto' and old.get('selection') == 'auto' else selection)
                    cfg = {'side': values['side'], 'device_id': device_id, 'selection': selection,
                           'fps': str(values.get('fps', 'auto')), 'resolution': values.get('resolution', 'auto'),
                           'enabled': True}
                    # Detach only when physical device/settings change. A side
                    # relabel updates routes, keeping acquisition and topics.
                    if old and any(old.get(k) != cfg[k] for k in ('device_id', 'fps', 'resolution')):
                        self._release(instance_id)
                    self._configs[instance_id] = cfg
                    self._save()
                    self._activate(instance_id)
                    return self._info(instance_id)
                if action == 'start':
                    if instance_id not in self._configs:
                        raise ValueError('Configure this wrist camera instance first')
                    self._configs[instance_id]['enabled'] = True
                    self._save()
                    self._activate(instance_id)
                    return self._info(instance_id)
                raise ValueError(f'Unknown action: {action}')
            except (ValueError, RuntimeError, OSError) as exc:
                if instance_id:
                    self._errors[instance_id] = str(exc)
                return {'state': 'error', 'error': str(exc), 'instance_id': instance_id}
