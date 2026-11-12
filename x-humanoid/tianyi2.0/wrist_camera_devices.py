"""Native camera capabilities. SDK discovery runs in a disposable child process."""
from __future__ import annotations

import glob
import multiprocessing as mp
import os
from pathlib import Path
import re
import shutil
import subprocess


def select_profiles(device, fps='auto', resolution='auto'):
    """Choose native streams; the current depth wire format requires 640x480."""
    profiles = device['profiles']
    choices = {'rgb': list(profiles.get('rgb', []))}
    if profiles.get('depth'):
        choices['depth'] = [p for p in profiles['depth']
                            if (p['width'], p['height']) == (640, 480)]
        if not choices['depth']:
            raise ValueError('Depth requires a native 640x480 stream')
    if resolution != 'auto':
        for channel in choices:
            choices[channel] = [p for p in choices[channel]
                                if f"{p['width']}x{p['height']}" == resolution]
    if any(not values for values in choices.values()):
        raise ValueError(f'Unsupported native resolution: {resolution}')
    rates = set.intersection(*({p['fps'] for p in values} for values in choices.values()))
    if fps != 'auto':
        rates &= {int(fps)}
    if not rates:
        raise ValueError('No supported common frame rate for the selected streams')
    rate = 15 if 15 in rates else min(rates, key=lambda n: (abs(n - 15), n))
    return {channel: min((p for p in values if p['fps'] == rate),
                         key=lambda p: (abs(p['width'] - 640) + abs(p['height'] - 480),
                                        p['format']))
            for channel, values in choices.items()}


def realsense_devices(rs):
    result = []
    for dev in rs.context().query_devices():
        profiles = {'rgb': [], 'depth': []}
        for sensor in dev.query_sensors():
            for p in sensor.get_stream_profiles():
                channel = ('rgb' if p.stream_type() == rs.stream.color else
                           'depth' if p.stream_type() == rs.stream.depth else None)
                allowed = (rs.format.rgb8, rs.format.bgr8) if channel == 'rgb' else (rs.format.z16,)
                if channel is None or p.format() not in allowed:
                    continue
                video = p.as_video_stream_profile()
                profile = {'width': video.width(), 'height': video.height(),
                           'fps': p.fps(), 'format': str(p.format()).split('.')[-1]}
                if profile not in profiles[channel]:
                    profiles[channel].append(profile)
        serial = dev.get_info(rs.camera_info.serial_number)
        result.append({'id': f'realsense:{serial}', 'serial': serial,
                       'name': dev.get_info(rs.camera_info.name), 'backend': 'realsense',
                       'capabilities': {c: bool(v) for c, v in profiles.items()},
                       'profiles': profiles})
    return result


def uvc_devices(nodes=None, by_id='/dev/v4l/by-id'):
    """Enumerate capture nodes without opening them or stealing vendor cameras."""
    result, buses = [], set()
    links = {}
    for path in sorted(glob.glob(f'{by_id}/*')):
        links.setdefault(os.path.realpath(path), path)
    paths = nodes if nodes is not None else glob.glob('/dev/video*')
    # Prefer a deterministic interface by stable identity, not video2/video10
    # lexical order. Kernel video indices can change after every reconnect.
    for path in sorted(paths, key=lambda p: links.get(os.path.realpath(p), p)):
        try:
            info = subprocess.check_output(['v4l2-ctl', '-d', path, '--info'],
                                           text=True, timeout=1, stderr=subprocess.DEVNULL,
                                           env={**os.environ, 'LC_ALL': 'C'})
            # Device Caps, unlike aggregate Capabilities, describes THIS node.
            caps = info.split('Device Caps', 1)[-1]
            name_match = re.search(r'Card type\s*:\s*(.+)', info)
            name = name_match[1].strip() if name_match else 'USB Camera'
            # These SDK-owned devices expose several raw nodes; never offer
            # their IR/depth interfaces as generic RGB, or open the head camera.
            if 'Video Capture' not in caps or any(v in name.lower() for v in ('realsense', 'orbbec')):
                continue
            bus_match = re.search(r'Bus info\s*:\s*(.+)', info)
            bus = bus_match[1].strip() if bus_match else os.path.realpath(path)
            if bus in buses:
                continue
            formats = subprocess.check_output(['v4l2-ctl', '-d', path, '--list-formats-ext'],
                                              text=True, timeout=1, stderr=subprocess.DEVNULL,
                                              env={**os.environ, 'LC_ALL': 'C'})
            profiles, fmt, size = [], None, None
            for line in formats.splitlines():
                m = re.search(r"\[\d+\]:\s*'([^']+)'", line)
                if m:
                    fmt, size = m[1], None
                m = re.search(r'Size: Discrete (\d+)x(\d+)', line)
                if m:
                    size = (int(m[1]), int(m[2]))
                m = re.search(r'\(([\d.]+) fps\)', line)
                if m and size and fmt in ('MJPG', 'JPEG', 'YUYV', 'UYVY', 'RGB3', 'BGR3'):
                    profiles.append({'width': size[0], 'height': size[1],
                                     'fps': round(float(m[1]), 3), 'format': fmt})
            if not profiles:
                continue
            stable_path = links.get(os.path.realpath(path), path)
            # by-id is device-specific; without a serial, the USB port is the
            # best available identity. Never bind to a renumberable video index.
            stable_id = re.sub(r'-video-index\d+$', '', Path(stable_path).name) if stable_path != path else bus
            if stable_id.startswith('/dev/video'):
                continue
            result.append({'id': f'uvc:{stable_id}', 'name': name, 'backend': 'uvc',
                           'path': stable_path, 'capabilities': {'rgb': True, 'depth': False},
                           'profiles': {'rgb': profiles, 'depth': []}})
            buses.add(bus)
        except (OSError, subprocess.SubprocessError):
            continue
    return result


def _scan(connection):
    from common import logsafe
    logsafe.install()
    devices, errors = [], []
    try:
        try:
            import pyrealsense2 as rs
            devices.extend(realsense_devices(rs))
        except Exception as exc:
            errors.append(f'RealSense discovery: {exc}')
        if shutil.which('v4l2-ctl'):
            devices.extend(uvc_devices())
        else:
            errors.append('UVC discovery requires v4l2-ctl (v4l-utils)')
        connection.send({'devices': devices, 'errors': errors})
    finally:
        connection.close()


def discover_devices():
    """Bound SDK crashes/hangs without importing the SDK in the body driver."""
    ctx = mp.get_context('spawn')
    receiver, sender = ctx.Pipe(duplex=False)
    proc = ctx.Process(target=_scan, args=(sender,), daemon=True)
    try:
        proc.start()
        sender.close()
        if receiver.poll(8):
            return receiver.recv()
        return {'devices': [], 'errors': ['Camera discovery timed out']}
    except (EOFError, OSError) as exc:
        return {'devices': [], 'errors': [f'Camera discovery failed: {exc}']}
    finally:
        if proc.pid is not None:
            proc.join(timeout=.2)
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=.5)
            if proc.is_alive():
                proc.kill()
                proc.join(timeout=.5)
            proc.close()
        receiver.close()
        sender.close()
