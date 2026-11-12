"""Standard V4L2/UVC RGB capture; never opens SDK-owned/head-camera nodes."""
import glob
import math
import os
from pathlib import Path
import re
import shutil
import subprocess

from common.cameras.interface import ActiveCameraInfo, BackendUnavailable, CameraFrame, physical_usb_id


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
            result.append({'physical_id': physical_usb_id(f'/sys/class/video4linux/{Path(path).name}/device'), 'id': f'uvc:{stable_id}', 'name': name, 'backend': 'uvc',
                           'path': stable_path, 'capabilities': {'rgb': True, 'depth': False},
                           'profiles': {'rgb': profiles, 'depth': []}})
            buses.add(bus)
        except (OSError, subprocess.SubprocessError):
            continue
    return result


class Capture:
    def __init__(self, cv2, device, mode):
        self._cap = cv2.VideoCapture(device['path'], cv2.CAP_V4L2)
        try:
            if not self._cap.isOpened():
                raise RuntimeError('USB camera could not be opened')
            if set(mode) != {'rgb'}:
                raise ValueError('UVC backend supports RGB only')
            p = mode['rgb']
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*p['format']))
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, p['width'])
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, p['height'])
            self._cap.set(cv2.CAP_PROP_FPS, p['fps'])
            width, height = round(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)), round(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if (width, height) != (p['width'], p['height']):
                raise RuntimeError(f'USB camera did not accept native resolution: {(width, height)}')
            fps = self._cap.get(cv2.CAP_PROP_FPS)
            if fps > 0 and abs(fps - p['fps']) > .5:
                raise RuntimeError(f'USB camera did not accept frame rate: {fps}')
            code = self._cap.get(cv2.CAP_PROP_FOURCC)
            actual_format = None
            if math.isfinite(code) and 0 < code <= 0xFFFFFFFF:
                raw_format = int(code).to_bytes(4, 'little')
                if all(32 <= value < 127 for value in raw_format):
                    actual_format = raw_format.decode('ascii')
            self._info = ActiveCameraInfo({'rgb': {**p, 'width': width, 'height': height,
                                                  'fps': fps if fps > 0 else None,
                                                  'format': actual_format}}, device['name'])
        except Exception:
            self.close()
            raise

    def info(self):
        return self._info

    def read(self, timeout_ms):
        # Some V4L2 drivers block in native read. The process/session watchdog
        # remains the upper bound even when the driver cannot honor a timeout.
        ok, data = self._cap.read()
        if not ok:
            raise RuntimeError('USB camera read failed or device disconnected')
        return {'rgb': CameraFrame(data.copy())}

    def close(self):
        if self._cap is not None:
            cap, self._cap = self._cap, None
            cap.release()


class Backend:
    id = 'uvc'

    def discover(self):
        if not shutil.which('v4l2-ctl'):
            raise BackendUnavailable('UVC discovery requires v4l2-ctl (v4l-utils)')
        return uvc_devices()

    def open(self, device, requested_mode):
        import cv2
        return Capture(cv2, device, requested_mode)


BACKEND = Backend()
