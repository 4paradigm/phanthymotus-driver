"""Reusable camera interface, automatic backend loading and isolation contracts."""
import importlib
import multiprocessing as mp
from pathlib import Path
import subprocess
import sys
import time
import uuid

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def registry():
    return importlib.import_module('common.cameras.registry')


def plugins(tmp_path, monkeypatch, sources):
    name = 'camera_plugins_' + uuid.uuid4().hex
    directory = tmp_path / name
    directory.mkdir()
    (directory / '__init__.py').write_text('')
    for backend, body in sources.items():
        (directory / f'{backend}.py').write_text(body)
    monkeypatch.syspath_prepend(str(tmp_path))
    monkeypatch.setattr(registry(), 'backend_modules', lambda: [f'{name}.{backend}' for backend in sources])


def source(backend, physical_id=None):
    return f'''
class Backend:
    id = {backend!r}
    def discover(self):
        return [{{'id': {backend!r} + ':serial', 'backend': self.id, 'name': 'Test camera',
                 'physical_id': {physical_id!r}, 'capabilities': {{'rgb': True, 'depth': False}},
                 'profiles': {{'rgb': [{{'width': 640, 'height': 480, 'fps': 15, 'format': 'bgr8'}}], 'depth': []}}}}]
    def open(self, device, mode):
        raise NotImplementedError()
BACKEND = Backend()
'''


def test_backend_module_inventory_is_automatic_without_importing_modules(monkeypatch, tmp_path):
    package = importlib.import_module('common.cameras.backends')
    (tmp_path / 'third_vendor.py').write_text("raise RuntimeError('must not import in parent')")
    (tmp_path / '_private.py').write_text('')
    monkeypatch.setattr(package, '__path__', [str(tmp_path)])
    assert registry().backend_modules() == ['common.cameras.backends.third_vendor']


def test_builtin_backends_and_registry_do_not_load_native_sdks_on_import():
    script = '''
import sys
from common.cameras.registry import backend_modules, get_backend
assert set(backend_modules()) >= {'common.cameras.backends.realsense', 'common.cameras.backends.uvc'}
assert get_backend('realsense').id == 'realsense'
assert get_backend('uvc').id == 'uvc'
assert 'pyrealsense2' not in sys.modules
assert 'cv2' not in sys.modules
'''
    subprocess.run([sys.executable, '-c', script], cwd=ROOT, check=True)


def test_missing_crashed_and_hung_backends_do_not_hide_a_working_backend(tmp_path, monkeypatch):
    plugins(tmp_path, monkeypatch, {
        'healthy': source('healthy'),
        'missing': "import nonexistent_camera_sdk_for_test\n",
        'crashed': "import os\nos._exit(23)\n",
        'hung': "import time\ntime.sleep(30)\n",
    })
    before = {p.pid for p in mp.active_children()}
    started = time.monotonic()
    result = registry().discover_devices(timeout_s=1.2)
    assert [d['id'] for d in result['devices']] == ['healthy:serial']
    assert set(result['failed_backends']) == {'missing', 'crashed', 'hung'}
    assert any('nonexistent_camera_sdk_for_test' in error for error in result['errors'])
    assert time.monotonic() - started < 4
    assert {p.pid for p in mp.active_children()} == before


def test_native_backend_claims_physical_device_once_before_uvc(tmp_path, monkeypatch):
    plugins(tmp_path, monkeypatch, {'uvc': source('uvc', 'usb:2-1'), 'vendor': source('vendor', 'usb:2-1')})
    result = registry().discover_devices(timeout_s=2)
    assert result['errors'] == []
    assert [d['id'] for d in result['devices']] == ['vendor:serial']


def test_unknown_physical_identity_is_not_guessed_from_similar_camera_names(tmp_path, monkeypatch):
    plugins(tmp_path, monkeypatch, {'uvc': source('uvc'), 'vendor': source('vendor')})
    result = registry().discover_devices(timeout_s=2)
    assert {d['id'] for d in result['devices']} == {'uvc:serial', 'vendor:serial'}


def test_unknown_backend_is_rejected_instead_of_opening_as_uvc():
    with pytest.raises(RuntimeError, match='unknown|Unknown|unavailable'):
        registry().get_backend('nonexistent_vendor')


def test_physical_usb_identity_ignores_video_and_interface_suffix():
    physical_id = importlib.import_module('common.cameras.interface').physical_usb_id
    assert physical_id('/sys/devices/usb2/2-1/2-1.3/2-1.3:1.0/video4linux/video10') == 'usb:2-1.3'
    assert physical_id('/sys/devices/usb2/2-1/2-1.3/2-1.3:1.2/video4linux/video2') == 'usb:2-1.3'
    assert physical_id('/dev/video2') is None


def test_uvc_capture_owns_frame_and_does_not_invent_an_unknown_actual_frame_rate():
    import numpy as np
    from types import SimpleNamespace as NS
    module = importlib.import_module('common.cameras.backends.uvc')
    raw = np.full((480, 640, 3), 17, np.uint8)
    released = []
    camera = NS(isOpened=lambda: True, set=lambda *a: True,
                get=lambda key: {1: 640, 2: 480, 3: 0, 4: 0}[key],
                read=lambda: (True, raw), release=lambda: released.append(True))
    cv2 = NS(VideoCapture=lambda *a: camera, CAP_V4L2=0, CAP_PROP_FOURCC=4,
             CAP_PROP_FRAME_WIDTH=1, CAP_PROP_FRAME_HEIGHT=2, CAP_PROP_FPS=3,
             VideoWriter_fourcc=lambda *a: 0)
    mode = {'rgb': {'width': 640, 'height': 480, 'fps': 15, 'format': 'MJPG'}}
    capture = module.Capture(cv2, {'name': 'Webcam', 'path': '/dev/camera'}, mode)
    try:
        assert capture.info().profiles['rgb']['fps'] is None
        frame = capture.read(1000)['rgb']
        raw[:] = 0
        assert frame.data[0, 0].tolist() == [17, 17, 17]
        assert frame.intrinsics is None and frame.timestamp_ms is None
    finally:
        capture.close()
        capture.close()
    assert released == [True]


@pytest.mark.parametrize('broken', [
    "{'id': 'broken:serial', 'backend': 'broken', 'name': 'Camera'}",
    "{'id': ['serial'], 'backend': 'broken', 'name': 'Camera'}",
    "{'id': 'broken:serial', 'backend': 'broken', 'name': 'Camera', 'capabilities': {'rgb': True, 'depth': False}, 'profiles': {'rgb': [{'width': 640, 'height': 480, 'fps': 0, 'format': 'BGR3'}]}}",
    "{'id': 'broken:serial', 'backend': 'broken', 'name': 'Camera', 'capabilities': {'rgb': True, 'depth': False}, 'profiles': {'rgb': []}, 'physical_id': []}",
])
def test_malformed_descriptors_are_isolated_to_the_backend(tmp_path, monkeypatch, broken):
    bad_source = f'''class Backend:
    id = 'broken'
    def discover(self): return [{broken}]
    def open(self, *args): raise NotImplementedError()
BACKEND = Backend()
'''
    plugins(tmp_path, monkeypatch, {'healthy': source('healthy'), 'broken': bad_source})
    result = registry().discover_devices(timeout_s=2)
    assert [d['id'] for d in result['devices']] == ['healthy:serial']
    assert result['failed_backends'] == ['broken']
    assert len(result['errors']) == 1 and result['errors'][0].startswith('broken discovery:')


@pytest.mark.parametrize('fourcc,expected', [(int.from_bytes(b'YUYV', 'little'), 'YUYV'), (0, None)])
def test_uvc_reports_negotiated_fourcc_or_unknown_instead_of_requested_value(fourcc, expected):
    from types import SimpleNamespace as NS
    module = importlib.import_module('common.cameras.backends.uvc')
    camera = NS(isOpened=lambda: True, set=lambda *a: False,
                get=lambda key: {1: 640, 2: 480, 3: 15, 4: fourcc}[key], release=lambda: None)
    cv2 = NS(VideoCapture=lambda *a: camera, CAP_V4L2=0, CAP_PROP_FOURCC=4,
             CAP_PROP_FRAME_WIDTH=1, CAP_PROP_FRAME_HEIGHT=2, CAP_PROP_FPS=3,
             VideoWriter_fourcc=lambda *a: 0)
    capture = module.Capture(cv2, {'name': 'Webcam', 'path': '/dev/camera'},
                             {'rgb': {'width': 640, 'height': 480, 'fps': 15, 'format': 'MJPG'}})
    try:
        assert capture.info().profiles['rgb']['format'] == expected
    finally:
        capture.close()
