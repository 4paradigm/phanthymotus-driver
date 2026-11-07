"""Card/Bundle -> socket IPC -> camera frameset hook -> persistent snapshot.

Only the RealSense/ROS boundaries are fake. No camera or motion SDK is opened.
"""
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT/'noetix/bumi'
sys.path.insert(0, str(ROOT))
from noetix.bumi.rgbd_snapshot import SnapshotClient, SnapshotProducer


class Frame:
    def __init__(self, number, stamp, pixels, domain='hardware_clock'):
        self.number, self.stamp, self.pixels, self.domain = number, stamp, pixels, domain
        intr = NS(width=2, height=2, fx=2., fy=2., ppx=1., ppy=1., model='none', coeffs=[0.]*5)
        self.profile = NS(as_video_stream_profile=lambda: NS(intrinsics=intr))

    def get_timestamp(self): return self.stamp
    def get_frame_number(self): return self.number
    def get_frame_timestamp_domain(self): return self.domain
    def get_data(self): return self.pixels


def frames(number, stamp=None, depth_number=None, skew=1, domain='hardware_clock'):
    stamp = number*30. if stamp is None else stamp
    rgb = Frame(number, stamp, np.full((2, 2, 3), [5, 10, 15], dtype=np.uint8))
    depth = Frame(number if depth_number is None else depth_number, stamp+skew,
                  np.array([[0, 1000], [2000, 65535]], dtype=np.uint16), domain)
    return NS(get_color_frame=lambda: rgb, get_depth_frame=lambda: depth)


@pytest.fixture
def harness(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location('snapshot_device_loader', BUMI/'tests/test_loco_destination.py')
    loader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loader)
    device, _ = loader._load_device(monkeypatch)
    monkeypatch.setattr(device, '_acp_notify', lambda *_args: pytest.fail('No ACP for synchronous snapshot'))
    parent, child = socket.socketpair()
    producer = SnapshotProducer(child, 'real-device-serial-in-production', .001)
    client = SnapshotClient(parent)
    camera = NS(is_running=lambda: True, capture_rgbd=client.capture)
    card = device.VisionCapturePlugin({'output_dir': str(tmp_path)}, camera)
    producer.observe(frames(1), frames(1))
    yield NS(device=device, producer=producer, client=client, camera=camera, card=card, output=tmp_path)
    card._rgbd_capture.cancel()
    client.close()
    producer.close()
    producer.sender.join(timeout=1)


def begin(h, args=None, dispatch=None):
    result = []
    invoke = dispatch or h.card.dispatch
    thread = threading.Thread(target=lambda: result.append(invoke('capture_rgbd', args or {})))
    thread.start()
    deadline = time.monotonic()+1
    while h.producer.pending is None and time.monotonic() < deadline:
        h.producer.poll()
        time.sleep(.001)
    assert h.producer.pending is not None
    return thread, result


def finish(h, thread, result, pair=None):
    pair = pair or frames(2)
    h.producer.observe(pair, pair)
    thread.join(timeout=2)
    assert not thread.is_alive()
    return result[0]


def make_bundle(h, monkeypatch):
    # Execute the real Bundle with ROS/SDK imports stubbed; keep the real Card.
    import types
    monkeypatch.setitem(sys.modules, 'device', h.device)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', types.ModuleType('rclpy.executors'))
    monkeypatch.setattr(h.device, 'CameraPlugin', lambda *_args: h.camera)
    main = types.ModuleType('snapshot_bundle')
    main.__file__ = str(BUMI/'main.py')
    exec(compile((BUMI/'main.py').read_text(encoding='utf-8'), main.__file__, 'exec'), main.__dict__)
    bundle = main.BumiDeviceBundle({'plugins': {'camera': {'enabled': True},
        'vision_capture': {'enabled': True, 'output_dir': str(h.output)}}}, 'fake', object(), None, None)
    # camera schema isn't under test here; the existing camera tests cover it.
    h.camera.get_tools = lambda: []
    return main, bundle


def test_bundle_real_card_call_and_files(harness, monkeypatch):
    h = harness
    _, bundle = make_bundle(h, monkeypatch)
    tool = next(t for t in bundle.get_all_tools() if t['name'] == 'vision_capture')
    assert 'capture_rgbd' in tool['inputSchema']['properties']['action']['enum']
    assert tool['inputSchema']['x-completion']['actions'] == ['record_video']
    def call(action, args):
        return bundle.dispatch('vision_capture', {'action': action, **args})
    thread, result = begin(h, dispatch=call)
    out = finish(h, thread, result)
    assert out['ok'] and out['guarantee_level'] == 'B'
    depth = np.load(out['files']['depth.npy'], allow_pickle=False)
    assert depth.dtype == np.dtype('<u2') and depth[0, 0] == 0
    assert depth[0, 1]*.001 == 1. and depth[1, 0]*.001 == 2.
    meta = json.loads(Path(out['files']['metadata.json']).read_text())
    assert meta['request_id'] == out['request_id'] and meta['invalid_depth_value'] == 0
    assert not meta['exposure_after_request_verified']
    assert meta['baseline']['rgb']['frame_number'] == 1
    assert meta['metadata']['rgb']['frame_number'] == 2
    assert Path(out['files']['rgb.ppm']).read_bytes().endswith(bytes([15, 10, 5])*4)
    import hashlib
    for name, info in meta['files'].items():
        data = Path(out['files'][name]).read_bytes()
        assert info['bytes'] == len(data) and info['sha256'] == hashlib.sha256(data).hexdigest()


def test_cached_device_frame_is_not_new(harness):
    h = harness
    thread, result = begin(h)
    h.producer.observe(frames(1), frames(1))
    assert thread.is_alive()
    out = finish(h, thread, result, frames(3, depth_number=8))
    assert out['ok']  # Stream counters need not equal one another.


@pytest.mark.parametrize('pair, code', [
    (frames(2, domain='system_time'), 'CLOCK_DOMAIN_CHANGED'),
    (frames(2, skew=30), 'PAIR_SKEW'),
    (frames(0), 'CAMERA_RESTART'),
    (frames(2, stamp=-1), 'CAMERA_RESTART'),
    (frames(2, stamp=float('nan')), 'INVALID_PAIR'),
    (frames(2, stamp=10000), 'CLOCK_DISCONTINUITY'),
])
def test_invalid_pairs_fail(harness, pair, code):
    thread, result = begin(harness)
    out = finish(harness, thread, result, pair)
    assert not out['ok'] and out['code'] == code
    assert not list(harness.output.rglob('metadata.json'))


def test_rgb_depth_domain_mismatch(harness):
    h = harness
    prior = frames(1, domain='system_time')
    h.producer.latest = None
    h.producer.observe(prior, prior)
    thread, result = begin(h)
    out = finish(h, thread, result, frames(2, domain='system_time'))
    assert out['code'] == 'PAIR_DOMAIN_MISMATCH'


def test_alignment_identity_mismatch(harness):
    h = harness
    thread, result = begin(h)
    h.producer.observe(frames(2), frames(3))
    thread.join(timeout=2)
    assert result[0]['code'] == 'INVALID_PAIR'


def test_epoch_change_during_request_is_rejected(harness):
    h = harness
    thread, result = begin(h)
    h.producer.epoch = 'new-camera-session'
    out = finish(h, thread, result)
    assert out['code'] == 'CAMERA_RESTART' and not out['ok']


def test_strict_exposure_is_explicitly_unsupported(harness):
    out = harness.card.dispatch('capture_rgbd', {'require_exposure_after_request': True})
    assert out['code'] == 'EXPOSURE_UNVERIFIABLE' and harness.producer.pending is None


def test_timeout_and_cancel_release_own_demand(harness):
    h = harness
    thread, result = begin(h, {'timeout_s': .1})
    thread.join(timeout=1)
    assert result[0]['code'] == 'TIMEOUT'
    # Consume cancellation without waiting for another camera frame.
    for _ in range(5): h.producer.poll()
    assert h.producer.pending is None
    thread, result = begin(h)
    assert h.card.dispatch('capture_rgbd', {})['code'] == 'BUSY'
    h.card.dispatch('stop', {})
    thread.join(timeout=1)
    assert result[0]['code'] == 'CANCELLED'
    for _ in range(5): h.producer.poll()
    assert h.producer.pending is None


def test_disconnect_is_failure(harness):
    thread, result = begin(harness)
    harness.producer.close()
    thread.join(timeout=1)
    assert not result[0]['ok']


def test_saving_failure_cleans_partial_files(harness, monkeypatch):
    h = harness
    def fail(*_args, **_kwargs): raise OSError('simulated disk failure')
    monkeypatch.setattr(np, 'save', fail)
    thread, result = begin(h)
    out = finish(h, thread, result)
    assert not out['ok'] and list((h.output/'rgbd').iterdir()) == []


def test_slow_save_does_not_block_stop_or_producer(harness, monkeypatch):
    h = harness
    entered, release = threading.Event(), threading.Event()
    original = np.save
    def slow(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return original(*args, **kwargs)
    monkeypatch.setattr(np, 'save', slow)
    thread, result = begin(h)
    pair = frames(2)
    h.producer.observe(pair, pair)
    assert entered.wait(1)
    try:
        h.card._rgbd_capture.cancel()
        h.producer.observe(frames(3), frames(3))
        assert h.producer.latest['rgb']['frame_number'] == 3
    finally:
        release.set()
        thread.join(timeout=2)
    assert result[0]['code'] == 'CANCELLED'
    assert list((h.output/'rgbd').iterdir()) == []


def test_snapshot_demand_independent_of_vo(harness):
    h = harness
    # The VO consumer owns this flag; the snapshot branch never reads/writes it.
    from noetix.bumi.visual_inputs import RawCameraPublisher
    raw = object.__new__(RawCameraPublisher)
    raw.enabled = False
    thread, result = begin(h)
    assert finish(h, thread, result)['ok'] and raw.enabled is False
    raw.enabled = True
    h.card._rgbd_capture.cancel()
    assert raw.enabled is True


@pytest.mark.skipif(os.name != 'posix', reason='Production FD inheritance is a Linux/POSIX boundary')
def test_real_child_process_fd_inheritance():
    parent, child = socket.socketpair()
    script = '''
import importlib.util, socket, sys, time
spec = importlib.util.spec_from_file_location('fake_frames_only', sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
from noetix.bumi.rgbd_snapshot import SnapshotProducer
p = SnapshotProducer(socket.socket(fileno=int(sys.argv[2])), 'fake-child-serial', .001)
p.observe(module.frames(1), module.frames(1))
try:
    for number in range(2, 80):
        p.poll()
        pair = module.frames(number)
        p.observe(pair, pair)
        time.sleep(.01)
finally:
    p.close()
'''
    process = subprocess.Popen([sys.executable, '-c', script, str(Path(__file__)), str(child.fileno())],
                               pass_fds=(child.fileno(),), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    child.close()
    client = SnapshotClient(parent)
    try:
        packet = client.capture('test-cross-process', time.monotonic()+4, threading.Event())
        assert packet['request_id'] == 'test-cross-process'
        assert packet['metadata']['source'] == 'fake-child-serial'
        assert packet['metadata']['rgb']['frame_number'] > packet['baseline']['rgb']['frame_number']
        _, errors = process.communicate(timeout=3)
        assert process.returncode == 0, errors.decode()
    finally:
        client.close()
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=2)


def test_existing_photo_and_video_contracts(harness, monkeypatch):
    h = harness
    counter = [0]
    def color_frame(after_sequence=None, timeout_s=5):
        counter[0] += 1
        return {'data': b'fake-JPEG-codec-boundary', 'timestamp_ms': time.time()*1000,
                'frame_sequence': counter[0]}, counter[0]
    h.camera.wait_for_color_frame = color_frame
    photo = h.card.dispatch('capture_photo', {})
    assert photo['ok'] and Path(photo['file_path']).read_bytes() == b'fake-JPEG-codec-boundary'
    # Exercise the unchanged async video worker and ACP; only encoder/codec I/O is fake.
    finished, callbacks, encoded = threading.Event(), [], []
    def complete(*args):
        callbacks.append(args)
        finished.set()
    monkeypatch.setattr(h.device, '_acp_notify', complete)
    class Encoder:
        def __init__(self, args, **kwargs):
            self.path = Path(args[-1])
            self.read_fd, write_fd = os.pipe()
            self.stdin = os.fdopen(write_fd, 'wb', buffering=0)
            self.returncode = None
        def poll(self): return self.returncode
        def wait(self, timeout):
            self.path.write_bytes(b'fake-encoded-MP4')
            self.returncode = 0
            os.close(self.read_fd)
            return 0
    monkeypatch.setattr(h.device.subprocess, 'Popen', Encoder)
    monkeypatch.setattr(h.card, '_write_video_frame', lambda _p, data, _e: encoded.append(data) or True)
    video = h.card.dispatch('record_video', {'duration_s': 1})
    assert video['ok'] and video['state'] == 'queued' and video['action_id']
    assert finished.wait(3)
    assert len(callbacks) == 1 and callbacks[0][0] == video['action_id']
    assert callbacks[0][1] == 'completed' and encoded
    assert Path(callbacks[0][2]['file_path']).read_bytes() == b'fake-encoded-MP4'


def test_existing_snapshot_directory_is_never_deleted(harness, monkeypatch):
    h = harness
    import rgbd_snapshot
    existing = h.output/'rgbd'/'existing'
    existing.mkdir(parents=True)
    (existing/'keep.txt').write_text('existing user data')
    monkeypatch.setattr(rgbd_snapshot.uuid, 'uuid4', lambda: NS(hex='existing'))
    thread, result = begin(h)
    out = finish(h, thread, result)
    assert not out['ok']
    assert (existing/'keep.txt').read_text() == 'existing user data'
    assert not (existing.parent/'.existing.partial').exists()


def test_mcp_stop_runs_during_capture(harness, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    from http.server import ThreadingHTTPServer
    from urllib.request import Request, urlopen

    h = harness
    main, bundle = make_bundle(h, monkeypatch)
    main._bundle = bundle
    with ThreadingHTTPServer(('127.0.0.1', 0), main.make_handler()) as server:
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        url = f'http://127.0.0.1:{server.server_port}/mcp'

        def call(action):
            payload = {'jsonrpc': '2.0', 'id': action, 'method': 'tools/call',
                       'params': {'name': 'vision_capture', 'arguments': {'action': action}}}
            request = Request(url, json.dumps(payload).encode(), {'Content-Type': 'application/json'})
            with urlopen(request, timeout=3) as response:
                return json.loads(json.load(response)['result']['content'][0]['text'])

        try:
            with ThreadPoolExecutor() as pool:
                capture = pool.submit(call, 'capture_rgbd')
                deadline = time.monotonic() + 2
                while h.producer.pending is None and time.monotonic() < deadline:
                    h.producer.poll()
                    time.sleep(.001)
                assert h.producer.pending is not None
                assert call('stop')['rgbd_cancel_requested'] is True
                assert capture.result(timeout=2)['code'] == 'CANCELLED'
        finally:
            server.shutdown()
            worker.join(timeout=2)


def test_partial_cancel_closes_channel(harness, monkeypatch):
    from noetix.bumi import rgbd_snapshot

    h = harness
    original_send = rgbd_snapshot.send_packet

    def partial_send(sock, message):
        if message['action'] == 'cancel':
            sock.send(b'\x00')
            raise TimeoutError('cancel frame partially sent')
        original_send(sock, message)

    monkeypatch.setattr(rgbd_snapshot, 'send_packet', partial_send)
    worker, result = begin(h)
    h.card.dispatch('stop', {})
    worker.join(timeout=2)
    assert result[0]['code'] == 'CANCELLED'
    assert h.client.sock.fileno() == -1
    for _ in range(5):
        h.producer.poll()
    assert h.producer.closed.is_set()


def test_counter_reset_cannot_reclassify_delayed_old_frames(harness):
    h = harness
    worker, result = begin(h)
    assert finish(h, worker, result, frames(0))['code'] == 'CAMERA_RESTART'
    h.producer.observe(frames(2), frames(2))
    result = []
    worker = threading.Thread(target=lambda: result.append(h.card.dispatch('capture_rgbd', {})))
    worker.start()
    deadline = time.monotonic() + 2
    while worker.is_alive() and time.monotonic() < deadline:
        h.producer.poll()
        time.sleep(.001)
    worker.join(timeout=2)
    assert result[0]['code'] == 'CAMERA_RESTART'
    assert not list(h.output.rglob('metadata.json'))


def test_stop_after_rename_before_commit_cleans_published_directory(harness, monkeypatch):
    h = harness
    rename = Path.rename

    def stop_during_publish(source, target):
        result = rename(source, target)
        if source.name.endswith('.partial'):
            assert h.card.dispatch('stop', {})['rgbd_cancel_requested'] is True
        return result

    monkeypatch.setattr(Path, 'rename', stop_during_publish)
    worker, result = begin(h)
    assert finish(h, worker, result)['code'] == 'CANCELLED'
    assert list((h.output/'rgbd').iterdir()) == []
    assert h.card.dispatch('stop', {})['rgbd_cancel_requested'] is False
