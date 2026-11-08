"""Card/Bundle -> socket IPC -> camera frameset hook -> persistent snapshot.

Only the RealSense/ROS boundaries are fake. No camera or motion SDK is opened.
"""
import ast
import asyncio
from contextlib import contextmanager
import importlib.util
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
from queue import Queue
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
    parent, child = socket.socketpair()
    producer = SnapshotProducer(child, 'real-device-serial-in-production', .001)
    client = SnapshotClient(parent)
    camera = NS(is_running=lambda: True, capture_rgbd=client.capture, stop=lambda: None,
                wait_for_color_frame=lambda *_args: (None, 0))
    card = device.VisionCapturePlugin({'output_dir': str(tmp_path)}, camera)
    producer.observe(frames(1), frames(1))
    callbacks = Queue()
    h = NS(device=device, producer=producer, client=client, camera=camera, card=card,
           output=tmp_path, callbacks=callbacks, callback_status=200, receive=None)

    class Receiver(BaseHTTPRequestHandler):
        def do_POST(self):
            payload = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            callbacks.put((self.path, self.headers.get_content_type(), payload))
            reply = h.receive(payload) if h.receive else {}
            self.send_response(h.callback_status)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(reply).encode())

        def log_message(self, *_args):
            pass

    with ThreadingHTTPServer(('127.0.0.1', 0), Receiver) as server:
        monkeypatch.setenv('AGENT_CORE_URL', f'http://127.0.0.1:{server.server_port}')
        server_thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02})
        server_thread.start()
        try:
            yield h
        finally:
            card._rgbd_capture.cancel(wait=True)
            client.close()
            producer.close()
            producer.sender.join(timeout=1)
            server.shutdown()
            server_thread.join(timeout=1)
    assert callbacks.empty(), 'Unexpected duplicate or unconsumed ACP callback'


def await_camera_request(h):
    deadline = time.monotonic()+1
    while h.producer.pending is None and time.monotonic() < deadline:
        h.producer.poll()
        time.sleep(.001)
    assert h.producer.pending is not None


def begin(h, args=None, dispatch=None):
    invoke = dispatch or h.card.dispatch
    ack = invoke('capture_rgbd', args or {})
    assert ack == {'ok': True, 'state': 'queued', 'action_id': ack['request_id'],
                   'request_id': ack['request_id']}
    await_camera_request(h)
    return ack


def completion(h, ack, status):
    path, content_type, payload = h.callbacks.get(timeout=3)
    assert path == '/api/acp/complete' and content_type == 'application/json'
    assert payload['tool'] == 'vision_capture'
    assert payload['action_id'] == ack['action_id'] and payload['status'] == status
    assert isinstance(payload['ts'], float)
    return payload['result']


def result_for(h, ack, status):
    result = completion(h, ack, status)
    assert result['request_id'] == ack['request_id']
    assert result['ok'] == (status == 'completed')
    return result


def finish(h, ack, pair=None, *, status='completed'):
    pair = pair or frames(2)
    h.producer.observe(pair, pair)
    return result_for(h, ack, status)


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
    assert tool['inputSchema']['x-completion']['actions'] == ['record_video', 'capture_rgbd']
    def call(action, args):
        return bundle.dispatch('vision_capture', {'action': action, **args})
    ack = begin(h, dispatch=call)
    out = finish(h, ack)
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
    ack = begin(h)
    h.producer.observe(frames(1), frames(1))
    assert h.card._rgbd_capture.info()['active']
    assert h.callbacks.empty()
    out = finish(h, ack, frames(3, depth_number=8))
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
    ack = begin(harness)
    out = finish(harness, ack, pair, status='error')
    assert not out['ok'] and out['code'] == code
    assert not list(harness.output.rglob('metadata.json'))


def test_rgb_depth_domain_mismatch(harness):
    h = harness
    prior = frames(1, domain='system_time')
    h.producer.latest = None
    h.producer.observe(prior, prior)
    ack = begin(h)
    out = finish(h, ack, frames(2, domain='system_time'), status='error')
    assert out['code'] == 'PAIR_DOMAIN_MISMATCH'


def test_alignment_identity_mismatch(harness):
    h = harness
    ack = begin(h)
    h.producer.observe(frames(2), frames(3))
    assert result_for(h, ack, 'error')['code'] == 'INVALID_PAIR'


def test_epoch_change_during_request_is_rejected(harness):
    h = harness
    ack = begin(h)
    h.producer.epoch = 'new-camera-session'
    out = finish(h, ack, status='error')
    assert out['code'] == 'CAMERA_RESTART' and not out['ok']


def test_strict_exposure_is_explicitly_unsupported(harness):
    out = harness.card.dispatch('capture_rgbd', {'require_exposure_after_request': True})
    assert out['code'] == 'EXPOSURE_UNVERIFIABLE' and harness.producer.pending is None


def test_timeout_and_cancel_release_own_demand(harness):
    h = harness
    ack = begin(h, {'timeout_s': .1})
    assert result_for(h, ack, 'error')['code'] == 'TIMEOUT'
    # Consume cancellation without waiting for another camera frame.
    for _ in range(5): h.producer.poll()
    assert h.producer.pending is None
    ack = begin(h)
    assert h.card.dispatch('capture_rgbd', {})['code'] == 'BUSY'
    assert h.card.dispatch('stop', {})['rgbd_cancel_requested'] is True
    assert h.card.dispatch('stop', {})['rgbd_cancel_requested'] is False
    assert result_for(h, ack, 'cancelled')['code'] == 'CANCELLED'
    for _ in range(5): h.producer.poll()
    assert h.producer.pending is None


def test_disconnect_is_failure(harness):
    ack = begin(harness)
    harness.producer.close()
    assert not result_for(harness, ack, 'error')['ok']


def test_saving_failure_cleans_partial_files(harness, monkeypatch):
    h = harness
    def fail(*_args, **_kwargs): raise OSError('simulated disk failure')
    monkeypatch.setattr(np, 'save', fail)
    ack = begin(h)
    out = finish(h, ack, status='error')
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
    ack = begin(h)
    pair = frames(2)
    h.producer.observe(pair, pair)
    assert entered.wait(1)
    try:
        h.card._rgbd_capture.cancel()
        h.producer.observe(frames(3), frames(3))
        assert h.producer.latest['rgb']['frame_number'] == 3
    finally:
        release.set()
    assert result_for(h, ack, 'cancelled')['code'] == 'CANCELLED'
    assert list((h.output/'rgbd').iterdir()) == []


def test_snapshot_demand_independent_of_vo(harness):
    h = harness
    # The VO consumer owns this flag; the snapshot branch never reads/writes it.
    from noetix.bumi.visual_inputs import RawCameraPublisher
    raw = object.__new__(RawCameraPublisher)
    raw.enabled = False
    ack = begin(h)
    assert finish(h, ack)['ok'] and raw.enabled is False
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
    encoded = []
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
    result = completion(h, video, 'completed')
    assert encoded and Path(result['file_path']).read_bytes() == b'fake-encoded-MP4'



def test_existing_snapshot_directory_is_never_deleted(harness, monkeypatch):
    h = harness
    import rgbd_snapshot
    existing = h.output/'rgbd'/'existing'
    existing.mkdir(parents=True)
    (existing/'keep.txt').write_text('existing user data')
    monkeypatch.setattr(rgbd_snapshot.uuid, 'uuid4', lambda: NS(hex='existing'))
    ack = begin(h)
    out = finish(h, ack, status='error')
    assert not out['ok']
    assert (existing/'keep.txt').read_text() == 'existing user data'
    assert not (existing.parent/'.existing.partial').exists()


@contextmanager
def mcp_server(h, monkeypatch, before_send=None):
    main, bundle = make_bundle(h, monkeypatch)
    main._bundle = bundle
    handler = main.make_handler()
    if before_send is not None:
        send = handler._send
        def send_with_gate(self, status, body):
            before_send()
            send(self, status, body)
        monkeypatch.setattr(handler, '_send', send_with_gate)
    with ThreadingHTTPServer(('127.0.0.1', 0), handler) as server:
        worker = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02})
        worker.start()
        try:
            yield f'http://127.0.0.1:{server.server_port}/mcp', bundle
        finally:
            bundle.stop_all()
            server.shutdown()
            worker.join(timeout=2)


def mcp_call(url, action):
    from urllib.request import Request, urlopen

    payload = {'jsonrpc': '2.0', 'id': action, 'method': 'tools/call',
               'params': {'name': 'vision_capture', 'arguments': {'action': action}}}
    request = Request(url, json.dumps(payload).encode(), {'Content-Type': 'application/json'})
    with urlopen(request, timeout=2) as response:
        return json.loads(json.load(response)['result']['content'][0]['text'])


def test_mcp_stop_runs_during_capture(harness, monkeypatch):
    h = harness
    with mcp_server(h, monkeypatch) as (url, _):
        # No frame is available yet; a synchronous action would hit the HTTP timeout.
        ack = mcp_call(url, 'capture_rgbd')
        assert ack['state'] == 'queued' and ack['action_id'] == ack['request_id']
        await_camera_request(h)
        assert mcp_call(url, 'stop')['rgbd_cancel_requested'] is True
        assert result_for(h, ack, 'cancelled')['code'] == 'CANCELLED'


def test_immediate_snapshot_waits_for_mcp_response_write(harness, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor

    h = harness
    writing, release = threading.Event(), threading.Event()

    def before_send():
        writing.set()
        assert release.wait(2)

    with mcp_server(h, monkeypatch, before_send=before_send) as (url, bundle):
        with ThreadPoolExecutor() as pool:
            response = pool.submit(mcp_call, url, 'capture_rgbd')
            try:
                assert writing.wait(1)
                await_camera_request(h)
                pair = frames(2)
                h.producer.observe(pair, pair)
                card = next(p for p in bundle._plugins if isinstance(p, h.device.VisionCapturePlugin))
                deadline = time.monotonic() + 1
                while card._rgbd_capture.info()['active'] and time.monotonic() < deadline:
                    time.sleep(.001)
                assert card._rgbd_capture.info()['last_completion']['status'] == 'completed'
                assert h.callbacks.empty()
                assert card.dispatch('stop', {})['rgbd_cancel_requested'] is False
            finally:
                release.set()
            ack = response.result(timeout=2)
        result = result_for(h, ack, 'completed')
        assert Path(result['files']['metadata.json']).exists()


def test_callback_failure_logs_and_keeps_terminal_result(harness, capsys):
    h = harness
    h.callback_status = 503
    ack = begin(h)
    result = finish(h, ack)
    h.card._rgbd_capture.worker.join(timeout=2)
    assert not h.card._rgbd_capture.worker.is_alive()
    log = capsys.readouterr().out
    assert f'callback failed for {ack["action_id"]}' in log and '503' in log
    info = h.card.dispatch('info', {})['rgbd_snapshot']
    assert not info['active']
    assert info['last_completion'] == {'action_id': ack['action_id'], 'status': 'completed', 'result': result}
    h.callback_status = 200
    second = begin(h)
    assert second['action_id'] != ack['action_id']
    assert finish(h, second, frames(3))['ok']


@pytest.fixture
def core_acp(monkeypatch, harness):
    source = os.environ.get('BUMI_ACP_CORE_SOURCE')
    if not source:
        pytest.skip('Set BUMI_ACP_CORE_SOURCE to agent-core/src to verify upstream ACP')
    path = Path(source)
    events = []
    async def enqueue(**event):
        events.append(event)
    monkeypatch.setitem(sys.modules, 'config', NS())
    monkeypatch.setitem(sys.modules, 'event_bus', NS(enqueue=enqueue))
    monkeypatch.setitem(sys.modules, 'benchmark_facts', NS(note_acp_post=lambda _body: None))

    def load(name):
        spec = importlib.util.spec_from_file_location(name, path/f'{name}.py')
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, name, module)
        spec.loader.exec_module(module)
        return module

    client, hooks = load('mcp_client'), load('hooks')
    # Execute the unmodified endpoint body without starting Core's unrelated services.
    entrypoint = path/'start.py'
    tree = ast.parse(entrypoint.read_text(encoding='utf-8'))
    route = next(node for node in tree.body
                 if isinstance(node, ast.AsyncFunctionDef) and node.name == 'acp_complete')
    route.decorator_list = []
    scope = {'json': json, 'mcp_client': client, 'fastapi': NS(Request=object)}
    exec(compile(ast.Module(body=[route], type_ignores=[]), str(entrypoint), 'exec'), scope)

    def connect(url, bundle):
        loop = asyncio.get_running_loop()
        replies = asyncio.Queue()
        async def receive(payload):
            async def body():
                return payload
            reply = await scope['acp_complete'](NS(json=body))
            replies.put_nowait(reply)
            return reply
        harness.receive = lambda body: asyncio.run_coroutine_threadsafe(receive(body), loop).result(2)
        tool = next(t for t in bundle.get_all_tools() if t['name'] == 'vision_capture')
        schema = tool['inputSchema']
        client.registry['bumi'] = {'url': url, 'online': True, 'tools': ['vision_capture'],
            'tool_meta': {'mcp__bumi__vision_capture': {'type': tool['type'],
                'completion': schema['x-completion']}},
            'input_schemas': {'mcp__bumi__vision_capture': schema}}
        hooks.register('bumi', 'vision_capture', schema['x-hooks'])
        return replies

    return NS(client=client, hooks=hooks, connect=connect, events=events)


@pytest.mark.parametrize('early', [False, True], ids=['registered-first', 'response-not-yet-parsed'])
def test_upstream_acp_pending_registration_race(harness, monkeypatch, core_acp, early):
    h, core = harness, core_acp

    async def scenario(url, bundle):
        replies = core.connect(url, bundle)
        if early:
            parsing_paused, resume = asyncio.Event(), asyncio.Event()
            response_class = core.client.aiohttp.ClientResponse
            read_json = response_class.json

            async def pause_before_parsing(response, *args, **kwargs):
                if str(response.url) == url:
                    parsing_paused.set()
                    await resume.wait()
                return await read_json(response, *args, **kwargs)

            monkeypatch.setattr(response_class, 'json', pause_before_parsing)
            call = asyncio.create_task(core.client.call_tool(
                'mcp__bumi__vision_capture', {'action': 'capture_rgbd'}))
            try:
                await asyncio.wait_for(parsing_paused.wait(), 2)
                assert not core.client.get_pending_actions()
                await asyncio.to_thread(await_camera_request, h)
                pair = frames(2)
                h.producer.observe(pair, pair)
                reply = await asyncio.wait_for(replies.get(), 2)
                assert reply['ok'] and not core.client.get_pending_actions()
            finally:
                resume.set()
            ack = json.loads(await asyncio.wait_for(call, 2))
        else:
            ack = json.loads(await core.client.call_tool(
                'mcp__bumi__vision_capture', {'action': 'capture_rgbd'}))
            assert core.client.get_pending_actions() == [ack['action_id']]
            await asyncio.to_thread(await_camera_request, h)
            pair = frames(2)
            h.producer.observe(pair, pair)
            reply = await asyncio.wait_for(replies.get(), 2)

        action_id = ack['action_id']
        assert reply == {'ok': True, 'action_id': action_id}
        result = result_for(h, ack, 'completed')
        assert Path(result['files']['metadata.json']).exists()
        assert core.events[0]['payload']['status'] == 'completed'
        assert core.client._pending_actions[action_id].is_set() is not early
        if early:
            assert action_id not in core.client._pending_results
            core.client._pending_timeouts[action_id] = .02
        else:
            assert core.client._pending_results[action_id]['result'] == result
        assert await core.client.await_pending() == {
            'status': 'timeout' if early else 'completed', 'actions': [action_id]}
        assert not core.client.get_pending_actions()

    with mcp_server(h, monkeypatch) as (url, bundle):
        asyncio.run(scenario(url, bundle))


def test_upstream_interrupt_hook_bypasses_pending_barrier(harness, monkeypatch, core_acp):
    h, core = harness, core_acp

    async def scenario(url, bundle):
        replies = core.connect(url, bundle)
        ack = json.loads(await core.client.call_tool(
            'mcp__bumi__vision_capture', {'action': 'capture_rgbd'}))
        await asyncio.to_thread(await_camera_request, h)
        action_id = ack['action_id']
        assert core.client.get_pending_actions() == [action_id]
        barrier = asyncio.create_task(core.client.await_pending())
        await asyncio.sleep(0)
        assert not barrier.done()
        try:
            hooks = await asyncio.wait_for(core.hooks.fire('on_interrupt_all'), 2)
            assert hooks[0]['result']['rgbd_cancel_requested'] is True
            assert await asyncio.wait_for(replies.get(), 2) == {'ok': True, 'action_id': action_id}
            assert result_for(h, ack, 'cancelled')['code'] == 'CANCELLED'
            assert await asyncio.wait_for(barrier, 2) == {'status': 'completed', 'actions': [action_id]}
            assert core.events[0]['payload']['status'] == 'cancelled'
            assert not core.client.get_pending_actions()
        finally:
            await core.client.cancel_and_reap([barrier])

    with mcp_server(h, monkeypatch) as (url, bundle):
        asyncio.run(scenario(url, bundle))


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
    ack = begin(h)
    h.card.dispatch('stop', {})
    assert result_for(h, ack, 'cancelled')['code'] == 'CANCELLED'
    assert h.client.sock.fileno() == -1
    for _ in range(5):
        h.producer.poll()
    assert h.producer.closed.is_set()


def test_counter_reset_cannot_reclassify_delayed_old_frames(harness):
    h = harness
    ack = begin(h)
    assert finish(h, ack, frames(0), status='error')['code'] == 'CAMERA_RESTART'
    h.producer.observe(frames(2), frames(2))
    second = h.card.dispatch('capture_rgbd', {})
    assert second['action_id'] != ack['action_id']
    deadline = time.monotonic() + 2
    while h.callbacks.empty() and time.monotonic() < deadline:
        h.producer.poll()
        time.sleep(.001)
    assert result_for(h, second, 'error')['code'] == 'CAMERA_RESTART'
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
    ack = begin(h)
    assert finish(h, ack, status='cancelled')['code'] == 'CANCELLED'
    assert list((h.output/'rgbd').iterdir()) == []
    assert h.card.dispatch('stop', {})['rgbd_cancel_requested'] is False
