"""Existing three cards share one calibrated depth producer; no hardware."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

from test_tianyi_body_mount import source, emit, STAMP, m


class Display:
    def __init__(self, name, topic, fmt):
        self.name, self.topic, self.fmt = name, topic, fmt
        self.starts = self.stops = 0
    def get_tool(self):
        return dict(name=self.name, type='sensor', inputSchema={},
                    topic_out=[dict(topic=self.topic, format=self.fmt)])
    def start(self): self.starts += 1
    def stop(self): self.stops += 1
    def dispatch(self, action, args): return dict(angular_velocity=dict(x=1.))


def test_depth_is_published_on_original_topic_with_matching_camera_info(source):
    depth = next(p for p in source._outputs() if p['format'] == 'image/depth-zlib')
    assert depth['topic'] == '/test/camera/head/depth'
    assert source._declarations()[1]['topic'] == depth['topic']
    emit(source)
    assert [t for t, _ in source.events if t.endswith('/depth')] == [depth['topic']]
    assert '/test/camera/rgbd/depth' not in source.pub_node.publishers


def test_original_three_cards_distribute_ports_without_new_depth_port(source):
    assert hasattr(m, 'RgbdInputCard')
    rgb = m.RgbdInputCard(source, 'color', Display('camera_head', '/test/camera/head', 'image/jpeg'))
    depth = m.RgbdInputCard(source, 'depth')
    imu = m.RgbdInputCard(source, 'imu', Display('imu', '/test/state/imu', 'data/json'))
    tools = [card.get_tool() for card in (rgb, depth, imu)]
    assert [t['name'] for t in tools] == ['camera_head', 'camera_depth', 'imu']
    assert [p['topic'] for p in tools[1]['topic_out']] == ['/test/camera/head/depth']
    assert tools[0]['topic_out'] == [dict(topic='/test/camera/head', format='image/jpeg',
                                        description='配准相机数据')]
    assert tools[0]['camera_info'][0]['topic'] == '/test/camera/head'
    assert '/test/camera/rgbd/color' not in source.pub_node.publishers
    assert [p['format'] for p in tools[2]['topic_out']] == ['data/json', 'sensor/imu']
    assert tools[0]['camera_info'][0]['K'] == tools[1]['camera_info'][0]['K']
    assert tools[0]['camera_info'][0]['id'] == tools[1]['camera_info'][0]['id']
    assert imu.dispatch('read', {}) == dict(angular_velocity=dict(x=1.))


def test_card_start_stop_keeps_sibling_owner_and_is_idempotent(source):
    assert hasattr(m, 'RgbdInputCard')
    source.managed = True
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=STAMP-1))
    display = Display('camera_head', '/test/camera/head', 'image/jpeg')
    rgb, depth = m.RgbdInputCard(source, 'color', display), m.RgbdInputCard(source, 'depth')
    rgb.start(); rgb.start(); depth.start()
    assert display.starts == 1
    assert source.running
    rgb.stop(); rgb.stop()
    assert display.stops == 1 and source.running
    assert rgb.dispatch('info', {})['state'] == 'idle'
    emit(source)
    assert depth.dispatch('info', {})['state'] == 'running'
    depth.stop()
    assert not source.running


def test_stopped_rgb_does_not_publish_but_shared_depth_keeps_original_header(source):
    assert hasattr(m, 'RgbdInputCard')
    source.managed = True
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=STAMP-1))
    rgb, depth = m.RgbdInputCard(source, 'color'), m.RgbdInputCard(source, 'depth')
    rgb.start(); depth.start(); rgb.stop()
    emit(source)
    images = [(t, msg) for t, msg in source.events if not t.endswith('/body_mount')]
    assert [t for t, _ in images] == ['/test/camera/head/depth']
    assert m._stamp(images[0][1]) == STAMP
    assert images[0][1].header.frame_id == source.geometry['frame_id']


def test_bundle_registers_existing_cards_and_private_source_only():
    root = Path(__file__).resolve().parents[1]
    tree = ast.parse((root/'x-humanoid/tianyi2.0/main.py').read_text())
    bundle = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TianyiDeviceBundle')
    init = ast.unparse(next(n for n in bundle.body if isinstance(n, ast.FunctionDef) and n.name == '__init__'))
    assert 'RgbdInputCard' in init
    assert 'monitor_only=True' in init
    assert 'self._plugins.append(RgbdCameraPlugin(' not in init
    assert 'name: camera_rgbd' not in (root/'x-humanoid/tianyi2.0/driver.yaml').read_text()


def test_restarting_depth_cannot_publish_a_pre_restart_capture(source):
    assert hasattr(m, 'RgbdInputCard')
    source.managed = True
    now = [STAMP-1]
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=now[0]))
    rgb, depth = m.RgbdInputCard(source, 'color'), m.RgbdInputCard(source, 'depth')
    rgb.start(); depth.start(); depth.stop()
    now[0] = STAMP+1
    depth.start()
    original = source.color_pub.publish
    def color(message):
        original(message)
        source.stop_event.set()
    source.color_pub.publish = color
    emit(source)
    assert not any(t == '/test/camera/head/depth' for t, _ in source.events)
    emit(source, STAMP+2)
    depths = [msg for t, msg in source.events if t == '/test/camera/head/depth']
    assert len(depths) == 1 and m._stamp(depths[0]) == STAMP+2


def imu_display(monkeypatch, publish):
    import json, threading, time, sys
    tree = ast.parse((Path(__file__).resolve().parents[1]/'x-humanoid/tianyi2.0/device.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ImuPlugin')
    subscriptions = []
    class Node:
        def __init__(self, *a, **kw): pass
        def create_publisher(self, *a): return NS(publish=publish)
        def create_subscription(self, *a):
            subscriptions.append(a)
            return object()
    scope = dict(Node=Node, _LOW_LAT_QOS=object(), String=lambda **kw: NS(**kw),
                 _JsonSensor=type('Sensor', (), {}), threading=threading, time=time, json=json)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), 'ImuPlugin', 'exec'), scope)
    ros = NS(ctx_tianyi=None, ctx_core=None, executor_tianyi=NS(add_node=lambda n: None),
             executor_core=NS(add_node=lambda n: None))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Imu=object))
    return scope['ImuPlugin']({}, 'test', ros), subscriptions


def test_original_imu_display_restart_joins_snapshot_and_reuses_subscriptions(monkeypatch):
    import threading
    entered = threading.Event()
    display, subscriptions = imu_display(monkeypatch, lambda msg: entered.set())
    try:
        display.start(); first = display._pub_thread
        assert entered.wait(1)
        display.stop()
        assert not first.is_alive()
        display.start(); display.start()
        assert len(subscriptions) == 2
    finally:
        display.stop()


def test_shared_depth_retains_original_configured_rate(source):
    from test_tianyi_rgbd_camera import info
    ros = NS(ctx_tianyi=source._body_context, ctx_core=None, executor_tianyi=NS(add_node=lambda n: None),
             executor_core=NS(add_node=lambda n: None))
    shared = m.RgbdCameraPlugin({'imu_enabled': False, 'hz': 8}, 'rate', ros)
    shared.running = shared.registered = True
    shared.infos = {'color': info(), 'depth': info()}
    sampled = []
    try:
        for stamp in (1_000_000_000, 1_100_000_000, 1_125_000_000, 1_250_000_000):
            msg = NS(width=640, height=480, header=NS(frame_id='optical',
                stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9)))
            shared._image('color', msg); shared._image('depth', msg)
            if not shared.pending.empty(): sampled.append(m._stamp(shared.pending.get_nowait()[1]))
        assert sampled == [1_000_000_000, 1_125_000_000, 1_250_000_000]
    finally:
        shared.stop()
        shared._source_context.try_shutdown()
        shared._source_spin.join(timeout=1)


def test_queued_frame_cannot_cross_depth_restart_even_with_tolerated_clock_skew(source):
    import numpy as np
    from test_tianyi_rgbd_camera import info
    source.managed = True
    now = [STAMP]
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=now[0]))
    rgb, depth = m.RgbdInputCard(source, 'color'), m.RgbdInputCard(source, 'depth')
    rgb.start(); depth.start()
    source.infos = {kind: info(frame='head_optical') for kind in ('color', 'depth')}
    stamp = STAMP+20_000_000
    header = NS(frame_id='head_optical', stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9))
    color = NS(header=deepcopy(header), encoding='bgr8', width=640, height=480, step=1920,
               data=np.zeros((480,640,3), np.uint8).tobytes())
    measured = NS(header=deepcopy(header), encoding='16UC1', is_bigendian=False,
                  width=640, height=480, step=1280, data=np.ones((480,640), '<u2').tobytes())
    source._image('color', color); source._image('depth', measured)
    depth.stop(); now[0] += 1_000_000; depth.start()
    original = source.color_pub.publish
    def publish(message):
        original(message); source.stop_event.set()
    source.color_pub.publish = publish
    source._work()
    assert not any(t == '/test/camera/head/depth' for t, _ in source.events)


@pytest.mark.parametrize('kind', ['color', 'depth'])
def test_image_info_reports_unavailable_until_real_calibration_and_capture(source, kind):
    card = m.RgbdInputCard(source, kind)
    card.running = True
    source.geometry = None
    source.registered = False
    source.count = 0
    status = card.dispatch('info', {})
    assert status['state'] == 'unavailable'
    assert status['camera_info'] == []
    assert status['reason']


def test_start_waits_through_transient_parameter_error_for_real_metadata(source):
    import threading, time
    source.managed = True
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=STAMP-1))
    geometry = source.geometry
    source.geometry = None
    source.registered = False
    source.count = 0
    source.error = 'temporary parameter service failure'
    depth = m.RgbdInputCard(source, 'depth')
    def ready():
        time.sleep(.02)
        with source.lock:
            source.geometry = geometry
            source.registered = True
            source.count = 1
            source.error = None
    worker = threading.Thread(target=ready)
    worker.start()
    try:
        status = depth.dispatch('start', {})
        assert status['state'] == 'running'
        assert status['camera_info'][0]['topic'] == '/test/camera/head/depth'
    finally:
        worker.join()


def test_full_source_restart_cannot_use_previous_capture_count_as_ready(source):
    source.managed = True
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=STAMP-1))
    geometry, work = source.geometry, source._work
    source.count = 17
    source.stop()
    source._work = lambda: None
    depth = m.RgbdInputCard(source, 'depth')
    depth.start(); source.worker.join()
    source._work = work
    source.geometry, source.registered = geometry, True
    assert depth.dispatch('info', {})['state'] == 'unavailable'
    emit(source)
    assert depth.dispatch('info', {})['state'] == 'running'


def test_no_image_sidecar_is_emitted_when_only_imu_card_is_active(source, monkeypatch):
    import queue
    source.managed = True
    source.sub_node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=STAMP-1))
    source.acquire('imu')
    original = source.pending.get
    def next_pair(**kwargs):
        if source.pending.empty():
            source.stop_event.set()
            raise queue.Empty
        return original(**kwargs)
    monkeypatch.setattr(source.pending, 'get', next_pair)
    emit(source)
    assert source.events == []


def test_imu_display_stop_waits_for_inflight_raw_publish(monkeypatch):
    import threading
    entered, finish, stopped = (threading.Event() for _ in range(3))
    published = []
    def publish(msg):
        entered.set()
        assert finish.wait(2)
        published.append(msg)
    display, _ = imu_display(monkeypatch, publish)
    display._running = True
    msg = NS(header=NS(stamp=NS(sec=1, nanosec=0)), linear_acceleration=NS(x=1., y=2., z=3.))
    callback = threading.Thread(target=display._on_camera_accel, args=(msg,))
    def stop():
        display.stop(); stopped.set()
    closer = threading.Thread(target=stop)
    callback.start()
    try:
        assert entered.wait(1)
        closer.start()
        assert not stopped.wait(.05)
    finally:
        finish.set(); callback.join(1)
        if closer.ident is not None: closer.join(1)
    assert stopped.is_set()
    before = len(published)
    display._on_camera_accel(msg)
    assert len(published) == before
