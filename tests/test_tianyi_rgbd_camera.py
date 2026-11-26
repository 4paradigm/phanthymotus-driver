import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

PATH=Path(__file__).resolve().parents[1]/'x-humanoid/tianyi2.0/rgbd_camera.py'
SPEC=importlib.util.spec_from_file_location('tianyi_rgbd',PATH)
m=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(m)


def info(focal=400, distortion=None, frame='optical', width=640, height=480):
    return NS(width=width,height=height,k=[focal,0,width/2,0,focal,height/2,0,0,1],
              d=distortion or [0.]*5,distortion_model='plumb_bob',header=NS(frame_id=frame))


def test_registered_lenses_with_different_intrinsics_are_reprojected_not_relabeled():
    result=m.make_geometry(info(800),info(400),registered=True)
    assert 800 <= result['K'][0,0] <= 820
    ray=80/result['K'][0,0]
    assert result['color_x'][240,400]==pytest.approx(320+800*ray)
    assert result['color_y'][240,400]==pytest.approx(240.)
    assert result['depth_x'][240,400]==pytest.approx(320+400*ray)
    assert result['K'][0,0]==result['K'][1,1]


def test_color_distortion_is_applied_when_sampling_rectified_rgb():
    result=m.make_geometry(info(400,[.1,0,0,0,0]),info(400),registered=True)
    x=80/result['K'][0,0]
    assert result['color_x'][240,400]==pytest.approx(400*x*(1+.1*x*x)+320,abs=1e-4)
    assert result['color_x'][240,400] > result['depth_x'][240,400]


def test_crop_scales_intrinsics_using_actual_depth_optical_geometry():
    color=info(689.25,width=1280,height=720)
    depth=info(619.2,width=1280,height=720)
    result=m.make_geometry(color,depth,registered=True)
    # The narrower RGB vertical field limits the common 4:3 viewport.
    assert 689.25*480/720 <= result['K'][0,0] <= 689.25*480/720*1.02
    assert result['color_x'].shape==(480,640)
    center = result['K'][0,2]
    assert np.interp(center, np.arange(640), result['depth_x'][240]) == pytest.approx(640.)


def test_output_pixel_centres_use_the_cropped_depth_calibration():
    color = info(400, width=1280, height=720)
    depth = info(750, width=1280, height=720)
    depth.k[2] = 638.
    result = m.make_geometry(color, depth, registered=True)
    assert result['K'][0, 2] == pytest.approx(318.5)
    assert result['depth_x'][0, 0] == pytest.approx(160.25)
    assert result['depth_y'][0, 0] == pytest.approx(.25)


@pytest.mark.parametrize('change',[
    lambda c,d:setattr(d.header,'frame_id','depth_lens'),
    lambda c,d:setattr(d,'d',[.1,0,0,0,0]),
    lambda c,d:setattr(c,'k',[0.]*9),
])
def test_source_does_not_invent_registration_or_calibration(change):
    c,d=info(),info();change(c,d)
    with pytest.raises(ValueError):m.make_geometry(c,d,registered=True)


def test_registration_must_be_confirmed_by_source_driver():
    with pytest.raises(ValueError):m.make_geometry(info(),info(),registered=False)


def test_output_preserves_optical_frame_and_declares_matching_intrinsics():
    geometry=m.make_geometry(info(800),info(),registered=True)
    declarations=m.camera_declarations('/test',geometry,'capture-clock')
    assert declarations[0]['id']==declarations[1]['id']
    assert declarations[0]['K']==declarations[1]['K']
    assert declarations[0]['vendor']['frame_id']=='optical'
    assert declarations[1]['vendor']['depth_unit']=='mm'
    assert declarations[1]['vendor']['depth_source']=='measured'


def test_healthy_source_info_is_not_flagged_as_an_mcp_error():
    import threading
    source=object.__new__(m.RgbdCameraPlugin)
    source.lock=threading.RLock();source.imu_lock=threading.RLock();source.root='/test';source.geometry=None
    source.running=True;source.registered=True;source.count=2;source.error=None
    assert 'error' not in source.dispatch('info',{})


def test_first_source_start_waits_for_calibration_before_core_snapshots_info():
    import threading,time
    source=object.__new__(m.RgbdCameraPlugin)
    source.lock=threading.RLock();source.imu_lock=threading.RLock();source.root='/test';source.clock='capture-clock'
    source.running=True;source.registered=False;source.count=0;source.error=None;source.geometry=None
    source.start=lambda:None
    geometry=m.make_geometry(info(),info(),registered=True)
    def ready():
        time.sleep(.02)
        with source.lock:source.geometry=geometry;source.registered=True;source.count=1
    thread=threading.Thread(target=ready);thread.start()
    try:
        result=source.dispatch('start',{})
        assert result['camera_info'] and result['frames_forwarded']==1
    finally:thread.join()


def test_recovered_vendor_parameters_clear_obsolete_error():
    import threading
    source=object.__new__(m.RgbdCameraPlugin)
    source.lock=threading.RLock();source.running=True;source.registered=False;source.error='old parameter failure'
    source.future=NS(result=lambda:NS(values=[NS(bool_value=True),NS(string_value='global'),NS(bool_value=True)]))
    source._parameter_response(source.future)
    assert source.registered and source.error is None


def test_frame_sampling_uses_capture_intervals_despite_callback_jitter(monkeypatch):
    from collections import deque
    import queue, threading
    source = object.__new__(m.RgbdCameraPlugin)
    source.managed = False
    source.lock = threading.RLock(); source.running = source.registered = True
    source.infos = {'color': info(), 'depth': info()}
    source.signature = source.geometry = None
    source.frames = {'color': deque(maxlen=3), 'depth': deque(maxlen=3)}
    source.pending = queue.Queue(maxsize=1); source.last_emit = 0.
    source.image_streams = {kind: dict(received=0, last_capture_ns=None, last_arrival_ns=None)
                            for kind in ('color', 'depth')}
    source.frame_interval_ns = 100_000_000
    source.stop_event = threading.Event(); source.error = None
    monkeypatch.setattr(m.time, 'monotonic', lambda: 1000.)
    monkeypatch.setattr(m.time, 'time_ns', lambda: 1_200_000_000)
    sampled = []
    for stamp in [1_000_000_000, 1_050_000_000, 1_100_000_000, 1_090_000_000,
                  1_100_000_000, 1_150_000_000, 1_200_000_000]:
        msg = NS(width=640, height=480, header=NS(frame_id='optical',
                 stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9)))
        source._image('color', msg); source._image('depth', msg)
        if not source.pending.empty(): sampled.append(m._stamp(source.pending.get_nowait()[1]))
    assert sampled == [1_000_000_000, 1_100_000_000, 1_200_000_000]


@pytest.mark.parametrize('age,accepted', [
    (-50_000_001, False), (-50_000_000, True), (500_000_000, True),
    (500_000_001, False), (2_000_000_000, False),
])
@pytest.mark.parametrize('kind', ['color', 'depth'])
def test_image_ingress_rejects_invalid_capture_age_before_pairing(monkeypatch, age, accepted, kind):
    import queue, threading
    from collections import deque
    source = m.RgbdCameraPlugin.__new__(m.RgbdCameraPlugin)
    source.lock = threading.RLock(); source.running = source.registered = True
    source.managed = False; source.infos = {'color': info(), 'depth': info()}
    source.geometry = None; source.signature = None
    source.frames = {'color': deque(maxlen=3), 'depth': deque(maxlen=3)}
    source.pending = queue.Queue(maxsize=1); source.last_emit = 0
    source.image_streams = {stream: dict(received=0, last_capture_ns=None, last_arrival_ns=None)
                            for stream in ('color', 'depth')}
    source.frame_interval_ns = 100_000_000
    now = 2_000_000_000
    monkeypatch.setattr(m.time, 'time_ns', lambda: now)
    stamp = now-age
    msg = NS(width=640, height=480, header=NS(frame_id='optical',
             stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9)))
    source._image(kind, msg)
    assert bool(source.frames[kind]) is accepted
    assert source.image_streams[kind]['received'] == 1


@pytest.mark.parametrize(("ages_ms", "clock_steps_ms", "first_color", "future_frames", "restart"), [
    ((704, 705), (-1792,), False, 20, True),
    ((704, 705), (-1792,), True, 20, True),
    ((-710, -710), (1792,), False, 20, True),
    ((704, 705), (None, -1792), False, 20, True),
    ((704, 705), (0,), False, 20, False),
    ((704, 425), (-1792,), False, 20, False),
    ((-114, -108), (0,), True, 20, True),
    ((-101, -100), (None,), False, 20, True),
    ((-101, -100), (0,), False, 14, False),
    ((-101, -40), (0,), False, 20, False),
])
def test_startup_recovers_shared_rgbd_clock_faults(
        monkeypatch, ages_ms, clock_steps_ms, first_color, future_frames, restart):
    import queue
    import sys
    import threading
    from collections import deque

    source = m.RgbdCameraPlugin.__new__(m.RgbdCameraPlugin)
    source.lock = threading.RLock()
    source.imu_lock = threading.RLock()
    source.stop_event = threading.Event()
    source.running = source.registered = True
    source.managed = False
    source.infos = {'color': info(), 'depth': info()}
    source.geometry = source.signature = None
    source.frames = {kind: deque(maxlen=3) for kind in source.infos}
    source.pending = queue.Queue(maxsize=1)
    source.last_emit = 0
    source.frame_interval_ns = 100_000_000
    source.image_streams = {kind: dict(received=0) for kind in source.infos}
    source.imu_samples = {}
    source._startup_clock_retry_at = 0.
    source._startup_clock_restart_requested = False
    source._startup_clock_restarting = False
    source.count = 0
    source.imu_publication_attempts = 0
    source.body_mount = NS(retry_pending=lambda: None)
    restarts = []
    steps = iter(clock_steps_ms)

    def measure_step():
        ms = next(steps, clock_steps_ms[-1])
        return None if ms is None else ms * 1_000_000

    monkeypatch.setitem(sys.modules, 'device', NS(CameraPlugin=NS(
        _orbbec_host_clock_step_ns=measure_step,
        _restart_orbbec_service=lambda **kwargs: restarts.append(kwargs))))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(CompressedImage=object))
    now = 2_000_000_000

    def send(kind, stamp):
        msg = NS(width=640, height=480, header=NS(frame_id='optical',
                 stamp=NS(sec=stamp // 10**9, nanosec=stamp % 10**9)))
        source._image(kind, msg)

    monkeypatch.setattr(m.time, 'time_ns', lambda: now)
    if first_color:
        send('color', now - 30_000_000)
        assert source.geometry is not None
        # By the time both streams are stale, the lone pre-step frame expired.
        now += 600_000_000
    for frame in range(20):
        for kind, age in zip(('color', 'depth'), ages_ms):
            if age < -50 and frame == 20 - future_frames - 1:
                # Break any future-frame streak without building a fresh pair.
                age = 520
            send(kind, now - age * 1_000_000)
        now += 33_333_333

    polls = iter((2., 4., 6., 8.))
    monotonic = 0.
    monkeypatch.setattr(m.time, 'monotonic', lambda: monotonic)
    get = source.pending.get

    def wait_for_pair(block=True, timeout=None):
        nonlocal monotonic
        if not block:
            return get(block=False)
        monotonic = next(polls)
        if monotonic == 8.:
            source.stop_event.set()
        raise queue.Empty

    with monkeypatch.context() as worker:
        worker.setattr(source.pending, 'get', wait_for_pair)
        source._work()
    assert source.pending.empty()
    assert len(restarts) == int(restart)

    now += 100_000_000
    capture_ns = now - 30_000_000
    if restart:
        assert not source.registered and not source.infos
        send('color', capture_ns)
        assert not source.frames['color']
        source.registered = True
        for kind in ('color', 'depth'):
            source._info(kind, info())
    for kind in ('color', 'depth'):
        send(kind, capture_ns)
    assert tuple(map(m._stamp, source.pending.get_nowait())) == (capture_ns, capture_ns)


@pytest.mark.parametrize('focal,distortion,offset', [
    (520., [0.]*5, (0., 0.)),
    (800., [.1, -.02, .001, -.001, 0.], (7., -5.)),
    (340., [-.08, .01, 0., 0., 0.], (-9., 4.)),
])
def test_common_field_has_no_remap_black_padding_and_preserves_ray_calibration(focal, distortion, offset):
    import cv2
    color, depth = info(focal, distortion), info(400.)
    color.k[2] += offset[0]; color.k[5] += offset[1]
    g = m.make_geometry(color, depth, registered=True)
    image = np.full((480, 640, 3), 200, np.uint8)
    rgb = cv2.remap(image, g['color_x'], g['color_y'], cv2.INTER_LINEAR)
    assert rgb.min() >= 199  # every output ray has real color, including its border
    measured = cv2.remap(np.full((480,640), 1250, np.uint16),
                         g['depth_x'], g['depth_y'], cv2.INTER_NEAREST)
    assert np.all(measured == 1250)
    u, v = np.meshgrid(np.arange(640), np.arange(480))
    x, y = (u-g['K'][0,2])/g['K'][0,0], (v-g['K'][1,2])/g['K'][1,1]
    np.testing.assert_allclose(g['depth_x'], depth.k[0]*x+depth.k[2], atol=1e-4)
    np.testing.assert_allclose(g['depth_y'], depth.k[4]*y+depth.k[5], atol=1e-4)
    declarations = m.camera_declarations('/test', g, 'capture')
    assert declarations[0]['K'] == declarations[1]['K'] == g['K'].ravel().tolist()


def test_common_field_does_not_claim_disjoint_lenses_are_registered():
    color, depth = info(520.), info(400.)
    color.k[2] = 2000.
    with pytest.raises(ValueError, match='common'):
        m.make_geometry(color, depth, registered=True)
