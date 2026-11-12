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
    assert result['K'][0,2]==pytest.approx(320.)
    assert result['color_x'].shape==(480,640)
    assert result['depth_x'][240,320]==pytest.approx(640.)


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


def test_recovered_vendor_parameters_clear_obsolete_error(monkeypatch):
    import sys,threading
    monkeypatch.setitem(sys.modules,'rcl_interfaces.srv',NS(GetParameters=object))
    source=object.__new__(m.RgbdCameraPlugin)
    source.lock=threading.RLock();source.running=True;source.registered=False;source.error='old parameter failure'
    source.future=NS(done=lambda:True,result=lambda:NS(values=[NS(bool_value=True),NS(string_value='global'),NS(bool_value=True)]))
    source._parameters()
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
    source.frame_interval_ns = 100_000_000
    source.stop_event = threading.Event(); source.error = None
    monkeypatch.setattr(m.time, 'monotonic', lambda: 1000.)
    sampled = []
    for stamp in [1_000_000_000, 1_050_000_000, 1_100_000_000, 1_200_000_000]:
        msg = NS(width=640, height=480, header=NS(frame_id='optical',
                 stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9)))
        source._image('color', msg); source._image('depth', msg)
        if not source.pending.empty(): sampled.append(m._stamp(source.pending.get_nowait()[1]))
    assert sampled == [1_000_000_000, 1_100_000_000, 1_200_000_000]


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
