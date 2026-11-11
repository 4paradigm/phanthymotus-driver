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
    assert result['color_x'][240,400]==pytest.approx(480.)
    assert result['color_y'][240,400]==pytest.approx(240.)
    assert result['depth_x'][240,400]==pytest.approx(400.)
    np.testing.assert_allclose(result['K'],[[400,0,320],[0,400,240],[0,0,1]])


def test_color_distortion_is_applied_when_sampling_rectified_rgb():
    result=m.make_geometry(info(400,[.1,0,0,0,0]),info(400),registered=True)
    # x=.2, r²=.04 -> distorted pixel 400*(.2*1.004)+320.
    assert result['color_x'][240,400]==pytest.approx(400.32,abs=1e-4)


def test_crop_scales_intrinsics_using_actual_depth_optical_geometry():
    color=info(689.25,width=1280,height=720)
    depth=info(619.2,width=1280,height=720)
    result=m.make_geometry(color,depth,registered=True)
    assert result['K'][0,0]==pytest.approx(412.8)
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
    source.lock=threading.RLock();source.root='/test';source.geometry=None
    source.running=True;source.registered=True;source.count=2;source.error=None
    assert 'error' not in source.dispatch('info',{})


def test_first_source_start_waits_for_calibration_before_core_snapshots_info():
    import threading,time
    source=object.__new__(m.RgbdCameraPlugin)
    source.lock=threading.RLock();source.root='/test';source.clock='capture-clock'
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
