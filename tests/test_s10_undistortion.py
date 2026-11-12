import importlib.util
import json
import subprocess
from pathlib import Path

import numpy as np
import pytest

from deep_robotics.s10_pro import undistortion as u


@pytest.fixture
def controller(tmp_path):
    return u.CameraUndistortion(
        ('camera_front', 'camera_rear', 'camera_left', 'camera_right'),
        config_path=tmp_path / 'settings.json', map_dir=tmp_path / 'maps')


@pytest.mark.parametrize('ident', sorted(u.CALIBRATIONS))
def test_map_matches_opencv(ident):
    cv2 = pytest.importorskip('cv2')
    cal = u.validate(u.CALIBRATIONS[ident])
    x, y, invalid = u.compute_maps(cal)
    k = np.array(u.source_matrix(cal)).reshape(3, 3)
    expected_x, expected_y = cv2.initUndistortRectifyMap(
        k, np.array(cal['D']), None, k, tuple(cal['source_resolution']), cv2.CV_32FC1)
    assert invalid == 0
    assert np.max(abs(x.astype(float) - expected_x)) <= 0.501
    assert np.max(abs(y.astype(float) - expected_y)) <= 0.501


def test_per_lane_settings_persist_and_rejected_settings_change_nothing(controller):
    changed = []
    controller.on_change = changed.append
    assert controller.active('camera_front')
    controller.configure('camera_front', {'undistort': False})
    assert changed == ['camera_front']
    assert not controller.active('camera_front')
    assert controller.active('camera_rear')
    loaded = u.CameraUndistortion(controller.lanes, config_path=controller.config_path)
    assert not loaded.active('camera_front')
    before = controller.config_path.read_bytes()
    generation = controller.generation('camera_rear')
    for bad in ({'calibration': {'K': [1]}}, {'undistort': 'yes'}, {'unknown': 1}):
        with pytest.raises(ValueError):
            controller.configure('camera_rear', bad)
    assert controller.config_path.read_bytes() == before
    assert controller.generation('camera_rear') == generation
    assert changed == ['camera_front']


def test_saved_calibration_from_an_earlier_build_is_ignored(tmp_path):
    path = tmp_path / 'settings.json'
    path.write_text(json.dumps({'version': 1, 'cameras': {
        'camera_front': {'undistort': True, 'calibration': {'K': [1]}}}}))
    loaded = u.CameraUndistortion(('camera_front',), config_path=path)
    assert loaded.settings('camera_front') == {'undistort': True}
    assert loaded.calibration('camera_front')[1] == 'table:' + u.camera_id('camera_front')


@pytest.mark.parametrize('model,coefficients', [
    ('plumb_bob', [0., 0., 0., 0.]),
    ('rational_polynomial', [-.2, .03, 0., 0., 0., .01, 0., 0.]),
    ('equidistant', [.01, -.001, 0., 0.]),
])
def test_every_supported_model_matches_opencv(model, coefficients):
    """The table only holds plumb_bob rows today; another lens may need the others."""
    cv2 = pytest.importorskip('cv2')
    cal = dict(u.CALIBRATIONS['deeprobotics/s10_pro/camera_front'],
               distortion_model=model, D=coefficients)
    x, y, _ = u.compute_maps(cal)
    k = np.asarray(u.source_matrix(cal)).reshape(3, 3)
    init = cv2.fisheye.initUndistortRectifyMap if model == 'equidistant' else cv2.initUndistortRectifyMap
    ex, ey = init(k, np.array(coefficients), np.eye(3), k,
                  tuple(cal['source_resolution']), cv2.CV_32FC1)
    valid = (x != u.INVALID) & (y != u.INVALID)
    assert np.max(abs(x[valid].astype(float) - ex[valid])) <= .501
    assert np.max(abs(y[valid].astype(float) - ey[valid])) <= .501


def test_declaration_tracks_the_switch(controller):
    from common.camera_info import build
    lane = 'camera_front'
    declared = controller.declaration(lane)
    declared.pop('undistortion')
    info = build(topic='/front', id=u.camera_id(lane), width=640, height=360, **declared)
    assert info['distortion_model'] == 'pinhole'
    assert info['D'] == [0.] * 5
    assert info['half_fov_rad'] == pytest.approx(0.7926, abs=.001)
    assert u.PIPELINE_STAGE in info['pipeline']
    assert controller.status(lane)['calibration_source'] == 'table:' + u.camera_id(lane)
    controller.configure(lane, {'undistort': False})
    assert 'K' not in controller.declaration(lane)
    assert controller.declaration(lane)['undistortion']['state'] == 'raw'
    raw = controller.declaration(lane)
    raw.pop('undistortion')
    info = build(topic='/front', id=u.camera_id(lane), width=640, height=360, **raw)
    # A distorted frame has no valid pinhole angle; declaring K's would be
    # ~10 degrees short at the edges, so the declaration says "unknown".
    assert info['half_fov_rad'] is None and info['K'] is None
    assert info['source'] == 'unknown'


def _camera_card(controller, monkeypatch):
    import importlib.util
    from types import SimpleNamespace
    driver = Path(__file__).parents[1] / 'deep_robotics' / 's10_pro'
    monkeypatch.syspath_prepend(str(driver))
    spec = importlib.util.spec_from_file_location('camera_card_device', driver / 'device.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes = SimpleNamespace(camera_undistortion=controller, topics={
        lane: {'topic': '/' + lane, 'format': 'image/jpeg'} for lane in controller.lanes})
    return module.S10StatePlugin(SimpleNamespace(start=lambda: None), nodes)


def test_camera_card_config_and_info_use_same_geometry(controller, monkeypatch):
    plugin = _camera_card(controller, monkeypatch)
    before = {t['name']: t for t in plugin.get_tools()}
    assert before['camera_front']['configSchema']['properties']['undistort']['default'] is True
    result = plugin.dispatch('config', {'_tool_name': 'camera_front', 'undistort': False})
    assert result['state'] == 'configured'
    assert result['camera_info'][0]['K'] is None
    assert plugin.dispatch('info', {'_tool_name': 'camera_rear'})['camera_info'][0]['K'] is not None
    result = plugin.dispatch('config', {'_tool_name': 'camera_front',
                                        'config': {'undistort': True}})
    assert result['camera_info'][0]['K'][0] == pytest.approx(314.963248)
    assert plugin.dispatch('info', {'_tool_name': 'camera_front'})['camera_info'] == result['camera_info']


@pytest.mark.parametrize('lane', ['camera_front', 'camera_rear', 'camera_left', 'camera_right'])
def test_camera_card_schema_declares_exactly_what_the_card_accepts(controller, monkeypatch, lane):
    """The advertised surface and the implementation must not drift: every
    parameter the schemas offer is accepted, and nothing the schemas do not
    offer (a per-robot calibration, say) is quietly accepted."""
    plugin = _camera_card(controller, monkeypatch)
    card = {t['name']: t for t in plugin.get_tools()}[lane]
    declared = card['inputSchema']['x-action-params']['config']['params']
    assert declared == ['undistort']
    assert set(card['inputSchema']['properties']) - {'action'} == set(declared)
    assert card['inputSchema']['properties']['undistort']['type'] == 'boolean'
    assert set(card['configSchema']['properties']) == set(declared)
    assert card['configSchema']['properties']['undistort']['type'] == 'boolean'
    for value in (False, True):
        result = plugin.dispatch('config', {'_tool_name': lane, 'undistort': value})
        assert result['state'] == 'configured'
        assert result['config'] == {'undistort': value}
    for hidden in ({'calibration': {'K': [1]}}, {'config': {'calibration': {}}}):
        result = plugin.dispatch('config', {'_tool_name': lane, **hidden})
        assert result['state'] == 'error' and 'undistort' in result['message']
        assert plugin.dispatch('info', {'_tool_name': lane})['config'] == {'undistort': True}


def test_only_changed_lane_restarts_and_old_generation_frames_are_dropped(controller):
    import io
    from deep_robotics.s10_pro.media_workers import MediaWorkers
    class Process:
        terminated = False
        def poll(self): return None
        def terminate(self): self.terminated = True
    worker = MediaWorkers(lambda *args: None)
    front, rear = Process(), Process()
    worker._rtsp_processes.update(camera_front=front, camera_rear=rear)
    worker.camera_undistortion = controller
    controller.on_change = worker.restart_rtsp_camera
    old = controller.generation('camera_front')
    controller.configure('camera_front', {'undistort': False})
    assert front.terminated and not rear.terminated
    published = []
    worker._on_result = lambda *a: published.append(a)
    data = b'\xff\xd8frame\xff\xd9'
    worker._read_mjpeg('camera_front', io.BytesIO(data), old)
    assert not published
    worker._read_mjpeg('camera_front', io.BytesIO(data), controller.generation('camera_front'))
    assert published


def test_ffmpeg_remap_reuses_static_maps_for_multiple_frames(controller, tmp_path):
    """Real filter execution, not a command-string assertion. Compare output
    pixels against the same integer remap evaluated with OpenCV."""
    cv2 = pytest.importorskip('cv2')
    import shutil
    exe = shutil.which('ffmpeg')
    if not exe and importlib.util.find_spec('imageio_ffmpeg'):
        import imageio_ffmpeg
        exe = imageio_ffmpeg.get_ffmpeg_exe()
    if not exe:
        pytest.skip('ffmpeg not installed')
    cal, _ = controller.calibration('camera_front')
    xp, yp, _ = u.write_maps(cal, controller.map_dir)
    w, h = cal['source_resolution']
    yy, xx = np.indices((h, w))
    frame = np.stack([xx % 256, yy % 256, (xx + yy) % 256], axis=2).astype('uint8')
    image = tmp_path / 'input.png'
    cv2.imwrite(str(image), frame)
    command = [exe, '-hide_banner', '-loglevel', 'error', '-loop', '1', '-framerate', '15',
               '-i', str(image), '-i', xp, '-i', yp, '-filter_complex',
               '[0:v]format=gbrp[src];[src][1:v][2:v]remap,scale=640:360[out]',
               '-map', '[out]', '-frames:v', '4', '-pix_fmt', 'bgr24', '-f', 'rawvideo', 'pipe:1']
    result = subprocess.run(command, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr.decode(errors='replace')
    assert len(result.stdout) == 4 * 640 * 360 * 3
    frames = np.frombuffer(result.stdout, np.uint8).reshape(4, 360, 640, 3)
    np.testing.assert_array_equal(frames[0], frames[-1])
    x, y, _ = u.compute_maps(cal)
    reference = cv2.remap(frame, x.astype('float32'), y.astype('float32'), cv2.INTER_NEAREST)
    reference = cv2.resize(reference, (640, 360), interpolation=cv2.INTER_LINEAR)
    assert np.mean(abs(frames[0].astype(float) - reference)) < 3
