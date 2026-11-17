"""Front LiDAR depth card contracts and lifecycle without ROS or a robot."""
from __future__ import annotations

import importlib
import json
import os
import signal
import struct
import sys
import threading
import time
import zlib
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "deep_robotics" / "s10_pro"
IDENTITY = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def module():
    return importlib.import_module("deep_robotics.s10_pro.front_depth")


class FakeWorker:
    def __init__(self, on_result, on_error):
        self.on_result, self.on_error = on_result, on_error
        self.jobs = []
        self.alive = False

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive

    def submit(self, job):
        self.jobs.append(job)
        return True

    def complete(self, index=-1):
        from deep_robotics.s10_pro.depth_projection import project_depth, fill_depth_holes
        job = self.jobs[index]
        packet, stats = project_depth(job["cloud"], job["K"], job["transform"],
                                      min_depth_m=job["min_depth_m"],
                                      max_depth_m=job["max_depth_m"])
        packet, fill_stats = fill_depth_holes(packet)
        stats.update(fill_stats)
        self.on_result(job, packet, stats)

    def close(self):
        self.alive = False


@pytest.fixture
def rig(tmp_path, monkeypatch):
    depth = module()
    from deep_robotics.s10_pro.undistortion import CameraUndistortion
    controller = CameraUndistortion(("camera_front",), config_path=tmp_path / "rgb.json",
                                   map_dir=tmp_path / "maps")
    publications, subscriptions, destroyed, publishers = [], [], [], []
    timers, destroyed_timers, publisher_infos = [], [], []

    class CompressedImage:
        def __init__(self):
            self.header = SimpleNamespace(frame_id="", stamp=SimpleNamespace(sec=0, nanosec=0))
            self.format, self.data = "", b""

    sensor = ModuleType("sensor_msgs.msg")
    sensor.CompressedImage = CompressedImage
    sensor.PointCloud2 = type("PointCloud2", (), {})
    groups = ModuleType("rclpy.callback_groups")
    groups.MutuallyExclusiveCallbackGroup = type("MutuallyExclusiveCallbackGroup", (), {})
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor)
    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", groups)
    monkeypatch.setattr(depth, "DepthWorker", FakeWorker)
    relays = []
    class FakeRelay:
        def __init__(self, config, topic, rate_hz, on_message, on_error):
            self.topic, self.on_message, self.on_error = topic, on_message, on_error
            self.alive = False
            relays.append(self)
        def start(self):
            self.alive = True
        def is_alive(self):
            return self.alive
        def close(self):
            self.alive = False
        def status(self):
            return {'state': 'streaming' if self.alive else 'stopped'}
    monkeypatch.setattr(depth, "CloudRelay", FakeRelay, raising=False)

    def subscribe(message_type, topic, callback, qos, **kwargs):
        handle = SimpleNamespace(topic=topic, callback=callback, qos=qos)
        subscriptions.append(handle)
        return handle

    publisher = SimpleNamespace(publish=publications.append)
    def create_publisher(lane, topic, message_type):
        publishers.append((lane, topic, message_type))
        return publisher
    def create_timer(period, callback, **kwargs):
        timer = SimpleNamespace(period=period, callback=callback)
        timers.append(timer)
        return timer

    nodes = SimpleNamespace(
        robot=SimpleNamespace(create_subscription=subscribe, destroy_subscription=destroyed.append,
                              create_timer=create_timer, destroy_timer=destroyed_timers.append,
                              get_publishers_info_by_topic=lambda topic: list(publisher_infos)),
        bridge=SimpleNamespace(create_publisher=create_publisher),
        camera_undistortion=controller, _sensor_qos=object(),
        front_camera_frame=lambda: (controller.generation('camera_front'), time.monotonic(), b'front-jpeg'))
    extrinsics_path = tmp_path / "extrinsics.json"
    monkeypatch.setenv("S10_FRONT_CAMERA_EXTRINSICS", str(extrinsics_path))
    plugin = depth.FrontDepthPlugin({}, "robot", nodes, config_path=tmp_path / "depth.json")
    return SimpleNamespace(plugin=plugin, nodes=nodes, controller=controller,
                           publications=publications, subscriptions=subscriptions,
                           destroyed=destroyed, path=tmp_path / "depth.json",
                           extrinsics_path=extrinsics_path, timers=timers,
                           destroyed_timers=destroyed_timers, publisher_infos=publisher_infos,
                           relays=relays, publishers=publishers)


def deployment_calibration(rig, **changes):
    # Synthetic transform: cloud +X is optical +Z; never a hardware default.
    record = {
        "version": 1, "camera_id": "deeprobotics/s10_pro/camera_front",
        "front_camera_stream": "rtsp://10.21.33.103:8554/video4",
        "cloud_topic": "/LIDAR/POINTS", "cloud_frame_id": "lidar_link",
        "camera_optical_frame_id": "measured_front_optical",
        "translation_unit": "m", "camera_axes": "x_right_y_down_z_forward",
        "camera_optical_from_cloud":
            [[0, 0, -1, 1], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]],
        **changes,
    }
    rig.extrinsics_path.write_text(json.dumps(record))
    return record


def test_canvas_has_no_topic_frame_or_matrix_settings(rig):
    properties = rig.plugin.get_tool()["configSchema"]["properties"]
    assert set(properties) == {"min_depth_m", "max_depth_m", "rate_hz"}


def test_unavailable_packaged_intrinsics_surfaces_the_resource_reason(rig, monkeypatch):
    configure(rig)
    reason = 'cannot load camera intrinsics /driver/resource/calibration/s10_pro_camera_intrinsics.json'
    monkeypatch.setattr(rig.controller, 'calibration', lambda lane: (None, reason))
    result = rig.plugin.dispatch('start', {})
    assert result['code'] == 'CALIBRATION_INVALID'
    assert reason in result['message']
    assert not rig.subscriptions


@pytest.mark.parametrize('frame', [None, (0, 0, b'old-image'), (999, None, b'wrong-generation')])
def test_standalone_card_rejects_missing_stale_or_wrong_generation_rgb(rig, frame):
    configure(rig)
    rig.plugin.dispatch('start', {})
    rig.nodes.front_camera_frame = lambda: frame
    rig.subscriptions[0].callback(cloud())
    assert not rig.plugin._worker.jobs
    assert rig.plugin.dispatch('info', {})['error_code'] == 'FRONT_RGB_UNAVAILABLE'


def test_standalone_card_supplies_front_rgb_without_another_card(rig):
    configure(rig)
    rig.plugin.dispatch('start', {})
    rig.subscriptions[0].callback(cloud())
    assert rig.plugin._worker.jobs[0]['jpeg'] == b'front-jpeg'
    assert rig.plugin._worker.jobs[0]['rgb_received_monotonic'] > 0
    assert not rig.plugin.get_tool().get('topic_in')


def test_missing_calibration_is_explicit_and_persists_in_status(rig):
    result = rig.plugin.dispatch("start", {})
    assert result["state"] == "error"
    assert result["code"] == "CALIBRATION_MISSING"
    assert str(rig.extrinsics_path) in result["message"]
    info = rig.plugin.dispatch("info", {})
    assert info["state"] == "error" and info["error_code"] == "CALIBRATION_MISSING"
    assert info["config"]["camera_optical_from_cloud"] is None
    assert not rig.subscriptions and not rig.path.exists()


def test_default_canvas_start_loads_directed_calibration_readonly(rig):
    record = deployment_calibration(rig)
    before = rig.extrinsics_path.read_bytes()
    result = rig.plugin.dispatch("start", {})
    assert result["state"] == "running", result
    handle = rig.subscriptions[-1]
    assert handle.topic == "/LIDAR/POINTS"
    message = cloud(frame="lidar_link")
    message.data = struct.pack("<fff", 2, 1, 1)
    handle.callback(message)
    rig.plugin._worker.complete()
    output = rig.publications[-1]
    assert output.header.frame_id == "measured_front_optical"
    assert (output.header.stamp.sec, output.header.stamp.nanosec) == (10, 123)
    raw = zlib.decompress(output.data)
    assert struct.unpack_from("<H", raw, (453 * 640 + 337) * 2)[0] == 2000
    assert result["config"]["cloud_frame_id"] == "lidar_link"
    metadata = rig.plugin.dispatch("info", {})["camera_info"][0]["vendor"]
    assert metadata["camera_optical_from_cloud"] == record["camera_optical_from_cloud"]
    assert metadata["extrinsics_source"] == "file:" + str(rig.extrinsics_path)
    assert metadata["reprojection_verified"] is False
    assert rig.extrinsics_path.read_bytes() == before
    assert not rig.path.exists()


def test_confirmed_robot_mounting_record_projects_front_right_and_up(rig):
    path = DRIVER / "resource/calibration/s10_pro_standard_front.json"
    plugin = module().FrontDepthPlugin({}, "robot", rig.nodes,
                                      config_path=rig.path, extrinsics_path=path)
    result = plugin.dispatch("start", {})
    assert result["state"] == "running", result
    # Body points: two metres in front of the optical centre, then right/up.
    points = [(2.27065, 0, .04673), (2.27065, -1, .04673), (2.27065, 0, 1.04673)]
    message = cloud(frame="lidar_link")
    message.width = 3
    message.row_step = 36
    message.data = b''.join(struct.pack('<fff', *point) for point in points)
    rig.subscriptions[-1].callback(message)
    plugin._worker.complete()
    depths = struct.unpack('<307200H', zlib.decompress(rig.publications[-1].data))
    assert depths[243 * 640 + 337] == 2000
    assert depths[243 * 640 + 494] == 2000
    assert depths[33 * 640 + 337] == 2000
    assert sum(value != 0 for value in depths) == 3
    assert result['camera_info'][0]['vendor']['reprojection_verified'] is False
    plugin.stop()


def test_confirmed_robot_transform_maps_its_optical_centre_to_zero():
    from deep_robotics.s10_pro.depth_projection import validate_transform
    record = json.loads((DRIVER / 'resource/calibration/s10_pro_standard_front.json').read_text())
    matrix = validate_transform(record['camera_optical_from_cloud'])
    camera_origin = [.27065, 0, .04673, 1]
    transformed = [sum(row[i] * camera_origin[i] for i in range(4)) for row in matrix]
    assert transformed == pytest.approx([0, 0, 0, 1])


def automatic_plugin(rig, monkeypatch, identity):
    monkeypatch.delenv('S10_FRONT_CAMERA_EXTRINSICS')
    monkeypatch.setattr(module(), 'read_robot_identity', lambda config: identity, raising=False)
    plugin = module().FrontDepthPlugin({}, 'robot', rig.nodes, config_path=rig.path)
    # Reproduce a fresh deployment with an empty data volume.
    plugin.extrinsics_path = rig.extrinsics_path
    return plugin


def test_empty_data_volume_automatically_loads_standard_model_calibration(rig, monkeypatch):
    plugin = automatic_plugin(rig, monkeypatch,
        '482fc8b97c9e88f56cad0e892fc3ebfecddc47c570d078588e448d81945b6640')
    result = plugin.dispatch('start', {})
    try:
        assert result['state'] == 'running', result
        assert result['config']['cloud_frame_id'] == 'lidar_link'
        assert result['config']['camera_optical_from_cloud'] == [
            [0, -1, 0, 0], [0, 0, -1, .04673],
            [1, 0, 0, -.27065], [0, 0, 0, 1]]
        assert result['camera_info'][0]['vendor']['extrinsics_source'].startswith('bundled:')
        assert not rig.extrinsics_path.exists() and not rig.path.exists()
    finally:
        plugin.stop()


def test_another_standard_robot_reuses_model_calibration(rig, monkeypatch):
    plugin = automatic_plugin(rig, monkeypatch, '0' * 64)
    try:
        result = plugin.dispatch('start', {})
        assert result['state'] == 'running', result
        assert result['config']['cloud_frame_id'] == 'lidar_link'
        vendor = result['camera_info'][0]['vendor']
        assert vendor['reprojection_verified'] is False
        assert vendor['temporal_synchronization'] is False
    finally:
        plugin.stop()


def test_standard_profile_does_not_require_identity_ssh(rig, monkeypatch):
    plugin = automatic_plugin(rig, monkeypatch, None)
    def unavailable(config):
        raise RuntimeError('identity unavailable')
    monkeypatch.setattr(module(), 'read_robot_identity', unavailable, raising=False)
    try:
        result = plugin.dispatch('start', {})
        assert result['state'] == 'running', result
    finally:
        plugin.stop()


@pytest.mark.parametrize('contents, code', [
    (None, 'CALIBRATION_MISSING'), ('{broken', 'CALIBRATION_INVALID'),
    ('{}', 'CALIBRATION_INVALID')])
def test_missing_or_invalid_standard_profile_is_explicit(rig, monkeypatch, tmp_path, contents, code):
    directory = tmp_path / 'bundled'
    directory.mkdir()
    if contents is not None:
        (directory / 's10_pro_standard_front.json').write_text(contents)
    monkeypatch.setattr(module(), 'BUNDLED_CALIBRATIONS', directory)
    plugin = automatic_plugin(rig, monkeypatch, '0' * 64)
    result = plugin.dispatch('start', {})
    assert result['code'] == code, result
    assert 's10_pro_standard_front.json' in result['message']
    assert not rig.subscriptions and not rig.relays


def test_deployment_override_takes_precedence_over_standard_profile(rig, monkeypatch):
    plugin = automatic_plugin(rig, monkeypatch, '0' * 64)
    record = deployment_calibration(rig)
    try:
        result = plugin.dispatch('start', {})
        assert result['state'] == 'running', result
        assert result['config']['camera_optical_from_cloud'] == record['camera_optical_from_cloud']
        assert result['camera_info'][0]['vendor']['extrinsics_source'] == 'file:' + str(rig.extrinsics_path)
    finally:
        plugin.stop()


def test_explicit_missing_calibration_path_does_not_use_bundled_record(rig, monkeypatch):
    def unexpected(config):
        pytest.fail('an explicit missing calibration must not perform identity discovery')
    monkeypatch.setattr(module(), 'read_robot_identity', unexpected, raising=False)
    assert rig.plugin.dispatch('start', {})['code'] == 'CALIBRATION_MISSING'


def test_present_invalid_deployment_record_does_not_use_bundled_record(rig, monkeypatch):
    plugin = automatic_plugin(rig, monkeypatch,
        '482fc8b97c9e88f56cad0e892fc3ebfecddc47c570d078588e448d81945b6640')
    rig.extrinsics_path.write_text('{}')
    assert plugin.dispatch('start', {})['code'] == 'CALIBRATION_INVALID'
    assert not rig.subscriptions


def test_standard_profile_is_reloaded_after_stop(rig, monkeypatch, tmp_path):
    import shutil
    directory = tmp_path / 'bundled'
    directory.mkdir()
    path = directory / 's10_pro_standard_front.json'
    shutil.copy(DRIVER / 'resource/calibration/s10_pro_standard_front.json', path)
    monkeypatch.setattr(module(), 'BUNDLED_CALIBRATIONS', directory)
    plugin = automatic_plugin(rig, monkeypatch, '0' * 64)
    assert plugin.dispatch('start', {})['state'] == 'running'
    plugin.stop()
    path.write_text('{}')
    assert plugin.dispatch('start', {})['code'] == 'CALIBRATION_INVALID'


def test_image_copied_calibration_can_start_with_an_empty_volume(rig, monkeypatch, tmp_path):
    import shlex
    import shutil
    image_root = tmp_path / 'image'
    # Stage the actual Docker COPY sources for calibration, not a test fixture.
    for line in (DRIVER / 'Dockerfile').read_text().splitlines():
        if not line.startswith('COPY ') or '--from=' in line:
            continue
        *sources, destination = shlex.split(line)[1:]
        if '/resource/calibration' not in destination:
            continue
        target = image_root / destination.lstrip('/')
        for source in sources:
            shutil.copytree(DRIVER / source, target, dirs_exist_ok=True)
    monkeypatch.setattr(module(), 'BUNDLED_CALIBRATIONS',
                        image_root / 'work/deep_robotics/s10_pro/resource/calibration')
    from deep_robotics.s10_pro import undistortion
    calibrations = undistortion._load_calibrations(
        image_root / 'work/deep_robotics/s10_pro/resource/calibration/s10_pro_camera_intrinsics.json')
    assert calibrations['deeprobotics/s10_pro/camera_front']['K'][0] == 314.963248
    plugin = automatic_plugin(rig, monkeypatch,
        '482fc8b97c9e88f56cad0e892fc3ebfecddc47c570d078588e448d81945b6640')
    try:
        result = plugin.dispatch('start', {})
        assert result['state'] == 'running', result
        assert not rig.extrinsics_path.exists()
    finally:
        plugin.stop()


@pytest.mark.parametrize("changes", [
    {"version": 2}, {"version": True}, {"translation_unit": "mm"},
    {"camera_axes": "x_forward_y_left_z_up"},
    {"camera_id": "deeprobotics/s10_pro/camera_rear"},
    {"front_camera_stream": "rtsp://10.21.33.103:8554/video0"},
    {"camera_optical_frame_id": ""}, {"cloud_frame_id": ""},
    {"cloud_frame_id": "map"}, {"cloud_topic": "/ALIGNED_POINTS"},
    {"camera_optical_from_cloud": None},
    {"camera_optical_from_cloud": [[-1, 0, 0, 0], [0, 1, 0, 0],
                                   [0, 0, 1, 0], [0, 0, 0, 1]]},
])
def test_invalid_deployment_record_never_falls_back_to_maintenance_config(rig, changes):
    configure(rig)
    deployment_calibration(rig, **changes)
    result = rig.plugin.dispatch("start", {})
    assert result["state"] == "error", result
    assert result["code"] == "CALIBRATION_INVALID"
    assert not rig.subscriptions and not rig.publications


def test_corrupt_or_incomplete_record_is_not_used(rig):
    for contents in ["{broken", "[]", '{"version": 1}']:
        rig.extrinsics_path.write_text(contents)
        result = rig.plugin.dispatch("start", {})
        assert result["state"] == "error" and result["code"] == "CALIBRATION_INVALID"
    assert not rig.subscriptions


def test_present_deployment_record_owns_topic_and_frame(rig):
    configure(rig)
    deployment_calibration(rig)
    result = rig.plugin.dispatch("start", {})
    assert result["config"]["cloud_topic"] == "/LIDAR/POINTS"
    assert result["config"]["cloud_frame_id"] == "lidar_link"
    rig.subscriptions[-1].callback(cloud(frame="front_lidar"))
    assert not rig.plugin._worker.jobs and not rig.publications


def test_record_is_bound_to_configured_front_stream(rig):
    deployment_calibration(rig)
    plugin = module().FrontDepthPlugin(
        {"camera_rtsp": {"camera_front": "rtsp://another-robot:8554/video4"}},
        "robot", rig.nodes, config_path=rig.path, extrinsics_path=rig.extrinsics_path)
    result = plugin.dispatch("start", {})
    assert result["state"] == "error" and result["code"] == "CALIBRATION_INVALID"
    assert not rig.subscriptions


def test_restart_reloads_record_and_never_reuses_removed_automatic_matrix(rig):
    deployment_calibration(rig)
    assert rig.plugin.dispatch("start", {})["state"] == "running"
    assert rig.plugin.dispatch("start", {})["state"] == "running"
    assert len(rig.subscriptions) == 1
    rig.plugin.dispatch("stop", {})
    deployment_calibration(rig, camera_optical_frame_id="new_optical_frame")
    assert rig.plugin.dispatch("start", {})["camera_info"][0]["vendor"]["frame_id"] == "new_optical_frame"
    rig.plugin.dispatch("stop", {})
    rig.extrinsics_path.unlink()
    result = rig.plugin.dispatch("start", {})
    assert result["state"] == "error" and result["code"] == "CALIBRATION_MISSING"
    assert not rig.publications


@pytest.mark.parametrize("publishers,code,state", [
    (0, "CLOUD_PUBLISHER_UNAVAILABLE", "no_publisher"),
    (1, "CLOUD_DATA_TIMEOUT", "stale"),
])
def test_source_timeout_distinguishes_absent_and_silent_publishers(rig, monkeypatch,
                                                                 publishers, code, state):
    configure(rig)
    clock = [100.]
    monkeypatch.setattr(module().time, "monotonic", lambda: clock[0])
    rig.publisher_infos.extend([SimpleNamespace()] * publishers)
    result = rig.plugin.dispatch("start", {})
    assert result["input"]["state"] == "waiting"
    assert result["input"]["observed_frame_id"] is None
    assert len(rig.timers) == 1
    clock[0] = 104.
    rig.timers[0].callback()
    info = rig.plugin.dispatch("info", {})
    assert info["state"] == "error" and info["error_code"] == code
    assert "/raw/front" in info["last_error"]
    assert info["input"]["state"] == state
    assert info["input"]["publisher_count"] == publishers
    assert not rig.plugin._worker.jobs and not rig.publications


def test_valid_cloud_recovers_timeout_and_reports_actual_frame(rig, monkeypatch):
    configure(rig)
    clock = [100.]
    monkeypatch.setattr(module().time, "monotonic", lambda: clock[0])
    rig.plugin.dispatch("start", {})
    clock[0] = 104.
    rig.timers[0].callback()
    assert rig.plugin.dispatch("info", {})["state"] == "error"
    rig.relays[0].on_message(cloud())
    rig.plugin._worker.complete()
    info = rig.plugin.dispatch("info", {})
    assert info["state"] == "running" and info["error_code"] is None
    assert info["input"]["state"] == "live"
    assert info["input"]["observed_frame_id"] == "front_lidar"
    assert info["input"]["last_valid_cloud_age_s"] == 0
    clock[0] = 108.
    rig.timers[0].callback()
    assert rig.plugin.dispatch("info", {})["input"]["state"] == "stale"


def test_mismatched_frame_is_diagnostic_and_never_replaces_calibrated_frame(rig):
    deployment_calibration(rig)
    rig.plugin.dispatch("start", {})
    rig.subscriptions[0].callback(cloud(frame="map"))
    info = rig.plugin.dispatch("info", {})
    assert info["error_code"] == "CLOUD_FRAME_MISMATCH"
    assert info["input"]["observed_frame_id"] == "map"
    assert info["config"]["cloud_frame_id"] == "lidar_link"
    assert not rig.plugin._worker.jobs and not rig.publications


def test_watchdog_is_owned_by_session_and_late_callbacks_are_ignored(rig, monkeypatch):
    configure(rig)
    clock = [100.]
    monkeypatch.setattr(module().time, "monotonic", lambda: clock[0])
    rig.plugin.dispatch("start", {})
    rig.plugin.dispatch("start", {})
    assert len(rig.timers) == 1
    old = rig.timers[0]
    rig.plugin.dispatch("stop", {})
    assert rig.destroyed_timers == [old]
    rig.plugin.dispatch("start", {})
    clock[0] = 104.
    old.callback()
    assert rig.plugin.dispatch("info", {})["state"] == "running"
    assert rig.plugin.dispatch("info", {})["input"]["state"] == "waiting"
    rig.plugin.dispatch("config", {"rate_hz": 4})
    assert rig.destroyed_timers == rig.timers
    assert rig.plugin.dispatch("info", {})["input"]["state"] == "idle"


def test_failed_watchdog_creation_releases_subscription_and_worker(rig):
    configure(rig)
    def fail_timer(*args, **kwargs):
        raise RuntimeError("timer unavailable")
    rig.nodes.robot.create_timer = fail_timer
    result = rig.plugin.dispatch("start", {})
    assert result["state"] == "error" and "timer unavailable" in result["message"]
    assert rig.destroyed == rig.subscriptions
    assert rig.plugin._worker is None


def test_repeated_malformed_payloads_never_refresh_valid_input_health(rig, monkeypatch):
    configure(rig)
    clock = [100.]
    monkeypatch.setattr(module().time, "monotonic", lambda: clock[0])
    rig.publisher_infos.append(SimpleNamespace())
    rig.plugin.dispatch("start", {})
    for index in range(1, 5):
        clock[0] = 100. + index
        message = cloud(sec=10 + index)
        message.data = b'\0' * 3
        rig.subscriptions[-1].callback(message)
    rig.timers[0].callback()
    info = rig.plugin.dispatch("info", {})
    assert info["input"]["last_valid_cloud_age_s"] is None
    assert info["input"]["state"] == "stale"
    assert info["error_code"] == "INVALID_CLOUD"
    assert "truncated" in info["last_error"]
    assert not rig.plugin._worker.jobs and not rig.publications


def test_valid_throttled_clouds_keep_input_live_at_low_output_rate(rig, monkeypatch):
    configure(rig, rate_hz=.1)
    clock = [100.]
    monkeypatch.setattr(module().time, "monotonic", lambda: clock[0])
    rig.publisher_infos.append(SimpleNamespace())
    rig.plugin.dispatch("start", {})
    rig.subscriptions[-1].callback(cloud())
    rig.plugin._worker.complete()
    clock[0] = 104.
    rig.subscriptions[-1].callback(cloud(sec=14))
    rig.timers[0].callback()
    info = rig.plugin.dispatch("info", {})
    assert info["input"]["state"] == "live" and info["error_code"] is None
    assert info["input"]["last_valid_cloud_age_s"] == 0
    assert len(rig.plugin._worker.jobs) == 1  # Receipt health is independent of output throttling.


def test_stream_binding_errors_do_not_expose_rtsp_credentials(rig):
    deployment_calibration(rig)
    plugin = module().FrontDepthPlugin(
        {"camera_rtsp": {"camera_front": "rtsp://viewer:not-a-real-secret@camera:8554/video4"}},
        "robot", rig.nodes, config_path=rig.path, extrinsics_path=rig.extrinsics_path)
    result = plugin.dispatch("start", {})
    assert result["state"] == "error" and result["code"] == "CALIBRATION_INVALID"
    assert "not-a-real-secret" not in result["message"]


def test_missing_ros_cloud_automatically_uses_ssh_without_canvas_settings(rig, monkeypatch):
    deployment_calibration(rig)
    clock = [100.]
    monkeypatch.setattr(module().time, 'monotonic', lambda: clock[0])
    rig.plugin.dispatch('start', {})
    clock[0] = 104.
    rig.timers[0].callback()
    info = rig.plugin.dispatch('info', {})
    assert info['input']['transport'] == 'ssh'
    assert len(rig.relays) == 1 and rig.relays[0].topic == '/LIDAR/POINTS'
    rig.relays[0].on_message(cloud(frame='lidar_link'))
    rig.plugin._worker.complete()
    output = rig.publications[-1]
    assert (output.header.stamp.sec, output.header.stamp.nanosec) == (10, 123)
    assert rig.plugin.dispatch('info', {})['input']['state'] == 'live'
    assert set(rig.plugin.get_tool()['configSchema']['properties']) == {'rate_hz', 'min_depth_m', 'max_depth_m'}


def test_healthy_direct_ros_does_not_start_ssh(rig, monkeypatch):
    configure(rig)
    clock = [100.]
    monkeypatch.setattr(module().time, 'monotonic', lambda: clock[0])
    rig.plugin.dispatch('start', {})
    clock[0] = 104.
    rig.subscriptions[0].callback(cloud())
    rig.timers[0].callback()
    assert rig.plugin.dispatch('info', {})['input']['transport'] == 'ros'
    assert not rig.relays


def test_missing_calibration_does_not_start_remote_subscription(rig):
    assert rig.plugin.dispatch('start', {})['code'] == 'CALIBRATION_MISSING'
    assert not rig.relays


def test_ssh_is_single_selected_transport_and_stale_callbacks_cannot_restart_it(rig, monkeypatch):
    configure(rig)
    clock = [100.]
    monkeypatch.setattr(module().time, 'monotonic', lambda: clock[0])
    rig.plugin.dispatch('start', {})
    clock[0] = 104.
    rig.timers[0].callback()
    rig.timers[0].callback()
    assert len(rig.relays) == 1
    relay, worker = rig.relays[0], rig.plugin._worker
    rig.subscriptions[0].callback(cloud(sec=50))
    assert not worker.jobs  # Direct DDS cannot race/duplicate the selected SSH input.
    relay.on_message(cloud(sec=10))
    assert len(worker.jobs) == 1
    rig.plugin.dispatch('stop', {})
    assert not relay.is_alive()
    relay.on_message(cloud(sec=11))
    relay.on_error('CLOUD_RELAY_FAILED', 'late failure')
    assert rig.plugin.dispatch('info', {})['state'] == 'idle'
    rig.plugin.dispatch('start', {})
    relay.on_message(cloud(sec=12))
    assert not rig.plugin._worker.jobs
    assert rig.plugin.dispatch('info', {})['input']['transport'] == 'ros'


def test_relay_errors_are_visible_and_recover_on_valid_input(rig, monkeypatch):
    configure(rig)
    clock = [100.]
    monkeypatch.setattr(module().time, 'monotonic', lambda: clock[0])
    rig.plugin.dispatch('start', {})
    clock[0] = 104.
    rig.timers[0].callback()
    relay = rig.relays[0]
    relay.on_error('CLOUD_RELAY_FAILED', 'SSH disconnected')
    assert rig.plugin.dispatch('info', {})['error_code'] == 'CLOUD_RELAY_FAILED'
    relay.on_message(cloud())
    rig.plugin._worker.complete()
    assert rig.plugin.dispatch('info', {})['error_code'] is None
    clock[0] = 108.
    rig.timers[0].callback()
    assert rig.plugin.dispatch('info', {})['error_code'] == 'CLOUD_RELAY_DATA_TIMEOUT'
    rig.plugin.dispatch('config', {'rate_hz': 4})
    assert not relay.is_alive()


def test_calibration_frame_error_never_triggers_transport_fallback(rig, monkeypatch):
    configure(rig)
    clock = [100.]
    monkeypatch.setattr(module().time, 'monotonic', lambda: clock[0])
    rig.plugin.dispatch('start', {})
    rig.subscriptions[0].callback(cloud(frame='map'))
    clock[0] = 104.
    rig.timers[0].callback()
    assert rig.plugin.dispatch('info', {})['error_code'] == 'CLOUD_FRAME_MISMATCH'
    assert not rig.relays


def configure(rig, **values):
    result = rig.plugin.dispatch("config", {
        "cloud_topic": "/raw/front", "cloud_frame_id": "front_lidar",
        "camera_optical_from_cloud": IDENTITY, **values})
    assert result["state"] == "configured", result
    return result


def cloud(sec=10, nanosec=123, frame="front_lidar"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        width=1, height=1, is_bigendian=False, point_step=12, row_step=12,
        fields=[SimpleNamespace(name=name, offset=i * 4, datatype=7, count=1)
                for i, name in enumerate(("x", "y", "z"))],
        data=struct.pack("<fff", 0, 0, 2))


def test_catalogue_and_runtime_advertise_exactly_one_new_card(monkeypatch, tmp_path):
    depth = module()
    monkeypatch.syspath_prepend(str(DRIVER))
    monkeypatch.setenv("S10_FRONT_DEPTH_CONFIG", str(tmp_path / "depth.json"))
    monkeypatch.setenv("S10_CAMERA_UNDISTORT_CONFIG", str(tmp_path / "rgb.json"))
    device = importlib.import_module("device")
    # No interface enumeration, network service, or native controller is needed
    # to inspect card definitions. Spatial construction has unrelated host I/O.
    monkeypatch.setattr(device, "S10ControlledSpatialPlugin", lambda *a: SimpleNamespace(
        get_tool=lambda: {"name": "controlled_spatial"}))
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    plugins = device.build_plugins(config)
    cards = [card for p in plugins for card in
             (p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()])]
    assert sum(c["name"] == "camera_front_depth" for c in cards) == 1
    manifest = yaml.safe_load((DRIVER / "driver.yaml").read_text())
    assert sum(c["name"] == "camera_front_depth" for c in manifest["cards"]) == 1
    assert any(isinstance(p, depth.FrontDepthPlugin) for p in plugins)


def test_card_wire_contract_geometry_and_time_provenance(rig):
    card = rig.plugin.get_tool()
    assert card["name"] == "camera_front_depth" and card["type"] == "sensor"
    assert [entry["topic"] for entry in card["topic_out"]] == [
        "/robot/camera_front/depth"]
    assert all(entry["format"] == "image/depth-zlib" for entry in card["topic_out"])
    assert not card.get("topic_in")
    assert set(card["inputSchema"]["x-action-params"]) == {"start", "stop", "info", "config"}
    info = card["camera_info"][0]
    assert len(card["camera_info"]) == 1
    assert (info["width"], info["height"]) == (640, 480)
    cal, _ = rig.controller.calibration("camera_front")
    assert info["K"][4] == pytest.approx(cal["K"][4] * 4 / 3)
    assert info["K"][5] == pytest.approx((cal["K"][5] + .5) * 4 / 3 - .5)
    assert info["D"] == [0.] * 5 and info["distortion_model"] == "pinhole"
    vendor = info["vendor"]
    assert vendor["depth_scale_m"] == .001 and vendor["invalid_depth_value"] == 0
    assert vendor["depth_quantity"] == "optical_z" and vendor["sparse"] is False
    assert vendor["hole_filling"] is True
    assert vendor["depth_source"] == "lidar_monocular_fusion"
    assert vendor["interpolated_pixels_are_measured"] is False
    assert not vendor.get("raw_depth_topic")
    assert vendor["depth_aligned_to_rgb"] is False
    assert vendor["timestamp_source"] == "lidar_header_unverified"
    assert vendor["temporal_synchronization"] is False
    assert vendor["rgb_depth_capture_skew_ms"] is None


def test_missing_extrinsic_or_frame_refuses_start_without_io(rig):
    assert rig.plugin.dispatch("start", {})["state"] == "error"
    assert not rig.subscriptions
    rig.plugin.dispatch("config", {"camera_optical_from_cloud": IDENTITY})
    result = rig.plugin.dispatch("start", {})
    assert result["state"] == "error" and "cloud_frame_id" in result["message"]
    assert not rig.subscriptions


def test_config_is_atomic_persistent_and_rejects_invalid_values(rig):
    configure(rig)
    before = rig.path.read_bytes()
    for invalid in ({"rate_hz": True}, {"rate_hz": 10 ** 400}, {"max_depth_m": 100},
                    {"cloud_topic": "relative"}, {"unexpected": 1},
                    {"min_depth_m": 60}, {"camera_optical_from_cloud": [[1]]}):
        assert rig.plugin.dispatch("config", invalid)["state"] == "error"
        assert rig.path.read_bytes() == before
    loaded = module().FrontDepthPlugin({}, "robot", rig.nodes, config_path=rig.path)
    assert loaded.dispatch("info", {})["config"]["cloud_frame_id"] == "front_lidar"
    assert loaded.dispatch("info", {})["config"]["camera_optical_from_cloud"] == IDENTITY


def test_persist_failure_does_not_apply_new_config(rig, monkeypatch):
    configure(rig)
    rig.plugin.dispatch("start", {})
    worker = rig.plugin._worker
    old = rig.plugin.dispatch("info", {})["config"]
    monkeypatch.setattr(module().os, "replace", lambda *a: (_ for _ in ()).throw(OSError("disk full")))
    result = rig.plugin.dispatch("config", {"cloud_frame_id": "different"})
    assert result["state"] == "error" and "disk full" in result["message"]
    assert rig.plugin.dispatch("info", {})["config"] == old
    assert rig.plugin._worker is worker and worker.is_alive()
    assert not rig.destroyed


def test_canvas_instance_transport_does_not_become_a_depth_setting(rig):
    from common.vendor_runtime import DriverBundle
    bundle = DriverBundle([rig.plugin])
    result = bundle.dispatch("camera_front_depth", {
        "action": "config", "instance_id": "canvas-depth-1",
        "cloud_topic": "/raw/front", "cloud_frame_id": "front_lidar",
        "camera_optical_from_cloud": json.dumps(IDENTITY),
    })
    assert result["state"] == "configured", result
    assert "instance_id" not in result["config"]
    assert "instance_id" not in json.loads(rig.path.read_text())["config"]
    assert bundle.dispatch("camera_front_depth", {
        "action": "start", "instance_id": "canvas-depth-1"})["state"] == "running"


def test_start_is_lazy_and_idempotent_preserves_source_stamp_and_metric_payload(rig):
    configure(rig)
    rig.plugin.start()  # Bundle startup only advertises the card.
    assert not rig.subscriptions
    assert rig.plugin.dispatch("start", {})["state"] == "running"
    assert rig.plugin.dispatch("start", {})["state"] == "running"
    assert len(rig.subscriptions) == 1
    handle = rig.subscriptions[0]
    assert handle.topic == "/raw/front" and handle.qos is rig.nodes._sensor_qos
    handle.callback(cloud())
    worker = rig.plugin._worker
    assert len(worker.jobs) == 1
    worker.complete()
    output = rig.publications[-1]
    assert output.format == "16UC1; compressedDepth zlib"
    assert output.header.frame_id == "s10_pro_camera_front_optical"
    assert (output.header.stamp.sec, output.header.stamp.nanosec) == (10, 123)
    raw = zlib.decompress(output.data)
    assert len(raw) == 640 * 480 * 2
    values = struct.unpack("<307200H", raw)
    assert values.count(2000) == 1 and values.count(0) == 307199
    assert rig.plugin.dispatch("info", {})["stats"]["published"] == 1


@pytest.mark.parametrize("bad", [cloud(frame="map"), cloud(sec=0, nanosec=0),
                                  cloud(nanosec=1000000000), cloud(sec=True)])
def test_unexpected_frame_and_missing_or_malformed_stamp_are_rejected(rig, bad):
    configure(rig)
    rig.plugin.dispatch("start", {})
    rig.subscriptions[-1].callback(bad)
    assert not rig.plugin._worker.jobs
    assert rig.plugin.dispatch("info", {})["stats"]["dropped"] == 1


def test_stop_and_config_discard_in_flight_results_and_do_not_close_shared_nodes(rig):
    configure(rig)
    rig.plugin.dispatch("start", {})
    old_handle, worker = rig.subscriptions[-1], rig.plugin._worker
    old_handle.callback(cloud())
    assert rig.plugin.dispatch("stop", {})["state"] == "idle"
    worker.complete()
    assert not rig.publications and rig.destroyed == [old_handle]
    rig.plugin.dispatch("start", {})
    old_handle.callback(cloud(sec=11))  # A queued callback from the old session.
    assert not rig.plugin._worker.jobs
    rig.subscriptions[-1].callback(cloud(sec=12))
    worker = rig.plugin._worker
    assert rig.plugin.dispatch("config", {"rate_hz": 4})["state"] == "configured"
    worker.complete()
    assert not rig.publications
    assert rig.plugin.dispatch("info", {})["state"] == "idle"


def test_camera_geometry_change_drops_old_result_and_blocks_raw_view(rig):
    configure(rig)
    rig.plugin.dispatch("start", {})
    rig.subscriptions[-1].callback(cloud())
    worker = rig.plugin._worker
    rig.controller.configure("camera_front", {"undistort": False})
    worker.complete()
    assert not rig.publications
    assert rig.plugin.dispatch("info", {})["state"] == "error"
    rig.plugin.dispatch("stop", {})
    assert rig.plugin.dispatch("start", {})["state"] == "error"


def test_dead_worker_is_reported_and_start_can_recover(rig):
    configure(rig)
    rig.plugin.dispatch("start", {})
    old = rig.plugin._worker
    old.alive = False
    assert rig.plugin.dispatch("info", {})["state"] == "error"
    assert rig.plugin.dispatch("start", {})["state"] == "running"
    assert rig.plugin._worker is not old and rig.plugin._worker.is_alive()


def test_depth_publication_preserves_front_camera_error(rig):
    configure(rig)
    rig.plugin.dispatch("start", {})
    generation = rig.controller.generation("camera_front")
    rig.controller.record_error("camera_front", generation, "RTSP unavailable")
    rig.subscriptions[-1].callback(cloud())
    rig.plugin._worker.complete()
    assert len(rig.publications) == 1
    assert rig.controller.status("camera_front")["error"] == "RTSP unavailable"
    # The existing RGB publication path still acknowledges camera recovery.
    rig.controller.publish_if_current("camera_front", generation, lambda: None)
    assert rig.controller.status("camera_front")["error"] is None


def test_geometry_change_at_publish_guard_records_drop_and_next_frame_recovers(rig, monkeypatch):
    configure(rig)
    rig.plugin.dispatch('start', {})
    callback = rig.subscriptions[-1].callback
    callback(cloud())
    guard = rig.controller.publish_if_current

    def change_before_publish(*args, **kwargs):
        rig.controller.configure('camera_front', {'undistort': False})
        rig.controller.configure('camera_front', {'undistort': True})
        return guard(*args, **kwargs)

    monkeypatch.setattr(rig.controller, 'publish_if_current', change_before_publish)
    rig.plugin._worker.complete()
    info = rig.plugin.dispatch('info', {})
    assert not rig.publications
    assert info['stats']['dropped'] == 1
    assert info['stats']['published'] == 0
    assert 'geometry changed' in info['last_error']

    monkeypatch.setattr(rig.controller, 'publish_if_current', guard)
    rig.controller.configure('camera_front', {'undistort': True})
    future = time.monotonic() + .3
    monkeypatch.setattr(module().time, 'monotonic', lambda: future)
    callback(cloud(sec=11))
    rig.plugin._worker.complete()
    info = rig.plugin.dispatch('info', {})
    assert len(rig.publications) == 1
    assert info['stats']['published'] == 1 and info['stats']['dropped'] == 1
    assert info['state'] == 'running' and info['last_error'] is None


def test_repeated_backward_and_stale_source_work_is_not_published(rig, monkeypatch):
    configure(rig)
    rig.plugin.dispatch("start", {})
    callback = rig.subscriptions[-1].callback
    callback(cloud())
    callback(cloud())
    callback(cloud(sec=9))
    assert len(rig.plugin._worker.jobs) == 1
    assert rig.plugin.dispatch("info", {})["stats"]["dropped"] == 2
    future = time.monotonic() + 4
    monkeypatch.setattr(module().time, "monotonic", lambda: future)
    rig.plugin._worker.complete()
    assert not rig.publications
    assert rig.plugin.dispatch("info", {})["last_error"] == "depth result is stale"


def test_failed_subscription_start_closes_owned_worker(rig, monkeypatch):
    configure(rig)
    workers = []

    def make_worker(*args):
        worker = FakeWorker(*args)
        workers.append(worker)
        return worker

    def fail_subscription(*args, **kwargs):
        raise RuntimeError("subscription failed")

    monkeypatch.setattr(module(), "DepthWorker", make_worker)
    rig.nodes.robot.create_subscription = fail_subscription
    result = rig.plugin.dispatch("start", {})
    assert result["state"] == "error" and "subscription failed" in result["message"]
    assert len(workers) == 1 and not workers[0].is_alive()
    info = rig.plugin.dispatch("info", {})
    assert info["state"] == "error" and info["error_code"] == "DEPTH_START_FAILED"
    assert "subscription failed" in info["last_error"]


def _worker_with_constant_monocular_model(jobs, results):
    # Keep the actual spawned worker, projection and fusion. Only the external
    # neural model is an oracle; no engine download or GPU belongs in IPC tests.
    depth = module()
    class ConstantModel:
        def predict(self, jpeg):
            assert jpeg == b'front-jpeg'
            return zlib.compress(struct.pack('<H', 2000) * (640 * 480))
    depth.MonocularDepth = ConstantModel
    depth._project_worker(jobs, results)


def test_real_worker_reports_bad_layout_then_completes_depth_and_closes(monkeypatch):
    depth = module()
    monkeypatch.setattr(depth, '_project_worker', _worker_with_constant_monocular_model)
    received, failures = [], []
    ready, failed = threading.Event(), threading.Event()

    def on_result(context, packet, stats):
        received.append((context, packet, stats))
        ready.set()

    def on_error(context, reason):
        failures.append(reason)
        failed.set()

    message = cloud()
    points = [((x - 320) * .02, (y - 240) * .02, 2)
              for y in range(60, 421, 24) for x in range(60, 581, 24)]
    message.width, message.row_step = len(points), len(points) * 12
    message.data = b''.join(struct.pack('<fff', *p) for p in points)
    source = {key: getattr(message, key) for key in
              ("width", "height", "point_step", "row_step", "is_bigendian", "data")}
    source["fields"] = [(f.name, f.offset, f.datatype, f.count) for f in message.fields]
    job = {"generation": 1, "camera_generation": 0, "stamp": (10, 123),
           "received_monotonic": time.monotonic(), "cloud": source,
           "jpeg": b'front-jpeg', "rgb_received_monotonic": time.monotonic(),
           "K": [100, 0, 320, 0, 100, 240, 0, 0, 1], "transform": IDENTITY,
           "min_depth_m": .1, "max_depth_m": 50.}
    worker = depth.DepthWorker(on_result, on_error)
    try:
        worker.start()
        assert worker.submit({**job, "cloud": {**source, "data": b""}})
        assert failed.wait(5), "worker did not report malformed point layout"
        assert len(failures) == 1 and "truncated" in failures[0]
        assert worker.submit(job)
        assert ready.wait(5), "worker did not return depth"
        context, packet, stats = received[0]
        assert context["stamp"] == (10, 123) and "cloud" not in context
        assert isinstance(packet, bytes)
        raw = zlib.decompress(packet)
        assert len(raw) == 640 * 480 * 2
        assert struct.unpack_from("<H", raw, (240 * 640 + 320) * 2)[0] == 2000
        assert struct.unpack_from("<H", raw, (240 * 640 + 322) * 2)[0] == 2000
        assert struct.unpack_from("<H", raw, (240 * 640 + 325) * 2)[0] == 2000
        assert stats["source_points"] == len(points)
        assert stats["sample_points"] == stats["valid_pixels"] == len(points)
        assert stats["filled_pixels"] == 0 and stats["fill_conflicts"] == 0
        assert stats["measured_coverage"] <= stats["filled_coverage"]
        assert stats['output_coverage'] == 1.0
        assert stats['monocular_pixels'] == 640 * 480 - len(points)
        assert set(struct.unpack('<307200H', raw)) == {2000}
    finally:
        worker.close()
    assert not worker.is_alive()


def _wait_for(predicate):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if predicate():
            return True
        threading.Event().wait(.01)
    return False


def _process_alive(worker):
    try:
        return worker._process.is_alive()
    except ValueError:  # Process.close() released the already-reaped child.
        return False


def _cleanup_faulted_worker(worker):
    # Keep a failing regression from leaking the intentionally damaged worker.
    if _process_alive(worker):
        worker._process.kill()
        worker._process.join(2)
    worker._closed.set()
    worker._results._writer.close()
    worker._collector.join(2)
    worker._jobs._reader.close()
    for channel in (worker._jobs, worker._results):
        channel.cancel_join_thread()
        channel.close()
        if channel._thread is not None:
            channel._thread.join(2)
        channel._writer.close()
        channel._reader.close()


def _partial_result_and_exit(jobs, results):
    # Model a child killed after the pickle frame header and before its body.
    os.write(results._writer.fileno(), struct.pack("!i", 4096) + b"partial")


def test_worker_close_reaps_suspended_child():
    worker = module().DepthWorker(lambda *a: None, lambda *a: None)
    worker.start()
    try:
        os.kill(worker._process.pid, signal.SIGSTOP)
        worker.close()
        assert not _process_alive(worker)
        assert not worker._collector.is_alive()
    finally:
        _cleanup_faulted_worker(worker)


def test_worker_close_releases_feeder_after_killed_large_transfer():
    worker = module().DepthWorker(lambda *a: None, lambda *a: None)
    worker.start()

    def child_is_receiving():
        acquired = worker._jobs._rlock.acquire(block=False)
        if acquired:
            worker._jobs._rlock.release()
        return not acquired

    try:
        assert _wait_for(child_is_receiving)
        os.kill(worker._process.pid, signal.SIGSTOP)
        assert worker.submit({"payload": b"x" * (32 * 1024 * 1024)})
        assert _wait_for(lambda: worker._jobs._reader.poll(0))
        worker._process.kill()
        worker._process.join(2)
        feeder = worker._jobs._thread
        worker.close()
        assert not feeder.is_alive()
        assert worker._jobs._reader.closed and worker._jobs._writer.closed
        assert not worker._collector.is_alive()
    finally:
        _cleanup_faulted_worker(worker)


def test_worker_close_releases_collector_after_partial_result(monkeypatch):
    monkeypatch.setattr(module(), "_project_worker", _partial_result_and_exit)
    worker = module().DepthWorker(lambda *a: None, lambda *a: None)
    worker.start()
    try:
        worker._process.join(5)
        assert worker._process.exitcode == 0
        worker.close()
        assert not worker._collector.is_alive()
        assert worker._results._reader.closed and worker._results._writer.closed
    finally:
        _cleanup_faulted_worker(worker)


def test_output_config_keeps_projection_worker_and_encoder_latency_fix():
    text = (DRIVER / "Dockerfile").read_text()
    assert "front_depth.py" in text and "depth_projection.py" in text
    from deep_robotics.s10_pro.media_workers import _rtsp_command
    cmd = _rtsp_command("rtsp://camera")
    assert cmd[cmd.index("-threads") + 1] == "1"

def test_only_filled_stream_uses_original_topic_stamp_and_estimated_provenance(rig):
    from deep_robotics.s10_pro.depth_projection import project_depth, fill_depth_holes
    configure(rig)
    assert rig.plugin.dispatch("start", {})["state"] == "running"
    message = cloud()
    message.width, message.row_step = 2, 24
    message.data = struct.pack("<ffffff", 0, 0, 2, .025, 0, 2)
    rig.subscriptions[0].callback(message)
    job = rig.plugin._worker.jobs[0]
    raw, _ = project_depth(job["cloud"], job["K"], job["transform"])
    filled, _ = fill_depth_holes(raw)
    rig.plugin._worker.complete()
    assert len(rig.publications) == 1
    estimate = rig.publications[0]
    assert estimate.data == filled
    assert zlib.decompress(estimate.data) != zlib.decompress(raw)
    assert estimate.header.stamp.sec == 10
    assert estimate.header.stamp.nanosec == 123
    info = rig.plugin.dispatch("info", {})
    assert info["stats"]["filled_pixels"] > 0
    assert [entry["topic"] for entry in info["topic_out"]] == ["/robot/camera_front/depth"]
    assert len(rig.publishers) == len(info["camera_info"]) == 1
    assert rig.publishers[0][1] == "/robot/camera_front/depth"
    filled_info = info["camera_info"][0]
    assert filled_info["topic"] == "/robot/camera_front/depth"
    assert filled_info["vendor"]["hole_filling"] is True
    assert filled_info["vendor"]["depth_aligned_to_rgb"] is False
    assert filled_info["vendor"]["interpolated_pixels_are_measured"] is False
    assert not filled_info["vendor"].get("raw_depth_topic")


def test_stopped_depth_card_cannot_publish_late_filled_result(rig):
    from deep_robotics.s10_pro.depth_projection import project_depth, fill_depth_holes
    configure(rig)
    rig.plugin.dispatch("start", {})
    rig.subscriptions[0].callback(cloud())
    job = rig.plugin._worker.jobs[0]
    raw, stats = project_depth(job["cloud"], job["K"], job["transform"])
    filled, _ = fill_depth_holes(raw)
    rig.plugin.dispatch("stop", {})
    rig.plugin._receive_result(job, filled, stats)
    assert rig.publications == []
