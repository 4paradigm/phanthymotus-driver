"""Deployment/runtime contracts exercised without USB or a production ROS graph."""
import importlib.util
import math
import os
from pathlib import Path
import signal
import sys
from types import SimpleNamespace as NS
import zlib

import pytest

BUMI = Path(__file__).resolve().parents[1] / 'noetix' / 'bumi'
sys.path.insert(0, str(BUMI))
from camera_runtime import CameraRuntime, packed_image, compressed_depth, camera_arguments
from fusion_runtime import FusionHealth, GravityWindow, fusion_parameters, check_dependencies


def image(encoding='16UC1', endian=False):
    return NS(width=2, height=2, encoding=encoding, step=6,
              is_bigendian=endian, data=b'\x01\x00\x02\x00XX\x03\x00\x04\x00YY')


def test_depth_bridge_strips_padding_and_keeps_z16_contract():
    assert zlib.decompress(compressed_depth(image())) == b'\x01\x00\x02\x00\x03\x00\x04\x00'
    assert zlib.decompress(compressed_depth(image(endian=True))) == b'\x00\x01\x00\x02\x00\x03\x00\x04'


@pytest.mark.parametrize('change', [dict(encoding='32FC1'), dict(step=3), dict(data=b'bad')])
def test_malformed_images_are_not_published_as_valid_depth(change):
    msg = image()
    msg.__dict__.update(change)
    with pytest.raises(ValueError):
        compressed_depth(msg)


def test_rgb_bridge_preserves_channel_bytes_and_removes_padding():
    msg = NS(width=1, height=2, step=4, encoding='rgb8', is_bigendian=False, data=b'abcXdefY')
    assert packed_image(msg, 3) == b'abcdef'


def test_one_camera_opener_enables_imu_and_alignment():
    args = camera_arguments()
    assert args[:4] == ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py']
    for value in ('enable_gyro:=true', 'enable_accel:=true', 'unite_imu_method:=2',
                  'align_depth.enable:=true', 'rgb_camera.color_profile:=640x480x30'):
        assert value in args
    assert not any('ROS_DOMAIN' in value for value in args)


class Process:
    def __init__(self, pid):
        self.pid = pid
        self.code = None
        self.stdout = []
        self.waited = False
    def poll(self):
        return self.code
    def wait(self, timeout):
        self.waited = True
        self.code = 0


def test_fusion_crash_restarts_only_fusion_and_inherits_domain(monkeypatch):
    import camera_runtime as module
    opened, signals = [], []
    def popen(args, **kwargs):
        proc = Process(len(opened)+100)
        opened.append((args, kwargs, proc))
        return proc
    monkeypatch.setenv('ROS_DOMAIN_ID', '42')
    monkeypatch.setattr(module.subprocess, 'Popen', popen)
    monkeypatch.setattr(module.os, 'killpg', lambda pid, sig: signals.append((pid, sig)))
    monkeypatch.setattr(module, '_group_alive', lambda pid: False, raising=False)
    runtime = CameraRuntime('ubuntu', lambda line: None)
    runtime.tick(0)
    assert len(opened) == 3
    assert sum('realsense2_camera' in args for args, _, _ in opened) == 1
    fusion = runtime.processes['fusion']
    fusion.code = 1
    runtime.tick(6)
    assert len(opened) == 4
    assert runtime.processes['camera'] is opened[0][2]
    assert runtime.processes['bridge'] is opened[1][2]
    assert opened[-1][1]['env']['ROS_DOMAIN_ID'] == '42'
    assert opened[-1][1]['env']['OMP_NUM_THREADS'] == '2'
    assert opened[-1][1]['start_new_session'] is True
    runtime.stop()
    assert all(proc.waited for _, _, proc in opened)
    assert all(sig == signal.SIGINT for _, sig in signals)
    before = len(opened)
    runtime.tick(20)
    assert len(opened) == before


def test_dead_launch_leader_still_has_its_children_reaped(monkeypatch):
    import camera_runtime as module
    signals = []
    proc = Process(1234)
    proc.code = 1
    monkeypatch.setattr(module.os, 'killpg', lambda pid, sig: signals.append((pid, sig)))
    monkeypatch.setattr(module, '_group_alive', lambda pid: False, raising=False)
    module.stop_process(proc)
    assert signals == [(1234, signal.SIGINT)]
    assert proc.waited


def test_unreapable_group_does_not_kill_camera_supervision(monkeypatch):
    # 真机复盘：pkill 后进程组残留僵尸，stop_process 抛 RuntimeError，watch
    # 线程未兜住直接死——安全互斥正常（无双采集）但相机从此无人重启。监督
    # 必须在单个进程组回收失败时存活：报错走 on_line，下个 tick 重试该名字，
    # 其余名字照常处理。
    import camera_runtime as module
    opened, lines = [], []
    def popen(args, **kwargs):
        proc = Process(len(opened) + 100)
        opened.append((args, kwargs, proc))
        return proc
    monkeypatch.setattr(module.subprocess, 'Popen', popen)
    runtime = CameraRuntime('ubuntu', lines.append)
    runtime.tick(0)
    assert len(opened) == 3
    runtime.processes['fusion'].code = 1          # fusion 挂了
    real_stop = module.stop_process
    monkeypatch.setattr(module, 'stop_process',
                        lambda proc: (_ for _ in ()).throw(RuntimeError(
                            'camera/fusion process group did not exit; refusing a duplicate owner')))
    runtime.tick(6)                               # 不得抛出
    assert any('did not exit' in line for line in lines)
    assert runtime.processes['camera'] is opened[0][2]   # 其余进程未受牵连
    monkeypatch.setattr(module, 'stop_process', real_stop)
    runtime.tick(12)                              # 组可回收后 fusion 重启
    assert len(opened) == 4
    assert 'fusion_runtime.py' in ' '.join(opened[-1][0])


def test_camera_runtime_can_leave_fusion_disabled():
    runtime = CameraRuntime('robot', print, fusion_enabled=False)
    assert set(runtime.commands) == {'camera', 'bridge'}


def test_gravity_needs_stationary_time_spanning_samples_and_recovers_after_motion():
    window = GravityWindow()
    for i in range(25):
        assert window.observe(i*0.01, (0, 0, 9.81), (0, 0, 0)) is None
    assert window.observe(0.26, (0, 0, 9.81), (0, 0, 0.2)) is None
    for i in range(30):
        result = window.observe(0.3+i*0.04, (0, 0, 9.81), (0, 0, 0))
    assert result == pytest.approx((0, 0))


def test_initial_gravity_corrects_camera_pitch_without_inventing_body_mount():
    pitch = math.radians(20)
    window = GravityWindow()
    for i in range(30):
        result = window.observe(i*0.04, (-9.81*math.sin(pitch), 0, 9.81*math.cos(pitch)), (0, 0, 0))
    assert result == pytest.approx((0, pitch))
    visual, ekf = fusion_parameters(*result)
    assert visual['frame_id'] == 'bumi_odom_base_link'
    assert visual['wait_imu_to_init'] is False
    # 宿主机内核无 HID_SENSOR_HUB → 相机 IMU 不可用；EKF 吃驱动桥接的本体 IMU。
    assert ekf['imu0'] == '/bumi_odom/body_imu'
    assert ekf['imu0_config'] == [False]*9 + [True]*3 + [False]*3
    assert visual['max_update_rate'] == 10.0
    assert visual['publish_tf'] is False


@pytest.mark.parametrize('missing', ['imu', 'visual', 'diagnostic', 'filtered'])
def test_each_required_stream_can_invalidate_tracking(missing):
    health = FusionHealth()
    for kind in health.kinds:
        health.observe(kind, 10**9, 10**9, True)
    assert health.status(10**9)['status'] == 'tracking'
    for kind in health.kinds:
        if kind != missing:
            health.observe(kind, 1_600_000_000, 1_600_000_000, True)
    assert health.status(1_600_000_000)['status'] == 'lost'
    health.observe(missing, 1_610_000_000, 1_610_000_000, True)
    assert health.status(1_610_000_000)['status'] == 'lost'


@pytest.mark.parametrize('stamp,now,valid', [(10**9, 10**9, False), (10**9, 10**9, True),
                                           (1_100_000_000, 2*10**9, True), (3*10**9, 2*10**9, True)])
def test_invalid_or_repeated_source_cannot_certify_tracking(stamp, now, valid):
    health = FusionHealth()
    for kind in health.kinds:
        health.observe(kind, 10**9, 10**9, True)
    health.observe('visual', stamp, now, valid)
    assert health.status(now)['status'] == 'lost'


def test_new_health_session_uses_new_segment_and_waits_for_all_inputs():
    first, second = FusionHealth(), FusionHealth()
    assert first.segment != second.segment
    second.observe('imu', 10**9, 10**9, True)
    assert second.status(10**9)['status'] == 'initializing'


def test_queued_previous_session_samples_cannot_initialize_new_segment():
    health = FusionHealth(started_ns=2*10**9)
    for kind in health.kinds:
        health.observe(kind, 1_900_000_000, 2*10**9, True)
    assert health.samples == {}
    assert health.status(2*10**9)['status'] == 'initializing'


def test_uninitialized_first_pose_waits_for_valid_estimate_without_certifying_it():
    health = FusionHealth()
    health.observe('visual', 10**9, 10**9, False)
    assert health.status(10**9)['status'] == 'initializing'
    assert 'visual' not in health.samples
    for kind in health.kinds:
        health.observe(kind, 1_100_000_000, 1_100_000_000, True)
    assert health.status(1_100_000_000)['status'] == 'tracking'


def test_missing_required_dependency_is_a_build_error(tmp_path):
    def missing(package):
        raise LookupError(package)
    with pytest.raises(RuntimeError, match='realsense2_camera'):
        check_dependencies(prefix_lookup=missing)


def test_camera_card_starts_shared_runtime_and_keeps_frame_cache(monkeypatch):
    spec = importlib.util.spec_from_file_location('bumi_camera_device_stubs',
                                                BUMI/'tests/test_loco_destination.py')
    fixture = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, fixture)
    spec.loader.exec_module(fixture)
    device, _ = fixture._load_device(monkeypatch)
    import camera_runtime
    instances = []
    class Runtime:
        def __init__(self, namespace, callback, enabled):
            self.namespace, self.callback, self.enabled = namespace, callback, enabled
            self.running = False
            instances.append(self)
        def start(self): self.running = True
        def stop(self): self.running = False
        def is_running(self): return self.running
    monkeypatch.setattr(camera_runtime, 'CameraRuntime', Runtime)
    nodes = []
    camera = device.CameraPlugin({}, 'robot', NS(add_node=nodes.append))
    cache = camera._frame_node
    camera.start()
    camera.start()
    assert len(instances) == 1
    assert instances[0].enabled is True
    assert instances[0].namespace == 'robot'
    assert camera.is_running()
    assert camera._frame_node is cache
    assert [t['topic_out'][0]['topic'] for t in camera.get_tools()] == [
        '/robot/camera/color', '/robot/camera/depth']
    camera.stop()
    assert not camera.is_running()
