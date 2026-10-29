"""The launch contract: one RTAB-Map visual estimator, no EKF child."""
from pathlib import Path

import pytest

from noetix.bumi.visual_odometry import ChildProcesses, parameter_file, parameters


def test_parameters_are_visual_only():
    visual = parameters('seg', 'robot_rgb_optical')
    assert visual['frame_id'] == 'robot_rgb_optical'
    assert visual['odom_frame_id'] == 'seg_odom'
    assert visual['publish_tf'] is False
    assert visual['wait_imu_to_init'] is False
    assert visual['approx_sync'] is True and visual['approx_sync_max_interval'] == .02
    assert visual['Odom/ResetCountdown'] == '0'
    # None of the EKF/fusion parameters may come back.
    for key in ('imu0', 'odom0', 'world_frame', 'two_d_mode', 'odom_frame',
                'smooth_lagged_data', 'imu0_config'):
        assert key not in visual


def test_start_launches_exactly_one_visual_node_with_private_topics(tmp_path):
    calls, processes = [], []

    class Process:
        pid = 10

        def poll(self):
            return None

    def popen(args, **kw):
        calls.append(args)
        processes.append(Process())
        return processes[-1]

    manager = ChildProcesses(tmp_path, popen=popen, stop=lambda p: None,
                             prefix=lambda package: Path('/opt/ros/humble'))
    manager.start('seg', 'robot_rgb_optical')
    assert len(calls) == 1 and len(manager.children) == 1
    args = calls[0]
    assert str(args[0]) == '/opt/ros/humble/lib/rtabmap_odom/rgbd_odometry'
    joined = ' '.join(str(v) for v in args)
    assert 'ekf_node' not in joined and 'robot_localization' not in joined
    for remap in ('rgb/image:=/seg/rgb', 'depth/image:=/seg/depth',
                  'rgb/camera_info:=/seg/info', 'odom:=/seg/visual',
                  'odom_info:=/seg/diagnostic', 'imu:=/seg/unused_imu'):
        assert remap in joined
    written = (tmp_path/'odometry.yaml').read_text()
    assert 'frame_id: robot_rgb_optical' in written
    assert 'odom_frame_id: seg_odom' in written
    written_lower = written.lower()
    assert 'imu0' not in written_lower and 'odom0' not in written_lower
    # The params file is hand-rendered; it must still parse and must not turn
    # the string "0" into an int parameter.
    import yaml
    parsed = yaml.safe_load(written)['/**']['ros__parameters']
    assert parsed['frame_id'] == 'robot_rgb_optical'
    assert parsed['odom_frame_id'] == 'seg_odom'
    assert parsed['publish_tf'] is False
    assert parsed['Odom/ResetCountdown'] == '0'
    assert parsed['approx_sync_max_interval'] == .02


def test_start_failure_propagates_and_reaps_nothing_left(tmp_path):
    def popen(args, **kw):
        raise OSError('rgbd_odometry executable unavailable')

    manager = ChildProcesses(tmp_path, popen=popen, stop=lambda p: None,
                             prefix=lambda package: Path('/opt/ros/humble'))
    with pytest.raises(OSError):
        manager.start('s', 'opt')
    assert manager.children == []


def test_second_start_is_refused_and_stop_reaps(tmp_path):
    reaped = []

    class Process:
        pid = 11

        def poll(self):
            return None

    manager = ChildProcesses(tmp_path, popen=lambda args, **kw: Process(),
                             stop=reaped.append, prefix=lambda package: Path('/x'))
    manager.start('s', 'opt')
    with pytest.raises(RuntimeError):
        manager.start('s', 'opt')
    manager.stop()
    assert len(reaped) == 1 and manager.children == []


def test_failed_detects_a_vanished_child(tmp_path):
    class Process:
        pid = 12
        dead = False

        def poll(self):
            return 1 if self.dead else None

    process = Process()
    manager = ChildProcesses(tmp_path, popen=lambda args, **kw: process,
                             stop=lambda p: None, prefix=lambda package: Path('/x'))
    manager.start('s', 'opt')
    assert manager.failed() is False
    process.dead = True
    assert manager.failed() is True
