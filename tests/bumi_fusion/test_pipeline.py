from pathlib import Path
import pytest
from noetix.bumi_odometry.pipeline import parameters, ChildProcesses


def test_visual_and_ekf_use_unique_segment_frames_and_no_imu_double_count():
    visual, ekf = parameters('robot_segment', (.1, -.2), '/robot/imu')
    assert visual['frame_id'] == 'robot_segment_base'
    assert visual['publish_tf'] is False and visual['wait_imu_to_init'] is False
    assert ekf['base_link_frame'] == 'robot_segment_base'
    assert ekf['imu0'] == '/robot/imu'
    assert ekf['imu0_config'] == [False]*9+[True]*3+[False]*3
    assert ekf['two_d_mode'] is False and ekf['use_control'] is False
    assert ekf['smooth_lagged_data'] is True


def test_second_child_start_failure_reaps_first_child(tmp_path):
    calls, reaped = [], []
    class Process:
        pid=10
        def poll(self): return None
    def popen(args, **kw):
        calls.append(args)
        if len(calls) == 2: raise OSError('EKF executable unavailable')
        return Process()
    manager=ChildProcesses(tmp_path, popen=popen, stop=reaped.append,
                           prefix=lambda package: Path('/opt/ros/humble'))
    with pytest.raises(OSError): manager.start('s', (.0,.0), '/s/imu')
    assert len(reaped) == 1 and not manager.children
