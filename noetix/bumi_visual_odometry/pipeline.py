"""RTAB-Map ``rgbd_odometry`` launch contract, without camera ownership.

Visual odometry only: one child process, one estimator. PR #425 started a second
robot_localization EKF child and fused the body gyro into it; that whole branch
is intentionally absent here.
"""
from pathlib import Path
import os
import signal
import subprocess

PACKAGE = 'rtabmap_odom'
EXECUTABLE = 'rgbd_odometry'


def parameters(segment, frame_id):
    """Parameters for the standalone visual estimator.

    ``frame_id`` is the optical frame the shared images are published in, so the
    estimator never needs a TF lookup to a body frame that has not been measured
    yet. ``publish_tf`` stays false: this sensor is read-only and must not fight
    the robot's existing TF tree.
    """
    return dict(frame_id=frame_id, odom_frame_id=segment+'_odom', publish_tf=False,
                wait_imu_to_init=False, approx_sync=True, approx_sync_max_interval=.02,
                qos=2, qos_camera_info=2, topic_queue_size=5, sync_queue_size=5,
                **{'Odom/ResetCountdown': '0'})


def stop_group(proc):
    for sig, timeout in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            proc.wait(timeout=2)
            return
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        # Reap remaining descendants even if the ROS executable leader died.
        try:
            os.killpg(proc.pid, 0)
        except ProcessLookupError:
            return
    raise RuntimeError('visual odometry process group did not exit')


class ChildProcesses:
    """The one RTAB-Map child; stopping it never touches the camera process."""

    def __init__(self, directory, *, popen=None, stop=None, prefix=None):
        self.directory = Path(directory)
        self.popen, self.reap = popen or subprocess.Popen, stop or stop_group
        self.prefix = prefix
        self.children = []

    def start(self, segment, frame_id):
        import yaml
        if self.children:
            raise RuntimeError('old visual odometry process still present')
        if self.prefix is None:
            from ament_index_python.packages import get_package_prefix
            self.prefix = get_package_prefix
        config = parameters(segment, frame_id)
        root = '/'+segment
        path = self.directory/'odometry.yaml'
        try:
            path.write_text(yaml.safe_dump({'/**': {'ros__parameters': config}}))
            args = [str(Path(self.prefix(PACKAGE))/'lib'/PACKAGE/EXECUTABLE), '--ros-args',
                    '-r', '__node:=rgbd_odometry', '-r', '__ns:='+root,
                    '--params-file', str(path)]
            for source, target in (('rgb/image', root+'/rgb'),
                                   ('depth/image', root+'/depth'),
                                   ('rgb/camera_info', root+'/info'),
                                   ('odom', root+'/visual'),
                                   ('odom_info', root+'/diagnostic'),
                                   # No IMU is fused; keep an unused remap so an
                                   # implicit subscription cannot pick a live topic.
                                   ('imu', root+'/unused_imu')):
                args.extend(['-r', source+':='+target])
            env = dict(os.environ, OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1')
            self.children.append(self.popen(args, env=env, start_new_session=True))
        except Exception:
            self.stop()
            raise

    def stop(self):
        failures = []
        for proc in reversed(self.children):
            try:
                self.reap(proc)
            except Exception as exc:
                failures.append(exc)
        if failures:
            raise RuntimeError('cannot reap visual odometry children: '+str(failures))
        self.children.clear()

    def failed(self):
        return any(p.poll() is not None for p in self.children)
