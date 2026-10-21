"""RTAB-Map and robot_localization launch contract, without camera ownership."""
from pathlib import Path
import os
import signal
import subprocess


def parameters(segment, initial_rp, imu_topic):
    base, odom = segment+'_base', segment+'_odom'
    roll, pitch = initial_rp
    visual = dict(frame_id=base, odom_frame_id=odom, publish_tf=False,
                  wait_imu_to_init=False, approx_sync=True, approx_sync_max_interval=.02,
                  qos=2, qos_camera_info=2, topic_queue_size=5, sync_queue_size=5,
                  initial_pose=f'0 0 0 {roll} {pitch} 0', **{'Odom/ResetCountdown':'0'})
    ekf = dict(frequency=30., sensor_timeout=.25, two_d_mode=False, publish_tf=True,
               world_frame=odom, odom_frame=odom, base_link_frame=base, map_frame=segment+'_map',
               odom0='/'+segment+'/visual_validated', odom0_config=[True]*6+[False]*9,
               odom0_differential=False, odom0_relative=False, odom0_queue_size=10,
               imu0=imu_topic, imu0_config=[False]*9+[True]*3+[False]*3,
               imu0_differential=False, imu0_queue_size=50,
               imu0_remove_gravitational_acceleration=False, use_control=False,
               smooth_lagged_data=True, history_length=1., print_diagnostics=True)
    return visual, ekf


def stop_group(proc):
    for sig, timeout in ((signal.SIGINT,3), (signal.SIGTERM,2), (signal.SIGKILL,2)):
        try: os.killpg(proc.pid, sig)
        except ProcessLookupError:
            proc.wait(timeout=2); return
        try: proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired: continue
        # Reap remaining descendants even if the ROS executable leader died.
        try: os.killpg(proc.pid, 0)
        except ProcessLookupError: return
    raise RuntimeError('fusion process group did not exit')


class ChildProcesses:
    def __init__(self, directory, *, popen=None, stop=None, prefix=None):
        self.directory = Path(directory)
        self.popen, self.reap = popen or subprocess.Popen, stop or stop_group
        self.prefix = prefix
        self.children = []

    def start(self, segment, initial_rp, imu_topic):
        import yaml
        if self.children:
            raise RuntimeError('old fusion processes still present')
        if self.prefix is None:
            from ament_index_python.packages import get_package_prefix
            self.prefix = get_package_prefix
        visual, ekf = parameters(segment, initial_rp, imu_topic)
        root='/'+segment
        try:
            for name, package, executable, config, remaps in (
                ('visual','rtabmap_odom','rgbd_odometry',visual,
                 [('rgb/image',root+'/rgb'),('depth/image',root+'/depth'),
                  ('rgb/camera_info',root+'/info'),('odom',root+'/visual'),
                  ('odom_info',root+'/diagnostic'),('imu',root+'/unused_imu')]),
                ('ekf','robot_localization','ekf_node',ekf,[('odometry/filtered',root+'/filtered')])):
                path=self.directory/(name+'.yaml')
                path.write_text(yaml.safe_dump({'/**':{'ros__parameters':config}}))
                args=[str(Path(self.prefix(package))/'lib'/package/executable), '--ros-args',
                      '-r','__node:='+name,'-r','__ns:='+root,'--params-file',str(path)]
                for source,target in remaps: args.extend(['-r',source+':='+target])
                env=dict(os.environ, OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1')
                self.children.append(self.popen(args,env=env,start_new_session=True))
        except Exception:
            self.stop(); raise

    def stop(self):
        failures=[]
        for proc in reversed(self.children):
            try: self.reap(proc)
            except Exception as exc: failures.append(exc)
        if failures: raise RuntimeError('cannot reap fusion children: '+str(failures))
        self.children.clear()

    def failed(self):
        return any(p.poll() is not None for p in self.children)
