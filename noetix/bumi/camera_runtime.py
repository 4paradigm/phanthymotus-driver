"""One RealSense owner, legacy camera ports, and independently restartable fusion."""
from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import zlib

CAMERA = '/bumi_odom/bumi_odom_camera'


def camera_arguments():
    return ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
            'camera_name:=bumi_odom_camera', 'camera_namespace:=bumi_odom',
            'enable_color:=true', 'enable_depth:=true', 'enable_gyro:=true',
            'enable_accel:=true', 'unite_imu_method:=2', 'enable_sync:=true',
            'align_depth.enable:=true', 'rgb_camera.color_profile:=640x480x30',
            'depth_module.depth_profile:=640x480x30', 'pointcloud.enable:=false',
            'publish_tf:=true']


def _group_alive(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_process(proc):
    """Reap an entire ros2-launch process group before another USB owner starts."""
    if proc is None:
        return
    # A ros2 launch leader can die while its camera child still owns USB.
    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            proc.wait(timeout=2)
            return
        deadline = time.monotonic()+timeout
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pass
        while _group_alive(proc.pid) and time.monotonic() < deadline:
            time.sleep(0.02)
        if not _group_alive(proc.pid):
            return
    raise RuntimeError('camera/fusion process group did not exit; refusing a duplicate owner')


class CameraRuntime:
    def __init__(self, namespace, on_line, fusion_enabled=True):
        root = Path(__file__).resolve().parent
        self.commands = {'camera': camera_arguments(),
                         'bridge': [sys.executable, str(root/'camera_runtime.py'), 'bridge', namespace]}
        if fusion_enabled:
            self.commands['fusion'] = [sys.executable, str(root/'fusion_runtime.py')]
        self.on_line = on_line
        self.processes = {}
        self.retry_at = {}
        self.stopped = threading.Event()
        self.thread = None
        self.lock = threading.Lock()

    def tick(self, now):
        with self.lock:
            if self.stopped.is_set():
                return
            for name, args in self.commands.items():
                proc = self.processes.get(name)
                if proc is not None and proc.poll() is None:
                    continue
                if now < self.retry_at.get(name, 0):
                    continue
                if proc is not None:
                    try:
                        stop_process(proc)
                    except RuntimeError as exc:
                        # A group holding zombies may never reap (hardware
                        # case: pkill left a defunct leader). The duplicate-
                        # owner guard must stay, but supervision has to
                        # survive: report, leave the name for the next tick.
                        self.on_line(f'[camera/{name}] {exc}')
                        self.retry_at[name] = now+5
                        continue
                env = dict(os.environ)
                if name == 'fusion':
                    env.update(OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1')
                try:
                    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                            env=env, start_new_session=True)
                    self.processes[name] = proc
                    threading.Thread(target=self._forward, args=(proc,), daemon=True).start()
                except OSError as exc:
                    self.on_line(f'[camera/{name}] start failed: {exc}')
                self.retry_at[name] = now+5

    def _forward(self, proc):
        for line in proc.stdout:
            self.on_line(line.decode(errors='replace').rstrip())

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return
        self.stopped.clear()
        self.tick(time.monotonic())
        def watch():
            while not self.stopped.wait(1):
                self.tick(time.monotonic())
        self.thread = threading.Thread(target=watch, daemon=True, name='bumi_camera_watch')
        self.thread.start()

    def stop(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=2)
        with self.lock:
            for name in reversed(self.commands):
                stop_process(self.processes.get(name))
            self.processes.clear()

    def is_running(self):
        proc = self.processes.get('camera')
        bridge = self.processes.get('bridge')
        return proc is not None and proc.poll() is None and bridge is not None and bridge.poll() is None


def packed_image(msg, bytes_per_pixel):
    width, height, step = msg.width, msg.height, msg.step
    row = width*bytes_per_pixel
    data = bytes(msg.data)
    if width <= 0 or height <= 0 or step < row or len(data) != step*height:
        raise ValueError('invalid camera image dimensions, stride or data length')
    return b''.join(data[i*step:i*step+row] for i in range(height))


def compressed_depth(msg):
    if msg.encoding not in ('16UC1', 'mono16'):
        raise ValueError('aligned depth must contain Z16 millimetres')
    raw = packed_image(msg, 2)
    if msg.is_bigendian:
        from array import array
        values = array('H')
        values.frombytes(raw)
        values.byteswap()
        raw = values.tobytes()
    return zlib.compress(raw, 1)


def run_bridge(namespace):
    import json
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo, CompressedImage

    rclpy.init()
    node = Node('bumi_camera_compatibility')
    color_pub = node.create_publisher(CompressedImage, f'/{namespace}/camera/color', qos_profile_sensor_data)
    depth_pub = node.create_publisher(CompressedImage, f'/{namespace}/camera/depth', qos_profile_sensor_data)
    intrinsics = None

    def color(msg):
        try:
            if msg.encoding not in ('rgb8', 'bgr8'):
                raise ValueError('unsupported RGB encoding: '+msg.encoding)
            pixels = np.frombuffer(packed_image(msg, 3), dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            ok, encoded = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise ValueError('JPEG encoder failed')
            out = CompressedImage()
            out.header, out.format, out.data = msg.header, 'jpeg', encoded.tobytes()
            color_pub.publish(out)
        except (ValueError, cv2.error) as exc:
            node.get_logger().warning(str(exc))

    def depth(msg):
        try:
            out = CompressedImage()
            out.header, out.format, out.data = msg.header, '16UC1; compressedDepth zlib', compressed_depth(msg)
            depth_pub.publish(out)
        except ValueError as exc:
            node.get_logger().warning(str(exc))

    def info(msg):
        nonlocal intrinsics
        payload = dict(width=msg.width, height=msg.height, fx=msg.k[0], fy=msg.k[4],
                       ppx=msg.k[2], ppy=msg.k[5], model='brown_conrady', coeffs=list(msg.d))
        if payload != intrinsics:
            intrinsics = payload
            print('[camera_subprocess] INTRINSICS '+json.dumps(dict(color=payload, depth=payload)), flush=True)

    node.create_subscription(Image, CAMERA+'/color/image_raw', color, qos_profile_sensor_data)
    node.create_subscription(Image, CAMERA+'/aligned_depth_to_color/image_raw', depth, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, CAMERA+'/color/camera_info', info, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    from common import logsafe
    logsafe.install(check_fd=False)
    if len(sys.argv) != 3 or sys.argv[1] != 'bridge':
        raise SystemExit('usage: camera_runtime.py bridge ROS_NAMESPACE')
    run_bridge(sys.argv[2])
