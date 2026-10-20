"""One supervised pyrealsense2 owner for the existing color/depth cards."""
from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def _group_alive(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_process(proc):
    """Reap the capture process group before another USB owner starts."""
    if proc is None:
        return
    # Keep the same no-overlap guard across capture restarts.
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
    raise RuntimeError('camera process group did not exit; refusing a duplicate owner')


class CameraRuntime:
    def __init__(self, namespace, on_line):
        root = Path(__file__).resolve().parent
        self.commands = {'camera': [sys.executable, str(root/'camera_runtime.py'), 'capture', namespace]}
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
        return proc is not None and proc.poll() is None


def capture(namespace):
    """Original direct RGB/Z16 capture, with aligned depth and bounded cleanup."""
    from common import logsafe
    logsafe.install(check_fd=False)
    import json
    import signal
    import zlib
    import numpy as np
    import cv2
    import pyrealsense2 as rs
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage

    try:
        from turbojpeg import TurboJPEG, TJPF_BGR
        encoder = TurboJPEG()
        def encode(image):
            return encoder.encode(image, pixel_format=TJPF_BGR, quality=80)
    except Exception:
        def encode(image):
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise RuntimeError('JPEG encoder failed')
            return encoded.tobytes()

    rclpy.init()
    node = Node('bumi_camera_sub')
    pipeline = rs.pipeline()
    # Let SIGTERM run the finally block after wait_for_frames returns (<=1s),
    # rather than leaving device release entirely to process teardown.
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    previous_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        color_pub = node.create_publisher(CompressedImage, f'/{namespace}/camera/color', qos)
        depth_pub = node.create_publisher(CompressedImage, f'/{namespace}/camera/depth', qos)
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipeline.start(config)
        align = rs.align(rs.stream.color)
        try:
            streams = {}
            for name, stream in (('color', rs.stream.color), ('depth_sensor', rs.stream.depth)):
                intr = pipeline.get_active_profile().get_stream(stream).as_video_stream_profile().intrinsics
                streams[name] = dict(width=intr.width, height=intr.height, fx=intr.fx, fy=intr.fy,
                                     ppx=intr.ppx, ppy=intr.ppy,
                                     model=str(intr.model).rsplit('.', 1)[-1], coeffs=list(intr.coeffs))
            # rs.align publishes depth in the color grid, so advertise that lens.
            streams['depth'] = dict(streams['color'])
            print('[camera_subprocess] INTRINSICS '+json.dumps(streams), flush=True)
        except Exception as exc:
            print(f'[camera_subprocess] intrinsics unavailable: {exc}', flush=True)

        count, started = 0, time.monotonic()
        while rclpy.ok():
            frames = align.process(pipeline.wait_for_frames(timeout_ms=1000))
            color = frames.get_color_frame()
            if color:
                msg = CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.format, msg.data = 'jpeg', encode(np.asanyarray(color.get_data()))
                color_pub.publish(msg)
            depth = frames.get_depth_frame()
            if depth:
                msg = CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.format = '16UC1; compressedDepth zlib'
                msg.data = zlib.compress(np.asanyarray(depth.get_data()).tobytes(), 1)
                depth_pub.publish(msg)
            count += 1
            if count % 300 == 0:
                print(f'[camera_subprocess] {count} frames, {count/(time.monotonic()-started):.1f} fps',
                      flush=True)
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'[camera_subprocess] error: {exc}', flush=True)
    finally:
        try:
            pipeline.stop()
        except RuntimeError:
            pass  # start() may have failed before taking ownership.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)


if __name__ == '__main__':
    from common import logsafe
    logsafe.install(check_fd=False)
    if len(sys.argv) != 3 or sys.argv[1] != 'capture':
        raise SystemExit('usage: camera_runtime.py capture ROS_NAMESPACE')
    capture(sys.argv[2])
