"""Calibrated RGB-D computation outputs, separate from dashboard camera crops.

The vendor must already express depth in the color optical reference frame.
Different intrinsics still require reprojection: sampling color along the depth
rays also removes its distortion. The resulting two 640x480 images share K.
"""
from collections import deque
from copy import deepcopy
import queue
import threading
import time
import zlib

import numpy as np


def make_geometry(color, depth, *, registered):
    if not registered or not color.header.frame_id or color.header.frame_id != depth.header.frame_id:
        raise ValueError('vendor-registered common optical reference is required')
    for info in (color, depth):
        k = np.asarray(info.k, float).reshape(3, 3)
        if (info.width <= 0 or info.height <= 0 or not np.isfinite(k).all() or
                k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]) or
                k[0, 1] != 0 or k[1, 0] != 0):
            raise ValueError('invalid source pinhole intrinsics')
    if np.any(np.asarray(depth.d) != 0):
        raise ValueError('registered depth must have rectified rays')
    if color.distortion_model not in ('plumb_bob', 'rational_polynomial') or len(color.d) not in (5, 8):
        raise ValueError('unsupported source color distortion')
    distortion = np.zeros(8)
    distortion[:len(color.d)] = color.d
    if not np.isfinite(distortion).all():
        raise ValueError('invalid color distortion')
    width, height = depth.width, depth.height
    crop_w, crop_h = min(width, height * 4 // 3), min(height, width * 3 // 4)
    left, top = (width - crop_w) // 2, (height - crop_h) // 2
    k = np.asarray(depth.k, float).reshape(3, 3).copy()
    k[0] *= 640 / crop_w
    k[1] *= 480 / crop_h
    k[0, 2] -= left * 640 / crop_w
    k[1, 2] -= top * 480 / crop_h
    u, v = np.meshgrid(np.arange(640), np.arange(480))
    x, y = (u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1]
    r2 = x*x + y*y
    k1, k2, p1, p2, k3, k4, k5, k6 = distortion
    radial = (1 + k1*r2 + k2*r2**2 + k3*r2**3) / (1 + k4*r2 + k5*r2**2 + k6*r2**3)
    xd = x*radial + 2*p1*x*y + p2*(r2 + 2*x*x)
    yd = y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y
    return dict(K=k, frame_id=depth.header.frame_id,
                color_x=(color.k[0]*xd + color.k[2]).astype(np.float32),
                color_y=(color.k[4]*yd + color.k[5]).astype(np.float32),
                depth_x=(depth.k[0]*x + depth.k[2]).astype(np.float32),
                depth_y=(depth.k[4]*y + depth.k[5]).astype(np.float32))


def camera_declarations(root, geometry, clock):
    common = dict(schema='motus.camera/1', id=root + '/rectified',
                  width=640, height=480, K=geometry['K'].ravel().tolist(), D=[0.] * 5,
                  distortion_model='pinhole', source='derived-from-K',
                  vendor=dict(frame_id=geometry['frame_id'], clock_id=clock,
                              registration='registered_rectified',
                              registration_source='orbbec_hw_registered_common_optical_rays',
                              timestamp_semantics='source_capture_mapped'))
    color = dict(deepcopy(common), topic=root + '/color', format='image/jpeg')
    depth = dict(deepcopy(common), topic=root + '/depth', format='image/depth-zlib')
    depth['vendor'].update(depth_unit='mm', depth_source='measured')
    return [color, depth]


def _stamp(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


class RgbdCameraPlugin:
    def __init__(self, config, namespace, ros2):
        from rclpy.node import Node
        self.root = '/' + namespace + '/camera/rgbd'
        self.clock = 'orbbec_head_global_ros_header'
        self.sub_node = Node('tianyi_rgbd_source', context=ros2.ctx_tianyi)
        self.pub_node = Node('tianyi_rgbd_output', context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self.sub_node)
        ros2.executor_core.add_node(self.pub_node)
        self.lock = threading.RLock()
        self.running = self.registered = self.attached = False
        self.geometry = None
        self.infos, self.signature = {}, None
        self.frames = {'color': deque(maxlen=3), 'depth': deque(maxlen=3)}
        self.pending = queue.Queue(maxsize=1)
        self.worker = None
        self.stop_event = threading.Event()
        self.last_emit = 0.
        self.count = 0
        self.error = None
        self.future = None

    def get_tool(self):
        return {'name': 'camera_rgbd', 'type': 'sensor',
                'description': '天轶配准 RGB-D 计算输入（原始采集时间、共同光学平面）',
                'inputSchema': {'type': 'object', 'properties': {}},
                'topic_out': self._outputs(), 'camera_info': self._declarations()}

    def _outputs(self):
        return [{'topic': self.root + '/color', 'format': 'image/jpeg'},
                {'topic': self.root + '/depth', 'format': 'image/depth-zlib'}]

    def _declarations(self):
        return camera_declarations(self.root, self.geometry, self.clock) if self.geometry is not None else []

    def start(self):
        from sensor_msgs.msg import Image, CameraInfo, CompressedImage
        from rclpy.qos import qos_profile_sensor_data
        from rcl_interfaces.srv import GetParameters
        with self.lock:
            if self.running:
                return
            if not self.attached:
                self.color_pub = self.pub_node.create_publisher(CompressedImage, self.root + '/color', qos_profile_sensor_data)
                self.depth_pub = self.pub_node.create_publisher(CompressedImage, self.root + '/depth', qos_profile_sensor_data)
                for kind in ('color', 'depth'):
                    self.sub_node.create_subscription(CameraInfo, '/ob_camera_head/' + kind + '/camera_info',
                        lambda m, k=kind: self._info(k, m), qos_profile_sensor_data)
                    self.sub_node.create_subscription(Image, '/ob_camera_head/' + kind + '/image_raw',
                        lambda m, k=kind: self._image(k, m), qos_profile_sensor_data)
                self.client = self.sub_node.create_client(GetParameters, '/ob_camera_head/ob_camera_head/get_parameters')
                self.sub_node.create_timer(.5, self._parameters)
                self.attached = True
            self.running = True
            self.registered = False
            self.geometry = None
            self.signature = None
            self.future = None
            self.error = None
            self.stop_event.clear()
            self.worker = threading.Thread(target=self._work, name='tianyi_rgbd_source', daemon=True)
            self.worker.start()

    def _parameters(self):
        from rcl_interfaces.srv import GetParameters
        with self.lock:
            if not self.running or self.registered:
                return
            if self.future is None:
                if self.client.service_is_ready():
                    request = GetParameters.Request()
                    request.names = ['depth_registration', 'time_domain', 'enable_sync_host_time']
                    self.future = self.client.call_async(request)
            elif self.future.done():
                try:
                    values = self.future.result().values
                    self.registered = (values[0].bool_value and values[1].string_value == 'global' and values[2].bool_value)
                    if not self.registered:
                        self.error = 'Orbbec requires registered depth and source global capture time'
                except Exception as error:
                    self.error = str(error)
                self.future = None

    def _info(self, kind, msg):
        with self.lock:
            self.infos[kind] = msg

    def _image(self, kind, msg):
        with self.lock:
            if not self.running or not self.registered or set(self.infos) != {'color', 'depth'}:
                return
            try:
                signature = tuple((tuple(self.infos[k].k), tuple(self.infos[k].d), self.infos[k].width,
                                   self.infos[k].height, self.infos[k].header.frame_id) for k in ('color', 'depth'))
                if signature != self.signature:
                    if self.signature is not None:
                        raise ValueError('source calibration changed; restart RGB-D source')
                    self.geometry = make_geometry(self.infos['color'], self.infos['depth'], registered=self.registered)
                    self.signature = signature
                if (msg.header.frame_id != self.geometry['frame_id'] or
                        msg.width != self.infos[kind].width or msg.height != self.infos[kind].height):
                    raise ValueError('source image geometry differs from calibration')
                self.frames[kind].append(msg)
                color, depth = self.frames['color'], self.frames['depth']
                while color and depth:
                    delta = _stamp(color[0]) - _stamp(depth[0])
                    if abs(delta) <= 20_000_000:
                        pair = color.popleft(), depth.popleft()
                        if time.monotonic() - self.last_emit >= .1:
                            if self.pending.full():
                                self.pending.get_nowait()
                            self.pending.put_nowait(pair)
                            self.last_emit = time.monotonic()
                        break
                    (color if delta < 0 else depth).popleft()
            except (ValueError, TypeError) as error:
                self.error = str(error)
                self.running = False
                self.stop_event.set()

    def _work(self):
        import cv2
        from sensor_msgs.msg import CompressedImage
        while not self.stop_event.is_set():
            try:
                color, depth = self.pending.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                if color.encoding not in ('rgb8', 'bgr8') or depth.encoding != '16UC1' or depth.is_bigendian:
                    raise ValueError('unsupported source RGB-D encoding')
                rgb = np.frombuffer(color.data, np.uint8).reshape(color.height, color.step)[:, :color.width*3].reshape(color.height, color.width, 3)
                if color.encoding == 'rgb8':
                    rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                z16 = np.frombuffer(depth.data, np.uint8).reshape(depth.height, depth.step)[:, :depth.width*2].copy().view('<u2').reshape(depth.height, depth.width)
                g = self.geometry
                rectified = cv2.remap(rgb, g['color_x'], g['color_y'], cv2.INTER_LINEAR)
                metric = cv2.remap(z16, g['depth_x'], g['depth_y'], cv2.INTER_NEAREST)
                visible = ((g['color_x'] >= 0) & (g['color_x'] < color.width-1) &
                           (g['color_y'] >= 0) & (g['color_y'] < color.height-1))
                metric[~visible] = 0
                ok, jpg = cv2.imencode('.jpg', rectified, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not ok:
                    raise ValueError('RGB-D color encoding failed')
                out_color, out_depth = CompressedImage(), CompressedImage()
                out_color.header, out_depth.header = deepcopy(color.header), deepcopy(depth.header)
                out_color.format, out_color.data = 'jpeg', jpg.tobytes()
                out_depth.format = '16UC1; compressedDepth zlib'
                out_depth.data = zlib.compress(metric.astype('<u2', copy=False).tobytes(), 1)
                with self.lock:
                    if not self.running or self.stop_event.is_set():
                        continue
                    self.color_pub.publish(out_color)
                    self.depth_pub.publish(out_depth)
                    self.count += 1
            except Exception as error:
                with self.lock:
                    self.error = str(error)
                    self.running = False
                    self.stop_event.set()

    def stop(self):
        with self.lock:
            self.running = False
            self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=5)
            if self.worker.is_alive():
                raise RuntimeError('RGB-D source worker did not stop')
        with self.lock:
            for frames in self.frames.values():
                frames.clear()
            while not self.pending.empty():
                self.pending.get_nowait()

    def dispatch(self, action, args):
        if action == 'start':
            self.start()
        elif action == 'stop':
            self.stop()
        with self.lock:
            return {'state': 'running' if self.running else 'idle', 'topic_out': self._outputs(),
                    'camera_info': self._declarations(), 'frames_forwarded': self.count,
                    'registered_source': self.registered, 'error': self.error}
