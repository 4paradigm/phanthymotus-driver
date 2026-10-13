"""Passive live-color resource; camera lifecycle is independent of arm control."""
import math
from pathlib import Path
import threading
import time

from common.vendor_runtime import action_schema, tool
from agilex.piper.camera_owner import CameraOwner
from agilex.piper.camera_worker import CameraSession


def camera_settings(config):
    camera = config.get('camera', {})
    live = camera.get('live', {})
    settings = dict(enabled=live.get('enabled', False), device=camera.get('device') or '/dev/piper-camera-1',
                    width=live.get('width', 640), height=live.get('height', 480),
                    fps=live.get('fps', 10), jpeg_quality=live.get('jpeg_quality', 80),
                    warmup_seconds=camera.get('warmup_seconds', 3))
    if type(settings['enabled']) is not bool:
        raise ValueError('camera.live.enabled 必须是布尔值')
    if settings['device'] not in ('/dev/piper-camera-1', '/dev/piper-camera-2'):
        raise ValueError('实时相机必须使用已按彩色接口绑定的 /dev/piper-camera-1 或 /dev/piper-camera-2')
    for key, low, high in (('width', 320, 1280), ('height', 240, 720), ('fps', 1, 30), ('jpeg_quality', 1, 95)):
        if type(settings[key]) is not int or not low <= settings[key] <= high:
            raise ValueError(f'{key} 必须是 [{low}, {high}] 范围内的整数')
    if (settings['width'], settings['height']) not in ((320, 240), (640, 480), (1280, 720)):
        raise ValueError('相机分辨率只支持 320×240、640×480 或 1280×720')
    warmup = settings['warmup_seconds']
    if isinstance(warmup, bool) or not isinstance(warmup, (int, float)) or not math.isfinite(warmup) or not 0 <= warmup <= 10:
        raise ValueError('相机预热时间必须在 [0, 10] 秒内')
    return settings


def offline_image(width, height):
    import cv2
    import numpy as np
    image = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(image, 'OFFLINE', (width // 6, height // 2), cv2.FONT_HERSHEY_SIMPLEX,
                max(.7, width / 500), (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(image, 'No current camera frame', (16, height - 24), cv2.FONT_HERSHEY_SIMPLEX,
                .45, (160, 160, 160), 1, cv2.LINE_AA)
    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise RuntimeError('无法生成断流提示图')
    return jpeg.tobytes()


class ImageTransport:
    def __init__(self, ros2, topic):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import CompressedImage
        self.ros2, self.message_type = ros2, CompressedImage
        self.node = Node('piper_live_' + topic.strip('/').replace('/', '_'), context=ros2.ctx_core)
        self.attached = False
        try:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.publisher = self.node.create_publisher(CompressedImage, topic, qos)
            ros2.executor_core.add_node(self.node)
            self.attached = True
        except BaseException:
            self.close()
            raise

    def publish(self, jpeg, stamp):
        msg = self.message_type()
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        msg.header.frame_id = 'piper_camera_color_optical_frame'
        msg.format, msg.data = 'jpeg', jpeg
        self.publisher.publish(msg)

    def close(self):
        node, self.node = self.node, None
        if node is not None:
            try:
                if self.attached:
                    self.ros2.executor_core.remove_node(node)
            finally:
                node.destroy_node()


class PiperCameraLive:
    STALE_SECONDS = 2.
    RETRY_SECONDS = 2.

    def __init__(self, runtime, ros2=None, *, session_factory=CameraSession,
                 publisher_factory=ImageTransport, owner_factory=CameraOwner):
        self.runtime, self.ros2 = runtime, ros2
        self.settings = camera_settings(runtime.config)
        self.topic = runtime.topic.rsplit('/', 1)[0] + '/camera/live'
        self.session_factory, self.publisher_factory, self.owner_factory = session_factory, publisher_factory, owner_factory
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._state = 'idle'
        self._error = None
        self._frames = 0
        self._last_time = None
        self._width = self._height = None
        self._capture_fps = self._publish_fps = 0.
        self._publisher = None

    def get_tools(self):
        schema = action_schema({'start': ([], '自动开始实时监控'), 'stop': ([], '停止采集并释放相机'),
                                'info': ([], '查看相机状态')}, {})
        schema['additionalProperties'] = False
        definition = tool('piper_camera_live', 'resource',
                          '自动显示绑定彩色相机的当前画面；在顶部「监控」查看，不控制机械臂。', schema,
                          topic_out=[{'topic': self.topic, 'format': 'image/jpeg'}])
        definition['title'] = '实时相机画面'
        if getattr(self.runtime, 'interface_language', None) == 'en':
            definition['title'] = 'Live camera'
            definition['description'] = 'Current bound color camera view in Monitor; no arm commands.'
            for action, description in (('start', 'Start live monitoring'), ('stop', 'Stop and release the camera'),
                                        ('info', 'Read camera status')):
                schema['x-action-params'][action]['description'] = description
        return [definition]

    def info(self):
        labels = {'idle': '已停止', 'disabled': '未启用相机采集', 'starting': '相机启动中',
                  'running': '实时画面正常', 'offline': '相机断流，正在重试', 'stopping': '相机停止中'}
        with self._lock:
            result = dict(state=self._state, 状态=labels[self._state], 设备名称=self.runtime.config.get('name', '机械臂 1'),
                        实际分辨率=None if self._width is None else [self._width, self._height],
                        采集帧率=round(self._capture_fps, 2), 发布帧率=round(self._publish_fps, 2),
                        目标帧率=self.settings['fps'], 有效帧数=self._frames, 最后有效时间=self._last_time,
                        错误原因=self._error, publishing=self._publisher is not None,
                        topic_out=[{'topic': self.topic, 'format': 'image/jpeg'}])
            if getattr(self.runtime, 'interface_language', None) == 'en':
                from agilex.piper.intent_control import public_error
                keys = {'状态': 'status', '设备名称': 'device', '实际分辨率': 'resolution', '采集帧率': 'capture_fps',
                        '发布帧率': 'publish_fps', '目标帧率': 'target_fps', '有效帧数': 'frames',
                        '最后有效时间': 'last_valid_time', '错误原因': 'error'}
                result = {keys.get(key, key): value for key, value in result.items()}
                result['status'] = self._state
                result['device'] = 'PiPER ' + self.runtime.config.get('can_interface', 'can0')
                result['error'] = public_error(result['error']) if result['error'] else None
            return result

    def start(self):
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                if self._stop.is_set():
                    raise RuntimeError('相机采集进程仍在停止，请稍后再启动')
                return self.info()
            if not self.settings['enabled'] or not self.runtime.camera_enabled:
                self._state = 'disabled'
                return self.info()
            self._stop = threading.Event()  # A new generation never clears the old cancellation token.
            self._state, self._error = 'starting', None
            self._thread = threading.Thread(target=self._run, args=(self._stop,), daemon=True, name='piper-live-supervisor')
            self._thread.start()
            return self.info()

    def stop(self):
        with self._lock:
            self._stop.set()
            thread = self._thread
            if thread is not None and thread.is_alive():
                self._state = 'stopping'
        if thread is not None:
            thread.join(timeout=5)
        with self._lock:
            if thread is None or not thread.is_alive():
                self._state = 'idle'
        return self.info()

    def dispatch(self, action, args):
        english = getattr(self.runtime, 'interface_language', None) == 'en'
        if set(args) - {'action', 'instance_id', '_tool_name'}:
            raise ValueError('Live camera accepts lifecycle arguments only.' if english else '实时相机卡不接受控制或拍照参数')
        action = args.get('action', 'info' if action == 'piper_camera_live' else action)
        if action not in ('start', 'stop', 'info'):
            raise ValueError('Live camera supports start, stop and info only.' if english else '实时相机只支持启动、停止和状态查询')
        try:
            return getattr(self, action)()
        except RuntimeError as exc:
            if english:
                from agilex.piper.intent_control import public_error
                raise RuntimeError(public_error(exc)) from exc
            raise

    def _publish(self, stop, jpeg, stamp):
        # Serialize with cancellation: no frame can be published after stop wins.
        with self._lock:
            if stop.is_set():
                return False
            self._publisher.publish(jpeg, stamp)
            return True

    def _offline(self, stop, error):
        with self._lock:
            if stop.is_set():
                return
            self._state, self._error = 'offline', str(error).replace('\n', ' ')[:300]
            self._capture_fps = self._publish_fps = 0.
        try:
            self._publish(stop, offline_image(self.settings['width'], self.settings['height']), time.time())
        except Exception as exc:
            with self._lock:
                self._error += '; 发布异常: ' + str(exc).replace('\n', ' ')[:150]

    def _close_session(self, session, stop):
        # Ownership is kept until the child is confirmed dead, even if stop times out.
        while not session.close():
            with self._lock:
                self._state = 'stopping' if stop.is_set() else 'offline'
                self._error = '旧采集进程尚未退出，保留占用锁并等待释放'
            time.sleep(.1)
        session.dispose()

    def _run(self, stop):
        publisher = None
        try:
            if self.ros2 is None:
                raise RuntimeError('没有 ROS 发布通道，未打开相机')
            publisher = self.publisher_factory(self.ros2, self.topic)
            with self._lock:
                self._publisher = publisher
            while not stop.is_set():
                owner = session = None
                try:
                    owner = self.owner_factory(self.runtime.data_dir, self.settings['device'])
                    if stop.is_set():
                        break
                    if not Path(self.settings['device']).exists():
                        raise RuntimeError('绑定的彩色相机不存在，请检查 USB 连接')
                    session = self.session_factory(self.settings)
                    began = window = time.monotonic()
                    last_sequence = count = 0
                    last_frame = None
                    while not stop.wait(.02):
                        frame, error = session.read()
                        if error:
                            raise RuntimeError(error)
                        now = time.monotonic()
                        if frame is not None and frame['sequence'] != last_sequence:
                            stamp = frame['timestamp']
                            if not math.isfinite(stamp) or not 0 <= time.time() - stamp <= self.STALE_SECONDS:
                                raise RuntimeError('相机画面时间无效或已过期')
                            if not self._publish(stop, frame['jpeg'], stamp):
                                break
                            last_sequence, last_frame = frame['sequence'], now
                            count += 1
                            with self._lock:
                                self._state, self._error = 'running', None
                                self._width, self._height = frame['width'], frame['height']
                                self._capture_fps = frame['capture_fps']
                                self._frames += 1
                                self._last_time = stamp
                                if now - window >= 1:
                                    self._publish_fps = count / (now - window)
                                    count, window = 0, now
                        timeout = self.STALE_SECONDS if last_frame is not None else self.settings['warmup_seconds'] + 5
                        if now - (last_frame if last_frame is not None else began) > timeout:
                            raise RuntimeError('超过 2 秒没有新画面' if last_frame is not None else '相机启动等待画面超时')
                except Exception as exc:
                    self._offline(stop, exc)
                finally:
                    if session is not None:
                        self._close_session(session, stop)
                    if owner is not None:
                        owner.close()
                if not stop.is_set():
                    stop.wait(self.RETRY_SECONDS)
        except Exception as exc:
            with self._lock:
                self._state, self._error = 'offline', str(exc).replace('\n', ' ')[:300]
        finally:
            try:
                if publisher is not None:
                    publisher.close()
            finally:
                with self._lock:
                    self._publisher = None
                    self._capture_fps = self._publish_fps = 0.
                    if stop.is_set():
                        self._state = 'idle'


def add_camera_live(plugins, config, ros2):
    plugins = list(plugins)
    runtime = next(p for p in plugins if hasattr(p, 'camera_enabled') and hasattr(p, 'capture'))
    if any(t['name'] == 'piper_camera_live' for p in plugins for t in p.get_tools()):
        raise ValueError('实时相机已注册，拒绝重复采集')
    original_capture = runtime.capture
    def capture(args):
        device = config.get('camera', {}).get('device') or '/dev/piper-camera-1'
        with CameraOwner(runtime.data_dir, device):
            return original_capture(args)
    runtime.capture = capture
    return [*plugins, PiperCameraLive(runtime, ros2)]
