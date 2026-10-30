"""In-process visual cards on the existing TRON2 MCP and ROS publisher.

Workers never touch the robot reader directly. All feedback and guarded
move_head dispatches run on the owning asyncio loop, as public MCP calls do.
"""
import asyncio
import threading

from head_follow.client import Client, require
from head_follow.engine import Engine as FollowEngine, tool as follow_tool
from yolo_detect.engine import Engine as YoloEngine, tool as yolo_tool, IMAGE_TOPIC, RESULT_TOPIC
from camera import CAMERA_SOURCES

CARDS = {'yolo_detect', 'head_visual_follow'}


def tools(enabled):
    return [factory() for name, factory in
            (('yolo_detect', yolo_tool), ('head_visual_follow', follow_tool)) if name in enabled]


def validate_enabled(enabled):
    if 'head_visual_follow' in enabled:
        require({'joint_state', 'camera', 'move_head', 'yolo_detect'} <= set(enabled),
                '头部跟随需要 joint_state、camera、move_head 和 yolo_detect')


class ImageBus:
    """Tap the same JPEG/stamp already published by the original camera card."""
    def __init__(self, publisher, enabled):
        self.publisher = publisher
        self.inputs = {source[0] for name, source in CAMERA_SOURCES.items() if name in enabled}
        self.lock = threading.RLock()
        self.binding = None

    def bind(self, topic, callback):
        require(topic in self.inputs, '所选相机未在 TRON2 驱动中启用')
        with self.lock:
            self.binding = (topic, callback)

    def unbind(self):
        with self.lock:
            self.binding = None

    def receive(self, topic, raw, stamp):
        with self.lock:
            binding = self.binding
        # Do not hold the bus lock while entering the engine lock. A stopped
        # generation discards a callback that was already copied here.
        if binding is not None and binding[0] == topic:
            binding[1](raw, stamp)

    def publish(self, image, result, stamp):
        self.publisher.publish(IMAGE_TOPIC, image, stamp=stamp)
        self.publisher.publish(RESULT_TOPIC, result)


class Runtime:
    def __init__(self, adapter, publisher, model=None):
        self.adapter = adapter
        self.loop = asyncio.get_running_loop()
        self.closed = False
        self.bus = ImageBus(publisher, adapter.enabled)
        self.yolo = YoloEngine(self.bus, model=model) if 'yolo_detect' in adapter.enabled else None
        self.follow = FollowEngine(Client(self.robot, self.detections, self.move)) \
            if 'head_visual_follow' in adapter.enabled else None

    def launch(self):
        if self.yolo:
            self.yolo.launch()
        if self.follow:
            self.follow.launch()

    def on_image(self, topic, raw, stamp):
        if not self.closed and stamp is not None:
            self.bus.receive(topic, raw, stamp)

    def _on_loop(self, operation, *args):
        require(not self.closed and not self.loop.is_closed(), '视觉控制正在关闭')
        # Never call this blocking bridge from the event-loop thread.
        try:
            current = asyncio.get_running_loop()
        except RuntimeError:
            current = None
        require(current is not self.loop, '视觉工作线程不可阻塞主控制循环')
        future = asyncio.run_coroutine_threadsafe(operation(*args), self.loop)
        # A timeout leaves the send uncertain. The follow engine records it
        # before this call and will neither resend nor automatically go home.
        return future.result(timeout=6)

    async def _robot(self):
        require(not self.closed, '视觉控制正在关闭')
        return self.adapter.health_data()

    def robot(self):
        return self._on_loop(self._robot)

    def detections(self):
        require(self.yolo is not None, '人物检测未启用')
        return self.yolo.info()

    async def _move(self, pitch, yaw, duration):
        require(not self.closed and self.follow is not None and self.follow.active and
                not self.follow.stop_requested.is_set(), '跟随已停用；不发送新的头部目标')
        # The identical validation, motion owner, collision/readiness guards
        # and ACP completion path used by the original public move_head card.
        return await self.adapter.execute_tool('move_head', {
            'action': 'move', 'pitch_deg': pitch, 'yaw_deg': yaw, 'time': duration})

    def move(self, pitch, yaw, duration):
        return self._on_loop(self._move, pitch, yaw, duration)

    async def call(self, name, arguments):
        require(not self.closed, '视觉控制正在关闭')
        engine = self.follow if name == 'head_visual_follow' else self.yolo if name == 'yolo_detect' else None
        require(engine is not None, '视觉卡片未启用')
        if engine is self.follow and arguments.get('action') == 'stop':
            self.follow.stop_requested.set()
        # GPU health probes, JPEG work and follow admission must never block
        # joint feedback, MCP serving or an in-flight completion verifier.
        try:
            return await asyncio.to_thread(engine.call, arguments)
        except (OSError, RuntimeError) as exc:
            # Start/probe failures are a normal MCP tool error, not a crashed
            # HTTP request. Follow.call only arms; actual sends run in step().
            raise ValueError(str(exc)[:240]) from exc

    def info(self):
        # Follow.info takes its control lock while a worker may wait for this
        # loop. Fetch both infos in a thread, never on the owning loop.
        return {name: engine.info() for name, engine in
                (('yolo_detect', self.yolo), ('head_visual_follow', self.follow)) if engine is not None}

    async def close(self):
        if self.closed:
            return
        self.closed = True
        if self.follow:
            self.follow.stop_requested.set()
            await asyncio.to_thread(self.follow.close)
        if self.yolo:
            await asyncio.to_thread(self.yolo.close)
