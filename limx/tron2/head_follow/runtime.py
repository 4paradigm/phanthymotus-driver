"""VOP subscription and guarded in-process integration for this TRON2 driver."""
import asyncio
import copy
import json
import math
import threading
import time
import uuid
from .client import Client, require, pose
from .engine import Engine, VOP_TOPIC, parse_frame
from vendor import motion_readiness

PREPARE_SECONDS = 1.5


class Runtime:
    def __init__(self, adapter, publisher, clock=time.monotonic, wall=time.time):
        self.adapter, self.publisher = adapter, publisher
        self.loop = asyncio.get_running_loop()
        self.clock, self.wall = clock, wall
        self.closed = False
        self.bound = False
        self.generation = 0
        self.subscription = None
        self.data_lock = threading.RLock()
        self.packet = None
        self.send_guard = None
        self.engine = Engine(Client(self.robot, self.detections, self.move), clock, wall)
        self.adapter.head.follow_guard = self.before_send

    def launch(self):
        self.engine.launch()

    def receive(self, raw, generation):
        if not isinstance(raw, str) or len(raw) > 256 * 1024:
            data = None
        else:
            try:
                data = json.loads(raw)
            except (ValueError, TypeError):
                data = None
        with self.data_lock:
            if self.closed or not self.bound or generation != self.generation:
                return
            self.packet = {'active':True,'input_topic':VOP_TOPIC,'received_at':self.wall(),'data':data}

    def detections(self):
        with self.data_lock:
            if self.closed or not self.bound or self.packet is None:
                return {'active':False,'input_topic':VOP_TOPIC}
            return copy.deepcopy(self.packet)

    def _bridge(self, operation, *args):
        require(not self.closed and not self.loop.is_closed(), '头部跟随正在关闭')
        try:
            current = asyncio.get_running_loop()
        except RuntimeError:
            current = None
        require(current is not self.loop, '工作线程不能阻塞TRON2控制循环')
        future = asyncio.run_coroutine_threadsafe(operation(*args), self.loop)
        # The engine records an unknown send before calling move. A timeout
        # never cancels/resends a possibly accepted native action.
        return future.result(timeout=6)

    async def _robot(self):
        require(not self.closed, '头部跟随正在关闭')
        health = self.adapter.health_data()
        status = self.adapter.reader.sample('robot_status')
        if status.get('available') is not True or not motion_readiness(status.get('data') or {})['ready']:
            health['ready'] = False
        return health

    def robot(self):
        return self._bridge(self._robot)

    async def _move(self, pitch, yaw, duration):
        require(not self.closed and self.bound and self.engine.active and not self.engine.stop_requested.is_set(),
                '跟随已停用；不发送新目标')
        p = pose(await self._robot())
        require(p['head_id'] == self.engine.owner_id, '其它头部指令已改变控制权')
        require(self.engine.arm_reference is not None and
                max(abs(a-b) for a,b in zip(p['q'][:14],self.engine.arm_reference)) <= math.radians(.5),
                '机械臂参考姿态已改变；不发送跟随目标')
        frame = parse_frame(self.detections(), self.wall())
        token = uuid.uuid4().hex
        self.send_guard = {'token':token,'generation':self.generation,'q':list(p['q']),
            'head_id':p['head_id'],'expires':self.clock()+1.,'observation_at':frame['received']}
        try:
            return await self.adapter.execute_tool('move_head', {'action':'move',
                'pitch_deg':pitch,'yaw_deg':yaw,'time':duration}, follow_guard=token)
        finally:
            self.send_guard = None

    def move(self, pitch, yaw, duration):
        return self._bridge(self._move, pitch, yaw, duration)

    def before_send(self, token):
        g = self.send_guard
        require(not self.closed and self.bound and g is not None and g['token'] == token and
                g['generation'] == self.generation and not self.engine.stop_requested.is_set() and
                self.engine.active and self.clock() <= g['expires'] and
                0 <= self.wall()-g['observation_at'] <= 1.5, '跟随已停止、指令或观测过期；未发送')
        job = self.adapter.head.active
        require(job is not None and job['details'].get('head_follow_guard') == token,
                '头部动作所有权改变，未发送')
        p = pose(self.adapter.health_data(), stationary=False)
        require(max(abs(a-b) for a,b in zip(p['q'],g['q'])) <= math.radians(.2) and
                (self.adapter.head.last or {}).get('action_id') == g['head_id'],
                '模式核对期间参考姿态或头部控制权变化，未发送')

    def bind(self):
        if self.bound:
            return
        self.generation += 1
        generation = self.generation
        sub = self.publisher.subscribe_json(VOP_TOPIC, lambda raw:self.receive(raw,generation))
        with self.data_lock:
            self.subscription = sub
            self.bound = True
            self.packet = None

    async def call(self, args):
        require(not self.closed, '头部跟随正在关闭')
        action = args.get('action','info')
        clean = {k:v for k,v in args.items() if k not in ('input_topic','input_topics')}
        require(not set(clean)-{'action','instance_id','_trace_id'}, '不支持的跟随参数')
        if action in ('start','follow'):
            # No canvas wire is needed. Legacy matching metadata is harmless;
            # callers cannot select a different camera, topic or endpoint.
            topics = args.get('input_topics') or []
            require(isinstance(topics,list) and all(isinstance(t,str) for t in topics), 'VOP输入话题无效')
            if args.get('input_topic'):
                topics = [args['input_topic'], *topics]
            require(all(topic == VOP_TOPIC for topic in topics), '本卡片独立读取TRON2头部VOP，不接受其它输入话题')
            self.bind()
        if action == 'stop':
            self.engine.stop_requested.set()
            with self.data_lock:
                self.generation += 1
                self.bound = False
                self.packet = None
                sub, self.subscription = self.subscription, None
            if sub is not None:
                self.publisher.unsubscribe(sub)
        if action == 'follow':
            generation = self.generation
            deadline = self.loop.time()+PREPARE_SECONDS
            while True:
                require(not self.closed and self.bound and generation == self.generation, '跟随准备已取消')
                try:
                    parse_frame(self.detections(), self.wall())
                    break
                except ValueError:
                    require(self.loop.time()<deadline, '等待头部VOP的新鲜人物检测；请先开启camera→VOP')
                    await asyncio.sleep(.05)
        return await asyncio.to_thread(self.engine.call, clean)

    def info(self):
        return dict(self.engine.info(), vop_subscribed=self.bound)

    async def close(self):
        if self.closed:
            return
        self.engine.stop_requested.set()
        await self.call({'action':'stop'})
        self.closed = True
        await asyncio.to_thread(self.engine.close)
