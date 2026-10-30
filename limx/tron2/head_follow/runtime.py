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
        self.follow_lock = asyncio.Lock()
        self.session = None
        self.session_task = None
        self.engine = Engine(Client(self.robot, self.detections, self.move), clock, wall)
        self.adapter.head.follow_guard = self.before_send

    def launch(self):
        self.engine.launch()

    def session_update(self, session, **fields):
        with self.data_lock:
            session.update(fields)

    def follow_result(self, session):
        return dict(self.info(), action_id=session['action_id'],
                    completion_protocol='acp', completion_pending=True)

    async def complete_session(self, session, status, reason, snapshot):
        if session['state'] != 'running':
            return
        # Commit the one terminal result before awaiting HTTP. A stop/shutdown
        # concurrent with reporting must not send a different terminal payload.
        result = {'action':'follow','reason':reason,'completion_protocol':'acp',
            'head_commands_sent':snapshot['head_commands_sent']-session['initial_head_commands'],
            'motion_sent':snapshot['head_commands_sent']>session['initial_head_commands'] or snapshot['send_result_unknown'],
            'motion_may_continue':bool(snapshot['head_motion'] or snapshot['send_result_unknown'] or snapshot['error'] or self.adapter.head.active),
            'send_result_unknown':snapshot['send_result_unknown'],
            'last_head_motion':copy.deepcopy(snapshot['head_motion']),
            'error':snapshot['error'], 'automatic_home_sent_by_completion':False}
        self.session_update(session, state='reporting', status=status, result=result)
        try:
            delivered = await self.adapter._report_completion(
                'head_visual_follow', session['action_id'], status, result)
        except Exception:
            delivered = False
        self.session_update(session, state='settled' if delivered else 'blocked_reporting',
                            callback_delivered=delivered, finished_at=self.wall())

    async def watch_session(self, session):
        # Let the MCP response return so Core can register the session before
        # an immediate stop/fault. The existing callback retries cover races.
        await asyncio.sleep(.1)
        while not self.closed and session['state']=='running':
            snapshot = await asyncio.to_thread(self.engine.info)
            if snapshot['error'] or snapshot['send_result_unknown'] and not snapshot['active']:
                await self.complete_session(session,'error','control_fault',snapshot)
                return
            if not snapshot['active'] and snapshot['head_motion'] is None:
                await self.complete_session(session,'cancelled','operator_stop',snapshot)
                return
            await asyncio.sleep(.1)

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
        # Empty arguments are the one public Execute operation. Lifecycle
        # actions remain internal and never produce a top-level action_id.
        action = args.get('action','follow')
        clean = {k:v for k,v in args.items() if k not in ('input_topic','input_topics')}
        clean.setdefault('action',action)
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
            async with self.follow_lock:
                snapshot = await asyncio.to_thread(self.engine.info)
                if self.session is not None:
                    if self.session['state']=='running' and snapshot['active'] and not self.engine.stop_requested.is_set():
                        return self.follow_result(self.session)
                    require(self.session['state']=='settled', '上次跟随会话尚未完成或回调未确认；请核对状态')
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
                admission = asyncio.create_task(asyncio.to_thread(self.engine.call, clean))
                try:
                    await asyncio.shield(admission)
                except asyncio.CancelledError:
                    # Losing the MCP caller must not leave an untracked enable
                    # running in a worker after the coroutine has disappeared.
                    self.engine.stop_requested.set()
                    try:
                        await admission
                    finally:
                        await asyncio.to_thread(self.engine.call, {'action':'stop'})
                    raise
                session = {'action_id':'tron2_follow_'+uuid.uuid4().hex, 'state':'running',
                    'status':None, 'callback_delivered':False, 'result':None, 'finished_at':None,
                    'started_at':self.wall(), 'initial_head_commands':snapshot['head_commands_sent']}
                with self.data_lock:
                    self.session = session
                self.session_task = asyncio.create_task(self.watch_session(session))
                return self.follow_result(session)
        await asyncio.to_thread(self.engine.call, clean)
        return self.info()

    def info(self):
        snapshot = self.engine.info()
        with self.data_lock:
            session = copy.deepcopy(self.session)
        return dict(snapshot, vop_subscribed=self.bound, session=session)

    async def close(self):
        if self.closed:
            return
        self.engine.stop_requested.set()
        await self.call({'action':'stop'})
        if self.session is not None:
            await self.complete_session(self.session,'cancelled','driver_shutdown',
                                        await asyncio.to_thread(self.engine.info))
        if self.session_task is not None:
            # A reporting callback has a finite timeout and must not be
            # cancelled midway and then retried as a different terminal state.
            if self.session_task.done() or self.session['state']!='running':
                await self.session_task
        self.closed = True
        await asyncio.to_thread(self.engine.close)
