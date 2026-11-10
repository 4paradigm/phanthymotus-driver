"""TRON2 stream-shaped dual-arm and head control; R1 loco_servo's contract, TRON2 ServoJ.

One input wire, ControlSink, lifecycle/pause hooks, explicit owner handover.
ROS callbacks only overwrite a bounded mailbox; all state and motor output run
on the existing asyncio loop. 30 Hz input and target 300 Hz vendor output are
separate rates. No catch-up bursts, automatic home or success ACK fabrication.
"""
import asyncio
import json
import math
import os
from pathlib import Path
import threading
import time

from common.control import ControlSink, parse_descriptor
from skeleton import skeleton_stream, JOINT_NAMES
from vendor import ARM_LOWER, ARM_UPPER, HEAD_RAD_LIMITS, motion_readiness, reject_constant

SERVO_LOWER = tuple(ARM_LOWER) + tuple(low for low, _ in HEAD_RAD_LIMITS)
SERVO_UPPER = tuple(ARM_UPPER) + tuple(high for _, high in HEAD_RAD_LIMITS)

DEFAULT_EXPECTED_HZ = 30.0
MAX_HZ = 50.0
OUTPUT_HZ = 300
WATCHDOG_MS = 200
MAX_OBS_AGE_MS = 300
MAX_FEEDBACK_AGE = .35
MAX_OUTPUT_GAP = .05
MAX_VELOCITY = .5  # rad/s, initial application envelope, not motor maximum
MAX_ACCELERATION = 1.5  # rad/s²
TRACKING_LIMIT = math.radians(8)


def build_descriptor(expected_hz=DEFAULT_EXPECTED_HZ):
    return {'control_interface': 'motus.control/1', 'mode': 'joint_position',
        'dof': 16, 'joint_names': list(JOINT_NAMES),
        'units': {'angle': 'rad', 'time': 's'}, 'frame': 'base_Link',
        'groups': [{'name': 'arm_l', 'offset': 0, 'count': 7, 'unit': 'rad', 'resource': 'arm_l'},
                   {'name': 'arm_r', 'offset': 7, 'count': 7, 'unit': 'rad', 'resource': 'arm_r'},
                   {'name': 'head', 'offset': 14, 'count': 2, 'unit': 'rad', 'resource': 'head'}],
        'limits': {'lower': list(SERVO_LOWER), 'upper': list(SERVO_UPPER),
            'max_velocity': [MAX_VELOCITY] * 16,
            'max_delta_per_step': [MAX_VELOCITY / expected_hz] * 16},
        'rate': {'expected_hz': expected_hz, 'max_hz': MAX_HZ,
            'watchdog_ms': WATCHDOG_MS, 'max_obs_age_ms': MAX_OBS_AGE_MS},
        'force_torque': None, 'urdf_ref': 'model'}


class Limiter:
    """Time-based slew from measured pose, including the very first target."""
    def __init__(self, q):
        if len(q) != 16 or any(not math.isfinite(v) or not low <= v <= high
                for v, low, high in zip(q, SERVO_LOWER, SERVO_UPPER)):
            raise ValueError('接管关节角超出厂家范围，不能从该姿态生成伺服轨迹')
        self.q, self.v = list(q), [0.] * 16

    def step(self, target, dt):
        if not 0 < dt <= MAX_OUTPUT_GAP:
            raise ValueError('伺服输出延迟过大，不追发积压目标')
        for i, goal in enumerate(target):
            # Critically damped position tracking; no abrupt snap at the goal.
            acceleration = max(-MAX_ACCELERATION, min(MAX_ACCELERATION,
                16. * (goal - self.q[i]) - 8. * self.v[i]))
            velocity = max(-MAX_VELOCITY, min(MAX_VELOCITY, self.v[i] + acceleration * dt))
            self.q[i] += velocity * dt
            self.v[i] = velocity
            bounded = max(SERVO_LOWER[i], min(SERVO_UPPER[i], self.q[i]))
            if bounded != self.q[i]:
                self.q[i], self.v[i] = bounded, 0.
        return list(self.q)


class ArmServoPlugin:
    PREFIX = 'loco_servo_arm'

    def __init__(self, adapter, config=None):
        self.adapter, self.reader = adapter, adapter.reader
        config = config or {}
        hz = config.get('expected_hz', DEFAULT_EXPECTED_HZ)
        if isinstance(hz, bool) or not isinstance(hz, (float, int)) or not math.isfinite(hz) or not 0 < hz <= MAX_HZ:
            raise ValueError('loco_servo_arm.expected_hz must be in (0, 50]')
        dry = config.get('dry_run', False)
        if type(dry) is not bool:
            raise ValueError('loco_servo_arm.dry_run must be boolean')
        self.dry_run = dry
        self.descriptor_raw = build_descriptor(hz)
        self.descriptor = parse_descriptor(self.descriptor_raw)
        self.publisher = None
        self.loop = None
        self.subscription = None
        self.running = self.paused = False
        self.sink = None
        self.input_topic = None
        self.target = None
        self.deadline = 0.
        self.output_task = self.watchdog_task = None
        self.commands = self.applied = self.overwritten = 0
        self.last_outcome = self.error = None
        self.max_output_gap = 0.
        self._pending_deadline = 0.
        self._mailbox_lock = threading.Lock()
        self._mailbox = None
        self._scheduled = False
        self._generation = 0
        self._subscription_generation = 0

    def get_tool(self):
        return {'name': self.PREFIX, 'type': 'actuator', 'multiInstance': False,
            'description': 'TRON2上肢双臂连续伺服：接收一路motus.control/1关节目标流，左臂7维+右臂7维+头部俯仰/偏航2维，单位rad；建议30Hz、最多50Hz。Driver限速插值并以目标300Hz发送ServoJ；双臂与头部同步控制，头部使用厂家原始方向，夹爪单独控制。普通动作使用arm_control/pose；本卡片供遥操作或执行模型接线使用。断流200ms或指令过期停止后续目标输出，不自动回零；不是硬件急停。',
            'inputSchema': {'type': 'object', 'properties': {
                'action': {'type': 'string', 'enum': ['start', 'stop', 'pause', 'resume', 'info']},
                'input_topic': {'type': 'string', 'description': 'control/joint指令流话题'}},
                'required': ['action'],
                'x-action-params': {
                    'pause': {'params': [], 'description': '暂停关节输出并清空旧目标，保留订阅'},
                    'resume': {'params': [], 'description': '恢复接收新目标，从实测关节角平滑接管'}},
                'x-hooks': {'on_interrupt_motion': {'action': 'pause'}, 'on_interrupt_all': {'action': 'pause'}},
                'x-is-dangerous': True, 'x-resource': ['arm_l', 'arm_r', 'head']},
            'topic_in': [{'format': 'control/joint', 'desc': 'motus.control/1 joint_position，16维rad，左臂、右臂、头部pitch/yaw'}],
            'configSchema': {'type': 'object', 'properties': {
                'dry_run': {'type': 'boolean', 'default': False, 'description': '空跑：接收并检查目标，不发送电机指令；切换后暂停，需resume继续'}}}}

    def info(self):
        return {'state': 'idle' if not self.running else 'paused' if self.paused else 'running',
            'input': self.input_topic, 'dry_run': self.dry_run,
            'control_interface': self.descriptor_raw, 'output_target_hz': OUTPUT_HZ,
            'commands_submitted': self.commands, 'targets_applied': self.applied,
            'mailbox_overwritten': self.overwritten, 'last_outcome': self.last_outcome,
            'error': self.error, 'max_output_gap_seconds': self.max_output_gap,
            'counters': dict(self.sink.counters) if self.sink else {},
            'output_active': self.target is not None, 'head_policy': 'commanded_pitch_yaw_native_radians',
            'vendor_acknowledged': False, 'arrival_verified': False,
            'physical_stop_verified': False, 'motion_sent': False}

    def _conflict(self):
        if self.adapter.motion.active or self.adapter.head.active:
            raise ValueError('双臂或头部动作正在占用控制权')
        follow = self.adapter.follow_runtime
        if follow is not None and follow.info().get('state') not in ('idle', 'paused', 'stopped', 'settled'):
            raise ValueError('请先暂停人物头部跟随')
        if self.reader.move_pending or self.reader.head_pending or self.reader._pose_servo is not None:
            raise ValueError('厂家运动通道正在占用')

    def _feedback(self):
        # Honor the site's existing maintenance switch; never create a file.
        if Path(os.environ.get('TRON2_MOTION_MAINTENANCE_FILE', '/config/tron2/motion-maintenance.json')).exists():
            raise ValueError('Driver正在维护，暂停伺服输出')
        profile = self.reader.configuration
        if profile.get('mode') != 'upper_limb' or profile.get('state') != 'identified':
            raise ValueError('loco_servo_arm仅支持已识别的DACH上肢构型')
        status = self.reader.sample('robot_status')
        if not status['available'] or not motion_readiness(status['data'])['ready'] or status['data'].get('motor') != 'OK':
            raise ValueError('机器人未处于上肢开发就绪状态')
        sample = self.reader.sample('joint_state')
        data = skeleton_stream(sample, profile)
        if not data['status']['available'] or sample['reception_age_seconds'] > MAX_FEEDBACK_AGE:
            raise ValueError('需要新鲜的16关节反馈')
        joints = {j['idx']: j['q'] for j in data['joints']}
        stamp = sample['data']['robot_timestamp']
        if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            raise ValueError('需要有效的厂家关节时间戳')
        return [joints[i] for i in range(16)], stamp

    def _clear(self):
        self.target = None
        self.deadline = 0.

    def _apply(self, values, gripper):
        if gripper is not None:
            raise ValueError('本卡片不控制夹爪，请使用独立夹爪卡片')
        self.target = list(values)
        self.deadline = self._pending_deadline
        self.applied += 1
        if self.output_task is None or self.output_task.done():
            self.output_task = asyncio.create_task(self._output())

    def _receive(self, raw, generation):
        """Bounded mailbox across the ROS thread; never queue a trajectory."""
        with self._mailbox_lock:
            if generation != self._generation or not self.running or self.paused:
                return
            if self._mailbox is not None:
                self.overwritten += 1
            self._mailbox = (raw, generation)
            if self._scheduled:
                return
            self._scheduled = True
        self.loop.call_soon_threadsafe(self._drain)

    def _drain(self):
        with self._mailbox_lock:
            item, self._mailbox = self._mailbox, None
            self._scheduled = False
        if item is None or item[1] != self._generation or not self.running or self.paused:
            return
        try:
            if not isinstance(item[0], str) or len(item[0]) > 32768:
                raise ValueError('控制消息超过32KB或不是JSON文本')
            message = json.loads(item[0], parse_constant=reject_constant)
            self._feedback()
            if self.output_task is None or self.output_task.done():
                self._conflict()
            now = time.time() * 1000
            # Validate before any timestamp arithmetic; ControlSink owns the rest.
            if not isinstance(message, dict):
                raise ValueError('控制消息必须为JSON对象')
            stamp, ttl = message.get('stamp_ms'), message.get('ttl_ms')
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (stamp, ttl)):
                raise ValueError('stamp_ms/ttl_ms必须为有限数值')
            if message.get('gripper') is not None:
                raise ValueError('本卡片不接收gripper字段')
            self._pending_deadline = time.monotonic() + max(0., (stamp + ttl - now) / 1000)
            outcome = self.sink.submit(message)
            self.last_outcome = {'verdict': outcome.verdict.value, 'reason': outcome.reason}
        except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
            self.last_outcome = {'verdict': 'rejected', 'reason': str(exc)}

    async def _output(self):
        sent = 0
        try:
            self._conflict()
            q, stamp = self._feedback()
            limiter = Limiter(q)
            last = time.monotonic()
            changed_at = last
            # Use the reader's existing exclusive writer and faster feedback lease.
            with self.reader.pose_feedback(), self.reader.pose_servo({'motion_tool': self.PREFIX}) as token:
                while self.running and not self.paused and self.target is not None:
                    now = time.monotonic()
                    if now >= self.deadline:
                        self._clear()
                        break
                    if token.get('error'):
                        raise ValueError(token['error'])
                    q, current_stamp = self._feedback()
                    if current_stamp != stamp:
                        if current_stamp < stamp:
                            raise ValueError('关节反馈时间戳倒退')
                        stamp, changed_at = current_stamp, now
                    if now - changed_at > MAX_FEEDBACK_AGE:
                        raise ValueError('关节反馈时间戳停止更新')
                    dt = now - last
                    if sent and dt > MAX_OUTPUT_GAP:
                        raise ValueError('ServoJ输出间隔超过50ms，不补发')
                    self.max_output_gap = max(self.max_output_gap, dt)
                    if max(abs(a-b) for a,b in zip(q, limiter.q)) > TRACKING_LIMIT:
                        raise ValueError('实际关节与当前输出轨迹偏差超过8度')
                    target = limiter.step(self.target, max(1e-6, dt))
                    if not self.dry_run:
                        await self.reader.send_arm_servoj(target, token)
                        self.commands += 1
                    sent += 1
                    last = now
                    await asyncio.sleep(max(0., 1 / OUTPUT_HZ - (time.monotonic() - now)))
        except Exception as exc:
            self.error = str(exc)
            self.paused = True
            self._clear()
        finally:
            self._clear()

    async def _watchdog(self):
        while self.running:
            outcome = self.sink.tick()
            if outcome:
                self.last_outcome = {'verdict': outcome.verdict.value, 'reason': outcome.reason}
            await asyncio.sleep(.02)

    async def _cancel_output(self):
        self._generation += 1
        with self._mailbox_lock:
            self._mailbox = None
        self._clear()
        task = self.output_task
        if task is not None:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        self.output_task = None

    async def call(self, args):
        action = args.get('action', 'info')
        if action == 'info':
            return self.info()
        if action == 'config':
            if type(args.get('dry_run')) is not bool:
                raise ValueError('dry_run必须为布尔值')
            if args['dry_run'] != self.dry_run:
                await self.preempt()
                self.dry_run = args['dry_run']
            return self.info()
        if action == 'start':
            if self.running:
                return self.info()
            topic = args.get('input_topic') or (args.get('input_topics') or [''])[0]
            if not isinstance(topic, str) or not topic.startswith('/') or len(topic) > 256:
                raise ValueError('请连接一路control/joint输入话题')
            if self.publisher is None:
                raise ValueError('ROS订阅尚未初始化')
            self._conflict()
            self.loop = asyncio.get_running_loop()
            self.sink = ControlSink(self.descriptor, self._apply, on_watchdog=self._clear, on_abort=self._clear)
            self.input_topic = topic
            self.running, self.paused, self.error = True, False, None
            try:
                self._subscription_generation += 1
                sid = self._subscription_generation
                self.subscription = self.publisher.subscribe_json(topic, lambda raw: self._receive(raw, self._generation)
                    if sid == self._subscription_generation else None)
                self.watchdog_task = asyncio.create_task(self._watchdog())
            except Exception:
                self.running = False
                raise
            return self.info()
        if action in ('stop', 'pause'):
            self.paused = True
            await self._cancel_output()
            if action == 'stop':
                self.running = False
                self._subscription_generation += 1
                if self.subscription is not None:
                    self.publisher.unsubscribe(self.subscription)
                    self.subscription = None
                if self.watchdog_task is not None:
                    self.watchdog_task.cancel()
                    await asyncio.gather(self.watchdog_task, return_exceptions=True)
                    self.watchdog_task = None
            return self.info()
        if action == 'resume':
            if not self.running:
                return self.info()
            self._conflict()
            self._feedback()
            self.error = None
            self.paused = False
            return self.info()
        raise ValueError('不支持的loco_servo_arm动作')

    async def preempt(self):
        if self.running:
            await self.call({'action': 'pause'})

    async def close(self):
        await self.call({'action': 'stop'})
