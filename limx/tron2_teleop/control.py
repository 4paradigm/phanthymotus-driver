"""Single asyncio owner for admission; numerical IPC never blocks the sender."""
import asyncio
import html
from pathlib import Path
import time
import uuid

import aiohttp

from common.motion.worker import NumericalWorker
from common.teleop_contract import (COMMAND_TOPIC, STATE_TOPIC, FEEDBACK_SCHEMA,
                                   MAX_INPUT_AGE_NS, binding_from_topic, canonical_instance, validate_input)
from limx.tron2_teleop.mapping import RelativeMapping
from limx.tron2_teleop.model import MODEL, validate_q

PERIOD = 1/300


def make_worker():
    return NumericalWorker(MODEL, driver_dir=Path(__file__).resolve().parents[2],
        solver_module='limx.tron2_teleop.kinematics', solver_class='Tron2IK')


def gripping(frame):
    return frame is not None and all(frame[side]['grip'] >= .5 for side in ('left', 'right'))


def released(frame):
    return frame is not None and all(frame[side]['grip'] < .5 for side in ('left', 'right'))


class TeleopControl:
    def __init__(self, robot, worker, *, clock_id, scale=.5):
        self.robot, self.worker, self.clock_id, self.scale = robot, worker, clock_id, scale
        self.active = self.closed = self.output = False
        self.frame = self.mapping = self.head = self.target = self.sending = None
        self.identity = None
        self.sequence = -1
        self.generation = self.mapping_epoch = self.feedback_sequence = 0
        self.robot_epoch = robot.epoch
        self.started_ns = 0
        self.session_id = None
        self.server_epoch = uuid.uuid4().hex
        self.reason, self.state, self.fault = None, 'idle', None
        self.commanded = self.last_send = None
        self.ik_ms = None
        self.missed_cycles = 0
        self.wake = asyncio.Event()

    def get_tool(self):
        return {'name': 'teleop_control', 'type': 'actuator', 'multiInstance': False,
            'description': 'TRON2双臂遥操：连接PICO teleop_device后启动项目。先松开双握把建立基准，再按住双握把跟随；松握暂停目标更新，再握沿用基准。停止项目停止发送，不能替代物理急停。仅允许一个运动控制来源。',
            'configSchema': {'type': 'object', 'additionalProperties': False, 'properties': {}},
            'inputSchema': {'type': 'object', 'required': ['action'], 'properties': {
                'action': {'type': 'string', 'enum': ['info', 'config', 'start', 'stop']},
                'input_topic': {'type': 'string'}},
                'x-resource': ['arm_l', 'arm_r', 'head'],
                'x-action-params': {'info': {'params': []}, 'config': {'params': []},
                                    'start': {'params': ['input_topic']}, 'stop': {'params': []}}},
            'topic_in': [{'port_id': 'command', 'format': 'data/teleop-cmd', 'topic': COMMAND_TOPIC}],
            'topic_out': [{'port_id': 'state', 'format': 'data/teleop-state', 'topic': STATE_TOPIC}]}

    def invalidate(self, reason, *, rebase=True):
        self.generation += 1
        self.target = self.commanded = self.last_send = None
        self.output = False
        self.reason = reason
        if self.sending is not None:
            self.sending.cancel()
        if rebase:
            self.mapping = self.head = None
        self.state = 'waiting' if self.active else 'idle'

    def start(self, input_topic):
        binding_from_topic(input_topic)
        if self.closed:
            raise ValueError('driver_closed')
        if self.active:
            return self.info()
        self.active, self.fault = True, None
        self.robot.servo_error = None
        self.invalidate('waiting_for_released_grips')
        self.frame = self.identity = None
        self.sequence = -1
        self.started_ns = time.monotonic_ns()
        self.session_id = uuid.uuid4().hex
        self.wake.set()
        return self.info()

    async def stop(self):
        self.active = False
        self.invalidate('stopped')
        self.frame = None
        pending = self.sending
        if pending is not None:
            await asyncio.gather(pending, return_exceptions=True)
        self.wake.set()
        return self.info()

    def receive(self, value):
        if not self.active or self.fault:
            return False
        try:
            frame = validate_input(value, instance_id=value.get('instance_id'),
                clock_id=self.clock_id, now_ns=time.monotonic_ns())
            if frame['received_monotonic_ns'] < self.started_ns:
                raise ValueError('input_before_start')
            identity = (canonical_instance(frame['instance_id']), frame['device_id'],
                        frame['connection_epoch'], frame['space_epoch'])
            if self.identity == identity:
                if frame['sequence'] <= self.sequence:
                    raise ValueError('input_sequence_replayed')
                if self.frame and frame['received_monotonic_ns'] <= self.frame['received_monotonic_ns']:
                    raise ValueError('input_time_replayed')
            else:
                if self.identity and identity[:2] == self.identity[:2] and (
                        identity[2] < self.identity[2] or identity[2] == self.identity[2] and identity[3] < self.identity[3]):
                    raise ValueError('input_epoch_replayed')
                self.invalidate('input_generation_changed')
                self.identity = identity
            self.sequence = frame['sequence']
            if not all(frame[key]['tracked'] for key in ('head_reference', 'left', 'right')):
                raise ValueError('tracking_lost')
            if not gripping(frame) and (gripping(self.frame) or self.target is not None):
                self.invalidate('grip_released', rebase=False)
            self.frame = frame
            self.wake.set()
            return True
        except (AttributeError, ValueError, TypeError, OverflowError, RecursionError) as exc:
            self.frame = None
            self.invalidate(str(exc)[:160] or 'invalid_input')
            return False

    def watchdog(self):
        if not self.active:
            return False
        if self.robot.epoch != self.robot_epoch:
            self.robot_epoch = self.robot.epoch
            self.invalidate('robot_connection_changed')
        reason = self.fault or self.robot.ready()
        if reason is None and (self.frame is None or not 0 <= time.monotonic_ns()-self.frame['received_monotonic_ns'] <= MAX_INPUT_AGE_NS):
            reason = 'input_stale'
        if reason:
            if self.reason != reason or self.mapping is not None or self.target is not None:
                self.invalidate(reason)
            if self.fault:
                self.state = 'error'
            return False
        return True

    async def solve_loop(self):
        processed = None
        while not self.closed:
            await self.wake.wait()
            self.wake.clear()
            if not self.watchdog():
                continue
            frame, generation, epoch = self.frame, self.generation, self.robot.epoch
            key = (generation, self.identity, frame['sequence'])
            if key == processed:
                continue
            processed = key
            q = list(self.robot.q)
            try:
                if self.mapping is None:
                    if not released(frame):
                        self.state, self.reason = 'waiting', 'release_both_grips_to_calibrate'
                        continue
                    self.state = 'calibrating'
                    poses = await asyncio.to_thread(self.worker.call, 'forward', {'q': q[:14]},
                                                    deadline=time.monotonic()+.1)
                    if generation != self.generation or epoch != self.robot.epoch or not self.watchdog() or not released(self.frame):
                        continue
                    self.mapping = RelativeMapping(frame, poses, self.scale)
                    self.head = q[14:]
                    self.mapping_epoch += 1
                    self.state, self.reason = 'ready', None
                    continue
                if not gripping(frame):
                    self.state, self.reason = 'paused', 'grip_released'
                    continue
                deadline = min(time.monotonic()+.1, frame['received_monotonic_ns']/1e9 + MAX_INPUT_AGE_NS/1e9)
                result = await asyncio.to_thread(self.worker.call, 'solve', {
                    'q': q[:14], 'commanded': self.commanded[:14] if self.commanded else None,
                    'values': self.mapping.targets(frame), 'measured_ns': int(self.robot.q_at*1e9)},
                    identity={'boot_id': self.clock_id, 'session_id': self.session_id,
                              'mapping_epoch': self.mapping_epoch, 'generation': generation}, deadline=deadline)
                if generation != self.generation or epoch != self.robot.epoch or not self.watchdog() or not gripping(self.frame):
                    continue
                target = validate_q(result['q'] + self.head)
                self.target = (target, frame['received_monotonic_ns'] + MAX_INPUT_AGE_NS, generation)
                self.ik_ms, self.reason = result['last_ms'], None
            except (ValueError, OSError, RuntimeError) as exc:
                if generation == self.generation:
                    self.invalidate(str(exc)[:160], rebase=False)
                    self.state = 'paused'

    async def send_loop(self):
        due = time.monotonic()
        while not self.closed:
            await asyncio.sleep(max(0, due-time.monotonic()))
            now = time.monotonic()
            if now-due >= PERIOD:
                self.missed_cycles += int((now-due)/PERIOD)
            due = now+PERIOD
            if not self.watchdog() or not gripping(self.frame) or self.target is None:
                continue
            target, expires, generation = self.target
            if time.monotonic_ns() > expires:
                self.invalidate('ik_target_expired', rebase=False)
                continue
            if self.last_send is not None and now-self.last_send > .05:
                self.invalidate('servo_deadline_missed', rebase=False)
                continue
            measured = self.robot.q
            previous = self.commanded or measured
            if any(abs(a-b) > .2 for a, b in zip(previous[:14], measured[:14])):
                self.fault = 'joint_tracking_error'
                self.invalidate(self.fault)
                continue
            if any(abs(a-b) > .1 for a, b in zip(self.head, measured[14:])):
                self.fault = 'head_moved_recalibration_required'
                self.invalidate(self.fault)
                continue
            dt = min(PERIOD, now-self.last_send) if self.last_send is not None else 0.
            step = self.worker.velocity*dt
            q = [a+max(-step, min(step, b-a)) for a, b in zip(previous[:14], target[:14])] + self.head
            self.sending = asyncio.create_task(self.robot.send(q))
            try:
                await self.sending
                if generation == self.generation and self.active:
                    self.commanded, self.last_send, self.output = q, time.monotonic(), True
                    self.state, self.reason = 'following', None
            except asyncio.CancelledError:
                if self.closed:
                    raise
            except (ValueError, OSError, aiohttp.ClientError, asyncio.TimeoutError) as exc:
                self.fault = 'servo_send_failed:' + (str(exc)[:100] or type(exc).__name__)
                self.invalidate(self.fault)
            finally:
                self.sending = None

    def info(self):
        return {'state': self.state, 'reason': self.reason, 'active': self.active,
                'topic_in': self.get_tool()['topic_in'], 'topic_out': self.get_tool()['topic_out'],
                'robot': self.robot.status(), 'mapping_epoch': self.mapping_epoch,
                'source_sequence': self.sequence, 'ik_ms': self.ik_ms,
                'output_active': self.output, 'commanded_q': self.commanded,
                'missed_servo_cycles': self.missed_cycles, 'physical_stop_confirmed': False,
                'collision_checked': False, 'head_policy': 'hold_session_start'}

    def feedback(self):
        self.feedback_sequence += 1
        identity = self.identity
        return {'schema': FEEDBACK_SCHEMA, 'instance_id': identity[0] if identity else 'tron2',
                'control_instance_id': 'tron2', 'clock_id': self.clock_id, 'server_epoch': self.server_epoch,
                'sequence': self.feedback_sequence, 'emitted_monotonic_ns': time.monotonic_ns(),
                'source_sequence': self.sequence, 'connection_epoch': identity[2] if identity else None,
                'space_epoch': identity[3] if identity else None, 'mapping_epoch': self.mapping_epoch,
                'operator_session_id': self.session_id, 'state': self.state, 'reason': self.reason,
                'capabilities': ['dual_arm'], 'execution': {**self.info(), 'hold_confirmed': False, 'stop_confirmed': False},
                'receipts': [], 'text': html.escape(f"TRON2双臂：{self.state}；{self.reason or '无'}")}
