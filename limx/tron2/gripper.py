"""LimX two-finger gripper: fixed side, correlated status and ACP completion."""
import asyncio
import math
import logging
import time
from uuid import uuid4

from vendor import MoveError

GRIPPER_SIDES = {'gripper_left': 'left', 'gripper_right': 'right'}
MOTION_ACTIONS = ('open', 'close')
TIMEOUT = 20
ACP_TIMEOUT = 40
POLL_SECONDS = .2
STABILITY_SECONDS = .5
TOLERANCE = 1
RECOVERY_TIMEOUT = 8
RECOVERY_STABILITY_SECONDS = 1
PREVIOUS_TIMEOUT = 40
LOG = logging.getLogger('tron2')
ACTION_FIELDS = {'info': set(), 'start': set(), 'stop': set(),
                 'open': {'opening','speed','force'}, 'close': {'speed','force'},
                 'recover': set()}
METADATA_FIELDS = {'action','instance_id','_trace_id'}


def normalize_arguments(action, args):
    if not isinstance(args, dict) or action not in ACTION_FIELDS:
        raise ValueError('不支持的夹爪操作或参数格式')
    known = METADATA_FIELDS | set().union(*ACTION_FIELDS.values())
    if set(args) - known:
        raise ValueError('当前操作包含未声明的参数')
    if 'action' in args and args['action'] != action:
        raise ValueError('操作与参数中的action不一致')
    # Some dashboard versions submit inputs hidden by x-action-params. Ignore
    # only declared, inactive fields; never reinterpret them as motor values.
    return {key:value for key,value in args.items() if key in ACTION_FIELDS[action] | METADATA_FIELDS}


def recovery_arguments(args):
    return normalize_arguments('recover', args)


def percentage(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 <= value <= 100 or not math.isfinite(value):
        raise ValueError(name + '请填写0至100的数值')
    return value


def command(action, args, side):
    args = normalize_arguments(action, args)
    if side not in ('left', 'right'):
        raise ValueError('请选择左侧或右侧夹爪')
    if action not in MOTION_ACTIONS:
        raise ValueError('不支持的夹爪操作')
    opening = percentage(args.get('opening'), '开口度') if action == 'open' else 0
    speed, force = percentage(args.get('speed'), '速度'), percentage(args.get('force'), '夹持力')
    if speed == 0:
        raise ValueError('执行开合时速度须大于0')
    # Never fill the other side with defaults: omission leaves it unchanged.
    return {side + '_opening': opening, side + '_speed': speed, side + '_force': force}


def state_data(data, selected_side=None):
    if not isinstance(data, dict) or data.get('result') != 'success':
        raise ValueError('夹爪状态不可用或厂家报告故障')
    out = {}
    for side in ((selected_side,) if selected_side else ('left', 'right')):
        out[side] = {key: percentage(data.get(side + '_' + key), side + '_' + key)
                     for key in ('opening', 'speed', 'force')}
    stamp = data.get('timestamp')
    if stamp is not None and (isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or
                              not 0 <= stamp <= 2**63 - 1 or not math.isfinite(stamp)):
        raise ValueError('夹爪状态时间戳无效')
    return {**out, 'vendor_timestamp_ms': stamp,
            'received_at': time.time(), 'measurement_time_verified': False,
            'value_unit': '0_to_100_dimensionless', 'grasp_verified': False}


def tool(metadata, side):
    label = '左' if side == 'left' else '右'
    return {'name': 'gripper_' + side, 'type': 'actuator', 'multiInstance': False,
            'description': 'TRON2' + label + '夹爪：open设置目标开口，close闭合，recover检查静止反馈后恢复本侧失败任务。开口0为最小闭合、100为最大张开；速度/力为相对档位，不代表毫米或牛顿。达到开口不代表夹住物体。恢复前须现场确认静止空载；恢复不是急停。',
            'inputSchema': {'type': 'object', 'additionalProperties': False,
                'properties': {
                    'action': {'type': 'string', 'enum': ['open', 'close', 'recover'],
                               'description': 'open设置目标开口；close闭合；recover恢复本侧失败任务'},
                    'opening': {'type': 'number', 'minimum': 0, 'maximum': 100, 'description': '目标开口度：0闭合，100张开；请填写实物可达到的目标'},
                    'speed': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 100, 'description': '开合速度档位：大于0至100'},
                    'force': {'type': 'number', 'minimum': 0, 'maximum': 100, 'description': '夹持力档位：0至100，并非牛顿'},
                    **metadata},
                'required': ['action'],
                'allOf': [{'if': {'properties': {'action': {'enum': list(MOTION_ACTIONS)}}, 'required': ['action']},
                           'then': {'required': ['speed', 'force']}},
                          {'if': {'properties': {'action': {'const': 'open'}}, 'required': ['action']},
                           'then': {'required': ['opening']}}],
                'x-action-params': {
                    'open': {'params': ['opening', 'speed', 'force'], 'description': '将本侧夹爪调整至填写的目标开口度'},
                    'close': {'params': ['speed', 'force'], 'description': '闭合本侧夹爪；未到最小开口不会报告到位或抓取成功'},
                    'recover': {'params': [], 'description': '现场确认本侧静止空载后使用；连续反馈验证后恢复，不重发动作、不把未到位改成成功'}},
                'x-resource': ['arm_l' if side == 'left' else 'arm_r'],
                'x-completion': {'actions': list(MOTION_ACTIONS), 'timeout': ACP_TIMEOUT}}}


class Gripper:
    def __init__(self, reader, report, side):
        if side not in ('left', 'right'):
            raise ValueError('Invalid fixed gripper side')
        self.side = side
        self.reader, self.report = reader, report
        self.active = self.last = self.task = self.sample = None
        self.closed = False
        self.peer = None
        self.recovering = False
        self.recovery_task = None
        self.waiting = None

    def info(self):
        age = None if self.sample is None else max(0, time.monotonic() - self.sample[0])
        available = self.reader.connected and age is not None and age <= self.reader.settings['stale_after_seconds'] \
            and self.reader.ws is self.sample[2] and self.reader.serial == self.sample[3]
        motion = dict(self.last) if self.last and (not self.active or
                    self.last['action_id'] == self.active['action_id']) else None
        if self.active:
            motion = {**(motion or {}), **{k: self.active[k] for k in ('action_id', 'action', 'side', 'state')},
                      'target_opening': self.active['target']}
        return {'side': self.side, 'state': self.active['state'] if self.active else
                    'recovering' if self.recovering else 'waiting_for_previous' if self.waiting else
                    'ready' if self.reader.connected else 'offline',
                'motion_sent': False, 'available': available,
                'reception_age_seconds': age, 'data': self.sample[1] if available else None,
                'motion': motion, 'recovering': self.recovering,
                'waiting_action': self.waiting['action'] if self.waiting else None,
                'can_accept_motion': not self.closed and not self.active and not self.recovering and
                    not self.waiting and
                    self.reader.connected and not (self.peer and (self.peer.closed or self.peer.active)),
                'grasp_verified': False}

    async def read_state(self):
        data = state_data(await self.reader.read_gripper(), self.side)
        self.sample = (time.monotonic(), data, self.reader.ws, self.reader.serial)
        return {**self.info(), 'read_only': True}

    async def recover(self, args):
        args = recovery_arguments(args)
        await self.wait_previous('recover', allow_blocked=True)
        if self.closed or self.recovering or self.peer and (self.peer.closed or self.peer.active):
            raise MoveError('驱动关闭、正在恢复或机械臂任务未结束，不能恢复夹爪', False)
        job = self.active
        if job is None:
            return {**self.info(), 'read_only': True, 'recovered': False, 'message': '没有需要恢复的夹爪任务'}
        if job['state'] != 'blocked_requires_operator_check' or \
                self.task and not self.task.done() or not self.last or self.last['action_id'] != job['action_id']:
            raise MoveError('本侧任务仍在执行或报告时不能恢复', False)
        self.recovering = True
        # Completion belongs to this recovery operation, not its HTTP caller's
        # lifetime (which may continue after recover returns).
        self.recovery_task = asyncio.get_running_loop().create_future()
        try:
            ws, serial = self.reader.ws, self.reader.serial
            deadline = time.monotonic() + RECOVERY_TIMEOUT
            start = None; count = 0; previous = None; stamp = None
            while time.monotonic() < deadline:
                if not self.reader.connected or self.reader.ws is not ws or self.reader.serial != serial:
                    raise ValueError('恢复期间连接发生变化，请重新检查；未解除任务保护')
                state = (await self.read_state())['data']
                if self.reader.ws is not ws or self.reader.serial != serial or not self.reader.connected:
                    raise ValueError('恢复期间连接发生变化，未解除任务保护')
                current_stamp = state['vendor_timestamp_ms']
                openings = [state[self.side]['opening']]
                fresh = current_stamp is not None and (stamp is None or current_stamp > stamp)
                stationary = state[self.side]['speed'] == 0
                stable = previous is None or all(abs(a-b) <= TOLERANCE for a,b in zip(openings,previous))
                stamp = current_stamp
                if fresh and stationary and stable:
                    if start is None:
                        start = time.monotonic()
                        previous = openings
                    count += 1
                    if count >= 3 and time.monotonic()-start >= RECOVERY_STABILITY_SECONDS:
                        break
                else:
                    start = None; count = 0; previous = None
                await asyncio.sleep(POLL_SECONDS)
            else:
                raise ValueError('未收到连续更新且稳定的零速度夹爪反馈，未解除任务保护')
            # Acknowledgement loss is not converted to success, nor is any
            # motion command retried. Only re-deliver the original ACP outcome.
            if not self.last['callback_delivered']:
                try:
                    delivered = await self.report(job['action_id'], self.last['status'],
                        {k:v for k,v in self.last.items() if k not in ('action_id','status','callback_delivered')})
                except Exception:
                    delivered = False
                if not delivered:
                    raise ValueError('原任务结果仍无法通知平台，未解除任务保护')
                self.last['callback_delivered'] = True
            if self.closed or self.reader.ws is not ws or self.reader.serial != serial or not self.reader.connected or \
                    self.peer and (self.peer.closed or self.peer.active):
                raise ValueError('恢复期间驱动、连接或机械臂状态变化，未解除任务保护')
            self.last['operator_recovery'] = {'side': self.side,
                'reported_stationary_verified': True, 'verified_samples': count,
                'observed_opening': state[self.side]['opening'], 'vendor_timestamp_ms': stamp,
                'motion_sent': False, 'recovered_at': time.time()}
            self.active = None
            self.recovering = False
            LOG.info('Gripper task recovered after stationary feedback verification: %s', job['action_id'])
            return {**self.info(), 'read_only': True, 'recovered': True,
                    'message': '已恢复后续操作；原任务结果保留，未发送任何开合或急停指令'}
        finally:
            self.recovering = False
            self.recovery_task.set_result(None)
            self.recovery_task = None

    def cancel_waiting(self):
        # Cancels only a request not yet admitted. Never claims to stop a motor.
        if self.waiting:
            self.waiting['cancel'].set()

    def blocked_error(self):
        job = self.active or {}
        last = self.last or {}
        label = '左' if self.side == 'left' else '右'
        return MoveError(label + '夹爪上一条' + str(job.get('action', '动作')) +
            '结果尚未确认：目标' + str(job.get('target')) + '，最近反馈' +
            str(last.get('reported_opening', job.get('reported_opening'))) +
            '；后续指令未发送。现场确认静止空载后使用本侧recover', False)

    async def wait_previous(self, action, allow_blocked=False):
        if self.closed:
            raise MoveError('夹爪驱动已关闭，后续指令未发送', False)
        if self.waiting:
            raise MoveError('本侧已有一条后续指令等待，请勿重复提交', False)
        previous = self.recovery_task if self.recovering else self.task if self.active else None
        if previous and not previous.done():
            wait = {'action': action, 'cancel': asyncio.Event()}
            self.waiting = wait
            ws, serial = self.reader.ws, self.reader.serial
            cancellation = asyncio.create_task(wait['cancel'].wait())
            try:
                # Wait on the real task outcome, not a fixed delay. Cancelling a
                # queued RPC cannot cancel the already admitted physical action.
                done, _ = await asyncio.wait({previous, cancellation}, timeout=PREVIOUS_TIMEOUT,
                                             return_when=asyncio.FIRST_COMPLETED)
                if self.closed or wait['cancel'].is_set():
                    raise MoveError('已取消本侧等待中的后续指令，未发送电机命令；当前动作未被急停', False)
                if previous not in done:
                    raise MoveError('等待本侧上一条任务完成超时，后续指令未发送；请检查任务状态', False)
                if not self.reader.connected or self.reader.ws is not ws or self.reader.serial != serial:
                    raise MoveError('等待期间机器人连接变化，后续指令未发送；请重新检查', False)
            finally:
                cancellation.cancel()
                await asyncio.gather(cancellation, return_exceptions=True)
                self.waiting = None
            # Cleanup above yields: stop or disconnect can arrive during it.
            if self.closed or wait['cancel'].is_set():
                raise MoveError('已取消本侧等待中的后续指令，未发送电机命令；当前动作未被急停', False)
            if not self.reader.connected or self.reader.ws is not ws or self.reader.serial != serial:
                raise MoveError('等待期间机器人连接变化，后续指令未发送；请重新检查', False)
        if self.active and not allow_blocked:
            raise self.blocked_error()

    async def dispatch(self, action, args):
        data = command(action, args, self.side)
        await self.wait_previous(action)
        if self.peer and self.peer.active and self.peer.active['state'] == 'reporting' and self.peer.active.get('arrival_verified'):
            await asyncio.shield(self.peer.task)
        if self.peer and (self.peer.closed or self.peer.active):
            raise MoveError('机械臂动作进行中或结果尚未确认，暂不接受夹爪开合', False)
        if self.closed or self.active or self.recovering:
            raise MoveError('本侧夹爪任务状态变化，后续指令未发送；请检查状态', False)
        future = asyncio.get_running_loop().create_future()
        future.add_done_callback(lambda f: f.exception() if not f.cancelled() else None)
        job = {'action_id': 'tron2_gripper_' + uuid4().hex, 'action': action, 'side': self.side,
               'target': data[self.side + '_opening'], 'state': 'checking', 'admission': future,
               'ws': self.reader.ws, 'serial': self.reader.serial, 'motion_sent': False}
        self.active = job
        self.task = asyncio.create_task(self._run(job, data))
        return await asyncio.shield(future)

    def same_session(self, job):
        if not self.reader.connected or self.reader.ws is not job['ws'] or self.reader.serial != job['serial']:
            raise ValueError('机器人连接已改变，动作结果尚未确认')

    async def verify_target(self, job):
        start = None; matches = 0
        last_stamp = job['before_timestamp']
        deadline = time.monotonic() + TIMEOUT
        while time.monotonic() < deadline:
            self.same_session(job)
            result = await self.read_state()  # Each sample is a new correlated query.
            self.same_session(job)
            actual = result['data'][job['side']]['opening']
            job['reported_opening'] = actual
            job['reported_speed'] = result['data'][job['side']]['speed']
            stamp = result['data']['vendor_timestamp_ms']
            now = time.monotonic()
            if stamp is None or stamp <= last_stamp:
                start = None; matches = 0
                await asyncio.sleep(POLL_SECONDS)
                continue
            last_stamp = stamp
            if abs(actual - job['target']) <= TOLERANCE and \
                    (not job.get('no_op') or job['reported_speed'] == 0):
                start = now if start is None else start
                matches += 1
                if matches >= 3 and now - start >= STABILITY_SECONDS:
                    return {'target_opening_verified': True, 'reported_opening': actual,
                            'verified_samples': matches, 'grasp_verified': False,
                            'verification': 'already_at_target_fresh_feedback' if job.get('no_op') else 'post_ack_correlated_vendor_state',
                            'no_op': bool(job.get('no_op'))}
            else:
                start = None; matches = 0
            await asyncio.sleep(POLL_SECONDS)
        raise ValueError('未确认达到目标开口'+str(job['target'])+'，最近反馈'+str(job.get('reported_opening'))+
                         '；检查实物停止后可使用recover恢复。未确认抓取成功')

    async def _run(self, job, data):
        admitted = False
        try:
            self.same_session(job)
            before = await self.read_state()  # Unsupported protocol or motor faults cannot lead to a write.
            job['before_timestamp'] = before['data']['vendor_timestamp_ms']
            if job['before_timestamp'] is None:
                raise ValueError('厂家状态缺少反馈时间戳，无法确认动作结果；未发送开合指令')
            self.same_session(job)
            job['no_op'] = abs(before['data'][job['side']]['opening'] - job['target']) <= TOLERANCE
            if job['no_op']:
                job['state'] = 'verifying_existing_target'
            else:
                job['state'] = 'awaiting_ack'
                job['motion_sent'] = None  # Cancellation during send cannot claim that no command left.
                try:
                    await self.reader.send_gripper(data)
                    job['motion_sent'] = True
                except MoveError as exc:
                    job['motion_sent'] = exc.motion_sent
                    if exc.motion_sent is False or exc.code == 'vendor_rejected':
                        raise
                    # Unknown send/ACK must not be retried or converted to success by position alone.
                    raise ValueError('夹爪指令回执未确认，请检查状态；不会自动重发') from exc
                job['state'] = 'moving'
            admitted = True
            job['admission'].set_result({'action_id': job['action_id'], 'action': job['action'],
                'side': job['side'], 'target_opening': job['target'], 'state': job['state'],
                'motion_sent': job['motion_sent'], 'no_op': job['no_op'],
                'target_opening_verified': False, 'grasp_verified': False,
                'can_accept_motion': False,
                'message': '指令已受理，仍在等待目标反馈和完成回调；本侧下一条指令会等待本条确认',
                'completion_protocol': 'acp'})
            result = await self.verify_target(job)
            status = 'completed'
        except asyncio.CancelledError:
            result, status = {'error': '驱动关闭，夹爪可能仍在运动'}, 'error'
        except Exception as exc:
            if not isinstance(exc, (ValueError, OSError)):
                exc = MoveError('夹爪处理失败，请检查连接和状态', job['motion_sent'])
            if not admitted and (job['motion_sent'] is False or isinstance(exc, MoveError) and exc.code == 'vendor_rejected'):
                self.active = None
                job['admission'].set_exception(exc)
                return
            result, status = {'error': str(exc)}, 'error'
        if not job['admission'].done():
            job['admission'].set_result({'action_id': job['action_id'], 'state': 'result_unconfirmed',
                'motion_sent': job['motion_sent'], 'target_opening_verified': False,
                'grasp_verified': False, 'completion_protocol': 'acp', 'message': result['error']})
        result.update(action=job['action'], side=job['side'], target_opening=job['target'],
                      motion_sent=job['motion_sent'], grasp_verified=False,
                      motion_may_continue=status != 'completed' and job['motion_sent'] is not False,
                      no_op=bool(job.get('no_op')))
        result.setdefault('target_opening_verified', False)
        result.update({k: job[k] for k in ('reported_opening','reported_speed') if k in job})
        if status != 'completed':
            LOG.warning('Gripper action %s failed: %s', job['action_id'], result.get('error'))
        job['target_opening_verified'] = result['target_opening_verified']
        job['state'] = 'reporting'
        try:
            delivered = await self.report(job['action_id'], status, result)
        except Exception:
            delivered = False
        self.last = {'action_id': job['action_id'], 'status': status, 'callback_delivered': delivered, **result}
        if delivered and (status == 'completed' or job['motion_sent'] is False):
            self.active = None
        else:
            job['state'] = 'blocked_requires_operator_check'

    async def close(self):
        self.closed = True
        self.cancel_waiting()
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
