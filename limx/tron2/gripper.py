"""LimX two-finger gripper: explicit side, correlated status and ACP completion."""
import asyncio
import math
import logging
import time
from uuid import uuid4

from vendor import MoveError

MOTION_ACTIONS = ('open', 'close', 'set_opening')
TIMEOUT = 20
ACP_TIMEOUT = 40
POLL_SECONDS = .2
STABILITY_SECONDS = .5
TOLERANCE = 1
RECOVERY_TIMEOUT = 8
RECOVERY_STABILITY_SECONDS = 1
LOG = logging.getLogger('tron2')
ACTION_FIELDS = {'info': set(), 'start': set(), 'stop': set(),
                 'open': {'side','speed','force'}, 'close': {'side','speed','force'},
                 'set_opening': {'side','opening','speed','force'},
                 'recover': {'side','confirm_stopped'}}
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
    args = normalize_arguments('recover', args)
    if args.get('side') not in ('left', 'right') or args.get('confirm_stopped') is not True:
        raise ValueError('检查实物已停止且无夹持物体后，选择原任务侧并勾选确认；此操作不发送急停或开合')
    return args


def percentage(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 <= value <= 100 or not math.isfinite(value):
        raise ValueError(name + '请填写0至100的数值')
    return value


def command(action, args):
    args = normalize_arguments(action, args)
    side = args.get('side')
    if side not in ('left', 'right'):
        raise ValueError('请选择左侧或右侧夹爪')
    if action not in MOTION_ACTIONS:
        raise ValueError('不支持的夹爪操作')
    permitted = {'action', 'side', 'speed', 'force', 'instance_id', '_trace_id'}
    if action == 'set_opening':
        permitted.add('opening')
    if set(args) - permitted:
        raise ValueError('当前操作包含不适用的参数')
    opening = percentage(args.get('opening'), '开口度') if action == 'set_opening' else 100 if action == 'open' else 0
    speed, force = percentage(args.get('speed'), '速度'), percentage(args.get('force'), '夹持力')
    if speed == 0:
        raise ValueError('执行开合时速度须大于0')
    # Never fill the other side with defaults: omission leaves it unchanged.
    return {side + '_opening': opening, side + '_speed': speed, side + '_force': force}


def state_data(data):
    if not isinstance(data, dict) or data.get('result') != 'success':
        raise ValueError('夹爪状态不可用或厂家报告故障')
    out = {}
    for side in ('left', 'right'):
        out[side] = {key: percentage(data.get(side + '_' + key), side + '_' + key)
                     for key in ('opening', 'speed', 'force')}
    stamp = data.get('timestamp')
    if stamp is not None and (isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or
                              not 0 <= stamp <= 2**63 - 1 or not math.isfinite(stamp)):
        raise ValueError('夹爪状态时间戳无效')
    return {'left': out['left'], 'right': out['right'], 'vendor_timestamp_ms': stamp,
            'received_at': time.time(), 'measurement_time_verified': False,
            'value_unit': '0_to_100_dimensionless', 'grasp_verified': False}


def tool(metadata):
    return {'name': 'gripper', 'type': 'actuator', 'multiInstance': False,
            'description': 'TRON2夹爪：选择左侧或右侧，张开、闭合或设置开口度。开口度0为最小闭合、100为最大张开；速度和夹持力为0至100的相对档位。读取状态不移动夹爪；达到目标开口不代表已夹住物体。',
            'inputSchema': {'type': 'object', 'additionalProperties': False,
                'properties': {
                    # New cards select recover with false confirmation, never a motor command.
                    'action': {'type': 'string', 'enum': ['recover', *MOTION_ACTIONS],
                               'description': 'open张开；close闭合；set_opening设置开口度；recover确认停止后恢复'},
                    'side': {'type': 'string', 'enum': ['left', 'right'], 'description': 'left左夹爪；right右夹爪'},
                    'opening': {'type': 'number', 'minimum': 0, 'maximum': 100, 'description': '目标开口度：0闭合，100张开'},
                    'speed': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 100, 'description': '开合速度档位：大于0至100'},
                    'force': {'type': 'number', 'minimum': 0, 'maximum': 100, 'description': '夹持力档位：0至100，并非牛顿'},
                    'confirm_stopped': {'type': 'boolean', 'enum': [False, True], 'default': False,
                                        'description': '检查实物停止且空载、周围无人手或障碍物后选择true；恢复不是急停'},
                    **metadata},
                'required': ['action'],
                'allOf': [{'if': {'properties': {'action': {'enum': list(MOTION_ACTIONS)}}, 'required': ['action']},
                           'then': {'required': ['side', 'speed', 'force']}},
                          {'if': {'properties': {'action': {'const': 'set_opening'}}, 'required': ['action']},
                           'then': {'required': ['opening']}},
                          {'if': {'properties': {'action': {'const': 'recover'}}, 'required': ['action']},
                           'then': {'required': ['side', 'confirm_stopped'], 'properties': {'confirm_stopped': {'const': True}}}}],
                'x-action-params': {
                    'open': {'params': ['side', 'speed', 'force'], 'description': '张开所选夹爪至最大开口'},
                    'close': {'params': ['side', 'speed', 'force'], 'description': '闭合所选夹爪；遇物体无法达到最小开口时不会报告目标到位或抓取成功'},
                    'set_opening': {'params': ['side', 'opening', 'speed', 'force'], 'description': '将所选夹爪调整至指定开口度'},
                    'recover': {'params': ['side', 'confirm_stopped'], 'description': '现场确认停止后，检查连续反馈并恢复失败任务；不补发动作，不把未到位改成成功'}},
                'x-resource': ['arm_l', 'arm_r'],
                'x-completion': {'actions': list(MOTION_ACTIONS), 'timeout': ACP_TIMEOUT}}}


class Gripper:
    def __init__(self, reader, report):
        self.reader, self.report = reader, report
        self.active = self.last = self.task = self.sample = None
        self.closed = False
        self.peer = None
        self.recovering = False

    def info(self):
        age = None if self.sample is None else max(0, time.monotonic() - self.sample[0])
        available = self.reader.connected and age is not None and age <= self.reader.settings['stale_after_seconds'] \
            and self.reader.ws is self.sample[2] and self.reader.serial == self.sample[3]
        motion = dict(self.last) if self.last and (not self.active or
                    self.last['action_id'] == self.active['action_id']) else None
        if self.active:
            motion = {**(motion or {}), **{k: self.active[k] for k in ('action_id', 'action', 'side', 'state')},
                      'target_opening': self.active['target']}
        return {'state': self.active['state'] if self.active else 'ready' if self.reader.connected else 'offline',
                'motion_sent': False, 'available': available,
                'reception_age_seconds': age, 'data': self.sample[1] if available else None,
                'motion': motion, 'recovering': self.recovering,
                'can_accept_motion': not self.closed and not self.active and not self.recovering and
                    self.reader.connected and not (self.peer and (self.peer.closed or self.peer.active)),
                'grasp_verified': False}

    async def read_state(self):
        data = state_data(await self.reader.read_gripper())
        self.sample = (time.monotonic(), data, self.reader.ws, self.reader.serial)
        return {**self.info(), 'read_only': True}

    async def recover(self, args):
        args = recovery_arguments(args)
        if self.closed or self.recovering or self.peer and (self.peer.closed or self.peer.active):
            raise MoveError('驱动关闭、正在恢复或机械臂任务未结束，不能恢复夹爪', False)
        job = self.active
        if job is None:
            return {**self.info(), 'read_only': True, 'recovered': False, 'message': '没有需要恢复的夹爪任务'}
        if job['side'] != args['side'] or job['state'] != 'blocked_requires_operator_check' or \
                self.task and not self.task.done() or not self.last or self.last['action_id'] != job['action_id']:
            raise MoveError('请选择原失败任务的侧；任务仍在执行或报告时不能恢复', False)
        self.recovering = True
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
                openings = [state[side]['opening'] for side in ('left', 'right')]
                fresh = current_stamp is not None and (stamp is None or current_stamp > stamp)
                stationary = all(state[side]['speed'] == 0 for side in ('left', 'right'))
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
            self.last['operator_recovery'] = {'confirmed_stopped': True, 'side': args['side'],
                'reported_stationary_verified': True, 'verified_samples': count,
                'observed_opening': state[args['side']]['opening'], 'vendor_timestamp_ms': stamp,
                'motion_sent': False, 'recovered_at': time.time()}
            self.active = None
            self.recovering = False
            LOG.info('Gripper task recovered after operator confirmation: %s', job['action_id'])
            return {**self.info(), 'read_only': True, 'recovered': True,
                    'message': '已恢复后续操作；原任务结果保留，未发送任何开合或急停指令'}
        finally:
            self.recovering = False

    async def dispatch(self, action, args):
        data = command(action, args)
        if self.active and self.active['state'] == 'reporting' and self.active.get('target_opening_verified'):
            await asyncio.shield(self.task)
        if self.peer and self.peer.active and self.peer.active['state'] == 'reporting' and self.peer.active.get('arrival_verified'):
            await asyncio.shield(self.peer.task)
        if self.peer and (self.peer.closed or self.peer.active):
            raise MoveError('机械臂动作进行中或结果尚未确认，暂不接受夹爪开合', False)
        if self.closed or self.active or self.recovering:
            raise MoveError('夹爪忙或上次动作结果尚未确认，请先检查状态', False)
        future = asyncio.get_running_loop().create_future()
        future.add_done_callback(lambda f: f.exception() if not f.cancelled() else None)
        job = {'action_id': 'tron2_gripper_' + uuid4().hex, 'action': action, 'side': args['side'],
               'target': data[args['side'] + '_opening'], 'state': 'checking', 'admission': future,
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
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
