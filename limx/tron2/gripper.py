"""LimX two-finger gripper: fixed side, correlated status and ACP completion."""
import asyncio
import math
import logging
import time
from uuid import uuid4

from vendor import MoveError

GRIPPER_SIDES = {'gripper_left': 'left', 'gripper_right': 'right'}
MAX_OPENING = 94  # Operator-selected control limit; raw vendor feedback stays 0..100.
MAX_OPENING_MM = 85
OPENING_MAPPING = 'nominal_linear_0_85mm_to_0_94_raw'
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
                 'open': {'opening_mm','speed','force'}, 'close': {'speed','force'},
                 'recover': set()}
METADATA_FIELDS = {'action','instance_id','_trace_id'}


def normalize_arguments(action, args):
    if not isinstance(args, dict) or action not in ACTION_FIELDS:
        raise ValueError('不支持的夹爪操作或参数格式')
    known = METADATA_FIELDS | set().union(*ACTION_FIELDS.values()) | {'opening'}
    if set(args) - known:
        raise ValueError('当前操作包含未声明的参数')
    if 'action' in args and args['action'] != action:
        raise ValueError('操作与参数中的action不一致')
    if action == 'open' and 'opening' in args:
        raise ValueError('旧opening参数已停用；请刷新卡片，使用opening_mm填写0至85mm开口大小')
    # Some dashboard versions submit inputs hidden by x-action-params. Ignore
    # only declared, inactive fields; never reinterpret them as motor values.
    return {key:value for key,value in args.items() if key in ACTION_FIELDS[action] | METADATA_FIELDS}


def recovery_arguments(args):
    return normalize_arguments('recover', args)


def percentage(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 <= value <= 100 or not math.isfinite(value):
        raise ValueError(name + '请填写0至100的数值')
    return value


def opening_target(value):
    value = percentage(value, '开口度')
    if value > MAX_OPENING:
        raise ValueError('目标开口度请填写0至'+str(MAX_OPENING)+'；当前最大张开目标为'+str(MAX_OPENING))
    return value


def opening_mm(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or \
            not 0 <= value <= MAX_OPENING_MM or not math.isfinite(value):
        raise ValueError('夹爪开口大小请填写0至85mm的数值')
    return value


def raw_from_mm(value):
    return round(opening_mm(value)*MAX_OPENING/MAX_OPENING_MM,6)


def mm_estimate(raw):
    return round(raw*MAX_OPENING_MM/MAX_OPENING,3)


def position_text(raw):
    return '尚无反馈' if raw is None else str(mm_estimate(raw))+'mm（底层'+str(raw)+'）'


def command(action, args, side):
    args = normalize_arguments(action, args)
    if side not in ('left', 'right'):
        raise ValueError('请选择左侧或右侧夹爪')
    if action not in MOTION_ACTIONS:
        raise ValueError('不支持的夹爪操作')
    opening = raw_from_mm(args.get('opening_mm')) if action == 'open' else 0
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
        out[side]['opening_mm_estimate'] = mm_estimate(out[side]['opening'])
    stamp = data.get('timestamp')
    if stamp is not None and (isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or
                              not 0 <= stamp <= 2**63 - 1 or not math.isfinite(stamp)):
        raise ValueError('夹爪状态时间戳无效')
    return {**out, 'vendor_timestamp_ms': stamp,
            'received_at': time.time(), 'measurement_time_verified': False,
            'value_unit': '0_to_100_dimensionless', 'opening_mapping': OPENING_MAPPING,
            'opening_mm_is_calibrated': False, 'grasp_verified': False}


def tool(metadata, side):
    label = '左' if side == 'left' else '右'
    return {'name': 'gripper_' + side, 'type': 'actuator', 'multiInstance': False,
            'description': 'TRON2' + label + '夹爪：open按毫米设置开口大小，0mm为最小闭合、85mm为最大目标；驱动自动线性换算为底层0至94。毫米值采用名义换算，实际间距精度待标定。close闭合，recover检查静止反馈后恢复本侧失败任务。速度/力为相对档位，并非牛顿。达到开口不代表夹住物体。恢复前须现场确认静止空载；恢复不是急停。',
            'inputSchema': {'type': 'object', 'additionalProperties': False,
                'properties': {
                    'action': {'type': 'string', 'enum': ['open', 'close', 'recover'],
                               'description': 'open设置目标开口；close闭合；recover恢复本侧失败任务'},
                    'opening_mm': {'type': 'number', 'title': '夹爪开口大小（mm）', 'minimum': 0, 'maximum': MAX_OPENING_MM, 'description': '夹爪开口大小（mm）：0至85；自动换算到底层0至94，实际间距精度待标定'},
                    'speed': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 100, 'description': '开合速度档位：大于0至100'},
                    'force': {'type': 'number', 'minimum': 0, 'maximum': 100, 'description': '夹持力档位：0至100，并非牛顿'},
                    **metadata},
                'required': ['action'],
                'allOf': [{'if': {'properties': {'action': {'enum': list(MOTION_ACTIONS)}}, 'required': ['action']},
                           'then': {'required': ['speed', 'force']}},
                          {'if': {'properties': {'action': {'const': 'open'}}, 'required': ['action']},
                           'then': {'required': ['opening_mm']}}],
                'x-action-params': {
                    'open': {'params': ['opening_mm', 'speed', 'force'], 'description': '按填写的毫米数调整本侧夹爪开口大小'},
                    'close': {'params': ['speed', 'force'], 'description': '闭合本侧夹爪；未到最小开口不会报告到位或抓取成功'},
                    'recover': {'params': [], 'description': '现场确认本侧静止空载后使用；连续反馈验证后恢复，不重发动作、不把未到位改成成功'}},
                'x-resource': ['arm_l' if side == 'left' else 'arm_r'],
                'x-completion': {'actions': list(MOTION_ACTIONS), 'timeout': ACP_TIMEOUT}}}


def combined_tool():
    """One customer entry; side selection never changes the two motion owners."""
    return {'name':'gripper','type':'actuator','multiInstance':False,
        'description':'设置所选夹爪开口：0mm闭合，85mm最大目标。毫米为名义线性换算，实际间距待标定；力为相对档位，不是牛顿。到位不代表抓取成功。',
        'inputSchema':{'type':'object','additionalProperties':False,
            'properties':{
                'side':{'type':'string','enum':['left','right'],'title':'左右爪','description':'left左爪；right右爪'},
                'opening_mm':{'type':'number','minimum':0,'maximum':85,'title':'开口大小（mm）','description':'0至85mm；0为闭合'},
                'speed':{'type':'number','exclusiveMinimum':0,'maximum':100,'description':'速度档位：大于0至100'},
                'force':{'type':'number','minimum':0,'maximum':100,'description':'力档位：0至100，并非牛顿'}},
            'required':['side','opening_mm','speed','force'],
            # Core currently supports static resource declarations only.
            'x-resource':['arm_l','arm_r'], 'x-completion':{'timeout':ACP_TIMEOUT}}}


def combined_arguments(args):
    """Validate everything before waiting, recovery, ownership or hardware I/O."""
    if not isinstance(args,dict) or set(args)-{'side','opening_mm','speed','force','instance_id','_trace_id'}:
        raise ValueError('请刷新夹爪卡片；只接受左右爪、开口大小、速度和力档位')
    side=args.get('side')
    if side not in ('left','right'):
        raise ValueError('请选择left左爪或right右爪')
    values={k:v for k,v in args.items() if k!='side'}
    command('open',values,side)
    # Zero millimetres retains the existing guarded close handover internally.
    action='close' if args['opening_mm']==0 else 'open'
    return side,action,normalize_arguments(action,values)


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
        self.handover_cancel = None
        self.pose_owner = None
        self.pose_step = self.pose_last = None

    def info(self):
        active=self.active or self.pose_step
        age = None if self.sample is None else max(0, time.monotonic() - self.sample[0])
        available = self.reader.connected and age is not None and age <= self.reader.settings['stale_after_seconds'] \
            and self.reader.ws is self.sample[2] and self.reader.serial == self.sample[3]
        motion = dict(self.last) if self.last and (not active or
                    self.last['action_id'] == active['action_id']) else None
        if active:
            motion = {**(motion or {}), **{k: active[k] for k in ('action_id', 'action', 'side', 'state')},
                      'target_opening': active['target'], 'target_opening_raw': active['target'],
                      'target_opening_mm': active['target_mm']}
        return {'side': self.side, 'state': active['state'] if active else
                    'recovering' if self.recovering else 'waiting_for_previous' if self.waiting else
                    'ready' if self.reader.connected else 'offline',
                'motion_sent': False, 'available': available,
                'reception_age_seconds': age, 'data': self.sample[1] if available else None,
                'motion': motion, 'recovering': self.recovering,
                'waiting_action': self.waiting['action'] if self.waiting else
                    'close' if self.handover_cancel else None,
                'can_accept_motion': not self.closed and not active and not self.recovering and
                    not self.waiting and
                    self.reader.connected and not (self.peer and (self.peer.closed or self.peer.active)),
                'pose_step_result':self.pose_last,'grasp_verified': False}

    async def read_state(self):
        data = state_data(await self.reader.read_gripper(), self.side)
        self.sample = (time.monotonic(), data, self.reader.ws, self.reader.serial)
        return {**self.info(), 'read_only': True}

    async def prepare_pose(self):
        """Read-only admission for the fixed right-jaw part of a pose."""
        def idle():
            if (self.side!='right' or self.closed or self.active or self.pose_step or self.recovering or
                    self.waiting or self.handover_cancel or self.pose_owner is None or
                    self.peer and (self.peer.closed or self.peer.active)):
                raise MoveError('挥手需要右夹爪及其他运动任务均已结束',False)
        idle();ws,serial=self.reader.ws,self.reader.serial
        state=(await self.read_state())['data'];idle()
        if (self.reader.ws is not ws or self.reader.serial!=serial or not self.reader.connected or
                state['vendor_timestamp_ms'] is None or state['right']['speed']!=0):
            raise MoveError('右夹爪反馈不完整或未静止；未开始挥手',False)

    async def execute_pose_step(self,parent,owner,frame,checkpoint):
        """Private child operation under the exact live arm/pose owner; no nested ACP."""
        def owned():
            if (self.pose_owner is not parent or parent.active is not owner or parent.closed or
                    owner.get('details',{}).get('motion_tool')!='pose' or
                    owner.get('details',{}).get('preset')!='wave' or owner.get('stage_kind')!='gripper' or
                    self.side!='right' or self.closed or self.active or self.recovering or self.waiting or self.handover_cancel):
                raise MoveError('夹爪步骤不属于当前挥手任务；未发送开合',False)
            checkpoint()
        owned()
        if self.pose_step is not None:raise MoveError('已有未确认的姿态夹爪步骤',False)
        values=frame['gripper'];data=command('open',{k:v for k,v in values.items() if k!='side'},self.side)
        step={'action_id':owner['action_id'],'parent_action_id':owner['action_id'],
            'action':'close' if values['opening_mm']==0 else 'open','side':self.side,
            'target':data['right_opening'],'target_mm':values['opening_mm'],
            'ws':owner['ws'],'serial':owner['serial'],'motion_sent':False,
            'vendor_acknowledged':False,'state':'checking'}
        self.pose_step=step;status='error'
        try:
            before=(await self.read_state())['data'];owned();self.same_session(step)
            step['before_timestamp']=before['vendor_timestamp_ms']
            if step['before_timestamp'] is None or before['right']['speed']!=0:
                raise MoveError('夹爪步骤前状态未确认静止；未发送新的开合',False)
            if owner['cancel_sequence']:
                status='cancelled'
                return {'motion_sent':False,'cancelled_before_send':True}
            step['no_op']=abs(before['right']['opening']-step['target'])<=TOLERANCE
            if not step['no_op']:
                step.update(state='awaiting_ack',motion_sent=None)
                reply=await self.reader.send_gripper(data)
                step['motion_sent']=True
                if not isinstance(reply,dict) or reply.get('result')!='success':
                    raise MoveError('挥手夹爪步骤回执不明；不执行后续步骤',True)
                step['vendor_acknowledged']=True
            step['state']='verifying_existing_target' if step['no_op'] else 'moving'
            # Lifecycle stop waits for THIS sent operation to settle; it never
            # creates the next close/open or forces an arm return.
            result=await self.verify_target(step,checkpoint=owned)
            owned();status='completed'
            result.update(action=step['action'],side=self.side,target_opening_mm=step['target_mm'],
                target_opening_raw=step['target'],reported_speed=step.get('reported_speed'),
                motion_sent=step['motion_sent'],vendor_acknowledged=step['vendor_acknowledged'],
                parent_action_id=owner['action_id'],completion_owner='pose')
            self.pose_last=result
            return result
        except (Exception,asyncio.CancelledError) as exc:
            if isinstance(exc,MoveError):step['motion_sent']=exc.motion_sent
            step['state']='blocked_requires_operator_check'
            self.pose_last={'parent_action_id':owner['action_id'],'status':'error',
                'motion_sent':step['motion_sent'],'target_opening_verified':False,
                'target_opening_mm':step['target_mm'],'completion_owner':'pose',
                'error':str(exc) or 'Pose gripper step cancelled/timed out',
                'error_code':getattr(exc,'code',None)}
            owner['gripper_step_error']=dict(self.pose_last)
            raise
        finally:
            if status in ('completed','cancelled') or step['motion_sent'] is False:
                self.pose_step=None

    def check_close_handover(self, job):
        last = self.last or {}
        if job.get('action') != 'open' or last.get('action_id') != job['action_id'] or \
                last.get('status') != 'error' or last.get('error_code') != 'target_not_reached' or \
                job.get('vendor_acknowledged') is not True or last.get('motion_sent') is not True:
            raise self.blocked_error()
        self.same_session(job)

    async def recover(self, args, *, close_handover=False):
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
        if close_handover:
            self.check_close_handover(job)
        self.recovering = True
        # Completion belongs to this recovery operation, not its HTTP caller's
        # lifetime (which may continue after recover returns).
        self.recovery_task = asyncio.get_running_loop().create_future()
        cancellation = asyncio.Event()
        if close_handover:
            self.handover_cancel = cancellation
        try:
            ws, serial = self.reader.ws, self.reader.serial
            deadline = time.monotonic() + RECOVERY_TIMEOUT
            start = None; count = 0; previous = None; stamp = None
            while time.monotonic() < deadline:
                if close_handover and (self.closed or cancellation.is_set()):
                    raise MoveError('已取消关闭接替检查，未发送关闭指令', False)
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
                if close_handover:
                    # Callback delivery can take seconds. Recheck the robot
                    # before using the earlier stationary proof for a close.
                    state = (await self.read_state())['data']
                    current_stamp = state['vendor_timestamp_ms']
                    if current_stamp is None or current_stamp <= stamp or \
                            state[self.side]['speed'] != 0 or \
                            abs(state[self.side]['opening']-previous[0]) > TOLERANCE:
                        raise ValueError('回调后静止反馈变化，关闭接替未放行')
                    stamp = current_stamp
                    count += 1
            if self.closed or self.reader.ws is not ws or self.reader.serial != serial or not self.reader.connected or \
                    self.peer and (self.peer.closed or self.peer.active) or cancellation.is_set():
                raise ValueError('恢复期间驱动、连接或机械臂状态变化，未解除任务保护')
            if close_handover:
                self.check_close_handover(job)
            self.last['close_handover' if close_handover else 'operator_recovery'] = {'side': self.side,
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
            self.handover_cancel = None

    def cancel_waiting(self):
        # Cancels only a request not yet admitted. Never claims to stop a motor.
        if self.waiting:
            self.waiting['cancel'].set()
        if self.handover_cancel:
            self.handover_cancel.set()
        if self.pose_step and self.pose_owner and self.pose_owner.active and \
                self.pose_step['parent_action_id']==self.pose_owner.active['action_id']:
            self.pose_owner.cancel_sequence()

    def blocked_error(self):
        job = self.active or {}
        last = self.last or {}
        label = '左' if self.side == 'left' else '右'
        return MoveError(label + '夹爪上一条' + str(job.get('action', '动作')) +
            '结果尚未确认：目标' + position_text(job.get('target')) + '，最近反馈' +
            position_text(last.get('reported_opening', job.get('reported_opening'))) +
            '；后续指令未发送。请检查实物与控制器，保留失败记录后联系维护人员恢复', False)

    async def wait_previous(self, action, allow_blocked=False):
        if self.pose_step is not None:
            raise MoveError('当前夹爪步骤属于姿态任务或结果未确认，不能由单独开合/恢复接替',False)
        if self.closed:
            raise MoveError('夹爪驱动已关闭，后续指令未发送', False)
        if self.waiting or self.handover_cancel:
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
        requested_mm = opening_mm(args.get('opening_mm')) if action == 'open' else 0
        await self.wait_previous(action, allow_blocked=action == 'close')
        previous_error = None
        if self.active:
            failed = self.active
            # A NEW explicit close may supersede only an acknowledged open
            # that missed its target. Preserve its failed outcome; never retry
            # an uncertain ACK, reinterpret a fault, or call94 an achieved100.
            self.check_close_handover(failed)
            await self.recover({}, close_handover=True)
            self.same_session(failed)
            previous_error = {k:self.last[k] for k in ('action_id','action','status','error','error_code',
                'target_opening','reported_opening','target_opening_verified','callback_delivered','close_handover',
                'stationary_target_miss','target_opening_mm','target_opening_raw','reported_opening_mm_estimate')
                if k in self.last}
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
               'target_mm': requested_mm,
               'ws': self.reader.ws, 'serial': self.reader.serial, 'motion_sent': False,
               'vendor_acknowledged': False}
        if previous_error:
            job['previous_action_error'] = previous_error
        self.active = job
        self.task = asyncio.create_task(self._run(job, data))
        return await asyncio.shield(future)

    def same_session(self, job):
        if not self.reader.connected or self.reader.ws is not job['ws'] or self.reader.serial != job['serial']:
            raise ValueError('机器人连接已改变，动作结果尚未确认')

    async def verify_target(self, job, checkpoint=None):
        start = None; matches = 0
        replacement = None; stationary_start = None; stationary_samples = 0; stationary_opening = None
        last_stamp = job['before_timestamp']
        deadline = time.monotonic() + TIMEOUT
        while time.monotonic() < deadline:
            self.same_session(job)
            if checkpoint is not None:checkpoint()
            result = await self.read_state()  # Each sample is a new correlated query.
            self.same_session(job)
            if checkpoint is not None:checkpoint()
            actual = result['data'][job['side']]['opening']
            job['reported_opening'] = actual
            job['reported_speed'] = result['data'][job['side']]['speed']
            stamp = result['data']['vendor_timestamp_ms']
            now = time.monotonic()
            if stamp is None or stamp <= last_stamp:
                start = None; matches = 0
                stationary_start = None; stationary_samples = 0; stationary_opening = None
                await asyncio.sleep(POLL_SECONDS)
                continue
            last_stamp = stamp
            if abs(actual - job['target']) <= TOLERANCE and \
                    (not (job.get('no_op') or job.get('parent_action_id')) or job['reported_speed'] == 0):
                start = now if start is None else start
                matches += 1
                if matches >= 3 and now - start >= STABILITY_SECONDS:
                    return {'target_opening_verified': True, 'reported_opening': actual,
                            'verified_samples': matches, 'grasp_verified': False,
                            'verification': 'already_at_target_fresh_feedback' if job.get('no_op') else 'post_ack_correlated_vendor_state',
                            'no_op': bool(job.get('no_op'))}
            else:
                start = None; matches = 0
            # An explicit queued close need not wait out the full open timeout
            # when fresh feedback already proves the acknowledged open is
            # stationary short of its target. Keep the original outcome an
            # error; normal recovery and ACP delivery still guard the close.
            waiting = self.waiting
            can_replace = job['action'] == 'open' and job.get('vendor_acknowledged') is True and \
                job['motion_sent'] is True and not job.get('no_op') and waiting and \
                waiting['action'] == 'close' and not waiting['cancel'].is_set() and not self.closed and \
                abs(actual-job['target']) > TOLERANCE and job['reported_speed'] == 0
            if can_replace:
                if replacement is not waiting or stationary_opening is None or \
                        abs(actual-stationary_opening) > TOLERANCE:
                    replacement = waiting; stationary_start = now
                    stationary_samples = 0; stationary_opening = actual
                stationary_samples += 1
                if stationary_samples >= 3 and now-stationary_start >= RECOVERY_STABILITY_SECONDS:
                    job['stationary_target_miss'] = {'verified_samples': stationary_samples,
                        'observed_opening': actual, 'vendor_timestamp_ms': stamp,
                        'reason': 'explicit_close_waiting_after_stationary_open'}
                    raise MoveError('收到新的close；原open目标'+position_text(job['target'])+'尚未到位，反馈'+position_text(actual)+
                                    '已连续静止；原open按失败结束，关闭仍须通过接替检查',
                                    True, 'target_not_reached')
            else:
                replacement = None; stationary_start = None; stationary_samples = 0; stationary_opening = None
            await asyncio.sleep(POLL_SECONDS)
        raise MoveError('未确认达到目标开口'+position_text(job['target'])+'，最近反馈'+position_text(job.get('reported_opening'))+
                        '；请检查实物与控制器并联系维护人员恢复。未确认抓取成功',
                        job['motion_sent'], 'target_not_reached')

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
                    job['vendor_acknowledged'] = True
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
                'target_opening_raw': job['target'], 'target_opening_mm': job['target_mm'],
                'opening_mapping': OPENING_MAPPING, 'opening_mm_is_calibrated': False,
                'motion_sent': job['motion_sent'], 'no_op': job['no_op'],
                'target_opening_verified': False, 'grasp_verified': False,
                'can_accept_motion': False,
                'message': '指令已受理，仍在等待目标反馈和完成回调；本侧下一条指令会等待本条确认',
                **({'previous_action_error': job['previous_action_error']} if 'previous_action_error' in job else {}),
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
            result, status = {'error': str(exc), 'error_code': getattr(exc, 'code', None)}, 'error'
        if not job['admission'].done():
            job['admission'].set_result({'action_id': job['action_id'], 'state': 'result_unconfirmed',
                'motion_sent': job['motion_sent'], 'target_opening_verified': False,
                'grasp_verified': False, 'completion_protocol': 'acp', 'message': result['error']})
        result.update(action=job['action'], side=job['side'], target_opening=job['target'],
                      target_opening_raw=job['target'], target_opening_mm=job['target_mm'],
                      opening_mapping=OPENING_MAPPING, opening_mm_is_calibrated=False,
                      motion_sent=job['motion_sent'], grasp_verified=False,
                      motion_may_continue=status != 'completed' and job['motion_sent'] is not False,
                      no_op=bool(job.get('no_op')), vendor_acknowledged=job['vendor_acknowledged'])
        if 'previous_action_error' in job:
            result['previous_action_error'] = job['previous_action_error']
        if 'stationary_target_miss' in job:
            result['stationary_target_miss'] = job['stationary_target_miss']
        result.setdefault('target_opening_verified', False)
        result.update({k: job[k] for k in ('reported_opening','reported_speed') if k in job})
        if 'reported_opening' in job:
            result['reported_opening_mm_estimate'] = mm_estimate(job['reported_opening'])
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
