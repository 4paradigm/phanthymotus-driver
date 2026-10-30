"""Explicit head pitch/yaw in degrees; feedback and ACP confirm completion."""
import asyncio
import math
import time

import completion
from completion import ArmCompletion
from skeleton import skeleton_stream
from vendor import validate_moveh, MoveError, HEAD_DEG_LIMITS, HEAD_RAD_LIMITS, HEAD_YAW_COMMAND_LIMIT_DEG, motion_readiness

RECOVERY_TIMEOUT = 3
COORDINATE_CONVENTION = 'pitch_up_positive_yaw_robot_left_positive'
CARD_DEG_LIMITS = ((-HEAD_DEG_LIMITS[0][1], -HEAD_DEG_LIMITS[0][0]), (-HEAD_YAW_COMMAND_LIMIT_DEG, HEAD_YAW_COMMAND_LIMIT_DEG))


def card_degrees(vendor_radians):
    """Customer angles; raw robot/skeleton coordinates remain untouched."""
    return [-math.degrees(vendor_radians[0]), math.degrees(vendor_radians[1])]


def arguments(args):
    values = []
    for key, (low, high) in zip(('pitch_deg','yaw_deg'), CARD_DEG_LIMITS):
        value = args.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(key + '须填写角度数值（°）')
        try:
            valid = math.isfinite(value) and low <= value <= high
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(key + '超出当前头部角度范围：' + str(low) + '至' + str(high) + '°；未发送转动指令')
        values.append(math.radians(-value if key=='pitch_deg' else value))
    target, duration = validate_moveh(values, args.get('time', 5))
    return target, duration


def tool():
    return {'name':'move_head','type':'actuator','multiInstance':False,
        'description':'转到指定头部角度（相对零位），到位后通知完成。',
        'inputSchema':{'type':'object','additionalProperties':False,
            'properties':{
                'pitch_deg':{'type':'number','minimum':CARD_DEG_LIMITS[0][0],'maximum':CARD_DEG_LIMITS[0][1],'title':'俯仰角度（°）','description':'上正下负；-60～45°'},
                'yaw_deg':{'type':'number','minimum':CARD_DEG_LIMITS[1][0],'maximum':CARD_DEG_LIMITS[1][1],'title':'左右角度（°）','description':'自身左正右负；-88～88°'},
                'time':{'type':'number','minimum':0.1,'maximum':60,'default':5,'title':'运动时间（秒）'}},
            'required':['pitch_deg','yaw_deg'],
            'x-resource':['head','arm_l','arm_r'],
            # No action filter: calls without an action still register ACP.
            # Lifecycle/info responses never carry action_id and create no pending task.
            'x-completion':{'timeout':completion.ACP_TIMEOUT}},
        'topic_in':[{'format':'control/joint','desc':'通用人物跟随的协商头部目标'}],
        'topic_out':[{'topic':'/limx_tron2/head_state','format':'data/json','desc':'头部实际反馈与动作完成状态'}]}


class HeadCompletion(ArmCompletion):
    joint_count = 2
    action_prefix = 'tron2_head_'
    require_ack = True

    def __init__(self, reader, report):
        super().__init__(reader, report)
        self.admission_lock = asyncio.Lock()
        self.recovering = False
        self.stream_guard = None

    async def dispatch(self, action, target, duration, details):
        # Invalid new input must never clear a previous task.
        target,duration=self.validate(target,duration)
        if self.admission_lock.locked():
            raise MoveError('头部正在审核另一条指令，请稍后再试',False,'head_admission_busy')
        async with self.admission_lock:
            recovered = await self.recover_stationary_failure()
            if not self.active:
                self.validate_admission_feedback()
            pitch,yaw=card_degrees(target)
            return await super().dispatch(action,target,duration,{**details,
                **({'previous_action_error':recovered} if recovered else {}),
                'target_pitch_deg':pitch,'target_yaw_deg':yaw,
                'target_vendor_rad':list(target),'coordinate_convention':COORDINATE_CONVENTION})

    def recovery_identity(self, job):
        peers = self.peers + ([self.peer] if self.peer else [])
        if self.closed or self.active is not job or not self.reader.connected or \
                self.reader.ws is not job['ws'] or self.reader.serial != job['serial']:
            raise MoveError('头部任务或连接已改变，不能自动解除旧任务',False,'head_recovery_changed')
        if any(peer.closed or peer.active or getattr(peer,'recovering',False) or getattr(peer,'waiting',None) for peer in peers):
            raise MoveError('其他运动任务尚未解除，未发送头部指令',False,'head_recovery_peer_busy')
        status = self.reader.sample('robot_status')
        if not status['available'] or not motion_readiness(status['data'])['ready']:
            raise MoveError('机器人运行状态未通过审核，旧任务保持保护',False,'head_recovery_not_ready')

    async def recover_stationary_failure(self):
        job, last = self.active, self.last or {}
        if not job or job['state'] != 'blocked_requires_operator_check':
            return None
        # Only a delivered, acknowledged target-miss is recoverable. Unknown ACK,
        # connection changes, reporting failures and ongoing tasks remain protected.
        if last.get('action_id') != job['action_id'] or last.get('status') != 'error' or \
                last.get('error_code') != 'target_not_reached' or last.get('vendor_acknowledged') is not True or \
                last.get('callback_delivered') is not True or last.get('motion_sent') is not True or \
                last.get('arrival_verified') is not False or not self.task or not self.task.done():
            raise self.blocked_error()
        self.recovering = True
        try:
            self.recovery_identity(job)
            start_sample, _, _ = self.feedback()
            start_stamp = start_sample['data']['robot_timestamp']
            await self.reader.head_control_status()  # Read-only fresh VR/mode check.
            self.recovery_identity(job)
            deadline=time.monotonic()+RECOVERY_TIMEOUT; window=[]; stamp=start_stamp
            while time.monotonic()<deadline:
                self.recovery_identity(job)
                self.validate_admission_feedback()
                sample=self.reader.sample('joint_state'); packet=skeleton_stream(sample)
                if not packet['status']['available'] or len(packet['joints'])!=16:
                    raise MoveError('缺少新鲜完整关节反馈，旧任务保持保护',False,'head_recovery_feedback')
                q=[j['q'] for j in packet['joints']]; dq=[j.get('dq') for j in packet['joints']]
                current=sample['data']['robot_timestamp']; now=time.monotonic()
                if any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>completion.VELOCITY_RAW_TOLERANCE for v in dq):
                    raise MoveError('关节仍在运动，旧头部任务保持保护',False,'head_recovery_not_stationary')
                if stamp is not None and current<stamp:
                    raise MoveError('反馈时间戳回退，旧任务保持保护',False,'head_recovery_feedback')
                if stamp is None or current>stamp:
                    stamp=current
                    if window and max(abs(math.degrees(a-b)) for a,b in zip(q,window[0][1]))>completion.STABILITY_DEG:
                        raise MoveError('关节位置仍在变化，旧任务保持保护',False,'head_recovery_not_stationary')
                    window.append((now,q))
                    if len(window)>=3 and now-window[0][0]>=completion.STABILITY_SECONDS:
                        self.recovery_identity(job)
                        record={key:last[key] for key in ('action_id','action','status','error','error_code',
                            'target_pitch_deg','target_yaw_deg','arrival_verified','callback_delivered') if key in last}
                        record['recovery']={'reason':'fresh_stationary_feedback_before_new_command',
                            'verified_samples':len(window),'observed_seconds':now-window[0][0],
                            'vendor_timestamp':current,'motion_sent':False}
                        # Preserve the failed result, never report it as completed.
                        self.last={**last,'recovery':record['recovery']}
                        self.active=None
                        return record
                await asyncio.sleep(.05)
            raise MoveError('未取得连续静止新反馈，旧任务保持保护',False,'head_recovery_timeout')
        finally:
            self.recovering=False

    def summary(self):
        result = dict(self.last or {})
        if self.active:
            job = self.active
            if result.get('action_id') != job['action_id']:
                result = {}
            result.update({key:job[key] for key in ('action_id','action','state')})
            result.update(job['details'])
            result['target_pitch_deg'],result['target_yaw_deg'] = card_degrees(job['target'])
            result['time'] = job['duration']
        if not result:
            return None
        result['recovering'] = self.recovering
        try:
            sample, q, dq = self.feedback()
            pitch,yaw=card_degrees(q)
            result.update(current_pitch_deg=pitch,current_yaw_deg=yaw,current_vendor_rad=q,
                          current_velocity_raw=dq,feedback_available=True,
                          reception_age_seconds=sample['reception_age_seconds'])
        except ValueError:
            result['feedback_available'] = False
        peers = self.peers + ([self.peer] if self.peer else [])
        result['can_accept_motion'] = not self.closed and not self.active and not any(
            peer.closed or peer.active or getattr(peer,'recovering',False) or getattr(peer,'waiting',None)
            for peer in peers)
        return result

    def blocked_error(self):
        summary = self.summary() or {}
        if self.closed:
            return MoveError('头部驱动已停止，未发送新的转动指令',False,'head_closed')
        reason = summary.get('error') or '上一条头部任务仍在执行或等待完成确认'
        return MoveError('头部上一条任务尚未解除：'+reason+'；未发送新的转动指令',
                         False,'head_requires_operator_check' if summary.get('state')=='blocked_requires_operator_check' else 'head_busy')

    def validate(self, target, duration):
        return validate_moveh(target, duration)

    def feedback(self):
        sample = self.reader.sample('joint_state')
        packet = skeleton_stream(sample)
        if not packet['status']['available']:
            raise ValueError('Fresh supported head feedback is required')
        joints = {j['idx']:j for j in packet['joints']}
        q = [joints[i]['q'] for i in (14,15)]
        dq = [joints[i].get('dq') for i in (14,15)]
        if any(value is None for value in dq):
            raise ValueError('Head velocity feedback is required')
        stamp = sample['data'].get('robot_timestamp')
        if type(stamp) not in (int,float) or not math.isfinite(stamp):
            raise ValueError('Head feedback timestamp is required')
        return sample, q, dq

    def validate_admission_feedback(self):
        # A fresh, stationary observation is an admission gate, not arrival proof.
        # Refusal occurs before task allocation (and is repeated before send).
        _, q, dq = self.feedback()
        tolerance = math.radians(completion.TOLERANCE_DEG)
        if any(not low-tolerance <= value <= high+tolerance
               for value, (low, high) in zip(q, HEAD_RAD_LIMITS)):
            raise MoveError('头部反馈超出机械范围，未发送转动指令',False,'head_feedback_out_of_range')
        if any(abs(value)>completion.VELOCITY_RAW_TOLERANCE for value in dq):
            raise MoveError('头部仍在运动，请静止后再执行；未发送转动指令',False,'head_not_stationary')

    async def send(self, target, duration):
        self.validate_admission_feedback()
        token = (self.active or {}).get('details',{}).get('head_stream')
        if token is not None:
            if self.stream_guard is None:
                raise MoveError('头部数据流保护未配置',False)
            return await self.reader.send_moveh(target, duration, guard=lambda:self.stream_guard(token))
        return await self.reader.send_moveh(target, duration)

    async def wait_for_arrival(self, job):
        sample, _, _ = self.feedback()
        stamp = sample['data']['robot_timestamp']
        not_before = time.monotonic() + job['duration']
        deadline = not_before + completion.ARRIVAL_GRACE
        window = []
        observed = None
        while time.monotonic() < deadline:
            if not self.reader.connected or self.reader.ws is not job['ws'] or self.reader.serial != job['serial']:
                raise ValueError('Head connection changed during movement; result unknown')
            sample, q, dq = self.feedback()
            observed = card_degrees(q)
            current = sample['data']['robot_timestamp']
            now = time.monotonic()
            if current > stamp:
                stamp = current
                error = max(abs(math.degrees(a-b)) for a,b in zip(q,job['target']))
                if now < not_before or error > completion.TOLERANCE_DEG or any(abs(v)>completion.VELOCITY_RAW_TOLERANCE for v in dq):
                    window = []
                else:
                    if window and max(abs(math.degrees(a-b)) for a,b in zip(q,window[0][1])) > completion.STABILITY_DEG:
                        window = []
                    window.append((now,q))
                    if len(window)>=3 and now-window[0][0]>=completion.STABILITY_SECONDS:
                        return {'arrival_verified':True,'head_target_verified':True,
                            'reported_pitch_deg':observed[0],'reported_yaw_deg':observed[1],
                            'reported_vendor_rad':list(q),
                            'max_error_deg':error,'tolerance_deg':completion.TOLERANCE_DEG,
                            'verified_samples':len(window),'verification':'post_duration_advancing_head_feedback'}
            else:
                # Repeated timestamps never count toward arrival.
                if current < stamp:
                    raise ValueError('Head feedback timestamp regressed')
            await asyncio.sleep(.05)
        current = ('无有效反馈' if observed is None else '俯仰%.2f°、偏航%.2f°'%tuple(observed))
        target = '俯仰%.2f°、偏航%.2f°'%tuple(card_degrees(job['target']))
        raise MoveError('头部未确认到位：目标'+target+'，最近反馈'+current+'；保持保护，请检查实物与控制器',True,'target_not_reached')
