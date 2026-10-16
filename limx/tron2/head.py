"""Explicit head pitch/yaw in degrees; feedback and ACP confirm completion."""
import asyncio
import math
import time

import completion
from completion import ArmCompletion
from skeleton import skeleton_stream
from vendor import validate_moveh, MoveError

COORDINATE_CONVENTION = 'pitch_up_positive_yaw_robot_left_positive'
# Rounded inward to 0.01 degree within the documented MoveH protocol limits.
# Mechanical limits remain unchanged; input is never clamped.
CARD_DEG_LIMITS = ((-59.58, 44.69), (-89.95, 89.95))


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
                'pitch_deg':{'type':'number','minimum':CARD_DEG_LIMITS[0][0],'maximum':CARD_DEG_LIMITS[0][1],'title':'俯仰角度（°）','description':'上正下负；-59.58～44.69°'},
                'yaw_deg':{'type':'number','minimum':CARD_DEG_LIMITS[1][0],'maximum':CARD_DEG_LIMITS[1][1],'title':'左右角度（°）','description':'自身左正右负；-89.95～89.95°'},
                'time':{'type':'number','minimum':0.1,'maximum':60,'default':5,'title':'运动时间（秒）'}},
            'required':['pitch_deg','yaw_deg'],
            'x-resource':['head','arm_l','arm_r'],
            # No action filter: calls without an action still register ACP.
            # Lifecycle/info responses never carry action_id and create no pending task.
            'x-completion':{'timeout':completion.ACP_TIMEOUT}}}


class HeadCompletion(ArmCompletion):
    joint_count = 2
    action_prefix = 'tron2_head_'
    require_ack = True

    async def dispatch(self, action, target, duration, details):
        target,duration=self.validate(target,duration)
        pitch,yaw=card_degrees(target)
        return await super().dispatch(action,target,duration,{**details,
            'target_pitch_deg':pitch,'target_yaw_deg':yaw,
            'target_vendor_rad':list(target),'coordinate_convention':COORDINATE_CONVENTION})

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

    async def send(self, target, duration):
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
