"""Explicit head pitch/yaw in degrees; feedback and ACP confirm completion."""
import asyncio
import math
import time

import completion
from completion import ArmCompletion
from skeleton import skeleton_stream
from vendor import validate_moveh


def arguments(args):
    values = []
    for key, low, high in (('pitch_deg', -45, 60), ('yaw_deg', -90, 90)):
        value = args.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(key + '须填写角度数值（°）')
        try:
            valid = math.isfinite(value) and low <= value <= high
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(key + '超出手册限位：' + str(low) + '至' + str(high) + '°')
        values.append(math.radians(value))
    target, duration = validate_moveh(values, args.get('time', 5))
    return target, duration


def tool(metadata):
    return {'name':'move_head','type':'actuator','multiInstance':False,
        'description':'TRON2头部转动：填写俯仰/偏航绝对角度（°）和时间（秒）。仅控制头部两轴，不移动双臂或开合夹爪；连续新反馈确认到位后通过ACP通知完成。正负方向按厂家坐标系。',
        'inputSchema':{'type':'object','additionalProperties':False,
            'properties':{
                'action':{'type':'string','enum':['move']},
                'pitch_deg':{'type':'number','minimum':-45,'maximum':60,'title':'俯仰角度（°）','description':'头部上下俯仰绝对目标，-45至60°；必须明确填写'},
                'yaw_deg':{'type':'number','minimum':-90,'maximum':90,'title':'偏航角度（°）','description':'头部左右转动绝对目标，-90至90°；必须明确填写'},
                'time':{'type':'number','minimum':0.1,'maximum':60,'default':5,'title':'运动时间（秒）'},
                **metadata},
            'required':['action','pitch_deg','yaw_deg'],
            'x-action-params':{'move':{'params':['pitch_deg','yaw_deg','time'],'description':'转到指定俯仰/偏航角度，反馈确认到位后完成'}},
            'x-resource':['head','arm_l','arm_r'],
            'x-completion':{'actions':['move'],'timeout':completion.ACP_TIMEOUT}}}


class HeadCompletion(ArmCompletion):
    joint_count = 2
    action_prefix = 'tron2_head_'
    require_ack = True

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
        while time.monotonic() < deadline:
            if not self.reader.connected or self.reader.ws is not job['ws'] or self.reader.serial != job['serial']:
                raise ValueError('Head connection changed during movement; result unknown')
            sample, q, dq = self.feedback()
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
                            'reported_pitch_deg':math.degrees(q[0]),'reported_yaw_deg':math.degrees(q[1]),
                            'max_error_deg':error,'tolerance_deg':completion.TOLERANCE_DEG,
                            'verified_samples':len(window),'verification':'post_duration_advancing_head_feedback'}
            else:
                # Repeated timestamps never count toward arrival.
                if current < stamp:
                    raise ValueError('Head feedback timestamp regressed')
            await asyncio.sleep(.05)
        raise ValueError('Head arrival not confirmed; motion owner retained for operator check')
