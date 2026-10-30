"""TRON2 adapter for generic ActuCore head tracking, not a tracking policy.

The existing HeadCompletion/MoveH path owns every accepted target. Streams
cannot clear an old fault, recover automatically, move arms or change modes.
"""
import asyncio
import copy
import json
import math
import re
import time
import uuid
from vendor import MoveError, motion_readiness
from skeleton import skeleton_stream

STATE_TOPIC = '/limx_tron2/head_state'


def descriptor():
    return {'control_interface':'motus.control/1', 'mode':'joint_position', 'dof':2,
        'joint_names':['head_pitch_Joint','head_yaw_Joint'], 'units':{'angle':'rad','time':'s'},
        'limits':{'lower':[-math.radians(30),-math.radians(45)],
                  'upper':[math.radians(55),math.radians(45)],
                  'max_velocity':[math.radians(6),math.radians(6)]},
        'rate':{'expected_hz':2,'max_hz':10,'watchdog_ms':500,'max_obs_age_ms':1500},
        'force_torque':None, 'groups':[{'name':'head','offset':0,'count':2,'unit':'rad','resource':'head'}],
        'head_tracking':{'schema':'motus.head-tracking/1',
            'axes':{'pitch':{'index':0,'sign':-1,'image_axis':1,'gain_rad':.22},
                    'yaw':{'index':1,'sign':1,'image_axis':0,'gain_rad':.26}},
            'neutral':[0.,0.], 'max_duration_s':15., 'camera_topic':'/limx_tron2/camera',
            'feedback_topic':STATE_TOPIC}}


def need(ok, message):
    if not ok:
        raise MoveError(message, False, 'head_stream_rejected')


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


class HeadStream:
    def __init__(self, adapter, publisher):
        from main import common_root
        import sys
        sys.path.insert(0, str(common_root(__file__)))
        from common.control import ControlSink
        self.adapter, self.publisher = adapter, publisher
        self.loop = asyncio.get_running_loop()
        self.session = uuid.uuid4().hex
        self.running = False
        self.epoch = 0
        self.subscription = self.task = None
        self.input_topic = None
        self.source = None
        self.last_seq = 0
        self.last_error = None
        self.request = None
        self.message = None
        self._sink = ControlSink(descriptor(), self._apply)
        self.adapter.head.stream_guard = self.before_send

    def state(self):
        head = self.adapter.head
        motion = head.summary() or {}
        req = copy.deepcopy(self.request)
        complete = motion.get('status') == 'completed' and motion.get('arrival_verified') is True and \
            motion.get('callback_delivered') is True and motion.get('motion_may_continue') is False and \
            motion.get('can_accept_motion') is True
        if req and req.get('action_id') == motion.get('action_id'):
            if complete:
                req['outcome'] = 'completed'
            elif motion.get('status') == 'error' or motion.get('state') == 'blocked_requires_operator_check':
                req['outcome'] = 'failed'
        result = {'schema':'motus.head-state/1', 'session':self.session, 'timestamp':time.time(),
            'available':False, 'ready':False, 'stationary':False, 'values':[], 'velocities':[],
            'sample_stamp':None, 'motion_id':motion.get('action_id'), 'request':req,
            'completion_verified':complete, 'measurement_time_verified':False,
            'stream_enabled':self.running, 'input_topic':self.input_topic}
        try:
            sample, q, dq = head.feedback()
            need(sample.get('available') is True and finite(sample.get('reception_age_seconds')) and
                 0 <= sample['reception_age_seconds'] <= 1, '头部反馈过期')
            packet = skeleton_stream(self.adapter.reader.sample('joint_state'))
            need(packet['status']['available'] and len(packet['joints']) == 16, '关节顺序无效')
            joints = {j['idx']:j for j in packet['joints']}
            arm_stationary = all(finite(joints[i].get('dq')) and abs(joints[i]['dq']) <= .05 for i in range(14))
            status = self.adapter.reader.sample('robot_status')
            peers = [self.adapter.motion, *self.adapter.grippers.values()]
            ready = status.get('available') is True and motion_readiness(status['data'])['ready'] and \
                self.adapter.reader.connected and not head.closed and not head.active and \
                not any(p.active or p.closed or getattr(p,'recovering',False) or getattr(p,'waiting',None) for p in peers)
            result.update(available=True, values=q, velocities=dq,
                sample_stamp=sample['data']['robot_timestamp'], ready=bool(ready and arm_stationary),
                stationary=arm_stationary and all(finite(v) and abs(v) <= .05 for v in dq))
        except (ValueError, KeyError, TypeError):
            pass
        return result

    def publish_state(self):
        self._sink.tick()  # Silence never sends a motor command or an automatic home.
        self.publisher.publish(STATE_TOPIC, self.state())

    async def call(self, args):
        action = args.get('action', 'info')
        if action == 'stop':
            self.running = False
            self.epoch += 1
            if self.subscription is not None:
                self.publisher.unsubscribe(self.subscription)
                self.subscription = None
            return {'state':'idle','motion_sent':False,'lifecycle_only':True,'control_interface':descriptor()}
        need(action == 'start', '不支持的头部数据流操作')
        inputs = args.get('input_topics') or []
        need(isinstance(inputs,list) and all(isinstance(v,str) for v in inputs), '头部输入话题无效')
        if args.get('input_topic'):
            inputs = [args['input_topic'], *inputs]
        inputs = list(dict.fromkeys(inputs))
        if not inputs:
            return {'state':'ready','motion_sent':False,'lifecycle_only':True,'control_interface':descriptor()}
        need(len(inputs) == 1 and len(inputs[0]) <= 200 and re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', inputs[0]), '只能连接一路头部指令流')
        if self.running:
            need(self.input_topic == inputs[0], '头部接线改变，请先停用项目')
            return {'state':'ready','motion_sent':False,'control_interface':descriptor()}
        need(not self.task or self.task.done(), '旧的头部请求尚未解除')
        self.epoch += 1
        epoch = self.epoch
        def receive(raw):
            if isinstance(raw,str) and len(raw) <= 16384:
                try:
                    msg = json.loads(raw)
                except (ValueError, TypeError):
                    return
                self.loop.call_soon_threadsafe(self.offer, msg, epoch)
        self.subscription = self.publisher.subscribe_json(inputs[0], receive)
        self.running = True
        self.input_topic = inputs[0]
        self.source = None
        self.last_seq = 0
        self._sink.reset()
        return {'state':'ready','motion_sent':False,'control_interface':descriptor()}

    def validate(self, msg, epoch, before_send=False):
        need(self.running and epoch == self.epoch, '头部数据流已停用，不发送新目标')
        need(isinstance(msg,dict) and msg.get('head_session') == self.session, '头部会话不匹配')
        now = time.time()*1000
        need(finite(msg.get('stamp_ms')) and finite(msg.get('ttl_ms')) and 0 < msg['ttl_ms'] <= 500 and
             -100 <= now-msg['stamp_ms'] <= msg['ttl_ms'], '头部指令已过期或时间异常')
        need(finite(msg.get('obs_stamp_ms')) and -100 <= now-msg['obs_stamp_ms'] <= 1500,
             '人物观测已过期或时间异常')
        need(isinstance(msg.get('source'),str) and 0 < len(msg['source']) <= 100 and
             (self.source is None or self.source == msg['source']), '另一个控制来源占有头部')
        need(type(msg.get('seq')) is int and msg['seq'] > 0, '头部指令序号无效')
        need(finite(msg.get('duration_s')) and .6 <= msg['duration_s'] <= 15, '头部运动时间无效')
        state = self.state()
        if before_send:
            need(self.adapter.head.active is not None and self.adapter.head.active['details'].get('head_stream') == {'epoch':epoch,'seq':msg['seq'],'source':msg['source']}, '头部任务占有已改变')
        else:
            need(state['ready'] and state['stationary'] and not self.adapter.head.active, '头部或其它执行器忙/不确定')
        need(state['available'] and state['stationary'], '没有静止新反馈')
        ref = msg.get('reference') or {}
        need(isinstance(ref,dict), '头部参考姿态格式无效')
        q = ref.get('values')
        need(isinstance(q,list) and len(q) == 2 and all(finite(v) for v in q) and
             finite(ref.get('sample_stamp')) and state['sample_stamp'] >= ref['sample_stamp'] and
             max(abs(a-b) for a,b in zip(q,state['values'])) <= math.radians(.2), '头部参考姿态已改变')
        last_id = (self.adapter.head.last or {}).get('action_id') if before_send else state['motion_id']
        need(last_id == ref.get('motion_id'), '检测到其它头部控制，不跟进旧目标')
        values = msg.get('values')
        need(isinstance(values,list) and len(values) == 2 and all(finite(v) for v in values), '头部目标无效')
        need(all(abs(a-b) / msg['duration_s'] <= cap + 1e-8 for a,b,cap in
             zip(values,q,descriptor()['limits']['max_velocity'])), '头部运动超过协商速度')

    def offer(self, msg, epoch):
        if not self.running or epoch != self.epoch:
            return
        if isinstance(msg,dict) and msg.get('source') == self.source and type(msg.get('seq')) is int and msg['seq'] <= self.last_seq:
            return
        try:
            self.validate(msg, epoch)
            # No backlog and no correction. Replayed/old seqs are dropped by
            # ControlSink; an accepted action must complete before the next one.
            need(not self.task or self.task.done(), '头部请求尚未确认')
            self.message = copy.deepcopy(msg)
            result = self._sink.submit(msg)
            if not result.applied:
                return
        except (ValueError, TypeError, KeyError) as exc:
            self.last_error = str(exc)[:160]
            if isinstance(msg,dict) and (self.source is None or msg.get('source') == self.source) and not self.adapter.head.active and (not self.task or self.task.done()):
                self.request = {'source':msg.get('source'),'seq':msg.get('seq'),
                                'outcome':'rejected','error':str(exc)[:160]}

    def _apply(self, values, gripper):
        need(gripper is None, '头部数据流不能控制夹爪')
        msg = copy.deepcopy(self.message)
        self.source = msg['source']
        self.last_seq = msg['seq']
        self.request = {'source':msg['source'],'seq':msg['seq'],'outcome':'pending'}
        self.task = asyncio.create_task(self.execute(msg, list(values), self.epoch))

    async def execute(self, msg, values, epoch):
        from head import card_degrees
        try:
            self.validate(msg, epoch)
            pitch, yaw = card_degrees(values)
            validated = self.adapter.call('move_head', {'action':'move',
                'pitch_deg':pitch,'yaw_deg':yaw,'time':msg['duration_s']})
            token = {'epoch':epoch,'seq':msg['seq'],'source':msg['source']}
            self.message = msg
            result = await self.adapter.head.dispatch('move', validated['joint'], validated['time'],
                {'target_pitch_deg':pitch,'target_yaw_deg':yaw,'head_stream':token})
            self.request.update(action_id=result['action_id'], outcome='accepted' if result.get('motion_sent') is True else 'unknown')
        except Exception as exc:
            self.request.update(outcome='failed',error=str(exc)[:160])

    def before_send(self, token):
        msg = self.message
        need(isinstance(msg,dict) and token == {'epoch':self.epoch,'seq':msg.get('seq'),'source':msg.get('source')}, '头部数据流任务已变化')
        self.validate(msg, self.epoch, before_send=True)

    async def close(self):
        await self.call({'action':'stop'})
        # The original HeadCompletion owns/cancels its send during shutdown.
        # Do not resend or cancel that shielded send from a second owner.
