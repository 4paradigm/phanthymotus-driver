"""Wired VOP input, owned visual servo and explicit pause/reset controls."""
import asyncio
import copy
import json
import math
import threading
import time
import uuid
from .client import require
from .engine import Policy, VOP_TOPIC, parse_frame
from .servo import Stream, CONTROL_SECONDS, SEND_SECONDS, MOVE_SECONDS, Limiter
from vendor import motion_readiness, MoveError
from head import card_degrees

PREPARE_SECONDS = 1.5


class Runtime:
    def __init__(self, adapter, publisher, clock=time.monotonic, wall=time.time):
        self.adapter,self.publisher=adapter,publisher
        self.clock,self.wall=clock,wall
        self.closed=False
        self.bound=False
        self.input_topic=None
        self.generation=0
        self.control_generation=0
        self.subscription=None
        self.data_lock=threading.RLock()
        self.packet=None
        self.lock=asyncio.Lock()
        self.follow=None
        self.control=None
        self.policy=Policy()
        self.commands=0
        self.mode='not_follow'
        self.non_follow_action='pause'
        self.reset_guard=None
        self.adapter.head.follow_guard=self.before_reset_send

    def launch(self):
        # No task, ownership or motor command until an explicit public action.
        pass

    @staticmethod
    def topics(args):
        values=args.get('input_topics') or []
        require(isinstance(values,list) and all(isinstance(v,str) for v in values),'VOP输入话题无效')
        if args.get('input_topic'):values=[args['input_topic'],*values]
        require(all(v==VOP_TOPIC for v in values),'请连接TRON2头部相机对应的VOP输出；不接受其它输入')
        return VOP_TOPIC if values else None

    def bind(self, topic):
        require(topic==VOP_TOPIC,'请将VOP输出连接到头部跟随左侧输入端')
        if self.bound:return
        self.generation+=1;generation=self.generation
        sub=self.publisher.subscribe_json(topic,lambda raw:self.receive(raw,generation))
        with self.data_lock:
            self.subscription,self.input_topic,self.bound=sub,topic,True
            self.packet=None

    def receive(self, raw, generation):
        try:
            data=json.loads(raw) if isinstance(raw,str) and len(raw)<=256*1024 else None
        except (ValueError,TypeError):data=None
        with self.data_lock:
            if self.closed or not self.bound or generation!=self.generation:return
            self.packet={'active':True,'input_topic':self.input_topic,'received_at':self.wall(),'data':data}

    def detections(self):
        with self.data_lock:
            return copy.deepcopy(self.packet) if self.bound and self.packet is not None else {'active':False,'input_topic':self.input_topic}

    def unbind(self):
        with self.data_lock:
            self.generation+=1;self.bound=False;self.input_topic=None;self.packet=None
            sub,self.subscription=self.subscription,None
        if sub is not None:self.publisher.unsubscribe(sub)

    def guard(self, job, require_frame):
        require(not self.closed and self.follow is job and not job['_stop'].is_set()
                and job['_generation']==self.control_generation and self.mode=='follow',
                '跟随已暂停或停止；不再发送目标')
        if require_frame:parse_frame(self.detections(),self.wall())

    def job_view(self, job):
        if job is None:return None
        return copy.deepcopy({k:v for k,v in tuple(job.items()) if not k.startswith('_')})

    def info(self):
        f=self.follow;c=self.control
        live_control=c is not None and c['state'] in ('accepted','waiting_previous','resetting','reporting')
        active=f is not None and f['state']=='running' and not f['_stop'].is_set()
        stream=f.get('_stream') if f else None
        pending=stream is not None and stream.job is not None and self.adapter.head.active is stream.job
        return {'service':'tron2-head-visual-follow','version':'0.4.0',
            'topic_in':[{'topic':VOP_TOPIC,'format':'data/json'}],'topic_out':[],
            'detection_topic':VOP_TOPIC,'input_connected':self.bound,'vop_subscribed':self.bound,
            'mode':self.mode,'non_follow_action':self.non_follow_action,'active':active,
            'state':c['state'] if live_control else self.policy.state if active else f['state'] if f and f['state'] in ('blocked_reporting','fault') else 'paused',
            'session':self.job_view(f),'control_action':self.job_view(c),
            'head_motion':({'action_id':stream.job['action_id'],'state':stream.job['state']} if pending else None),
            'last_head_motion':copy.deepcopy(self.adapter.head.last),
            'send_result_unknown':bool(stream and stream.unknown),'head_commands_sent':self.commands,
            'error':f.get('error') if f else None,'input_error':self.policy.input_error,
            'target_center_normalized':self.policy.point,'events':list(self.policy.events),
            'control_hz':1/CONTROL_SECONDS,'maximum_head_command_hz':1/SEND_SECONDS,
            'transport':'two_axis_ack_serialized_moveh','move_horizon_seconds':MOVE_SECONDS,
            'continuous_motor_servo_claimed':False,'measurement_time_verified':False,
            'arm_and_gripper_commands_sent':0,
            'stop_note':'暂停阻止新目标；已受理的短时转头仍会完成，不构成硬件急停。'}

    def accepted(self, job):
        result=dict(self.info(),action_id=job['action_id'],completion_protocol='acp',completion_pending=True)
        if job['kind']!='follow':result['expected_duration_s']=60.
        return result

    def new_job(self, kind):
        return {'action_id':'tron2_follow_'+kind+'_'+uuid.uuid4().hex,'kind':kind,
            'state':'accepted','status':None,'callback_delivered':False,'result':None,'error':None,
            'motion_sent':False,'started_at':self.wall(), '_stop':asyncio.Event(),
            '_generation':self.control_generation,'_task':None}

    async def report(self, job, status, result):
        # The one result is fixed before HTTP; callback retries use this same id/payload.
        job.update(state='reporting',status=status,result=copy.deepcopy(result))
        try:delivered=await self.adapter._report_completion('head_visual_follow',job['action_id'],status,result)
        except Exception:delivered=False
        job.update(callback_delivered=delivered,state='settled' if delivered else 'blocked_reporting')
        return delivered

    async def run_follow(self, job):
        stream=job['_stream'];error=None;settled=False
        try:
            await asyncio.sleep(.1)  # Allow the initial MCP response to register ACP.
            previous=self.clock()
            while not job['_stop'].is_set() and not self.closed:
                p=stream.read();now=self.clock();dt=now-previous
                require(0<dt<=.5,'控制循环延迟过大；不追发积压目标')
                previous=now
                wanted=self.policy.update(self.detections(),self.wall(),now,[p['pitch'],p['yaw']])
                if wanted is None:
                    # Invalid/stale input cannot produce either tracking or home.
                    # Drop future targets. The last accepted short MoveH may finish.
                    stream.limiter=Limiter([p['pitch'],p['yaw']])
                    await asyncio.sleep(CONTROL_SECONDS);continue
                target=stream.limiter.step(wanted,min(.1,dt))
                if stream.last_send is None or now-stream.last_send>=SEND_SECONDS:
                    baseline=stream.last_target or [p['pitch'],p['yaw']]
                    if max(abs(a-b) for a,b in zip(target,baseline))>=.025:
                        try:await stream.send(target)
                        except ValueError:
                            if job['_stop'].is_set() and not stream.unknown:break
                            raise
                        self.commands+=1;job['motion_sent']=True
                await asyncio.sleep(CONTROL_SECONDS)
        except asyncio.CancelledError:
            error='驱动关闭时跟随任务被取消；不能确认物理停止'
        except Exception as exc:
            error=str(exc)[:240]
        finally:
            job['_stop'].set()
            if not stream.unknown:
                try:await stream.finish();settled=True
                except Exception as exc:error=error or str(exc)[:240]
            else:error=error or '头部发送结果不明；不重试、不自动复位'
            job['error']=error
            result={'action':'follow','completion_protocol':'acp','reason':'control_fault' if error else 'operator_pause',
                'motion_sent':job['motion_sent'] or stream.unknown,'motion_may_continue':not settled or stream.unknown,
                'physical_stop_verified':settled,'send_result_unknown':stream.unknown,
                'head_commands_sent':stream.sent,'error':error,'automatic_home_sent_by_completion':False}
            if settled:
                stream.job.update(state='reporting',arrival_verified=True)
            delivered=await self.report(job,'error' if error else 'cancelled',result)
            stream.release(delivered,None if settled else error or '未确认停止')
            if error and delivered:job['state']='fault'
            if self.mode=='follow':self.mode='not_follow';self.non_follow_action='pause'

    def request_stop(self):
        self.control_generation+=1
        if self.follow is not None:self.follow['_stop'].set()
        self.mode='not_follow';self.non_follow_action='pause'

    def before_reset_send(self, token):
        try:self._before_reset_send(token)
        except ValueError as exc:raise MoveError(str(exc),False,'head_reset_guard_rejected') from exc

    def _before_reset_send(self, token):
        guard=self.reset_guard
        require(guard is not None and guard['token']==token and not self.closed
                and not guard['job']['_stop'].is_set() and self.control is guard['job']
                and self.mode=='not_follow' and self.clock()<=guard['expires'],
                '头部复位已取消或保护过期；未发送')
        require((self.adapter.head.active or {}).get('details',{}).get('head_follow_guard')==token,
                '复位动作所有权改变；未发送')
        _,arm,_=self.adapter.motion.feedback()
        _,head,_=self.adapter.head.feedback()
        require(max(abs(a-b) for a,b in zip(arm+head,guard['q']))<=math.radians(.2),
                '复位模式核对期间机器人姿态改变；未发送')

    async def previous_finished(self, previous):
        if previous is None:return
        await asyncio.shield(previous['_task'])
        require(previous['callback_delivered'] is True,'上次动作ACP回调未确认；不发送新目标')
        if previous['kind']=='follow':
            require((previous.get('result') or {}).get('physical_stop_verified') is True
                    and not (previous.get('result') or {}).get('send_result_unknown'),'上次头部运动结果不明；不自动复位')

    async def run_control(self, job, previous):
        status='completed';error=None;native=None
        try:
            await asyncio.sleep(.1)
            job['state']='waiting_previous'
            await self.previous_finished(previous)
            require(not job['_stop'].is_set() and not self.closed,'项目已停止，取消尚未发送的头部操作')
            if job['kind']=='reset':
                job['state']='resetting'
                await self.adapter.head.ensure_available()
                sample,q,dq=self.adapter.head.feedback()
                duration=max(3.,max(map(abs,card_degrees(q)))/6.)
                status_sample=self.adapter.reader.sample('robot_status')
                require(status_sample.get('available') is True and motion_readiness(status_sample.get('data') or {})['ready'],
                        '机器人模式或保护未通过；未发送复位')
                _,arm,_=self.adapter.motion.feedback()
                token=uuid.uuid4().hex
                self.reset_guard={'token':token,'job':job,'q':arm+q,'expires':self.clock()+3.}
                try:
                    native=await self.adapter.execute_tool('move_head',{'action':'move','pitch_deg':0.,'yaw_deg':0.,'time':duration},follow_guard=token)
                finally:self.reset_guard=None
                job['motion_sent']=native.get('motion_sent')
                await asyncio.shield(self.adapter.head.task)
                native=self.adapter.head.summary()
                require(native.get('status')=='completed' and native.get('arrival_verified') is True
                        and native.get('callback_delivered') is True and native.get('motion_may_continue') is False,
                        '头部复位未确认；不重试')
            else:
                await self.adapter.head.ensure_available()
                self.adapter.head.validate_admission_feedback()
            if job['_stop'].is_set():status='cancelled'
        except Exception as exc:
            status='cancelled' if job['_stop'].is_set() and not job['motion_sent'] else 'error'
            error=str(exc)[:240]
        result={'action':'not_follow','non_follow_action':job['kind'],'completion_protocol':'acp',
            'motion_sent':job['motion_sent'],'motion_may_continue':bool(self.adapter.head.active),
            'reset_verified':job['kind']=='reset' and error is None and native is not None,
            'head_motion':native,'error':error}
        job['error']=error
        await self.report(job,status,result)

    async def call(self, args):
        require(isinstance(args,dict),'跟随参数必须是对象')
        require(not set(args)-{'action','non_follow_action','input_topic','input_topics','instance_id','_trace_id'},'不支持的跟随参数')
        action=args.get('action','follow')
        require(action in ('follow','not_follow','start','stop','info'),'不支持的跟随操作')
        topic=self.topics(args) if action in ('follow','start') else None
        if action=='info':return self.info()
        if action=='stop':
            self.request_stop()
            if self.control is not None:self.control['_stop'].set()
            self.unbind();return self.info()
        require(not self.closed,'头部跟随已关闭')
        if action=='start':
            if topic:self.bind(topic)
            return self.info()
        if action=='not_follow':
            kind=args.get('non_follow_action','pause')
            require(kind in ('pause','reset'),'不跟随模式只支持暂停或头部复位')
            self.request_stop()
            async with self.lock:
                previous=self.follow
                if self.control is not None:
                    task=self.control['_task']
                    if not task.done():
                        require(self.control['kind']==kind,'上次头部操作尚未完成，请等待确认')
                        return self.accepted(self.control)
                    require(self.control['callback_delivered'],'上次头部操作ACP回调未确认')
                self.non_follow_action=kind
                job=self.new_job(kind);self.control=job
                job['_task']=asyncio.create_task(self.run_control(job,previous))
                return self.accepted(job)
        async with self.lock:
            if self.control is not None:
                require(self.control['_task'].done() and self.control['callback_delivered'],'暂停或复位尚未完成/回调未确认')
            if self.follow is not None:
                task=self.follow['_task']
                if task is not None and not task.done():
                    require(not self.follow['_stop'].is_set(),'跟随仍在停止确认，请稍候')
                    return self.accepted(self.follow)
                require(self.follow['callback_delivered'],'上次跟随ACP回调未确认')
            topic=topic or self.input_topic
            require(topic==VOP_TOPIC,'请连接 camera → VOP → head_visual_follow 后执行')
            self.bind(topic)
            generation=self.control_generation;deadline=self.clock()+PREPARE_SECONDS
            while True:
                require(generation==self.control_generation and not self.closed and self.bound,'跟随准备已取消')
                try:parse_frame(self.detections(),self.wall());break
                except ValueError:
                    require(self.clock()<deadline,'等待VOP的新鲜人物检测；请检查camera→VOP连线')
                    await asyncio.sleep(.05)
            self.mode='follow';self.policy=Policy()
            job=self.new_job('follow');self.follow=job
            stream=Stream(self.adapter,lambda fresh:self.guard(job,fresh),self.clock);job['_stream']=stream
            try:await stream.start(job['action_id'])
            except BaseException:
                job['_stop'].set();self.follow=None;self.mode='not_follow'
                raise
            job['state']='running'
            job['_task']=asyncio.create_task(self.run_follow(job))
            self.adapter.head.task=job['_task']
            return self.accepted(job)

    async def close(self):
        if self.closed:return
        self.request_stop()
        if self.control is not None:self.control['_stop'].set()
        self.unbind()
        for job in (self.follow,self.control):
            if job is not None and job.get('_task') is not None:
                await asyncio.shield(job['_task'])
        self.closed=True
