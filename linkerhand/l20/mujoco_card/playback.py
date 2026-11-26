"""Route explicitly connected trajectory topics to independent virtual hands."""
from collections import deque
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import re
import threading
import time
import uuid

from mujoco_card.trajectories import FORMAT, VERSION, SIDES, FPS, channel, validate_packet


@dataclass
class Slot:
    side: str
    pose: list
    topics: tuple = ()
    active: dict | None = None
    pending: str | None = None
    owner_topic: str = ''
    last: dict | None = None
    started: float = 0
    enabled: bool = False
    rejected: str = ''
    render_failures: int = 0
    seen: deque = field(default_factory=lambda:deque(maxlen=64))


class Playback:
    def __init__(self, models, initial_poses, bus, directory=None):
        self.models, self.initial_poses, self.bus = models, initial_poses, bus
        self.lock = threading.Condition(threading.RLock())
        self.sources, self.slots = {}, {}
        self.path = Path(directory)/'instances.json' if directory else None
        self.closed = threading.Event()
        if self.path and self.path.exists():
            saved=json.loads(self.path.read_text())
            self.sources=saved['sources']
            for key, item in saved['slots'].items():
                slot=Slot(item['side'],item['pose'],last=item.get('last'))
                self.slots[key]=slot
                if item.get('enabled'):
                    self.start(key,slot.side,item['topics'],save=False)
        self.thread=threading.Thread(target=self._stream,daemon=True)
        self.thread.start()

    def _save(self):
        if not self.path:return
        self.path.parent.mkdir(exist_ok=True,parents=True)
        doc={'sources':self.sources,'slots':{k:{'side':s.side,'pose':s.pose,'last':s.last,
             'topics':s.topics,'enabled':s.enabled} for k,s in self.slots.items()}}
        temp=self.path.with_suffix('.tmp')
        temp.write_text(json.dumps(doc,separators=(',',':')))
        os.replace(temp,self.path)

    def side_for(self,name,key,args,configuring=False):
        ident=name+'/'+key
        with self.lock:
            side=self.sources.get(ident,'right')
            request=args.get('bound_side') or (args.get('config') or {}).get('bound_side') or side
            if request not in SIDES:raise ValueError('请选择左手或右手')
            if request!=side and not configuring:raise ValueError('动作左右手与实例配置不符')
            if configuring and self.sources.get(ident)!=request:
                slot=self.slots.get(key) if name=='l20_mujoco' else None
                if slot and slot.side!=request:
                    if slot.active or slot.pending:raise ValueError('当前模拟手正在播放')
                    self.stop(key)
                    del self.slots[key]
                if name!='l20_mujoco' and request!=side:
                    old=channel(side,name,key)
                    if any(s.active or s.pending for s in self.slots.values() if old in s.topics):
                        raise ValueError('请待当前动作结束后换手')
                self.sources[ident]=request
                self._save()
            return request

    def slot(self,key,side):
        with self.lock:
            if key not in self.slots:
                self.slots[key]=Slot(side,list(self.initial_poses[side]))
            slot=self.slots[key]
            if slot.side!=side:raise ValueError('模拟实例左右手发生冲突')
            return slot

    def start(self,key,side,topics,save=True):
        if not isinstance(topics,(list,tuple)) or len(topics)>16:
            raise ValueError('无效的输入连线')
        topics=tuple(dict.fromkeys(topics))
        if any(not isinstance(t,str) or not re.fullmatch(r'/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*',t) for t in topics):
            raise ValueError('输入应为画布解析出的 ROS 话题')
        for t in topics:
            if t.startswith('/l20_sim/') and t.split('/')[2]!=side:raise ValueError('上游与模拟手左右不一致')
        with self.lock:
            s=self.slot(key,side)
            if s.enabled and s.topics==topics:return
            if s.active or s.pending:raise ValueError('当前模拟手正在播放')
            if any(o.enabled and o.side==side for k,o in self.slots.items() if k!=key):
                raise ValueError('同侧已经绑定另一张模拟卡')
            self.bus.bind(key,topics,lambda topic,p:self.receive(key,topic,p),channel(side,'preview',key))
            s.enabled=True;s.topics=topics
            if save:self._save()

    def cancel_source(self, topic):
        with self.lock:
            for s in self.slots.values():
                if s.owner_topic == topic:
                    self._advance(s)
                    if s.pending:s.seen.append(s.pending)
                    s.active=s.pending=None
                    s.owner_topic=''
            self.lock.notify_all()

    def stop(self,key):
        with self.lock:
            s=self.slots.get(key)
            if s:
                self._advance(s)
                if s.pending:s.seen.append(s.pending)
                s.active=s.pending=None;s.enabled=False;s.topics=()
                self.bus.unbind(key)
                self._save();self.lock.notify_all()

    def _advance(self,s):
        if not s.active:return
        packet=s.active
        idx=int((time.monotonic()-s.started)*1000/packet['frame_ms'])
        s.pose=packet['frames'][min(idx,len(packet['frames'])-1)]
        if idx>=len(packet['frames'])-1:
            s.last=packet;s.active=None
            self._save();self.lock.notify_all()

    def receive(self,key,topic,packet):
        with self.lock:
            s=self.slots.get(key)
            if not s or not s.enabled or topic not in s.topics:return
            try:
                validate_packet(packet)
                if packet['side']!=s.side:raise ValueError('收到异侧轨迹')
                if packet['id'] in s.seen:return
                self._advance(s)
                if s.active or (s.pending and s.pending!=packet['id']):
                    s.seen.append(packet['id'])
                    raise ValueError('模拟手忙，已拒绝新动作')
                s.seen.append(packet['id']);s.pending=None
                s.active=packet;s.owner_topic=topic;s.started=time.monotonic();s.pose=packet['frames'][0];s.rejected=''
            except ValueError as e:
                s.rejected=str(e)
            self.lock.notify_all()

    def submit(self,topic,side,action,build):
        token=uuid.uuid4().hex
        with self.lock:
            targets=[s for s in self.slots.values() if s.enabled and s.side==side and topic in s.topics]
            if len(targets)!=1:raise ValueError('请将该动作实例连到同侧模拟卡，并启动该仿真连线')
            s=targets[0];self._advance(s)
            if s.active or s.pending:raise ValueError('模拟手忙，已拒绝新动作')
            s.pending=token;s.owner_topic=topic;pose=s.pose.copy()
        try:
            packet=validate_packet({'schema':VERSION,'id':token,'side':side,'action':action,
                                    'frame_ms':1000//FPS,'frames':build(pose)})
            self.bus.publish(topic,packet)
            with self.lock:
                self.lock.wait_for(lambda:s.pending!=token,timeout=2.5)
                accepted=any(p and p['id']==token for p in (s.active,s.last))
                if not accepted:raise ValueError('轨迹未被模拟卡接收，请检查连线')
            return {'state':'playing','simulation_only':True,'id':token,'bound_side':side,
                    'frame_count':len(packet['frames']),'topic':topic}
        finally:
            with self.lock:
                if s.pending==token:s.seen.append(token);s.pending=None

    def read(self,key,side):
        with self.lock:
            s=self.slot(key,side);self._advance(s)
            latest=s.last;active=s.active
            return {'state': 'running' if s.enabled else 'idle',
                    'side':side,'enabled':s.enabled,'busy':bool(active or s.pending),'active_id':active['id'] if active else '',
                    'action':(active or latest or {}).get('action',''),'last_id':latest['id'] if latest else '',
                    'count':len(latest['frames']) if latest else 0,'frame_ms':latest['frame_ms'] if latest else 50,
                    'pose':s.pose.copy(),'rejected':s.rejected,'topic_in':[{'topic':t,'format':FORMAT} for t in s.topics],
                    'topic_out':[{'topic':channel(side,'preview',key),'format':'image/jpeg'}]}

    def replay(self,key,side,run_id,offset,count):
        with self.lock:
            s=self.slot(key,side);self._advance(s);p=s.last
            if not p or p['id']!=run_id:raise ValueError('回放已更新，请重新读取最近动作')
            if type(offset)!=int or type(count)!=int or not 0<=offset<len(p['frames']) or not 1<=count<=32:
                raise ValueError('无效回放范围')
            return p['frames'][offset:offset+count]

    def _stream(self):
        cache={}
        while not self.closed.wait(.08):
            with self.lock:
                jobs=[]
                for key,s in self.slots.items():
                    if s.enabled:
                        self._advance(s)
                        jobs.append((key,s,tuple(s.pose)))
            for key,slot,pose in jobs:
                side = slot.side
                try:
                    prev=cache.get(key)
                    if prev and prev[:2]==(side,pose) and time.monotonic()-prev[2]<1:continue
                    jpeg=prev[3] if prev and prev[:2]==(side,pose) else self.models[side].jpeg(pose)
                    self.bus.image(key,jpeg)
                    cache[key]=(side,pose,time.monotonic(),jpeg)
                    slot.render_failures = 0
                except Exception as exc:
                    slot.render_failures += 1
                    if slot.render_failures == 1 or slot.render_failures % 100 == 0:
                        detail = ascii(f"{type(exc).__name__}: {str(exc)[:120]}")[1:-1][:180]
                        print(f"[sim preview] {side} failures={slot.render_failures}: {detail}", flush=True)

    def shutdown(self):
        self.closed.set();self.thread.join(5)
        with self.lock:self._save()
