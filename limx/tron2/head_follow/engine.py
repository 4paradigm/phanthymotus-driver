"""VOP association and image-based head velocity policy; no hardware transport."""
import math
from .client import require, finite

VERSION = '0.4.0'
CAMERA = '/limx_tron2/camera'
VOP_TOPIC = CAMERA + '/objects'
MAX_FRAME_AGE = 1.5
LOSS_SECONDS = 1.5


def tool():
    return {'name':'head_visual_follow','type':'actuator','multiInstance':False,
        'description':'连接VOP人物检测后进行平滑头部跟随；不跟随模式可暂停或复位头部。',
        'inputSchema':{'type':'object','additionalProperties':False,'properties':{
            'action':{'type':'string','enum':['follow','not_follow','start','stop','info'],
                'default':'follow','title':'模式','oneOf':[
                    {'const':'follow','title':'人物跟随'}, {'const':'not_follow','title':'不跟随模式'}]},
            'non_follow_action':{'type':'string','enum':['pause','reset'],'default':'pause',
                'title':'不跟随操作','oneOf':[{'const':'pause','title':'暂停跟随'},{'const':'reset','title':'头部复位'}]},
            'input_topic':{'type':'string','description':'由画布VOP输出连线注入；仅接受TRON2头部相机的对象话题'}},
            'required':[],'x-resource':['head','arm_l','arm_r'],
            'x-action-params':{'follow':{'params':[]},'not_follow':{'params':['non_follow_action']}},
            'x-completion':{'background':True,'timeout':0,'silent_completion':True}},
        'topic_in':[{'topic':VOP_TOPIC,'format':'data/json','desc':'VOP人物检测输入'}], 'topic_out':[]}


def center(point):
    return tuple(point)


def associate(anchor, points):
    """Spatial continuity of measured VOP centers, never invented boxes."""
    ranked = sorted((math.dist(anchor, point), point) for point in points)
    if not ranked or ranked[0][0] > .23:
        return None
    if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < .06:
        return None
    return ranked[0][1]


def parse_frame(packet, now):
    require(isinstance(packet, dict) and packet.get('active') is True and
            packet.get('input_topic') == VOP_TOPIC and finite(packet.get('received_at')) and
            0 <= now - packet['received_at'] <= MAX_FRAME_AGE, '等待头部相机的新鲜 VOP 人物检测')
    data = packet.get('data')
    require(isinstance(data, dict) and finite(data.get('timestamp')) and
            0 <= now - data['timestamp'] <= MAX_FRAME_AGE and data.get('available') is not False and
            not data.get('error'), 'VOP 检测不可用或已过期')
    values = data.get('objects')
    require(isinstance(values, list) and len(values) <= 1000 and
            ('count' not in data or type(data['count']) is int and data['count'] == len(values)),
            'VOP 对象列表无效')
    latency = data.get('latency_ms', 0)
    require(finite(latency) and 0 <= latency <= MAX_FRAME_AGE * 1000, 'VOP 处理时间无效')
    observed = data['timestamp'] - latency / 1000
    require(0 <= now - observed <= MAX_FRAME_AGE, 'VOP 处理结果已过期')
    points = []
    for item in values:
        require(isinstance(item, dict), 'VOP 检测项无效')
        if item.get('name') not in ('person', 'human', '人'):
            continue
        xy, score = item.get('position'), item.get('confidence')
        require(isinstance(xy, list) and len(xy) == 2 and all(finite(v) and -1 <= v <= 1 for v in xy)
                and finite(score) and 0 <= score <= 1, 'VOP 人物位置无效')
        if score >= .55:
            points.append([(xy[0]+1)/2, (xy[1]+1)/2])
    # timestamp is VOP publication time, not a hardware-frame identifier.
    return {'seq': data['timestamp'], 'received': observed, 'points': points,
            'time_basis': 'vop_processing_start' if 'latency_ms' in data else 'vop_publication'}


class Policy:
    """Only distinct fresh VOP results may acquire/lose a measured person."""
    def __init__(self):
        self.seq = self.received = None
        self.target = self.candidate = None
        self.candidate_since = self.missing_since = None
        self.misses = 0
        self.homing = False
        self.state = 'waiting_person'
        self.input_error = None
        self.time_basis = None
        self.events = []
        self.point = None

    def event(self, name):
        self.events.append({'event':name})
        self.events = self.events[-30:]

    def _missing(self, now):
        self.misses += 1
        if self.missing_since is None:
            self.missing_since = now
        if self.misses >= 3 and now-self.missing_since >= LOSS_SECONDS:
            self.homing = True
            self.state = 'returning_home'
            self.event('person_lost_confirmed')

    def update(self, packet, wall, now, angles):
        try:
            frame = parse_frame(packet, wall)
        except ValueError as exc:
            self.input_error = str(exc)[:200]
            self.missing_since = None
            self.misses = 0
            self.state = 'input_wait'
            return None
        self.input_error = None
        if self.seq is not None:
            require(frame['seq'] >= self.seq and frame['received'] >= self.received,
                    'VOP时间回退；跟随停止，不重试')
        fresh = self.seq is None or (frame['seq'] > self.seq and frame['received'] > self.received)
        if fresh:
            self.seq, self.received, self.time_basis = frame['seq'],frame['received'],frame['time_basis']
            boxes = frame['points']
            if not self.homing:
                if self.target is None:
                    central = sorted(boxes,key=lambda p:math.dist(p,(.5,.5)))
                    box = central[0] if central else None
                    if len(central)>1 and abs(math.dist(central[0],(.5,.5))-math.dist(central[1],(.5,.5)))<.08:
                        self.candidate = self.point = None
                        self.missing_since = None;self.misses = 0;self.state='waiting_person'
                        return [0.,0.]
                    if box is None:
                        self.candidate = self.point = None
                        self.state = 'waiting_person'
                        self._missing(now)
                    else:
                        self.missing_since = None;self.misses = 0
                        if self.candidate is None or associate(self.candidate,[box]) is None:
                            self.candidate, self.candidate_since = box,now
                        elif now-self.candidate_since >= .25:
                            self.target = box;self.point = list(box)
                            self.state = 'tracking';self.event('person_acquired')
                else:
                    box = associate(self.target,boxes)
                    if box is None:
                        self.point = None;self.state = 'occluded';self._missing(now)
                    else:
                        self.target = box
                        # A small observation filter is separate from the actuator's
                        # velocity/acceleration/jerk limiter and never invents boxes.
                        self.point = list(box) if self.point is None else [a+.45*(b-a) for a,b in zip(self.point,box)]
                        self.missing_since = None;self.misses = 0;self.state = 'tracking'
        if self.homing:
            self.state = 'returning_home'
            if max(map(abs,angles)) <= .35:
                self.homing = False;self.target = self.candidate = self.point = None
                self.candidate_since = self.missing_since = None;self.misses = 0
                self.state = 'waiting_person';self.event('home_center_observed')
                return [0.,0.]
            return [max(-8.,min(8.,-a*2.)) for a in angles]
        if self.point is None or self.state not in ('tracking',):
            return [0.,0.]
        x,y = self.point
        # Camera image y-down / x-right; card pitch-up / robot-left positive.
        return [0. if abs(y-.5)<=.06 else max(-8.,min(8.,-(y-.5)*20.)),
                0. if abs(x-.5)<=.06 else max(-12.,min(12.,-(x-.5)*28.))]
