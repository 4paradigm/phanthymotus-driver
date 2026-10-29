"""Bounded, serial person following; no arm, gripper or vendor transport."""
import copy
import math
import threading
import time
from client import Client, Error, require, finite, pose, terminal

VERSION = '0.1.2'
CAMERA = '/limx_tron2/camera'
MAX_FRAME_AGE = 1.5
LOSS_SECONDS = 1.5


def tool():
    return {'name': 'head_visual_follow', 'type': 'actuator', 'multiInstance': False,
            'description': '跟随画面中央的人转头；短暂遮挡时保持，人物离开后缓慢回正。停止后不再发送新转动。',
            'inputSchema': {'type': 'object', 'additionalProperties': False, 'properties': {
                'action': {'type': 'string', 'enum': ['follow', 'pause', 'stop', 'info'],
                           'default': 'follow', 'title': '操作'}}, 'required': ['action'],
                'x-resource': ['head', 'arm_l', 'arm_r'], 'x-action-params': {
                    'follow': {'params': [], 'description': '开始跟随中央人物'},
                    'pause': {'params': [], 'description': '停止跟随，保持当前方向'},
                    'stop': {'params': [], 'description': '停止跟随，保持当前方向'},
                    'info': {'params': [], 'description': '查看跟随状态'}}}}


def center(box):
    return ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)


def area(box):
    return (box[2] - box[0]) * (box[3] - box[1])


def iou(a, b):
    overlap = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    return overlap / (area(a) + area(b) - overlap)


def associate(anchor, boxes):
    """Conservative spatial association, not biometric identity recognition."""
    ax, ay = center(anchor)
    matches = []
    for box in boxes:
        bx, by = center(box)
        distance = math.hypot(ax - bx, ay - by)
        ratio = area(box) / area(anchor)
        if .45 <= ratio <= 2.2 and distance <= .23 and (iou(anchor, box) >= .12 or distance <= .12):
            matches.append((iou(anchor, box) - distance, box))
    matches.sort(key=lambda item: item[0], reverse=True)
    if not matches:
        return None
    if len(matches) > 1 and matches[0][0] - matches[1][0] < .15:
        return None
    return matches[0][1]


def parse_frame(health, now):
    require(isinstance(health, dict) and health.get('active') is True and health.get('available') is True and
            health.get('input_topic') == CAMERA and health.get('last_error') is None, '等待头部相机的新鲜人物检测')
    data = health.get('latest_result')
    require(isinstance(data, dict) and data.get('available') is True and data.get('input_topic') == CAMERA and
            data.get('box_coordinate_system') == 'original_image_pixels', '检测来源不是头部原图')
    received, published = data.get('source_received_at'), data.get('published_at')
    require(finite(received) and finite(published) and 0 <= now - received <= MAX_FRAME_AGE and
            received <= published <= now + .1, '检测画面已过期或时间异常')
    seq, w, h = data.get('frame_sequence'), data.get('image_width'), data.get('image_height')
    require(type(seq) is int and seq > 0 and type(w) is int and type(h) is int and
            16 <= w <= 4096 and 16 <= h <= 4096, '检测帧编号或尺寸无效')
    stamp = data.get('source_header_stamp')
    require(isinstance(stamp, dict) and type(stamp.get('sec')) is int and
            type(stamp.get('nanosec')) is int and 0 <= stamp['nanosec'] < 1000000000,
            '缺少相机帧标识')
    detections = data.get('detections')
    require(isinstance(detections, list) and len(detections) <= 1000, '人物检测格式异常')
    boxes = []
    for item in detections:
        require(isinstance(item, dict), '检测项格式异常')
        if item.get('label') not in ('person', 'human', '人'):
            continue
        b, score = item.get('box_xyxy'), item.get('confidence')
        require(isinstance(b, list) and len(b) == 4 and all(finite(v) for v in b) and
                0 <= b[0] < b[2] <= w and 0 <= b[1] < b[3] <= h and finite(score) and 0 <= score <= 1,
                '人物框异常')
        box = [b[0]/w, b[1]/h, b[2]/w, b[3]/h]
        if score >= .55 and area(box) >= .015:
            boxes.append(box)
    return {'seq': seq, 'received': received, 'header': (stamp['sec'], stamp['nanosec']), 'boxes': boxes}


class Engine:
    def __init__(self, client=None, clock=time.monotonic, wall=time.time):
        self.client = client or Client()
        self.clock, self.wall = clock, wall
        self.lock = threading.RLock()
        # A single lock serializes control start/stop and the one physical send.
        self.control = threading.RLock()
        self.active = False
        self.state = 'idle'
        self.error = None
        self.input_error = None
        self.pending = None
        self.unknown_send = False
        self.target = None
        self.candidate = None
        self.candidate_since = None
        self.last_seen = None
        self.missing_since = None
        self.misses = 0
        self.returning = False
        self.seq = None
        self.received = None
        self.header = None
        self.after_frame = float('-inf')
        self.arm_reference = None
        self.owner_id = None
        self.certified_pose = None
        self.stable = []
        self.commands = 0
        self.events = []
        self.wake = threading.Event()
        self.stop_requested = threading.Event()
        self.closed = False
        self.thread = None

    def event(self, name, **fields):
        self.events.append({'event': name, 'at': self.wall(), **fields})
        self.events = self.events[-30:]

    def info(self):
        with self.control:
            return copy.deepcopy({'service': 'tron2-head-visual-follow', 'version': VERSION,
                'release': __import__('os').environ.get('FOLLOW_RELEASE', ''),
                'state': self.state, 'active': self.active, 'target_box_normalized': self.target,
                'input_error': self.input_error, 'error': self.error, 'head_motion': self.pending,
                'send_result_unknown': self.unknown_send, 'head_commands_sent': self.commands,
                'last_frame_sequence': self.seq, 'return_home_deg': {'pitch': 0, 'yaw': 0},
                'head_limits_deg': {'pitch': [-55, 30], 'yaw': [-45, 45]},
                'lost_timeout_seconds': LOSS_SECONDS, 'max_step_deg': None, 'driver_stationary_proof_reused': True,
                'events': self.events, 'arm_and_gripper_commands_sent': 0,
                'stop_note': '停止只阻止新指令；已经受理的本次转头仍会完成；pause不构成中途制动。'})

    def call(self, args):
        require(isinstance(args, dict) and not set(args) - {'action', 'instance_id', '_trace_id'}, '不支持的跟随参数')
        action = {'pause': 'stop'}.get(args.get('action'), args.get('action', 'info'))
        if action == 'stop':
            self.stop_requested.set()
        with self.control:
            if action in ('info', 'start'):
                # Motus project lifecycle must not arm autonomous movement.
                return self.info()
            if action == 'stop':
                self.active = False
                self.state = 'stopping' if self.pending else 'idle'
                self.returning = False
                self.event('operator_stop')
                return self.info()
            require(action == 'follow', '不支持的操作')
            require(not self.closed and not self.pending and not self.unknown_send, '上次转头尚未确认；请先核对状态')
            if self.active:
                return self.info()
            p = pose(self.client.robot())
            parse_frame(self.client.detections(), self.wall())
            self.owner_id = p['head_id']
            self.arm_reference = p['q'][:14]
            self.target = self.candidate = self.last_seen = self.missing_since = None
            self.candidate_since = None
            self.misses = 0
            self.returning = False
            self.seq = self.received = self.header = None
            self.after_frame = self.wall()
            self.certified_pose = None
            self.stable = []
            self.error = self.input_error = None
            self.stop_requested.clear()
            self.active, self.state = True, 'waiting_person'
            self.event('follow_enabled')
            self.wake.set()
            return self.info()

    def fail(self, exc):
        self.active = False
        self.state = 'fault'
        self.error = str(exc)[:300]
        self.returning = False
        self.event('control_fault', error=self.error)

    def check_robot(self, health):
        p = pose(health, stationary=self.pending is None)
        require(self.arm_reference is None or max(abs(a-b) for a,b in zip(p['q'][:14], self.arm_reference)) <= math.radians(.5),
                '机械臂位置改变；跟随停止')
        if self.pending:
            motion = health.get('head_motion') or {}
            require(motion.get('action_id') == self.pending['action_id'], '头部任务标识改变；不再发送新指令')
            if motion.get('status') == 'error' or motion.get('state') == 'blocked_requires_operator_check':
                raise Error('上次头部动作失败；不重试、不自动回正')
            if terminal(motion) and motion.get('can_accept_motion') is True:
                target = self.pending['target']
                require(max(abs(p['pitch']-target[0]), abs(p['yaw']-target[1])) <= 1, '反馈未到目标；不自动纠正')
                self.event('head_arrival_verified', action_id=motion['action_id'])
                self.certified_pose = p if motion.get('verification') == 'post_duration_advancing_head_feedback' and \
                    type(motion.get('verified_samples')) is int and motion['verified_samples'] >= 3 else None
                self.pending = None
                self.after_frame = self.wall()
                self.stable = []
                if not self.active:
                    self.state = 'idle' if not self.error else 'fault'
            else:
                require(self.clock() <= self.pending['deadline'], '头部完成确认超时；不重发')
                return None
        else:
            require(p['head_id'] == self.owner_id, '检测到其他头部控制指令；跟随停止')
        # Reuse only this exact action's stationary proof, while fresh feedback
        # still agrees. Initial admission / absent proof retains the full window.
        if self.certified_pose is not None:
            current = pose(health)
            if current['head_id'] == self.owner_id and max(abs(a-b) for a,b in
                    zip(current['q'][14:], self.certified_pose['q'][14:])) <= math.radians(.2):
                return current
            self.certified_pose = None
        now = self.clock()
        if not self.stable or p['stamp'] > self.stable[-1][1]['stamp']:
            if self.stable and max(abs(a-b) for a,b in zip(p['q'], self.stable[0][1]['q'])) > math.radians(.2):
                self.stable = []
            self.stable.append((now, p))
        if len(self.stable) < 3 or now-self.stable[0][0] < .5:
            return None
        self.stable = self.stable[-20:]
        return p

    def dispatch(self, p, pitch, yaw, kind):
        if self.stop_requested.is_set() or not self.active:
            return False
        pitch = max(-55., min(30., pitch))
        yaw = max(-45., min(45., yaw))
        delta = max(abs(pitch-p['pitch']), abs(yaw-p['yaw']))
        if delta < .4:
            return False
        duration = max(.6, max(abs(pitch-p['pitch']), abs(yaw-p['yaw'])) / 6.)
        # Record uncertainty BEFORE the one mutation, never resend on timeout.
        self.unknown_send = True
        self.event('head_send_requested', kind=kind, pitch=pitch, yaw=yaw)
        result = self.client.move(pitch, yaw, duration)
        require(result['action_id'] != self.owner_id, '重复头部任务标识；结果不明')
        self.owner_id = result['action_id']
        self.pending = {'action_id': self.owner_id, 'target': [pitch, yaw], 'kind': kind,
                        'deadline': self.clock()+duration+20}
        self.unknown_send = False
        self.commands += 1
        self.stable = []
        return True

    def step(self):
        with self.control:
            if not self.active and not self.pending:
                return
            try:
                p = self.check_robot(self.client.robot())
                if p is None or not self.active or self.stop_requested.is_set():
                    return
                try:
                    frame = parse_frame(self.client.detections(), self.wall())
                except Exception as exc:
                    self.input_error = str(exc)[:200]
                    self.state = 'input_wait'
                    # Transport/inference failure is NOT evidence of an absent person.
                    self.missing_since = None
                    self.misses = 0
                    return
                self.input_error = None
                if self.seq is not None:
                    require(frame['seq'] >= self.seq and frame['received'] >= self.received, '检测流重启或时间回退；请重新开始跟随')
                    if frame['seq'] == self.seq or frame['received'] == self.received or frame['header'] == self.header:
                        return
                self.seq, self.received, self.header = frame['seq'], frame['received'], frame['header']
                if frame['received'] <= self.after_frame:
                    return
                if self.returning:
                    self.state = 'returning_home'
                    if max(abs(p['pitch']), abs(p['yaw'])) <= .8:
                        self.returning = False
                        self.target = self.candidate = None
                        self.candidate_since = None
                        self.state = 'waiting_person'
                        self.event('home_verified')
                    else:
                        self.dispatch(p, 0., 0., 'return_home')
                    return
                boxes = frame['boxes']
                if self.target is None:
                    central = [b for b in boxes if abs(center(b)[0]-.5) <= .2 and abs(center(b)[1]-.5) <= .35]
                    central.sort(key=lambda b: math.hypot(center(b)[0]-.5, center(b)[1]-.5))
                    box = central[0] if central else None
                    if box is None or (len(central) > 1 and abs(math.dist(center(central[0]), (.5,.5))-
                            math.dist(center(central[1]), (.5,.5))) < .08):
                        self.candidate = None
                        self.state = 'waiting_person'
                        return
                    if self.candidate is None or associate(self.candidate, [box]) is None:
                        self.candidate, self.candidate_since = box, self.clock()
                        return
                    if self.clock()-self.candidate_since < .25:
                        return
                    self.target = box
                    self.event('person_acquired', box=box)
                else:
                    box = associate(self.target, boxes)
                    if box is None:
                        self.misses += 1
                        if self.missing_since is None:
                            self.missing_since = self.clock()
                        self.state = 'occluded'
                        if self.misses >= 3 and self.clock()-self.missing_since >= LOSS_SECONDS:
                            self.returning = True
                            self.state = 'returning_home'
                            self.event('person_lost_confirmed')
                            self.dispatch(p, 0., 0., 'return_home')
                        return
                    self.target = box
                self.last_seen = self.clock()
                self.missing_since = None
                self.misses = 0
                self.state = 'tracking'
                # Aim at upper torso, with an image deadband to avoid hunting.
                x, _ = center(self.target)
                y = self.target[1] + .30*(self.target[3]-self.target[1])
                ex, ey = x-.5, y-.5
                dp = 0. if abs(ey) <= .08 else -ey*25.
                dy = 0. if abs(ex) <= .08 else -ex*30.
                self.dispatch(p, p['pitch']+dp, p['yaw']+dy, 'follow')
            except Exception as exc:
                self.fail(exc)

    def launch(self):
        def worker():
            while not self.closed:
                self.step()
                self.wake.wait(.075)
                self.wake.clear()
        self.thread = threading.Thread(target=worker, name='serial-head-follow', daemon=True)
        self.thread.start()

    def close(self):
        with self.control:
            self.active = False
            self.closed = True
        self.wake.set()
        if self.thread:
            self.thread.join(timeout=10)
