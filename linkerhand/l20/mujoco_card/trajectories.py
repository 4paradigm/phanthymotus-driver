"""Simulation-only instances of the L20 motion cards and their DDS contract."""
import math
import re
import threading
from types import SimpleNamespace

from adaptive_grasp_device import AdaptiveGraspPlugin, _finger_tip_travel_percent
from can_client import FINGER_NAMES
from common.vendor_runtime import action_schema, tool
from gesture_device import ACTIONS, GesturePlugin
from grasp_feedback import Contact, FeedbackGrasp, settings_from
from hand_device import SPEED_RATIOS

FORMAT = 'data/l20_joint_trajectory'
VERSION = 'motus.l20.trajectory/1'
SIDES = ('left', 'right')
FPS = 20
SIDE_CONFIG = {'type': 'object', 'properties': {'bound_side': {
    'type': 'string', 'enum': list(SIDES), 'default': 'right', 'scope': 'instance',
    'title': '模拟手', 'description': '一个实例绑定一只虚拟手。'}}}


def instance(args):
    value = args.get('instance_id', '')
    if not isinstance(value, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,100}', value):
        raise ValueError('请在画布实例上使用此动作')
    return value


def channel(side, name, key):
    # Encode the original instance identity without ROS name collisions.
    return f'/l20_sim/{side}/{name}/i_{key.encode().hex()}'


def validate_packet(packet):
    if not isinstance(packet, dict) or packet.get('schema') != VERSION or packet.get('side') not in SIDES:
        raise ValueError('轨迹协议或左右手无效')
    for field in ('id', 'action'):
        if not isinstance(packet.get(field), str) or not 1 <= len(packet[field]) <= 100:
            raise ValueError('轨迹缺少动作标识')
    frames, ms = packet.get('frames'), packet.get('frame_ms')
    if type(ms) is not int or not 20 <= ms <= 1000:
        raise ValueError('frame_ms 允许 20–1000 毫秒')
    if not isinstance(frames, list) or not 1 <= len(frames) <= 600:
        raise ValueError('一次动作需要 1–600 帧')
    if any(not isinstance(f, list) or len(f) != 20 or
           any(type(v) is not int or not 0 <= v <= 255 for v in f) for f in frames):
        raise ValueError('每帧应为 L20 的 20 个 0–255 整数位置')
    return packet


class VirtualJoints:
    """The existing gesture/clearance planners write their setpoints to this recorder."""
    def __init__(self, start, step):
        self.frames = [list(start)]
        self.step = step

    def get_positions_vector(self):
        return self.frames[-1].copy()

    def set_joint_positions(self, target):
        initial = self.frames[-1]
        moving = [i for i in range(20) if not 11 <= i <= 14]
        count = max(1, math.ceil(max(abs(target[i] - initial[i]) for i in moving) / self.step))
        for n in range(1, count + 1):
            self.frames.append([round(a + (b - a)*n/count) for a,b in zip(initial,target)])


class MotionPlanner:
    def __init__(self, config):
        plugins = config['plugins']
        self.gestures = GesturePlugin(None, plugins['hand_gesture'], config)
        self.grasp = AdaptiveGraspPlugin(None, plugins['hand_grasp'], config)
        h = plugins['hand']
        self.open = {s: h.get(f'{s}_open_pose', h['open_pose']) for s in SIDES}

    def plan(self, name, action, side, current, args):
        ratio = args.get('speed_ratio', '1.0x')
        if ratio not in SPEED_RATIOS:
            raise ValueError('无效的速度倍率')
        joints = VirtualJoints(current, max(1, round(7 * SPEED_RATIOS[ratio])))
        if name == 'hand_gesture':
            poses = self.gestures._calibration[side]
            speed = self.gestures._speed(ratio,self.gestures.speed)
            if action == 'number':
                number = args.get('number', 0)
                if type(number) is not int or not 0 <= number <= 9:
                    raise ValueError('数字应为 0–9')
                self.gestures._number(joints,side,poses,{'speed':speed,'number':number})
            elif action in ACTIONS:
                self.gestures._pose(joints,poses[action],speed)
            else:
                raise ValueError('未知模拟手势')
        else:
            profile = settings_from({'gentle_limits': True, **{k:args[k] for k in ('approach_travel',) if k in args}})
            prep = self.grasp.prep_pose.copy()
            from adaptive_grasp_device import RIGHT_GENTLE_THUMB_YAW_OFFSET
            if side == 'right':
                prep[5] = min(255,prep[5] + RIGHT_GENTLE_THUMB_YAW_OFFSET)
            ctrl = SimpleNamespace(_open_pose=self.open[side], _motion_lock=threading.RLock())
            if action in ('prepare','release'):
                if action == 'release':
                    joints.set_joint_positions(prep)
                self.grasp._open_hand(joints,ctrl)
                if action == 'prepare':
                    joints.set_joint_positions(prep)
            elif action in ('adaptive_grasp','close_until_contact'):
                if action == 'adaptive_grasp':
                    self.grasp._open_hand(joints,ctrl)
                joints.set_joint_positions(prep)
                feedback = FeedbackGrasp(profile,prep,self.grasp._feedback_close_limit(side,True),
                                         finger_tip_travel_percent=_finger_tip_travel_percent(side,True))
                empty = dict.fromkeys(FINGER_NAMES,Contact(0,0,0,0))
                for n in range(300):
                    feedback.update(empty,feedback.target.copy(),n/FPS,False)
                    if feedback.search_stop:
                        break
                    joints.frames.append(feedback.target.copy())
            else:
                raise ValueError('未知模拟抓取动作')
        return joints.frames


class SimulatedMotionCard:
    def __init__(self, name, runtime, planner):
        self.name, self.runtime, self.planner = name, runtime, planner

    def get_tool(self):
        original = self.planner.gestures if self.name == 'hand_gesture' else self.planner.grasp
        old = original.get_tool()['inputSchema']
        names = list(ACTIONS) if self.name == 'hand_gesture' else ['adaptive_grasp','prepare','close_until_contact','release']
        names = sorted(names) if self.name == 'hand_gesture' else names
        descriptions = {'adaptive_grasp':'空手收拢（完整轨迹）','close_until_contact':'空手收拢至行程限位',
                        'prepare':'张开到准备姿态','release':'张开手', 'number':'数字手势',
                        'v_sign_gesture':'V 字手势','ok_gesture':'OK 手势','thumb_up_gesture':'点赞','natural_gesture':'自然姿态'}
        actions = {}
        for name in names:
            fields = ['speed_ratio'] if self.name == 'hand_gesture' else []
            if name in ('adaptive_grasp','close_until_contact'): fields.append('approach_travel')
            if name == 'number': fields.append('number')
            actions[name] = (fields, descriptions[name])
        actions.update({a:([],a) for a in ('start','stop','info','config')})
        fields = {k:old['properties'][k] for k in ('speed_ratio','number','approach_travel') if k in old['properties']}
        fields['output_topic'] = {'type': 'string', 'description': '画布实例的轨迹输出话题'}
        schema = action_schema(actions,fields)
        schema['properties']['action']['oneOf'] = [{'const':a,'title':descriptions.get(a,a)} for a in actions]
        result = tool(self.name,'actuator','仅仿真 · 输出虚拟手轨迹到已连线的 MuJoCo',schema,
                      topic_out=[{'format':FORMAT}])
        result.update(multiInstance=True, title=('手势' if self.name=='hand_gesture' else '抓取')+' · 仅仿真',configSchema=SIDE_CONFIG)
        return result

    def dispatch(self, action, args):
        # Canvas manual execution identifies an actuator by its resolved output port.
        # Workbench, lifecycle and RPC calls normally identify it by instance_id.
        output = args.get('output_topic')
        if not args.get('instance_id') and output:
            match = re.fullmatch(r'/l20_sim/(left|right)/' + self.name + r'/i_([a-f0-9]{2,200})', str(output))
            if not match:
                raise ValueError('轨迹输出话题与这张动作卡不匹配')
            try:
                key = bytes.fromhex(match[2]).decode('ascii')
            except ValueError as exc:
                raise ValueError('轨迹输出话题的实例编号无效') from exc
            args = {**args, 'instance_id': key}
        if not args.get('instance_id') and action in ('info','start','config','stop'):
            return {'state':'ready','hardware':False,'topic_out':[]}
        key = instance(args)
        side = self.runtime.side_for(self.name,key,args,configuring=action=='config')
        topic = channel(side,self.name,key)
        if output and output != topic:
            raise ValueError('轨迹输出话题与实例侧别不匹配')
        state = {'hardware':False,'bound_side':side,'state':'ready','topic_out':[{'topic':topic,'format':FORMAT}]}
        if action == 'stop':
            self.runtime.cancel_source(topic)
        if action in ('info','config','start','stop'):
            return state
        return self.runtime.submit(topic,side,action,lambda pose:self.planner.plan(self.name,action,side,pose,args))
