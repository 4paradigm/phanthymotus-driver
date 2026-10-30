"""Feedback validation and a callback to the existing guarded move_head path."""
import math


class Error(ValueError):
    pass


def require(ok, message):
    if not ok:
        raise Error(message)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class Client:
    """Validated callbacks into the owning TRON2 adapter, with no HTTP listener."""
    def __init__(self, robot, detections, move):
        self.robot = robot
        self.detections = detections
        self._move = move

    def move(self, pitch, yaw, duration):
        require(all(finite(v) for v in (pitch, yaw, duration)), '无效头部目标')
        require(-55 <= pitch <= 30 and -45 <= yaw <= 45 and .6 <= duration <= 15,
                '头部目标超出跟随范围')
        data = self._move(pitch, yaw, duration)
        require(isinstance(data, dict) and data.get('motion_sent') is True and
                isinstance(data.get('action_id'), str), '头部请求结果不明；不重发')
        return data


def terminal(motion):
    return isinstance(motion, dict) and motion.get('status') == 'completed' and \
        motion.get('arrival_verified') is True and motion.get('motion_may_continue') is False and \
        motion.get('callback_delivered') is True and \
        motion.get('state') not in ('moving', 'reporting', 'blocked_requires_operator_check')


def pose(health, stationary=True):
    require(isinstance(health, dict) and health.get('robot_connected') is True and health.get('ready') is True,
            '机器人未就绪；不发送新指令')
    require(str(health.get('version', '')).startswith('0.12.'), '需核对新的驱动版本兼容性')
    sample = health.get('samples', {}).get('joint_state') or {}
    data = sample.get('data') or {}
    q, dq = data.get('q'), data.get('dq')
    require(sample.get('available') is True and finite(sample.get('reception_age_seconds')) and
            0 <= sample['reception_age_seconds'] <= 1 and isinstance(q, list) and
            isinstance(dq, list) and len(q) == len(dq) == 16 and all(finite(v) for v in q + dq) and
            finite(data.get('robot_timestamp')), '缺少新鲜完整的关节反馈')
    for name in ('arm_motion', 'pose_motion'):
        motion = health.get(name)
        require(motion is None or terminal(motion), '其他机械臂动作仍在执行或结果不明')
    for owner in health.get('grippers', {}).values():
        require(owner.get('state') in ('ready', 'idle') and not owner.get('recovering') and
                not owner.get('waiting_action'), '夹爪任务未解除')
    require(all(abs(v) <= .05 for v in dq[:14]), '机械臂未静止')
    if stationary:
        require(all(abs(v) <= .05 for v in dq[14:]), '头部未静止')
        head = health.get('head_motion')
        require(head is None or terminal(head), '头部已有动作或结果不明')
        require(head is None or head.get('can_accept_motion') is True, '头部驱动尚未释放')
    pitch, yaw = -math.degrees(q[14]), math.degrees(q[15])
    require(-55 <= pitch <= 30 and -45 <= yaw <= 45, '当前头部在跟随工作范围外；请先单独调整')
    return {'pitch': pitch, 'yaw': yaw, 'q': q, 'stamp': data['robot_timestamp'],
            'head_id': (health.get('head_motion') or {}).get('action_id')}
