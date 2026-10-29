"""Only loopback reads and the existing, protected two-axis move_head tool."""
import json
import math
import urllib.request


class Error(RuntimeError):
    pass


def require(ok, message):
    if not ok:
        raise Error(message)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise Error('服务重定向被拒绝')


class Client:
    def request(self, port, path, body=None):
        require(port in (15791, 15792) and path in ('/health', '/mcp'), '不支持的接口')
        request = urllib.request.Request(
            'http://127.0.0.1:%d%s' % (port, path),
            data=None if body is None else json.dumps(body, allow_nan=False).encode(),
            headers={'Content-Type': 'application/json'})
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
        with opener.open(request, timeout=6 if body is not None else 2) as response:
            raw = response.read(1024 * 1024 + 1)
        require(len(raw) <= 1024 * 1024, '服务响应超限')
        return json.loads(raw)

    def robot(self):
        return self.request(15791, '/health')

    def detections(self):
        return self.request(15792, '/health')

    def move(self, pitch, yaw, duration):
        require(all(finite(v) for v in (pitch, yaw, duration)), '无效头部目标')
        require(-55 <= pitch <= 30 and -45 <= yaw <= 45 and .6 <= duration <= 15,
                '头部目标超出跟随范围')
        reply = self.request(15791, '/mcp', {
            'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call',
            'params': {'name': 'move_head', 'arguments': {
                'action': 'move', 'pitch_deg': pitch, 'yaw_deg': yaw, 'time': duration}}})
        require('error' not in reply and isinstance(reply.get('result'), dict), '头部请求结果不明；不重发')
        result = reply['result']
        data = result.get('structuredContent')
        if not isinstance(data, dict):
            data = json.loads(next(c['text'] for c in result.get('content', []) if c['type'] == 'text'))
        require(result.get('isError') is not True, '头部驱动拒绝：' + str(data.get('error', data))[:240])
        require(data.get('motion_sent') is True and isinstance(data.get('action_id'), str),
                '头部请求结果不明；不重发')
        return data


def terminal(motion):
    return isinstance(motion, dict) and motion.get('status') == 'completed' and \
        motion.get('arrival_verified') is True and motion.get('motion_may_continue') is False and \
        motion.get('callback_delivered') is True and \
        motion.get('state') not in ('moving', 'reporting', 'blocked_requires_operator_check')


def pose(health, stationary=True):
    require(isinstance(health, dict) and health.get('robot_connected') is True and health.get('ready') is True,
            '机器人未就绪；不发送新指令')
    require(str(health.get('version', '')).startswith('0.11.'), '需核对新的驱动版本兼容性')
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
