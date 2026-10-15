"""Vendor WebSocket reader with explicit, correlated MoveJ calls."""
import asyncio
import json
import math
import re
import time
import uuid

import aiohttp
from state import joint_state, select_fields, STATUS_FIELDS


def reject_constant(value):
    raise ValueError('Non-finite JSON number')


# LimX SDK guide 3.6.1: left arm 7 then right arm 7 (radians), no head.
ARM_LOWER = (-3.1416, -.2618, -3.6652, -2.6180, -1.7453, -.7854, -1.5708,
             -3.1416, -3.1940, -1.4835, -2.6180, -1.3963, -.7854, -1.5708)
ARM_UPPER = (2.6005, 3.1940, 1.4835, .2618, 1.3963, .7854, 1.5708,
             2.6005, .2618, 3.6652, .2618, 1.7453, .7854, 1.5708)


def validate_movej(joint, duration):
    if not isinstance(joint, list) or len(joint) != 14:
        raise ValueError('MoveJ requires 14 arm angles in rad, excluding the 2 head joints')
    for index, value in enumerate(joint):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError('Joint angles must be finite numbers')
        try:
            valid = math.isfinite(value) and ARM_LOWER[index] <= value <= ARM_UPPER[index]
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError('Joint angle outside vendor limit at index ' + str(index))
    if isinstance(duration, bool) or not isinstance(duration, (int, float)):
        raise ValueError('time must be a finite number from 0.1 to 60 seconds')
    try:
        valid = math.isfinite(duration) and .1 <= duration <= 60
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError('time must be a finite number from 0.1 to 60 seconds')
    return list(joint), duration


class MoveError(ValueError):
    def __init__(self, message, motion_sent, code='unknown'):
        super().__init__(message)
        self.motion_sent = motion_sent
        self.code = code


def motion_readiness(data):
    """Admit the documented upper-level WebSocket developer mode for MoveJ.

    LimX SDK guide section 3.1 specifies upper-level developer mode on port 5000.
    *_valid fields are vendor capability/validity flags, not independent safety
    certification. Require explicit booleans; never infer readiness from missing
    status. Compliance/drag mode must be off for a commanded position move.
    """
    if data.get('working_mode') != 'developer_mode':
        return {'ready': False, 'reason': 'MoveJ requires upper-level developer_mode'}
    for key in ('working_mode_valid','collision_enable_valid','compliant_enable_valid'):
        if data.get(key) is not True:
            return {'ready': False, 'reason': key + ' must be explicitly true'}
    if data.get('collision_enable') is not True:
        return {'ready': False, 'reason': 'Collision protection must be explicitly enabled'}
    if data.get('compliant_enable') is not False:
        return {'ready': False, 'reason': 'Compliance/drag mode must be explicitly disabled for MoveJ'}
    return {'ready': True, 'reason': None}


class RobotReader:
    def __init__(self, settings):
        self.settings = settings
        self.connected = False
        self.serial = settings['serial'] or None
        self.info = self.joints = None
        self.info_at = self.joints_at = None
        self.info_received = self.joints_received = None
        self.last_error = None
        self.last_round_trip = None
        self.ws = None
        self.move_pending = None
        self.gripper_pending = {}

    def disconnect(self, reason):
        self.connected = False
        self.ws = None
        for _, future in self.gripper_pending.values():
            if not future.done():
                future.set_exception(MoveError('夹爪连接中断，结果未确认；不会自动重发', None))
        if self.move_pending:
            future = self.move_pending[1]
            if not future.done():
                future.set_exception(MoveError('Disconnected after send; result unknown, do not automatically retry', None))
        self.info = self.joints = None
        self.info_at = self.joints_at = None
        self.info_received = self.joints_received = None
        self.last_round_trip = None
        self.last_error = reason

    def accept_info(self, body):
        data = body.get('data')
        if not isinstance(data, dict):
            raise ValueError('Expected robot information object')
        serial = body.get('accid') or data.get('accid')
        if not isinstance(serial, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', serial) \
                or ('accid' in data and data['accid'] != serial) \
                or (self.serial is not None and serial != self.serial):
            raise ValueError('Robot information serial mismatch')
        # Validate only published fields; do not expose unrelated vendor payloads.
        select_fields(data, STATUS_FIELDS)
        self.serial = serial
        self.info, self.info_at, self.info_received = data, time.monotonic(), time.time()
        self.connected, self.last_error = True, None

    async def receive(self, ws, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise asyncio.TimeoutError('Robot read timeout')
        msg = await asyncio.wait_for(ws.receive(), remaining)
        if msg.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED,
                        aiohttp.WSMsgType.CLOSING, aiohttp.WSMsgType.ERROR):
            raise ConnectionError('Robot WebSocket disconnected')
        if msg.type != aiohttp.WSMsgType.TEXT:
            return {}
        body = json.loads(msg.data, parse_constant=reject_constant)
        if not isinstance(body, dict):
            raise ValueError('Expected robot JSON object')
        pending = self.gripper_pending.get(body.get('guid'))
        if pending and body.get('accid') == self.serial:
            expected, future = pending
            if not future.done() and body.get('title') == expected:
                future.set_result(body.get('data'))
            elif not future.done() and body.get('title') == 'notify_invalid_request':
                future.set_result({'result': 'fail_invalid_cmd'})
        if self.move_pending and body.get('guid') == self.move_pending[0] \
                and body.get('accid') == self.serial:
            future = self.move_pending[1]
            if not future.done() and body.get('title') == 'response_movej':
                future.set_result(body.get('data'))
            elif not future.done() and body.get('title') == 'notify_invalid_request':
                future.set_result({'result': 'fail_invalid_cmd'})
        if body.get('title') == 'notify_robot_info':
            self.accept_info(body)
        return body

    async def read_joints(self, ws):
        guid = str(uuid.uuid4())
        started = time.monotonic()
        # Deliberately no generic command API or caller-supplied command/payload.
        await ws.send_json({'title': 'request_get_joint_state',
                           'accid': self.serial, 'guid': guid,
                           'timestamp': int(time.time() * 1000), 'data': {}})
        deadline = started + self.settings['response_timeout_seconds']
        while True:
            body = await self.receive(ws, deadline)
            if body.get('guid') != guid:
                continue
            if body.get('title') == 'notify_invalid_request':
                raise ValueError('Robot rejected read-only query')
            if body.get('title') != 'response_get_joint_state':
                continue
            if body.get('accid') != self.serial:
                raise ValueError('Joint response serial mismatch')
            data = joint_state(body.get('data'))
            self.joints = data
            self.joints_at, self.joints_received = time.monotonic(), time.time()
            self.last_round_trip = self.joints_at - started
            self.last_error = None
            return

    async def send_movej(self, q, move_time):
        """Send once and await correlated acknowledgement; never claim arrival."""
        q, move_time = validate_movej(q, move_time)
        if not self.connected or self.ws is None or self.ws.closed or not self.serial:
            raise MoveError('Robot is disconnected; no motion command sent', False)
        status=self.sample('robot_status')
        if not status['available']:
            raise MoveError('Robot status is stale; no motion command sent', False)
        readiness=motion_readiness(status['data'])
        if not readiness['ready']:
            raise MoveError(readiness['reason'] + '; no motion command sent', False, 'vendor_not_ready')
        if self.move_pending:
            raise MoveError('A MoveJ reply is pending; no additional command sent', False)
        guid = str(uuid.uuid4())
        future = asyncio.get_running_loop().create_future()
        self.move_pending = (guid, future)
        try:
            try:
                await asyncio.wait_for(self.ws.send_json({'title': 'request_movej',
                    'accid': self.serial, 'guid': guid, 'timestamp': int(time.time()*1000),
                    'data': {'joint': q, 'time': move_time}}), 5)
            except (OSError, aiohttp.ClientError, asyncio.TimeoutError) as exc:
                raise MoveError('Send result unknown; do not automatically retry', None) from exc
            try:
                reply = await asyncio.wait_for(future, self.settings['response_timeout_seconds'])
            except asyncio.TimeoutError as exc:
                raise MoveError('MoveJ acknowledgement timed out; result unknown, do not automatically retry', True) from exc
            if not isinstance(reply, dict) or reply.get('result') != 'success':
                raise MoveError('Vendor rejected MoveJ; no arrival confirmed', True, 'vendor_rejected')
            return {'action': 'move', 'joint_count': 14, 'time': move_time,
                    'status': 'accepted', 'motion_sent': True, 'arrival_verified': False}
        finally:
            self.move_pending = None
            if not future.done():
                future.cancel()
            elif not future.cancelled():
                future.exception()  # retrieve a disconnect exception even if send failed

    async def _gripper_request(self, title, data, motion):
        if (motion is True and title != 'request_set_limx_2fclaw_cmd') or \
                (motion is False and (title != 'request_get_limx_2fclaw_state' or data != {})) or type(motion) is not bool:
            raise MoveError('不支持的夹爪协议请求', False)
        if not self.connected or self.ws is None or self.ws.closed or not self.serial:
            raise MoveError('机器人未连接，未发送夹爪指令', False)
        ws, serial = self.ws, self.serial
        guid = str(uuid.uuid4())
        future = asyncio.get_running_loop().create_future()
        self.gripper_pending[guid] = (title.replace('request_', 'response_', 1), future)
        sent = False
        try:
            try:
                await asyncio.wait_for(ws.send_json({'title': title, 'accid': serial, 'guid': guid,
                    'timestamp': int(time.time() * 1000), 'data': data}), 5)
                sent = True
                reply = await asyncio.wait_for(future, self.settings['response_timeout_seconds'])
            except (OSError, aiohttp.ClientError, asyncio.TimeoutError) as exc:
                raise MoveError('夹爪请求结果未确认；不会自动重发', (True if sent else None) if motion else False) from exc
            if self.ws is not ws or self.serial != serial or not self.connected:
                raise MoveError('机器人连接已改变，夹爪结果未确认', True if motion else False)
            if not isinstance(reply, dict) or reply.get('result') != 'success':
                raise MoveError('厂家拒绝夹爪请求或报告电机故障', True if motion else False, 'vendor_rejected')
            return reply
        finally:
            self.gripper_pending.pop(guid, None)
            if not future.done():
                future.cancel()
            elif not future.cancelled():
                future.exception()

    async def read_gripper(self):
        return await self._gripper_request('request_get_limx_2fclaw_state', {}, False)

    async def send_gripper(self, data):
        # This private transport accepts exactly one validated side, never a generic vendor command.
        from gripper import percentage, opening_target
        keys = set(data) if isinstance(data, dict) else set()
        sides = [side for side in ('left', 'right') if keys == {side + '_' + key for key in ('opening', 'speed', 'force')}]
        if len(sides) != 1:
            raise MoveError('夹爪参数必须只包含所选侧', False)
        for key, value in data.items():
            percentage(value, key)
        opening_target(data[sides[0] + '_opening'])
        if data[sides[0] + '_speed'] == 0:
            raise MoveError('开合速度须大于0', False)
        status = self.sample('robot_status')
        if not status['available'] or status['data'].get('working_mode') != 'developer_mode' or status['data'].get('working_mode_valid') is not True:
            raise MoveError('需要机器人处于上层开发模式且状态反馈有效，未发送夹爪指令', False)
        return await self._gripper_request('request_set_limx_2fclaw_cmd', data, True)

    def sample(self, name):
        if name not in ('joint_state', 'robot_status'):
            raise ValueError('Unknown sensor')
        info_card = name != 'joint_state'
        at = self.info_at if info_card else self.joints_at
        received = self.info_received if info_card else self.joints_received
        age = None if at is None else max(0, time.monotonic() - at)
        available = self.connected and age is not None \
            and age <= self.settings['stale_after_seconds']
        status = ('offline' if not self.connected else
                  'unavailable' if age is None else 'recently_received' if available else 'stale')
        data = None
        if available:
            if name == 'joint_state':
                data = self.joints
            elif name == 'robot_status':
                data = select_fields(self.info, STATUS_FIELDS)
            else:
                raise ValueError('Unknown sensor')
        return {'schema_version': 1, 'accid': self.serial, 'read_only': True, 'connected': self.connected,
                'available': available, 'sample_status': status,
                'received_at': received, 'reception_age_seconds': age,
                'measurement_time_verified': False, 'data': data}

    async def run(self, session):
        while True:
            try:
                self.disconnect(None)
                async with session.ws_connect(self.settings['url'], autoping=True,
                        max_msg_size=256 * 1024) as ws:
                    self.ws = ws
                    deadline = time.monotonic() + self.settings['response_timeout_seconds']
                    while not self.connected:
                        await self.receive(ws, deadline)
                    while True:
                        started = time.monotonic()
                        await self.read_joints(ws)
                        await asyncio.sleep(max(0, self.settings['poll_interval_seconds']
                                                - (time.monotonic() - started)))
            except (OSError, ValueError, TypeError, aiohttp.ClientError,
                    asyncio.TimeoutError) as exc:
                self.disconnect(type(exc).__name__)
                # Do not include endpoint/serial or vendor payloads in logs.
                await asyncio.sleep(2)
            finally:
                self.disconnect(self.last_error)
