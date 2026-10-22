"""Direct LimX WebSocket feedback and ServoJ; no Motus-driver client."""
import asyncio
import json
import re
import time
import uuid

import aiohttp

from limx.tron2_teleop.model import feedback_q, validate_q

JOINT_MAX_AGE = .2
STATUS_MAX_AGE = 3.


def decode(text):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate_json_key')
            result[key] = value
        return result
    def reject(value):
        raise ValueError('nonfinite_json')
    value = json.loads(text, object_pairs_hook=unique, parse_constant=reject)
    if not isinstance(value, dict):
        raise ValueError('invalid_json_object')
    return value


class Robot:
    def __init__(self, url, serial=''):
        self.url, self.serial = url, serial or None
        self.ws = None
        self.epoch = 0
        self.info = self.q = None
        self.info_at = self.q_at = 0.
        self.pending = None
        self.error = 'robot_disconnected'
        self.servo_error = None
        self.sent_count = 0

    def ready(self):
        now = time.monotonic()
        if self.ws is None or self.ws.closed or self.q is None or self.info is None:
            return 'robot_disconnected'
        if not 0 <= now-self.q_at <= JOINT_MAX_AGE:
            return 'joint_feedback_stale'
        if not 0 <= now-self.info_at <= STATUS_MAX_AGE:
            return 'robot_status_stale'
        if self.servo_error:
            return self.servo_error
        if self.info.get('motor') != 'OK':
            return 'motor_not_ready'
        if self.info.get('working_mode') != 'developer_mode':
            return 'developer_mode_required'
        for key in ('working_mode_valid', 'collision_enable_valid', 'compliant_enable_valid', 'collision_enable'):
            if self.info.get(key) is not True:
                return key + '_required'
        if self.info.get('compliant_enable') is not False:
            return 'compliance_must_be_off'
        return None

    def packet(self, title, data):
        return {'accid': self.serial, 'title': title, 'timestamp': int(time.time()*1000),
                'guid': uuid.uuid4().hex, 'data': data}

    def accept(self, body):
        title, data, serial = body.get('title'), body.get('data'), body.get('accid')
        if title == 'notify_robot_info':
            if (not isinstance(serial, str) or not re.fullmatch(r'DACH_TRON2A_[A-Za-z0-9_-]{1,64}', serial)
                    or self.serial not in (None, serial) or not isinstance(data, dict)
                    or data.get('accid', serial) != serial or not isinstance(data.get('sw_version'), str)):
                raise ValueError('robot_identity_mismatch')
            self.serial = serial
            self.info, self.info_at = data, time.monotonic()
        elif title in ('notify_servoJ', 'notify_servoj', 'notify_invalid_request'):
            if serial != self.serial:
                raise ValueError('robot_identity_mismatch')
            self.servo_error = 'vendor_rejected_command'
        elif title == 'response_get_joint_state' and self.pending and body.get('guid') == self.pending[0]:
            if serial != self.serial:
                raise ValueError('robot_identity_mismatch')
            q = feedback_q(data)
            self.q, self.q_at = q, time.monotonic()
            future = self.pending[1]
            if not future.done():
                future.set_result(None)

    async def receive(self, ws):
        async for message in ws:
            if message.type == aiohttp.WSMsgType.TEXT:
                self.accept(decode(message.data))
            elif message.type == aiohttp.WSMsgType.ERROR:
                raise ConnectionError('robot_receive_failed')
        raise ConnectionError('robot_disconnected')

    async def poll(self, ws):
        while self.info is None:
            await asyncio.sleep(.01)
        while True:
            packet = self.packet('request_get_joint_state', {})
            future = asyncio.get_running_loop().create_future()
            self.pending = (packet['guid'], future)
            try:
                await asyncio.wait_for(ws.send_json(packet), .1)
                await asyncio.wait_for(future, .15)
            finally:
                self.pending = None
            await asyncio.sleep(.02)

    async def send(self, q):
        q = validate_q(q)
        reason = self.ready()
        if reason:
            raise ValueError(reason)
        # No ACK exists for ServoJ. A completed send only proves socket submission.
        await asyncio.wait_for(self.ws.send_json(self.packet('request_servoj', {'q': q, 'filter_ratio': 1.0})), .01)
        self.sent_count += 1

    async def run(self, session):
        while True:
            tasks = []
            try:
                async with session.ws_connect(self.url, max_msg_size=65536, heartbeat=2) as ws:
                    self.ws, self.epoch = ws, self.epoch+1
                    self.error = None
                    tasks = [asyncio.create_task(self.receive(ws)), asyncio.create_task(self.poll(ws))]
                    done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                    for task in done:
                        task.result()
            except (OSError, ValueError, TypeError, aiohttp.ClientError, asyncio.TimeoutError) as exc:
                self.error = str(exc)[:160] or type(exc).__name__
            finally:
                self.ws = None
                self.info = self.q = None
                self.info_at = self.q_at = 0.
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
            await asyncio.sleep(1)

    def status(self):
        now = time.monotonic()
        return {'connected': self.ws is not None and not self.ws.closed, 'connection_epoch': self.epoch,
                'ready': self.ready() is None, 'reason': self.ready(), 'last_error': self.error,
                'joint_age_seconds': now-self.q_at if self.q is not None else None,
                'q': list(self.q) if self.q is not None and now-self.q_at <= JOINT_MAX_AGE else None,
                'status_age_seconds': now-self.info_at if self.info is not None else None,
                'sent_count': self.sent_count, 'measurement_time_verified': False}
