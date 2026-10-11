"""Vendor WebSocket reader: the only application request is joint-state query."""
import asyncio
import json
import re
import time
import uuid

import aiohttp
from state import joint_state, select_fields, STATUS_FIELDS, battery_state


def reject_constant(value):
    raise ValueError('Non-finite JSON number')


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

    def disconnect(self, reason):
        self.connected = False
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
        battery_state(data)
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

    def sample(self, name):
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
            elif name == 'battery_state':
                data = battery_state(self.info)
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
