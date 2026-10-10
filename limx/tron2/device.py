#!/usr/bin/env python3
"""Commissioned fixed-pose reset, enabled by the Motus card lifecycle."""
import asyncio
import contextlib
import hashlib
import json
import logging
import os
import time
from pathlib import Path

import aiohttp
from aiohttp import web

import pose
from motion import RobotClient, MotionFailure, preflight, plan_preview

LOG = logging.getLogger('tron2-reset')
PORT = 15791
VERSION = '0.3.0'
SERVER_NAME = 'LimX TRON2 Arms Reset'
POSE_PATH = Path(os.environ.get('TRON2_HOME_FILE', '/data/home.json'))
CONFIG_PATH = Path(os.environ.get('TRON2_COMMISSION_FILE', '/data/commissioning.json'))
LATCH_PATH = Path('/runstate/unconfirmed-motion.json')


class Adapter:
    def __init__(self, session, path=POSE_PATH, config_path=CONFIG_PATH,
                 latch_path=LATCH_PATH, robot_factory=RobotClient):
        self.session = session
        self.path = path
        self.registered_id = None
        self.config_path, self.latch_path = config_path, latch_path
        self.robot_factory = robot_factory
        self.lock = asyncio.Lock()
        self.platform_active = False
        self.lifecycle_epoch = 0
        self.active_task = None
        self.core_ssl = True
        self.stopping = False

    def config(self, digest):
        config = json.loads(self.config_path.read_text())
        if not isinstance(config, dict) or config.get('commissioned') is not True \
                or config.get('high_level_confirmed_by_operator') is not True \
                or config.get('pose_sha256') != digest \
                or not isinstance(config.get('expected_working_mode'), str) \
                or not config['expected_working_mode']:
            raise ValueError('Commissioning is incomplete or the recorded pose changed')
        return config

    def load_pose(self):
        raw = self.path.read_bytes()
        data = json.loads(raw)
        pose.validate_pose(data)
        return data, hashlib.sha256(raw).hexdigest()

    @staticmethod
    def tools():
        return [{'name': 'reset_pose', 'type': 'actuator', 'multiInstance': False,
                 'description': 'TRON 2 双臂固定姿态复位：平台开启智能控制后可执行 execute；按差值计算时间，连续反馈验收到位。只能使用已 commissioning 的目标，stop 禁用后续调用，不是急停。',
                 'inputSchema': {'type': 'object', 'additionalProperties': False,
                                 'properties': {'action': {'type': 'string',
                                     'enum': ['info', 'preview', 'verify_at_home', 'status', 'execute']}},
                                 'required': ['action'],
                                 'x-action-params': {
                                     'info': {'params': [], 'description': '查看已记录的复位姿态；不运动'},
                                     'preview': {'params': [], 'description': '比较当前与复位姿态；不运动'},
                                     'verify_at_home': {'params': [], 'description': '只检查当前双臂是否在记录姿态；不运动'},
                                     'status': {'params': [], 'description': '直接查询厂家示教状态；不切换模式、不运动'},
                                     'execute': {'params': [], 'description': '平台启用后，按关节差值计算时间执行 MoveJ 回到固定姿态并检查到位'}}}}]

    async def direct_status(self):
        async with self.robot_factory(self.session) as robot:
            current, teach = await robot.status()
        return {'robot': current['robot'], 'teach_status': teach,
                'joint_state': current['joint_state'], 'motion_sent': False,
                'automatic_high_level_confirmation': False}

    async def clear_latch(self, body):
        if body != {'confirm_inspected': True} or self.lock.locked():
            raise ValueError('Inspect the robot before clearing; no operation may be running')
        async with self.lock:
            self.platform_active = False
            self.lifecycle_epoch += 1
            await self.current()  # Fresh, healthy and stationary, including head.
            if self.latch_path.exists():
                self.latch_path.unlink()
            return {'armed': False, 'motion_sent': False,
                    'note': 'Unconfirmed-operation latch cleared. No stop command was sent.'}

    async def execute(self):
        if self.stopping:
            raise ValueError('Service is shutting down; no motion sent')
        if self.lock.locked():
            raise ValueError('A reset operation is already in progress')
        async with self.lock:
            if not self.platform_active:
                raise ValueError('Enable intelligent control in Motus before reset; no motion sent')
            epoch = self.lifecycle_epoch
            if self.latch_path.exists():
                raise ValueError('Previous operation is unconfirmed; no automatic retry')
            home, digest = self.load_pose()
            config = self.config(digest)
            async def before_send(current):
                if self.stopping or not self.platform_active or self.lifecycle_epoch != epoch:
                    raise ValueError('Platform stopped or restarted before send; no motion sent')
                if self.load_pose()[1] != digest or self.config(digest) != config:
                    raise ValueError('Target or commissioning changed before send')
                # Durable marker created BEFORE send; restart cannot silently retry.
                with self.latch_path.open('x') as file:
                    json.dump({'pose_sha256': digest, 'attempted_at': time.time()}, file)
                    file.flush()
                    os.fsync(file.fileno())
                directory = os.open(str(self.latch_path.parent), os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
            try:
                async with self.robot_factory(self.session) as robot:
                    result = await robot.move_and_verify(home, config, before_send)
            except MotionFailure:
                raise
            except (OSError, ValueError, TypeError, KeyError, aiohttp.ClientError, TimeoutError) as exc:
                # A transport-close error after a successful send is still an uncertain result.
                raise MotionFailure(str(exc), self.latch_path.exists()) from exc
            try:
                self.latch_path.unlink()
            except OSError as exc:
                raise MotionFailure('Arrived, but durable latch could not be cleared: ' + str(exc), True) from exc
            result['pose_sha256'] = digest
            return result

    async def operator(self, request):
        try:
            body = await request.json()
            if request.match_info['operation'] == 'clear':
                result = await self.clear_latch(body)
            else:
                raise ValueError('Unknown operator operation')
            return web.json_response(result)
        except (OSError, ValueError, TypeError, KeyError, aiohttp.ClientError, TimeoutError) as exc:
            return web.json_response({'error': str(exc), 'motion_sent': False}, status=409)

    async def current(self):
        # Self-contained: no separate prototype/read-only container is required.
        async with self.robot_factory(self.session) as robot:
            state, _teach = await robot.ready_state()
        pose.validate_snapshot(state)
        return state

    async def call(self, args):
        if not isinstance(args, dict) or 'action' not in args:
            raise ValueError('An action object is required')
        # Agent Core sends the canvas card ID on start, info, stop and manual
        # calls. It is routing metadata, never a robot joint or motion setting.
        unknown = set(args) - {'action', 'instance_id'}
        if unknown:
            raise ValueError('Unsupported parameter keys: ' +
                             ', '.join(sorted(str(k)[:64] for k in unknown)) +
                             '; custom target angles are forbidden')
        instance = args.get('instance_id', '')
        if not isinstance(instance, str) or len(instance) > 128 or \
                any(ord(c) < 32 or ord(c) == 127 for c in instance):
            raise ValueError('instance_id must be a short canvas card identifier')
        action = args['action']
        if not isinstance(action, str):
            raise ValueError('action must be a string')
        if action in ('start', 'stop'):
            if action == 'start':
                if self.stopping:
                    raise ValueError('Service is shutting down')
                home, digest = self.load_pose()
                self.config(digest)
                self.platform_active = True
            else:
                self.platform_active = False
            self.lifecycle_epoch += 1
            return {'state': 'executing' if self.lock.locked() else
                            ('ready' if action == 'start' else 'idle'),
                    'platform_active': self.platform_active,
                    'motion_enabled': self.platform_active and not self.latch_path.exists(),
                    'motion_sent': False, 'operation_in_progress': self.lock.locked(),
                    'note': 'Lifecycle controls future requests; stop does NOT stop physical motion.'}
        if action == 'status':
            if self.lock.locked():
                raise ValueError('Operation in progress')
            return await self.direct_status()
        if action == 'execute':
            if self.active_task is not None and not self.active_task.done():
                raise ValueError('Operation in progress; do not retry')
            # Keep completion monitoring alive if the HTTP caller disconnects/cancels.
            self.active_task = asyncio.create_task(self.execute())
            self.active_task.add_done_callback(lambda task: task.exception()
                if not task.cancelled() else None)
            return await asyncio.shield(self.active_task)
        if action not in ('info', 'preview', 'verify_at_home'):
            raise ValueError('Unsupported action')
        home, digest = self.load_pose()
        if action == 'info':
            return {'accid': home['accid'], 'target_q': home['target_q'],
                    'joint_names': home['joint_names'], 'unit': 'rad',
                    'recorded_at': home['recorded_at'], 'pose_sha256': digest,
                    'state': 'ready', 'motion_enabled': False, 'motion_sent': False,
                    'scope': 'arms_only', 'head_policy': 'unchanged', 'gripper_policy': 'unchanged'}
        current = await self.current()
        result = plan_preview(home, current)
        result.update({'pose_sha256': digest, 'motion_enabled': False,
                       'working_mode': current['robot'].get('working_mode'),
                       'working_mode_valid': current['robot'].get('working_mode_valid')})
        if action == 'verify_at_home':
            result['status'] = 'observation_only'
            result['tolerance_deg'] = 1.0
            result['at_home'] = result['max_joint_change_deg'] <= 1.0
        return result

    async def rpc(self, request):
        try:
            body = await request.json()
        except (ValueError, UnicodeError):
            return web.json_response({'jsonrpc': '2.0', 'id': None,
                                      'error': {'code': -32700, 'message': 'Parse error'}})
        if not isinstance(body, dict) or body.get('jsonrpc') != '2.0':
            return web.json_response({'jsonrpc': '2.0', 'id': None,
                                      'error': {'code': -32600, 'message': 'Invalid request'}})
        if 'id' not in body:
            return web.Response(status=204)
        rid, method = body['id'], body.get('method')
        try:
            params = body.get('params', {})
            if not isinstance(params, dict):
                raise ValueError('Parameters must be an object')
            if method == 'initialize':
                result = {'protocolVersion': '2024-11-05', 'capabilities': {'tools': {}},
                          'serverInfo': {'name': SERVER_NAME, 'version': VERSION}}
            elif method == 'tools/list':
                result = {'tools': self.tools()}
            elif method == 'resources/list':
                result = {'resources': []}
            elif method == 'ping':
                result = {}
            elif method == 'tools/call':
                if params.get('name') != 'reset_pose':
                    raise ValueError('Unknown tool')
                try:
                    data = await self.call(params.get('arguments', {}))
                    error = False
                except MotionFailure as exc:
                    data, error = {'error': str(exc)[:300],
                        'motion_sent': 'unknown' if exc.attempted else False,
                        'motion_attempted': exc.attempted,
                        'arrival_verified': False,
                        'operator_inspection_required': exc.attempted,
                        'automatic_retry': False}, True
                except (OSError, ValueError, TypeError, KeyError, aiohttp.ClientError,
                        TimeoutError) as exc:
                    data, error = {'error': str(exc)[:300], 'motion_sent': False}, True
                result = {'content': [{'type': 'text', 'text': json.dumps(data,
                           ensure_ascii=False, allow_nan=False)}], 'isError': error}
            else:
                return web.json_response({'jsonrpc': '2.0', 'id': rid,
                         'error': {'code': -32601, 'message': 'Method not found'}})
            return web.json_response({'jsonrpc': '2.0', 'id': rid, 'result': result})
        except (ValueError, TypeError) as exc:
            return web.json_response({'jsonrpc': '2.0', 'id': rid,
                         'error': {'code': -32602, 'message': str(exc)}})

    async def health(self, request):
        try:
            home, digest = self.load_pose()
            enabled = self.platform_active and not self.stopping
            return web.json_response({'pose_valid': True, 'pose_sha256': digest,
                                      'accid': home['accid'],
                                      'motion_enabled': enabled and not self.latch_path.exists(),
                                      'motion_capability': True, 'version': VERSION,
                                      'armed': False, 'platform_active': enabled,
                                      'unconfirmed_operation': self.latch_path.exists(),
                                      'operation_in_progress': self.lock.locked(),
                                      'registered_mcp_id': self.registered_id})
        except (OSError, ValueError, TypeError) as exc:
            return web.json_response({'pose_valid': False, 'error': str(exc), 'version': VERSION,
                'armed': False, 'platform_active': False, 'motion_enabled': False,
                'operation_in_progress': self.lock.locked(),
                'unconfirmed_operation': self.latch_path.exists(),
                'registered_mcp_id': self.registered_id}, status=503)

    async def register(self):
        last = None
        while True:
            try:
                async with self.session.post(os.environ.get('AGENT_CORE_URL', 'https://phanthy-motus:15678').rstrip('/') + '/api/mcp',
                     ssl=self.core_ssl, allow_redirects=False, json={
                         'name': SERVER_NAME, 'transport': 'http',
                         'url': 'http://127.0.0.1:15791/mcp', 'category': 'driver',
                         'render_hint': 'data/json'}) as resp:
                    if resp.status != 200:
                        raise ValueError('Core HTTP ' + str(resp.status))
                    result = await resp.json()
                if result.get('code') != 200 or not result.get('data', {}).get('id'):
                    raise ValueError('Core registration failed')
                self.registered_id = result['data']['id']
                if last != 'ok':
                    LOG.info('Registered with Motus: %s', self.registered_id)
                last = 'ok'
            except (OSError, ValueError, TypeError, aiohttp.ClientError, TimeoutError) as exc:
                self.registered_id = None
                if last != 'error':
                    LOG.warning('Registration pending: %s', str(exc)[:240])
                last = 'error'
            await asyncio.sleep(30 if last == 'ok' else 5)
