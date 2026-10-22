"""Standalone TRON2 dual-arm teleoperation MCP service."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from common import logsafe
logsafe.install()

import asyncio
import json
import logging
import math
import os
import re
import signal
import ssl
import time
from urllib.parse import urlparse

import aiohttp
from aiohttp import web
import yaml

from limx.tron2_teleop.bus import TeleopBus
from limx.tron2_teleop.control import TeleopControl, make_worker
from limx.tron2_teleop.vendor import Robot, decode

PORT = 15794
VERSION = '0.1.0'
NAME = 'LimX TRON2 Teleoperation'
LOG = logging.getLogger('tron2-teleop')


def load_settings():
    path = Path(os.environ.get('CONFIG_PATH', Path(__file__).with_name('config.yaml')))
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict) or config.get('mcp_port') != PORT:
        raise ValueError('expected_mcp_port_15794')
    robot = config.get('robot', {})
    if not isinstance(robot, dict):
        raise ValueError('invalid_robot_config')
    url = os.environ.get('TRON2_ROBOT_URL') or robot.get('url', '')
    parsed = urlparse(url)
    if (parsed.scheme not in ('ws', 'wss') or not parsed.hostname or parsed.port != 5000
            or parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in ('', '/')):
        raise ValueError('vendor_websocket_port_5000_required')
    serial = os.environ.get('TRON2_SERIAL') or robot.get('serial', '')
    if not isinstance(serial, str) or serial and not re.fullmatch(r'DACH_TRON2A_[A-Za-z0-9_-]{1,64}', serial):
        raise ValueError('invalid_robot_serial')
    scale = config.get('position_scale', .5)
    if type(scale) not in (float, int) or not math.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError('invalid_position_scale')
    return url, serial, scale


class Adapter:
    def __init__(self, control):
        self.control, self.registered_id = control, None
        self.lifecycle = asyncio.Lock()

    async def rpc(self, request):
        try:
            body = decode(await request.text())
        except (ValueError, UnicodeError, RecursionError):
            return web.json_response({'jsonrpc': '2.0', 'id': None, 'error': {'code': -32700, 'message': 'Invalid JSON'}})
        rid = body.get('id')
        if body.get('jsonrpc') != '2.0':
            return web.json_response({'jsonrpc': '2.0', 'id': rid, 'error': {'code': -32600, 'message': 'Invalid request'}})
        if 'id' not in body:
            return web.Response(status=204)
        try:
            method = body.get('method')
            if method == 'initialize':
                result = {'protocolVersion': '2024-11-05', 'capabilities': {'tools': {}},
                          'serverInfo': {'name': NAME, 'version': VERSION}}
            elif method == 'tools/list':
                result = {'tools': [self.control.get_tool()]}
            elif method == 'resources/list':
                result = {'resources': []}
            elif method == 'ping':
                result = {}
            elif method == 'tools/call':
                params = body.get('params', {})
                if not isinstance(params, dict) or params.get('name') != 'teleop_control':
                    raise ValueError('unknown_tool')
                args = params.get('arguments', {})
                if not isinstance(args, dict):
                    raise ValueError('invalid_arguments')
                async with self.lifecycle:
                    action = args.get('action', 'info')
                    if action in ('info', 'config'):
                        data = self.control.info()
                    elif action == 'start':
                        data = self.control.start(args.get('input_topic'))
                    elif action == 'stop':
                        data = await self.control.stop()
                    else:
                        raise ValueError('unknown_action')
                result = {'content': [{'type': 'text', 'text': json.dumps(data, ensure_ascii=False, allow_nan=False)}], 'isError': False}
            else:
                return web.json_response({'jsonrpc': '2.0', 'id': rid, 'error': {'code': -32601, 'message': 'Method not found'}})
            return web.json_response({'jsonrpc': '2.0', 'id': rid, 'result': result})
        except (ValueError, TypeError) as exc:
            return web.json_response({'jsonrpc': '2.0', 'id': rid, 'error': {'code': -32602, 'message': str(exc)[:160]}})

    async def health(self, request):
        return web.json_response({'version': VERSION, 'registered_mcp_id': self.registered_id,
                                 **self.control.info()}, status=200 if self.control.robot.ready() is None else 503)

    async def register(self, session):
        url = os.environ.get('AGENT_CORE_URL', 'https://phanthy-motus:15678').rstrip('/')
        ca = os.environ.get('AGENT_CORE_CA_CERT')
        context = ssl.create_default_context(cafile=ca) if ca else ssl.create_default_context()
        last_error = None
        while True:
            try:
                async with session.post(url + '/api/mcp', ssl=context, allow_redirects=False, json={
                    'name': NAME, 'transport': 'http', 'url': f'http://127.0.0.1:{PORT}/mcp',
                    'category': 'driver', 'render_hint': 'data/teleop-state'}) as response:
                    if response.status != 200:
                        raise ValueError('registration_http_' + str(response.status))
                    body = await response.json()
                if body.get('code') != 200 or not isinstance(body.get('data'), dict) or not body['data'].get('id'):
                    raise ValueError('registration_id_missing')
                self.registered_id, last_error = body['data']['id'], None
            except (OSError, ValueError, TypeError, AttributeError, aiohttp.ClientError, asyncio.TimeoutError) as exc:
                self.registered_id = None
                error = type(exc).__name__
                if last_error != error:
                    LOG.warning('Core registration pending: %s', error)
                last_error = error
            await asyncio.sleep(30 if self.registered_id else 5)


def application(adapter):
    app = web.Application(client_max_size=16384)
    app.router.add_post('/mcp', adapter.rpc)
    app.router.add_get('/health', adapter.health)
    return app


async def pump(bus, control):
    next_feedback = 0.
    while True:
        if bus.error:
            control.fault = 'dds_failed'
            await control.stop()
            raise RuntimeError(bus.error)
        raw = bus.take()
        if raw is not None:
            try:
                control.receive(decode(raw))
            except (ValueError, RecursionError):
                control.receive({})
        control.watchdog()
        if time.monotonic() >= next_feedback:
            bus.feedback(control.feedback())
            next_feedback = time.monotonic()+.2
        await asyncio.sleep(.005)


async def run():
    url, serial, scale = load_settings()
    clock_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    worker = bus = control = runner = None
    tasks = []
    loop = asyncio.get_running_loop()
    stopping = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopping.set)
    try:
        bus = TeleopBus()
        worker = await asyncio.to_thread(make_worker)
        robot = Robot(url, serial)
        control = TeleopControl(robot, worker, clock_id=clock_id, scale=scale)
        adapter = Adapter(control)
        runner = web.AppRunner(application(adapter), access_log=None)
        await runner.setup()
        await web.TCPSite(runner, '127.0.0.1', PORT).start()
        async with aiohttp.ClientSession(trust_env=False, timeout=aiohttp.ClientTimeout(total=5)) as session:
            tasks = [asyncio.create_task(robot.run(session)), asyncio.create_task(control.solve_loop()),
                     asyncio.create_task(control.send_loop()), asyncio.create_task(pump(bus, control)),
                     asyncio.create_task(adapter.register(session)), asyncio.create_task(stopping.wait())]
            try:
                done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                for task in done:
                    task.result()
            finally:
                control.closed = True
                await control.stop()
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
    finally:
        if runner:
            await runner.cleanup()
        if worker:
            await asyncio.to_thread(worker.close)
        if bus:
            bus.close()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(sig)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(levelname)s [tron2-teleop] %(message)s')
    asyncio.run(run())
