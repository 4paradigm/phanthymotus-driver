#!/usr/bin/env python3
"""LimX TRON2 read-only sensors; no robot control interface."""
from pathlib import Path
import sys


def common_root(entry):
    directory = Path(entry).resolve().parent
    for candidate in (directory, *directory.parents):
        if (candidate / 'common' / 'logsafe.py').is_file():
            return candidate
    raise ImportError('Missing common/logsafe.py in image or repository')


sys.path.insert(0, str(common_root(__file__)))
from common import logsafe
logsafe.install()

import asyncio
import logging
import math
import os
import re
import signal
import ssl
from urllib.parse import urlparse

import aiohttp
from aiohttp import web
import yaml


def load_settings():
    path = Path(os.environ.get('CONFIG_PATH', Path(__file__).with_name('config.yaml')))
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict) or config.get('mcp_port') != 15791:
        raise ValueError('Expected MCP port 15791')
    robot = config.get('robot', {})
    if not isinstance(robot, dict):
        raise ValueError('Expected robot configuration')
    serial = os.environ.get('TRON2_SERIAL') or robot.get('serial', '')
    if not isinstance(serial, str) or (serial and not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', serial)):
        raise ValueError('Invalid TRON2_SERIAL')
    url = os.environ.get('TRON2_ROBOT_URL') or robot.get('url', '')
    if not isinstance(url, str):
        raise ValueError('Expected vendor WebSocket URL')
    parsed = urlparse(url)
    if parsed.scheme not in ('ws', 'wss') or not parsed.hostname or parsed.port != 5000 \
            or parsed.username or parsed.password or parsed.query or parsed.fragment \
            or parsed.path not in ('', '/'):
        raise ValueError('Configure vendor WebSocket on port 5000')
    settings = {'serial': serial, 'url': url}
    for name, default, low, high in (
            ('poll_interval_seconds', .5, .1, 5),
            ('response_timeout_seconds', 5, 1, 10),
            ('stale_after_seconds', 5, 1, 30)):
        value = robot.get(name, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value) or not low <= value <= high:
            raise ValueError('Invalid ' + name)
        settings[name] = value
    if settings['stale_after_seconds'] < settings['poll_interval_seconds']:
        raise ValueError('Stale threshold must cover the poll interval')
    plugins = config.get('plugins', {})
    if not isinstance(plugins, dict) or set(plugins) - {'joint_state', 'robot_status', 'battery_state'}:
        raise ValueError('Only the three sensor plugins may be configured')
    enabled = []
    for name, value in plugins.items():
        if not isinstance(value, dict) or type(value.get('enabled')) is not bool:
            raise ValueError('Expected boolean enabled for ' + name)
        if value['enabled']:
            enabled.append(name)
    if not enabled:
        raise ValueError('Enable at least one read-only sensor')
    settings['enabled_cards'] = enabled
    return settings


async def run():
    from device import Adapter, TOPICS, PORT, LOG
    from ros2 import JsonPublisher
    settings = load_settings()
    publisher = JsonPublisher([TOPICS[name] for name in settings['enabled_cards']])
    tasks = []
    runner = None
    loop = asyncio.get_running_loop()
    stopping = asyncio.Event()
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(signum, stopping.set)
        async with aiohttp.ClientSession(trust_env=False,
                timeout=aiohttp.ClientTimeout(total=8)) as session:
            adapter = Adapter(publisher.publish, settings)
            ca = os.environ.get('AGENT_CORE_CA_CERT')
            if ca:
                adapter.core_ssl = ssl.create_default_context(cafile=ca)
            app = web.Application(client_max_size=16 * 1024)
            app.router.add_post('/mcp', adapter.rpc)
            app.router.add_get('/health', adapter.health)
            runner = web.AppRunner(app, access_log=None)
            await runner.setup()
            await web.TCPSite(runner, '127.0.0.1', PORT).start()
            LOG.info('Read-only MCP on loopback:%s; sensor poll interval %ss',
                     PORT, settings['poll_interval_seconds'])
            tasks = [asyncio.create_task(adapter.reader.run(session)),
                     asyncio.create_task(adapter.publish_loop()),
                     asyncio.create_task(adapter.register_loop(session)),
                     asyncio.create_task(stopping.wait())]
            try:
                done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                for task in done:
                    task.result()
            finally:
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
    finally:
        if runner:
            await runner.cleanup()
        publisher.close()
        for signum in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(signum)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(levelname)s [tron2] %(message)s')
    asyncio.run(run())
