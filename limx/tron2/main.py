#!/usr/bin/env python3
"""LimX TRON2 fixed-pose reset MCP server. Startup never moves the robot."""
from pathlib import Path
import sys

# common/ is staged beside this file in images; use the repo root in checkouts.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from common import logsafe
logsafe.install()

import asyncio
import contextlib
import logging
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
    if not isinstance(config, dict) or config.get('mcp_port') != 15791 \
            or config.get('plugins', {}).get('reset_pose', {}).get('enabled') is not True:
        raise ValueError('Expected port 15791 and enabled reset_pose plugin')
    robot = config.get('robot', {})
    serial = os.environ.get('TRON2_SERIAL') or robot.get('serial', '')
    if not isinstance(serial, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', serial):
        raise ValueError('Configure TRON2_SERIAL for this robot before startup')
    url = os.environ.get('TRON2_ROBOT_URL') or robot.get('url', '')
    parsed = urlparse(url)
    if parsed.scheme not in ('ws', 'wss') or not parsed.hostname or parsed.port != 5000 \
            or parsed.username or parsed.password or parsed.query or parsed.fragment \
            or parsed.path not in ('', '/'):
        raise ValueError('Configure the vendor WebSocket endpoint on port 5000')
    os.environ['TRON2_SERIAL'] = serial
    os.environ['TRON2_ROBOT_URL'] = url
    return config


async def run():
    load_settings()  # Bind identity once, before importing the robot contracts.
    from device import Adapter, PORT, LOG
    async with aiohttp.ClientSession(trust_env=False,
                                    timeout=aiohttp.ClientTimeout(total=8)) as session:
        adapter = Adapter(session)
        ca = os.environ.get('AGENT_CORE_CA_CERT')
        if ca:
            adapter.core_ssl = ssl.create_default_context(cafile=ca)
        app = web.Application(client_max_size=16 * 1024)
        app.router.add_post('/mcp', adapter.rpc)
        app.router.add_get('/health', adapter.health)
        app.router.add_post('/operator/{operation}', adapter.operator)
        runner = web.AppRunner(app, access_log=None)
        await runner.setup()
        task = None
        stopping = asyncio.Event()
        loop = asyncio.get_running_loop()
        for signum in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(signum, stopping.set)
        try:
            await web.TCPSite(runner, '127.0.0.1', PORT).start()
            LOG.info('MCP on loopback:%s; fixed target; disabled until platform start', PORT)
            # An unconfigured pose does not prevent status queries or tool discovery.
            task = asyncio.create_task(adapter.register())
            await stopping.wait()
        finally:
            adapter.stopping = True
            adapter.platform_active = False
            adapter.lifecycle_epoch += 1
            if task:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await task
            if adapter.active_task and not adapter.active_task.done():
                # Keep the feedback monitor alive during graceful service shutdown.
                with contextlib.suppress(Exception):
                    await asyncio.shield(adapter.active_task)
            await runner.cleanup()
            for signum in (signal.SIGINT, signal.SIGTERM):
                loop.remove_signal_handler(signum)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(levelname)s [tron2-reset] %(message)s')
    asyncio.run(run())
