"""Real aiohttp transport on loopback; simulated robot, never physical motion."""
import asyncio
import json
from pathlib import Path
import tempfile
import unittest

import aiohttp
from aiohttp import web
import pose
from device import Adapter
from test_control import FakeRobot, snapshot


class HTTPTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        pose.SERIAL = 'TEST_TRON2A'
        home = root / 'home.json'
        pose.save_pose(home, pose.make_pose([snapshot(100), snapshot(102), snapshot(104)]))
        self.adapter = Adapter(None, home, root / 'config.json', root / 'latch', FakeRobot)
        self.adapter.config_path.write_text(json.dumps({
            'commissioned': True, 'high_level_confirmed_by_operator': True,
            'pose_sha256': self.adapter.load_pose()[1],
            'expected_working_mode': 'test-confirmed-mode',
            'teach_state_pointer': '/active', 'teach_inactive_value': False}))
        FakeRobot.state, FakeRobot.teach, FakeRobot.calls = snapshot(), {'active': False}, []
        FakeRobot.uncertain, FakeRobot.started, FakeRobot.finish = False, None, None
        app = web.Application(client_max_size=16 * 1024)
        app.router.add_post('/mcp', self.adapter.rpc)
        app.router.add_post('/operator/{operation}', self.adapter.operator)
        app.router.add_get('/health', self.adapter.health)
        self.runner = web.AppRunner(app, access_log=None)
        await self.runner.setup()
        server = await asyncio.get_running_loop().create_server(
            self.runner.server, '127.0.0.1', 0)
        self.server = server
        self.url = 'http://127.0.0.1:' + str(server.sockets[0].getsockname()[1])
        self.session = aiohttp.ClientSession(trust_env=False)

    async def asyncTearDown(self):
        await self.session.close()
        self.server.close()
        await self.server.wait_closed()
        await self.runner.cleanup()
        self.tmp.cleanup()

    async def call(self, action, **args):
        payload = {'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call',
                   'params': {'name': 'reset_pose', 'arguments': {
                       'action': action, 'instance_id': 'actual-canvas-id', **args}}}
        async with self.session.post(self.url + '/mcp', json=payload) as resp:
            self.assertEqual(resp.status, 200)
            reply = await resp.json()
        result = reply['result']
        return result['isError'], json.loads(result['content'][0]['text'])

    async def test_real_http_canvas_start_info_preview_and_stop_are_read_only(self):
        error, data = await self.call('start')
        self.assertFalse(error)
        self.assertEqual(data['state'], 'ready')
        for action in ('info', 'preview', 'verify_at_home', 'status', 'stop'):
            error, data = await self.call(action)
            self.assertFalse(error, data)
            self.assertFalse(data['motion_sent'])
        self.assertNotIn('move', FakeRobot.calls)

    async def test_platform_enabled_execute_needs_no_operator_arm(self):
        error, result = await self.call('execute')
        self.assertTrue(error)
        self.assertFalse(result['motion_sent'])
        error, _ = await self.call('start')
        self.assertFalse(error)
        error, result = await self.call('execute', target_q=[0]*14)
        self.assertTrue(error)
        self.assertFalse(result['motion_sent'])
        for _ in range(2):
            error, result = await self.call('execute')
            self.assertFalse(error, result)
            self.assertEqual(result['status'], 'arrived')
        self.assertEqual(FakeRobot.calls.count('move'), 2)
        await self.call('stop')
        error, result = await self.call('execute')
        self.assertTrue(error)
        self.assertFalse(result['motion_sent'])
        self.assertEqual(FakeRobot.calls.count('move'), 2)

    async def test_removed_arm_route_cannot_override_platform_and_restart_is_disabled(self):
        async with self.session.post(self.url + '/operator/arm', json={
                'confirm_high_level': True, 'confirm_clear_path': True}) as resp:
            self.assertEqual(resp.status, 409)
        self.assertFalse(self.adapter.platform_active)
        self.adapter.stopping = True
        error, _ = await self.call('start')
        self.assertTrue(error)
        self.assertNotIn('move', FakeRobot.calls)
        replacement = Adapter(None, self.adapter.path, self.adapter.config_path,
                              self.adapter.latch_path, FakeRobot)
        self.assertFalse(replacement.platform_active)

    async def test_mcp_handshake_and_tool_discovery(self):
        async with self.session.post(self.url + '/mcp', json={
                'jsonrpc': '2.0', 'id': 1, 'method': 'initialize'}) as resp:
            self.assertEqual((await resp.json())['result']['serverInfo']['version'], '0.3.0')
        async with self.session.post(self.url + '/mcp', json={
                'jsonrpc': '2.0', 'id': 2, 'method': 'tools/list'}) as resp:
            tools = (await resp.json())['result']['tools']
        self.assertEqual(tools[0]['type'], 'actuator')
        self.assertIn('execute', tools[0]['inputSchema']['properties']['action']['enum'])


if __name__ == '__main__':
    unittest.main()
