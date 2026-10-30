"""Synthetic VOP -> native MoveH arrival -> real session ACP HTTP callback."""
import asyncio
import json
import os
import time
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

import aiohttp
from aiohttp import web
from test_sensors import settings, info, joints
from device import Adapter
from state import joint_state
from head_follow.runtime import Runtime


class Bus:
    def publish(self, *args):
        pass

    def subscribe_json(self, topic, callback):
        self.callback = callback
        return topic

    def unsubscribe(self, sub):
        self.callback = None


async def exercise():
    bus = Bus()
    pending, callbacks, sent, completed = set(), [], [], []
    settled = asyncio.Event()

    async def callback(request):
        data = await request.json()
        assert data['action_id'] in pending, data
        assert 'Authorization' not in request.headers
        callbacks.append(data)
        if data['tool'] == 'head_visual_follow':
            settled.set()
        return web.json_response({'ok': True, 'action_id': data['action_id']})

    app = web.Application()
    app.router.add_post('/api/acp/complete', callback)
    acp = web.AppRunner(app)
    await acp.setup()
    site = web.TCPSite(acp, '127.0.0.1', 0)
    await site.start()
    core_url = 'http://127.0.0.1:%s' % site._server.sockets[0].getsockname()[1]
    try:
        async with aiohttp.ClientSession(trust_env=False) as session:
            adapter = Adapter(bus.publish, settings(enabled_cards=[
                'joint_state', 'camera', 'move_head', 'head_visual_follow']), session=session)
            reader, q = adapter.reader, [0.] * 16

            def feedback():
                reader.accept_info(info())
                reader.joints = joint_state(joints(q=q[:], dq=[0.] * 16, timestamp=time.monotonic()))
                reader.joints_at, reader.joints_received = time.monotonic(), time.time()

            async def wire(data):
                assert data['title'] == 'request_moveh'
                pending.add(adapter.head.active['action_id'])
                sent.append(data)
                q[14:] = data['data']['joint']
                feedback()
                reader.head_pending[1].set_result({'result': 'success'})

            reader.ws = SimpleNamespace(closed=False, send_json=wire)
            reader.head_control_status = AsyncMock(return_value={'vr_head_follow': False})
            reader.send_movej, reader.send_gripper = AsyncMock(), AsyncMock()
            feedback()
            runtime = Runtime(adapter, bus)
            adapter.follow_runtime = runtime
            mcp_app = web.Application()
            mcp_app.router.add_post('/mcp', adapter.rpc)
            mcp = web.AppRunner(mcp_app)
            await mcp.setup()
            rpc_site = web.TCPSite(mcp, '127.0.0.1', 0)
            await rpc_site.start()
            rpc_url = 'http://127.0.0.1:%s/mcp' % rpc_site._server.sockets[0].getsockname()[1]

            def detection(people):
                bus.callback(json.dumps({'timestamp': time.time(), 'latency_ms': 0, 'count': len(people),
                    'objects': [{'name': 'person', 'confidence': .95, 'position': xy} for xy in people]}))

            try:
                with patch.dict(os.environ, AGENT_CORE_URL=core_url):
                    await runtime.call({'action': 'start'})
                    detection([[.8, 0.]])
                    # A real MCP Execute with no action selector returns one session id.
                    async with session.post(rpc_url, json={'jsonrpc': '2.0', 'id': 1,
                            'method': 'tools/call', 'params': {'name': 'head_visual_follow', 'arguments': {}}}) as reply:
                        body = await reply.json()
                    assert not body['result']['isError'], body
                    accepted = json.loads(body['result']['content'][0]['text'])
                    pending.add(accepted['action_id'])
                    assert accepted['completion_pending'] and not callbacks
                    runtime.launch()
                    started, stage = time.monotonic(), 'follow'
                    while time.monotonic() - started < 18:
                        feedback()
                        state = await asyncio.to_thread(runtime.info)
                        assert state['state'] != 'fault', state
                        if stage == 'follow' and len(sent) == 1 and state['head_motion'] is None:
                            completed.append(adapter.head.summary())
                            stage = 'home'
                        elif stage == 'home' and len(sent) == 2 and any(e['event'] == 'home_verified' for e in state['events']):
                            completed.append(adapter.head.summary())
                            break
                        detection([[.8, 0.]] if stage == 'follow' else [])
                        await asyncio.sleep(.075)
                    assert len(sent) == 2 and len(completed) == 2, (sent, state)
                    assert all(r['status'] == 'completed' and r['arrival_verified'] and r['callback_delivered']
                        and r['verification'] == 'post_duration_advancing_head_feedback'
                        and r['verified_samples'] >= 3 for r in completed), completed
                    assert sent[0]['data']['joint'][1] < 0 and sent[1]['data']['joint'] == [0., 0.]
                    assert not settled.is_set(), 'Home must not settle the continuous session'
                    assert runtime.info()['session']['action_id'] == accepted['action_id']
                    assert runtime.info()['session']['state'] == 'running'
                    await runtime.call({'action': 'stop'})
                    await asyncio.wait_for(settled.wait(), 2)
                    await runtime.session_task
                    terminal = [r for r in callbacks if r['tool'] == 'head_visual_follow']
                    assert len(terminal) == 1 and terminal[0]['action_id'] == accepted['action_id'], terminal
                    assert terminal[0]['status'] == 'cancelled'
                    assert not terminal[0]['result']['motion_may_continue']
                    assert runtime.info()['session']['callback_delivered']
                    assert reader.send_movej.await_count == reader.send_gripper.await_count == 0
                    return {'head_sends': len(sent), 'verified_arrivals': len(completed),
                            'arm_or_gripper_sends': 0, 'session_callbacks': len(terminal)}
            finally:
                await runtime.close()
                await adapter.head.close()
                await adapter.motion.close()
                await mcp.cleanup()
    finally:
        await acp.cleanup()


class NativeFollowEndToEndTests(unittest.IsolatedAsyncioTestCase):
    async def test_vop_follow_home_and_session_acp_over_real_http(self):
        self.assertEqual(await exercise(), {'head_sends': 2, 'verified_arrivals': 2,
                                          'arm_or_gripper_sends': 0, 'session_callbacks': 1})


if __name__ == '__main__':
    unittest.main()
