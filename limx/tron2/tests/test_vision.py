"""Single-driver integration: fake GPU/vendor, real loopback MCP, no hardware."""
import asyncio
import copy
import hashlib
import json
import math
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock

import aiohttp
from aiohttp import web
import yaml

from test_sensors import settings, info, joints
from test_yolo_detect import Model, jpeg
from device import Adapter, VERSION
from state import joint_state
from vision import Runtime, ImageBus, validate_enabled
from yolo_detect.engine import IMAGE_TOPIC, RESULT_TOPIC
from head_follow.client import Error

DRIVER = Path(__file__).resolve().parents[1]
ENABLED = ['joint_state', 'camera', 'camera_left', 'camera_right', 'move_head',
           'yolo_detect', 'head_visual_follow']


class Publisher:
    def __init__(self):
        self.sent = []

    def publish(self, topic, value, stamp=None):
        self.sent.append((topic, value, copy.deepcopy(stamp)))


class IntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.publisher = Publisher()
        self.adapter = Adapter(self.publisher.publish, settings(enabled_cards=ENABLED))
        self.reader = self.adapter.reader
        self.reader.accept_info(info())
        self.reader.ws = SimpleNamespace(closed=False)
        self.feedback()
        self.model = Model()
        self.runtime = Runtime(self.adapter, self.publisher, model=self.model)
        self.adapter.vision = self.runtime
        self.adapter.head.dispatch = AsyncMock(return_value={'motion_sent': True, 'action_id': 'head-test'})
        self.adapter.motion.dispatch = AsyncMock()
        self.runner = None

    def feedback(self):
        self.reader.joints = joint_state(joints(q=[0.] * 16, timestamp=time.monotonic()))
        self.reader.joints_at = time.monotonic()
        self.reader.joints_received = time.time()

    async def asyncTearDown(self):
        await self.runtime.close()
        if self.runner:
            await self.runner.cleanup()

    async def http(self):
        app = web.Application(client_max_size=16 * 1024)
        app.router.add_post('/mcp', self.adapter.rpc)
        app.router.add_get('/health', self.adapter.health)
        self.runner = web.AppRunner(app)
        await self.runner.setup()
        site = web.TCPSite(self.runner, '127.0.0.1', 0)
        await site.start()
        return 'http://127.0.0.1:%d' % site._server.sockets[0].getsockname()[1]

    async def detected_frame(self):
        await self.adapter.execute_tool('yolo_detect', {'action': 'start'})
        stamp = {'sec': 10, 'nanosec': 22, 'frame_id': 'tron2_head_camera'}
        raw = jpeg()
        self.runtime.on_image('/limx_tron2/camera', raw, stamp)
        self.assertTrue(await asyncio.to_thread(self.runtime.yolo.step))
        return raw, stamp

    async def test_same_mcp_discovers_old_and_new_cards_without_starting(self):
        names = [t['name'] for t in self.adapter.tools()]
        self.assertEqual(len(names), len(set(names)))
        for name in ENABLED + ['model']:
            self.assertIn(name, names)
        follow = next(t for t in self.adapter.tools() if t['name'] == 'head_visual_follow')
        actions = follow['inputSchema']['properties']['action']['enum']
        self.assertEqual([a for a in actions if a not in ('start', 'stop', 'info')], ['follow'])
        self.assertFalse(self.runtime.follow.active)
        self.assertFalse(self.runtime.yolo.active)
        self.assertEqual(self.model.health_calls, 0)

    async def test_project_start_and_info_never_arm_follow_or_send_motion(self):
        self.runtime.launch()
        for action in ('start', 'info', 'stop'):
            result = await self.adapter.execute_tool('head_visual_follow', {'action': action})
            self.assertFalse(result['active'])
        await asyncio.sleep(.09)
        self.assertEqual(self.model.health_calls, 0)
        self.adapter.head.dispatch.assert_not_awaited()
        self.adapter.motion.dispatch.assert_not_awaited()

    async def test_shared_jpeg_and_stamp_preserved_without_another_subscription(self):
        raw, stamp = await self.detected_frame()
        result = self.runtime.yolo.info()['latest_result']
        self.assertEqual(self.model.images, [raw])
        self.assertEqual(result['source_header_stamp'], stamp)
        self.assertEqual(result['source_jpeg_sha256'], hashlib.sha256(raw).hexdigest())
        topic, annotated, output_stamp = self.publisher.sent[-2]
        self.assertEqual(topic, IMAGE_TOPIC)
        self.assertEqual(output_stamp, stamp)
        self.assertEqual(self.publisher.sent[-1][0], RESULT_TOPIC)
        self.assertEqual(result['annotated_jpeg_sha256'], hashlib.sha256(annotated).hexdigest())
        self.adapter.head.dispatch.assert_not_awaited()

    async def test_wrong_camera_ignored_and_stopped_generation_cannot_feed(self):
        await self.adapter.execute_tool('yolo_detect', {'action': 'start'})
        old = self.runtime.bus.binding[1]
        self.runtime.on_image('/limx_tron2/camera_left', jpeg(), {})
        self.assertIsNone(self.runtime.yolo.pending)
        await self.adapter.execute_tool('yolo_detect', {'action': 'stop'})
        old(jpeg(), {})
        self.assertIsNone(self.runtime.yolo.pending)
        self.assertIsNone(self.runtime.bus.binding)

    async def test_only_enabled_camera_can_be_selected(self):
        bus = ImageBus(self.publisher, ['camera'])
        with self.assertRaises(Error):
            bus.bind('/limx_tron2/camera_left', lambda *args: None)
        self.assertIsNone(bus.binding)

    async def test_follow_enable_uses_same_loop_feedback_without_deadlock(self):
        await self.detected_frame()
        self.feedback()
        result = await asyncio.wait_for(self.adapter.execute_tool(
            'head_visual_follow', {'action': 'follow'}), 2)
        self.assertTrue(result['active'])
        self.assertEqual(self.runtime.follow.arm_reference, [0.] * 14)
        self.adapter.head.dispatch.assert_not_awaited()

    async def test_follow_callback_uses_original_move_head_validation_and_owner(self):
        self.runtime.follow.active = True
        result = await asyncio.to_thread(self.runtime.follow.client.move, 2., -3., .8)
        self.assertEqual(result['action_id'], 'head-test')
        args = self.adapter.head.dispatch.await_args.args
        self.assertEqual(args[0], 'move')
        self.assertEqual(args[1], [-math.radians(2), math.radians(-3)])
        self.assertEqual(args[2], .8)
        self.adapter.motion.dispatch.assert_not_awaited()
        with self.assertRaises(Error):
            await asyncio.to_thread(self.runtime.follow.client.move, 60., 0., 1.)
        self.assertEqual(self.adapter.head.dispatch.await_count, 1)

    async def test_original_busy_or_readiness_rejection_is_not_bypassed(self):
        self.runtime.follow.active = True
        self.adapter.head.dispatch.side_effect = ValueError('head_busy')
        with self.assertRaisesRegex(ValueError, 'head_busy'):
            await asyncio.to_thread(self.runtime.follow.client.move, 2., 0., 1.)
        self.assertEqual(self.adapter.head.dispatch.await_count, 1)

    async def test_stop_gate_rejects_queued_move_before_ownership(self):
        self.runtime.follow.active = True
        self.runtime.follow.stop_requested.set()
        with self.assertRaises(Error):
            await asyncio.to_thread(self.runtime.follow.client.move, 2., 0., 1.)
        self.adapter.head.dispatch.assert_not_awaited()

    async def test_unknown_send_is_not_retried_or_converted_to_home(self):
        self.runtime.follow.active = True
        self.adapter.head.dispatch.side_effect = TimeoutError('unknown send')
        p = {'pitch': 0., 'yaw': 0.}
        with self.assertRaises(TimeoutError):
            await asyncio.to_thread(self.runtime.follow.dispatch, p, 2., 0., 'follow')
        self.assertTrue(self.runtime.follow.unknown_send)
        with self.assertRaises(Error):
            await self.adapter.execute_tool('head_visual_follow', {'action': 'follow'})
        self.assertEqual(self.adapter.head.dispatch.await_count, 1)

    async def test_same_loop_bridge_is_rejected_instead_of_blocking(self):
        with self.assertRaisesRegex(Error, '主控制循环'):
            self.runtime.robot()

    async def test_health_does_not_deadlock_with_worker_waiting_for_main_loop(self):
        acquired = threading.Event()
        def worker():
            with self.runtime.follow.control:
                acquired.set()
                return self.runtime.robot()
        task = asyncio.create_task(asyncio.to_thread(worker))
        await asyncio.to_thread(acquired.wait, 1)
        response = await asyncio.wait_for(self.adapter.health(None), 2)
        await asyncio.wait_for(task, 2)
        self.assertIn('vision', json.loads(response.body))

    async def test_model_health_failure_is_a_tool_error_not_http_500(self):
        def failed():
            raise OSError('GPU unavailable')
        self.model.health = failed
        url = await self.http()
        async with aiohttp.ClientSession() as session:
            async with session.post(url + '/mcp', json={'jsonrpc': '2.0', 'id': 1,
                    'method': 'tools/call', 'params': {'name': 'yolo_detect',
                    'arguments': {'action': 'start'}}}) as response:
                self.assertEqual(response.status, 200)
                self.assertTrue((await response.json())['result']['isError'])
        self.assertFalse(self.runtime.yolo.active)

    async def test_real_single_http_endpoint_serves_both_cards_and_reports_missing_detection(self):
        url = await self.http()
        async with aiohttp.ClientSession() as session:
            async def rpc(method, params=None):
                async with session.post(url + '/mcp', json={'jsonrpc': '2.0', 'id': 1,
                        'method': method, 'params': params or {}}) as response:
                    self.assertEqual(response.status, 200)
                    return await response.json()
            names = {t['name'] for t in (await rpc('tools/list'))['result']['tools']}
            self.assertTrue({'yolo_detect', 'head_visual_follow', 'move_head'} <= names)
            result = await rpc('tools/call', {'name': 'head_visual_follow', 'arguments': {'action': 'follow'}})
            self.assertTrue(result['result']['isError'])
            self.assertIn('新鲜人物检测', result['result']['content'][0]['text'])
            await self.detected_frame()
            self.feedback()
            result = await rpc('tools/call', {'name': 'head_visual_follow', 'arguments': {'action': 'follow'}})
            self.assertFalse(result['result']['isError'])
            async with session.get(url + '/health') as response:
                h = await response.json()
                self.assertEqual(h['version'], VERSION)
                self.assertTrue(h['vision']['head_visual_follow']['active'])
        self.adapter.head.dispatch.assert_not_awaited()

    async def test_live_worker_keeps_producing_on_shared_publisher_and_stops_cleanly(self):
        self.runtime.launch()
        await self.adapter.execute_tool('yolo_detect', {'action': 'start'})
        for sec in (10, 11):
            count = self.runtime.yolo.published
            self.runtime.on_image('/limx_tron2/camera', jpeg(), {'sec': sec, 'nanosec': 0, 'frame_id': 'head'})
            deadline = time.monotonic() + 2
            while self.runtime.yolo.published == count and time.monotonic() < deadline:
                await asyncio.sleep(.02)
            self.assertGreater(self.runtime.yolo.published, count)
        await self.runtime.close()
        self.assertFalse(self.runtime.yolo.thread.is_alive())
        self.assertFalse(self.runtime.follow.thread.is_alive())
        self.adapter.head.dispatch.assert_not_awaited()

    async def test_waiting_for_gpu_does_not_block_same_mcp_listener(self):
        entered, release = threading.Event(), threading.Event()
        def health():
            entered.set()
            if not release.wait(2):
                raise OSError('mock GPU wait timed out')
        self.model.health = health
        pending = asyncio.create_task(self.adapter.execute_tool('yolo_detect', {'action': 'start'}))
        try:
            self.assertTrue(await asyncio.to_thread(entered.wait, 1))
            url = await self.http()
            async with aiohttp.ClientSession() as session:
                async with session.post(url + '/mcp', json={'jsonrpc': '2.0', 'id': 1,
                        'method': 'tools/list'}) as response:
                    data = await asyncio.wait_for(response.json(), .5)
                    self.assertIn('result', data)
        finally:
            release.set()
            await pending

    async def test_existing_camera_mjpeg_relay_feeds_detector_without_second_connection(self):
        from camera import CameraStream
        requests = []
        done = asyncio.Event()
        raw = jpeg()
        part = b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + \
            str(len(raw)).encode() + b'\r\n\r\n' + raw + b'\r\n'
        async def stream(request):
            requests.append(request.path)
            response = web.StreamResponse(headers={'Content-Type':
                'multipart/x-mixed-replace; boundary=frame'})
            await response.prepare(request)
            try:
                while not done.is_set():
                    await response.write(part)
                    await asyncio.sleep(.02)
            except (ConnectionError, RuntimeError):
                pass
            return response
        app = web.Application()
        app.router.add_get('/stream', stream)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '127.0.0.1', 0)
        await site.start()
        url = 'http://127.0.0.1:%d/stream' % site._server.sockets[0].getsockname()[1]
        def relay(topic, image):
            now = time.time_ns()
            stamp = {'sec': now // 10**9, 'nanosec': now % 10**9, 'frame_id': 'head'}
            self.publisher.publish(topic, image, stamp=stamp)
            self.runtime.on_image(topic, image, stamp)
        self.runtime.launch()
        await self.adapter.execute_tool('yolo_detect', {'action': 'start'})
        async with aiohttp.ClientSession() as session:
            camera = CameraStream(url, relay, session)
            try:
                await camera.call('start')
                deadline = time.monotonic() + 2
                while not self.runtime.yolo.published and time.monotonic() < deadline:
                    await asyncio.sleep(.02)
                self.assertGreater(self.runtime.yolo.published, 0)
                result = self.runtime.yolo.info()['latest_result']
                self.assertEqual(result['source_jpeg_sha256'], hashlib.sha256(raw).hexdigest())
                self.assertEqual(len(requests), 1)
                self.assertTrue(camera.info()['available'])
            finally:
                done.set()
                await camera.close()
                await runner.cleanup()


class PackagingTests(unittest.TestCase):
    def test_only_parent_driver_manifest_and_container_remain(self):
        manifest = yaml.safe_load((DRIVER / 'driver.yaml').read_text())
        self.assertEqual(manifest['id'], 'limx-tron2-driver')
        self.assertEqual(manifest['port'], 15791)
        self.assertTrue({'head_visual_follow', 'yolo_detect'} <= {c['name'] for c in manifest['cards']})
        for folder in ('head_follow', 'yolo_detect'):
            for name in ('Dockerfile', 'driver.yaml', 'server.py'):
                self.assertFalse((DRIVER / folder / name).exists())
        self.assertEqual(set(yaml.safe_load((DRIVER / 'deploy/service.yml').read_text())), {'limx-tron2'})

    def test_original_docker_context_includes_engines_but_no_models_or_new_services(self):
        docker = (DRIVER / 'Dockerfile').read_text()
        self.assertIn('EXPOSE 15791', docker)
        self.assertNotIn('15792', docker)
        self.assertNotIn('15793', docker)
        self.assertIn('head_follow/engine.py', docker)
        self.assertIn('yolo_detect/engine.py', docker)
        self.assertNotIn('COPY tests', docker)
        self.assertNotIn('torch', (DRIVER / 'requirements.txt').read_text().lower())
        self.assertNotIn('ultralytics', (DRIVER / 'requirements.txt').read_text().lower())

    def test_follow_configuration_requires_existing_head_camera_and_control(self):
        validate_enabled(ENABLED)
        for name in ('joint_state', 'camera', 'move_head', 'yolo_detect'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                validate_enabled([x for x in ENABLED if x != name])
        validate_enabled(['joint_state'])

    def test_readonly_checker_accepts_new_cards_and_rejects_wrong_output(self):
        from check import validate_visual_tools
        adapter = Adapter(lambda *args: None, settings(enabled_cards=ENABLED))
        discovered = adapter.tools()
        validate_visual_tools(discovered)
        broken = copy.deepcopy(discovered)
        next(t for t in broken if t['name'] == 'yolo_detect')['topic_out'] = []
        with self.assertRaisesRegex(ValueError, 'YOLO'):
            validate_visual_tools(broken)
        with self.assertRaisesRegex(ValueError, 'Follow'):
            validate_visual_tools([t for t in discovered if t['name'] != 'yolo_detect'])


if __name__ == '__main__':
    unittest.main()
