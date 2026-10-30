"""Synthetic MJPEG lifecycle and binary ROS publication; no private camera images."""
import asyncio
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import MagicMock, patch

import aiohttp
from aiohttp import web

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import camera
from camera import CameraStream, TOPIC, CAMERA_SOURCES, jpeg_size
from device import Adapter
import ros2

# Header/framing fixture, intentionally not an entropy-decoding test image.
JPEG = (b'\xff\xd8\xff\xc0\x00\x0b\x08\x01\xe0\x02\x80\x01\x01\x11\x00'
        b'\xff\xda\x00\x08\x01\x01\x00\x00\x3f\x00\x01\x02\xff\xd9')
PART = b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(JPEG)).encode()+b'\r\n\r\n'+JPEG+b'\r\n'


class JPEGTests(unittest.TestCase):
    def test_bounded_header_dimensions(self):
        self.assertEqual(jpeg_size(JPEG), (640,480))
        for data in (JPEG[:-1], b'not jpeg', JPEG[:8], JPEG.replace(b'\x02\x80',b'\x20\x00'),
                     b'\xff\xd8\xff\xda\x00\x08\x01\x01\x00\x00\x3f\x00\x01\xff\xd9'):
            with self.subTest(data=data), self.assertRaises(ValueError): jpeg_size(data)
        with patch.object(camera,'MAX_FRAME',len(JPEG)-1), self.assertRaises(ValueError): jpeg_size(JPEG)


class CameraTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.requests = 0
        self.closed = 0
        self.mode = 'valid'
        self.source_modes = {}
        self.published = []
        self.session = aiohttp.ClientSession(trust_env=False)
        app = web.Application()
        app.router.add_get('/stream', self.stream)
        self.runner = web.AppRunner(app)
        await self.runner.setup()
        site = web.TCPSite(self.runner,'127.0.0.1',0)
        await site.start()
        port = site._server.sockets[0].getsockname()[1]
        self.url = f'http://127.0.0.1:{port}/stream'
        self.camera = CameraStream(self.url,lambda topic,data:self.published.append((topic,data)),self.session)
        self.timing = patch.multiple(camera, FRAME_TIMEOUT=.12, START_TIMEOUT=.35)
        self.timing.start()

    async def asyncTearDown(self):
        await self.camera.close()
        await self.session.close()
        await self.runner.cleanup()
        self.timing.stop()

    async def stream(self, request):
        self.requests += 1
        mode = self.source_modes.get(request.query.get('source'), self.mode)
        if mode == 'status': return web.Response(status=503)
        if mode == 'redirect': return web.Response(status=302,headers={'Location':self.url})
        if mode == 'mime': return web.Response(text='not video')
        headers = {'Content-Type':'multipart/x-mixed-replace; boundary=frame'}
        if mode == 'compressed': headers['Content-Encoding']='gzip'
        response = web.StreamResponse(headers=headers)
        await response.prepare(request)
        try:
            if mode == 'stall':
                await response.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n')
                await asyncio.sleep(.25)
            elif mode == 'truncated':
                await response.write(PART[:-5])
                await response.write_eof()
            elif mode == 'badframe':
                await response.write(PART.replace(JPEG,b'x'*len(JPEG)))
                await response.write(b'--frame--\r\n')
            elif mode == 'partmime':
                await response.write(PART.replace(b'image/jpeg',b'application/json'))
                await response.write(b'--frame--\r\n')
            else:
                for _ in range(100):
                    await response.write(PART)
                    await asyncio.sleep(.01)
        except (ConnectionError, RuntimeError): pass
        finally: self.closed += 1
        return response

    async def wait_for(self, predicate, timeout=1):
        deadline=asyncio.get_running_loop().time()+timeout
        while not predicate():
            if asyncio.get_running_loop().time()>deadline: self.fail('Condition timed out')
            await asyncio.sleep(.01)

    async def test_live_frames_unchanged_throttled_and_lifecycle_releases_socket(self):
        info=await self.camera.call('start')
        self.assertTrue(info['available'])
        task=self.camera.task
        await self.camera.call('start')
        self.assertIs(task,self.camera.task)
        await asyncio.sleep(.23)
        self.assertEqual(self.requests,1)
        self.assertGreater(self.camera.frames_received,self.camera.frames_published)
        self.assertGreaterEqual(len(self.published),3)
        self.assertLessEqual(len(self.published),4)
        self.assertTrue(all(t==TOPIC and d==JPEG for t,d in self.published))
        self.assertEqual((info['width'],info['height']),(640,480))
        stopped=await self.camera.call('stop')
        self.assertFalse(stopped['active']); self.assertFalse(stopped['available'])
        self.assertIsNone(stopped['width']); self.assertIsNone(self.camera.task)
        await self.wait_for(lambda:self.closed==1)
        count=len(self.published)
        await asyncio.sleep(.13)
        self.assertEqual(len(self.published),count)
        await self.camera.call('start')
        self.assertEqual(self.requests,2)

    async def test_stop_aborts_waiting_start_promptly(self):
        self.mode='stall'
        starting=asyncio.create_task(self.camera.call('start'))
        await self.wait_for(lambda:self.requests==1)
        await self.camera.call('stop')
        with self.assertRaisesRegex(ValueError,'stopped'):
            await asyncio.wait_for(starting,.1)
        self.assertIsNone(self.camera.task)

    async def test_invalid_responses_and_stalled_frame_never_publish(self):
        for mode in ('status','redirect','mime','compressed','badframe','partmime','truncated','stall'):
            self.mode=mode
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError,'did not provide'):
                await self.camera.call('start')
            self.assertEqual(self.published,[])
            self.assertFalse(self.camera.info()['active'])

    async def test_stream_loss_clears_dimensions_and_reconnects(self):
        await self.camera.call('start')
        self.mode='badframe'
        # First finite valid connection closes after ~1s; next one is invalid.
        await self.wait_for(lambda:self.camera.error is not None,timeout=1.5)
        lost=self.camera.info()
        self.assertFalse(lost['available']); self.assertIsNone(lost['width'])
        self.mode='valid'
        await self.wait_for(lambda:self.camera.info()['available'],timeout=1.5)
        self.assertGreater(self.requests,1)

    async def test_frame_size_limit_and_whole_frame_timeout(self):
        with patch.object(camera,'MAX_FRAME',len(JPEG)-1):
            with self.assertRaises(ValueError): await self.camera.call('start')
        self.assertEqual(self.published,[])

    async def test_camera_only_health_and_idle_video_do_not_block_other_sensors(self):
        settings={'enabled_cards':['camera'], 'camera_url':self.url, 'url':'ws://127.0.0.1:5000',
                  'serial':'','poll_interval_seconds':.5,'response_timeout_seconds':5,'stale_after_seconds':5}
        adapter=Adapter(lambda *args:None,settings,session=self.session)
        self.camera=adapter.camera
        self.assertEqual((await adapter.health(None)).status,503)
        await self.camera.call('start')
        self.assertEqual((await adapter.health(None)).status,200)
        await self.camera.call('stop')
        adapter.reader.connected=True
        adapter.snapshot=MagicMock(return_value={'joint_state':{'available':True}})
        self.assertEqual((await adapter.health(None)).status,200)
        adapter.snapshot.return_value={'joint_state':{'available':False}}
        self.assertEqual((await adapter.health(None)).status,503)

    async def test_wrist_streams_publish_separate_topics_and_stop_independently(self):
        settings={'enabled_cards':list(CAMERA_SOURCES),
                  'camera_urls':{name:self.url+'?source='+name for name in CAMERA_SOURCES},
                  'url':'ws://127.0.0.1:5000','serial':'','poll_interval_seconds':.5,
                  'response_timeout_seconds':5,'stale_after_seconds':5}
        adapter=Adapter(lambda topic,data:self.published.append((topic,data)),settings,session=self.session)
        self.camera=adapter.camera
        self.assertEqual({tool['name'] for tool in adapter.tools()},set(CAMERA_SOURCES))
        try:
            replies=await asyncio.gather(adapter.cameras['camera_left'].call('start'),adapter.cameras['camera_right'].call('start'))
            self.assertTrue(all(info['available'] for info in replies))
            self.assertEqual({info['topic_out'][0]['topic'] for info in replies},
                             {CAMERA_SOURCES[name][0] for name in ('camera_left','camera_right')})
            self.assertFalse(adapter.camera.info()['active'])
            await adapter.cameras['camera_left'].call('stop')
            left=adapter.cameras['camera_left'].frames_published
            right=adapter.cameras['camera_right'].frames_published
            await asyncio.sleep(.14)
            self.assertEqual(adapter.cameras['camera_left'].frames_published,left)
            self.assertGreater(adapter.cameras['camera_right'].frames_published,right)
            self.assertNotIn(TOPIC,{topic for topic,data in self.published})
            for name in ('camera_left','camera_right'):
                with self.assertRaises(ValueError):adapter.call(name,{'url':'http://wrong-host/'})
        finally:await asyncio.gather(*(stream.close() for stream in adapter.cameras.values()))

    async def test_one_failed_camera_does_not_stop_other_camera_or_json_feedback(self):
        settings={'enabled_cards':['camera_left','camera_right'],
                  'camera_urls':{name:self.url+'?source='+name for name in CAMERA_SOURCES},
                  'url':'ws://127.0.0.1:5000','serial':'','poll_interval_seconds':.5,
                  'response_timeout_seconds':5,'stale_after_seconds':5}
        adapter=Adapter(lambda *args:None,settings,session=self.session)
        self.source_modes['camera_right']='badframe'
        try:
            await adapter.cameras['camera_left'].call('start')
            before=adapter.cameras['camera_left'].frames_published
            with self.assertRaises(ValueError):await adapter.cameras['camera_right'].call('start')
            self.assertGreater(adapter.cameras['camera_left'].frames_published,before)
            self.assertTrue(adapter.cameras['camera_left'].info()['available'])
            self.assertFalse(adapter.cameras['camera_right'].info()['available'])
            self.assertEqual((await adapter.health(None)).status,503)
            adapter.reader.connected=True
            adapter.snapshot=MagicMock(return_value={'joint_state':{'available':True}})
            self.assertEqual((await adapter.health(None)).status,200)
        finally:await asyncio.gather(*(stream.close() for stream in adapter.cameras.values()))

    async def test_rpc_registration_info_and_rejection_never_issue_motion(self):
        settings={'enabled_cards':['camera'],'camera_url':self.url, 'url':'ws://127.0.0.1:5000',
                  'serial':'', 'poll_interval_seconds':.5,'response_timeout_seconds':5,'stale_after_seconds':5}
        adapter=Adapter(lambda topic,data:self.published.append((topic,data)),settings,session=self.session)
        self.camera=adapter.camera
        self.assertEqual(adapter.snapshot(),{})
        tool=adapter.tools()[0]
        self.assertEqual(tool['type'],'sensor')
        self.assertEqual(tool['topic_out'],[{'topic':TOPIC,'format':'image/jpeg'}])
        self.assertFalse(adapter.call('camera',{})['active'])
        self.assertEqual(self.requests,0)
        for args in ({'url':self.url}, {'topic':'control'}, {'action':'move'}, {'_trace_id':'x\n'}):
            with self.assertRaises(ValueError): adapter.call('camera',args)
        app=web.Application(); app.router.add_post('/mcp',adapter.rpc)
        runner=web.AppRunner(app); await runner.setup()
        site=web.TCPSite(runner,'127.0.0.1',0); await site.start()
        url='http://127.0.0.1:'+str(site._server.sockets[0].getsockname()[1])+'/mcp'
        adapter.reader.execute_movej=MagicMock(side_effect=AssertionError('Must not move'))
        try:
            for action in ('start','info','stop'):
                async with self.session.post(url,json={'jsonrpc':'2.0','id':1,'method':'tools/call',
                        'params':{'name':'camera','arguments':{'action':action,'instance_id':'private-id','_trace_id':'private-trace'}}}) as resp:
                    reply=await resp.json()
                self.assertFalse(reply['result'].get('isError',False))
                text=reply['result']['content'][0]['text']
                self.assertNotIn('private-',text); self.assertNotIn(self.url,text)
                data=json.loads(text)
                self.assertTrue(data['read_only'])
                self.assertEqual(data['available'],action!='stop')
            adapter.reader.execute_movej.assert_not_called()
        finally: await runner.cleanup()


class ROSImageTests(unittest.TestCase):
    def test_compressed_jpeg_binary_qos_and_stamp(self):
        rclpy=types.ModuleType('rclpy')
        rclpy.init=MagicMock(); rclpy.ok=MagicMock(return_value=True); rclpy.shutdown=MagicMock()
        node=MagicMock(); sinks={source[0]:MagicMock() for source in CAMERA_SOURCES.values()}
        sinks['/limx_tron2/yolo/annotated']=MagicMock()
        node.create_publisher.side_effect=lambda kind,topic,qos:sinks[topic]
        node.get_clock().now().to_msg.side_effect=lambda:types.SimpleNamespace(sec=10,nanosec=20)
        rclpy.create_node=MagicMock(return_value=node)
        qos=types.ModuleType('rclpy.qos'); qos.QoSProfile=MagicMock()
        qos.ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='best_effort')
        qos.HistoryPolicy=types.SimpleNamespace(KEEP_LAST='keep_last')
        qos.DurabilityPolicy=types.SimpleNamespace(VOLATILE='volatile')
        images=types.ModuleType('sensor_msgs.msg')
        class CompressedImage:
            def __init__(self): self.header=types.SimpleNamespace()
        images.CompressedImage=CompressedImage
        strings=types.ModuleType('std_msgs.msg'); strings.String=type('String',(),{})
        modules={'rclpy':rclpy,'rclpy.qos':qos,'std_msgs':types.ModuleType('std_msgs'),
                 'std_msgs.msg':strings,'sensor_msgs':types.ModuleType('sensor_msgs'),'sensor_msgs.msg':images}
        with tempfile.TemporaryDirectory() as temp:
            profile=Path(temp)/'dds.xml'; profile.write_text('<profiles/>')
            with patch.dict(sys.modules,modules),patch.dict(os.environ,{
                    'FASTRTPS_DEFAULT_PROFILES_FILE':str(profile),'ROS_DOMAIN_ID':'42','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp'}):
                publisher=ros2.JsonPublisher([],image_topics=list(sinks))
                for topic in sinks:
                    stamp=publisher.publish(topic,JPEG)
                    self.assertEqual(stamp,{'sec':10,'nanosec':20,'frame_id':ros2.IMAGE_FRAMES[topic]})
                    message=sinks[topic].publish.call_args[0][0]
                    self.assertEqual(message.data,JPEG); self.assertEqual(message.format,'jpeg')
                    self.assertEqual((message.header.stamp.sec,message.header.stamp.nanosec),(10,20))
                    self.assertEqual(message.header.frame_id,ros2.IMAGE_FRAMES[topic])
                    sinks[topic].publish.assert_called_once()
                source_stamp={'sec':101,'nanosec':202,'frame_id':'tron2_head_camera'}
                publisher.publish('/limx_tron2/yolo/annotated',JPEG,stamp=source_stamp)
                annotated=sinks['/limx_tron2/yolo/annotated'].publish.call_args.args[0]
                self.assertEqual((annotated.header.stamp.sec,annotated.header.stamp.nanosec),(101,202))
                self.assertEqual(annotated.header.frame_id,'tron2_head_camera')
                self.assertEqual(qos.QoSProfile.call_args.kwargs['depth'],1)
                publisher.close(); node.destroy_node.assert_called_once(); rclpy.shutdown.assert_called_once()
