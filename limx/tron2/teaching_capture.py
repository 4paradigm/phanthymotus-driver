"""Read-only episode capture on the existing vendor reader and MJPEG sources."""
import asyncio
import hashlib
import time

import aiohttp
from camera import CAMERA_SOURCES, FRAME_TIMEOUT, next_frame
from episode import CAMERAS, EpisodeWriter
from gripper import state_data
from skeleton import JOINT_NAMES, MODEL_PATH


class Capture:
    def __init__(self, reader, session, camera_urls, directory):
        self.reader, self.session, self.urls, self.directory = reader, session, camera_urls, directory
        self.writer = None
        self.tasks = []
        self.gripper = None
        self.last_source = None
        self.sequence = 0
        self.duplicate_samples = 0
        self.last_received = None
        self.lock = asyncio.Lock()

    def binding(self):
        info = self.reader.sample('robot_status')
        if not info['available'] or not info['data'].get('sw_version'):
            raise ValueError('fresh_device_firmware_required')
        return {'serial':self.reader.serial, 'model':'DACH_TRON2A',
            'model_sha256':hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest(),
            'firmware':info['data']['sw_version']}

    def info(self):
        writer = self.writer
        return dict(recording=bool(writer and writer.accepting), read_only=True, motion_sent=False,
            robot_mode_changed=False, episode_id=writer.ident if writer else None,
            frames=self.sequence, duplicate_source_samples=self.duplicate_samples,
            error=writer.error if writer else None,
            complete=bool(writer and writer.manifest['complete']),
            message='停止录制只停止写文件，不停止原厂 VR 或机器人')

    async def probe(self):
        async def camera(name, url):
            try:
                timeout=aiohttp.ClientTimeout(total=4)
                async with self.session.get(url,timeout=timeout,allow_redirects=False) as response:
                    if response.status!=200:
                        raise ValueError('camera_unavailable')
                    await next_frame(aiohttp.MultipartReader.from_response(response))
                return dict(id=name,label=name+' RGB',ok=True)
            except (OSError,ValueError,aiohttp.ClientError,asyncio.TimeoutError):
                return dict(id=name,label=name+' RGB',ok=False,reason='图像不可用；可继续录制诊断数据')
        checks=await asyncio.gather(*(camera(name,self.urls[key]) for name,key in zip(CAMERAS,CAMERA_SOURCES)))
        try:
            state_data(await self.reader.read_gripper())
            checks.append(dict(id='grippers',label='双夹爪实测开口',ok=True))
        except (ValueError,asyncio.TimeoutError):
            checks.append(dict(id='grippers',label='双夹爪实测开口',ok=False,reason='夹爪不可用；可继续录制诊断数据'))
        return checks

    async def start(self, task, outcome, notes):
        async with self.lock:
            if self.tasks or self.reader.teaching_observer is not None:
                raise ValueError('recording_already_active')
            binding = self.binding()
            if not self.reader.sample('joint_state')['available']:
                raise ValueError('joint_feedback_required')
            # A failed gripper read still permits diagnostic capture, explicitly incomplete.
            self.gripper = None
            try:
                raw = await self.reader.read_gripper()
                state_data(raw)
                self.gripper = (raw, time.monotonic_ns())
            except (ValueError, asyncio.TimeoutError):
                pass
            self.writer = EpisodeWriter(self.directory, binding, task, outcome, notes)
            self.last_source = None
            self.sequence = self.duplicate_samples = 0
            self.last_received = time.monotonic()
            self.reader.teaching_observer = self.observe
            if self.reader._joint_poll_wake is not None:
                self.reader._joint_poll_wake.set()
            self.tasks = [asyncio.create_task(self._grippers()), asyncio.create_task(self._watchdog())]
            for name, key in zip(CAMERAS, CAMERA_SOURCES):
                self.tasks.append(asyncio.create_task(self._camera(name, self.urls[key])))
            return self.info()

    def observe(self, data, received_ns):
        if not self.writer or not self.writer.accepting:
            return
        try:
            names = data['names'] or JOINT_NAMES
            if len(data['q']) != 16 or set(names) != set(JOINT_NAMES):
                raise ValueError('unsupported_joint_layout')
            order = [names.index(name) for name in JOINT_NAMES]
            source = data['robot_timestamp']
            if source is not None and self.last_source is not None:
                if source < self.last_source:
                    raise ValueError('source_clock_reversed')
                if source == self.last_source:
                    self.duplicate_samples += 1
                    return
            self.last_source = source
            self.last_received = time.monotonic()
            grip = self.gripper
            recent = grip is not None and 0 <= received_ns-grip[1] <= 100_000_000
            frame = dict(sequence=self.sequence, received_ns=received_ns, received_unix_ns=time.time_ns(),
                source_timestamp=source, source_clock='vendor_opaque_unverified', measurement_time_verified=False,
                q=[data['q'][i] for i in order],
                dq=[data['dq'][i] for i in order] if data['dq'] else [],
                tau=[data['tau'][i] for i in order] if data['tau'] else [],
                gripper=[grip[0]['left_opening'],grip[0]['right_opening']] if recent else None,
                gripper_received_ns=grip[1] if recent else None,
                gripper_source_timestamp=grip[0].get('timestamp') if recent else None,
                gripper_feedback=dict(grip[0]) if recent else None,
                vendor_target=None, vendor_target_available=False)
            if self.writer.put('state', frame):
                self.sequence += 1
        except (ValueError, KeyError, TypeError) as exc:
            self.writer.fail(str(exc))

    async def _grippers(self):
        previous = None
        while self.writer.accepting:
            started = time.monotonic()
            try:
                raw = await self.reader.read_gripper()
                state_data(raw)
                stamp = raw.get('timestamp')
                if stamp is not None and previous is not None and stamp < previous:
                    self.writer.fail('gripper_source_clock_reversed')
                    return
                if stamp is None or stamp != previous:
                    self.gripper = (raw, time.monotonic_ns())
                previous = stamp
            except (ValueError, asyncio.TimeoutError):
                self.gripper = None
            await asyncio.sleep(max(0, .02-(time.monotonic()-started)))

    async def _camera(self, name, url):
        try:
            timeout = aiohttp.ClientTimeout(total=None, sock_connect=3, sock_read=3)
            async with self.session.get(url, timeout=timeout, allow_redirects=False, auto_decompress=False) as response:
                if response.status != 200 or response.headers.get('Content-Type','').split(';')[0] != 'multipart/x-mixed-replace':
                    raise ValueError('camera_stream_unavailable')
                multipart = aiohttp.MultipartReader.from_response(response)
                while self.writer.accepting:
                    jpeg, _ = await asyncio.wait_for(next_frame(multipart), FRAME_TIMEOUT)
                    self.writer.put(name, {'received_ns':time.monotonic_ns(), 'source_timestamp':None, 'jpeg':jpeg})
        except (OSError, ValueError, aiohttp.ClientError, asyncio.TimeoutError) as exc:
            # Retain diagnosis, but do not certify a truncated multi-camera episode.
            self.writer.fail({'head':'头部','left_wrist':'左腕','right_wrist':'右腕'}[name]+'相机数据无效或中断；已保留未完成录制')

    async def _watchdog(self):
        while self.writer.accepting:
            if not self.reader.connected or time.monotonic()-self.last_received > .5:
                self.writer.fail('joint_feedback_interrupted')
                break
            await asyncio.sleep(.05)

    async def finish(self):
        async with self.lock:
            self.reader.teaching_observer = None
            for task in self.tasks:
                task.cancel()
            await asyncio.gather(*self.tasks, return_exceptions=True)
            self.tasks = []
            if self.writer is None:
                return self.info()
            manifest = await asyncio.to_thread(self.writer.finish)
            return {**self.info(), 'complete':manifest['complete'], 'filename':self.writer.ident,
                'training_complete':manifest.get('training_complete',False), 'quality':manifest.get('quality')}
