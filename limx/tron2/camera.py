"""Independent on-demand RGB MJPEG relays; no motion, recording or new codecs."""
import asyncio
import time
import aiohttp

CAMERA_SOURCES = {
    'camera': ('/limx_tron2/camera', '/camera/top/color/image_raw/compressed'),
    'camera_left': ('/limx_tron2/camera_left', '/camera/left/color/image_rect_raw/compressed'),
    'camera_right': ('/limx_tron2/camera_right', '/camera/right/color/image_rect_raw/compressed'),
}
TOPIC = CAMERA_SOURCES['camera'][0]
MAX_FRAME = 4 * 1024 * 1024
MAX_DIMENSION = 4096
FRAME_TIMEOUT = 3
START_TIMEOUT = 8
PUBLISH_INTERVAL = .1


def jpeg_size(data):
    """Check bounded JPEG framing and declared dimensions; do not re-encode."""
    if not 4 <= len(data) <= MAX_FRAME or not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
        raise ValueError('Invalid JPEG framing or size')
    pos = 2
    dimensions = None
    while pos < len(data)-2:
        if data[pos] != 0xff:
            raise ValueError('Invalid JPEG header')
        while pos < len(data) and data[pos] == 0xff:
            pos += 1
        if pos >= len(data):
            break
        marker = data[pos]; pos += 1
        if marker in (0xd8, 0xd9, 0x00) or 0xd0 <= marker <= 0xd7:
            raise ValueError('Unexpected JPEG header marker')
        if pos+2 > len(data):
            break
        length = int.from_bytes(data[pos:pos+2], 'big')
        if length < 2 or pos+length > len(data)-2:
            raise ValueError('Truncated JPEG segment')
        if marker in (0xc0, 0xc1, 0xc2):
            if length < 8:
                raise ValueError('Truncated JPEG dimensions')
            height = int.from_bytes(data[pos+3:pos+5], 'big')
            width = int.from_bytes(data[pos+5:pos+7], 'big')
            if not 1 <= width <= MAX_DIMENSION or not 1 <= height <= MAX_DIMENSION:
                raise ValueError('Unsupported JPEG dimensions')
            dimensions = (width, height)
        if marker == 0xda:
            if dimensions is None or pos+length >= len(data)-2:
                raise ValueError('Missing JPEG dimensions or scan')
            return dimensions
        pos += length
    raise ValueError('Missing JPEG scan')


async def next_frame(reader):
    try:
        return await read_frame(reader)
    except AssertionError as exc:
        # aiohttp uses assertions for malformed multipart delimiters/EOF.
        # Treat bad vendor framing as a reconnectable stream failure.
        raise ValueError('Malformed camera multipart framing') from exc


async def read_frame(reader):
    part = await reader.next()
    if not isinstance(part, aiohttp.BodyPartReader):
        raise ValueError('Expected one JPEG multipart frame')
    if part.headers.get('Content-Type', '').split(';')[0].strip().lower() != 'image/jpeg':
        raise ValueError('Expected JPEG camera frame')
    data = bytearray()
    while not part.at_eof():
        data.extend(await part.read_chunk(16384))
        if len(data) > MAX_FRAME:
            raise ValueError('Camera frame exceeds size limit')
    dimensions = jpeg_size(data)
    return bytes(data), dimensions


class CameraStream:
    def __init__(self, url, publish, session=None, topic=TOPIC):
        self.url, self.publish, self.session = url, publish, session
        self.topic = topic
        self.task = None
        self.active = False
        self.received_at = self.last_frame_at = None
        self.width = self.height = self.frame_bytes = None
        self.frames_received = self.frames_published = 0
        self.error = None
        # Construct synchronization only inside an async lifecycle call. Info
        # and discovery remain usable without an event loop (also Python 3.9).
        self.first_frame = None
        self.lock = None
        self.epoch = 0

    def info(self):
        age = None if self.last_frame_at is None else max(0, time.monotonic()-self.last_frame_at)
        available = self.active and age is not None and age < FRAME_TIMEOUT
        state = 'running' if available else 'reconnecting' if self.active and self.error \
                else 'starting' if self.active else 'idle'
        return {'state': state, 'read_only': True, 'active': self.active, 'available': available,
                'frames_received': self.frames_received, 'frames_published': self.frames_published,
                'width': self.width, 'height': self.height, 'last_frame_bytes': self.frame_bytes,
                'publish_fps_limit': 10, 'received_at': self.received_at,
                'reception_age_seconds': age, 'measurement_time_verified': False,
                'last_error': self.error, 'topic_out': [{'topic': self.topic, 'format': 'image/jpeg'}]}

    async def call(self, action):
        if action == 'info':
            return self.info()
        if action not in ('start', 'stop'):
            raise ValueError('Unsupported camera action')
        if self.lock is None:
            self.lock = asyncio.Lock()
        async with self.lock:
            if action == 'stop':
                await self._close()
                return self.info()
            if self.session is None:
                raise ValueError('Camera HTTP session is not ready')
            if not self.active:
                self.active = True
                self.error = None
                self.first_frame = asyncio.Event()
                self.epoch += 1
                self.task = asyncio.create_task(self.run(self.epoch))
            epoch, first_frame = self.epoch, self.first_frame
        # Do not hold the lifecycle lock while waiting: stop must promptly abort
        # a starting stream, and an old failed start must not stop a newer one.
        try:
            await asyncio.wait_for(first_frame.wait(), START_TIMEOUT)
        except asyncio.TimeoutError as exc:
            async with self.lock:
                if epoch == self.epoch:
                    await self._close()
            raise ValueError('Camera did not provide a frame; check vendor camera service') from exc
        if epoch != self.epoch or not self.info()['available']:
            raise ValueError('Camera start was stopped or no recent frame is available')
        return self.info()

    def unavailable(self):
        self.last_frame_at = self.received_at = None
        self.width = self.height = self.frame_bytes = None
        if self.first_frame is not None:
            self.first_frame.clear()

    async def _close(self):
        self.active = False
        self.epoch += 1
        if self.first_frame is not None:
            self.first_frame.set()  # wake a start waiter before discarding its event
        if self.task:
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
            self.task = None
        self.last_frame_at = self.received_at = None
        self.width = self.height = self.frame_bytes = None
        self.error = None

    async def close(self):
        if self.lock is None:
            self.lock = asyncio.Lock()
        async with self.lock:
            await self._close()

    async def run(self, epoch):
        while self.active and epoch == self.epoch:
            try:
                timeout = aiohttp.ClientTimeout(total=None, sock_connect=3, sock_read=5)
                async with self.session.get(self.url, timeout=timeout, allow_redirects=False, auto_decompress=False,
                        headers={'Accept': 'multipart/x-mixed-replace'}) as response:
                    if response.status != 200:
                        raise ValueError('Camera HTTP service unavailable')
                    if response.headers.get('Content-Type', '').split(';')[0].strip().lower() != 'multipart/x-mixed-replace' \
                            or response.headers.get('Content-Encoding', 'identity').lower() != 'identity':
                        raise ValueError('Expected uncompressed multipart camera stream')
                    reader = aiohttp.MultipartReader.from_response(response)
                    last_publish = None
                    while self.active and epoch == self.epoch:
                        data, dimensions = await asyncio.wait_for(next_frame(reader), FRAME_TIMEOUT)
                        self.frames_received += 1
                        now = time.monotonic()
                        if last_publish is not None and now-last_publish < PUBLISH_INTERVAL:
                            continue
                        self.publish(self.topic, data)
                        self.width, self.height = dimensions
                        self.frame_bytes = len(data)
                        last_publish = self.last_frame_at = now
                        self.received_at = time.time()
                        self.frames_published += 1
                        self.error = None
                        self.first_frame.set()
            except (OSError, ValueError, RuntimeError, aiohttp.ClientError, asyncio.TimeoutError) as exc:
                self.error = type(exc).__name__
                self.unavailable()
                await asyncio.sleep(1)
