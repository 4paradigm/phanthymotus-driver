"""ROS2 AudioChunk -> isolated rosbridge -> verified chassis PulseAudio sink."""
import base64
import json
import queue
import re
import threading
import time

ROOT = '/motus/kuavo5w/speaker'
EOF = b'\x01\x00\xff\xff\x01\x00\xff\xff'


class Speaker:
    def __init__(self, node, media_factory):
        self.node, self.factory = node, media_factory
        self.lock = threading.RLock()
        self.client = self.sub = self.session = self.worker = None
        self.topic = None
        self.stop_event = threading.Event()
        self.buffer = queue.Queue(maxsize=100)  # at most ten seconds, never unbounded
        self.muted = False
        self.error = None
        self.stop_confirmed = True
        self.remote_status = {}

    def tool(self):
        return {'name': 'speaker', 'type': 'actuator', 'multiInstance': False,
                'description': 'Kuavo chassis speaker: stream mono signed PCM16, 16 kHz audio; start checks the chassis playback service.',
                'topic_in': [{'format': 'audio/pcm-16k'}],
                'inputSchema': {'type': 'object', 'properties': {
                    'action': {'type': 'string', 'enum': ['start', 'stop', 'info', 'interrupt']},
                    'input_topic': {'type': 'string', 'description': 'AudioChunk input topic provided by canvas'}},
                    'required': ['action'], 'x-resource': 'mouth',
                    'x-action-params': {name: {'params': ['input_topic'] if name == 'start' else []}
                                        for name in ('start', 'stop', 'info', 'interrupt')},
                    'x-hooks': {'on_interrupt_all': {'action': 'interrupt'},
                                'on_interrupt_speak': {'action': 'interrupt'}}}}

    def _call(self, action):
        result = self.client.call_service(ROOT + '/' + action, 'std_srvs/Trigger', {},
                                          timeout=.5 if action == 'status' else 1.)
        if result.get('success') is not True:
            raise RuntimeError('chassis speaker service rejected ' + action)
        status = json.loads(result['message'])
        if not isinstance(status, dict):
            raise ValueError('invalid chassis speaker response')
        return status

    def _clear(self):
        while True:
            try:
                self.buffer.get_nowait()
            except queue.Empty:
                return

    def _chunk(self, message, session):
        # Lifecycle calls may wait for remote acknowledgement. Never make the
        # shared ROS2 executor wait behind those calls for stale audio.
        if not self.lock.acquire(blocking=False):
            return
        try:
            if self.session != session or self.stop_event.is_set():
                return
            if message.format not in ('', 'pcm', 'pcm_s16le', 'pcm_16k_16bit_mono', 'audio/pcm-16k'):
                self.error = 'expected audio/pcm-16k'
                self.stop_event.set()
                return
            if len(message.data) > 32000 or len(message.data) % 2:
                self.error = 'invalid or oversized PCM16k chunk'
                self.stop_event.set()
                return
            pcm = bytes(message.data)
            if pcm == EOF:
                self.muted = False
                return
            if self.muted:
                return
            try:
                for offset in range(0, len(pcm), 3200):
                    self.buffer.put_nowait(pcm[offset:offset + 3200])
            except queue.Full:
                self.error = 'speaker input exceeds ten-second buffer'
                self.stop_event.set()
        finally:
            self.lock.release()

    def _run(self, client, session, generation, event):
        sequence, deadline, check_at = 0, time.monotonic(), 0.
        try:
            while not event.is_set():
                if client.connection != (True, generation):
                    raise ConnectionError('speaker bridge connection changed; start again')
                now = time.monotonic()
                if now >= check_at:
                    status = self._call('status')
                    self.remote_status = status
                    if status.get('session') != session or status.get('error'):
                        raise RuntimeError('chassis playback session failed; start again')
                    check_at = time.monotonic() + .5
                try:
                    pcm = self.buffer.get(timeout=.1)
                except queue.Empty:
                    pcm = b''  # renew the lease while waiting for the next utterance
                if event.is_set():
                    break
                client.publish(ROOT + '/pcm', 'std_msgs/String', {'data': json.dumps({
                    'session': session, 'seq': sequence,
                    'pcm': base64.b64encode(pcm).decode('ascii')})})
                sequence += 1
                deadline = max(deadline, time.monotonic()) + len(pcm) / 32000
                event.wait(max(0., deadline - time.monotonic() - .1))
        except Exception as exc:
            self.error = type(exc).__name__ + ': ' + str(exc)[:160]
            event.set()
        finally:
            self._clear()
            try:
                self.remote_status = self._call('stop')
                self.stop_confirmed = self.remote_status.get('session') is None
            except Exception:
                pass  # chassis watchdog independently stops on lease loss

    def _stop(self):
        self.stop_event.set()
        self.session = None
        if self.sub is not None:
            self.node.destroy_subscription(self.sub)
            self.sub = None
        if self.worker is not None:
            self.worker.join(timeout=2.)
            if self.worker.is_alive():
                raise RuntimeError('speaker worker stop is not confirmed')
            self.worker = None
        elif self.client is not None and not self.stop_confirmed:
            try:
                self.remote_status = self._call('stop')
                self.stop_confirmed = self.remote_status.get('session') is None
            except Exception:
                pass
        if self.client is not None:
            self.client.close()
            self.client = None
        self._clear()
        self.topic = None
        return self.stop_confirmed

    def dispatch(self, action, args):
        try:
            with self.lock:
                if action == 'info':
                    return {'state': 'error' if self.error else ('ready' if self.session else 'idle'),
                            'error': self.error, 'topic': self.topic,
                            'buffer_chunks': self.buffer.qsize(), 'output_status': self.remote_status,
                            'format': 'audio/pcm-16k', 'output': 'chassis_analog'}
                if action == 'stop':
                    if not self._stop():
                        raise RuntimeError('speaker stop unconfirmed; chassis lease expires within 1.5 seconds of last packet')
                    return {'state': 'idle'}
                if action not in ('start', 'interrupt'):
                    return {'state': 'error', 'error': 'unknown speaker action'}
                if action == 'interrupt' and self.topic is None:
                    return {'state': 'idle'}
                topic = args.get('input_topic') if action == 'start' else self.topic
                if not isinstance(topic, str) or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*', topic) or '//' in topic:
                    raise ValueError('start requires an absolute AudioChunk input_topic')
                if action == 'start' and self.topic == topic and self.session and not self.stop_event.is_set():
                    return {'state': 'ready', 'topic': topic}
                self._stop()
                if self.factory is None:
                    raise RuntimeError('speaker requires an isolated media bridge')
                from audio_msgs.msg import AudioChunk
                from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
                self.client = self.factory()
                self.client.start()
                deadline = time.monotonic() + 3.
                while not self.client.connection[0] and time.monotonic() < deadline:
                    time.sleep(.02)
                self.client.advertise(ROOT + '/pcm', 'std_msgs/String')
                status = self._call('start')
                session = status.get('session')
                if not isinstance(session, str) or not re.fullmatch('[a-f0-9]{32}', session):
                    raise RuntimeError('chassis playback session unavailable')
                self.session, self.topic = session, topic
                self.remote_status = status
                self.stop_confirmed = False
                self.error, self.muted = None, action == 'interrupt'
                self.stop_event = threading.Event()
                self.sub = self.node.create_subscription(AudioChunk, topic,
                    lambda msg, token=session: self._chunk(msg, token), QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                        depth=10, durability=DurabilityPolicy.VOLATILE))
                self.worker = threading.Thread(target=self._run,
                    args=(self.client, session, self.client.connection[1], self.stop_event),
                    daemon=True, name='speaker_pcm')
                self.worker.start()
                return {'state': 'ready', 'topic': topic, 'muted_until_eof': self.muted}
        except Exception as exc:
            self.error = type(exc).__name__ + ': ' + str(exc)[:160]
            # No motion enable/resume paths are used for audio.
            if action != 'stop':
                with self.lock:
                    try:
                        self._stop()
                    except Exception:
                        pass
            return {'state': 'error', 'error': self.error}

    def close(self):
        return self.dispatch('stop', {})
