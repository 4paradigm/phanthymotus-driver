"""On-demand microphone capture over the driver's shared authenticated SSH path."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import re
import select
import struct
import threading
import time

from bootstrap import load_secrets

BLOCK = 1024


def read_exact(process, size, stop, timeout=5.):
    end, data = time.monotonic()+timeout, bytearray()
    while len(data) < size:
        if stop.is_set():
            raise InterruptedError('microphone_stopped')
        remaining = end-time.monotonic()
        if remaining <= 0:
            raise TimeoutError('microphone_stream_timeout')
        if not select.select([process.stdout], [], [], min(.2, remaining))[0]:
            continue
        chunk = os.read(process.stdout.fileno(), size-len(data))
        if not chunk:
            raise ConnectionError('microphone_stream_disconnected')
        data.extend(chunk)
    return bytes(data)


class Capture:
    def __init__(self, node, process, topic):
        from audio_msgs.msg import AudioChunk
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.message_type, self.node, self.process, self.topic = AudioChunk, node, process, topic
        self.publisher = node.create_publisher(AudioChunk, topic, QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self.stats = {'state': 'loading', 'chunks': 0, 'bytes': 0, 'error': None}
        self.worker = threading.Thread(target=self._run, daemon=True, name='microphone-capture')
        self.worker.start()

    def _run(self):
        heartbeat = 0.
        previous_stamp = None
        first_stamp, first_receipt = None, None
        try:
            header = bytearray()
            header_deadline = time.monotonic()+8.
            while not header.endswith(b'\n') and len(header) <= 256:
                header.extend(read_exact(self.process, 1, self.stop, max(.001, header_deadline-time.monotonic())))
            if json.loads(header) != {'ok': True, 'format': 'audio/pcm-16k'}:
                raise RuntimeError('invalid_microphone_stream_header')
            while not self.stop.is_set():
                if time.monotonic()-heartbeat >= 1.:
                    self.process.stdin.write(b'.')
                    self.process.stdin.flush()
                    heartbeat = time.monotonic()
                stamp, size = struct.unpack('!dI', read_exact(self.process, 12, self.stop))
                if size != BLOCK or not math.isfinite(stamp) or (previous_stamp is not None and stamp < previous_stamp):
                    raise RuntimeError('invalid_microphone_frame')
                previous_stamp = stamp
                now = time.monotonic()
                if first_stamp is None:
                    first_stamp, first_receipt = stamp, now
                elif (now-first_receipt)-(stamp-first_stamp) > 1.:
                    raise RuntimeError('microphone_audio_delayed')
                data = read_exact(self.process, size, self.stop)
                values = struct.unpack('<512h', data)
                message = self.message_type()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.format = 'audio/pcm-16k'
                message.data = list(data)
                with self.lock:
                    if self.stop.is_set():
                        break
                    self.publisher.publish(message)
                    self.stats.update(state='running', chunks=self.stats['chunks']+1,
                        bytes=self.stats['bytes']+len(data), rms=math.sqrt(sum(x*x for x in values)/len(values)),
                        peak=max(map(abs, values)), last_frame_at=time.time())
                self.ready.set()
        except Exception as exc:
            with self.lock:
                if not self.stop.is_set():
                    self.stats.update(state='error', error=type(exc).__name__+': microphone capture ended; refresh devices and start again')
            self.ready.set()
        finally:
            self._terminate()

    def _terminate(self):
        # Closing stdin lets the remote helper terminate arecord/parecord cleanly.
        try:
            self.process.stdin.close()
        except OSError:
            pass
        try:
            self.process.wait(timeout=2)
        except Exception:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except Exception:
                self.process.kill()
                self.process.wait()

    def snapshot(self):
        with self.lock:
            result = dict(self.stats)
        result['topic_out'] = [{'topic': self.topic, 'format': 'audio/pcm-16k'}]
        result['timestamp_basis'] = 'driver_receipt_time; remote stamps used for ordering only'
        result['acoustically_verified'] = False
        return result

    def close(self):
        with self.lock:
            self.stop.set()
        self.worker.join(timeout=6)
        if self.worker.is_alive():
            self.process.kill()
            self.worker.join(timeout=1)
        if self.worker.is_alive():
            raise RuntimeError('microphone_capture_shutdown_unconfirmed')
        self.node.destroy_publisher(self.publisher)
        self.process.stdout.close()


class Microphones:
    def __init__(self, node, namespace, provisioner, config=None):
        self.node, self.namespace, self.manager = node, namespace.strip('/'), provisioner
        self.default = (config or {}).get('source', '')
        self.source = (Path(__file__).parent/'host/microphone_rpc.py').read_text()
        self.lock = threading.RLock()
        self.probe_lock = threading.Lock()
        self.devices, self.errors, self.sessions, self.configs = [], {}, {}, {}
        self.starting = {}
        self.closed = threading.Event()
        self.worker = None

    def _secrets(self):
        return load_secrets(os.environ.get('KUAVO_SSH_SECRETS', str(self.manager.state_dir/'ssh-secrets.json')))

    def refresh(self):
        with self.probe_lock:
            devices, errors = [], {}
            def probe(role):
                target = self.manager._target(role)
                if target.get('enabled', True) is False:
                    return role, []
                result = self.manager.transport.run(role, target, {'operation': 'probe'},
                                                    self.source, self._secrets(), timeout=15)
                return role, result['result']['devices']
            with ThreadPoolExecutor(max_workers=3) as pool:
                futures = {role: pool.submit(probe, role) for role in ('host', 'controller', 'chassis')}
                for role, future in futures.items():
                    try:
                        _, entries = future.result()
                        for entry in entries:
                            devices.append({**entry, 'id': role+'/'+entry['id'], 'role': role})
                    except Exception as exc:
                        errors[role] = type(exc).__name__
            with self.lock:
                self.devices, self.errors = devices, errors
            return {'state': 'ready', 'available_devices': devices, 'probe_errors': errors}

    def start_background(self):
        def discover():
            while not self.closed.is_set():
                self.refresh()
                self.closed.wait(15.)
        with self.lock:
            if self.worker is None and not self.closed.is_set():
                self.worker = threading.Thread(target=discover, daemon=True, name='microphone-discovery')
                self.worker.start()

    def _external_devices(self):
        # One ALSA endpoint per USB capture device; PulseAudio aliases and
        # unverified motherboard jacks must not appear as physical microphones.
        return [d for d in self.devices if d.get('usb') and '/alsa/' in d['id']]

    def tools(self):
        with self.lock:
            choices = [{'const': d['id'], 'title': d['role']+': '+d['name']}
                       for d in self._external_devices()]
        selector = {'type': 'string', 'scope': 'instance',
                    'description': 'USB microphone connected to a robot board. Shared SSH settings are used.',
                    'oneOf': choices or [{'const': '', 'title': 'No USB microphone detected — connect one and refresh the driver cards'}]}
        return [{'name': 'ext_mic', 'type': 'sensor', 'multiInstance': True,
            'description': '16 kHz mono PCM16 USB microphone. Select an input in card settings; enabling/disabling the sensor starts/stops capture.',
            'topic_in': [], 'topic_out': [{'format': 'audio/pcm-16k'}],
            'inputSchema': {'type': 'object', 'properties': {}},
            'configSchema': {'type': 'object', 'properties': {'device_index': selector}}}]

    def _key(self, card, args):
        instance = args.get('instance_id', '') if card == 'ext_mic' else 'default'
        if not isinstance(instance, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', instance):
            raise ValueError('instance_id_required_use_letters_numbers_hyphens_underscores')
        return card, instance

    def dispatch(self, card, action, args):
        key, token = None, None
        try:
            if card != 'ext_mic':
                raise ValueError('unverified_builtin_microphone_not_available_use_ext_mic')
            if action == 'refresh_devices':
                return self.refresh()
            if action == 'info' and card == 'ext_mic' and not args.get('instance_id'):
                with self.lock:
                    return {'state': 'idle', 'available_devices': list(self._external_devices()),
                            'probe_errors': dict(self.errors), 'active_instances': [k[1] for k in self.sessions if k[0] == card]}
            key = self._key(card, args)
            with self.lock:
                if self.closed.is_set():
                    raise RuntimeError('driver_is_shutting_down')
                session = self.sessions.get(key)
                if action == 'info':
                    return session.snapshot() if session else {'state': 'idle',
                        'available_devices': list(self._external_devices()), 'probe_errors': dict(self.errors)}
                if action == 'config':
                    if session or key in self.starting:
                        raise RuntimeError('stop_before_changing_microphone')
                    selection = args.get('device_index')
                    if not isinstance(selection, str) or len(selection) > 256:
                        raise ValueError('device_index_required')
                    self.configs[key] = selection
                    return {'state': 'configured', 'device_index': selection}
                if action == 'stop':
                    self.starting.pop(key, None)
                    if session:
                        session.close()
                        del self.sessions[key]
                    return {'state': 'idle'}
                if action != 'start':
                    raise ValueError('unsupported_microphone_action')
                selection = args.get('device_index') or self.configs.get(key) or (self.default if card == 'mic' else '')
                if session and session.snapshot()['state'] == 'error':
                    session.close()
                    del self.sessions[key]
                    session = None
                if session:
                    if selection and selection != self.configs.get(key):
                        raise RuntimeError('stop_before_changing_microphone')
                    return session.snapshot()
                if key in self.starting:
                    return {'state': 'loading', 'message': 'Microphone discovery is in progress.'}
                token = object()
                self.starting[key] = token
            self.refresh()
            with self.lock:
                if self.closed.is_set():
                    raise RuntimeError('driver_is_shutting_down')
                if self.starting.get(key) is not token:
                    return {'state': 'idle'}
                # Auto-select only a unique USB capture endpoint, never a motherboard jack.
                if not selection:
                    usb = [d['id'] for d in self.devices if d.get('usb') and '/alsa/' in d['id']]
                    if len(usb) == 1:
                        selection = usb[0]
                if selection not in {d['id'] for d in self._external_devices()}:
                    raise RuntimeError('USB_microphone_unavailable_connect_device_and_refresh_driver_cards')
                if any(value == selection and k in self.sessions for k, value in self.configs.items() if k != key):
                    raise RuntimeError('microphone_device_in_use')
                role, device = selection.split('/', 1)
                process = self.manager.transport.open_stream(role, self.manager._target(role),
                    {'operation': 'capture', 'device': device}, self.source, self._secrets())
                topic_instance = 'i_'+key[1].encode().hex() if card == 'ext_mic' else 'default'
                topic = '/'+ '/'.join(part for part in (self.namespace, card, topic_instance, 'audio') if part)
                try:
                    session = Capture(self.node, process, topic)
                except Exception:
                    process.stdin.close()
                    process.terminate()
                    process.wait(timeout=3)
                    process.stdout.close()
                    raise
                self.sessions[key], self.configs[key] = session, selection
                self.starting.pop(key, None)
            session.ready.wait(10)
            result = session.snapshot()
            with self.lock:
                if self.sessions.get(key) is not session:
                    return {'state': 'idle'}
            if result['state'] != 'running':
                with self.lock:
                    if self.sessions.get(key) is session:
                        session.close()
                        del self.sessions[key]
                return {'state': 'error', 'error': result.get('error') or 'microphone_no_audio_within_10s'}
            return {**result, 'device_index': selection}
        except Exception as exc:
            with self.lock:
                if token is not None and self.starting.get(key) is token:
                    self.starting.pop(key, None)
            return {'state': 'error', 'error': str(exc).encode('unicode_escape').decode()[:240]}

    def close(self):
        self.closed.set()
        with self.lock:
            self.starting.clear()
            for session in self.sessions.values():
                session.close()
            self.sessions.clear()
        if self.worker:
            self.worker.join(timeout=1)
