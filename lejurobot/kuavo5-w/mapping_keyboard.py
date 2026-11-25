"""Browser lease -> image-owned chassis watchdog -> factory RobotMotion."""
import json
import math
import os
from pathlib import Path
import select
import threading
import time
from uuid import uuid4

from bootstrap import load_secrets

LINEAR_LIMIT, ANGULAR_LIMIT, LEASE_S = .15, .3, .3


class FactoryKeyboardStream:
    def __init__(self, client):
        self.stop_lock = threading.Lock()
        manager = client.provisioner
        target = manager._target('chassis')
        source = ('NAVIGATION_SOURCE = ' + repr(client._source) + '\n' +
                  (Path(__file__).parent/'host/manual_keyboard.py').read_text())
        self.process = manager.transport.open_stream('chassis', target,
            {'master': manager.settings.get('master', 'http://192.168.26.1:11311'),
             'ros_ip': target['ros_ip']}, source,
            load_secrets(os.environ.get('KUAVO_SSH_SECRETS', str(manager.state_dir/'ssh-secrets.json'))))
        os.set_blocking(self.process.stdin.fileno(), False)

    def read(self, timeout):
        end, data = time.monotonic()+timeout, bytearray()
        while time.monotonic() < end:
            if not select.select([self.process.stdout], [], [], min(.05, max(0., end-time.monotonic())))[0]:
                continue
            part = os.read(self.process.stdout.fileno(), 1)
            if not part:
                raise ConnectionError('factory_keyboard_stream_disconnected')
            if part == b'\n':
                value = json.loads(data)
                if not isinstance(value, dict):
                    raise ValueError('invalid_keyboard_guard_reply')
                return value
            data.extend(part)
            if len(data) > 1024:
                raise ValueError('oversized_keyboard_guard_reply')
        raise TimeoutError('factory_keyboard_guard_timeout')

    def send(self, value):
        payload = (json.dumps(value)+'\n').encode()
        if len(payload) > 1024 or os.write(self.process.stdin.fileno(), payload) != len(payload):
            raise RuntimeError('keyboard_stream_write_failed')

    def stop(self):
        with self.stop_lock:
            if not self.process.stdin.closed:
                self.process.stdin.close()  # EOF also stops the chassis-local guard.

    def close(self):
        self.stop()
        try:
            self.process.wait(timeout=1)
        except Exception:
            self.process.kill()
            self.process.wait(timeout=1)
        self.process.stdout.close()


class MappingKeyboard:
    def __init__(self, spatial):
        self.spatial, self.bundle = spatial, spatial.bundle
        self.lock = self.bundle.tracker.lock
        self.session = None
        self.thread = None
        self.last_error = None

    def info(self):
        with self.lock:
            s = self.session
            return {'state': s['phase'] if s else 'stop_unconfirmed' if self.spatial._keyboard_pending else 'idle',
                    'busy': s is not None or self.spatial._keyboard_pending,
                    'session_id': s['id'] if s else None, 'last_error': self.last_error,
                    'linear_limit_m_s': LINEAR_LIMIT, 'angular_limit_rad_s': ANGULAR_LIMIT,
                    'watchdog_ms': int(LEASE_S*1000), 'route': 'factory_RobotMotion_MANUAL'}

    def open(self):
        with self.lock:
            self.spatial._restore()
            mapping = self.spatial.store.state('mapping') if self.spatial.mapping_active() else None
            if not mapping or mapping.get('phase') != 'mapping':
                raise RuntimeError('confirmed_mapping_session_required')
            if self.session is not None or self.spatial.unresolved():
                raise RuntimeError('keyboard_or_chassis_operation_busy')
            if self.bundle._transition is not None or 'base' in self.bundle.tracker._active:
                raise RuntimeError('base_control_is_busy')
            if (self.spatial._closed.is_set() or not self.bundle.ros1.connection[0]
                    or self.bundle._resume_unresolved() or self.bundle._mode_unresolved()
                    or self.bundle.state.motion_send_uncertain):
                raise RuntimeError('shared_control_is_disconnected_or_unconfirmed')
            self.bundle.state.require_motion_enabled()
            # Enabling a panel does not enable the whole body. Manual control
            # needs the already confirmed /enable_vel_control=False handoff.
            if self.bundle.state.read('upper_velocity_enabled')[0] is not False:
                raise RuntimeError('upper_velocity_must_be_disabled_for_mapping')
            self.bundle.base_servo.mark_paused()
            aid, cancelled = self.bundle.tracker.begin('base', 'mapping_keyboard', notify=False)
            s = {'id': uuid4().hex, 'action_id': aid, 'cancelled': cancelled,
                 'epoch': self.bundle._stop_epoch, 'connection': self.bundle.ros1.connection,
                 'mapping_token': mapping['token'], 'phase': 'loading', 'last_at': time.monotonic(),
                 'seq': -1, 'values': [0., 0.], 'stop': threading.Event()}
            self.session, self.last_error = s, None
            self.spatial._keyboard_pending = True
            self.spatial._persist()
            self.thread = threading.Thread(target=self._run, args=(s,), daemon=True, name='mapping-keyboard')
            self.thread.start()
            return self.info()

    def command(self, args):
        values, seq = args.get('values'), args.get('seq')
        if (not isinstance(values, list) or len(values) != 2 or type(seq) is not int
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
                or abs(values[0]) > LINEAR_LIMIT or abs(values[1]) > ANGULAR_LIMIT):
            raise ValueError('invalid_keyboard_command')
        with self.lock:
            s = self.session
            if (s is None or args.get('session_id') != s['id'] or s['stop'].is_set()
                    or seq <= s['seq'] or time.monotonic()-s['last_at'] > LEASE_S):
                raise RuntimeError('keyboard_lease_expired_or_replayed_reopen_panel')
            s['seq'], s['last_at'] = seq, time.monotonic()
            s['values'] = list(values) if s['phase'] == 'ready' else [0., 0.]
            return self.info()

    def invalidate(self):
        """No IO; called inside the common whole-body stop lock."""
        with self.lock:
            if self.session:
                self.session['phase'] = 'stopping'
                self.session['stop'].set()

    def stop(self, session_id=None):
        with self.lock:
            if self.session and session_id is not None and session_id != self.session['id']:
                raise RuntimeError('keyboard_session_changed')
            self.invalidate()
            worker = self.thread
            stream = self.session.get('stream') if self.session else None
        if stream is not None:
            try:
                stream.stop()
            except Exception:
                pass
        if worker and worker is not threading.current_thread():
            worker.join(timeout=1)
        return self.info()

    def _valid(self, s):
        with self.lock:
            mapping = self.spatial.store.state('mapping') or {}
            return (self.session is s and not s['stop'].is_set() and not s['cancelled'].is_set()
                and not self.spatial._closed.is_set() and s['epoch'] == self.bundle._stop_epoch
                and s['connection'][0] and s['connection'] == self.bundle.ros1.connection
                and self.bundle.tracker.current('base', s['action_id'])
                and mapping.get('token') == s['mapping_token'] and mapping.get('phase') == 'mapping'
                and self.bundle.state.read('upper_velocity_enabled')[0] is False
                and time.monotonic()-s['last_at'] <= LEASE_S)

    def _run(self, s):
        stream, stopped = None, False
        try:
            stream = self.spatial.client.keyboard_stream()
            with self.lock:
                s['stream'] = stream
            reply = stream.read(45)  # Initial hold/cancel is allowed no motion.
            if reply.get('state') != 'held':
                raise RuntimeError('factory_keyboard_preparation_failed')
            seq, activated = 0, False
            while True:
                with self.lock:
                    if not self._valid(s):
                        break
                    values = list(s['values']) if activated else [0., 0.]
                    # Nonblocking single-frame write under the publication lock:
                    # a stop epoch cannot be overtaken by an older direction.
                    stream.send({'nonce': reply['nonce'], 'seq': seq, 'values': values})
                reply = stream.read(8 if not activated else .25)
                if reply.get('state') != 'ready' or reply.get('seq') != seq:
                    raise RuntimeError('factory_keyboard_command_unconfirmed')
                seq += 1
                if not activated:
                    with self.lock:
                        if self._valid(s):
                            s['phase'], s['values'] = 'ready', [0., 0.]
                    activated = True
                s['stop'].wait(.08)
        except Exception:
            if not s['stop'].is_set() and not s['cancelled'].is_set():
                self.last_error = 'factory_keyboard_guard_failed_inspect_stop_confirmation'
        finally:
            with self.lock:
                s['phase'] = 'stopping'
            if stream is not None:
                try:
                    stream.stop()
                    end = time.monotonic()+8
                    while time.monotonic() < end:
                        reply = stream.read(max(.01, end-time.monotonic()))
                        if (reply.get('state') == 'stopped' and reply.get('stationary') is True
                                and reply.get('software_estop') is True):
                            stopped = True
                            break
                except Exception:
                    pass
                finally:
                    try:
                        stream.close()
                    except Exception:
                        stopped = False
            with self.lock:
                if stopped:
                    self.spatial._keyboard_pending = False
                    self.spatial._persist()
                else:
                    self.last_error = 'keyboard_stop_unconfirmed_use_stop_nav'
                self.bundle.tracker.finish('base', s['action_id'], 'mapping_keyboard',
                    'cancelled' if stopped else 'error', {'reason': 'keyboard_closed', 'stop_confirmed': stopped})
                if self.session is s:
                    self.session = None
