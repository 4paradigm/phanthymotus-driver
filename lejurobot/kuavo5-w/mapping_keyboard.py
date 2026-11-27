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
FRAME_INTERVAL_S = .08


def confirmed_stop(reply):
    return (reply.get('state') == 'stopped' and reply.get('stationary') is True
            and (reply.get('software_estop') is True or (reply.get('manual_zero_sent') is True
                and reply.get('completion_basis') == 'factory_manual_zero_and_fresh_stationary_odometry')))


def guard_error(reply):
    code = reply.get('error')
    return code if isinstance(code, str) and code in {
        'keyboard_initial_stop_unconfirmed', 'keyboard_chassis_feedback_is_stale',
        'keyboard_requires_healthy_manual_mapping', 'keyboard_activation_requires_zero',
        'keyboard_initial_stop_ownership_lost', 'keyboard_release_rejected',
        'keyboard_command_expired', 'keyboard_watchdog_expired',
        'keyboard_software_stop_is_active', 'keyboard_stop_rejected',
        'keyboard_fallback_stop_unconfirmed',
        'factory_keyboard_stream_disconnected', 'factory_keyboard_guard_timeout',
        'invalid_factory_websocket_handshake', 'factory_websocket_closed',
        'factory_websocket_api_error', 'factory_motion_rejected',
        'factory_websocket_disconnected', 'factory_websocket_write_busy',
        'factory_keyboard_guard_failed'} else 'factory_keyboard_guard_failed'


def stream_source(navigation_source, bridge=False):
    root = Path(__file__).parent / 'host'
    manual = (root / 'manual_keyboard.py').read_text()
    if bridge:
        # Strip the module entry point before embedding other source literals.
        manual = manual.rsplit("if __name__ == '__main__':", 1)[0]
    source = 'NAVIGATION_SOURCE = ' + repr(navigation_source) + '\n' + manual
    if bridge:
        source += '\n' + (root / 'upper_input_gate.py').read_text()
        source += '\n' + (root / 'upper_velocity_bridge.py').read_text()
    return source


class FactoryKeyboardStream:
    def __init__(self, client, bridge=False):
        self.stop_lock = threading.Lock()
        self._read_buffer = bytearray()
        manager = client.provisioner
        target = manager._target('chassis')
        source = stream_source(client._source, bridge)
        self.process = manager.transport.open_stream('chassis', target,
            {'master': target.get('master', 'http://127.0.0.1:11311'),
             'ros_ip': target['ros_ip'],
             'upper_master': manager.settings.get('master', 'http://192.168.26.1:11311')}, source,
            load_secrets(os.environ.get('KUAVO_SSH_SECRETS', str(manager.state_dir/'ssh-secrets.json'))))
        os.set_blocking(self.process.stdin.fileno(), False)

    def read(self, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            newline = self._read_buffer.find(b'\n')
            if newline >= 0:
                if newline > 1024:
                    raise ValueError('oversized_keyboard_guard_reply')
                data = bytes(self._read_buffer[:newline])
                del self._read_buffer[:newline+1]
                value = json.loads(data)
                if not isinstance(value, dict):
                    raise ValueError('invalid_keyboard_guard_reply')
                return value
            if len(self._read_buffer) > 1024:
                raise ValueError('oversized_keyboard_guard_reply')
            if not select.select([self.process.stdout], [], [], min(.05, max(0., end-time.monotonic())))[0]:
                continue
            part = os.read(self.process.stdout.fileno(), 4096)
            if not part:
                raise ConnectionError('factory_keyboard_stream_disconnected')
            self._read_buffer.extend(part)
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
        self.last_failure = None
        self.last_roundtrip_ms = None
        self.max_roundtrip_ms = None

    def info(self):
        with self.lock:
            s = self.session
            return {'state': s['phase'] if s else 'stop_unconfirmed' if self.spatial._keyboard_pending else 'idle',
                    'busy': s is not None or self.spatial._keyboard_pending,
                    'session_id': s['id'] if s else None, 'last_error': self.last_error,
                    'last_failure': self.last_failure,
                    'last_roundtrip_ms': self.last_roundtrip_ms,
                    'max_roundtrip_ms': self.max_roundtrip_ms,
                    'linear_limit_m_s': LINEAR_LIMIT, 'angular_limit_rad_s': ANGULAR_LIMIT,
                    'watchdog_ms': int(LEASE_S*1000), 'route': 'factory_RobotMotion_MANUAL'}

    def open(self):
        with self.lock:
            self.spatial._restore()
            mapping = self.spatial.store.state('mapping') if self.spatial.mapping_active() else None
            if self.spatial.mapping_console_status()['mapping_state'] == 'starting':
                return {'state': 'loading', 'mapping_state': 'starting',
                        'message': 'Mapping is starting. Waiting for completion before preparing keyboard control.'}
            if not mapping or mapping.get('phase') != 'mapping':
                raise RuntimeError('confirmed_mapping_session_required')
            if (self.spatial._operations.locked() or self.spatial._mapping_console_grant !=
                    (mapping['token'], self.bundle._stop_epoch, self.bundle.ros1.connection)):
                raise RuntimeError('open_mapping_console_to_control_mapping')
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
            self.last_failure = None
            self.last_roundtrip_ms = self.max_roundtrip_ms = None
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
            if confirmed_stop(reply):
                stopped = True
                if reply.get('error'):
                    self.last_error = self.last_failure = guard_error(reply)
                return
            if reply.get('state') != 'held':
                raise RuntimeError(guard_error(reply))
            seq, activated = 0, False
            while True:
                sent_at = time.monotonic()
                with self.lock:
                    if not self._valid(s):
                        break
                    values = list(s['values']) if activated else [0., 0.]
                    # Nonblocking single-frame write under the publication lock:
                    # a stop epoch cannot be overtaken by an older direction.
                    stream.send({'nonce': reply['nonce'], 'seq': seq, 'values': values})
                # This is an acknowledgement deadline, not the chassis motion
                # lease. The chassis independently sends zero at 300ms. A
                # delayed reply must not turn a shorter 250ms SSH read into a
                # false watchdog failure, nor cause a stale command retry.
                reply = stream.read(8 if not activated else 1.)
                elapsed = time.monotonic()-sent_at
                self.last_roundtrip_ms = round(elapsed*1000, 1)
                self.max_roundtrip_ms = max(self.max_roundtrip_ms or 0., self.last_roundtrip_ms)
                if confirmed_stop(reply):
                    stopped = True
                    self.last_error = guard_error(reply) if reply.get('error') else 'keyboard_remote_stopped'
                    if reply.get('error'):
                        self.last_failure = self.last_error
                    break
                if reply.get('state') != 'ready' or reply.get('seq') != seq:
                    raise RuntimeError(guard_error(reply))
                seq += 1
                if not activated:
                    with self.lock:
                        if self._valid(s):
                            s['phase'], s['values'] = 'ready', [0., 0.]
                    activated = True
                # Pace start-to-start; SSH latency already consumed part (or
                # all) of the interval. Do not add another 80ms after each ACK.
                s['stop'].wait(max(0., FRAME_INTERVAL_S-elapsed))
        except Exception as exc:
            if not s['stop'].is_set() and not s['cancelled'].is_set():
                self.last_error = guard_error({'error':str(exc)})
                self.last_failure = self.last_error
        finally:
            with self.lock:
                s['phase'] = 'stopping'
            if stream is not None:
                try:
                    stream.stop()
                    end = time.monotonic()+8
                    while not stopped and time.monotonic() < end:
                        reply = stream.read(max(.01, end-time.monotonic()))
                        if confirmed_stop(reply):
                            stopped = True
                            if reply.get('error'):
                                self.last_error = self.last_failure = guard_error(reply)
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


class UpperVelocityBridge:
    """Driver heartbeat grants the chassis bridge only the current base lease."""
    def __init__(self, bundle):
        self.bundle = bundle
        self.lock = threading.Lock()
        self.thread = None
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.forwarded = 0
        self.remote_state = 'unavailable'
        self.input_owner = None
        self.input_ack_count = 0
        self.return_requested = False
        self.return_thread = None
        self.monitor_thread = None
        self.closed = threading.Event()
        self.remote_error = None
        self.last_control = None
        self.last_epoch = getattr(bundle, '_stop_epoch', 0)
        self.external_resume = False

    def request_return(self, tool):
        if tool in ('base_drive','base_servo','controlled_spatial'):
            self.return_requested = True

    def start_monitor(self):
        if self.monitor_thread is not None:return
        def monitor():
            while not self.closed.wait(.1):
                try:self.consider_return()
                except Exception:self.remote_error='remote_return_state_unavailable'
        self.monitor_thread=threading.Thread(target=monitor,daemon=True,name='remote-control-return')
        self.monitor_thread.start()

    def consider_return(self):
        b=self.bundle
        with b.tracker.lock:
            try:enabled=b.state.confirmed_enable()
            except RuntimeError:return
            if self.last_epoch!=b._stop_epoch:
                self.last_epoch=b._stop_epoch
                self.return_requested=self.external_resume=False
                self.last_control=enabled
                return
            if self.last_control is False and enabled is True:
                # An operator's explicit controller re-enable is a new action;
                # this monitor never issues enable_control itself.
                self.return_requested=self.external_resume=True
            self.last_control=enabled
            if (not self.return_requested or enabled is not True or b._closing
                    or b._transition is not None or 'base' in b.tracker._active
                    or b.spatial.unresolved() or b.spatial.mapping_active()
                    or (b.state.motion_blocked and not self.external_resume)
                    or (self.return_thread and self.return_thread.is_alive())):
                return
            if (b._upper_route_key==(b._stop_epoch,b.ros1.connection)
                    and b.state.read('upper_velocity_enabled')[0] is True):
                self.return_requested=self.external_resume=False
                return  # The existing upper route can hand back after neutral.
            epoch=b._stop_epoch
            self.return_requested=self.external_resume=False
        def restore():
            def validate():
                if b.state.confirmed_enable() is not True:
                    raise RuntimeError('remote_return_requires_existing_enable')
            try:
                lease=b.start_motion('base','remote_control',(3,),lambda:None,validate,
                                     request_epoch=epoch,notify=False)
                b.tracker.finish('base',lease[0],'remote_control','cancelled',{'reason':'remote_handoff_ready'})
                self.remote_error=None
            except Exception:
                self.remote_error='remote_return_preparation_unconfirmed'
        self.return_thread=threading.Thread(target=restore,daemon=True,name='remote-control-handoff')
        self.return_thread.start()

    def ensure(self):
        with self.lock:
            self.ready.clear()
            if self.thread is None or not self.thread.is_alive():
                self.stop_event = threading.Event()
                self.ready = threading.Event()
                self.error, self.forwarded = None, 0
                self.thread = threading.Thread(target=self._run, daemon=True, name='upper-velocity-bridge')
                self.thread.start()
        if not self.ready.wait(15) or self.error:
            raise RuntimeError('upper_velocity_bridge_not_ready')

    def wait_remote_ready(self, check, timeout=5.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            check()
            if self.error or self.thread is None or not self.thread.is_alive():
                raise RuntimeError('upper_velocity_bridge_failed_during_remote_handoff')
            if self.remote_state == 'ready':
                return
            self.closed.wait(.05)
        raise RuntimeError('upper_bridge_remote_ready_unconfirmed')

    def wait_input_owner(self, expected, check, timeout=5.):
        initial = self.input_ack_count
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            check()
            if self.error or self.thread is None or not self.thread.is_alive():
                raise RuntimeError('upper_velocity_bridge_failed_during_input_handoff')
            if self.input_ack_count > initial and self.input_owner == expected:
                return
            self.closed.wait(.05)
        raise RuntimeError('upper_bridge_input_owner_unconfirmed')

    def allowed(self):
        with self.bundle.tracker.lock:
            active = self.bundle.tracker._active.get('base')
            return bool(active and active[1] in ('base_drive','base_servo')
                and not self.bundle._closing and self.bundle._transition is None
                and not self.bundle.state.motion_blocked
                and self.bundle._upper_route_key == (self.bundle._stop_epoch,self.bundle.ros1.connection)
                and self.bundle.state.read('upper_velocity_enabled')[0] is True
                and not self.bundle.spatial.mapping_active())

    def owner(self):
        with self.bundle.tracker.lock:
            b=self.bundle
            active=b.tracker._active.get('base')
            transition=b._transition
            boot_final = (transition is not None and transition.get('tool') == 'boot_remote_ready'
                          and not b.state.motion_blocked
                          and b._upper_route_key == (b._stop_epoch,b.ros1.connection))
            if transition is not None and not boot_final:
                if (transition.get('tool') == 'boot_remote_ready'
                        and transition.get('boot_zero_input') is True):
                    return 'card'  # Input only; allowed() still blocks output.
                return 'card' if transition.get('tool') in ('base_drive','base_servo','remote_control') else 'hold'
            if active:
                return 'card' if active[1] in ('base_drive','base_servo') else 'hold'
            try:enabled=b.state.confirmed_enable()
            except RuntimeError:enabled=False
            if (not b._closing and not b.state.motion_blocked and not b.spatial.unresolved()
                    and not b.spatial.mapping_active() and enabled is True
                    and b.state.read('upper_velocity_enabled')[0] is True
                    and b._upper_route_key==(b._stop_epoch,b.ros1.connection)):
                return 'remote'
            return 'hold'

    def _run(self):
        stream, stop_confirmed = None, False
        try:
            stream = FactoryKeyboardStream(self.bundle.spatial.client, bridge=True)
            reply = stream.read(12)
            if reply.get('state') != 'held':raise RuntimeError('upper_bridge_handshake_failed')
            seq = 0
            while not self.stop_event.is_set():
                owner=self.owner()
                stream.send({'nonce':reply['nonce'],'seq':seq,'allow':self.allowed() or owner=='remote','owner':owner})
                reply = stream.read(1.)
                if (reply.get('state')=='stopped' and reply.get('stationary') is True
                        and reply.get('bridge_zero_sent') is True):
                    stop_confirmed=True
                    raise RuntimeError('upper_bridge_remote_stopped')
                if reply.get('state') != 'ready' or reply.get('seq') != seq:
                    raise RuntimeError('upper_bridge_lease_failed')
                if reply.get('forwarded_nonzero'):self.forwarded += 1
                self.remote_state=reply.get('remote_state','unavailable')
                self.input_owner=reply.get('input_owner')
                self.input_ack_count += 1
                if reply.get('input_owner')==owner:
                    self.ready.set()
                seq += 1
                self.stop_event.wait(.04)
        except Exception as exc:
            self.error = str(exc) if str(exc).startswith(('upper_bridge_','factory_keyboard_')) else type(exc).__name__
            self.ready.set()
            # The chassis helper zeros independently. Prevent further driver
            # publication once an established stream fails.
            with self.bundle.tracker.lock:
                self.bundle._upper_route_key = None
        finally:
            if stream is not None:
                try:
                    stream.stop()
                    if not stop_confirmed:
                        reply = stream.read(8.)
                        stop_confirmed = (reply.get('state') == 'stopped' and reply.get('stationary') is True
                                          and reply.get('bridge_zero_sent') is True)
                    if not stop_confirmed:self.error = 'upper_bridge_stop_unconfirmed'
                except Exception:
                    self.error = self.error or 'upper_bridge_stop_unconfirmed'
                finally:
                    stream.close()
                    if not stop_confirmed:
                        with self.bundle.tracker.lock:
                            self.bundle.state.motion_blocked = True
                            if self.bundle.spatial._preparation is None:
                                self.bundle.spatial._preparation = {'delivery_uncertain':False,
                                    'stage':'upper_bridge_stop_unconfirmed'}
                                self.bundle.spatial._persist()

    def info(self):
        return {'state':'error' if self.error else 'ready' if self.ready.is_set() else 'idle',
                'error':self.error,'forwarded_nonzero_frames':self.forwarded,
                'remote_state':self.remote_state,
                'remote_return_error':self.remote_error,
                'watchdog_ms':300}

    def close(self):
        self.closed.set()
        self.stop_event.set()
        if self.thread and self.thread is not threading.current_thread():self.thread.join(timeout=10)
        if self.monitor_thread:self.monitor_thread.join(timeout=1)
        if self.return_thread:self.return_thread.join(timeout=1)
