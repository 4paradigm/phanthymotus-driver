"""One-shot, persistent first-boot handoff to G12; never a recovery loop.

The controller boot owner still initializes and pauses. This worker can consume
one fresh boot permit, under the same transition/native locks as all cards.
An operator action, stop, reconnect, failed guard or container restart never
replays a consumed permit. No nonzero motion or posture targets are sent here.
"""
import threading
import time
from pathlib import Path
from uuid import UUID
from spatial_client import NavigationStateUnavailable
from bootstrap import ProvisionError


class BootRemote:
    WINDOW_S = 60.
    CHASSIS_READ_WAIT_S = 20.

    def __init__(self, bundle, enabled=False):
        if type(enabled) is not bool:
            raise ValueError('host_runtime.remote_ready_on_first_boot must be a YAML boolean')
        self.bundle, self.enabled = bundle, enabled
        self.closed = threading.Event()
        self.cancelled = False
        self.thread = None
        self.status = {'state': 'waiting' if enabled else 'disabled'}
        self.token = None
        self.connection = None
        self.deadline = None
        self.stage = 'waiting_for_controller_boot'
        self.enrolled = not enabled
        if enabled:
            # Bind the automatic attempt before MCP starts accepting requests,
            # including restarts that occur before controller feedback arrives.
            try:
                host_boot = str(UUID(Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
                self.enrolled = bundle.spatial.store.claim_boot_remote('driver-host:' + host_boot)
                if not self.enrolled:
                    self.status = {'state': 'skipped', 'reason': 'driver_already_started_this_host_boot'}
            except Exception:
                self.status = {'state': 'skipped', 'reason': 'persistent_host_boot_permit_unavailable'}

    def info(self):
        with self.bundle.tracker.lock:
            return dict(self.status)

    def cancel(self, reason):
        # Only flags/Event.set and atomic reference assignments: lifecycle-only
        # callers can cancel without waiting for tracker.lock. Never do IO here.
        self.cancelled = True
        token = self.token
        if token is not None:
            token['cancelled'].set()
        if self.enabled and self.status['state'] not in ('ready', 'error', 'skipped'):
            self.status = {'state': 'skipped', 'reason': reason}

    @staticmethod
    def permit(boot):
        if not boot or boot.get('state') != 'complete':
            return None
        try:
            identity = str(UUID(boot['boot_id']))
        except (KeyError, ValueError, TypeError, AttributeError):
            return None
        if (boot.get('launch_ownership') != 'service' or not boot.get('initialization_requested')
                or not boot.get('initialization_accepted') or not boot.get('boot_pause_confirmed')
                or boot.get('initial_control_mode') != 3):
            return None
        return identity

    def _check(self, token):
        b = self.bundle
        b._check_transition(token)
        if self.closed.is_set() or self.cancelled:
            raise InterruptedError('boot_remote_preparation_cancelled')
        b.state.require_motion_enabled()
        b.sensors.require_profile()
        boot = b.state.snapshot()['controller_boot']
        if (self.permit(boot) != token['boot_id'] or boot['age_s'] > 1.
                or type(boot.get('ready_age_s')) not in (int, float)
                or not 0 <= boot['ready_age_s'] <= self.WINDOW_S
                or (self.deadline is not None and time.monotonic() > self.deadline)
                or not isinstance(boot.get('g12_owner'), str) or not boot['g12_owner'].startswith('/')
                or boot.get('g12_neutral') is not True
                or type(boot.get('g12_age_s')) not in (int, float) or not 0 <= boot['g12_age_s'] <= .3
                or b.spatial.mapping_active() or b.spatial._map_pending is not None
                or b.spatial._task is not None or b.spatial._keyboard_pending
                or b.tracker._active):
            raise RuntimeError('boot_remote_guard_changed')

    def _readiness(self, boot):
        """All asynchronous read-only dependencies share the same boot window."""
        if boot['age_s'] > 1.:
            return 'waiting_for_fresh_boot_feedback'
        try:
            self.bundle.state.confirmed_enable()
        except RuntimeError:
            return 'waiting_for_enable_verification'
        try:
            self.bundle.sensors.require_profile()
        except RuntimeError:
            return 'waiting_for_robot_identity'
        if (not isinstance(boot.get('g12_owner'), str) or not boot['g12_owner'].startswith('/')
                or type(boot.get('g12_age_s')) not in (int, float)
                or not 0 <= boot['g12_age_s'] <= .3 or boot.get('g12_neutral') is not True):
            return 'waiting_for_verified_neutral_g12'
        return None

    def _await_prewrite(self, token):
        b = self.bundle
        while True:
            with b.tracker.lock:
                b._check_transition(token)
                if self.closed.is_set() or self.cancelled:
                    raise InterruptedError('boot_remote_preparation_cancelled')
                boot = b.state.snapshot()['controller_boot']
                if (self.permit(boot) != token['boot_id'] or self.deadline is None
                        or time.monotonic() > self.deadline):
                    raise RuntimeError('boot_remote_prewrite_window_expired_or_boot_changed')
                stage = self._readiness(boot)
                if stage is None:
                    self._check(token)
                    return
                self.status = {'state': 'preparing', 'stage': stage, 'motion_target_sent': False}
            self.closed.wait(.2)

    def attempt(self):
        """Return False only while waiting for the first terminal boot report."""
        b = self.bundle
        with b.tracker.lock:
            if not self.enabled or not self.enrolled:
                return True
            connection = b.ros1.connection
            if self.connection is None and connection[0]:
                self.connection = connection
            elif self.connection is not None and connection != self.connection:
                self.status = {'state': 'skipped', 'reason': 'control_connection_changed_no_automatic_retry'}
                return True
            boot = b.state.snapshot()['controller_boot']
            identity = self.permit(boot)
            if identity is None:
                if boot and boot.get('state') in ('complete', 'failed'):
                    self.status = {'state': 'skipped', 'reason': 'fresh_boot_permit_unavailable'}
                    return True
                return False
            age = boot.get('ready_age_s')
            if type(age) in (int, float) and 0 <= age <= self.WINDOW_S:
                proposed = time.monotonic() + self.WINDOW_S - age
                self.deadline = proposed if self.deadline is None else min(self.deadline, proposed)
            # The boot report and enable-verification probe are independent
            # subscribers. Receiving COMPLETE first is not a failed handoff.
            # Keep waiting read-only without consuming the controller permit.
            if (not self.cancelled and not self.closed.is_set()
                    and type(age) in (int, float) and 0 <= age <= self.WINDOW_S
                    and (self.deadline is None or time.monotonic() <= self.deadline)
                    and b._transition is None and not b.spatial.unresolved()
                    and not b.spatial.mapping_active() and not b.tracker._active):
                stage = self._readiness(boot)
                if stage:
                    self.status = {'state': 'waiting', 'stage': stage,
                                   'motion_target_sent': False}
                    return False
            # An INSERT OR IGNORE transaction claims BEFORE all control writes.
            # Persist skips as well: operator actions may precede boot feedback.
            if not b.spatial.store.claim_boot_remote(identity):
                self.status = {'state': 'skipped', 'reason': 'boot_permit_already_consumed'}
                return True
            if (self.cancelled or self.closed.is_set() or type(age) not in (int, float)
                    or not 0 <= age <= self.WINDOW_S or boot['age_s'] > 1.
                    or (self.deadline is not None and time.monotonic() > self.deadline)
                    or b._transition is not None or b.spatial.unresolved()
                    or b.spatial.mapping_active() or b.tracker._active):
                self.status = {'state': 'skipped', 'reason': 'boot_window_expired_or_control_already_used'}
                return True
            token = None
            try:
                # Never resume an already-enabled/manual/external controller.
                if b.state.confirmed_enable() is not False:
                    raise RuntimeError('boot_remote_requires_confirmed_initial_pause')
                token, _, was_blocked = b._begin_control('boot_remote_ready')
                token['was_blocked'] = was_blocked
                token['boot_id'], token['tool'] = identity, 'boot_remote_ready'
                self.token = token
                self._check(token)
                self.status = {'state': 'preparing', 'message': 'Preparing G12 remote control; no motion target is sent.'}
            except Exception as exc:
                if token is not None:
                    b.state.motion_blocked = token['was_blocked']
                    b._end_control(token)
                self.token = None
                self.status = {'state': 'skipped', 'reason': str(exc)[:200]}
                return True
        writes_attempted = False
        try:
            # The two ROS masters have no native velocity link. Establish the
            # held bridge before enabling; lifecycle start must not be needed
            # to make the first remote command reach the chassis.
            self.stage = 'establish_velocity_bridge'
            self.status.update(stage=self.stage)
            b.upper_bridge.ensure()
            self._await_prewrite(token)
            with b.spatial._native_lock:
                # The chassis web service may become ready after the controller.
                # Retry only observation, inside this single claimed attempt;
                # never replay a hold, enable, mux switch or release operation.
                deadline = time.monotonic() + self.CHASSIS_READ_WAIT_S
                while True:
                    self._await_prewrite(token)
                    try:
                        observation = b.spatial.client.observe(required=('agvState', 'state'))
                        break
                    except NavigationStateUnavailable:
                        # The factory web UI may lag behind the actual chassis.
                        # This fallback only reads ROS health and odometry; it
                        # performs no mode, hold, enable or task mutation.
                        fallback = getattr(b.spatial.client, 'observe_remote_chassis', None)
                        if callable(fallback):
                            try:
                                observation = fallback()
                                break
                            except (NavigationStateUnavailable, ProvisionError) as exc:
                                # Only transport/read readiness failures retry.
                                # Explicit health/mode rejection is terminal.
                                if (isinstance(exc, ProvisionError) and str(exc) not in
                                        ('ssh_operation_timeout', 'ssh_auth_host_key_or_remote_access_failed')):
                                    raise
                                if time.monotonic() >= deadline:
                                    raise
                        if time.monotonic() >= deadline:
                            raise
                        self.status = {'state': 'preparing', 'stage': 'waiting_for_chassis_feedback',
                                       'motion_target_sent': False}
                        self.closed.wait(.25)
                b.spatial._check_chassis_faults(observation)
                if (observation['agvState'].get('isMapping') is not False
                        or observation['agvState'].get('runningMode') != 'AUTO'
                        or observation['state'].get('driving') is not False):
                    raise RuntimeError('boot_remote_requires_idle_auto_nonmapping_chassis')
                self._await_prewrite(token)
                with b.tracker.lock:
                    self._check(token)
                    writes_attempted = True
                    self.stage = 'hold_and_cancel_old_chassis_task'
                    b.spatial._preparation = {'delivery_uncertain': True, 'owner': 'boot_remote_ready'}
                    b.spatial._persist()
                held = b.spatial.client.native('soft_stop')
                if not (held.get('success') is True and held.get('stationary') is True
                        and held.get('software_estop') is True):
                    raise RuntimeError('boot_remote_chassis_hold_unconfirmed')
                with b.tracker.lock:
                    self._check(token)
                self.stage = 'disable_upper_velocity'
                b.confirm_control(token, '/enable_vel_control', 'std_srvs/SetBool',
                    {'data': False}, 'success', 'upper_velocity_enabled', False)
                with b.tracker.lock:
                    self._check(token)
                    b.twist('/cmd_vel')
                issued = time.monotonic()
                self.stage = 'enable_initialized_controller'
                b.confirm_control(token, '/enable_control', 'std_srvs/SetBool',
                    {'data': True}, 'success', 'control_enabled', True)
                self.stage = 'confirm_base_arm_mode'
                if b.wait_mode_feedback(token, issued) != 3:
                    raise RuntimeError('boot_remote_requires_initialized_base_arm')
                with b.tracker.lock:
                    self._check(token)
                self.stage = 'enable_upper_velocity'
                b.confirm_control(token, '/enable_vel_control', 'std_srvs/SetBool',
                    {'data': True}, 'success', 'upper_velocity_enabled', True)
                with b.tracker.lock:
                    self._check(token)
                    b.twist('/cmd_vel')
                # Chassis-local checks gate the FINAL release: AUTO, no mapping,
                # fresh stationary odometry, zero upper output and fresh G12.
                # This RPC shares the native lock with stop/map/navigation.
                self.stage = 'verify_neutral_zero_and_release_chassis'
                result = b.spatial.client.native('remote_ready',
                    g12_owner=b.state.snapshot()['controller_boot'].get('g12_owner'))
                if not (result.get('success') is True and result.get('stationary') is True
                        and result.get('software_estop') is False and result.get('old_task_cancelled') is True):
                    raise RuntimeError('boot_remote_chassis_release_unconfirmed')
                with b.tracker.lock:
                    self._check(token)
                    if b.state.confirmed_enable() is not True or b.state.read('upper_velocity_enabled')[0] is not True:
                        raise RuntimeError('boot_remote_control_feedback_changed')
                    b.state.motion_blocked = False
                    b.spatial._preparation = None
                    b.spatial._persist()
                    b.spatial.base_preparation_required = False
                    b._upper_route_key = (b._stop_epoch, b.ros1.connection)
                self.stage = 'confirm_remote_input_and_velocity_bridge'
                def check_remote_ready():
                    with b.tracker.lock:
                        self._check(token)
                        if (b.state.confirmed_enable() is not True
                                or b.state.read('upper_velocity_enabled')[0] is not True):
                            raise RuntimeError('boot_remote_control_feedback_changed')
                b.upper_bridge.wait_remote_ready(check_remote_ready)
                with b.tracker.lock:
                    check_remote_ready()
                    self.status = {'state': 'ready', 'controller': 'g12_upper_mpc', 'motion_target_sent': False}
        except Exception as exc:
            if writes_attempted:
                b.pause('boot_remote_preparation_failed', expected_transition=token)
            with b.tracker.lock:
                if not writes_attempted and b._transition is token:
                    b.state.motion_blocked = token['was_blocked']
                self.status = {'state': 'error', 'stage': self.stage, 'control_writes_attempted': writes_attempted,
                               'error': str(exc)[:200], 'automatic_retry': False}
        finally:
            with b.tracker.lock:
                b._end_control(token)
                self.token = None
        return True

    def start(self):
        if not self.enabled or not self.enrolled or self.thread is not None:
            return
        def run():
            while not self.closed.is_set():
                try:
                    if self.attempt():
                        return
                except Exception as exc:
                    with self.bundle.tracker.lock:
                        self.status = {'state': 'error', 'error': str(exc)[:200], 'automatic_retry': False}
                    return
                self.closed.wait(.2)
        self.thread = threading.Thread(target=run, daemon=True, name='boot-remote-ready')
        self.thread.start()

    def close(self):
        self.closed.set()
        with self.bundle.tracker.lock:
            self.cancel('driver_shutdown')
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(5)
