"""Correlated Go1 SDK RPC with an independent, persistent stop latch."""
import multiprocessing
import queue
import threading
import time
import uuid


class SdkError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def _execute(client, request, stop_signal, epoch):
    name, args = request['cmd'], request.get('args', [])
    kwargs = request.get('kwargs', {})
    if time.monotonic() >= request['deadline']:
        raise SdkError('SDK_TIMEOUT', 'Expired request was not applied')
    if name in ('snapshot', 'diagnostics', 'desired_gait'):
        return getattr(client, name)(*args, **kwargs)
    # Only short memory writes here; never wait for network under this lock.
    with epoch.get_lock():
        if request['epoch'] != epoch.value:
            raise SdkError('CANCELLED', 'Command superseded by stop')
        if not client.available:
            raise SdkError('STUB_MODE', 'SDK unavailable')
        owner = request.get('owner')
        current = getattr(client, '_control_owner', None)
        if name == 'claim_control':
            if current and current != owner:
                raise SdkError('RESOURCE_BUSY', 'Another loco action owns the driver')
            if not current and client.snapshot().get('commanded_motion', {}).get('active'):
                raise SdkError('RESOURCE_BUSY', 'Existing card has an active movement')
            client.stop_move()
            client._control_owner = owner
            stop_signal.clear()
            return {'accepted': True}
        if name == 'release_control':
            if current == owner:
                client._control_owner = None
            return {'released': current == owner}
        if stop_signal.is_set():
            raise SdkError('STOP_LATCHED', 'Start a new loco action to rearm')
        if current and current != owner:
            raise SdkError('RESOURCE_BUSY', 'loco action owns the driver')
        if name not in ('move', 'stop_move', 'set_posture', 'set_gait'):
            raise SdkError('UNKNOWN_COMMAND', name)
        return getattr(client, name)(*args, **kwargs)


def _sdk_worker(cmd_q, result_q, network_iface, target_ip, target_port,
                local_port, stop_signal, epoch):
    # spawn starts a fresh interpreter; protect Python logs before SDK import.
    from common import logsafe
    logsafe.install(check_fd=False)
    from go1_sdk_client import Go1HighSdkClient
    client = Go1HighSdkClient(network_iface, target_ip, target_port, local_port)
    client._stop_signal = stop_signal
    client._control_owner = None
    client.start()
    result_q.put({'available': client.available})
    try:
        while True:
            request = cmd_q.get()
            if request is None:
                break
            reply = {'id': request['id']}
            try:
                reply['result'] = _execute(client, request, stop_signal, epoch)
            except Exception as exc:
                reply.update(error=str(exc), code=getattr(exc, 'code', 'SDK_ERROR'))
            result_q.put(reply)
    finally:
        stop_signal.set()
        client.stop()


class SdkProxy:
    def __init__(self, network_iface='', target_ip='192.168.123.161',
                 target_port=8082, local_port=8090):
        ctx = multiprocessing.get_context('spawn')
        self._cmd_q, self._result_q = ctx.Queue(), ctx.Queue()
        self._stop_signal = ctx.Event()
        self._epoch = ctx.Value('Q', 0)
        self._lock = threading.Lock()
        self._stopped = False
        self._proc = ctx.Process(target=_sdk_worker, args=(
            self._cmd_q, self._result_q, network_iface, target_ip, target_port,
            local_port, self._stop_signal, self._epoch), daemon=True,
            name='go1_sdk_worker')
        self._proc.start()
        try:
            self.available = self._result_q.get(timeout=15).get('available', False)
        except queue.Empty:
            self.available = False

    def start(self):
        pass

    @property
    def control_epoch(self):
        return self._epoch.value

    def request_stop(self):
        """Request only, NOT physical stop confirmation. Bypasses the RPC lock."""
        with self._epoch.get_lock():
            self._epoch.value += 1
            self._stop_signal.set()
        return {'accepted': True, 'stop_confirmed': False}

    def stop(self):
        self.request_stop()
        self._stopped = True
        self._cmd_q.put(None)
        self._proc.join(timeout=3)

    def _call(self, cmd, args=None, kwargs=None, timeout=1., owner=None, epoch=None):
        deadline = time.monotonic() + timeout
        generation = self.control_epoch if epoch is None else epoch
        if not self._lock.acquire(timeout=timeout):
            raise SdkError('SDK_TIMEOUT', 'SDK queue busy')
        try:
            if self._stopped or not self._proc.is_alive():
                raise SdkError('SDK_UNAVAILABLE', 'SDK worker stopped')
            rid = uuid.uuid4().hex
            self._cmd_q.put({'id': rid, 'cmd': cmd, 'args': args or [],
                             'kwargs': kwargs or {}, 'owner': owner,
                             'epoch': generation, 'deadline': deadline})
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise SdkError('SDK_TIMEOUT', cmd + ' timed out; execution unknown')
                try:
                    reply = self._result_q.get(timeout=remaining)
                except queue.Empty as exc:
                    raise SdkError('SDK_TIMEOUT', cmd + ' timed out; execution unknown') from exc
                if reply.get('id') != rid:
                    continue
                if 'error' in reply:
                    raise SdkError(reply.get('code', 'SDK_ERROR'), reply['error'])
                return reply['result']
        finally:
            self._lock.release()

    def snapshot(self):
        try:
            return self._call('snapshot', timeout=0.25)
        except SdkError as exc:
            return {'fresh': False, 'error': exc.code}

    def diagnostics(self):
        return self._call('diagnostics')

    def claim_control(self, owner, epoch):
        return self._call('claim_control', owner=owner, epoch=epoch)

    def release_control(self, owner):
        return self._call('release_control', owner=owner)

    def control_move(self, owner, epoch, vx, vy, vyaw, until):
        return self._call('move', [vx, vy, vyaw, 1], kwargs={'until': until},
                          owner=owner, epoch=epoch, timeout=0.25)

    def control_posture(self, owner, epoch, mode):
        return self._call('set_posture', [mode], owner=owner, epoch=epoch, timeout=0.25)

    def move(self, vx=0., vy=0., vyaw=0., gait=None):
        return self._call('move', [vx, vy, vyaw, gait])

    def stop_move(self):
        return self._call('stop_move')

    def set_posture(self, mode, euler=(0., 0., 0.), body_height=0., foot_raise=0., speed_level=0):
        return self._call('set_posture', [mode, euler, body_height, foot_raise, speed_level])

    def set_gait(self, gait):
        return self._call('set_gait', [int(gait)])

    def desired_gait(self):
        return self._call('desired_gait')
