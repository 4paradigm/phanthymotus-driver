"""Background image provisioning and fixed-operation service checks."""
import json
import threading
import urllib.request
from bootstrap import AutoProvisioner


class StartupChecks:
    def __init__(self, enabled=True, provisioning=None):
        self.enabled = enabled
        self.status = {'state': 'waiting' if enabled else 'disabled'}
        self._closed = threading.Event()
        self._thread = None
        self.provisioner = AutoProvisioner({**(provisioning or {}),
            'enabled': enabled and (provisioning or {}).get('enabled', True)})

    def start(self):
        self.provisioner.start()
        if self.enabled:
            self._thread = threading.Thread(target=self._run, daemon=True, name='kuavo-host-checks')
            self._thread.start()

    def _run(self):
        while not self._closed.is_set():
            try:
                request = urllib.request.Request('http://127.0.0.1:15748/ensure', data=b'{}',
                                                 headers={'Content-Type': 'application/json'})
                # Ignore proxy variables for the trusted loopback manager.
                with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(request, timeout=4) as response:
                    result = json.loads(response.read(4096))
                ready = result.get('requested') and result.get('rosbridge_listening')
                self.status = {**result, 'state': 'ready' if ready else 'starting',
                               'provisioning': self.provisioner.snapshot()}
            except Exception:
                self.status = {'state': 'starting', 'error': 'host services unavailable; inspect provisioning targets',
                               'provisioning': self.provisioner.snapshot()}
            self._closed.wait(5)

    def ensure_camera(self, card):
        # Called by a sensor start/recovery worker, never by the ROS receiver or
        # globally for inactive camera cards. No arbitrary service/unit names.
        scope = {'camera_head': 'head', 'camera_depth': 'depth'}[card]
        self.provisioner.request('host')
        request = urllib.request.Request(f'http://127.0.0.1:15748/ensure/{scope}', data=b'{}',
                                         headers={'Content-Type': 'application/json'})
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(request, timeout=4) as response:
            result = json.loads(response.read(4096))
        if result.get('requested') is not True:
            raise RuntimeError('head camera provisioning is not ready; inspect host_runtime.provisioning')

    def ensure_speaker(self):
        self.provisioner.request('chassis')

    def close(self):
        self._closed.set()
        self.provisioner.close()
        if self._thread:
            self._thread.join(timeout=5)
