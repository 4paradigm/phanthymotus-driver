"""Bounded background host checks. Never launches/enables motion controllers."""
import json
import threading
import urllib.request


class StartupChecks:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.status = {'state': 'waiting' if enabled else 'disabled'}
        self._closed = threading.Event()
        self._thread = None

    def start(self):
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
                self.status = {**result, 'state': 'ready' if ready else 'starting'}
            except Exception:
                self.status = {'state': 'unavailable', 'error': 'Install host/install.py once on the ROS1 host; no controller auto-start'}
            self._closed.wait(5)

    def ensure_camera(self, card):
        # Called by a sensor start/recovery worker, never by the ROS receiver or
        # globally for inactive camera cards. No arbitrary service/unit names.
        scope = {'camera_head': 'head', 'camera_depth': 'depth'}[card]
        request = urllib.request.Request(f'http://127.0.0.1:15748/ensure/{scope}', data=b'{}',
                                         headers={'Content-Type': 'application/json'})
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(request, timeout=4) as response:
            result = json.loads(response.read(4096))
        if result.get('requested') is not True:
            raise RuntimeError('head camera host services unavailable; run host/install.py once')

    def close(self):
        self._closed.set()
        if self._thread:
            self._thread.join(timeout=5)
