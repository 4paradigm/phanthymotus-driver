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

    def close(self):
        self._closed.set()
        if self._thread:
            self._thread.join(timeout=5)
