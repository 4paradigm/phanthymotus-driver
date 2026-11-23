#!/usr/bin/env python3
"""Loopback-only, fixed-operation host startup helper. No arbitrary commands."""
import json
import socket
import subprocess
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 15748
SERVICES = ('kuavo5w-rosbridge.service', 'kuavo5w-battery-relay.service')
_lock = threading.Lock()


CAMERA_SERVICES = {
    'head': ('kuavo5w-head-camera.service', 'kuavo5w-head-jpeg-relay.service', 'kuavo5w-head-imu-relay.service'),
    'depth': ('kuavo5w-head-camera.service', 'kuavo5w-head-jpeg-relay.service', 'kuavo5w-head-imu-relay.service'),
}


def ensure_services(scope='base'):
    with _lock:
        if scope == 'base':
            units = list(SERVICES)
        elif scope in CAMERA_SERVICES:
            units = [SERVICES[0], *CAMERA_SERVICES[scope]]
            if not all(Path('/etc/systemd/system/' + unit).is_file() for unit in CAMERA_SERVICES[scope]):
                return {'requested': False, 'error': 'camera_services_not_installed'}
        else:
            raise ValueError('unsupported fixed service scope')
        # systemctl start is idempotent: active services are left running.
        result = subprocess.run(['systemctl', 'start', '--no-block', *units],
                                capture_output=True, timeout=3)
        active = subprocess.run(['systemctl', 'is-active', '--quiet', *units],
                                capture_output=True, timeout=1).returncode == 0
        try:
            with socket.create_connection(('127.0.0.1', 9090), timeout=.5):
                connected = True
        except OSError:
            connected = False
        return {'requested': result.returncode == 0, 'services_active': active,
                'scope': scope, 'rosbridge_listening': connected}


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        self.request.settimeout(4)
        super().setup()

    def do_POST(self):
        if (self.client_address[0] != '127.0.0.1' or self.headers.get_all('Origin')
                or self.headers.get_all('Host') not in ([f'127.0.0.1:{PORT}'], [f'localhost:{PORT}'])
                or self.path not in ('/ensure', '/ensure/head', '/ensure/depth')
                or self.headers.get('Content-Type') != 'application/json'
                or self.headers.get_all('Content-Length') != ['2'] or self.headers.get('Transfer-Encoding')):
            self.send_error(403)
            return
        if self.rfile.read(2) != b'{}':
            self.send_error(400)
            return
        try:
            payload = ensure_services() if self.path == '/ensure' else ensure_services(self.path.rsplit('/', 1)[1])
            status = 200
        except (OSError, subprocess.SubprocessError):
            payload, status = {'requested': False, 'error': 'host_service_start_failed'}, 503
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass  # Caller reports state transitions; do not log polling requests.


if __name__ == '__main__':
    ThreadingHTTPServer(('127.0.0.1', PORT), Handler).serve_forever()
