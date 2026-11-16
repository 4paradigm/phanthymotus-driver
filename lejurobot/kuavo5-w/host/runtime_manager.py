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


def ensure_services():
    with _lock:
        units = list(SERVICES)
        for unit in ('kuavo5w-head-camera.service', 'kuavo5w-head-jpeg-relay.service'):
            if Path('/etc/systemd/system/' + unit).is_file():
                units.append(unit)
        result = subprocess.run(['systemctl', 'start', '--no-block', *units],
                                capture_output=True, timeout=3)
        try:
            with socket.create_connection(('127.0.0.1', 9090), timeout=.5):
                connected = True
        except OSError:
            connected = False
        return {'requested': result.returncode == 0, 'rosbridge_listening': connected}


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        self.request.settimeout(4)
        super().setup()

    def do_POST(self):
        if (self.client_address[0] != '127.0.0.1' or self.headers.get_all('Origin')
                or self.headers.get_all('Host') not in ([f'127.0.0.1:{PORT}'], [f'localhost:{PORT}'])
                or self.path != '/ensure' or self.headers.get('Content-Type') != 'application/json'
                or self.headers.get_all('Content-Length') != ['2'] or self.headers.get('Transfer-Encoding')):
            self.send_error(403)
            return
        if self.rfile.read(2) != b'{}':
            self.send_error(400)
            return
        try:
            payload, status = ensure_services(), 200
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
