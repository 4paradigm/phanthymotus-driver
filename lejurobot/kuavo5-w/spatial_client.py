"""JaTen map protocol + native, task-ID-based Leju navigation.

ROS calls use existing SSH configuration and the generated classes on the robot.
Map commands use the same request/result protocol as the factory map editor.
No motion, map changes or service restarts occur during construction/probing.
"""
import json
from contextlib import closing
import os
import socket
from pathlib import Path
import time
import urllib.request
from uuid import uuid4


from bootstrap import load_secrets


class NavigationStateUnavailable(RuntimeError):
    """Bounded diagnostics for a read-only, retryable state acquisition."""
    def __init__(self, missing, reason='timeout'):
        self.missing = tuple(sorted(missing))
        self.reason = reason
        super().__init__('fresh_navigation_state_unavailable: missing=' + ','.join(self.missing)
                         + '; reason=' + reason)


class JatenClient:
    COMMANDS = {'ChangeMode', 'StartCartoMapping', 'SaveCartoMap',
                'ChangeCartoMap', 'ChangeRoadMap', 'AgvSystemRestart'}

    def __init__(self, provisioner):
        self.provisioner = provisioner
        address = provisioner._target('chassis')['address']
        address = '[' + address + ']' if ':' in address else address
        self.base = 'http://' + address + ':8888'
        self.ws_url = 'ws://' + address + ':8888/ws/home'
        self._http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self._source = (Path(__file__).parent / 'host/navigation_rpc.py').read_text()

    def native(self, operation, **args):
        manager = self.provisioner
        target = manager._target('chassis')
        secrets = load_secrets(os.environ.get('KUAVO_SSH_SECRETS', str(manager.state_dir / 'ssh-secrets.json')))
        request = {'operation': operation, 'args': args, 'ros_ip': target['ros_ip'],
                   'master': manager.settings.get('master', 'http://192.168.26.1:11311')}
        response = manager.transport.run('chassis', target, request, self._source, secrets, timeout=45 if operation in ('prepare', 'soft_stop') else 25 if operation == 'cancel' else 15)
        result = response.get('result')
        if not isinstance(result, dict):
            raise RuntimeError('invalid_native_navigation_response')
        return result

    def http(self, path, body=None):
        request = urllib.request.Request(self.base + path, data=body,
                                         headers={'Content-Type': 'application/json'})
        with self._http.open(request, timeout=5) as response:
            raw = response.read(2_000_001)
        if len(raw) > 2_000_000:
            raise RuntimeError('oversized_map_response')
        result = json.loads(raw)
        if not isinstance(result, dict) or result.get('success') is not True or result.get('code') != 200:
            raise RuntimeError('map_api_request_failed')
        return result.get('data')

    def maps(self):
        result = self.http('/map/list?type=pgm', b'')
        if not isinstance(result, list):
            raise RuntimeError('invalid_grid_map_list')
        return result

    def current_map(self):
        result = self.http('/map/current?returnMapData=false')
        if not isinstance(result, dict) or not result.get('mapId') or not result.get('mapMd5'):
            raise RuntimeError('current_map_identity_unavailable')
        return result

    def map_identity(self):
        before = self.current_map()
        fingerprint = self.native('map_identity', map_name=before['mapId']).get('sha256')
        if (not isinstance(fingerprint, str) or len(fingerprint) != 64
                or any(c not in '0123456789abcdef' for c in fingerprint)):
            raise RuntimeError('grid_map_identity_unavailable')
        if self.current_map() != before:
            raise RuntimeError('map_changed_during_identity_read')
        return before['mapId'], fingerprint + ':' + before['mapMd5']

    @staticmethod
    def _frame(raw):
        if not isinstance(raw, str) or len(raw) > 2_000_000:
            raise RuntimeError('invalid_navigation_websocket_frame')
        value = json.loads(raw)
        if not isinstance(value, dict):
            raise RuntimeError('invalid_navigation_websocket_frame')
        return value

    def observe(self, required=None):
        # Base readiness needs fresh faults/driving, not map localization. A
        # navigation request still uses all four fields by default.
        import websocket
        required = frozenset(required or ('pose2d', 'agvState', 'state', 'localizationStatus'))
        supported = {'pose2d', 'agvState', 'state', 'localizationStatus'}
        if not required <= supported:
            raise ValueError('unsupported_navigation_state_field')
        seen, received, end = {}, {}, time.monotonic() + 5
        def missing():
            now = time.monotonic()
            return required - {key for key, at in received.items() if now-at <= 2.}
        try:
            # Fresh socket per snapshot; never accept a prior connection cache.
            with closing(websocket.create_connection(self.ws_url, timeout=2, http_proxy_host=None)) as ws:
                while time.monotonic() < end:
                    try:
                        frame = self._frame(ws.recv())
                    except websocket.WebSocketTimeoutException:
                        continue
                    key, data = frame.get('type'), frame.get('data')
                    if key == 'state' and isinstance(data, dict):
                        seen[key] = data
                        received[key] = time.monotonic()
                    elif key in ('pose2d', 'agvState', 'localizationStatus') and isinstance(data, dict):
                        if isinstance(data.get('data'), dict):
                            seen[key] = data['data']
                            received[key] = time.monotonic()
                    if not missing():
                        return seen
        except (OSError, socket.timeout, websocket.WebSocketException):
            raise NavigationStateUnavailable(missing(), 'connection') from None
        raise NavigationStateUnavailable(missing())

    def command(self, method, params, timeout=25):
        import websocket
        if method not in self.COMMANDS:
            raise ValueError('unsupported_map_command')
        request_id = uuid4().hex
        end = time.monotonic() + timeout
        with closing(websocket.create_connection(self.ws_url, timeout=2, http_proxy_host=None)) as ws:
            ws.send(json.dumps({'method': method, 'id': request_id, 'params': params}))
            while time.monotonic() < end:
                try:
                    frame = self._frame(ws.recv())
                except websocket.WebSocketTimeoutException:
                    continue
                data = frame.get('data', {})
                if (frame.get('type') == 'taskResult' and isinstance(data, dict)
                        and data.get('id') == request_id and data.get('method') == method):
                    if data.get('error'):
                        raise RuntimeError('factory_map_command_rejected')
                    return data
        # Never retry a mutation: a lost acknowledgement is uncertain delivery.
        raise TimeoutError('factory_map_command_result_unconfirmed')
