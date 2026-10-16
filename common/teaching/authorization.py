"""Read saved execution policy or verify a one-use local Core UI approval.

The MCP caller cannot create an approval by setting booleans. Core owns the
authenticated approval endpoint; Driver owns its scope and execution lifetime.
This uses the existing same-host Core/Driver deployment boundary.
"""
import http.client
import json
import os
import re
import ssl
from urllib.parse import urlsplit


def consume_approval(approval_id):
    if not isinstance(approval_id, str) or not re.fullmatch('[a-f0-9]{64}', approval_id):
        raise ValueError('invalid_session_approval')
    return _core_request('/api/skills/package/approval/consume', {'approval_id': approval_id})


def read_execution_policy(tool, instance_id, device_binding, asset_id='', asset_binding=''):
    return _core_request('/api/skills/package/execution/read', {
        'tool': tool, 'instance_id': instance_id, 'device_binding': device_binding,
        'asset_id': asset_id, 'asset_binding': asset_binding})


def _core_request(path, body):
    url = urlsplit(os.environ.get('AGENT_CORE_URL', 'https://localhost:15678'))
    if (url.hostname not in ('localhost', '127.0.0.1', '::1') or url.username or url.password
            or url.path not in ('', '/') or url.query or url.fragment or url.scheme not in ('http', 'https')):
        raise ValueError('session_approval_requires_local_core')
    kwargs = {'timeout': 3}
    if url.scheme == 'https':
        # Core uses a self-signed certificate on the host loopback interface.
        kwargs['context'] = ssl._create_unverified_context()
    cls = http.client.HTTPSConnection if url.scheme == 'https' else http.client.HTTPConnection
    connection = cls(url.hostname, url.port, **kwargs)
    try:
        connection.request('POST', path, json.dumps(body), {'Content-Type': 'application/json'})
        response = connection.getresponse()
        raw = response.read(4097)
        if response.status != 200 or len(raw) > 4096:
            raise ValueError('session_approval_rejected')
        return json.loads(raw)['data']
    finally:
        connection.close()
