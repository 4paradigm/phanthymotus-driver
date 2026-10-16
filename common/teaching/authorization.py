"""Verify a one-use approval made through the authenticated local Core UI.

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
        connection.request('POST', '/api/skills/package/approval/consume',
                           json.dumps({'approval_id': approval_id}), {'Content-Type': 'application/json'})
        response = connection.getresponse()
        raw = response.read(4097)
        if response.status != 200 or len(raw) > 4096:
            raise ValueError('session_approval_rejected')
        return json.loads(raw)['data']
    finally:
        connection.close()
