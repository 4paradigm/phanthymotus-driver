"""README_dev ACP: one terminal HTTP POST from the action worker."""
import json
import logging
import ssl
import urllib.request
from urllib.parse import urlsplit


def complete(action_id, status, result, tool):
    import os as _os
    import time
    url = _os.environ.get('AGENT_CORE_URL', 'https://localhost:15678')
    context = ssl.create_default_context()
    # Agent Core's local endpoint uses its platform-provisioned self-signed cert.
    # Remote Core deployments retain ordinary CA and hostname validation.
    if urlsplit(url).hostname in ('localhost', '127.0.0.1', '::1'):
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    payload = json.dumps({'action_id': action_id, 'status': status, 'result': result,
                          'tool': tool, 'ts': time.time()}, allow_nan=False).encode()
    request = urllib.request.Request(url.rstrip('/') + '/api/acp/complete', data=payload,
        headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urllib.request.urlopen(request, timeout=5, context=context):
            pass
    except Exception as error:
        # No driver-side receipt/outbox protocol; Core owns the pending barrier.
        logging.warning('ACP completion POST failed (%s)', type(error).__name__)
