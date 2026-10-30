"""S10-only Core registration with verified TLS; no motion responsibilities."""
from __future__ import annotations

import hashlib
import hmac
import http.client
import json
import logging
import os
import ssl
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit


class CoreRegistration:
    def __init__(self, base_url=None, *, ca_file=None, pinned_leaf_file=None):
        parsed = urlsplit(base_url or os.environ.get('AGENT_CORE_URL', 'https://localhost:15678'))
        if (parsed.scheme != 'https' or not parsed.hostname or parsed.username is not None
                or parsed.password is not None or parsed.query or parsed.fragment):
            raise ValueError('Core registration requires an HTTPS URL without credentials, query or fragment')
        self.host, self.port = parsed.hostname, parsed.port or 443
        self.path = parsed.path.rstrip('/') + '/api/mcp'
        ca_file = ca_file or os.environ.get('S10_REGISTRATION_CA_FILE') or os.environ.get('S10_ACP_CA_FILE')
        pinned_leaf_file = (pinned_leaf_file or os.environ.get('S10_REGISTRATION_PINNED_LEAF_FILE')
                            or os.environ.get('S10_ACP_PINNED_LEAF_FILE'))
        if ca_file and pinned_leaf_file:
            raise ValueError('Choose either CA verification or explicit leaf trust for registration')
        self.digest = None
        if pinned_leaf_file:
            pem = Path(pinned_leaf_file).read_text(encoding='ascii')
            if pem.count('-----BEGIN CERTIFICATE-----') != 1:
                raise ValueError('Registration pin must contain exactly one PEM certificate')
            self.digest = hashlib.sha256(ssl.PEM_cert_to_DER_cert(pem)).digest()
            self.context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
            # An operator-supplied exact leaf pin replaces hostname matching;
            # chain validation and exact peer matching remain mandatory.
            self.context.check_hostname = False
            self.context.verify_mode = ssl.CERT_REQUIRED
            self.context.load_verify_locations(cadata=pem)
        else:
            self.context = ssl.create_default_context(cafile=ca_file)

    def send(self, payload):
        connection = http.client.HTTPSConnection(self.host, self.port, timeout=3, context=self.context)
        try:
            connection.connect()
            if self.digest is not None:
                peer = connection.sock.getpeercert(binary_form=True)
                if not peer or not hmac.compare_digest(hashlib.sha256(peer).digest(), self.digest):
                    raise ssl.SSLCertVerificationError('Core registration leaf pin mismatch')
            body = json.dumps(payload, allow_nan=False).encode('utf-8')
            connection.request('POST', self.path, body=body, headers={'Content-Type': 'application/json'})
            response = connection.getresponse()
            if not 200 <= response.status < 300:
                raise RuntimeError('Core registration returned a non-success status')
        finally:
            connection.close()


def start_registration(port, config, driver_id):
    # Fail closed if private trust configuration is missing/invalid. Registration
    # failure must not prevent the independent MCP endpoint from starting.
    try:
        transport = CoreRegistration()
    except Exception:
        logging.error('Core registration TLS configuration invalid; registration disabled')
        return None
    payload = {'id': driver_id, 'name': config.get('name', driver_id),
               'url': f'http://localhost:{port}/mcp', 'transport': 'http', 'category': 'driver'}

    def loop():
        while True:
            try:
                transport.send(payload)
                time.sleep(30)
            except Exception:
                logging.warning('Verified Core registration failed; retrying in 5s')
                time.sleep(5)
    thread = threading.Thread(target=loop, daemon=True, name='register')
    thread.start()
    return thread
