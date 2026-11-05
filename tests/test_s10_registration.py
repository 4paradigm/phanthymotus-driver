"""ROS-free tests for S10 registration trust and request ordering."""
import hashlib
import ssl
from unittest.mock import MagicMock, patch

import pytest
from deep_robotics.s10_pro.registration import CoreRegistration


@pytest.fixture(autouse=True)
def clean_trust(monkeypatch):
    for name in ('S10_REGISTRATION_CA_FILE', 'S10_REGISTRATION_PINNED_LEAF_FILE',
                 'S10_ACP_CA_FILE', 'S10_ACP_PINNED_LEAF_FILE'):
        monkeypatch.delenv(name, raising=False)


def test_default_context_requires_ca_and_hostname():
    client = CoreRegistration('https://localhost:15678')
    assert client.context.verify_mode == ssl.CERT_REQUIRED
    assert client.context.check_hostname


@pytest.mark.parametrize('url', ['http://localhost', 'https://u:p@localhost', 'https://localhost?x=1'])
def test_reject_unverified_url_modes(url):
    with pytest.raises(ValueError):
        CoreRegistration(url)


def test_pin_checked_before_post_and_connection_closed():
    client = CoreRegistration('https://localhost:15678')
    client.digest = hashlib.sha256(b'approved').digest()
    connection = MagicMock()
    connection.sock.getpeercert.return_value = b'impostor'
    with patch('deep_robotics.s10_pro.registration.http.client.HTTPSConnection', return_value=connection):
        with pytest.raises(ssl.SSLCertVerificationError):
            client.send({'id': 'driver'})
    connection.request.assert_not_called()
    connection.close.assert_called_once()


def test_exact_pin_posts_only_after_connect_and_rejects_redirect():
    client = CoreRegistration('https://localhost:15678')
    client.digest = hashlib.sha256(b'approved').digest()
    connection = MagicMock()
    connection.sock.getpeercert.return_value = b'approved'
    connection.getresponse.return_value.status = 302
    with patch('deep_robotics.s10_pro.registration.http.client.HTTPSConnection', return_value=connection):
        with pytest.raises(RuntimeError):
            client.send({'id': 'driver'})
    assert connection.method_calls[0][0] == 'connect'
    connection.request.assert_called_once()
    assert connection.request.call_args.args[:2] == ('POST', '/api/mcp')
    connection.close.assert_called_once()


def test_private_pin_context_and_existing_acp_trust_fallback(tmp_path, monkeypatch):
    cert = tmp_path / 'leaf.pem'
    cert.write_text('-----BEGIN CERTIFICATE-----\nfixture\n-----END CERTIFICATE-----')
    monkeypatch.setenv('S10_ACP_PINNED_LEAF_FILE', str(cert))
    context = MagicMock()
    with patch('deep_robotics.s10_pro.registration.ssl.PEM_cert_to_DER_cert', return_value=b'approved'), \
         patch('deep_robotics.s10_pro.registration.ssl.SSLContext', return_value=context):
        client = CoreRegistration('https://localhost')
    assert context.verify_mode == ssl.CERT_REQUIRED
    assert context.check_hostname is False
    context.load_verify_locations.assert_called_once_with(cadata=cert.read_text())
    assert client.digest == hashlib.sha256(b'approved').digest()
