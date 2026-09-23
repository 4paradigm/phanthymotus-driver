"""Persistent, device-only TLS identity. No Agent Core keys or privileges."""

from __future__ import annotations
import datetime
import ipaddress
import os
import socket
from pathlib import Path
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID
from ext_vr.tls_files import open_tls_directory, read_tls_file


def validate_dds_profile(path):
    """Do not let a missing profile silently expand the local control domain."""
    import xml.etree.ElementTree as ET

    root = ET.parse(path).getroot()
    for element in root.iter():
        element.tag = element.tag.split("}")[-1]
    defaults = [
        p for p in root.iter("participant") if p.get("is_default_profile") == "true"
    ]
    if (
        len(defaults) != 1
        or defaults[0].findtext("rtps/useBuiltinTransports") != "false"
    ):
        raise ValueError("pico_dds_requires_loopback_profile")
    ids = [p.text for p in defaults[0].findall("rtps/userTransports/transport_id")]
    definitions = {
        p.findtext("transport_id"): p for p in root.iter("transport_descriptor")
    }
    if not ids:
        raise ValueError("pico_dds_requires_loopback_profile")
    for name in ids:
        descriptor = definitions.get(name)
        if descriptor is None or descriptor.findtext("type") != "UDPv4":
            raise ValueError("pico_dds_requires_loopback_profile")
        addresses = [a.text for a in descriptor.findall("interfaceWhiteList/address")]
        if not addresses or any(
            not ipaddress.ip_address(a).is_loopback for a in addresses
        ):
            raise ValueError("pico_dds_requires_loopback_profile")


def prepare_config(config):
    result = dict(config)
    root = Path(result.get("state_dir", "/var/lib/motus/pico"))
    root.mkdir(parents=True, exist_ok=True, mode=0o700)
    host = os.environ.get("PICO_PUBLIC_HOST") or result.get("public_host")
    if not host:
        # Routing lookup only; no UDP packet is sent.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            try:
                probe.connect(("192.0.2.1", 9))
                host = probe.getsockname()[0]
            except OSError:
                host = "127.0.0.1"
    if any(x in host for x in ("/", "@", "?", "#", " ")):
        raise ValueError("invalid_pico_public_host")
    tls = root / "tls"
    tls.mkdir(mode=0o700, exist_ok=True)
    cert_path, key_path = tls / "cert.pem", tls / "key.pem"
    if cert_path.is_symlink() or key_path.is_symlink():
        raise ValueError("pico_tls_symlink")
    if cert_path.exists() != key_path.exists():
        raise ValueError("pico_tls_identity_incomplete")
    if not cert_path.exists():
        key = ec.generate_private_key(ec.SECP256R1())
        name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "PICO Driver")])
        try:
            alternative = x509.IPAddress(ipaddress.ip_address(host))
        except ValueError:
            alternative = x509.DNSName(host)
        now = datetime.datetime.now(datetime.timezone.utc)
        cert = (
            x509.CertificateBuilder()
            .subject_name(name)
            .issuer_name(name)
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - datetime.timedelta(minutes=5))
            .not_valid_after(now + datetime.timedelta(days=365))
            .add_extension(x509.SubjectAlternativeName([alternative]), critical=False)
            .sign(key, hashes.SHA256())
        )
        for path, data in (
            (
                key_path,
                key.private_bytes(
                    serialization.Encoding.PEM,
                    serialization.PrivateFormat.PKCS8,
                    serialization.NoEncryption(),
                ),
            ),
            (cert_path, cert.public_bytes(serialization.Encoding.PEM)),
        ):
            with open_tls_directory(tls) as directory:
                fd = os.open(path.name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=directory)
                with os.fdopen(fd, "wb") as stream:
                    stream.write(data)
    # Existing identity is never silently rotated when host configuration changes.
    # Validate existing as well as newly-created material without following paths.
    read_tls_file(cert_path)
    read_tls_file(key_path)
    hostname = "[" + host + "]" if ":" in host else host
    result.update(
        state_dir=str(root),
        tls_cert_file=str(cert_path),
        tls_key_file=str(key_path),
        public_wss_url=f"wss://{hostname}:{int(result.get('port',15741))}/ws/teleop-capture",
    )
    return result
