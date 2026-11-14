"""Test-only operator key: real Ed25519, never a production authorization switch."""

import atexit
import base64
import json
import tempfile
from pathlib import Path

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

from sound_facing_authority import OperatorAuthority, canonical


_temporary = tempfile.TemporaryDirectory(prefix="synthetic-sound-operator-")
atexit.register(_temporary.cleanup)
PRIVATE_KEY = Ed25519PrivateKey.generate()
PUBLIC_KEY_PATH = Path(_temporary.name) / "operator.pub.pem"
PUBLIC_KEY_PATH.write_bytes(PRIVATE_KEY.public_key().public_bytes(
    serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo))
PUBLIC_KEY_PATH.chmod(0o400)


def signed(authority, valid_for_s=300, key=PRIVATE_KEY, **changes):
    request = authority.snapshot()["request"]
    payload = {k: v for k, v in request.items() if k != "max_valid_for_s"}
    payload.update(valid_for_s=valid_for_s, **changes)
    return json.dumps({"payload": payload, "signature": base64.b64encode(
        key.sign(canonical(payload))).decode("ascii")})


def approved_authority(clock=None, device="sim"):
    kwargs = {"monotonic": clock.mono} if clock is not None else {}
    authority = OperatorAuthority(device, PUBLIC_KEY_PATH, **kwargs)
    authority.authorize(signed(authority))
    return authority


def approve_engine(engine, valid_for_s=300):
    result = engine.prepare_authorization()
    assert "error" not in result, result
    result = engine.authorize(signed(engine.authority, valid_for_s))
    assert "error" not in result, result
