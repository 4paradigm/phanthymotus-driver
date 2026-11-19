"""Independent operator signatures; MCP data and public IDs are never authority.

Only the operator's public key belongs on the robot. Keep the signing key on a
trusted operator workstation, outside Core/Agent/Driver filesystems and tools.
An administrator able to replace this verifier or its key remains trusted.
"""

from __future__ import annotations

import base64
import hashlib
import json
import os
import stat
import threading
import time
import uuid


SCHEMA = "motus.bumi.sound-facing.authorization/1"
MAX_VALID_FOR_S = 300
PUBLIC_KEY_ENV = "BUMI_SOUND_FACING_OPERATOR_PUBLIC_KEY"
_FIELDS = {"schema", "device", "process_nonce", "request_id", "session_id", "resource"}


def canonical(payload):
    return json.dumps(payload, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True, allow_nan=False).encode("ascii")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate_authorization_field")
        result[key] = value
    return result


class OperatorAuthority:
    """One pinned key, one process/intent/session, one non-renewable permission.

    The short tracking session can be renewed by Agent, but never beyond the
    operator's deadline. Permission age starts when the challenge was created,
    not when an old signature happens to arrive. Invalidation retires its IDs.
    """

    def __init__(self, device, public_key_path=None, *, monotonic=time.monotonic):
        self._mono = monotonic
        self._lock = threading.RLock()
        self._device = device
        self._process_nonce = uuid.uuid4().hex
        self._key = None
        self._key_id = None
        self._unavailable = "operator_public_key_unconfigured"
        path = public_key_path if public_key_path is not None else os.getenv(PUBLIC_KEY_ENV)
        if path:
            try:
                from cryptography.hazmat.primitives import serialization
                from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey

                fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
                with os.fdopen(fd, "rb") as stream:
                    metadata = os.fstat(stream.fileno())
                    if (not stat.S_ISREG(metadata.st_mode) or metadata.st_size > 4096
                            or metadata.st_uid not in (0, os.geteuid())
                            or metadata.st_mode & 0o022):
                        raise ValueError("untrusted_public_key_file")
                    data = stream.read(4097)
                key = serialization.load_pem_public_key(data)
                if not isinstance(key, Ed25519PublicKey):
                    raise ValueError("ed25519_public_key_required")
                self._key = key
                self._key_id = hashlib.sha256(key.public_bytes(
                    serialization.Encoding.Raw, serialization.PublicFormat.Raw)).hexdigest()
                self._unavailable = None
            except Exception:
                # No permissive fallback, runtime key reload, or MCP key override.
                self._unavailable = "operator_public_key_unavailable"
        self.invalidate()

    def invalidate(self):
        with self._lock:
            self._request = {"schema": SCHEMA, "device": self._device,
                             "process_nonce": self._process_nonce,
                             "request_id": uuid.uuid4().hex,
                             "session_id": uuid.uuid4().hex, "resource": "chassis"}
            self._created = self._mono()
            self._until = 0.0
            self._accepted = False
            self._claimed = False

    def snapshot(self):
        with self._lock:
            remaining = max(0.0, self._until - self._mono()) if self._accepted else 0.0
            return {"available": self._key is not None, "error": self._unavailable,
                    "key_sha256": self._key_id, "remaining_s": remaining,
                    "state": "unavailable" if self._key is None else
                             "active" if remaining and self._claimed else
                             "approved" if remaining else "approval_required",
                    "request": {**self._request, "max_valid_for_s": MAX_VALID_FOR_S}}

    def authorize(self, document):
        """Verify outside the grant lock; recheck the exact intent at commit."""
        if self._key is None:
            raise ValueError("operator_authorization_unavailable")
        if not isinstance(document, str) or len(document) > 4096:
            raise ValueError("invalid_operator_authorization")
        try:
            proof = json.loads(document, object_pairs_hook=_unique_object)
            if not isinstance(proof, dict) or set(proof) != {"payload", "signature"}:
                raise ValueError
            payload = proof["payload"]
            if (not isinstance(payload, dict) or set(payload) != _FIELDS | {"valid_for_s"}
                    or any(not isinstance(payload[k], str) for k in _FIELDS)
                    or type(payload["valid_for_s"]) is not int
                    or not 1 <= payload["valid_for_s"] <= MAX_VALID_FOR_S
                    or not isinstance(proof["signature"], str)):
                raise ValueError
            signature = base64.b64decode(proof["signature"], validate=True)
            if len(signature) != 64:
                raise ValueError
            self._key.verify(signature, canonical(payload))
        except Exception as exc:
            raise ValueError("invalid_operator_authorization") from exc
        with self._lock:
            if {k: payload[k] for k in _FIELDS} != self._request:
                raise ValueError("authorization_intent_mismatch")
            if self._accepted:
                raise ValueError("authorization_already_used")
            until = self._created + payload["valid_for_s"]
            if self._mono() >= until:
                raise ValueError("operator_authorization_expired")
            self._until, self._accepted = until, True
            return self.snapshot()

    def claim(self):
        with self._lock:
            if self._claimed or not self._accepted or self._mono() >= self._until:
                raise ValueError("operator_authorization_required")
            self._claimed = True
            return self._request["session_id"]

    def deadline(self, session_id):
        with self._lock:
            if (self._key is None or not self._accepted or not self._claimed
                    or session_id != self._request["session_id"]
                    or self._mono() >= self._until):
                raise ValueError("operator_authorization_expired_or_revoked")
            return self._until
