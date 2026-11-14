"""Real signature/intent enforcement, with an explicitly synthetic operator key."""

import base64
import json
import sys
from pathlib import Path

import pytest
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sound_facing import SoundFacing
from sound_facing_authority import OperatorAuthority
from sound_authorization_fixtures import (
    PUBLIC_KEY_PATH, approved_authority, signed,
)
from test_sound_facing import Clock, Motion, feed, finish_and_settle


def test_unconfigured_production_path_is_fail_closed(monkeypatch):
    monkeypatch.delenv("BUMI_SOUND_FACING_OPERATOR_PUBLIC_KEY", raising=False)
    motion = Motion()
    engine = SoundFacing(motion)
    engine.start()
    assert engine.enable()["error"] == "operator_authorization_unavailable"
    assert not motion.calls and motion.owner is None
    assert not engine.disable()["enabled"]


def test_signature_does_not_enable_and_authorized_agent_can_track_and_renew():
    clock, motion = Clock(), Motion()
    authority = OperatorAuthority("sim", PUBLIC_KEY_PATH, monotonic=clock.mono)
    engine = SoundFacing(motion, monotonic=clock.mono, wall_ns=clock.wall, authority=authority)
    engine.start()
    assert engine.enable()["error"] == "operator_authorization_required"
    assert "error" not in engine.authorize(signed(authority, 60))
    assert not engine.session_id and not motion.calls
    session = engine.enable()["session_id"]
    for _ in range(12):
        feed(clock, engine, (90, 90, 90))
        finish_and_settle(clock, motion, engine)
        assert engine.renew(session)["session_id"] == session
    assert sum(motion.calls) == 1080 and engine._expires <= authority.deadline(session)


@pytest.mark.parametrize("field,value", [
    ("device", "other"), ("resource", "arm"), ("process_nonce", "old-process"),
    ("request_id", "old-intent"), ("session_id", "public-other-session"),
    ("schema", "other-schema"),
])
def test_even_signed_wrong_scope_or_intent_is_rejected(field, value):
    authority = OperatorAuthority("sim", PUBLIC_KEY_PATH)
    with pytest.raises(ValueError, match="intent_mismatch"):
        authority.authorize(signed(authority, **{field: value}))


@pytest.mark.parametrize("document", ["true", "null", "[]", "{}", "not-json", "x" * 4097,
    '{"payload":{},"payload":{},"signature":"x"}',
    '{"authorized":true}', '{"session_id":"public"}',
])
def test_ordinary_flags_public_ids_and_parser_variants_are_not_authority(document):
    authority = OperatorAuthority("sim", PUBLIC_KEY_PATH)
    with pytest.raises(ValueError):
        authority.authorize(document)
    with pytest.raises(ValueError):
        authority.claim()


@pytest.mark.parametrize("kind", ["wrong_key", "tampered", "short_signature", "bad_base64",
                                  "boolean_ttl", "float_ttl", "oversized_ttl", "extra_field"])
def test_cryptographic_and_payload_forgery_rejected(kind):
    authority = OperatorAuthority("sim", PUBLIC_KEY_PATH)
    proof = json.loads(signed(authority))
    if kind == "wrong_key":
        proof = json.loads(signed(authority, key=Ed25519PrivateKey.generate()))
    elif kind == "tampered":
        proof["payload"]["valid_for_s"] = 299
    elif kind == "short_signature":
        proof["signature"] = base64.b64encode(b"x").decode()
    elif kind == "bad_base64":
        proof["signature"] = "!" * 88
    elif kind == "boolean_ttl":
        proof["payload"]["valid_for_s"] = True
    elif kind == "float_ttl":
        proof["payload"]["valid_for_s"] = 10.0
    elif kind == "oversized_ttl":
        proof["payload"]["valid_for_s"] = 301
    else:
        proof["payload"]["authorized"] = True
    with pytest.raises(ValueError, match="invalid_operator_authorization"):
        authority.authorize(json.dumps(proof))


def test_public_session_id_cannot_claim_unapproved_permission():
    authority = OperatorAuthority("sim", PUBLIC_KEY_PATH)
    public_id = authority.snapshot()["request"]["session_id"]
    assert public_id
    with pytest.raises(ValueError):
        authority.deadline(public_id)
    with pytest.raises(ValueError):
        authority.claim()


def test_replay_never_extends_deadline_and_expired_grant_cannot_renew():
    clock, motion = Clock(), Motion()
    authority = OperatorAuthority("sim", PUBLIC_KEY_PATH, monotonic=clock.mono)
    engine = SoundFacing(motion, monotonic=clock.mono, wall_ns=clock.wall, authority=authority)
    engine.start()
    proof = signed(authority, 2)
    engine.authorize(proof)
    session = engine.enable()["session_id"]
    clock.advance(1)
    engine.observe({"state": "no_event"})
    assert engine.renew(session)["session_remaining_s"] == pytest.approx(1)
    assert engine.authorize(proof)["error"] == "authorization_already_used"
    clock.advance(1.01)
    assert engine.renew(session)["error"] == "session_not_active_or_mismatched"
    assert engine.reason == "operator_authorization_expired_or_revoked"
    assert not engine.session_id and not motion.owner
    assert engine.authorize(proof)["error"] == "authorization_intent_mismatch"


def test_revocation_retires_the_intent_and_cannot_resurrect():
    clock, motion = Clock(), Motion()
    authority = approved_authority(clock)
    engine = SoundFacing(motion, monotonic=clock.mono, wall_ns=clock.wall, authority=authority)
    engine.start(); session = engine.enable()["session_id"]
    old_proof = signed(authority)
    feed(clock, engine)
    authority.invalidate()  # simulate a trusted policy revocation, no new permission
    engine.tick()
    assert motion.cancelled and engine.session_id is None
    assert engine.renew(session)["error"] == "session_not_active_or_mismatched"
    assert engine.authorize(old_proof)["error"] == "authorization_intent_mismatch"
    assert len(motion.calls) == 1


def test_authority_exception_cancels_and_stop_needs_no_authorization(monkeypatch):
    clock, motion = Clock(), Motion()
    authority = approved_authority(clock)
    engine = SoundFacing(motion, monotonic=clock.mono, wall_ns=clock.wall, authority=authority)
    engine.start(); engine.enable(); feed(clock, engine)
    monkeypatch.setattr(authority, "deadline", lambda _: (_ for _ in ()).throw(RuntimeError("offline")))
    engine.tick()
    assert engine.session_id is None and motion.cancelled
    assert not engine.disable()["enabled"]


@pytest.mark.parametrize("kind", ["missing", "symlink", "writable", "private_key", "non_ed25519"])
def test_invalid_pin_is_unavailable_without_fallback(tmp_path, kind):
    pin = tmp_path / "operator.pem"
    if kind == "symlink":
        pin.symlink_to(PUBLIC_KEY_PATH)
    elif kind == "writable":
        pin.write_bytes(PUBLIC_KEY_PATH.read_bytes()); pin.chmod(0o666)
    elif kind == "private_key":
        pin.write_bytes(Ed25519PrivateKey.generate().private_bytes(serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8, serialization.NoEncryption()))
    elif kind == "non_ed25519":
        from cryptography.hazmat.primitives.asymmetric import ec
        pin.write_bytes(ec.generate_private_key(ec.SECP256R1()).public_key().public_bytes(
            serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo))
    authority = OperatorAuthority("sim", pin)
    assert authority.snapshot()["available"] is False
    with pytest.raises(ValueError, match="unavailable"):
        authority.authorize("{}")


def test_no_runtime_key_reload_or_key_path_in_request(tmp_path):
    pin = tmp_path / "pin.pem"; pin.write_bytes(PUBLIC_KEY_PATH.read_bytes())
    authority = OperatorAuthority("sim", pin)
    pin.write_bytes(b"changed")
    authority.authorize(signed(authority))  # pinned in memory at trusted startup
    assert "public_key_path" not in authority.snapshot()["request"]


def test_expiry_during_enable_preflight_is_cleaned_up_without_throwing():
    clock, motion = Clock(), Motion()
    authority = OperatorAuthority("sim", PUBLIC_KEY_PATH, monotonic=clock.mono)
    authority.authorize(signed(authority, 1))
    original = motion.relative_motion_health
    def delayed(owner):
        clock.advance(2)
        return original(owner)
    motion.relative_motion_health = delayed
    engine = SoundFacing(motion, monotonic=clock.mono, wall_ns=clock.wall, authority=authority)
    engine.start();result = engine.enable()
    assert not result["enabled"] and motion.owner is None and not motion.calls
    assert result["state"] == "fault"
