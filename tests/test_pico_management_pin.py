"""Management PIN storage and session boundaries without HTTP or hardware."""
import json
from pathlib import Path
import stat
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pico/4ultra"))
from ext_vr.capture import CaptureError
from ext_vr.management_pin import ManagementPin


@pytest.fixture
def manager(tmp_path):
    now = [1000.0]
    return ManagementPin(tmp_path / "state" / "management-pin.json", clock=lambda: now[0]), now


def rejected(code, status, method, *args):
    with pytest.raises(CaptureError) as caught:
        method(*args)
    assert caught.value.code == code
    assert caught.value.status == status


@pytest.mark.parametrize("pin", [None, 1234, "", "123", "12345", "12a4", "１２３４", "123\n", " 123"])
def test_configuration_rejects_non_ascii_four_digits(manager, pin):
    auth, _ = manager
    with pytest.raises(ValueError, match="management_pin_must_be_four_digits"):
        auth.configure(pin)
    assert not auth.configured
    assert not auth.path.exists()


def test_pin_persists_only_salted_hash_with_private_permissions(manager):
    auth, _ = manager
    assert auth.configure("0000") is True
    record = json.loads(auth.path.read_text())
    assert set(record) == {"salt", "digest"}
    assert len(bytes.fromhex(record["salt"])) == 16
    assert len(bytes.fromhex(record["digest"])) == 32
    assert record["digest"] != "0000"
    assert stat.S_IMODE(auth.path.stat().st_mode) == 0o600
    assert not auth.path.with_suffix(".tmp").exists()
    restored = ManagementPin(auth.path)
    restored.authorize(restored.login("0000"))


def test_unconfigured_management_is_unavailable(manager):
    auth, _ = manager
    rejected("management_pin_not_configured", 503, auth.login, "1234")
    rejected("management_pin_not_configured", 503, auth.authorize, None)


def test_fifth_wrong_pin_locks_all_logins_for_five_minutes(manager):
    auth, now = manager
    auth.configure("1234")
    for _ in range(4):
        rejected("management_pin_invalid", 403, auth.login, "4321")
    rejected("management_rate_limited", 429, auth.login, "4321")
    rejected("management_rate_limited", 429, auth.login, "1234")
    now[0] += 299.999
    rejected("management_rate_limited", 429, auth.login, "1234")
    now[0] += 0.001
    auth.authorize(auth.login("1234"))
    rejected("management_pin_invalid", 403, auth.login, "4321")


def test_successful_login_resets_failure_count(manager):
    auth, _ = manager
    auth.configure("1234")
    for _ in range(4):
        rejected("management_pin_invalid", 403, auth.login, "bad")
    auth.login("1234")
    rejected("management_pin_invalid", 403, auth.login, "bad")


def test_session_expires_at_900_seconds_without_sliding_refresh(manager):
    auth, now = manager
    auth.configure("1234")
    token = auth.login("1234")
    now[0] += 899.999
    auth.authorize(token)
    now[0] += 0.001
    rejected("management_session_expired", 401, auth.authorize, token)
    rejected("management_pin_required", 401, auth.authorize, None)
    auth.authorize(auth.login("1234"))


def test_logout_invalidates_only_that_session_and_is_idempotent(manager):
    auth, _ = manager
    auth.configure("1234")
    first, second = auth.login("1234"), auth.login("1234")
    assert first != second
    auth.logout(first)
    auth.logout(first)
    auth.logout(None)
    rejected("management_session_expired", 401, auth.authorize, first)
    auth.authorize(second)


def test_same_pin_save_preserves_session_but_changed_pin_revokes_it(manager):
    auth, _ = manager
    auth.configure("1234")
    token = auth.login("1234")
    before = auth.path.read_bytes()
    assert auth.configure("1234") is False
    assert auth.path.read_bytes() == before
    auth.authorize(token)
    assert auth.configure("5678") is True
    rejected("management_session_expired", 401, auth.authorize, token)
    rejected("management_pin_invalid", 403, auth.login, "1234")
    auth.authorize(auth.login("5678"))
    assert stat.S_IMODE(auth.path.stat().st_mode) == 0o600


def test_restart_keeps_pin_but_invalidates_sessions(manager):
    auth, now = manager
    auth.configure("1234")
    token = auth.login("1234")
    restarted = ManagementPin(auth.path, clock=lambda: now[0])
    rejected("management_session_expired", 401, restarted.authorize, token)
    restarted.authorize(restarted.login("1234"))


@pytest.mark.parametrize("mode", [0o644, 0o640, 0o666, 0o400])
def test_existing_pin_file_with_wrong_permissions_is_rejected(manager, mode):
    auth, _ = manager
    auth.configure("1234")
    auth.path.chmod(mode)
    with pytest.raises(ValueError, match="regular_0600"):
        ManagementPin(auth.path)
    with pytest.raises(ValueError, match="regular_0600"):
        auth.configure("1234")
    assert stat.S_IMODE(auth.path.stat().st_mode) == mode  # No silent chmod.


@pytest.mark.parametrize("mode", [0o755, 0o750, 0o777])
def test_existing_pin_parent_must_be_private(manager, mode):
    auth, _ = manager
    auth.configure("1234")
    auth.path.parent.chmod(mode)
    with pytest.raises(ValueError, match="private_0700"):
        ManagementPin(auth.path)
    with pytest.raises(ValueError, match="private_0700"):
        auth.configure("5678")
    assert stat.S_IMODE(auth.path.parent.stat().st_mode) == mode


def test_pin_symlink_and_parent_symlink_are_rejected(manager, tmp_path):
    auth, _ = manager
    auth.configure("1234")
    original = auth.path.read_bytes()
    alias = auth.path.parent / "linked.json"
    alias.symlink_to(auth.path)
    with pytest.raises(ValueError, match="regular_0600"):
        ManagementPin(alias)
    parent_alias = tmp_path / "linked-state"
    parent_alias.symlink_to(auth.path.parent, target_is_directory=True)
    with pytest.raises(ValueError, match="private_0700"):
        ManagementPin(parent_alias / auth.path.name)
    assert auth.path.read_bytes() == original


def test_nonregular_pin_state_rejected_without_opening(manager):
    import os
    auth, _ = manager
    auth.path.parent.mkdir(mode=0o700)
    os.mkfifo(auth.path, mode=0o600)
    with pytest.raises(ValueError, match="regular_0600"):
        ManagementPin(auth.path)


def test_pin_atomic_save_never_uses_existing_predictable_temporary(manager, tmp_path):
    auth, _ = manager
    auth.configure("1234")
    unrelated = tmp_path / "unrelated"
    unrelated.write_text("keep")
    old_temporary = auth.path.with_suffix(".tmp")
    old_temporary.symlink_to(unrelated)
    auth.configure("5678")
    assert unrelated.read_text() == "keep"
    assert old_temporary.is_symlink()
    assert not list(auth.path.parent.glob(".management-pin-*.tmp"))
