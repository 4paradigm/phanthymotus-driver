"""Huya URL-only, persistence, credential redaction contract."""
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from live_stream_config import HuyaCredentialStore, validate_huya_url

URL = ("rtmp://tx.direct.huya.com/huyalive/111-222-A-333"
       "?seq=12345&type=simple")


@pytest.mark.parametrize("url", [
    "rtmp://example.com/huyalive/stream",
    "https://tx.direct.huya.com/huyalive/foo",
    "rtmp://tx.direct.huya.com/other/foo",
    "rtmp://user:password@tx.direct.huya.com/huyalive/foo",
    "rtmp://tx.direct.huya.com:9999/huyalive/foo",
    "rtmp://tx.direct.huya.com/huyalive/foo#fragment",
    "rtmp://tx.direct.huya.com/huyalive/",
    "rtmp://tx.direct.huya.com/huyalive/foo\nFORGED",
    "file:///tmp/secret",
    "",
])
def test_rejects_non_huya_and_bad_urls(url):
    with pytest.raises(ValueError):
        validate_huya_url(url)


def test_accepts_valid_huya_url():
    assert validate_huya_url(URL) == URL


def test_credential_save_and_restore_is_private(tmp_path):
    path = tmp_path / "data" / "huya-url"
    store = HuyaCredentialStore(path)
    assert store.read() is None
    store.save(URL)
    assert store.read() == URL
    if os.name != "nt":
        assert (path.stat().st_mode & 0o077) == 0
    assert list(path.parent.glob(".huya-*")) == []


def test_symlink_not_overwritten(tmp_path):
    target = tmp_path / "target"
    target.write_text("not a credential")
    link = tmp_path / "link"
    try:
        link.symlink_to(target)
    except (OSError, NotImplementedError):
        pytest.skip("Symlink unavailable")
    with pytest.raises(OSError):
        HuyaCredentialStore(link).save(URL)
    assert target.read_text() == "not a credential"
