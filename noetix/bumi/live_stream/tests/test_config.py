"""Huya URL-only, persistence, credential redaction contract."""
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from live_stream.config import HuyaCredentialStore, validate_huya_url, validate_stream_url

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


XHS_URL = "rtmp://live-push.xhscdn.com/live/123456789?txSecret=example&txTime=ffffffff"


def test_xiaohongshu_regional_server_from_official_live_assistant():
    server = "rtmp://live-push-reg-tx.xhscdn.com/live"
    stream_key = "987?txSecret=fake_only&txTime=ffffffff&txDelayTime=0&vendor=tencent"
    platform, url = validate_stream_url(server + "/" + stream_key)
    assert platform == "xiaohongshu"
    assert url == server + "/" + stream_key


@pytest.mark.parametrize("host", [
    "fake-live-push-reg-tx.xhscdn.com",
    "live-push-reg-tx.xhscdn.com.evil.example",
    "live-push-reg-tx.evil.example",
])
def test_regional_server_near_miss_rejected(host):
    with pytest.raises(ValueError):
        validate_stream_url(
            f"rtmp://{host}/live/987?txSecret=fake_only&txTime=ffffffff"
        )


def test_xiaohongshu_server_and_stream_key_form_full_url():
    server = "rtmp://live-push.xhscdn.com/live"
    stream_key = "123456789?txSecret=example&txTime=ffffffff"
    platform, url = validate_stream_url(server + "/" + stream_key)
    assert platform == "xiaohongshu"
    assert url == XHS_URL


@pytest.mark.parametrize("url", [
    "rtmp://fake-live-push.xhscdn.com/live/id?txSecret=x&txTime=y",
    "rtmp://live-push.xhscdn.com.evil.com/live/id?txSecret=x&txTime=y",
    "rtmp://live-push.xhscdn.com/live/id?txSecret=x",
    "rtmp://live-push.xhscdn.com/live/id?txSecret=&txTime=y",
    "rtmp://live-push.xhscdn.com:5555/live/id?txSecret=x&txTime=y",
    "file:///live/id?txSecret=x&txTime=y",
])
def test_rejects_invalid_or_impostor_xiaohongshu_urls(url):
    with pytest.raises(ValueError):
        validate_stream_url(url)


def test_xiaohongshu_store_is_private_and_restored(tmp_path):
    store = HuyaCredentialStore(tmp_path / "private" / "rtmp")
    store.save(XHS_URL)
    assert store.read() == XHS_URL


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

@pytest.mark.skipif(os.name == "nt", reason="Linux directory-fd security only")
def test_linux_rejects_world_readable_credential_directory(tmp_path):
    parent = tmp_path / "credentials"
    parent.mkdir(mode=0o700)
    parent.chmod(0o755)
    store = HuyaCredentialStore(parent / "url")
    with pytest.raises(OSError, match="owner-only"):
        store.save(URL)
    assert store.read() is None


@pytest.mark.skipif(os.name == "nt", reason="Linux directory-fd security only")
def test_linux_rejects_symlink_parent(tmp_path):
    actual = tmp_path / "actual"
    actual.mkdir(mode=0o700)
    symlink = tmp_path / "shortcut"
    symlink.symlink_to(actual, target_is_directory=True)
    with pytest.raises(OSError):
        HuyaCredentialStore(symlink / "url").save(URL)
    assert not (actual / "url").exists()


@pytest.mark.skipif(os.name == "nt", reason="Linux directory-fd security only")
def test_linux_atomic_replacement_retains_private_mode(tmp_path):
    store = HuyaCredentialStore(tmp_path / "private" / "url")
    store.save(URL)
    store.save(URL.replace("seq=12345", "seq=67890"))
    assert store.read().endswith("seq=67890&type=simple")
    assert (store.path.stat().st_mode & 0o077) == 0
    assert sorted(p.name for p in store.path.parent.iterdir()) == ["url"]
