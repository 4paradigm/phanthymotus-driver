"""Host identity is observed, never guessed from platform names or timestamps."""
from common import capture_clock


def test_reference_uses_actual_kernel_boot_identity(monkeypatch):
    monkeypatch.setattr(capture_clock.Path, 'read_text', lambda p: '12345678-1234-1234-1234-123456789abc\n')
    assert capture_clock.host_clock_reference() == dict(clock_id='host_realtime',
        domain_id='12345678-1234-1234-1234-123456789abc', timestamp_basis='host_realtime')


def test_missing_or_invalid_identity_never_mints_a_fallback(monkeypatch):
    monkeypatch.setattr(capture_clock.Path, 'read_text', lambda p: 'not-a-boot-id')
    assert capture_clock.host_clock_reference() is None
    def missing(p): raise OSError('unavailable')
    monkeypatch.setattr(capture_clock.Path, 'read_text', missing)
    assert capture_clock.host_clock_reference() is None
