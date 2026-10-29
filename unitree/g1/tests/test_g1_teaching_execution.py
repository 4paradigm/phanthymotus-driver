"""G1 regression definitions for per-skill policy and stop races (SDK fixture)."""
import pytest

from test_g1_teaching import g1_teaching, call, save, wait_for


def allow(monkeypatch):
    monkeypatch.setattr('common.teaching.service.read_execution_policy', lambda *a, **k: {
        'enabled': True, 'source': 'skill', 'binding_changed': False})


def test_saved_policy_survives_stop_but_model_config_cannot_grant(g1_teaching, monkeypatch):
    service, _, _, _, channel, _ = g1_teaching
    _, saved, _ = save(service); asset = saved['asset_id']
    before = len(channel.positions)
    with pytest.raises(ValueError, match='unknown_teaching_action'):
        call(service, 'config', auto_execute_verified=True)
    assert call(service, 'preview', asset_id=asset)['confirmation_required']
    allow(monkeypatch)
    preview = call(service, 'preview', asset_id=asset)
    assert not preview['confirmation_required'] and len(channel.positions) == before
    call(service, 'stop')
    assert not call(service, 'preview', asset_id=asset)['confirmation_required']


@pytest.mark.parametrize('when', ['policy_read', 'play'])
def test_stop_during_policy_check_or_dispatch_rejects_old_request(g1_teaching, monkeypatch, when):
    service, card, _, _, channel, _ = g1_teaching
    _, saved, _ = save(service); asset = saved['asset_id']
    allow(monkeypatch)
    preview = call(service, 'preview', asset_id=asset)
    if when == 'policy_read':
        def stopped(*a, **k):
            card.stop()
            return {'enabled': True, 'source': 'skill', 'binding_changed': False}
        monkeypatch.setattr('common.teaching.service.read_execution_policy', stopped)
    else:
        play = service.adapter.play
        def stopped(filename, args):
            card.stop()
            return play(filename, args)
        monkeypatch.setattr(service.adapter, 'play', stopped)
    before = len(channel.positions)
    with pytest.raises(ValueError, match='execution_context_changed'):
        call(service, 'run', **preview['execution_arguments'])
    assert len(channel.positions) == before
