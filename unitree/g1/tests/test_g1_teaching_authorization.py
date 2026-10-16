"""Session scope and revocation use the real G1 controller; no robot connection."""
import pytest

from test_g1_teaching import g1_teaching, call, save, wait_for


def approve(service, asset_id, monkeypatch):
    challenge = call(service, 'session_challenge', asset_id=asset_id)
    monkeypatch.setattr('common.teaching.service.consume_approval', lambda value: {
        'nonce': challenge['nonce'], 'asset_id': asset_id, 'tool': service.name,
        'instance_id': '', 'seconds': 1800})
    return call(service, 'session_authorize', asset_id=asset_id, approval_id='a'*64)


def test_forged_grant_cannot_authorize_and_valid_grant_supports_repeated_calls(g1_teaching, monkeypatch):
    service, _, _, _, channel, _ = g1_teaching
    _, saved, _ = save(service)
    asset = saved['asset_id']; before = len(channel.positions)
    with pytest.raises(ValueError, match='session_approval_expired'):
        call(service, 'session_authorize', asset_id=asset, approval_id='a'*64)
    call(service, 'session_challenge', asset_id=asset)
    with pytest.raises(ValueError, match='invalid_session_approval'):
        call(service, 'session_authorize', asset_id=asset, approval_id='forged', workspace_clear=True, take_control=True)
    assert not call(service, 'session_status', asset_id=asset)['active']
    assert len(channel.positions) == before
    assert approve(service, asset, monkeypatch)['active']
    assert len(channel.positions) == before
    for _ in range(2):
        preview = call(service, 'preview', asset_id=asset)
        assert preview['confirmation_required'] is False
        assert 'workspace_clear' not in preview['execution_arguments']
        result = call(service, 'run', **preview['execution_arguments'])
        wait_for(lambda: (service.last_run or {}).get('action_id') == result['action_id'])
        assert service.last_run['status'] == 'completed'
        assert call(service, 'session_status', asset_id=asset)['active']
    call(service, 'stop')
    assert not call(service, 'session_status', asset_id=asset)['active']
    preview = call(service, 'preview', asset_id=asset)
    assert preview['confirmation_required']
    with pytest.raises(ValueError, match='user_confirmation_required'):
        call(service, 'run', asset_id=asset, execute=True, preview_id=preview['preview_id'])


@pytest.mark.parametrize('change', ['expire', 'external_stop', 'reboot', 'changed_asset', 'feedback_reconnect', 'calibration'])
def test_grant_scope_changes_revoke_before_motion(g1_teaching, monkeypatch, change):
    service, card, arm, _, channel, _ = g1_teaching
    _, saved, _ = save(service); asset = saved['asset_id']
    approve(service, asset, monkeypatch)
    before = len(channel.positions)
    if change == 'expire': service.grants[asset]['expires'] = 0
    if change == 'external_stop': card.stop()
    if change == 'reboot': arm.boot_id = 'another-boot'
    if change == 'changed_asset': service.grants[asset]['sha256'] = 'different-clip'
    if change == 'feedback_reconnect': arm.feedback_epoch += 1
    if change == 'calibration': arm.profile_sha256 = 'new-calibration'
    assert call(service, 'preview', asset_id=asset)['confirmation_required']
    assert len(channel.positions) == before


def test_stop_racing_authorized_execution_cannot_reuse_grant(g1_teaching, monkeypatch):
    service, card, _, _, channel, _ = g1_teaching
    _, saved, _ = save(service); asset = saved['asset_id']
    approve(service, asset, monkeypatch)
    preview = call(service, 'preview', asset_id=asset)
    play = service.adapter.play
    def stopped(filename, args):
        card.stop()
        return play(filename, args)
    monkeypatch.setattr(service.adapter, 'play', stopped)
    before = len(channel.positions)
    with pytest.raises(ValueError, match='session_authorization_expired'):
        call(service, 'run', **preview['execution_arguments'])
    assert len(channel.positions) == before and asset not in service.grants
