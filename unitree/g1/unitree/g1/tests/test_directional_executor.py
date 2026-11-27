"""Point policy cannot leak into another owner's ordinary/replay lease."""
import pytest

from common.motion.protocol import envelope
from _point_test_helpers import direct_setup, TARGET, assert_released


def command(arm, values, *, seq=1):
    now = arm.clock()
    return envelope(dict(boot_id=arm.boot_id, session_id=arm.session_id, secret=arm.secret),
                    seq=seq, source_seq=seq, mapping_epoch=0,
                    generated_ns=now, valid_until_ns=now+200_000_000,
                    mode='joint_position', dof=10, values=values, **arm.versions)


def test_point_guard_failure_is_cleared_before_ordinary_replay(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, allowed, _, writes, advance = direct_setup(tmp_path, monkeypatch)

    def revoke(phase, q):
        if phase == 'moving' and writes:
            allowed[0] = False

    with pytest.raises(RuntimeError, match='directional_guard:'):
        backend.execute(TARGET, .2, stop, revoke)
    assert_released(arm, channel)
    assert arm.status()['directional_fault'].startswith('directional_guard:')
    old_session = arm.status()['release']['session_id']
    ordinary = []

    def publish(q, *, weight):
        ordinary.append(list(q))
        sample['q'][:] = q
        return True

    monkeypatch.setattr(channel, 'publish_servo_position', publish)
    monkeypatch.setattr(channel, 'publish_point_position',
                        lambda *a, **k: pytest.fail('ordinary replay used point writer'))
    monkeypatch.setattr(backend, 'stationary',
                        lambda: pytest.fail('ordinary replay used point guard'))
    assert 'error' not in arm.dispatch('prepare_operator_session', {})
    lease = arm.dispatch('claim', {})
    assert lease['state'] == 'ready' and lease['session_id'] != old_session
    assert arm._directional_guard is None and arm._lease_velocity is None
    assert arm._directional_envelope_failure is None
    assert 'directional_fault' not in arm.status()
    target = [q+.02 for q in sample['q']]
    arm.replay_deadline_ns = arm.clock()+200_000_000
    arm.write_replay(command(arm, target))
    assert ordinary == [target] and arm.last_q == target
    result = arm.dispatch('stop', lease)
    for _ in range(500):
        if result['state'] == 'completed':
            break
        advance(.02)
        result = arm.status()['release']
    assert result['state'] == 'completed' and 'session_id' not in result
    assert_released(arm, channel)


def test_point_lease_cannot_use_replay_to_bypass_its_path_policy(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, writes, _ = direct_setup(tmp_path, monkeypatch)
    checked = []

    def try_replay(phase, q):
        if phase == 'moving' and writes and not checked:
            previous, sequence = arm.latest, arm.seq
            with pytest.raises(ValueError, match='directional_replay_not_supported'):
                arm.accept_control(command(arm, sample['q'], seq=sequence+1), replay=True)
            assert arm.latest is previous and arm.seq == sequence
            checked.append(True)
            stop.set()

    with pytest.raises(RuntimeError, match='^cancelled$'):
        backend.execute(TARGET, .2, stop, try_replay)
    assert checked
    assert_released(arm, channel)
