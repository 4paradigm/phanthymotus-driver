from types import SimpleNamespace
import pytest
from test_arm_stream import rig


@pytest.mark.parametrize('fsm', [500, 801])
def test_both_confirmed_modes_admitted_without_switching(tmp_path, fsm):
    arm, sample, _, _, _, sdk = rig(tmp_path, claim=False, servo=True)
    arm._feedback = sample
    arm._snapshot_provider = None
    arm._fsm = {'arm_ns': arm.clock(), 'fsm_id': fsm}
    arm._observer_started_ns = arm.clock()-600_000_000
    arm._arm_observer = SimpleNamespace(MatchedPublisherCount=lambda: 1)
    assert arm._fresh()[1] == sample['q']
    assert sdk == []
    arm._fsm['fsm_id'] = 0
    with pytest.raises(ValueError, match='motion_mode_unavailable'):
        arm._fresh()
