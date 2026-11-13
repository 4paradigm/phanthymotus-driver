"""Synthetic model, freshness and lifecycle tests; no CAN or camera access."""
from copy import deepcopy
from itertools import product
from types import SimpleNamespace as NS
import json
import pytest
from agilex.piper.table_clearance import (TableClearance, add_table_clearance,
    clearance_settings, table_distance)

def calibrated(**changes):
    box = list(map(list, product([-1., 1.], repeat=3)))
    cfg = dict(enabled=True, verified=True, calibration_id='synthetic-only',
        can_interface='can0', base_frame='sdk_base', plane_base_mm=[0, 0, 1, 0],
        body_vertices_mm=box, left_finger_vertices_mm=box,
        right_finger_vertices_mm=box, opening_axis=[0, 1, 0],
        opening_range_mm=[0, 70], uncertainty_mm=2.)
    cfg.update(changes)
    return clearance_settings({'table_clearance': cfg})


def snapshot(**changes):
    s = dict(connected=True, can_interface='can0',
        end_pose=dict(position_mm=[0, 0, 100], orientation_deg=[0, 0, 0], feedback_time=100.),
        gripper=dict(width_mm=20, feedback_time=100.))
    s.update(changes)
    return s


def test_clearance_translation_opening_rotation_and_penetration():
    cfg = calibrated()
    s = snapshot()
    result = table_distance(s, cfg, 100.)
    assert result['clearance_mm'] == 99
    assert result['conservative_clearance_mm'] == 97
    s['end_pose']['orientation_deg'] = [90, 0, 0]
    assert table_distance(s, cfg, 100.)['clearance_mm'] == pytest.approx(89)
    s['gripper']['width_mm'] = 40
    assert table_distance(s, cfg, 100.)['clearance_mm'] == pytest.approx(79)
    s['end_pose']['position_mm'][2] = 0
    assert table_distance(s, cfg, 100.)['clearance_mm'] == pytest.approx(-21)


def test_tilted_table_uses_normal_distance():
    cfg = calibrated(plane_base_mm=[0, .6, .8, -5])
    result = table_distance(snapshot(), cfg, 100.)
    assert result['clearance_mm'] == pytest.approx(.8*99 - .6*11 - 5)


@pytest.mark.parametrize('mutate,reason', [
    (lambda s: s.update(connected=False), 'disconnected'),
    (lambda s: s.update(can_interface='can1'), 'calibration_arm_mismatch'),
    (lambda s: s['gripper'].update(width_mm=71), 'opening_outside_calibrated_range'),
    (lambda s: s['end_pose'].update(feedback_time=99), 'missing_or_stale_feedback'),
    (lambda s: s['gripper'].update(feedback_time=101), 'missing_or_stale_feedback'),
    (lambda s: s['gripper'].update(feedback_time=99.8), 'pose_gripper_time_skew'),
])
def test_unavailable_clears_numbers(mutate, reason):
    s = snapshot(); mutate(s)
    result = table_distance(s, calibrated(), 100.)
    assert result['reason'] == reason
    assert result['clearance_mm'] is result['lowest_point_base_mm'] is None


@pytest.mark.parametrize('changes', [dict(plane_base_mm=[0,0,2,0]),
    dict(plane_base_mm=[0,0,-1,0]), dict(opening_axis=[0,0,0]),
    dict(body_vertices_mm=[[0,0,0]]*4), dict(uncertainty_mm=None),
    dict(can_interface=''), dict(verified='yes'), dict(opening_range_mm=[10,0])])
def test_bad_calibration(changes):
    with pytest.raises(ValueError):
        calibrated(**changes)


def test_no_calibration_is_not_zero_distance():
    result = table_distance({}, clearance_settings({}), 100.)
    assert result['reason'] == 'calibration_unverified'
    assert result['clearance_mm'] is None


class Publisher:
    def __init__(self, ros, namespace, topics, period, callback):
        self.callback, self.messages, self.closed = callback, [], False
        assert period == .2
    def publish(self, fmt, result):
        assert fmt == 'data/json'
        self.messages.append(deepcopy(result))
    def close(self):
        self.closed = True


def test_lifecycle_freshness_restart_and_read_failure():
    clock = NS(wall=100., mono=10.)
    reads = []
    def read():
        reads.append(1)
        return snapshot()
    card = TableClearance(read, {'table_clearance': calibrated()}, 'arm1', object(),
        clock=lambda: clock.wall, monotonic=lambda: clock.mono, publisher_factory=Publisher)
    card.start(); old = card.transport
    assert not reads
    old.callback()
    assert old.messages[-1]['clearance_mm'] == 99
    assert old.messages[-1]['advisory_only']
    clock.mono += .6
    assert card.info()['result']['clearance_mm'] is None
    card.stop(); assert old.closed
    card.start(); old.callback()
    assert card.info()['result']['reason'] == 'waiting_for_feedback'
    card.transport.callback()
    result = card.info()
    result['result']['lowest_point_base_mm'][0] = 999
    assert card.info()['result']['lowest_point_base_mm'][0] != 999
    clock.wall = 101
    card.transport.callback()
    assert card.info()['result']['clearance_mm'] is None
    card.read_snapshot = lambda: (_ for _ in ()).throw(RuntimeError('read failed'))
    card.transport.callback()
    assert card.info()['result']['reason'] == 'invalid_feedback'
    card.stop()


def test_passive_bundle_registration_and_rejected_control_parameters():
    from common.vendor_runtime import DriverBundle
    from agilex.piper.device import build_plugins
    assert add_table_clearance([], {}, 'arm1', None) == []
    bundle = DriverBundle(build_plugins({'table_clearance': {'enabled': True}}, 'arm1', None))
    tool = next(t for t in bundle.get_all_tools() if t['name'] == 'piper_table_clearance')
    assert tool['type'] == 'sensor'
    assert bundle.dispatch('piper_table_clearance', {'action':'start'})['running']
    card = bundle.plugins[-1]
    card.tick(card.generation)
    assert card.info()['result']['reason'] == 'calibration_unverified'
    for args in ({'execute': True}, {'input_topic': '/control'}):
        with pytest.raises(ValueError):
            bundle.dispatch('piper_table_clearance', {'action':'start', **args})
    assert bundle.dispatch('piper_table_clearance', {'action':'stop'})['result']['state'] == 'idle'


def test_verified_build_plugins_reads_sdk_pose_and_opening_without_commands(tmp_path):
    from agilex.piper.device import build_plugins
    from agilex.piper.unified_control import UnifiedControlPlugin
    from common.vendor_runtime import DriverBundle
    from test_piper_intent import IntentDriver

    plugins = build_plugins({'data_dir': str(tmp_path), 'can_interface': 'can0',
        'table_clearance': calibrated()}, 'arm1', None)
    runtime, background = plugins[:2]
    background.auto_connect = False
    def forbid_connection(*args, **kwargs):
        pytest.fail('The measurement must not establish a hardware connection')
    runtime.factory = forbid_connection
    # Model an already connected shared receiver, using the existing recording SDK.
    driver = runtime.driver = IntentDriver()
    sdk = driver.arm
    sdk.pose = (0, 0, 100000, 0, 0, 0)
    sdk.width = 20.
    card, bundle = plugins[-1], DriverBundle(plugins)
    try:
        assert card.read_snapshot.__self__ is runtime
        assert card.read_snapshot.__func__ is UnifiedControlPlugin.read_monitor_snapshot
        bundle.dispatch('piper_table_clearance', {'action': 'start'})
        card.tick(card.generation)
        result = card.info()['result']
        assert result['state'] == 'ok'
        assert result['clearance_mm'] == pytest.approx(99.)
        assert result['opening_mm'] == 20.
        json.dumps(result, allow_nan=False)

        sdk.pose = (0, 0, 100000, 90000, 0, 0)
        sdk.width = 40.
        card.tick(card.generation)
        assert card.info()['result']['clearance_mm'] == pytest.approx(79.)
        sdk.pose_frozen = True
        card.tick(card.generation)
        assert card.info()['result']['state'] == 'unavailable'
        assert card.info()['result']['clearance_mm'] is None
        sdk.pose_frozen = False
        card.tick(card.generation)
        assert card.info()['result']['state'] == 'ok'
        # Reads, Start and Stop must not enable, move or query firmware.
        assert sdk.calls == []
    finally:
        card.stop()
        assert sdk.calls == []
        runtime.driver = None
        bundle.stop_all()


@pytest.mark.parametrize('stage', ['vertices', 'plane', 'pose'])
def test_finite_inputs_overflow_is_unavailable_and_json_safe(stage):
    s, cfg = snapshot(), calibrated()
    if stage == 'vertices':
        cfg = calibrated(body_vertices_mm=[[x * 1.7e308, y * 1.7e308, z * 1.7e308]
            for x, y, z in product([-1., 1.], repeat=3)])
        s['end_pose']['position_mm'] = [1.7e308] * 3
    elif stage == 'plane':
        cfg = calibrated(plane_base_mm=[0, 0, 1, 1.7e308])
        s['end_pose']['position_mm'][2] = 1.7e308
    else:
        cfg = calibrated(plane_base_mm=[.6, 0, .8, 0])
        s['end_pose']['position_mm'] = [1.7e308] * 3
    result = table_distance(s, cfg, 100.)
    assert result['state'] == 'unavailable'
    assert result['reason'] == 'nonfinite_geometry'
    assert result['clearance_mm'] is result['lowest_point_base_mm'] is None
    json.dumps(result, allow_nan=False)
