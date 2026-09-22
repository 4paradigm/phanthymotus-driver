"""Actual /2 start paths reject incompatible declarations before any output.

No ROS/device is opened: reuse the real model/gate fixture and compile only the
real ArmPlugin method to avoid unrelated vendor imports, as the chain test does.
"""
import ast
import copy

import pytest

from test_motion_control import DRIVER, MotionControl, chain
from tianyi_motion.protocol import validate_descriptor


@pytest.fixture
def cards(chain, monkeypatch):
    c, arm = chain.c, chain.e.arm
    monkeypatch.setattr(c, 'start', MotionControl.start.__get__(c))
    cls = next(n for n in ast.parse((DRIVER/'device.py').read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'ArmPlugin')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'dispatch')
    scope = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(DRIVER/'device.py'), 'exec'), scope)
    arm.dispatch = lambda action, args: scope['dispatch'](arm, action, args)
    return chain


def set_field(value, path, replacement):
    for key in path[:-1]:
        value = value[key]
    value[path[-1]] = replacement


BAD_FIELDS = [
    (('schema',), 'motus.control/1'),
    (('control_interface',), 'motus.control/1'),
    (('protocol_version',), 2.0),
    (('mode',), 'eef_pose'),
    (('dof',), 14.0),
    (('dof',), True),
    (('frame',), 'other_torso'),
    (('model_version',), 'f'*64),
    (('calibration_version',), None),
    (('units', 'angle'), 'deg'),
    (('units', 'time'), 'ms'),
    (('groups', 0, 'offset'), 0.0),
    (('groups', 0, 'offset'), False),
    (('groups', 1, 'count'), 7.0),
    (('groups', 1, 'offset'), 0),
    (('groups', 0, 'mode'), 'eef_pose'),
    (('groups', 0, 'unit'), 'deg'),
    (('groups', 0, 'resource'), 'hand_l'),
    (('groups', 0, 'name'), 'arm_r'),
    (('groups',), []),
    (('joint_names', 1), 'unknown_joint'),
    (('joint_names',), ['duplicate']*14),
    (('joint_names',), []),
    (('rate', 'max_hz'), 60),
    (('rate', 'expected_hz'), 20),
    (('rate', 'watchdog_ms'), 300),
    (('rate', 'max_hz'), float('nan')),
    (('rate', 'max_hz'), 10**400),
    (('rate', 'expected_hz'), True),
    (('limits', 'lower', 0), -3),
    (('limits', 'upper', 0), float('inf')),
    (('limits', 'max_velocity', 0), True),
    (('limits', 'max_velocity', 0), 0),
    (('limits', 'max_velocity'), [1]*13),
    (('limits',), None),
]


@pytest.mark.parametrize('tool', ['motion_control', 'arm'])
@pytest.mark.parametrize('path,replacement', BAD_FIELDS)
def test_actual_start_rejects_bad_descriptor_then_accepts_current(cards, tool, path, replacement):
    c, arm = cards.c, cards.e.arm
    target = c if tool == 'motion_control' else arm
    good = c.control_interface('joint_position')
    bad = copy.deepcopy(good)
    set_field(bad, path, replacement)
    reply = target.dispatch('start', {'control_interface': bad})
    code = 'motion_control_execution_binding_mismatch' if tool == 'motion_control' else 'arm_control_interface_mismatch'
    assert reply['state'] == 'error' and reply['code'] == code
    assert c._worker is None and not cards.e.gate.session_id
    assert not cards.commands and not cards.p.writes
    assert target.dispatch('start', {'control_interface': good})['state'] == 'ready'
    assert not cards.e.gate.session_id and not cards.commands and not cards.p.writes


@pytest.mark.parametrize('tool', ['motion_control', 'arm'])
@pytest.mark.parametrize('bad', [None, [], {}, 'joint_position'])
def test_actual_start_rejects_non_descriptor(cards, tool, bad):
    target = cards.c if tool == 'motion_control' else cards.e.arm
    assert target.dispatch('start', {'control_interface': bad})['state'] == 'error'
    assert cards.c._worker is None and not cards.e.gate.session_id and not cards.p.writes


@pytest.mark.parametrize('mode', ['eef_pose', 'joint_position'])
@pytest.mark.parametrize('field', [
    'control_interface', 'schema', 'protocol_version', 'mode', 'dof',
    'model_version', 'calibration_version', 'frame', 'units', 'groups', 'rate',
])
def test_required_fields_cannot_be_omitted(chain, mode, field):
    good = chain.c.control_interface(mode)
    bad = copy.deepcopy(good)
    del bad[field]
    with pytest.raises(ValueError, match='invalid_control_descriptor'):
        validate_descriptor(bad, good)


@pytest.mark.parametrize('path,replacement', [
    (('effector_ids',), ['right', 'left']),
    (('effector_ids',), None),
    (('units', 'orientation'), 'wxyz'),
    (('units', 'position'), 'cm'),
    (('rate', 'watchdog_ms'), 100),
])
def test_eef_has_pose_contract_not_joint_metadata(chain, path, replacement):
    good = chain.c.control_interface('eef_pose')
    assert 'joint_names' not in good and 'limits' not in good and 'force_torque' not in good
    validate_descriptor(good, good)
    bad = copy.deepcopy(good)
    set_field(bad, path, replacement)
    with pytest.raises(ValueError, match='invalid_control_descriptor'):
        validate_descriptor(bad, good)


@pytest.mark.parametrize('tool', ['motion_control', 'arm'])
def test_start_allows_extensions_and_equal_numeric_capabilities(cards, tool):
    c = cards.c
    target = c if tool == 'motion_control' else cards.e.arm
    value = c.control_interface('joint_position')
    value.update(display_name='future optional field', force_torque=None)
    value['groups'][0]['display_name'] = 'left'
    value['units']['display_hint'] = 'radians'
    value['rate'].update(max_hz=50.0, expected_hz=50.0, watchdog_ms=100.0, extension=True)
    value['limits'].update(lower=[-2]*14, upper=[2]*14, max_velocity=[1]*14, extension=True)
    assert target.dispatch('start', {'control_interface': value})['state'] == 'ready'
    assert not cards.e.gate.session_id and not cards.p.writes


def test_port_descriptor_must_match_even_when_singular_is_correct(cards):
    c = cards.c
    good = c.control_interface('joint_position')
    bad = {**good, 'model_version': 'f'*64}
    assert c.dispatch('start', {'control_interfaces': {'joints': bad}})['state'] == 'error'
    assert c.dispatch('start', {'control_interface': good,
                              'control_interfaces': {'joints': bad}})['state'] == 'error'
    assert c.dispatch('start', {'control_interface': {},
                              'control_interfaces': {'joints': good}})['state'] == 'error'
    assert c.dispatch('start', {'control_interfaces': []})['state'] == 'error'
    assert c._worker is None
    assert c.dispatch('start', {'control_interfaces': {'joints': good}})['state'] == 'ready'
    assert not cards.e.gate.session_id and not cards.p.writes


@pytest.mark.parametrize('tool', ['motion_control', 'arm'])
def test_legacy_no_descriptor_start_stays_available(cards, tool):
    target = cards.c if tool == 'motion_control' else cards.e.arm
    assert target.dispatch('start', {})['state'] == 'ready'
    assert not cards.e.gate.session_id and not cards.p.writes


def test_descriptor_producer_rejects_unknown_mode(chain):
    with pytest.raises(ValueError, match='invalid_control_interface'):
        chain.c.control_interface('velocity')
