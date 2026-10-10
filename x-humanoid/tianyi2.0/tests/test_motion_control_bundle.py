"""Actual Bundle/Arm/Hand/executor constructors, scoped ROS allocation stubs.

No start, claim, DDS process, publisher or hardware action runs in these tests.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType

import pytest

from test_servo import FakeROS2, device_mod, ros_stubs  # noqa: F401
from teleop_executor import ARM_NAMES

DRIVER = Path(__file__).resolve().parents[1]


@pytest.fixture
def bundle_class(monkeypatch, ros_stubs):
    # Import the complete entrypoint; only ROS allocation and stdout wrapping
    # are replaced. The constructor under test is not copied through AST.
    from common import logsafe
    monkeypatch.setattr(logsafe, 'install', lambda: None)
    monkeypatch.syspath_prepend(str(DRIVER))
    executors = ModuleType('rclpy.executors')
    context = ModuleType('rclpy.context')
    context.Context = type('Context', (), {})
    monkeypatch.setitem(sys.modules, 'rclpy.executors', executors)
    monkeypatch.setitem(sys.modules, 'rclpy.context', context)
    monkeypatch.setattr(sys.modules['rclpy'], 'executors', executors, raising=False)
    # Optional microphone discovery imports cv2 in production; it is unrelated
    # to constructing these three motion cards and must never run here.
    external = ModuleType('ext_devices')
    def unexpected_discovery(*args, **kwargs):raise AssertionError('device discovery was not requested')
    external._parse_arecord_output = external._parse_hw_params = unexpected_discovery
    monkeypatch.setitem(sys.modules, 'ext_devices', external)
    spec = importlib.util.spec_from_file_location('_tianyi_bundle_motion_test', DRIVER/'main.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.TianyiDeviceBundle


def calibration(tmp_path, hands=False):
    urdf = tmp_path/'model.urdf'
    urdf.write_text('<robot name="bundle-test">'+''.join(
        f'<joint name="{name}"><limit lower="-1" upper="1" velocity="1"/></joint>'
        for name in ARM_NAMES)+'</robot>')
    # This fixture tests the executor profile loader, not numerical geometry.
    data = {'schema':'motus.tianyi-calibration.v1', 'urdf_path':str(urdf),
        'urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),
        'arm_joint_names':ARM_NAMES, 'joint_velocity_rad_s':.2, 'hands_enabled':hands,
        'hands':{side:{'open':[0]*6,'closed':[80]*6} for side in ('left','right')}}
    path = tmp_path/'calibration.json'
    path.write_text(json.dumps(data))
    return path


def config(path, *, motion=True, arm=True, hand=False):
    return {'plugins':{'arm':{'enabled':arm},'hand':{'enabled':hand}},
        'teleop':{'enabled':True,'live_enabled':True,'calibration_path':str(path)},
        'motion_control':{'enabled':motion,'calibration_path':str(path)}}


def test_live_arms_only_actual_bundle_initializes_without_hand_or_claim(bundle_class, tmp_path):
    path = calibration(tmp_path)
    cfg = config(path)
    # The motion-control path must also override an older executor path.
    cfg['teleop']['calibration_path'] = str(tmp_path/'not-present.json')
    bundle = bundle_class(cfg, 'offline', FakeROS2(), None)
    executor = bundle._teleop
    assert executor.profile_error is None and executor.profile['hands_enabled'] is False
    assert executor.hand is None and isinstance(executor.arm, device_mod.ArmPlugin)
    assert executor.motion_control in bundle._plugins
    assert executor.arm._motion_control is executor.motion_control
    assert executor.gate.live_enabled and not executor.gate.session_id
    assert executor.node is None and executor._bus_process is None
    assert executor.arm._pos_publisher is None and executor.arm._ctrl_publisher is None
    assert not executor._output_ready and executor.motion_control._worker is None


@pytest.mark.parametrize('profile_mode', ['hands', 'unspecified', 'invalid', 'missing'])
def test_live_missing_hand_is_not_allowed_by_unvalidated_or_hand_profile(bundle_class, tmp_path, profile_mode):
    path = calibration(tmp_path, hands=profile_mode == 'hands')
    data = json.loads(path.read_text())
    if profile_mode == 'unspecified':data.pop('hands_enabled')
    if profile_mode == 'invalid':data['urdf_sha256'] = '0'*64
    path.write_text(json.dumps(data))
    if profile_mode == 'missing':path = tmp_path/'missing.json'
    with pytest.raises(ValueError, match='requires arm and hand'):
        bundle_class(config(path), 'offline', FakeROS2(), None)


def test_legacy_hand_requirement_and_required_arm_are_preserved(bundle_class, tmp_path):
    path = calibration(tmp_path)
    with pytest.raises(ValueError, match='requires arm and hand'):
        bundle_class(config(path, motion=False), 'offline', FakeROS2(), None)
    with pytest.raises(ValueError, match='requires arm and hand'):
        bundle_class(config(path, arm=False, hand=True), 'offline', FakeROS2(), None)


def test_legacy_hands_profile_initializes_with_both_plugins_but_does_not_start(bundle_class, tmp_path):
    path = calibration(tmp_path, hands=True)
    bundle = bundle_class(config(path, motion=False, hand=True), 'offline', FakeROS2(), None)
    executor = bundle._teleop
    assert executor.profile['hands_enabled'] is True
    assert isinstance(executor.hand, device_mod.HandPlugin)
    assert executor.hand._left_pub is None and executor.hand._right_pub is None
    assert not executor.gate.session_id and not executor._output_ready
