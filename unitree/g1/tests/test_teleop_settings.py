"""Configuration migration must not create an operator session or publish motion."""
import pytest
from types import SimpleNamespace
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop_control import TeleopControl


def test_driver_preset_and_fixed_canvas_schema():
    executor = SimpleNamespace(cfg={})
    motion = SimpleNamespace(executor=executor, clock_id='offline')
    card = TeleopControl({'mode': 'live',
                         'calibration_path': '/driver/model.json', 'position_scale': 1.}, motion)
    assert card.live
    assert card.cfg['calibration_path'] == '/driver/model.json'
    assert card.mapping.scale == 1.
    assert card._operator is None and card._thread is None
    tool = card.get_tool()
    assert set(tool['configSchema']['properties']) == {'usage_guide'}
    assert 'teleop_device' in tool['description']
    assert 'instance_id' not in tool['inputSchema']['properties']
    assert 'usage_guide' not in tool['inputSchema']['properties']
    assert tool['inputSchema']['x-action-params']['config']['params'] == []


def test_internal_shadow_is_still_supported(tmp_path):
    motion = SimpleNamespace(executor=SimpleNamespace(cfg={}), clock_id='offline')
    card = TeleopControl({'state_path': str(tmp_path / 'none'), 'mode': 'shadow'}, motion)
    assert not card.live and not card.executor.cfg['live_enabled']


def input_card(tmp_path):
    motion = SimpleNamespace(executor=SimpleNamespace(cfg={}), clock_id='offline')
    card = TeleopControl({'state_path': str(tmp_path / 'none'), 'mode': 'shadow'}, motion, clock=lambda: 1_000_000_000)
    card.binding = {'command_topic':'/teleop/command', 'feedback_topic':'/teleop/state'}
    pose = {'tracked':True, 'position':[0.,0.,1.], 'orientation_xyzw':[0.,0.,0.,1.]}
    frame = {'schema':'motus.teleop.command/1','kind':'input','instance_id':'card-random',
      'device_id':'paired-device','connection_epoch':1,'space_epoch':1,'sequence':1,
      'source_monotonic_ns':1,'received_monotonic_ns':1_000_000_000,'clock_id':'offline',
      'tracking_frame':'tracking_x_forward_y_left_z_up','head_reference':dict(pose),
      'left':dict(pose,grip=0.,trigger=0.),'right':dict(pose,grip=0.,trigger=0.)}
    return card, frame


def test_canvas_input_accepts_new_source_and_preserves_generation_checks(tmp_path):
    card, frame = input_card(tmp_path)
    assert card.receive(frame)
    assert card.receive(frame) is False
    assert card.receive(dict(frame, instance_id='another-device-card', sequence=2))
    card._operator = 'active-session'
    card._mapping_identity = (frame['device_id'], frame['space_epoch'])
    changed = dict(frame, instance_id='new-card', device_id='new-device', sequence=1)
    assert card.receive(changed) and card._needs_calibration
    with pytest.raises(ValueError, match='old_input_generation'):
        card.receive(dict(changed, connection_epoch=0, sequence=2))
    with pytest.raises(ValueError, match='device_input_only'):
        card.receive({'kind':'operation','action':'begin'})




def test_monitor_summary_has_execution_state_and_escapes_html(tmp_path):
    card, _ = input_card(tmp_path)
    status=dict(state='idle', reason=None, output_active=False, ownership_held=False,
                hold_confirmed=False, stop_confirmed=False, applied_sequence=-1, feedback={})
    card.motion.gate=SimpleNamespace(status=lambda:status)
    card.motion._decision={};card.motion._finish={}
    card._reason='<test>'
    value=card.feedback()
    assert '执行：idle' in value['text'] and '&lt;test&gt;' in value['text']
    assert '<test>' not in value['text']


def test_arm_release_fences_input_until_explicit_canvas_start(tmp_path):
    card, frame = input_card(tmp_path)
    cancelled = []
    card.motion.cancel_pending = lambda: cancelled.append(True)
    assert card.receive(frame)
    card._operator = 'previous-session'
    generation = card._generation
    card.cancel_for_arm_release()
    assert card._closed.is_set() and card._operator is None and card._latest is None
    assert card._generation == generation+1 and cancelled == [True]
    frame['sequence'] += 1
    assert card.receive(frame) is False


@pytest.mark.parametrize('mode,scale,error', [('invalid', 1., 'invalid_mode'),
    ('live', 0., 'invalid_position_scale'), ('live', float('nan'), 'invalid_position_scale'),
    ('live', True, 'invalid_position_scale'), ('live', 1.1, 'invalid_position_scale')])
def test_driver_preset_is_validated_at_startup(mode, scale, error):
    motion = SimpleNamespace(executor=SimpleNamespace(cfg={}), clock_id='offline')
    with pytest.raises(ValueError, match=error):
        TeleopControl({'mode': mode, 'position_scale': scale}, motion)
    assert motion.executor.cfg == {}


def test_config_action_does_not_reset_active_mapping():
    motion = SimpleNamespace(executor=SimpleNamespace(cfg={}), clock_id='offline')
    card = TeleopControl({}, motion)
    anchor = object()
    card.mapping.anchor = anchor
    card._operator = 'active-session'
    result = card.dispatch('config', {'mode': 'shadow', 'position_scale': .1,
                                      'calibration_path': '/old', 'usage_guide': '无需配置'})
    assert result['operator_session_id'] == 'active-session'
    assert card.mapping.anchor is anchor and card.mapping.scale == 1.
    assert card.live
