"""Configuration migration must not create an operator session or publish motion."""
import json
from types import SimpleNamespace
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop_control import TeleopControl


def test_legacy_canvas_settings_cannot_override_driver_preset(tmp_path):
    path = tmp_path / 'settings.json'
    path.write_text(json.dumps({'mode': 'shadow', 'calibration_path': '/old', 'position_scale': .1}))
    executor = SimpleNamespace(cfg={})
    motion = SimpleNamespace(executor=executor, clock_id='offline')
    card = TeleopControl({'state_path': str(path), 'mode': 'live',
                         'calibration_path': '/driver/model.json', 'position_scale': 1.}, motion)
    assert card.live and card._config_error is None
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
    card.binding = {'namespace':'', 'instance_id':None, 'command_topic':'/teleop/command', 'feedback_topic':'/teleop/state'}
    pose = {'tracked':True, 'position':[0.,0.,1.], 'orientation_xyzw':[0.,0.,0.,1.]}
    frame = {'schema':'motus.teleop.command/1','kind':'input','instance_id':'card-random',
      'device_id':'paired-device','connection_epoch':1,'space_epoch':1,'sequence':1,
      'source_monotonic_ns':1,'received_monotonic_ns':1_000_000_000,'clock_id':'offline',
      'tracking_frame':'tracking_x_forward_y_left_z_up','head_reference':dict(pose),
      'left':dict(pose,grip=0.,trigger=0.),'right':dict(pose,grip=0.,trigger=0.)}
    return card, frame


def test_fixed_topic_pins_valid_source_and_rejects_other_instances(tmp_path):
    import pytest
    card, frame = input_card(tmp_path)
    assert card.receive(frame)
    assert card.binding['instance_id'] == 'card_random'
    assert card.receive(frame) is False
    with pytest.raises(ValueError, match='input_binding_mismatch'):
        card.receive(dict(frame, instance_id='another-device-card', sequence=2))
    with pytest.raises(ValueError, match='device_input_only'):
        card.receive({'kind':'operation','action':'begin'})


def test_canvas_session_prepares_only_after_both_grips_released(tmp_path):
    card, frame = input_card(tmp_path)
    calls=[]
    def begin(generation, action):
        calls.append(action)
        card._operator='session'
    card._begin=begin
    frame['left']['grip']=frame['right']['grip']=1.
    card.receive(frame);card.step()
    assert calls == [] and card._reason == 'release_grips_before_enable'
    frame['sequence']=2;frame['left']['grip']=frame['right']['grip']=0.
    card.receive(frame);card.step()
    assert calls == ['begin']
    card._hold=lambda *a,**k:None
    card.step()
    assert calls == ['begin']
    # A real tracking-space reset requires neutral input before another baseline.
    card._needs_calibration=True;card.step()
    assert calls == ['begin','calibrate']


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
