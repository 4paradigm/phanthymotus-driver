import json
import sys
import types

import pytest

from test_direction_card import _card, _load_card


def test_only_sensor_card_and_shared_settings(monkeypatch):
    module = _load_card(monkeypatch)
    card = module.SoundDirectionPlugin({}, 'robot', types.SimpleNamespace(add_node=lambda _: None))
    assert [tool['name'] for tool in card.get_tools()] == ['sound_direction']
    tool = card.get_tool()
    assert tool['topic_out'][0]['format'] == 'sensor/sound-direction-v1'
    assert set(tool['configSchema']['properties']) == set(module.DEFAULT_PARAMETERS)
    assert all(field['scope'] == 'shared' for field in tool['configSchema']['properties'].values())
    assert tool['inputSchema']['properties']['action']['enum'] == ['info', 'start', 'stop']


def test_complete_calibration_file_falls_back_and_rejects_nonfinite(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    local = tmp_path / 'local.json'
    default = tmp_path / 'default.json'
    monkeypatch.setattr(module, 'CALIBRATION_PATH', local)
    monkeypatch.setattr(module, 'DEFAULT_CALIBRATION_PATH', default)
    default.write_text(json.dumps({'front': [1, 0, 0], 'right': [0, 1, 0]}))
    assert module.load_calibration()['right'] == [0, 1, 0]
    local.write_text(json.dumps({'front': [2, 0, 0], 'right': [0, 2, 0]}))
    assert module.load_calibration()['right'] == [0, 2, 0]
    local.write_text('{"front":[NaN,0,0],"right":[0,2,0]}')
    assert module.load_calibration()['right'] == [0, 1, 0]
    default.write_text('{"front":[1,0,0]}')
    assert module.load_calibration() == {}


def test_degenerate_calibration_falls_back_or_blocks_start(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    local = tmp_path / 'local.json'
    default = tmp_path / 'default.json'
    monkeypatch.setattr(module, 'CALIBRATION_PATH', local)
    monkeypatch.setattr(module, 'DEFAULT_CALIBRATION_PATH', default)
    local.write_text(json.dumps({'front': [1, 0, 0], 'right': [1, 0.01, 0]}))
    default.write_text(json.dumps({'front': [1, 0, 0], 'right': [0, 1, 0]}))
    assert module.load_calibration()['right'] == [0, 1, 0]

    default.write_text(json.dumps({'front': [1, 0, 0], 'right': [1, 0, 0]}))
    assert module.load_calibration() == {}
    card = _card(module)
    assert card.dispatch('start', {})['state'] == 'unavailable'
    assert card.dispatch('info', {})['calibrated_directions'] == []


def test_angle_boundary_and_window_end_use_last_frame_samples(monkeypatch):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, 'estimate_signature', lambda *args: (1, 0, 0))
    monkeypatch.setattr(module, 'estimate_angle', lambda *args: 270)
    msg = module.activity_payload([1] * (8 * 1024),
        {'front': [1, 0, 0], 'right': [0, 1, 0]},
        (1700000000000000, 1700000000020000, 320), 1700000000040000)
    assert msg.state == 1
    assert msg.angle_deg == -90
    assert msg.window_start_us == 1700000000000000
    assert msg.window_end_us == 1700000000040000
    assert msg.published_us == 1700000000040000


@pytest.mark.parametrize('timestamps, expected_end', [
    ((1700000000000000, 1700000000020000), 1700000000060000),
    ((1700000000000000000, 1700000000020000000), 0),
])
def test_worker_publishes_structured_direction_message(monkeypatch, tmp_path, timestamps, expected_end):
    module = _load_card(monkeypatch)
    calibration = tmp_path / 'calibration.json'
    calibration.write_text(json.dumps({'front': [1, 0, 0], 'right': [0, 1, 0]}))
    monkeypatch.setattr(module, 'CALIBRATION_PATH', calibration)
    monkeypatch.setattr(module, 'SoundActivityGate', lambda **kw:
                        types.SimpleNamespace(accepts=lambda _: True))
    monkeypatch.setattr(module, 'estimate_signature', lambda *args: (1, 0, 0))
    monkeypatch.setattr(module, 'estimate_angle', lambda *args: 270)
    published = []

    class Node:
        def __init__(self, name): pass
        def create_publisher(self, message_type, topic, qos):
            assert message_type is module.SoundDirection
            return types.SimpleNamespace(publish=published.append)

    class Done(BaseException): pass
    frames = iter(timestamps)
    def capture():
        try: timestamp_us = next(frames)
        except StopIteration: raise Done()
        return types.SimpleNamespace(timestamp_us=timestamp_us,
            channels=8, sample_rate=16000, audio_data=[100] * (8 * 640))

    monkeypatch.setattr(module, 'Node', Node)
    monkeypatch.setattr(module.rclpy, 'init', lambda: None, raising=False)
    monkeypatch.setitem(sys.modules, 'common', types.SimpleNamespace(
        logsafe=types.SimpleNamespace(install=lambda **kw: None)))
    monkeypatch.setitem(sys.modules, 'mediacontrol_py', types.SimpleNamespace(
        MediaController=types.SimpleNamespace(instance=lambda:
            types.SimpleNamespace(init=lambda: None, get_audio_capture_data=capture))))
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    try: module._direction_subprocess('robot')
    except Done: pass
    assert len(published) == 1
    assert published[0].angle_deg == -90
    assert published[0].window_end_us == expected_end
