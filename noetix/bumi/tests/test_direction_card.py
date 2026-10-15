"""Contract tests for Bumi's single sound-direction sensor."""

import importlib.util
import json
import queue
import sys
import threading
import time
import types
from pathlib import Path

import yaml

from test_mic_card import _load_device


def _load_card(monkeypatch):
    _load_device(monkeypatch)
    path = Path(__file__).resolve().parents[1] / 'direction_card.py'
    monkeypatch.syspath_prepend(str(path.parent))
    spec = importlib.util.spec_from_file_location('bumi_direction_card_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _card(module):
    return module.SoundDirectionPlugin({}, 'robot',
        types.SimpleNamespace(add_node=lambda node: None))


def test_bundle_exposes_only_one_direction_card(monkeypatch, tmp_path):
    device, _ = _load_device(monkeypatch)
    direction = _load_card(monkeypatch)
    monkeypatch.setitem(sys.modules, 'device', device)
    monkeypatch.setitem(sys.modules, 'direction_card', direction)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', types.ModuleType('rclpy.executors'))
    path = Path(__file__).resolve().parents[1] / 'main.py'
    spec = importlib.util.spec_from_file_location('bumi_direction_bundle_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    cfg = {'plugins': {'mic': {'enabled': True}, 'sound_direction': {'enabled': True}}}
    bundle = module.BumiDeviceBundle(cfg, 'robot',
        types.SimpleNamespace(add_node=lambda node: None), None, object())
    assert {tool['name'] for tool in bundle.get_all_tools()} == {'mic', 'sound_direction'}
    manifest = yaml.safe_load((path.parent / 'driver.yaml').read_text())
    assert 'sound_direction_control' not in {card['name'] for card in manifest['cards']}
    monkeypatch.setattr(direction, 'load_calibration', lambda: {})
    monkeypatch.setattr(direction, 'SETTINGS_PATH', tmp_path / 'settings.json')
    assert bundle.dispatch('sound_direction', {
        'action': 'config', 'sensitivity': 'low'})['state'] == 'configured'
    assert json.loads((tmp_path / 'settings.json').read_text())['sensitivity'] == 'low'
    assert bundle.dispatch('sound_direction', {'action': 'info'})['state'] == 'unavailable'
    assert bundle.dispatch('sound_direction', {'action': 'start'})['state'] == 'unavailable'
    assert bundle.dispatch('sound_direction', {'action': 'stop'}) == {'state': 'idle'}


def test_missing_calibration_exposes_unavailable_direction(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, 'CALIBRATION_PATH', tmp_path / 'override.json')
    monkeypatch.setattr(module, 'DEFAULT_CALIBRATION_PATH', tmp_path / 'packaged.json')
    config = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config.yaml').read_text())
    assert config['plugins']['sound_direction']['enabled'] is True
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *args, **kwargs:
                        (_ for _ in ()).throw(AssertionError('worker must not start')))
    card = _card(module)
    assert card.start() is False
    info = card.dispatch('info', {})
    assert info['state'] == 'unavailable'
    assert info['calibrated_directions'] == []
    assert 'calibration' in card.dispatch('start', {})['message']


def test_dispatch_stop_keeps_worker_and_publishes_nothing(monkeypatch):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, 'load_calibration', lambda: {'front': [1, 0, 0], 'right': [0, 1, 0]})
    card = _card(module)
    terminated = []

    class Process:
        stdout = ()
        def poll(self): return None
        def terminate(self): terminated.append(True)
        def wait(self, timeout=None): return 0

    proc = Process()
    card._proc = proc
    valid = module.SoundDirection()
    valid.state = valid.VALID
    valid.angle_deg = -90
    card._on_direction(valid)
    published_before = len(card._node.published)

    result = card.dispatch('stop', {})

    assert result == {'state': 'idle'}
    assert card._proc is proc          # 与其他 sensor 卡一致：stop 不碰 worker
    assert terminated == []
    assert len(card._node.published) == published_before  # 不发 NO_EVENT
    info = card.dispatch('info', {})
    assert info['state'] == 'running'
    assert info['sound_direction']['angle_deg'] == -90


def test_dispatch_start_never_spawns(monkeypatch):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, 'load_calibration', lambda: {'front': [1, 0, 0], 'right': [0, 1, 0]})
    spawns = []

    def fake_popen(*args, **kwargs):
        spawns.append(True)
        return types.SimpleNamespace(stdout=(), poll=lambda: None,
                                     terminate=lambda: None, wait=lambda **kwargs: 0)
    monkeypatch.setattr(module.subprocess, 'Popen', fake_popen)
    card = _card(module)
    card._proc = None

    result = card.dispatch('start', {})

    assert spawns == []                # 与其他 sensor 卡一致：start 永不 spawn
    assert result == {'state': 'idle', 'topic_out': card.get_tool()['topic_out']}

def test_packaged_robot_calibration_starts_without_machine_override(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, 'CALIBRATION_PATH', tmp_path / 'missing.json')
    calibration = module.load_calibration()
    assert set(calibration) == {'front', 'right'}
    assert module.estimate_angle(calibration['front'], **calibration) == 0
    assert module.estimate_angle(calibration['right'], **calibration) == 270   # 左正：右侧 -90°
    proc = types.SimpleNamespace(stdout=(b'__BUMI_DIRECTION_READY__\n',),
        poll=lambda: None, terminate=lambda: None, wait=lambda **kwargs: 0)
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *args, **kwargs: proc)
    card = _card(module)
    assert card.dispatch('info', {})['calibrated_directions'] == ['front', 'right']
    assert card.dispatch('start', {})['state'] == 'idle'   # 纯状态应答，不 spawn
    assert card.start() is True                            # spawn 只走 plugin.start()
    assert card.dispatch('start', {})['state'] == 'running'
    card.stop()


def test_config_saves_sensitivity_and_rejects_invalid(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    path = tmp_path / 'settings.json'
    monkeypatch.setattr(module, 'SETTINGS_PATH', path)
    card = _card(module)
    assert card.dispatch('config', {'sensitivity': 'low'})['state'] == 'configured'
    assert json.loads(path.read_text())['sensitivity'] == 'low'
    assert card.dispatch('config', {'sensitivity': 'turbo'})['state'] == 'invalid_parameters'
    assert card.dispatch('config', {'update_interval_ms': 50})['state'] == 'invalid_parameters'
    assert json.loads(path.read_text())['sensitivity'] == 'low'
    assert card.dispatch('set_parameters', {'sensitivity': 'normal'}) is None


def test_config_restarts_worker_and_waits_for_ready(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, 'SETTINGS_PATH', tmp_path / 'settings.json')
    monkeypatch.setattr(module, 'load_calibration', lambda: {'front': [1, 0, 0], 'right': [0, 1, 0]})
    card = _card(module)
    events = []

    class Process:
        stdout = (b'__BUMI_DIRECTION_READY__\n',)
        def poll(self): return None
        def terminate(self): events.append('stop')
        def wait(self, timeout=None): events.append('reaped'); return 0

    card._proc = Process()
    def spawn(*args, **kwargs):
        events.append(('spawn', module.load_parameters()))
        return Process()
    monkeypatch.setattr(module.subprocess, 'Popen', spawn)
    result = card.dispatch('config', {'sensitivity': 'low'})
    assert result['state'] == 'configured'
    assert events == ['stop', 'reaped', ('spawn', result['parameters'])]


def test_config_reports_failed_worker_restart(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, 'SETTINGS_PATH', tmp_path / 'settings.json')
    monkeypatch.setattr(module, 'load_calibration', lambda: {'front': [1, 0, 0], 'right': [0, 1, 0]})
    card = _card(module)
    card._proc = types.SimpleNamespace(poll=lambda: None)
    starts = []
    monkeypatch.setattr(card, 'stop', lambda: setattr(card, '_proc', None))
    def restart():
        starts.append(module.load_parameters()['sensitivity'])
        if len(starts) == 1:
            raise OSError('spawn failed')
        card._proc = types.SimpleNamespace(poll=lambda: None)
        return True
    monkeypatch.setattr(card, 'start', restart)
    result = card.dispatch('config', {'sensitivity': 'low'})
    assert result['state'] == 'error'
    assert starts == ['low', 'normal']
    assert result['running_parameters']['sensitivity'] == 'normal'
    assert result['file_parameters']['sensitivity'] == 'normal'
    assert module.load_parameters()['sensitivity'] == 'normal'


def test_worker_exit_clears_published_direction(monkeypatch):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, 'load_calibration', lambda: {'front': [1, 0, 0], 'right': [0, 1, 0]})
    card = _card(module)
    lines = queue.Queue()
    lines.put(b'__BUMI_DIRECTION_READY__\n')

    class Process:
        exit_code = None
        def __init__(self):
            self.stdout = self._lines()
        def _lines(self):
            while True:
                line = lines.get()
                if line is None:
                    return
                yield line
        def poll(self): return self.exit_code
        def wait(self, timeout=None): return self.exit_code

    proc = Process()
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *args, **kwargs: proc)
    assert card.start() is True
    valid = module.SoundDirection()
    valid.state = valid.VALID
    valid.angle_deg = 90
    card._on_direction(valid)
    proc.exit_code = 137
    lines.put(None)
    deadline = time.monotonic() + 1
    while not card._node.published and time.monotonic() < deadline:
        time.sleep(0.01)
    assert card._node.published[-1].state == module.SoundDirection.NO_EVENT
    assert card._proc is None
    assert card._last_direction is None


def test_ready_timeout_waits_15_seconds_and_returns_false(monkeypatch):
    module = _load_card(monkeypatch)
    monkeypatch.setattr(module, 'load_calibration', lambda: {'front': [1, 0, 0], 'right': [0, 1, 0]})
    card = _card(module)
    waits = []

    class Event:
        def set(self): pass
        def wait(self, timeout):
            waits.append(timeout)
            return False

    class Process:
        stdout = ()
        def poll(self): return None
        def terminate(self): pass
        def wait(self, timeout=None): return 0

    monkeypatch.setattr(module, 'threading', types.SimpleNamespace(
        Event=Event, Thread=threading.Thread, RLock=threading.RLock))
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *args, **kwargs: Process())
    assert card.start() is False
    assert waits == [15]


def test_no_event_clears_last_direction(monkeypatch):
    module = _load_card(monkeypatch)
    card = _card(module)
    card._proc = types.SimpleNamespace(poll=lambda: None)
    valid = module.SoundDirection()
    valid.state = valid.VALID
    valid.angle_deg = -90
    card._on_direction(valid)
    assert card.dispatch('info', {})['sound_direction']['angle_deg'] == -90
    clear = module.SoundDirection()
    clear.state = clear.NO_EVENT
    card._on_direction(clear)
    assert card.dispatch('info', {})['sound_direction'] == {'state': 'no_event'}


def test_stop_publishes_no_event(monkeypatch):
    # 钉住 plugin.stop() 的驱动关停清屏路径；dispatch('stop') 由
    # test_dispatch_stop_keeps_worker_and_publishes_nothing 覆盖（不再清屏）。
    module = _load_card(monkeypatch)
    card = _card(module)
    card.stop()
    assert card._node.published[-1].state == module.SoundDirection.NO_EVENT


def test_legacy_settings_file_maps_to_presets(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    path = tmp_path / 'settings.json'
    monkeypatch.setattr(module, 'SETTINGS_PATH', path)

    legacy = {'onset_level': 10.0, 'onset_ratio': 2.0, 'burst_level': 15.0,
              'burst_ratio': 4.0, 'update_interval_ms': 100}
    path.write_text(json.dumps(legacy))
    assert module.load_parameters() == {'sensitivity': 'normal'}

    path.write_text(json.dumps({**legacy, 'onset_ratio': 2.6}))
    assert module.load_parameters() == {'sensitivity': 'low'}

    path.write_text(json.dumps({**legacy, 'onset_ratio': 1.5}))
    assert module.load_parameters() == {'sensitivity': 'high'}

    # 手调过的刷新节流一律丢弃，固定 100ms
    path.write_text(json.dumps({**legacy, 'update_interval_ms': 50}))
    assert module.load_parameters() == {'sensitivity': 'normal'}
    assert module.UPDATE_INTERVAL_MS == 100

    # 迁移不回写盘：文件保持旧格式
    assert set(json.loads(path.read_text())) == set(legacy)

    # 混键/非法/损坏 -> 新默认
    path.write_text(json.dumps({'onset_ratio': 2.0, 'sensitivity': 'low'}))
    assert module.load_parameters() == {'sensitivity': 'normal'}
    path.write_text('not json')
    assert module.load_parameters() == {'sensitivity': 'normal'}
    # 不可哈希/错误类型的值不得抛 TypeError，一律回默认
    path.write_text(json.dumps({'sensitivity': ['high']}))
    assert module.load_parameters() == {'sensitivity': 'normal'}
    path.write_text(json.dumps({'sensitivity': {'a': 1}}))
    assert module.load_parameters() == {'sensitivity': 'normal'}
    path.write_text(json.dumps({'sensitivity': 5}))
    assert module.load_parameters() == {'sensitivity': 'normal'}


def test_high_sensitivity_rejects_modest_rises_and_accepts_clear_speech(monkeypatch):
    module = _load_card(monkeypatch)
    gate = module.SoundActivityGate(**module.SENSITIVITY_GATE_PARAMS['high'])

    def frame(level):
        wave = module.np.tile([level, -level], 320)
        return module.np.stack([wave] * 4 + [module.np.zeros_like(wave)] * 4,
                               axis=1).reshape(-1)

    assert not any(gate.accepts(frame(40)) for _ in range(8))
    assert not gate.accepts(frame(70))
    assert not gate.accepts(frame(70))
    assert not gate.accepts(frame(80))
    assert gate.accepts(frame(80))


def test_sensitivity_preset_table_matches_documentation(monkeypatch):
    module = _load_card(monkeypatch)
    assert module.DEFAULT_PARAMETERS == {'sensitivity': 'normal'}
    assert set(module.SENSITIVITY_GATE_PARAMS) == {'high', 'normal', 'low'}
    # 三档全部逐值冻结：任何一档的语义漂移都必须炸测试（含后续风扇修复的改档）。
    assert module.SENSITIVITY_GATE_PARAMS == {
        'high':   {'onset_level': 8.0,  'onset_ratio': 1.8,
                   'burst_level': 12.0, 'burst_ratio': 3.2},
        'normal': {'onset_level': 10.0, 'onset_ratio': 2.0,
                   'burst_level': 15.0, 'burst_ratio': 4.0},
        'low':    {'onset_level': 15.0, 'onset_ratio': 2.5,
                   'burst_level': 25.0, 'burst_ratio': 5.0},
    }
    assert module.UPDATE_INTERVAL_MS == 100


def test_config_rejects_legacy_key_names(monkeypatch, tmp_path):
    module = _load_card(monkeypatch)
    path = tmp_path / 'settings.json'
    monkeypatch.setattr(module, 'SETTINGS_PATH', path)
    card = _card(module)
    assert card.dispatch('config', {'sensitivity': 'normal'})['state'] == 'configured'
    for legacy_key, value in [('onset_ratio', 2.4), ('burst_ratio', 0.5),
                              ('update_interval_ms', 50), ('onset_level', 12.0)]:
        assert card.dispatch('config', {legacy_key: value})['state'] == 'invalid_parameters'
    # 不可哈希/错误类型的值也要走 invalid_parameters，而不是 TypeError
    for bad in (['high'], {'a': 1}, 5):
        assert card.dispatch('config', {'sensitivity': bad})['state'] == 'invalid_parameters'
    assert json.loads(path.read_text()) == {'sensitivity': 'normal'}
