"""Instance contracts, resource sharing, preview and reconnect without ROS/USB."""
import ast
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'x-humanoid/tianyi2.0')]
SPEC = importlib.util.spec_from_file_location('wrist_cameras', ROOT / 'x-humanoid/tianyi2.0/wrist_cameras.py')
wrist = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = wrist
SPEC.loader.exec_module(wrist)


def device(id='realsense:abc', depth=True):
    return {'id': id, 'name': 'USB RGB-D' if depth else 'USB Webcam',
            'backend': 'realsense' if depth else 'uvc', 'serial': id.split(':')[-1],
            'path': '/dev/v4l/by-id/webcam', 'capabilities': {'rgb': True, 'depth': depth},
            'profiles': {'rgb': [{'width': 640, 'height': 480, 'fps': 15, 'format': 'rgb8'}],
                         'depth': [{'width': 640, 'height': 480, 'fps': 15, 'format': 'z16'}] if depth else []}}


class Session:
    created = []

    def __init__(self, namespace, device, profiles, jpeg_quality=85):
        self.device, self.profiles = device, profiles
        self.routes, self.closed, self.failed = {}, False, False
        self.created.append(self)

    def add(self, instance_id, side):
        self.routes[instance_id] = side

    def remove(self, instance_id):
        self.routes.pop(instance_id, None)

    def close(self):
        self.closed = True

    def healthy(self):
        return not self.closed and not self.failed

    def info(self, instance_id, side):
        return {'state': 'running', 'error': None, 'side': side,
                'streams': {'rgb': {'state': 'running'},
                            'depth': {'state': 'running' if self.device['capabilities']['depth'] else 'unavailable'}}}


@pytest.fixture
def plugin(monkeypatch, tmp_path):
    Session.created = []
    monkeypatch.setattr(wrist, 'WristCameraSession', Session)
    p = wrist.WristCameraPlugin({'state_file': str(tmp_path / 'wrist.json')}, 'robot')
    monkeypatch.setattr(wrist, 'discover_devices', lambda: {'devices': [device(), device('realsense:def')], 'errors': []})
    p._refresh()
    yield p
    p.stop()


def configure(p, id='card-a', side='left', target='realsense:abc', **settings):
    return p.dispatch('config', {'instance_id': id, 'side': side, 'device_id': target, **settings})


def test_one_repeatable_tool_has_two_ordered_outputs_and_only_instance_fields(plugin):
    tool, = plugin.get_tools()
    assert (tool['name'], tool['type'], tool['multiInstance']) == ('wrist_camera', 'sensor', True)
    assert [p['format'] for p in tool['topic_out']] == ['image/jpeg', 'image/depth-zlib']
    props = tool['configSchema']['properties']
    assert set(props) == {'side', 'device_id', 'fps', 'resolution'}
    assert all(v['scope'] == 'instance' for v in props.values())
    assert {v['const'] for v in props['device_id']['oneOf']} == {'auto', 'realsense:abc', 'realsense:def'}


def test_info_derives_both_topics_before_config_without_starting_a_camera(plugin):
    info = plugin.dispatch('info', {'instance_id': 'card-a'})
    assert info['state'] == 'unconfigured'
    assert [v['topic'] for v in info['topic_out']] == ['/robot/wrist_camera/card_a/rgb', '/robot/wrist_camera/card_a/depth']
    assert not Session.created


def test_two_devices_start_previews_on_config_and_stop_independently(plugin):
    assert configure(plugin)['state'] == 'running'
    configure(plugin, 'card-b', 'right', 'realsense:def')
    assert len(Session.created) == 2
    first, second = Session.created
    plugin.dispatch('stop', {'instance_id': 'card-a'})
    assert first.closed and not second.closed
    assert plugin.dispatch('info', {'instance_id': 'card-b'})['state'] == 'running'
    plugin.dispatch('start', {'instance_id': 'card-a'})
    assert len(Session.created) == 3


def test_shared_device_has_one_owner_and_stopping_one_keeps_other(plugin):
    configure(plugin)
    configure(plugin, 'card-b', 'right')
    session, = Session.created
    assert session.routes == {'card-a': 'left', 'card-b': 'right'}
    plugin.dispatch('stop', {'instance_id': 'card-a'})
    assert not session.closed and session.routes == {'card-b': 'right'}
    plugin.dispatch('stop', {'instance_id': 'card-b'})
    assert session.closed


def test_config_replay_does_not_reopen_capture_or_rearm_stopped_card(plugin):
    configure(plugin)
    original, = Session.created
    configure(plugin)
    assert Session.created == [original]
    plugin.dispatch('stop', {'instance_id': 'card-a'})
    assert configure(plugin)['state'] == 'idle'
    assert Session.created == [original]
    plugin.dispatch('start', {'instance_id': 'card-a'})
    assert len(Session.created) == 2


def test_side_change_preserves_topics_and_shared_capture(plugin):
    before = configure(plugin)
    configure(plugin, 'card-b', 'right')
    after = configure(plugin, side='right')
    assert after['side'] == 'right'
    assert before['topic_out'] == after['topic_out']
    assert len(Session.created) == 1


def test_swap_devices_while_stopped_preserves_instance_topics(plugin):
    first = configure(plugin)
    second = configure(plugin, 'card-b', 'right', 'realsense:def')
    plugin.dispatch('stop', {})
    assert configure(plugin, target='realsense:def')['topic_out'] == first['topic_out']
    assert configure(plugin, 'card-b', 'right')['topic_out'] == second['topic_out']


def test_auto_refuses_ambiguous_cameras_and_pins_single_device(plugin, monkeypatch):
    result = configure(plugin, target='auto')
    assert result['state'] == 'error' and 'multiple' in result['error'].lower()
    monkeypatch.setattr(wrist, 'discover_devices', lambda: {'devices': [device()], 'errors': []})
    plugin._refresh()
    result = plugin.dispatch('start', {'instance_id': 'card-a'})
    assert result['device_id'] == 'realsense:abc'
    monkeypatch.setattr(wrist, 'discover_devices', lambda: {'devices': [device('realsense:def')], 'errors': []})
    plugin._refresh()
    result = plugin.dispatch('info', {'instance_id': 'card-a'})
    assert result['device_id'] == 'realsense:abc' and result['state'] == 'error'
    assert len(Session.created) == 1


def test_hotplug_reconnects_only_requested_instances_and_refreshes_options(plugin, monkeypatch):
    configure(plugin)
    configure(plugin, 'card-b', 'right', 'realsense:def')
    plugin.dispatch('stop', {'instance_id': 'card-b'})
    monkeypatch.setattr(wrist, 'discover_devices', lambda: {'devices': [], 'errors': []})
    plugin._refresh()
    assert Session.created[0].closed
    assert plugin.dispatch('info', {'instance_id': 'card-a'})['state'] == 'error'
    monkeypatch.setattr(wrist, 'discover_devices', lambda: {'devices': [device(), device('realsense:def')], 'errors': []})
    plugin._refresh()
    assert len(Session.created) == 3
    assert plugin.dispatch('info', {'instance_id': 'card-a'})['state'] == 'running'
    assert plugin.dispatch('info', {'instance_id': 'card-b'})['state'] == 'idle'


def test_failed_worker_restarts_but_info_is_read_only(plugin):
    configure(plugin)
    Session.created[0].failed = True
    plugin.dispatch('info', {'instance_id': 'card-a'})
    assert len(Session.created) == 1
    plugin._refresh()
    assert len(Session.created) == 2 and Session.created[0].closed


def test_uvc_preserves_two_ports_and_reports_unavailable_depth(plugin, monkeypatch):
    monkeypatch.setattr(wrist, 'discover_devices', lambda: {'devices': [device('uvc:webcam', False)], 'errors': []})
    plugin._refresh()
    result = configure(plugin, target='uvc:webcam')
    assert result['state'] == 'running'
    assert result['streams']['depth']['state'] == 'unavailable'
    assert len(result['topic_out']) == 2


def test_settings_conflict_on_shared_device_leaves_existing_instance_running(plugin):
    configure(plugin)
    other = device()
    other['profiles']['rgb'].append({'width': 640, 'height': 480, 'fps': 30, 'format': 'rgb8'})
    other['profiles']['depth'].append({'width': 640, 'height': 480, 'fps': 30, 'format': 'z16'})
    plugin._devices = [other]
    result = configure(plugin, 'card-b', fps='30')
    assert result['state'] == 'error' and 'settings' in result['error'].lower()
    assert plugin.dispatch('info', {'instance_id': 'card-a'})['state'] == 'running'
    assert len(Session.created) == 1


def test_persistence_restores_bindings_preview_and_intentional_stop(plugin, monkeypatch):
    configure(plugin)
    configure(plugin, 'card-b', 'right', 'realsense:def')
    plugin.dispatch('stop', {'instance_id': 'card-b'})
    state = json.loads(Path(plugin._state_file).read_text())
    assert state['card-a']['device_id'] == 'realsense:abc'
    plugin.stop()
    restored = wrist.WristCameraPlugin({'state_file': plugin._state_file}, 'robot')
    try:
        restored._refresh()
        assert restored.dispatch('info', {'instance_id': 'card-a'})['state'] == 'running'
        assert restored.dispatch('info', {'instance_id': 'card-b'})['state'] == 'idle'
    finally:
        restored.stop()


@pytest.mark.parametrize('args', [{}, {'instance_id': '../bad', 'side': 'left'},
                                  {'instance_id': 'a', 'side': 'up'},
                                  {'instance_id': 'a', 'side': 'left', 'rotation': 180}])
def test_invalid_config_never_opens_device(plugin, args):
    assert plugin.dispatch('config', args)['state'] == 'error'
    assert not Session.created


def test_bundle_registers_one_card_and_boot_does_not_choose_devices(plugin, monkeypatch):
    source = ROOT / 'x-humanoid/tianyi2.0'
    tree = ast.parse((source / 'main.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TianyiDeviceBundle')
    scope = {'DualDomainROS2': object, 'os': wrist.os}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), 'main.py', 'exec'), scope)
    monkeypatch.setattr(wrist.WristCameraPlugin, 'start', lambda self: None)
    cfg = yaml.safe_load((source / 'config.yaml').read_text())
    bundle = scope['TianyiDeviceBundle']({'plugins': {'wrist_camera': cfg['plugins']['wrist_camera']}}, 'robot', None, None)
    assert [t['name'] for t in bundle.get_all_tools()] == ['wrist_camera']
    manifest = yaml.safe_load((source / 'driver.yaml').read_text())
    names = [v['name'] for v in manifest['cards']]
    assert names.count('wrist_camera') == 1
    assert not any(n.startswith('camera_wrist_') for n in names)
    assert not Session.created


def test_auto_config_replay_keeps_pinned_device_when_another_camera_appears(plugin, monkeypatch):
    monkeypatch.setattr(wrist, 'discover_devices', lambda: {'devices': [device()], 'errors': []})
    plugin._refresh()
    assert configure(plugin, target='auto')['device_id'] == 'realsense:abc'
    monkeypatch.setattr(wrist, 'discover_devices', lambda: {'devices': [device(), device('realsense:def')], 'errors': []})
    plugin._refresh()
    result = configure(plugin, target='auto')
    assert result['state'] == 'running' and result['device_id'] == 'realsense:abc'
    assert len(Session.created) == 1


def test_failed_discovery_does_not_remove_a_healthy_capture(plugin, monkeypatch):
    configure(plugin)
    monkeypatch.setattr(wrist, 'discover_devices', lambda: {'devices': [], 'errors': ['Camera discovery timed out']})
    plugin._refresh()
    assert plugin.dispatch('info', {'instance_id': 'card-a'})['state'] == 'running'
    assert not Session.created[0].closed


def test_instance_ids_that_normalize_to_same_topic_are_rejected(plugin):
    configure(plugin, 'card-a')
    result = configure(plugin, 'card_a')
    assert result['state'] == 'error' and 'conflicts' in result['error']
    assert len(Session.created) == 1


def test_discovery_timeout_still_recovers_failed_capture_using_pinned_descriptor(plugin, monkeypatch):
    configure(plugin)
    Session.created[0].failed = True
    monkeypatch.setattr(wrist, 'discover_devices', lambda: {'devices': [], 'errors': ['Camera discovery timed out']})
    plugin._refresh()
    assert Session.created[0].closed
    assert len(Session.created) == 2
    assert plugin.dispatch('info', {'instance_id': 'card-a'})['device_id'] == 'realsense:abc'
