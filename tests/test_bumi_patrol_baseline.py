"""Offline Bumi patrol drafts, including the real bundle's HTTP MCP route.

Synthetic observations only. ROS imports are replaced; no SDK, device, image
recognition, navigation, or notification delivery is exercised.
"""

from copy import deepcopy
from datetime import datetime, timedelta, timezone
from http.server import ThreadingHTTPServer
import importlib.util
import json
from pathlib import Path
import sys
import threading
import types
from urllib.request import Request, urlopen

import pytest


ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / 'noetix' / 'bumi'
SPEC = importlib.util.spec_from_file_location('bumi_patrol_for_test', BUMI / 'patrol_baseline.py')
patrol_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(patrol_module)


@pytest.fixture
def profile():
    return json.loads((BUMI / 'examples/patrol-baseline-demo.json').read_text(encoding='utf-8'))


@pytest.fixture
def patrol(profile):
    return patrol_module.PatrolBaseline(profile)


def evidence(**overrides):
    return {
        'point_id': 'east-exit', 'view_id': 'exit-front', 'kind': 'visual',
        'captured_at': datetime.now(timezone.utc).isoformat(),
        'reference': 'synthetic:test-frame-001',
        'observed': '测试画面显示通行区域内有占用物。', **overrides,
    }


def report(**overrides):
    return {
        'action': 'prepare_report', 'point_id': 'east-exit', 'check_id': 'clear-walkway',
        'run_id': 'test-round', 'revision': 'normal-v1', 'decision': 'deviation',
        'priority': 'urgent', 'assessment': '通行区域被占用，需要值班人员确认。',
        'evidence': evidence(), **overrides,
    }


def write_profile(tmp_path, profile):
    path = tmp_path / 'baseline.json'
    path.write_text(json.dumps(profile, ensure_ascii=False), encoding='utf-8')
    return path


def test_plan_and_read_keep_order_and_isolate_snapshot(patrol, profile):
    plan = patrol.call({'action': 'plan'})
    assert [point['point_id'] for point in plan['points']] == ['east-exit', 'pantry', 'server-room']
    assert plan['revision'] == 'normal-v1'
    assert 'checks' not in plan['points'][0]
    point = patrol.call({'action': 'read', 'point_id': 'east-exit'})
    point['checks'][0]['normal'] = 'Changed by caller'
    profile['points'][0]['checks'][0]['route_id'] = 'it-operations'
    reread = patrol.call({'action': 'read', 'point_id': 'east-exit'})
    assert '有效通行区域' in reread['checks'][0]['normal']
    assert reread['checks'][0]['route_id'] == 'facilities'


def test_operator_route_and_report_key_are_stable(patrol):
    request = report(route_id='it-operations')
    draft = patrol.call(request)
    assert (draft['route_id'], draft['status'], draft['priority']) == (
        'facilities', 'deviation_report', 'urgent')
    assert draft['assessment'] == request['assessment']
    assert draft['report_key'] == patrol.call(request)['report_key']
    assert draft['delivery'] == 'rehearsal_only'
    assert draft['sent'] is False


@pytest.mark.parametrize(('overrides', 'reason'), [
    ({'view_id': 'wrong-view'}, 'point_view_or_source_mismatch'),
    ({'point_id': 'pantry'}, 'point_view_or_source_mismatch'),
    ({'kind': 'sensor'}, 'point_view_or_source_mismatch'),
    ({'captured_at': '2026-11-08T08:00:00'}, 'invalid_evidence'),
    ({'captured_at': '0001-01-01T00:00:00+01:00'}, 'invalid_evidence'),
    ({'captured_at': None}, 'invalid_evidence'),
    ({'reference': ''}, 'invalid_evidence'),
])
def test_mismatched_or_malformed_evidence_is_review_only(patrol, overrides, reason):
    draft = patrol.call(report(evidence=evidence(**overrides)))
    assert (draft['status'], draft['priority'], draft['route_id']) == (
        'needs_review', 'review', 'duty-review')
    assert draft['review_reason'] == reason
    assert draft['evidence'] is None
    assert draft['sent'] is False


@pytest.mark.parametrize('offset_seconds', [-60, 60])
def test_stale_or_future_evidence_is_review_only(patrol, offset_seconds):
    stamp = datetime.now(timezone.utc) + timedelta(seconds=offset_seconds)
    draft = patrol.call(report(evidence=evidence(captured_at=stamp.isoformat())))
    assert draft['status'] == 'needs_review'
    assert draft['priority'] == 'review'
    assert draft['review_reason'] == 'evidence_outside_time_window'
    assert draft['sent'] is False


@pytest.mark.parametrize('bad_evidence', [None, {}, [], '{', 'x' * 2049])
def test_missing_or_invalid_evidence_never_confirms_deviation(patrol, bad_evidence):
    draft = patrol.call(report(evidence=bad_evidence))
    assert draft['status'] == 'needs_review'
    assert draft['route_id'] == 'duty-review'
    assert draft['evidence'] is None
    assert draft['sent'] is False


def test_agent_uncertainty_retains_fresh_evidence_for_review(patrol):
    draft = patrol.call(report(decision='needs_review', evidence=evidence(observed='画面有反光，成因不明。')))
    assert draft['route_id'] == 'duty-review'
    assert draft['review_reason'] == 'agent_needs_review'
    assert draft['evidence']['observed'] == '画面有反光，成因不明。'
    assert draft['sent'] is False


def test_site_profile_routes_to_it_but_never_sends(profile):
    card = patrol_module.PatrolBaseline({**profile, 'mode': 'site'})
    draft = card.call(report(
        point_id='server-room', check_id='panel-state', priority='routine',
        evidence=evidence(point_id='server-room', view_id='status-panel', observed='测试面板有告警。')))
    assert (draft['route_id'], draft['delivery'], draft['sent']) == ('it-operations', 'pending', False)


@pytest.mark.parametrize('overrides', [
    {'revision': 'previous'}, {'check_id': 'unknown'}, {'point_id': 'unknown'},
    {'decision': 'normal'}, {'priority': 'critical'}, {'assessment': ''},
])
def test_invalid_report_identity_or_decision_is_rejected(patrol, overrides):
    with pytest.raises(ValueError):
        patrol.call(report(**overrides))


@pytest.mark.parametrize('overrides', [
    {'review_route': 'unknown'}, {'schema': 'unknown'}, {'mode': 'automatic'},
    {'max_age_seconds': True}, {'max_age_seconds': 301}, {'points': []},
])
def test_invalid_baselines_are_rejected(profile, overrides):
    with pytest.raises(ValueError):
        patrol_module.PatrolBaseline({**profile, **overrides})


def test_duplicate_points_and_checks_are_rejected(profile):
    duplicate_point = deepcopy(profile)
    duplicate_point['points'].append(deepcopy(duplicate_point['points'][0]))
    with pytest.raises(ValueError, match='Duplicate point'):
        patrol_module.PatrolBaseline(duplicate_point)
    profile['points'][0]['checks'].append(deepcopy(profile['points'][0]['checks'][0]))
    with pytest.raises(ValueError, match='Duplicate check'):
        patrol_module.PatrolBaseline(profile)


def test_plugin_lifecycle_and_operator_path_snapshot(tmp_path, profile):
    path = write_profile(tmp_path, profile)
    card = patrol_module.PatrolBaselinePlugin({'baseline_path': str(path)}, 'test', object())
    assert card.dispatch('info', {})['state'] == 'idle'
    assert card.start()['state'] == 'ready'
    assert card.dispatch('info', {})['hardware_command'] is False
    assert card.stop()['state'] == 'idle'
    path.write_text('not JSON anymore', encoding='utf-8')
    assert card.dispatch('plan', {})['revision'] == profile['revision']
    assert card.dispatch('start', {'action': 'stop'})['state'] == 'ready'
    with pytest.raises(ValueError, match='operator configuration'):
        card.dispatch('plan', {'baseline_path': str(tmp_path / 'other.json')})
    with pytest.raises(ValueError, match='arguments must be an object'):
        card.dispatch('plan', [])
    with pytest.raises(ValueError, match='Unknown patrol action'):
        card.dispatch('set_baseline', {'revision': 'new'})


def test_default_single_point_is_relative_to_module_not_cwd(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    card = patrol_module.PatrolBaselinePlugin({}, 'unused', None)
    plan = card.dispatch('plan', {})
    assert plan['mode'] == 'rehearsal'
    assert plan['max_age_seconds'] == 90
    assert [point['point_id'] for point in plan['points']] == ['fixed-passage']
    point = card.dispatch('read', {'point_id': 'fixed-passage'})
    assert point['view_id'] == 'bumi-front-fixed'
    draft = card.dispatch('prepare_report', {
        'run_id': 'fixed-test', 'revision': plan['revision'], 'point_id': 'fixed-passage',
        'check_id': point['checks'][0]['check_id'], 'decision': 'deviation',
        'priority': 'urgent',
    })
    assert draft['status'] == 'needs_review'
    assert draft['review_reason'] == 'missing_evidence'
    assert draft['sent'] is False


def test_explicit_relative_baseline_path(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    card = patrol_module.PatrolBaselinePlugin(
        {'baseline_path': 'examples/patrol-baseline-demo.json'}, 'unused', None)
    assert len(card.dispatch('plan', {})['points']) == 3


@pytest.mark.parametrize('config', [None, [], {'baseline_path': ''}, {'baseline_path': 3}])
def test_invalid_plugin_configuration_is_rejected(config):
    with pytest.raises(ValueError):
        patrol_module.PatrolBaselinePlugin(config, 'unused', None)


@pytest.mark.parametrize('raw', [b'{', b'[]', b'\xff', b'{"schema":"invalid"}'])
def test_invalid_file_fails_explicitly_without_demo_fallback(tmp_path, raw):
    path = tmp_path / 'invalid.json'
    path.write_bytes(raw)
    with pytest.raises(ValueError, match='Cannot load patrol baseline'):
        patrol_module.PatrolBaselinePlugin({'baseline_path': str(path)}, 'unused', None)


def test_missing_or_oversized_baseline_fails(tmp_path):
    path = tmp_path / 'missing.json'
    with pytest.raises(ValueError, match='Cannot load patrol baseline'):
        patrol_module.PatrolBaselinePlugin({'baseline_path': str(path)}, 'unused', None)
    path.write_bytes(b' ' * (patrol_module.MAX_BASELINE_BYTES + 1))
    with pytest.raises(ValueError, match='exceeds 2 MiB'):
        patrol_module.PatrolBaselinePlugin({'baseline_path': str(path)}, 'unused', None)


@pytest.fixture
def bundle_module(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT))
    monkeypatch.syspath_prepend(str(BUMI))
    fake_rclpy = types.ModuleType('rclpy')
    fake_executors = types.ModuleType('rclpy.executors')
    fake_rclpy.executors = fake_executors
    monkeypatch.setitem(sys.modules, 'rclpy', fake_rclpy)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', fake_executors)
    monkeypatch.setitem(sys.modules, 'patrol_baseline', patrol_module)
    import common.logsafe
    monkeypatch.setattr(common.logsafe, 'install', lambda: None)
    spec = importlib.util.spec_from_file_location('bumi_patrol_bundle_test', BUMI / 'main.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bundle_disabled_means_no_patrol_tool_or_file_read(bundle_module):
    bundle = bundle_module.BumiDeviceBundle(
        {'plugins': {'patrol_baseline': {'enabled': False, 'baseline_path': 'does-not-exist'}}},
        'test', None, None, None)
    assert bundle.get_all_tools() == []


def test_enabled_bundle_registers_without_robot_controllers(bundle_module):
    bundle = bundle_module.BumiDeviceBundle(
        {'plugins': {'patrol_baseline': {'enabled': True}}}, 'test', None, None, None)
    assert [tool['name'] for tool in bundle.get_all_tools()] == ['patrol_baseline']
    bundle.start_all()
    assert bundle.dispatch('patrol_baseline', {'action': 'info'})['state'] == 'ready'
    bundle.stop_all()
    assert bundle.dispatch('patrol_baseline', {'action': 'info'})['state'] == 'idle'


def test_bad_enabled_bundle_config_fails_before_card_registration(bundle_module):
    with pytest.raises(ValueError, match='Cannot load patrol baseline'):
        bundle_module.BumiDeviceBundle(
            {'plugins': {'patrol_baseline': {'enabled': True, 'baseline_path': 'does-not-exist'}}},
            'test', None, None, None)


def test_real_bundle_http_tool_discovery_and_draft(bundle_module, tmp_path, profile):
    path = write_profile(tmp_path, profile)
    bundle_module._bundle = bundle_module.BumiDeviceBundle(
        {'plugins': {'patrol_baseline': {'enabled': True, 'baseline_path': str(path)}}},
        'test', None, None, None)
    server = ThreadingHTTPServer(('127.0.0.1', 0), bundle_module.make_handler())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        def rpc(method, params=None):
            payload = {'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params or {}}
            request = Request(
                f'http://127.0.0.1:{server.server_port}/mcp',
                data=json.dumps(payload).encode(), headers={'Content-Type': 'application/json'})
            with urlopen(request, timeout=3) as response:
                return json.load(response)

        assert rpc('initialize')['result']['protocolVersion'] == '2024-11-05'
        tool = rpc('tools/list')['result']['tools'][0]
        assert tool['name'] == 'patrol_baseline'
        assert tool['type'] == 'processor'
        assert tool['topic_in'] == tool['topic_out'] == []
        assert 'baseline_path' not in tool['inputSchema']['properties']
        assert 'revision' in tool['inputSchema']['x-action-params']['prepare_report']['required']
        for manual in (False, True):
            request = report()
            if manual:
                request['evidence'] = json.dumps(request['evidence'], ensure_ascii=False)
            result = rpc('tools/call', {'name': 'patrol_baseline', 'arguments': request})
            draft = json.loads(result['result']['content'][0]['text'])
            assert draft['route_id'] == 'facilities'
            assert draft['status'] == 'deviation_report'
            assert draft['sent'] is False
        result = rpc('tools/call', {
            'name': 'patrol_baseline', 'arguments': report(revision='stale-version')})
        assert result['error']['code'] == -32603
        assert 'Baseline revision changed' in result['error']['message']
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
