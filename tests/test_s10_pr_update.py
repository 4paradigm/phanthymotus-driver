"""PR updater routing and failure boundaries; never connects or pushes to GitHub."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('s10_pr_updater', ROOT / 'scripts/update_s10_pr.py')
u = importlib.util.module_from_spec(spec)
spec.loader.exec_module(u)


@pytest.mark.parametrize('name', [
    'deep_robotics/s10_pro/loco_move.py', 'tests/test_s10_loco_move.py',
    'scripts/update_s10_pr.py', 'deep_robotics/s10_pro/deploy/LOCO_MOVE.md'])
def test_release_scope_includes_new_source_and_checks(name):
    assert u.allowed_path(name)


@pytest.mark.parametrize('name', [
    'common/vendor_runtime.py', 'unitree/g1/main.py', '.env',
    'deep_robotics/s10_pro/state/tasks.sqlite3',
    'deep_robotics/s10_pro/deploy/operator-authorization.json',
    'deep_robotics/s10_pro/secrets.pem', '/tmp/loco_move.py',
    'deep_robotics/s10_pro/../../.env', 'tests/fixtures/site.json'])
def test_unrelated_files_and_site_secrets_are_excluded(name):
    assert not u.allowed_path(name)


@pytest.fixture
def updater(tmp_path, monkeypatch):
    (tmp_path / 'scripts').mkdir()
    target = json.loads((ROOT / 'scripts/s10_pr_target.json').read_text())
    (tmp_path / 'scripts/s10_pr_target.json').write_text(json.dumps(target))
    source = 'deep_robotics/s10_pro/loco_move.py'
    path = tmp_path / source
    path.parent.mkdir(parents=True)
    path.write_text('candidate = 1\n')
    state = {'branch': target['head_branch'], 'head': 'a' * 40, 'remote': 'a' * 40,
             'staged': [], 'calls': [], 'checked': 0}
    def git(*args, capture=True):
        state['calls'].append(args)
        if args[:2] == ('branch', '--show-current'):
            return state['branch']
        if args[:2] == ('remote', 'get-url'):
            return 'https://github.com/' + target['head_repository'] + '.git'
        if args[:2] == ('rev-parse', 'HEAD'):
            return state['head']
        if args[:3] == ('diff', '--cached', '--name-only'):
            return '\n'.join(state['staged'])
        if args[0] == 'add':
            state['staged'] = list(args[2:])
        if args[0] == 'commit':
            state['head'] = 'b' * 40
            state['staged'] = []
        if args[0] == 'push':
            state['remote'] = state['head']
        return ''
    def checks():
        state['checked'] += 1
    monkeypatch.setattr(u, 'ROOT', tmp_path)
    monkeypatch.setattr(u, 'git', git)
    monkeypatch.setattr(u, 'remote_sha', lambda *a: state['remote'])
    monkeypatch.setattr(u, 'check_pr_metadata', lambda *a: state['remote'])
    monkeypatch.setattr(u, 'run_checks', checks)
    monkeypatch.setattr(u, 'changed_files', lambda: [source, 'notes.txt'])
    return state, source, path


def test_check_only_never_stages_commits_or_pushes(updater, monkeypatch):
    state, _, _ = updater
    monkeypatch.setattr(sys, 'argv', ['update_s10_pr.py', '--offline'])
    u.main()
    assert state['checked'] == 1
    assert not any(c[0] in ('add', 'commit', 'push') for c in state['calls'])


def test_publish_only_stages_s10_and_pushes_explicit_original_ref(updater, monkeypatch):
    state, source, _ = updater
    monkeypatch.setattr(sys, 'argv', ['update_s10_pr.py', '--publish'])
    u.main()
    assert ('add', '--', source) in state['calls']
    pushes = [c for c in state['calls'] if c[0] == 'push']
    assert pushes == [('push', 'origin', 'HEAD:refs/heads/feat/s10-base-posture-timed-move')]
    assert not any('--force' in ' '.join(c) or c[0] in ('reset', 'checkout', 'rebase')
                   for c in state['calls'])


@pytest.mark.parametrize('case', ['wrong_branch', 'staged', 'changed_during_checks', 'remote_changed'])
def test_unsafe_state_stops_before_commit_or_push(updater, monkeypatch, case):
    state, _, path = updater
    monkeypatch.setattr(sys, 'argv', ['update_s10_pr.py', '--publish'])
    if case == 'wrong_branch':
        state['branch'] = 'main'
    elif case == 'staged':
        state['staged'] = ['other.py']
    elif case == 'changed_during_checks':
        monkeypatch.setattr(u, 'run_checks', lambda: path.write_text('candidate = 2\n'))
    else:
        monkeypatch.setattr(u, 'check_pr_metadata', lambda *a: 'c' * 40)
    with pytest.raises(u.CheckFailed):
        u.main()
    assert not any(c[0] in ('add', 'commit', 'push') for c in state['calls'])
