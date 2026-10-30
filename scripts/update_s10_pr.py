#!/usr/bin/env python3
"""Check S10 changes, or explicitly commit/push them to the existing PR branch."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
SCRIPT_FILES = {
    'scripts/update_s10_pr.py', 'scripts/check_s10_release.py',
    'scripts/s10_pr_target.json',
    'scripts/s10-review-requirements.txt',
}


class CheckFailed(RuntimeError):
    pass


def git(*args, capture=True):
    result = subprocess.run(['git', *args], cwd=ROOT, capture_output=capture,
                            text=True, check=False)
    if result.returncode:
        raise CheckFailed(f'Git operation failed: {args[0]} (exit {result.returncode})')
    return result.stdout.strip() if capture else ''


def allowed_path(name):
    path = Path(name)
    if path.is_absolute() or '..' in path.parts:
        return False
    if name.startswith('deep_robotics/s10_pro/'):
        # Source and documented disabled policy examples only, never site state.
        if any(p in {'state', '__pycache__', '.env', '.git', 'logs', 'node_modules'} for p in path.parts):
            return False
        if path.suffix in {'.db', '.sqlite3', '.pem', '.key', '.tar', '.gz', '.zip', '.log', '.pyc'}:
            return False
        if path.suffix == '.json':
            return name == 'deep_robotics/s10_pro/vendor/s10-nos-drdds-1.0.23/SOURCE.json'
        return path.suffix in {'.py', '.md', '.yaml', '.yml', '.txt', '.xml', '.urdf', '.msg', '.srv', '.cmake', '.service'} or path.name in {'Dockerfile', 'LICENSE', 'CMakeLists.txt'}
    return name in SCRIPT_FILES or name == 'tests/test_s10_motion.py'


def changed_files():
    raw = subprocess.check_output(['git', 'ls-files', '-z', '--modified', '--deleted',
                                   '--others', '--exclude-standard'], cwd=ROOT)
    return sorted(set(n.decode() for n in raw.split(b'\0') if n))


def snapshot(names):
    return {n: hashlib.sha256((ROOT / n).read_bytes()).hexdigest()
            if (ROOT / n).is_file() else 'deleted' for n in names}


def remote_sha(remote, ref):
    lines = git('ls-remote', remote, ref).splitlines()
    if len(lines) != 1 or lines[0].split()[1] != ref:
        raise CheckFailed(f'Cannot identify remote ref: {ref}')
    return lines[0].split()[0]


def normalize_remote(url):
    match = re.fullmatch(r'(?:https://github.com/|git@github.com:)([^/]+/[^/]+?)(?:\.git)?', url)
    return match[1] if match else None


def check_pr_metadata(target):
    # Reuse existing GitHub authentication without logging credentials or
    # opening a browser/login prompt. Anonymous API access is also supported.
    token = os.environ.get('GH_TOKEN') or os.environ.get('GITHUB_TOKEN')
    if not token and shutil.which('gh'):
        r = subprocess.run(['gh', 'auth', 'token'], capture_output=True, text=True)
        if r.returncode == 0:
            token = r.stdout.strip()
    if not token:
        try:
            env = {**os.environ, 'GIT_TERMINAL_PROMPT': '0', 'GCM_INTERACTIVE': 'never'}
            r = subprocess.run(['git', 'credential', 'fill'], cwd=ROOT, env=env,
                input='protocol=https\nhost=github.com\n\n', capture_output=True, text=True, timeout=10)
            if r.returncode == 0:
                token = dict(line.split('=', 1) for line in r.stdout.splitlines() if '=' in line).get('password')
        except subprocess.TimeoutExpired:
            pass
    headers = {'Accept': 'application/vnd.github+json', 'User-Agent': 'S10-PR-Update'}
    if token:
        headers['Authorization'] = 'Bearer ' + token
    request = urllib.request.Request(
        f"https://api.github.com/repos/{target['repository']}/pulls/{target['number']}", headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            data = json.load(response)
    except urllib.error.HTTPError as exc:
        raise CheckFailed(f'GitHub PR metadata unavailable (HTTP {exc.code}); authenticate gh or set GH_TOKEN before publishing') from None
    except (urllib.error.URLError, TimeoutError):
        raise CheckFailed('Cannot read GitHub PR metadata; nothing was committed or pushed') from None
    if (data.get('state') != 'open' or data.get('merged')
            or data['head']['ref'] != target['head_branch']
            or data['head']['repo']['full_name'].lower() != target['head_repository'].lower()):
        raise CheckFailed('PR is closed or its original branch/repository changed')
    return data['head']['sha']


def run_checks():
    for command in ([sys.executable, 'scripts/check_s10_release.py'],
                    [sys.executable, 'scripts/check_service_yml.py', 'deep_robotics/s10_pro']):
        subprocess.run(command, cwd=ROOT, check=True)
    subprocess.run([sys.executable, '-m', 'pytest', '-q', 'tests/test_s10_motion.py'], cwd=ROOT, check=True)
    git('diff', '--check')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--publish', action='store_true', help='Commit and push after checks; otherwise check only')
    parser.add_argument('--offline', action='store_true', help='Local checks only; cannot publish')
    parser.add_argument('--message', default='feat(s10-pro): unify locomotion card and diagnose stop confirmation')
    args = parser.parse_args()
    if args.offline and args.publish:
        parser.error('--offline cannot be combined with --publish')
    target = json.loads((ROOT / 'scripts/s10_pr_target.json').read_text())
    if git('branch', '--show-current') != target['head_branch']:
        raise CheckFailed('Use the original PR branch; this script never switches branches')
    for mode in ([], ['--push']):
        if normalize_remote(git('remote', 'get-url', *mode, target['remote'])) != target['head_repository']:
            raise CheckFailed('Original PR fork remote does not match')
    if git('diff', '--cached', '--name-only'):
        raise CheckFailed('Index already contains staged changes; preserve/review them before using this script')
    if git('ls-files', '-u'):
        raise CheckFailed('Resolve merge conflicts first')
    head = git('rev-parse', 'HEAD')
    pr_sha = None
    upstream = f"https://github.com/{target['repository']}.git"
    pr_ref = f"refs/pull/{target['number']}/head"
    branch_ref = f"refs/heads/{target['head_branch']}"
    if not args.offline:
        pr_sha = remote_sha(upstream, pr_ref)
        if remote_sha(target['remote'], branch_ref) != pr_sha:
            raise CheckFailed('PR head and original remote branch differ')
        git('fetch', '--no-tags', target['remote'], branch_ref)
        git('merge-base', '--is-ancestor', pr_sha, 'HEAD')
        committed = git('diff', '--name-only', pr_sha, 'HEAD').splitlines()
        if any(not allowed_path(n) for n in committed):
            raise CheckFailed('Unpushed commits contain changes outside the S10 release scope')
    names = changed_files()
    selected = [n for n in names if allowed_path(n)]
    unrelated = sorted(set(names) - set(selected))
    print(f"Target: PR #{target['number']} / {target['head_repository']}:{target['head_branch']}", flush=True)
    print(f'S10 changed files: {len(selected)}; unrelated files left untouched: {len(unrelated)}', flush=True)
    before = snapshot(selected)
    run_checks()
    if git('rev-parse', 'HEAD') != head or selected != [n for n in changed_files() if allowed_path(n)] or snapshot(selected) != before:
        raise CheckFailed('Source changed during checks; rerun to test the current contents')
    if not args.publish:
        print('Checks passed. No commit or push performed. Use --publish to update the existing PR.', flush=True)
        return
    metadata_sha = check_pr_metadata(target)
    if metadata_sha != pr_sha or remote_sha(target['remote'], branch_ref) != pr_sha:
        raise CheckFailed('Remote branch changed during checks; no commit or push performed')
    if selected:
        git('add', '--', *selected)
        staged = git('diff', '--cached', '--name-only').splitlines()
        if sorted(staged) != sorted(selected) or snapshot(selected) != before:
            raise CheckFailed('Index/source changed while staging; review the index before rerunning')
        git('commit', '-m', args.message, capture=False)
    if git('rev-parse', 'HEAD') == pr_sha:
        print('PR already contains these changes; nothing to push.', flush=True)
        return
    # A normal explicit-ref push rejects concurrent/divergent remote updates.
    # Never amend, rebase, reset, create a PR, comment, or use a force option.
    git('push', target['remote'], f'HEAD:{branch_ref}', capture=False)
    new_head = git('rev-parse', 'HEAD')
    if remote_sha(upstream, pr_ref) != new_head:
        raise CheckFailed('Branch pushed, but GitHub PR ref is not yet updated; verify the PR page before retrying')
    print(f"Updated https://github.com/{target['repository']}/pull/{target['number']} to {new_head[:12]}", flush=True)


if __name__ == '__main__':
    try:
        main()
    except (CheckFailed, subprocess.CalledProcessError) as exc:
        print(f'Stopped: {exc}', file=sys.stderr)
        sys.exit(1)
