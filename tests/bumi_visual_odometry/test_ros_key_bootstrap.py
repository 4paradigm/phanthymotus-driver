"""Run the Dockerfile's key bootstrap without network access or host key changes."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


DOCKERFILE = Path(__file__).resolve().parents[2]/'noetix/bumi/Dockerfile'
KEYRING = '/usr/share/keyrings/ros-archive-keyring.gpg'


def bootstrap(tmp_path, scenario, existing=False):
    key = tmp_path/'ros.gpg'
    log = tmp_path/'curl.jsonl'
    if existing:
        key.write_bytes(b'existing trusted key')
    curl = tmp_path/'curl'
    curl.write_text('#!'+sys.executable+'\n'+'''
import json, os, sys
from pathlib import Path
args = sys.argv[1:]
url = next(arg for arg in args if arg.startswith(('https://', 'http://')))
with open(os.environ['KEY_TEST_LOG'], 'a') as log:
    log.write(json.dumps(args)+'\\n')
output = Path(args[args.index('-o')+1])
scenario = os.environ['KEY_TEST_SCENARIO']
if url.startswith('http://'):
    output.write_bytes(b'attacker key')
    sys.exit(0)
if scenario == 'empty':
    output.write_bytes(b'')
    sys.exit(0)
if scenario == 'primary' or (scenario == 'fallback' and 'githubusercontent' in url):
    output.write_bytes(b'trusted HTTPS key')
    sys.exit(0)
if scenario == 'partial':
    output.write_bytes(b'failed partial download')
sys.exit(18)
''')
    curl.chmod(0o755)
    text = DOCKERFILE.read_text()
    block = text.split('RUN set -eu;', 1)[1].split('. /etc/os-release;', 1)[0]
    script = 'set -eu;'+block.replace('\\\n', '').replace(KEYRING, str(key))
    env = dict(os.environ, PATH=str(tmp_path)+os.pathsep+os.environ['PATH'],
               KEY_TEST_LOG=str(log), KEY_TEST_SCENARIO=scenario)
    result = subprocess.run(['/bin/sh', '-c', script], env=env,
                            capture_output=True, text=True, timeout=5)
    calls = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return result, key, calls


@pytest.mark.parametrize('scenario, count', [('primary', 1), ('fallback', 2)])
def test_https_key_sources_work_without_protocol_downgrade(tmp_path, scenario, count):
    result, key, calls = bootstrap(tmp_path, scenario)
    assert result.returncode == 0, result.stderr
    assert key.read_bytes() == b'trusted HTTPS key'
    assert len(calls) == count
    for args in calls:
        assert args[args.index('--proto')+1] == '=https'
        assert args[args.index('--proto-redir')+1] == '=https'


def test_https_failures_never_fall_back_to_an_attacker_http_key(tmp_path):
    result, key, calls = bootstrap(tmp_path, 'unavailable')
    assert result.returncode != 0
    assert not key.exists()
    assert calls
    assert all(not arg.startswith('http://') for args in calls for arg in args)


def test_partial_failed_download_cannot_be_used_as_a_key(tmp_path):
    result, key, calls = bootstrap(tmp_path, 'partial')
    assert calls
    assert result.returncode != 0
    assert not key.exists()


def test_preinstalled_key_is_preserved_without_downloading(tmp_path):
    result, key, calls = bootstrap(tmp_path, 'unavailable', existing=True)
    assert result.returncode == 0, result.stderr
    assert key.read_bytes() == b'existing trusted key'
    assert calls == []


def test_successful_empty_download_is_rejected(tmp_path):
    result, key, calls = bootstrap(tmp_path, 'empty')
    assert calls
    assert result.returncode != 0
    assert not key.read_bytes()
