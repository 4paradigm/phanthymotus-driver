import importlib.util
from pathlib import Path
import subprocess
import sys
import time
import pytest

MODULE=Path(__file__).resolve().parents[1]/'franka/fr3/remote_backend.py'
spec=importlib.util.spec_from_file_location('remote_backend',MODULE)
remote=importlib.util.module_from_spec(spec);spec.loader.exec_module(remote)


def test_robot_allowlist_rejects_unknown_address():
    assert remote.allowed_robot('172.16.0.2','172.16.0.2,172.16.0.3')=='172.16.0.2'
    with pytest.raises(ValueError):remote.allowed_robot('172.16.0.4','172.16.0.2')


def test_missing_heartbeat_expires_without_robot(tmp_path):
    fake=tmp_path/'fake-native'
    fake.write_text('#!/usr/bin/env python3\nimport time\ntime.sleep(30)\n')
    fake.chmod(0o700)
    p=subprocess.Popen([sys.executable,str(MODULE),'server','172.16.0.2',
        '--allowed','172.16.0.2','--native',str(fake)],stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        p.wait(timeout=7)
        assert p.returncode!=0
        assert b'heartbeat lease expired' in p.stderr.read()
    finally:
        if p.poll() is None:p.kill();p.wait()
        p.stdin.close();p.stdout.close();p.stderr.close()


def test_heartbeat_is_filtered_and_eof_stops_backend(tmp_path):
    fake=tmp_path/'fake-native'
    fake.write_text('#!/usr/bin/env python3\nimport sys\nfor line in sys.stdin: print(line.strip(),flush=True)\n')
    fake.chmod(0o700)
    p=subprocess.Popen([sys.executable,str(MODULE),'server','172.16.0.2',
        '--allowed','172.16.0.2','--native',str(fake)],stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        p.stdin.write(b'@FR3_PING\nINFO test\n');p.stdin.flush()
        import select
        assert select.select([p.stdout],[],[],3)[0]
        assert p.stdout.readline()==b'INFO test\n'
        p.stdin.close();p.wait(timeout=7)
    finally:
        if p.poll() is None:p.kill();p.wait()
        p.stdout.close();p.stderr.close()


def test_forced_key_rejects_shell_and_unknown_robot():
    entry=MODULE.with_name('remote_forced_entry.py')
    spec=importlib.util.spec_from_file_location('forced_entry',entry)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    assert module.requested_robot('fr3-backend 172.16.0.2', ('172.16.0.2',))=='172.16.0.2'
    for command in ('bash','fr3-backend 172.16.0.4','fr3-backend 172.16.0.2; id',
                    'fr3-backend 172.16.0.2 --native /tmp/other'):
        with pytest.raises(ValueError):module.requested_robot(command, ('172.16.0.2',))
