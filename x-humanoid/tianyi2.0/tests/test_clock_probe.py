"""Clock request bounds belong to the independent HTTP observer, not IPC receipt."""
import importlib.util
from pathlib import Path
import time
import pytest

ROOT = Path(__file__).parents[1]

def module():
    spec = importlib.util.spec_from_file_location('tianyi_clock_probe', ROOT/'clock_probe.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_probe_keeps_child_bounds_when_parent_delays_reading():
    import subprocess,sys
    # Independent local HTTP server models a source clock without sharing the
    # test parent's interpreter or needing robot hardware.
    server = subprocess.Popen([sys.executable, '-u', '-c', '''
import http.server,time
class Handler(http.server.BaseHTTPRequestHandler):
 def do_GET(self):
  body=str(time.monotonic_ns()//1000000).encode()
  self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
 def log_message(self,*args):pass
s=http.server.HTTPServer(('127.0.0.1',0),Handler)
print(s.server_port,flush=True);s.serve_forever()
'''],stdout=subprocess.PIPE,text=True)
    try:
        port=int(server.stdout.readline()); m=module()
        probe=m.ClockProbe('127.0.0.1',port,'')
        try:
            first=probe.read()
            probe._process.stdin.write('2\n');probe._process.stdin.flush()
            time.sleep(.15)
            import json
            delayed=json.loads(probe._process.stdout.readline())
            assert delayed['after_ns']-delayed['before_ns'] < 100_000_000
            assert time.time_ns()-delayed['after_ns'] >= 100_000_000
            assert delayed['monotonic_ns'] > first['monotonic_ns']
            assert delayed['device_ms']*1_000_000 >= delayed['monotonic_ns']-1_000_000
        finally:probe.close()
        assert probe._process.poll() is not None
    finally:
        server.terminate();server.wait(timeout=2);server.stdout.close()


def test_dead_probe_fails_closed_and_close_is_repeatable():
    m=module();probe=m.ClockProbe('127.0.0.1',1,'')
    probe._process.kill();probe._process.wait(timeout=2)
    with pytest.raises((ValueError,OSError),match='clock|pipe|Pipe'):
        probe.read()
    probe.close();probe.close()


@pytest.mark.parametrize('response,reason', [
    ('{"sequence":99}', 'sequence'), ('not-json', 'Expecting'),
    ('', 'closed'), (None, 'timeout'),
])
def test_bad_or_late_channel_is_retired(monkeypatch, response, reason):
    import subprocess,sys
    m=module(); launch=subprocess.Popen
    code="import sys,time;sys.stdin.readline();" + (
        'time.sleep(2)' if response is None else 'print('+repr(response)+',flush=True)' if response else '')
    monkeypatch.setattr(m.subprocess,'Popen',lambda args,**kw: launch([sys.executable,'-u','-c',code],**kw))
    p=m.ClockProbe('127.0.0.1',1,'')
    try:
        with pytest.raises(ValueError,match=reason):p.read()
        with pytest.raises(ValueError,match='unavailable'):p.read()
    finally:p.close()
    assert p._process.poll() is not None


def test_broken_stdin_flush_still_reaps_worker(monkeypatch):
    import subprocess,sys
    m=module();launch=subprocess.Popen
    code='import os,time;os.close(0);print("ready",flush=True);time.sleep(30)'
    monkeypatch.setattr(m.subprocess,'Popen',lambda args,**kw:launch([sys.executable,'-u','-c',code],**kw))
    p=m.ClockProbe('127.0.0.1',1,'');assert p._process.stdout.readline().strip()=='ready'
    try:
        with pytest.raises(BrokenPipeError):p.read()
        p.close()
        assert p._process.poll() is not None
    finally:
        if p._process.poll() is None:p._process.kill();p._process.wait()
