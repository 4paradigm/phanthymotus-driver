"""Real private-process callbacks, synthetic marker files only; no robot I/O."""
import os
from pathlib import Path
import threading
import time

import pytest

from deep_robotics.s10_pro.posture_process import BoundedPostureProcess, PostureProcessUnavailable
from deep_robotics.s10_pro.motion_dds import AuthorityUnavailable


FACTORY_SOURCE = '''
from pathlib import Path
import time
import subprocess
import sys
class Adapter:
    def __init__(self, config): self.config=config
    def send(self, rid, action, reason, authority, cancel_event):
        if self.config.get('child_marker'):
            subprocess.Popen([sys.executable, '-c',
                'import time,pathlib;time.sleep(.2);pathlib.Path('+repr(self.config['child_marker'])+').write_text("late child")'])
        time.sleep(self.config.get('send_delay',0))
        Path(self.config['marker']).write_text('synthetic send only')
        return {'sent':True,'accepted':None}
    def feedback(self,rid,action):
        time.sleep(self.config.get('feedback_delay',0))
        if self.config.get('feedback_marker'):
            Path(self.config['feedback_marker']).write_text('synthetic feedback only')
        return {'request_id':rid,'fresh':True,'confirmed':False}
def create(config):
    time.sleep(config.get('startup_delay',0))
    return Adapter(config)
'''


@pytest.fixture
def adapter(tmp_path, monkeypatch):
    (tmp_path/'synthetic_posture.py').write_text(FACTORY_SOURCE)
    monkeypatch.setenv('PYTHONPATH', str(tmp_path)+os.pathsep+os.environ.get('PYTHONPATH',''))
    config={'marker':str(tmp_path/'sent.txt')}
    clients=[]
    def make(**options):
        changes=options.pop('config',{})
        client=BoundedPostureProcess('synthetic_posture:create',{**config,**changes},**options)
        clients.append(client)
        return client
    yield make,Path(config['marker']),tmp_path
    for client in clients: client.close()


def permit():
    now=time.monotonic()
    return {'owned':True,'fresh':True,'received_monotonic':now,'age_s':0.,'expires_monotonic':now+.5,
            'request_id':'synthetic-request','regular_mode_confirmed':True,'allowed_actions':['stand']}


def send(client,event=None):
    return client.send('synthetic-request','stand','synthetic_cancel',permit(),event or threading.Event())


def test_constructor_has_no_process_and_queries_never_prepare(adapter):
    make,marker,_=adapter
    client=make()
    assert not client.status()['worker_running']
    with pytest.raises(PostureProcessUnavailable):send(client)
    with pytest.raises(PostureProcessUnavailable):client.feedback('synthetic-request','stand')
    assert not client.status()['worker_running'] and not marker.exists()


def test_explicit_prepare_invokes_no_send_and_reuses_ready_session(adapter):
    make,marker,_=adapter;client=make()
    assert client.prepare()['prepared'] and not marker.exists()
    assert client.prepare()['prepared'] and not marker.exists()
    assert send(client)['sent'] and marker.read_text()=='synthetic send only'
    assert not client.feedback('synthetic-request','stand')['confirmed']
    client.close()
    with pytest.raises(PostureProcessUnavailable):send(client)
    with pytest.raises(PostureProcessUnavailable):client.prepare()


def test_blocked_send_is_killed_before_late_marker_and_never_restarted(adapter):
    make,marker,_=adapter;client=make(config={'send_delay':.3},send_timeout_s=.04)
    client.prepare();started=time.monotonic()
    with pytest.raises(TimeoutError):send(client)
    assert time.monotonic()-started < .7
    assert client.status()['closed'] and not client.status()['worker_running']
    time.sleep(.35)
    assert not marker.exists()
    with pytest.raises(PostureProcessUnavailable):send(client)
    with pytest.raises(PostureProcessUnavailable):client.prepare()


def test_blocked_feedback_is_killed_before_late_marker(adapter):
    make,_,tmp=adapter;marker=tmp/'feedback.txt'
    client=make(config={'feedback_delay':.3,'feedback_marker':str(marker)},feedback_timeout_s=.03)
    client.prepare()
    with pytest.raises(TimeoutError):client.feedback('synthetic-request','stand')
    time.sleep(.35)
    assert client.status()['closed'] and not marker.exists()


def test_timeout_reaps_callback_owned_children(adapter):
    make,marker,tmp=adapter;child=tmp/'late-child.txt'
    client=make(config={'send_delay':.3,'child_marker':str(child)},send_timeout_s=.04)
    client.prepare()
    with pytest.raises(TimeoutError):send(client)
    time.sleep(.35)
    assert not marker.exists() and not child.exists()


def test_shorter_source_authority_expiry_bounds_send(adapter):
    make,marker,_=adapter;client=make(config={'send_delay':.3},send_timeout_s=.2)
    client.prepare();authority=permit();authority['expires_monotonic']=time.monotonic()+.03
    with pytest.raises(TimeoutError):
        client.send('synthetic-request','stand','cancel',authority,threading.Event())
    time.sleep(.35)
    assert not marker.exists() and client.status()['closed']


def test_inflight_cancel_kills_worker_and_prevents_late_send(adapter):
    make,marker,_=adapter;client=make(config={'send_delay':.3},send_timeout_s=.2)
    client.prepare();event=threading.Event();timer=threading.Timer(.02,event.set);timer.start()
    try:
        with pytest.raises(InterruptedError):send(client,event)
    finally:timer.join()
    time.sleep(.35)
    assert client.status()['closed'] and not marker.exists()


def test_factory_startup_timeout_kills_session_without_callbacks(adapter):
    make,marker,_=adapter;client=make(config={'startup_delay':.3})
    with pytest.raises(TimeoutError):client.prepare(timeout_s=.05)
    assert client.status()['closed'] and not marker.exists()


@pytest.mark.parametrize('changes',[{'regular_mode_confirmed':False},{'allowed_actions':[]},
    {'request_id':'another-request'},{'fresh':False}])
def test_invalid_authority_never_invokes_private_adapter(adapter,changes):
    make,marker,_=adapter;client=make();client.prepare()
    with pytest.raises((PermissionError,ValueError,AuthorityUnavailable)):
        client.send('synthetic-request','stand','cancel',{**permit(),**changes},threading.Event())
    assert not marker.exists()
