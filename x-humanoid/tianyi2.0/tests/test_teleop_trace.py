import json
from types import SimpleNamespace
from test_motion_stream import rig
from teleop_executor import TeleopExecutor

def test_trace_records_decisions_without_secret_and_requires_idle_control():
    gate,_,_,advance,packet=rig()
    e=TeleopExecutor({},'isolated',None,SimpleNamespace(_pos_publisher=True),None,[])
    e.gate=gate
    assert e._dispatch('trace_start',{})['code']=='trace_control_requires_idle'
    # Enable the observer fixture without changing the already leased plant.
    e._trace_enabled=True;e._trace_id='fixture'
    command=packet();e._command(SimpleNamespace(data=json.dumps(command)))
    first=e._trace_snapshot()['events'][0]
    assert first['accepted'] and first['seq']==command['seq']
    assert command['mac'] not in json.dumps(first) and gate.secret not in json.dumps(first)
    e._command(SimpleNamespace(data=json.dumps(command)))
    assert e._trace_snapshot()['events'][-1]['reason']=='stale_sequence'
    for i in range(40):e._trace('test',index=i)
    snapshot=e._trace_snapshot()
    assert snapshot['event_count']==42 and len(snapshot['events'])==32
    assert len(json.dumps(snapshot))<32768
    gate.hold(release=True);gate.tick();advance(20);gate.tick()
    assert e._dispatch('trace_stop',{})['enabled'] is False
    e._trace('late')
    assert e._trace_snapshot()['event_count']==42
    assert e._dispatch('trace_start',{})['event_count']==0

def test_trace_cannot_crash_rejection_of_out_of_float_range_numbers():
    gate,_,_,_,packet=rig()
    e=TeleopExecutor({},'isolated',None,SimpleNamespace(_pos_publisher=True),None,[])
    e.gate=gate;e._trace_enabled=True;e._trace_id='fixture'
    command=packet();command['q'][0]=10**400
    e._command(SimpleNamespace(data=json.dumps(command)))
    row=e._trace_snapshot()['events'][-1]
    assert row['accepted'] is False and 'q_rad' not in row
