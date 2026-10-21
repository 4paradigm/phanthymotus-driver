import importlib.util
from pathlib import Path
import pytest
from test_feedback import binding

PATH=Path(__file__).resolve().parents[2]/'noetix/bumi_odometry/platform/overlay/agent-core/src/odometry_graph_binding.py'
spec=importlib.util.spec_from_file_location('graph_binding_test',PATH)
graph=importlib.util.module_from_spec(spec);spec.loader.exec_module(graph)


def layout():
    return {'cards':[{'id':'sensor','mcpId':'sensor-mcp','toolName':'rgbd_imu_odometry','topicOut':[{'topic':'/odometry','format':'state/odom'}]},
                     {'id':'destination','mcpId':'bumi-mcp','toolName':'loco_target'}],
            'connections':[{'id':'edge','fromCardId':'sensor','fromPortIdx':0,'toCardId':'destination','toPortIdx':0,'format':'state/odom'}]}


def test_saved_graph_drives_actual_consumer_without_topic_discovery(monkeypatch):
    from noetix.bumi.odometry_feedback import DistanceFeedback
    f=DistanceFeedback(None,now_ns=lambda:10**10,monotonic=lambda:10.)
    subscribed=[];monkeypatch.setattr(f,'_subscribe',subscribed.append)
    data=layout(); b=graph.binding_for(data,data['cards'][1],now_ns=10**10)
    result=f.apply(b)
    assert subscribed==['/odometry'] and result['topic_in'][0]['topic']=='/odometry'
    with pytest.raises(ValueError,match='unavailable'): f.freeze()
    data['connections']=[]
    b2=graph.binding_for(data,data['cards'][1],now_ns=10**10+1)
    f.apply(b2)
    assert f.freeze()['source']=='imu_integrated_experimental'


def test_identity_changes_even_when_topic_does_not():
    data=layout();a=graph.binding_for(data,data['cards'][1],now_ns=1)
    data['connections'][0]['id']='new-edge'
    b=graph.binding_for(data,data['cards'][1],now_ns=1)
    assert a['revision']!=b['revision'] and a['input_topic']==b['input_topic']


@pytest.mark.parametrize('fault',['missing-source','wrong-sensor','multi-edge','unresolved'])
def test_invalid_graph_is_not_disconnected_imu(fault):
    data=layout()
    if fault=='missing-source': data['cards'].pop(0)
    elif fault=='wrong-sensor': data['cards'][0]['toolName']='motion_state'
    elif fault=='multi-edge': data['connections']*=2
    else: data['cards'][0]['topicOut']=[]
    b=graph.binding_for(data,data['cards'][-1],now_ns=1)
    assert b['resolved'] is False


def test_persisted_topic_conflicting_with_live_source_is_unresolved():
    data=layout();data['connections'][0]['fromTopic']='/stale'
    b=graph.binding_for(data,data['cards'][1],now_ns=1,
                        find_tool=lambda *args:{'topic_out':[{'topic':'/odometry','format':'state/odom'}]})
    assert b['resolved'] is False


def test_backend_renewal_delivers_saved_connection_and_receives_ack(monkeypatch):
    import asyncio
    import sys
    from types import ModuleType,SimpleNamespace as NS
    from noetix.bumi.odometry_feedback import DistanceFeedback
    layout_data=layout();clock=[10**10]
    f=DistanceFeedback(None,now_ns=lambda:clock[0],monotonic=lambda:clock[0]/1e9)
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    cfg=ModuleType('config');cfg.main={'canvas_layout':layout_data,'core':{'project_running':True}}
    tc=ModuleType('tool_config')
    tc.find_tool=lambda mcp,tool:({'inputSchema':{'x-feedback-binding':1}} if tool=='loco_target'
                                else {'topic_out':[{'topic':'/odometry','format':'state/odom'}]})
    api=ModuleType('api');api.__path__=[];manage=ModuleType('api.mcp_manage')
    async def call(mcp,req,timeout_s):
        assert mcp=='bumi-mcp'
        answer={'state':'ready',**f.apply(req.arguments['feedback_binding'])}
        return {'code':200,'data':{'result':{'content':[{'type':'text','text':__import__('json').dumps(answer)}]}}}
    manage.mcp_call_tool=call;manage.MCPCallRequest=lambda **kw:NS(**kw)
    for key,value in [('config',cfg),('tool_config',tc),('api',api),('api.mcp_manage',manage)]:monkeypatch.setitem(sys.modules,key,value)
    monkeypatch.setattr(graph.time,'time_ns',lambda:clock[0]);monkeypatch.setattr(graph,'_last_issued',0)
    asyncio.run(graph.refresh())
    assert f.topic=='/odometry'
    layout_data['connections']=[];clock[0]+=1
    asyncio.run(graph.refresh(force=True))
    assert f.freeze()['source']=='imu_integrated_experimental'


@pytest.mark.parametrize('bad_ack',['old-lease','wrong-topic','error'])
def test_renewal_requires_exact_lease_and_stops_on_bad_ack(monkeypatch,bad_ack):
    import asyncio
    import sys
    from types import ModuleType,SimpleNamespace as NS
    calls=[]
    cfg=ModuleType('config');cfg.main={'canvas_layout':layout(),'core':{'project_running':True}}
    tc=ModuleType('tool_config');tc.find_tool=lambda mcp,tool:({'inputSchema':{'x-feedback-binding':1}} if tool=='loco_target' else {'topic_out':[{'topic':'/odometry','format':'state/odom'}]})
    api=ModuleType('api');api.__path__=[];manage=ModuleType('api.mcp_manage')
    async def call(mcp,req,timeout_s):
        calls.append(req.arguments['action'])
        if calls[-1]=='stop': return {'data':{'state':'stopped'}}
        accepted=dict(req.arguments['feedback_binding'])
        if bad_ack=='old-lease': accepted['issued_ns']-=1
        return {'data':{'state':'error' if bad_ack=='error' else 'ready','feedback_binding':accepted,
                        'topic_in':[{'topic':'/wrong' if bad_ack=='wrong-topic' else '/odometry'}]}}
    manage.mcp_call_tool=call;manage.MCPCallRequest=lambda **kw:NS(**kw)
    for key,value in [('config',cfg),('tool_config',tc),('api',api),('api.mcp_manage',manage)]:monkeypatch.setitem(sys.modules,key,value)
    monkeypatch.setattr(graph,'_failures',{})
    asyncio.run(graph.refresh())
    assert calls==['config','stop']
    assert graph._failures


@pytest.mark.parametrize('live_ports',[[],[{'topic':'/odometry','format':'other'}]])
def test_saved_port_cannot_replace_missing_or_changed_live_sensor_contract(live_ports):
    data=layout()
    b=graph.binding_for(data,data['cards'][1],now_ns=1,
                        find_tool=lambda *args:{'topic_out':live_ports})
    assert b['resolved'] is False
