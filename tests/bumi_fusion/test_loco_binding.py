import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
from test_feedback import binding


def device(monkeypatch):
    path=Path(__file__).resolve().parents[2]/'noetix/bumi/tests/test_loco_destination.py'
    spec=importlib.util.spec_from_file_location('fusion_loco_stubs',path)
    fixtures=importlib.util.module_from_spec(spec); spec.loader.exec_module(fixtures)
    monkeypatch.syspath_prepend(str(path.parent.parent))
    module,_=fixtures._load_device(monkeypatch)
    monkeypatch.setattr(module,'_get_default_cmd',lambda:None)
    p=module.LocoPlugin({},'robot',None,NS(get_mode=lambda:2,publish_cmd=lambda *args:None))
    monkeypatch.setattr(module,'_acp_notify',lambda *args:None)
    return module,p


def test_start_reports_subscription_and_refuses_missing_bound_odometry(monkeypatch):
    _,p=device(monkeypatch)
    f=p._distance_feedback
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    monkeypatch.setattr(f,'now_ns',lambda:10_000_000_000)
    result=p.dispatch('start',{'_tool_name':'loco_destination','feedback_binding':binding(topic='/sensor/odom')})
    assert result['topic_in'][0]['topic']=='/sensor/odom'
    result=p.dispatch('move',{'_tool_name':'loco_destination','distance_m':1})
    assert result['state']=='error' and 'unavailable' in result['error']
    assert 'action_id' not in result
    p.stop()


def test_distance_binding_change_does_not_change_rotation_algorithm(monkeypatch):
    _,p=device(monkeypatch)
    assert p._loco_destination_tool()['topic_in'][0]['format']=='state/odom'
    assert 'x-feedback-binding' in p._loco_destination_tool()['inputSchema']
    p._destination_action='rotate'; p._rotate_state['state']='running'
    p._abort_distance('connection changed')
    assert not p._rotate_stop_event.is_set()
    p._rotate_state['state']='idle'; p.stop()


def test_production_mode_refuses_distance_until_platform_confirms_empty_or_connected_graph(monkeypatch):
    module,_=device(monkeypatch)
    p=module.LocoPlugin({'require_feedback_binding':True},'robot',None,NS(get_mode=lambda:2,publish_cmd=lambda *args:None))
    result=p.dispatch('move',{'_tool_name':'loco_destination','distance_m':1})
    assert result['state']=='error' and 'binding' in result['error']
    assert 'action_id' not in result
    p.stop()
