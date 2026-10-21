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
    result=p.dispatch('start',{'_tool_name':'loco_target','feedback_binding':binding(topic='/sensor/odom')})
    assert result['topic_in'][0]['topic']=='/sensor/odom'
    result=p.dispatch('move',{'_tool_name':'loco_target','distance_m':1})
    assert result['state']=='error' and 'unavailable' in result['error']
    assert 'action_id' not in result
    p.stop()


def test_distance_binding_change_does_not_change_rotation_algorithm(monkeypatch):
    _,p=device(monkeypatch)
    assert p._loco_target_tool()['topic_in'][0]['format']=='state/odom'
    assert 'x-feedback-binding' in p._loco_target_tool()['inputSchema']
    p._destination_action='rotate'; p._rotate_state['state']='running'
    p._abort_distance('connection changed')
    assert not p._rotate_stop_event.is_set()
    p._rotate_state['state']='idle'; p.stop()


def test_production_mode_refuses_distance_until_platform_confirms_empty_or_connected_graph(monkeypatch):
    module,_=device(monkeypatch)
    p=module.LocoPlugin({'require_feedback_binding':True},'robot',None,NS(get_mode=lambda:2,publish_cmd=lambda *args:None))
    result=p.dispatch('move',{'_tool_name':'loco_target','distance_m':1})
    assert result['state']=='error' and 'binding' in result['error']
    assert 'action_id' not in result
    p.stop()


def test_default_config_allows_standalone_imu_without_sidecar_or_platform(monkeypatch):
    import yaml
    module,p=device(monkeypatch)
    config=yaml.safe_load((Path(__file__).resolve().parents[2]/'noetix/bumi/config.yaml').read_text())
    p=module.LocoPlugin(config['plugins']['loco'],'robot',None,
                       NS(get_mode=lambda:2,publish_cmd=lambda *args:None,get_imu_data=lambda:NS(ori=[0,0,0,1],angular_vel=[0,0,0],linear_acc=[0,0,9.81])))
    assert p._distance_preflight({'distance_m':.3})==(.3,None)
    assert p._distance_context['source']=='imu_integrated_experimental'
    p.stop()


def test_legacy_connected_topic_without_graph_binding_cannot_become_imu(monkeypatch):
    _,p=device(monkeypatch)
    p._high_ctrl.get_imu_data=lambda:NS(ori=[0,0,0,1],angular_vel=[0,0,0],linear_acc=[0,0,9.81])
    result=p.dispatch('start',{'_tool_name':'loco_target','input_topic':'/sensor/odom'})
    assert result['state']=='error'
    result=p.dispatch('move',{'_tool_name':'loco_target','distance_m':.3})
    assert result['state']=='error' and 'binding' in result['error']
    assert 'action_id' not in result
    p.stop()



def test_real_fused_worker_rewire_brakes_and_completes_once(monkeypatch):
    import threading
    from test_feedback import sample
    module,p=device(monkeypatch)
    f=p._distance_feedback; commands=[]; terminals=[]; moving=threading.Event()
    def publish(x,y,z,*args):
        commands.append((x,y,z))
        if x or y: moving.set()
    p._high_ctrl.publish_cmd=publish
    monkeypatch.setattr(module,'_acp_notify',lambda *args:terminals.append(args))
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    monkeypatch.setattr(f,'now_ns',lambda:10_000_000_000)
    p.dispatch('start',{'_tool_name':'loco_target','feedback_binding':binding(topic='/sensor/odom')})
    f.update(sample())
    started=p.dispatch('move',{'_tool_name':'loco_target','distance_m':3,'direction':'left'})
    try:
        assert started['state']=='running',started
        assert started['feedback_source']=='rgbd_imu_ekf'
        assert started['expected_duration_s']==60
        assert moving.wait(1)
        assert p.dispatch('move',{'_tool_name':'loco','vx':.5})['state']=='error'
        reply=p.dispatch('config',{'_tool_name':'loco_target','feedback_binding':binding('new-edge',topic='/sensor/odom',issued=10_000_000_001)})
        assert reply['state']=='ready'
        p._rotate_worker.join(1)
        assert len(terminals)==1
        assert terminals[0][0]==started['action_id'] and terminals[0][1]=='error'
        assert terminals[0][2]['feedback_source']=='rgbd_imu_ekf'
        assert 'changed' in terminals[0][2]['error']
        assert commands[-1]==(0,0,0)
        assert any(y>0 for x,y,z in commands)
    finally: p.stop()
