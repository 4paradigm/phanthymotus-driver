"""Canvas lifecycle/config protocol, source isolation and Chinese readout."""
from types import SimpleNamespace as NS
import json
import socket
import struct

import pytest

from agilex.piper.table_clearance import TableClearance
from agilex.piper.clearance_feedback import PassiveFeedback
from test_piper_table_clearance import calibrated,snapshot


def test_start_reports_canvas_running_without_overwriting_measurement_state():
    card=TableClearance(snapshot,{'table_clearance':calibrated()},'test',clock=lambda:100.)
    result=card.start()
    assert result['state']=='running'
    card.tick(card.generation)
    assert card.info()['state']=='running'
    assert card.info()['result']['state']=='ok'
    assert card.info()['result']['读数']['离桌间隙']=='99.0 mm'
    assert all(k in card.info()['result']['字段说明'] for k in card.result)
    card.stop()


def test_stop_reports_idle_and_clears_measurement_for_canvas():
    card=TableClearance(snapshot,{'table_clearance':calibrated()},'test',clock=lambda:100.)
    card.start();card.tick(card.generation)
    result=card.stop()
    assert result['state']=='idle'
    assert result['result']['state']=='idle'
    assert result['result']['读数']['离桌间隙']=='暂无有效距离'


def test_manual_canvas_buttons_and_terminal_idle_publication():
    from test_piper_table_clearance import Publisher
    card=TableClearance(snapshot,{'table_clearance':calibrated()},'test',ros2=object(),
        clock=lambda:100.,publisher_factory=Publisher)
    tool=card.get_tools()[0]
    assert tool['inputSchema']['x-manual-execution'] is True
    assert 'begin_measurement' in tool['inputSchema']['properties']['action']['enum']
    assert tool['configSchema']['properties']['verified']['advanced'] is True
    assert card.dispatch('begin_measurement',{})['state']=='running'
    transport=card.transport
    transport.callback()
    assert card.dispatch('pause_measurement',{})['state']=='idle'
    assert transport.messages[-1]['state']=='idle'
    assert transport.messages[-1]['clearance_mm'] is None
    assert transport.messages[-1]['读数']['状态说明']=='测距已暂停，请点击开始测距'


class Source:
    def __init__(self,interface):self.interface=interface;self.started=False;self.closed=False
    def start(self):self.started=True
    def close(self):self.closed=True
    def snapshot(self):return snapshot(can_interface=self.interface)


def test_canvas_config_source_change_invalidates_calibration_and_old_numbers():
    calls=[]
    card=TableClearance(lambda:calls.append(1) or snapshot(),{'table_clearance':calibrated()},
        'test',clock=lambda:100.,source_factory=Source)
    form=card.get_tools()[0]['configSchema']['properties']
    assert form['can_interface']['scope']=='instance'
    assert {o['const'] for o in form['mode']['oneOf']}=={'calibrated','model_estimate'}
    card.start();card.tick(card.generation)
    result=card.dispatch('config',{'can_interface':'can1','instance_id':'card1'})
    assert result['state']=='running' and card.source.started
    assert card.cfg['verified'] is False and card.cfg['calibration_id']==''
    assert result['result']['读数']['离桌间隙']=='暂无有效距离'
    card.tick(card.generation)
    assert len(calls)==1
    assert card.info()['result']['reason']=='calibration_unverified'
    source=card.source
    card.stop();assert source.closed


def test_invalid_config_does_not_stop_live_card_and_estimate_mode_works():
    card=TableClearance(lambda:dict(snapshot(),joints_deg=[0.]*6,feedback_time=100.),
        {'table_clearance':calibrated()},'test',clock=lambda:100.)
    card.start();card.tick(card.generation)
    with pytest.raises(ValueError):card.dispatch('config',{'verified':True,'plane_base_mm':[0,0,0,0]})
    assert card.running and card.info()['result']['clearance_mm']==99.
    card.dispatch('config',dict(mode='model_estimate',estimate_profile_path='builtin:piper_100mm',table_z_mm=0))
    card.tick(card.generation)
    assert card.info()['state']=='running' and card.info()['result']['state']=='estimated'
    assert card.cfg['verified'] is False
    json.dumps(card.info(),ensure_ascii=False,allow_nan=False)
    card.stop()


def test_passive_feedback_filtered_receive_only_and_staleness():
    clock=NS(wall=100.,tick=10.)
    calls=[]
    class Bus:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def setsockopt(self,*args):calls.append(('filter',args))
        def settimeout(self,t):calls.append(('timeout',t))
        def bind(self,address):calls.append(('bind',address))
        def recv(self,size):source.cancel.set();raise socket.timeout()
        def send(self,*args):pytest.fail('A passive receiver must never send')
    source=PassiveFeedback('can1',socket_factory=lambda *args:Bus(),clock=lambda:clock.wall,monotonic=lambda:clock.tick)
    source._run()
    assert ('bind',('can1',)) in calls
    def packet(ident,raw):return struct.pack('=IB3x8s',ident,8,raw)
    for ident in [0x2A2,0x2A3,0x2A4,0x2A5,0x2A6,0x2A7]:
        source.accept_packet(packet(ident,struct.pack('>ii',1000,2000)))
    source.accept_packet(packet(0x2A8,struct.pack('>ii',51700,0)))
    result=source.snapshot()
    assert result['can_interface']=='can1' and result['fresh']
    assert result['joints_deg']==[1.,2.]*3
    assert result['gripper']['width_mm']==51.7
    assert result['end_pose']['position_mm']==[1.,2.,1.]
    source.accept_packet(packet(0x800002A5,struct.pack('>ii',9999,9999)))
    assert source.snapshot()['joints_deg']==[1.,2.]*3
    clock.tick+=.6
    assert source.snapshot()['fresh'] is False
    assert 'gripper' not in source.snapshot()
