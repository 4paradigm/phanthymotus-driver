"""Pure wrapper checks and optional isolated native-DDS integration.

Native tests require paths to the existing SDK's offline IDLC/runtime. They use
only loopback and domain 173; no robot, controller SDK or motor writer is used.
"""
from __future__ import annotations

import ast
import importlib.util
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time

import pytest

ROOT=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('bumi_raw_probe_test',ROOT/'scripts/bumi_payload_raw_probe.py')
probe=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)
SDK=ROOT/'noetix/bumi/noetix_sdk_bumi'


@pytest.mark.parametrize('machine,expected', [('aarch64','aarch64'),('arm64','aarch64'),('AMD64','x86_64'),('x86_64','x86_64')])
def test_architecture(machine,expected):
    assert probe.architecture(machine)==expected


def test_unsupported_architecture():
    with pytest.raises(ValueError):
        probe.architecture('riscv64')


def test_all_shipped_type_metadata_is_checked():
    header=(SDK/'include/Robot_status.hpp').read_text()
    assert len(probe.schema_blobs(header))==12
    assert probe.verify_schema(header,header)=='66d8d9b39025b4f7214e6d0eae4efc33d45872ce0b2a1aafac05b30ec95140fe'
    mutated=header.replace('0x90,','0x91,',1)
    with pytest.raises(ValueError,match='SCHEMA_MISMATCH'):
        probe.verify_schema(header,mutated)
    with pytest.raises(ValueError,match='SCHEMA_MISMATCH'):
        probe.verify_schema(header,'')


def test_native_probe_has_no_controller_calls_or_actuator_writer():
    for forbidden in ('HighController','LowController','publish_cmd','set_joint','DataWriter','/cmd','ControlCmd'):
        assert forbidden not in probe.CPP
    assert 'DDS_BUILTIN_TOPIC_DCPSPUBLICATION' in probe.CPP
    assert '"RobotStatus::StatusData"' in probe.CPP
    assert 'libdriver' not in probe.CPP


@pytest.mark.parametrize('args', [['--domain','-1'],['--domain','201'],['--seconds','0'],['--seconds','nan'],['--discovery-seconds','inf']])
def test_invalid_parameters_never_connect(args):
    done=subprocess.run(['python3',str(ROOT/'scripts/bumi_payload_raw_probe.py'),*args],capture_output=True,text=True)
    assert done.returncode==2
    assert 'probe_metadata' not in done.stdout


PUBLISHER=r'''
#include <dds/dds.hpp>
#include "Robot_status.hpp"
#include <chrono>
#include <cmath>
#include <thread>
int main(int argc,char** argv) {
    int count=std::stoi(argv[1]);
    std::string mode=argv[2];
    dds::domain::DomainParticipant participant(173);
    auto sq=participant.default_publisher_qos();
    sq << dds::core::policy::Partition("payload-probe-tests");
    dds::pub::Publisher publisher(participant,sq);
    dds::topic::Topic<RobotStatus::StatusData> topic(participant,"payload_probe_fixture");
    dds::pub::DataWriter<RobotStatus::StatusData> writer(publisher,topic);
    dds::pub::DataWriter<RobotStatus::StatusData> second(dds::core::null);
    if(count==2) second = dds::pub::DataWriter<RobotStatus::StatusData>(publisher,topic);
    unsigned serial=0;
    while(true) {
        RobotStatus::StatusData data;
        data.workmode(2);
        data.motorstatearray().timestamp(++serial);
        std::vector<RobotStatus::MotorState> motors;
        for(int id=20;id>=0;--id) {
            double torque=mode=="zero" ? 0 : (id==0 ? 0.8 : id==10 ? -0.5 : 0.0);
            if(mode=="alternating" && serial%2) torque=0;
            if(mode=="nan" && id==0) torque=std::nan("");
            motors.emplace_back(0.0,0.0,torque,(mode=="mapping" && id==0) ? 1 : id,id==4 ? 257 : 0,40);
        }
        data.motorstatearray().motorstates(motors);
        writer.write(data);
        if(count==2) second.write(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
'''


@pytest.fixture(scope='module')
def native(tmp_path_factory):
    lib=os.environ.get('BUMI_PROBE_TEST_LIBRARY_DIR')
    idlc=os.environ.get('BUMI_PROBE_TEST_IDLC')
    if not lib or not idlc:
        pytest.skip('native DDS runtime/IDLC paths not provided; pure tests need no SDK binaries')
    folder=tmp_path_factory.mktemp('native_payload_probe')
    # Reproduce the exact ARM64 package condition on the host architecture:
    # idlc is present but has mode 0644. Only its temporary copy may change.
    source_idlc=folder/'sdk-idlc-0644'
    shutil.copyfile(idlc,source_idlc)
    source_idlc.chmod(0o644)
    checksum=probe.hashlib.sha256(source_idlc.read_bytes()).hexdigest()
    executable=probe.build_probe(SDK,Path(lib),folder,idlc=source_idlc,idlc_library_dir=Path(lib))
    assert source_idlc.stat().st_mode & 0o777 == 0o644
    assert probe.hashlib.sha256(source_idlc.read_bytes()).hexdigest()==checksum
    assert (folder/'offline-idlc').stat().st_mode & 0o777 == 0o700
    source=folder/'publisher.cpp'
    source.write_text(PUBLISHER)
    headers=SDK/'dds/x86_64/include'
    publisher=folder/'publisher'
    command=['g++','-std=c++17','-O0','-pthread',str(source),str(folder/'Robot_status.cpp'),
             '-I'+str(folder),'-I'+str(headers),'-I'+str(headers/'ddscxx'),'-I'+str(headers/'ddsc'),
             str(Path(lib)/'libddscxx.so'),str(Path(lib)/'libddsc.so'),'-Wl,-rpath,'+lib,'-o',str(publisher)]
    subprocess.run(command,check=True,capture_output=True,text=True,timeout=90)
    # Isolate both fixtures from all physical interfaces and native domain 0.
    xml='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
    env={**os.environ,'CYCLONEDDS_URI':xml}
    return executable,publisher,env


def run_native(native,count=1,mode='normal',cancel=False):
    executable,publisher,env=native
    source=None
    if count:
        source=subprocess.Popen([str(publisher),str(count),mode],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        time.sleep(0.25)
        assert source.poll() is None
    try:
        command=[str(executable),'173','1','1','']
        if cancel:
            child=subprocess.Popen(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            time.sleep(0.25)
            child.send_signal(signal.SIGINT)
            out,err=child.communicate(timeout=4)
            return child.returncode,[json.loads(line) for line in out.splitlines()],err
        done=subprocess.run(command,env=env,capture_output=True,text=True,timeout=5)
        return done.returncode,[json.loads(line) for line in done.stdout.splitlines()],done.stderr
    finally:
        if source:
            source.terminate()
            source.communicate(timeout=3)


def test_native_signed_torque_raw_errors_and_timestamp(native):
    code,events,err=run_native(native)
    assert code==0,(events,err)
    summary=next(e for e in events if e['event']=='summary')
    assert summary['received_dds_samples']>10
    assert summary['motor_timestamp_changes']>10
    assert summary['mapping_failures']==0
    assert summary['left_arm_tau'][0]['min']==pytest.approx(0.8)
    assert summary['right_arm_tau'][0]['max']==pytest.approx(-0.5)
    assert summary['nonzero_error_codes']=={'4':[257]} # no uint8 truncation or guessed interpretation
    assert summary['mass_kg'] is None
    assert summary['all_arm_zero_fraction']==0
    assert summary['left_arm_tau'][0]['zero_count']==0
    assert summary['left_arm_tau'][0]['mean']==pytest.approx(0.8)
    raw=next(e for e in events if e['event']=='raw_status')
    assert raw['mapping_ok']
    assert raw['motors'][0]['motor_id']==20 # no assumption about array order


def test_native_zero_is_an_observation_not_mass(native):
    code,events,err=run_native(native,mode='zero')
    assert code==0,(events,err)
    summary=next(e for e in events if e['event']=='summary')
    assert all(v['min']==v['max']==0 for side in ('left_arm_tau','right_arm_tau') for v in summary[side])
    assert summary['mass_kg'] is None
    assert summary['all_arm_zero_fraction']==1
    assert summary['left_arm_tau'][0]['zero_fraction']==1


def test_native_missing_writer_is_not_reported_as_zero(native):
    code,events,err=run_native(native,count=0)
    assert code==3,(events,err)
    assert events[-1]['error_code']=='NO_STATUS_WRITER'
    assert not any(e['event']=='summary' for e in events)


def test_native_duplicate_writers_are_not_fused(native):
    code,events,err=run_native(native,count=2)
    assert code==3,(events,err)
    assert events[-1]['error_code']=='AMBIGUOUS_WRITER'


def test_native_invalid_motor_mapping_reported(native):
    code,events,err=run_native(native,mode='mapping')
    assert code==0,(events,err)
    summary=next(e for e in events if e['event']=='summary')
    assert summary['mapping_failures']>0


def test_native_nonfinite_torque_not_printed_as_nan(native):
    code,events,err=run_native(native,mode='nan')
    assert code==0,(events,err)
    summary=next(e for e in events if e['event']=='summary')
    assert summary['left_arm_tau'][0]['count']==0
    assert summary['left_arm_tau'][0]['min'] is None


def test_native_cancellation_exits_without_side_effects(native):
    code,events,err=run_native(native,cancel=True)
    assert code==130,(events,err)


def test_missing_idlc_file_distinguished_from_non_executable(tmp_path):
    libs=Path(os.environ.get('BUMI_PROBE_TEST_LIBRARY_DIR','/does/not/exist'))
    if not libs.is_dir():
        pytest.skip('native DDS library path not provided')
    with pytest.raises(ValueError,match='compiler file missing'):
        probe.build_probe(SDK,libs,tmp_path,idlc=tmp_path/'absent-idlc')
    assert not (tmp_path/'probe.cpp').exists()


def test_native_alternating_zeros_are_not_filtered(native):
    code,events,err=run_native(native,mode='alternating')
    assert code==0,(events,err)
    summary=next(e for e in events if e['event']=='summary')
    assert summary['all_arm_zero_fraction']==pytest.approx(0.5,abs=0.05)
    assert summary['left_arm_tau'][0]['zero_fraction']==pytest.approx(0.5,abs=0.05)
    assert summary['left_arm_tau'][0]['mean']==pytest.approx(0.4,abs=0.04)
    assert summary['right_arm_tau'][0]['mean']==pytest.approx(-0.25,abs=0.03)


def test_getter_statistics_keep_zeros_and_signed_response():
    data=[]
    for v in (0,0.8,0,0.8):
        tau=[0.0]*21
        tau[0]=v
        tau[10]=-v
        data.append({'tau':tau})
    result=probe.torque_statistics(data)
    assert result['left_arm_tau'][0]['mean']==pytest.approx(0.4)
    assert result['right_arm_tau'][0]['mean']==pytest.approx(-0.4)
    assert result['left_arm_tau'][0]['zero_fraction']==0.5


def test_mcp_observer_deduplicates_and_only_calls_info():
    import threading
    from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
    calls=[]
    stamp=time.monotonic()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def do_POST(self):
            call=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            calls.append(call)
            sample={'motor_ids':list(range(21)),'serial':1,'host_monotonic_s':stamp,'tau':[0.0]*21,'workmode':2}
            payload={'diagnostics':{'latest':sample}}
            encoded=json.dumps({'result':{'content':[{'type':'text','text':json.dumps(payload)}]}}).encode()
            self.send_response(200)
            self.send_header('Content-Length',str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=server.serve_forever)
    thread.start()
    observer=probe.MCPObserver(f'http://127.0.0.1:{server.server_port}/mcp')
    observer.start()
    try:
        time.sleep(0.35)
    finally:
        observer.stop()
        server.shutdown()
        thread.join(2)
        server.server_close()
    summary=observer.summary(stamp-1,stamp+1)
    assert len(calls)>=2
    assert all(c['method']=='tools/call' and c['params']=={'name':'payload_mass','arguments':{'action':'info'}} for c in calls)
    assert summary['unique_getter_observations_in_window']==1
    assert summary['all_arm_zero_fraction']==1
    assert summary['comparison_type'].endswith('not_frame_aligned')
    assert observer.summary(stamp+1,stamp+2)['unique_getter_observations_in_window']==0


def test_native_and_getter_comparison_uses_overlapping_host_window(native):
    import threading
    from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
    serial=[0]
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*a): pass
        def do_POST(self):
            request=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            assert request['params']=={'name':'payload_mass','arguments':{'action':'info'}}
            serial[0]+=1
            sample={'motor_ids':list(range(21)),'serial':serial[0], 'host_monotonic_s':time.monotonic(),
                    'tau':[0.0]*21,'workmode':2}
            raw=json.dumps({'result':{'content':[{'text':json.dumps({'diagnostics':{'latest':sample}})}]}}).encode()
            self.send_response(200)
            self.send_header('Content-Length',str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=server.serve_forever)
    thread.start()
    observer=probe.MCPObserver(f'http://127.0.0.1:{server.server_port}/mcp')
    observer.start()
    try:
        code,events,err=run_native(native)
    finally:
        observer.stop()
        server.shutdown()
        thread.join(2)
        server.server_close()
    assert code==0,(events,err)
    summary=next(e for e in events if e['event']=='summary')
    getter=observer.summary(summary['host_monotonic_start_s'],summary['host_monotonic_end_s'])
    assert getter['unique_getter_observations_in_window']>=5
    assert getter['left_arm_tau'][0]['mean']==0
    assert summary['left_arm_tau'][0]['mean']==pytest.approx(0.8)
    assert summary['workmode_counts']=={'2':summary['received_dds_samples']}
    assert summary['motor_timestamp_regressions']==0
    assert summary['motor_timestamp_repeats']==0
    assert getter['comparison_type'].endswith('not_frame_aligned')


def test_getter_empty_and_large_finite_statistics():
    empty=probe.torque_statistics([])
    assert empty['left_arm_tau'][0]['mean'] is None
    assert empty['left_arm_tau'][0]['zero_fraction'] is None
    data=[{'tau':[v]*21} for v in (1e300,-1e300)]
    result=probe.torque_statistics(data)
    assert result['left_arm_tau'][0]['mean']==0
    assert result['left_arm_tau'][0]['population_stddev']==pytest.approx(1e300)
