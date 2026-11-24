import importlib.util
import io
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import microphone
import bootstrap
spec = importlib.util.spec_from_file_location('microphone_rpc', ROOT/'host/microphone_rpc.py')
remote = importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote)


def wait(predicate):
    end = time.monotonic()+3
    while time.monotonic()<end:
        if predicate():
            return True
        time.sleep(.01)
    return False


class Message:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)


class Node:
    def __init__(self):
        self.messages = []
        self.created, self.destroyed = [], []
    def create_publisher(self, kind, topic, qos):
        publisher = types.SimpleNamespace(publish=self.messages.append)
        self.created.append((topic, qos))
        return publisher
    def destroy_publisher(self, publisher):
        self.destroyed.append(publisher)
    def get_clock(self):
        return types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:'now'))


def process_fixture(mode='valid'):
    source = """import os,sys,struct,time
os.write(1,b'{"ok":true,"format":"audio/pcm-16k"}\\n')
MODE
while sys.stdin.buffer.read(1):pass
"""
    packet = struct.pack('!dI', time.time(), microphone.BLOCK) + struct.pack('<512h', *([123]*512))
    if mode == 'valid':
        content = 'os.write(1,'+repr(packet[:23])+');time.sleep(.01);os.write(1,'+repr(packet[23:])+')'
    elif mode == 'invalid':
        content = 'os.write(1,struct.pack("!dI",float("nan"),1024))'
    else:
        content = 'sys.exit(0)'
    return subprocess.Popen([sys.executable, '-u', '-c', source.replace('MODE', content)],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, bufsize=0)


class MicrophoneTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.node = Node()
        self.modules = patch.dict(sys.modules, {
            'audio_msgs.msg':types.SimpleNamespace(AudioChunk=Message),
            'rclpy.qos':types.SimpleNamespace(QoSProfile=lambda **kw:types.SimpleNamespace(**kw),
                ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='best'),
                DurabilityPolicy=types.SimpleNamespace(VOLATILE='volatile'))})
        self.modules.start()
        self.addCleanup(self.modules.stop)
        self.manager = types.SimpleNamespace(state_dir=Path(self.temp.name), transport=Mock(),
            _target=lambda role:{'address':'example.invalid','user':'test'})
        self.manager.transport.run.return_value={'ok':True,'result':{'devices':[
            {'id':'alsa/plughw:CARD=USB,DEV=0','name':'USB microphone','usb':True}]}}
        self.mics = microphone.Microphones(self.node,'robot',self.manager)
        self.addCleanup(self.mics.close)

    def test_probe_excludes_playback_monitors_and_unconnected_jetson_routes(self):
        root=Path(self.temp.name)
        (root/'pcm').write_text('00-00: APE routing : capture 1\n01-00: USB Audio : capture 1\n02-00: HDMI : playback 1\n')
        for index,name in ((0,'APE'),(1,'USB')):
            (root/('card'+str(index))).mkdir()
            (root/('card'+str(index))/'id').write_text(name)
        (root/'card1/usbid').write_text('abcd:1234')
        result=types.SimpleNamespace(returncode=0,stdout='0 alsa_output.card.monitor mod s16le\n1 alsa_input.usb-mic mod s16le\n2 auto_null.monitor mod s16le\n')
        with patch.object(remote.shutil,'which',return_value='/bin/tool'), patch.object(remote.subprocess,'run',return_value=result):
            devices=remote.probe(root)['devices']
        self.assertEqual([d['id'] for d in devices],['alsa/plughw:CARD=USB,DEV=0','pulse/alsa_input.usb-mic'])
        self.assertTrue(all(d['usb'] for d in devices))
        with patch.object(remote,'probe',return_value={'devices':devices}):
            self.assertEqual(remote.capture_command(devices[0]['id'])[0],'arecord')
            for bad in ('pulse/alsa_output.card.monitor','alsa/default','alsa/;touch /tmp/unsafe'):
                with self.assertRaises(ValueError):
                    remote.capture_command(bad)

    def test_only_usb_capture_inputs_are_visible_and_analog_cannot_start(self):
        self.mics.devices = [
            {'id':'chassis/alsa/plughw:CARD=PCH,DEV=0','role':'chassis','name':'Analog','usb':False},
            {'id':'host/pulse/alsa_input.usb-mic','role':'host','name':'USB alias','usb':True},
            {'id':'host/alsa/plughw:CARD=USB,DEV=0','role':'host','name':'USB mic','usb':True}]
        choices = self.mics.tools()[0]['configSchema']['properties']['device_index']['oneOf']
        self.assertEqual([c['const'] for c in choices], ['host/alsa/plughw:CARD=USB,DEV=0'])
        self.assertEqual(self.mics.dispatch('mic','start',{})['state'],'error')
        self.mics.refresh = lambda:None
        result=self.mics.dispatch('ext_mic','start',{'instance_id':'one',
            'device_index':'chassis/alsa/plughw:CARD=PCH,DEV=0'})
        self.assertEqual(result['state'],'error')
        self.manager.transport.open_stream.assert_not_called()

    def test_fragmented_real_pipe_publishes_correct_audio_and_stop_reaps_recorder(self):
        process=process_fixture()
        capture=microphone.Capture(self.node,process,'/robot/mic/default/audio')
        try:
            self.assertTrue(capture.ready.wait(3))
            self.assertEqual(capture.snapshot()['state'],'running')
            self.assertEqual(bytes(self.node.messages[0].data),struct.pack('<512h',*([123]*512)))
            self.assertEqual(self.node.messages[0].format,'audio/pcm-16k')
            self.assertEqual(capture.snapshot()['rms'],123)
        finally:
            capture.close()
        self.assertIsNotNone(process.poll())
        self.assertEqual(len(self.node.destroyed),1)

    def test_malformed_frame_and_disconnect_report_error_without_fake_audio(self):
        for mode in ('invalid','disconnect'):
            capture=microphone.Capture(self.node,process_fixture(mode),'/test')
            try:
                self.assertTrue(capture.ready.wait(3))
                self.assertEqual(capture.snapshot()['state'],'error')
                self.assertFalse(self.node.messages)
            finally:
                capture.close()

    def test_ext_mic_config_multi_instance_and_shared_credentials(self):
        self.mics.refresh()
        tool=next(t for t in self.mics.tools() if t['name']=='ext_mic')
        self.assertTrue(tool['multiInstance'])
        self.assertEqual(tool['inputSchema'], {'type':'object','properties':{}})
        self.assertEqual([t['name'] for t in self.mics.tools()], ['ext_mic'])
        self.assertEqual(set(tool['configSchema']['properties']),{'device_index'})
        self.assertEqual(self.mics.dispatch('ext_mic','start',{})['state'],'error')
        device='host/alsa/plughw:CARD=USB,DEV=0'
        self.assertEqual(self.mics.dispatch('ext_mic','config',{'instance_id':'one','device_index':device})['state'],'configured')
        self.manager.transport.open_stream.side_effect=lambda *a:process_fixture()
        result=self.mics.dispatch('ext_mic','start',{'instance_id':'one'})
        self.assertEqual(result['state'],'running')
        self.assertEqual(result['topic_out'][0]['topic'],'/robot/ext_mic/i_6f6e65/audio')
        self.assertEqual(self.mics.dispatch('ext_mic','start',{'instance_id':'two','device_index':device})['state'],'error')
        self.assertEqual(self.mics.dispatch('ext_mic','stop',{'instance_id':'one'})['state'],'idle')
        self.assertFalse(self.mics.sessions)

    def test_missing_or_ambiguous_mic_is_not_replaced_with_a_simulated_source(self):
        result=self.mics.dispatch('ext_mic','start',{'instance_id':'one'})
        self.assertEqual(result['state'],'error')  # Three boards, no arbitrary auto-pick.
        self.assertFalse(self.node.messages)
        self.manager.transport.open_stream.assert_not_called()
        self.manager.transport.run.return_value={'ok':True,'result':{'devices':[]}}
        self.assertEqual(self.mics.dispatch('ext_mic','start',{'instance_id':'one'})['state'],'error')

    def test_stop_during_discovery_prevents_late_capture_start(self):
        entered,release=threading.Event(),threading.Event()
        def refresh():
            entered.set();release.wait(3)
            self.mics.devices=[{'id':'host/alsa/test','usb':True}]
        self.mics.refresh=refresh
        results=[]
        worker=threading.Thread(target=lambda:results.append(self.mics.dispatch('ext_mic','start',{'instance_id':'one','device_index':'host/alsa/test'})))
        worker.start()
        try:
            self.assertTrue(entered.wait(2))
            self.assertEqual(self.mics.dispatch('ext_mic','stop',{'instance_id':'one'})['state'],'idle')
            release.set();worker.join(3)
            self.assertEqual(results[0]['state'],'idle')
            self.manager.transport.open_stream.assert_not_called()
        finally:
            release.set();worker.join(3)

    def test_unique_usb_input_can_be_selected_without_credentials_on_card(self):
        def probe(role, *args, **kwargs):
            devices = [{'id':'alsa/plughw:CARD=USB,DEV=0','name':'USB microphone','usb':True}] if role=='host' else []
            return {'ok':True,'result':{'devices':devices}}
        self.manager.transport.run.side_effect=probe
        self.manager.transport.open_stream.side_effect=lambda *a:process_fixture()
        result=self.mics.dispatch('ext_mic','start',{'instance_id':'one'})
        self.assertEqual(result['state'],'running')
        self.assertEqual(result['device_index'],'host/alsa/plughw:CARD=USB,DEV=0')
        self.assertEqual(self.mics.dispatch('ext_mic','stop',{'instance_id':'one'})['state'],'idle')

    def test_remote_recorder_is_reaped_after_stdin_closes_or_heartbeat_expires(self):
        for close_input in (True,False):
            pidfile=Path(self.temp.name)/('recorder-'+str(close_input))
            child="import os,time;from pathlib import Path;Path("+repr(str(pidfile))+").write_text(str(os.getpid()));\nwhile True:os.write(1,b'0'*1024);time.sleep(.01)"
            command=[sys.executable,'-u','-c',child]
            script="import importlib.util;spec=importlib.util.spec_from_file_location('remote',"+repr(str(ROOT/'host/microphone_rpc.py'))+");m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.capture_command=lambda device:"+repr(command)+";m.stream('fixture',lease_s=.5)"
            process=subprocess.Popen([sys.executable,'-u','-c',script],stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
            try:
                self.assertTrue(wait(pidfile.is_file))
                if close_input:process.stdin.close()
                process.wait(3)
                with self.assertRaises(ProcessLookupError):
                    os.kill(int(pidfile.read_text()),0)
            finally:
                if process.poll() is None:
                    process.kill();process.wait()
                process.stdin.close();process.stdout.close()

    def test_stream_uses_shared_secret_transport_without_password_in_argv(self):
        process=Mock()
        with patch.object(bootstrap.subprocess,'Popen',return_value=process) as spawn:
            transport=bootstrap.SshTransport(Path(self.temp.name))
            self.assertIs(transport.open_stream('host',{'address':'127.0.0.1','user':'robot'},
                {'operation':'capture','device':'alsa/test'},'print(1)',
                {'host':{'password':'offline-test-secret'}}),process)
        self.assertNotIn('offline-test-secret',' '.join(spawn.call_args.args[0]))
        self.assertEqual(spawn.call_args.kwargs['env']['SSHPASS'],'offline-test-secret')
        payload=process.stdin.write.call_args.args[0]
        self.assertTrue(payload.endswith(b'\n'))
        self.assertEqual(json.loads(payload)['operation'],'capture')
