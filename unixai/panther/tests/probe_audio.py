"""Run inside an isolated driver-image container; no deployment changes.
Captures ambient mic PCM and plays a quiet 0.5s tone on a private DDS topic.
Prints signal statistics only; never stores room audio.
"""
import array
import json
import math
import os
import subprocess
import threading
import time


def main():
    os.environ['ROS_DOMAIN_ID'] = '42'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from audio_msgs.msg import AudioChunk
    from audio import AudioPlugin, _resolve_pulse, _pulse_user
    config = {'server': 'auto', 'uid': 1001, 'gid': 1002,
              'source': 'alsa_input.usb-AISpeech.in_Mic_Arry_8_4169385A3935-00.multichannel-input',
              'sink': 'alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo'}
    mic = AudioPlugin('mic', 'panther_validation', config)
    speaker = AudioPlugin('speaker', 'panther_validation', config)
    rclpy.init()
    node = Node('panther_audio_validation')
    received = []
    invalid = []
    def on_audio(msg):
        if msg.format != 'audio/pcm-16k' or len(msg.data) != 2048:
            invalid.append([msg.format, len(msg.data)])
        received.append(bytes(msg.data))
    node.create_subscription(AudioChunk, mic.topic, on_audio, qos_profile_sensor_data)
    pub = node.create_publisher(AudioChunk, '/panther_validation/speaker/input', 10)
    event = threading.Event()
    def spin():
        while not event.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
    thread = threading.Thread(target=spin)
    thread.start()
    monitor = None
    report = {}
    try:
        report['mic_start'] = mic.dispatch('start', {})
        time.sleep(2)
        report['speaker_start'] = speaker.dispatch('start', {'input_topic': '/panther_validation/speaker/input'})
        pulse = _resolve_pulse({**config, 'device': config['sink']})
        env = dict(os.environ, PULSE_SERVER=pulse['server'])
        monitor = subprocess.Popen(['parec', '--raw', '--format=s16le', '--rate=16000', '--channels=1',
                    '--latency-msec=40', '--device='+config['sink']+'.monitor'],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env, **_pulse_user(config))
        captured = []
        def drain():
            while True:
                raw = monitor.stdout.read(2048)
                if not raw:
                    return
                captured.append(raw)
        reader = threading.Thread(target=drain)
        reader.start()
        time.sleep(1)
        # 64ms frames at the advertised stream rate, low amplitude.
        for i in range(16):
            samples = array.array('h', [int(1000*math.sin(2*math.pi*440*(i*1024+j)/16000))
                                      if i < 8 else 0 for j in range(1024)])
            msg = AudioChunk()
            msg.format = 'audio/pcm-16k'
            msg.data = list(samples.tobytes())
            pub.publish(msg)
            time.sleep(0.064)
        time.sleep(1)
        report['mic_info'] = mic.dispatch('info', {})
        report['speaker_info'] = speaker.dispatch('info', {})
        monitor.terminate()
        monitor.wait(timeout=3)
        reader.join(timeout=3)
        for key, frames in [('mic', received), ('sink_monitor', captured)]:
            samples = array.array('h')
            samples.frombytes(b''.join(frames))
            report[key] = {'bytes': len(samples)*2, 'seconds': len(samples)/16000,
                           'rms': math.sqrt(sum(x*x for x in samples)/max(1,len(samples))),
                           'peak': max(map(abs, samples), default=0)}
        report['invalid_frames'] = invalid
        assert report['mic']['bytes'] > 32000 and report['mic']['peak'] > 0, report
        assert report['sink_monitor']['peak'] > 100, report
        assert report['speaker_info']['frames'] == 16 and not report['speaker_info']['dropped'], report
        assert not invalid, report
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    finally:
        if monitor and monitor.poll() is None:
            monitor.terminate()
        mic.stop()
        speaker.stop()
        event.set()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
